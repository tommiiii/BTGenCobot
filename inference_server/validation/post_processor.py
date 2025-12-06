"""Post-processing filters to prevent model hallucination and runaway generation"""

import re
import logging
from typing import Optional, Tuple
from xml.etree import ElementTree as ET

logger = logging.getLogger(__name__)


class GenerationFilter:
    """Detects and fixes common model hallucination patterns"""
    
    def __init__(
        self, 
        max_depth: int = 4,
        max_repeated_patterns: int = 3,
        max_total_nodes: int = 20
    ):
        """
        Initialize generation filter
        
        Args:
            max_depth: Maximum allowed nesting depth
            max_repeated_patterns: Maximum times a pattern can repeat sequentially
            max_total_nodes: Maximum total number of BT nodes
        """
        self.max_depth = max_depth
        self.max_repeated_patterns = max_repeated_patterns
        self.max_total_nodes = max_total_nodes
    
    def detect_runaway_generation(self, xml_content: str) -> Tuple[bool, Optional[str]]:
        """
        Detect if generation has gone off the rails

        Returns:
            (is_runaway, reason)
        """
        depth_issue, depth_reason = self._check_depth(xml_content)
        if depth_issue:
            return True, depth_reason

        pattern_issue, pattern_reason = self._check_repeated_patterns(xml_content)
        if pattern_issue:
            return True, pattern_reason

        count_issue, count_reason = self._check_node_count(xml_content)
        if count_issue:
            return True, count_reason

        return False, None
    
    def _check_depth(self, xml_content: str) -> Tuple[bool, Optional[str]]:
        """Check XML nesting depth"""
        try:
            root = ET.fromstring(xml_content)
            max_depth = self._get_max_depth(root)
            
            if max_depth > self.max_depth:
                return True, f"Excessive nesting depth: {max_depth} (max: {self.max_depth})"
            
            return False, None
        except ET.ParseError:
            return False, None
    
    def _get_max_depth(self, element, current_depth=0) -> int:
        """Recursively calculate maximum depth of XML tree"""
        if len(element) == 0:
            return current_depth
        
        return max(self._get_max_depth(child, current_depth + 1) for child in element)
    
    def _check_repeated_patterns(self, xml_content: str) -> Tuple[bool, Optional[str]]:
        """
        Check for repeated incremental naming patterns
        e.g., CheckCupPoseCondition1, CheckCupPoseCondition2...
        """
        name_pattern = r'name="([^"]+)"'
        names = re.findall(name_pattern, xml_content)

        base_pattern = r'(.+?)(\d+)$'

        pattern_counts = {}
        for name in names:
            match = re.match(base_pattern, name)
            if match:
                base_name = match.group(1)
                pattern_counts[base_name] = pattern_counts.get(base_name, 0) + 1

        for base_name, count in pattern_counts.items():
            if count > self.max_repeated_patterns:
                return True, f"Repeated pattern detected: '{base_name}*' appears {count} times (max: {self.max_repeated_patterns})"

        return False, None
    
    def _check_node_count(self, xml_content: str) -> Tuple[bool, Optional[str]]:
        """Check total number of BT nodes"""
        try:
            root = ET.fromstring(xml_content)
            bt_tree = root.find('.//BehaviorTree')

            if bt_tree is not None:
                node_count = sum(1 for _ in bt_tree.iter()) - 1

                if node_count > self.max_total_nodes:
                    return True, f"Too many nodes: {node_count} (max: {self.max_total_nodes})"

            return False, None
        except ET.ParseError:
            return False, None
    
    def truncate_at_valid_point(self, xml_content: str) -> Optional[str]:
        """
        Truncate XML at the first valid stopping point before runaway generation
        
        Strategy:
        1. Find the first complete, valid BehaviorTree
        2. Cut off everything after the first reasonable depth
        3. Properly close all open tags
        
        Returns:
            Truncated XML if successful, None if tree becomes empty/invalid
        """
        try:
            root = ET.fromstring(xml_content)

            bt_tree = root.find('.//BehaviorTree')
            if bt_tree is None:
                return None

            self._prune_deep_branches(bt_tree, current_depth=0)

            if len(bt_tree) == 0:
                logger.warning("Pruning resulted in empty BehaviorTree - tree is too deep")
                return None

            xml_declaration = '<?xml version="1.0"?>\n'
            return xml_declaration + ET.tostring(root, encoding='unicode')

        except ET.ParseError:
            return self._salvage_malformed_xml(xml_content)
    
    def _prune_deep_branches(self, element, current_depth: int):
        """Recursively prune branches that exceed max depth"""
        if current_depth >= self.max_depth:
            for child in list(element):
                if self._is_control_node(child):
                    element.remove(child)
            return

        for child in list(element):
            self._prune_deep_branches(child, current_depth + 1)

            if self._is_control_node(child) and len(child) == 0:
                element.remove(child)
    
    def _is_control_node(self, element) -> bool:
        """Check if element is a control node (requires children)"""
        control_nodes = {
            'Sequence', 'Fallback', 'Parallel', 'ReactiveSequence', 
            'ReactiveFallback', 'RecoveryNode', 'PipelineSequence',
            'RateController', 'Inverter', 'ForceSuccess', 'ForceFailure'
        }
        return element.tag in control_nodes
    
    def _salvage_malformed_xml(self, xml_content: str) -> Optional[str]:
        """
        Attempt to salvage malformed XML by finding the first valid complete tree
        
        Strategy:
        1. Find the first </BehaviorTree> tag
        2. Count and balance opening/closing tags up to that point
        3. Add missing closing tags for root element
        """
        try:
            bt_close_match = re.search(r'</BehaviorTree>', xml_content)
            if not bt_close_match:
                logger.warning("Could not find </BehaviorTree> tag")
                return None

            truncated = xml_content[:bt_close_match.end()]

            if not truncated.rstrip().endswith('</root>'):
                truncated += '\n</root>'

            ET.fromstring(truncated)
            return truncated

        except Exception as e:
            logger.warning(f"Failed to salvage XML: {e}")
            return None
    
    def apply_filters(self, xml_content: str) -> Tuple[str, bool, Optional[str]]:
        """
        Apply all filters to generated XML
        
        Returns:
            (filtered_xml, was_modified, reason)
        """
        is_runaway, reason = self.detect_runaway_generation(xml_content)

        if not is_runaway:
            return xml_content, False, None

        logger.warning(f"Runaway generation detected: {reason}")
        logger.warning("Attempting to truncate at valid stopping point...")

        truncated = self.truncate_at_valid_point(xml_content)

        if truncated:
            logger.info("Successfully truncated runaway generation")
            return truncated, True, reason
        else:
            logger.error("Failed to truncate - returning original")
            return xml_content, False, f"Could not fix: {reason}"


def create_default_filter() -> GenerationFilter:
    """Create filter with default settings"""
    return GenerationFilter(
        max_depth=4,
        max_repeated_patterns=3,
        max_total_nodes=20
    )


if __name__ == "__main__":
    # Test the filter with a problematic example
    
    # Example 1: Excessive nesting with action at the end
    bad_xml = """<?xml version="1.0"?>
<root BTCPP_format="4" main_tree_to_execute="Test">
  <BehaviorTree ID="Test">
    <Sequence name="Level1">
      <Sequence name="Level2">
        <Sequence name="Level3">
          <Sequence name="Level4">
            <Sequence name="Level5">
              <Sequence name="Level6">
                <Sequence name="Level7">
                  <Sequence name="Level8">
                    <Sequence name="Level9">
                      <Sequence name="Level10">
                        <Wait wait_duration="1.0"/>
                      </Sequence>
                    </Sequence>
                  </Sequence>
                </Sequence>
              </Sequence>
            </Sequence>
          </Sequence>
        </Sequence>
      </Sequence>
    </Sequence>
  </BehaviorTree>
</root>"""
    
    filter_obj = create_default_filter()
    
    print("Testing excessive nesting detection...")
    is_runaway, reason = filter_obj.detect_runaway_generation(bad_xml)
    print(f"Runaway: {is_runaway}, Reason: {reason}")
    
    if is_runaway:
        print("\nAttempting truncation...")
        filtered, modified, reason = filter_obj.apply_filters(bad_xml)
        print(f"Modified: {modified}")
        print(f"Filtered XML:\n{filtered}")
    
    # Example 2: Repeated patterns
    repeated_xml = """<?xml version="1.0"?>
<root BTCPP_format="4" main_tree_to_execute="Test">
  <BehaviorTree ID="Test">
    <Sequence name="CheckCupPoseCondition1">
      <Sequence name="CheckCupPoseCondition2">
        <Sequence name="CheckCupPoseCondition3">
          <Sequence name="CheckCupPoseCondition4">
            <Sequence name="CheckCupPoseCondition5">
              <Wait wait_duration="1.0"/>
            </Sequence>
          </Sequence>
        </Sequence>
      </Sequence>
    </Sequence>
  </BehaviorTree>
</root>"""
    
    print("\n" + "="*80)
    print("Testing repeated pattern detection...")
    is_runaway, reason = filter_obj.detect_runaway_generation(repeated_xml)
    print(f"Runaway: {is_runaway}, Reason: {reason}")
    
    if is_runaway:
        print("\nAttempting truncation...")
        filtered, modified, reason = filter_obj.apply_filters(repeated_xml)
        print(f"Modified: {modified}")
        print(f"Filtered XML:\n{filtered}")
