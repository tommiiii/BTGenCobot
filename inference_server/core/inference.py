"""BehaviorTree Generation with CFG-Constrained LLM Inference"""
import time
import logging
import re
from typing import Optional, Dict, Any, Tuple, List
from pathlib import Path

logger = logging.getLogger(__name__)


def generate_restricted_grammar(allowed_actions: List[str], structure: Optional[str] = None, max_depth: int = 5) -> str:
    """
    Generate a restricted EBNF grammar that allows specified actions with full control flow support.
    Uses depth-limited hierarchy to prevent degenerate infinite nesting.
    
    Args:
        allowed_actions: List of allowed action names (e.g., ["SpinLeft", "BackUp", "DetectObject"])
        structure: Optional structure hint (e.g., "Fallback", "Retry", "Repeat") to guide generation
        max_depth: Maximum nesting depth (default 5 levels)
                        
    Returns:
        Complete EBNF grammar string with the specified actions and control flow support
    """
    # Map action names to their grammar rules
    action_rules = {
        "ComputePathToPose": 'compute_path: "<ComputePathToPose" " " goal_attr " " path_attr (" " planner_id_attr)? (" " name_attr)? " "? "/>"',
        "FollowPath": 'follow_path: "<FollowPath" " " path_attr (" " controller_id_attr)? (" " name_attr)? " "? "/>"',
        "NavigateToPose": 'navigate_to_pose: "<NavigateToPose" " " goal_attr (" " behavior_tree_attr)? (" " name_attr)? " "? "/>"',
        "SpinLeft": 'spin_left: "<SpinLeft" " " spin_dist_attr " " time_allowance_attr (" " name_attr)? " "? "/>"',
        "SpinRight": 'spin_right: "<SpinRight" " " spin_dist_attr " " time_allowance_attr (" " name_attr)? " "? "/>"',
        "DriveOnHeading": 'drive_on_heading: "<DriveOnHeading" " " dist_to_travel_attr " " speed_attr " " time_allowance_attr (" " name_attr)? " "? "/>"',
        "BackUp": 'backup: "<BackUp" " " backup_dist_attr " " backup_speed_attr " " time_allowance_attr (" " name_attr)? " "? "/>"',
        "Wait": 'wait: "<Wait" " " wait_duration_attr (" " name_attr)? " "? "/>"',
        "DetectObject": 'detect_obj: "<DetectObject" " " object_description_attr " " detect_target_pose_attr " " detect_object_pose_attr " " detect_object_height_attr " " detect_object_width_attr (" " name_attr)? " "? "/>"',
        "PickObject": 'pick_obj: "<PickObject" " " pick_target_pose_attr " " pick_object_height_attr " " pick_object_width_attr (" " name_attr)? " "? "/>"',
        "PlaceObject": 'place_obj: "<PlaceObject" " " place_target_pose_attr (" " name_attr)? " "? "/>"',
        "ClearEntireCostmap": 'clear_costmap: "<ClearEntireCostmap" (" " costmap_attr)? (" " name_attr)? " "? "/>"'
    }
    
    # Map action names to their rule names for the action_node definition
    action_rule_names = {
        "ComputePathToPose": "compute_path",
        "FollowPath": "follow_path",
        "NavigateToPose": "navigate_to_pose",
        "SpinLeft": "spin_left",
        "SpinRight": "spin_right",
        "DriveOnHeading": "drive_on_heading",
        "BackUp": "backup",
        "Wait": "wait",
        "DetectObject": "detect_obj",
        "PickObject": "pick_obj",
        "PlaceObject": "place_obj",
        "ClearEntireCostmap": "clear_costmap"
    }
    
    # Condition node rules
    condition_rules = {
        "GoalReached": 'goal_reached: "<GoalReached" ((" " | "\\t") attribute)* " "? "/>"',
        "IsStuck": 'is_stuck: "<IsStuck" ((" " | "\\t") attribute)* " "? "/>"',
        "IsBatteryLow": 'is_battery_low: "<IsBatteryLow" ((" " | "\\t") attribute)* " "? "/>"',
        "GoalUpdated": 'goal_updated: "<GoalUpdated" ((" " | "\\t") attribute)* " "? "/>"',
        "TimeExpired": 'time_expired: "<TimeExpired" " " time_expired_attr ((" " | "\\t") attribute)* " "? "/>"',
        "DistanceTraveled": 'distance_traveled: "<DistanceTraveled" " " distance_attr ((" " | "\\t") attribute)* " "? "/>"'
    }
    
    condition_rule_names = {
        "GoalReached": "goal_reached",
        "IsStuck": "is_stuck",
        "IsBatteryLow": "is_battery_low",
        "GoalUpdated": "goal_updated",
        "TimeExpired": "time_expired",
        "DistanceTraveled": "distance_traveled"
    }
    
    if not allowed_actions:
        raise ValueError("No actions provided in allowed_actions")
    
    # Get unique actions and their rule definitions
    unique_actions = list(dict.fromkeys(allowed_actions))  # Preserves order, removes duplicates
    allowed_action_definitions = []
    unique_action_rule_names = []
    
    # Also check for condition nodes in allowed_actions
    allowed_condition_definitions = []
    unique_condition_rule_names = []
    
    for action in unique_actions:
        if action in action_rules:
            unique_action_rule_names.append(action_rule_names[action])
            allowed_action_definitions.append(action_rules[action])
        elif action in condition_rules:
            unique_condition_rule_names.append(condition_rule_names[action])
            allowed_condition_definitions.append(condition_rules[action])
        else:
            logger.warning(f"Unknown action/condition: {action}, skipping")
    
    if not unique_action_rule_names and not unique_condition_rule_names:
        raise ValueError(f"No valid actions/conditions found in allowed_actions: {allowed_actions}")
    
    # Build action_node rule
    if unique_action_rule_names:
        action_node_rule = f'action_node: {" | ".join(unique_action_rule_names)}'
    else:
        action_node_rule = 'action_node: wait  // fallback'
        allowed_action_definitions.append(action_rules["Wait"])
    
    # Build condition_node rule
    has_conditions = bool(unique_condition_rule_names)
    if has_conditions:
        condition_node_rule = f'condition_node: {" | ".join(unique_condition_rule_names)}'
    else:
        condition_node_rule = '// No conditions needed'
    
    logger.info(f"Generated grammar for actions: {unique_action_rule_names}, conditions: {unique_condition_rule_names}, structure: {structure}")
    
    # Base grammar structure with depth-limited control flow support
    # Supports both full XML document format and BehaviorTree-only format (matches finetuned model output)
    grammar = r"""// BehaviorTree XML Grammar - Dynamically Generated with Depth-Limited Control Flow
// Supports nested control nodes with maximum depth to prevent infinite nesting

// Two possible start formats: full XML document or just BehaviorTree element
start: full_document | behavior_tree_only

// Full document with XML declaration
full_document: xml_decl root

xml_decl: "<?xml" WS "version" WS? "=" WS? version_val "?>"
version_val: "\"1.0\""

root: "<root" " " "BTCPP_format=\"4\"" " " main_tree_attr ((" " | "\t") attribute)* " "? ">" WS? behavior_tree WS? "</root>"
main_tree_attr: "main_tree_to_execute=\"" att_value_content "\""

// BehaviorTree element only (matches finetuned model output)
behavior_tree_only: behavior_tree

behavior_tree: "<BehaviorTree" ((" " | "\t") attribute)* " "? ">" WS? bt_content WS? "</BehaviorTree>"

// Root level content - Level 1
bt_content: node_l1

"""
    
    # Build node definitions for each level
    # Levels 1 through max_depth-1 can contain control/decorator nodes
    # Level max_depth (leaf level) only contains actions and conditions
    
    for level in range(1, max_depth + 1):
        is_leaf_level = (level == max_depth)
        next_level = level + 1
        
        # Build node_l{level} definition
        node_options = ["action_node"]
        if has_conditions:
            node_options.append("condition_node")
        
        if not is_leaf_level:
            node_options.extend([f"control_l{level}", f"decorator_l{level}"])
        
        grammar += f"// Level {level}" + (" (leaf level - no more nesting)" if is_leaf_level else "") + "\n"
        grammar += f"node_l{level}: {' | '.join(node_options)}\n\n"
        
        # Add control and decorator nodes for non-leaf levels
        if not is_leaf_level:
            child_node = f"node_l{next_level}"
            child_list = f"node_list_l{next_level}"
            
            # Control nodes at this level
            grammar += f"// Level {level} control nodes\n"
            grammar += f'control_l{level}: sequence_l{level} | fallback_l{level} | parallel_l{level} | reactive_seq_l{level} | reactive_fb_l{level}\n'
            grammar += f'sequence_l{level}: "<Sequence" ((" " | "\\t") attribute)* " "? ">" WS? {child_list} WS? "</Sequence>"\n'
            grammar += f'fallback_l{level}: "<Fallback" ((" " | "\\t") attribute)* " "? ">" WS? {child_list} WS? "</Fallback>"\n'
            grammar += f'parallel_l{level}: "<Parallel" ((" " | "\\t") attribute)* " "? ">" WS? {child_list} WS? "</Parallel>"\n'
            grammar += f'reactive_seq_l{level}: "<ReactiveSequence" ((" " | "\\t") attribute)* " "? ">" WS? {child_list} WS? "</ReactiveSequence>"\n'
            grammar += f'reactive_fb_l{level}: "<ReactiveFallback" ((" " | "\\t") attribute)* " "? ">" WS? {child_list} WS? "</ReactiveFallback>"\n\n'
            
            # Decorator nodes at this level
            grammar += f"// Level {level} decorator nodes\n"
            grammar += f'decorator_l{level}: inverter_l{level} | force_success_l{level} | force_failure_l{level} | repeat_l{level} | retry_l{level} | keep_running_l{level} | rate_ctrl_l{level}\n'
            grammar += f'inverter_l{level}: "<Inverter" ((" " | "\\t") attribute)* " "? ">" WS? {child_node} WS? "</Inverter>"\n'
            grammar += f'force_success_l{level}: "<ForceSuccess" ((" " | "\\t") attribute)* " "? ">" WS? {child_node} WS? "</ForceSuccess>"\n'
            grammar += f'force_failure_l{level}: "<ForceFailure" ((" " | "\\t") attribute)* " "? ">" WS? {child_node} WS? "</ForceFailure>"\n'
            grammar += f'repeat_l{level}: "<Repeat" " " num_cycles_attr ((" " | "\\t") attribute)* " "? ">" WS? {child_node} WS? "</Repeat>"\n'
            grammar += f'retry_l{level}: "<RetryUntilSuccessful" " " num_attempts_attr ((" " | "\\t") attribute)* " "? ">" WS? {child_node} WS? "</RetryUntilSuccessful>"\n'
            grammar += f'keep_running_l{level}: "<KeepRunningUntilFailure" ((" " | "\\t") attribute)* " "? ">" WS? {child_node} WS? "</KeepRunningUntilFailure>"\n'
            grammar += f'rate_ctrl_l{level}: "<RateController" " " hz_attr ((" " | "\\t") attribute)* " "? ">" WS? {child_node} WS? "</RateController>"\n\n'
            
            # Node list at next level
            grammar += f"// Node list for level {next_level}\n"
            grammar += f'{child_list}: {child_node} (WS? {child_node})*\n\n'
    
    # Add condition node rule if we have conditions
    if has_conditions:
        grammar += f"// Condition nodes\n{condition_node_rule}\n\n"
        grammar += "// Condition definitions\n"
        grammar += "\n".join(allowed_condition_definitions) + "\n\n"
    
    # Add action node rule
    grammar += f"// Action nodes (leaf nodes)\n{action_node_rule}\n\n"
    
    # Add all the allowed action rule definitions
    grammar += "// Action definitions\n"
    grammar += "\n".join(allowed_action_definitions) + "\n\n"
    
    # Add attribute definitions - using raw string
    grammar += r"""// Specific parameter definitions for actions
goal_attr: "goal" " "? "=" " "? "\"" goal_var "\""
path_attr: "path" " "? "=" " "? "\"" path_var "\""
planner_id_attr: "planner_id" " "? "=" " "? "\"" att_value_content "\""
controller_id_attr: "controller_id" " "? "=" " "? "\"" att_value_content "\""
behavior_tree_attr: "behavior_tree" " "? "=" " "? "\"" att_value_content "\""
spin_dist_attr: "spin_dist" " "? "=" " "? "\"" numeric_value "\""
dist_to_travel_attr: "dist_to_travel" " "? "=" " "? "\"" numeric_value "\""
speed_attr: "speed" " "? "=" " "? "\"" numeric_value "\""
backup_dist_attr: "backup_dist" " "? "=" " "? "\"" numeric_value "\""
backup_speed_attr: "backup_speed" " "? "=" " "? "\"" numeric_value "\""
time_allowance_attr: "time_allowance" " "? "=" " "? "\"" numeric_value "\""
wait_duration_attr: "wait_duration" " "? "=" " "? "\"" numeric_value "\""
object_description_attr: "object_description" " "? "=" " "? "\"" att_value_content "\""
detect_target_pose_attr: "target_pose" " "? "=" " "? "\"{goal}\""
detect_object_pose_attr: "object_pose" " "? "=" " "? "\"{object_pose}\""
detect_object_height_attr: "object_height" " "? "=" " "? "\"{object_height}\""
detect_object_width_attr: "object_width" " "? "=" " "? "\"{object_width}\""
pick_target_pose_attr: "target_pose" " "? "=" " "? "\"{object_pose}\""
pick_object_height_attr: "object_height" " "? "=" " "? "\"{object_height}\""
pick_object_width_attr: "object_width" " "? "=" " "? "\"{object_width}\""
place_target_pose_attr: "target_pose" " "? "=" " "? "\"{object_pose}\""
costmap_attr: "costmap" " "? "=" " "? "\"" att_value_content "\""
name_attr: "name" " "? "=" " "? "\"" att_value_content "\""

// Decorator node attributes
num_cycles_attr: "num_cycles" " "? "=" " "? "\"" numeric_value "\""
num_attempts_attr: "num_attempts" " "? "=" " "? "\"" numeric_value "\""
hz_attr: "hz" " "? "=" " "? "\"" numeric_value "\""

// Condition node attributes
time_expired_attr: "seconds" " "? "=" " "? "\"" numeric_value "\""
distance_attr: "distance" " "? "=" " "? "\"" numeric_value "\""

// Variable name constraints - force specific variable names
goal_var: "{goal}"
path_var: "{path}"

numeric_value: /[0-9]+\.?[0-9]*/
att_value_content: /[^<&";]*/

// Generic attribute for control nodes and root elements
attribute: att_name " "? "=" " "? "\"" att_value_content "\""
att_name: /[a-zA-Z_][a-zA-Z0-9_]*/

WS: /[ \t\n\r]+/
"""
    
    return grammar


def parse_allowed_actions(rewritten_input: str) -> Tuple[Optional[List[str]], Optional[str]]:
    """
    Parse the 'Actions:' and 'Structure:' lines from rewritten input.
    
    Args:
        rewritten_input: Rewritten input string from query rewriter
        
    Returns:
        Tuple of (list of action names, structure hint string) or (None, None) if not found
        
    Example:
        Input: "Structure: Fallback[...]\nActions: Spin, BackUp\nTask: ..."
        Output: (["Spin", "BackUp"], "Fallback")
    """
    if not rewritten_input:
        return None, None
    
    actions = None
    structure = None
    
    # Look for "Actions:" line
    actions_match = re.search(r'Actions:\s*([^\n]+)', rewritten_input, re.IGNORECASE)
    if actions_match:
        actions_str = actions_match.group(1).strip()
        # Split by comma and clean up
        actions = [action.strip() for action in actions_str.split(',')]
        actions = [action for action in actions if action]  # Remove empty strings
    
    # Look for "Structure:" line to understand control flow needs
    structure_match = re.search(r'Structure:\s*([^\n]+)', rewritten_input, re.IGNORECASE)
    if structure_match:
        structure_str = structure_match.group(1).strip()
        # Extract main control structure type
        if 'Fallback' in structure_str:
            structure = 'Fallback'
        elif 'RetryUntilSuccessful' in structure_str or 'Retry' in structure_str:
            structure = 'Retry'
        elif 'Repeat' in structure_str:
            structure = 'Repeat'
        elif 'ReactiveSequence' in structure_str:
            structure = 'ReactiveSequence'
        elif 'ReactiveFallback' in structure_str:
            structure = 'ReactiveFallback'
        elif 'Sequence' in structure_str:
            structure = 'Sequence'
        else:
            structure = structure_str
    
    if actions:
        logger.info(f"Parsed allowed actions: {actions}, structure: {structure}")
    
    return actions, structure


class BTGenerator:
    """BehaviorTree generator using Transformers + Outlines CFG constraints"""

    def __init__(self, model_path: str, adapter_path: Optional[str] = None):
        """Initialize the BT generator

        Args:
            model_path: Path to the base model directory
            adapter_path: Optional path to the LoRA adapter directory
        """
        self.model_path = Path(model_path)
        self.adapter_path = Path(adapter_path) if adapter_path else None
        self.model = None
        self.tokenizer = None
        self.outlines_model = None
        self.xml_pattern = None
        self.model_loaded = False

        logger.info(f"Initializing with model: {self.model_path}")
        if self.adapter_path:
            logger.info(f"Using adapter: {self.adapter_path}")

    def load_model(self):
        """Load the model with transformers backend"""
        try:
            logger.info("Loading model with transformers...")
            start_time = time.time()

            import torch
            from transformers import AutoTokenizer, AutoModelForCausalLM
            from peft import PeftModel
            import outlines

            logger.info(f"Loading base model from: {self.model_path}")
            self.tokenizer = AutoTokenizer.from_pretrained(str(self.model_path))

            device = "mps" if torch.backends.mps.is_available() else "cpu"
            logger.info(f"Using device: {device}")

            base_model = AutoModelForCausalLM.from_pretrained(
                str(self.model_path),
                torch_dtype=torch.float16 if device == "mps" else torch.float32,
                device_map="auto",
                low_cpu_mem_usage=True
            )

            if self.adapter_path:
                logger.info(f"Loading LoRA adapter from: {self.adapter_path}")
                self.model = PeftModel.from_pretrained(base_model, str(self.adapter_path))
                logger.info("Adapter loaded successfully")
            else:
                self.model = base_model
                logger.info("No adapter specified, using base model only")

            logger.info("Creating Outlines wrapper with transformers backend...")
            self.outlines_model = outlines.from_transformers(
                self.model,  # type: ignore
                self.tokenizer
            )

            grammar_path = Path(__file__).parent.parent / "bt_grammar.ebnf"
            logger.info(f"Loading grammar from: {grammar_path}")

            if not grammar_path.exists():
                raise FileNotFoundError(f"Grammar file not found: {grammar_path}")

            bt_grammar = grammar_path.read_text()

            logger.info("Creating CFG pattern for XML...")
            try:
                from outlines.types import CFG
                self.xml_pattern = CFG(bt_grammar)
                logger.info("XML pattern created successfully (using outlines.types.CFG)")
            except Exception as e:
                logger.warning(f"Failed to create XML CFG pattern: {e}")
                self.xml_pattern = None

            load_time = time.time() - start_time
            self.model_loaded = True

            logger.info(f"Model loaded successfully in {load_time:.2f} seconds")
            logger.info(f"XML pattern available: {self.xml_pattern is not None}")

        except Exception as e:
            logger.error(f"Failed to load model: {str(e)}")
            raise

    def generate_xml(
        self,
        prompt: str,
        max_tokens: int = 2048,
        temperature: float = 0.3,
        custom_grammar: Optional[str] = None
    ) -> Tuple[Optional[str], Optional[str]]:
        """
        Generate BT XML using CFG-constrained generation

        Args:
            prompt: Complete prompt for generation
            max_tokens: Maximum tokens to generate (default 2048, set to None for unlimited)
            temperature: Sampling temperature (default 0.3, lower=more deterministic)
            custom_grammar: Optional custom grammar to use instead of default

        Returns:
            Tuple of (generated_xml, error_message)
        """
        if not self.model_loaded:
            return None, "Model not loaded"

        # Determine which grammar to use
        if custom_grammar:
            grammar_to_use = custom_grammar
            logger.info("Using custom restricted grammar")
        elif self.xml_pattern is not None:
            # Use the default grammar pattern
            grammar_to_use = self.xml_pattern
            logger.info("Using default grammar")
        else:
            return None, "No grammar available"
        
        if self.outlines_model is None:
            return None, "Outlines model not available"

        try:
            start_time = time.time()

            # If custom_grammar is a string, create CFG from it
            if isinstance(grammar_to_use, str):
                from outlines.types import CFG
                logger.info("Creating CFG pattern from custom grammar...")
                cfg_creation_start = time.time()
                cfg_pattern = CFG(grammar_to_use)
                logger.info(f"CFG pattern created in {time.time() - cfg_creation_start:.2f}s")
            else:
                cfg_pattern = grammar_to_use

            # Call the Outlines model directly with the CFG output type
            logger.info("Starting model generation...")
            result = self.outlines_model(
                prompt,
                cfg_pattern,
                max_new_tokens=max_tokens,
                temperature=temperature
            )

            gen_time = time.time() - start_time
            logger.info(f"XML generated in {gen_time:.2f}s")
            
            if not result:
                logger.warning("Model returned empty result")

            return result, None

        except Exception as e:
            error_msg = f"XML generation failed: {str(e)}"
            logger.error(error_msg)
            import traceback
            logger.error(f"Traceback: {traceback.format_exc()}")
            return None, error_msg

    def generate_bt(
        self,
        command: str,
        max_tokens: int = 2048,
        temperature: float = 0.6,
        prompt_format: str = "chat",
        rewritten_input: Optional[str] = None,
        custom_instruction: Optional[str] = None
    ) -> Dict[str, Any]:
        """
        Generate BehaviorTree XML from natural language command

        Args:
            command: Natural language command
            max_tokens: Maximum tokens to generate
            temperature: Sampling temperature
            prompt_format: Format of prompt - "chat" (Llama chat) or "alpaca" (instruction format)
            rewritten_input: Rewritten input from query rewriter (used directly in prompt)
            custom_instruction: Optional custom instruction to override default alpaca_instruction.txt

        Returns:
            Dictionary with:
            {
                "bt_xml": str,
                "generation_time_ms": int,
                "method_used": str,
                "success": bool,
                "error": Optional[str]
            }
        """
        if not self.model_loaded:
            return {
                "bt_xml": None,
                "generation_time_ms": 0,
                "method_used": None,
                "success": False,
                "error": "Model not loaded"
            }

        from prompts import build_prompt, build_alpaca_prompt, extract_xml_from_response
        from validation.validator import validate_bt_xml, validate_action_space, validate_semantic_structure
        from validation.post_processor import create_default_filter

        start_time = time.time()

        if self.xml_pattern is not None:
            logger.info(f"Generating BT for: {command}")

            # Parse allowed actions from rewritten input if available
            custom_grammar = None
            if rewritten_input:
                allowed_actions, structure = parse_allowed_actions(rewritten_input)
                if allowed_actions:
                    try:
                        grammar_str = generate_restricted_grammar(allowed_actions, structure)
                        custom_grammar = grammar_str
                        logger.info(f"Using restricted grammar with actions: {allowed_actions}, structure: {structure}")
                        logger.debug(f"Generated grammar:\n{grammar_str}")
                    except Exception as e:
                        logger.warning(f"Failed to generate restricted grammar: {e}, using default")
                        custom_grammar = None

            if prompt_format == "alpaca":
                prompt = build_alpaca_prompt(command, rewritten_input=rewritten_input, custom_instruction=custom_instruction)
            else:
                prompt = build_prompt(command, output_format="xml", tokenizer=self.tokenizer)

            logger.info(f"=== PROMPT ===\n{prompt}\n=== END PROMPT ===")
            xml_result, error = self.generate_xml(prompt, max_tokens, temperature, custom_grammar=custom_grammar)

            if xml_result:
                xml_result = extract_xml_from_response(xml_result)

                filter_obj = create_default_filter()
                filtered_xml, was_modified, filter_reason = filter_obj.apply_filters(xml_result)

                if was_modified:
                    logger.info(f"Post-processing applied: {filter_reason}")
                    xml_result = filtered_xml

                is_valid, val_error = validate_bt_xml(xml_result, strict=False)

                if not is_valid:
                    logger.warning(f"Validation failed: {val_error}")
                    gen_time_ms = int((time.time() - start_time) * 1000)
                    return {
                        "bt_xml": xml_result,
                        "generation_time_ms": gen_time_ms,
                        "method_used": "cfg",
                        "success": False,
                        "error": f"Validation failed: {val_error}"
                    }

                action_valid, action_issues = validate_action_space(xml_result)
                if not action_valid:
                    logger.warning(f"Invalid actions: {'; '.join(action_issues)}")
                    gen_time_ms = int((time.time() - start_time) * 1000)
                    return {
                        "bt_xml": xml_result,
                        "generation_time_ms": gen_time_ms,
                        "method_used": "cfg",
                        "success": False,
                        "error": f"Invalid actions: {'; '.join(action_issues)}"
                    }

                semantic_valid, semantic_warnings = validate_semantic_structure(xml_result)

                gen_time_ms = int((time.time() - start_time) * 1000)
                logger.info(f"✓ BT generated successfully in {gen_time_ms}ms")

                all_warnings = []
                if was_modified:
                    all_warnings.append(filter_reason)
                if semantic_warnings:
                    all_warnings.extend(semantic_warnings)

                warning_msg = "; ".join(all_warnings) if all_warnings else None

                return {
                    "bt_xml": xml_result,
                    "generation_time_ms": gen_time_ms,
                    "method_used": "cfg",
                    "success": True,
                    "error": None,
                    "warning": warning_msg
                }
            else:
                logger.warning(f"XML generation failed: {error}")
                gen_time_ms = int((time.time() - start_time) * 1000)
                return {
                    "bt_xml": None,
                    "generation_time_ms": gen_time_ms,
                    "method_used": "cfg",
                    "success": False,
                    "error": error
                }

        gen_time_ms = int((time.time() - start_time) * 1000)
        return {
            "bt_xml": None,
            "generation_time_ms": gen_time_ms,
            "method_used": None,
            "success": False,
            "error": "All generation methods failed"
        }


# Global generator instance
_generator: Optional[BTGenerator] = None


def get_generator() -> BTGenerator:
    """Get or create the global generator instance"""
    global _generator
    if _generator is None:
        from core.config import get_model_path, get_adapter_path
        _generator = BTGenerator(get_model_path(), get_adapter_path())
    return _generator


def initialize_generator():
    """Initialize and load the generator"""
    generator = get_generator()
    if not generator.model_loaded:
        generator.load_model()
    return generator
