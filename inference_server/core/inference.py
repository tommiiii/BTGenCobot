"""BehaviorTree Generation with CFG-Constrained LLM Inference"""
import time
import logging
import re
from typing import Optional, Dict, Any, Tuple, List
from pathlib import Path

logger = logging.getLogger(__name__)


def generate_restricted_grammar(allowed_actions: List[str]) -> str:
    """
    Generate a restricted EBNF grammar that only allows specified actions.
    
    Args:
        allowed_actions: List of allowed action names in execution order (e.g., ["Spin", "BackUp", "Spin"])
                        Can include duplicates for multiple instances of same action
        
    Returns:
        Complete EBNF grammar string with only the specified actions
    """
    # Map action names to their grammar rules
    action_rules = {
        "ComputePathToPose": 'compute_path: "<ComputePathToPose" " " goal_attr " " path_attr (" " planner_id_attr)? (" " name_attr)? " "? "/>"',
        "FollowPath": 'follow_path: "<FollowPath" " " path_attr (" " controller_id_attr)? (" " name_attr)? " "? "/>"',
        "Spin": 'spin: "<Spin" " " spin_dist_attr " " time_allowance_attr (" " name_attr)? " "? "/>"',
        "DriveOnHeading": 'drive_on_heading: "<DriveOnHeading" " " dist_to_travel_attr " " speed_attr " " time_allowance_attr (" " name_attr)? " "? "/>"',
        "BackUp": 'backup: "<BackUp" " " backup_dist_attr " " backup_speed_attr " " time_allowance_attr (" " name_attr)? " "? "/>"',
        "Wait": 'wait: "<Wait" " " wait_duration_attr (" " name_attr)? " "? "/>"',
        "DetectObject": 'detect_obj: "<DetectObject" " " object_description_attr " " target_pose_attr (" " name_attr)? " "? "/>"',
        "PickObject": 'pick_obj: "<PickObject" " " target_pose_attr (" " name_attr)? " "? "/>"',
        "PlaceObject": 'place_obj: "<PlaceObject" " " target_pose_attr (" " name_attr)? " "? "/>"'
    }
    
    # Map action names to their rule names for the action_node definition
    action_rule_names = {
        "ComputePathToPose": "compute_path",
        "FollowPath": "follow_path",
        "Spin": "spin",
        "DriveOnHeading": "drive_on_heading",
        "BackUp": "backup",
        "Wait": "wait",
        "DetectObject": "detect_obj",
        "PickObject": "pick_obj",
        "PlaceObject": "place_obj"
    }
    
    if not allowed_actions:
        raise ValueError("No actions provided in allowed_actions")
    
    # Get unique actions and their rule definitions
    unique_actions = list(dict.fromkeys(allowed_actions))  # Preserves order, removes duplicates
    allowed_rule_definitions = []
    unique_rule_names = []
    
    for action in unique_actions:
        if action not in action_rules:
            logger.warning(f"Unknown action: {action}, skipping")
            continue
        unique_rule_names.append(action_rule_names[action])
        allowed_rule_definitions.append(action_rules[action])
    
    if not unique_rule_names:
        raise ValueError(f"No valid actions found in allowed_actions: {allowed_actions}")
    
    # Build sequence based on exact action order
    action_sequence = []
    for action in allowed_actions:
        if action in action_rule_names:
            action_sequence.append(action_rule_names[action])
    
    # Generate bt_content rule based on action count and types
    if len(action_sequence) == 1:
        # Single action - wrap in Sequence for consistent structure
        bt_content_rule = f'bt_content: "<Sequence" ((" " | "\\t") attribute)* " "? ">" WS? {action_sequence[0]} WS? "</Sequence>"'
    else:
        # Multiple actions - create explicit sequence
        sequence_parts = " WS? ".join(action_sequence)
        bt_content_rule = f'bt_content: "<Sequence" ((" " | "\\t") attribute)* " "? ">" WS? {sequence_parts} WS? "</Sequence>"'
    
    logger.info(f"Generated grammar for action sequence: {action_sequence}")
    
    # Base grammar structure (same for all) - using raw string to preserve escapes
    base_grammar = r"""// BehaviorTree XML Grammar - Dynamically Restricted
// Based on W3C XML 1.0 specification, adapted for Lark EBNF syntax

start: xml_decl root

xml_decl: "<?xml" WS "version" WS? "=" WS? version_val "?>"
version_val: "\"1.0\""

root: "<root" " " "BTCPP_format=\"4\"" " " main_tree_attr ((" " | "\t") attribute)* " "? ">" WS? behavior_tree WS? "</root>"
main_tree_attr: "main_tree_to_execute=\"" att_value_content "\""

behavior_tree: "<BehaviorTree" ((" " | "\t") attribute)* " "? ">" WS? bt_content WS? "</BehaviorTree>"

// Root level: can be a single action OR a control node
"""
    
    # Add the bt_content rule
    grammar = base_grammar + bt_content_rule + "\n\n"
    
    # Note: bt_content already defines the complete structure we need,
    # so we don't need additional control node definitions
    
    # Add all the allowed action rule definitions
    grammar += "// Allowed action definitions\n"
    grammar += "\n".join(allowed_rule_definitions) + "\n\n"
    
    # Add attribute definitions (all actions might need these) - using raw string
    grammar += r"""// Specific parameter definitions
goal_attr: "goal" " "? "=" " "? "\"" att_value_content "\""
path_attr: "path" " "? "=" " "? "\"" att_value_content "\""
planner_id_attr: "planner_id" " "? "=" " "? "\"" att_value_content "\""
controller_id_attr: "controller_id" " "? "=" " "? "\"" att_value_content "\""
spin_dist_attr: ("spin_dist" " "? "=" " "? "\"-" numeric_value "\"") | ("spin_dist" " "? "=" " "? "\"" numeric_value "\"")
dist_to_travel_attr: "dist_to_travel" " "? "=" " "? "\"" numeric_value "\""
speed_attr: "speed" " "? "=" " "? "\"" numeric_value "\""
backup_dist_attr: "backup_dist" " "? "=" " "? "\"" numeric_value "\""
backup_speed_attr: "backup_speed" " "? "=" " "? "\"" numeric_value "\""
time_allowance_attr: "time_allowance" " "? "=" " "? "\"" numeric_value "\""
wait_duration_attr: "wait_duration" " "? "=" " "? "\"" numeric_value "\""
object_description_attr: "object_description" " "? "=" " "? "\"" att_value_content "\""
target_pose_attr: "target_pose" " "? "=" " "? "\"" att_value_content "\""
name_attr: "name" " "? "=" " "? "\"" att_value_content "\""

numeric_value: /[0-9]+\.?[0-9]*/
att_value_content: /[^<&";]*/

// Generic attribute for control nodes and root elements
attribute: att_name " "? "=" " "? "\"" att_value_content "\""
att_name: /[a-zA-Z_][a-zA-Z0-9_]*/

WS: /[ \t\n\r]+/
"""
    
    return grammar


def parse_allowed_actions(rewritten_input: str) -> Optional[List[str]]:
    """
    Parse the 'Actions:' line from rewritten input to extract allowed actions.
    
    Args:
        rewritten_input: Rewritten input string from query rewriter
        
    Returns:
        List of action names, or None if no Actions: line found
        
    Example:
        Input: "Actions: Spin\\nTask: Rotate the robot..."
        Output: ["Spin"]
    """
    if not rewritten_input:
        return None
    
    # Look for "Actions:" line
    match = re.search(r'Actions:\s*([^\n]+)', rewritten_input, re.IGNORECASE)
    if not match:
        return None
    
    actions_str = match.group(1).strip()
    
    # Split by comma and clean up
    actions = [action.strip() for action in actions_str.split(',')]
    actions = [action for action in actions if action]  # Remove empty strings
    
    if not actions:
        return None
    
    logger.info(f"Parsed allowed actions from rewritten input: {actions}")
    return actions


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
                cfg_pattern = CFG(grammar_to_use)
            else:
                cfg_pattern = grammar_to_use

            # Call the Outlines model directly with the CFG output type
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
            return None, error_msg

    def generate_bt(
        self,
        command: str,
        max_tokens: int = 2048,
        temperature: float = 0.6,
        use_few_shot: bool = True,
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
            use_few_shot: Whether to include few-shot examples in prompt
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
                allowed_actions = parse_allowed_actions(rewritten_input)
                if allowed_actions:
                    try:
                        grammar_str = generate_restricted_grammar(allowed_actions)
                        custom_grammar = grammar_str
                        logger.info(f"Using restricted grammar with actions: {allowed_actions}")
                        logger.debug(f"Generated grammar:\n{grammar_str}")
                    except Exception as e:
                        logger.warning(f"Failed to generate restricted grammar: {e}, using default")
                        custom_grammar = None

            if prompt_format == "alpaca":
                prompt = build_alpaca_prompt(command, use_few_shot=use_few_shot, rewritten_input=rewritten_input, custom_instruction=custom_instruction)
            else:
                prompt = build_prompt(command, use_few_shot=use_few_shot, output_format="xml", tokenizer=self.tokenizer)

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
