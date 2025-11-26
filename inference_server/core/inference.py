"""BehaviorTree Generation with CFG-Constrained LLM Inference"""
import time
import logging
from typing import Optional, Dict, Any, Tuple
from pathlib import Path

logger = logging.getLogger(__name__)


class BTGenerator:
    """BehaviorTree generator using Transformers + Outlines CFG constraints"""

    def __init__(self, model_path: str, adapter_path: str = None):
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
                self.model,
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
                logger.info("XML pattern created successfully (using CFG)")
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
        temperature: float = 0.3
    ) -> Tuple[Optional[str], Optional[str]]:
        """
        Generate BT XML using CFG-constrained generation

        Args:
            prompt: Complete prompt for generation
            max_tokens: Maximum tokens to generate (default 2048, set to None for unlimited)
            temperature: Sampling temperature (default 0.3, lower=more deterministic)

        Returns:
            Tuple of (generated_xml, error_message)
        """
        if not self.model_loaded:
            return None, "Model not loaded"

        if self.xml_pattern is None:
            return None, "XML pattern not available"

        try:
            logger.info(f"Generating XML with CFG constraints (temperature={temperature})...")
            start_time = time.time()

            result = self.outlines_model(
                prompt,
                output_type=self.xml_pattern,
                max_new_tokens=max_tokens,
                temperature=temperature
            )

            gen_time = time.time() - start_time
            logger.info(f"XML generation completed in {gen_time:.2f}s")

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
        rewritten_input: Optional[str] = None
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
            logger.info(f"Attempting XML generation for command: {command}")
            logger.info(f"Using few-shot examples: {use_few_shot}")
            logger.info(f"Prompt format: {prompt_format}")

            if prompt_format == "alpaca":
                prompt = build_alpaca_prompt(command, use_few_shot=use_few_shot, rewritten_input=rewritten_input)
            else:
                prompt = build_prompt(command, use_few_shot=use_few_shot, output_format="xml", tokenizer=self.tokenizer)

            logger.info(f"Prompt length: {len(prompt)} chars")

            xml_result, error = self.generate_xml(prompt, max_tokens, temperature)

            if xml_result:
                xml_result = extract_xml_from_response(xml_result)

                logger.info("=" * 80)
                logger.info("Generated XML:")
                logger.info(xml_result)
                logger.info("=" * 80)

                filter_obj = create_default_filter()
                filtered_xml, was_modified, filter_reason = filter_obj.apply_filters(xml_result)

                if was_modified:
                    logger.warning(f"Post-processing applied: {filter_reason}")
                    logger.info("Filtered XML:")
                    logger.info(filtered_xml)
                    xml_result = filtered_xml

                is_valid, val_error = validate_bt_xml(xml_result, strict=False)

                if not is_valid:
                    logger.warning(f"Generated XML validation failed: {val_error}")
                    logger.warning("Invalid XML content:")
                    logger.warning(xml_result)
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
                    logger.error("Action space validation failed:")
                    for issue in action_issues:
                        logger.error(f"  - {issue}")
                    gen_time_ms = int((time.time() - start_time) * 1000)
                    return {
                        "bt_xml": xml_result,
                        "generation_time_ms": gen_time_ms,
                        "method_used": "cfg",
                        "success": False,
                        "error": f"Invalid actions: {'; '.join(action_issues)}"
                    }

                semantic_valid, semantic_warnings = validate_semantic_structure(xml_result)
                if semantic_warnings:
                    logger.warning("Semantic validation warnings:")
                    for warning in semantic_warnings:
                        logger.warning(f"  - {warning}")

                gen_time_ms = int((time.time() - start_time) * 1000)
                logger.info("XML generation succeeded")

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
