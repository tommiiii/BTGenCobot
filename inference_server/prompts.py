"""Prompt Templates for BehaviorTree Generation"""
import json
from pathlib import Path
from typing import Optional

_PROMPTS_DIR = Path(__file__).parent / "prompts"

def _load_prompt_file(filename: str) -> str:
    """Load prompt content from file"""
    file_path = _PROMPTS_DIR / filename
    return file_path.read_text().strip()

def _load_few_shot_examples() -> list:
    """Load few-shot examples from JSON"""
    file_path = _PROMPTS_DIR / "few_shot_examples.json"
    return json.loads(file_path.read_text())

SYSTEM_PROMPT = _load_prompt_file("system_prompt.txt")
ALPACA_INSTRUCTION = _load_prompt_file("alpaca_instruction.txt")
AVAILABLE_ACTIONS = _load_prompt_file("available_actions.txt")
FEW_SHOT_EXAMPLES = _load_few_shot_examples()


def build_prompt(command: str, use_few_shot: bool = True, output_format: str = "xml", tokenizer=None) -> str:
    """
    Build the complete prompt for BT generation using Llama chat format

    Args:
        command: Natural language command from user
        use_few_shot: Whether to include few-shot examples
        output_format: "xml" for direct XML, "json" for JSON intermediate
        tokenizer: Tokenizer with chat template (if None, returns raw text)

    Returns:
        Complete prompt string formatted for Llama chat
    """
    messages = [
        {"role": "system", "content": SYSTEM_PROMPT}
    ]

    if use_few_shot:
        for example in FEW_SHOT_EXAMPLES:
            messages.append({
                "role": "user",
                "content": f"Command: {example['command']}"
            })
            messages.append({
                "role": "assistant",
                "content": example['xml']
            })

    messages.append({
        "role": "user",
        "content": f"Command: {command}"
    })

    if tokenizer is not None:
        try:
            prompt = tokenizer.apply_chat_template(
                messages,
                tokenize=False,
                add_generation_prompt=True
            )
            return prompt
        except Exception as e:
            logger = __import__('logging').getLogger(__name__)
            logger.warning(f"Failed to apply chat template: {e}, falling back to raw format")

    prompt = ""
    for msg in messages:
        role = msg["role"].upper()
        content = msg["content"]
        prompt += f"<|{role}|>\n{content}\n\n"
    prompt += "<|ASSISTANT|>\n"

    return prompt


def build_alpaca_prompt(command: str, use_few_shot: bool = False, rewritten_input: Optional[str] = None, custom_instruction: Optional[str] = None) -> str:
    """
    Build Alpaca prompt matching the training data format

    Args:
        command: Natural language command from user (used if rewritten_input not provided)
        use_few_shot: Whether to include few-shot examples
        rewritten_input: Rewritten input from query rewriter (used directly)
        custom_instruction: Optional custom instruction to override ALPACA_INSTRUCTION

    Returns:
        Alpaca-formatted prompt string
    """
    # System instruction (use custom if provided, otherwise default)
    instruction = custom_instruction if custom_instruction is not None else ALPACA_INSTRUCTION
    prompt = f"### Instruction:\n{instruction}\n\n"

    # Add few-shot examples if requested
    if use_few_shot:
        for example in FEW_SHOT_EXAMPLES:
            # Examples now already contain "Actions:" format
            prompt += f"### Input:\n{example['command']}\n\n"
            prompt += f"### Response:\n{example['xml']}\n\n"

    # User input
    if rewritten_input:
        # Use rewritten input directly (from query rewriter)
        prompt += f"### Input:\n{rewritten_input}\n\n"
    else:
        # Fallback: basic format with command
        prompt += f"### Input:\nThe behavior tree should: {command}\n\n{AVAILABLE_ACTIONS}\n\n"

    prompt += "### Response:\n"

    return prompt


def extract_xml_from_response(response: str) -> str:
    """
    Extract XML from model response (in case model adds extra text)
    Also fixes common XML declaration errors

    Args:
        response: Raw model output

    Returns:
        Extracted XML string
    """
    import re

    xml_pattern = r'(<\?xml.*?</root>)'
    match = re.search(xml_pattern, response, re.DOTALL)

    if match:
        xml_content = match.group(1)
    elif response.strip().startswith('<?xml'):
        xml_content = response.strip()
    else:
        return response

    xml_content = re.sub(r'<\?xml\s+version\s*=\s*"([^"]+)"\?\?>', r'<?xml version="\1"?>', xml_content)
    xml_content = re.sub(r'<\?xml\s+version\s*=\s*"([^"]+)\?"?\?>', r'<?xml version="\1"?>', xml_content)

    return xml_content


if __name__ == "__main__":
    test_command = "Pick up the green bottle and place it in the recycling bin"

    print("=== XML Generation Prompt ===")
    print(build_prompt(test_command, use_few_shot=True))
