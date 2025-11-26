"""Query rewriting module to transform simple commands into detailed behavioral descriptions"""
import os
import logging
from typing import Optional
from litellm import completion

logger = logging.getLogger(__name__)

AVAILABLE_ACTIONS = """
Available Actions (use EXACT parameter names shown):
- ComputePathToPose: goal="{goal}", path="{path}", planner_id="GridBased"
- FollowPath: path="{path}", controller_id="FollowPath"
- Spin: spin_dist="X.XX" (in radians: 90°=1.57, 180°=3.14, left=positive, right=negative)
- BackUp: backup_dist="X.X", backup_speed="X.X"
- Wait: wait_duration="X"
- DetectObject: object_description="...", target_pose="{target_pose}"
- PickObject: target_pose="{target_pose}"
- PlaceObject: target_pose="{target_pose}"

CRITICAL: Use these EXACT parameter names. Do NOT use variations like "angle", "spin_angle", "radians", etc.
"""

REWRITE_SYSTEM_PROMPT = f"""Transform simple robot commands into detailed behavioral descriptions for behavior tree generation.

{AVAILABLE_ACTIONS}

Rules:
1. Mention exact parameter VALUES in the description (not just names)
2. Rotation: left = positive value, right = negative value (90°=1.57, 180°=3.14)
3. For navigation: List BOTH ComputePathToPose AND FollowPath
4. List ONLY the actions needed

Output format:
[Concise description with specific radian values, e.g. "spin_dist of -1.57 radians"]

Actions: [Only action names, comma-separated]"""

REWRITE_USER_TEMPLATE = """Transform this simple robot command into a detailed behavioral description:

Command: {command}

Provide the detailed description and required actions."""


class QueryRewriter:
    """Rewrites simple commands into detailed behavioral descriptions"""

    def __init__(
        self,
        model: str = "openrouter/anthropic/claude-haiku-4.5",
        api_base: str = "https://openrouter.ai/api/v1",
        max_tokens: int = 500,
        temperature: float = 0.3
    ):
        """
        Initialize query rewriter

        Args:
            model: Model to use via LiteLLM (OpenRouter format)
            api_base: API base URL for OpenRouter
            max_tokens: Maximum tokens for response
            temperature: Temperature for generation
        """
        self.model = model
        self.api_base = api_base
        self.max_tokens = max_tokens
        self.temperature = temperature

        # Check for API key
        if not os.getenv("OPENROUTER_API_KEY"):
            logger.warning("OPENROUTER_API_KEY not found in environment variables")

    def rewrite_query(self, command: str) -> Optional[str]:
        """
        Rewrite a simple command into detailed behavioral description

        Args:
            command: Simple robot command (e.g., "rotate left")

        Returns:
            Rewritten text ready to use as model input, or None if failed
        """
        try:
            user_prompt = REWRITE_USER_TEMPLATE.format(command=command)

            logger.info(f"Rewriting query: {command}")

            response = completion(
                model=self.model,
                messages=[
                    {"role": "system", "content": REWRITE_SYSTEM_PROMPT},
                    {"role": "user", "content": user_prompt}
                ],
                api_base=self.api_base,
                max_tokens=self.max_tokens,
                temperature=self.temperature
            )

            content = response.choices[0].message.content.strip()
            logger.info(f"Rewrite response: {content}")

            # Return raw response - no parsing
            return content

        except Exception as e:
            logger.error(f"Query rewriting failed: {e}")
            return None


# Global rewriter instance
_rewriter: Optional[QueryRewriter] = None


def initialize_rewriter(**kwargs) -> QueryRewriter:
    """Initialize the global query rewriter"""
    global _rewriter
    _rewriter = QueryRewriter(**kwargs)
    return _rewriter


def get_rewriter() -> Optional[QueryRewriter]:
    """Get the global query rewriter instance"""
    return _rewriter


def rewrite_command(command: str) -> Optional[str]:
    """
    Convenience function to rewrite a command using the global rewriter

    Args:
        command: Simple robot command

    Returns:
        Rewritten text ready to use as model input, or None if failed
    """
    if _rewriter is None:
        logger.warning("Query rewriter not initialized")
        return None

    return _rewriter.rewrite_query(command)
