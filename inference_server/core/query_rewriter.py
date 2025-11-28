"""Query rewriting module to transform simple commands into detailed behavioral descriptions"""
import os
import logging
from typing import Optional
from litellm import completion

logger = logging.getLogger(__name__)

AVAILABLE_ACTIONS = """
Available Actions (use EXACT parameter names shown):
- ComputePathToPose: goal="{goal}", path="{path}", planner_id="GridBased" [For navigation to NAMED LOCATIONS like "kitchen", "bedroom"]
- FollowPath: path="{path}", controller_id="FollowPath" [Always follows ComputePathToPose]
- Spin: spin_dist="X.XX" (RADIANS: left/CCW=POSITIVE, right/CW=NEGATIVE. Examples: 1.57=90°left, -1.57=90°right, 3.14=180°left, -3.14=180°right) [For rotation in place]
- DriveOnHeading: dist_to_travel="X.X", speed="X.X"
  * dist_to_travel is ALWAYS POSITIVE for forward movement
  * Example: dist_to_travel="2.0", speed="0.2" for moving forward 2 meters
- BackUp: backup_dist="X.X", backup_speed="X.X"
  * backup_dist is ALWAYS POSITIVE (robot moves backward)
  * Example: backup_dist="1.0", backup_speed="0.1" for moving backward 1 meter
- Wait: wait_duration="X" [For pausing]
- DetectObject: object_description="..." [For vision-based object detection - DO NOT specify target_pose parameter]
- PickObject: [For grasping objects - DO NOT specify parameters]
- PlaceObject: [For placing objects - DO NOT specify parameters]

CRITICAL:
- Use these EXACT parameter names. Do NOT use variations like "angle", "spin_angle", "radians", etc.
- For DetectObject, PickObject, PlaceObject: Only mention the object_description value (e.g., "red cup"), never mention target_pose or other output parameters
"""

REWRITE_SYSTEM_PROMPT = f"""Transform simple robot commands into detailed behavioral descriptions for behavior tree generation.

{AVAILABLE_ACTIONS}

Rules:
1. Mention exact parameter VALUES in the description (not just names)
2. ROTATION DIRECTION: Left/CCW = POSITIVE spin_dist, Right/CW = NEGATIVE spin_dist (MUST include minus sign for right turns!)
3. For navigating to NAMED locations (e.g., "kitchen", "bedroom"): List BOTH ComputePathToPose AND FollowPath
4. For navigating to DETECTED OBJECTS (e.g., "red cup", "blue box"): List DetectObject, ComputePathToPose, FollowPath (and PickObject/PlaceObject if needed)
5. For blind forward movement by distance (no target): Use DriveOnHeading
6. For blind backward movement by distance: Use BackUp
7. List ONLY the actions needed
8. CRITICAL: If the same action is used multiple times, list that action MULTIPLE times in the Actions line

Output format (MUST start with Actions line):
Actions: [All action instances needed, comma-separated, in execution order. List the same action MULTIPLE times if it's used multiple times]

[Concise description mentioning specific parameter values for EACH action instance]

Examples:
- For "rotate left 90 degrees": "Actions: Spin" (with POSITIVE spin_dist="1.57" for counterclockwise/left rotation)
- For "rotate right 90 degrees": "Actions: Spin" (with NEGATIVE spin_dist="-1.57" for clockwise/right rotation - note the minus sign!)
- For "move forward 2m": "Actions: DriveOnHeading" (with dist_to_travel="2.0", speed="0.2")
- For "move backward 1m": "Actions: BackUp" (with backup_dist="1.0", backup_speed="0.1")
- For "turn right then turn left": "Actions: Spin, Spin" (TWO Spin actions - first with NEGATIVE spin_dist="-1.57" for right, second with POSITIVE spin_dist="1.57" for left)
- For "turn around 180 degrees": "Actions: Spin" (with spin_dist="3.14" or spin_dist="-3.14" depending on which direction)
- For "go to kitchen": "Actions: ComputePathToPose, FollowPath" (both needed for navigation to named location)
- For "get closer to the red cup": "Actions: DetectObject, ComputePathToPose, FollowPath" (detect object, then navigate to it)
- For "pick up red box": "Actions: DetectObject, ComputePathToPose, FollowPath, PickObject" (detect, navigate, pick)
- For "place the cup on the table": "Actions: DetectObject, ComputePathToPose, FollowPath, PlaceObject" (detect table, navigate, place)"""

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
