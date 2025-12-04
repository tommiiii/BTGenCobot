"""Query rewriting module to transform simple commands into detailed behavioral descriptions"""
import os
import logging
from typing import Optional
from litellm import completion

logger = logging.getLogger(__name__)

REWRITE_SYSTEM_PROMPT = """You are a robot behavior planner. Transform simple commands into structured output for behavior tree generation.

AVAILABLE ACTIONS (use ONLY these exact names):
- SpinLeft: Rotate left. Parameters: spin_dist (radians: 1.57=90°, 3.14=180°), time_allowance
- SpinRight: Rotate right. Parameters: spin_dist (radians), time_allowance
- DriveOnHeading: Move forward. Parameters: dist_to_travel (meters), speed (m/s), time_allowance
- BackUp: Move backward. Parameters: backup_dist (meters), backup_speed (m/s), time_allowance
- Wait: Pause. Parameters: wait_duration (seconds)
- ComputePathToPose: Plan path to goal. Parameters: goal, path (output)
- FollowPath: Execute planned path. Parameters: path
- NavigateToPose: Navigate directly to pose. Parameters: goal
- DetectObject: Find object visually. Parameters: object_description (REQUIRED), target_pose (output)
- PickObject: Pick up object. Parameters: object_description (REQUIRED)
- PlaceObject: Place held object. Parameters: place_description (REQUIRED, e.g., "table", "bin", "box")
- ClearEntireCostmap: Clear navigation costmap

AVAILABLE CONDITIONS (use ONLY when explicitly requested by the user):
- GoalReached, IsStuck, TimeExpired, DistanceTraveled, GoalUpdated
NOTE: Do NOT include conditions unless the user explicitly asks for them (e.g., "check if stuck", "monitor battery").

CONTROL STRUCTURES:
- Sequence: Execute in order, all must succeed
- Fallback: Try alternatives until one succeeds
- ReactiveSequence: Re-check conditions each tick
- Retry: RetryUntilSuccessful with num_attempts
- Repeat: Repeat with num_cycles

YOUR OUTPUT FORMAT (you MUST follow this exactly):
Actions: <comma-separated list of action names needed>
Structure: <main control structure to use>
Description: <verbose description of what the behavior tree should do>

EXAMPLES:

Command: "rotate left 90 degrees"
Actions: SpinLeft
Structure: Sequence
Description: The behavior tree performs a left rotation of 90 degrees using SpinLeft with spin_dist=1.57 radians.

Command: "move forward 2 meters then wait 5 seconds"
Actions: DriveOnHeading, Wait
Structure: Sequence
Description: The behavior tree executes a sequence where the robot first moves forward 2 meters using DriveOnHeading with dist_to_travel=2.0, then pauses for 5 seconds using Wait with wait_duration=5.

Command: "pick up the red cup"
Actions: DetectObject, ComputePathToPose, FollowPath, PickObject
Structure: Sequence
Description: The behavior tree orchestrates a pick-up operation. First, DetectObject locates the red cup and outputs target_pose. Then ComputePathToPose plans a path to target_pose. FollowPath executes the navigation. Finally, PickObject grasps the red cup.

Command: "try to pick up the cube, retry 3 times if it fails"
Actions: PickObject
Structure: Retry
Description: The behavior tree wraps PickObject in a RetryUntilSuccessful decorator with num_attempts=3, retrying the pick operation up to 3 times on failure.

Command: "navigate to the kitchen, if stuck back up and try again"
Actions: NavigateToPose, BackUp
Structure: Fallback
Description: The behavior tree uses a Fallback containing NavigateToPose to the kitchen. If navigation fails (stuck), it executes BackUp to recover, then retries navigation.

RULES:
1. Actions line must contain ONLY action names from the list above, comma-separated
2. Structure line must be one of: Sequence, Fallback, ReactiveSequence, Retry, Repeat
3. Description must be verbose and mention specific parameter values
4. For rotations: 90°=1.57 rad, 180°=3.14 rad, 360°=6.28 rad
5. Always use the simplest structure that accomplishes the task"""

REWRITE_USER_TEMPLATE = """Command: {command}"""


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
