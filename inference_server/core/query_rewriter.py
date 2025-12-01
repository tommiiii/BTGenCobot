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
- SpinLeft: spin_dist="X.XX" (RADIANS, always POSITIVE. Examples: 1.57=90°, 3.14=180°) [For LEFT/COUNTERCLOCKWISE rotation]
- SpinRight: spin_dist="X.XX" (RADIANS, always POSITIVE. Examples: 1.57=90°, 3.14=180°) [For RIGHT/CLOCKWISE rotation]
- DriveOnHeading: dist_to_travel="X.X", speed="X.X"
  * dist_to_travel is ALWAYS POSITIVE for forward movement
  * Example: dist_to_travel="2.0", speed="0.2" for moving forward 2 meters
- BackUp: backup_dist="X.X", backup_speed="X.X"
  * backup_dist is ALWAYS POSITIVE (robot moves backward)
  * Example: backup_dist="1.0", backup_speed="0.1" for moving backward 1 meter
- Wait: wait_duration="X" [For pausing]
- DetectObject: object_description="..." [For vision-based object detection and localization]
  * Outputs: target_pose (approach position, 0.5m from object), object_pose (actual object location)
  * DO NOT specify output parameters in action description
- PickObject: [For grasping objects with the manipulator]
  * Uses object_pose from DetectObject (actual object location)
- PlaceObject: [For placing objects with the manipulator]
  * Can use object_pose or a specified location
- ClearEntireCostmap: costmap="global" or "local" [For clearing costmaps when stuck]

Control Nodes (for complex behaviors):
- Sequence: Execute children in order. All must succeed.
- Fallback: Try children in order until one succeeds.
- ReactiveSequence: Re-evaluates from first child each tick.
- ReactiveFallback: Re-evaluates from first child each tick.
- Parallel: Execute children simultaneously.

Decorator Nodes (modify child behavior):
- Inverter: Invert child result.
- Repeat: num_cycles="N" - Repeat child N times.
- RetryUntilSuccessful: num_attempts="N" - Retry child until success.
- ForceSuccess: Always return SUCCESS.
- KeepRunningUntilFailure: Keep running until child fails.

Condition Nodes (check state):
- GoalReached: Check if navigation goal reached.
- IsStuck: Check if robot is stuck.
- IsBatteryLow: Check battery level.
- TimeExpired: seconds="N" - Check if time exceeded.

CRITICAL:
- Use these EXACT parameter names. Do NOT use variations like "angle", "spin_angle", "radians", etc.
- For DetectObject, PickObject, PlaceObject: Only mention the object_description value (e.g., "red cup"), never mention target_pose, object_pose, or other output parameters
"""

REWRITE_SYSTEM_PROMPT = f"""Transform simple robot commands into detailed behavioral descriptions for behavior tree generation.

{AVAILABLE_ACTIONS}

Rules:
1. Mention exact parameter VALUES in the description (not just names)
2. ROTATION DIRECTION: Use SpinLeft for left/counterclockwise rotation, SpinRight for right/clockwise rotation
3. For navigating to NAMED locations (e.g., "kitchen", "bedroom"): List BOTH ComputePathToPose AND FollowPath
4. For navigating to DETECTED OBJECTS (e.g., "red cup", "blue box"): List DetectObject, ComputePathToPose, FollowPath (and PickObject/PlaceObject if needed)
5. For blind forward movement by distance (no target): Use DriveOnHeading
6. For blind backward movement by distance: Use BackUp
7. List ONLY the actions needed
8. CRITICAL: If the same action is used multiple times, list that action MULTIPLE times in the Actions line

CONTROL FLOW RULES:
9. For "if X fails, do Y" or "try X, otherwise Y": Use Structure: Fallback[X, Y]
10. For "repeat N times": Use Structure: Repeat[action_sequence]
11. For "keep trying until success": Use Structure: RetryUntilSuccessful[action]
12. For "while not stuck, do X": Use Structure: ReactiveSequence[Inverter[IsStuck], X]
13. For recovery behaviors: Use Fallback with main action and recovery sequence

Output format (MUST start with Actions or Structure line):

For SIMPLE sequences (just actions in order):
Actions: [All action instances needed, comma-separated, in execution order]
[Concise description mentioning specific parameter values]

For COMPLEX behaviors (requiring control structures):
Structure: ControlNode[children...]
Actions: [All unique actions used]
[Concise description of the behavior tree structure and parameter values]

Examples:
- For "rotate left 90 degrees": "Actions: SpinLeft" (with spin_dist="1.57" for 90° left rotation)
- For "rotate right 90 degrees": "Actions: SpinRight" (with spin_dist="1.57" for 90° right rotation)
- For "move forward 2m": "Actions: DriveOnHeading" (with dist_to_travel="2.0", speed="0.2")
- For "move backward 1m": "Actions: BackUp" (with backup_dist="1.0", backup_speed="0.1")
- For "turn right then turn left": "Actions: SpinRight, SpinLeft" (first SpinRight with spin_dist="1.57", then SpinLeft with spin_dist="1.57")
- For "turn around 180 degrees": "Actions: SpinLeft" (with spin_dist="3.14") or "Actions: SpinRight" (with spin_dist="3.14")
- For "go to kitchen": "Actions: ComputePathToPose, FollowPath" (both needed for navigation to named location)
- For "get closer to the red cup": "Actions: DetectObject, ComputePathToPose, FollowPath" (detect object, then navigate to it)
- For "pick up red box": "Actions: DetectObject, ComputePathToPose, FollowPath, PickObject" (detect, navigate, pick)
- For "place the cup on the table": "Actions: DetectObject, ComputePathToPose, FollowPath, PlaceObject" (detect table, navigate, place)

COMPLEX BEHAVIOR EXAMPLES:
- For "navigate to kitchen, if stuck back up and try again":
  Structure: Fallback[Sequence[ComputePathToPose, FollowPath], Sequence[BackUp, ComputePathToPose, FollowPath]]
  Actions: ComputePathToPose, FollowPath, BackUp
  Navigate to kitchen using path planning. If navigation fails, back up 0.5m at 0.1m/s, then retry navigation.

- For "try to detect the cup 3 times":
  Structure: RetryUntilSuccessful[DetectObject]
  Actions: DetectObject
  Retry detecting the cup up to 3 attempts (num_attempts="3").

- For "patrol between kitchen and bedroom 5 times":
  Structure: Repeat[Sequence[navigate_to_kitchen, navigate_to_bedroom]]
  Actions: ComputePathToPose, FollowPath
  Repeat patrol sequence 5 times (num_cycles="5"). Each cycle: navigate to kitchen, then navigate to bedroom.

- For "search for the red ball by spinning":
  Structure: Fallback[DetectObject, Sequence[SpinLeft, DetectObject]]
  Actions: DetectObject, SpinLeft
  Try to detect the red ball. If not found, spin left 180° and try detection again."""

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
