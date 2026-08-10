"""Query rewriting module to transform simple commands into detailed behavioral descriptions"""
import json
import os
import re
import logging
from typing import Optional
from litellm import completion

logger = logging.getLogger(__name__)

REWRITE_SYSTEM_PROMPT = """You are a robot behavior planner. Transform commands into structured output for behavior tree generation.

AVAILABLE ACTIONS (use ONLY these exact names):
- SpinLeft: Rotate left. Parameters: spin_dist (radians: 1.57=90°, 3.14=180°), time_allowance
- SpinRight: Rotate right. Parameters: spin_dist (radians), time_allowance
- DriveOnHeading: Move forward. Parameters: dist_to_travel (meters), speed (m/s), time_allowance
- BackUp: Move backward. Parameters: backup_dist (meters), backup_speed (m/s), time_allowance
- Wait: Pause. Parameters: wait_duration (seconds)
- ComputePathToPose: Plan path to goal. Parameters: goal, path (output)
- FollowPath: Execute planned path. Parameters: path
- NavigateToPose: Navigate directly to pose. Parameters: goal
- NavigateSemantic: Navigate using persistent Hydra scene-graph memory. Parameters: entity_ref (REQUIRED, formatted as "room:label" or "object:label"), entity_type, reacquire
- DetectObject: Find object visually. Parameters: object_description (REQUIRED), target_pose (output), object_pose (output)
- PickObject: Tilt the head, detect locally once, and pick up an object. Parameters: object_description (REQUIRED)
- PlaceObject: Place held object. Parameters: place_description (REQUIRED, e.g., "table", "bin", "box")
- ClearEntireCostmap: Clear navigation costmap

IMPORTANT EXECUTION CONSTRAINTS:
- NavigateSemantic uses the persistent graph for the long-range approach to a named source or destination.
- PlaceObject uses support geometry resolved from the scene graph after the robot has approached the destination.
- For a named place target, use NavigateSemantic for that target immediately before PlaceObject.
- Do NOT add DetectObject, ComputePathToPose, FollowPath, or NavigateToPose before PlaceObject.
- Use NavigateSemantic for named rooms and remembered objects. Never invent metric coordinates or Hydra node IDs.
- For pickup, NavigateSemantic resolves the source through Hydra or one live navigation fallback with reacquire=true, then PickObject performs a fresh close-range measurement and grasps.
- Use NavigateToPose only when the user explicitly supplies a metric pose.

AVAILABLE CONDITIONS (use when the task requires checking state):
- GoalReached, IsStuck, IsBatteryLow, TimeExpired, DistanceTraveled, GoalUpdated

CONTROL STRUCTURES:
- Sequence: Execute in order, all must succeed
- Fallback: Try alternatives until one succeeds
- ReactiveSequence: Re-check conditions each tick
- Retry: RetryUntilSuccessful with num_attempts
- Repeat: Repeat with num_cycles

OUTPUT FORMAT - Output EXACTLY these 4 lines, nothing else:
Actions: <comma-separated action names>
Structure: <control structure>
SemanticRefs: <one entity_ref="room:label" or entity_ref="object:label" for every NavigateSemantic, in action order; or none>
Description: <what the tree does>

EXAMPLES:

Command: "rotate left 90 degrees"
Actions: SpinLeft
Structure: Sequence
SemanticRefs: none
Description: The behavior tree performs a left rotation of 90 degrees using SpinLeft with spin_dist=1.57 radians.

Command: "move forward 2 meters then wait 5 seconds"
Actions: DriveOnHeading, Wait
Structure: Sequence
SemanticRefs: none
Description: The behavior tree executes a sequence where the robot first moves forward 2 meters using DriveOnHeading with dist_to_travel=2.0, then pauses for 5 seconds using Wait with wait_duration=5.

Command: "pick up the red cup"
Actions: NavigateSemantic, PickObject
Structure: Sequence
SemanticRefs: entity_ref="object:red cup"
Description: NavigateSemantic first approaches entity_ref="object:red cup" using scene-graph memory. PickObject then tilts the head, performs one fresh local detection, and grasps the red cup.

Command: "pick up the blue ball then place it on the box"
Actions: NavigateSemantic, PickObject, NavigateSemantic, PlaceObject
Structure: Sequence
SemanticRefs: entity_ref="object:blue ball", entity_ref="object:box"
Description: NavigateSemantic approaches entity_ref="object:blue ball", PickObject grasps it, NavigateSemantic approaches entity_ref="object:box", and PlaceObject places the held object.

Command: "try to pick up the cube, retry 3 times if it fails"
Actions: PickObject, BackUp
Structure: Retry
SemanticRefs: none
Description: The behavior tree wraps PickObject in a RetryUntilSuccessful decorator with num_attempts=3, with BackUp as recovery action.

Command: "navigate to the kitchen, if stuck back up and try again"
Actions: NavigateSemantic, BackUp
Structure: Fallback
SemanticRefs: entity_ref="room:kitchen"
Description: The behavior tree uses a Fallback containing NavigateSemantic with entity_ref="room:kitchen". If navigation fails, it executes BackUp to recover, then retries.

Command: "go to charging station, but if battery is not low just wait"
Actions: NavigateSemantic, Wait, IsBatteryLow
Structure: Fallback
SemanticRefs: entity_ref="room:charging station"
Description: The behavior tree uses a Fallback. First checks IsBatteryLow condition - if true, NavigateSemantic uses entity_ref="room:charging station". If battery is fine, just Wait.

Command: "place the object on the green bin"
Actions: NavigateSemantic, PlaceObject
Structure: Sequence
SemanticRefs: entity_ref="object:green bin"
Description: NavigateSemantic approaches entity_ref="object:green bin" using scene-graph memory. PlaceObject then uses the resolved support geometry and places the held object.

RULES:
1. Output ONLY the 4 lines (Actions, Structure, SemanticRefs, Description) - no explanations, no options, no questions
2. Actions must be from the available list - approximate if needed, never refuse
3. If a requested condition isn't available, use the closest match or omit it
4. SemanticRefs must contain exactly one typed entity_ref for each NavigateSemantic, in the same order
5. Copy each semantic label from the command; never replace it with an example or generic term
6. Always produce valid output - never ask for clarification"""

REWRITE_USER_TEMPLATE = """Command: {command}"""


def _extract_location(text: str) -> str:
    """Extract a location/goal name from a phrase."""
    for strip in ["go to", "navigate to", "move to", "travel to", "go back to",
                  "return to", "back to", "head to"]:
        text = text.replace(strip, "").strip()
    # remove leading 'the' only once
    text = re.sub(r'^the\s+', '', text.strip(), flags=re.IGNORECASE)
    return text.strip(" .,") or "goal"


def _semantic_type(label: str, scene_graph_context: Optional[str]) -> str:
    """Resolve a destination against Hydra's current room/object labels."""
    try:
        context = json.loads(scene_graph_context or "")
    except (TypeError, json.JSONDecodeError):
        return "room"
    normalized = label.strip().lower()
    objects = {str(value).strip().lower() for value in context.get("objects", [])}
    rooms = {str(value).strip().lower() for value in context.get("rooms", [])}
    if normalized in objects:
        return "object"
    if normalized in rooms:
        return "room"
    return "object" if context.get("ready") else "room"


def _local_rewrite(
    command: str,
    scene_graph_context: Optional[str] = None,
) -> str:
    """
    Offline keyword-based fallback for query rewriting.
    Handles simple sequences AND complex Fallback/Recovery/Retry patterns.
    Produces the structured format (Actions / Structure / Description)
    that the fine-tuned model expects.
    """
    cmd = command.lower()

    # ── COMPLEX PATTERNS (checked first) ───────────────────────────────────

    # Pattern: retry N times  →  RetryUntilSuccessful
    retry_match = re.search(r"retr(?:y|ies?)(?: up to| at most)?\s*(\d+)\s*times?", cmd)
    if retry_match:
        n = retry_match.group(1)
        # What are we retrying? Extract the base action from the part before "retry"
        base_cmd = command[:retry_match.start()].strip(" ,;")
        base = _local_rewrite(base_cmd, scene_graph_context) if base_cmd else None
        if base:
            actions_line = base.split("\n")[0]  # "Actions: ..."
            desc_line = base.split("Description:")[-1].strip()
            actions = actions_line.replace("Actions:", "").strip()
            return (
                f"Actions: {actions}, BackUp\n"
                f"Structure: Retry\n"
                f"Description: The behavior tree wraps the following operation in a "
                f"RetryUntilSuccessful decorator with num_attempts={n}: {desc_line} "
                f"If it fails, BackUp is used as recovery before retrying."
            )

    # Pattern: "if fails / if it fails / if blocked / if stuck → do Y" or
    #          "try X, otherwise Y" / "X else Y" / "X or else go to Y"
    _fallback_splits = [
        "if that fails", "if it fails", "if fails",
        "otherwise", "or else", ", else", "if blocked",
        "if you can't", "if unable", "if not possible",
        "if the door is closed", "if you cannot",
    ]
    split_keyword = None
    split_pos = -1
    for kw in _fallback_splits:
        pos = cmd.find(kw)
        if pos != -1 and (split_pos == -1 or pos < split_pos):
            split_keyword = kw
            split_pos = pos

    if split_keyword and split_pos > 5:
        main_part = command[:split_pos].strip(" ,;")
        recovery_part = command[split_pos + len(split_keyword):].strip(" ,;")

        main_desc = (
            _local_rewrite(main_part, scene_graph_context) if main_part else None
        )
        recovery_desc = (
            _local_rewrite(recovery_part, scene_graph_context)
            if recovery_part
            else None
        )

        if main_desc and recovery_desc:
            main_actions = main_desc.split("\n")[0].replace("Actions:", "").strip()
            main_text = main_desc.split("Description:")[-1].strip()
            rec_actions = recovery_desc.split("\n")[0].replace("Actions:", "").strip()
            rec_text = recovery_desc.split("Description:")[-1].strip()

            all_actions = ", ".join(dict.fromkeys(
                [a.strip() for a in (main_actions + ", " + rec_actions).split(",") if a.strip()]
            ))
            return (
                f"Actions: {all_actions}\n"
                f"Structure: Fallback\n"
                f"Description: The behavior tree uses a Fallback node with two branches. "
                f"Primary branch: {main_text} "
                f"If the primary branch fails, the recovery branch executes: {rec_text}"
            )

    # Pattern: "X, if stuck → back up"  (with or without leading 'and')
    stuck_match = re.search(r"(?:,\s*|\band\s+)(?:if (?:it (?:gets? )?)?stuck|if (?:it (?:is )?)?blocked)", cmd)
    if stuck_match:
        main_part = command[:stuck_match.start()].strip(" ,;")
        main_desc = (
            _local_rewrite(main_part, scene_graph_context) if main_part else None
        )
        if main_desc:
            main_actions = main_desc.split("\n")[0].replace("Actions:", "").strip()
            main_text = main_desc.split("Description:")[-1].strip()
            return (
                f"Actions: {main_actions}, ClearEntireCostmap, BackUp\n"
                f"Structure: Fallback\n"
                f"Description: The behavior tree uses a Fallback node. "
                f"Primary branch: {main_text} "
                f"If stuck, the recovery branch clears the costmap with ClearEntireCostmap "
                f"and backs up with BackUp."
            )

    # Pattern: "go to A, if fails go to B" without explicit keyword — detect
    # two semantic navigation targets separated by comma/then
    nav_targets = re.findall(
        r"(?:go to|navigate to|move to|head to|go back to|return to)\s+([\w\s]+?)(?:,|\.|;|$)",
        cmd
    )
    if len(nav_targets) >= 2:
        loc_a = nav_targets[0].strip()
        loc_b = nav_targets[1].strip()
        # Only use Fallback if there's a conditional hint between them
        between = cmd[cmd.find(loc_a) + len(loc_a):cmd.find(loc_b)]
        if any(w in between for w in ["if", "or", "else", "can't", "cannot", "fail", "instead"]):
            return (
                f"Actions: NavigateSemantic, NavigateSemantic\n"
                f"Structure: Fallback\n"
                f"Description: The behavior tree uses a Fallback node. "
                f"Primary branch: NavigateSemantic uses entity_ref=\"room:{loc_a}\". "
                f"If that fails, the recovery branch uses NavigateSemantic "
                f"with entity_ref=\"room:{loc_b}\" instead."
            )

    # ── SIMPLE PATTERNS ─────────────────────────────────────────────────────

    if (
        any(token in cmd for token in ["move forward", "go forward", "drive"])
        and "wait" in cmd
    ):
        distance = re.search(
            r"(?:move forward|go forward|drive)(?:\s+for)?\s+"
            r"(\d+(?:\.\d+)?)",
            cmd,
        )
        duration = re.search(
            r"wait(?:\s+for)?\s+(\d+(?:\.\d+)?)",
            cmd,
        )
        return (
            "Actions: DriveOnHeading, Wait\n"
            "Structure: Sequence\n"
            "Description: DriveOnHeading moves forward "
            f"{distance.group(1) if distance else '1.0'} meters, then Wait "
            f"pauses for {duration.group(1) if duration else '2.0'} seconds."
        )

    # ------------------------------------------------------------------ pick
    if any(w in cmd for w in ["pick up", "pick", "grab", "grasp", "take"]):
        pick_match = re.search(
            r"\b(?:pick up|pick|grab|grasp|take)\s+(?:the\s+)?"
            r"(.+?)(?=\s+(?:(?:and\s+)?then\s+|and\s+)?"
            r"(?:place|put|set down|deposit)\b|$)",
            command,
            flags=re.IGNORECASE,
        )
        obj = pick_match.group(1).strip(" .,") if pick_match else "object"
        place_match = re.search(
            r"\b(?:place|put|set down|deposit)"
            r"(?:\s+it|\s+the object)?\s+"
            r"(?:on|onto|in|into|at)\s+(?:the\s+)?(.+)$",
            command,
            flags=re.IGNORECASE,
        )
        if place_match:
            destination = place_match.group(1).strip(" .,")
            return (
                f"Actions: NavigateSemantic, PickObject, NavigateSemantic, PlaceObject\n"
                f"Structure: Sequence\n"
                f"Description: The behavior tree orchestrates a pick-and-place operation. "
                f"NavigateSemantic approaches entity_ref=\"object:{obj}\" from persistent scene-graph memory. "
                f"PickObject tilts the head, performs one fresh local detection, and grasps the {obj}. "
                f"NavigateSemantic then approaches entity_ref=\"object:{destination}\" from persistent scene-graph memory. "
                f"PlaceObject performs one local surface/depth estimate and places it on the {destination}."
            )
        return (
            f"Actions: NavigateSemantic, PickObject\n"
            f"Structure: Sequence\n"
            f"Description: The behavior tree orchestrates a pick-up operation. "
            f"NavigateSemantic approaches entity_ref=\"object:{obj}\" from persistent scene-graph memory. "
            f"PickObject tilts the head, performs one fresh local detection, and grasps the {obj}."
        )

    # ------------------------------------------------------------------ back up / back away
    if re.match(r'^back\s*(?:up|away|off)?$', cmd.strip()):
        return (
            "Actions: BackUp\nStructure: Sequence\n"
            "Description: The behavior tree backs the robot up using BackUp with backup_dist=0.5."
        )

    # ----------------------------------------------------------------- place
    if any(w in cmd for w in ["place", "put", "set down", "deposit"]):
        for trigger in ["place", "put", "set down", "deposit"]:
            idx = cmd.find(trigger)
            if idx != -1:
                dest = command[idx + len(trigger):].strip().lstrip("the ").strip() or "table"
                break
        destination_match = re.search(
            r"(?:on|onto|in|into|at)\s+(?:the\s+)?(.+)$",
            dest,
            flags=re.IGNORECASE,
        )
        if destination_match:
            dest = destination_match.group(1).strip(" .,")
        return (
            f"Actions: NavigateSemantic, PlaceObject\n"
            f"Structure: Sequence\n"
            f"Description: The behavior tree places the held object. "
            f"NavigateSemantic approaches entity_ref=\"object:{dest}\" using scene-graph memory. "
            f"PlaceObject then performs one local surface/depth estimate and deposits the object at the {dest}."
        )

    # ---------------------------------------------------------------- navigate
    if any(w in cmd for w in ["navigate", "go to", "move to", "travel to"]):
        dest = cmd.split("to")[-1].strip() if "to" in cmd else "goal"
        dest = re.sub(r'^the\s+', '', dest, flags=re.IGNORECASE).strip(" .,") or "goal"
        entity_type = _semantic_type(dest, scene_graph_context)
        return (
            f"Actions: NavigateSemantic\n"
            f"Structure: Sequence\n"
            f'Description: NavigateSemantic navigates to '
            f'entity_ref="{entity_type}:{dest}".'
        )

    # ----------------------------------------------------------------- rotate
    if any(w in cmd for w in ["spin", "rotat", "turn left", "turn right"]):
        if "right" in cmd:
            return (
                "Actions: SpinRight\nStructure: Sequence\n"
                "Description: The behavior tree rotates the robot to the right using SpinRight with spin_dist=1.57."
            )
        return (
            "Actions: SpinLeft\nStructure: Sequence\n"
            "Description: The behavior tree rotates the robot to the left using SpinLeft with spin_dist=1.57."
        )

    # ------------------------------------------------------------------ move
    if any(w in cmd for w in ["move forward", "drive", "go forward"]):
        dist = "1.0"
        for part in cmd.split():
            try:
                dist = str(float(part))
                break
            except ValueError:
                pass
        return (
            f"Actions: DriveOnHeading\nStructure: Sequence\n"
            f"Description: The behavior tree drives the robot forward {dist} meters using DriveOnHeading."
        )

    # ----------------------------------------------------------------- detect
    if any(w in cmd for w in ["detect", "find", "look for", "search"]):
        obj = cmd.split("for")[-1].strip() if "for" in cmd else "object"
        return (
            f"Actions: DetectObject\nStructure: Sequence\n"
            f"Description: The behavior tree detects an object. "
            f"DetectObject locates the {obj} and outputs target_pose."
        )

    # ----------------------------------------------------------------- wait
    if "wait" in cmd:
        duration = "5"
        for part in cmd.split():
            try:
                duration = str(int(float(part)))
                break
            except ValueError:
                pass
        return (
            f"Actions: Wait\nStructure: Sequence\n"
            f"Description: The behavior tree pauses the robot for {duration} seconds using Wait."
        )

    # ---------------------------------------------------------------- generic
    return (
        f"Actions: NavigateToPose\nStructure: Sequence\n"
        f"Description: The behavior tree executes the command: {command}. "
        f"NavigateToPose drives the robot to the goal."
    )


class QueryRewriter:
    """Rewrites simple commands into detailed behavioral descriptions"""

    def __init__(
        self,
        model: str = "openrouter/deepseek/deepseek-v4-flash",
        api_base: str = "https://openrouter.ai/api/v1",
        max_tokens: int = 500,
        temperature: float = 0.0
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

    def rewrite_query(
        self,
        command: str,
        scene_graph_context: Optional[str] = None,
    ) -> Optional[str]:
        """
        Rewrite a simple command into detailed behavioral description

        Args:
            command: Simple robot command (e.g., "rotate left")

        Returns:
            Rewritten text ready to use as model input, or None if failed
        """
        # The inference server is designed to run fully locally. Avoid a slow,
        # guaranteed-to-fail OpenRouter request when no optional API key was
        # configured; the deterministic local rewrite is also preferable for
        # constrained generation in that case.
        simple_navigation = re.fullmatch(
            r"\s*(?:navigate|go|move|travel)\s+to\s+.+",
            command,
            flags=re.IGNORECASE,
        )
        if simple_navigation or not os.getenv("OPENROUTER_API_KEY"):
            logger.info("Using deterministic local query rewriter")
            return _local_rewrite(command, scene_graph_context)

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

            return content

        except Exception as e:
            logger.error(f"Query rewriting failed: {e}")
            logger.info("Falling back to local keyword-based rewriter")
            return _local_rewrite(command, scene_graph_context)


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


def rewrite_command(
    command: str,
    scene_graph_context: Optional[str] = None,
) -> Optional[str]:
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

    return _rewriter.rewrite_query(command, scene_graph_context)
