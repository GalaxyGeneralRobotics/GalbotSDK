"""Execute absolute-joint VLA action chunks with Galbot SDK interfaces.

VLA control requires observations and actions to stay aligned in time. This
example uses ``get_synced_observation`` to acquire a synchronized camera image
and robot joint state for each model request. It then uses
``set_joint_commands`` to stream the model's 23-dimensional absolute-joint
action chunk frame by frame at the configured control frequency.

For the training-data workflow, Galbot officially supports demonstration data
collection with leader-follower arms. See the TM01 user manual:
https://developer.galbot.com/docs/tm01/1.4.0/zh/tm01
The collected MCAP data can be converted to the LeRobot dataset format with
galbot-mcap2lerobot for subsequent model training:
https://github.com/GalaxyGeneralRobotics/galbot-mcap2lerobot
After deploying the trained model as an inference service, use this example as
a reference for acquiring synchronized observations and executing its action
chunks on the robot.

This example demonstrates absolute joint-position model output. If a model
instead outputs end-effector poses (EE poses such as
``[x, y, z, qx, qy, qz, qw]``), use ``set_end_effector_command`` for real-time
Cartesian control. See
``examples/g1/python/galbot_robot/set_end_effector_commands.py`` for that API.

Before running, release the emergency-stop button and clear the area around the
robot. Keep a hand near the emergency-stop button during execution.
"""

import json
import time
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

import numpy as np

try:
    from galbot_sdk.g1 import (
        ControlStatus,
        G1JointGroup,
        GalbotRobot,
        JointCommand,
        SensorType,
    )
except ImportError:
    print(
        "Failed to import galbot_sdk. Install the SDK or add it to PYTHONPATH "
        "before running this example."
    )
    raise SystemExit(1)


# The order of these groups defines the meaning of every action vector:
# 5 + 2 + 7 + 1 + 7 + 1 = 23 dimensions.
ACTION_GROUPS = [
    "leg",
    "head",
    "left_arm",
    "left_gripper",
    "right_arm",
    "right_gripper",
]

GROUP_SLICES = {
    "leg": slice(0, 5),
    "head": slice(5, 7),
    "left_arm": slice(7, 14),
    "left_gripper": slice(14, 15),
    "right_arm": slice(15, 22),
    "right_gripper": slice(22, 23),
}

GROUP_JOINT_NAMES = {
    "leg": [f"leg_joint{i}" for i in range(1, 6)],
    "head": [f"head_joint{i}" for i in range(1, 3)],
    "left_arm": [f"left_arm_joint{i}" for i in range(1, 8)],
    "left_gripper": ["left_gripper_joint1"],
    "right_arm": [f"right_arm_joint{i}" for i in range(1, 8)],
    "right_gripper": ["right_gripper_joint1"],
}

CONTROL_FREQUENCY_HZ = 20.0
# Number of model action frames returned and executed in one chunk.
ACTION_CHUNK_SIZE = 30
GRIPPER_VELOCITY = 0.1
GRIPPER_EFFORT = 10.0
TASK_PROMPT = "Based on the current view, predict the next robot action."
# Shared by the G1 Python and C++ tutorial examples.
ACTION_JSON_PATH = (
    Path(__file__).resolve().parents[2] / "assets" / "tutorials" / "example5_vla.json"
)


def confirm_safe_environment() -> None:
    """Require an explicit confirmation before sending robot commands."""
    print(
        "WARNING: Release the emergency-stop button, keep the robot in working "
        "mode, and clear people and obstacles from its motion range."
    )
    answer = input("Continue with the VLA execution example? [y/N]: ").strip().lower()
    if answer not in {"y", "yes"}:
        raise SystemExit("Execution cancelled by the user.")


def action_joint_names() -> List[str]:
    """Return joint names in exactly the same order as an action vector."""
    return [
        joint_name for group in ACTION_GROUPS for joint_name in GROUP_JOINT_NAMES[group]
    ]


def collect_observation(robot: GalbotRobot) -> Dict[str, object]:
    """Collect the synchronized image and state normally sent to a VLA server."""
    observation = robot.get_synced_observation(
        [SensorType.HEAD_LEFT_CAMERA],
        True,
    )
    if not observation:
        raise RuntimeError("get_synced_observation failed")

    image_message = observation.rgb_data_map.get(SensorType.HEAD_LEFT_CAMERA)
    if image_message is None:
        raise RuntimeError("Synchronized head-left image is missing")
    if observation.joint_state is None:
        raise RuntimeError("Synchronized joint state is missing")

    state_by_name = {
        joint_state.joint_name: joint_state
        for joint_state in observation.joint_state.joint_state_vec
    }
    missing_names = [name for name in action_joint_names() if name not in state_by_name]
    if missing_names:
        raise RuntimeError(f"Joint state is missing joints: {missing_names}")

    state = np.asarray(
        [state_by_name[name].position for name in action_joint_names()],
        dtype=np.float32,
    )
    return {
        "state": state,
        "image": bytes(image_message.data),
        "prompt": TASK_PROMPT,
    }


def load_mock_actions(json_path: Path) -> np.ndarray:
    """Load the action frames returned by the simulated VLA service."""
    if not json_path.is_file():
        raise FileNotFoundError(f"Mock VLA action file does not exist: {json_path}")

    with json_path.open("r", encoding="utf-8") as file:
        payload = json.load(file)

    actions = np.asarray(payload.get("frames"), dtype=np.float32)
    if payload.get("num_frames") != len(actions):
        raise ValueError(
            "Mock num_frames does not match the number of frames in the JSON: "
            f"{payload.get('num_frames')} != {len(actions)}"
        )
    return validate_action_chunk(actions)


def mock_vla_server(
    observation: Dict[str, object],
    all_actions: np.ndarray,
    cursor: int,
    chunk_size: int = ACTION_CHUNK_SIZE,
) -> Tuple[Dict[str, np.ndarray], int]:
    """Simulate one model request and return the next JSON action chunk.

    A real VLA service would replace this function and infer actions from the
    observation. This local version directly returns consecutive absolute joint
    frames loaded from the copied client-demo JSON file.
    """
    state = np.asarray(observation["state"], dtype=np.float32)
    expected_dim = len(action_joint_names())
    if state.shape != (expected_dim,):
        raise ValueError(
            f"Observation state has shape {state.shape}; expected ({expected_dim},)"
        )

    start = int(cursor)
    end = min(start + int(chunk_size), len(all_actions))
    actions = all_actions[start:end]
    print(
        "Mock VLA server received one request: "
        f"prompt={observation['prompt']!r}, "
        f"image_bytes={len(observation['image'])}, frames={start}:{end}"
    )
    print(f"Mock VLA server returned action chunk: shape={actions.shape}")
    return {"actions": actions}, end


def move_to_first_action(robot: GalbotRobot, first_action: np.ndarray) -> None:
    """Move safely to the absolute pose used by the copied JSON trajectory."""
    # Stabilize the lower body first, then move both arms and the head together.
    for position_groups in (["leg"], ["head", "left_arm", "right_arm"]):
        joint_positions = np.concatenate(
            [first_action[GROUP_SLICES[group]] for group in position_groups]
        ).tolist()
        status = robot.set_joint_positions(
            joint_positions=joint_positions,
            joint_groups=position_groups,
            joint_names=[],
            is_blocking=True,
            speed_rad_s=0.2,
            timeout_s=10.0,
        )
        if status != ControlStatus.SUCCESS:
            raise RuntimeError(
                f"Failed to move {position_groups} to the first JSON action: {status}"
            )

    gripper_targets = (
        (G1JointGroup.left_gripper, first_action[GROUP_SLICES["left_gripper"]][0]),
        (
            G1JointGroup.right_gripper,
            first_action[GROUP_SLICES["right_gripper"]][0],
        ),
    )
    for joint_group, position in gripper_targets:
        status = robot.set_gripper_command(
            end_effector=joint_group,
            width_m=float(position),
            velocity_mps=0.1,
            effort=GRIPPER_EFFORT,
            is_blocking=True,
        )
        if status != ControlStatus.SUCCESS:
            raise RuntimeError(
                f"Failed to move {joint_group} to the first JSON action: {status}"
            )

    print("Robot reached the first pose in the mock VLA action file.")


def validate_action_chunk(actions: np.ndarray) -> np.ndarray:
    """Validate the model result before converting it to SDK commands."""
    actions = np.asarray(actions, dtype=np.float32)
    expected_dim = len(action_joint_names())
    if actions.ndim != 2 or actions.shape[1] != expected_dim:
        raise ValueError(
            "VLA actions must have shape (T, D), where "
            f"D={expected_dim}; got {actions.shape}"
        )
    if not np.all(np.isfinite(actions)):
        raise ValueError("VLA actions contain NaN or infinity")
    return actions


def build_joint_commands(action: Sequence[float]) -> List[JointCommand]:
    """Convert one model action frame to SDK ``JointCommand`` objects."""
    values = list(float(value) for value in action)
    expected_dim = len(action_joint_names())
    if len(values) != expected_dim:
        raise ValueError(f"Action dimension is {len(values)}; expected {expected_dim}")

    gripper_indices = {
        action_joint_names().index("left_gripper_joint1"),
        action_joint_names().index("right_gripper_joint1"),
    }
    commands = []
    for index, value in enumerate(values):
        command = JointCommand()
        command.position = value
        if index in gripper_indices:
            command.velocity = GRIPPER_VELOCITY
            command.effort = GRIPPER_EFFORT
        commands.append(command)
    return commands


def execute_action_chunk(robot: GalbotRobot, actions: np.ndarray) -> None:
    """Stream one VLA action chunk with the high-frequency SDK interface."""
    actions = validate_action_chunk(actions)
    if actions.shape[0] == 0:
        print("The VLA model returned an empty chunk; execution is complete.")
        return

    period_s = 1.0 / CONTROL_FREQUENCY_HZ
    next_tick = time.monotonic()
    print(
        f"Streaming {actions.shape[0]} frames at {CONTROL_FREQUENCY_HZ:.1f} Hz "
        "with set_joint_commands..."
    )

    for frame_index, action in enumerate(actions, start=1):
        status = robot.set_joint_commands(
            joint_commands=build_joint_commands(action),
            joint_groups=ACTION_GROUPS,
            joint_names=[],
            time_from_start_s=0.0,
        )
        if status != ControlStatus.SUCCESS:
            raise RuntimeError(
                f"set_joint_commands failed at frame {frame_index}: {status}"
            )

        next_tick += period_s
        remaining_s = next_tick - time.monotonic()
        if remaining_s > 0.0:
            time.sleep(remaining_s)

    print("Action chunk execution completed.")


def main() -> None:
    """Request and execute JSON action chunks until the mock service is empty."""
    confirm_safe_environment()
    robot = GalbotRobot()
    initialized = False

    try:
        if not robot.init({SensorType.HEAD_LEFT_CAMERA}, True):
            raise RuntimeError("GalbotRobot initialization failed")
        initialized = True

        # Allow the first synchronized image and joint state to become ready.
        time.sleep(5.0)

        all_actions = load_mock_actions(ACTION_JSON_PATH)
        print(f"Loaded mock VLA actions from: {ACTION_JSON_PATH}")
        print(f"Action data shape: {all_actions.shape} (full23, including head)")
        move_to_first_action(robot, all_actions[0])

        cursor = 0
        while True:
            observation = collect_observation(robot)
            response, cursor = mock_vla_server(
                observation,
                all_actions,
                cursor,
            )
            if response["actions"].shape[0] == 0:
                print("Mock VLA server has no more actions.")
                break
            execute_action_chunk(robot, response["actions"])

        print("VLA execution example finished successfully.")
    finally:
        if initialized:
            robot.request_shutdown()
            robot.wait_for_shutdown()
            robot.destroy()
            print("SDK resources released.")


if __name__ == "__main__":
    main()
