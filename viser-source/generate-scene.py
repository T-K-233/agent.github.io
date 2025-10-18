"""
python generate-scene.py
"""

from pathlib import Path
import time

import numpy as np
from robot_descriptions.loaders.yourdfpy import load_robot_description
import imageio.v3 as iio

import viser
from viser.extras import ViserUrdf


if __name__ == "__main__":
    server = viser.ViserServer()

    server.scene.set_background_image(
        iio.imread("./viser-source/scene-background.jpg"),
        format="auto",
    )

    server.gui.configure_theme(
        dark_mode=True,
        show_logo=False,
    )

    # Create grid.
    server.scene.add_grid(
        "/grid",
        width=2,
        height=2,
        position=(
            0.0,
            0.0,
            0.0,
        ),
    )

    robot_base = server.scene.add_frame("/robot", show_axes=False)
    urdf = load_robot_description(
        "g1_description",
        load_meshes=True,
        build_scene_graph=True,
        load_collision_meshes=False,
        build_collision_scene_graph=False,
    )
    viser_urdf = ViserUrdf(
        server,
        urdf_or_path=urdf,
        root_node_name="/robot",
    )

    motion_file = "viser-source/idle.npz"
    motion_data = np.load(motion_file, allow_pickle=True)
    fps = int(motion_data["fps"])
    num_frames = motion_data["dof_positions"].shape[0]
    frame_duration = 1.0 / float(fps)

    # HACK: Move the robot base rightward a bit
    base_offset = np.array([0.0, 0.1, 0.0], dtype=np.float32)

    joint_positions = np.zeros(urdf.num_actuated_joints, dtype=np.float32)
    # for joint_name, (
    #     lower,
    #     upper,
    # ) in viser_urdf.get_actuated_joint_limits().items():
    #     lower = lower if lower is not None else -np.pi
    #     upper = upper if upper is not None else np.pi
    #     initial_pos = 0.0 if lower < -0.1 and upper > 0.1 else (lower + upper) / 2.0
    #     initial_config.append(initial_pos)

    # Set initial robot configuration.
    joint_positions[:] = motion_data["dof_positions"][0]
    robot_base.position = motion_data["body_positions"][0, 0, :] + base_offset
    robot_base.wxyz = motion_data["body_rotations"][0, 0, :]
    viser_urdf.update_cfg(joint_positions)

    # Create serializer.
    serializer = server.get_scene_serializer()

    for frame_index in range(15, num_frames):
        joint_positions[:] = motion_data["dof_positions"][frame_index]
        robot_base.position = motion_data["body_positions"][frame_index, 0, :] + base_offset
        robot_base.wxyz = motion_data["body_rotations"][frame_index, 0, :]
        viser_urdf.update_cfg(joint_positions)
        serializer.insert_sleep(frame_duration)

    data = serializer.serialize()
    Path("recordings/recording.viser").write_bytes(data)
    print(f"Saved {num_frames} frames")

    frame_index = 0
    while True:
        joint_positions[:] = motion_data["dof_positions"][frame_index]
        robot_base.position = motion_data["body_positions"][frame_index, 0, :] + base_offset
        robot_base.wxyz = motion_data["body_rotations"][frame_index, 0, :]
        print(f"frame_index: {frame_index}, position: {robot_base.position}")
        viser_urdf.update_cfg(joint_positions)

        frame_index = (frame_index + 1) % num_frames
        time.sleep(0.01)

