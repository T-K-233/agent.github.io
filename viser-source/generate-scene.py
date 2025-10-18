"""
python generate-scene.py
"""

import time
from pathlib import Path

import numpy as np
import tyro
from robot_descriptions.loaders.yourdfpy import load_robot_description
import imageio.v3 as iio

import viser
from viser.extras import ViserUrdf


def main() -> None:

    server = viser.ViserServer()

    server.scene.set_background_image(
        iio.imread("./viser-source/scene-background.jpg"),
        format="auto",
    )

    server.gui.configure_theme(
        dark_mode=True,
        show_logo=False,
    )

    # # Create grid.
    # server.scene.add_grid(
    #     "/grid",
    #     width=2,
    #     height=2,
    #     position=(
    #         0.0,
    #         0.0,
    #         0.0,
    #     ),
    # )

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
        load_meshes=True,
        load_collision_meshes=False,
    )

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
    viser_urdf.update_cfg(joint_positions)

    frame_count = 0
        
    # Create serializer.
    serializer = server.get_scene_serializer()

    num_frames = 100

    for t in range(num_frames):
        # config[0] += np.sin(frame_count * 0.1)

        viser_urdf.update_cfg(joint_positions)
        # Add a frame delay.
        serializer.insert_sleep(0.02)

        frame_count += 1

    print(f"Saved {num_frames} frames")

    # Save the complete animation.
    data = serializer.serialize()  # Returns bytes
    Path("recordings/recording.viser").write_bytes(data)

    print(f"Saved {num_frames} frames")

    # Sleep forever.
    while True:
        viser_urdf.update_cfg(joint_positions)
        time.sleep(0.02)


if __name__ == "__main__":
    tyro.cli(main)
