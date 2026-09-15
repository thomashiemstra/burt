"""Render a short GIF of the simulated robot trotting forward.

Runs the exact same QuadController + IK + SimRobotController stack as run_sim.py,
but headless (PyBullet DIRECT mode) and driven by a scripted forward-walk command
instead of a joystick. Frames are grabbed from a chase camera that follows the
robot and stitched into an animated GIF for the README.

Usage:
    .venv/Scripts/python make_walk_gif.py
    .venv/Scripts/python make_walk_gif.py --out media/burt_walk.gif --width 480 --height 360
"""
import argparse
import os

import numpy as np
import pybullet as p
from PIL import Image
from numpy import loadtxt

from src.quad.Config import Configuration
from src.quad.QuadController import QuadController
from src.quad.QuadCommand import QuadCommand
from src.quad.Kinematics import four_legs_inverse_kinematics
from src.quad.State import State, BehaviorState
from src.StanceManager import StanceManager
from src.sim.SimRobotController import SimRobotController


def render_frame(client, target, width, height, distance, yaw, pitch):
    """Grab one RGB frame from a chase camera aimed at `target`."""
    view = p.computeViewMatrixFromYawPitchRoll(
        cameraTargetPosition=target,
        distance=distance,
        yaw=yaw,
        pitch=pitch,
        roll=0,
        upAxisIndex=2,
        physicsClientId=client,
    )
    proj = p.computeProjectionMatrixFOV(
        fov=55, aspect=width / height, nearVal=0.05, farVal=8.0,
        physicsClientId=client,
    )
    _, _, rgb, _, _ = p.getCameraImage(
        width, height, view, proj,
        shadow=1,
        lightDirection=[1, 1, 1.5],
        renderer=p.ER_TINY_RENDERER,
        physicsClientId=client,
    )
    rgb = np.reshape(np.asarray(rgb, dtype=np.uint8), (height, width, 4))
    return Image.fromarray(rgb[:, :, :3], "RGB")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default=os.path.join("media", "burt_walk.gif"),
                        help="Output GIF path.")
    parser.add_argument("--width", type=int, default=480)
    parser.add_argument("--height", type=int, default=360)
    parser.add_argument("--warmup-steps", type=int, default=40,
                        help="Control steps spent settling in REST before trotting.")
    parser.add_argument("--walk-steps", type=int, default=420,
                        help="Control steps spent trotting forward.")
    parser.add_argument("--capture-every", type=int, default=5,
                        help="Capture one frame every N control steps.")
    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    servo_offsets = loadtxt(
        os.path.join(script_dir, "src/quad/offsets.txt"),
        comments="#", delimiter=",", unpack=False, dtype=int,
    )
    config = Configuration(servo_offsets)

    quad_controller = QuadController(config, four_legs_inverse_kinematics)
    quad_robot = SimRobotController(
        config, os.path.join(script_dir, "urdf/burt.urdf"), gui=False)
    client = quad_robot.client

    stance_manager = StanceManager()
    state = State(config)
    state.behavior_state = BehaviorState.REST
    stance_manager.apply_stance(state.stance, config)
    quad_robot.enable_motors()

    # Mirror run_sim.py's stepping cadence: recompute the gait at the control
    # period, but advance physics at the finer 240 Hz PyBullet timestep.
    physics_dt = 1.0 / 240.0
    control_period = config.dt * config.delay_factor
    substeps = max(1, round(control_period / physics_dt))

    command = QuadCommand(config)

    frames = []
    total_steps = args.warmup_steps + args.walk_steps
    for step in range(total_steps):
        if step == args.warmup_steps:
            state.behavior_state = BehaviorState.TROT

        # Scripted command: stand still to settle, then walk straight forward.
        if state.behavior_state == BehaviorState.TROT:
            command.horizontal_velocity = np.array([config.max_x_velocity, 0.0])
        else:
            command.horizontal_velocity = np.array([0.0, 0.0])
        command.yaw_rate = 0.0

        quad_controller.run(state, command)
        quad_robot.set_actuator_positions(state)
        quad_robot.step(substeps)

        if step % args.capture_every == 0:
            base_pos, _ = p.getBasePositionAndOrientation(
                quad_robot.robot, physicsClientId=client)
            # Chase camera: track the robot's x/y, hold a fixed 3/4 view.
            target = [base_pos[0], base_pos[1], 0.10]
            frames.append(render_frame(
                client, target, args.width, args.height,
                distance=0.85, yaw=50, pitch=-22))

    quad_robot.disconnect()

    if not frames:
        raise SystemExit("No frames captured - check step/capture settings.")

    # Quantize every frame against one shared palette so colours stay stable
    # (avoids per-frame palette flicker) and the file stays small.
    palette = frames[0].quantize(colors=255, method=Image.MEDIANCUT)
    frames_p = [f.quantize(palette=palette, dither=Image.NONE) for f in frames]

    duration_ms = int(control_period * args.capture_every * 1000)
    out_path = os.path.join(script_dir, args.out)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    frames_p[0].save(
        out_path,
        save_all=True,
        append_images=frames_p[1:],
        duration=duration_ms,
        loop=0,
        disposal=2,
        optimize=True,
    )
    size_kb = os.path.getsize(out_path) / 1024
    print(f"Wrote {len(frames_p)} frames to {out_path} "
          f"({args.width}x{args.height}, ~{duration_ms}ms/frame, {size_kb:.0f} KB)")


if __name__ == "__main__":
    main()
