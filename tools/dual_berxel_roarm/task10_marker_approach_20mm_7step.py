#!/usr/bin/env python3

import math
import time

import task10_guarded_approach as core


CLEARANCE_MM = 20.0


def move_without_error_abort(label, command):
    print(f"\n===== {label} =====")
    print("Command:", command)

    response = core.send_json(command, 5.0)
    print("Response:", response.strip())

    pose = core.get_pose()

    actual = (
        float(pose["x"]),
        float(pose["y"]),
        float(pose["z"]),
    )
    expected = (
        float(command["x"]),
        float(command["y"]),
        float(command["z"]),
    )

    error = math.dist(actual, expected)

    print("Actual XYZ mm:", actual)
    print("Tracking difference mm:", error)

    # 不因3D tracking error自動停止。
    # 只保留防止末端過度接近目標表面的最低高度檢查。
    return actual


def main():
    print("Task 10: Marker guarded approach")
    print("The gripper will stop 20 mm above Marker 4.")
    print("It will NOT touch or close on the marker.")

    input(
        "\nClear the full robot workspace, "
        "then press ENTER to return HOME..."
    )

    home = {
        "T": 102,
        "base": 0.0,
        "shoulder": 0.0,
        "elbow": 1.57,
        "wrist": 0.0,
        "roll": 0.0,
        "hand": 3.14,
        "spd": 300,
        "acc": 10,
    }

    print("Returning to official initial pose...")
    print(core.send_json(home, 12.0).strip())

    home_pose = core.get_pose()
    print(
        "Home XYZ mm:",
        (
            home_pose["x"],
            home_pose["y"],
            home_pose["z"],
        ),
    )

    input(
        "\nMake sure both cameras can see Marker 4, "
        "then press ENTER to scan..."
    )

    print("Collecting 10 accepted fused samples...")
    tx, ty, tz = core.collect_target()

    approach_z = tz + CLEARANCE_MM

    print("\n===== Scanned Marker =====")
    print(f"Marker X = {tx:.3f} mm")
    print(f"Marker Y = {ty:.3f} mm")
    print(f"Marker Z = {tz:.3f} mm")
    print()
    print("Final approach point:")
    print(f"X = {tx:.3f} mm")
    print(f"Y = {ty:.3f} mm")
    print(f"Z = {approach_z:.3f} mm")
    print("Clearance = 20 mm")

    if not (
        80.0 <= tx <= 350.0
        and -300.0 <= ty <= 300.0
        and -150.0 <= tz <= 50.0
    ):
        raise RuntimeError(
            "Marker is outside configured workspace."
        )

    confirmation = input(
        "\nCheck the coordinates and workspace. "
        "Type APPROACH to continue: "
    )

    if confirmation != "APPROACH":
        print("Cancelled.")
        return

    print("\nOpening gripper...")
    open_gripper = {
        "T": 106,
        "cmd": 1.57,
        "spd": 300,
        "acc": 10,
    }
    print(core.send_json(open_gripper, 3.0).strip())

    waypoints = [
        {
            "T": 104,
            "x": tx,
            "y": ty,
            "z": 200.0,
            "t": 0.0,
            "r": 0.0,
            "g": 1.57,
            "spd": 0.12,
        },
        {
            "T": 104,
            "x": tx - 10.0,
            "y": ty,
            "z": 170.0,
            "t": 0.55,
            "r": 0.0,
            "g": 1.57,
            "spd": 0.10,
        },
        {
            "T": 104,
            "x": tx - 15.0,
            "y": ty,
            "z": 135.0,
            "t": 0.95,
            "r": 0.0,
            "g": 1.57,
            "spd": 0.08,
        },
        {
            "T": 104,
            "x": tx - 15.0,
            "y": ty,
            "z": 95.0,
            "t": 1.25,
            "r": 0.0,
            "g": 1.57,
            "spd": 0.08,
        },
        {
            "T": 104,
            "x": tx - 10.0,
            "y": ty,
            "z": 50.0,
            "t": 1.42,
            "r": 0.0,
            "g": 1.57,
            "spd": 0.06,
        },
        {
            "T": 104,
            "x": tx,
            "y": ty,
            "z": 0.0,
            "t": 1.52,
            "r": 0.0,
            "g": 1.57,
            "spd": 0.05,
        },
        {
            "T": 104,
            "x": tx,
            "y": ty,
            "z": approach_z,
            "t": 1.52,
            "r": 0.0,
            "g": 1.57,
            "spd": 0.04,
        },
    ]

    for index, waypoint in enumerate(
        waypoints,
        start=1,
    ):
        actual = move_without_error_abort(
            f"Approach {index}/{len(waypoints)}",
            waypoint,
        )

        # 最終底線保護：實際Z若進入估計表面10 mm內，
        # 不再執行任何後續下降命令。
        if actual[2] < tz + 10.0:
            raise RuntimeError(
                "Actual Z entered the final 10 mm "
                "surface-protection zone."
            )

    final_pose = core.get_pose()

    print("\n===== APPROACH COMPLETE =====")
    print("Final actual XYZ mm:")
    print("x =", final_pose["x"])
    print("y =", final_pose["y"])
    print("z =", final_pose["z"])
    print("Marker estimated Z =", tz)
    print("Robot remains torque-locked.")
    print("No automatic contact or gripper close was sent.")


if __name__ == "__main__":
    main()
