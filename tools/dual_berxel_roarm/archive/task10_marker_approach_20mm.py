#!/usr/bin/env python3

import math
import time

import task10_guarded_approach as core


CLEARANCE_MM = 20.0


def move_without_error_abort(label, command):
    print(f"\n===== {label} =====")
    print("Command:", command)

    # 只短暫等待命令進入控制器。
    response = core.send_json(command, 0.3)
    if response.strip():
        print("Response:", response.strip())

    deadline = time.time() + 15.0
    previous = None
    stable_count = 0
    actual = None

    while time.time() < deadline:
        try:
            pose = core.get_pose()
            actual = (
                float(pose["x"]),
                float(pose["y"]),
                float(pose["z"]),
            )
        except Exception:
            time.sleep(0.10)
            continue

        if previous is not None:
            movement = math.dist(
                actual,
                previous,
            )

            if movement < 1.0:
                stable_count += 1
            else:
                stable_count = 0

        previous = actual

        print(
            f"\rActual XYZ mm: "
            f"{actual[0]:.1f}, "
            f"{actual[1]:.1f}, "
            f"{actual[2]:.1f}",
            end="",
            flush=True,
        )

        if stable_count >= 2:
            print("\nMotion settled.")
            break

        time.sleep(0.10)

    if actual is None:
        raise RuntimeError(
            "No robot feedback after movement command."
        )

    expected = (
        float(command["x"]),
        float(command["y"]),
        float(command["z"]),
    )
    error = math.dist(actual, expected)

    print("Tracking difference mm:", error)
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
        "Type approach to continue: "
    )

    if confirmation != "approach":
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
            "z": 170.0,
            "t": 0.75,
            "r": 0.0,
            "g": 1.57,
            "spd": 0.12,
        },
        {
            "T": 104,
            "x": tx - 10.0,
            "y": ty,
            "z": 50.0,
            "t": 1.42,
            "r": 0.0,
            "g": 1.57,
            "spd": 0.08,
        },
        {
            "T": 104,
            "x": tx,
            "y": ty,
            "z": approach_z,
            "t": 1.52,
            "r": 0.0,
            "g": 1.57,
            "spd": 0.05,
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
