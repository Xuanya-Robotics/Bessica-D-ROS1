import sys
import tty
import termios
import select
from demo_movement import BessicaDMotion
import moveit_commander

# Function to check for keyboard input
def is_key_pressed():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

if __name__ == "__main__":
    bessica_movement = BessicaDMotion()

    # Define your joint states
    robot_state_goal = {
        "left_arm_joint1": 0.27,
        "left_arm_joint2": 0.8830215248551526,
        "left_arm_joint3": -0.60,
        "left_arm_joint4": -1.55,
        "left_arm_joint5": 0.39,
        "left_arm_joint6": 0.47,
        "left_arm_joint7": 0.46,
        "right_arm_joint1": -0.33,
        "right_arm_joint2": 0.8155098878549327,
        "right_arm_joint3": 0.005,
        "right_arm_joint4": -1.52,
        "right_arm_joint5": 0.0007,
        "right_arm_joint6": 0.70,
        "right_arm_joint7": 0.35
    }

    """
    position: [-0.046797839284243144, -0.005370243852290474, 0.006904599238658614, 0.2815542133986454, -0.005370243852290474, 0.2324548410348491,
      0.046797839284243144, 0.0, 
      -0.15113400555731138, 0.020713797715976832, -1.0349227081056487, 0.5239823644448914, 0.3475314950124973, -0.5562038275586327, 0.3413940734670227, 0.0]

    """
    robot_state_goal2 = {
        "left_arm_joint1":-0.046797839284243144,
        "left_arm_joint2":-0.005370243852290474,
        "left_arm_joint3":0.006904599238658614,
        "left_arm_joint4":0.2815542133986454,
        "left_arm_joint5":-0.005370243852290474,
        "left_arm_joint6":0.2324548410348491,
        "left_arm_joint7":0.046797839284243144,
        "right_arm_joint1":-0.15113400555731138,
        "right_arm_joint2":0.020713797715976832,
        "right_arm_joint3":-1.0349227081056487,
        "right_arm_joint4":0.5239823644448914,
        "right_arm_joint5":0.3475314950124973,
        "right_arm_joint6":-0.5562038275586327,
        "right_arm_joint7":0.3413940734670227
    }
    robot_state_home = {
        "left_arm_joint1": 0.0,
        "left_arm_joint2": 0.0,
        "left_arm_joint3": 0.0,
        "left_arm_joint4": 0.0,
        "left_arm_joint5": 0.0,
        "left_arm_joint6": 0.0,
        "left_arm_joint7": 0.0,
        "right_arm_joint1": 0.0,
        "right_arm_joint2": 0.0,
        "right_arm_joint3": 0.0,
        "right_arm_joint4": 0.0,
        "right_arm_joint5": 0.0,
        "right_arm_joint6": 0.0,
        "right_arm_joint7": 0.0
    }

    # Store the original terminal settings
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        # Set the terminal to raw mode
        tty.setcbreak(sys.stdin.fileno())

        print("Press 'q' to quit.")
        while True:
            # Check for 'q' key press to exit
            if is_key_pressed() and sys.stdin.read(1) == 'q':
                print("'q' pressed, shutting down.")
                break

            # Move to goal position
            print("Moving to goal position...")
            bessica_movement.move_to_joint_state(robot_state_goal)
            # bessica_movement.move_to_joint_state(robot_state_goal)
            print("Moved to goal position.")

            # Get and print current joint values
            left_vals, right_vals = bessica_movement.get_both_arms_joint_values()
            print("Left Arm Joints:", [round(j, 2) for j in left_vals])
            print("Right Arm Joints:", [round(j, 2) for j in right_vals])

            # Move back to home position
            print("Moving back to home position...")
            bessica_movement.move_to_joint_state(robot_state_home)
            print("Moved back to home position.")

    finally:
        # Restore the original terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        # Ensure a clean shutdown of MoveIt
        moveit_commander.roscpp_shutdown()
        print("Program terminated and resources released.")



