# Test move_linear method
from robot_arm.robot_arm import RobotArm


def test_move_linear():
    ra = RobotArm('169.254.190.254')
    ra.move_linear(100, 0, 0, 0, 0, 0)
    # Add assertions to verify the behavior
    ra.move_linear(0, 100, 0, 0, 0, 0)
    
    ra.move_linear(0, 0, 100, 0, 0, 0)
    ra.move_linear(0, 0, 0, 90, 0, 0)
    ra.move_linear(0, 0, 0, 0, 90, 0)
    ra.move_linear(0, 0, 0, 0, 0, 90)
    # You can add more tests as needed


if __name__ == "__main__":
    test_move_linear()
    print("move_linear test completed.")