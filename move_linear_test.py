# Test move_linear method with py_trees
from robot_arm.position import Cartesian
from robot_arm.robot_arm import RobotArm
import py_trees
import time

class MoveLinear(py_trees.behaviour.Behaviour):
    def __init__(self, name, robot_arm : RobotArm, target_position : Cartesian):
        super().__init__(name)
        self.robot_arm: RobotArm = robot_arm
        self.target_position: Cartesian = target_position
    
    def update(self):
        self.feedback_message = f"Starting move to {self.target_position}"
        if not hasattr(self, 'started'):
            self.robot_arm.move_linear(self.target_position)
            self.started = True
            self.feedback_message = f"Moving to {self.name}"
            return py_trees.common.Status.RUNNING
        # Wait until the robot arm is not moving
        if self.robot_arm.is_moving():
            return py_trees.common.Status.RUNNING
        
        delattr(self, 'started')
        self.feedback_message = f"Reached position"
        return py_trees.common.Status.SUCCESS


class MoveGripper(py_trees.behaviour.Behaviour):
    def __init__(self, robot_arm : RobotArm, position, name):
        super().__init__(name=name)
        self.ra: RobotArm = robot_arm
        self.position = position
        
    def update(self):
        if not hasattr(self, 'started'):
            self.ra.move_gripper(self.position)
            self.started = True
            self.feedback_message = f"Gripper moving to {self.position}..."
            return py_trees.common.Status.RUNNING

        if self.ra.is_gripper_moving():
            self.feedback_message = f"Gripper still moving to {self.position}..."
            return py_trees.common.Status.RUNNING
        else:
            delattr(self, 'started')
            self.feedback_message = f"Gripper reached {self.position}"
            return py_trees.common.Status.SUCCESS


def create_behavior_tree(ra):
    #{'x': -227.483, 'y': -394.018, 'z': 278.261, 'rx': -110.224, 'ry': -90.0, 'rz': 170.224}
    # home = Cartesian(-227.483, -394.018, 278.261, -110.224, -90.0, 170.224)
    home = Cartesian(0,0,0,0,0,0)
    desired_pos = Cartesian(home.x, home.y, home.z, home.rx, home.ry, home.rz)
    
    cube_size = 200
    
    corners = [
        # Move in bottom plane
        Cartesian(desired_pos.x, desired_pos.y, desired_pos.z, desired_pos.rx, desired_pos.ry, desired_pos.rz),
        Cartesian(desired_pos.x, desired_pos.y, desired_pos.z + cube_size, desired_pos.rx, desired_pos.ry, desired_pos.rz),
        Cartesian(desired_pos.x - cube_size, desired_pos.y, desired_pos.z + cube_size, desired_pos.rx, desired_pos.ry, desired_pos.rz),
        Cartesian(desired_pos.x - cube_size, desired_pos.y, desired_pos.z, desired_pos.rx, desired_pos.ry, desired_pos.rz),
        Cartesian(desired_pos.x, desired_pos.y, desired_pos.z, desired_pos.rx, desired_pos.ry, desired_pos.rz),
        
        # Move in top plane
        Cartesian(desired_pos.x, desired_pos.y + cube_size, desired_pos.z, desired_pos.rx, desired_pos.ry, desired_pos.rz),
        Cartesian(desired_pos.x, desired_pos.y + cube_size, desired_pos.z + cube_size, desired_pos.rx, desired_pos.ry, desired_pos.rz),
        Cartesian(desired_pos.x - cube_size, desired_pos.y + cube_size, desired_pos.z + cube_size, desired_pos.rx, desired_pos.ry, desired_pos.rz),
        Cartesian(desired_pos.x - cube_size, desired_pos.y + cube_size, desired_pos.z, desired_pos.rx, desired_pos.ry, desired_pos.rz),
        Cartesian(desired_pos.x, desired_pos.y + cube_size, desired_pos.z, desired_pos.rx, desired_pos.ry, desired_pos.rz),
        
        
        
    ]
    # Create sequence for rectangle drawing
    rectangle_sequence = py_trees.composites.Sequence("DrawRectangle", memory=True)
    for i, corner in enumerate(corners):
        rectangle_sequence.add_child(MoveLinear(f"MoveToCorner{i+1}", ra, corner))
    
    gripper_open_close_sequence = py_trees.composites.Sequence("GripperSequence", memory=True)
    gripper_open_close_sequence.add_children([
        MoveGripper(ra, 140, "OpenGripper"),
        MoveGripper(ra, 100, "CloseGripper")
    ])
    
    
    
    # Wrap in retry or repeat decorator for infinite loop
    repeat_rectangle = py_trees.decorators.Repeat("RepeatRectangle", child=rectangle_sequence, num_success=py_trees.common.ParallelPolicy.SuccessOnAll)
    repeat_open_close = py_trees.decorators.Repeat("RepeatGripper", child=gripper_open_close_sequence, num_success=py_trees.common.ParallelPolicy.SuccessOnAll)
    
    parallel = py_trees.composites.Parallel("ParallelActions", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    parallel.add_children([repeat_rectangle, ])
    
    # Main sequence
    root = py_trees.composites.Sequence("Main", memory=True)
    root.add_children([
        MoveLinear("MoveToHome", ra, home),
        MoveLinear("MoveToStart", ra, desired_pos),
        parallel
    ])
    
    return py_trees.trees.BehaviourTree(root)


if __name__ == "__main__":
    ra = RobotArm('169.254.190.254')
    # print(ra.read_cartesian_pos().__dict__)
    ra.set_speed(1.0)
    
    tree = create_behavior_tree(ra)
    tree.setup(timeout=15)
    
    while True:
        tree.tick()
        print(py_trees.display.unicode_tree(tree.root, show_status=True))