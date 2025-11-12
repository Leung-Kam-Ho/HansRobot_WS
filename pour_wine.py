from robot_arm import *
import time
import atexit
import py_trees
from py_trees.decorators import Repeat
from py_trees.composites import Sequence, Selector
from py_trees.behaviours import Success
from py_trees.common import Status
from math import inf


class MoveToPosition(py_trees.behaviour.Behaviour):
    def __init__(self, robot_arm, position, name):
        super().__init__(name=name)
        self.ra = robot_arm
        self.position = position
        
    def update(self):
        if not hasattr(self, 'started'):
            self.ra.move_joint(self.position)
            self.started = True
            self.feedback_message = f"Moving to {self.name}..."
            return Status.RUNNING

        if self.ra.is_moving():
            self.feedback_message = f"Still moving to {self.name}..."
            return Status.RUNNING
        else:
            delattr(self, 'started')
            self.feedback_message = f"Reached {self.name}"
            return Status.SUCCESS


class MoveGripper(py_trees.behaviour.Behaviour):
    def __init__(self, robot_arm, position, name):
        super().__init__(name=name)
        self.ra = robot_arm
        self.position = position
        
    def update(self):
        if not hasattr(self, 'started'):
            self.ra.move_gripper(self.position)
            self.started = True
            self.feedback_message = f"Gripper moving to {self.position}..."
            return Status.RUNNING

        if self.ra.is_gripper_moving():
            self.feedback_message = f"Gripper still moving to {self.position}..."
            return Status.RUNNING
        else:
            delattr(self, 'started')
            self.feedback_message = f"Gripper reached {self.position}"
            return Status.SUCCESS


class SetSpeed(py_trees.behaviour.Behaviour):
    def __init__(self, robot_arm, speed, name):
        super().__init__(name=name)
        self.ra = robot_arm
        self.speed = speed
        
    def update(self):
        self.ra.setSpeed(self.speed)
        self.feedback_message = f"Speed set to {self.speed}"
        return Status.SUCCESS


class Wait(py_trees.behaviour.Behaviour):
    def __init__(self, duration, name):
        super().__init__(name=name)
        self.duration = duration
        
    def update(self):
        if not hasattr(self, 'start_time'):
            self.start_time = time.time()
            self.feedback_message = f"Waiting for {self.duration}s..."
            return Status.RUNNING
        
        elapsed = time.time() - self.start_time
        if elapsed < self.duration:
            self.feedback_message = f"Waiting... {self.duration - elapsed:.1f}s remaining"
            return Status.RUNNING
        else:
            delattr(self, 'start_time')
            self.feedback_message = f"Wait completed"
            return Status.SUCCESS


def main():
    try:
        ra = RobotArm('169.254.190.254')
    except:
        pass
    
    # Define positions
    positions = {
        'zero': JPosition(0, 0, 0, 0, 0, 0),
        'home': JPosition(-65.127, -44.867, 130.668, -34.806, -86.075, 3.022),
        'infront_beer': JPosition(-94, -56, 89, -68, -75, 29),
        'around_beer': JPosition(-83.147, -67.820, 69.243, -60.232, -66.391, 36.582),
        'ready_pour': JPosition(-73.174, -46.835, 91.888, -54.300, -59.557, 33.708),
        'pour': JPosition(-73.187, -46.839, 91.874, -54.319, -59.557, -62.221),
        'around_cup' : JPosition(-54.194,-54.647,103.428,-25.451,-69.755,9.709),
        'ready_cup_pour': JPosition(-77.645,-47.430,84.537,-58.462,-59.464,39.802)
    }
    normal_speed = 0.5
    # Build behavior tree
    root = Sequence("PouringSequence", memory=True)
    
    root.add_children([
        MoveToPosition(ra, positions['zero'], "Move to Zero Position"),
        # MoveGripper(ra, 0, "Open Gripper"),
        # MoveGripper(ra, 140, "Open Gripper"),
        # SetSpeed(ra, normal_speed, "Set Normal Speed"),
        # MoveToPosition(ra, positions['home'], "Move to Home"),
        # MoveToPosition(ra, positions['infront_beer'], "Move In Front of Cup"),
        # MoveToPosition(ra, positions['around_beer'], "Move Around Cup"),
        # MoveGripper(ra, 0, "Close Gripper"),
        # MoveToPosition(ra, positions['ready_pour'], "Move to Ready Pour"),
        # SetSpeed(ra, 0.3, "Set Slow Speed"),
        # MoveToPosition(ra, positions['pour'], "Pour"),
        # Wait(2, "Wait for Pouring"),
        # SetSpeed(ra, normal_speed, "Set Normal Speed"),
        # MoveToPosition(ra, positions['ready_pour'], "Move to Ready Position"),
        # MoveToPosition(ra, positions['around_beer'], "Move Around Cup"),
        # MoveGripper(ra, 140, "Open Gripper"),
        # MoveToPosition(ra, positions['infront_beer'], "Move In Front of Cup"),
        # MoveToPosition(ra, positions['home'], "Move to Home"),
        # MoveToPosition(ra, positions['around_cup'], "Move Around Cup"),
        # MoveGripper(ra, 0, "Close Gripper"),
        # MoveToPosition(ra, positions['ready_cup_pour'], "Ready for Next Pour")
        
    ])
    
    # root = Repeat( name="Repeat Pouring Sequence", child=root, num_success=inf)
    # Execute behavior tree
    root.setup_with_descendants()
    
    while root.status != Status.SUCCESS:
        root.tick_once()
        
        # Display the tree
        print(py_trees.display.ascii_tree(root, show_status=True))
        
        time.sleep(0.01)  # Small delay to prevent busy waiting
    
    print("Pouring sequence completed!")


if __name__ == '__main__':
    main()