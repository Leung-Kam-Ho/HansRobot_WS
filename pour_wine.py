from HansRobot_Elfin_robot_arm.robot_arm import *
import time
import atexit
import py_trees
from py_trees.composites import Sequence, Selector
from py_trees.behaviours import Success
from py_trees.common import Status


class MoveToPosition(py_trees.behaviour.Behaviour):
    def __init__(self, robot_arm, position, name):
        super().__init__(name=name)
        self.ra = robot_arm
        self.position = position
        
    def update(self):
        if not hasattr(self, 'started'):
            self.ra.moveJoint(self.position)
            self.started = True
            self.feedback_message = f"Moving to {self.name}..."
            return Status.RUNNING
        
        if self.ra.isMoving():
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
            self.ra.moveGripper(self.position)
            self.started = True
            self.feedback_message = f"Gripper moving to {self.position}..."
            return Status.RUNNING
        
        if self.ra.isGripperMoving():
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
        'home': JPosition(-65.127, -44.867, 130.668, -34.806, -86.075, 3.022),
        'infront_cup': JPosition(-94, -56, 89, -68, -75, 29),
        'around_cup': JPosition(-83.147, -67.820, 69.243, -60.232, -66.391, 36.582),
        'ready_pour': JPosition(-73.174, -46.835, 91.888, -54.300, -59.557, 33.708),
        'pour': JPosition(-73.187, -46.839, 91.874, -54.319, -59.557, -62.221)
    }
    
    # Build behavior tree
    root = Sequence("PouringSequence", memory=True)
    
    root.add_children([
        MoveGripper(ra, 140, "Open Gripper"),
        SetSpeed(ra, 1, "Set Normal Speed"),
        MoveToPosition(ra, positions['home'], "Move to Home"),
        MoveToPosition(ra, positions['infront_cup'], "Move In Front of Cup"),
        MoveToPosition(ra, positions['around_cup'], "Move Around Cup"),
        MoveGripper(ra, 0, "Close Gripper"),
        MoveToPosition(ra, positions['ready_pour'], "Move to Ready Pour"),
        SetSpeed(ra, 0.3, "Set Slow Speed"),
        MoveToPosition(ra, positions['pour'], "Pour"),
        Wait(2, "Wait for Pouring"),
        SetSpeed(ra, 1, "Set Normal Speed"),
        MoveToPosition(ra, positions['ready_pour'], "Move to Ready Position"),
        MoveToPosition(ra, positions['around_cup'], "Move Around Cup"),
        MoveGripper(ra, 140, "Open Gripper"),
        MoveToPosition(ra, positions['infront_cup'], "Move In Front of Cup"),
        MoveToPosition(ra, positions['home'], "Move to Home")
    ])
    
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