# Test move_linear method with py_trees (relative moves)
from pathlib import Path
from robot_arm.position import Cartesian, JPosition
from robot_arm.robot_arm import RobotArm
import py_trees
import time

class MoveJoint(py_trees.behaviour.Behaviour):
    def __init__(self, name, robot_arm: RobotArm, target_joints: JPosition, relative: bool = True):
        super().__init__(name)
        self.robot_arm: RobotArm = robot_arm
        self.relative = relative

        # Helpers to normalize between sequences and JPosition objects
        def _jvals(j):
            try:
                return tuple(j)  # if JPosition is iterable or a sequence
            except TypeError:
                # fallback to common attribute names if not iterable
                return tuple(getattr(j, n) for n in ('j1', 'j2', 'j3', 'j4', 'j5', 'j6'))

        def _to_jpos(x):
            if isinstance(x, JPosition):
                return x
            vals = _jvals(x)
            return JPosition(*vals)

        # store target as a JPosition
        self.target_joints: JPosition = _to_jpos(target_joints)
        self._jvals = _jvals  # keep helper for update()

    def update(self):
        self.feedback_message = f"Starting joint move ({'relative' if self.relative else 'absolute'}) to {self.target_joints}"
        if not hasattr(self, 'started'):
            # read current joint positions and compute absolute target if relative
            current_raw = self.robot_arm.read_joint_pos()
            current_vals = self._jvals(current_raw)
            target_vals = self._jvals(self.target_joints)

            if self.relative:
                target = [c + d for c, d in zip(current_vals, target_vals)]
            else:
                target = list(target_vals)

            # RobotArm.move_joint expects a JPosition with attributes j1..j6; convert the list to JPosition
            target_jpos = JPosition(*target)
            self.robot_arm.move_joint(target_jpos)
            self.started = True
            self.feedback_message = f"Moving joints -> target {target_jpos}"
            return py_trees.common.Status.RUNNING

        # wait until the robot arm finished moving
        if self.robot_arm.is_moving():
            return py_trees.common.Status.RUNNING

        delattr(self, 'started')
        self.feedback_message = "Reached joint target"
        return py_trees.common.Status.SUCCESS

class MoveLinear(py_trees.behaviour.Behaviour):
    def __init__(self, name, robot_arm: RobotArm, target_delta: Cartesian, relative: bool = True):
        super().__init__(name)
        self.robot_arm: RobotArm = robot_arm
        # target_delta is interpreted as a delta when relative=True, otherwise as absolute target
        self.target_delta: Cartesian = target_delta
        self.relative = relative

    def update(self):
        self.feedback_message = f"Starting move ({'relative' if self.relative else 'absolute'}) to {self.target_delta}"
        if not hasattr(self, 'started'):
            if self.relative:
                # read current position at the moment of starting and compute absolute target
                current = self.robot_arm.read_cartesian_pos()
                target = Cartesian(
                    current.x + self.target_delta.x,
                    current.y + self.target_delta.y,
                    current.z + self.target_delta.z,
                    current.rx + self.target_delta.rx,
                    current.ry + self.target_delta.ry,
                    current.rz + self.target_delta.rz,
                )
            else:
                target = self.target_delta
            self.robot_arm.move_linear(target)
            self.started = True
            self.feedback_message = f"Moving to {self.name} -> target {target}"
            return py_trees.common.Status.RUNNING

        # Wait until the robot arm is not moving
        if self.robot_arm.is_moving():
            return py_trees.common.Status.RUNNING

        delattr(self, 'started')
        self.feedback_message = f"Reached position"
        return py_trees.common.Status.SUCCESS


class MoveGripper(py_trees.behaviour.Behaviour):
    def __init__(self, robot_arm: RobotArm, position, name):
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


def create_behavior_tree(ra: RobotArm):
    # We'll perform moves as deltas relative to the current position at each MoveLinear start.
    width_range = (0,370)
    depth_range = (0, 200)

    # define the relative moves that trace the bottom rectangle, move to top plane, then trace top rectangle, and return
    deltas = [
        # bottom plane rectangle (relative to current)
        Cartesian(0, 0, +depth_range[1], 0, 0, 0),   # up in z
        Cartesian(-width_range[1], 0, 0, 0, 0, 0),  # -x
        Cartesian(0, 0, -depth_range[1], 0, 0, 0),  # -z
        Cartesian(+width_range[1], 0, 0, 0, 0, 0),  # +x (back to start)
    ]

    # Create sequence for rectangle drawing using relative moves
    rectangle_sequence = py_trees.composites.Sequence("DrawCubeRelative", memory=True)
    for i, delta in enumerate(deltas):
        rectangle_sequence.add_child(MoveLinear(f"RelMove{i+1}", ra, delta, relative=True))

    # Gripper sequence (unchanged)
    gripper_open_close_sequence = py_trees.composites.Sequence("GripperSequence", memory=True)
    gripper_open_close_sequence.add_children([
        MoveGripper(ra, 130, "OpenGripper"),
        MoveGripper(ra, 0, "CloseGripper")
    ])

    # Repeat indefinitely both the cube path and gripper cycle
    repeat_rectangle = py_trees.decorators.Repeat("RepeatRectangle", child=rectangle_sequence, num_success=None)
    repeat_open_close = py_trees.decorators.Repeat("RepeatGripper", child=gripper_open_close_sequence, num_success=None)

    parallel = py_trees.composites.Parallel("ParallelActions", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    parallel.add_children([repeat_rectangle, ])

    # Main sequence: initial gripper operations, then go into repeating parallel actions
    root = py_trees.composites.Sequence("Main", memory=True)
    root.add_children([
        MoveLinear("Init Position", ra, Cartesian(0, -210, 0, 0, 0, 90), relative=False),
        MoveGripper(ra, 130, "OpenGripper"),
        MoveGripper(ra, 0, "CloseGripper"),
        MoveGripper(ra, 130, "OpenGripper"),
        # MoveJoint("Look at table", ra, JPosition(0, 0, 0, 0, -25, 0), relative=True),
        # repeat_rectangle,  # start repeating the relative cube path
        parallel
    ])

    return py_trees.trees.BehaviourTree(root)


if __name__ == "__main__":
    ra = RobotArm('169.254.190.254')
    ra.set_speed(.4)

    tree = create_behavior_tree(ra)

    py_trees.display.render_dot_tree(tree.root, target_directory=Path() / "Tree", name="moveInCube_relative_tree")
    tree.setup(timeout=15)

    while True:
        tree.tick()
        print(py_trees.display.unicode_tree(tree.root, show_status=True))
        time.sleep(0.1)