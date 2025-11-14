import lmstudio as lms
from pydantic import BaseModel, Field
import cv2
import py_trees
from py_trees.common import Status
from robot_arm.robot_arm import RobotArm
from robot_arm.position import Cartesian
import time
from MoveInCube import MoveLinear


class PositionInfo(BaseModel):
    # return the move direction based on bottle position
    position: str = Field(..., pattern="^(|left|right|center)$")


# Blackboard keys
class BB:
    FRAME = "frame"
    FRAME_COUNT = "frame_count"
    POSITION = "position"
    CAP = "cap"
    MODEL = "model"
    ROBOT_ARM = "robot_arm"


class InitializeCamera(py_trees.behaviour.Behaviour):
    def __init__(self, name="Initialize Camera"):
        super().__init__(name)

    def setup(self, **kwargs):
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(BB.CAP, access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(
            BB.FRAME_COUNT, access=py_trees.common.Access.WRITE
        )

    def update(self):
        cap = cv2.VideoCapture("bottle.MOV")
        if cap.isOpened():
            self.blackboard.set(BB.CAP, cap)
            self.blackboard.set(BB.FRAME_COUNT, 0)
            return Status.SUCCESS
        return Status.FAILURE


class InitializeModel(py_trees.behaviour.Behaviour):
    def __init__(self, name="Initialize Model"):
        super().__init__(name)

    def setup(self, **kwargs):
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(BB.MODEL, access=py_trees.common.Access.WRITE)

    def update(self):
        model = lms.llm("internvl3_5-2b")
        self.blackboard.set(BB.MODEL, model)
        return Status.SUCCESS


class InitializeRobotArm(py_trees.behaviour.Behaviour):
    def __init__(self, name="Initialize Robot Arm"):
        super().__init__(name)

    def setup(self, **kwargs):
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(BB.ROBOT_ARM, access=py_trees.common.Access.WRITE)

    def update(self):
        ra = RobotArm("169.254.190.254")
        ra.set_speed(0.4)
        self.blackboard.set(BB.ROBOT_ARM, ra)
        return Status.SUCCESS


class InitialMove(py_trees.behaviour.Behaviour):
    def __init__(self, name="Initial Move"):
        super().__init__(name)

    def setup(self, **kwargs):
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(BB.ROBOT_ARM, access=py_trees.common.Access.READ)

    def update(self):
        ra = self.blackboard.get(BB.ROBOT_ARM)
        ra.move_linear(Cartesian(0, -210, 0, 0, 0, 90))
        while ra.is_moving():
            time.sleep(0.1)
        return Status.SUCCESS


class CaptureFrame(py_trees.behaviour.Behaviour):
    def __init__(self, name="Capture Frame"):
        super().__init__(name)

    def setup(self, **kwargs):
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(BB.CAP, access=py_trees.common.Access.READ)
        self.blackboard.register_key(BB.FRAME, access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(
            BB.FRAME_COUNT, access=py_trees.common.Access.WRITE
        )

    def update(self):
        cap = self.blackboard.get(BB.CAP)
        ret, frame = cap.read()

        if not ret:
            return Status.FAILURE

        frame = cv2.resize(frame, (640, 480))
        frame_count = self.blackboard.get(BB.FRAME_COUNT) + 1
        self.blackboard.set(BB.FRAME, frame)
        self.blackboard.set(BB.FRAME_COUNT, frame_count)
        return Status.SUCCESS


class DetectBottlePosition(py_trees.behaviour.Behaviour):
    def __init__(self, name="Detect Bottle Position"):
        super().__init__(name)

    def setup(self, **kwargs):
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(BB.FRAME, access=py_trees.common.Access.READ)
        self.blackboard.register_key(BB.MODEL, access=py_trees.common.Access.READ)
        self.blackboard.register_key(BB.POSITION, access=py_trees.common.Access.WRITE)

    def update(self):
        frame = self.blackboard.get(BB.FRAME)
        model = self.blackboard.get(BB.MODEL)

        cv2.imwrite("input.jpg", frame.copy())

        chat = lms.Chat()
        prompt = """
        You are an robotics arm, the view from your camera is provided.

        location of the bottle horizontally is divided into three sections: left, center, right.
        Your task is to identify the horizontal position of the bottle in the image, and respond with one of the following options: "left", "center", "right", or "" (if the bottle is not visible).
        """

        image_handle = lms.prepare_image("input.jpg")
        chat.add_user_message(prompt, images=[image_handle])

        result = model.respond(chat, response_format=PositionInfo)
        print(result.parsed)

        self.blackboard.set(BB.POSITION, result.parsed.get("position"))
        return Status.SUCCESS


class MoveBasedOnPosition(py_trees.behaviour.Behaviour):
    def __init__(self, name="Move Based on Position"):
        super().__init__(name)

    def setup(self, **kwargs):
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(BB.POSITION, access=py_trees.common.Access.READ)
        self.blackboard.register_key(BB.ROBOT_ARM, access=py_trees.common.Access.READ)

    def update(self):
        position = self.blackboard.get(BB.POSITION)
        ra = self.blackboard.get(BB.ROBOT_ARM)

        if position == "left":
            # Move right (positive x)
            delta = Cartesian(10, 0, 0, 0, 0, 0)
            ra.move_linear(
                Cartesian(
                    ra.read_cartesian_pos().x + delta.x,
                    ra.read_cartesian_pos().y + delta.y,
                    ra.read_cartesian_pos().z + delta.z,
                    ra.read_cartesian_pos().rx + delta.rx,
                    ra.read_cartesian_pos().ry + delta.ry,
                    ra.read_cartesian_pos().rz + delta.rz,
                )
            )
            self.feedback_message = "Moving right due to bottle on left"
        elif position == "right":
            # Move left (negative x)
            delta = Cartesian(-10, 0, 0, 0, 0, 0)
            ra.move_linear(
                Cartesian(
                    ra.read_cartesian_pos().x + delta.x,
                    ra.read_cartesian_pos().y + delta.y,
                    ra.read_cartesian_pos().z + delta.z,
                    ra.read_cartesian_pos().rx + delta.rx,
                    ra.read_cartesian_pos().ry + delta.ry,
                    ra.read_cartesian_pos().rz + delta.rz,
                )
            )
            self.feedback_message = "Moving left due to bottle on right"
        elif position == "center":
            self.feedback_message = "Bottle in center, no movement"
            return Status.SUCCESS
        else:
            self.feedback_message = "Bottle not visible, no movement"
            return Status.SUCCESS

        # Wait for movement to complete
        while ra.is_moving():
            time.sleep(0.1)
        return Status.SUCCESS


# adapt MoveLinear to be a behaviour


class DisplayFrame(py_trees.behaviour.Behaviour):
    def __init__(self, name="Display Frame"):
        super().__init__(name)

    def setup(self, **kwargs):
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(BB.FRAME, access=py_trees.common.Access.READ)

    def update(self):
        frame = self.blackboard.get(BB.FRAME)
        cv2.imshow("Frame", frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            return Status.FAILURE
        return Status.SUCCESS


def create_tree():
    # Initialize nodes
    init_camera = InitializeCamera()
    init_model = InitializeModel()
    init_robot = InitializeRobotArm()
    initial_move = InitialMove()

    # Main loop sequence
    capture = CaptureFrame()
    detect = DetectBottlePosition()
    move = MoveBasedOnPosition()
    display = DisplayFrame()

    # Main processing (runs every frame)
    processing = py_trees.composites.Sequence(
        name="Processing", memory=True, children=[capture, detect, move, display]
    )

    # Initialization
    initialization = py_trees.composites.Parallel(
        name="Initialization",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
        children=[init_camera, init_model, init_robot],
    )

    # Root sequence
    root = py_trees.composites.Sequence(
        name="Track and Move System",
        memory=True,
        children=[
            initialization,
            initial_move,
            py_trees.decorators.Repeat(
                name="Main Loop",
                child=processing,
                num_success=10000,
            ),
        ],
    )
    return root


if __name__ == "__main__":
    tree = create_tree()
    tree.setup_with_descendants()

    try:
        while True:
            tree.tick_once()
            if tree.status == Status.FAILURE:
                break
    finally:
        cv2.destroyAllWindows()
