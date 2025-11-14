import lmstudio as lms
from pydantic import BaseModel, Field
import cv2
import py_trees
from py_trees.common import Status

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


class InitializeCamera(py_trees.behaviour.Behaviour):
    def __init__(self, name="Initialize Camera"):
        super().__init__(name)
        
    def setup(self):
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(BB.CAP, access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(BB.FRAME_COUNT, access=py_trees.common.Access.WRITE)
        
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
        
    def setup(self):
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(BB.MODEL, access=py_trees.common.Access.WRITE)
        
    def update(self):
        model = lms.llm("internvl3_5-2b")
        self.blackboard.set(BB.MODEL, model)
        return Status.SUCCESS


class CaptureFrame(py_trees.behaviour.Behaviour):
    def __init__(self, name="Capture Frame"):
        super().__init__(name)
        
    def setup(self):
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(BB.CAP, access=py_trees.common.Access.READ)
        self.blackboard.register_key(BB.FRAME, access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(BB.FRAME_COUNT, access=py_trees.common.Access.WRITE)
        
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


class DrawGrid(py_trees.behaviour.Behaviour):
    def __init__(self, name="Draw Grid"):
        super().__init__(name)
        
    def setup(self):
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(BB.FRAME, access=py_trees.common.Access.WRITE)
        
    def update(self):
        frame = self.blackboard.get(BB.FRAME)
        frame_size = (frame.shape[1], frame.shape[0])
        
        for i in range(1, 3):
            cv2.line(frame, (i * frame_size[0] // 3, 0), 
                    (i * frame_size[0] // 3, frame_size[1]), (0, 255, 0), 2)
        self.blackboard.set(BB.FRAME, frame)
        return Status.SUCCESS


class CheckFrameInterval(py_trees.behaviour.Behaviour):
    def __init__(self, interval=30, name="Check Frame Interval"):
        super().__init__(name)
        self.interval = interval
        
    def setup(self):
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(BB.FRAME_COUNT, access=py_trees.common.Access.READ)
        
    def update(self):
        frame_count = self.blackboard.get(BB.FRAME_COUNT)
        if frame_count % self.interval == 0:
            return Status.SUCCESS
        return Status.FAILURE


class DetectBottlePosition(py_trees.behaviour.Behaviour):
    def __init__(self, name="Detect Bottle Position"):
        super().__init__(name)
        
    def setup(self):
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


class DisplayFrame(py_trees.behaviour.Behaviour):
    def __init__(self, name="Display Frame"):
        super().__init__(name)
        
    def setup(self):
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(BB.FRAME, access=py_trees.common.Access.READ)
        
    def update(self):
        frame = self.blackboard.get(BB.FRAME)
        cv2.imshow("Frame", frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            return Status.FAILURE
        return Status.SUCCESS


def create_tree():
    # Initialize nodes
    init_camera = InitializeCamera()
    init_model = InitializeModel()
    
    # Main loop sequence
    capture = CaptureFrame()
    draw_grid = DrawGrid()
    check_interval = CheckFrameInterval(interval=30)
    detect = DetectBottlePosition()
    display = DisplayFrame()
    
    # Detection sequence (only runs every 30 frames)
    detection_sequence = py_trees.composites.Sequence(
        name="Detection Sequence",
        memory=True,
        children=[check_interval, detect]
    )
    
    # Main processing (runs every frame)
    processing = py_trees.composites.Sequence(
        name="Processing",
        memory=True,
        children=[
            capture,
            draw_grid,
            py_trees.composites.Selector(
                name="Optional Detection",
                memory=True,
                children=[detection_sequence, py_trees.behaviours.Success()]
            ),
            display
        ]
    )
    
    # Initialization
    initialization = py_trees.composites.Parallel(
        name="Initialization",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
        children=[init_camera, init_model]
    )
    
    # Root sequence
    root = py_trees.composites.Sequence(
        name="Robot Vision System",
        memory=True,
        children=[
            initialization,
            py_trees.decorators.Repeat(
                name="Main Loop",
                child=processing,
                num_success=py_trees.common.ParallelPolicy.SuccessOnAll()
            )
        ]
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
