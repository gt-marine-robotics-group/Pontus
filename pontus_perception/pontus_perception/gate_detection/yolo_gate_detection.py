from pontus_msgs.msg import YOLOResultArray
from pontus_msgs.msg import YOLOResult
import numpy as np
from sensor_msgs.msg import CameraInfo


class YoloGateDetection:
    def __init__(self):
        pass

    @staticmethod
    def detect_gate(left_yolo_result: YOLOResultArray, right_yolo_result: YOLOResultArray, camera_info: CameraInfo, Tx_override = None):
        left_camera_gates = [None, None]
        right_camera_gates = [None, None]
        for result in left_yolo_result.results:
            if result.class_id == 0:
                left_camera_gates[0] = np.array([(result.x1 + result.x2) / 2 , (result.y1 + result.y2) / 2])
            if result.class_id == 1:
                left_camera_gates[1] = np.array([(result.x1 + result.x2) / 2 , (result.y1 + result.y2) / 2])

        for result in right_yolo_result.results:
            if result.class_id == 0:
                right_camera_gates[0] = np.array([(result.x1 + result.x2) / 2 , (result.y1 + result.y2) / 2])
            if result.class_id == 1:
                right_camera_gates[1] = np.array([(result.x1 + result.x2) / 2 , (result.y1 + result.y2) / 2])
        
        # If gate is not seen by both cameras, return none
        if any(gate is None for gate in left_camera_gates) or any(gate is None for gate in right_camera_gates):
            return None, None
        
        # Tx = -fx * B
        f = camera_info.k[0]
        c_x = camera_info.k[2]
        c_y = camera_info.k[5]
        Tx = -camera_info.p[3]
        if Tx_override is not None:
            Tx = Tx_override
        
        # Calculate locaitons
        disparity_first_gate = left_camera_gates[0][0] - right_camera_gates[0][0]
        first_gate_Z = Tx / disparity_first_gate
        first_gate_X = (left_camera_gates[0][0] - c_x) * first_gate_Z / f
        first_gate_Y = (left_camera_gates[0][1] - c_y) * first_gate_Z / f

        first_gate_optical_frame = np.array([first_gate_X, first_gate_Y, first_gate_Z])

        disparity_second_gate = left_camera_gates[1][0] - right_camera_gates[1][0]
        second_gate_Z = Tx / disparity_second_gate
        second_gate_X = (left_camera_gates[1][0] - c_x) * second_gate_Z / f
        second_gate_Y = (left_camera_gates[1][1] - c_y) * second_gate_Z / f

        second_gate_optical_frame = np.array([second_gate_X, second_gate_Y, second_gate_Z])

        R = np.array([[0, 0, 1],
            [-1, 0, 0],
            [0, -1, 0]])

        first_gate_body_frame =  np.dot(first_gate_optical_frame, R.T)
        second_gate_body_frame =  np.dot(second_gate_optical_frame, R.T)

        left_gate = first_gate_body_frame if first_gate_body_frame[1] > second_gate_body_frame[1] else second_gate_body_frame
        right_gate = first_gate_body_frame if first_gate_body_frame[1] < second_gate_body_frame[1] else second_gate_body_frame

        return left_gate, right_gate


        