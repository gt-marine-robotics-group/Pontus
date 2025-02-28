from pontus_msgs.msg import YOLOResultArray
from pontus_msgs.msg import YOLOResult
import numpy as np
from sensor_msgs.msg import CameraInfo


class YoloVerticalMarkerDetection:
    def __init__(self):
        pass

    @staticmethod
    def detect(left_yolo_result: YOLOResultArray, right_yolo_result: YOLOResultArray, camera_info: CameraInfo, Tx_override = -1.0):
        left_camera_VM = None
        right_camera_VM = None
        for result in left_yolo_result.results:
            if result.class_id == 2:
                left_camera_VM = np.array([(result.x1 + result.x2) / 2 , (result.y1 + result.y2) / 2])

        for result in right_yolo_result.results:
            if result.class_id == 2:
                right_camera_VM = np.array([(result.x1 + result.x2) / 2 , (result.y1 + result.y2) / 2])
        
        # If gate is not seen by both cameras, return none
        if left_camera_VM is None or right_camera_VM is None:
            return None
        
        # Tx = -fx * B
        f = camera_info.k[0]
        c_x = camera_info.k[2]
        c_y = camera_info.k[5]
        Tx = -camera_info.p[3]
        if Tx_override != -1.0:
            Tx = Tx_override
        
        # Calculate locaitons
        disparity_VM = left_camera_VM[0] - right_camera_VM[0]
        vm_Z = Tx / disparity_VM
        vm_X = (left_camera_VM[0] - c_x) * vm_Z / f
        vm_Y = (left_camera_VM[1] - c_y) * vm_Z / f

        vm_optical_frame = np.array([vm_X, vm_Y, vm_Z])

        R = np.array([[0, 0, 1],
            [-1, 0, 0],
            [0, -1, 0]])

        vm_body_frame =  np.dot(vm_optical_frame, R.T)
        
        scaling_factor = None
        scaling_factor = 0.896
        if scaling_factor:
            vm_body_frame[0] = vm_body_frame[0] / scaling_factor
            vm_body_frame[1] = vm_body_frame[1] / scaling_factor
        return vm_body_frame


        