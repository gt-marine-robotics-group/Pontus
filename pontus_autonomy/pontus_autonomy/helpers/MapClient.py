import time
from pontus_autonomy.base_run import BaseTask
from std_srvs.srv import Empty

class MapClient:
    def __init__(self, node: BaseTask):
        self.node = node
        self.map_reset_service = node.create_client(Empty, "/pontus/reset_semantic_map")

    def reset_map(self):
        if self.map_reset_service.wait_for_service(timeout_sec=5.0):
            self.map_reset_service.call_async(Empty.Request())
        else:
            self.node.get_logger().warn("Failed to find Semantic Map reset service")
