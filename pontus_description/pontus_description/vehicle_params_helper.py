import asyncio
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rcl_interfaces.srv import ListParameters, GetParameters

class VehicleParams():

    def __init__(self, node: Node):
        self.params = {}

        self.list_params_client = node.create_client(ListParameters, "/vehicle_parameter_server/list_parameters")
        self.get_params_client = node.create_client(GetParameters, "/vehicle_parameter_server/get_parameters")

        if not self.list_params_client.wait_for_service(timeout_sec=5.0):
            print("Service to list vehicle parameters not available")
            return

        listRequest = ListParameters.Request()
        list_future = self.list_params_client.call_async(listRequest)
        rclpy.spin_until_future_complete(node, list_future)
        list_response = list_future.result()

        if list_response is None:
            print("Failed to list vehicle parameters")
            return

        self.param_names = list_response.result.names

        if not self.get_params_client.wait_for_service(timeout_sec=5.0):
            print("Service to get vehicle parameters not available")
            return

        getRequest = GetParameters.Request()
        getRequest.names = self.param_names
        params_future = self.get_params_client.call_async(getRequest)
        rclpy.spin_until_future_complete(node, params_future)

        params_response = params_future.result()

        if params_response is None:
            print("Failed to get vehicle parameters")
            return

        for name, value in zip(self.param_names, params_response.values):
            if name != "use_sim_time" and name != "start_type_description_service":
                self.params[name] = value.double_value
                setattr(self, name, value.double_value)