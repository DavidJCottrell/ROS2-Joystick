import joystick # type: ignore

import rclpy # type: ignore
from rclpy.node import Node # type: ignore

from acf_msg_types.msg import JoystickEnabled # type: ignore
from std_msgs.msg import String # type: ignore
from geometry_msgs.msg import Twist, Vector3 # type: ignore
import sys # type: ignore
from std_srvs.srv import SetBool # type: ignore
from concurrent.futures import Future

class JoystickNode(Node):

    def __init__(self, chosen_crane: str) -> None:
        super().__init__(f'{chosen_crane}_joystick_node')

        self.is_enabled: bool = False

        # "grapple_crane" or "mast_crane"
        self.crane_name: str = chosen_crane
        self.other_crane: str = "mast_crane" if self.crane_name == "grapple_crane" else "grapple_crane"

        self.joystick_position_pub = self.create_publisher(Twist, 'joystick/' + self.crane_name + '/demand', 10)

        self.joystick_state_pub = self.create_publisher(JoystickEnabled, 'joystick/' + self.crane_name + '/state', 10)
        #self.other_joystick_state_pub = self.create_publisher(JoystickEnabled, 'joystick/' + self.other_crane + '/state', 10)

        self.joystick_state_pub.publish(JoystickEnabled(enabled = False))

        self.create_service(SetBool, 'joystick_node/' + self.crane_name + '/service/enable', self.enable_disable_callback)

        def other_crane_enabled_disabled_callback(other_crane_msg: JoystickEnabled):
            if other_crane_msg.enabled == True:
                self.get_logger().info(f"disabling {self.other_crane} joytsick")
                self.publish_demand(Twist(linear=Vector3(x=0.0,y=0.0,z=0.0)))
                self.is_enabled = False
                self.joystick_state_pub.publish(JoystickEnabled(enabled = False))

        self.create_subscription(JoystickEnabled, 'joystick/' + self.other_crane + '/state', other_crane_enabled_disabled_callback, 10)

    # # Calls the disable service for the other crane's joystick node
    # def disable_other_joystick_node(self) -> None:
        
    #     self.get_logger().info(f"disabling {self.other_crane} joytsick")
    #     cli = self.create_client(SetBool, 'joystick_node/' + self.other_crane + '/service/enable')

    #     # Determine if the other joystick service exists 
    #     if cli.service_is_ready():
    #         req = SetBool.Request()
    #         req.data = False
    #         self.fut = cli.call_async(req)
    #         def done_callback(result: Future):
    #             response: SetBool.Response = result.result()
    #             if response.success == True:
    #                 self.get_logger().info(f"Publishing message to {self.other_joystick_state_pub.topic_name} : False")
    #                 self.other_joystick_state_pub.publish(JoystickEnabled(enabled = False))
    #             else:
    #                 self.get_logger().error(f"Failed to disable {self.other_crane} joystick")
    #         self.fut.add_done_callback(done_callback)

    def enable_disable_callback(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        self.get_logger().info(f"{'enabling' if request.data else 'disabling'} {self.crane_name} joytsick")
        
        # If the joystick is given the enable command, attempt to disable the other node
        #if request.data == True:
            #self.disable_other_joystick_node()

        msg = JoystickEnabled(enabled = request.data)
        if request.data == False:
            self.publish_demand(Twist(linear=Vector3(x=0.0,y=0.0,z=0.0)))
        self.is_enabled = request.data

        response.success, response.message =  True, f"Joystick {'enabled' if request.data else 'disabled'}"
        success_msg = "Joystick: Enabled" if self.is_enabled else "Joystick: Disabled"

        self.get_logger().info(f'Success: {response.success}, {success_msg if response.success else f", Message: {response.message}"}')
        
        self.joystick_state_pub.publish(msg)
        return response

    def publish_demand(self, msg : Twist) -> None:
        if self.is_enabled:
            self.joystick_position_pub.publish(msg)
            self.get_logger().info("%s publishing demand" % msg.linear)
        else:
            self.get_logger().info(f'' + self.crane_name + ' joystick is disabled')