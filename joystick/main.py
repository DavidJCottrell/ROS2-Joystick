import joystick # type: ignore

import threading # type: ignore
import rclpy # type: ignore
from rclpy.node import Node # type: ignore

def main(args : None = None):

    rclpy.init(args=args)

    parameter_server = Node("parameter_server")
    
    parameter_server.declare_parameter('crane')
    parameter_server.declare_parameter('trigger_btn')
    parameter_server.declare_parameter('threshold')
    parameter_server.declare_parameter('accuracy')

    chosen_crane = parameter_server.get_parameter('crane').get_parameter_value().string_value
    trigger_btn = parameter_server.get_parameter('trigger_btn').get_parameter_value().string_value
    threshold = parameter_server.get_parameter('threshold').get_parameter_value().double_value
    accuracy = parameter_server.get_parameter('accuracy').get_parameter_value().double_value

    # The crane that the joystick should control should always be specified 
    if not (chosen_crane == "mast_crane" or chosen_crane == "grapple_crane"):
        sys.exit('Error: Please choose a valid crane argument ("grapple_crane" or "mast_crane")')

    joystick_node = joystick.JoystickNode(chosen_crane)

    controller = joystick.JoystickController(joystick_node, trigger_btn, threshold, accuracy)

    try:
        print("Starting Joystick Node")
        
        service_thread = threading.Thread(target=rclpy.spin, args=[joystick_node])
        service_thread.start()
        
        while 1: controller.process_events()

    except (KeyboardInterrupt, SystemExit) as _:
        rclpy.shutdown()
        service_thread.join()
        print("Joystick Node Finished.")


if __name__ == '__main__':
    main()
