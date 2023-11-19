import time
import serial
import json
import rclpy
import threading

from rclpy.node import Node

from std_msgs.msg import String


class DriverNode(Node):
    def __init__(self):
        super().__init__('pico_robot_driver')
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.publisher = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.loop_run = True

        self.ser = None
        serial_port = self.get_parameter(
            'serial_port').get_parameter_value().string_value
        try:
            self.ser = serial.Serial(serial_port)
        except Exception as error:
            self.get_logger().error(error)
            return
        self.t = threading.Thread(target=self.__logic_loop)
        self.t.start()

    def stop_loop(self):
        self.loop_run = False
        self.t.join()

    def __logic_loop(self):
        while self.loop_run:
            serial_port = self.get_parameter(
                'serial_port').get_parameter_value().string_value
            self.ser.reset_input_buffer()
            t = self.ser.readline()[:-2]
            try:
                out = json.loads(t)
                print(out)
            except:
                pass
            time.sleep(0.1)
        self.ser.close()

    def timer_callback(self):
        msg = String()
        msg.data = f"Hello World: {self.i}"
        self.publisher.publish(msg)
        self.get_logger().info(f"Publishing: {msg.data}")
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    driver_node = DriverNode()
    if driver_node.ser is not None:
        rclpy.spin(driver_node)
        driver_node.ser.close()

    driver_node.stop_loop()
    driver_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
