import numpy as np
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
        self.declare_parameter('wheel_distance', 0.235)
        self.wheel_distance = self.get_parameter(
            'wheel_distance').get_parameter_value().double_value

        self.publisher = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.loop_run = True

        # odom variables
        self.first_measure = True
        self.pos_curr = np.array([0.0, 0.0, 0.0])  # x, y, omega
        self.spd_curr = np.array([0.0, 0.0, 0.0])  # dx, dy, delta omega
        self.wheels_curr = np.array([0.0, 0.0])    # left, right
        self.last_updated = time.time()

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
            self.ser.reset_input_buffer()
            t = self.ser.readline()[:-2]
            try:
                out = json.loads(t)
                self.__update_odom(out['ENCODER_L'], out['ENCODER_R'])
                print(self.pos_curr)

            except:
                pass
            time.sleep(0.1)
        self.ser.close()

    def __update_odom(self, new_left, new_right):
        wheels_new = np.array([new_left, new_right])
        wheels_delta = wheels_new - self.wheels_curr
        d = (wheels_delta[0] + wheels_delta[1]) / 2
        delta_omega = (wheels_delta[1] - wheels_delta[0]) / self.wheel_distance
        delta_pos = np.array([
            d*np.cos(self.pos_curr[2] + 0.5 * delta_omega),
            d*np.sin(self.pos_curr[2] + 0.5 * delta_omega),
            delta_omega
        ])
        self.pos_curr += delta_pos
        self.wheels_curr = wheels_new
        if self.first_measure:
            self.first_measure = False
            self.last_updated = time.time()
            return
        dt = time.time() - self.last_updated
        self.spd_curr = delta_pos / dt
        self.last_updated = time.time()

    def timer_callback(self):
        msg = String()
        self.publisher.publish(msg)


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
