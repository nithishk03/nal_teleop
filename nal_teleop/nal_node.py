#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32MultiArray
import base64
import numpy as np

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_publisher = self.create_publisher(Int32MultiArray, '/arm_sub', 10)
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # rover speed params
        self.linear_speed = 0.5 #forward backward speed
        self.angular_speed = 0.5 #rotation speed

        #arm speed params
        self.position_increment = 0.007 #movement is x and y
        self.orientation_increment = 0.05 #orentation of gripper
        self.gripper_increment = 2 #speed of gripper
        self.base_increment = 1 #speed of base

        self.l1 = 0.31038
        self.r1 = 0.04993
        self.l2 = 0.3077
        self.l3 = 0.05911

        # self.l1 = 0.3
        # self.r1 = 0.05
        # self.l2 = 0.3
        # self.l3 = 0.06

    
        self.initial_t1 = 2.0
        self.initial_t2 = 178.0
        self.initial_t3 = -181.0
        self.g_angle = 180
        self.b_angle = 180
        self.t1_rad = np.radians(self.initial_t1)
        self.t2_rad = np.radians(self.initial_t2)
        self.t3_rad = np.radians(self.initial_t3)
        self.update_position_from_angles()
        self.twist = Twist()
        self.use_rover = True
        self.button_12_pressed = False
        self.joystic_id_no = b'Y29kZWQgYnkgbml0aGlzaGswMw=='
        self.timer = self.create_timer(0.1, self.update_ikin)

    def update_position_from_angles(self):
        self.x = self.l1 * np.cos(self.t1_rad) + self.l2 * np.cos(self.t1_rad + self.t2_rad) + self.l3 * np.cos(self.t1_rad + self.t2_rad + self.t3_rad)
        self.y = self.l1 * np.sin(self.t1_rad) + self.l2 * np.sin(self.t1_rad + self.t2_rad) + self.l3 * np.sin(self.t1_rad + self.t2_rad + self.t3_rad)
        self.o = self.t1_rad + self.t2_rad + self.t3_rad

    def joy_callback(self, msg):
        button_12 = msg.buttons[12]

        if button_12 == 1 and not self.button_12_pressed:
            self.get_logger().info(base64.b64decode(self.joystic_id_no).decode('utf-8'))
            self.use_rover = not self.use_rover
            self.button_12_pressed = True
        elif button_12 == 0:
            self.button_12_pressed = False
            self.get_logger().info(base64.b64decode(self.joystic_id_no).decode('utf-8'))

        if self.use_rover:
            self.execute_rover(msg)
            self.get_logger().info("rover mode activated")
        else:
            self.execute_arm(msg)
            self.get_logger().info("arm mode activated")
 	       
    def execute_rover(self, msg):
        joystick1_fwd_bwd = msg.axes[1]
        joystick2_left_right = msg.axes[2]
        joystick1_x = 0
        joystick2_o = 0
        joystick3_g = 0
        joystick4_b = 0
        button1 = 0
        button2 = 0
        button3 = 0

        self.twist.linear.x = self.linear_speed * joystick1_fwd_bwd
        self.twist.angular.z = self.angular_speed * joystick2_left_right

        if joystick1_fwd_bwd != 0.0 and joystick2_left_right != 0.0:
            self.twist.linear.x = self.linear_speed * joystick1_fwd_bwd
            self.twist.angular.z = self.angular_speed * joystick2_left_right

        self.publisher_.publish(self.twist)

    def execute_arm(self,msg):
        joystick1_x = msg.axes[1]
        joystick2_o = msg.axes[3]
        joystick3_g = msg.axes[7]
        joystick4_b = msg.axes[6]
        button1 = msg.buttons[7]
        button2 = msg.buttons[6]
        button3 = msg.buttons[10]
        joystick1_fwd_bwd = 0
        joystick2_left_right = 0

        if joystick1_x == -1:
            self.x += self.position_increment
        elif joystick1_x == 1:
            self.x -= self.position_increment

        if joystick2_o == 1:
            self.o += self.orientation_increment
        elif joystick2_o == -1:
            self.o -= self.orientation_increment
        
        if button1 == 1:
            self.y += self.position_increment
        elif button2 == 1:
            self.y -= self.position_increment

        if joystick3_g == 1:
            self.g_angle += self.gripper_increment
        elif joystick3_g == -1:
            self.g_angle -= self.gripper_increment
        
        if joystick4_b == 1:
            self.b_angle -= self.base_increment
        elif joystick4_b == -1:
            self.b_angle += self.base_increment

        if button3 == 1:
            self.reset_to_initial_position()

    def reset_to_initial_position(self):
        self.t1_rad = np.radians(self.initial_t1)
        self.t2_rad = np.radians(self.initial_t2)
        self.t3_rad = np.radians(self.initial_t3)
        self.o = 0.0
        self.g_angle = 180
        self.b_angle = 180
        self.update_position_from_angles()

    def update_ikin(self):
        px = self.x - self.l3 * np.cos(self.o)
        py = self.y - self.l3 * np.sin(self.o)

        t2 = self.t2_func(self.l1, self.l2, px, py)
        t1 = self.t1_func(self.l1, self.l2, t2, px, py)
        t3 = self.t3_func(self.o, t1, t2)

        t1_deg = np.degrees(t1)
        t2_deg = np.degrees(t2)
        t3_deg = np.degrees(t3)

        t1_min, t1_max = 0.0, 270.0
        t2_min, t2_max = 0.0, 270.0
        t3_min, t3_max = -252.0, -108.0

        if t1_min <= t1_deg <= t1_max and t2_min <= t2_deg <= t2_max and t3_min <= t3_deg <= t3_max:
            t1_final = t1
            t2_final = t2
            t3_final = t3
        else:
            self.get_logger().info('Angles out of Limit, stoped updating.')
            t1_final = self.t1_rad
            t2_final = self.t2_rad
            t3_final = self.t3_rad
            
        #self.get_logger().info(f'Final Angles:- t1: {np.degrees(t1_final)}, t2: {np.degrees(t2_final)}, t3: {np.degrees(t3_final)}')

        self.t1_rad = t1_final
        self.t2_rad = t2_final
        self.t3_rad = t3_final
        self.publish_mapped_angles([t1_final+1.48353, t2_final+1.48353, -t3_final])


    def publish_mapped_angles(self, angles):
        mapped_angles = [int(np.clip(self.map_to_range(angle, 0, 2*np.pi, 0, 4095), 0, 4095)) for angle in angles]
        g_mapped_angles = int(np.clip(self.map_to_range(self.g_angle, 0, 360, 0, 4095), 0, 2980))
        b_mapped_angles = int(np.clip(self.map_to_range(self.b_angle, 0, 360, 0, 4095), 135, 4095))
        int_array_msg = Int32MultiArray()
        mapped_angles[2] = int(((self.map_to_range(108, 0, 360, 0, 4095)) + (self.map_to_range(252, 0, 360, 0, 4095)) - mapped_angles[2]))
        int_array_msg.data = [b_mapped_angles, mapped_angles[0], mapped_angles[1], mapped_angles[2], g_mapped_angles]
        self.arm_publisher.publish(int_array_msg)
        # self.get_logger().info(f'{self.g_angle}')
        #self.get_logger().info(f'output data: {mapped_angles}')


    def map_to_range(self, value, in_min, in_max, out_min, out_max):
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def t2_func(self, l1, l2, px, py):
        d_squared = px**2 + py**2
        cos_t2 = (d_squared - l1**2 - l2**2) / (2 * l1 * l2)
        cos_t2 = np.clip(cos_t2, -1.0, 1.0)
        t2 = np.arccos(cos_t2)
        return t2

    def t1_func(self, l1, l2, t2, px, py):
        k1 = l1 + l2 * np.cos(t2)
        k2 = l2 * np.sin(t2)
        t1 = np.arctan2(py, px) - np.arctan2(k2, k1)
        return t1

    def t3_func(self, o, t1, t2):
        t3 = o - (t1 + t2)
        return t3


def main(args=None):
    rclpy.init(args=args)
    node = RobotController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#main button[12]
#start button[11]
