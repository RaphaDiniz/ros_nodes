#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tkinter as tk
from tkinter import ttk
import threading

class FirstNode(Node):

    def __init__(self):
        super().__init__("first_node")
        self.directions_pub_ = self.create_publisher(String, "directions", 10)
        self.get_logger().info("First node has started")

    def publish_direction(self, direction, value1, value2):
        if value1 > 0:
            message = f"e {direction} {value1} {value1}"
        elif value2 > 0:
            message = f"m {direction} {value2} {value2}"
        else:
            message = direction
        message = String(data=message)
        self.directions_pub_.publish(message)
        self.get_logger().info(f"Published: {message.data}")



def create_gui(node):
    root = tk.Tk()
    root.title("Control GUI")

    # Crie as abas
    tab_control = ttk.Notebook(root)
    tab1 = ttk.Frame(tab_control)
    tab2 = ttk.Frame(tab_control)
    tab_control.add(tab1, text="RPM Control")
    tab_control.add(tab2, text="Velocity Control")
    tab_control.pack(expand=1, fill="both")

    # Aba 1 - RPM Control
    rpm_label = tk.Label(tab1, text="RPM:")
    rpm_label.pack()

    rpm_slider = tk.Scale(tab1, from_=0, to=100, orient=tk.HORIZONTAL)
    rpm_slider.pack()

    def publish_rpm_callback(direction):
        node.publish_direction(direction, rpm_slider.get(), 0)

    button_a_rpm = tk.Button(tab1, text="A", command=lambda: publish_rpm_callback("A"))
    button_w_rpm = tk.Button(tab1, text="W", command=lambda: publish_rpm_callback("W"))
    button_s_rpm = tk.Button(tab1, text="S", command=lambda: publish_rpm_callback("S"))
    button_d_rpm = tk.Button(tab1, text="D", command=lambda: publish_rpm_callback("D"))

    button_a_rpm.pack(side=tk.LEFT)
    button_w_rpm.pack(side=tk.LEFT)
    button_s_rpm.pack(side=tk.LEFT)
    button_d_rpm.pack(side=tk.LEFT)

    # Aba 2 - Velocity Control
    velocity_label = tk.Label(tab2, text="Velocity:")
    velocity_label.pack()

    velocity_slider = tk.Scale(tab2, from_=0, to=100, orient=tk.HORIZONTAL)
    velocity_slider.pack()

    def publish_velocity_callback(direction):
        node.publish_direction(direction, 0, velocity_slider.get())

    button_a_velocity = tk.Button(tab2, text="A", command=lambda: publish_velocity_callback("A"))
    button_w_velocity = tk.Button(tab2, text="W", command=lambda: publish_velocity_callback("W"))
    button_s_velocity = tk.Button(tab2, text="S", command=lambda: publish_velocity_callback("S"))
    button_d_velocity = tk.Button(tab2, text="D", command=lambda: publish_velocity_callback("D"))

    button_a_velocity.pack(side=tk.LEFT)
    button_w_velocity.pack(side=tk.LEFT)
    button_s_velocity.pack(side=tk.LEFT)
    button_d_velocity.pack(side=tk.LEFT)

    root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    node = FirstNode()
    
    # Inicie a GUI em uma thread separada
    gui_thread = threading.Thread(target=create_gui, args=(node,))
    gui_thread.start()
    
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
