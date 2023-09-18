#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Importe o tipo de mensagem apropriado para suas direções

class DrawCircleNode(Node):

    def __init__(self):
        super().__init__("draw_circle")
        self.directions_sub_ = self.create_subscription(String, "directions", self.directions_callback, 10)  # Assine o mesmo tópico "directions" aqui
        self.get_logger().info("Draw circle node has started")

    def directions_callback(self, msg):
        self.get_logger().info(f"Received direction: {msg.data}")
        # Adicione aqui a lógica para controlar o movimento do círculo com base na direção recebida

def main(args=None):
    rclpy.init(args=args)
    node = DrawCircleNode()
    rclpy.spin(node)
    rclpy.shutdown()
