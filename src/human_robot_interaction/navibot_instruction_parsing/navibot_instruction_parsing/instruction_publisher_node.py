#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2 node for publishing behavioral instructions.

This node publishes natural language behavioral instructions from terminal input
to test and demonstrate the instruction parsing functionality.

Author: Wang Junhui <wjh_9696@163.com>
License: MIT
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from navibot_interfaces.msg import BehavioralInstruction
import json
import time
import logging
import threading
from typing import Dict, Any, List, Optional


class InstructionPublisherNode(Node):
    """
    ROS2 node for publishing behavioral instructions from terminal input.
    
    This node reads instructions from terminal input and publishes them
    for testing the instruction parsing system.
    """
    
    def __init__(self):
        """Initialize the instruction publisher node."""
        super().__init__('instruction_publisher_node')
        
        # Configure logging
        self.logger = self.get_logger()
        
        # Declare parameters
        self._declare_parameters()
        
        # Create publisher
        self._create_publisher()
        
        # Initialize terminal input thread
        self._running = True
        
        self.logger.info("Enter instructions in the terminal (type 'quit' to exit):")
    
    def _declare_parameters(self):
        """Declare node parameters."""
        self.declare_parameter('log_level', 'INFO')
        
        # Set logging level
        log_level = self.get_parameter('log_level').get_parameter_value().string_value
        # Set both Python logging and ROS2 logging levels
        logging.getLogger().setLevel(getattr(logging, log_level.upper()))
        # Also set the ROS2 logger level
        if hasattr(self.logger, 'set_level'):
            self.logger.set_level(getattr(logging, log_level.upper()))
    
    def _create_publisher(self):
        """Create ROS2 publisher for behavioral instructions."""
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        self.instruction_pub = self.create_publisher(
            BehavioralInstruction,
            'behavioral_instructions',
            qos_profile
        )
        
    
    def start_terminal_input(self):
        """Start terminal input thread."""
        input_thread = threading.Thread(target=self._terminal_input_loop, daemon=True)
        input_thread.start()
        return input_thread
    
    def _terminal_input_loop(self):
        """Terminal input loop for reading user instructions."""
        while self._running:
            try:
                instruction = input("> ").strip()
                
                if instruction.lower() in ['quit', 'exit', 'q']:
                    self._running = False
                    import os
                    import signal
                    os.kill(os.getpid(), signal.SIGINT)
                    break
                
                if instruction:
                    self.publish_raw_instruction(instruction)
                    
            except (EOFError, KeyboardInterrupt):
                self._running = False
                import os
                import signal
                os.kill(os.getpid(), signal.SIGINT)
                break
            except Exception as e:
                self.logger.error(f"Error reading input: {e}")
    
    def publish_raw_instruction(self, instruction: str) -> bool:
        """
        Publish a raw instruction from terminal input.
        
        Args:
            instruction: Raw instruction text from terminal
            
        Returns:
            bool: True if published successfully
        """
        try:
            # Create and publish message
            msg = BehavioralInstruction()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.online_constraints = [instruction]  # Terminal input as online constraint
            msg.offline_constraints = []  # Empty offline constraints
            
            self.logger.info(f"Publishing to topic 'behavioral_instructions': online={msg.online_constraints}, offline={msg.offline_constraints}")
            self.instruction_pub.publish(msg)
            self.logger.info(f"Successfully published instruction: {instruction}")
            
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to publish instruction: {e}")
            return False
    
    def stop(self):
        """Stop the node."""
        self._running = False
    
    


def main(args=None):
    """Main function to run the instruction publisher node."""
    rclpy.init(args=args)
    
    try:
        node = InstructionPublisherNode()
        
        # Start terminal input thread
        input_thread = node.start_terminal_input()
        
        # Keep node running
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\nShutting down...")
    except Exception as e:
        print(f"Node error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
