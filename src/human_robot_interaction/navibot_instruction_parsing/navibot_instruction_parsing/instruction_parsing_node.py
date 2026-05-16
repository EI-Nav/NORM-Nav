#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2 node for behavioral instruction parsing.

This node provides a service interface for parsing natural language
behavioral instructions into structured representations.

Author: Wang Junhui <wjh_9696@163.com>
License: MIT
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from navibot_interfaces.msg import BehavioralInstruction
import logging
from typing import Any, Optional, List

from .instruction_parser import BehavioralInstructionParser, InstructionInput
from .llm_interface import create_llm_interface

# Import ROS2 message types
try:
    from navibot_interfaces.msg import BehavioralConstraintArray
except ImportError:
    BehavioralConstraintArray = None


class InstructionParsingNode(Node):
    """
    ROS2 node for behavioral instruction parsing.
    
    This node provides services for parsing natural language behavioral
    instructions into structured representations suitable for navigation.
    """
    
    def __init__(self):
        """Initialize the instruction parsing node."""
        super().__init__('instruction_parsing_node')
        
        # Configure logging
        self.logger = self.get_logger()
        
        # Declare parameters
        self._declare_parameters()
        
        # Initialize LLM interface
        self._initialize_llm_interface()
        
        # Get default values from parameters
        default_values = {
            'direction_constrain': self.get_parameter('output.defaults.direction_constrain').get_parameter_value().double_value,
            'velocity_constrain': self.get_parameter('output.defaults.velocity_constrain').get_parameter_value().double_value,
            'traversability_constrain': self.get_parameter('output.defaults.traversability_constrain').get_parameter_value().integer_value
        }
        
        # Initialize instruction parser with default values
        self.parser = BehavioralInstructionParser(self.llm_interface, default_values)
        
        # Create publishers and subscribers
        self._create_publishers()
        self._create_subscribers()
        
        # Create service
        self._create_service()
        
    
    def _declare_parameters(self) -> None:
        """Declare node parameters."""
        # LLM configuration
        self.declare_parameter('llm.interface_type', 'openai')
        self.declare_parameter('llm.api_key', '')
        self.declare_parameter('llm.model', 'qwen-flash')
        self.declare_parameter('llm.max_tokens', 1000)
        self.declare_parameter('llm.temperature', 0.1)
        self.declare_parameter('llm.base_url', 'https://dashscope.aliyuncs.com/compatible-mode/v1')
        
        # Node configuration
        self.declare_parameter('publish_structured_output', True)
        self.declare_parameter('log_level', 'INFO')
        
        # Processing configuration
        self.declare_parameter('processing.llm_timeout', 30.0)
        self.declare_parameter('processing.max_retries', 3)
        self.declare_parameter('processing.retry_delay', 1.0)
        self.declare_parameter('processing.enable_caching', True)
        self.declare_parameter('processing.cache_timeout', 300.0)
        self.declare_parameter('processing.enable_cot', True)
        self.declare_parameter('processing.log_reasoning', True)
        
        # Output configuration
        self.declare_parameter('output.include_metadata', True)
        self.declare_parameter('output.validate_constraints', True)
        self.declare_parameter('output.defaults.direction_constrain', 0.0)
        self.declare_parameter('output.defaults.velocity_constrain', 0.5)
        self.declare_parameter('output.defaults.traversability_constrain', 0)
        
        # Conflict resolution
        self.declare_parameter('conflict_resolution.strategy', 'online_priority')
        self.declare_parameter('conflict_resolution.log_conflicts', True)
        self.declare_parameter('conflict_resolution.max_constraints_per_object', 5)
        
        # Set logging level
        log_level = self.get_parameter('log_level').get_parameter_value().string_value
        logging.getLogger().setLevel(getattr(logging, log_level.upper()))
        
        # Validate parameters
        self._validate_parameters()
    
    def _initialize_llm_interface(self) -> None:
        """Initialize the LLM interface based on configuration."""
        interface_type = self.get_parameter('llm.interface_type').get_parameter_value().string_value
        
        config = {
            'api_key': self.get_parameter('llm.api_key').get_parameter_value().string_value,
            'model': self.get_parameter('llm.model').get_parameter_value().string_value,
            'max_tokens': self.get_parameter('llm.max_tokens').get_parameter_value().integer_value,
            'temperature': self.get_parameter('llm.temperature').get_parameter_value().double_value,
            'base_url': self.get_parameter('llm.base_url').get_parameter_value().string_value,
        }
        
        try:
            self.llm_interface = create_llm_interface(interface_type, config)
            # Log interface and model information
            self.logger.info(f"LLM interface initialized: {interface_type}")
            self.logger.info(f"Model: {config['model']}")
        except Exception as e:
            self.logger.error(f"Failed to initialize LLM interface: {e}")
            raise
    
    def _create_publishers(self) -> None:
        """Create ROS2 publishers."""
        # Use RELIABLE QoS for behavioral constraints to ensure all subscribers receive critical navigation instructions
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Use BehavioralConstraintArray if available, otherwise fallback to String
        if BehavioralConstraintArray is not None:
            self.structured_output_pub = self.create_publisher(
                BehavioralConstraintArray,
                'behavioral_constraints',
                qos_profile
            )
        else:
            self.structured_output_pub = self.create_publisher(
                String,
                'behavioral_constraints',
                qos_profile
            )
        
    
    def _create_subscribers(self) -> None:
        """Create ROS2 subscribers."""
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        self.instruction_sub = self.create_subscription(
            BehavioralInstruction,
            'behavioral_instructions',
            self._instruction_callback,
            qos_profile
        )
        
    
    def _create_service(self) -> None:
        """Create ROS2 service for instruction parsing."""
        # Import service interface from navibot_interfaces
        try:
            from navibot_interfaces.srv import ParseInstructions
        except ImportError:
            # Fallback for development - create a simple service interface
            self.logger.warning("Service interface not available - using topic-based communication")
            return
        
        self.parse_service = self.create_service(
            ParseInstructions,
            'parse_behavioral_instructions',
            self._parse_instructions_callback
        )
        
    
    def _instruction_callback(self, msg: BehavioralInstruction) -> None:
        """
        Callback for incoming behavioral instructions.
        
        Args:
            msg: BehavioralInstruction message containing instructions
        """
        try:
            # 1. Input validation
            if not self._validate_instruction_message(msg):
                self.logger.warn("Invalid instruction message received")
                return
            
            # Parse instructions directly from message fields
            instruction_input = InstructionInput(
                online_constraints=msg.online_constraints,
                offline_constraints=msg.offline_constraints
            )
            
            # Display input instructions
            self.logger.debug("Processing instruction message")
            if instruction_input.online_constraints:
                self.logger.debug(f"Online constraints: {instruction_input.online_constraints}")
            if instruction_input.offline_constraints:
                self.logger.debug(f"Offline constraints: {instruction_input.offline_constraints}")
            
            # Get CoT configuration
            enable_cot = self.get_parameter('processing.enable_cot').get_parameter_value().bool_value
            
            structured_output = self.parser.parse_instructions(instruction_input, enable_cot)
            
            # Display model response (simplified)
            self.logger.info(f"Parsed {len(structured_output.constraints)} behavioral constraints")
            for i, constraint in enumerate(structured_output.constraints):
                self.logger.debug(f"Constraint {i+1}: objects={constraint.object_list}, "
                               f"direction={constraint.direction_constrain}, "
                               f"velocity={constraint.velocity_constrain}, "
                               f"traversability={constraint.traversability_constrain}")
            
            # Publish structured output
            if self.get_parameter('publish_structured_output').get_parameter_value().bool_value:
                if BehavioralConstraintArray is not None:
                    # Use ROS2 message
                    output_msg = structured_output.to_ros_msg_array()
                    self.structured_output_pub.publish(output_msg)
                    self.logger.debug("Published BehavioralConstraintArray to 'behavioral_constraints' topic")
                else:
                    # Fallback to JSON string
                    output_msg = String()
                    output_msg.data = structured_output.to_json()
                    self.structured_output_pub.publish(output_msg)
                    self.logger.debug("Published JSON string to 'behavioral_constraints' topic")
            
        except ValueError as e:
            self.logger.error(f"Value error in instruction callback: {e}")
            self._recover_from_value_error()
            
        except RuntimeError as e:
            self.logger.error(f"Runtime error in instruction callback: {e}")
            self._recover_from_runtime_error()
            
        except Exception as e:
            self.logger.error(f"Unexpected error in instruction callback: {e}")
            self._handle_unexpected_error(e)
    
    def _parse_instructions_callback(self, request: Any, response: Any) -> Any:
        """
        Service callback for parsing behavioral instructions.
        
        Args:
            request: Service request
            response: Service response
            
        Returns:
            Service response with structured output
        """
        try:
            # 1. Input validation
            if not self._validate_service_request(request):
                self.logger.warn("Invalid service request received")
                response.success = False
                response.structured_output = ""
                response.message = "Invalid request parameters"
                return response
            
            # Parse request
            instruction_input = InstructionInput(
                online_constraints=request.online_constraints,
                offline_constraints=request.offline_constraints
            )
            
            # Display input instructions
            self.logger.debug("Processing service request")
            if instruction_input.online_constraints:
                self.logger.debug(f"Online constraints: {instruction_input.online_constraints}")
            if instruction_input.offline_constraints:
                self.logger.debug(f"Offline constraints: {instruction_input.offline_constraints}")
            
            # Get CoT configuration
            enable_cot = self.get_parameter('processing.enable_cot').get_parameter_value().bool_value
            
            # Parse instructions
            structured_output = self.parser.parse_instructions(instruction_input, enable_cot)
            
            # Display model response (simplified)
            self.logger.info(f"Parsed {len(structured_output.constraints)} behavioral constraints")
            for i, constraint in enumerate(structured_output.constraints):
                self.logger.debug(f"Constraint {i+1}: objects={constraint.object_list}, "
                               f"direction={constraint.direction_constrain}, "
                               f"velocity={constraint.velocity_constrain}, "
                               f"traversability={constraint.traversability_constrain}")
            
            # Prepare response
            response.success = True
            response.structured_output = structured_output.to_json()
            response.message = f"Successfully parsed {len(structured_output.constraints)} constraints"
            
            self.logger.debug(f"Service response: {response.message}")
            
        except ValueError as e:
            self.logger.error(f"Value error in service callback: {e}")
            response.success = False
            response.structured_output = ""
            response.message = f"Value error: {str(e)}"
            
        except RuntimeError as e:
            self.logger.error(f"Runtime error in service callback: {e}")
            response.success = False
            response.structured_output = ""
            response.message = f"Runtime error: {str(e)}"
            
        except Exception as e:
            self.logger.error(f"Unexpected error in service callback: {e}")
            response.success = False
            response.structured_output = ""
            response.message = f"Error: {str(e)}"
        
        return response
    
    def _validate_parameters(self) -> None:
        """Validate all node parameters."""
        try:
            # Validate LLM parameters
            interface_type = self.get_parameter('llm.interface_type').get_parameter_value().string_value
            valid_interfaces = ['openai', 'local']
            if interface_type not in valid_interfaces:
                raise ValueError(f"Invalid LLM interface type: {interface_type}. Valid types: {valid_interfaces}")
            
            temperature = self.get_parameter('llm.temperature').get_parameter_value().double_value
            if not 0.0 <= temperature <= 2.0:
                raise ValueError(f"Temperature must be between 0.0 and 2.0: {temperature}")
            
            max_tokens = self.get_parameter('llm.max_tokens').get_parameter_value().integer_value
            if not 1 <= max_tokens <= 10000:
                raise ValueError(f"Max tokens must be between 1 and 10000: {max_tokens}")
            
            # Validate processing parameters
            llm_timeout = self.get_parameter('processing.llm_timeout').get_parameter_value().double_value
            if not 1.0 <= llm_timeout <= 300.0:
                raise ValueError(f"LLM timeout must be between 1.0 and 300.0 seconds: {llm_timeout}")
            
            max_retries = self.get_parameter('processing.max_retries').get_parameter_value().integer_value
            if not 0 <= max_retries <= 10:
                raise ValueError(f"Max retries must be between 0 and 10: {max_retries}")
            
            # Validate output parameters
            direction_constrain = self.get_parameter('output.defaults.direction_constrain').get_parameter_value().double_value
            if not -1.0 <= direction_constrain <= 1.0:
                raise ValueError(f"Default direction constraint must be between -1.0 and 1.0: {direction_constrain}")
            
            velocity_constrain = self.get_parameter('output.defaults.velocity_constrain').get_parameter_value().double_value
            if not 0.0 <= velocity_constrain <= 1.0:
                raise ValueError(f"Default velocity constraint must be between 0.0 and 1.0: {velocity_constrain}")
            
            traversability_constrain = self.get_parameter('output.defaults.traversability_constrain').get_parameter_value().integer_value
            if traversability_constrain not in [0, 1, 2]:
                raise ValueError(f"Default traversability constraint must be 0, 1, or 2: {traversability_constrain}")
            
            self.logger.debug("All parameters validated successfully")
            
        except Exception as e:
            self.logger.error(f"Parameter validation failed: {e}")
            self._set_default_parameters()
            raise
    
    def _set_default_parameters(self) -> None:
        """Set default parameters when validation fails."""
        self.logger.warn("Setting default parameters due to validation failure")
        
        # Set safe default values
        self.set_parameters([
            rclpy.parameter.Parameter('llm.interface_type', value='openai'),
            rclpy.parameter.Parameter('llm.temperature', value=0.1),
            rclpy.parameter.Parameter('llm.max_tokens', value=1000),
            rclpy.parameter.Parameter('processing.llm_timeout', value=30.0),
            rclpy.parameter.Parameter('processing.max_retries', value=3),
            rclpy.parameter.Parameter('output.defaults.direction_constrain', value=0.0),
            rclpy.parameter.Parameter('output.defaults.velocity_constrain', value=0.5),
            rclpy.parameter.Parameter('output.defaults.traversability_constrain', value=0)
        ])
    
    def _validate_instruction_message(self, msg: BehavioralInstruction) -> bool:
        """Validate instruction message."""
        if not msg:
            return False
        
        # Check if at least one constraint list is provided
        if not msg.online_constraints and not msg.offline_constraints:
            return False
        
        # Check if constraint lists are valid
        if msg.online_constraints is None or msg.offline_constraints is None:
            return False
        
        return True
    
    def _validate_service_request(self, request: Any) -> bool:
        """Validate service request."""
        if not request:
            return False
        
        # Check if request has required attributes
        if not hasattr(request, 'online_constraints') or not hasattr(request, 'offline_constraints'):
            return False
        
        # Check if at least one constraint list is provided
        if not request.online_constraints and not request.offline_constraints:
            return False
        
        return True
    
    def _recover_from_value_error(self) -> None:
        """Recover from value error."""
        self.logger.info("Attempting recovery from value error")
        # Reset parser state if needed
        if hasattr(self, 'parser'):
            self.logger.debug("Parser state reset")
    
    def _recover_from_runtime_error(self) -> None:
        """Recover from runtime error."""
        self.logger.info("Attempting recovery from runtime error")
        # Reinitialize LLM interface if needed
        try:
            self._initialize_llm_interface()
            self.logger.info("LLM interface reinitialized")
        except Exception as e:
            self.logger.error(f"Failed to reinitialize LLM interface: {e}")
    
    def _handle_unexpected_error(self, e: Exception) -> None:
        """Handle unexpected errors."""
        self.logger.error(f"Handling unexpected error: {e}")
        # Log error statistics
        self.logger.error(f"Error type: {type(e).__name__}")
        self.logger.error(f"Error message: {str(e)}")
    
    def destroy_node(self) -> None:
        """Node destruction with resource cleanup."""
        self.logger.info("Destroying instruction parsing node")
        
        # Cleanup LLM interface
        if hasattr(self, 'llm_interface'):
            try:
                # Close any open connections
                if hasattr(self.llm_interface, 'close'):
                    self.llm_interface.close()
                self.logger.debug("LLM interface cleaned up")
            except Exception as e:
                self.logger.warn(f"Error cleaning up LLM interface: {e}")
        
        # Cleanup parser
        if hasattr(self, 'parser'):
            try:
                # Reset parser state
                if hasattr(self.parser, 'reset'):
                    self.parser.reset()
                self.logger.debug("Parser cleaned up")
            except Exception as e:
                self.logger.warn(f"Error cleaning up parser: {e}")
        
        # Call parent destroy
        super().destroy_node()
        self.logger.info("Instruction parsing node destroyed")


def main(args: Optional[List[str]] = None) -> None:
    """Main function to run the instruction parsing node."""
    rclpy.init(args=args)
    
    try:
        node = InstructionParsingNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Node error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
