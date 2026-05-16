#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Behavioral instruction parsing module for natural language navigation commands.

This module implements the core functionality for parsing natural language
behavioral instructions into structured representations that can be used
by the multi-layer costmap system.

Author: Wang Junhui <wjh_9696@163.com>
License: MIT
"""

from typing import List, Dict, Any, Optional, Tuple
from dataclasses import dataclass
from enum import Enum
import json
import logging

from .llm_interface import LLMInterface
from .structured_output import BehavioralConstraint, StructuredOutput


class ConstraintType(Enum):
    """Type of behavioral constraint."""
    ONLINE = "online"  # Short-term specifications
    OFFLINE = "offline"  # Long-term specifications


@dataclass
class InstructionInput:
    """Input structure for behavioral instructions."""
    online_constraints: List[str]
    offline_constraints: List[str]


class BehavioralInstructionParser:
    """
    Core behavioral instruction parser.
    
    This class handles the parsing of natural language behavioral instructions
    into structured representations suitable for navigation systems.
    """
    
    def __init__(self, llm_interface: LLMInterface, default_values: Optional[Dict[str, Any]] = None):
        """
        Initialize the behavioral instruction parser.
        
        Args:
            llm_interface: LLM interface for natural language processing
            default_values: Default values for constraints when not specified
        """
        self.llm_interface = llm_interface
        self.logger = logging.getLogger(__name__)
        self.default_values = default_values or {
            'direction_constrain': None,
            'velocity_constrain': None,
            'traversability_constrain': 0
        }
        
    def parse_instructions(self, instruction_input: InstructionInput, enable_cot: bool = True) -> StructuredOutput:
        """
        Parse behavioral instructions into structured output.
        
        Args:
            instruction_input: Input containing online and offline constraints
            enable_cot: Whether to enable Chain of Thought reasoning
            
        Returns:
            StructuredOutput: Parsed behavioral constraints
        """
        try:
            # Store instruction input for potential fallback use
            self._last_instruction_input = instruction_input
            
            # Step 1: Merge online and offline constraints
            merged_instructions = self._merge_instructions(instruction_input)
            
            # Step 2: Process with LLM (with retry mechanism)
            max_retries = 2
            structured_constraints = []
            reasoning_process = {}
            
            for attempt in range(max_retries + 1):
                try:
                    llm_output = self._process_with_llm(merged_instructions, enable_cot)
                    structured_constraints, reasoning_process = self._parse_structured_output(llm_output)
                    
                    # If we got valid constraints, break out of retry loop
                    if structured_constraints:
                        break
                    elif attempt < max_retries:
                        self.logger.warning("Attempt %d failed, retrying with simplified prompt...", attempt + 1)
                        # Try with simplified prompt on retry
                        enable_cot = False
                        
                except ConnectionError as e:
                    self.logger.error("Connection error on attempt %d: %s", attempt + 1, e)
                    if attempt == max_retries:
                        self.logger.warning("All parsing attempts failed due to connection issues, using fallback constraints")
                        structured_constraints, reasoning_process = self._create_fallback_constraints(instruction_input)
                        
                except TimeoutError as e:
                    self.logger.error("Timeout error on attempt %d: %s", attempt + 1, e)
                    if attempt == max_retries:
                        self.logger.warning("All parsing attempts failed due to timeout, using fallback constraints")
                        structured_constraints, reasoning_process = self._create_fallback_constraints(instruction_input)
                        
                except ValueError as e:
                    self.logger.error("Value error on attempt %d: %s", attempt + 1, e)
                    if attempt == max_retries:
                        self.logger.warning("All parsing attempts failed due to value error, using fallback constraints")
                        structured_constraints, reasoning_process = self._create_fallback_constraints(instruction_input)
                        
                except Exception as e:
                    self.logger.error("Unexpected error on attempt %d: %s", attempt + 1, e)
                    if attempt == max_retries:
                        self.logger.warning("All parsing attempts failed, using fallback constraints")
                        structured_constraints, reasoning_process = self._create_fallback_constraints(instruction_input)
            
            # Step 4: Resolve conflicts (online constraints have priority)
            resolved_constraints = self._resolve_conflicts(structured_constraints)
            
            # Step 5: Create structured output with reasoning metadata
            metadata = {
                "reasoning_process": reasoning_process,
                "total_constraints": len(resolved_constraints),
                "cot_enabled": enable_cot
            }
            
            return StructuredOutput(constraints=resolved_constraints, metadata=metadata)
            
        except Exception as e:
            self.logger.error("Critical error in parse_instructions: %s", e)
            # Return minimal fallback output
            fallback_constraints, fallback_metadata = self._create_fallback_constraints(instruction_input)
            return StructuredOutput(constraints=fallback_constraints, metadata=fallback_metadata)
    
    def _merge_instructions(self, instruction_input: InstructionInput) -> str:
        """
        Merge online and offline constraints into a single instruction set.
        
        Args:
            instruction_input: Input containing constraints
            
        Returns:
            str: Merged instruction text
        """
        merged_parts = []
        
        if instruction_input.offline_constraints:
            merged_parts.append("Long-term behavioral constraints:")
            for constraint in instruction_input.offline_constraints:
                merged_parts.append(f"- {constraint}")
        
        if instruction_input.online_constraints:
            merged_parts.append("Short-term behavioral constraints:")
            for constraint in instruction_input.online_constraints:
                merged_parts.append(f"- {constraint}")
        
        return "\n".join(merged_parts)
    
    def _process_with_llm(self, merged_instructions: str, enable_cot: bool = True) -> str:
        """
        Process merged instructions with LLM.
        
        Args:
            merged_instructions: Merged instruction text
            enable_cot: Whether to enable Chain of Thought reasoning
            
        Returns:
            str: LLM response
        """
        try:
            prompt = self._build_llm_prompt(merged_instructions, enable_cot)
            self.logger.debug("Processing LLM request with CoT: %s", enable_cot)
            response = self.llm_interface.process_instruction(prompt)
            self.logger.debug("LLM processing completed successfully")
            return response
            
        except ConnectionError as e:
            self.logger.error("LLM connection error: %s", e)
            raise
            
        except TimeoutError as e:
            self.logger.error("LLM timeout error: %s", e)
            raise
            
        except ValueError as e:
            self.logger.error("LLM value error: %s", e)
            raise
            
        except Exception as e:
            self.logger.error("LLM processing error: %s", e)
            raise
    
    def _build_llm_prompt(self, instructions: str, enable_cot: bool = True) -> str:
        """
        Build prompt for LLM processing with optional Chain of Thought.
        
        Args:
            instructions: Instruction text
            enable_cot: Whether to enable Chain of Thought reasoning
            
        Returns:
            str: Formatted prompt
        """
        if enable_cot:
            prompt_parts = [
                "You are a behavioral instruction parser for a mobile robot navigation system.",
                "Parse the following natural language behavioral instructions into structured format.",
                "",
                "Instructions:",
                instructions,
                "",
                "Please follow this step-by-step reasoning process:",
                "",
                "1. **Analysis**: Analyze the instructions and identify:",
                "   - What objects/landmarks are mentioned?",
                "   - What spatial preferences are expressed (left, right, center)?",
                "   - What velocity requirements are mentioned (slow, fast, normal)?",
                "   - What object traversability constraints are implied?",
                "",
                "2. **Reasoning**: For each identified constraint, explain:",
                "   - Why this constraint is needed based on the instruction",
                "   - How the semantic meaning maps to numeric values",
                "   - What priority this constraint should have",
                "",
                "3. **Mapping**: Convert semantic instructions to numeric values:",
                "   - Direction: -1.0 (strong left) to 1.0 (strong right), 0.0 (center)",
                "   - Velocity: 0.0 (stop), 0.1-0.3 (very slow), 0.3-0.5 (slow), 0.5 (normal), 0.5-0.7 (slightly fast), 0.7-1.0 (fast)",
                "   - Object Traversability: 0 (no constraint), 1 (traversable), 2 (non-traversable)",
                "",
                "4. **Output**: Provide the final JSON array of behavioral constraints.",
                "",
                "Output format:",
                "```",
                "Analysis: [Your step-by-step analysis]",
                "Reasoning: [Your reasoning for each constraint]",
                "Mapping: [How you mapped semantic to numeric values]",
                "JSON: [",
                "  {",
                "    \"object_list\": [\"object1\", \"object2\", ...],",
                "    \"direction_constrain\": <numeric_value_or_null>,  # -1.0 (left) to 1.0 (right), 0.0 (center), null (no constraint)",
                "    \"velocity_constrain\": <numeric_value_or_null>,  # 0.0 (stop), 0.1-0.3 (very slow), 0.3-0.5 (slow), 0.5 (normal), 0.5-0.7 (slightly fast), 0.7-1.0 (fast), null (no constraint)",
                "    \"traversability_constrain\": <integer_value>  # Object traversability: 0 (no constraint), 1 (traversable), 2 (non-traversable)",
                "  }",
                "]",
                "```",
                "",
                "Rules:",
                "1. Extract behavioral landmarks from the instructions",
                "2. For direction constraints:",
                "   - Use -1.0 for strong left preference, 1.0 for strong right preference",
                "   - Use 0.0 for center preference",
                "   - Use intermediate values (e.g., -0.3 for slight left, 0.7 for slight right)",
                "   - Use null if no direction preference is specified",
                "3. For velocity constraints:",
                "   - Use 0.0 for stop (complete halt)",
                "   - Use 0.1-0.3 for very slow movement",
                "   - Use 0.3-0.5 for slow movement",
                "   - Use 0.5 for normal speed",
                "   - Use 0.5-0.7 for slightly fast movement",
                "   - Use 0.7-1.0 for fast movement",
                "   - Use null if no velocity preference is specified",
                "4. For object traversability constraints:",
                "   - Use 1 if the instruction explicitly states the object should be traversable",
                "   - Use 2 if the instruction explicitly states the object should not be traversable",
                "   - Use 0 if no explicit object traversability constraint is mentioned",
                "5. Online constraints have priority over offline constraints in case of conflicts",
                "6. If no explicit behavioral specification is provided, set the corresponding field to null",
                "7. A single behavioral constraint can be associated with multiple objects",
                "8. Output all text in English only",
                "9. Provide numeric values that reflect the semantic intensity of the instruction",
                "10. Be explicit about your reasoning process in the Analysis, Reasoning, and Mapping sections",
                "11. Object constraint assignment rules:",
                "    - When instructions explicitly mention objects, apply ALL behavioral constraints (direction, velocity, traversability) to that object",
                "    - Only use 'self' as object_list when instructions mention NO objects at all",
                "    - Examples:",
                "      * 'Go slowly' → {\"object_list\": [\"self\"], \"velocity_constrain\": 0.3, ...}",
                "      * 'Drive on the left side of the road' → {\"object_list\": [\"road\"], \"direction_constrain\": -0.5, \"traversability_constrain\": 1, ...}",
                "      * 'Drive slowly on the left side of the road' → {\"object_list\": [\"road\"], \"direction_constrain\": -0.5, \"velocity_constrain\": 0.3, \"traversability_constrain\": 1, ...}"
            ]
        else:
            prompt_parts = [
                "You are a behavioral instruction parser for a mobile robot navigation system.",
                "Parse the following natural language behavioral instructions into structured format.",
                "",
                "Instructions:",
                instructions,
                "",
                "Output format: JSON array of behavioral constraints with the following structure:",
                "[",
                "  {",
                "    \"object_list\": [\"object1\", \"object2\", ...],",
                "    \"direction_constrain\": <numeric_value_or_null>,  # -1.0 (left) to 1.0 (right), 0.0 (center), null (no constraint)",
                "    \"velocity_constrain\": <numeric_value_or_null>,  # 0.0 (stop), 0.1-0.3 (very slow), 0.3-0.5 (slow), 0.5 (normal), 0.5-0.7 (slightly fast), 0.7-1.0 (fast), null (no constraint)",
                "    \"traversability_constrain\": <integer_value>  # Object traversability: 0 (no constraint), 1 (traversable), 2 (non-traversable)",
                "  }",
                "]",
                "",
                "Rules:",
                "1. Extract behavioral landmarks from the instructions",
                "2. For direction constraints:",
                "   - Use -1.0 for strong left preference, 1.0 for strong right preference",
                "   - Use 0.0 for center preference",
                "   - Use intermediate values (e.g., -0.3 for slight left, 0.7 for slight right)",
                "   - Use null if no direction preference is specified",
                "3. For velocity constraints:",
                "   - Use 0.0 for stop (complete halt)",
                "   - Use 0.1-0.3 for very slow movement",
                "   - Use 0.3-0.5 for slow movement",
                "   - Use 0.5 for normal speed",
                "   - Use 0.5-0.7 for slightly fast movement",
                "   - Use 0.7-1.0 for fast movement",
                "   - Use null if no velocity preference is specified",
                "4. For object traversability constraints:",
                "   - Use 1 if the instruction explicitly states the object should be traversable",
                "   - Use 2 if the instruction explicitly states the object should not be traversable",
                "   - Use 0 if no explicit object traversability constraint is mentioned",
                "5. Online constraints have priority over offline constraints in case of conflicts",
                "6. If no explicit behavioral specification is provided, set the corresponding field to null",
                "7. A single behavioral constraint can be associated with multiple objects",
                "8. Output all text in English only",
                "9. Provide numeric values that reflect the semantic intensity of the instruction",
                "10. Object constraint assignment rules:",
                "    - When instructions explicitly mention objects, apply ALL behavioral constraints (direction, velocity, traversability) to that object",
                "    - Only use 'self' as object_list when instructions mention NO objects at all",
                "    - Examples:",
                "      * 'Go slowly' → {\"object_list\": [\"self\"], \"velocity_constrain\": 0.3, ...}",
                "      * 'Drive on the left side of the road' → {\"object_list\": [\"road\"], \"direction_constrain\": -0.5, \"traversability_constrain\": 1, ...}",
                "      * 'Drive slowly on the left side of the road' → {\"object_list\": [\"road\"], \"direction_constrain\": -0.5, \"velocity_constrain\": 0.3, \"traversability_constrain\": 1, ...}",
                "",
                "Output only the JSON array, no additional text:"
            ]
        
        return "\n".join(prompt_parts)
    
    def _parse_structured_output(self, llm_output: str) -> Tuple[List[BehavioralConstraint], Dict[str, str]]:
        """
        Parse LLM output into structured constraints with reasoning process.
        
        Args:
            llm_output: Raw LLM response
            
        Returns:
            Tuple[List[BehavioralConstraint], Dict[str, str]]: Parsed constraints and reasoning process
        """
        try:
            # Extract reasoning process and JSON
            reasoning_process = self._extract_reasoning_process(llm_output)
            cleaned_output = self._clean_llm_output(llm_output)
            
            # Debug: Log the cleaned output for troubleshooting
            self.logger.debug("Cleaned LLM output: %s", cleaned_output)
            
            constraints_data = json.loads(cleaned_output)
            
            constraints = []
            for constraint_data in constraints_data:
                # Handle both old string format and new list format
                object_list = constraint_data.get("object_list", [])
                if isinstance(object_list, str):
                    object_list = [object_list] if object_list else []
                elif not isinstance(object_list, list):
                    object_list = []
                
                # Handle direction constraint (numeric or null)
                direction_constrain = constraint_data.get("direction_constrain")
                if direction_constrain is None or direction_constrain == "null":
                    direction_constrain = self.default_values.get('direction_constrain')
                elif isinstance(direction_constrain, (int, float)):
                    direction_constrain = float(direction_constrain)
                else:
                    direction_constrain = self.default_values.get('direction_constrain')
                
                # Handle velocity constraint (numeric or null)
                velocity_constrain = constraint_data.get("velocity_constrain")
                if velocity_constrain is None or velocity_constrain == "null":
                    velocity_constrain = self.default_values.get('velocity_constrain')
                elif isinstance(velocity_constrain, (int, float)):
                    velocity_constrain = float(velocity_constrain)
                else:
                    velocity_constrain = self.default_values.get('velocity_constrain')
                
                # Handle traversability constraint with default value
                traversability_constrain = constraint_data.get("traversability_constrain")
                if traversability_constrain is None or traversability_constrain == "null":
                    traversability_constrain = self.default_values.get('traversability_constrain', 0)
                elif isinstance(traversability_constrain, str):
                    # Convert string to integer for backward compatibility
                    if traversability_constrain == "null":
                        traversability_constrain = 0
                    elif traversability_constrain == "traversable":
                        traversability_constrain = 1
                    elif traversability_constrain == "non-traversable":
                        traversability_constrain = 2
                    else:
                        traversability_constrain = 0
                
                constraint = BehavioralConstraint(
                    object_list=object_list,
                    direction_constrain=direction_constrain,
                    velocity_constrain=velocity_constrain,
                    traversability_constrain=traversability_constrain
                )
                constraints.append(constraint)
            
            return constraints, reasoning_process
            
        except (json.JSONDecodeError, KeyError, ValueError) as e:
            self.logger.error("Failed to parse LLM output: %s", e)
            self.logger.error("Raw LLM output that failed to parse: %s", llm_output)
            
            # Try to provide a fallback response with default constraints
            try:
                # Extract object names from the original instructions if possible
                fallback_constraints = []
                if hasattr(self, '_last_instruction_input'):
                    # Try to create a basic constraint from the instruction
                    instruction_text = " ".join(self._last_instruction_input.online_constraints + 
                                               self._last_instruction_input.offline_constraints)
                    
                    # Simple keyword extraction for objects (priority: objects first)
                    objects = []
                    text_lower = instruction_text.lower()
                    
                    # Extract explicit objects
                    if any(word in text_lower for word in ['road', 'street', 'path', 'way']):
                        objects.append('road')
                    if any(word in text_lower for word in ['obstacle', 'barrier', 'block']):
                        objects.append('obstacle')
                    
                    # Extract behavioral constraints
                    direction_constrain = self.default_values.get('direction_constrain')
                    velocity_constrain = self.default_values.get('velocity_constrain')
                    
                    if any(word in text_lower for word in ['left']):
                        direction_constrain = -0.5
                    elif any(word in text_lower for word in ['right']):
                        direction_constrain = 0.5
                    elif any(word in text_lower for word in ['center', 'middle']):
                        direction_constrain = 0.0
                    
                    if any(word in text_lower for word in ['slow', 'slowly']):
                        velocity_constrain = 0.2
                    elif any(word in text_lower for word in ['fast', 'quickly']):
                        velocity_constrain = 0.8
                    
                    # Object assignment logic: prioritize explicit objects, use 'self' if no objects but has constraints
                    if not objects:
                        has_direction_constraint = direction_constrain != self.default_values.get('direction_constrain')
                        has_velocity_constraint = velocity_constrain != self.default_values.get('velocity_constrain')
                        
                        if has_direction_constraint or has_velocity_constraint:
                            # Use 'self' when no objects mentioned but constraints exist
                            objects = ['self']
                        else:
                            # Fallback to default behavior for backward compatibility
                            objects = ['navigation_landmark']
                    
                    fallback_constraint = BehavioralConstraint(
                        object_list=objects,
                        direction_constrain=direction_constrain,
                        velocity_constrain=velocity_constrain,
                        traversability_constrain=self.default_values.get('traversability_constrain', 0)
                    )
                    fallback_constraints.append(fallback_constraint)
                    self.logger.debug("Created fallback constraint: %s", fallback_constraint)
                
                return fallback_constraints, {"error": f"JSON parsing failed: {str(e)}", "fallback_used": True}
            except Exception as fallback_error:
                self.logger.error("Fallback constraint creation also failed: %s", fallback_error)
                return [], {"error": f"Complete parsing failure: {str(e)}", "fallback_used": False}
    
    def _clean_llm_output(self, output: str) -> str:
        """
        Clean LLM output to extract JSON with robust error handling.
        
        Args:
            output: Raw LLM output
            
        Returns:
            str: Cleaned JSON string
        """
        # Debug: Log raw output
        self.logger.debug("Raw LLM output: %s", output)
        
        # Step 1: Remove markdown code blocks if present
        if "```json" in output:
            start = output.find("```json") + 7
            end = output.find("```", start)
            if end != -1:
                output = output[start:end]
        elif "```" in output:
            start = output.find("```") + 3
            end = output.find("```", start)
            if end != -1:
                output = output[start:end]
        
        # Step 2: Find JSON array boundaries with multiple strategies
        json_candidates = []
        
        # Strategy 1: Look for complete JSON array
        start_bracket = output.find("[")
        end_bracket = output.rfind("]")
        if start_bracket != -1 and end_bracket != -1 and end_bracket > start_bracket:
            candidate = output[start_bracket:end_bracket + 1]
            json_candidates.append(candidate)
        
        # Strategy 2: Look for JSON after "JSON:" marker
        json_marker = output.find("JSON:")
        if json_marker != -1:
            json_section = output[json_marker + 5:].strip()
            if json_section.startswith("["):
                end_bracket = json_section.rfind("]")
                if end_bracket != -1:
                    candidate = json_section[:end_bracket + 1]
                    json_candidates.append(candidate)
        
        # Strategy 3: Look for any valid JSON array in the text
        lines = output.split('\n')
        current_json = []
        in_json = False
        bracket_count = 0
        
        for line in lines:
            line = line.strip()
            if not line:
                continue
                
            # Skip analysis sections
            if line.startswith(('Analysis:', 'Reasoning:', 'Mapping:', 'JSON:')):
                if in_json and bracket_count == 0:
                    # We found a complete JSON block
                    json_candidates.append('\n'.join(current_json))
                    current_json = []
                    in_json = False
                continue
            
            # Check if this line starts JSON
            if line.startswith('[') and not in_json:
                in_json = True
                current_json = [line]
                bracket_count = line.count('[') - line.count(']')
            elif in_json:
                current_json.append(line)
                bracket_count += line.count('[') - line.count(']')
                
                # Check if we've completed the JSON
                if bracket_count == 0:
                    json_candidates.append('\n'.join(current_json))
                    current_json = []
                    in_json = False
        
        # Step 3: Try to parse each candidate and return the first valid one
        for candidate in json_candidates:
            try:
                # Test if it's valid JSON
                json.loads(candidate)
                self.logger.debug("Successfully extracted JSON: %s", candidate)
                return candidate.strip()
            except (json.JSONDecodeError, ValueError):
                continue
        
        # Step 4: If no valid JSON found, try to extract and fix common issues
        if json_candidates:
            # Try to fix the most likely candidate
            candidate = json_candidates[0]
            
            # Remove trailing commas
            candidate = candidate.replace(',]', ']').replace(',}', '}')
            
            # Try to fix incomplete JSON by adding missing brackets
            if candidate.count('[') > candidate.count(']'):
                candidate += ']' * (candidate.count('[') - candidate.count(']'))
            if candidate.count('{') > candidate.count('}'):
                candidate += '}' * (candidate.count('{') - candidate.count('}'))
            
            try:
                json.loads(candidate)
                self.logger.debug("Successfully fixed JSON: %s", candidate)
                return candidate.strip()
            except (json.JSONDecodeError, ValueError):
                pass
        
        # Step 5: If all else fails, return empty array
        self.logger.warning("Failed to extract valid JSON from LLM output, returning empty array")
        return "[]"
    
    def _extract_reasoning_process(self, output: str) -> Dict[str, str]:
        """
        Extract reasoning process from LLM output.
        
        Args:
            output: Raw LLM output
            
        Returns:
            Dict[str, str]: Extracted reasoning sections
        """
        reasoning = {
            "analysis": "",
            "reasoning": "",
            "mapping": ""
        }
        
        # Extract Analysis section
        analysis_match = self._extract_section(output, "Analysis:")
        if analysis_match:
            reasoning["analysis"] = analysis_match.strip()
        
        # Extract Reasoning section
        reasoning_match = self._extract_section(output, "Reasoning:")
        if reasoning_match:
            reasoning["reasoning"] = reasoning_match.strip()
        
        # Extract Mapping section
        mapping_match = self._extract_section(output, "Mapping:")
        if mapping_match:
            reasoning["mapping"] = mapping_match.strip()
        
        return reasoning
    
    def _extract_section(self, text: str, section_name: str) -> Optional[str]:
        """
        Extract a specific section from the text.
        
        Args:
            text: Full text to search
            section_name: Name of the section to extract
            
        Returns:
            Optional[str]: Extracted section content or None
        """
        lines = text.split('\n')
        section_start = None
        section_end = None
        
        # Find section start
        for i, line in enumerate(lines):
            if section_name in line:
                section_start = i
                break
        
        if section_start is None:
            return None
        
        # Find section end (next section or end of text)
        for i in range(section_start + 1, len(lines)):
            line = lines[i].strip()
            if line.startswith(("Analysis:", "Reasoning:", "Mapping:", "JSON:")) and line != section_name:
                section_end = i
                break
        
        if section_end is None:
            section_end = len(lines)
        
        # Extract section content
        section_lines = lines[section_start:section_end]
        return '\n'.join(section_lines)
    
    def _resolve_conflicts(self, constraints: List[BehavioralConstraint]) -> List[BehavioralConstraint]:
        """
        Resolve conflicts between constraints (online constraints have priority).
        
        Args:
            constraints: List of parsed constraints
            
        Returns:
            List[BehavioralConstraint]: Resolved constraints
        """
        # Group constraints by object
        object_constraints = {}
        for constraint in constraints:
            for obj in constraint.object_list:
                if obj not in object_constraints:
                    object_constraints[obj] = []
                object_constraints[obj].append(constraint)
        
        resolved = []
        processed_constraints = set()
        
        for obj, obj_constraints in object_constraints.items():
            if len(obj_constraints) == 1:
                constraint = obj_constraints[0]
                if id(constraint) not in processed_constraints:
                    resolved.append(constraint)
                    processed_constraints.add(id(constraint))
            else:
                # Resolve conflicts by prioritizing online constraints
                # For now, we'll take the last constraint as it represents the most recent
                # In a more sophisticated implementation, we could track constraint sources
                constraint = obj_constraints[-1]
                if id(constraint) not in processed_constraints:
                    resolved.append(constraint)
                    processed_constraints.add(id(constraint))
        
        return resolved
    
    def _create_fallback_constraints(self, instruction_input: InstructionInput) -> Tuple[List[BehavioralConstraint], Dict[str, str]]:
        """
        Create fallback constraints when LLM parsing fails.
        
        Args:
            instruction_input: Original instruction input
            
        Returns:
            Tuple[List[BehavioralConstraint], Dict[str, str]]: Fallback constraints and metadata
        """
        fallback_constraints = []
        instruction_text = " ".join(instruction_input.online_constraints + instruction_input.offline_constraints)
        
        # Simple keyword-based constraint creation
        objects = []
        direction_constrain = self.default_values.get('direction_constrain')
        velocity_constrain = self.default_values.get('velocity_constrain')
        traversability_constrain = self.default_values.get('traversability_constrain', 0)
        
        # Extract objects from instruction text (priority: objects first)
        text_lower = instruction_text.lower()
        if any(word in text_lower for word in ['road', 'street', 'path', 'way', '马路', '道路']):
            objects.append('road')
        if any(word in text_lower for word in ['obstacle', 'barrier', 'block', '障碍', '阻挡']):
            objects.append('obstacle')
        
        # Simple direction mapping
        if any(word in text_lower for word in ['left', '左']):
            direction_constrain = -0.5
        elif any(word in text_lower for word in ['right', '右']):
            direction_constrain = 0.5
        elif any(word in text_lower for word in ['center', 'middle', '中', '中间']):
            direction_constrain = 0.0
        
        # Simple velocity mapping
        if any(word in text_lower for word in ['slow', 'slowly', '慢', '缓慢']):
            velocity_constrain = 0.2
        elif any(word in text_lower for word in ['fast', 'quickly', '快', '快速']):
            velocity_constrain = 0.8
        else:
            velocity_constrain = 0.5  # Default normal speed
        
        # Object assignment logic: prioritize explicit objects, use 'self' if no objects but has constraints
        if not objects:
            # Check if we have any behavioral constraints (direction or velocity)
            has_direction_constraint = direction_constrain != self.default_values.get('direction_constrain')
            has_velocity_constraint = velocity_constrain != self.default_values.get('velocity_constrain')
            
            if has_direction_constraint or has_velocity_constraint:
                # Use 'self' when no objects mentioned but constraints exist
                objects = ['self']
            else:
                # Fallback to default behavior for backward compatibility
                objects = ['navigation_landmark']
        
        fallback_constraint = BehavioralConstraint(
            object_list=objects,
            direction_constrain=direction_constrain,
            velocity_constrain=velocity_constrain,
            traversability_constrain=traversability_constrain
        )
        fallback_constraints.append(fallback_constraint)
        
        reasoning_process = {
            "fallback_used": True,
            "fallback_reason": "LLM parsing failed, using keyword-based fallback",
            "extracted_objects": objects,
            "instruction_text": instruction_text
        }
        
        self.logger.debug("Created fallback constraint: %s", fallback_constraint)
        return fallback_constraints, reasoning_process
