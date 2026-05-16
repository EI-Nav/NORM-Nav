#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LLM interface for behavioral instruction parsing.

This module provides interfaces to various Large Language Models for
processing natural language behavioral instructions.

Author: Wang Junhui <wjh_9696@163.com>
License: MIT
"""

from abc import ABC, abstractmethod
from typing import Dict, Any, Optional
import logging
import os


class LLMInterface(ABC):
    """Abstract base class for LLM interfaces."""
    
    def __init__(self, config: Optional[Dict[str, Any]] = None):
        """
        Initialize LLM interface.
        
        Args:
            config: Configuration parameters for the LLM
        """
        self.config = config or {}
        self.logger = logging.getLogger(__name__)
    
    @abstractmethod
    def process_instruction(self, prompt: str) -> str:
        """
        Process a behavioral instruction prompt.
        
        Args:
            prompt: The instruction prompt to process
            
        Returns:
            str: LLM response
        """
        raise NotImplementedError("Subclasses must implement process_instruction")


class OpenAIInterface(LLMInterface):
    """OpenAI API interface for LLM processing."""
    
    def __init__(self, config: Optional[Dict[str, Any]] = None):
        """
        Initialize OpenAI interface.
        
        Args:
            config: Configuration with API key and model settings
        """
        super().__init__(config)
        
        try:
            import openai
            self.openai = openai
        except ImportError as exc:
            raise ImportError("OpenAI package not installed. Install with: pip install openai") from exc
        
        # Set API key from config or environment
        api_key = (
            self.config.get("api_key")
            or os.getenv("LLM_API_KEY")
            or os.getenv("OPENAI_API_KEY")
        )
        if not api_key:
            raise ValueError(
                "LLM API key not provided. Set LLM_API_KEY (or OPENAI_API_KEY), "
                "or pass llm.api_key via ROS parameters."
            )
        
        # Get base URL from config
        base_url = self.config.get("base_url", "https://api.openai.com/v1")
        
        self.client = self.openai.OpenAI(
            api_key=api_key,
            base_url=base_url
        )
        self.model = self.config.get("model", "gpt-3.5-turbo")
        self.max_tokens = self.config.get("max_tokens", 1000)
        self.temperature = self.config.get("temperature", 0.1)
    
    def process_instruction(self, prompt: str) -> str:
        """
        Process instruction using OpenAI API.
        
        Args:
            prompt: The instruction prompt to process
            
        Returns:
            str: LLM response
        """
        try:
            self.logger.debug("Processing OpenAI request with model: %s", self.model)
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": "You are a behavioral instruction parser for mobile robot navigation."},
                    {"role": "user", "content": prompt}
                ],
                max_tokens=self.max_tokens,
                temperature=self.temperature
            )
            
            self.logger.debug("OpenAI API request completed successfully")
            return response.choices[0].message.content
            
        except self.openai.APITimeoutError as e:
            self.logger.error("OpenAI API timeout error: %s", e)
            raise TimeoutError(f"OpenAI API request timed out: {e}") from e
            
        except self.openai.APIConnectionError as e:
            self.logger.error("OpenAI API connection error: %s", e)
            raise ConnectionError(f"Failed to connect to OpenAI API: {e}") from e
            
        except self.openai.RateLimitError as e:
            self.logger.error("OpenAI API rate limit error: %s", e)
            raise RuntimeError(f"OpenAI API rate limit exceeded: {e}") from e
            
        except self.openai.AuthenticationError as e:
            self.logger.error("OpenAI API authentication error: %s", e)
            raise ValueError(f"OpenAI API authentication failed: {e}") from e
            
        except self.openai.BadRequestError as e:
            self.logger.error("OpenAI API bad request error: %s", e)
            raise ValueError(f"Invalid request to OpenAI API: {e}") from e
            
        except Exception as e:
            self.logger.error("Unexpected OpenAI API error: %s", e)
            raise


class LocalLLMInterface(LLMInterface):
    """Interface for local LLM models (e.g., Ollama, local transformers)."""
    
    def __init__(self, config: Optional[Dict[str, Any]] = None):
        """
        Initialize local LLM interface.
        
        Args:
            config: Configuration with model settings
        """
        super().__init__(config)
        self.model = self.config.get("model", "llama2")
        self.base_url = self.config.get("base_url", "http://localhost:11434")
        self.max_tokens = self.config.get("max_tokens", 1000)
        self.temperature = self.config.get("temperature", 0.1)
    
    def process_instruction(self, prompt: str) -> str:
        """
        Process instruction using local LLM.
        
        Args:
            prompt: The instruction prompt to process
            
        Returns:
            str: LLM response
        """
        try:
            import requests
            
            self.logger.debug("Processing local LLM request with model: %s", self.model)
            response = requests.post(
                f"{self.base_url}/api/generate",
                json={
                    "model": self.model,
                    "prompt": prompt,
                    "stream": False,
                    "options": {
                        "num_predict": self.max_tokens,
                        "temperature": self.temperature
                    }
                },
                timeout=30
            )
            
            if response.status_code == 200:
                self.logger.debug("Local LLM request completed successfully")
                return response.json()["response"]
            elif response.status_code == 404:
                raise ConnectionError(f"Local LLM service not found at {self.base_url}")
            elif response.status_code == 500:
                raise RuntimeError(f"Local LLM server error: {response.status_code}")
            else:
                raise RuntimeError(f"Local LLM API error: {response.status_code}")
                
        except ImportError as exc:
            raise ImportError("Requests package not installed. Install with: pip install requests") from exc
            
        except requests.exceptions.ConnectionError as e:
            self.logger.error("Local LLM connection error: %s", e)
            raise ConnectionError(f"Failed to connect to local LLM at {self.base_url}: {e}") from e
            
        except requests.exceptions.Timeout as e:
            self.logger.error("Local LLM timeout error: %s", e)
            raise TimeoutError(f"Local LLM request timed out: {e}") from e
            
        except requests.exceptions.RequestException as e:
            self.logger.error("Local LLM request error: %s", e)
            raise RuntimeError(f"Local LLM request failed: {e}") from e
            
        except Exception as e:
            self.logger.error("Unexpected local LLM error: %s", e)
            raise


def create_llm_interface(interface_type: str, config: Optional[Dict[str, Any]] = None) -> LLMInterface:
    """
    Factory function to create LLM interface instances.
    
    Args:
        interface_type: Type of interface to create ("openai", "local")
        config: Configuration parameters
        
    Returns:
        LLMInterface: Configured LLM interface instance
    """
    if interface_type.lower() == "openai":
        return OpenAIInterface(config)
    elif interface_type.lower() == "local":
        return LocalLLMInterface(config)
    else:
        raise ValueError(f"Unknown LLM interface type: {interface_type}")
