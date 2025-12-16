"""
Base Agent Framework for Claude Code Subagents

This module defines the abstract base class for all AI agents in the system.
"""

from abc import ABC, abstractmethod
from typing import Any, Dict, List, Optional
from pydantic import BaseModel
from dataclasses import dataclass


class AgentInput(BaseModel):
    """Base input model for agents."""
    pass


class AgentOutput(BaseModel):
    """Base output model for agents."""
    success: bool = True
    error: Optional[str] = None


@dataclass
class AgentTool:
    """Definition of a tool available to an agent."""
    name: str
    description: str
    parameters: Dict[str, Any]


class BaseAgent(ABC):
    """
    Abstract base class for all Claude Code Subagents.

    Each agent should:
    1. Define a unique name and description
    2. Implement the execute() method
    3. Define available tools via get_tools()
    4. Provide a system prompt via get_system_prompt()
    """

    name: str = "BaseAgent"
    description: str = "Base agent class"
    version: str = "1.0.0"

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        """
        Initialize the agent with optional configuration.

        Args:
            config: Optional configuration dictionary
        """
        self.config = config or {}
        self._initialize()

    def _initialize(self) -> None:
        """
        Initialize agent-specific resources.
        Override in subclasses for custom initialization.
        """
        pass

    @abstractmethod
    async def execute(self, input_data: AgentInput) -> AgentOutput:
        """
        Execute the agent's main task.

        Args:
            input_data: The input data for the agent

        Returns:
            AgentOutput with the results
        """
        pass

    def get_tools(self) -> List[AgentTool]:
        """
        Return list of tools this agent can use.
        Override in subclasses to define available tools.
        """
        return []

    def get_system_prompt(self) -> str:
        """
        Return the system prompt for this agent.
        Override in subclasses for custom prompts.
        """
        return f"You are {self.name}. {self.description}"

    def validate_input(self, input_data: AgentInput) -> bool:
        """
        Validate input data before execution.

        Args:
            input_data: The input to validate

        Returns:
            True if valid, raises exception otherwise
        """
        return True

    def __repr__(self) -> str:
        return f"<{self.name} v{self.version}>"
