"""
OpenAI Agents SDK Integration for RAG Chatbot
Uses official agents package with LitellmModel and Runner
"""

from typing import AsyncIterator, Dict, Any
from agents import Agent, Runner
from agents.extensions.models.litellm_model import LitellmModel
from .config import settings
from .tools import search_textbook


class RagChatbotAgent:
    """
    RAG Chatbot Agent using OpenAI Agents SDK.

    Features:
    - Agentic retrieval: Agent autonomously decides when to search textbook
    - Tool-calling: Uses @function_tool decorated search_textbook
    - LiteLLM integration: Routes to Groq llama-3.1-70b-instant
    - Streaming: Progressive response generation
    """

    def __init__(self):
        """Initialize the agent with LiteLLM model and search tool."""
        self.model = LitellmModel(
            model=settings.LITELLM_MODEL,
            api_key=settings.GROQ_API_KEY
        )

        self.agent = Agent(
            name="Physical AI Assistant",
            instructions="""You are an expert assistant for the Physical AI & Humanoid Robotics textbook.

Your knowledge comes from a comprehensive textbook covering:
- Physical AI concepts and embodied intelligence
- ROS 2 (Robot Operating System 2)
- Gazebo, Unity, and NVIDIA Isaac Sim simulation
- Vision-Language-Action (VLA) models
- Humanoid robotics and hardware integration

Guidelines:
1. Use the search_textbook tool to find relevant information when users ask questions
2. Always ground your answers in the textbook content returned by the tool
3. Cite specific chapters when providing information (e.g., "According to Chapter 3...")
4. If the user has selected text, consider it as additional context for your answer
5. Be clear, technical, and accurate
6. If information isn't found in the textbook, say so explicitly

Remember: You MUST use search_textbook before answering any factual questions about Physical AI topics.""",
            model=self.model,
            tools=[search_textbook],
        )

    async def run(
        self,
        user_message: str,
        selected_text: str | None = None,
        session_id: str | None = None
    ) -> AsyncIterator[Dict[str, Any]]:
        """
        Run the agent and stream responses.

        Args:
            user_message: User's question
            selected_text: Optional text selected by user for context
            session_id: Optional session ID for conversation history

        Yields:
            Dict with type: "chunk" | "tool_call" | "citations" | "done" | "error"
        """
        try:
            # Build the full query with selected text context if provided
            full_query = user_message
            if selected_text:
                full_query = f"""User selected this text for context:
---
{selected_text}
---

User's question: {user_message}

Please answer their question with consideration of the selected text."""

            print(f"[debug] Running agent with query: {user_message[:100]}...")

            # Run the agent using Runner
            # Note: Runner.run() handles tool execution automatically
            result = await Runner.run(self.agent, full_query)

            # Extract the final output
            final_output = result.final_output

            # Stream the response word by word to simulate progressive display
            if final_output:
                words = final_output.split()
                for word in words:
                    yield {
                        "type": "chunk",
                        "content": word + " "
                    }

            # Extract citations from the tool results if available
            citations = self._extract_citations_from_result(result)
            if citations:
                yield {
                    "type": "citations",
                    "data": citations
                }

            yield {"type": "done"}

        except Exception as e:
            print(f"[error] Agent run failed: {str(e)}")
            yield {
                "type": "error",
                "content": f"Agent error: {str(e)}"
            }
            yield {"type": "done"}

    def _extract_citations_from_result(self, result) -> list[Dict[str, Any]]:
        """
        Extract chapter citations from agent result.

        Parses the result to find chapter references mentioned in the response.
        """
        citations = []

        # Check if result has messages with tool calls
        if hasattr(result, 'messages'):
            for message in result.messages:
                if hasattr(message, 'tool_calls'):
                    for tool_call in message.tool_calls:
                        # Extract chapter info from search results
                        if hasattr(tool_call, 'result'):
                            result_text = str(tool_call.result)
                            # Parse chapter references from the formatted output
                            if "📚 Chapter References:" in result_text:
                                lines = result_text.split("\n")
                                in_citations = False
                                for line in lines:
                                    if "📚 Chapter References:" in line:
                                        in_citations = True
                                        continue
                                    if in_citations and line.strip().startswith("-"):
                                        # Extract chapter title from "- Chapter Title (relevance: X.XXX)"
                                        title_part = line.split("(relevance:")[0].strip("- ").strip()
                                        # Generate chapter_id from title
                                        # e.g., "Chapter 3: ROS 2 Fundamentals" -> "ch03"
                                        if title_part.startswith("Chapter "):
                                            ch_num = title_part.split(":")[0].replace("Chapter ", "").strip()
                                            chapter_id = f"ch{ch_num.zfill(2)}"
                                            citations.append({
                                                "chapter_id": chapter_id,
                                                "chapter_title": title_part
                                            })

        return citations


# Singleton instance
_agent_instance = None


def get_agent() -> RagChatbotAgent:
    """Get or create singleton agent instance."""
    global _agent_instance
    if _agent_instance is None:
        _agent_instance = RagChatbotAgent()
    return _agent_instance
