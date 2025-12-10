"""
Lightweight Agent Implementation using LiteLLM Function Calling
Mimics the OpenAI Agents SDK pattern without external dependencies
"""

import json
import os
from typing import AsyncIterator, Dict, Any
from litellm import acompletion
from .config import settings
from .tools import search_textbook, RETRIEVAL_TOOL, execute_tool

# Set Groq API key as environment variable for LiteLLM
os.environ["GROQ_API_KEY"] = settings.GROQ_API_KEY


class SimpleRagAgent:
    """
    Lightweight RAG Agent using LiteLLM function calling.

    This provides the same functionality as the OpenAI Agents SDK pattern
    but uses direct LiteLLM calls with function calling support.
    """

    def __init__(self):
        """Initialize the agent with model and tools."""
        self.model = settings.LITELLM_MODEL
        self.tools = [RETRIEVAL_TOOL]
        self.system_prompt = """You are an expert assistant for the Physical AI & Humanoid Robotics textbook.

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

Remember: You MUST use search_textbook before answering any factual questions about Physical AI topics."""

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
            # Build messages
            messages = [
                {"role": "system", "content": self.system_prompt}
            ]

            # Add selected text context if provided
            if selected_text:
                messages.append({
                    "role": "system",
                    "content": f"User selected this text for context:\n---\n{selected_text}\n---"
                })

            messages.append({"role": "user", "content": user_message})

            print(f"[debug] Running agent with query: {user_message[:100]}...")

            # First pass: Check if agent wants to call tools
            initial_response = await acompletion(
                model=self.model,
                messages=messages,
                tools=self.tools,
                tool_choice="auto",
                temperature=0.7,
                max_tokens=500,
                api_key=settings.GROQ_API_KEY
            )

            assistant_message = initial_response.choices[0].message

            # Check if agent wants to call tools
            if hasattr(assistant_message, 'tool_calls') and assistant_message.tool_calls:
                yield {
                    "type": "tool_call",
                    "content": "Searching textbook...",
                    "tool": "search_textbook"
                }

                # Execute tool calls
                tool_results = []
                all_citations = []

                for tool_call in assistant_message.tool_calls:
                    function_name = tool_call.function.name
                    function_args = json.loads(tool_call.function.arguments)

                    print(f"[debug] Calling tool: {function_name} with args: {function_args}")

                    # Execute tool
                    result = execute_tool(function_name, function_args)
                    result_data = json.loads(result)

                    tool_results.append({
                        "role": "tool",
                        "tool_call_id": tool_call.id,
                        "name": function_name,
                        "content": result
                    })

                    # Extract citations
                    if result_data.get("success") and result_data.get("citations"):
                        all_citations.extend(result_data["citations"])

                # Add tool results to conversation
                messages.append({
                    "role": "assistant",
                    "content": assistant_message.content,
                    "tool_calls": [
                        {
                            "id": tc.id,
                            "type": "function",
                            "function": {
                                "name": tc.function.name,
                                "arguments": tc.function.arguments
                            }
                        }
                        for tc in assistant_message.tool_calls
                    ]
                })
                messages.extend(tool_results)

                # Second pass: Generate final response with tool results
                final_response = await acompletion(
                    model=self.model,
                    messages=messages,
                    temperature=0.7,
                    max_tokens=500,
                    stream=True,
                    api_key=settings.GROQ_API_KEY
                )

                # Stream the final response
                async for chunk in final_response:
                    if hasattr(chunk.choices[0].delta, 'content'):
                        content = chunk.choices[0].delta.content
                        if content:
                            yield {
                                "type": "chunk",
                                "content": content
                            }

                # Send citations after response completes
                if all_citations:
                    # Deduplicate citations by chapter_id
                    unique_citations = {
                        cite["chapter_id"]: cite
                        for cite in all_citations
                    }
                    yield {
                        "type": "citations",
                        "data": list(unique_citations.values())
                    }

            else:
                # No tool calls needed - stream direct response
                stream_response = await acompletion(
                    model=self.model,
                    messages=messages,
                    temperature=0.7,
                    max_tokens=500,
                    stream=True,
                    api_key=settings.GROQ_API_KEY
                )

                async for chunk in stream_response:
                    if hasattr(chunk.choices[0].delta, 'content'):
                        content = chunk.choices[0].delta.content
                        if content:
                            yield {
                                "type": "chunk",
                                "content": content
                            }

            # Signal completion
            yield {"type": "done"}

        except Exception as e:
            print(f"[error] Agent run failed: {str(e)}")
            import traceback
            traceback.print_exc()
            yield {
                "type": "error",
                "content": f"Agent error: {str(e)}"
            }
            yield {"type": "done"}


# Singleton instance
_agent_instance = None


def get_agent() -> SimpleRagAgent:
    """Get or create singleton agent instance."""
    global _agent_instance
    if _agent_instance is None:
        _agent_instance = SimpleRagAgent()
    return _agent_instance
