"""
RAG Chatbot Tools for LiteLLM Function Calling
Provides retrieval tools for semantic search in Physical AI textbook
"""

import json
from typing import List, Dict, Any
from qdrant_client import QdrantClient
import cohere
from .config import settings


# Initialize clients globally
qdrant = QdrantClient(
    url=settings.QDRANT_URL,
    api_key=settings.QDRANT_API_KEY,
)

co = cohere.Client(settings.COHERE_API_KEY)


def search_textbook(query: str, limit: int = 5) -> str:
    """
    Search the Physical AI & Humanoid Robotics textbook for relevant information.

    Use this tool when the user asks questions about:
    - Physical AI concepts and definitions
    - ROS 2 (Robot Operating System)
    - Gazebo, Unity, or Isaac Sim simulation
    - Vision-Language-Action (VLA) models
    - Humanoid robotics and hardware
    - Any robotics-related topics

    Args:
        query: The search query or question to find in the textbook
        limit: Maximum number of relevant chunks to return (default: 5, max: 10)

    Returns:
        A formatted string with relevant textbook content and chapter citations
    """
    try:
        print(f"[debug] Searching textbook for: {query}")

        # Generate embedding for query
        embed_response = co.embed(
            texts=[query],
            model="embed-english-v3.0",
            input_type="search_query",
            embedding_types=["float"]
        )
        query_embedding = embed_response.embeddings.float[0]

        # Search Qdrant
        search_results = qdrant.query_points(
            collection_name=settings.QDRANT_COLLECTION,
            query=query_embedding,
            limit=min(limit, 10),  # Cap at 10
            score_threshold=0.6,
            with_payload=True
        ).points

        if not search_results:
            return "No relevant information found in the textbook for this query."

        # Format results as readable text
        output_lines = ["Found relevant information from the textbook:\n"]
        citations = []

        for i, result in enumerate(search_results, 1):
            chunk_text = result.payload.get("content", "")
            chapter_id = result.payload.get("chapter_id", "unknown")
            chapter_title = result.payload.get("chapter_title", "Unknown Chapter")
            similarity = round(result.score, 3)

            output_lines.append(f"\n[Source {i}] From {chapter_title} (similarity: {similarity})")
            output_lines.append(chunk_text)
            output_lines.append("")

            # Track unique citations
            if chapter_id not in [c["chapter_id"] for c in citations]:
                citations.append({
                    "chapter_id": chapter_id,
                    "chapter_title": chapter_title,
                    "similarity": similarity
                })

        # Add citations summary
        output_lines.append("\n📚 Chapter References:")
        for cite in citations:
            output_lines.append(f"- {cite['chapter_title']} (relevance: {cite['similarity']})")

        return "\n".join(output_lines)

    except Exception as e:
        print(f"[error] Search failed: {str(e)}")
        return f"Error searching textbook: {str(e)}"


# Tool definition for LiteLLM function calling
RETRIEVAL_TOOL = {
    "type": "function",
    "function": {
        "name": "search_textbook",
        "description": (
            "Search the Physical AI & Humanoid Robotics textbook for relevant information. "
            "Use this tool when the user asks questions about Physical AI concepts, robotics, "
            "ROS 2, simulation, hardware, or any topic covered in the textbook. "
            "The tool returns relevant text chunks with citations to specific chapters."
        ),
        "parameters": {
            "type": "object",
            "properties": {
                "query": {
                    "type": "string",
                    "description": "The search query. Should be a clear question or topic related to Physical AI."
                },
                "limit": {
                    "type": "integer",
                    "description": "Maximum number of results to return (default: 5, max: 10)",
                    "default": 5
                }
            },
            "required": ["query"]
        }
    }
}


# Map function names to actual functions
TOOL_FUNCTIONS = {
    "search_textbook": search_textbook
}


def execute_tool(tool_name: str, arguments: Dict[str, Any]) -> str:
    """
    Execute a tool by name with given arguments.

    Args:
        tool_name: Name of the tool to execute
        arguments: Dictionary of arguments to pass to the tool

    Returns:
        Tool execution result as JSON string
    """
    if tool_name not in TOOL_FUNCTIONS:
        return json.dumps({
            "success": False,
            "error": f"Unknown tool: {tool_name}"
        })

    try:
        func = TOOL_FUNCTIONS[tool_name]
        result_text = func(**arguments)

        # Parse citations from result text
        citations = []
        if "📚 Chapter References:" in result_text:
            lines = result_text.split("\n")
            in_citations = False
            for line in lines:
                if "📚 Chapter References:" in line:
                    in_citations = True
                    continue
                if in_citations and line.strip().startswith("-"):
                    # Extract chapter info
                    title_part = line.split("(relevance:")[0].strip("- ").strip()
                    # Generate chapter_id from title if it contains "Chapter X"
                    if "Chapter " in title_part:
                        ch_num_str = title_part.split("Chapter ")[1].split(":")[0].strip()
                        try:
                            ch_num = int(ch_num_str)
                            chapter_id = f"ch{ch_num:02d}"
                            citations.append({
                                "chapter_id": chapter_id,
                                "chapter_title": title_part
                            })
                        except:
                            pass

        return json.dumps({
            "success": True,
            "result": result_text,
            "citations": citations
        })
    except Exception as e:
        return json.dumps({
            "success": False,
            "error": f"Tool execution error: {str(e)}"
        })
