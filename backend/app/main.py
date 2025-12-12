from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse, JSONResponse
import json
from typing import Optional, Dict, Any
from pydantic import BaseModel
from .config import settings
from .agent_simple import get_agent
from .agents import invoke_agent, list_available_agents, get_agent_info
from .auth_routes import router as auth_router
from .translation_routes import router as translation_router
from .personalization_routes import router as personalization_router

app = FastAPI(title="Physical AI RAG Chatbot with Agents SDK + Urdu Translation + Personalization")

# Include authentication routes
app.include_router(auth_router)

# Include translation routes
app.include_router(translation_router)

# Include personalization routes
app.include_router(personalization_router)

# CORS middleware - Allow all origins for development
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allow all origins in development
    allow_credentials=False,  # Must be False when allow_origins is "*"
    allow_methods=["*"],
    allow_headers=["*"],
)


class QueryRequest(BaseModel):
    message: str
    selected_text: Optional[str] = None
    session_id: Optional[str] = None


class AgentRequest(BaseModel):
    agent: str
    context: str
    user_profile: Optional[Dict[str, Any]] = None


@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {
        "status": "healthy",
        "model": settings.LITELLM_MODEL,
        "agent": "OpenAI Agents SDK + LiteLLM"
    }


@app.post("/query")
async def query_endpoint(request: QueryRequest):
    """
    Main RAG query endpoint with OpenAI Agents SDK.

    The agent autonomously decides when to search the textbook using the
    search_textbook tool, then synthesizes responses with proper citations.

    Returns:
        StreamingResponse: SSE stream with chunks, citations, and completion signals
    """
    try:
        # Get the agent singleton
        agent = get_agent()

        # Stream response using SSE format
        async def generate():
            try:
                # Run the agent and stream results
                async for event in agent.run(
                    user_message=request.message,
                    selected_text=request.selected_text,
                    session_id=request.session_id
                ):
                    # Convert event to SSE format: data: {json}\n\n
                    yield f"data: {json.dumps(event)}\n\n"

            except Exception as e:
                # Send error event
                yield f"data: {json.dumps({'type': 'error', 'message': str(e)})}\n\n"
                yield f"data: {json.dumps({'type': 'done'})}\n\n"

        return StreamingResponse(generate(), media_type="text/event-stream")

    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/api/agent")
async def agent_endpoint(request: AgentRequest):
    """
    Invoke a Claude Code agent with given context.

    Request body:
        {
            "agent": "HardwareAdvisor",
            "context": "What hardware do I need for Isaac Sim?",
            "user_profile": {...}  // optional
        }

    Response:
        {
            "result": "Agent's response...",
            "model": "groq/llama-3.3-70b-versatile",
            "cached": true,
            "agent": "HardwareAdvisor"
        }
    """
    try:
        result = await invoke_agent(
            agent_name=request.agent,
            context=request.context,
            user_profile=request.user_profile
        )

        if "error" in result:
            raise HTTPException(status_code=400, detail=result["error"])

        return JSONResponse(content=result)

    except Exception as e:
        print(f"[API Error] /api/agent failed: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/api/agents")
async def list_agents_endpoint():
    """
    List all available Claude Code agents.

    Response:
        {
            "agents": ["HardwareAdvisor", "LabGenerator", ...]
        }
    """
    try:
        agents = list_available_agents()
        return JSONResponse(content={"agents": agents})
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/api/agents/{agent_name}")
async def get_agent_info_endpoint(agent_name: str):
    """
    Get metadata about a specific agent.

    Response:
        {
            "name": "HardwareAdvisor",
            "description": "...",
            "model": "groq/llama-3.3-70b-versatile",
            "examples": [...]
        }
    """
    try:
        info = get_agent_info(agent_name)

        if not info:
            raise HTTPException(status_code=404, detail=f"Agent '{agent_name}' not found")

        return JSONResponse(content=info)
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/")
async def root():
    return {
        "message": "Physical AI RAG Chatbot API with OpenAI Agents SDK + Reusable Agents",
        "docs": "/docs",
        "agent": "Groq + LiteLLM + Agents SDK",
        "tools": ["search_textbook"],
        "agents": list_available_agents()
    }


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8003)
