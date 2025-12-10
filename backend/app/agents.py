"""
Agent system for invoking reusable Claude Code agents defined in .claude/agents/
"""
import os
import yaml
from pathlib import Path
from typing import Dict, Any, Optional
from litellm import completion
from .config import settings

# Path to agent definitions
AGENTS_DIR = Path(__file__).parent.parent.parent / ".claude" / "agents"

# Cache for loaded agent configs
_agent_cache: Dict[str, Dict[str, Any]] = {}


def load_agent_config(agent_name: str) -> Optional[Dict[str, Any]]:
    """
    Load agent configuration from YAML file.

    Args:
        agent_name: Name of the agent (e.g., "HardwareAdvisor")

    Returns:
        Agent configuration dict or None if not found
    """
    # Check cache first
    if agent_name in _agent_cache:
        return _agent_cache[agent_name]

    # Build filename (convert CamelCase to kebab-case)
    # HardwareAdvisor -> hardware-advisor.yaml
    filename = "".join([
        c if c.islower() or c.isdigit() else f"-{c.lower()}"
        for c in agent_name
    ]).lstrip("-") + ".yaml"

    agent_file = AGENTS_DIR / filename

    if not agent_file.exists():
        print(f"[agents] Agent file not found: {agent_file}")
        return None

    try:
        with open(agent_file, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)

        # Validate required fields
        if not config.get('name'):
            print(f"[agents] Missing 'name' in {filename}")
            return None

        if not config.get('system'):
            print(f"[agents] Missing 'system' prompt in {filename}")
            return None

        # Cache the config
        _agent_cache[agent_name] = config
        print(f"[agents] Loaded agent: {agent_name} from {filename}")

        return config

    except Exception as e:
        print(f"[agents] Error loading {filename}: {e}")
        return None


async def invoke_agent(
    agent_name: str,
    context: str,
    user_profile: Optional[Dict[str, Any]] = None
) -> Dict[str, Any]:
    """
    Invoke a Claude Code agent with given context.

    Args:
        agent_name: Name of the agent to invoke
        context: User's context/question for the agent
        user_profile: Optional user profile data (hardware, experience, etc.)

    Returns:
        Dict with:
            - result: Agent's response (string)
            - model: Model used
            - cached: Whether config was cached
            - error: Error message if failed (optional)
    """
    try:
        # Load agent configuration
        config = load_agent_config(agent_name)

        if not config:
            return {
                "error": f"Agent '{agent_name}' not found",
                "available_agents": list_available_agents()
            }

        # Build messages
        messages = [
            {
                "role": "system",
                "content": config['system']
            }
        ]

        # Add user profile context if provided
        if user_profile:
            profile_context = f"""
User Profile:
- Hardware GPU: {user_profile.get('hardware_gpu', 'unknown')}
- Hardware Jetson: {user_profile.get('hardware_jetson', 'unknown')}
- Hardware Camera: {user_profile.get('hardware_camera', 'unknown')}
- Experience Level: {user_profile.get('experience_level', 'unknown')}
- Preferred Simulator: {user_profile.get('preferred_simulator', 'unknown')}
- Learning Pace: {user_profile.get('learning_pace', 'unknown')}
"""
            messages.append({
                "role": "system",
                "content": profile_context
            })

        # Add user context
        messages.append({
            "role": "user",
            "content": context
        })

        # Get model (use config model or default to Groq)
        model = config.get('model', settings.LITELLM_MODEL)

        print(f"[agents] Invoking {agent_name} with model {model}")

        # Call LiteLLM
        response = await completion(
            model=model,
            messages=messages,
            temperature=config.get('temperature', 0.7),
            max_tokens=config.get('max_tokens', 1500),
            api_key=settings.GROQ_API_KEY
        )

        result = response.choices[0].message.content

        return {
            "result": result,
            "model": model,
            "cached": agent_name in _agent_cache,
            "agent": agent_name
        }

    except Exception as e:
        print(f"[agents] Error invoking {agent_name}: {e}")
        return {
            "error": str(e),
            "agent": agent_name
        }


def list_available_agents() -> list[str]:
    """
    List all available agents in .claude/agents/ directory.

    Returns:
        List of agent names (without .yaml extension)
    """
    if not AGENTS_DIR.exists():
        return []

    agents = []
    for file in AGENTS_DIR.glob("*.yaml"):
        # Convert filename to agent name
        # hardware-advisor.yaml -> HardwareAdvisor
        name_parts = file.stem.split('-')
        agent_name = ''.join(word.capitalize() for word in name_parts)
        agents.append(agent_name)

    return sorted(agents)


def get_agent_info(agent_name: str) -> Optional[Dict[str, Any]]:
    """
    Get agent metadata without invoking it.

    Args:
        agent_name: Name of the agent

    Returns:
        Dict with agent metadata or None
    """
    config = load_agent_config(agent_name)

    if not config:
        return None

    return {
        "name": config.get('name'),
        "description": config.get('description', 'No description available'),
        "model": config.get('model', settings.LITELLM_MODEL),
        "examples": config.get('examples', [])
    }


# Preload all agents on module import
def preload_agents():
    """Preload all agent configs into cache."""
    print("[agents] Preloading agent configurations...")
    for agent_name in list_available_agents():
        load_agent_config(agent_name)
    print(f"[agents] Preloaded {len(_agent_cache)} agents")


# Auto-preload on import
if os.getenv("PRELOAD_AGENTS", "true").lower() == "true":
    preload_agents()
