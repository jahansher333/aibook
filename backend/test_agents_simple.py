"""
Simple test for agent system - no API calls, just loading and listing.
"""
import asyncio
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from app.agents import list_available_agents, get_agent_info, load_agent_config


async def main():
    print("=" * 80)
    print("AGENT SYSTEM SIMPLE TEST")
    print("=" * 80)

    # Test 1: List agents
    print("\n[Test 1] Listing available agents...")
    agents = list_available_agents()
    print(f"Found {len(agents)} agents:")
    for agent in agents:
        print(f"  - {agent}")

    if len(agents) == 0:
        print("[FAIL] No agents found!")
        return False

    print(f"[OK] Found {len(agents)} agents")

    # Test 2: Load each agent config
    print("\n[Test 2] Loading agent configurations...")
    for agent_name in agents:
        config = load_agent_config(agent_name)
        if config:
            print(f"  [OK] {agent_name}: {len(config.get('system', ''))} char system prompt")
        else:
            print(f"  [FAIL] {agent_name}: Failed to load")
            return False

    print("[OK] All agents loaded successfully")

    # Test 3: Get agent info
    print("\n[Test 3] Getting agent metadata...")
    for agent_name in agents[:3]:  # Test first 3
        info = get_agent_info(agent_name)
        if info:
            print(f"  [OK] {agent_name}:")
            print(f"       Description: {info['description'][:60]}...")
            print(f"       Model: {info['model']}")
        else:
            print(f"  [FAIL] {agent_name}: No info available")
            return False

    print("[OK] Agent metadata retrieved successfully")

    # Summary
    print("\n" + "=" * 80)
    print("TEST SUMMARY")
    print("=" * 80)
    print(f"[OK] {len(agents)} agents found and loaded")
    print(f"[OK] All configurations valid")
    print(f"[OK] Metadata accessible")
    print("\n>>> ALL TESTS PASSED")

    return True


if __name__ == "__main__":
    success = asyncio.run(main())
    sys.exit(0 if success else 1)
