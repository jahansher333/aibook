"""
Test script for Claude Code reusable agents system.
"""
import asyncio
import sys
from pathlib import Path

# Add app to path
sys.path.insert(0, str(Path(__file__).parent))

from app.agents import invoke_agent, list_available_agents, get_agent_info


async def test_list_agents():
    """Test listing all available agents"""
    print("\n=== Test 1: List Available Agents ===")
    agents = list_available_agents()
    print(f"Found {len(agents)} agents:")
    for agent in agents:
        print(f"  - {agent}")
    return agents


async def test_agent_info(agent_name: str):
    """Test getting agent metadata"""
    print(f"\n=== Test 2: Get Agent Info ({agent_name}) ===")
    info = get_agent_info(agent_name)
    if info:
        print(f"Name: {info['name']}")
        print(f"Description: {info['description']}")
        print(f"Model: {info['model']}")
        print(f"Examples: {len(info.get('examples', []))} examples")
    else:
        print(f"Agent '{agent_name}' not found!")
    return info


async def test_invoke_agent(agent_name: str, context: str):
    """Test invoking an agent"""
    print(f"\n=== Test 3: Invoke Agent ({agent_name}) ===")
    print(f"Context: {context[:100]}...")
    print("\nCalling agent...")

    result = await invoke_agent(agent_name, context)

    if "error" in result:
        print(f"\n[ERROR] {result['error']}")
        if "available_agents" in result:
            print(f"Available agents: {result['available_agents']}")
        return None

    print(f"\n[OK] SUCCESS")
    print(f"Model used: {result['model']}")
    print(f"Cached: {result['cached']}")
    print(f"\nResponse:\n{'-' * 60}")
    print(result['result'][:500])
    if len(result['result']) > 500:
        print(f"\n... (truncated, full response is {len(result['result'])} characters)")
    print(f"{'-' * 60}")

    return result


async def test_hardware_advisor():
    """Test HardwareAdvisor agent with realistic context"""
    print("\n" + "=" * 80)
    print("HARDWARE ADVISOR TEST")
    print("=" * 80)

    context = """
I'm a student taking the Physical AI course and want to run Isaac Sim for Chapter 6.
My hardware:
- CPU: Intel i5-11400
- RAM: 16GB
- GPU: None (integrated graphics only)
- Budget: $300-500 for upgrades

What are my options for running Isaac Sim? Should I buy a GPU or use cloud?
"""

    result = await test_invoke_agent("HardwareAdvisor", context)
    return result is not None


async def test_urdu_translator():
    """Test UrduTranslator agent"""
    print("\n" + "=" * 80)
    print("URDU TRANSLATOR TEST")
    print("=" * 80)

    context = """
Translate this to Urdu:

## Introduction to Physical AI

Physical AI represents the next frontier in robotics and artificial intelligence.
It combines computer vision, natural language processing, and robotic control
to create intelligent systems that can interact with the physical world.

ROS 2 (Robot Operating System 2) is a powerful framework for building robotic applications.
"""

    result = await test_invoke_agent("UrduTranslator", context)
    return result is not None


async def test_lab_generator():
    """Test LabGenerator agent"""
    print("\n" + "=" * 80)
    print("LAB GENERATOR TEST")
    print("=" * 80)

    context = """
Create a hands-on lab for Chapter 2: ROS 2 Fundamentals.
Topic: Publisher and Subscriber nodes.
Students should create two nodes that communicate via a topic.
Include verification steps and common errors.
"""

    result = await test_invoke_agent("LabGenerator", context)
    return result is not None


async def main():
    """Run all tests"""
    print("\n" + "=" * 80)
    print("CLAUDE CODE REUSABLE AGENTS - TEST SUITE")
    print("=" * 80)

    try:
        # Test 1: List agents
        agents = await test_list_agents()

        if not agents:
            print("\n❌ FATAL: No agents found! Check .claude/agents/ directory.")
            return False

        # Test 2: Get agent info
        test_agent = agents[0] if agents else "HardwareAdvisor"
        info = await test_agent_info(test_agent)

        if not info:
            print(f"\n❌ FATAL: Could not load agent info for {test_agent}")
            return False

        # Test 3: Invoke HardwareAdvisor
        success_hardware = await test_hardware_advisor()

        # Test 4: Invoke UrduTranslator
        success_urdu = await test_urdu_translator()

        # Test 5: Invoke LabGenerator
        success_lab = await test_lab_generator()

        # Summary
        print("\n" + "=" * 80)
        print("TEST SUMMARY")
        print("=" * 80)
        print(f"[OK] Agents found: {len(agents)}")
        print(f"[{'OK' if success_hardware else 'FAIL'}] HardwareAdvisor test")
        print(f"[{'OK' if success_urdu else 'FAIL'}] UrduTranslator test")
        print(f"[{'OK' if success_lab else 'FAIL'}] LabGenerator test")

        all_passed = success_hardware and success_urdu and success_lab

        if all_passed:
            print("\n>>> ALL TESTS PASSED!")
        else:
            print("\n>>> SOME TESTS FAILED")

        return all_passed

    except KeyboardInterrupt:
        print("\n\n[INTERRUPTED] Tests interrupted by user")
        return False
    except Exception as e:
        print(f"\n\n[ERROR] FATAL ERROR: {e}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    success = asyncio.run(main())
    sys.exit(0 if success else 1)
