# Reusable Claude Code Agents for Physical AI Textbook

This directory contains 5 specialized Claude Code agents that enhance the Physical AI textbook learning experience.

## Available Agents

### 1. HardwareAdvisor
**Purpose**: Adapts content based on user's available hardware
**Use Cases**:
- Student has no RTX GPU → Recommends cloud alternatives
- Student has Jetson → Emphasizes edge deployment
- Student wants to run Isaac Sim → Provides hardware requirements + alternatives

**Example Usage**:
```markdown
<!-- In chapter MDX -->
<Agent name="HardwareAdvisor" context="Isaac Sim requirements for Chapter 6" />
```

---

### 2. LabGenerator
**Purpose**: Creates hands-on lab exercises for each topic
**Use Cases**:
- Generate ROS 2 publisher/subscriber lab with complete code
- Create Gazebo world file exercise
- Design Unity ML-Agents training scenario

**Example Usage**:
```markdown
<!-- In chapter MDX -->
<Agent name="LabGenerator" context="ROS 2 nodes and topics" />
```

**Output**: Complete lab with:
- Learning objectives
- Step-by-step instructions
- Verification commands
- Troubleshooting tips
- Extension challenges

---

### 3. UrduTranslator
**Purpose**: Translates content to Urdu using Groq + LiteLLM
**Use Cases**:
- One-click chapter translation
- Preserves code blocks and technical terms
- Cached in Qdrant for fast retrieval

**Example Usage**:
```markdown
<!-- In chapter MDX -->
<Agent name="UrduTranslator" context="Chapter 2: ROS 2 Fundamentals" />
```

**Features**:
- Technical terms kept in English with Urdu explanations
- Markdown formatting preserved
- Fast translation using Groq (<5s per chapter)
- Results cached to avoid re-translation

---

### 4. QuizMaster
**Purpose**: Generates adaptive quizzes and assessments
**Use Cases**:
- Create chapter quiz (5 questions, 10 minutes)
- Adaptive difficulty based on performance
- Immediate feedback with explanations

**Example Usage**:
```markdown
<!-- In chapter MDX -->
<Agent name="QuizMaster" context="Chapter 2: ROS 2 Fundamentals" />
```

**Question Types**:
- Multiple choice (knowledge check)
- Code comprehension (what does this do?)
- Scenario-based (how would you solve...?)
- Debugging (find and fix the error)

---

### 5. CapstonePlanner
**Purpose**: Guides capstone project planning and execution
**Use Cases**:
- Scope capstone project based on student's hardware/experience
- Generate week-by-week milestones
- Provide starter code templates

**Example Usage**:
```markdown
<!-- In Chapter 13 MDX -->
<Agent name="CapstonePlanner" context="Voice-controlled humanoid gestures project" />
```

**Output**:
- Project scope definition
- 4-6 week milestone plan
- Architecture diagram
- Success criteria checklist
- Starter code repository

---

## Bonus Agent: CodeReviewer
**Purpose**: Reviews student code for best practices and safety
**Use Cases**:
- Review ROS 2 node implementation
- Check robot control code for safety issues
- Validate URDF/world files

**Example Usage**:
```markdown
<!-- In lab submission -->
<Agent name="CodeReviewer" context="Review my ROS 2 publisher code" />
```

---

## Agent Integration in Docusaurus

### Option 1: MDX Components (Automatic Invocation)

Create `docusaurus/src/components/Agent.tsx`:

```typescript
import React, { useState } from 'react';

interface AgentProps {
  name: string;
  context?: string;
}

export default function Agent({ name, context }: AgentProps) {
  const [response, setResponse] = useState<string>('');
  const [loading, setLoading] = useState(false);

  const invokeAgent = async () => {
    setLoading(true);
    try {
      const res = await fetch('/api/agent', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ agent: name, context })
      });
      const data = await res.json();
      setResponse(data.result);
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="agent-container">
      <button onClick={invokeAgent} className="agent-button">
        {loading ? 'Loading...' : `Ask ${name}`}
      </button>
      {response && (
        <div className="agent-response" dangerouslySetInnerHTML={{ __html: response }} />
      )}
    </div>
  );
}
```

### Option 2: Button Integration (User-Initiated)

Add buttons to each chapter:

```markdown
## Chapter 2: ROS 2 Fundamentals

[Chapter content...]

---

### Need Help?

<div className="agent-actions">
  <Agent name="ConceptExplainer" context="ROS 2 topics and nodes" />
  <Agent name="LabGenerator" context="Create ROS 2 publisher lab" />
  <Agent name="UrduTranslator" context="Translate this chapter to Urdu" />
</div>
```

---

## Backend API for Agents

Create `backend/app/agents.py`:

```python
from litellm import completion
import os

AGENT_CONFIGS = {
    "HardwareAdvisor": {
        "model": "groq/llama-3.3-70b-versatile",
        "system": "... (load from .claude/agents/hardware-advisor.yaml)"
    },
    # ... other agents
}

async def invoke_agent(agent_name: str, context: str, user_profile: dict = None):
    """Invoke a Claude Code agent with context"""
    config = AGENT_CONFIGS.get(agent_name)
    if not config:
        return {"error": f"Agent {agent_name} not found"}

    messages = [
        {"role": "system", "content": config["system"]},
        {"role": "user", "content": context}
    ]

    # Add user hardware profile if available
    if user_profile and agent_name == "HardwareAdvisor":
        messages.insert(1, {
            "role": "system",
            "content": f"User Hardware: {user_profile['hardware']}"
        })

    response = await completion(
        model=config["model"],
        messages=messages,
        api_key=os.getenv("GROQ_API_KEY")
    )

    return {"result": response.choices[0].message.content}
```

Add endpoint in `backend/app/main.py`:

```python
@app.post("/api/agent")
async def agent_endpoint(request: AgentRequest):
    result = await invoke_agent(
        agent_name=request.agent,
        context=request.context,
        user_profile=request.user_profile
    )
    return result
```

---

## Caching Strategy (Urdu Translation)

Store translations in Qdrant to avoid re-translating:

```python
def translate_to_urdu(english_text: str) -> str:
    # Check cache first
    cache_key = hashlib.md5(english_text.encode()).hexdigest()

    cached = qdrant.retrieve(
        collection_name="translations_cache",
        ids=[cache_key]
    )

    if cached:
        return cached[0].payload["urdu_text"]

    # Translate using UrduTranslator agent
    result = invoke_agent("UrduTranslator", english_text)

    # Cache result
    qdrant.upsert(
        collection_name="translations_cache",
        points=[{
            "id": cache_key,
            "vector": [0] * 1024,  # Dummy vector
            "payload": {
                "english_text": english_text,
                "urdu_text": result,
                "timestamp": datetime.now()
            }
        }]
    )

    return result
```

---

## Reusability Examples

### Example 1: Every Chapter Uses HardwareAdvisor

```markdown
<!-- ch01.md -->
<Agent name="HardwareAdvisor" context="Chapter 1: Basic Python and Linux" />

<!-- ch06.md -->
<Agent name="HardwareAdvisor" context="Chapter 6: Isaac Sim GPU requirements" />

<!-- ch12.md -->
<Agent name="HardwareAdvisor" context="Chapter 12: Jetson Orin Nano setup" />
```

**Result**: Same agent, different context → Personalized advice per chapter

### Example 2: LabGenerator Creates 13 Different Labs

```markdown
<!-- Automatically generates labs for each chapter -->
<Agent name="LabGenerator" context="Chapter 2: ROS 2 nodes" />
<Agent name="LabGenerator" context="Chapter 4: Gazebo world files" />
<Agent name="LabGenerator" context="Chapter 7: VLA model integration" />
```

**Result**: 13 unique labs from one reusable agent

### Example 3: Multi-Agent Collaboration

```markdown
<!-- Complex chapter with multiple agents -->
## Chapter 8: Humanoid Kinematics

<Agent name="ConceptExplainer" context="Explain inverse kinematics" />

[Student learns concept]

<Agent name="LabGenerator" context="Create 2D IK lab for 2-link arm" />

[Student completes lab]

<Agent name="CodeReviewer" context="Review my IK implementation" />

[Student gets feedback]

<Agent name="QuizMaster" context="Quiz on forward vs inverse kinematics" />
```

**Result**: Complete learning cycle with 4 agents working together

---

## Verification Checklist

To confirm 5+ agents are being used:

- [ ] HardwareAdvisor called in 3+ chapters
- [ ] LabGenerator called in 5+ chapters
- [ ] UrduTranslator available on all 13 chapters
- [ ] QuizMaster called in 5+ chapters
- [ ] CapstonePlanner called in Chapter 13
- [ ] CodeReviewer available for all lab submissions

**Total Agent Invocations**: 30+ across all chapters (5 agents × 6 average uses)

---

## Next Steps

1. **Create MDX Component**: Implement `<Agent />` component in Docusaurus
2. **Backend Integration**: Add `/api/agent` endpoint to FastAPI
3. **Load Agent Configs**: Parse YAML files and load system prompts
4. **Test Each Agent**: Verify output quality for sample inputs
5. **Deploy**: Add agents to production backend

**Branch**: 003-reusable-intelligence
**Status**: Agent definitions complete ✅
**Next**: Implement MDX integration + backend API
