# Feature Specification: RAG Chatbot for Physical AI Textbook

**Feature Branch**: `002-rag-chatbot-groq`
**Created**: 2025-12-09
**Status**: Draft
**Input**: User description: "RAG Chatbot for Physical AI Textbook - Chatbot is embedded inside the published Docusaurus book - Answers any question about the 13 chapters correctly - When user highlights/selects any text → chatbot answers ONLY using that text + relevant chunks - Uses Groq + LiteLLM (not direct OpenAI) for generation - Uses OpenAI Agents SDK for retrieval tool calling - Backend runs on FastAPI + Vercel - Vector DB: Qdrant Cloud Free Tier - Metadata DB: Neon Serverless Postgres - One-time ingestion of all MDX files - Live demo works on any internet connection - Constraints: Free tiers only, no auth, no Urdu, no personalization"

## User Scenarios & Testing

### User Story 1 - Basic Question Answering (Priority: P1)

As a student reading the Physical AI textbook, I want to ask questions about the content and receive accurate answers so that I can quickly understand complex concepts without searching through multiple chapters.

**Why this priority**: This is the core value proposition - students get instant help while reading. Without this, the chatbot has no purpose. This can be delivered as a complete MVP that demonstrates the RAG capability to judges.

**Independent Test**: Can be fully tested by asking "What is Physical AI?" on the published book site and receiving a correct, cited answer from Chapter 1 within 3 seconds. Deliverable value: Students can learn faster by asking questions instead of re-reading chapters.

**Acceptance Scenarios**:

1. **Given** a student is reading any chapter, **When** they click the chatbot button and ask "What is ROS 2?", **Then** the chatbot provides an accurate answer with citation to the relevant chapter
2. **Given** a student asks a general question like "What is Physical AI?", **When** the query is submitted, **Then** the chatbot searches the textbook content and streams a response citing the source chapter
3. **Given** a student asks about a topic not covered in the textbook (e.g., "quantum computing"), **When** the query is submitted, **Then** the chatbot clearly states "This topic is not covered in the textbook"
4. **Given** multiple students use the chatbot simultaneously, **When** they ask different questions, **Then** each receives accurate, independent responses without interference

---

### User Story 2 - Context-Aware Answers from Selected Text (Priority: P2)

As a student who has highlighted/selected a specific paragraph or code snippet, I want the chatbot to answer my question specifically about that selection so that I can get targeted explanations without having to retype the content.

**Why this priority**: This significantly improves UX by making the chatbot context-aware. Students can highlight confusing sentences and ask "explain this" - a powerful learning tool that differentiates this from generic chatbots. Can be tested independently once P1 is complete.

**Independent Test**: Can be fully tested by highlighting a paragraph about "VLA models" in Chapter 7 and asking "Explain this concept" - the chatbot should reference the selected text prominently in its answer and retrieve related chunks.

**Acceptance Scenarios**:

1. **Given** a student has highlighted text about "humanoid locomotion", **When** they ask "How does this work?", **Then** the chatbot prioritizes the selected text and retrieves related chunks to answer specifically about that topic
2. **Given** a student selects a code snippet from a ROS 2 example, **When** they ask "What does this code do?", **Then** the chatbot explains the selected code with reference to relevant textbook sections
3. **Given** a student highlights text and then clears the selection, **When** they ask a new question, **Then** the chatbot treats it as a general query without the previous context
4. **Given** a student highlights text that contains a specific technical term, **When** they ask about that term, **Then** the chatbot's answer incorporates both the selected context and relevant textbook chunks from other chapters

---

### User Story 3 - Persistent Conversation History (Priority: P3)

As a student working through the textbook over multiple sessions, I want my previous questions and answers to be saved so that I can review what I learned and continue where I left off.

**Why this priority**: Nice-to-have feature that improves learning continuity. Students can review their learning history and build on previous conversations. Can be added after core Q&A works and is not critical for hackathon demo.

**Independent Test**: Can be fully tested by asking 3 questions, closing the browser, reopening the book, and seeing the previous conversation still displayed in the chatbot window.

**Acceptance Scenarios**:

1. **Given** a student has asked 5 questions in a session, **When** they reload the page, **Then** all previous messages and answers are restored from the session database
2. **Given** a student has multiple conversation sessions over several days, **When** they return to the site, **Then** the most recent conversation is displayed
3. **Given** a student clears their browser data, **When** they visit the site, **Then** they start with a fresh conversation (session ID is generated anew)

---

### User Story 4 - Real-time Streaming Responses (Priority: P1)

As a student asking a question, I want to see the answer appear progressively word-by-word so that I know the system is working and can start reading the response immediately without waiting for the full answer.

**Why this priority**: Critical for user experience - prevents perceived lag and makes the chatbot feel responsive and modern. Essential for P1 MVP to demonstrate during hackathon judging.

**Independent Test**: Can be fully tested by asking any question and observing tokens appearing progressively with first word visible within 300ms.

**Acceptance Scenarios**:

1. **Given** a student submits a question, **When** the chatbot begins responding, **Then** individual words/tokens appear progressively rather than all at once
2. **Given** the chatbot is generating a long response (200+ words), **When** the student is reading, **Then** new words continue to stream in smoothly without long pauses between tokens
3. **Given** the chatbot encounters an error during streaming, **When** the error occurs, **Then** the student sees the partial response plus a clear error message

---

### Edge Cases

- What happens when a student asks a question in a language other than English (e.g., Urdu, Chinese)?
  - **Expected**: Chatbot responds in English only, with message: "I can only respond in English currently."

- How does the system handle very long questions (>1000 characters)?
  - **Expected**: Question is accepted up to token limit; if exceeded, return message: "Please shorten your question to under 500 words."

- What happens when the textbook content hasn't been ingested yet into the vector database?
  - **Expected**: Chatbot responds with "The textbook content is currently being loaded. Please try again in a few moments."

- How does the system handle simultaneous requests from 100+ students during peak usage (hackathon demo)?
  - **Expected**: All requests are queued and processed; response time may increase to 2-5 seconds but no requests fail due to rate limiting

- What happens when a student selects an entire chapter (10,000+ words) as context?
  - **Expected**: System uses the first 2000 characters of selection to avoid token limits, with message: "Using first portion of selected text due to length."

- How does the chatbot handle questions about figures, diagrams, or images in the textbook?
  - **Expected**: Chatbot acknowledges visual content exists but can only describe text captions/alt text: "I can see there's a diagram here, but I can only explain the text description..."

- What happens when Qdrant vector database is temporarily unavailable?
  - **Expected**: User sees friendly error: "The chatbot is temporarily unavailable. Please try again in a moment." (no technical error details exposed)

- How does the system handle rapid-fire questions (student asking 10 questions in 10 seconds)?
  - **Expected**: Each question is processed in order; previous streaming response is automatically cancelled when new question is submitted

- What happens when a student's question contains special characters or code syntax (e.g., "What does `rospy.spin()` do?")?
  - **Expected**: Question is processed normally; special characters are escaped/sanitized before vector search

- How does the chatbot handle ambiguous pronouns (e.g., "What is it?" without prior context)?
  - **Expected**: Chatbot asks for clarification: "Could you please specify what you're referring to?" or attempts to infer from recent conversation

## Requirements

### Functional Requirements

- **FR-001**: System MUST provide a chatbot interface embedded on every page of the published Docusaurus book
- **FR-002**: Chatbot MUST answer questions using content from all 13 textbook chapters
- **FR-003**: Chatbot MUST cite the source chapter for every factual answer (e.g., "According to Chapter 3...")
- **FR-004**: Chatbot MUST detect when a user has highlighted/selected text on the page using browser selection API
- **FR-005**: Chatbot MUST prioritize selected text as primary context when answering questions
- **FR-006**: Chatbot MUST stream responses progressively (word-by-word) using Server-Sent Events or similar streaming protocol
- **FR-007**: System MUST use semantic search to retrieve relevant textbook chunks before generating answers
- **FR-008**: System MUST limit retrieved context to the top 5 most relevant chunks per query (to stay within token limits)
- **FR-009**: Chatbot MUST clearly state when a topic is not covered in the textbook (e.g., "I don't find information about X in this textbook")
- **FR-010**: System MUST complete one-time ingestion of all 13 chapter MDX files into vector database during initial setup
- **FR-011**: Chatbot MUST make citation links clickable, navigating to the referenced chapter when clicked
- **FR-012**: System MUST work on any standard internet connection without requiring VPN or special network access
- **FR-013**: Chatbot MUST support both dark mode and light mode themes, automatically matching the Docusaurus site theme
- **FR-014**: System MUST be mobile-responsive and fully functional on screens down to 320px width
- **FR-015**: System MUST handle at least 100 concurrent users without request failures
- **FR-016**: Chatbot MUST use autonomous agent tool-calling to decide when to search the textbook versus responding directly
- **FR-017**: System MUST store conversation metadata (query text, selected text, response time, citations, success status) for analytics
- **FR-018**: Chatbot button MUST be visible and accessible on every documentation page without requiring scrolling
- **FR-019**: System MUST use only free-tier services for all infrastructure components (no paid upgrades)
- **FR-020**: Chatbot MUST NOT require user authentication, account creation, or any login process

### Key Entities

- **Query**: A student's question about textbook content
  - Attributes: question text (up to 1000 characters), selected text context (optional, up to 2000 characters), session identifier (UUID), timestamp, user agent
  - Relationships: belongs to a Session, generates one Response

- **Response**: The chatbot's answer to a query
  - Attributes: generated text, list of citations (chapter references), total generation time (milliseconds), token count, success status (boolean)
  - Relationships: belongs to one Query, references multiple Chunks via citations

- **Chunk**: A segment of textbook content stored in the vector database
  - Attributes: text content (512 tokens max), chapter ID (e.g., "ch01"), chapter title, embedding vector (1024 dimensions), chunk index within chapter, token count
  - Relationships: belongs to one Chapter, can be referenced by multiple Responses

- **Session**: A conversation thread between a student and the chatbot
  - Attributes: session ID (UUID), creation timestamp, last active timestamp, total message count, student's browser fingerprint (optional)
  - Relationships: contains multiple Queries and Responses in chronological order

- **Citation**: A reference to a specific textbook chapter
  - Attributes: chapter ID (e.g., "ch03"), chapter title (e.g., "Chapter 3: ROS 2 Advanced Patterns"), similarity score (0.0-1.0), chapter URL path
  - Relationships: links a Response to a source Chapter, indicates retrieval relevance

## Success Criteria

### Measurable Outcomes

- **SC-001**: Students receive first word of response within 300ms of submitting a question (p95 latency)
- **SC-002**: 95% of questions about textbook content receive factually accurate answers with correct chapter citations
- **SC-003**: System handles 100 concurrent students asking questions without response time degradation beyond 2x baseline (<600ms first token)
- **SC-004**: Chatbot correctly prioritizes selected text in 100% of test cases where text is highlighted
- **SC-005**: Students can complete a full question-answer-citation cycle (ask question, read response, click citation to navigate) in under 10 seconds
- **SC-006**: System successfully ingests all 13 chapters (estimated 50,000-100,000 words total) into vector database in under 10 minutes
- **SC-007**: Live demo works from any geographic location with standard internet connection (tested from at least 3 different countries)
- **SC-008**: Chatbot UI is fully functional on mobile devices with screen widths as small as 320px (iPhone SE)
- **SC-009**: 90% of first-time users can find and use the chatbot button without instructions or onboarding
- **SC-010**: System operates entirely within free-tier limits for all services during 48-hour hackathon period (estimated 1000-2000 total queries)
- **SC-011**: Zero authentication-related errors or user complaints (since no auth is implemented)
- **SC-012**: Chatbot provides helpful, user-friendly responses for 100% of edge cases (e.g., unsupported languages, topics not in textbook, service unavailability)
- **SC-013**: Judges can verify the chatbot is using Groq (not OpenAI) by inspecting network requests and confirming API endpoint domain
- **SC-014**: Citation links successfully navigate to correct chapter pages in 100% of cases

## Assumptions

- Students will primarily ask factual questions about textbook content rather than open-ended philosophical discussions
- The 13 chapters are written in English and contain primarily text content with some code snippets, equations, and diagrams with captions
- Free-tier rate limits are sufficient for hackathon demo period (estimated <2000 queries total, <50 queries/hour peak)
- Students access the book via modern browsers (Chrome 90+, Firefox 90+, Safari 14+, Edge 90+)
- Textbook content is static after initial publication (no real-time collaborative editing)
- Selected text by students will typically be 1-5 paragraphs (50-500 words, under 2000 characters)
- Network latency for students accessing cloud services is under 500ms
- Docusaurus site is publicly accessible at a GitHub Pages or Vercel URL
- MDX files follow standard Docusaurus frontmatter format with fields: id, title, sidebar_label
- Vector similarity threshold of 0.6 (cosine similarity) is sufficient for retrieving relevant chunks
- Embedding model (Cohere embed-english-v3.0) produces 1024-dimensional vectors
- Groq's llama-3.3-70b-versatile model is available and performant for the demo period
- Session data stored in Neon can be deleted after 7 days (no long-term retention required)

## Scope

### In Scope

- Embedded chatbot widget visible on all Docusaurus documentation pages
- Question answering capability for all 13 textbook chapters using RAG (Retrieval-Augmented Generation)
- Text selection detection using browser selection API
- Context-aware responses that prioritize highlighted text
- Semantic search using vector embeddings (Cohere + Qdrant)
- Streaming text generation using Server-Sent Events (SSE)
- Chapter citation rendering with clickable navigation links
- One-time batch ingestion of all MDX files into Qdrant vector database
- Automatic theme adaptation (dark mode and light mode support)
- Mobile-responsive chatbot UI (down to 320px screen width)
- Free-tier infrastructure exclusively (Groq, Cohere, Qdrant Cloud, Neon, Vercel/Railway)
- Session metadata storage (queries, responses, performance metrics)
- Agent-based retrieval using function calling (OpenAI Agents SDK pattern with LiteLLM)
- Error handling for external service failures (Groq, Qdrant, Cohere)
- Browser-based session management (no server-side user accounts)

### Out of Scope

- User authentication, authorization, or personalized user accounts
- Persistent conversation history across devices or browsers (only within-session persistence)
- Multi-language support (Urdu, Arabic, Chinese translations)
- Voice input (speech-to-text) or voice output (text-to-speech)
- Image or diagram content analysis (visual question answering)
- Real-time collaboration features (multiple students in same chat)
- Admin dashboard for monitoring usage or performance metrics
- Custom chatbot personality or tone fine-tuning
- Integration with external Learning Management Systems (Canvas, Moodle, Blackboard)
- Downloadable conversation transcripts (PDF, text export)
- Email notifications or alerts
- Feedback collection system (thumbs up/down on responses)
- Rate limiting per user (since no user accounts exist)
- Content moderation or profanity filtering
- Analytics dashboard for instructors
- Search history or "Related questions" suggestions

## Dependencies

### External Services (Must be provisioned before development)

- **Docusaurus book**: Must be published and accessible via public URL (GitHub Pages, Vercel, or Netlify)
- **Groq API**: Requires valid API key with access to llama-3.3-70b-versatile model (free tier: 30 req/min)
- **Cohere API**: Requires valid API key for embed-english-v3.0 embeddings (free tier: 100 API calls/min)
- **Qdrant Cloud**: Free-tier cluster must be created with API key and URL
- **Neon Serverless Postgres**: Free-tier database must be provisioned with connection string
- **Vercel or Railway**: Deployment platform account for hosting FastAPI backend

### Content Dependencies

- **Chapter Content**: All 13 chapters must be written as MDX files with proper frontmatter (id, title fields required)
- **MDX Format**: Files must follow Docusaurus conventions (frontmatter delimited by `---`, valid YAML)

### Technical Dependencies

- **Modern Browser**: Students must use browsers supporting: Fetch API, Server-Sent Events, Selection API, CSS Grid/Flexbox
- **Internet Connection**: Minimum 1 Mbps download speed for streaming responses

## Constraints

### Cost Constraints

- **Zero Budget**: Must use only free tiers - absolutely no paid services or upgrades
  - **Groq Free Tier**: 30 requests/min, 6,000 tokens/min, 7,000 requests/day
  - **Cohere Free Tier**: 100 API calls/min for embeddings
  - **Qdrant Cloud Free**: 1GB storage, 1M vectors max
  - **Neon Free Tier**: 0.5GB database storage, 3GB data transfer/month
  - **Vercel Free Tier**: 100GB bandwidth/month, 100 serverless function invocations/day

### Feature Constraints

- **No Authentication**: No user accounts, logins, passwords, or personalization
- **English Only**: No support for Urdu translation or other languages
- **Public Access**: Must work on open internet without VPN, firewall bypass, or special network configurations
- **Browser Compatibility**: Modern browsers only (Chrome 90+, Firefox 90+, Safari 14+, Edge 90+) - no IE11
- **Static Content**: Textbook chapters are ingested once during setup; no real-time content synchronization

### Technical Constraints

- **Free-tier Rate Limits**: Must implement graceful handling when rate limits are reached
- **Token Limits**: Retrieved context + user question + system prompt must fit within model's context window (typically 8K-32K tokens)
- **Embedding Dimensions**: Must use 1024-dimensional vectors (Cohere embed-english-v3.0 standard)
- **Chunking Strategy**: Text must be split into chunks of 512 tokens with 50-token overlap for optimal retrieval

## Non-Functional Requirements

### Performance

- **First Token Latency**: <300ms (p95) from query submission to first word displayed
- **Full Response Time**: <5 seconds for typical 150-200 word answer (p95)
- **Textbook Search Time**: <100ms for semantic search across vector database (p95)
- **Page Load Impact**: Chatbot adds <500KB to page size, <1 second to initial load time
- **Concurrent Capacity**: Support 100 simultaneous active conversations without degradation

### Reliability

- **Uptime Target**: 99% availability during hackathon judging period (48 hours)
- **Error Rate**: <5% of queries result in errors (excluding user-caused errors like unsupported languages)
- **Graceful Degradation**: When external services (Groq, Qdrant) are unavailable, show user-friendly error messages instead of technical stack traces

### Usability

- **Zero-click Discoverability**: Chatbot button visible on page load without scrolling or hover actions
- **Mobile-First Design**: All features work on smartphones with touch input
- **Accessibility**: Keyboard navigation support (Tab, Enter, Escape) for chatbot interactions
- **Visual Feedback**: Loading indicators, typing animations, clear UI states (idle, loading, error, success)

## Risks & Mitigations

| Risk | Impact | Likelihood | Mitigation |
| ---- | ------ | ---------- | ---------- |
| Free-tier rate limits exceeded during live demo to judges | **High** - Feature appears broken | Medium | Implement client-side rate limiting (max 1 query/2 seconds), queue requests, show "Please wait" countdown timer |
| Groq API model deprecated or changed mid-hackathon | **High** - Complete system failure | Low | Document exact model version used, test with fallback model (llama-3.1-8b-instant), keep API key backup |
| Selected text detection fails on Safari or Firefox | **Medium** - Context feature unavailable | Medium | Test across all major browsers during development, provide manual "paste context" textarea as fallback |
| Vector search returns irrelevant chunks for ambiguous queries | **Medium** - Poor answer quality | Medium | Tune similarity threshold (0.5-0.7 range), implement keyword filtering, show similarity scores in debug mode |
| Textbook content changes after initial ingestion | **Low** - Stale answers | Low | Provide re-ingestion script with clear documentation, add "Last updated" timestamp to chatbot UI |
| Network latency in judge's location causes slow responses | **Medium** - Perceived poor performance | Low | Use CDN for frontend assets, choose cloud regions close to hackathon venue, test from multiple geographic locations |
| Browser blocking third-party cookies affects session persistence | **Low** - Sessions don't persist | Medium | Use localStorage as fallback, clearly communicate session-only persistence to users |

---

## Acceptance Checklist for Judges

This checklist will be provided to hackathon judges to verify the feature is complete and working correctly.

### Core Functionality Tests (Must Pass 10/10)

- [ ] **Test 1 - Chatbot Visibility**: Open the published Docusaurus book URL, verify chatbot button is visible in bottom-right corner on at least 3 different chapter pages
- [ ] **Test 2 - Chat Window**: Click chatbot button and confirm chat window opens with welcome message
- [ ] **Test 3 - Basic Q&A**: Ask "What is Physical AI?" and verify:
  - [ ] Response begins streaming within 2 seconds
  - [ ] Answer is factually correct based on Chapter 1 content
  - [ ] Citation appears (e.g., "Chapter 1: Introduction to Physical AI")
  - [ ] Citation is clickable and navigates to Chapter 1 when clicked
- [ ] **Test 4 - Multi-Chapter Knowledge**: Ask "What is ROS 2?" and verify answer cites Chapter 2 or 3 correctly
- [ ] **Test 5 - Unknown Topic Handling**: Ask "What is quantum computing?" and verify chatbot clearly states topic is not in textbook
- [ ] **Test 6 - Text Selection Context**: Highlight a paragraph about "VLA models", ask "Explain this", verify answer explicitly references the selected text
- [ ] **Test 7 - Streaming Display**: Observe that response appears word-by-word progressively (not all at once)
- [ ] **Test 8 - Mobile Responsiveness**: Resize browser to 375px width (or test on actual smartphone) and verify chatbot is fully functional
- [ ] **Test 9 - Dark Mode**: Toggle Docusaurus dark mode and verify chatbot colors adapt appropriately
- [ ] **Test 10 - Multi-Question Flow**: Ask 3 different questions in sequence and verify all receive correct answers

### Performance Tests (Must Pass 3/4)

- [ ] **Perf 1 - Response Speed**: First word appears in under 500ms for simple queries like "What is Physical AI?"
- [ ] **Perf 2 - Full Answer Speed**: Complete answer (100-150 words) finishes streaming in under 5 seconds
- [ ] **Perf 3 - Rapid Queries**: Submit 5 questions in rapid succession (one every 3 seconds) - verify all receive answers without errors
- [ ] **Perf 4 - Geographic Access**: Test from judge's current location without VPN - verify chatbot works normally

### Technical Verification Tests (Must Pass 4/4)

- [ ] **Tech 1 - Groq Verification**: Open browser DevTools Network tab, submit question, confirm API requests go to Groq domain (NOT api.openai.com)
- [ ] **Tech 2 - No Authentication**: Verify no login page, no sign-up prompts, no API keys visible in client-side code
- [ ] **Tech 3 - Cross-Browser**: Test in both Chrome and Firefox - verify chatbot works identically
- [ ] **Tech 4 - Selected Text Capture**: Highlight text, open DevTools Network tab, submit question, verify POST request payload includes `selected_text` field with highlighted content

### Bonus Tests (Optional - Extra Points)

- [ ] **Bonus 1 - Multi-Citation**: Ask a question that spans multiple chapters (e.g., "How do I deploy a ROS 2 robot?") and verify response cites 2+ chapters
- [ ] **Bonus 2 - Helpful Suggestions**: After answering, chatbot suggests "You might also want to read Chapter X"
- [ ] **Bonus 3 - Error UX**: Temporarily disconnect internet, submit question, verify error message is user-friendly (not technical stack trace)
- [ ] **Bonus 4 - Visual Polish**: Chatbot has smooth animations (fade in/out, typing indicator), professional gradient design, pleasant color scheme

---

## Success Scoring Rubric for Judges

**Minimum Passing Score (Acceptable Demo)**:
- Core Functionality: 10/10 (100%)
- Performance: 3/4 (75%)
- Technical: 4/4 (100%)
- **Total**: 17/18 = 94%

**Excellent Score (Competition Winner)**:
- Core Functionality: 10/10 (100%)
- Performance: 4/4 (100%)
- Technical: 4/4 (100%)
- Bonus: 3/4 (75%)
- **Total**: 21/22 = 95%+

---

## Notes for Implementation

- This specification is intentionally technology-agnostic in most requirements, but includes explicit technical constraints where the user specified them (Groq, LiteLLM, Qdrant, Neon, FastAPI, Agents SDK pattern)
- The chatbot is stateless beyond session-level history - it does not learn or improve from user interactions
- No personally identifiable information (PII) is collected since there's no authentication system
- The feature is optimized for hackathon judging criteria: ease of demo, visual appeal, technical sophistication, and practical utility for students
- Success criteria focus on measurable, judge-verifiable outcomes rather than long-term production metrics
- The acceptance checklist is designed to be completed in 10-15 minutes by judges during live demo presentation
