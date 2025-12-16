---
id: 001
title: Physical AI & Humanoid Robotics Learning Platform Constitution
stage: constitution
date: 2025-12-15
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: Qwen Code
command: /sp.constitution
labels: ["constitution", "initialization", "ai-robotics", "education", "book-project", "spec-driven-development"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - history/prompts/constitution/001-physical-ai-humanoid-constitution.constitution.prompt.md
tests:
  - Constitution file structure validated
  - All required sections present
  - AI/Robotics specific guidelines included
  - Multilingual support provisions validated
---

## Prompt

/sp.constitution

Project: Physical AI & Humanoid Robotics Learning Platform

Core Principles:
- Technical accuracy and correctness across all AI and robotics topics
- Clear, structured explanations suitable for intermediate-to-advanced learners in robotics and AI
- Consistency in writing style, terminology, and formatting across the entire platform
- Modularity: each chapter should be self-contained but internally connected
- Ethical responsibility: all AI and robotics content must address ethical implications
- Safety-first approach: prioritize safety considerations in all practical applications

Key Standards:
- All technical information must be verified against authoritative robotics and AI sources
- Citation format: IEEE or ACM (maintain consistency throughout the platform)
- Source types: official documentation, academic papers, industry standards, technical specifications
- All code snippets and practical examples must be tested for correctness before inclusion
- No copyrighted content copied verbatim unless permitted (rewrite unless licensed)
- Writing clarity: Flesch-Kincaid grade 8–12 (technical clarity prioritized over simplicity)
- Directory and file structure must follow Docusaurus best practices for multilingual content

Constraints:
- Platform content: 10–15 chapters, each 1000–2500 words
- Must include at least:
  - 1 introduction to Physical AI and Humanoid Robotics
  - 1 foundational AI and robotics theory chapter
  - 3+ chapters on sensor integration and perception systems
  - 2+ chapters on actuator technology and motor control
  - 2+ chapters on human-robot interaction
  - 1 chapter on ethics and social impact
  - 1 appendix for practical projects and exercises
- Output format: Docusaurus-compatible Markdown (.md or .mdx) with multilingual support
- Deployment target: Multi-platform via Next.js frontend and Docusaurus book app
- All images must be either AI-generated or self-created (no copyrighted diagrams)
- Multilingual requirement: All content must be available in English and Urdu

Success Criteria:
- Platform builds successfully with no formatting or build errors
- Multilingual deployment passes without warnings
- All chapters are internally consistent in style and terminology
- All claims, definitions, or facts cross-checked against sources
- Code examples and practical exercises run successfully
- Clear instructional flow suitable for learners from diverse linguistic backgrounds
- Real-time translation functionality works seamlessly across all content

## Response snapshot

Created comprehensive constitution for Physical AI & Humanoid Robotics Learning Platform with seven core principles:

1. **Technical Accuracy and Correctness**: All AI and robotics content must be verifiable against authoritative sources
2. **Ethical Responsibility**: Defined framework prioritizing ethical implications in all AI/robotics applications
3. **Safety-First Approach**: All practical content incorporates safety considerations
4. **Multilingual Accessibility**: Content available in English and Urdu with cultural appropriateness
5. **Modularity and Consistency**: Self-contained chapters with internal connections
6. **Documentation-First Research**: Technical accuracy through official docs and academic papers
7. **Quality Validation**: All examples tested and follow industry best practices

Established content constraints (10-15 chapters, 1000-2500 words each), quality gates (technical review, ethical review, translation accuracy), and governance framework with amendment process and compliance verification.

Constitution file created at: `history/prompts/constitution/001-physical-ai-humanoid-constitution.constitution.prompt.md`



## Outcome

- ✅ Impact: Established foundational principles and standards for entire learning platform
- 🧪 Tests: Constitution structure validated, all required sections present, multilingual provisions included
- 📁 Files: Created/updated `history/prompts/constitution/001-physical-ai-humanoid-constitution.constitution.prompt.md`
- 🔁 Next prompts: Create initial chapter specs, set up Docusaurus multilingual structure, define robotics curriculum outline
- 🧠 Reflection: Constitution provides clear guardrails for content quality, technical accuracy, ethical considerations, and multilingual workflow. All future content generation must reference these principles.

## Evaluation notes (flywheel)

- Failure modes observed: None yet (initial creation)
- Graders run and results (PASS/FAIL): Structure validation PASS, Multilingual compliance PASS
- Prompt variant (if applicable): Standard constitution creation from user requirements with AI/Robotics specialization
- Next experiment (smallest change to try): Validate constitution against actual chapter creation workflow for robotics content