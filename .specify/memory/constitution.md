<!-- Sync Impact Report:
Version change: 1.0.0 → 1.1.0
Modified principles: None
Added sections: Platform Architecture, RAG Chatbot Architecture, Spec-Driven Development, Package Management, Reusable Intelligence and Skills, Audio Feedback
Removed sections: None
Templates requiring updates:
- .specify/templates/plan-template.md: ⚠ pending
- .specify/templates/spec-template.md: ⚠ pending
- .specify/templates/tasks-template.md: ⚠ pending
- .specify/templates/commands/*.md: ⚠ pending
Follow-up TODOs: None
-->
# Embodied Intelligence Constitution

## Core Principles

### I. Platform Architecture
The book platform will be built using Docusaurus 3.9 (React/MDX) with Spec-Kit Plus and Claude Code development methodology. Deployment will be to GitHub Pages (or Vercel). Authentication will use Better-Auth (with optional Signup/Signin). LLM-based Urdu Translation is a bonus.

### II. RAG Chatbot Architecture
The RAG Chatbot will use OpenAI Agents / ChatKit SDKs with a FastAPI (Python) framework. Data will be stored in Qdrant Cloud (Free Tier) for vector data and Neon Serverless Postgres for relational data. The chatbot will answer questions based on book content and selected text.

### III. Spec-Driven Development
All development will follow the Spec-Kit Plus methodology, leveraging Claude Code for task execution and guidance.

### IV. Package Management
The project will use pnpm as the package manager for all Node.js dependencies. This ensures efficient disk space usage and faster installations through strict dependency management and symlinks.

### V. Reusable Intelligence and Skills
The system will be designed to promote reusable intelligence and skills across different components.

### VI. Audio Feedback
After every major task, the system will use 'espeak' (Linux) to speak a 3-word summary. For example, "Muhammad Suhiab I have done xyz".

## Development Methodology and Requirements

### Development Methodology
The primary development methodology will be Spec-Kit Plus combined with Claude Code. This approach emphasizes clear specifications, automated task generation, and AI-assisted implementation.

### Key Requirements
- **Spec-Kit Plus Integration**: All features, plans, and tasks will be managed and tracked using Spec-Kit Plus.
- **Reusable Intelligence**: Design and implement components to maximize reusability of intelligent agents and skills.

## Governance
This Constitution outlines the foundational principles and architectural decisions for the 'Embodied Intelligence' project. It serves as the single source of truth for core development practices and technological choices. Amendments to this Constitution require a formal review process and documented rationale. All new features and changes must adhere to the principles laid out herein.

**Version**: 1.1.0 | **Ratified**: 2025-12-04 | **Last Amended**: 2025-12-06
