# Implementation Plan: Project Initialization: Setup foundational publishing platform

**Branch**: `001-docusaurus-init` | **Date**: 2025-12-05 | **Spec**: /home/giaic/code/dockathon/specs/001-docusaurus-init/spec.md
**Input**: Feature specification from `/specs/[###-feature-name]/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

The primary requirement is to set up a Docusaurus-based web publishing platform for the 'Embodied Intelligence' textbook, including content structure and audible notifications. The technical approach involves initializing Docusaurus 3.9 with TypeScript, configuring branding, cleaning up default content, structuring content directories, and creating a 'scripts/speak.sh' utility for audio alerts.

## Technical Context

<!--
  ACTION REQUIRED: Replace the content in this section with the technical details
  for the project. The structure here is presented in advisory capacity to guide
  the iteration process.
-->

**Language/Version**: JavaScript/TypeScript (Docusaurus 3.9)
**Primary Dependencies**: Docusaurus 3.9, Node.js, npm/yarn/pnpm/bun, espeak
**Storage**: N/A (static site)
**Testing**: Docusaurus build process, shell script execution for `espeak`
**Target Platform**: Web (Static site), Linux (for `espeak`)
**Project Type**: Web
**Performance Goals**: No specific targets
**Constraints**: Assumes `espeak` availability on the user's local terminal
**Scale/Scope**: Foundational publishing platform for a textbook.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- **I. Platform Architecture**: The plan aligns with using Docusaurus 3.9 with Spec-Kit Plus and Claude Code methodology.
- **V. Audio Feedback**: The plan includes an 'espeak' utility for audible feedback, aligning with the constitution's principle.

## Project Structure

### Documentation (this feature)

```text
specs/[###-feature]/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)
<!--
  ACTION REQUIRED: Replace the placeholder tree below with the concrete layout
  for this feature. Delete unused options and expand the chosen structure with
  real paths (e.g., apps/admin, packages/something). The delivered plan must
  not include Option labels.
-->

```text
.
├── blog/                     # Default Docusaurus blog (will be removed)
├── docs/
│   ├── intro.md              # Default Docusaurus intro (will be removed)
│   ├── 01-foundations/
│   │   └── _index.md         # Placeholder for Part 1
│   ├── 02-simulation/
│   │   └── _index.md         # Placeholder for Part 2
│   ├── 03-perception/
│   │   └── _index.md         # Placeholder for Part 3
│   └── 04-vla/
│       └── _index.md         # Placeholder for Part 4
├── src/                      # React components, pages, etc.
├── static/                   # Static assets
├── scripts/
│   └── speak.sh              # Audible notification utility
├── docusaurus.config.ts      # Docusaurus configuration (TypeScript)
├── package.json
├── tsconfig.json
└── yarn.lock (or package-lock.json/pnpm-lock.yaml/bun.lockb)
```

**Structure Decision**: The project will follow the standard Docusaurus static site structure, with specific modifications for content organization and the addition of a custom 'speak.sh' script. Default blog and introduction docs will be removed, and dedicated directories for the textbook parts will be created under `docs/`.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |
