# Research Findings for Docusaurus 3.9 Setup

## 1. Docusaurus Initialization with TypeScript

**Decision**: Use `npx create-docusaurus@latest my-website classic --typescript` for initialization. The `my-website` placeholder will be replaced with `.` to initialize in the current directory.

**Rationale**: This command is directly from the Docusaurus documentation and ensures the use of the `classic` template with TypeScript support, as required by the plan and implied by `mcp.json`.

**Alternatives considered**: None, as the user explicitly requested TypeScript support and the `classic` template.

## 2. Docusaurus Configuration for Branding (Title, Tagline, Organization Name, Project Name)

**Decision**: The `title` and `tagline` will be set directly in `docusaurus.config.ts`. The organization name and project name are not directly configurable in `docusaurus.config.ts` for display but are typically derived from `package.json` or project structure. For the purpose of display, the `title` will encompass "Embodied Intelligence" and the `tagline` will be "Physical AI & Humanoid Robotics".

**Rationale**: The Docusaurus documentation (`/websites/docusaurus_io` in Context7) clearly shows `title` and `tagline` as top-level properties in the `Config` object within `docusaurus.config.ts`. The `navbar.title` within `themeConfig` can also be used for the display title. "Panaversity" and "embodied-intelligence-book" will be considered project-level metadata or used in other configuration files if needed by the Docusaurus setup.

**Alternatives considered**: Setting `navbar.title` separately, but the main `title` property is sufficient for the primary branding.

## 3. Cleanup of Default Content (Blog, Tutorial/Docs) and Navigation Links

**Decision**:
1.  To remove the default blog, set `blog: false` within the `@docusaurus/preset-classic` configuration in `docusaurus.config.ts`.
2.  To remove default tutorial/docs, the docs plugin will be configured, and its associated navigation links will be removed from `themeConfig.navbar.items`. The content files themselves (e.g., `docs/intro.md`) will be deleted.

**Rationale**: The Docusaurus documentation (`/websites/docusaurus_io` in Context7) provides examples of disabling the blog plugin (`docs: false, blog: false`) and configuring navbar items. Explicitly removing navigation links and deleting content files ensures a clean slate as required by the user story.

**Alternatives considered**:
*   Modifying the existing blog/tutorial content, but the requirement is to remove it entirely.
*   Not explicitly removing navigation links, but this would leave broken links, which is undesirable.

## 4. Folder Creation for Four Parts (`01-foundations`, `02-simulation`, `03-perception`, `04-vla`)

**Decision**:
1.  Create the directories `docs/01-foundations`, `docs/02-simulation`, `docs/03-perception`, `docs/04-vla`.
2.  Within each directory, create a placeholder file (e.g., `_index.md` or `README.md`) to ensure the Docusaurus build process has content to work with and the directories are recognized.

**Rationale**: This aligns with the "Establish Textbook Content Structure" user story (US-4) and its functional requirement FR-004. Placeholder files prevent build errors from empty directories and provide a clear starting point for authors.

**Alternatives considered**: Relying on Docusaurus to generate these, but explicit creation ensures the exact structure is met.

## 5. Audible Notification Utility (`scripts/speak.sh`)

**Decision**: Create a shell script `scripts/speak.sh` that takes a message as an argument and executes `espeak` with that message.

**Rationale**: This directly addresses the "Audible Notification for Progress" user story (US-5) and functional requirement FR-006, which explicitly states to "directly call `espeak` with hardcoded messages for each action" and to "assume `espeak` is always available and do not handle its absence."

**Alternatives considered**: Implementing the `espeak` call directly in JavaScript/TypeScript, but a dedicated shell script provides a clean separation of concerns for this specific utility.

## 6. Docusaurus Version

**Decision**: Use Docusaurus 3.9 as specified in the `/sp.plan` command.

**Rationale**: This is a direct instruction from the user.

**Alternatives considered**: None, as a specific version was provided.
