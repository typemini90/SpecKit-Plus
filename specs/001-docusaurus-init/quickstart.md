# Quickstart: Docusaurus Publishing Platform Setup

This guide provides a quick overview of setting up the Docusaurus-based publishing platform for the 'Embodied Intelligence' textbook.

## 1. Prerequisites

Ensure you have the following installed:

*   Node.js (LTS version recommended)
*   npm, yarn, pnpm, or bun (a package manager)
*   `espeak` (for audible notifications on Linux-based systems)

## 2. Initialization

To initialize the Docusaurus project in the current directory, run the following command:

```bash
npx create-docusaurus@latest . classic --typescript
```

This will set up a new Docusaurus site with the classic template and TypeScript support.

## 3. Configuration and Content Structure

After initialization, you will need to:

*   Update `docusaurus.config.ts` with the correct title, tagline, organization name, and project name.
*   Remove default content (blog, intro docs) and their navigation links.
*   Create the content directories `docs/01-foundations`, `docs/02-simulation`, `docs/03-perception`, `docs/04-vla` with placeholder files.

## 4. Audible Notifications

An `espeak`-based audible notification utility (`scripts/speak.sh`) will be provided to announce agent actions. Ensure `espeak` is installed on your system for this feature to work.

## 5. Running the Development Server

To start the development server and view your site locally:

```bash
npm start
```

This will typically open the site in your browser at `http://localhost:3000`.
