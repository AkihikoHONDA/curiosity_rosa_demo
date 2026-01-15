# AGENTS.md — Repository Rules (curiosity_rosa_demo)

This repository is developed with VS Code + Codex (agent mode).
Global/common working rules are defined in the user-level `~/.codex/AGENTS.md`.
This file defines **repo-specific** rules: where to find docs, what is the contract, and when to stop for human review.

---

## 1. Where to find the project documents

Development documents live under `docs_dev/` (unless explicitly stated otherwise in `README.md`).

Primary references:
- `docs_dev/tasks/task_{n}.md` (task definition / DoD)
- `docs_dev/tasks_overview.md` (task dependency / scope boundaries)
- `docs_dev/design.md` (architecture, I/F, constraints)
- `docs_dev/requirements.md` (requirements, success criteria)
- `docs_dev/user_input.md` (background; lower priority)

---

## 2. Project contract (must not be changed without approval)

This project has explicit hard/soft locks.
Treat the following file as the **single source of truth** for “do-not-change” items:

- `docs_dev/PROJECT_CONTRACT.md`

Rules:
- Do not change any contract items (I/F names/types, fixed phrases, config schema, etc.) by default.
- If a contract change seems necessary, stop at the planning gate and raise it as a **proposal** (reason, impact, migration plan, tests). Do not edit code yet.

---

## 3. Mandatory stop points (human review)

For every task implementation, the agent must follow the gates defined in `~/.codex/AGENTS.md`.

Additional repo rule:
- Always stop after the planning gate (Gate B) and wait for the user’s “OK” before making edits.
- Also stop before any documentation backflow updates (Gate E), unless explicitly instructed.
- Even when no documentation update is needed, explicitly state that Gate E is not required before proceeding to Gate F.

---

## 4. Task prompt expectation (for this repo)

When the user says “Implement task_{n}”, interpret it as:

- Target: `docs_dev/tasks/task_{n}.md`
- References: `docs_dev/tasks_overview.md`, `docs_dev/design.md`, `docs_dev/requirements.md`, `docs_dev/PROJECT_CONTRACT.md`
- Output first: Gate A (questions if needed) or Gate B (implementation plan), then stop.

---

## 5. External actions / dependency changes

Before running external/networked actions or adding dependencies (apt/pip/git, etc.), present:
- what will be executed
- why it is needed
- expected impact

Then wait for approval.
