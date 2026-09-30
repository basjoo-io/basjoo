---
name: brainstorming
description: Use before any creative work, when the user says "let's brainstorm", wants to think through options, describes a vague or ambitious feature request, asks what to build, presents a problem with multiple valid solutions, or seems unsure about scope or direction. Triggers: brainstorm, 头脑风暴, 设计讨论, think through, explore idea.
argument-hint: "[feature idea or problem to explore]"
---

# Brainstorming Ideas Into Designs

**Announce at start:** "I'm using the brainstorming skill to explore this idea."

Use the current date from session context when dating design documents.

## Overview

Brainstorming answers **WHAT** to build; the **writing-arch-plans** skill answers **HOW**. The durable artifact is a **design document** (`*-design.md`) strong enough that planning does not need to invent product behavior, scope boundaries, or success criteria. This skill does not implement code.

## When to Use

- Before any creative work — feature design, component creation, or behavior modification
- When the user says "let's brainstorm" or wants to think through options
- When requirements are vague, ambitious, or have multiple valid solutions
- When scope or direction is unclear

**When NOT to use:** Quick-help requests, factual questions, or single-step tasks that do not need a brainstorm.

## Core Principles

1. **Assess scope first** — Match ceremony to the size and ambiguity of the work.
2. **Be a thinking partner** — Suggest alternatives, challenge assumptions, and explore what-ifs instead of only extracting requirements.
3. **Resolve product decisions here** — User-facing behavior, scope boundaries, and success criteria belong in this workflow; detailed implementation belongs in planning.
4. **Keep implementation out of the design doc by default** — Omit libraries, schemas, endpoints, file layouts, or code-level design unless the brainstorm is inherently technical or architectural.
5. **Right-size the artifact** — Simple work gets a compact doc or brief alignment; larger work gets fuller structure. Skip ceremony that does not help planning.
6. **Apply YAGNI to carrying cost, not coding effort** — Prefer the simplest approach that delivers meaningful value; avoid speculative complexity, but include low-cost polish when ongoing cost stays small.

## Anti-Pattern: "This Is Too Simple To Need A Design"

Every project goes through this process. A single-function utility, a config change, a "quick fix" — all of them. "Simple" projects are where unexamined assumptions cause the most wasted work. The design can be short (a few sentences for truly simple projects), but you MUST present it and get approval before implementation.

## Interaction Rules

1. **Ask one question at a time** — Do not batch several unrelated questions into one message.
2. **Prefer single-select multiple choice** — Use `AskQuestion` (Cursor) when choosing one direction, one priority, or one next step.
3. **Use multi-select rarely and intentionally** — Only for compatible sets (goals, constraints, non-goals, success criteria that can coexist). If prioritization matters, follow up with which item is primary.
4. **Use the platform question tool** — In Cursor, prefer `AskQuestion` for blocking multiple-choice; otherwise numbered options in chat and wait for a reply before proceeding.

## Output Rules (MANDATORY — read before anything else)

Brainstorming produces **exactly one artifact**: a design document.

- **Output path**: `docs/brainstorms/YYYY-MM-DD-<topic>-design.md` in the current workspace
- **NEVER** write design documents into skill, agent, or command implementation directories
- **NEVER** create implementation files (SKILL.md, scripts, configs) during brainstorming
- If another skill is co-attached (e.g. `create-skill`), brainstorming runs first and
  produces only the design document. Implementation files are created in a later stage.
- Phase 5 writes the design doc. Gate A approves the scope. Only after Gate A can
  implementation begin — in a **separate session**.
- **Keep outputs concise** — short sections and bullets; only enough detail for the next decision.
- **Use repo-relative paths** — When referencing files in the design doc, use paths relative to the repo root, never absolute paths. Absolute paths break portability across machines and worktrees.

Create TodoWrite todos from this workflow:

```
TodoWrite todos:
  - id: "bs-1", content: "Phase 1: Understand the idea (questions + constraints)", status: "pending"
  - id: "bs-2", content: "Phase 2: Research (dispatch agents)", status: "pending"
  - id: "bs-3", content: "Phase 3: Explore approaches (2-3 options)", status: "pending"
  - id: "bs-4", content: "Phase 4: Present design (sections)", status: "pending"
  - id: "bs-5", content: "Phase 5: Document + self-review + quality review", status: "pending"
  - id: "bs-6", content: "Phase 6: Gate A approval", status: "pending"

Note: These `bs-` prefixes match the orchestrator's TodoWrite policy for Stage 1.
```

## The Process

### Phase 0: Task File, Resume, Assess, and Route

#### 0.1 Task file

1. If invoked by `/run-ce-workflow`: task file already exists — read `docs/tasks/<task-id>.yaml`
2. If invoked standalone: create `docs/tasks/<task-id>.yaml` from template, ask user for task-id

#### 0.1b Classify task domain

Before proceeding, classify whether this is a software task. The key question: **does the task involve building, modifying, or architecting software?** — not whether the task _mentions_ software topics.

- **Software** (continue to 0.2) — the task references code, repositories, APIs, databases, or asks to build/modify/debug/deploy software.
- **Non-software brainstorming** (route to universal brainstorming) — BOTH conditions must be true: none of the software signals above are present AND the task describes something the user wants to explore, decide, or think through in a non-software domain. Read [references/universal-brainstorming.md](./references/universal-brainstorming.md) and use those facilitation principles instead of the software phases below.
- **Neither** (respond directly, skip all phases) — the input is a quick-help request, error message, factual question, or single-step task that doesn't need a brainstorm.

#### 0.2 Resume existing work

If the user references an existing topic or document, or there is an obvious recent matching `*-design.md` in `docs/brainstorms/`:

- Read the document
- Confirm: "Found an existing design doc for [topic]. Continue from this, or start fresh?"
- If resuming, summarize state briefly, continue from existing decisions and outstanding questions, and update that document instead of duplicating

#### 0.3 Short-circuit when requirements are already clear

**Clear-requirements indicators:** specific acceptance criteria, referenced patterns, exact expected behavior, constrained well-defined scope.

If already clear: keep the interaction brief — confirm understanding and offer concise next steps rather than forcing a long brainstorm. Only write a short design doc when a durable handoff to planning would help. You may skip heavy Phase 1 exploration and jump to lightweight confirmation, research (if still needed), or approaches.

**Hard constraint — "Resolve before planning" questions:** Even when short-circuiting, if your analysis identifies questions that would be tagged `Resolve before planning` in the design doc (scope-affecting decisions, default behavior choices, safety trade-offs), you MUST ask them via Phase 1.3 collaborative dialogue BEFORE writing the design document. Do not defer product decisions to planning by labeling them "Outstanding Questions" without first giving the user the chance to decide.

#### 0.4 Classify scope

From the feature description plus a light repo scan:

- **Lightweight** — small, well-bounded, low ambiguity
- **Standard** — normal feature or bounded refactor with decisions to make
- **Deep** — cross-cutting, strategic, or highly ambiguous

If scope is unclear, ask one targeted question, then proceed.

**Feature description:** If the user passed slash-arg / `argument-hint` text, use it; if empty, ask once: "What would you like to explore? Describe the feature, problem, or improvement." Do not proceed without a minimal feature description.

### Phase 1: Understanding the Idea

#### 1.1 Context scan (match depth to scope)

**Lightweight** — Search for the topic, check for similar existing work, move on.

**Standard and Deep:**

- _Constraint check_ — Read `AGENTS.md` and `.cursor/rules/` for workflow, product, or scope constraints. If they add nothing, move on.
- _Topic scan_ — Search relevant terms; read the most relevant artifact (brainstorm, plan, spec, skill, feature doc); skim adjacent examples.

If nothing obvious appears after a short scan, say so and continue. Avoid deep inspection of tests, migrations, deployment, or low-level architecture unless the brainstorm is itself about a technical decision.

Two rules govern technical depth during the scan:

1. **Verify before claiming** — When the brainstorm touches checkable infrastructure (database tables, routes, config files, dependencies, model definitions), read the relevant source files to confirm what actually exists. Any claim that something is absent — a missing table, an endpoint that doesn't exist, a dependency not in the build file, a config option with no current support — must be verified against the codebase first; if not verified, label it as an unverified assumption.

2. **Defer design decisions to planning** — Implementation details like schemas, migration strategies, endpoint structure, or deployment topology belong in planning, not here — unless the brainstorm is itself about a technical or architectural decision.

#### 1.2 Product pressure test (before approaches)

Match depth to scope. **Lightweight:** real user problem? duplicating existing coverage? clearly better near-zero-cost framing? **Standard:** right problem vs proxy? outcome that matters? what if we do nothing? highest-leverage move (as framed, reframe, adjacent addition, simplify, nothing)? **Deep:** add durable capability in 6-12 months and whether this advances it vs local patch.

Use results to sharpen the conversation, not to override user intent.

#### 1.3 Collaborative dialogue

- **Ask what they're already thinking** — Before offering your own ideas, find out what the user has considered, tried, or rejected. This surfaces hidden context and prevents fixation on AI-generated framings.
- Check project state (files, docs, recent commits) as needed
- Ask **one question at a time**; use `AskQuestion` for multiple-choice (see Interaction Rules)
- Start broad (problem, users, value), then narrow (constraints, exclusions, edge cases)
- Focus on purpose, constraints, success criteria; make behavior concrete enough that planning will not need to invent it
- Bring alternatives and challenges, not only questions

**Working in existing codebases:**

- Explore the current structure before proposing changes. Follow existing patterns.
- Where existing code has problems that affect the work (e.g., a file that's grown too large, unclear boundaries, tangled responsibilities), include targeted improvements as part of the design — the way a good developer improves code they're working in.
- Don't propose unrelated refactoring. Stay focused on what serves the current goal.

**Exit:** Continue until the idea is clear or the user wants to proceed.

### Phase 2: Research

Dispatch research agents in parallel via Task tool:

| Agent                       | Purpose                                                       | When                                                                |
| --------------------------- | ------------------------------------------------------------- | ------------------------------------------------------------------- |
| `repo-research-analyst`     | Explore codebase structure, existing patterns                 | Always                                                              |
| `best-practices-researcher` | Industry best practices for the approach                      | Always                                                              |
| `learnings-researcher`      | Search the **knowledge** learnings map for past solutions | Always (skip if `~/.cursor/plugins/local/knowledge/` not found) |
| `git-history-analyzer`      | Why existing code is the way it is                            | When modifying existing code                                        |
| `framework-docs-researcher` | Library/framework documentation                               | When new dependencies involved                                      |

Also run GitNexus `context` / `impact` if modifying existing code.

Collect research summaries and present key findings to user before proposing approaches.

If any research agent returns no results or times out, note the gap and proceed — don't block the entire brainstorm on one missing source.

### Phase 3: Exploring Approaches

If multiple plausible directions remain, propose **2-3 concrete approaches** from research and conversation; otherwise state the recommended direction directly.

Use at least one **non-obvious angle** — inversion (what if we did the opposite?), constraint removal (what if X weren't a limitation?), or analogy from how another domain solves this. The first approaches that come to mind are usually variations on the same axis.

When useful, include one **challenger option**: an adjacent addition or reframe that increases usefulness, compounding value, or durability without disproportionate **carrying cost**. Present it beside the baseline, not as default; omit when work is already over-scoped or the baseline is clearly right.

**Presentation strategy (scope-dependent):**

- **Lightweight** — Lead with your recommendation and explain why. Fast decision for bounded work.
- **Standard and Deep** — Present all approaches first, then evaluate. Let the user see all options before hearing which one is recommended — leading with a recommendation before the user has seen alternatives anchors the conversation prematurely. State your recommendation after presenting all options.

For each approach: brief description (2-3 sentences), pros/cons, key risks/unknowns, when it fits best. Prefer simpler options when extra complexity creates real carrying cost; do not reject low-cost, high-value polish solely for minimalism.

If one approach is clearly best, skip the full menu and state the recommendation.

If relevant, label each approach as **reuse** (existing pattern), **extend** (existing capability), or **net-new**.

Use `AskQuestion` with labeled options; include "Need more info before deciding" when choices are complex.

**YAGNI ruthlessly** — remove unnecessary features from all designs (carrying cost, not just coding effort).

### Phase 4: Presenting the Design

- Present the design in sections of 200-300 words
- **Do NOT use `AskQuestion` for design sections** (it hides preceding text in Cursor)
- End each section with: "Does this look right? (continue / needs changes / go back)"
- Cover: architecture, components, data flow, error handling, testing

**Design for isolation and clarity:**

- Break the system into smaller units that each have one clear purpose, communicate through well-defined interfaces, and can be understood and tested independently.
- For each unit, you should be able to answer: what does it do, how do you use it, and what does it depend on?
- Can someone understand what a unit does without reading its internals? Can you change the internals without breaking consumers? If not, the boundaries need work.
- Smaller, well-bounded units are also easier for agents to work with — agents reason better about code they can hold in context at once, and edits are more reliable when files are focused.

**Visual companion:** When presenting design sections that involve visual content (UI mockups, architecture diagrams, layout comparisons), use Cursor's browser MCP canvas to show them visually. The test: **would the user understand this better by seeing it than reading it?** See [visual-companion.md](./references/visual-companion.md) for detailed guidance.

Ensure the final **design document** includes at minimum:

- `## Problem Frame`
- `## Success Criteria` — how we will know the right problem is solved
- `## Requirements` — for **Standard** and **Deep** work, use stable IDs (**R1**, **R2**, ...) grouped under bold topic headers when multiple distinct concerns exist (numbering does not restart per group; no duplicate requirements across groups). For very small docs (1-3 simple items), plain bullets are acceptable.
- `## Scope Boundaries`
- `## Outstanding Questions` — split **Resolve before planning** vs **Deferred to planning** (tag deferred items e.g. `[Technical]`, `[Needs research]` when useful)

Include when useful: key decisions/rationale, dependencies/assumptions, alternatives considered, high-level technical direction only when inherently part of the product/architecture decision.

**Before finishing Phase 4 / writing the file, check:**

- What would planning still have to invent if this brainstorm stopped now?
- Do any requirements conflict with stated non-goals?
- Are unresolved items product decisions vs planning questions?
- Did implementation detail leak in unnecessarily?
- Is there a low-cost change that would materially improve usefulness?

If planning would still need to invent behavior, scope boundaries, or success criteria, the brainstorm is not complete.

Be ready to go back and clarify.

**Related prompts:** [spec-document-reviewer-prompt.md](./references/spec-document-reviewer-prompt.md), [visual-companion.md](./references/visual-companion.md), [scripts/](./scripts/) for optional tooling.

### Phase 5: Document, Self-Review & Quality Review

When a design document was created or updated:

#### 5.1 Write the document

1. Write validated design to `docs/brainstorms/YYYY-MM-DD-<topic>-design.md` (ensure `docs/brainstorms/` exists)
2. Update task file: `paths.design: <path>`

#### 5.2 Spec self-review (inline, fix immediately)

After writing the document, look at it with fresh eyes:

1. **Placeholder scan** — Any "TBD", "TODO", incomplete sections, or vague requirements? Fix them.
2. **Internal consistency** — Do any sections contradict each other? Does the architecture match the feature descriptions?
3. **Scope check** — Is this focused enough for a single implementation plan, or does it need decomposition?
4. **Ambiguity check** — Could any requirement be interpreted two different ways? If so, pick one and make it explicit.
5. **Claim verification** — Do any requirements claim infrastructure is absent without that claim having been verified against the codebase? If so, verify now or label as an unverified assumption.

Fix any issues inline. No need to re-review — just fix and move on.

#### 5.3 User review gate

After self-review, ask the user to review the written document before proceeding:

> "Design doc written to `<path>`. Please review it and let me know if you want to make any changes before we continue."

Wait for the user's response. If they request changes, make them and re-run the self-review. Only proceed once the user approves.

#### 5.4 Formal document review

1. Dispatch the **`document-reviewer`** subagent via Task on the design doc — pass **absolute path** and context such as "brainstorm design document". The agent returns a structured `PASS/FAIL` verdict, which maps directly to the `document_review_passed` gate artifact.
2. If the reviewer returns **FAIL** (blocking findings), surface the findings so the user can fix before proceeding. Minor findings (suggested improvements) do not block.
3. When review returns **PASS**, continue to Phase 6. If the user wants a deeper multi-persona review, offer **Run additional document review** using the `document-review` skill (see Phase 6) — but the gate artifact is always set by the lightweight `document-reviewer` agent verdict.
4. Update task file: `stages.brainstorm.gate_a.artifacts.design_doc_exists: true`
5. Update task file: `stages.brainstorm.gate_a.artifacts.document_review_passed: true/false`

### Phase 6: Gate A and Handoff

#### 6.1 Gate A (scope approval)

Compute gate artifacts. Check `gate_a.policy` in task YAML before presenting:

- If `policy == auto_if_pass` and all artifacts are `true`: auto-approve (write `gate_a.status: approved`, `gate_a.date`, `gate_a.auto_approved: true`), skip AskQuestion.
- If `policy == auto`: always auto-approve regardless of artifacts.
- Otherwise (`manual` or `auto_if_pass` with any artifact `false`): present via AskQuestion:
  - Design document: exists / missing
  - Document review: passed / has issues (passed = no P0/P1 findings remaining)
  - Options: **Approve scope** / **Needs revision** / **Reject**

**If `Resolve before planning` (in the design doc) is non-empty:** by default work those questions before offering "Proceed to planning"; if the user insists on proceeding, convert each item to an explicit decision, assumption, or **Deferred to planning** entry first. Do not treat handoff as complete while blocking product questions remain unresolved by explicit decision.

On **Approve scope**: update `gate_a.status: approved`, `gate_a.date`, advance `status: arch_plan`. Implementation stays in a **separate session** after Gate A.

#### 6.2 Next-step menu (after Gate A or when brainstorm loop completes)

Present only options that apply, using `AskQuestion` when appropriate:

- **Proceed to planning (recommended)** — Continue with the **writing-arch-plans** skill (load and follow `writing-arch-plans/SKILL.md`). Pass the design document path when one exists; otherwise pass a concise summary of finalized decisions. This is the standard next step after brainstorming.
- **Proceed directly to work** — Offer only when scope is lightweight, success criteria and boundaries are clear, and no meaningful technical/research blockers remain; otherwise omit.
- **Run additional document review** — When a design doc exists: rerun **document-reviewer** agent for lightweight gate, or use **document-review** skill for deeper multi-persona analysis; when review completes, return to this menu without a closing summary yet.
- **Ask more questions** — Return to Phase 1.3; continue one question at a time; when satisfied, return to Phase 6 options (no closing summary yet).
- **Done for now** — Pause; user can resume later.

#### 6.3 Closing summary

Use only when ending this workflow run (not when cycling back to the menu):

```text
Brainstorm complete!

Design doc: docs/brainstorms/YYYY-MM-DD-<topic>-design.md  # if created

Key decisions:
- [Decision 1]
- [Decision 2]

Recommended next step: writing-arch-plans skill (architecture planning)
```

## Workflow reminders

- **Research first** — Complete Phase 2 agent + GitNexus research before locking approaches.
- **Be flexible** — Go back and clarify when something does not make sense.
