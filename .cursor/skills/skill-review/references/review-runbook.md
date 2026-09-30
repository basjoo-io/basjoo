## Review Runbook — How to Review a Skill

Use this runbook when asked to review any `.cursor/skills/<name>/SKILL.md` file.
Follow all 4 steps in order. Do NOT skip steps.

---

### Step 1: Load the Section Index

You are already reading the `skill-review` skill. The Section Index in `SKILL.md`
lists all available reference files. Proceed to Step 2.

---

### Step 2: Select Reference Files by Review Dimension

Load ONLY the files relevant to your review scope:

| Review Dimension                        | File to Load                             |
| --------------------------------------- | ---------------------------------------- |
| Is the description well-written?        | `references/05-description-writing.md`   |
| Is the file structure correct?          | `references/03-file-structure.md`        |
| Are core principles followed?           | `references/04-core-principles.md`       |
| Are Cursor features used correctly?     | `references/08-cursor-features.md`       |
| Are there anti-patterns?                | `references/09-anti-patterns.md`         |
| Is it release-ready?                    | `references/11-pre-release-checklist.md` |
| Are examples clear and realistic?       | `references/examples.md`                 |
| How does it compare to real skills? | `references/examples.md`                 |

**Rule**: Load at most 3 files. If the review scope is narrow, load 1–2.

---

### Step 3: Score Against Checklist

After loading relevant reference files, evaluate the target skill:

**3.1 Core Quality** (from `references/11-pre-release-checklist.md`)

- [ ] YAML frontmatter has `name`, `description`
- [ ] `description` follows "Use when... Triggers: ..." formula
- [ ] `description` does NOT summarize the workflow (anti-pattern §9.7)
- [ ] Skill is ≤ 500 lines, or uses `references/` subdirectory pattern

**3.2 Structure** (from `references/03-file-structure.md`)

- [ ] Stored at `.cursor/skills/<skill-name>/SKILL.md` (not in `~/.cursor/skills-cursor/`)
- [ ] Has recommended sections (Overview / When to Use / Workflow / Quick Reference)

**3.3 Agent-Native Parity**

- [ ] All referenced paths are local (no external URLs agents cannot access)
- [ ] Steps use imperative mood ("Read X", "Call Y"), not passive ("X can be read")
- [ ] Gate conditions are explicit — no ambiguous "if needed" or "optionally" on required steps

**3.4 Anti-Patterns** (from `references/09-anti-patterns.md`)

- [ ] No narrative storytelling content (§9.1)
- [ ] No excessive options without a default recommendation (§9.2)
- [ ] No Windows-style paths (§9.4)
- [ ] `description` does not contain workflow steps (§9.7)

---

### Step 4: Output Review Report

Structure your output using this template:

```markdown
## Skill Review: <skill-name>

**Overall Rating**: PASS | NEEDS WORK | FAIL

| Severity | Issue | Location | Suggested Fix |
| -------- | ----- | -------- | ------------- |
| 🔴 HIGH  |       |          |               |
| 🟡 MED   |       |          |               |
| 🟢 LOW   |       |          |               |

## **Positive patterns worth keeping**:

**Top 3 Fixes (priority order)**:

1.
2.
3.
```

**Rating guide**:

- `PASS`: 0 HIGH, ≤ 2 MED issues
- `NEEDS WORK`: 1–2 HIGH or > 2 MED issues
- `FAIL`: > 2 HIGH issues or skill is not usable by agents
