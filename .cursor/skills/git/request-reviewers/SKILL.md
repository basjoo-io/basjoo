---
name: request-reviewers
description: >-
  Request PR reviewers based on personal config. Use when the user says
  "拉人review", "request reviewers", "加reviewer", or explicitly references
  @request-reviewers or /request-reviewers.
---

# Request PR Reviewers

**Core principle:** Personal config drives who to request; READABILITY file gates readability reviewers.

**Announce at start:** "I'm using the request-reviewers skill to request PR reviewers."

**Activation:** When your human partner asks to add reviewers, or when called by another skill (e.g., publishing-pr). Do NOT auto-trigger without explicit intent.

## When to Use

- Your human partner says "拉人review", "request reviewers", "加reviewer"
- Called by publishing-pr after PR creation
- Explicitly referenced via `@request-reviewers` or `/request-reviewers`

## When NOT to Use

- PR doesn't exist yet (create it first with publishing-pr)
- Reviewing code yourself (use reviewing-code)

---

## Personal Config

**Location:** `.cursor/skills/personal/reviewers.yaml`

This file is gitignored. Each team member creates their own.

**Format:**

```yaml
readability:
  enabled: true
  rules:
    # Rules are evaluated in order. First matching rule wins per file.
    # Each rule: extensions (required), paths (optional filter), preferred (required).
    - extensions: [.py]
      paths: [src/]
      preferred: [alice]
    - extensions: [.py]
      preferred: [bob]
    - extensions: [.h, .cc, .cpp, .hpp, .cu]
      preferred: [charlie]
    - extensions: [.js, .jsx, .ts, .tsx]
      preferred: [alice]
    - extensions: [.proto]
      preferred: [dave]

ownership:
  .cursor/skills/:
    - alice
    - bob
  src/:
    - charlie
    - dave
```

**No config file = no reviewers requested.** The skill reports this and exits.

### Readability Rules

Rules are evaluated **in order, first match wins** per changed file:

1. For each changed file, find the first rule where the file's extension matches AND (if `paths` is set) the file path starts with one of the listed prefixes
2. A rule without `paths` matches all files with the given extensions
3. The `preferred` reviewers from the matched rule are collected

### Reviewer Types

| Type            | Source of Truth                | Personal Config Role            | Gate                                                   |
| --------------- | ------------------------------ | ------------------------------- | ------------------------------------------------------ |
| **Readability** | `READABILITY` file (repo root) | Lists preferred reviewers       | Must appear in READABILITY for the relevant file types |
| **Ownership**   | Personal config only           | Defines path → reviewer mapping | No external gate                                       |

---

## Workflow

**Create TodoWrite todos from these steps:**

| Step | Action              | Description                                         |
| ---- | ------------------- | --------------------------------------------------- |
| 1    | Identify PR & files | Get PR number and changed file list                 |
| 2    | Read config         | Load personal reviewers.yaml                        |
| 3    | Resolve readability | Match file types → READABILITY → personal preferred |
| 4    | Resolve ownership   | Match file paths → personal ownership mapping       |
| 5    | Mark ready          | If PR is draft, mark as ready for review            |
| 6    | Request & report    | Deduplicate, request, summarize                     |

### Step 1: Identify PR & Changed Files

```bash
PR_NUMBER=$(gh pr view --json number -q .number 2>/dev/null || echo "")
```

If empty, report "No PR found for current branch" and stop.

```bash
PR_AUTHOR=$(gh pr view $PR_NUMBER --json author -q .author.login)
CHANGED_FILES=$(gh pr view $PR_NUMBER --json files -q '.files[].path')
```

### Step 2: Read Personal Config

**Read** `.cursor/skills/personal/reviewers.yaml`.

If missing, report with setup instructions and stop:

```
No personal reviewer config found at .cursor/skills/personal/reviewers.yaml
Skipping reviewer request. Create the file — see SKILL.md for format.
```

### Step 3: Resolve Readability Reviewers

Skip if `readability.enabled` is not `true`.

1. **Read** the `READABILITY` file at repo root and parse extension → eligible reviewer mapping
2. For each changed file:
   a. Find the **first matching rule** from `readability.rules` (extension match + optional path match)
   b. Collect the rule's `preferred` reviewers
3. **Gate check**: For each preferred reviewer, verify they appear in the READABILITY file for at least one matching extension. Skip with warning if not.
4. **Exclude** PR author

**Gate rule:** A preferred reviewer is only requested if they appear in the READABILITY file for the relevant file types. Reviewers in personal config but NOT in READABILITY are skipped with a warning.

### Step 4: Resolve Ownership Reviewers

For each path prefix in `ownership`:

1. Check if any changed file starts with that prefix
2. If matched, add the configured reviewers
3. **Exclude** PR author

No external gate — any GitHub user can be listed.

### Step 5: Mark Ready for Review

If the PR is a draft, mark it as ready:

```bash
gh pr ready $PR_NUMBER
```

If already ready, skip.

### Step 6: Request & Report

1. **Merge** readability and ownership reviewer lists (deduplicate)
2. **Request** all reviewers via GitHub API:

```bash
OWNER_REPO=$(gh repo view --json nameWithOwner -q .nameWithOwner)
gh api "repos/$OWNER_REPO/pulls/$PR_NUMBER/requested_reviewers" \
  -X POST \
  -f 'reviewers[]=reviewer1' \
  -f 'reviewers[]=reviewer2' \
  -f 'reviewers[]=reviewer3'
```

3. **Report:**

```markdown
## Reviewer Request Summary — PR #<number>

| Type        | Reviewer | Status                             |
| ----------- | -------- | ---------------------------------- |
| Readability | alice    | ✅ Requested                       |
| Readability | bob      | ⏭️ Skipped (author)                |
| Readability | eve      | ⚠️ Not in READABILITY              |
| Ownership   | charlie  | ✅ Requested                       |
| Ownership   | alice    | ✅ Already requested (readability) |
```

---

## Integration

**Called by:**

- **finishing-a-development-branch** — invoked after PR is published and CI passes
- Manual invocation
