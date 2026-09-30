---
name: address-pr-comments
description: >-
  Address PR review comments via gh CLI.
  Use when your human partner says "处理评论", "fix PR comments", "回复review", "address review feedback",
  or explicitly references @address-pr-comments or /address-pr-comments.
---

# Address PR Review Comments

**Prerequisites:** Requires `gh` CLI authenticated (`gh auth login`).

**Core principle:** Triage before action — understand all comments first, then batch-fix systematically.

**Announce at start:** "I'm using the address-pr-comments skill to address PR review comments."

**Activation:** Do NOT auto-trigger on bare keywords like "comment" or "review" without explicit fix intent.

## When to Use

- PR has unaddressed review comments that need fixing or replying
- Your human partner shares a PR URL or says "fix comments on PR #XXX"
- After receiving code review feedback that requires code changes

## When NOT to Use

- Performing a code review as reviewer (use **code-review** skill)
- Creating a new PR (use **publishing-pr**)
- Resolving merge/rebase conflicts (use **fixing-rebase-conflict**)

---

## Workflow

**Create TodoWrite todos from these steps:**

| Step | Action                | Description                                       |
| ---- | --------------------- | ------------------------------------------------- |
| 1    | Identify PR           | Get PR number from URL or current branch          |
| 2    | Fetch review comments | Get all review comments via gh API                |
| 3    | Triage comments       | Classify by source (bot/human) and status         |
| 4    | Present report        | Bot comments auto-processed; human comments gated |
| 5    | Fix & reply           | Fix code, reply, or mark won't-fix                |
| 6    | Commit & push         | Group commits logically, then push                |
| 7    | Resolve threads       | Mark addressed threads as resolved via GraphQL    |
| 8    | Summary               | Report completion status                          |

---

## Step 1: Identify PR

**Option A** — PR number given (e.g., URL `https://github.com/the project/the project/pull/3269`):
Extract the number directly.

**Option B** — No PR number given:

```bash
gh pr view --json number -q .number
```

Store PR number and repo info for subsequent steps:

```bash
PR_NUMBER=<number>
OWNER_REPO=$(gh repo view --json nameWithOwner -q .nameWithOwner)
```

## Step 2: Fetch Review Comments

Fetch all three comment sources:

### 2a) Inline code review comments

```bash
gh api "repos/$OWNER_REPO/pulls/$PR_NUMBER/comments" --paginate
```

Each comment has:

| Field            | Meaning                              |
| ---------------- | ------------------------------------ |
| `id`             | Comment ID (use for `in_reply_to`)   |
| `in_reply_to_id` | Parent comment ID (null = top-level) |
| `path`           | File path                            |
| `line`           | Line number in the diff              |
| `body`           | Comment text                         |
| `user.login`     | Author                               |

### 2b) General PR discussion comments (issue comments)

```bash
gh api "repos/$OWNER_REPO/issues/$PR_NUMBER/comments" --paginate
```

### 2c) Review body comments

Reviewers can leave a top-level comment attached to a review submission (visible in the PR timeline as "X reviewed ... and left a comment"). These are **not** returned by the inline comments or issue comments APIs — they live on the review object itself.

```bash
gh api "repos/$OWNER_REPO/pulls/$PR_NUMBER/reviews" --paginate
```

Filter for reviews where `body` is non-empty and `state` is `COMMENTED` or `CHANGES_REQUESTED`. Each review has:

| Field        | Meaning                                            |
| ------------ | -------------------------------------------------- |
| `id`         | Review ID                                          |
| `body`       | Review body comment (may be empty)                 |
| `state`      | `COMMENTED`, `APPROVED`, `CHANGES_REQUESTED`, etc. |
| `user.login` | Reviewer                                           |

**Replying:** Review body comments don't support `in_reply_to` threading. Reply via an issue comment (Step 5e) quoting the original text.

## Step 3: Triage Comments

### 3a. Classify by Source

| Source    | Identification                                                                                           | Processing                        |
| --------- | -------------------------------------------------------------------------------------------------------- | --------------------------------- |
| **Bot**   | `user.login` ends with `[bot]` or is a known bot (e.g., `cursor[bot]`, `github-actions[bot]`, `copilot`) | Auto-process without confirmation |
| **Human** | All other authors                                                                                        | Requires AskQuestion confirmation |

### 3b. Group into Threads

Group comments using `in_reply_to_id` (all replies sharing the same top-level parent form one thread).

### 3c. Find Unaddressed Comments

**Read the full thread** and use semantic analysis to determine what is still actionable. Do NOT rely on mechanical rules like "check last commenter" — reviewers may post multiple comments where later ones override earlier ones.

| Scenario                                                                | Result               |
| ----------------------------------------------------------------------- | -------------------- |
| Reviewer: "Fix variable name" → no reply                                | Unaddressed          |
| Reviewer: "Fix this" → author replied with `[bot]` marker               | Addressed            |
| Reviewer: "Fix A" → author replied → reviewer: "Still wrong"            | Unaddressed          |
| Reviewer: "Fix A" → reviewer: "Fix B" → reviewer: "Ignore B, it's fine" | Only A is actionable |
| Author self-review: "TODO: refactor this" → no reply                    | Unaddressed          |

**Distinguishing agent replies from human comments**: Agent replies are prefixed with `🤖 [bot]`. Treat `🤖 [bot]`-prefixed replies as agent responses, non-prefixed comments from the same account as human-authored.

### 3d. Determine Action

For each unaddressed comment, **Read** the referenced file and line:

| Status               | Meaning                                                                |
| -------------------- | ---------------------------------------------------------------------- |
| 🔧 **Needs fix**     | Code still has the issue                                               |
| ✅ **Already fixed** | Code already addresses the concern                                     |
| 💬 **Needs reply**   | Question or design discussion, no code change needed                   |
| 🚫 **Won't fix**     | False positive, disagree with suggestion, or intentional design choice |

## Step 4: Present Report & Confirm

### Bot comments → auto-process

Bot comments (bugbot, copilot, etc.) are processed automatically without asking. Proceed directly to Step 5.

### Human comments → AskQuestion gate

```markdown
## PR #XXX Review Comments Report

### Unaddressed Comments (N total)

| #   | Source   | Author      | File:Line   | Status       | Summary     |
| --- | -------- | ----------- | ----------- | ------------ | ----------- |
| 1   | 🤖 Bot   | cursor[bot] | file.py:42  | 🔧 Needs fix | Description |
| 2   | 👤 Human | reviewer    | file.py:88  | 🔧 Needs fix | Description |
| 3   | 👤 Human | reviewer    | file.py:100 | 🚫 Won't fix | Description |
```

```json
AskQuestion({
  "title": "Select comments to address",
  "questions": [{
    "id": "comment_selection",
    "prompt": "Bot comments (#1) will be auto-processed. For human comments (#2, #3), how to proceed?",
    "options": [
      {"id": "all", "label": "Address all"},
      {"id": "select", "label": "Let me choose"},
      {"id": "cancel", "label": "Cancel"}
    ]
  }]
})
```

## Step 5: Fix & Reply

**All replies MUST start with `🤖 [bot]` prefix** to distinguish agent replies from human comments during future triage.

### 5a. Already Fixed → Reply Only

For **review comments** (inline on code):

```bash
gh api "repos/$OWNER_REPO/pulls/$PR_NUMBER/comments" \
  -X POST \
  -f body="🤖 [bot] Done. <brief explanation of what was fixed and where>" \
  -F in_reply_to=<COMMENT_ID>
```

For **issue comments** (general PR discussion), see [5e. Replying to Issue Comments](#5e-replying-to-issue-comments).

### 5b. Needs Code Fix → Fix, Then Reply

1. **Edit** the code to address the comment
2. **Verify** the fix resolves the concern
3. Reply will be sent after commit+push in Step 6

### 5c. Needs Reply Only (No Code Change)

For **review comments** (inline on code):

```bash
gh api "repos/$OWNER_REPO/pulls/$PR_NUMBER/comments" \
  -X POST \
  -f body="🤖 [bot] <your response>" \
  -F in_reply_to=<COMMENT_ID>
```

For **issue comments** (general PR discussion), see [5e. Replying to Issue Comments](#5e-replying-to-issue-comments).

### 5d. Won't Fix / False Positive

For bugbot false positives or disagreements with reviewer suggestions.

For **review comments** (inline on code):

```bash
gh api "repos/$OWNER_REPO/pulls/$PR_NUMBER/comments" \
  -X POST \
  -f body="🤖 [bot] Won't fix — <concise reason: why this is intentional or a false positive>" \
  -F in_reply_to=<COMMENT_ID>
```

For **issue comments** (general PR discussion), see [5e. Replying to Issue Comments](#5e-replying-to-issue-comments).

⚠️ **For human reviewer comments**: Only use "won't fix" if your human partner confirmed via AskQuestion. Never auto-reject human feedback.

### 5e. Replying to Issue Comments

GitHub issue comments (`/issues/PR/comments`) are **flat** — they do NOT support `in_reply_to` threading like review comments do. A bare reply appears as a standalone comment with no context, which is confusing to readers.

**MANDATORY**: When replying to an issue comment, **always quote the original comment** using `>` blockquote syntax to provide context:

```bash
gh api "repos/$OWNER_REPO/issues/$PR_NUMBER/comments" \
  -X POST \
  -f body='> <quoted original comment text>

🤖 [bot] <your response>'
```

**Guidelines:**

- Quote the full original comment if short; for long comments, quote the key question/request
- Keep the `🤖 [bot]` prefix after the blockquote for agent reply identification
- If the original comment is very long, truncate with `...` but keep the core question

### Handling "Pending Review" Errors

If replying returns a **422 error** about pending reviews, see [troubleshooting.md](troubleshooting.md#pending-review-422-error).

## Step 6: Commit & Push

**Skip** if no code changes were made (reply-only comments).

### Commit Granularity

- **Single topic** (all comments about the same issue): one commit
- **Multiple unrelated topics**: group by logical change, one commit per group
- Use descriptive scope in the commit message:

```bash
git add <related files>
git commit -m "$(cat <<'EOF'
fix(review): <specific change description>

Addresses PR comment from <reviewer>: <brief summary>
EOF
)"
```

Repeat for each logical group. Then format and push all at once:

```bash
clang-format -i <changed files>
git add <changed files>
git diff --cached --quiet || git commit -m "style: format code"
git push --force-with-lease
```

⚠️ If push is rejected (non-fast-forward), **NEVER** use `git pull --rebase` — it pulls in unrelated commits and pollutes the PR. The branch is single-author, `--force-with-lease` is safe.

After push, reply to each fixed comment with the commit hash:

```bash
gh api "repos/$OWNER_REPO/pulls/$PR_NUMBER/comments" \
  -X POST \
  -f body="🤖 [bot] Fixed in <commit_hash>. <brief explanation>" \
  -F in_reply_to=<COMMENT_ID>
```

## Step 7: Resolve Threads

After replying to each addressed comment, resolve its review thread:

```bash
OWNER=$(echo "$OWNER_REPO" | cut -d/ -f1)
REPO=$(echo "$OWNER_REPO" | cut -d/ -f2)

THREAD_ID=$(gh api graphql -f query='
query {
  repository(owner: "'"$OWNER"'", name: "'"$REPO"'") {
    pullRequest(number: '$PR_NUMBER') {
      reviewThreads(first: 100) {
        nodes {
          id
          isResolved
          comments(first: 1) { nodes { databaseId } }
        }
      }
    }
  }
}' --jq '.data.repository.pullRequest.reviewThreads.nodes[]
  | select(.comments.nodes[0].databaseId == '$COMMENT_ID') | .id')

gh api graphql -f query='
mutation {
  resolveReviewThread(input: {threadId: "'"$THREAD_ID"'"}) {
    thread { isResolved }
  }
}'
```

Skip resolution for "won't fix" comments from human reviewers — let the reviewer decide whether to resolve.

## Step 8: Summary

Present completion report:

```markdown
## PR #XXX Comments Resolution Summary

| #   | Source   | Comment             | Action              | Status       |
| --- | -------- | ------------------- | ------------------- | ------------ |
| 1   | 🤖 Bot   | cursor[bot]: "desc" | Fixed + resolved    | ✅ Done      |
| 2   | 👤 Human | reviewer: "desc"    | Fixed + resolved    | ✅ Done      |
| 3   | 👤 Human | reviewer: "desc"    | Won't fix + replied | 🚫 Won't fix |
| 4   | 👤 Human | reviewer: "desc"    | Skipped             | ⏭️ Skipped   |
```

---

## Quick Reference: gh API Commands

| Action                  | Command                                                                                                                                  |
| ----------------------- | ---------------------------------------------------------------------------------------------------------------------------------------- |
| Fetch inline comments   | `gh api repos/OWNER/REPO/pulls/PR/comments --paginate`                                                                                   |
| Fetch issue comments    | `gh api repos/OWNER/REPO/issues/PR/comments --paginate`                                                                                  |
| Fetch review bodies     | `gh api repos/OWNER/REPO/pulls/PR/reviews --paginate` (filter non-empty `body`)                                                          |
| Reply to review comment | `gh api repos/OWNER/REPO/pulls/PR/comments -X POST -f body="..." -F in_reply_to=ID`                                                      |
| Reply to issue comment  | `gh api repos/OWNER/REPO/issues/PR/comments -X POST -f body="> quote\n\n🤖 [bot] reply"` (⚠️ must quote original — no threading support) |
| Submit pending review   | `gh api repos/OWNER/REPO/pulls/PR/reviews/REVIEW_ID/events -X POST -f event="COMMENT"`                                                   |
| Resolve thread          | `gh api graphql -f query='mutation { resolveReviewThread(...) }'`                                                                        |
| Get PR info             | `gh pr view PR --json number,title,body,author`                                                                                          |
