# Persona Catalog

15 reviewer personas organized into always-on, cross-cutting conditional, and stack-specific conditional layers, plus project-specific agents. The orchestrator uses this catalog to select which reviewers to spawn for each review.

## Always-on (4 personas + 2 project agents)

Spawned on every review regardless of diff content.

**Persona agents (structured JSON output):**

| Persona             | subagent_type                | Focus                                                                                     |
| ------------------- | ---------------------------- | ----------------------------------------------------------------------------------------- |
| `correctness`       | `correctness-reviewer`       | Logic errors, edge cases, state bugs, error propagation, intent compliance                |
| `testing`           | `testing-reviewer`           | Coverage gaps, weak assertions, brittle tests, missing edge case tests                    |
| `maintainability`   | `maintainability-reviewer`   | Coupling, complexity, naming, dead code, premature abstraction                            |
| `project-standards` | `project-standards-reviewer` | AGENTS.md and `.cursor/rules/` compliance -- frontmatter, references, naming, portability |

**project agents (unstructured output, synthesized separately):**

| subagent_type           | Focus                                                                            |
| ----------------------- | -------------------------------------------------------------------------------- |
| `agent-native-reviewer` | Verify new features are agent-accessible                                         |
| `learnings-researcher`  | Search docs/solutions/ for past issues related to this PR's modules and patterns |

## Cross-cutting Conditional (8 personas)

Spawned when the orchestrator identifies relevant patterns in the diff. The orchestrator reads the full diff and reasons about selection -- this is agent judgment, not keyword matching.

| Persona             | subagent_type                  | Select when diff touches...                                                                                      |
| ------------------- | ------------------------------ | ---------------------------------------------------------------------------------------------------------------- |
| `security`          | `security-reviewer`            | Auth middleware, public endpoints, user input handling, permission checks, secrets management                    |
| `performance`       | `performance-reviewer`         | Database queries, hot loops, data transforms, caching layers, async/concurrent code                              |
| `api-contract`      | `api-contract-reviewer`        | Route definitions, proto service changes, exported type signatures, API versioning                               |
| `data-migrations`   | `data-migrations-reviewer`     | Migration files, schema changes, backfill scripts, data transformations                                          |
| `reliability`       | `reliability-reviewer`         | Error handling, retry logic, circuit breakers, timeouts, background jobs, async handlers                         |
| `adversarial`       | `adversarial-reviewer`         | Diff has >=50 changed non-test/non-generated lines, OR touches auth, data mutations, external API integrations   |
| `cli-readiness`     | `cli-agent-readiness-reviewer` | CLI command definitions, argument parsing, CLI framework usage                                                   |
| `previous-comments` | `previous-comments-reviewer`   | **PR-only.** Reviewing a PR that has existing review comments or threads. Skip when no PR metadata was gathered. |

## Stack-specific Conditional (3 personas)

These reviewers keep their original opinionated lens. They are additive with the cross-cutting personas above.

| Persona                | subagent_type                   | Select when diff touches...                                                                      |
| ---------------------- | ------------------------------- | ------------------------------------------------------------------------------------------------ |
| `kieran-python`        | `kieran-python-reviewer`        | Python modules, endpoints, services, scripts, or typed domain code                               |
| `kieran-typescript`    | `kieran-typescript-reviewer`    | TypeScript components, services, hooks, utilities, or shared types                               |
| `julik-frontend-races` | `julik-frontend-races-reviewer` | Stimulus/Turbo controllers, DOM event wiring, timers, async UI flows, frontend state transitions |

## Conditional Agents (migration/deployment specific)

Spawn when the diff includes schema changes, data migrations, or deployment-sensitive components.

| subagent_type                   | Focus                                                                                        |
| ------------------------------- | -------------------------------------------------------------------------------------------- |
| `schema-drift-detector`         | Cross-references proto schema changes against the PR's stated scope to catch unrelated drift |
| `deployment-verification-agent` | Produces Go/No-Go deployment checklist with verification queries and rollback procedures     |

## Selection Rules

1. **Always spawn all 4 always-on personas** plus the 2 always-on agents.
2. **For each cross-cutting conditional persona**, the orchestrator reads the diff and decides whether the persona's domain is relevant. This is a judgment call, not a keyword match.
3. **For each stack-specific conditional persona**, use file types and changed patterns as a starting point, then decide whether the diff actually introduces meaningful work for that reviewer. Do not spawn language-specific reviewers just because one config or generated file happens to match the extension.
4. **For conditional agents**, spawn when the diff includes proto schema changes, data migration scripts, or deployment-sensitive configuration.
5. **Announce the team** before spawning with a one-line justification per conditional reviewer selected.
