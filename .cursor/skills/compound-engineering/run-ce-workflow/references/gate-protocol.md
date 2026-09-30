# Gate Protocol

6 human gates (A–F) enforce quality checkpoints. Each gate is a Go/No-Go decision with machine-computed artifacts.

## Gate Execution

At each gate:

1. **Compute** all `artifacts` in the gate's YAML section (boolean checks)
2. **Write** results to `docs/tasks/<task-id>.yaml`
3. **Apply gate policy** (from `gate.policy`, default `manual`):
   - **manual**: Present via AskQuestion (see below)
   - **auto_if_pass**: If ALL artifacts are `true` → auto-approve: write `gate.status: approved`, `gate.date`, log `gate.auto_approved: true`, advance `status`. If ANY artifact is `false` → fall back to AskQuestion.
   - **auto**: Always auto-approve regardless of artifacts, log `gate.auto_approved: true` (use with caution).
4. **AskQuestion** (when triggered by policy):
   - Gate name and purpose
   - Artifact check results (pass/fail for each)
   - Link to relevant document
   - Options: "Approve" / "Reject (explain why)" / "Needs revision"
5. **On approve:** write `gate.status: approved`, `gate.date`, advance `status`
6. **On reject:** write `gate.status: rejected`, `gate.reason`, stay in current stage

After approval, **recommend starting a fresh session** (see [session-management.md](session-management.md)).

## Artifact Evaluation

| Artifact                     | How to evaluate                                                                                                                                                                                                                                                                                     |
| ---------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `*_doc_exists`               | `ls` the path from `paths` section in task YAML                                                                                                                                                                                                                                                     |
| `document_review_passed`     | `document-reviewer` agent returned PASS — defined as no blocking findings remaining. Minor issues do not block. **YAML path varies by stage:** Stage 1 uses `stages.brainstorm.gate_a.artifacts.document_review_passed`; Stages 2/4 use `stages.<stage>.document_review_passed` at stage level. |
| `audit_passed`               | Stage audit `status == passed`                                                                                                                                                                                                                                                                      |
| `p1_findings_zero`           | `findings_p1 == 0` in stage YAML                                                                                                                                                                                                                                                                    |
| `agent_native_review_passed` | `agent-native-reviewer` returned no blocking findings                                                                                                                                                                                                                                               |
| `tests_compile`              | `cmake --build` succeeds for test targets                                                                                                                                                                                                                                                             |
| `tests_fail_as_expected`     | `ctest` fails with expected error patterns                                                                                                                                                                                                                                                     |
| `all_tests_green`            | `ctest --preset linux-gcc-x64` passes                                                                                                                                                                                                                                                                           |
| `drift_points_zero`          | `drift_points == 0` in code_audit stage                                                                                                                                                                                                                                                             |
| `detect_changes_clean`       | GitNexus `detect_changes` shows only expected files                                                                                                                                                                                                                                                 |
| `verification_method_set`    | `stages.verify.verification_method` is not empty                                                                                                                                                                                                                                                    |
| `evidence_collected`         | `stages.verify.evidence` has at least one entry                                                                                                                                                                                                                                                     |
| `all_evidence_passing`       | Every entry in `stages.verify.evidence` has `result: pass`                                                                                                                                                                                                                                          |

## Transition Validation

Before advancing `status`, verify:

| Transition               | Required                                                                                |
| ------------------------ | --------------------------------------------------------------------------------------- |
| `brainstorm → arch_plan` | Gate A approved, design doc exists                                                      |
| `arch_plan → arch_audit` | Plan doc exists, `stages.arch_plan.document_review_passed == true`                      |
| `arch_audit → impl_plan` | Gate B approved, audit passed, p1=0                                                     |
| `impl_plan → impl_audit` | Implementation doc exists, `stages.impl_plan.document_review_passed == true`            |
| `impl_audit → tdd`       | Gate C approved, audit passed, p1=0                                                     |
| `tdd → execute`          | Gate D approved, tests compile, tests fail as expected                                  |
| `execute → code_audit`   | `stages.execute.all_checkpoints_done == true`, `stages.execute.all_tests_green == true` |
| `code_audit → ship`      | Gate E approved, audit passed, p1=0, drift=0                                            |
| `ship → verify`          | PR created + CI green, OR local merge completed, OR `stages.ship.pr == skipped`         |
| `verify → compound`      | Gate F approved                                                                         |
| `compound → done`        | `stages.compound.retro_written == true`, `stages.compound.knowledge_synced == true`     |

### Aborted Status

When `status` is set to `aborted`: task YAML is preserved for inspection. The user can resume by setting `--stage <stage>` to re-enter at any prior stage. Abort does not delete artifacts or branches.

## Task State File Contract

- **Format:** Pure YAML (not markdown with frontmatter)
- **Location:** `docs/tasks/<task-id>.yaml`
- **Naming:** `task_id` uses kebab-case, matches filename
- **Path authority:** `paths` section is the single source of truth — read paths from here, never construct from templates
- **Validation:** Before advancing status, check that target stage's gate artifacts are all `true`

## Status Field Validation

Before writing any field, validate enum values:

| Field                    | Valid values                                                                                                                                                                |
| ------------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `status`                 | `brainstorm` \| `arch_plan` \| `arch_audit` \| `impl_plan` \| `impl_audit` \| `tdd` \| `execute` \| `code_audit` \| `ship` \| `verify` \| `compound` \| `done` \| `aborted` |
| `gate.status`            | `pending` \| `approved` \| `rejected`                                                                                                                                       |
| `gate.policy`            | `manual` \| `auto_if_pass` \| `auto`                                                                                                                                        |
| `audit_triage_policy`    | `manual` \| `auto_safe` \| `auto_all`                                                                                                                                       |
| `findings[].disposition` | `reported` \| `approved` \| `fixed` \| `wontfix` \| `rejected` \| `out_of_scope`                                                                                            |
| `findings[].priority`    | `p1` \| `p2` \| `p3`                                                                                                                                                        |
| `checkpoint_policy`      | `manual` \| `auto`                                                                                                                                                          |
| `verification_method`    | `simulation` \| `integration_test` \| `manual_test`                                                                                                                         |
| `evidence[].result`      | `pass` \| `fail`                                                                                                                                                            |
