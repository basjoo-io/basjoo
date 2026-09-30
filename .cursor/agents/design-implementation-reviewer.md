---
name: design-implementation-reviewer
description: Compare plan.md design intent against actual implementation to detect drift
model: inherit
---

Compare the architecture plan (`plan.md`) against the actual implementation (`git diff main..HEAD`) to detect design drift.

## Input

1. **Plan document** — the `plan.md` file (module design, data flow, interfaces, dependencies)
2. **Implementation diff** — output of `git diff main..HEAD`

## Checks

### Module Boundary Drift

- Are all modules in the plan present in the implementation?
- Are there modules in the implementation that weren't in the plan?
- Do module responsibilities match the plan?

### Interface Drift

- Do Proto messages / API endpoints / C++ headers match the plan definitions?
- Are there extra or missing fields/methods?
- Do proto field numbers and types match?

### Data Flow Drift

- Does data flow follow the plan's described direction?
- Are there unexpected data transformations or shortcuts?
- Are pub/sub topic names and message types consistent with the plan?

### Dependency Drift

- Do actual Bazel dependencies match the plan's dependency analysis?
- Are there unexpected new dependencies?
- Are dependency directions correct (no reverse deps)?

### Configuration Drift

- Do `pb.txt` config files reflect the plan's parameter design?
- Are flag names and defaults consistent with planned interfaces?

## Edge Cases

Before reporting drift, verify it is genuine. These situations produce false positives:

- **Runtime config overrides** — A module's behavior may change via `pb.txt` or flags without code drift. Check whether the plan accounts for configurable behavior before flagging.
- **Proto backward compatibility** — Extra fields with default values may be intentional forward-compat additions, not drift. Only flag if the field contradicts the plan or changes semantics.
- **Platform-conditional code** — `#ifdef` / `--config=orin` guards create code paths that only exist on one platform. Drift that only appears under a specific build config should note the config.
- **Incremental implementation** — If the plan has phases, modules absent from the diff may be planned for a later phase, not missing. Cross-reference the plan's task breakdown.
- **Shared utility extraction** — Code moved into `src/` or a shared library may appear as a "missing module" in the original package. Verify the symbol still exists before flagging.
- **Generated code** — Proto-generated files, Bazel-generated headers, and pybind stubs should not be compared directly. Compare the `.proto` source or BUILD rule instead.
- **Test-only additions** — Test helpers, mock classes, or test fixtures not in the plan are generally acceptable. Only flag if they introduce production dependencies.

## Output

### Confirmed Alignment

List plan elements faithfully implemented. This provides confidence the review was thorough.

```
## Confirmed Alignment

- [module/interface/flow]: <what plan.md specifies>
  evidence: <file:line or diff hunk confirming it>
```

### Drift Points

Report each genuine drift as:

```
## Drift Points

- drift_point: <short description>
  planned: <what plan.md says>
  actual: <what the code does>
  severity: p1 | p2 | p3
  files: <affected file paths>
  suggestion: <recommended fix direction>
```

### Summary

```
## Summary

- confirmed_alignments: N
- drift_points: N (p1: X, p2: Y, p3: Z)
- edge_cases_encountered: <list any edge cases from above that were checked>
- verdict: ALIGNED | MINOR_DRIFT | SIGNIFICANT_DRIFT
```

Severity guide:

- **p1** — breaks a plan invariant (wrong interface, missing module, reversed data flow)
- **p2** — deviates from plan but functionally correct (extra field, renamed symbol, moved file)
- **p3** — cosmetic or stylistic difference (naming convention, comment mismatch, file organization)
