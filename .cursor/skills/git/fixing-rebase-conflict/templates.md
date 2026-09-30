# Presentation Templates

Templates for presenting conflict analysis and resolution summaries.

## Conflict Summary Template

```
## Rebase Conflict Summary

**Commit being applied:** <hash> - <message>
**Total conflicts:** <N> file(s)

| # | File | Conflict Type | Recommendation |
|---|------|---------------|----------------|
| 1 | path/to/file1.cc | api_change | Accept HEAD + fix deps |
| 2 | path/to/file2.py | import_addition | Keep both |
```

## Per-Conflict Detail Template

```
### Conflict #N: <filename>

**Our intent:** <what our branch was trying to do>
**Their intent:** <what master changed>
**Root cause:** <why these changes conflict>

**Our version (HEAD):**
<our changes>

**Their version (origin/master):**
<master's changes>

**Recommended resolution:**
<recommendation with reasoning>

**Proposed merged code:**
<actual code>
```

## Rebase Complete Template

```
## Rebase Complete

### Commit Structure
<git log --oneline output>

### File-Level Changes
| File | Action | Key Changes |
|------|--------|-------------|

### Strategies Applied
| Conflict Type | Strategy | Count |
|---------------|----------|-------|

### Next Steps
git push -f origin <branch_name>
```
