# Conflict Type Classification

Reference for classifying and handling rebase conflicts consistently.

## Types

| Type              | Description                                           | Default Strategy                      |
| ----------------- | ----------------------------------------------------- | ------------------------------------- |
| `import_addition` | Both sides added different imports/includes           | Keep both                             |
| `api_change`      | API signature or implementation changed               | Accept HEAD, fix dependent code       |
| `style_diff`      | Code style differences (naming, comments, formatting) | Accept HEAD                           |
| `logic_conflict`  | Business logic changes on both sides                  | Requires manual decision              |
| `file_structure`  | File/directory reorganization                         | Trace movement, apply to new location |
| `add_add`         | Both sides created same file                          | Smart merge or accept HEAD            |
| `delete_modify`   | One side deleted, other modified                      | Requires manual decision              |

## Common Patterns

| Type              | Typical Scenario                   | Notes                                                                                                                             |
| ----------------- | ---------------------------------- | --------------------------------------------------------------------------------------------------------------------------------- |
| `import_addition` | Both sides added different imports | Usually safe to keep both                                                                                                         |
| `api_change`      | Both sides modified same function  | Analyze intent; may need manual merge                                                                                             |
| `file_structure`  | One side moved/renamed code        | Trace the movement, apply to new location                                                                                         |
| `delete_modify`   | One side deleted, other modified   | Discuss whether feature is still needed                                                                                           |
| `add_add`         | Both sides created similar files   | ⚠️ For BUILD files: Grep full file for target name before "keep both" — upstream may have the same target at a different position |

## Learning Mechanism

Track user decisions per conflict type during a session. When user decides on a type, apply the same strategy to all subsequent conflicts of that type:

```
You chose "accept HEAD" for `api_change` type conflicts.
3 more conflicts of the same type will be resolved automatically using this strategy.
```

Decision flow per conflict:

1. **Classify** the conflict type from the table above
2. **Check** if user already decided on this type in this session
3. **If yes** → apply same strategy automatically
4. **If no** → ask for decision and remember it
