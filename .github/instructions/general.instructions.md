---
applyTo: "**/*"
---
# General coding guidance

## Structure

- Avoid long stretches of blank lines; separate logical blocks with a single blank line.
- Prefer symmetric branches (`if` / `else`) with similar ordering of steps.
- Use early returns to reduce nesting when it clarifies intent.

## Naming

Names should encode **behavior** or **meaning**, not internal implementation details, unless necessary for clarity.

## Constants

Replace unexplained numeric literals with named constants when the number’s meaning is not obvious from context.

## Comments

- English only.
- Comment **why** and non-obvious constraints (units, tolerance, API preconditions), not what the next line obviously does.
