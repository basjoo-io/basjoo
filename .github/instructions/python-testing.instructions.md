---
applyTo: "**/*_test.py,**/test_*.py"
---
# Python testing

## Runner

```bash
uv run pytest
```

## Structure

Follow **Arrange → Act → Assert**; keep tests independent.

## Fixtures

Use `@pytest.fixture` in `conftest.py` or local modules for reusable setup.

## Parametrization

Use `@pytest.mark.parametrize` for multiple input/output cases.

## Exceptions

Use `pytest.raises` with `match=` when asserting error messages.

## Mocks

Mock external systems (network, disk, time), not the core logic under test unless isolating a side effect.
