# Contributing

Thank you for considering a contribution to this project! The following notes explain how to set up your environment, run the tests and lint the code base.

## Installing Development Requirements

All development dependencies are installed via the helper script `scripts/setup_tests.sh`. This installs the package in editable mode together with the optional `tests` extras defined in `pyproject.toml`.

```bash
./scripts/setup_tests.sh
```

The script must be executed once before running the unit tests.

## Running the Test Suite

After installing the dependencies, execute the tests with [pytest](https://pytest.org/):

```bash
pytest -q
```

This repository also provides a `Makefile` target `make test` that combines the dependency installation and the pytest run.

## Linting

Static analysis is performed with [Ruff](https://github.com/astral-sh/ruff) and [Flake8](https://flake8.pycqa.org/). Run both tools on the `src` and `tests` directories:

```bash
ruff check src tests
flake8 src tests
```

The Ruff configuration resides in `pyproject.toml` while Flake8 reads settings from `.flake8`.
