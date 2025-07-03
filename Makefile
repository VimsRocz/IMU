
.PHONY: deps test

# Install all Python dependencies needed for running the code and tests.
deps:
	pip install --upgrade pip setuptools wheel build
       pip install -e .[tests]

# Install dependencies and run the full test suite.
test: deps
	pytest -q


