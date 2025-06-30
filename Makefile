.PHONY: deps-test test

# Install all packages needed for running the tests
deps-test:
	pip install -r requirements-dev.txt -r requirements.txt

# Install dependencies and execute the test suite
test: deps-test
        pytest -q Python/tests

