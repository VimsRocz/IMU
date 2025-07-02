.PHONY: test install-test-deps

install-test-deps:
	pip install -r requirements.txt -r requirements-dev.txt

test: install-test-deps
	PYTHONPATH=src pytest -q
