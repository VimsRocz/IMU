.PHONY: test

test:
	pip install -r requirements.txt -r requirements-dev.txt
	PYTHONPATH=src pytest -q
