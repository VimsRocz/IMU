.PHONY: test

test:
	pip install -r requirements.txt
	PYTHONPATH=src pytest -q
