.PHONY: deps test

deps:
	pip install -r requirements.txt -r requirements-dev.txt

test: deps
       PYTHONPATH=src pytest -q
