.PHONY: test
test:
	pip install -r requirements-dev.txt -r requirements.txt
	pytest -q

