.PHONY: test clean docs

# to imitate SLURM set only single node
export SLURM_LOCALID=0
# assume you have installed need packages
export SPHINX_MOCK_REQUIREMENTS=1


clean:
	# clean all temp runs
	rm -rf $(shell find . -name "mlruns")

	rm -rf _ckpt_*
	rm -rf .mypy_cache
	rm -rf .pytest_cache
	rm -rf ./docs/build

	rm -rf build
	rm -rf dist


test: clean
	# Review the CONTRIBUTING documentation for other ways to test.
	pip install -e . \
	-r requirements/docs.txt \
	python -m coverage report

docs: clean
	pip install -e . --quiet -r requirements/docs.txt
	cd docs && $(MAKE) html

update:
	git submodule update --init --recursive --remote
