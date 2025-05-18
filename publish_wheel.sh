#!/bin/bash -e

# This script builds and publishes the Python package to PyPI.

rm -rf dist
rm -rf build
rm -rf *.egg-info

# Install build deps
pip install .[dev]

# Run tests
pytest

# Check code style
black --check .

# Build the package
python -m build

# Publish the package to PyPI
python -m twine upload dist/*