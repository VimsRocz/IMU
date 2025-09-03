import sys
from pathlib import Path

# Add the project root and the 'src' directory to sys.path so tests can
# import application modules without installing the package.
ROOT = Path(__file__).resolve().parents[2]
PYTHON_DIR = ROOT / 'PYTHON'
PYTHON_SRC = PYTHON_DIR / 'src'
sys.path.insert(0, str(PYTHON_SRC))
sys.path.insert(0, str(PYTHON_DIR))
sys.path.insert(0, str(ROOT))
