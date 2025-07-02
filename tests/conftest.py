import sys
from pathlib import Path

# Add the project root and the 'src' directory to sys.path so tests can
# import application modules without installing the package.
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
sys.path.insert(0, str(ROOT))
