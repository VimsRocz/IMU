"""
Compatibility layer: expose symbols from utils_legacy so
`from utils import NAME` continues to work everywhere.

Supports both execution contexts:
- When `src` is on `sys.path` (legacy), `import utils_legacy` works.
- When importing as a package (`import src.utils`), use relative import.
"""
import importlib

try:
    # Prefer relative import from parent package (src)
    _legacy = importlib.import_module('..utils_legacy', __package__)
except Exception:  # pragma: no cover - legacy path insertion case
    _legacy = importlib.import_module('utils_legacy')

def __getattr__(name):
    return getattr(_legacy, name)

def __dir__():
    return sorted(set(list(globals().keys()) + dir(_legacy)))
