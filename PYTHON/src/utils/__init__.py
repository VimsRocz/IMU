"""
Compatibility layer: expose symbols from utils_legacy so
`from utils import NAME` continues to work everywhere.
"""
import importlib
_legacy = importlib.import_module('utils_legacy')

def __getattr__(name):
    return getattr(_legacy, name)

def __dir__():
    return sorted(set(list(globals().keys()) + dir(_legacy)))
