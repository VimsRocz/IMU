import os
from src.utils import check_free_space


def test_check_free_space(tmp_path):
    # Should have at least some space available
    assert check_free_space(tmp_path, min_free_mb=1)
    # Unrealistically high requirement should fail
    assert not check_free_space(tmp_path, min_free_mb=10**6)
