import pytest
import scipy.io
from src.utils import save_mat

np = pytest.importorskip("numpy")
pytest.importorskip("scipy")

def test_save_mat(tmp_path):
    data = {'a': np.array([1,2,3])}
    f = tmp_path / 'out.mat'
    save_mat(f, data)
    assert f.exists()
    m = scipy.io.loadmat(f)
    assert 'a' in m
    assert np.allclose(m['a'].flatten(), [1,2,3])

