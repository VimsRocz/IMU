import os, numpy as np, scipy.io
from utils import save_mat

def test_save_mat(tmp_path):
    data = {'a': np.array([1,2,3])}
    f = tmp_path / 'out.mat'
    save_mat(f, data)
    assert f.exists()
    m = scipy.io.loadmat(f)
    assert 'a' in m
    assert np.allclose(m['a'].flatten(), [1,2,3])

