import runpy
import sys
import subprocess
import pathlib
import types


def _run_script(monkeypatch, args):
    cmd = {}

    def fake_run(c, check):
        cmd['value'] = c
        return subprocess.CompletedProcess(c, 0)

    monkeypatch.setattr(subprocess, 'run', fake_run)
    monkeypatch.setattr(pathlib.Path, 'glob', lambda self, pattern: [])
    monkeypatch.setattr(sys, 'argv', ['run_triad_only.py'] + args)
    repo_root = pathlib.Path(__file__).resolve().parents[1]
    monkeypatch.syspath_prepend(str(repo_root))
    # stub heavy modules
    po = types.ModuleType('plot_overlay')
    po.plot_overlay = lambda *a, **k: None
    vt = types.ModuleType('validate_with_truth')
    vt.load_estimate = lambda *a, **k: {}
    vt.assemble_frames = lambda *a, **k: {}
    monkeypatch.setitem(sys.modules, 'plot_overlay', po)
    monkeypatch.setitem(sys.modules, 'validate_with_truth', vt)
    runpy.run_path('run_triad_only.py', run_name='__main__')
    return cmd['value']


def test_default_datasets(monkeypatch):
    cmd = _run_script(monkeypatch, [])
    assert '--datasets' in cmd
    idx = cmd.index('--datasets')
    assert cmd[idx + 1] == 'X001,X002'


def test_override_dataset(monkeypatch):
    cmd = _run_script(monkeypatch, ['--datasets', 'X002'])
    idx = cmd.index('--datasets')
    assert cmd[idx + 1] == 'X002'
