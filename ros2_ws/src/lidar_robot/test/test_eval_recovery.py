"""tools/eval_recovery.py runs on the simulated floor and writes the results table (no key needed)."""
import json
import os
import sys

import pytest

TOOLS = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', '..', 'tools')


def test_eval_recovery_rules_beat_no_recovery(tmp_path, monkeypatch):
    if not os.path.exists(os.path.join(TOOLS, 'eval_recovery.py')):
        pytest.skip('tools/ not in this checkout')
    monkeypatch.delenv('GROQ_API_KEY', raising=False)
    sys.path.insert(0, TOOLS)
    import eval_recovery
    summ = eval_recovery.main(['--policies', 'none,rules,llm', '--only', 'person_in_corridor', '--out', str(tmp_path)])
    assert summ['rules']['success'] == 1 and summ['none']['success'] == 0
    assert 'llm' not in summ                                   # no key: skipped, and the results say so
    data = json.loads((tmp_path / 'recovery_eval.json').read_text())
    assert 'GROQ_API_KEY' in data['notes'][0]
    md = (tmp_path / 'recovery_eval.md').read_text()
    assert '| rules |' in md and 'person_in_corridor' in md
    assert (tmp_path / 'recovery_eval.csv').exists()
