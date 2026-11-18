"""tools/import_knowledge.py: a text file of '## title' blocks lands in the notebook, re-import updates."""
import importlib.util
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
TOOL = os.path.join(HERE, '..', '..', '..', '..', 'tools', 'import_knowledge.py')


def _tool():
    if not os.path.isfile(TOOL):                 # the robot image holds the package only, not tools/
        import pytest
        pytest.skip('tools/ not in this checkout')
    spec = importlib.util.spec_from_file_location('import_knowledge', TOOL)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


SAMPLE = """# Department facts

## HOD of CSE
The HOD office is on the first floor,
next to Lab 3.
tags: hod, head, office

## Library timings
Open 9:30 am to 5 pm, Monday to Saturday.
"""


def test_parse_blocks_text_and_tags():
    facts = _tool().parse(SAMPLE)
    titles = [f[0] for f in facts]
    assert titles == ['HOD of CSE', 'Library timings']      # the bare '# Department facts' heading has no text
    assert facts[0][1] == 'The HOD office is on the first floor, next to Lab 3.'
    assert facts[0][2] == ['hod', 'head', 'office']
    assert facts[1][2] == []


def test_import_adds_then_updates(tmp_path):
    tool = _tool()
    src = tmp_path / 'info.txt'
    nb = tmp_path / 'knowledge.json'
    src.write_text(SAMPLE)
    tool.main([str(src), '--notebook', str(nb)])
    data = json.loads(nb.read_text())['entries']
    assert len(data) == 2
    src.write_text(SAMPLE.replace('Lab 3', 'Lab 4'))
    tool.main([str(src), '--notebook', str(nb)])
    data = json.loads(nb.read_text())['entries']
    assert len(data) == 2                                     # same titles: updated, not duplicated
    assert 'Lab 4' in data[0]['text']


def test_dry_run_writes_nothing(tmp_path):
    src = tmp_path / 'info.txt'
    nb = tmp_path / 'knowledge.json'
    src.write_text(SAMPLE)
    _tool().main([str(src), '--notebook', str(nb), '--dry-run'])
    assert not nb.exists()


PLAIN = """Pathik team
Pathik was built by Arpit Awasthi, Neelaksh Thakur, Rashi and Yogit, final-year B.Tech CSE (AI & DS) students.

The library is open from 9:30 am to 5 pm, Monday to Saturday. Books are issued for 14 days.
"""


def test_plain_paragraphs_become_facts():
    from lidar_robot.knowledge import parse_info
    facts = parse_info(PLAIN)
    assert facts[0][0] == 'Pathik team' and facts[0][1].startswith('Pathik was built by')
    assert facts[1][0].startswith('The library is open')            # no short first line: first words
    assert 'Books are issued' in facts[1][1]


def test_info_txt_is_read_and_followed(tmp_path):
    import os
    import time
    from lidar_robot.knowledge import Knowledge
    (tmp_path / 'info.txt').write_text(PLAIN)
    kb = Knowledge(str(tmp_path / 'knowledge.json'))
    best = kb.search('who built pathik team')[0][1]
    assert best['id'].startswith('info-')
    assert kb.summary()['from_file'] == 2
    try:
        kb.remove(best['id'])
        assert False, 'file facts are read-only'
    except ValueError as e:
        assert 'info.txt' in str(e)
    (tmp_path / 'info.txt').write_text(PLAIN + '\nCanteen\nThe canteen opens at 8 am.\n')
    os.utime(tmp_path / 'info.txt', (time.time() + 5, time.time() + 5))
    assert kb.search('where is the canteen')[0][1]['title'] == 'Canteen'     # picked up without a restart
