#!/usr/bin/env python3
"""Copy facts from a text file into the editable notebook (knowledge.json).

EASIER: just copy the file to ~/maps/info.txt on the robot; it is read automatically (see knowledge.py).
Use this tool only when you want the facts to become normal, editable entries in the app.

Write a plain text file like this (one block per fact, a blank line between blocks):

    ## HOD of CSE
    The Head of the CSE (AI & DS) department is Dr. ... . The HOD office is on the first floor, next to Lab 3.
    tags: hod, head of department, cse, office

    ## Library timings
    The library is open 9:30 am to 5:00 pm, Monday to Saturday.
    tags: library, timing, open, kab khulti

Then on the Pi:
    sudo systemctl stop robot
    python3 ~/slam-robot/tools/import_knowledge.py ~/department_info.txt          # adds/updates entries
    python3 ~/slam-robot/tools/import_knowledge.py ~/department_info.txt --replace  # wipe old entries first
    sudo systemctl start robot

An entry whose title already exists is updated, not duplicated. Stop the robot first: the running robot
keeps its own copy and would overwrite the file the next time someone edits the notebook in the app.
Rules from docs/chat.md still apply: no phone numbers or private details, one fact per block.
"""
import argparse
import os
import re
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'ros2_ws', 'src', 'lidar_robot'))
from lidar_robot.knowledge import Knowledge, MAX_TEXT, MAX_TITLE, parse_info  # noqa: E402

PHONE = re.compile(r'(?<!\d)(?:\+?91[\s-]?)?[6-9]\d{9}(?!\d)')


def parse(text):
    return parse_info(text)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('file', help='text file with ## title blocks')
    ap.add_argument('--notebook', default=os.path.expanduser('~/maps/knowledge.json'))
    ap.add_argument('--replace', action='store_true', help='delete all existing entries first')
    ap.add_argument('--dry-run', action='store_true', help='only show what would change')
    a = ap.parse_args(argv)

    with open(a.file, encoding='utf-8') as f:
        facts = parse(f.read())
    if not facts:
        sys.exit('no "## title" blocks found in ' + a.file)

    kb = Knowledge(a.notebook)
    if a.replace and not a.dry_run:
        for e in list(kb.entries):
            kb.remove(e['id'])
    by_title = {e['title'].lower(): e for e in kb.entries}
    added = updated = 0
    for title, text, tags in facts:
        notes = []
        if len(title) > MAX_TITLE or len(text) > MAX_TEXT:
            notes.append(f'cut to {MAX_TITLE}/{MAX_TEXT} characters')
        if PHONE.search(text):
            notes.append('has a phone number: anyone at the robot will hear it')
        old = by_title.get(title.lower())
        action = 'update' if old else 'add   '
        print(f'{action} {title}' + (f'   ({"; ".join(notes)})' if notes else ''))
        if a.dry_run:
            continue
        if old:
            kb.update(old['id'], title=title, text=text, tags=tags)
            updated += 1
        else:
            kb.add(title, text, tags)
            added += 1
    if a.dry_run:
        print(f'\n{len(facts)} facts read; nothing written (--dry-run)')
    else:
        print(f'\n{added} added, {updated} updated, {len(kb.entries)} entries in {a.notebook}')
        print('start the robot again: sudo systemctl start robot')


if __name__ == '__main__':
    main()
