"""Which language the robot answers in: decided by rules from what the person said, never by the model.

    reply_language('lab 2 kahan hai')        -> 'hi'   (Hinglish in Latin letters)
    reply_language('लैब 2 कहाँ है')            -> 'hi'   (Devanagari)
    reply_language('where is lab 2')         -> 'en'
    reply_language('where is lab 2, hindi mein batao') -> 'hi'   (asked for Hindi)
    reply_language('lab 2 kahan hai, in English')      -> 'en'   (asked for English)

Hindi only when the person spoke Hindi or asked for it; English otherwise (27 Nov: the choice used to be
left to the model and was not predictable).
"""
import re

from lidar_robot.speech import DEVANAGARI, is_hinglish

ASK_ENGLISH = re.compile(r'\b(in english|english (mein|me|main|men)|english m[ae]in? (bolo|batao|boliye|bataiye)|'
                         r'speak english|angrezi)\b', re.I)
ASK_HINDI = re.compile(r'\b(in hindi|hindi (mein|me|main|men)|hindi (bolo|boliye|batao|bataiye)|speak hindi)\b'
                       r'|हिंदी|हिन्दी', re.I)


def reply_language(text):
    """'hi' or 'en' for the reply to `text`. An explicit ask wins, then the language of the words."""
    t = text or ''
    if ASK_ENGLISH.search(t):
        return 'en'
    if ASK_HINDI.search(t) or DEVANAGARI.search(t) or is_hinglish(t, min_hits=1):
        return 'hi'
    return 'en'


def instruction(lang):
    """The fixed language line for a language-model prompt."""
    if lang == 'hi':
        return ('LANGUAGE (fixed, do not change it): write "reply" in Hinglish, Hindi in Latin letters '
                '("Lab 2 pehli manzil par hai"), and "speak" fully in Devanagari.')
    return 'LANGUAGE (fixed, do not change it): write "reply" in plain English only. "speak" is null.'
