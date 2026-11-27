"""Robot voice: short spoken phrases through the speaker on the Pi (offline).

    Speaker(engine='auto')   Piper (natural offline voices, English + Hindi) when scripts/setup_voice.sh has
                             installed it, else espeak-ng, else silent (never an error)
    sp.say('Going to Lab 3.', key='going')

* One phrase at a time (a queue, never two voices on top of each other); a full queue drops the oldest.
* Rate-limited: the same key not again within gap_s (8 s), and at most max_per_min phrases a minute
  (important keys like a question to a person or a low battery always get through).
* Volume 0..1.5 (piper: samples scaled before aplay; espeak: amplitude).
* Text with Devanagari letters uses the Hindi voice, and so does Hinglish in Latin letters ("Main Lab 2 ja
  rahi hoon"): its common Hindi words are turned into Devanagari first (HINDI_WORDS), names stay as they are.
  Everything else uses the English voice. The conversation agent also sends a Devanagari `speak` version.
* Speed: Piper length_scale (rate, 1.0 = the voice's own speed, 1.2 = 20 % slower) with a short pause
  between sentences; espeak words per minute (speed). Piper streams straight into aplay, so the first
  sentence plays while the rest is still being made (much less waiting on a Pi 4).
"""
import array
import glob
import os
import queue
import re
import shutil
import subprocess
import threading
import time

ARABIC = re.compile('[\u0600-\u06ff\u0750-\u077f\ufb50-\ufdff\ufe70-\ufeff]')
DEVANAGARI = re.compile('[ऀ-ॿ]')
PRIORITY_KEYS = {'recovery_ask', 'battery', 'confirm', 'estop', 'voice_reply', 'dock'}
PIPER_DIRS = ['~/piper', '/opt/piper', '/usr/local/share/piper']


def find_piper():
    exe = shutil.which('piper')
    for d in PIPER_DIRS:
        p = os.path.join(os.path.expanduser(d), 'piper')
        if not exe and os.path.isfile(p) and os.access(p, os.X_OK):
            exe = p
    return exe


def find_voice(lang, explicit=''):
    """explicit path or voice name (hi_IN-priyamvada-medium), else the first ~/piper/voices/<lang>_*.onnx."""
    if explicit and os.path.isfile(os.path.expanduser(explicit)):
        return os.path.expanduser(explicit)
    if explicit:
        name = explicit if explicit.endswith('.onnx') else explicit + '.onnx'
        for d in PIPER_DIRS:
            p = os.path.join(os.path.expanduser(d), 'voices', os.path.basename(name))
            if os.path.isfile(p):
                return p
    for d in PIPER_DIRS:
        hits = sorted(glob.glob(os.path.join(os.path.expanduser(d), 'voices', f'{lang}_*.onnx')))
        if hits:
            return hits[0]
    return None


def piper_args(exe, model, rate=1.0, pause_s=0.3):
    """Piper command: raw 16-bit audio on stdout, slower by `rate`, `pause_s` of silence after each sentence."""
    return [exe, '--model', model, '--output_raw', '--length_scale', f'{rate:.2f}',
            '--sentence_silence', f'{pause_s:.2f}']


def stream_pcm(src, dst, volume, chunk=4096):
    """Copy 16-bit PCM from src to dst as it arrives (volume applied), keeping sample boundaries.
    Returns the number of bytes written."""
    carry, n = b'', 0
    read = getattr(src, 'read1', src.read)
    while True:
        data = read(chunk)
        if not data:
            break
        data = carry + data
        cut = len(data) - len(data) % 2
        carry = data[cut:]
        out = scale_pcm(data[:cut], volume)
        dst.write(out)
        dst.flush()
        n += len(out)
    return n


def scale_pcm(raw, volume):
    """16-bit little-endian mono PCM bytes * volume, clipped."""
    if abs(volume - 1.0) < 1e-3:
        return raw
    a = array.array('h')
    a.frombytes(raw[:len(raw) - len(raw) % 2])
    for i, v in enumerate(a):
        s = int(v * volume)
        a[i] = 32767 if s > 32767 else -32768 if s < -32768 else s
    return a.tobytes()


# Hinglish in Latin letters ("Main Lab 2 ja rahi hoon"): the English voice reads these words as English and
# mangles them (27 Nov). Common Hindi words -> Devanagari for the Hindi voice; names (Lab 2, HOD office) stay.
HINDI_WORDS = {
    'main': 'मैं', 'mai': 'मैं', 'hoon': 'हूँ', 'hun': 'हूँ', 'hu': 'हूँ', 'hai': 'है', 'hain': 'हैं', 'ho': 'हो',
    'tha': 'था', 'thi': 'थी', 'ja': 'जा', 'jaa': 'जा', 'jao': 'जाओ', 'jaiye': 'जाइए', 'rahi': 'रही', 'raha': 'रहा',
    'rahe': 'रहे', 'kar': 'कर', 'karo': 'करो', 'karna': 'करना', 'karke': 'करके', 'karein': 'करें', 'kijiye': 'कीजिए',
    'sakti': 'सकती', 'sakta': 'सकता', 'sakte': 'सकते', 'aap': 'आप', 'aapka': 'आपका', 'aapki': 'आपकी',
    'aapko': 'आपको', 'tum': 'तुम', 'mera': 'मेरा', 'meri': 'मेरी', 'mere': 'मेरे', 'mujhe': 'मुझे', 'hum': 'हम',
    'hamara': 'हमारा', 'hamare': 'हमारे', 'kya': 'क्या', 'kaun': 'कौन', 'kab': 'कब', 'kahan': 'कहाँ',
    'kaise': 'कैसे', 'kyun': 'क्यों', 'kyu': 'क्यों', 'kitna': 'कितना', 'kitne': 'कितने', 'kitni': 'कितनी',
    'ka': 'का', 'ki': 'की', 'ke': 'के', 'ko': 'को', 'se': 'से', 'mein': 'में', 'me': 'में', 'par': 'पर',
    'tak': 'तक', 'aur': 'और', 'ya': 'या', 'bhi': 'भी', 'nahi': 'नहीं', 'nahin': 'नहीं', 'haan': 'हाँ',
    'ji': 'जी', 'yahan': 'यहाँ', 'wahan': 'वहाँ', 'idhar': 'इधर', 'udhar': 'उधर', 'abhi': 'अभी', 'phir': 'फिर',
    'pehle': 'पहले', 'baad': 'बाद', 'saath': 'साथ', 'liye': 'लिए', 'theek': 'ठीक', 'thik': 'ठीक',
    'accha': 'अच्छा', 'achha': 'अच्छा', 'chalo': 'चलो', 'chaliye': 'चलिए', 'aaiye': 'आइए', 'gaya': 'गया',
    'gayi': 'गई', 'gaye': 'गए', 'pahunch': 'पहुँच', 'pahunchi': 'पहुँची', 'pahuncha': 'पहुँचा', 'ruko': 'रुको',
    'rukiye': 'रुकिए', 'rasta': 'रास्ता', 'raasta': 'रास्ता', 'kamra': 'कमरा', 'kripya': 'कृपया',
    'dhanyavaad': 'धन्यवाद', 'dhanyawad': 'धन्यवाद', 'shukriya': 'शुक्रिया', 'namaste': 'नमस्ते', 'maaf': 'माफ़',
    'bataiye': 'बताइए', 'batao': 'बताओ', 'bata': 'बता', 'chahiye': 'चाहिए', 'wala': 'वाला', 'wali': 'वाली',
    'bahar': 'बाहर', 'paas': 'पास', 'saamne': 'सामने', 'andar': 'अंदर', 'upar': 'ऊपर', 'neeche': 'नीचे',
    'ek': 'एक', 'teen': 'तीन', 'mil': 'मिल', 'milega': 'मिलेगा', 'milegi': 'मिलेगी', 'dijiye': 'दीजिए',
    'de': 'दे', 'diya': 'दिया', 'lijiye': 'लीजिए', 'intezaar': 'इंतज़ार', 'samay': 'समय', 'aaj': 'आज', 'kal': 'कल',
    'kisne': 'किसने', 'banaya': 'बनाया', 'kidhar': 'किधर', 'tumhe': 'तुम्हें', 'tumhara': 'तुम्हारा',
    'tumhari': 'तुम्हारी', 'tumko': 'तुमको', 'aapne': 'आपने', 'kaunsa': 'कौनसा', 'kaunsi': 'कौनसी',
    'konsa': 'कौनसा', 'kaisa': 'कैसा', 'kaisi': 'कैसी', 'kyon': 'क्यों', 'lao': 'लाओ', 'lekar': 'लेकर',
    'dena': 'देना', 'bolo': 'बोलो', 'boliye': 'बोलिए', 'suno': 'सुनो', 'dekho': 'देखो', 'jana': 'जाना',
    'jaana': 'जाना', 'aana': 'आना', 'aao': 'आओ', 'wapas': 'वापस', 'vapas': 'वापस', 'madad': 'मदद',
    'poochiye': 'पूछिए', 'naam': 'नाम', 'kahin': 'कहीं', 'koi': 'कोई', 'sab': 'सब', 'bahut': 'बहुत',
}
# Also ordinary English words: they never decide on their own that a sentence is Hinglish.
AMBIGUOUS = {'main', 'mai', 'me', 'hu', 'ho', 'tha', 'thi', 'ya', 'par', 'de', 'mil', 'ji', 'kal', 'ek', 'teen'}
LATIN_WORD = re.compile(r"[A-Za-z]+")


def hindi_hits(text):
    """(clearly Hindi words, all words) in Latin-letter text."""
    words = [w.lower() for w in LATIN_WORD.findall(text or '')]
    return sum(1 for w in words if w in HINDI_WORDS and w not in AMBIGUOUS), len(words)


def is_hinglish(text, min_hits=2):
    """Hindi written in Latin letters: at least `min_hits` clearly Hindi words making up a quarter of the words.
    The voice needs 2 (an English reply with one Hindi name stays English); what a person said needs 1."""
    if DEVANAGARI.search(text or ''):
        return False
    hits, n = hindi_hits(text)
    return hits >= min_hits and hits >= 0.25 * n


def to_devanagari(text):
    """Known Hindi words -> Devanagari so the Hindi voice says them right; other words (names) unchanged."""
    return LATIN_WORD.sub(lambda m: HINDI_WORDS.get(m.group(0).lower(), m.group(0)), text)


def prepare_text(text, voices, one_voice=''):
    """-> (voice, text to speak). Hinglish goes to the Hindi voice, with its Hindi words in Devanagari."""
    if voices.get('hi') and is_hinglish(text):
        return 'hi', to_devanagari(text)
    return pick_lang(text, voices, one_voice), text


def pick_lang(text, voices, one_voice='hi'):
    """Which voice reads this text. Devanagari (and Hinglish in Latin letters) needs the Hindi voice. With
    one_voice='hi' the Hindi voice reads English too (Indian-accented English), so the robot is always the SAME
    person; '' = English voice for English, Hindi voice for Hindi (two different people: 17 Nov kiosk complaint)."""
    if (DEVANAGARI.search(text) or is_hinglish(text)) and voices.get('hi'):
        return 'hi'
    if one_voice in ('hi', 'en') and voices.get(one_voice):
        return one_voice
    return 'en' if voices.get('en') else 'hi'


class Speaker:
    def __init__(self, enabled=True, voice='en-in', speed=150, min_gap_s=8.0, runner=None, clock=time.monotonic,
                 engine='auto', volume=1.0, voice_en='', voice_hi='', max_per_min=8, audio_device='',
                 rate=1.15, pause_s=0.3, one_voice=''):
        self.enabled = enabled
        self.one_voice = one_voice
        self.rate = max(0.6, min(2.0, float(rate)))
        self.pause_s = max(0.0, min(1.5, float(pause_s)))
        self.voice, self.speed, self.min_gap_s = voice, speed, min_gap_s
        self.volume = max(0.0, min(1.5, float(volume)))
        self.max_per_min = max_per_min
        self.clock = clock
        self.audio_device = audio_device
        self._last = {}
        self._recent = []
        self.spoken = []                       # recent phrases, for the app's log
        self.engine = 'none'
        self._runner = runner
        if runner is None:
            self._runner = self._pick(engine, voice_en, voice_hi)
        else:
            self.engine = 'custom'
        self.available = self._runner is not None
        self._q = queue.Queue(maxsize=4)
        self._worker = None

    # ------------------------------------------------------------ engines
    def _pick(self, engine, voice_en, voice_hi):
        piper = find_piper() if engine in ('auto', 'piper') else None
        if piper and shutil.which('aplay'):
            self.piper = piper
            self.voices = {'en': find_voice('en', voice_en), 'hi': find_voice('hi', voice_hi)}
            if self.voices['en'] or self.voices['hi']:
                self.engine = 'piper'
                return self._run_piper
        exe = (shutil.which('espeak-ng') or shutil.which('espeak')) if engine in ('auto', 'espeak') else None
        if exe:
            self.engine = 'espeak'
            self._espeak = exe
            return self._run_espeak
        return None

    def _run_piper(self, text):
        lang, text = prepare_text(text, self.voices, self.one_voice)
        model = self.voices.get(lang) or self.voices.get('en') or self.voices.get('hi')
        rate = 22050
        cfg = model + '.json'
        if os.path.isfile(cfg):
            try:
                import json
                rate = int(json.load(open(cfg)).get('audio', {}).get('sample_rate', rate))
            except (ValueError, OSError):
                pass
        cmd = ['aplay', '-q', '-r', str(rate), '-f', 'S16_LE', '-t', 'raw', '-c', '1']
        if self.audio_device:
            cmd[1:1] = ['-D', self.audio_device]
        tts = subprocess.Popen(piper_args(self.piper, model, self.rate, self.pause_s), stdin=subprocess.PIPE,
                               stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        play = subprocess.Popen(cmd + ['-'], stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL)
        try:
            tts.stdin.write(text.encode())
            tts.stdin.close()
            stream_pcm(tts.stdout, play.stdin, self.volume)
            play.stdin.close()
            play.wait(timeout=60)
            tts.wait(timeout=5)
        except (OSError, subprocess.TimeoutExpired):
            pass
        finally:
            for proc in (tts, play):
                if proc.poll() is None:
                    proc.kill()

    def _run_espeak(self, text):
        if is_hinglish(text):
            text = to_devanagari(text)
        v = 'hi' if DEVANAGARI.search(text) or self.one_voice == 'hi' else self.voice
        subprocess.run([self._espeak, '-v', v, '-s', str(int(self.speed / self.rate)), '-a', str(int(100 * self.volume)), text],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=20)

    # ------------------------------------------------------------ public
    def say(self, text, key=None, gap_s=None):
        """Speak unless the same key was spoken less than gap_s ago (or too much was said this minute).
        Returns True if queued."""
        if not (self.enabled and text):
            return False
        if ARABIC.search(text) and not DEVANAGARI.search(text):
            text = 'Sorry, please say that again.'      # Urdu/Arabic script: no voice here can read it
        key = key or text
        now = self.clock()
        if now - self._last.get(key, -1e9) < (self.min_gap_s if gap_s is None else gap_s):
            return False
        self._recent = [t for t in self._recent if now - t < 60.0]
        if key not in PRIORITY_KEYS and len(self._recent) >= self.max_per_min:
            return False
        self._last[key] = now
        self._recent.append(now)
        self.spoken.insert(0, text)
        del self.spoken[10:]
        if self._runner:
            if self._q.full():
                try:
                    self._q.get_nowait()             # drop the oldest: say what matters now
                except queue.Empty:
                    pass
            self._q.put_nowait(text)
            if self._worker is None or not self._worker.is_alive():
                self._worker = threading.Thread(target=self._drain, daemon=True)
                self._worker.start()
        return True

    def set_volume(self, v):
        self.volume = max(0.0, min(1.5, float(v)))

    def busy(self):
        return not self._q.empty() or bool(self._worker and self._worker.is_alive() and getattr(self, '_talking', False))

    def _drain(self):
        while True:
            try:
                text = self._q.get(timeout=0.5)
            except queue.Empty:
                return
            self._talking = True
            try:
                self._runner(text)
            except Exception:
                pass
            finally:
                self._talking = False

    def wait(self, timeout=10.0):
        """Tests: until everything queued has been spoken."""
        t0 = time.monotonic()
        while (not self._q.empty() or getattr(self, '_talking', False)) and time.monotonic() - t0 < timeout:
            time.sleep(0.01)
