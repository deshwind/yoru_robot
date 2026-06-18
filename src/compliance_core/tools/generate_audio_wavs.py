#!/usr/bin/env python3
"""Generate the warning audio files for audio_warning_node.

Preference order:
  1. gTTS  (natural Google voice, needs internet once) -> {name}.mp3
  2. espeak-ng / espeak / pico2wave (offline TTS)      -> {name}.wav
  3. attention tones (pure Python, no deps)            -> {name}.wav

Run from the workspace root, then rebuild so the files are installed:
  python3 src/compliance_core/tools/generate_audio_wavs.py
  colcon build --symlink-install --packages-select compliance_core
"""

import math
import os
import shutil
import struct
import subprocess
import wave

OUT_DIR = os.path.join(os.path.dirname(__file__), '..', 'audio')
MESSAGES = {
    'pa_warning': 'Attention please. Smoking and vaping are prohibited in this '
                  'area. Please stop immediately.',
    'direct_warning': 'This is a final warning. Smoking and vaping are not '
                      'permitted here. This incident will be reported.',
}


def gtts(text, path_mp3):
    try:
        from gtts import gTTS
        gTTS(text=text, lang='en', slow=False).save(path_mp3)
        return True
    except Exception as exc:  # noqa: BLE001 - no internet / not installed
        print(f'  gTTS unavailable ({exc.__class__.__name__}), falling back')
        return False


def tts_offline(text, path_wav):
    if shutil.which('espeak-ng'):
        subprocess.run(['espeak-ng', '-s', '140', '-w', path_wav, text], check=True)
        return True
    if shutil.which('espeak'):
        subprocess.run(['espeak', '-s', '140', '-w', path_wav, text], check=True)
        return True
    if shutil.which('pico2wave'):
        subprocess.run(['pico2wave', '-w', path_wav, text], check=True)
        return True
    return False


def tone(path, pattern):
    """pattern: list of (frequency_hz, seconds); 0 Hz = silence."""
    rate = 22050
    frames = []
    for freq, dur in pattern:
        for i in range(int(rate * dur)):
            v = 0.0 if freq == 0 else 0.6 * math.sin(2 * math.pi * freq * i / rate)
            frames.append(struct.pack('<h', int(v * 32767)))
    with wave.open(path, 'wb') as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(rate)
        w.writeframes(b''.join(frames))


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    for name, text in MESSAGES.items():
        mp3 = os.path.join(OUT_DIR, f'{name}.mp3')
        wav = os.path.join(OUT_DIR, f'{name}.wav')
        if gtts(text, mp3):
            print(f'{mp3}: natural voice (gTTS)')
        elif tts_offline(text, wav):
            print(f'{wav}: spoken message (offline TTS)')
        else:
            tone(wav, [(880, 0.3), (0, 0.1), (880, 0.3), (0, 0.1), (660, 0.5)]
                 if name == 'pa_warning'
                 else [(440, 0.6), (0, 0.1), (440, 0.6), (0, 0.1), (330, 0.8)])
            print(f'{wav}: attention tone (install espeak-ng or gTTS for speech)')


if __name__ == '__main__':
    main()
