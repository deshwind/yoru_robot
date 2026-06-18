"""Audio warning node with text-to-speech (dissertation Section 4.8).

Speaks the robot's warnings. Backend chain (first available wins):

  1. espeak-ng  - offline runtime TTS, speaks the actual message text
                  (including dynamic parts like the room name).
                  Install with: sudo apt install espeak-ng
  2. audio file - pre-generated speech in the package audio/ folder:
                  {kind}.mp3 via gst-play-1.0, or {kind}.wav via aplay.
                  Generate natural voices with tools/generate_audio_wavs.py (gTTS).
  3. log only   - warning text is logged (always happens regardless).
"""

import json
import os
import shutil
import subprocess

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String


class AudioWarningNode(Node):

    def __init__(self):
        super().__init__('audio_warning_node')

        default_dir = ''
        try:
            default_dir = os.path.join(
                get_package_share_directory('compliance_core'), 'audio')
        except Exception:  # noqa: BLE001 - share dir may not exist yet
            pass

        self.declare_parameter('use_audio', True)
        self.declare_parameter('audio_dir', default_dir)
        self.declare_parameter('aplay_device', 'default')
        self.declare_parameter('espeak_speed', 140)
        self.declare_parameter('espeak_amplitude', 200)
        self.declare_parameter('espeak_voice', 'en')

        self.espeak = shutil.which('espeak-ng') or shutil.which('espeak')
        self.gst_play = shutil.which('gst-play-1.0')
        self.proc = None  # current playback process; new warning preempts

        self.create_subscription(String, '/compliance/pa_warning',
                                 lambda m: self.play('pa_warning', m), 10)
        self.create_subscription(String, '/compliance/direct_warning',
                                 lambda m: self.play('direct_warning', m), 10)

        if not self.get_parameter('use_audio').value:
            backend = 'log-only (use_audio: false)'
        elif self.espeak:
            backend = f'espeak TTS ({self.espeak})'
        else:
            backend = 'audio files (install espeak-ng for dynamic speech)'
        self.get_logger().info(f'Audio warning node ready - backend: {backend}')

    def play(self, kind, msg):
        try:
            text = json.loads(msg.data).get('message', msg.data)
        except ValueError:
            text = msg.data
        self.get_logger().warn(f'[{kind.upper()}] {text}')

        if not self.get_parameter('use_audio').value:
            return

        # Stop any still-running announcement before starting a new one
        if self.proc is not None and self.proc.poll() is None:
            self.proc.terminate()

        if self.espeak:
            self._run([self.espeak,
                       '-s', str(self.get_parameter('espeak_speed').value),
                       '-a', str(self.get_parameter('espeak_amplitude').value),
                       '-v', self.get_parameter('espeak_voice').value,
                       text])
            return

        audio_dir = self.get_parameter('audio_dir').value
        mp3 = os.path.join(audio_dir, f'{kind}.mp3')
        wav = os.path.join(audio_dir, f'{kind}.wav')
        if self.gst_play and os.path.isfile(mp3):
            self._run([self.gst_play, '-q', mp3])
        elif os.path.isfile(wav):
            self._run(['aplay', '-q', '-D',
                       self.get_parameter('aplay_device').value, wav])
        else:
            self.get_logger().warn(
                f'No TTS engine and no audio file for "{kind}" in {audio_dir}')

    def _run(self, cmd):
        try:
            self.proc = subprocess.Popen(
                cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except OSError as exc:
            self.get_logger().error(f'Audio playback failed: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = AudioWarningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
