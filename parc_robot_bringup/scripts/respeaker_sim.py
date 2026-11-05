#!/usr/bin/env python3
"""Stand-in for the robot's ReSpeaker node, fed by the host computer's microphone.

Publishes the topics of the physical robot's respeaker_node:
  /audio, /audio/channel0..5, /speech_audio   audio_common_msgs/AudioData
  /is_speeching                               std_msgs/Bool
  /sound_direction                            std_msgs/Int32
  /sound_localization                         geometry_msgs/PoseStamped

Audio is 16 kHz, signed 16-bit little-endian, mono, in chunks of `chunk_samples`
(1024 samples gives the 15.6 messages/s observed on the robot). A single
microphone carries no direction, so the direction is the fixed `doa_deg`
parameter.
"""

import collections
import math
import queue
import subprocess
import threading
import time
import wave

import numpy as np
import rclpy
from audio_common_msgs.msg import AudioData
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int32

BYTES_PER_SAMPLE = 2
CHANNEL_COUNT = 6
RAW_MIC_CHANNELS = (1, 2, 3, 4)
SILENCE_DB = -120.0

STREAM_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class RespeakerSim(Node):
    def __init__(self):
        super().__init__("respeaker_node")
        self.declare_parameter("source", "mic")  # mic | file | silence
        self.declare_parameter("device", "default")  # ALSA/Pulse device for arecord
        self.declare_parameter("wav_file", "")  # 16 kHz mono 16-bit, looped
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("chunk_samples", 1024)
        self.declare_parameter("gain", 1.0)
        self.declare_parameter("doa_deg", 0)
        self.declare_parameter("vad_min_db", -45.0)
        self.declare_parameter("vad_margin_db", 10.0)
        self.declare_parameter("vad_hangover_s", 0.5)
        self.declare_parameter("preroll_s", 0.3)

        p = self.get_parameter
        self.source = p("source").value
        self.rate = p("sample_rate").value
        self.chunk = p("chunk_samples").value
        self.gain = p("gain").value
        self.doa_deg = int(p("doa_deg").value) % 360
        self.vad_min_db = p("vad_min_db").value
        self.vad_margin_db = p("vad_margin_db").value
        self.hangover_chunks = max(1, round(p("vad_hangover_s").value * self.rate / self.chunk))
        preroll_chunks = max(0, round(p("preroll_s").value * self.rate / self.chunk))

        self.audio_pub = self.create_publisher(AudioData, "/audio", STREAM_QOS)
        self.channel_pubs = [
            self.create_publisher(AudioData, f"/audio/channel{i}", STREAM_QOS)
            for i in range(CHANNEL_COUNT)
        ]
        self.speech_pub = self.create_publisher(AudioData, "/speech_audio", STREAM_QOS)
        self.speeching_pub = self.create_publisher(Bool, "/is_speeching", LATCHED_QOS)
        self.direction_pub = self.create_publisher(Int32, "/sound_direction", LATCHED_QOS)
        self.localization_pub = self.create_publisher(
            PoseStamped, "/sound_localization", LATCHED_QOS
        )

        self.noise_floor_db = self.vad_min_db - self.vad_margin_db
        self.speaking = False
        self.quiet_chunks = 0
        self.preroll = collections.deque(maxlen=preroll_chunks)
        self.utterance = []
        self.publish_speeching(False)

        self.chunks = queue.Queue(maxsize=256)
        self.stop = threading.Event()
        self.process = None
        threading.Thread(target=self.capture_loop, daemon=True).start()

        self.create_timer(0.005, self.drain_chunks)
        self.create_timer(0.1, self.publish_direction)

    # ------------------------------------------------------------------ capture
    def capture_loop(self):
        generator = {
            "mic": self.read_microphone,
            "file": self.read_wav,
            "silence": self.read_silence,
        }.get(self.source)
        if generator is None:
            self.get_logger().error(f"Unknown source '{self.source}', using silence")
            generator = self.read_silence

        while not self.stop.is_set():
            for data in generator():
                if self.stop.is_set():
                    return
                try:
                    self.chunks.put_nowait(data)
                except queue.Full:
                    pass
            if generator is not self.read_silence:
                self.get_logger().error(
                    f"Audio source '{self.source}' stopped; publishing silence"
                )
                generator = self.read_silence

    def paced(self):
        period = self.chunk / self.rate
        deadline = time.monotonic()
        while not self.stop.is_set():
            deadline += period
            delay = deadline - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            yield

    def read_silence(self):
        silence = bytes(self.chunk * BYTES_PER_SAMPLE)
        for _ in self.paced():
            yield silence

    def read_wav(self):
        path = self.get_parameter("wav_file").value
        try:
            with wave.open(path, "rb") as wav:
                if (wav.getnchannels(), wav.getsampwidth(), wav.getframerate()) != (
                    1,
                    BYTES_PER_SAMPLE,
                    self.rate,
                ):
                    self.get_logger().error(
                        f"{path} must be mono, 16-bit, {self.rate} Hz"
                    )
                    return
                samples = wav.readframes(wav.getnframes())
        except (OSError, wave.Error) as err:
            self.get_logger().error(f"Cannot read wav_file '{path}': {err}")
            return

        step = self.chunk * BYTES_PER_SAMPLE
        position = 0
        for _ in self.paced():
            data = samples[position : position + step]
            position = (position + step) % max(len(samples), 1)
            yield data.ljust(step, b"\x00")

    def read_microphone(self):
        command = [
            "arecord",
            "-q",
            "-D",
            self.get_parameter("device").value,
            "-t",
            "raw",
            "-f",
            "S16_LE",
            "-r",
            str(self.rate),
            "-c",
            "1",
        ]
        try:
            self.process = subprocess.Popen(
                command, stdout=subprocess.PIPE, stderr=subprocess.PIPE
            )
        except OSError as err:
            self.get_logger().error(f"Cannot start arecord (install alsa-utils): {err}")
            return

        step = self.chunk * BYTES_PER_SAMPLE
        while not self.stop.is_set():
            data = self.process.stdout.read(step)
            if len(data) < step:
                error = self.process.stderr.read().decode(errors="replace").strip()
                self.get_logger().error(
                    f"Microphone capture failed on device '{self.get_parameter('device').value}': "
                    f"{error or 'no data'}. List devices with 'arecord -L'."
                )
                return
            yield data

    # ------------------------------------------------------------------ publish
    def drain_chunks(self):
        for _ in range(self.chunks.qsize()):
            self.process_chunk(self.chunks.get_nowait())

    def process_chunk(self, raw):
        samples = np.frombuffer(raw, dtype=np.int16)
        if self.gain != 1.0:
            samples = np.clip(samples * self.gain, -32768, 32767).astype(np.int16)
            raw = samples.tobytes()

        self.publish_audio(raw)
        self.update_speech_state(raw, samples)

    def publish_audio(self, raw):
        mic = AudioData(data=raw)
        silent = AudioData(data=bytes(len(raw)))
        self.audio_pub.publish(mic)
        for index, pub in enumerate(self.channel_pubs):
            pub.publish(mic if index == 0 or index in RAW_MIC_CHANNELS else silent)

    def update_speech_state(self, raw, samples):
        rms = math.sqrt(float(np.mean((samples / 32768.0) ** 2)))
        level_db = 20.0 * math.log10(rms) if rms > 0.0 else SILENCE_DB
        threshold_db = max(self.vad_min_db, self.noise_floor_db + self.vad_margin_db)
        voiced = level_db > threshold_db

        if not self.speaking:
            self.noise_floor_db = 0.98 * self.noise_floor_db + 0.02 * level_db
            if voiced:
                self.speaking = True
                self.quiet_chunks = 0
                self.utterance = list(self.preroll)
                self.publish_speeching(True)
            else:
                self.preroll.append(raw)
                return

        self.utterance.append(raw)
        self.quiet_chunks = 0 if voiced else self.quiet_chunks + 1
        if self.quiet_chunks >= self.hangover_chunks:
            self.speaking = False
            self.preroll.clear()
            self.speech_pub.publish(AudioData(data=b"".join(self.utterance)))
            self.utterance = []
            self.publish_speeching(False)

    def publish_speeching(self, value):
        self.speeching_pub.publish(Bool(data=value))

    def publish_direction(self):
        self.direction_pub.publish(Int32(data=self.doa_deg))
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "base_link"
        angle = math.radians(self.doa_deg)
        pose.pose.position.x = math.cos(angle)
        pose.pose.position.y = math.sin(angle)
        pose.pose.orientation.w = 1.0
        self.localization_pub.publish(pose)

    def destroy_node(self):
        self.stop.set()
        if self.process is not None:
            self.process.terminate()
        super().destroy_node()


def main():
    rclpy.init()
    node = RespeakerSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
