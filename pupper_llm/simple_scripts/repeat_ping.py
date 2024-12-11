import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyaudio
import wave
import numpy as np
import webrtcvad
import collections
import time
from openai import OpenAI

# Audio parameters
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000
CHUNK_DURATION_MS = 30
PADDING_DURATION_MS = 1500
CHUNK_SIZE = int(RATE * CHUNK_DURATION_MS / 1000)
PADDING_SIZE = int(RATE * PADDING_DURATION_MS / 1000)

# VAD parameters
vad = webrtcvad.Vad(1)  # Set aggressiveness to 1 (least aggressive)

client = OpenAI(api_key='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')

class CommandLinePublisher(Node):
    def __init__(self):
        super().__init__('command_line_publisher')

        # Create a publisher for the user query topic
        self.publisher_ = self.create_publisher(
            String,
            'user_query_topic',  # Replace with the topic name used in your GPT-4 node
            10
        )
        self.get_logger().info('Command Line Publisher Node has started.')

    def publish_message(self, message):
        msg = String()
        msg.data = message
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{message}"')
            
    def transcribe_audio_with_whisper(self, filename, sample_rate=16000):
        try:
            print("Transcribing audio using Whisper model...")
            audio_file = open(filename, "rb")
            transcription = client.audio.transcriptions.create(
                model="whisper-1", file=audio_file, response_format="text")
            self.get_logger().info(f"Transcription response: {transcription}")
            return transcription
        except Exception as e:
            print(f"Error during transcription: {e}")
            return None


def is_speech(audio_chunk, threshold):
    return np.abs(np.frombuffer(audio_chunk, dtype=np.int16)).mean() > threshold

def calibrate_noise(stream, duration=5):
    print("Calibrating noise floor. Please remain silent...")
    chunks = []
    for _ in range(0, int(RATE / CHUNK_SIZE * duration)):
        chunks.append(np.frombuffer(stream.read(CHUNK_SIZE), dtype=np.int16))
    noise_floor = np.concatenate(chunks).mean() + 100  # Adding a small buffer
    print(f"Noise floor set to {noise_floor}")
    return noise_floor

def create_audio_stream():
    p = pyaudio.PyAudio()
    stream = p.open(format=FORMAT,
                    channels=CHANNELS,
                    rate=RATE,
                    input=True,
                    frames_per_buffer=CHUNK_SIZE)
    return p, stream

def record_audio():
    rclpy.init()

    command_publisher = CommandLinePublisher()

    p, stream = create_audio_stream()
    noise_floor = calibrate_noise(stream)
    
    try:
        while True:  # Main loop for continuous recording
            print("Listening... (Press Ctrl+C to stop)")

            frames = collections.deque(maxlen=PADDING_SIZE)
            recording = False
            recording_frames = []
            num_voiced = 0
            num_unvoiced = 0
            
            while True:  # Inner loop for a single recording session
                try:
                    chunk = stream.read(CHUNK_SIZE)
                    is_voiced = vad.is_speech(chunk, RATE) and is_speech(chunk, noise_floor)

                    if not recording:
                        frames.append(chunk)
                        if is_voiced:
                            num_voiced += 1
                        else:
                            num_voiced = 0
                        
                        if num_voiced > 10:  # Require 10 consecutive voiced chunks to start recording
                            print("Speech detected, recording...")
                            recording = True
                            recording_frames = list(frames)
                    else:
                        recording_frames.append(chunk)
                        if is_voiced:
                            num_unvoiced = 0
                        else:
                            num_unvoiced += 1
                        
                        if num_unvoiced > 30:  # Stop after ~2 second of silence
                            break
                except IOError as e:
                    if e.errno == pyaudio.paInputOverflowed:
                        print("Warning: Input overflowed, resetting stream.")
                        stream.close()
                        p.terminate()
                        p, stream = create_audio_stream()
                    else:
                        raise

            print("Finished recording")
            audio_data = b''.join(recording_frames)

            filename = "recorded_audio.wav"

            if audio_data:
                save_audio(audio_data, filename)
                try:
                    print("Transcribing audio using Whisper model...")
                    audio_file = open(filename, "rb")
                    transcription = client.audio.transcriptions.create(
                        model="whisper-1", file=audio_file, response_format="text")
                    command_publisher.publish_message(transcription)
                    print(f"Transcription: {transcription}")
                    
                except Exception as e:
                    print(f"Error during transcription: {e}")
            else:
                print("No audio was recorded.")

            # Reset the stream for the next recording session
            stream.close()
            p.terminate()
            p, stream = create_audio_stream()

    except KeyboardInterrupt:
        print("Stopping...")
        command_publisher.destroy_node()
        rclpy.shutdown()
    finally:
        if stream.is_active():
            stream.stop_stream()
        stream.close()
        p.terminate()

def save_audio(audio_data, filename):
    wf = wave.open(filename, 'wb')
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(pyaudio.get_sample_size(FORMAT))
    wf.setframerate(RATE)
    wf.writeframes(audio_data)
    wf.close()
    print(f"Audio saved as {filename}")

if __name__ == "__main__":
    record_audio()