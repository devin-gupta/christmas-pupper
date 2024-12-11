import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sounddevice as sd
import webrtcvad
import numpy as np
import wave
import io
import whisper as wh

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
        self.model = wh.load_model("tiny")

    def publish_message(self, message):
        msg = String()
        msg.data = message
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{message}"')

    def transcribe_audio_with_whisper(self, audio_data, sample_rate=16000):
        try:
            print("Transcribing audio using Whisper model...")
            wav_io = audio_to_wav(audio_data, sample_rate)
            wav_io.seek(0)
            transcription = self.model.transcribe(wav_io.read(), fp16=False)
            self.get_logger().info(f"Transcription: {transcription}")
            return transcription['text']
        except Exception as e:
            print(f"Error during transcription: {e}")
            return None

def audio_to_wav(audio_data, sample_rate=16000):
    """Convert numpy array audio to WAV format"""
    wav_io = io.BytesIO()
    with wave.open(wav_io, 'wb') as wav_file:
        wav_file.setnchannels(1)
        wav_file.setsampwidth(2)  # 16-bit PCM
        wav_file.setframerate(sample_rate)
        wav_file.writeframes(audio_data.tobytes())
    return wav_io

def main(args=None):
    rclpy.init(args=args)

    command_publisher = CommandLinePublisher()
    vad = webrtcvad.Vad()
    vad.set_mode(3)  # Set VAD aggressiveness mode (0-3, 3 is most aggressive)

    sample_rate = 16000
    chunk_duration = 0.03  # 30 ms
    chunk_size = int(sample_rate * chunk_duration)

    audio_buffer = []
    silent_chunks = 0
    max_silent_chunks = 20  # Adjust to control how much silence ends a recording

    command_publisher.get_logger().info("Listening for speech...")

    try:
        def audio_callback(indata, frames, time, status):
            nonlocal audio_buffer, silent_chunks
            if status:
                print(f"Audio callback error: {status}")

            # Convert to 16-bit PCM format
            pcm_data = (indata * 32768).astype(np.int16)

            for i in range(0, len(pcm_data), chunk_size):
                chunk = pcm_data[i:i + chunk_size]
                if len(chunk) < chunk_size:
                    continue

                is_speech = vad.is_speech(chunk.tobytes(), sample_rate)

                if is_speech:
                    audio_buffer.append(chunk)
                    silent_chunks = 0
                elif audio_buffer:
                    silent_chunks += 1
                    if silent_chunks > max_silent_chunks:
                        # End of speech detected
                        full_audio = np.concatenate(audio_buffer)
                        audio_buffer = []
                        silent_chunks = 0

                        command_publisher.get_logger().info("Processing detected speech...")
                        user_input = command_publisher.transcribe_audio_with_whisper(full_audio, sample_rate)

                        if user_input and user_input.lower() == 'exit':
                            command_publisher.get_logger().info("Exiting the publisher.")
                            raise KeyboardInterrupt

                        if user_input:
                            command_publisher.publish_message(user_input)

        # Start the audio stream
        with sd.InputStream(samplerate=sample_rate, channels=1, callback=audio_callback):
            command_publisher.get_logger().info("Audio stream started. Speak into the microphone.")
            rclpy.spin(command_publisher)

    except KeyboardInterrupt:
        print("Interrupted by user. Exiting...")

    command_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
