from dataclasses import dataclass
import wave
import io
import pyaudio

############ Mic Config ############

@dataclass
class MicConfig:
    chunk: int = 12000
    rate: int = 48000
    channels: int = 1
    record_seconds: int = 5
    fmt: int = pyaudio.paInt16
    device_index: int = 10
    buffer_size: int = 24000
    # check your device index
    # import pyaudio
    # p = pyaudio.PyAudio()
    # [(i, p.get_device_info_by_index(i)['name']) for i in range(p.get_device_count())]


class MicController:
    def __init__(self, config: MicConfig = MicConfig()):
        self.config = config
        self.frames = []
        self.audio = None     # open_stream()에서 생성
        self.stream = None
        self.sample_width = None  # 스트림 열 때 샘플 폭을 저장

    def open_stream(self):
        """새로운 PyAudio 인스턴스를 생성하고 스트림을 엽니다."""
        self.audio = pyaudio.PyAudio()
        self.sample_width = self.audio.get_sample_size(self.config.fmt)
        self.stream = self.audio.open(
            format=self.config.fmt,
            channels=self.config.channels,
            rate=self.config.rate,
            input=True,
            frames_per_buffer=self.config.chunk,
            # input_device_index=self.config.device_index
        )

    def record_audio(self):
        print("start recording for 5 seconds")
        self.frames = []  # 이전 프레임 초기화
        num_chunks = int(self.config.rate / self.config.chunk * self.config.record_seconds)

        for _ in range(num_chunks):
            data = self.stream.read(self.config.chunk, exception_on_overflow=False)
            self.frames.append(data)

    def close_stream(self):
        """스트림과 PyAudio 인스턴스를 종료합니다."""
        print("stop recording")
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
            # self.stream = None
        if self.audio:
            self.audio.terminate()
            self.audio = None

    def save_wav(self, filename):
        """녹음된 데이터를 WAV 파일로 저장합니다."""
        with wave.open(filename, 'wb') as wf:
            wf.setnchannels(self.config.channels)
            wf.setsampwidth(self.sample_width)
            wf.setframerate(self.config.rate)
            wf.writeframes(b''.join(self.frames))
        print("✅ 파일 저장 완료!")

    def get_wav_data(self):
        wav_buffer = io.BytesIO()
        with wave.open(wav_buffer, 'wb') as wf:
            wf.setnchannels(self.config.channels)
            wf.setsampwidth(self.audio.get_sample_size(self.config.fmt))
            wf.setframerate(self.config.rate)
            wf.writeframes(b''.join(self.frames))
        return wav_buffer.getvalue()
