
from langchain.chat_models import ChatOpenAI


class STT:
    def __init__(self, openai_api_key):
        self.stt = ChatOpenAI(
            openai_api_key=openai_api_key,
            model="gpt-4o-audio-preview",
            temperature=0,
            max_tokens=None,
            timeout=None,
            max_retries=2
        )

    def speech2text(self, speech_data):
        messages = [
            ("human", [
                {"type": "text", "text": "Transcribe the following:"},
                {"type": "input_audio", "input_audio": {"data": speech_data, "format": "wav"}},
            ])
        ]

        output_message = self.stt.invoke(messages)
        print("output_message.content: ", output_message.content)
        return output_message

    
