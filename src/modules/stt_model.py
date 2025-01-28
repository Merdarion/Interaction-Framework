import numpy as np
import torch
from datetime import datetime, timedelta
import os
import re
import whisper

from time import sleep

def detect_repetitions(text):
    """
    Detects repeated words and phrases in the text.

    Args:
        text (str): The text to be checked.

    Returns:
        bool: True if repetitive pattern is detected, False otherwise.
    """
    # Check for repeated words
    if re.search(r'\b(\w+)[\s\W]*\1\b', text):
        return True
    # Check for repeated phrases (up to 3 words)
    for i in range(3, 0, -1):
        pattern = r'\b((?:\w+[\s\W]+){' + str(i) + r'}\w+)[\s\W]*\1\b'
        if re.search(pattern, text):
            return True
    return False

def whisper_stt(audio_model, data_queue, transcribe, key_input="e"):
    """
    Continuously processes audio data from a queue using a speech-to-text model until a complete phrase is detected.

    This function listens for audio data from the provided queue, processes it, and uses a speech recognition model to
    transcribe the audio into text. The transcription process continues until a complete phrase is detected or a 
    timeout period is exceeded.

    Args:
        audio_model: A speech recognition model with a `transcribe` method that accepts audio data as input.
        data_queue: A queue object that contains raw audio data in 16-bit integer PCM format.

    Returns:
        str: The transcribed text of the detected phrase. If no complete phrase is detected, it returns an empty string.

    Raises:
        KeyboardInterrupt: If the process is interrupted by the user.
    
    Notes:
        - The function expects the `audio_model` to have a `transcribe` method that returns a dictionary with a 'text' key.
        - The audio data in the queue is expected to be in raw PCM format with 16-bit integers.
        - The function processes audio data continuously until a complete phrase is detected or the user interrupts the process.
        - The `phrase_timeout` variable determines the timeout period for detecting a complete phrase.
    """
    phrase_time = None
    text = ""

    while True:    
        try:
            now = datetime.now()
            phrase_complete = False
            
            if not data_queue.empty():
                # Combine audio data from queue
                audio_data = b''.join(data_queue.queue)
                data_queue.queue.clear()
                
                # Convert in-ram buffer to something the model can use directly without needing a temp file.
                # Convert data from 16 bit wide integers to floating point with a width of 32 bits.
                # Clamp the audio stream frequency to a PCM wavelength compatible default of 32768hz max.

                audio_np = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32) / 32768.0


                if transcribe:
                    print("Transcribing...")
                    if key_input == "e":
                        result = audio_model.transcribe(audio_np, temperature=0, fp16=torch.cuda.is_available(), language="english")
                    elif key_input == "s":
                        result = audio_model.transcribe(audio_np, temperature=0, fp16=torch.cuda.is_available(), language="swedish")

                    text += " " + result['text'].strip().lower()
                else:
                    return None
                    
                if text[1:] in ("", " ", "you", "thank you", "thank you!", "thank you.", "you thank you", "you  thank you") or detect_repetitions(text):
                    print("I could not understand.")
                    text = ""
                    break
                else:
                    phrase_time = now
                    print("I've heard:", text[1:])

            if phrase_time and now - phrase_time > timedelta(seconds = 2):
                phrase_complete = True

            if phrase_complete and text != "" and text != ' ' and not detect_repetitions(text):
                sending = text[1:]
                text = ""

                return sending
            
            elif phrase_complete and (text == "" or text == " " or detect_repetitions(text)):
                text = ""
            else:
                sleep(0.05)

        except KeyboardInterrupt:
            break

def main():
    model = "base.en"
    audio_model = whisper.load_model(model)
    whisper_stt(audio_model)

if __name__ == "__main__":
    main()
