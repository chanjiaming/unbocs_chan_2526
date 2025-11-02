#!/home/eevee/unbocs_2526_venv/bin/python

import socket
import os
import pygame
import speech_recognition as sr
from gtts import gTTS
import subprocess
import rospy 
from chan_gemini_config import configure_gemini

generative_model = configure_gemini()
address = "/home/eevee/catkin_ws/src/unbocs_chan_2526"
class GeminiChat:
    def __init__(self):
        self.user_name = None
        self.conversation_over = False  # Track when the user says goodbye
        pygame.init()
        rospy.loginfo("Chat Initialized")

    def check_internet_connection(self):
        try:
            socket.create_connection(("8.8.8.8", 53), timeout=3)
            return True
        except OSError:
            return False

    def speak(self, text):
        """Speak the given text using gTTS and play it using pygame."""
        tts = gTTS(text=text, lang='en')
        audio_file = "temp_audio.mp3"
        tts.save(audio_file)
        pygame.mixer.music.load(audio_file)
        pygame.mixer.music.play()
        while pygame.mixer.music.get_busy():
            continue
        try:
            os.remove(audio_file)
        except OSError:
            pass

    def generate_gemini_response(self, prompt):
        """Generate a response using the Gemini model."""
        if generative_model:
            try:
                response = generative_model.generate_content([prompt])
                if response and response.text:
                    return response.text.strip()
                else:
                    rospy.logwarn("Model returned no response.")
            except Exception as e:
                rospy.logerr(f"Failed to generate response: {e}")
        else:
            rospy.logwarn("Model is not configured.")
        return None

    def listen(self):
        """Capture audio input and convert it to text using SpeechRecognition."""
        recognizer = sr.Recognizer()
        microphone = sr.Microphone()

        with microphone as source:
            rospy.loginfo("Listening for your response...")
            recognizer.adjust_for_ambient_noise(source)
            audio = recognizer.listen(source)

        try:
            rospy.loginfo("Recognizing speech...")
            user_input = recognizer.recognize_google(audio)
            rospy.loginfo(f"You: {user_input}")
            return user_input
        except sr.UnknownValueError:
            rospy.loginfo("Sorry, I didn't catch that.")
            self.speak("Sorry, I didn't catch that. Could you please repeat?")
            return None
        except sr.RequestError:
            rospy.logwarn("Could not request results from the speech recognition service.")
            self.speak("I couldn't connect to the speech recognition service. Please try again.")
            return None

    def run(self):
        welcome_message = "Welcome to Airost! Would you like to ask any question?"
        self.speak(welcome_message)
        rospy.loginfo("Welcome to Airost! Would you like to ask any question?")

        while not self.conversation_over and not rospy.is_shutdown():
            user_input = self.listen()

            if user_input is None:
                continue

            if "goodbye" in user_input.lower():
                self.conversation_over = True
                farewell_message = "Goodbye! It was nice talking to you."
                rospy.loginfo(f"{farewell_message}")
                self.speak(farewell_message)
                break

            bot_response = self.generate_gemini_response(user_input)

            if bot_response:
                rospy.loginfo(f"{bot_response}")
                self.speak(bot_response)
            else:
                error_message = "I didn't quite understand that. Could you please repeat?"
                rospy.loginfo(f"{error_message}")
                self.speak(error_message)

        # After saying goodbye, restart the hand waving script
        self.initiate_hand_wave_detection()

    def initiate_hand_wave_detection(self):
        """Launch the hand_waving script after saying goodbye."""
        rospy.loginfo("Restarting hand wave detection...")
        subprocess.Popen(["python3", address + "/handwave_detector_ros_1.py"])


if __name__ == "__main__":
    rospy.init_node('gemini_chat_node', anonymous=True)
    try:
        gemini_chat = GeminiChat()
        gemini_chat.run()
    except rospy.ROSInterruptException:
        pass
