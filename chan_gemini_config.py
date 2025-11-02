#!/usr/bin/env python3

import google.generativeai as genai
import rospy
from chan_config import API_KEY

def configure_gemini():
    try:
        genai.configure(api_key=API_KEY)
        generation_config = {
            "temperature": 0.5,
            "top_p": 0.95,
            "top_k": 64,
            "max_output_tokens": 200,
            "response_mime_type": "text/plain",
        }

        safety_settings = [
            {
                "category": "HARM_CATEGORY_HARASSMENT",
                "threshold": "BLOCK_NONE",
            },
            {
                "category": "HARM_CATEGORY_HATE_SPEECH",
                "threshold": "BLOCK_ONLY_HIGH",
            },
            {
                "category": "HARM_CATEGORY_SEXUALLY_EXPLICIT",
                "threshold": "BLOCK_ONLY_HIGH",
            },
            {
                "category": "HARM_CATEGORY_DANGEROUS_CONTENT",
                "threshold": "BLOCK_ONLY_HIGH",
            },
        ]

        generative_model = genai.GenerativeModel(
            model_name="gemini-1.5-flash",
            safety_settings=safety_settings,
            generation_config=generation_config,
            system_instruction="""If the input is NOT about Airost, then aswer without referring to following prompt. Just answer normally, like greeting"
                "Gemini you are now representing to promoto Airost, which is a robotic club to the new students, limit your answer into 20 to 30 words at most."
                "What is Airost? Airost is a student club in UTM that promotes collaboration between students, "
                "industries, and universities in AI, Robotics, and IoT. Airost provides a platform for students to discover their "
                "interest and talents in engineering, technology, and entrepreneurship, enhancing their employability. "
                "Our adviser is AP. DR. Yeong Che Fai, and we have won numerous achievements in competitions such as the "
                "Huawei Malaysia Sales Elite Challenge, National Instrument Autonomous Robotics Competition, Robocup Japan and Malaysia, and more.""",
        )

        return generative_model

    except Exception as e:
        rospy.logerr("Failed to configure Generative AI: %s", e)
        return None
