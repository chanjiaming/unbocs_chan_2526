import os

BASE_PATH = "/home/eevee/catkin_ws/src/fz_gemini"
API_KEY = "AIzaSyCoo4_XPXDreL1TJ4sGTklL2MKoEyxt46Q"
VOSK_MODEL_PATH = os.path.join(BASE_PATH, "vosk-model-small-en-us-0.15")
CAMERA_TOPIC = "/camera/rgb/image_raw"

# /camera/rgb/image_raw
# CAMERA_TOPIC = "/usb_cam/image_raw"
