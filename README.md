please create a ros pkg, catkin_make, mkdir scripts, put these 4 files into scripts then source devel/setup.bash
please modify the address variable in both gemini_conversation_ros_1.py and handwave_detector_ros_1.py
please make sure your python version is > 3.9, 
or else create an venv outside the ros pkg and activate that venv
python -m pip install --upgrade pip setuptools wheel
pip install google-generativeai pygame SpeechRecognition gTTS mediapipe opencv-python numpy
finally, rosrun <ur_pkg_name> handwave_detector_ros_1.py
