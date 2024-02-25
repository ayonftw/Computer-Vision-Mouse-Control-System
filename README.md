Eye and Hand Gesture Controlled Mouse
Overview
This project utilizes computer vision techniques to control the mouse cursor through eye and hand gestures. It combines the use of facial landmark detection and hand tracking to enable users to interact with their computer without traditional input devices like a mouse or keyboard.

Features
Eye Control: The mouse cursor is controlled by the user's eye movements. The program tracks the position of the user's eyes using facial landmark detection and adjusts the cursor accordingly.

Blink Detection: The system detects eye blinks to perform actions such as clicking or dragging the mouse cursor.

Hand Gesture Control: In addition to eye control, the program also tracks hand gestures using hand tracking techniques. Users can move the cursor by moving their hand in front of the camera.

Click and Drag: The system allows users to perform mouse clicks and drag operations using a combination of eye blinks and hand movements.

Requirements
Python 3.x
OpenCV
Mediapipe
PyAutoGUI
NumPy
Usage
Install the required dependencies using pip install -r requirements.txt.
Run the script eye_hand_control.py.
Position yourself in front of the camera so that your face and hands are clearly visible.
Use your eye movements and hand gestures to control the mouse cursor.
Press 'q' to quit the program.
