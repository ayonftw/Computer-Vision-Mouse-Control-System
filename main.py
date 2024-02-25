import cv2
import mediapipe as mp
import pyautogui
import time
import numpy as np

face_mesh = mp.solutions.face_mesh.FaceMesh(refine_landmarks=True)
hands = mp.solutions.hands.Hands(static_image_mode=False, max_num_hands=1, min_detection_confidence=0.5,
                                 min_tracking_confidence=0.5)
screen_w, screen_h = pyautogui.size()

cam = cv2.VideoCapture(0)

eye_closed_start = None
dragging = False


def calculate_eye_aspect_ratio(landmarks, eye_points):
    p1 = np.array([landmarks[eye_points[0]].x, landmarks[eye_points[0]].y])
    p2 = np.array([landmarks[eye_points[1]].x, landmarks[eye_points[1]].y])
    eye_distance = np.linalg.norm(p1 - p2)
    return eye_distance


while True:
    _, frame = cam.read()
    frame = cv2.flip(frame, 1)
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Process face mesh
    face_output = face_mesh.process(rgb_frame)
    landmark_points = face_output.multi_face_landmarks
    frame_h, frame_w, _ = frame.shape

    # Process hand landmarks
    hands_output = hands.process(rgb_frame)
    hand_landmarks = None
    if hands_output.multi_hand_landmarks:
        hand_landmarks = hands_output.multi_hand_landmarks[0]

    if landmark_points:
        landmarks = landmark_points[0].landmark

        # Calculate screen position based on eye landmarks
        eye_x = int((landmarks[474].x + landmarks[475].x) / 2 * frame_w)
        eye_y = int((landmarks[474].y + landmarks[475].y) / 2 * frame_h)
        screen_x = int(eye_x / frame_w * screen_w)
        screen_y = int(eye_y / frame_h * screen_h)

        # Move mouse cursor using eyes
        pyautogui.moveTo(screen_x, screen_y)

        # Check if the left eye is closed for blink or drag
        left_eye_aspect_ratio = calculate_eye_aspect_ratio(landmarks, [145, 159])
        if left_eye_aspect_ratio < 0.006:  # Threshold for eye closure
            if eye_closed_start is None:
                eye_closed_start = time.time()  # Mark the start time of eye closure
            elif time.time() - eye_closed_start > 0.5:  # If eyes are closed longer than 0.5 seconds
                if not dragging:
                    pyautogui.mouseDown()  # Start dragging
                    dragging = True
        else:
            if dragging:
                pyautogui.mouseUp()  # Release the drag if eyes are opened
                dragging = False
            elif eye_closed_start is not None and time.time() - eye_closed_start <= 0.5:
                pyautogui.click()  # Perform a click if eyes were not closed long enough for dragging
            eye_closed_start = None  # Reset the eye closure start time

    # Use hand gestures to control mouse cursor
    if hand_landmarks:
        for landmark in hand_landmarks.landmark:
            hand_x = int(landmark.x * frame_w)
            hand_y = int(landmark.y * frame_h)
            screen_x = int(hand_x / frame_w * screen_w)
            screen_y = int(hand_y / frame_h * screen_h)
            pyautogui.moveTo(screen_x, screen_y)

    cv2.imshow('Combined Control', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to quit
        break

cam.release()
cv2.destroyAllWindows()