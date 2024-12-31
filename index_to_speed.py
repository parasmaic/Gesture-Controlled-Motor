import cv2
import mediapipe as mp
import numpy as np
import serial
import struct
import time

#initialize hands module
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
hands = mp_hands.Hands()

#set up serial communication
try:
    ser = serial.Serial('/dev/tty.usbmodemDC5475C46BF02', 115200, timeout=1)
    time.sleep(2)
except serial.SerialException as e:
    print("serial error:", e)
    exit()

#set up opencv window
cv2.namedWindow("Hand Tracking", cv2.WINDOW_NORMAL)
cap = cv2.VideoCapture(1)

#variables
padding = 150
min_x = padding
max_x = None

#check if fist is made
def is_fist(hand_landmarks):
    wrist = hand_landmarks.landmark[mp_hands.HandLandmark.WRIST]
    distances = []
    for tip in [mp_hands.HandLandmark.INDEX_FINGER_TIP, mp_hands.HandLandmark.MIDDLE_FINGER_TIP,
                mp_hands.HandLandmark.RING_FINGER_TIP, mp_hands.HandLandmark.PINKY_TIP]:
        d = np.sqrt((hand_landmarks.landmark[tip].x - wrist.x) ** 2 +
                    (hand_landmarks.landmark[tip].y - wrist.y) ** 2)
        distances.append(d)
    return all(d < 0.25 for d in distances)

while cap.isOpened():
    ret, frame = cap.read()
    #for debugging
    if not ret:
        print("failed to grab frame")
        break

    frame = cv2.flip(frame, 1) #flip frame
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) #convert to rgb
    results = hands.process(frame_rgb)
    h, w, _ = frame.shape

    if max_x is None:
        max_x = w - padding

    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

            if is_fist(hand_landmarks):
                speed = 0
                x = None
            else:
                index_tip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
                x = int(index_tip.x * w)

                #drawing blue dot on fingertip
                cv2.circle(frame, (int(index_tip.x * w), int(index_tip.y * h)), 5, (255, 0, 0), -1)

                if x < min_x:
                    speed = 0
                elif x > max_x:
                    speed = 100
                else:
                    speed = ((x - min_x) / (max_x - min_x)) * 100

            if x is not None:
                cv2.putText(frame, f"Index X: {x}", (10, frame.shape[0] - 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.putText(frame, f"Speed: {speed:.2f}%", (10, frame.shape[0] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

            #sending speed to arduino
            try:
                ser.write(struct.pack("f", speed))
            except serial.SerialException as e:
                print("serial write error:", e)

    cv2.imshow("Hand Tracking", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
ser.close()
