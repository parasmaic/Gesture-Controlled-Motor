import cv2
import mediapipe as mp
import numpy as np
import serial
import time
import struct

#initialize mediapipe hands
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

#initialize serial communication
ser = serial.Serial('/dev/tty.usbmodemDC5475C46BF02', 115200)
time.sleep(2) 

cap = cv2.VideoCapture(1)

if not cap.isOpened(): #for debugging
    print("error: could not open video capture")
    exit()

padding = 150 
min_x = padding
max_x = None  #will be determined based on frame width

#check if fist is made
def is_fist(hand_landmarks):
    wrist = hand_landmarks.landmark[mp_hands.HandLandmark.WRIST]
    distances = []
    
    for finger_tip in [mp_hands.HandLandmark.INDEX_FINGER_TIP, mp_hands.HandLandmark.MIDDLE_FINGER_TIP,
                       mp_hands.HandLandmark.RING_FINGER_TIP, mp_hands.HandLandmark.PINKY_TIP]:
        finger_tip_landmark = hand_landmarks.landmark[finger_tip]
        distance = np.sqrt((finger_tip_landmark.x - wrist.x) ** 2 + (finger_tip_landmark.y - wrist.y) ** 2)
        distances.append(distance)
    
    return all(distance < 0.25 for distance in distances)

#initialize hands object
with mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=1,
    min_detection_confidence=0.7
) as hands:
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print("error: failed to grab frame")
            break

        #flip, convert to RGB, and process frame
        frame = cv2.flip(frame, 1)
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = hands.process(frame_rgb)

        h, w, _ = frame.shape
        if max_x is None:
            max_x = w - padding
        
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

                if is_fist(hand_landmarks):
                    angle = 0 
                    x = None
                else:
                    index_finger_tip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
                    x = int(index_finger_tip.x * w)

                    #draw blue dot on fingertip
                    cv2.circle(frame, (x, int(index_finger_tip.y * h)), 5, (255, 0, 0), -1)

                    #calculate angle based on x position
                    if x < min_x:
                        angle = 0
                    elif x > max_x:
                        angle = 180
                    else:
                        angle = ((x - min_x) / (max_x - min_x)) * 180

                if x is not None:
                    # displaying the x coordinate of the index finger
                    cv2.putText(frame, f"Index Finger Tip X: {x}", (10, frame.shape[0] - 40),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
                #displaying the angle
                cv2.putText(frame, f"Angle: {angle:.2f} degrees", (10, frame.shape[0] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
                
                #send angle to the Arduino as a float
                ser.write(struct.pack("f", angle))

        cv2.imshow('Hand Detection', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()
ser.close()
