#For Landmark Indicies, See:https://medium.com/@sreeananthakannan/hand-tracking-with-21-landmarks-c386beafeaf2

import cv2
import mediapipe as mp
import numpy as np
import time
import serial

cap = cv2.VideoCapture(0)
mpHands = mp.solutions.hands
hands = mpHands.Hands()
mpDraw = mp.solutions.drawing_utils

escapeCount = 0
resetExit = True


#This code writes data to Serial Port
# def Serialsend(y):
#     y = y/2
#     if y < 0:
#         y = 0
#     if y > 255:
#         y = 255
#     y = int(y)
#     #print('sending', y)
#     time.sleep(.00001)
#     ardu.write(str(y).encode())
#     time.sleep(.00001)
#     #ardu.close()

wrist = np.array([1.0,1.0,1.0])
thumb_cmc = np.array([1.0,1.0,1.0])
thumb_mcp = np.array([1.0,1.0,1.0])
thumb_ip = np.array([1.0,1.0,1.0])
thumb_tip = np.array([1.0,1.0,1.0])
index_mcp = np.array([1.0,1.0,1.0])
index_pip = np.array([1.0,1.0,1.0])
index_dip = np.array([1.0,1.0,1.0])
index_tip = np.array([1.0,1.0,1.0])
middle_mcp = np.array([1.0,1.0,1.0])
middle_pip = np.array([1.0,1.0,1.0])
middle_dip = np.array([1.0,1.0,1.0])
middle_tip = np.array([1.0,1.0,1.0])
ring_mcp = np.array([1.0,1.0,1.0])
ring_pip = np.array([1.0,1.0,1.0])
ring_dip = np.array([1.0,1.0,1.0])
ring_tip = np.array([1.0,1.0,1.0])
pinky_mcp = np.array([1.0,1.0,1.0])
pinky_pip = np.array([1.0,1.0,1.0])
pinky_dip = np.array([1.0,1.0,1.0])
pinky_tip = np.array([1.0,1.0,1.0])



while True:
   # ardu = serial.Serial('/dev/cu.usbmodem144301', 9600, timeout=.001)
    success, image = cap.read()
    imageRGB = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    results = hands.process(imageRGB)

    # checking whether a hand is detected
    #if results.multi_hand_landmarks:

     #   for hand_no, hand_landmarks in enumerate(results.multi_hand_landmarks):
            #print(f'HAND NUMBER: {hand_no + 1}')
            #print('---`--------------------')

            #for i in range(21):
               #print(f'{mpHands.HandLandmark(i).name}:')
               #print(f'{hand_landmarks.landmark[mpHands.HandLandmark(i).value]}')


    if results.multi_hand_landmarks:
        for handLms in results.multi_hand_landmarks:  # working with each hand
            for id, lm in enumerate(handLms.landmark):
                h, w, c = image.shape
                cx, cy = int(lm.x * w), int(lm.y * h) #Give position of landmarks
                if id == 0:
                    wrist = np.array([lm.x, lm.y, lm.z])
                if id == 1:
                    thumb_cmc = np.array([lm.x, lm.y, lm.z])
                if id == 2:
                    thumb_mcp = np.array([lm.x, lm.y, lm.z])
                if id == 3:
                    thumb_ip = np.array([lm.x, lm.y, lm.z])
                if id == 4:
                    thumb_tip = wrist = np.array([lm.x, lm.y, lm.z])
                if id == 5:
                    index_mcp = np.array([lm.x, lm.y, lm.z])
                if id == 6:
                    index_pip = np.array([lm.x, lm.y, lm.z])
                if id == 7:
                    index_dip = np.array([lm.x, lm.y, lm.z])
                if id == 8: #Index finger tip = ID 8
                    # y = cy
                    # Serialsend(y) #write landmark position to Serial
                    index_tip = np.array([lm.x, lm.y, lm.z])
                    if (cx < 1300 and cx > 1158) and (cy > 0 and cy < 75):
                        escapeCount += 1
                        if resetExit:
                            resetExit = False
                    else:
                        resetExit = True
                if id == 9:
                    middle_mcp = np.array([lm.x, lm.y, lm.z])
                if id == 10:
                    middle_pip = np.array([lm.x, lm.y, lm.z])
                if id == 11:
                    middle_dip = np.array([lm.x, lm.y, lm.z])
                if id == 12:
                    middle_tip = np.array([lm.x, lm.y, lm.z])
                if id == 13:
                    ring_mcp = np.array([lm.x, lm.y, lm.z])
                if id == 14:
                    ring_pip = np.array([lm.x, lm.y, lm.z])
                if id == 15:
                    ring_dip = np.array([lm.x, lm.y, lm.z])
                if id == 16:
                    ring_tip = np.array([lm.x, lm.y, lm.z])
                if id == 17:
                    pinky_mcp = np.array([lm.x, lm.y, lm.z])
                if id == 18:
                    pinky_pip = np.array([lm.x, lm.y, lm.z])
                if id == 19:
                    pinky_dip = np.array([lm.x, lm.y, lm.z])
                if id == 20:
                    pinky_tip = np.array([lm.x, lm.y, lm.z])

                    x01 = wrist-thumb_cmc
                    x12 = thumb_mcp - thumb_cmc
                    x23 = thumb_cmc - thumb_ip
                    x34 = thumb_ip - thumb_tip
                    x05 = wrist - index_mcp
                    x56 = index_mcp - index_pip
                    x67 = index_pip - index_dip
                    x78 = index_dip - index_tip
                    x09 = wrist + middle_mcp
                    x910 = middle_mcp - middle_pip
                    x1011 = middle_pip - middle_dip
                    x1112 = middle_dip - middle_tip
                    x013 = wrist-ring_mcp
                    x1314 = ring_mcp - ring_pip
                    x1415 = ring_pip - ring_dip
                    x1516 = ring_dip - ring_tip
                    x017 = wrist - pinky_mcp
                    x1718 = pinky_mcp - pinky_pip
                    x1819 = pinky_pip - pinky_dip
                    x1920 = pinky_dip - pinky_tip
                    angle1 = np.degrees(np.arccos(np.dot(x01,-x12)/(np.linalg.norm(x01) * np.linalg.norm(x12))))
                    angle2 = np.degrees(np.arccos(np.dot(x12,x23)/(np.linalg.norm(x12) * np.linalg.norm(x23))))
                    angle3 = np.degrees(np.arccos(np.dot(x23,-x34)/(np.linalg.norm(x23) * np.linalg.norm(x34))))
                    angle5 = np.degrees(np.arccos(np.dot(x05,-x56)/(np.linalg.norm(x05) * np.linalg.norm(x56))))
                    angle6 = np.degrees(np.arccos(np.dot(x56,-x67)/(np.linalg.norm(x56) * np.linalg.norm(x67))))
                    angle7 = np.degrees(np.arccos(np.dot(x67,-x78)/(np.linalg.norm(x67) * np.linalg.norm(x78))))
                    angle9 = np.degrees(np.arccos(np.dot(x09,-x910)/(np.linalg.norm(x09) * np.linalg.norm(x910))))
                    angle10 = np.degrees(np.arccos(np.dot(x910,-x1011)/(np.linalg.norm(x910) * np.linalg.norm(x1011))))
                    angle11 = np.degrees(np.arccos(np.dot(x1011,-x1112)/(np.linalg.norm(x1011) * np.linalg.norm(x1112))))
                    angle13 = np.degrees(np.arccos(np.dot(x013,-x1314)/(np.linalg.norm(x013) * np.linalg.norm(x1314))))
                    angle14 = np.degrees(np.arccos(np.dot(x1314,-x1415)/(np.linalg.norm(x1314) * np.linalg.norm(x1415))))
                    angle15 = np.degrees(np.arccos(np.dot(x1415,-x1516)/(np.linalg.norm(x1415) * np.linalg.norm(x1516))))
                    angle17 = np.degrees(np.arccos(np.dot(x017,-x1718)/(np.linalg.norm(x017) * np.linalg.norm(x1718))))
                    angle18 = np.degrees(np.arccos(np.dot(x1718,-x1819)/(np.linalg.norm(x1718) * np.linalg.norm(x1819))))
                    angle19 = np.degrees(np.arccos(np.dot(x1819,-x1920)/(np.linalg.norm(x1819) * np.linalg.norm(x1920))))
                    if not np.isnan(angle1):
                        angle1 = int(angle1)
                    if not np.isnan(angle2):
                        angle2 = int(angle2)
                    if not np.isnan(angle3):
                        angle3 = int(angle3)
                    if not np.isnan(angle5):
                        angle5 = int(angle5) + 50
                    if not np.isnan(angle6):
                        angle6 = int(angle6)
                    if not np.isnan(angle7):
                        angle7 = int(angle7)
                    if not np.isnan(angle9):
                        angle9 = int(angle9) + 40
                    if not np.isnan(angle10):
                        angle10 = int(angle10)
                    if not np.isnan(angle11):
                        angle11 = int(angle11)
                    if not np.isnan(angle13):
                        angle13 = int(angle13) + 40
                    if not np.isnan(angle14):
                        angle14 = int(angle14)
                    if not np.isnan(angle15):
                        angle15 = int(angle15)
                    if not np.isnan(angle17):
                        angle17 = int(angle17) + 38
                    if not np.isnan(angle18):
                        angle18 = int(angle18)
                    if not np.isnan(angle19):
                        angle19 = int(angle19)
                    print("#############BREAK##########")
                    print('Angle1: ', angle1)
                    print('Angle2: ', angle2)
                    print('Angle3: ', angle3)
                    print('Angle5: ', angle5)
                    print('Angle6: ', angle6)
                    print('Angle7: ', angle7)
                    print('Angle9: ', angle9)
                    print('Angle10: ', angle10)
                    print('Angle11: ', angle11)
                    print('Angle13: ', angle13)
                    print('Angle14: ', angle14)
                    print('Angle15: ', angle15)
                    print('Angle17: ', angle17)
                    print('Angle18: ', angle18)
                    print('Angle19: ', angle19)
            mpDraw.draw_landmarks(image, handLms, mpHands.HAND_CONNECTIONS)


    cv2.imshow("Output", image)
    cv2.waitKey(1)

    if resetExit:
        escapeCount = 0

    k = cv2.waitKey(33)
    if k == 27 or escapeCount > 10:  # Esc key to stop
        break