#For Landmark Indicies, See:https://medium.com/@sreeananthakannan/hand-tracking-with-21-landmarks-c386beafeaf2

import cv2
import mediapipe as mp
import numpy as np
import time
import serial

cap = cv2.VideoCapture(0)
mpHands = mp.solutions.hands
hands = mpHands.Hands(static_image_mode=False, max_num_hands=1, min_detection_confidence=0.9)
mpDraw = mp.solutions.drawing_utils

escapeCount = 0
resetExit = True


#This code writes data to Serial Port
def Serialsend(anglenum, angleval):
    #print('sending', y)
    time.sleep(.0001)
    message = str(anglenum) + ", " + str(angleval) + '\n'
    ardu.write(message.encode())

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
    ardu = serial.Serial('/dev/cu.usbserial-141140', 115200)  ##INSERT PORT NAME AND BAUD RATE
    success, image = cap.read()
    imageRGB = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    results = hands.process(imageRGB)



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
                if id == 8:
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

                    x569 = (index_mcp+middle_mcp)/2 - index_pip
                    x5910 = (index_mcp+middle_mcp)/2 - middle_pip

                    x91013 = (middle_mcp+ring_mcp)/2 - middle_pip
                    x91314 = (middle_mcp+ring_mcp)/2 - ring_pip

                    x131417 = (ring_mcp+pinky_mcp)/2 - ring_pip
                    x131718 = (ring_mcp + pinky_mcp)/2 - pinky_pip

                    x08 = wrist - index_tip

                    x59 = index_mcp - middle_mcp
                    x913 = middle_mcp - ring_mcp
                    x1317 = ring_mcp - pinky_mcp

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
                    angle20 = 2 * np.degrees(np.arccos(np.dot(x56, x910) / (np.linalg.norm(x56) * np.linalg.norm(x910))))
                    angle21 = 2 * np.degrees(np.arccos(np.dot(x910,x1314)/(np.linalg.norm(x910) * np.linalg.norm(x1314))))
                    angle22 = 2 * np.degrees(np.arccos(np.dot(x1314,x1718)/(np.linalg.norm(x1314) * np.linalg.norm(x1718))))
                    angle23 = -angle20 / 2

                    if not np.isnan(angle1):
                        angle1 = int(angle1)
                    if not np.isnan(angle2):
                        angle2 = int(angle2)
                    if not np.isnan(angle3):
                        angle3 = int(angle3)
                    if not np.isnan(angle5):
                        angle5 = int(angle5)
                    if not np.isnan(angle6):
                        angle6 = int(angle6)
                    if not np.isnan(angle7):
                        angle7 = int(angle7)
                    if not np.isnan(angle9):
                        angle9 = int(angle9)
                    if not np.isnan(angle10):
                        angle10 = int(angle10)
                    if not np.isnan(angle11):
                        angle11 = int(angle11)
                    if not np.isnan(angle13):
                        angle13 = int(angle13)
                    if not np.isnan(angle14):
                        angle14 = int(angle14)
                    if not np.isnan(angle15):
                        angle15 = int(angle15)
                    if not np.isnan(angle17):
                        angle17 = int(angle17)
                    if not np.isnan(angle18):
                        angle18 = int(angle18)
                    if not np.isnan(angle19):
                        angle19 = int(angle19)
                    if not np.isnan(angle20):
                        angle20 = int(angle20)
                    if not np.isnan(angle21):
                         angle21 = int(angle21)
                    if not np.isnan(angle22):
                        angle22 = int(angle22)
                    if not np.isnan(angle23):
                        angle23 = int(angle23)
                    # if 238-(angle13 + angle14 + angle15)/2 > 100:
                    #     angle23 = 0

                    print("#############BREAK##########")
                    # print('Angle1: ', angle1) #THUMB CMC (In Palm)
                    # print('Angle2: ', angle2) #Thumb Base Knuckle
                    # print('Angle3: ', angle3) #Thumb Middle Knuckle

                    ThumbKnuckles = 360 - angle2 - angle3
                    ThumbKnuckles = Saturate(ThumbKnuckles, 0, 180)
                    Serialsend(1, ThumbKnuckles)
                    print('Thumb Knuckles', ThumbKnuckles)
                    # print('Angle5: ', angle5) #Index Base Knuckle
                    # print('Angle6: ', angle6) #Index Second Knuckle
                    # print('Angle7: ', angle7) #Index Top Knuckle

                    IndexKnuckles = 230-(angle5 + angle6 + angle7)/2
                    IndexKnuckles = Saturate(IndexKnuckles, 0, 180)
                    Serialsend(2, IndexKnuckles)
                    print('Index Knuckles', IndexKnuckles)
                    # print('Angle9: ', angle9) #Middle Base Knuckle
                    # print('Angle10: ', angle10) #Middle Second Knuckle
                    # print('Angle11: ', angle11) #Middle Top Knuckle

                    MiddleKnuckles = 250-(angle9 + angle10 + angle11)/2
                    MiddleKnuckles = Saturate(MiddleKnuckles, 0, 180)
                    Serialsend(3, MiddleKnuckles)
                    print('Middle Knuckles', MiddleKnuckles)
                    # print('Angle13: ', angle13) #Ring Base Knuckle
                    # print('Angle14: ', angle14) #Ring Second Knuckle
                    # print('Angle15: ', angle15) #Ring Top Knuckle

                    RingKnuckles = 238-(angle13 + angle14 + angle15)/2
                    RingKnuckles = Saturate(RingKnuckles, 0, 180)
                    Serialsend(4, RingKnuckles)
                    print('Ring Knuckles', RingKnuckles)
                    # print('Angle17: ', angle17) #Pinky Base Knuckle
                    # print('Angle18: ', angle18) #Pinky Middle Knuckle
                    # print('Angle19: ', angle19) #Pinky Top Knuckle

                    PinkyKnuckles = 240-(angle17 + angle18 + angle19)/2
                    PinkyKnuckles = Saturate(PinkyKnuckles, 0, 180)
                    Serialsend(5, PinkyKnuckles)
                    print('Pinky Knuckles', PinkyKnuckles)

                    IndexTilt = abs(angle23)
                    if IndexKnuckles > 90:
                        IndexTilt = 0
                    IndexTilt = Saturate(IndexTilt, 0, 25)
                    Serialsend(7, IndexTilt)
                    print('Ref ', angle23) #Ring Finger for reference

                    MiddleTilt = angle23 + angle20
                    if MiddleKnuckles > 85:
                        MiddleTilt = 0
                    MiddleTilt = Saturate(MiddleTilt, 0, 25)
                    Serialsend(8, MiddleTilt)
                    print('Angle20: ', MiddleTilt) #Between index and middle

                    RingTilt = MiddleTilt+angle21
                    if RingKnuckles > 100:
                        RingTilt = MiddleTilt
                    RingTilt = Saturate(RingTilt, 0, 35)
                    Serialsend(9, RingTilt)
                    print('Angle21: ', RingTilt) #Between Middle and Ring

                    PinkyTilt = RingTilt + angle22/1.3 - 10
                    if PinkyKnuckles > 100:
                        PinkyTilt = RingTilt
                    PinkyTilt = Saturate(PinkyTilt, RingTilt, 55)
                    Serialsend(10, PinkyTilt)
                    print('Angle22 ', PinkyTilt) #Between Ring and Pinky
            mpDraw.draw_landmarks(image, handLms, mpHands.HAND_CONNECTIONS)


    cv2.imshow("Output", image)
    cv2.waitKey(1)

    if resetExit:
        escapeCount = 0

    k = cv2.waitKey(33)
   
    if k == 27 or escapeCount > 10:  # Esc key to stop
        break
