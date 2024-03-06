import cv2 as cv
from cv2 import aruco
import numpy as np
import math
import time

import serial

# arduino = serial.Serial('com6', 9600)


calib_data_path = r"C:\Users\91779\Desktop\AStra\IMAGE PROcesss\calib_data\MultiMatrix.npz"

calib_data = np.load(calib_data_path)
print(calib_data.files)
cam_mat = calib_data["camMatrix"]
dist_coef = calib_data["distCoef"]
r_vectors = calib_data["rVector"]
t_vectors = calib_data["tVector"]



dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_100)
parameters =  cv.aruco.DetectorParameters()
detector = cv.aruco.ArucoDetector(dictionary, parameters)

MARKER_SIZE = 5

marker_dict = dictionary

param_markers = parameters

cap = cv.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break
    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    marker_corners, marker_IDs, reject = aruco.detectMarkers(
        gray_frame, marker_dict, parameters=param_markers
    )

    if marker_corners:
        rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
            marker_corners, MARKER_SIZE, cam_mat, dist_coef
        )
        total_markers = range(0, marker_IDs.size)

        #angle ranges from - 180 to 180 using this function
    
        def ranges(angle):
            return (angle + 180) % 360 - 180


        for i in range(len(marker_IDs)):
            # Convert rotation vector to rotation matrix
            R, _ = cv.Rodrigues(rVec[i])
            
            # Extract roll, pitch, and yaw angles from rotation matrix
            roll, pitch, yaw = [math.degrees(x) for x in cv.RQDecomp3x3(R)[0]]
            
            # Print roll, pitch, and yaw angles
            
            
            print(f"Roll: {ranges(roll):.2f} degrees",f"Pitch: {ranges(pitch):.2f} degrees",f"Yaw: {ranges(yaw):.2f} degrees")

            print(f"Roll: {ranges(roll):.2f} degrees")
            print(f"Pitch: {ranges(pitch):.2f} degrees")
            print(f"Yaw: {ranges(yaw):.2f} degrees")

        for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
            cv.polylines(
                frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA
            )
            corners = corners.reshape(4, 2)
            corners = corners.astype(int)
            top_right = corners[0].ravel()
            top_left = corners[1].ravel()
            bottom_right = corners[2].ravel()
            bottom_left = corners[3].ravel()


            
            # calculate the distance
            distance = np.sqrt(
                tVec[i][0][2] ** 2 + tVec[i][0][0] ** 2 + tVec[i][0][1] ** 2
            )

           
            # Send distance to Arduino serial port
            # arduino.write(str((distance)).encode())
            # print(str((distance)).encode())
            # last_distance = distance
           
            # for pose of the marker
            point = cv.drawFrameAxes(frame, cam_mat, dist_coef, rVec[i], tVec[i], 4, 4)
            cv.putText(
                frame,
                f"id: {ids[0]} Dist: {round(distance*2.54, 2)}",
                top_left,
                cv.FONT_HERSHEY_PLAIN,
                1.3,
                (0, 0, 255),
                2,
                cv.LINE_AA,
            )
            cv.putText(
                frame,
                f"x:{round(tVec[i][0][0],1)} y: {round(tVec[i][0][1],1)} ",
                bottom_right,
                cv.FONT_HERSHEY_PLAIN,
                1.0,
                (0, 0, 255),
                2,
                cv.LINE_AA,
            )
            #cv.putText(
            #     frame,
            #     # f"Roll: {ranges(roll):.2f} Pitch: {ranges(pitch):.2f} Yaw: {ranges(yaw):.2f}  ",
            #     bottom_left,
            #     cv.FONT_HERSHEY_PLAIN,
            #     1.0,
            #     (0, 0, 255),
            #     2,
            #     cv.LINE_AA,
            # )

            # print(ids, "  ", corners)

            print( " id: ",ids)
            print() 
    cv.imshow("frame", frame)
    # time.sleep(1)
    key = cv.waitKey(1)
    if key == ord("q"):
        break
    
cap.release()
cv.destroyAllWindows()



#everthing is in inches
