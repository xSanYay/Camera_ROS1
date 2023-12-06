import numpy as np

import cv2

import cv2.aruco as aruco

import math

import time

#Checks if a matrix is a valid rotation matrix.

def isRotationMatrix(R):

    Rt=np.transpose(R)

    shouldBeIdentity = np.dot (Rt, R)

    I= np.identity(3, dtype=R.dtype)

    n = np.linalg.norm(I - shouldBeIdentity)

    return n< 1e-6

#Calculates rotation matrix to euler angles

#The result is the same as MATLAB except the order # of the euler angles (x and z are swapped).

def rotationMatrixToEulerAngles (R): 
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] *R[0, 0]+R[1, 0] *R[1, 0])

    singular = sy < 1e-6

    if not singular:

        x = math.atan2(R[2, 1], R[2,2])

        y = math.atan2(-R[2, 0], sy)

        z = math.atan2 (R[1, 0], R[0, 0])

    else:

        x = math.atan2(-R[1, 2], R[1, 1])

        y = math.atan2(-R[2, 0], sy)

        z=0

    return np.array([x, y, z])

marker_size = 10

calib_data_path = r"C:\Users\91779\Desktop\internship\calib_data\MultiMatrix.npz"

calib_data = np.load(calib_data_path)
print(calib_data.files)

camera_matrix = calib_data["camMatrix"]
camera_distortion = calib_data["distCoef"]

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)

cap = cv2.VideoCapture (0)

camera_width=640

camera_height = 480

camera_frame_rate=40

cap.set(2, camera_width)

cap.set(4, camera_height)

cap.set(5, camera_frame_rate)

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
parameters =  cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, parameters)

while True:

# grab a frame

    ret, frame = cap.read()

    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # Convert to Grayscale

    # Find all the aruco markers in the image 
    
    corners,ids, rejected = detector.detectMarkers(frame)

    if ids is not None:

        aruco.drawDetectedMarkers (frame, corners ) # draw a box around all the detected markers

        #get pose of all single markers
        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, marker_size,camera_matrix,camera_distortion)
        rvec = rvec[0][0]
        tvec = tvec[0][0]

        # aruco.drawAxis(frame, camera_matrix, camera_distortion, rvec, tvec, 100)
        
        rvec_flipped = rvec * -1

        tvec_flipped = tvec * -1

        rotation_matrix, jacobian = cv2.Rodrigues(rvec_flipped)

        realworld_tvec = np.dot(rotation_matrix, tvec_flipped)
        # print(realworld_tvec)
        
        pitch, roll, yaw = rotationMatrixToEulerAngles(rotation_matrix)
        distance = np.sqrt(realworld_tvec[0]**2 + realworld_tvec[1]**2+ realworld_tvec[2]**2)
        tvec_str = f"x-{realworld_tvec[0]:4.0f} y={realworld_tvec[1]:4.0f} direction={math.degrees(roll):4.0f} "
        if distance < 40 :
            print("STOP")        
        elif math.degrees(roll)<-6:
            print("Turn right")
        elif math.degrees(roll)>6:
            print("Turn left")
        else:
            print("move straight")
        cv2.putText(frame,f"id: {ids[0]} Dist: {round(distance, 2)}",(20,100),cv2.FONT_HERSHEY_PLAIN,1.3,(0, 0, 255),2,cv2.LINE_AA)
        cv2.putText(frame, tvec_str, (20, 460), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 255), 2, cv2.LINE_AA)
        # current_time = time.time()
        # elapsed_time = current_time - start_time
        # current_fps = 1 / elapsed_time
        # print(f"Current FPS: {current_fps:.2f}")
    # else:
    #     print("No markers detected.")
        
    cv2.imshow("frame", frame)

    key= cv2.waitKey(1) & 0xFF

    if key==ord('q'): break

cap.release() 
cv2.destroyAllWindows()

# start_time = time.time()
    
    