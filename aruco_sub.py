#!/usr/bin/env python
import rospy # Python library for ROS
from sensor_msgs.msg import Image
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
from cv2 import aruco
import numpy as np
import math
import time
import os




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



relative_path = "~/opencam/src/cv_basics/data/MultiMatrix.npz"

# Use os.path.expanduser to get the full path
calib_data_path = os.path.expanduser(relative_path)





calib_data = np.load(calib_data_path)
print(calib_data.files)

camera_matrix = calib_data["camMatrix"]
camera_distortion = calib_data["distCoef"]

dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)





aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters_create()



















def callback(data):

  # Used to convert between ROS and OpenCV images
  br = CvBridge()

  # Output debugging information to the terminal
  rospy.loginfo("receiving video frame")

  # Convert ROS Image message to OpenCV image
  frame = br.imgmsg_to_cv2(data)
  

  # Display image
    
  

 

  #ret, frame = cap2.read()

  gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # Convert to Grayscale

    # Find all the aruco markers in the image 
    
  corners,ids, rejected = cv2.aruco.detectMarkers(frame,dictionary,parameters=parameters)

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
        
        
        pitch, roll, yaw = rotationMatrixToEulerAngles(rotation_matrix)
        distance = np.sqrt(realworld_tvec[0]**2 + realworld_tvec[1]**2+ realworld_tvec[2]**2)
        tvec_str = "x={realworld_tvec[0]:4.0f} y={realworld_tvec[1]:4.0f} direction={math.degrees(roll):4.0f}"
        if distance < 40 :
            print("STOP")        
        elif math.degrees(roll)<-6:
            print("Turn right")
        elif math.degrees(roll)>6:
            print("Turn left")
        else:
            print("move straight")
        cv2.putText(frame, "id: {0} Dist: {1}".format(ids[0], round(distance, 2)), (20, 100),cv2.FONT_HERSHEY_PLAIN, 1.3, (0, 0, 255), 2, cv2.LINE_AA)

	cv2.putText(frame, tvec_str, (20, 460), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 255), 2, cv2.LINE_AA)

  else:
    	rospy.loginfo("No markers detected.")
        
  cv2.imshow("frame", frame)
  cv2.waitKey(1)






























  

def receive_message():

  # Tells rospy the name of the node. Anonymous = True makes sure the node 
  # has a unique name. Random numbers are added to the end of the name.
  rospy.init_node('video_sub_py', anonymous=True)

  # Node is subscribing to the video_frames topic
  rospy.Subscriber('video_frames', Image, callback)

  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()

  # Close down the video stream when done
  cv2.destroyAllWindows()

if __name__ == '__main__':
  receive_message()
