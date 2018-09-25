import time
# import cv2.aruco as A
from cv2 import aruco as A
import numpy as np
import cv2
import os

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
board = cv2.aruco.CharucoBoard_create(3,3,.025,.0125,dictionary)
img = board.draw((200*3,200*3))

#Dump the calibration board to a file
# cv2.imwrite('charuco.png',img)


#Start capturing images for calibration
# cap = cv2.VideoCapture(0)

allCorners = []
allIds = []
decimator = 0


input_image_directory = '/Users/sebastinsanty/Desktop/charuco/'

input_image_path = []

for image in sorted(os.listdir(input_image_directory)):
	if not image.startswith('.'):
		final_path = input_image_directory + str(image)
		print(final_path)
		input_image_path.append(final_path)


for image in input_image_path:
	frame = cv2.imread(image)
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	res = cv2.aruco.detectMarkers(gray,dictionary)

	if len(res[0])>0:
		res2 = cv2.aruco.interpolateCornersCharuco(res[0],res[1],gray,board)
		if res2[1] is not None and res2[2] is not None and len(res2[1])>3:
			allCorners.append(res2[1])
			allIds.append(res2[2])
	
	cv2.aruco.drawDetectedMarkers(gray,res[0],res[1])

	cv2.imshow('frame',gray)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

imsize = gray.shape

cal = cv2.aruco.calibrateCameraCharuco(allCorners,allIds,board,imsize,None,None)

#Calibration fails for lots of reasons. Release the video if we do
# try:
# 	cal = cv2.aruco.calibrateCameraCharuco(allCorners,allIds,board,imsize,None,None)
# except:
# 	pass
    # cap.release()

# print "Cal 0 ", cal[0]
print("Camera Matrix:  ", cal[1])
print("Distortion coefficients: ", cal[2])
print("rvecs", cal[3])
print("tvecs", cal[4])
# print("Cal 5", cal[5])




# for i in range(3):

#     ret,frame = cap.read()
#     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#     res = cv2.aruco.detectMarkers(gray,dictionary)

#     if len(res[0])>0:
#         res2 = cv2.aruco.interpolateCornersCharuco(res[0],res[1],gray,board)
#         if res2[1] is not None and res2[2] is not None and len(res2[1])>3 and decimator%3==0:
#             allCorners.append(res2[1])
#             allIds.append(res2[2])

#         cv2.aruco.drawDetectedMarkers(gray,res[0],res[1])

#     cv2.imshow('frame',gray)
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break
#     decimator+=1

# imsize = gray.shape

# #Calibration fails for lots of reasons. Release the video if we do
# try:
# 	cal = cv2.aruco.calibrateCameraCharuco(allCorners,allIds,board,imsize,None,None)
# except:
# 	pass
#     # cap.release()

# # print "Cal 0 ", cal[0]
# print "Camera Matrix:  ", cal[1]
# print "Distortion coefficients: ", cal[2]

# cap.release()
cv2.destroyAllWindows()