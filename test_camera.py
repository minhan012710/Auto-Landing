import cv2

from smart_landing.rosetta_camera import RosettaCamera
from aruco_detection import *
cam = RosettaCamera()
cam.connect()

windowName = "Video Test"
cv2.namedWindow(windowName, cv2.WINDOW_NORMAL)

arm_and_takeoff(5)


while True:
	image,ts,yaw = cam.getImage()
	# print("ts: ", ts, ", yaw:", yaw)
	if image is not None:
		x, y, z = get_vector_movement(image)
		vector_pad = Vector(x, y)
		yaw = get_yaw(vector_pad)
		if yaw > THRESHOLD_YAW:
			xoay(yaw_angle=yaw)
		elif x > THRESHOLD_X:
			bay_ngang(x)
		else:
			di_xuong(z)
	key = cv2.waitKey(1)
	if key == 27:
		break
	

	if cv2.getWindowProperty(windowName, cv2.WND_PROP_VISIBLE) < 1:
		break

vehicle.mode = VehicleMode("LAND")


time.sleep(1)
while vehicle.armed:
	print("Waiting for drone to land...")
	time.sleep(1)
print("Drone has landed.")