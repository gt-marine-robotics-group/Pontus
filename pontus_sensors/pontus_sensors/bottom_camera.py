import cv2

cap = cv2.VideoCapture(0)

while cap.isOpened():
	ret, frame = cap.read()
	cv2.imshow("Frame", frame)
	key = cv2.waitKey(1) & 0xFF
	if key == ord('q'):
		break
