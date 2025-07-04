import cv2

cap = cv2.VideoCapture(2)

while cap.isOpened():
    ret,frame=cap.read()

    if not ret:
        break

    cv2.imshow("frame",frame)
    cv2.waitKey(20)
