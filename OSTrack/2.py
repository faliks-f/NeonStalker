import cv2
# read local file .avi
cap = cv2.VideoCapture('depth_output1.avi')
while cap.isOpened():
    ret, frame = cap.read()
    print(ret)
    if not ret:
        break
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break