import cv2

def list_webcam_indexes(max_cameras=10):
    indexes = []
    for i in range(max_cameras):
        cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)  # CAP_DSHOW helps on Windows[web:172][web:173][web:181]
        if not cap.isOpened():
            cap.release()
            continue
        ret, frame = cap.read()
        if ret:
            print(f"Camera index {i} OK")
            indexes.append(i)
        cap.release()
    return indexes

print("OpenCV version:", cv2.__version__)
cams = list_webcam_indexes()
print("Cameras found:", cams)
