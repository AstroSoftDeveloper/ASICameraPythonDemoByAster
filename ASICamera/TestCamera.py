import random
import cv2
import ASICamera as asi

def main():
    if asi.get_num_of_connected_cameras() == 0:
        return

    cam_id = 0

    asi.open_camera(cam_id)
    asi.init_camera(cam_id)
    asi.set_roi_format(cam_id, 1280, 960, 1, asi.IMG_TYPE.IMG_RGB24)
    asi.start_video_capture(cam_id)

    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
    font = cv2.FONT_HERSHEY_DUPLEX
    fontScale = 0.5

    while (True):
        frame = asi.get_video_data(cam_id, 100)
        frame = cv2.resize(frame, (640, 480))

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5)

        for (x, y, w, h) in faces:
            cv2.rectangle(frame, (x, y), (x+w, y+h), (128, 255, 0), 1)
            
        cv2.imshow("Video", frame)
        if cv2.waitKey(40) & 0xFF == ord('q'):
            break

    asi.stop_video_capture(cam_id)
    asi.close_camera(cam_id)

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()