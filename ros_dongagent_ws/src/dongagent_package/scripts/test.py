import cv2
import threading

class WebCamera:
    def __init__(self, device_id):
        self.cap = cv2.VideoCapture(device_id)
        self.thread = None
        self.stopped = False

    def start(self):
        if self.thread is None:
            self.thread = threading.Thread(target=self.update, args=())
            self.thread.start()
            return True
        return False

    def update(self):
        while True:
            if self.stopped:
                return
            ret, frame = self.cap.read()
            if not ret:
                continue
            # Perform any desired operations on the frame here

    def read(self):
        ret, frame = self.cap.read()
        if ret:
            return frame
        return None

    def stop(self):
        self.stopped = True
        self.thread.join()
        self.cap.release()

def save_image(frame, path):
    cv2.imwrite(path, frame)

def main():
    camera = WebCamera()
    camera.start()
    frame = camera.read()
    if frame is not None:
        save_image(frame, 'image.png')
    else:
        print('error')
    camera.stop()
    camera.join()
    

if __name__ == "__main__":
    main() 
    main()