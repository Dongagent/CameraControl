import cv2


def show_webcam(mirror=False):
    cam = cv2.VideoCapture(2)
    while True:
        ret_val, img = cam.read()
        if mirror: 
            img = cv2.flip(img, 1)
        cv2.imshow('my webcam', img)
        if cv2.waitKey(1) == 27: 
            break  # esc to quit
    cv2.destroyAllWindows()


def main():
    show_webcam(mirror=True)

def main2():
    import cv2
    import threading

    def save_image(frame):
        # Save the current frame as an image
        cv2.imwrite("frame.jpg", frame)

    def read_stream():
        # Open the video stream
        cap = cv2.VideoCapture(2)

        while True:
            # Read a frame from the stream
            ret, frame = cap.read()

            # Break the loop if the stream is over
            if not ret:
                break

    # Start a new thread to read the stream
    stream_thread = threading.Thread(target=read_stream)
    stream_thread.start()

    # Call the function to save the current frame as an image
    save_image()

    # Wait for the stream thread to finish
    stream_thread.join()

if __name__ == '__main__':
    main2()