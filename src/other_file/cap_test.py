import cv2

def test_video_capture():
    # Open the default camera (0). Change to a different index if needed.
    # Open the video device (/dev/video2)
    cap = cv2.VideoCapture(0)

    # Set the resolution to 1920x1080
    # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    # Set the FPS to 31
    # cap.set(cv2.CAP_PROP_FPS, 31)

    # Set the format to MJPG
    # cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

    # Get FPS, width, and height
    fps = cap.get(cv2.CAP_PROP_FPS)
    width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

    print(f"FPS: {fps}")
    print(f"Width: {int(width)}")
    print(f"Height: {int(height)}")

    if not cap.isOpened():
        print("Error: Could not open video capture.")
        return

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()

        if not ret:
            print("Error: Could not read frame.")
            break

        # Display the resulting frame
        # cv2.imshow('Video Capture Test', frame)

        # Break the loop if 'q' is pressed
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     break

    # Release the capture and close any OpenCV windows
    cap.release()
    cv2.destroyAllWindows()

# Run the video capture test
if __name__ == "__main__":
    test_video_capture()
