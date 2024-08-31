import cv2


"""
xhost +local:root
"""


def main():
    capture = cv2.VideoCapture(0)

    while True:
        has_frame, frame = capture.read()
        if not has_frame:
            print("No frame")
            break

        cv2.imshow("Camera", frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break


if __name__ == "__main__":
    main()
