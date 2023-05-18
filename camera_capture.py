import cv2
import os

def main():
    # Set the output folder
    output_folder = 'captured_images/test_images'
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    # Open the webcam
    cap = cv2.VideoCapture(0)

    # Set the image counter
    img_counter = 0

    while True:
        # Read the frame from the webcam
        ret, frame = cap.read()

        # Display the frame
        cv2.imshow('Capture Images', frame)

        # Check for keypresses
        key = cv2.waitKey(1) & 0xFF

        # If the spacebar is pressed, save the frame to the output folder
        if key == ord(' '):
            img_name = os.path.join(output_folder, "test_image_{}.png".format(img_counter))
            cv2.imwrite(img_name, frame)
            print("{} written!".format(img_name))
            img_counter += 1

        # If the 'q' key is pressed, break the loop
        elif key == ord('q'):
            break

    # Release the webcam and destroy the windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()