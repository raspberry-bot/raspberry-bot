from uuid import uuid4

import cv2

if __name__ == '__main__':
    resource = 0
    resource_name = "/dev/video%d" % resource
    cap = cv2.VideoCapture(resource)
    if not cap.isOpened():
        print("Error opening resource: " + str(resource))
        print("Maybe opencv VideoCapture can't open it")
        exit(0)

    print("Correctly opened resource, starting to show feed.")
    rval, frame = cap.read()
    while rval:
        img_name = "%s.png".format(str(uuid4()))
        cv2.imwrite(img_name, frame)

        cv2.imshow("Stream: " + resource_name, frame)
        rval, frame = cap.read()
        key = cv2.waitKey(20)
        # print "key pressed: " + str(key)
        # exit on ESC, you may want to uncomment the print to know which key is ESC for you
        if key == 27 or key == 1048603:
            break
    cv2.destroyWindow("preview")
