import os
import cv2
from camera_Base_File import camera_Base_Cl


class Camera_Cl(camera_Base_Cl):
    video_source = 0
    
    # jwc: Status Camera Mode
    ##jwc o print("*** DEBUG: camera_opencv.py ***")
    print("*** DEBUG: Camera_Cl-01: camera_opencv.py ***")

    def __init__(self):
        if os.environ.get('OPENCV_CAMERA_SOURCE'):
            Camera_Cl.set_video_source(int(os.environ['OPENCV_CAMERA_SOURCE']))
        super(Camera_Cl, self).__init__()

    @staticmethod
    def set_video_source(source):
        Camera_Cl.video_source = source

    @staticmethod
    def frames():
        Camera_Cl = cv2.VideoCapture(Camera_Cl.video_source)
        if not Camera_Cl.isOpened():
            raise RuntimeError('Could not start Camera_Cl.')

        while True:
            # read current frame
            _, img = Camera_Cl.read()

            # encode as a jpeg image and return it
            ##jwc o yield cv2.imencode('.jpg', img)[1].tobytes()
            # jwc: Flip image vertically
            ## n cause ': cannot connect to X server' crash:  yield cv2.imencode('.jpg', cv2.imshow("Vertical flip", img))[1].tobytes()
            # jwc: just flip camera physically and accomodate with servo controls for now.
            yield cv2.imencode('.jpg', img)[1].tobytes()
