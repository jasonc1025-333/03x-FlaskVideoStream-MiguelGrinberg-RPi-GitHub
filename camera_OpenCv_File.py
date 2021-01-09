import os
import cv2
from camera_Base_File import camera_Base_Cl


class camera_Cl(camera_Base_Cl):
    video_source = 0
    
    # jwc: Status Camera Mode
    ##jwc o print("*** DEBUG: camera_opencv.py ***")
    print("*** DEBUG: camera_Cl-01: camera_opencv.py ***")

    def __init__(self):
        if os.environ.get('OPENCV_CAMERA_SOURCE'):
            camera_Cl.set_video_source(int(os.environ['OPENCV_CAMERA_SOURCE']))
        super(camera_Cl, self).__init__()

    @staticmethod
    def set_video_source(source):
        camera_Cl.video_source = source

    @staticmethod
    def frames():
        camera_Cl = cv2.VideoCapture(camera_Cl.video_source)
        if not camera_Cl.isOpened():
            raise RuntimeError('Could not start camera_Cl.')

        while True:
            # read current frame
            _, img = camera_Cl.read()

            # encode as a jpeg image and return it
            ##jwc o yield cv2.imencode('.jpg', img)[1].tobytes()
            # jwc: Flip image vertically
            ## n cause ': cannot connect to X server' crash:  yield cv2.imencode('.jpg', cv2.imshow("Vertical flip", img))[1].tobytes()
            # jwc: just flip camera physically and accomodate with servo controls for now.
            yield cv2.imencode('.jpg', img)[1].tobytes()
