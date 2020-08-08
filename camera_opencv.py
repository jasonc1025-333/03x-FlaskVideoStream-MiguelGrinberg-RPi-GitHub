import os
import cv2
from base_camera import BaseCamera


class Camera(BaseCamera):
    video_source = 0
    
    # jwc: Status Camera Mode
    print("*** DEBUG: camera_opencv.py ***")

    def __init__(self):
        if os.environ.get('OPENCV_CAMERA_SOURCE'):
            Camera.set_video_source(int(os.environ['OPENCV_CAMERA_SOURCE']))
        super(Camera, self).__init__()

    @staticmethod
    def set_video_source(source):
        Camera.video_source = source

    @staticmethod
    def frames():
        camera = cv2.VideoCapture(Camera.video_source)
        if not camera.isOpened():
            raise RuntimeError('Could not start camera.')

        while True:
            # read current frame
            _, img = camera.read()

            # encode as a jpeg image and return it
            ##jwc o yield cv2.imencode('.jpg', img)[1].tobytes()
            # jwc: Flip image vertically
            ## n cause ': cannot connect to X server' crash:  yield cv2.imencode('.jpg', cv2.imshow("Vertical flip", img))[1].tobytes()
            # jwc: just flip camera physically and accomodate with servo controls for now.
            yield cv2.imencode('.jpg', img)[1].tobytes()
