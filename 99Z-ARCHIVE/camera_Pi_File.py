import io
import time
import picamera
from camera_Base_File import camera_Base_Cl


class Camera_Cl(camera_Base_Cl):

    # jwc: Status Camera_Cl Mode
    ##jwc o print("*** DEBUG: camera_pi.py ***")
    print("*** DEBUG: Camera_Cl-01: camera_pi.py ***")

    @staticmethod
    def frames():
        with picamera.PiCamera() as Camera_Cl:
            # let Camera_Cl warm up
            time.sleep(2)

            stream = io.BytesIO()
            for _ in Camera_Cl.capture_continuous(stream, 'jpeg',
                                                 use_video_port=True):
                # return current frame
                stream.seek(0)
                yield stream.read()

                # reset stream for next frame
                stream.seek(0)
                stream.truncate()
