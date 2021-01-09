import time
from camera_Base_File import camera_Base_Cl


class Camera_Cl(camera_Base_Cl):
    """An emulated Camera_Cl implementation that streams a repeated sequence of
    files 1.jpg, 2.jpg and 3.jpg at a rate of one frame per second."""
    imgs = [open(f + '.jpg', 'rb').read() for f in ['1', '2', '3']]

    # jwc: Status Camera_Cl Mode
    print("*** DEBUG: Camera_Cl.py ***")

    @staticmethod
    def frames():
        while True:
            time.sleep(1)
            yield Camera_Cl.imgs[int(time.time()) % 3]
