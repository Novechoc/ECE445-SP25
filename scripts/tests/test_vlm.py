import time, sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from UR3e import UR3e
from csi_camera import capture_image

ur = UR3e()
ur.home()
capture_image()

from perception import name_objects
print(name_objects())

del(ur)