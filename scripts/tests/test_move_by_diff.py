from UR3e import UR3e

ur = UR3e()

import time

while True:
    ur.move_by_diff([0, -0.01, 0])

# ur.move_to([0.2, -0.3, 0.15])