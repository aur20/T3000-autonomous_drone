import sys
from pymavlink import mavutil

mav = mavutil.mavlink_connection('udpin:' + sys.argv[1])
mav.wait_heartbeat()

while True:
    m = mav.recv_match(type='HEARTBEAT', blocking=True)
    if m is not None:
        print(m)