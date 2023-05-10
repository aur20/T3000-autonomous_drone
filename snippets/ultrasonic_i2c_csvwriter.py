#MODIFIED!
#  Raspberry Pi Master for Arduino Slave
#  i2c_master_pi.py
#  Connects to Arduino via I2C

#  DroneBot Workshop 2019
#  https://dronebotworkshop.com

from smbus2 import SMBus
import struct
import csv
import time

addr = 0x8 # bus address
bus = SMBus(1) # indicates /dev/ic2-1

start_time = time.time()
f = open('sensor_log.csv', 'w')
csvwriter = csv.writer(f)
csvwriter.writerow(["# Time", "Sensor A", "Sensor B", "Sensor C", "Sensor D"]) 

print ("Monitoring 4 channel sensor data")
while True:
    try:
        req = bus.read_i2c_block_data(addr, ord('a'), 16) # get all sensors
        #print("Length of received data: %s Type: %s Data: %s" % (len(req), type(req), req))
        t = time.time() - start_time
        r = list(map(lambda a: a[0], struct.iter_unpack('<f', bytearray(req)))) # cast tuple to useful data
        print([t] + r)
        csvwriter.writerow([t] + r)
        time.sleep(0.25)


    except (KeyboardInterrupt, SystemExit):
        f.close()
        print("\nKeyboardinterrupt caught. Finished.")
        raise
