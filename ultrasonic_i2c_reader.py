#MODIFIED!
#  Raspberry Pi Master for Arduino Slave
#  i2c_master_pi.py
#  Connects to Arduino via I2C

#  DroneBot Workshop 2019
#  https://dronebotworkshop.com

from smbus2 import SMBus
import struct

addr = 0x8 # bus address of target device
bus = SMBus(1) # indicates /dev/ic2-1

print ("Enter 1..4 or a to get corresponding sensor")
while True:
	action = input(">>>>   ")
	if action == "a":
		c = 0
		req = bus.read_i2c_block_data(addr, ord('a'), 16) # get all sensors
	elif action == "t":
		temp = input("Enter a value for environmental temperature:")
		data = bytearray(struct.pack("f", float(temp)))
		req = bus.write_i2c_block_data(addr, ord('t'), data) # get first sensor
		continue
	elif action == "1":
		c = 0
		req = bus.read_i2c_block_data(addr, 1, 4) # get first sensor
	elif action == "2":
		c = 1
		req = bus.read_i2c_block_data(addr, 2, 4) # get second sensor
	elif action == "3":
		c = 2
		req = bus.read_i2c_block_data(addr, 3, 4) # get third sensor
	elif action == "4":
		c = 3
		req = bus.read_i2c_block_data(addr, 4, 4) # get fourth sensor
	else:
		break

	#print("Length of received data: %s Type: %s Data: %s" % (len(req), type(req), req))
	r = struct.iter_unpack('<f', bytearray(req))

	for i in r:
		c = c + 1;
		print("Sensor %i: %f" % (c, i[0]))
