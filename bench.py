#!/usr/bin/env python
import sys
from pymavlink import mavutil
from pymavlink.dialects.v10 import ardupilotmega as mavlink1

def main(master):
    # Start a connection
    #master = 'tcp:192.168.1.193:5760'
    print("Starting by connecting to... %s" % master)
#    mavutil.set_dialect(ardupilotmega)
    try:
        the_connection = mavutil.mavlink_connection(master, dialect='ardupilotmega')
    except Exception as e:
        print("Error when connecting: ", e)
        return 1
    # Wait for the first heartbeat 
    #   This sets the system and component ID of remote system for the link
    try:
        the_connection.wait_heartbeat(timeout=3)
        if the_connection.target_system < 1:
            print("Error: No device connected.")
            return 1
    except Exception as e:
        print("Error from heartbeat: ", e)
        return 1

    # Successfull connection indicator
    print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

    # Disarm drone in any case... see https://github.com/ArduPilot/MAVProxy/blob/a708305241005c934bf6843d6bddeeb60a4c3b0d/MAVProxy/modules/mavproxy_swarm.py#L8935
    the_connection.mav.command_long_send(
        the_connection.target_system, the_connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # command
        0,  # confirmation
        0,  # param1 (0 to indicate disarm)
        21196,  # param2 (indicates force disarm)
        0,  # param3
        0,  # param4
        0,  # param5
        0,  # param6
        0)  # param7

    # Request RC stream with rate of 100Hz
    the_connection.mav.request_data_stream_send(the_connection.target_system, the_connection.target_component,
                                            mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 100, 1)

    # Monitor RC data
    c = 0
    msg = the_connection.recv_match(type='RC_CHANNELS_RAW', blocking=True, timeout=3)
    if not msg:
        print("Error: No data received.")
        return 1
    if msg.get_type() == "BAD_DATA":
        if mavutil.all_printable(msg.data):
            sys.stdout.write(msg.data)
            sys.stdout.flush()
        print("Error: BAD_DATA received.")
        return 1

    while c < 10:
        # Get last status of parameter
        msg = the_connection.recv_match(type='C_CHANNELS_RAW', blocking=True)
        last_time = msg.time_boot_ms
        last_val = msg.chan1_scaled
        new_val = 1000 + (last_val + 10) % 1000 # scale 0% to 100% from 1000us to 2000us
        # Set new value for parameter
        the_connection.mav.RC_CHANNELS_OVERRIDE_send(
            the_connection.target_system, the_connection.target_component,
            new_val,  # param1 = chan1_raw
            0,  # param2 = ignore
            0,  # param3
            0,  # param4
            0,  # param5
            0,  # param6
            0)  # param7

        msg = the_connection.recv_match(type='RC_CHANNELS_RAW', condition='RC_CHANNELS_RAW.chan1_raw==new_val', blocking=True)
        delay = msg.time_boot_ms - last_time
        print("Delay to device: %u" % delay)
        c += 1
    return 0

if __name__ == '__main__':
    from optparse import OptionParser
    parser = OptionParser("bench.py [options]")
    parser.add_option("-u", "--udp", dest="udp", action='store_true',
                      help="If set uses UDP instead of TCP protocol.",
                      default=False)
    parser.add_option("--master", dest="master", type="string",
                      metavar="host[:port]", help="Connection to MAVLink server[:port]. Defaults to localhost:5760",
                      default="localhost")
    (opts, args) = parser.parse_args()

       
    if opts.udp == True:
        if opts.master.find(":") == -1:
            opts.master += ":14550"
        opts.master = "udpin:" + opts.master
    else:
        if opts.master.find(":") == -1:
            opts.master += ":5760"
        opts.master = "tcp:" + opts.master
        
    i = main(master=opts.master)
    exit(i)
