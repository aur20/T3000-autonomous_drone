# Comparison of MAVLINK messages sent over TCP/UDP

Status: untested and not working

## What is to be achieved
- this device simulates being a QCS connecting to a remote system via **TCP or UDP**
- this QCS requests data to be periodically streamed
- this QCS changes a parameter in the data, prints out maximal delay for setting the parameter
- we decide which protocol is best suited for sending MAVLINK-messages

## Requirements
Python 3.x and pip are required to run this demo. Open a CLI in the current directory and perform: 
```
pip3 install -r requirements.txt
```
