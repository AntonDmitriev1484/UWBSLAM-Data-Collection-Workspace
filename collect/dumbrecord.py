import sys
import os
import argparse
import threading
import multiprocessing
import serial
import cv2
import numpy as np
import time
import signal
import csv
import json
import re
import matplotlib.pyplot as plt
from datetime import datetime

# === argparse setup ===
parser = argparse.ArgumentParser(description="Stream collector with timed capture")
parser.add_argument("delay", type=int, help="Delay before starting collection (seconds)")
parser.add_argument("duration", type=int, help="Duration of data collection (seconds)")
parser.add_argument("trial_name", required=True, help="Trial name for saving data")
parser.add_argument("--port", "-p", default=0, type=int, help="ACM port Decawave is connected on")
parser.add_argument("--id", "-i", default=1, type=int, help="Decawave network ID")

args = parser.parse_args()

# === Setup directories ===
out_dir = f"../post/out/{args.trial_name}_post/"
os.makedirs(out_dir,exist_ok=True)


def host_timestamp():
    return time.perf_counter_ns() / 1e6  # host time in ms

start_collection = threading.Event()
end_threads_event = threading.Event()

# === Serial setup (Decawave) ===
TAG_PORT = f"COM{args.port}"
try:
    TAG_SERIAL = serial.Serial(TAG_PORT, 115200, timeout=0.5)
except serial.SerialException as e:
    print(f"Failed to open serial port {TAG_PORT}: {e}")
    sys.exit(1)

def read_from_serial(ser):
    ser.reset_input_buffer()
    return ser.readline().decode('utf-8', errors='ignore')

def write_to_serial(ser, s):
    ser.write((s + "\n").encode('utf-8'))

def fetch_decawave_time():
    write_to_serial(TAG_SERIAL, "AT+TIME")

def initialize_decawave():
    write_to_serial(TAG_SERIAL, f"AT+ID {args.id}")
    write_to_serial(TAG_SERIAL, "AT+STARTBLE")
    write_to_serial(TAG_SERIAL, "AT+STARTUWB")


# {
# "t": 1757623684.288217,
# "type": "uwb",
# "id": 2,
# "range": 1.0834720134735107,
# "exchange": 24812,
# "maxnoise": 1533,
# "firstpathamp1": 6384,
# "firstpathamp2": 7257,
# "firstpathamp3": 5133,
# "stdnoise": 64,
# "maxgrowthcir": 1981,
# "rxpreamcount": 121,
# "firstpath": 48107
# }

# line_fields = ['id', 'range', 'exchange', 'maxnoise', 'firstpathamp1', 'firstpathamp2', 'firstpathamp3', 
#                'stdnoise', 'maxgrowthcir','rxpreamcount', 'firstpath', 'realcir', 'complexcir']
line_fields = ['id', 'range', 'exchange', 'maxnoise', 'firstpathamp1', 'firstpathamp2', 'firstpathamp3', 
               'stdnoise', 'maxgrowthcir','rxpreamcount', 'firstpath']
# add realcir and complexcir later as those are harder to parse.

def firmware_output_to_json(line):
    # Converts output json line to my "all.json" uwb range format that is compatible with my plotting code.
    js = {}
    line_values = line.split(',')
    for field, val in zip(line_fields, line_values):
        js[field] = val
    js["t"] = time.time()
    js["type"] = "uwb"
    return js

uwb = []
deca_time = []

def decawave_listener():
    """ Continuously read Decawave serial and log UWB + hardware time """
    range_counter = 0

    initialize_decawave()

    start_collection.wait()

    while not end_threads_event.is_set():
        line = read_from_serial(TAG_SERIAL)
        if not line:
            continue

        if "<log_err>" and ("{" in line and "}" in line):
            print(line.strip())
            uwb.append(firmware_output_to_json(line)) # Do I need to do any conversion from here to my json formats compatible with my plotting code?
            range_counter += 1

    print("Decawave listener stopped.")


# === Cleanup + plotting ===
def on_interrupt(sig=None, frame=None):
    print("\nInterrupt or timeout — stopping threads...")

    end_threads_event.set()
    time.sleep(0.5)

    # Stop serial
    if TAG_SERIAL.is_open:
        TAG_SERIAL.close()
        
    class NumpyEncoder(json.JSONEncoder):
        def default(self, obj):
            if isinstance(obj, np.ndarray):
                return obj.tolist()
            if hasattr(obj, '__dict__'):
                return vars(obj)
            return super().default(obj)
    json.dump(uwb, open(out_dir+"/all.json", 'w'), cls=NumpyEncoder, indent=1)

    sys.exit(0)

signal.signal(signal.SIGINT, on_interrupt)

# === Main logic ===
if __name__ == "__main__":
    # Countdown delay
    for i in range(args.delay, 0, -1):
        print(f"Starting in {i}...")
        time.sleep(1)

    time.sleep(0.1)
    start_collection.set()
    print("Starting Decawave listener...")
    decawave_thread = multiprocessing.Process(target=decawave_listener) # Remember Python threads have stupid global lock.
    decawave_thread.start()

    # Duration timer
    print(f"Collecting data for {args.duration} seconds...")
    try:
        time.sleep(args.duration)
    except KeyboardInterrupt:
        pass

    # Stop everything after duration
    on_interrupt()
