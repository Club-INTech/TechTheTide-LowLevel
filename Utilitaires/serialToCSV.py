#!/usr/bin/python3
# This script is supposed to do the same as serialToCSV.sh, but using python, so avoiding the blocking bash read
import os
import signal
import sys as suus  # SUUS
import time

print("Warning! this script must be run from the Utilities directory")


def pythonGraph():
	signal.signal(signal.SIGHUP)
	print(filename, outFile, )
	o = open(filename, 'a')
	o.write("DATAEND")
	os.system("python3 extraction_asserv.py " + outFile + " " + suus.argv[1])
	exit(0)


device = "/dev/ttyACM0"

if not os.path.exists("serialOutput"):
	os.mkdir("serialOutput")
if len(suus.argv) == 3:
	filename = "./serialOutput/serialOutput-" + suus.argv[1] + "," + suus.argv[2] + "," + suus.argv[3] + ".csv"
else:
	filename = "./serialOutput/serialOutput.csv"

if not os.path.exists(filename):
	filename = "." + filename.split('/')[-2] + str(time.localtime()) + ".csv"
outFile = filename.split('/')[-1]

if not os.path.isfile(device):
	print("Waiting for serial " + device)
	print("Retrying every 10ms")

	while not os.path.isfile(device):
		time.sleep(0.01)

signal.signal(signal.SIGINT, pythonGraph)
signal.signal(signal.SIGTERM, pythonGraph)
signal.signal(signal.SIGHUP, pythonGraph)

f = open(device, 'w')
f.write('')
os.system("stty -F /dev/ttyACM0 115200 raw -echo -echoe -echok")
csv = open(filename, 'w')

while True:
	line = f.readline()
	print(line)
	csv.write(line)
	if line in "DATAEND":
		break

pythonGraph()
