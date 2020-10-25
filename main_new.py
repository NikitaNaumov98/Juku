import threading
import serial

def communication(userc):
	global speedthread
	if userc == "gs":
		ser.write((userc + "\n").encode('UTF-8'))
		answer = ser.readline()
		print(answer.decode('UTF-8').rstrip("\n"))
	elif userc.startswith("sd:"):
		spdlist = userc.split(":")
		if len(spdlist) == 4:
			speedthread = threading.Thread(target=setspeed, args=(spdlist,))
			speedthread.start()
	elif userc.startswith("d:"):
		spdlist = userc.split(":")
		if len(spdlist) == 2:
			ser.write(("d:" + spdlist[1] + "\n").encode('UTF-8'))
	elif userc.startswith("sv:"):
		poslist = userc.split(":")
		if len(poslist) == 2:
			ser.write(("sv:" + poslist[1] + "\n").encode('UTF-8'))
	elif userc.startswith("rf:"):
		rflist = userc.split(":")
		if len(rflist) == 2:
			ser.write((rflist[1] + "\n").encode('UTF-8'))

def setspeed(spdlist):
	global speedthread
	oldspeedthread = speedthread
	isspeedset = True
	while True:
		ser.write(("sd:" + spdlist[1] + ":" + spdlist[2] + ":" + spdlist[3] + "\n").encode('UTF-8'))
		fromserial = ser.readline()
		if isspeedset == True:
			print(fromserial.decode('UTF-8').rstrip("\n"))
			isspeedset = False
		if speedthread != oldspeedthread:
			break

ser = serial.Serial("/dev/ttyACM0", timeout=2, write_timeout=2)
speedthread = None
	
while True:
	userc = input("Type command:")
	if ser.in_waiting > 0:
		inputbuff = ser.readline()
		inputbuff = inputbuff.decode('UTF-8').rstrip("\n")
		print(printbuff)
	if userc == "stop":
		if speedthread != None:
			speedthread = None
			ser.readline()
		ser.write(("sd:0:0:0\n").encode('UTF-8'))
		answer = ser.readline()
		print(answer.decode('UTF-8').rstrip("\n"))
		ser.close()
		break
	communication(userc)
