import threading
import serial

def communication(userc):
	global speedthread, spdlist, comm
	if userc == "gs":
		comm = []
		comm.append(userc)
		speedthread = threading.Thread(target=setspeed, args=(spdlist, comm))
		speedthread.start()
	elif userc.startswith("sd:"):
		newspdlist = userc.split(":")
		if len(newspdlist) == 4:
			spdlist = newspdlist
			speedthread = threading.Thread(target=setspeed, args=(spdlist, comm))
			speedthread.start()
	elif userc.startswith("d:") or userc.startswith("sv:") or userc.startswith("rf:"):
		newcomm = userc.split(":")
		if len(newcomm) == 2:
			comm = newcomm
			speedthread = threading.Thread(target=setspeed, args=(spdlist, comm))
			speedthread.start()

def setspeed(spdlist, comm):
	global speedthread
	oldspeedthread = speedthread
	isspeedset = True
	if len(comm) == 1:
		if comm[0] == "gs":
			ser.readline()
			ser.write(("gs\n").encode('UTF-8'))
			answer = ser.readline()
			print(answer.decode('UTF-8').rstrip("\n"))
	elif len(comm) == 2:
		if comm[0] == "d:" or comm[0] == "sv:" or comm[0] == "rf:":
			ser.readline()
			ser.write((comm[0] + comm[1] + "\n").encode('UTF-8'))
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
spdlist = ["sd", "0", "0", "0"]
comm = []

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
