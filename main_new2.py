import threading
import serial
import queue
import time

def communication(thread_queue):
	global ser, speed
	while True:
		if ser.in_waiting > 0:
			inputbuff = ser.readline()
			inputbuff = inputbuff.decode('UTF-8').rstrip("\n")
			print(printbuff)
		try:
			comm = thread_queue.get_nowait()
			if comm == "gs":
				ser.write((comm + "\n").encode('UTF-8'))
				answer = ser.readline()
				print(answer.decode('UTF-8').rstrip("\n"))
			elif comm.startswith("d:"):
				spdlist = comm.split(":")
				if len(spdlist) == 2:
					ser.write(("d:" + spdlist[1] + "\n").encode('UTF-8'))
			elif comm.startswith("sv:"):
				poslist = comm.split(":")
				if len(poslist) == 2:
					ser.write(("sv:" + poslist[1] + "\n").encode('UTF-8'))
			elif comm.startswith("rf:"):
				rflist = comm.split(":")
				if len(rflist) == 2:
					ser.write((rflist[1] + "\n").encode('UTF-8'))
			elif comm.startswith("sd:"):
				spdlist = comm.split(":")
				if len(spdlist) == 4:
					speed = "sd:" + spdlist[1] + ":" + spdlist[2] + ":" + spdlist[3] + "\n"
					ser.write(speed.encode('UTF-8'))
					print(ser.readline().decode('UTF-8').rstrip("\n"))
			elif comm == "stop":
				speed = "sd:0:0:0\n"
				ser.write(speed.encode('UTF-8'))
				answer = ser.readline()
				print(answer.decode('UTF-8').rstrip("\n"))
				ser.close()
				break
		except queue.Empty:
			pass
		if speed != "sd:0:0:0\n":
			ser.write(speed.encode('UTF-8'))
			ser.readline()

ser = serial.Serial("/dev/ttyACM0", timeout=2, write_timeout=2)
thread_queue = queue.Queue()
speed = "sd:0:0:0\n"

mainboard_thread = threading.Thread(target=communication, args=(thread_queue, ))
mainboard_thread.start()

while True:
	userc = input("Type command:")
	thread_queue.put_nowait(userc)
	if userc == "stop":
		time.sleep(5)
		break
