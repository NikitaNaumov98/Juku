import serial

def communication(userc):
    if userc == "gs":
        ser.write((userc + "\n").encode('UTF-8'))
        answer = ser.readline()
        print(answer.decode('UTF-8').rstrip("\n"))
    elif userc.startswith("sd:"):
        spdlist = userc.split(":")
        if len(spdlist) == 4:
            ser.write(("sd:" + spdlist[1] + ":" + spdlist[2] + ":" + spdlist[3] + "\n").encode('UTF-8'))
            print(ser.readline().decode('UTF-8').rstrip("\n"))
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

ser = serial.Serial("/dev/ttyACM0", timeout=2, write_timeout=2)

while True:
    userc = input("Type command:")
    if ser.in_waiting > 0:
        inputbuff = ser.readline()
        inputbuff = inputbuff.decode('UTF-8').rstrip("\n")
        print(printbuff)
    if userc == "stop":
        ser.write(("sd:0:0:0\n").encode('UTF-8'))
        answer = ser.readline()
        print(answer.decode('UTF-8').rstrip("\n"))
        ser.close()
        break
    communication(userc)
