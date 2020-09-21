import serial

def communication(userc):
    if userc == "gs":
        ser.write(userc + "\n")
        answer = ser.readline()
        answer = answer.rstrip("\n")
        print(answer)
    elif userc.startswith("sd:"):
        spdlist = userc.split(":")
        if len(spdlist) == 4:
            ser.write("sd:" + spdlist[1] + ":" + spdlist[2] + ":" + spdlist[3] + "\n")
    elif userc.startswith("d:"):
        spdlist = userc.split(":")
        if len(spdlist) == 2:
            ser.write("d:" + spdlist[1] + "\n")
    elif userc.startswith("sv:"):
        poslist = userc.split(":")
        if len(poslist) == 2:
            ser.write("sv:" + poslist[1] + "\n")
    elif userc.startswith("rf:"):
        rflist = userc.split(":")
        if len(rflist) == 2:
            ser.write(rflist[1] + "\n")

#proovida kasutada serial.tools.list_ports
#muuta timeout kui on vaja
ser = serial.Serial("/dev/ttyUSB0", timeout=2, write_timeout=2)

while True:
    userc = input("Type command:")
    if ser.in_writing > 0:
        inputbuff = ser.read()
        inputbuff = inputbuff.rstrip("\n")
        print(printbuff)
    if userc == "stop":
        ser.write("sd:0:0:0\n")
        ser.close()
        break
    communication(userc)
