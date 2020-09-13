import serial

devname = "/dev/ttyUSB0"

ser = serial.Serial(devname)

while True:
    userc = input("Type command:")
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
        ser.write(userc + "\n")
