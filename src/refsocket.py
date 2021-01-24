import socket
import json
import threading

command = "stop"
basket = "color"

port = 8110

def handler(ip,port,robot_id):
    global basket
    global command
    connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    connection.connect((ip,port))
    connection.send(json.dump("Juku has joined the game."))
    while(True):
        data = json.loads(connection.recv(1024))
        id = data["targets"].index(robot_id)
        basket = data["baskets"][int(id)]
        command = data["signal"]

#referee_thread = threading.Thread(target=handler(,port,juku),args=(1,),daemon=True)

