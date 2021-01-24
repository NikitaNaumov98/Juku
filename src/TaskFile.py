from Movement import *
from Serial import *
from imageprocessing import *
import threading
from controllermovement import *
import sys
sys.path.append('..')

var_dict = {"s1":0,"s2":0,"s3":0,"t":0,"bal":0,"bask":0,"throw":False}

def change_vars(new_list):
    global var_dict
    with lock:
        for x in new_list:
            var_dict[x] = new_list[x]
    return var_dict

def get_vars(the_dict):
    global  var_dict
    with lock:
        for x in the_dict:
            the_dict[x] = var_dict[x]
    return the_dict

if __name__ == "__main__":
    roboserial = SerialClass("/dev/ttyACM0")
    serialt = threading.Thread(target=roboserial, daemon=True)
    imaget = threading.Thread(target=image_processing, daemon=True)


    serialt.start()
    imaget.start()


    while(True):
        print(var_dict)
        var_dict["s1"] = 10






 
 



