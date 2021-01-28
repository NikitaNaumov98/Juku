from multiprocessing import Process, Value, Manager
import imageprocessing

def vision_worker(run, pros_val, speeds,lock):
    while 1:
        if(run.value):
            vision_process(pros_val,speeds)

def vision_process(pros_val,speeds):

    imageprocessing.uus_kaamerapilt(pros_val,speeds)

def processing_worker(run,pros_val,speeds):
    while 1:
        if(run.value):
            processing_process(pros_val,speeds)

def processing_process(pros_val,speeds):

    imageprocessing.protsessipilti(pros_val,speeds)

