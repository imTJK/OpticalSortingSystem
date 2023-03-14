from ultralytics import YOLO
from PIL import Image
import cv2
import numpy as np

def main():
    model = YOLO("yolov8n.pt")  # load a pretrained model (recommended for training)

    img = "test2.jpg"
    inputs = [img]
    results = model(inputs)

    name_codes = []
    codename=[]
    results = results[0].numpy()    #extarct classification codes
    a = results[0]
    a = str(a)
    #print(len(a))
    #tmp = results[:,5]
    #tmp = str(tmp)
    for i in a:
       if i != " " and i != "[" and i !="]":   #keep only the numbers
            name_codes.append(i)
    codename.append(int(name_codes[-2]+name_codes[-1]))
    for i in codename:
        codename = results.names[int(i)]     #read dictionary entries
    print(codename)



if __name__ == '__main__':
    main()
