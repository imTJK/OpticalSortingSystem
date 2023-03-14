from ultralytics import YOLO
from PIL import Image
import cv2
import numpy as np

model = YOLO("/home/tj/Programming/ComputerVision/runs/detect/train/weights/best.pt")  # load a pretrained model (recommended for training)

inputs = [img]
results = model.predict(source=img)
print(results)
name_codes = []
results = results[0].numpy()    #extarct classification codes

tmp = results[:,5]
tmp = str(tmp)
for i in tmp:
    if i != " " and i != "[" and i !="]":
        name_codes.append(i)
print(name_codes[1])

for i in name_codes:
    print(results.names[int(i)])

