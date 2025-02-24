'''
script to convert yolo model (.pt) into tensorrt format (.engine)

likely has some bugs, as this was just migrated from the node
'''
from ultralytics import YOLO

path = "" # path to yolo model
model_name = "" # name of model file. ie. "yolov8.pt"
yolo = YOLO(path) 

yolo.export(format="engine", dynamic=True)
yolo.export(
    format="engine",
    dynamic=True,
    batch=8,
    workspace=4,
    int8=True,
    data="args.yaml",
)
dot_loc = model_name.index('.')
to_new_model_name = model_name[:dot_loc]
yolo = YOLO(to_new_model_name+'.engine', task='detect')
yolo.fuse()
