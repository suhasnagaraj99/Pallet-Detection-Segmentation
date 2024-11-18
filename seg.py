from ultralytics import YOLO

model = YOLO("yolo11s-seg.pt")

results = model.train(data="/home/suhas99/Suhas/seg/dataset/data.yaml", epochs=100, imgsz=640, batch=6)