from ultralytics import YOLO

model = YOLO("best.pt")

metrics = model.val(data="/home/suhas99/Suhas/seg/dataset/data.yaml", imgsz=640, batch=6)