from ultralytics import YOLO

# Load YOLOv10n model from scratch
model = YOLO("yolov10n.yaml")

# Train the model
model.train(data="dataset_sim.yaml", epochs=100, imgsz=640)
