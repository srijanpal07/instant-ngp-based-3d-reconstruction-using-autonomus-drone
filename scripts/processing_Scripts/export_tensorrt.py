from ultralytics import YOLO

# Load a pretrained YOLOv8 model
model = YOLO('/home/swarm7/Documents/3d_person/yolo/train_yolo2/runs/detect/train/weights/best.pt')

# Export the model
model.export(format='engine', imgsz=640, device=0)
