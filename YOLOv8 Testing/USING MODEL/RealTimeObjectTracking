from ultralytics import YOLO

model = YOLO('best.pt')

results = model.track(source=0, show=True, conf=0.5, save=False, device= 0, stream=True, tracker='bytetrack.yaml')  # generator of Results objects

for r in results:
        boxes = r.boxes  # Boxes object for bbox outputs
        masks = r.masks  # Masks object for segment masks outputs
        probs = r.probs  # Class probabilities for classification outputs

