from ultralytics import YOLO
import cv2
from PIL import Image

# Load the model
model = YOLO('model.pt') 

image_path = 'sample_images/amz_00778.png'
image = cv2.imread(image_path)

result = model.predict(image)[0]
boxes = result.boxes  # Boxes object for bbox outputs


for box in boxes:
  coordinates = box.xyxy[0]

  x1, y1, x2, y2 = int(coordinates[0]), int(coordinates[1]), int(coordinates[2]), int(coordinates[3])
  conf = box.conf.item()
  class_pred = box.cls.item()

  cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)

  label = f"{model.names[class_pred]}: conf: {conf}"
  cv2.putText(image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)

cv2.imshow('Result', image)
cv2.waitKey(0)
cv2.destroyAllWindows()