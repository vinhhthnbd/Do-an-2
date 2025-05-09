import supervision as sv
import ultralytics
from ultralytics import YOLO
import cv2
import numpy as np

model = YOLO('best.pt')

category_dict = {
    0: '247', 1: 'Chinsu', 2: 'Pepsi-xanh', 3: 'Redbull', 4: 'Revive trang',
    5: 'Revive-chanh', 6: 'Tea Plus', 7: 'aquavina', 8: 'coca-chai', 9: 'coca-lon',
    10: 'fanta-cam', 11: 'sprite-chai', 12: 'sprite-lon', 13: 'sting'
}

cap = cv2.VideoCapture(0)
qr = cv2.QRCodeDetector()

object_counts = {name: 0 for name in category_dict.values()}
seen_tracker_ids = set()

# Function that draws the correct placement contour
def QR_contour(decoded_info, points):
    black = np.zeros((int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)), int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))), np.uint8)
    mask_dict = {}
    contour_dict = {}

    for name, coordinate in zip(decoded_info, points):
        name = name.split(';')
        if name[1] == 'R':
            mask = cv2.rectangle(black.copy(), (0,coordinate[2,1].astype(int) - 200), coordinate[2].astype(int), 255, -1)
        elif name[1] == 'L':
            mask = cv2.rectangle(black.copy(), coordinate[3].astype(int), (black.shape[1],coordinate[3,1].astype(int) - 200), 255, -1)
        else:
            mask = cv2.rectangle(black.copy(), (0,coordinate[2,1].astype(int) - 200), (black.shape[1],coordinate[2,1].astype(int)), 255, -1)

        if name[0] not in mask_dict:
            mask_dict[name[0]] = mask
        else:
            mask_dict[name[0]] = np.bitwise_and(mask_dict[name[0]], mask)

    for mask in mask_dict:
        contour_dict[mask], retval = cv2.findContours(mask_dict[mask], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    return contour_dict

# Function that check for the item's placement
def point_in_box(x, y, item_id, contour_dict):
    if cv2.pointPolygonTest(contour_dict[item_id][0], (x,y), False) == -1:
        return False
    else:
        return True

while True:
    ret, frame = cap.read()
    if not ret:
        break

    results = model.track(frame, conf=0.5, tracker="bytetrack.yaml", persist=True, classes=None, verbose=False)[0]
    detections = sv.Detections.from_ultralytics(results)

    # Detect QR codes and draw the 
    retval, decoded_info, points, straight_qrcode = qr.detectAndDecodeMulti(frame)
    if retval == True:
        contour_dict = QR_contour(decoded_info, points)
        for contour in contour_dict:
            cv2.drawContours(frame, contour_dict[contour], -1, (0,255,0), 3)

    if detections.tracker_id is not None:
        for box, class_id, tracker_id in zip(detections.xyxy, detections.class_id, detections.tracker_id):
            class_name = category_dict.get(class_id, "Unknown")

            if tracker_id not in seen_tracker_ids:
                try:
                    if point_in_box(int((box[0] + box[2])/2), int((box[1] + box[3])/2), class_name, contour_dict):
                        object_counts[class_name] += 1
                        seen_tracker_ids.add(tracker_id)
                except:
                    pass
                
            # Annotate the detection on the frame
            cv2.rectangle(frame, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (255, 0, 0), 2)

    cv2.imshow('Frame',frame)

    if cv2.waitKey(20) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

print(object_counts)