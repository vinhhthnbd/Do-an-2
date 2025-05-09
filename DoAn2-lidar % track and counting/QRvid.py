import cv2
import numpy as np

img = cv2.imread('shelf3.jpg')

qr = cv2.QRCodeDetector()

retval, decoded_info, points, straight_qrcode = qr.detectAndDecodeMulti(img)

def QR_contour(decoded_info, points):
    black = np.zeros(img.shape[:2], np.uint8)
    mask_dict = {}
    contour_dict = {}

    for i in range(len(decoded_info)):
        if decoded_info[i][2] == 'R':
            mask = cv2.rectangle(black.copy(), (0,points[i,2,1].astype(int) - 200), points[i,2].astype(int), 255, -1)
        else:
            mask = cv2.rectangle(black.copy(), points[i,3].astype(int), (black.shape[1],points[i,3,1].astype(int) - 200), 255, -1)

        if decoded_info[i][0] not in mask_dict:
            mask_dict[decoded_info[i][0]] = mask
        else:
            mask_dict[decoded_info[i][0]] = np.bitwise_and(mask_dict[decoded_info[i][0]], mask)

    for mask in mask_dict:
        contour_dict[mask], retval = cv2.findContours(mask_dict[mask], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    return contour_dict

def point_in_box(x, y, item_id, contour_dict):
    if cv2.pointPolygonTest(contour_dict[item_id][0], (x,y), False) == -1:
        return 0
    else:
        return 1

contour_dict = QR_contour(decoded_info, points)

if point_in_box(900,1020,"B",contour_dict) == 0:
    cv2.putText(img, "Out", (800,1000), cv2.FONT_HERSHEY_SIMPLEX, 4, (255,0,0), 3, cv2.LINE_AA)
else:
    cv2.putText(img, "In", (800,1000), cv2.FONT_HERSHEY_SIMPLEX, 4, (255,0,0), 3, cv2.LINE_AA)

for contour in contour_dict:
    cv2.drawContours(img, contour_dict[contour], -1, (0,255,0), 3)

cv2.imshow('Output', cv2.resize(img, (600,600)))
cv2.waitKey(0)
cv2.destroyAllWindows()