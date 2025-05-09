import cv2
import os
import supervision as sv
from ultralytics import YOLO
import ultralytics

# Load the YOLO model
model = YOLO("best.pt")

# Custom class names
category_dict = {
    0: '247', 1: 'Chinsu', 2: 'Pepsi-xanh', 3: 'Redbull', 4: 'Revive trang',
    5: 'Revive-chanh', 6: 'Tea Plus', 7: 'aquavina', 8: 'coca-chai', 9: 'coca-lon',
    10: 'fanta-cam', 11: 'sprite-chai', 12: 'sprite-lon', 13: 'sting'
}

# Define variables to store video source
video_source = "Input_video/video4.mp4"
output_video_path = "output_video_5.avi"  

# Function to process the video
def process_video(video_source, output_video_path):
    cap = cv2.VideoCapture(video_source)

    if not cap.isOpened():
        raise RuntimeError("Error: Could not open video source.")

    # Define VideoWriter to save the processed video
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    fps = cap.get(cv2.CAP_PROP_FPS)
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    out = cv2.VideoWriter(output_video_path, fourcc, fps, (width, height))

    # Initialize variable to count objects
    object_counts = {name: 0 for name in category_dict.values()}
    # Set to track seen tracker_ids across the video
    seen_tracker_ids = set()

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Run the YOLO model on the current frame
        results = model.track(frame, conf=0.5,tracker="Input_video/bytetrack.yaml", persist=True, classes=None, verbose=False)[0]

        # Convert results to Detections
        detections = sv.Detections.from_ultralytics(results)

        # Vẽ bounding boxes và nhãn
        if detections.tracker_id is not None:
            for box, class_id, confidence, tracker_id in zip(
                    detections.xyxy, detections.class_id, detections.confidence, detections.tracker_id):
                if tracker_id is None:  # Bỏ qua nếu tracker_id không tồn tại
                    continue

                class_name = category_dict.get(class_id, "Unknown")
                
                # Check if tracker_id has not been counted yet
                if tracker_id not in seen_tracker_ids:
                    object_counts[class_name] += 1
                    seen_tracker_ids.add(tracker_id)  # Đánh dấu tracker_id đã gặp
                
                # Annotate the detection on the frame
                cv2.rectangle(frame, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (255, 0, 0), 2)
                cv2.putText(frame, f"Tracker ID: {tracker_id}, {class_name}: {confidence:.2f}", 
                            (int(box[0]), int(box[1] - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 
                            (255, 0, 0), 1)  # Increased text size

        # Display object count information
        y_offset = 40
        for class_name, count in object_counts.items():
            if count > 0:
                cv2.putText(frame, f"{class_name}: {count}", (20, y_offset),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)  # Increased text size
                y_offset += 30

        # Write the frame
        out.write(frame)

    cap.release()
    out.release()

# Process the video
process_video(video_source, output_video_path)

# Move output file to a specific directory
import shutil
shutil.move('output_video_5.avi', 'D:/Do an/Track&Counting/Output_Video/output_video_5.avi')
