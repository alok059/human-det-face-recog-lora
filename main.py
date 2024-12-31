import cv2
import time
from ultralytics import YOLO
import face_recognition
import numpy as np
import serial

# Initialize YOLO model
model = YOLO('yolov8n.pt')
model.to('cpu')

# Known face encodings
known_face_encodings = []
known_face_names = []

def arduino_sending(arduino, data):
    """Send data to Arduino and ensure buffer is flushed."""
    print(f"Sending to Arduino: {data}")
    arduino.write((data + '\n').encode('utf-8'))
    arduino.flush()  # Ensure transmission
    time.sleep(0.1)  # Reduced delay for faster communication

def load_known_faces():
    known_faces = [
        ("Faces/Gurjot1.jpg", "Gurjot"),
        ("Faces/Alok1.jpg", "Alok"),
    ]
    
    for filename, name in known_faces:
        image = face_recognition.load_image_file(filename)
        encodings = face_recognition.face_encodings(image)
        if encodings:
            known_face_encodings.append(encodings[0])
            known_face_names.append(name)
        else:
            print(f"Warning: No face found in {filename}")

load_known_faces()

def detect_persons(frame):
    """Detect persons in the frame using YOLO."""
    results = model(frame, stream=True)
    boxes = []
    for result in results:
        for box in result.boxes:
            if int(box.cls) == 0:  # Class 0 for 'person'
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                boxes.append((x1, y1, x2, y2))
    return boxes

def recognize_face_from_frame(face_image):
    """Recognize face from the detected face image."""
    rgb_image = cv2.cvtColor(face_image, cv2.COLOR_BGR2RGB)
    face_encodings = face_recognition.face_encodings(rgb_image)
    for face_encoding in face_encodings:
        face_distances = face_recognition.face_distance(known_face_encodings, face_encoding)
        best_match_index = np.argmin(face_distances)
        if face_distances[best_match_index] < 0.5:
            return known_face_names[best_match_index]
    return "Unknown"

arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
time.sleep(2)  # Allow Arduino to reset

video_capture = cv2.VideoCapture(0)
video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

FRAME_SKIP = 3
frame_count = 0

try:
    while True:
        ret, frame = video_capture.read()
        if not ret:
            break

        frame_count += 1
        if frame_count % FRAME_SKIP != 0:
            continue  # Skip frames to reduce workload

        detected_persons = detect_persons(frame)
        for bbox in detected_persons:
            x1, y1, x2, y2 = bbox
            person_image = frame[y1:y2, x1:x2]
            recognition_status = recognize_face_from_frame(person_image)

            if recognition_status == "Unknown":
                arduino_sending(arduino, "B")
            else:
                arduino_sending(arduino, "A")
        
        time.sleep(1)  # Reduced to avoid excessive delay
finally:
    video_capture.release()
    arduino.close()
    cv2.destroyAllWindows()
