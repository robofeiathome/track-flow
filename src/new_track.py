import cv2
from ultralytics import YOLO

class tracker:
    def __init__(self) -> None:   
        self.model = YOLO('yolov8n.pt')
        self.source = 0
        self.id_to_follow = 1
        self.last_centroid_x = None
        self.risk_matrix = [
            [9, 8, 7],[7, 6, 5],[5, 4, 3],[2, 1, 2],[3, 4, 5],[5, 6, 7],[7, 8, 9]   
        ]

    def calculate_centroid(self, bbox):
        x_min, y_min, x_max, y_max = bbox
        return int((x_min + x_max) / 2), int((y_min + y_max) / 2)

    def calculate_risk(self, centroid, frame_width):
        segment_width = frame_width // 7
        segment = centroid[0] // segment_width
        
        direction = "STATIONARY"
        if self.last_centroid_x:
            if centroid[0] < self.last_centroid_x:
                direction = "LEFT"
            elif centroid[0] > self.last_centroid_x:
                direction = "RIGHT"
        self.last_centroid_x = centroid[0]
        
        direction_map = {"LEFT": 0, "STATIONARY": 1, "RIGHT": 2}
        risk = self.risk_matrix[segment][direction_map[direction]]
        return risk

    def main_track(self):
        cap = cv2.VideoCapture(self.source)
        frame_width = int(cap.get(3))

        while cap.isOpened():
            success, frame = cap.read()
            if success:
                results = self.model.track(frame, persist=True, classes=0, verbose=False, device=0)
                annotated_frame = results[0].plot()

                for r in results:
                    if hasattr(r, 'boxes') and r.boxes and hasattr(r.boxes, 'id') and r.boxes.id is not None:
                        ids = r.boxes.id.tolist()
                        bbox_coords_list = r.boxes.xyxy.tolist()

                        for detected_id, bbox_coords in zip(ids, bbox_coords_list):
                            if detected_id == self.id_to_follow:
                                centroid = self.calculate_centroid(bbox_coords)
                                risk_value = self.calculate_risk(centroid, frame_width)
                                print(f"Risk value for ID {self.id_to_follow}:", risk_value)
                                cv2.circle(annotated_frame, centroid, 5, (0, 255, 0), -1)

                cv2.imshow("YOLOv8 Tracking", annotated_frame)

                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            else:
                break

        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":    
    try:
        tracker().main_track()
    except KeyboardInterrupt:
        print("End of program :)")
