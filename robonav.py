import cv2
import numpy as np

def apply_filter(frame, filter_type):
    if filter_type == 'edges':
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)  # Adjusted parameters for better detection
        return edges
    else:
        return frame

def find_clear_path(edges, baseline_edges, dynamic_threshold):
    # Compare current edges with baseline to find clear path
    diff = cv2.absdiff(edges, baseline_edges)
    height, width = edges.shape
    regions = 3  # Dividing into left, center, right
    region_width = width // regions
    
    clear_path = []
    for i in range(regions):
        region = diff[:, i * region_width:(i + 1) * region_width]
        edge_count = cv2.countNonZero(region)
        if edge_count < dynamic_threshold:
            clear_path.append(i)
    
    return clear_path

def calculate_dynamic_threshold(baseline_edges):
    height, width = baseline_edges.shape
    regions = 3
    region_width = width // regions
    total_edges = 0
    for i in range(regions):
        region = baseline_edges[:, i * region_width:(i + 1) * region_width]
        total_edges += cv2.countNonZero(region)
    dynamic_threshold = (total_edges // regions) * 1.5  # Adjust multiplier as needed
    return dynamic_threshold

def calibrate_camera(cap, filter_type, num_frames=5):
    baseline_frames = []
    for _ in range(num_frames):
        ret, frame = cap.read()
        if not ret:
            continue
        edges = apply_filter(frame, filter_type)
        baseline_frames.append(edges)
        cv2.imshow('Calibration', frame)
        cv2.waitKey(100)
    
    cv2.destroyWindow('Calibration')
    
    if len(baseline_frames) == 0:
        print("Error: Couldn't capture enough baseline frames.")
        return None, None
    
    baseline_edges = np.median(baseline_frames, axis=0).astype(np.uint8)
    dynamic_threshold = calculate_dynamic_threshold(baseline_edges)
    return baseline_edges, dynamic_threshold

def estimate_distance(edges):
    # Simple distance estimation based on the position and size of the edges
    height, width = edges.shape
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    distance = "Unknown"
    
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        if y + h > height // 2:
            distance = "Close"
            break
        elif y < height // 2:
            distance = "Far"
    
    if len(contours) == 0:
        distance = "Clear"
    
    return distance

def navigate_based_on_edges_and_obstacles(edges, frame, baseline_edges, dynamic_threshold):
    clear_path = find_clear_path(edges, baseline_edges, dynamic_threshold)
    distance_estimation = estimate_distance(edges)
    
    height, width = edges.shape
    regions = 3
    region_width = width // regions
    
    for i in range(regions):
        color = (0, 255, 0) if i in clear_path else (0, 0, 255)
        cv2.rectangle(frame, (i * region_width, 0), ((i + 1) * region_width, height), color, 2)
    
    direction = "Blocked"
    if clear_path:
        if 0 in clear_path:
            direction = "Left"
        if 1 in clear_path:
            direction = "Center"
        if 2 in clear_path:
            direction = "Right"
    
    message = "Path Clear" if clear_path else "Path Blocked"
    message += f" | Distance: {distance_estimation} | Direction: {direction}"
    
    cv2.putText(frame, message, (10, height - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    
    return frame

def main():
    cap = cv2.VideoCapture(0)  # Primary camera
    filter_type = 'edges'
    
    # Calibration step
    print("Calibrating... Ensure the path is clear for at least 2 feet in front of the camera.")
    baseline_edges, dynamic_threshold = calibrate_camera(cap, filter_type)
    if baseline_edges is None:
        return
    
    cap_left = cv2.VideoCapture(1)  # Left camera
    cap_right = cv2.VideoCapture(2)  # Right camera

    while True:
        ret, frame_center = cap.read()
        ret_left, frame_left = cap_left.read()
        ret_right, frame_right = cap_right.read()
        
        if not ret or not ret_left or not ret_right:
            break

        edges_center = apply_filter(frame_center, filter_type)
        edges_left = apply_filter(frame_left, filter_type)
        edges_right = apply_filter(frame_right, filter_type)
        
        nav_frame_center = navigate_based_on_edges_and_obstacles(edges_center, frame_center, baseline_edges, dynamic_threshold)
        nav_frame_left = navigate_based_on_edges_and_obstacles(edges_left, frame_left, baseline_edges, dynamic_threshold)
        nav_frame_right = navigate_based_on_edges_and_obstacles(edges_right, frame_right, baseline_edges, dynamic_threshold)

        # Combine frames for display
        combined_frame = np.hstack((nav_frame_left, nav_frame_center, nav_frame_right))
        
        # Display the combined navigation frame
        cv2.imshow('Webcam - Navigation', combined_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cap_left.release()
    cap_right.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
