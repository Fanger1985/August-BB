import cv2
import numpy as np
import asyncio
import aiohttp

# URLs for ultrasonic sensor data
ULTRASONIC_FRONT_URL = 'http://<ESP32_WROOM_IP>/ultrasonic/front'
ULTRASONIC_LEFT_URL = 'http://<ESP32_WROOM_IP>/ultrasonic/left'
ULTRASONIC_RIGHT_URL = 'http://<ESP32_WROOM_IP>/ultrasonic/right'

# URLs for motor control
MOTOR_FORWARD_URL = 'http://<ESP32_WROOM_IP>/motor/forward'
MOTOR_BACKWARD_URL = 'http://<ESP32_WROOM_IP>/motor/backward'
MOTOR_LEFT_URL = 'http://<ESP32_WROOM_IP>/motor/left'
MOTOR_RIGHT_URL = 'http://<ESP32_WROOM_IP>/motor/right'
MOTOR_STOP_URL = 'http://<ESP32_WROOM_IP>/motor/stop'

# URL for gyro sensor data
GYRO_SENSOR_URL = 'http://<ESP32_WROOM_IP>/gyro'

async def fetch_data(session, url):
    try:
        async with session.get(url) as response:
            return await response.json()
    except Exception as e:
        print(f"Error fetching data from {url}: {e}")
        return {'distance': 100}  # Default to 100 in case of an error

async def send_command(session, url):
    try:
        async with session.get(url) as response:
            return await response.text()
    except Exception as e:
        print(f"Error sending command to {url}: {e}")
        return "Error"

def apply_filter(frame, filter_type):
    if filter_type == 'edges':
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        return edges
    else:
        return frame

def find_clear_path(edges, baseline_edges, dynamic_threshold):
    diff = cv2.absdiff(edges, baseline_edges)
    height, width = edges.shape
    regions = 3
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
    dynamic_threshold = (total_edges // regions) * 1.5
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
    
    return frame, direction

async def main():
    cap_center = cv2.VideoCapture(0)  # Primary camera
    cap_left = cv2.VideoCapture('http://<LEFT_ESP32_CAM_IP>/stream')    # Left camera
    cap_right = cv2.VideoCapture('http://<RIGHT_ESP32_CAM_IP>/stream')   # Right camera
    
    filter_type = 'edges'
    
    # Check camera availability
    if not cap_center.isOpened():
        print("Error: Primary camera not available.")
        return
    
    if not cap_left.isOpened():
        print("Warning: Left camera not available. Using primary camera only.")
        cap_left = None
    
    if not cap_right.isOpened():
        print("Warning: Right camera not available. Using primary camera only.")
        cap_right = None
    
    # Calibration step
    print("Calibrating... Ensure the path is clear for at least 2 feet in front of the camera.")
    baseline_edges, dynamic_threshold = calibrate_camera(cap_center, filter_type)
    if baseline_edges is None:
        return
    
    async with aiohttp.ClientSession() as session:
        while True:
            ret_center, frame_center = cap_center.read()
            ret_left, frame_left = cap_left.read() if cap_left else (False, None)
            ret_right, frame_right = cap_right.read() if cap_right else (False, None)
            
            if not ret_center:
                break

            edges_center = apply_filter(frame_center, filter_type)
            nav_frame_center, direction = navigate_based_on_edges_and_obstacles(edges_center, frame_center, baseline_edges, dynamic_threshold)

            if ret_left:
                edges_left = apply_filter(frame_left, filter_type)
                nav_frame_left, _ = navigate_based_on_edges_and_obstacles(edges_left, frame_left, baseline_edges, dynamic_threshold)
            else:
                nav_frame_left = frame_center

            if ret_right:
                edges_right = apply_filter(frame_right, filter_type)
                nav_frame_right, _ = navigate_based_on_edges_and_obstacles(edges_right, frame_right, baseline_edges, dynamic_threshold)
            else:
                nav_frame_right = frame_center

            combined_frame = np.hstack((nav_frame_left, nav_frame_center, nav_frame_right)) if ret_left and ret_right else nav_frame_center
            
            ultrasonic_front = await fetch_data(session, ULTRASONIC_FRONT_URL)
            ultrasonic_left = await fetch_data(session, ULTRASONIC_LEFT_URL)
            ultrasonic_right = await fetch_data(session, ULTRASONIC_RIGHT_URL)
            gyro_data = await fetch_data(session, GYRO_SENSOR_URL)
            
            height, width = combined_frame.shape[:2]
            
            if ultrasonic_front['distance'] < 30:
                message = "Obstacle Ahead! Ultrasonic Distance: Close"
                cv2.putText(combined_frame, message, (10, height - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                await send_command(session, MOTOR_STOP_URL)
            elif direction == "Blocked":
                await send_command(session, MOTOR_STOP_URL)
            elif direction == "Left" and ultrasonic_left['distance'] >= 30:
                await send_command(session, MOTOR_LEFT_URL)
            elif direction == "Right" and ultrasonic_right['distance'] >= 30:
                await send_command(session, MOTOR_RIGHT_URL)
            else:
                await send_command(session, MOTOR_FORWARD_URL)
            
            # Display gyro sensor data
            gyro_message = f"Gyro: {gyro_data}"
            cv2.putText(combined_frame, gyro_message, (10, height - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            
            cv2.imshow('Webcam - Navigation', combined_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                await send_command(session, MOTOR_STOP_URL)
                break

        cap_center.release()
        if cap_left:
            cap_left.release()
        if cap_right:
            cap_right.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    asyncio.run(main())
