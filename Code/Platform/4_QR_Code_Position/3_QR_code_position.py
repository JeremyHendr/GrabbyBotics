import cv2 as cv
import numpy as np
from pyzbar.pyzbar import decode
from pyzbar.pyzbar import ZBarSymbol


#NOTE: Change these parameters in accordance to your hardware
#----------------------------------------------------
camera_ID = 1 # usually 0 is the builtin camera
camera_width = 2560 # Width for stereo camera (will be split in half)
camera_height = 720
#----------------------------------------------------
resize_scale = 0.6 #0.4 for 1440*3840

# Load stereo calibration parameters
print("Loading stereo calibration parameters...")
calibration_data = None
matrix_left = None
matrix_right = None
baseline = None

try:
    calibration_data = np.load("../Calibration/Saved_parameters/retification_informations.npz", allow_pickle=True)
    leftMapX = calibration_data['leftMapX']
    leftMapY = calibration_data['leftMapY']
    rightMapX = calibration_data['rightMapX']
    rightMapY = calibration_data['rightMapY']
    leftROI = calibration_data['leftROI']
    rightROI = calibration_data['rightROI']
    imageSize = tuple(calibration_data['imageSize'])
    
    # Try to load camera matrices and other parameters
    if 'matrix_left' in calibration_data:
        matrix_left = calibration_data['matrix_left']
        matrix_right = calibration_data['matrix_right']
        print(f"✓ Camera matrices found")
    else:
        print(f"⚠ Camera matrices NOT found in calibration file")
        print(f"  Creating approximate camera matrix from image size...")
        # Create approximate camera matrix if not available
        # Assumes 50 degree field of view and principal point at center
        f = imageSize[0] / (2 * np.tan(np.radians(25)))  # focal length calculation
        matrix_left = np.array([
            [f, 0, imageSize[0]/2],
            [0, f, imageSize[1]/2],
            [0, 0, 1]
        ], dtype=np.float32)
        matrix_right = matrix_left.copy()
        print(f"  Using approximate focal length: {f:.1f} pixels")
    
    if 'translation_vector' in calibration_data:
        translation_vector = calibration_data['translation_vector']
        # Baseline is the magnitude of the translation vector's x-component (in real world units)
        # Assuming it's in mm from calibration
        baseline = abs(float(translation_vector[0][0]))  # in mm
        print(f"✓ Baseline extracted: {baseline:.2f} mm")
    else:
        print(f"⚠ Translation vector NOT found in calibration file")
        print(f"  Using approximate baseline: 60 mm")
        baseline = 60  # mm - default stereo baseline
    
    if 'leftProjection' in calibration_data:
        leftProjection = calibration_data['leftProjection']
        rightProjection = calibration_data['rightProjection']
        print(f"✓ Projection matrices found")
    
    print("Calibration parameters loaded successfully")
except Exception as e:
    print(f"Warning: Could not load all calibration parameters: {e}")
    print("Will use approximate values for 3D positioning") 
    leftMapX = leftMapY = rightMapX = rightMapY = None

print("\nConnecting to Camera...")
cam = cv.VideoCapture(camera_ID)
# cam.set(cv.CAP_PROP_FPS, 60)
cam.set(cv.CAP_PROP_FRAME_WIDTH, camera_width)
cam.set(cv.CAP_PROP_FRAME_HEIGHT, camera_height)

ret, frame = cam.read()
assert ret, "No frame readed"

last_picture_taken = frame
last_picture_taken = cv.resize(last_picture_taken, None, fx=resize_scale, fy=resize_scale)

height = frame.shape[0]
width = frame.shape[1] 
print("Picture width: "+str(width)+"     height: "+str(height))
print("Connected to camera")
cv.namedWindow("window", cv.WINDOW_AUTOSIZE)


def get_qr_center(qr_code):
    """Extract the center coordinates of a QR code bounding box"""
    rect = qr_code.rect
    center_x = rect.left + rect.width // 2
    center_y = rect.top + rect.height // 2
    return (center_x, center_y)


def find_matching_qr_code(qr_left, qr_codes_right, y_tolerance=10):
    """
    Find matching QR code in right frame based on proximity in y-coordinate
    (epipolar constraint: matched points should have similar y-coordinates in rectified images)
    
    Args:
        qr_left: QR code from left frame
        qr_codes_right: List of QR codes from right frame
        y_tolerance: Maximum y-coordinate difference for a match (pixels)
    
    Returns:
        Matching QR code or None
    """
    if not qr_codes_right:
        return None
    
    center_left = get_qr_center(qr_left)
    
    for qr_right in qr_codes_right:
        center_right = get_qr_center(qr_right)
        
        # Check if y-coordinates are similar (epipolar constraint)
        if abs(center_left[1] - center_right[1]) < y_tolerance:
            # Also check if it's the same QR code data
            if qr_left.data == qr_right.data:
                return qr_right
    
    return None


def extract_focal_length(camera_matrix):
    """Extract focal length from camera matrix"""
    if camera_matrix is not None:
        # Focal length is typically the same for both x and y (in a well-calibrated camera)
        f = (camera_matrix[0, 0] + camera_matrix[1, 1]) / 2
        return f
    return None


def extract_principal_point(camera_matrix):
    """Extract principal point from camera matrix"""
    if camera_matrix is not None:
        cx = camera_matrix[0, 2]
        cy = camera_matrix[1, 2]
        return cx, cy
    return None


def triangulate_qr_position(qr_left, qr_right, matrix_left, matrix_right, baseline):
    """
    Calculate 3D position of QR code using stereo triangulation
    
    Origin is at the LEFT camera's optical center
    X-axis: pointing right
    Y-axis: pointing down  
    Z-axis: pointing forward (into the scene)
    
    Args:
        qr_left: QR code from left frame
        qr_right: QR code from right frame
        matrix_left: Camera intrinsic matrix for left camera
        matrix_right: Camera intrinsic matrix for right camera
        baseline: Baseline distance between cameras in mm
    
    Returns:
        (x, y, z) coordinates in 3D space relative to left camera origin
    """
    # Get centers
    pt_left = np.array(get_qr_center(qr_left), dtype=np.float32)
    pt_right = np.array(get_qr_center(qr_right), dtype=np.float32)
    
    # Extract focal length and principal point
    f = extract_focal_length(matrix_left)
    cx, cy = extract_principal_point(matrix_left)
    
    if f is None or baseline is None:
        print("  ⚠ Missing calibration data for 3D calculation")
        return None
    
    # Disparity = x_left - x_right
    disparity = pt_left[0] - pt_right[0]
    
    print(f"  [DEBUG] Focal length: {f:.1f} pixels")
    print(f"  [DEBUG] Principal point: ({cx:.1f}, {cy:.1f})")
    print(f"  [DEBUG] Baseline: {baseline:.2f} mm")
    print(f"  [DEBUG] Left point: {pt_left}, Right point: {pt_right}")
    print(f"  [DEBUG] Disparity: {disparity:.2f} pixels")
    
    if disparity <= 0:
        print(f"  ⚠ Invalid disparity: {disparity:.2f} (must be > 0)")
        return None  # No valid disparity
    
    # Calculate depth: Z = (f * baseline) / disparity
    Z = (f * baseline) / disparity
    
    # Calculate X and Y using the image coordinates and camera parameters
    # X = (x_left - cx) * Z / f
    # Y = (y_left - cy) * Z / f
    x_center = pt_left[0]
    y_center = pt_left[1]
    
    X = (x_center - cx) * Z / f
    Y = (y_center - cy) * Z / f
    
    print(f"  [DEBUG] Calculated Z: {Z:.2f} mm")
    
    return (X, Y, Z)


def draw_qr_code(frame, qr_code, color=(0, 255, 0)):
    """Draw QR code bounding box on frame"""
    rect = qr_code.rect
    cv.rectangle(frame, (rect.left, rect.top), 
                (rect.left + rect.width, rect.top + rect.height), color, 2)
    return frame


def create_display_layout(left_frame, right_frame, qr_data=None, position_3d=None, distance=None):
    """
    Create display layout with left/right camera views and position overlay
    
    Args:
        left_frame: Left camera frame
        right_frame: Right camera frame
        qr_data: QR code data string (optional)
        position_3d: (X, Y, Z) tuple in mm (optional)
        distance: Distance in mm (optional)
    
    Returns:
        Combined display frame
    """
    # Ensure frames are 3-channel color
    if len(left_frame.shape) == 2:
        left_frame = cv.cvtColor(left_frame, cv.COLOR_GRAY2BGR)
    if len(right_frame.shape) == 2:
        right_frame = cv.cvtColor(right_frame, cv.COLOR_GRAY2BGR)
    
    # Create copies to draw on
    left_display = left_frame.copy()
    right_display = right_frame.copy()
    
    # Add labels for left and right cameras
    font = cv.FONT_HERSHEY_SIMPLEX
    font_scale = 0.7
    font_thickness = 2
    
    # Left camera label
    cv.putText(left_display, "LEFT", (10, 30), font, font_scale, (0, 255, 255), font_thickness)
    
    # Right camera label
    cv.putText(right_display, "RIGHT", (10, 30), font, font_scale, (0, 255, 255), font_thickness)
    
    # Combine left and right horizontally
    combined_view = np.hstack((left_display, right_display))
    
    # Add position information overlay on top right if available
    if position_3d is not None:
        X, Y, Z = position_3d
        
        # Create semi-transparent overlay box
        overlay = combined_view.copy()
        box_width = 400
        box_height = 160
        x_start = combined_view.shape[1] - box_width - 10
        y_start = 10
        
        cv.rectangle(overlay, (x_start, y_start), 
                    (x_start + box_width, y_start + box_height), 
                    (0, 0, 0), -1)
        cv.addWeighted(overlay, 0.6, combined_view, 0.4, 0, combined_view)
        
        # Add border
        cv.rectangle(combined_view, (x_start, y_start), 
                    (x_start + box_width, y_start + box_height), 
                    (0, 255, 0), 2)
        
        # Add text
        font_scale_info = 0.6
        font_thickness_info = 2
        line_height = 30
        
        # Title
        cv.putText(combined_view, "3D POSITION", 
                  (x_start + 10, y_start + 25), 
                  font, font_scale_info, (0, 255, 0), font_thickness_info)
        
        # Position values
        cv.putText(combined_view, f"X: {X:8.1f} mm", 
                  (x_start + 10, y_start + 25 + line_height), 
                  font, font_scale_info, (255, 255, 255), 1)
        
        cv.putText(combined_view, f"Y: {Y:8.1f} mm", 
                  (x_start + 10, y_start + 25 + 2*line_height), 
                  font, font_scale_info, (255, 255, 255), 1)
        
        cv.putText(combined_view, f"Z: {Z:8.1f} mm", 
                  (x_start + 10, y_start + 25 + 3*line_height), 
                  font, font_scale_info, (255, 255, 255), 1)
        
        # Distance
        if distance is not None:
            cv.putText(combined_view, f"Dist: {distance:6.1f} mm", 
                      (x_start + 10, y_start + 25 + 4*line_height), 
                      font, font_scale_info, (0, 255, 255), font_thickness_info)
    
    # Add QR data at bottom if available
    if qr_data is not None:
        # Create bottom bar for QR data
        overlay = combined_view.copy()
        bar_height = 50
        y_bar = combined_view.shape[0] - bar_height
        
        cv.rectangle(overlay, (0, y_bar), 
                    (combined_view.shape[1], combined_view.shape[0]), 
                    (0, 0, 0), -1)
        cv.addWeighted(overlay, 0.7, combined_view, 0.3, 0, combined_view)
        
        # Add QR data text
        cv.putText(combined_view, f"QR Data: {qr_data}", 
                  (10, y_bar + 32), 
                  font, 0.7, (0, 255, 0), 2)
    
    return combined_view


# State machine: waiting_for_qr = True means we're actively searching, False means we found one
waiting_for_qr = True
last_qr_data = None
last_position_3d = None
last_distance = None

print("\nPress 's' to start QR code detection and tracking")
print("Press 'e' to exit\n")

while True:
    key = cv.waitKey(1)
    ret = False
    while not ret:
        ret, frame = cam.read()
    
    frame_live = cv.resize(frame, None, fx=resize_scale, fy=resize_scale)

    if key == 115:  # 's' - start/restart QR code detection
        waiting_for_qr = True
        # Reset all stored QR code data
        last_qr_data = None
        last_position_3d = None
        last_distance = None
        print("\n" + "="*60)
        print("Starting QR code detection...")
        print("="*60)

    # Split the frame into left and right for display
    left = frame[0:height, 0:int(width/2)]
    right = frame[0:height, int(width/2):(width)]
    
    # Resize for display
    left_display = cv.resize(left, None, fx=resize_scale, fy=resize_scale)
    right_display = cv.resize(right, None, fx=resize_scale, fy=resize_scale)

    if waiting_for_qr:
        # Detect QR codes using pyzbar
        qr_codes_left = decode(left)
        qr_codes_right = decode(right)
        
        # Try to find matching QR codes
        found_match = False
        if qr_codes_left:
            for i, qr_left in enumerate(qr_codes_left):
                qr_right = find_matching_qr_code(qr_left, qr_codes_right)
                
                if qr_right:
                    found_match = True
                    waiting_for_qr = False  # Stop looping
                    
                    print(f"\n{'='*60}")
                    print(f"✓✓✓ MATCHING QR CODE FOUND ✓✓✓")
                    print(f"{'='*60}")
                    print(f"Data: {qr_left.data.decode('utf-8')}")
                    print(f"Type: {qr_left.type}")
                    
                    # Save QR data
                    last_qr_data = qr_left.data.decode('utf-8')
                    
                    center_left = get_qr_center(qr_left)
                    center_right = get_qr_center(qr_right)
                    print(f"\nCenter in Left frame: {center_left}")
                    print(f"Center in Right frame: {center_right}")
                    disparity = center_left[0] - center_right[0]
                    print(f"Disparity: {disparity:.2f} pixels")
                    
                    # Calculate 3D position
                    if matrix_left is not None and baseline is not None:
                        print(f"\n  Calculating 3D position...")
                        position_3d = triangulate_qr_position(qr_left, qr_right, 
                                                             matrix_left, matrix_right, baseline)
                        if position_3d:
                            X, Y, Z = position_3d
                            print(f"\n📍 3D POSITION (relative to LEFT camera origin):")
                            print(f"   X (right):    {X:10.2f} mm")
                            print(f"   Y (down):     {Y:10.2f} mm")
                            print(f"   Z (forward):  {Z:10.2f} mm")
                            distance = np.sqrt(X**2 + Y**2 + Z**2)
                            print(f"   Distance:     {distance:10.2f} mm")
                            
                            # Save position data for display
                            last_position_3d = position_3d
                            last_distance = distance
                        else:
                            print(f"  ✗ Could not calculate 3D position (invalid disparity)")
                            last_position_3d = None
                            last_distance = None
                    else:
                        print(f"  ⚠ Cannot calculate 3D position: matrix_left={matrix_left is not None}, baseline={baseline}")
                        last_position_3d = None
                        last_distance = None
                    
                    # Draw QR code boxes on display frames
                    draw_qr_code(left_display, qr_left, (0, 255, 0))
                    draw_qr_code(right_display, qr_right, (0, 255, 0))
                    
                    print(f"{'='*60}")
                    print("Press 's' to detect another QR code")
                    print(f"{'='*60}\n")
                    break
        
    if key == 101:  # 'e' - exit
        break

    # Create display layout with live top view and fixed bottom view
    live_view = create_display_layout(left_display, right_display, last_qr_data, last_position_3d, last_distance)
    
    # For bottom view, keep the last detected frame or use current if none detected yet
    if last_position_3d is not None:
        # Use the saved frame with QR codes drawn
        bottom_view = create_display_layout(left_display, right_display, last_qr_data, last_position_3d, last_distance)
    else:
        # No detection yet, just show camera views without overlay
        bottom_view = create_display_layout(left_display, right_display)
    
    # Stack vertically
    imOut = np.vstack((live_view, bottom_view))
    cv.imshow("window", imOut)

cam.release()
cv.destroyAllWindows()