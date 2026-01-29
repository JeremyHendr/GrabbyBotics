import cv2 as cv
import numpy as np
from pyzbar.pyzbar import decode

#NOTE: Change these parameters in accordance to your hardware
#----------------------------------------------------
camera_ID = 1  # usually 0 is the builtin camera
camera_width = 2560  # Width for stereo camera (will be split in half)
camera_height = 720
#----------------------------------------------------
resize_scale = 0.6

# Load SGBM parameters
print("Loading SGBM parameters...")
try:
    sgbm_params = np.load("../Calibration/Saved_parameters/sgbm_parameters.npz")
    minDisparity = int(sgbm_params['minDisparity'])
    numDisparities = int(sgbm_params['numDisparities'])
    blockSize = int(sgbm_params['blockSize'])
    P1 = int(sgbm_params['P1'])
    P2 = int(sgbm_params['P2'])
    disp12MaxDiff = int(sgbm_params['disp12MaxDiff'])
    preFilterCap = int(sgbm_params['preFilterCap'])
    uniquenessRatio = int(sgbm_params['uniquenessRatio'])
    speckleWindowSize = int(sgbm_params['speckleWindowSize'])
    speckleRange = int(sgbm_params['speckleRange'])
    mode = int(sgbm_params['mode'])
    print("✓ SGBM parameters loaded successfully")
except:
    print("⚠ Could not load SGBM parameters, using defaults")
    minDisparity = 0
    numDisparities = 64
    blockSize = 5
    P1 = 200
    P2 = 800
    disp12MaxDiff = 1
    preFilterCap = 63
    uniquenessRatio = 10
    speckleWindowSize = 100
    speckleRange = 32
    mode = 0

# Load stereo calibration parameters
print("Loading stereo calibration parameters...")
matrix_left = None
matrix_right = None
baseline = None
leftMapX = None
leftMapY = None
rightMapX = None
rightMapY = None

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
        print(f"⚠ Camera matrices NOT found, creating approximate")
        f = imageSize[0] / (2 * np.tan(np.radians(25)))
        matrix_left = np.array([
            [f, 0, imageSize[0]/2],
            [0, f, imageSize[1]/2],
            [0, 0, 1]
        ], dtype=np.float32)
        matrix_right = matrix_left.copy()
        print(f"  Approximate focal length: {f:.1f} pixels")
    
    if 'translation_vector' in calibration_data:
        translation_vector = calibration_data['translation_vector']
        baseline = abs(float(translation_vector[0][0]))
        print(f"✓ Baseline extracted: {baseline:.2f} mm")
    else:
        print(f"⚠ Translation vector NOT found, using 60 mm")
        baseline = 60
    
    print("Calibration parameters loaded successfully")
except Exception as e:
    print(f"Warning: Could not load calibration parameters: {e}")
    baseline = 60
    f = 700
    matrix_left = np.array([[f, 0, 640], [0, f, 360], [0, 0, 1]], dtype=np.float32)
    matrix_right = matrix_left.copy()

# Create SGBM stereo matcher
if mode == 0:
    stereo = cv.StereoSGBM_create(
        minDisparity=minDisparity,
        numDisparities=numDisparities,
        blockSize=blockSize,
        P1=P1,
        P2=P2,
        disp12MaxDiff=disp12MaxDiff,
        preFilterCap=preFilterCap,
        uniquenessRatio=uniquenessRatio,
        speckleWindowSize=speckleWindowSize,
        speckleRange=speckleRange,
        mode=cv.STEREO_SGBM_MODE_SGBM
    )
else:
    stereo = cv.StereoSGBM_create(
        minDisparity=minDisparity,
        numDisparities=numDisparities,
        blockSize=blockSize,
        P1=P1,
        P2=P2,
        disp12MaxDiff=disp12MaxDiff,
        preFilterCap=preFilterCap,
        uniquenessRatio=uniquenessRatio,
        speckleWindowSize=speckleWindowSize,
        speckleRange=speckleRange,
        mode=cv.STEREO_SGBM_MODE_HH
    )

print("Connecting to Camera...")
cam = cv.VideoCapture(camera_ID)
cam.set(cv.CAP_PROP_FRAME_WIDTH, camera_width)
cam.set(cv.CAP_PROP_FRAME_HEIGHT, camera_height)

ret, frame = cam.read()
assert ret, "No frame read"

height = frame.shape[0]
width = frame.shape[1]
print(f"Picture width: {width}  height: {height}")
print("Connected to camera")
cv.namedWindow("window", cv.WINDOW_AUTOSIZE)


def get_qr_center(qr_code):
    """Extract the center coordinates of a QR code bounding box"""
    rect = qr_code.rect
    center_x = rect.left + rect.width // 2
    center_y = rect.top + rect.height // 2
    return (center_x, center_y)


def find_matching_qr_code(qr_left, qr_codes_right, y_tolerance=10):
    """Find matching QR code in right frame"""
    if not qr_codes_right:
        return None
    
    center_left = get_qr_center(qr_left)
    
    for qr_right in qr_codes_right:
        center_right = get_qr_center(qr_right)
        
        if abs(center_left[1] - center_right[1]) < y_tolerance:
            if qr_left.data == qr_right.data:
                return qr_right
    
    return None


def extract_focal_length(camera_matrix):
    """Extract focal length from camera matrix"""
    if camera_matrix is not None:
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


def triangulate_qr_position(qr_left, qr_right, matrix_left, baseline):
    """Calculate 3D position using triangulation"""
    pt_left = np.array(get_qr_center(qr_left), dtype=np.float32)
    pt_right = np.array(get_qr_center(qr_right), dtype=np.float32)
    
    f = extract_focal_length(matrix_left)
    cx, cy = extract_principal_point(matrix_left)
    
    if f is None or baseline is None:
        return None, None
    
    disparity = pt_left[0] - pt_right[0]
    
    if disparity <= 0:
        return None, None
    
    # Calculate depth: Z = (f * baseline) / disparity
    Z = (f * baseline) / disparity
    
    x_center = pt_left[0]
    y_center = pt_left[1]
    
    X = (x_center - cx) * Z / f
    Y = (y_center - cy) * Z / f
    
    distance = np.sqrt(X**2 + Y**2 + Z**2)
    
    return (X, Y, Z), distance


def get_depth_from_disparity_map(disparity_map, qr_code, matrix_left, baseline):
    """Get depth at QR code using region average from disparity map"""
    rect = qr_code.rect
    cx, cy = int(rect.left + rect.width // 2), int(rect.top + rect.height // 2)
    
    f = extract_focal_length(matrix_left)
    
    if f is None or baseline is None:
        return None, None, None
    
    # Use entire QR code region for depth estimation (not just center)
    x1 = max(0, rect.left)
    x2 = min(disparity_map.shape[1], rect.left + rect.width)
    y1 = max(0, rect.top)
    y2 = min(disparity_map.shape[0], rect.top + rect.height)
    
    region = disparity_map[y1:y2, x1:x2]
    valid_region = region[region > 0]
    
    print(f"    [DEBUG] QR region: x[{x1}:{x2}] y[{y1}:{y2}]")
    print(f"    [DEBUG] Region shape: {region.shape}, Valid pixels: {len(valid_region)}/{region.size}")
    
    if len(valid_region) < 5:
        print(f"    [DEBUG] Too few valid pixels in QR region, expanding search...")
        # Expand search region if QR region is too sparse
        expand = 30
        x1 = max(0, rect.left - expand)
        x2 = min(disparity_map.shape[1], rect.left + rect.width + expand)
        y1 = max(0, rect.top - expand)
        y2 = min(disparity_map.shape[0], rect.top + rect.height + expand)
        
        region = disparity_map[y1:y2, x1:x2]
        valid_region = region[region > 0]
        print(f"    [DEBUG] Expanded region: x[{x1}:{x2}] y[{y1}:{y2}], Valid pixels: {len(valid_region)}")
    
    if len(valid_region) == 0:
        print(f"    [DEBUG] No valid disparity values found!")
        return None, None, None
    
    # Use median for robustness
    avg_disparity = np.median(valid_region)
    mean_disparity = np.mean(valid_region)
    std_disparity = np.std(valid_region)
    
    print(f"    [DEBUG] Disparity - Median: {avg_disparity:.2f}, Mean: {mean_disparity:.2f}, Std: {std_disparity:.2f}")
    
    if avg_disparity <= 0:
        print(f"    [DEBUG] Invalid disparity value: {avg_disparity}")
        return None, None, None
    
    # Calculate depth: Z = (f * baseline) / disparity (baseline in mm, result in mm)
    Z = (f * baseline) / avg_disparity
    
    # Calculate X, Y for 3D position
    cx_img, cy_img = extract_principal_point(matrix_left)
    X = (cx - cx_img) * Z / f
    Y = (cy - cy_img) * Z / f
    
    distance = np.sqrt(X**2 + Y**2 + Z**2)
    
    print(f"    [DEBUG] Focal length: {f:.1f}, Baseline: {baseline:.2f} mm")
    print(f"    [DEBUG] Calculated Z: {Z:.1f} mm, Distance: {distance:.1f} mm")
    
    # Return region for visualization
    return (X, Y, Z), distance, region


def draw_qr_code(frame, qr_code, color=(0, 255, 0)):
    """Draw QR code bounding box on frame"""
    rect = qr_code.rect
    cv.rectangle(frame, (rect.left, rect.top), 
                (rect.left + rect.width, rect.top + rect.height), color, 2)
    return frame


def visualize_disparity_with_qr(disparity_map, qr_code, resize_scale=0.6):
    """Visualize disparity map with QR code region highlighted"""
    disparity_viz = disparity_map.copy()
    
    # Handle invalid values
    valid_disp = disparity_viz[disparity_viz > 0]
    if len(valid_disp) == 0:
        print("  No valid disparity in entire map!")
        return None
    
    disp_min, disp_max = np.min(valid_disp), np.max(valid_disp)
    
    # Normalize for visualization
    disparity_viz[disparity_viz <= 0] = disp_min
    disparity_norm = ((disparity_viz - disp_min) / (disp_max - disp_min + 1e-6) * 255).astype(np.uint8)
    disparity_colored = cv.applyColorMap(disparity_norm, cv.COLORMAP_JET)
    
    # Draw QR code region
    rect = qr_code.rect
    cv.rectangle(disparity_colored, (rect.left, rect.top), 
                (rect.left + rect.width, rect.top + rect.height), 
                (255, 255, 255), 2)  # White box
    
    # Draw center point
    center_x = rect.left + rect.width // 2
    center_y = rect.top + rect.height // 2
    cv.circle(disparity_colored, (center_x, center_y), 5, (0, 255, 255), -1)
    
    # Resize for display
    disparity_display = cv.resize(disparity_colored, None, fx=resize_scale, fy=resize_scale)
    
    return disparity_display


def create_display_layout(left_frame, right_frame, qr_data=None, 
                         distance_triangulation=None, distance_depth_map=None):
    """Create display layout with comparison overlay"""
    if len(left_frame.shape) == 2:
        left_frame = cv.cvtColor(left_frame, cv.COLOR_GRAY2BGR)
    if len(right_frame.shape) == 2:
        right_frame = cv.cvtColor(right_frame, cv.COLOR_GRAY2BGR)
    
    left_display = left_frame.copy()
    right_display = right_frame.copy()
    
    font = cv.FONT_HERSHEY_SIMPLEX
    font_scale = 0.7
    font_thickness = 2
    
    # Labels
    cv.putText(left_display, "LEFT", (10, 30), font, font_scale, (0, 255, 255), font_thickness)
    cv.putText(right_display, "RIGHT", (10, 30), font, font_scale, (0, 255, 255), font_thickness)
    
    combined_view = np.hstack((left_display, right_display))
    
    # Add comparison overlay on top right
    if distance_triangulation is not None or distance_depth_map is not None:
        overlay = combined_view.copy()
        box_width = 450
        box_height = 180
        x_start = combined_view.shape[1] - box_width - 10
        y_start = 10
        
        cv.rectangle(overlay, (x_start, y_start), 
                    (x_start + box_width, y_start + box_height), 
                    (0, 0, 0), -1)
        cv.addWeighted(overlay, 0.6, combined_view, 0.4, 0, combined_view)
        
        cv.rectangle(combined_view, (x_start, y_start), 
                    (x_start + box_width, y_start + box_height), 
                    (0, 255, 255), 2)
        
        font_scale_info = 0.6
        font_thickness_info = 1
        line_height = 30
        
        # Title
        cv.putText(combined_view, "DISTANCE COMPARISON", 
                  (x_start + 10, y_start + 25), 
                  font, font_scale_info, (0, 255, 255), 2)
        
        # Triangulation distance
        if distance_triangulation is not None:
            cv.putText(combined_view, f"Triangulation: {distance_triangulation:7.1f} mm", 
                      (x_start + 10, y_start + 25 + line_height), 
                      font, font_scale_info, (0, 255, 0), font_thickness_info)
        else:
            cv.putText(combined_view, "Triangulation: N/A", 
                      (x_start + 10, y_start + 25 + line_height), 
                      font, font_scale_info, (128, 128, 128), font_thickness_info)
        
        # Depth map distance
        if distance_depth_map is not None:
            cv.putText(combined_view, f"Depth Map:     {distance_depth_map:7.1f} mm", 
                      (x_start + 10, y_start + 25 + 2*line_height), 
                      font, font_scale_info, (255, 128, 0), font_thickness_info)
        else:
            cv.putText(combined_view, "Depth Map:     N/A", 
                      (x_start + 10, y_start + 25 + 2*line_height), 
                      font, font_scale_info, (128, 128, 128), font_thickness_info)
        
        # Difference
        if distance_triangulation is not None and distance_depth_map is not None:
            diff = abs(distance_triangulation - distance_depth_map)
            percent_diff = (diff / distance_triangulation) * 100
            cv.putText(combined_view, f"Difference:    {diff:7.1f} mm ({percent_diff:5.1f}%)", 
                      (x_start + 10, y_start + 25 + 3*line_height), 
                      font, font_scale_info, (255, 255, 255), font_thickness_info)
        
        # Agreement indicator
        if distance_triangulation is not None and distance_depth_map is not None:
            if percent_diff < 5:
                status = "Excellent"
                color = (0, 255, 0)
            elif percent_diff < 10:
                status = "Good"
                color = (0, 255, 255)
            elif percent_diff < 20:
                status = "Fair"
                color = (0, 165, 255)
            else:
                status = "Poor"
                color = (0, 0, 255)
            
            cv.putText(combined_view, f"Agreement:     {status}", 
                      (x_start + 10, y_start + 25 + 4*line_height), 
                      font, font_scale_info, color, 2)
    
    # Add QR data at bottom
    if qr_data is not None:
        overlay = combined_view.copy()
        bar_height = 50
        y_bar = combined_view.shape[0] - bar_height
        
        cv.rectangle(overlay, (0, y_bar), 
                    (combined_view.shape[1], combined_view.shape[0]), 
                    (0, 0, 0), -1)
        cv.addWeighted(overlay, 0.7, combined_view, 0.3, 0, combined_view)
        
        cv.putText(combined_view, f"QR Data: {qr_data}", 
                  (10, y_bar + 32), 
                  font, 0.7, (0, 255, 0), 2)
    
    return combined_view


# State variables
waiting_for_qr = True
last_qr_data = None
last_distance_triangulation = None
last_distance_depth_map = None

print("\nPress 's' to start QR code detection")
print("Press 'e' to exit\n")

while True:
    key = cv.waitKey(1)
    ret = False
    while not ret:
        ret, frame = cam.read()
    
    # Rotate frame 180 degrees (camera mounted upside down)
    frame = cv.rotate(frame, cv.ROTATE_180)
    
    # Split frame
    left = frame[0:height, 0:int(width/2)]
    right = frame[0:height, int(width/2):(width)]
    
    # Apply rectification if available
    if leftMapX is not None:
        left_rect = cv.remap(left, leftMapX, leftMapY, cv.INTER_LINEAR)
        right_rect = cv.remap(right, rightMapX, rightMapY, cv.INTER_LINEAR)
    else:
        left_rect = left
        right_rect = right
    
    # Resize for display
    left_display = cv.resize(left_rect, None, fx=resize_scale, fy=resize_scale)
    right_display = cv.resize(right_rect, None, fx=resize_scale, fy=resize_scale)

    if key == 115:  # 's' - start/restart
        waiting_for_qr = True
        last_qr_data = None
        last_distance_triangulation = None
        last_distance_depth_map = None
        print("\n" + "="*60)
        print("Starting QR code detection...")
        print("="*60)

    if waiting_for_qr:
        # Detect QR codes
        qr_codes_left = decode(left_rect)
        qr_codes_right = decode(right_rect)
        
        if qr_codes_left:
            for qr_left in qr_codes_left:
                qr_right = find_matching_qr_code(qr_left, qr_codes_right)
                
                if qr_right:
                    waiting_for_qr = False
                    
                    print(f"\n{'='*60}")
                    print(f"✓✓✓ MATCHING QR CODE FOUND ✓✓✓")
                    print(f"{'='*60}")
                    print(f"Data: {qr_left.data.decode('utf-8')}")
                    
                    last_qr_data = qr_left.data.decode('utf-8')
                    
                    # METHOD 1: Triangulation
                    print(f"\n[METHOD 1: Triangulation]")
                    position_tri, distance_tri = triangulate_qr_position(qr_left, qr_right, 
                                                                         matrix_left, baseline)
                    if distance_tri is not None:
                        X, Y, Z = position_tri
                        print(f"  Position: X={X:.1f}, Y={Y:.1f}, Z={Z:.1f} mm")
                        print(f"  Distance: {distance_tri:.1f} mm")
                        last_distance_triangulation = distance_tri
                    else:
                        print(f"  Could not calculate (invalid disparity)")
                        last_distance_triangulation = None
                    
                    # METHOD 2: Depth Map
                    print(f"\n[METHOD 2: Depth Map (SGBM)]")
                    # Compute disparity map
                    gray_left = cv.cvtColor(left_rect, cv.COLOR_BGR2GRAY)
                    gray_right = cv.cvtColor(right_rect, cv.COLOR_BGR2GRAY)
                    disparity_map = stereo.compute(gray_left, gray_right).astype(np.float32) / 16.0
                    
                    position_depth, distance_depth, disparity_region = get_depth_from_disparity_map(
                        disparity_map, qr_left, matrix_left, baseline)
                    
                    if distance_depth is not None:
                        X, Y, Z = position_depth
                        print(f"  Position: X={X:.1f}, Y={Y:.1f}, Z={Z:.1f} mm")
                        print(f"  Distance: {distance_depth:.1f} mm")
                        last_distance_depth_map = distance_depth
                    else:
                        print(f"  Could not calculate (no valid depth)")
                        last_distance_depth_map = None
                    
                    # Visualize disparity map with QR region highlighted
                    print(f"\n[DISPARITY MAP VISUALIZATION]")
                    disparity_display = visualize_disparity_with_qr(disparity_map, qr_left, resize_scale)
                    if disparity_display is not None:
                        cv.imshow("Disparity Map", disparity_display)
                        print(f"  Disparity map displayed (white box = QR region, yellow dot = center)")
                    
                    # Also print raw disparity statistics for debugging
                    valid_disp = disparity_map[disparity_map > 0]
                    if len(valid_disp) > 0:
                        print(f"  Global disparity range: {np.min(valid_disp):.2f} to {np.max(valid_disp):.2f} pixels")
                        print(f"  Global disparity mean: {np.mean(valid_disp):.2f} pixels")
                    
                    # Comparison
                    print(f"\n[COMPARISON]")
                    if distance_tri is not None and distance_depth is not None:
                        diff = abs(distance_tri - distance_depth)
                        percent_diff = (diff / distance_tri) * 100
                        print(f"  Difference: {diff:.1f} mm ({percent_diff:.1f}%)")
                        
                        if percent_diff < 5:
                            print(f"  Agreement: ✓ Excellent")
                        elif percent_diff < 10:
                            print(f"  Agreement: ✓ Good")
                        elif percent_diff < 20:
                            print(f"  Agreement: ⚠ Fair")
                        else:
                            print(f"  Agreement: ✗ Poor")
                    else:
                        print(f"  Cannot compare (missing data)")
                    
                    # Draw QR codes
                    draw_qr_code(left_display, qr_left, (0, 255, 0))
                    draw_qr_code(right_display, qr_right, (0, 255, 0))
                    
                    print(f"{'='*60}")
                    print("Press 's' to detect another QR code")
                    print(f"{'='*60}\n")
                    break
        
    if key == 101:  # 'e' - exit
        break

    # Create display
    display = create_display_layout(left_display, right_display, 
                                   last_qr_data, 
                                   last_distance_triangulation, 
                                   last_distance_depth_map)
    
    cv.imshow("window", display)

cam.release()
cv.destroyAllWindows()
print("Program closed")
