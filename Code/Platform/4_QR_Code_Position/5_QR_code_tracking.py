import cv2 as cv
import numpy as np
from pyzbar.pyzbar import decode
import warnings
import time

# Suppress pyzbar assertion warnings
warnings.filterwarnings("ignore")

#NOTE: Change these parameters in accordance to your hardware
#----------------------------------------------------
camera_ID = 1  # usually 0 is the builtin camera
camera_width = 2560  # Width for stereo camera (will be split in half)
camera_height = 720
#----------------------------------------------------
resize_scale = 1.2

# Color palette to keep IDs visually distinct
id_colors = [
    (50, 205, 50),     # lime green
    (0, 215, 255),     # gold
    (255, 105, 65),    # blue-ish
    (255, 0, 255),     # magenta
    (0, 191, 255),     # deep sky blue
    (180, 105, 255),   # pink-ish
]

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

def create_stereo_matcher(num_disp):
    num_disp = max(16, int(num_disp))
    if num_disp % 16 != 0:
        num_disp = (num_disp // 16 + 1) * 16

    if mode == 0:
        return cv.StereoSGBM_create(
            minDisparity=minDisparity,
            numDisparities=num_disp,
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

    return cv.StereoSGBM_create(
        minDisparity=minDisparity,
        numDisparities=num_disp,
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


# Create SGBM stereo matcher
numDisparities = max(16, (numDisparities // 16) * 16)
stereo = create_stereo_matcher(numDisparities)

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


def match_qr_codes(qr_left_list, qr_right_list, y_tolerance=10):
    """Match QR codes between left and right frames based on content and row proximity."""
    matches = []
    right_by_data = {}
    for qr in qr_right_list:
        right_by_data.setdefault(qr.data, []).append(qr)

    for qr_left in qr_left_list:
        candidates = right_by_data.get(qr_left.data, [])
        best = None
        best_dy = None
        center_left = get_qr_center(qr_left)
        for qr_right in candidates:
            dy = abs(center_left[1] - get_qr_center(qr_right)[1])
            if dy < y_tolerance and (best_dy is None or dy < best_dy):
                best = qr_right
                best_dy = dy
        if best is not None:
            matches.append((qr_left, best))
    return matches


def assign_qr_id(data_bytes, id_lookup):
    """Return a stable numeric ID for a QR payload."""
    if data_bytes not in id_lookup:
        id_lookup[data_bytes] = len(id_lookup) + 1
    return id_lookup[data_bytes]


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
    """
    Calculate 3D position using triangulation.
    
    Coordinate system:
    - Origin: centered between the two cameras
    - X-axis: pointing right (positive = right)
    - Y-axis: pointing up (inverted from image coordinates)
    - Z-axis: pointing forward (into the scene)
    """
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
    
    # X centered between cameras (baseline/2 offset from left camera), Y inverted
    X = (x_center - cx) * Z / f + baseline / 2
    Y = -(y_center - cy) * Z / f
    
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
    
    if len(valid_region) < 5:
        # Expand search region if QR region is too sparse
        expand = 30
        x1 = max(0, rect.left - expand)
        x2 = min(disparity_map.shape[1], rect.left + rect.width + expand)
        y1 = max(0, rect.top - expand)
        y2 = min(disparity_map.shape[0], rect.top + rect.height + expand)
        
        region = disparity_map[y1:y2, x1:x2]
        valid_region = region[region > 0]
    
    if len(valid_region) == 0:
        return None, None, None
    
    # Use median for robustness
    avg_disparity = np.median(valid_region)
    
    if avg_disparity <= 0:
        return None, None, None
    
    # Calculate depth: Z = (f * baseline) / disparity (baseline in mm, result in mm)
    Z = (f * baseline) / avg_disparity
    
    # Calculate X, Y for 3D position
    cx_img, cy_img = extract_principal_point(matrix_left)
    X = (cx - cx_img) * Z / f
    Y = (cy - cy_img) * Z / f
    
    distance = np.sqrt(X**2 + Y**2 + Z**2)
    
    # Return region for visualization
    return (X, Y, Z), distance, region


def compute_confidence(distance_tri, distance_depth):
    """Confidence based on agreement between triangulation and depth-map distances."""
    if distance_tri is None or distance_depth is None:
        return 0.0, None
    diff = abs(distance_tri - distance_depth)
    mean_d = max((distance_tri + distance_depth) * 0.5, 1e-6)
    rel = diff / mean_d
    confidence = max(0.0, min(100.0, 100.0 * (1.0 - rel)))
    return confidence, diff


def draw_qr_code(frame, qr_code, color=(0, 255, 0)):
    """Draw QR code bounding box on frame"""
    rect = qr_code.rect
    cv.rectangle(frame, (rect.left, rect.top), 
                (rect.left + rect.width, rect.top + rect.height), color, 2)
    return frame


def draw_tracked_qr(frame, qr_code, track_info):
    """Draw bounding box and labels for a tracked QR code."""
    color = id_colors[(track_info['id'] - 1) % len(id_colors)]
    rect = qr_code.rect
    cv.rectangle(frame, (rect.left, rect.top),
                (rect.left + rect.width, rect.top + rect.height), color, 2)

    label = f"ID {track_info['id']}"  # short label above box
    cv.putText(frame, label, (rect.left, max(20, rect.top - 10)),
              cv.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    pos = track_info.get('position')
    if pos is not None:
        X, Y, Z = pos
        pos_text = f"X={X:.0f} Y={Y:.0f} Z={Z:.0f} mm"
        cv.putText(frame, pos_text, (rect.left, rect.top + rect.height + 20),
                  cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    conf = track_info.get('confidence')
    if conf is not None:
        conf_text = f"Conf={conf:.0f}%"
        cv.putText(frame, conf_text, (rect.left, rect.top + rect.height + 40),
                  cv.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    content = track_info.get('data')
    if content:
        content_text = content[:40] + ('…' if len(content) > 40 else '')
        cv.putText(frame, content_text, (rect.left, rect.top + rect.height + 60),
                  cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

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


def create_display_layout(left_frame, right_frame, tracked_qrs):
    """Create the side-by-side view with an info bar of all tracked QR codes."""
def create_display_layout(left_frame, tracked_qrs):
    """Create fullscreen left image with optional info bar."""
    if len(left_frame.shape) == 2:
        left_frame = cv.cvtColor(left_frame, cv.COLOR_GRAY2BGR)

    display = left_frame.copy()

    if tracked_qrs:
        overlay = display.copy()
        line_height = 28
        bar_height = line_height * (len(tracked_qrs) + 1) + 12
        y_start = display.shape[0] - bar_height

        cv.rectangle(overlay, (0, y_start), (display.shape[1], display.shape[0]), (0, 0, 0), -1)
        cv.addWeighted(overlay, 0.6, display, 0.4, 0, display)

        font = cv.FONT_HERSHEY_SIMPLEX
        cv.putText(display, "TRACKED QR CODES", (10, y_start + line_height - 8), font, 0.7, (0, 255, 255), 2)

        for idx, track in enumerate(tracked_qrs):
            y = y_start + line_height * (idx + 1) + 8
            X, Y, Z = track['position'] if track.get('position') is not None else (0, 0, 0)
            conf = track.get('confidence', 0)
            text = f"ID {track['id']}: {track['data']} | X={X:.0f} Y={Y:.0f} Z={Z:.0f} mm | Conf={conf:.0f}%"
            color = track['color']
            cv.putText(display, text, (10, y), font, 0.6, color, 2)

    return display


# State variables
qr_id_lookup = {}

print("\nLive QR tracking running...")
print("Press 's' to reset IDs, 'e' to exit.\n")

last_debug_time = time.time()
debug_interval = 0.5  # 500ms

while True:
    key = cv.waitKey(1) & 0xFF

    ret = False
    while not ret:
        ret, frame = cam.read()

    frame = cv.rotate(frame, cv.ROTATE_180)  # camera is mounted upside down

    left = frame[0:height, 0:int(width/2)]
    right = frame[0:height, int(width/2):(width)]

    if leftMapX is not None:
        left_rect = cv.remap(left, leftMapX, leftMapY, cv.INTER_LINEAR)
        right_rect = cv.remap(right, rightMapX, rightMapY, cv.INTER_LINEAR)
    else:
        left_rect = left
        right_rect = right

    if key == 115:  # 's' - reset IDs
        qr_id_lookup.clear()
        print("ID map cleared")

    if key == 101:  # 'e' - exit
        break

    # Decode with warning suppression
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        qr_codes_left = decode(left_rect)
        qr_codes_right = decode(right_rect)

    matched_pairs = match_qr_codes(qr_codes_left, qr_codes_right, y_tolerance=12)
    tracked_qrs = []

    left_annotated = left_rect.copy()
    right_annotated = right_rect.copy()

    disparity_map = None
    if matched_pairs:
        gray_left = cv.cvtColor(left_rect, cv.COLOR_BGR2GRAY)
        gray_right = cv.cvtColor(right_rect, cv.COLOR_BGR2GRAY)
        disparity_map = stereo.compute(gray_left, gray_right).astype(np.float32) / 16.0

    for qr_left, qr_right in matched_pairs:
        data_str = qr_left.data.decode('utf-8') if qr_left.data else ""
        qr_id = assign_qr_id(qr_left.data, qr_id_lookup)

        position_tri, distance_tri = triangulate_qr_position(qr_left, qr_right, matrix_left, baseline)

        position_depth = None
        distance_depth = None
        if disparity_map is not None:
            position_depth, distance_depth, _ = get_depth_from_disparity_map(
                disparity_map, qr_left, matrix_left, baseline)

        fused_position = None
        fused_z = None
        if position_tri is not None and position_depth is not None:
            fused_z = 0.5 * (position_tri[2] + position_depth[2])
            fused_position = (position_tri[0], position_tri[1], fused_z)
        elif position_tri is not None:
            fused_position = position_tri
            fused_z = position_tri[2]
        elif position_depth is not None:
            fused_position = position_depth
            fused_z = position_depth[2]

        confidence, diff = compute_confidence(distance_tri, distance_depth)

        track_info = {
            'id': qr_id,
            'data': data_str,
            'position': fused_position,
            'distance_tri': distance_tri,
            'distance_depth': distance_depth,
            'confidence': confidence,
            'color': id_colors[(qr_id - 1) % len(id_colors)]
        }

        tracked_qrs.append(track_info)
        draw_tracked_qr(left_annotated, qr_left, track_info)
        draw_qr_code(right_annotated, qr_right, track_info['color'])

    display = create_display_layout(left_annotated, tracked_qrs)
    cv.imshow("window", display)

    current_time = time.time()
    if current_time - last_debug_time >= debug_interval:
        for track in tracked_qrs:
            d_tri = track.get('distance_tri')
            d_depth = track.get('distance_depth')
            Z = track['position'][2] if track['position'] else 0
            conf = track.get('confidence', 0)
            print(f"ID {track['id']}: DepthMap={d_depth:.1f}mm | Triangulation={d_tri:.1f}mm | Z={Z:.1f}mm | Conf={conf:.1f}%")
        last_debug_time = current_time

cam.release()
cv.destroyAllWindows()
print("Program closed")
