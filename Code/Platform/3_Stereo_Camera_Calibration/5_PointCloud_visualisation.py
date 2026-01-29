# Live Point Cloud Visualization from Stereo Camera
# This script loads SGBM parameters and displays a real-time 3D point cloud

import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

#NOTE: Change these parameters according to your hardware
#----------------------------------------------------
camera_ID = 1  # Usually 0 is the builtin camera
camera_width = 2560  # Width for stereo camera (will be split in half)
camera_height = 720
#----------------------------------------------------

# Load SGBM parameters
try:
    sgbm_params = np.load("Saved_parameters/sgbm_parameters.npz")
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
    print("Loaded SGBM parameters successfully")
    print(f"numDisparities: {numDisparities}, blockSize: {blockSize}")
except:
    print("Error: Could not load SGBM parameters from Saved_parameters/sgbm_parameters.npz")
    print("Please run SGBM_live_tuner.py first and save parameters")
    exit()

# Load rectification parameters
try:
    calibration = np.load("Saved_parameters/retification_informations.npz")
    leftMapX = calibration['leftMapX']
    leftMapY = calibration['leftMapY']
    rightMapX = calibration['rightMapX']
    rightMapY = calibration['rightMapY']
    use_rectification = True
    print("Loaded rectification parameters")
except:
    use_rectification = False
    print("Warning: No rectification parameters found, using raw images")

# Connect to camera
print("Connecting to camera...")
cam = cv.VideoCapture(camera_ID)
cam.set(cv.CAP_PROP_FRAME_WIDTH, camera_width)
cam.set(cv.CAP_PROP_FRAME_HEIGHT, camera_height)

ret, frame = cam.read()
assert ret, "Failed to read from camera"
print("Camera connected successfully")

height = frame.shape[0]
width = frame.shape[1]
half_width = int(width / 2)

# Create SGBM object with loaded parameters
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

print("SGBM stereo matcher created")

# Camera parameters (approximate - adjust based on your camera)
# These should ideally come from calibration
focal_length = 700  # pixels (approximate)
baseline = 0.06  # meters (6cm between cameras - adjust for your setup)

# Camera positions (in meters, relative to center between cameras)
left_camera_pos = np.array([-baseline/2, 0, 0])  # Left camera position
right_camera_pos = np.array([baseline/2, 0, 0])  # Right camera position
camera_look_direction = np.array([0, 0, 1])  # Both cameras looking in +Z direction

print("\nStarting point cloud visualization...")
print("Controls:")
print("- Press 's' to capture and create point cloud")
print("- Press 'e' to exit")

# Create OpenCV window for camera preview
cv.namedWindow('Camera Preview', cv.WINDOW_AUTOSIZE)

frame_count = 0
fig = None
ax = None

def update_angles_plot(ax_angles, ax_3d):
    """Update the azimuth and elevation plot"""
    elev = ax_3d.elev
    azim = ax_3d.azim
    
    ax_angles.clear()
    
    # Create a simple plot showing current angles
    angles_text = f'Elevation: {elev:.1f}°\nAzimuth: {azim:.1f}°'
    ax_angles.text(0.5, 0.5, angles_text, ha='center', va='center', 
                   fontsize=16, transform=ax_angles.transAxes,
                   bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.7))
    
    # Draw angle indicators
    theta = np.linspace(0, 2*np.pi, 100)
    ax_angles.plot(np.cos(theta), np.sin(theta), 'k-', alpha=0.3)
    ax_angles.set_xlim(-1.5, 1.5)
    ax_angles.set_ylim(-1.5, 1.5)
    ax_angles.set_aspect('equal')
    ax_angles.set_title('View Angles')
    ax_angles.axis('off')

def on_move(event, ax_angles, ax_3d):
    """Update angles plot when 3D plot is rotated"""
    try:
        update_angles_plot(ax_angles, ax_3d)
        plt.pause(0.01)
    except:
        pass

try:
    while True:
        # Read frame from camera for preview
        ret = False
        while not ret:
            ret, frame = cam.read()
        
        # Show camera preview
        preview = cv.resize(frame, None, fx=0.5, fy=0.5)
        cv.putText(preview, "Press 's' to capture point cloud, 'e' to exit", 
                   (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv.imshow('Camera Preview', preview)
        
        key = cv.waitKey(1)
        
        if key == 101:  # 'e' - exit
            break
        
        if key != 115:  # Not 's' - continue to next frame
            continue
        
        # 's' pressed - capture single frame for point cloud
        print("\nCapturing frame for point cloud...")
        
        # Read frame from camera
        ret = False
        while not ret:
            ret, frame = cam.read()
        
        # Crop frame into left and right images
        left = frame[0:height, 0:half_width]
        right = frame[0:height, half_width:width]
        
        # Apply rectification if available
        if use_rectification:
            left = cv.remap(left, leftMapX, leftMapY, cv.INTER_LINEAR)
            right = cv.remap(right, rightMapX, rightMapY, cv.INTER_LINEAR)
        
        # Convert to grayscale
        gray_left = cv.cvtColor(left, cv.COLOR_BGR2GRAY)
        gray_right = cv.cvtColor(right, cv.COLOR_BGR2GRAY)
        
        # Compute disparity map
        disparity = stereo.compute(gray_left, gray_right).astype(np.float32) / 16.0
        
        # Filter invalid disparities
        valid_mask = (disparity > minDisparity) & (disparity < (minDisparity + numDisparities))
        
        # Compute depth from disparity: Z = (focal_length * baseline) / disparity
        depth = np.zeros_like(disparity)
        depth[valid_mask] = (focal_length * baseline) / (disparity[valid_mask] + 1e-6)
        
        # Limit depth range for better visualization (0.1m to 5m)
        depth[depth > 5.0] = 0
        depth[depth < 0.1] = 0
        
        # Create 3D points
        h, w = depth.shape
        
        # Create mesh grid for pixel coordinates
        u, v = np.meshgrid(np.arange(w), np.arange(h))
        
        # Convert to 3D coordinates
        # X = (u - cx) * Z / f
        # Y = (v - cy) * Z / f
        cx = w / 2.0
        cy = h / 2.0
        
        x = (u - cx) * depth / focal_length
        y = (v - cy) * depth / focal_length
        z = depth
        
        # Stack coordinates
        points = np.stack((x, y, z), axis=-1)
        
        # Flatten and filter valid points
        points = points.reshape(-1, 3)
        colors = left.reshape(-1, 3) / 255.0  # Normalize to [0, 1]
        
        valid_points_mask = (points[:, 2] > 0.1) & (points[:, 2] < 5.0)
        combined_points = points[valid_points_mask]
        combined_colors = colors[valid_points_mask]
        
        # Downsample point cloud
        step = 8
        combined_points = combined_points[::step]
        combined_colors = combined_colors[::step]
        
        print(f"Captured {len(combined_points)} points")
        
        # Update point cloud visualization
        if combined_points.shape[0] > 0:
            # Hide OpenCV window while matplotlib is open
            cv.destroyWindow('Camera Preview')
            
            # Create new figure with subplots
            if fig is None or not plt.fignum_exists(1):
                plt.close('all')
                fig = plt.figure(figsize=(18, 8))
                
            fig.clear()
            
            # Create subplots: point cloud, image, view angles
            ax = fig.add_subplot(131, projection='3d')
            ax_img = fig.add_subplot(132)
            ax_angles = fig.add_subplot(133)
            
            # Plot point cloud
            ax.scatter(combined_points[:, 0], combined_points[:, 1], combined_points[:, 2], 
                      c=combined_colors, s=1, marker='.')
            
            # Add camera positions and viewing directions
            arrow_length = 0.5
            
            # Left camera
            ax.scatter(*left_camera_pos, color='red', s=100, marker='o', label='Left Camera')
            ax.quiver(left_camera_pos[0], left_camera_pos[1], left_camera_pos[2],
                     camera_look_direction[0]*arrow_length, camera_look_direction[1]*arrow_length, 
                     camera_look_direction[2]*arrow_length, color='red', arrow_length_ratio=0.2, linewidth=2)
            
            # Right camera
            ax.scatter(*right_camera_pos, color='blue', s=100, marker='o', label='Right Camera')
            ax.quiver(right_camera_pos[0], right_camera_pos[1], right_camera_pos[2],
                     camera_look_direction[0]*arrow_length, camera_look_direction[1]*arrow_length, 
                     camera_look_direction[2]*arrow_length, color='blue', arrow_length_ratio=0.2, linewidth=2)
            
            ax.legend()
            
            # Set labels and limits for point cloud
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            ax.set_zlabel('Z (m)')
            frame_count += 1
            ax.set_title(f'Point Cloud #{frame_count} ({len(combined_points)} points)')
            
            # Set axis limits for better visualization
            ax.set_xlim([-1, 1])
            ax.set_ylim([-1, 1])
            ax.set_zlim([0, 3])
            
            # Set initial view (90 deg around X, 180 deg around Z)
            ax.view_init(elev=-75, azim=-90)
            
            # Show original image
            left_rgb = cv.cvtColor(left, cv.COLOR_BGR2RGB)
            ax_img.imshow(left_rgb)
            ax_img.set_title('Source Image')
            ax_img.axis('off')
            
            # Initialize angles plot
            update_angles_plot(ax_angles, ax)
            
            # Connect event for viewing angle updates
            fig.canvas.mpl_connect('motion_notify_event', lambda event: on_move(event, ax_angles, ax))
            
            plt.tight_layout()
            
            # Show plot and wait for user to close it
            print("Point cloud window opened. Interact with the 3D plot, then close the window to continue.")
            plt.show(block=True)
            
            # Recreate OpenCV window after matplotlib is closed
            cv.namedWindow('Camera Preview', cv.WINDOW_AUTOSIZE)

except KeyboardInterrupt:
    print("\nInterrupted by user")

except Exception as e:
    print(f"\nError: {e}")

finally:
    # Cleanup
    cam.release()
    cv.destroyAllWindows()
    if fig is not None:
        try:
            plt.close(fig)
        except:
            pass
    print("Point cloud visualization closed")
