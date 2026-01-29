# QR Code 3D Position Detection using Stereo Vision

This project detects QR codes in a stereo camera setup and calculates their 3D position in space using triangulation.

## Coordinate System and Camera Setup

### Origin and Coordinate Frame

The **origin of the coordinate system is at the LEFT camera's optical center**.

**Coordinate axes:**
- **X-axis**: Points to the right (horizontal, positive = right side)
- **Y-axis**: Points downward (vertical, positive = down)  
- **Z-axis**: Points forward into the scene (depth, positive = away from camera)

```
        Y (down)
        |
        |
        |______ X (right)
       /
      /
     Z (forward)
```

### Stereo Camera Configuration

The stereo setup consists of two cameras arranged horizontally:

- **Left Camera**: Reference camera (origin of coordinate system)
- **Right Camera**: Positioned to the right of the left camera

**Baseline**: The distance between the two camera centers (typically 60-120 mm)

### Extrinsic Parameters

The relationship between the two cameras is described by extrinsic parameters:

#### Rotation Matrix (R)
A 3×3 matrix describing the rotation from the left camera frame to the right camera frame. For a parallel stereo setup (rectified cameras), this is typically close to the identity matrix.

```
R = [[r11  r12  r13]
     [r21  r22  r23]
     [r31  r32  r33]]
```

#### Translation Vector (T)
A 3×1 vector describing the translation from the left camera center to the right camera center.

```
T = [[Tx]    (horizontal displacement - baseline)
     [Ty]    (vertical displacement - ideally 0)
     [Tz]]   (depth displacement - ideally 0)
```

For an ideal horizontal stereo setup:
- `Tx = -baseline` (negative because right camera is to the left's right)
- `Ty ≈ 0` (cameras at same height)
- `Tz ≈ 0` (cameras at same depth)

**Note**: The baseline used in triangulation is `baseline = |Tx|` (absolute value of the x-component of the translation vector).

### Intrinsic Parameters

Each camera has its own intrinsic parameters stored in the camera matrix:

```
K = [[fx   0  cx]
     [ 0  fy  cy]
     [ 0   0   1]]
```

Where:
- `fx, fy`: Focal lengths in pixels (horizontal and vertical)
- `cx, cy`: Principal point (optical center) in pixels

### 3D Position Calculation

The 3D position of a detected QR code is calculated using:

1. **Disparity**: `d = x_left - x_right` (difference in pixel coordinates)
2. **Depth**: `Z = (f × baseline) / d`
3. **X coordinate**: `X = (x_left - cx) × Z / f`
4. **Y coordinate**: `Y = (y_left - cy) × Z / f`

Where:
- `f`: Focal length from camera matrix
- `cx, cy`: Principal point from camera matrix
- `baseline`: Distance between cameras (from translation vector)

### Example Position Output

When a QR code is detected, the output shows:

```
📍 3D POSITION (relative to LEFT camera origin):
   X (right):      45.3 mm    (45.3 mm to the right of center)
   Y (down):      -20.1 mm    (20.1 mm above the center)
   Z (forward):   500.0 mm    (500 mm in front of camera)
   Distance:      502.5 mm    (straight-line distance from origin)
```

### Coordinate System Visualization

```
                    Right Camera
                         ↓
    Left Camera    [baseline]
         ↓              ↓
        [●]------------[●]
         |              
         | Z            
         |↗             
         |              
      Y ↓|              
         |_____ X →     
                        
                [QR Code]
              Position (X,Y,Z)
```

## Files Description

- `3_QR_code_position.py`: Main script for QR code detection and 3D positioning
- `2_get_camera_parameters.py`: Calibration script (in ../Calibration/)
- `retification_informations.npz`: Calibration data containing all parameters

## Calibration Parameters Stored

The calibration file contains:
- `matrix_left`, `matrix_right`: Intrinsic camera matrices (K)
- `translation_vector`: Translation between cameras (contains baseline)
- `rotation_matrix`: Rotation between cameras
- `leftProjection`, `rightProjection`: Projection matrices for rectification
- `leftMapX`, `leftMapY`, `rightMapX`, `rightMapY`: Rectification maps
- `distortion_coef_left`, `distortion_coef_right`: Lens distortion coefficients

## Usage

1. Run calibration (if not done already):
   ```bash
   cd ../Calibration
   python 2_get_camera_parameters.py
   ```

2. Run QR code detection:
   ```bash
   python 3_QR_code_position.py
   ```

3. Controls:
   - Press **'s'** to start/restart QR code detection
   - Press **'e'** to exit

## Display Layout

The window shows:
- **Top**: Live stereo camera feed (left and right views)
- **Bottom**: Last detected QR code with position overlay
- **Top-right overlay**: 3D position information (X, Y, Z, Distance)
- **Bottom bar**: QR code data
- **Labels**: "LEFT" and "RIGHT" on camera views
- **Green boxes**: Detected QR codes

## Requirements

- Python 3.x
- OpenCV (`cv2`)
- NumPy
- pyzbar
- Stereo camera or two synchronized cameras

## References

- Coordinate system follows OpenCV stereo vision conventions
- Origin at left camera is standard in stereo vision
- Extrinsic parameters from `cv2.stereoCalibrate()`
- 3D reconstruction using triangulation from disparity
