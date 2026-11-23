# AUTOKLINIK 3D — iPhone wheel alignment prototype

## Goal
Experimental wheel alignment measurement using one iPhone, printed fiducial markers on wheels and fixed reference posts, plus an optional phone mount for a second inclination reading.

**Status:** engineering specification / prototype plan. Not validated for professional alignment or road-safety decisions. This README does not claim a working iOS build.

## Measurement workflow
1. Park on a level surface, check tire pressures and vehicle ride height, secure the car.
2. Mount rigid planar AprilTag/ArUco targets on each wheel, ideally via a hub-centered fixture that avoids tire-sidewall movement. Put reference targets on stable posts.
3. Calibrate the iPhone camera intrinsics and lens distortion for the selected camera, focus and zoom.
4. Capture overlapping video/images while walking around the vehicle. Each image should contain several visible targets, including fixed references; solve camera poses and target poses jointly.
5. Estimate each wheel plane and wheel-center pose. Rotate wheels and repeat to characterize and compensate rim/fixture runout.
6. For an affected wheel, attach the phone to a **rigid calibrated wheel fixture**, record its orientation using CoreMotion, and compare to the optical estimate. The phone's gyro alone does **not** provide an absolute heading or reliable toe angle; its orientation drifts. Gravity-referenced pitch/roll may help inclination/camber checks after careful calibration.
7. Report per-wheel measurements with uncertainty and calibration status. Reject insufficient coverage or inconsistent observations.

## Technical approach
- iOS: SwiftUI, AVFoundation camera capture, CoreMotion motion data.
- Fiducials: AprilTag or ArUco detector with known physical marker dimensions.
- Geometry: calibrated PnP + multi-view bundle adjustment; establish scale and a vehicle coordinate frame with reference targets and measured distances.
- Outputs: preliminary camber/toe estimates **only after validation**, plus observation quality, uncertainty, and a capture log.
- Avoid assuming iPhone LiDAR is present. Camera-only processing is a separate feasibility milestone.

## Important limitations
- Camera-only views without sufficient common reference points cannot guarantee accurate wheel geometry.
- Gyroscope integration drifts; magnetometer is susceptible to workshop metal interference.
- Phone attachment changes reference orientation unless mount offset is calibrated.
- Suspension motion, uneven floors, wheel runout, lighting, motion blur and marker placement introduce errors.
- Do not present measurements as suitable for customer repair decisions until compared against calibrated commercial alignment equipment.

## MVP acceptance criteria
- Detect uniquely numbered targets in a saved capture and reject duplicate IDs or unknown marker sizes.
- Show target detection count and reprojection error for each frame.
- Persist a capture session with timestamp, marker map, camera calibration and phone-motion log.
- Identify missing fixed-reference coverage rather than inventing a 3D result.
- Validate repeatability across 10 independent scans and compare camber/toe against a calibrated alignment rack; define tolerances from the validation results, not guesses.

## Next implementation step
Implement the SwiftUI capture screen and marker detector, then test with printed targets at known distances before attempting wheel-angle estimates.
