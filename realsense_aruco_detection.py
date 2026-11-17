import pyrealsense2 as rs
import numpy as np
import cv2

pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(
    rs.stream.color,
    640,
    480,
    rs.format.bgr8,
    30
)

pipeline.start(config)

aruco_dict = cv2.aruco.getPredefinedDictionary(
    cv2.aruco.DICT_4X4_50
)

aruco_params = cv2.aruco.DetectorParameters()

detector = cv2.aruco.ArucoDetector(
    aruco_dict,
    aruco_params
)
kp = 0.15
kd = 0.15
prev_theta_error = 0
prev_time = cv2.getTickCount() / cv2.getTickFrequency()

try:
    while True:

        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            continue

        image = np.asanyarray(color_frame.get_data())
        corners, ids, rejected = detector.detectMarkers(image)
        font = cv2.FONT_HERSHEY_SIMPLEX
        if ids is not None:
            profile = pipeline.get_active_profile()
            color_profile = rs.video_stream_profile(
            profile.get_stream(rs.stream.color)
            )
            intrinsics = color_profile.get_intrinsics()
            camera_matrix = np.array([
            [intrinsics.fx, 0, intrinsics.ppx],
            [0, intrinsics.fy, intrinsics.ppy],
            [0, 0, 1]
            ])
            dist_coeffs = np.array(intrinsics.coeffs)
            marker_size = 0.1

            
            cv2.aruco.drawDetectedMarkers(
            image,
            corners,
            ids
            )

            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners,
            marker_size,
            camera_matrix,
            dist_coeffs
            )
            x = tvecs[0][0][0]
            y = tvecs[0][0][1]
            z = tvecs[0][0][2]

            cv2.putText(
                image,
                f"Pose: x = {x:.2f},y={y:.2f},z={z:.2f}",
                (20,50),
                font,
                1,
                (0,255,255),
                2
            )
            theta_error = np.arctan2(y, z)
            current_time = cv2.getTickCount() / cv2.getTickFrequency()
            dt = current_time - prev_time
            theta_dot = 0
            if dt>0:
                theta_dot = (theta_error - prev_theta_error) / dt
            
            pitch_cmd = kp * theta_error + kd * theta_dot
            prev_theta_error = theta_error
            prev_time = current_time
            cv2.drawFrameAxes(
            image,
            camera_matrix,
            dist_coeffs,
            rvecs[0],
            tvecs[0],
            0.05
            )

            cv2.putText(
            image,
            f"Theta Error: {np.degrees(theta_error):.2f}",
            (20,120),
            font,
            1,
            (0,255,255),
            2
            )
            cv2.putText(
            image,
            f"Pitch Cmd: {pitch_cmd:.2f}",
            (20,170),
            font,
            1,
            (0,255,0),
            2
            )

        cv2.imshow("RGB Feed", image)


        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
