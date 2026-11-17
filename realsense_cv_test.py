import pyrealsense2 as rs
import numpy as np
import cv2
import time

# Configure camera
pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(
    rs.stream.color,
    640,
    480,
    rs.format.bgr8,
    30
)

# Start camera
pipeline.start(config)
prev_frame_time = 0
new_frame_time = 0

try:
    while True:

        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            continue

        image = np.asanyarray(color_frame.get_data())
        font = cv2.FONT_HERSHEY_SIMPLEX
        new_frame_time = time.time()
        fps = 1/(new_frame_time-prev_frame_time)
        prev_frame_time = new_frame_time
        fps = str(int(fps))
        cv2.putText(image, fps, (7, 70), font, 3, (100, 255, 0), 3, cv2.LINE_AA)
        cv2.imshow("RGB Feed", image)

        key = cv2.waitKey(1)

        if key == 27:
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
