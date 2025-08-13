import pyrealsense2 as rs

# Create a pipeline
pipeline = rs.pipeline()

# Configure the pipeline to stream pose data from T265
config = rs.config()
config.enable_stream(rs.stream.pose)

# Start streaming
pipeline.start(config)

try:
    while True:
        # Wait for a new set of frames
        frames = pipeline.wait_for_frames()

        # Get the pose frame
        pose_frame = frames.get_pose_frame()
        if pose_frame:
            # Get the pose data
            data = pose_frame.get_pose_data()

            # Access pose information (translation, rotation, velocity, etc.)
            print(f"Translation: x={data.translation.x:.2f}, y={data.translation.y:.2f}, z={data.translation.z:.2f}")

finally:
    # Stop streaming
    pipeline.stop()