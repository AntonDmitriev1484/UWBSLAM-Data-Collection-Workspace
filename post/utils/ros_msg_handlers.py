import cv2
import numpy as np

def proc_range(msg): #TODO update this to new UWB topic
    msg = msg.ranges[0]
    timestamp = msg.timestamp.sec + (msg.timestamp.nanosec * 1e-9)

    j = {
        "t":timestamp,
        "type": "uwb",
        "id": msg.id,
        "range": msg.range,
        "exchange": msg.exchange,
        "maxnoise": msg.maxnoise,
        "firstpathamp1": msg.firstpathamp1,
        "firstpathamp2": msg.firstpathamp2,
        "firstpathamp3": msg.firstpathamp3,
        "stdnoise": msg.stdnoise,
        "maxgrowthcir": msg.maxgrowthcir,
        "rxpreamcount": msg.rxpreamcount,
        "firstpath": msg.firstpath
    }

    return j

def proc_rgb_frame(msg):
    #rgb8 encoding

    timestamp = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)
    encoding = msg.encoding
    arr = msg.data

    # Make new file in out_rgb, labeled with timestamp.
    img_np = np.frombuffer(arr, dtype=np.uint8).reshape((msg.height, msg.width, 3))
    name = str(timestamp)+".png"
    cv2.imwrite(out_rgb+"/"+name, cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)) # Not exactly sure what cvtColor does...

    return {"t":timestamp, "type":"rgb", "name":name}

def proc_depth_frame(msg):
    timestamp = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)
    encoding = msg.encoding
    arr = msg.data

    img_np = np.frombuffer(arr, dtype=np.uint16).reshape((msg.height, msg.width)) # Output says unit8 but encoding says 16UC1
    name = str(timestamp)+".png"
    cv2.imwrite(out_depth+"/"+name, img_np)

    return {"t":timestamp, "type":"depth", "name":name}


# I set it to unify accel and gyro, does unified accel and gyro go to the accel topic?
def proc_imu(msg):

    # I should be looking at a topic called 'imu' -> Interesting, I think I forgot to listen to this topic.
    # Despite unite_imu being set to 2, there is no 'imu' topic available in the ros2 topics list

    timestamp = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)
    return {"t":timestamp, "type":"imu", "ax": msg.linear_acceleration.x, "ay": msg.linear_acceleration.y, "az": msg.linear_acceleration.z, 
            "gx":msg.angular_velocity.x, "gy": msg.angular_velocity.y, "gz":msg.angular_velocity.z}