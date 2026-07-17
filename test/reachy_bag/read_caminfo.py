import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
so=rosbag2_py.StorageOptions(uri="/work/my_bag", storage_id="sqlite3")
co=rosbag2_py.ConverterOptions("","")
r=rosbag2_py.SequentialReader(); r.open(so,co)
types={t.name:t.type for t in r.get_all_topics_and_types()}
want={"/teleop_camera/depth/camera_info","/camera/depth/camera_info"}
seen=set()
while r.has_next() and seen!=want:
    topic,data,ts=r.read_next()
    if topic in want and topic not in seen:
        seen.add(topic)
        msg=deserialize_message(data,get_message(types[topic]))
        print(f"\n{topic}: {msg.width}x{msg.height} frame={msg.header.frame_id} model={msg.distortion_model}")
        print(" K=",[round(x,2) for x in msg.k])
        print(" D=",[round(x,4) for x in msg.d][:8])
