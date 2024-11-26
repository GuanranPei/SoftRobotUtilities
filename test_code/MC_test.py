import roslibpy

def callback(message):
    # 打印接收到的完整消息
    print("Received message:")
    print(message)

    # 提取具体内容
    header = message.get('header', {})
    pose = message.get('pose', {})
    position = pose.get('position', {})
    orientation = pose.get('orientation', {})

    print("\nParsed Data:")
    print(f"Header - seq: {header.get('seq')}, stamp: {header.get('stamp')}, frame_id: {header.get('frame_id')}")
    print(f"Position - x: {position.get('x')}, y: {position.get('y')}, z: {position.get('z')}")
    print(f"Orientation - x: {orientation.get('x')}, y: {orientation.get('y')}, z: {orientation.get('z')}, w: {orientation.get('w')}")

# 创建与ROS的连接
client = roslibpy.Ros(host='169.254.118.70', port=9090)
client.run()

# 创建订阅者
topic = roslibpy.Topic(client, '/optitrack/GRP_sample_1/pose', 'geometry_msgs/PoseStamped')
topic.subscribe(callback)

# 保持连接
try:
    print("Subscribing to /optitrack/GRP_sample_1/pose. Press Ctrl+C to exit.")
    while client.is_connected:
        pass
except KeyboardInterrupt:
    print("\nDisconnecting...")
    topic.unsubscribe()
    client.terminate()