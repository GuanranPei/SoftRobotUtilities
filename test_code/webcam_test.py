import cv2

# 创建摄像头对象，选择摄像头（索引号可能需要根据设备调整，通常0是默认摄像头）
cam = cv2.VideoCapture(0, cv2.CAP_DSHOW)

# 设置分辨率（1920x1080）
cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

# 设置手动曝光模式和曝光值
# 注意：部分摄像头可能不支持完全手动曝光，这需要查看摄像头的驱动支持
cam.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # 0.25 表示手动模式
cam.set(cv2.CAP_PROP_EXPOSURE, -6)        # 曝光值

# 预览摄像头图像（通过窗口显示）
print("Press 'q' to close the preview window.")
while True:
    ret, frame = cam.read()
    if not ret:
        print("Failed to grab frame")
        break

    # 显示图像
    cv2.imshow('Preview', frame)

    # 按 'q' 键退出预览
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 获取图像并保存
ret, img = cam.read()
if ret:
    cv2.imwrite('captured_image.jpg', img)
    print("Image saved as 'captured_image.jpg'")
else:
    print("Failed to capture image")

# 释放摄像头资源
cam.release()
cv2.destroyAllWindows()
