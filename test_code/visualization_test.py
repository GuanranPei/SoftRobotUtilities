from test_code.visualization_pcc import plot_single_sections as plt_soft
import matplotlib.pyplot as plt

s = 1
deltax = 1
deltay = 1

sections, [circle0, circle1] = plt_soft(s, deltax, deltay)
print(sections.shape)
print(circle0.shape)
print(circle1.shape)

# Plot circles
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_box_aspect([1, 1, 1])
# 手动设置坐标范围，确保各轴范围一致
ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 1])

circle_0 = ax.plot(circle0[:, 0], circle0[:, 1], circle0[:, 2], color='k', linewidth=1.5)
circle_1 = ax.plot(circle1[:, 0], circle1[:, 1], circle1[:, 2], color='k', linewidth=1.5)

# Plot central axis
ax.plot(sections[0,:], sections[1,:], sections[2,:], color='blue', label="Path")

plt.pause(0.1)

# 更新数据并刷新图形
for i in range(10):
    # 清除旧的绘图
    ax.cla()

    # 生成新的数据
    sections, [circle0, circle1] = plt_soft(1, 1 + 0.1 * i, 1 + 0.1 * i)

    # 绘制新数据
    ax.plot(circle0[:, 0], circle0[:, 1], circle0[:, 2], color='k', linewidth=1.5)
    ax.plot(circle1[:, 0], circle1[:, 1], circle1[:, 2], color='k', linewidth=1.5)
    ax.plot(sections[0, :], sections[1, :], sections[2, :], color='blue', label="Path")

    # 重设坐标范围
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])

    plt.pause(0.1)  # 刷新图形