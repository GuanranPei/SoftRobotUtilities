import matplotlib.pyplot as plt
import numpy as np
import tkinter as tk
from tkinter import ttk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# 生成数据存储变量
maxDataPoints = 50
time = np.arange(1, maxDataPoints + 1)
storage_base_1 = np.zeros((maxDataPoints, 3))
storage_1_2 = np.zeros((maxDataPoints, 3))
storage_2_3 = np.zeros((maxDataPoints, 3))

# 创建 UI 窗口
root = tk.Tk()
root.title('IMU 位姿曲线图')

# 创建 Matplotlib 图形和子图
fig, axs = plt.subplots(3, 1, figsize=(10, 8))

# 绘制 base_1 图
axs[0].plot(time, storage_base_1[:, 1], 'r', linewidth=2, label='pitch')
axs[0].plot(time, storage_base_1[:, 2], 'g', linewidth=2, label='roll')
axs[0].axhline(0, color='k', linestyle='--', linewidth=2, label='y = 0')
axs[0].set_title('base_1')
axs[0].set_xlabel('newest points')
axs[0].set_ylabel('angle/deg')
axs[0].legend()
axs[0].set_xlim([min(time), max(time)])

# 绘制 1_2 图
axs[1].plot(time, storage_1_2[:, 1], 'r', linewidth=2, label='pitch')
axs[1].plot(time, storage_1_2[:, 2], 'g', linewidth=2, label='roll')
axs[1].axhline(0, color='k', linestyle='--', linewidth=2, label='y = 0')
axs[1].set_title('1_2')
axs[1].set_xlabel('newest points')
axs[1].set_ylabel('angle/deg')
axs[1].legend()
axs[1].set_xlim([min(time), max(time)])

# 绘制 2_3 图
axs[2].plot(time, storage_2_3[:, 1], 'r', linewidth=2, label='pitch')
axs[2].plot(time, storage_2_3[:, 2], 'g', linewidth=2, label='roll')
axs[2].axhline(0, color='k', linestyle='--', linewidth=2, label='y = 0')
axs[2].set_title('2_3')
axs[2].set_xlabel('newest points')
axs[2].set_ylabel('angle/deg')
axs[2].legend()
axs[2].set_xlim([min(time), max(time)])

# 调整布局
plt.tight_layout()

# 按钮回调函数
def on_button_click():
    print("Save Cal button clicked")
    root.destroy()  # 关闭窗口

# 创建按钮并绑定回调函数
button = ttk.Button(root, text='Save Cal', command=on_button_click)
button.pack()

# 嵌入 Matplotlib 图形到 Tkinter 窗口中
canvas = FigureCanvasTkAgg(fig, master=root)
canvas.draw()
canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

# 启动 Tkinter 主循环
root.mainloop()

