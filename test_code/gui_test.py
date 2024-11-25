import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import numpy as np

# 创建应用程序
app = QtGui.QApplication([])

# 创建窗口
win = pg.GraphicsLayoutWidget(show=True)
win.setWindowTitle("Real-Time Plot with Button")

# 创建绘图区域
plot = win.addPlot(title="Real-Time Plot")
curve = plot.plot(pen="r")  # 设置曲线为红色
plot.setYRange(-1.5, 1.5)  # 设置 y 轴范围

# 初始化数据
x = np.linspace(0, 2 * np.pi, 500)  # x 数据
y = np.sin(x)  # 初始 y 数据
phase = 0  # 相位变量

# 数据更新函数
def update():
    global phase, x
    phase += 0.1
    y = np.sin(x + phase)  # 生成新的 y 数据
    curve.setData(x, y)  # 更新曲线数据

# 创建定时器，设置刷新频率为 50Hz
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(1000)  # 每隔 20 毫秒更新一次

# 创建按钮关闭功能
def stop_plot():
    timer.stop()  # 停止定时器
    win.close()  # 关闭窗口
    main_window.close()

# 创建按钮
button = QtGui.QPushButton("Stop")
button.clicked.connect(stop_plot)  # 按钮点击后触发关闭功能

# 将按钮添加到窗口
layout = QtGui.QVBoxLayout()  # 创建一个垂直布局
layout.addWidget(win)  # 添加绘图窗口
layout.addWidget(button)  # 添加按钮

# 创建主窗口来容纳绘图和按钮
main_window = QtGui.QWidget()
main_window.setLayout(layout)
main_window.resize(800, 600)
main_window.setWindowTitle("Real-Time Plot with Button")
main_window.show()

# 开始事件循环
app.exec_()

print("plotting finished")
print("calculate and store homing offset...")