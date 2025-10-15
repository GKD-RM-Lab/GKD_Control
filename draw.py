import re
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import numpy as np

# === 配置部分 ===
LOG_FILE = "fric_log.txt"
# 支持 "set: 0, left: 0, right: -0.0012401"
pattern = re.compile(
    r"set:\s*(-?\d+\.\d+),\s*left:\s*(-?\d+\.\d+),\s*right:\s*(-?\d+\.\d+)"
)

# === 读取文件 ===
set_values = []
left_values = []
right_values = []

with open(LOG_FILE, "r") as f:
    for line in f:
        match = pattern.search(line)
        if match:
            set_values.append(float(match.group(1)))
            left_values.append(-float(match.group(2)))
            right_values.append(float(match.group(3)))

if not left_values:
    print("未在 fric_log.txt 中检测到任何数据，")
    exit()

# === 基本配置 ===
window_size = 100
max_index = len(left_values) - window_size
if max_index < 0:
    max_index = 0
x_all = np.arange(len(left_values))

# === 布局：两张图 + 滑块 ===
fig = plt.figure(figsize=(10, 7))
ax_overview = plt.subplot2grid((3, 1), (0, 0))   # 上方预览
ax_detail = plt.subplot2grid((3, 1), (1, 0), rowspan=2)  # 下方主图
plt.subplots_adjust(bottom=0.15, hspace=0.3)

# === 绘制上方预览图 ===
ax_overview.plot(x_all, set_values, label="set", color="tab:green", alpha=0.6)
ax_overview.plot(x_all, left_values, label="left(qufan)", color="tab:blue", alpha=0.6)
ax_overview.plot(x_all, right_values, label="right", color="tab:orange", alpha=0.6)
ax_overview.set_title("Overview of All Data")
ax_overview.legend()
ax_overview.grid(True)

# 在预览图上加一个矩形框表示当前窗口范围
rect = ax_overview.axvspan(0, window_size, color='gray', alpha=0.3)

# === 绘制下方细节图 ===
x_init = x_all[:window_size]
line_set, = ax_detail.plot(x_init, set_values[:window_size], label="set", color="tab:green")
line_left, = ax_detail.plot(x_init, left_values[:window_size], label="left(qufan)", color="tab:blue")
line_right, = ax_detail.plot(x_init, right_values[:window_size], label="right", color="tab:orange")

ax_detail.set_title("Zoomed Detail (use slider below)")
ax_detail.set_xlabel("Sample index")
ax_detail.set_ylabel("Value")
ax_detail.legend()
ax_detail.grid(True)

# === 滑块 ===
ax_slider = plt.axes([0.15, 0.05, 0.7, 0.03])
slider = Slider(ax_slider, "Position", 0, max_index, valinit=0, valstep=1)

# === 更新函数 ===
def update(val):
    start = int(slider.val)
    end = start + window_size
    if end > len(left_values):
        end = len(left_values)

    x_new = x_all[start:end]

    # 更新下方细节图
    line_set.set_data(x_new, set_values[start:end])
    line_left.set_data(x_new, left_values[start:end])
    line_right.set_data(x_new, right_values[start:end])

    ax_detail.set_xlim(start, end)
    ax_detail.relim()
    ax_detail.autoscale_view()

    # 更新上方矩形
    rect.set_xy([(start, 0), (start, 1), (end, 1), (end, 0)])
    rect.set_x(start)
    rect.set_width(end - start)

    fig.canvas.draw_idle()

slider.on_changed(update)

plt.show()
