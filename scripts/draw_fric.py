import re
import matplotlib.pyplot as plt
import numpy as np
import time

# === 配置部分 ===
LOG_FILE = "../log/fric_log.txt"
# 支持 "set: 0.0, left: 0.0, right: -0.0012401"
pattern = re.compile(
    r"set:\s*(-?\d+\.\d+),\s*left:\s*(-?\d+\.\d+),\s*right:\s*(-?\d+\.\d+)"
)

# === 读取文件 ===
set_values = []
left_values = []
right_values = []

try:
    with open(LOG_FILE, "r") as f:
        for line in f:
            match = pattern.search(line)
            if match:
                # 注意: 这里的读取逻辑保持不变，确保数据正确
                set_values.append(float(match.group(1)))
                left_values.append(-float(match.group(2))) # 保持反转操作
                right_values.append(float(match.group(3)))

except FileNotFoundError:
    print(f"错误：未找到日志文件 {LOG_FILE}")
    exit()


if not left_values:
    print("未在日志文件中检测到任何有效数据，")
    exit()

# === 基本配置 ===
# 虽然不再是交互式，但保留 window_size 的概念可以用于初始展示
window_size = 100 
x_all = np.arange(len(left_values))

# === 布局：两张图（移除滑块/按钮空间） ===
fig = plt.figure(figsize=(10, 7))
ax_overview = plt.subplot2grid((2, 1), (0, 0))   # 上方预览 (调整为 2x1 网格)
ax_detail = plt.subplot2grid((2, 1), (1, 0))  # 下方主图
plt.subplots_adjust(hspace=0.3) # 保持紧凑

# === 绘制上方预览图 (全数据) ===
ax_overview.plot(x_all, set_values, label="set", color="tab:green", alpha=0.6)
ax_overview.plot(x_all, left_values, label="left(qufan)", color="tab:blue", alpha=0.6)
ax_overview.plot(x_all, right_values, label="right", color="tab:orange", alpha=0.6)
ax_overview.set_title("Overview of All Data")
ax_overview.legend(loc='upper right')
ax_overview.grid(True)


# === 绘制下方细节图 (仅展示前 N 个数据点) ===
x_init = x_all[:window_size]
ax_detail.plot(x_init, set_values[:window_size], label="set", color="tab:green")
ax_detail.plot(x_init, left_values[:window_size], label="left(qufan)", color="tab:blue")
ax_detail.plot(x_init, right_values[:window_size], label="right", color="tab:orange")

ax_detail.set_title(f"Detail of First {window_size} Samples")
ax_detail.set_xlabel("Sample index")
ax_detail.set_ylabel("Value")
ax_detail.legend(loc='upper right')
ax_detail.grid(True)


# =======================================================
# 🚀 新增：自动保存图表
# =======================================================

# 使用时间戳作为文件名，确保每次运行不会覆盖上次的结果
timestamp = time.strftime("%Y%m%d_%H%M%S")
filename = f"fric_plot_{timestamp}.png"

# 保存图表。bbox_inches='tight' 确保所有标签和标题都被包含
fig.savefig(filename, bbox_inches='tight', dpi=300)

print(f"\n✅ 图表已成功保存为本地文件: {filename}")

# 移除 plt.show()，程序到此结束