import re
import matplotlib.pyplot as plt
import numpy as np
import time

# === 配置部分 ===
# 更改：使用新的日志文件 trigger.txt
LOG_FILE = "../log/trigger_log.txt"

# 更改：新的正则表达式，只匹配 set 和 trigger 数据
# 假设 trigger.txt 中的数据格式类似: "set: 0.1, trigger: 100"
# 或者 "set: -0.05, trigger: -20"
# 注意: 如果您的 trigger.txt 格式与此假设不符，请告诉我，我会调整正则表达式。
pattern = re.compile(
    r"set:\s*(-?\d+\.?\d*),\s*trigger:\s*(-?\d+\.?\d*)" # 兼容整数和小数
)

# === 读取文件 ===
set_values = []
trigger_values = [] # 更改：只保留 set 和 trigger

try:
    with open(LOG_FILE, "r") as f:
        for line in f:
            match = pattern.search(line)
            if match:
                # 更改：只读取 set 和 trigger
                set_values.append(float(match.group(1)))
                trigger_values.append(float(match.group(2)))

except FileNotFoundError:
    print(f"错误：未找到日志文件 {LOG_FILE}")
    exit()


if not set_values: # 检查任意一个列表即可
    print(f"未在日志文件 {LOG_FILE} 中检测到任何有效数据。")
    exit()

# === 基本配置 ===
x_all = np.arange(len(set_values))

# === 布局：一张图 (Overview) ===
# 更改：只创建一个 subplot
fig = plt.figure(figsize=(10, 5)) 
ax_overview = fig.add_subplot(1, 1, 1) # 1行1列第1个

# === 绘制 Overview 图 (全数据) ===
ax_overview.plot(x_all, set_values, label="set", color="tab:green", alpha=0.9)
# 更改：绘制 trigger 数据
ax_overview.plot(x_all, trigger_values, label="trigger", color="tab:red", alpha=0.9)

# 更改：更新标题和标签
ax_overview.set_title(f"Overview of Set and Trigger Data ({LOG_FILE})")
ax_overview.set_xlabel("Sample index")
ax_overview.set_ylabel("Value")
ax_overview.legend(loc='upper right')
ax_overview.grid(True)


# =======================================================
# 🚀 自动保存图表
# =======================================================

# 使用时间戳作为文件名，确保每次运行不会覆盖上次的结果
timestamp = time.strftime("%Y%m%d_%H%M%S")
# 更改：更新文件名
filename = f"set_trigger_plot_{timestamp}.png" 

# 保存图表。bbox_inches='tight' 确保所有标签和标题都被包含
fig.savefig(filename, bbox_inches='tight', dpi=300)

print(f"\n✅ 图表已成功保存为本地文件: {filename}")

# 移除 plt.show()，程序到此结束