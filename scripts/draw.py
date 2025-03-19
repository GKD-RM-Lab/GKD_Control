#!/sbin/python3
import matplotlib.pyplot as plt

# 初始化存储数据的列表
y_values_1 = []
y_values_2 = []

# 读取文件
file_path = 'log8.txt'  # 请修改为实际文件路径
with open(file_path, 'r') as file:
    for line_number, line in enumerate(file, start=1):
        # 跳过空行
        line = line.strip()
        if not line:
            continue
        
        # 解析两个浮点数
        try:
            val1, val2 = map(float, line.split(','))
            y_values_1.append(val1)
            y_values_2.append(val2)
        except ValueError:
            print(f"跳过格式错误的行: 第{line_number}行")
            continue

# 创建x轴数据（行号从1开始）
x_values = list(range(1, len(y_values_1) + 1))

# 绘制图形
plt.figure(figsize=(10, 6))

# 绘制第一个值的折线图
plt.plot(x_values, y_values_1, 
         label='第一个值', 
         marker='o', 
         linestyle='-',
         color='blue')

# 绘制第二个值的折线图
plt.plot(x_values, y_values_2, 
         label='第二个值', 
         marker='s', 
         linestyle='--',
         color='orange')

# 添加图表元素
plt.title('数据可视化')
plt.xlabel('行号（X轴）')
plt.ylabel('数值（Y轴）')
plt.legend()
plt.grid(True, linestyle='--', alpha=0.7)
plt.tight_layout()

# 显示图表
plt.show()
