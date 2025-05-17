#!/usr/bin/python3
import pandas as pd
import matplotlib.pyplot as plt

# 读取数据
data = pd.read_csv('filtered_data.csv')

# 绘制图像
plt.figure(figsize=(10, 6))
plt.plot(data['time'], data['raw_x'], label='Raw X', color='blue', alpha=0.5)
plt.plot(data['time'], data['filtered_x'], label='Filtered X', color='red', linewidth=2)
plt.xlabel('Time (s)')
plt.ylabel('X Value')
plt.title('Raw X vs Filtered X over Time')
plt.legend()
plt.grid(True)
plt.show()