import matplotlib.pyplot as plt

# -------------------------- 1. 数据读取与解析 --------------------------
# 初始化存储数据的列表（序号、X坐标、Y坐标、颜色）
cone_ids = []    # 第二列：锥桶序号
cone_x = []      # 第三列：X坐标
cone_y = []      # 第四列：Y坐标
cone_colors = [] # 第五列：颜色（r→红色，b→蓝色）

# 读取txt文件（需确保脚本与txt文件在同一文件夹，或修改为完整文件路径）
with open("auto_color_map.txt", "r", encoding="utf-8") as f:
    for line in f:
        # 跳过空行，分割每行数据（按空格分割，忽略多余空格）
        parts = line.strip().split()
        if len(parts) != 5:  # 确保每行数据格式正确（5个字段）
            continue
        
        # 提取目标列数据并转换格式
        cone_ids.append(int(parts[1]))          # 序号：转换为整数
        cone_x.append(float(parts[2]))          # X坐标：转换为浮点数
        cone_y.append(float(parts[3]))          # Y坐标：转换为浮点数
        # 颜色映射（matplotlib支持'r'/'b'缩写，直接使用）
        cone_colors.append('red' if parts[4] == 'r' else 'blue')

# -------------------------- 2. 绘图配置与可视化 --------------------------
# 创建画布（设置尺寸，单位为英寸）
plt.figure(figsize=(8, 16))

# 绘制锥桶点（圆点样式）
# s=80：点的大小，alpha=0.8：透明度（避免重叠遮挡），edgecolors：点的边框色（增强清晰度）
scatter = plt.scatter(
    x=cone_x, 
    y=cone_y, 
    c=cone_colors, 
    marker='o',  # 圆点样式
    s=80, 
    alpha=0.8, 
    edgecolors='black', 
    linewidth=0.5
)

# 为每个点添加序号标注（在点的右侧上方偏移，避免遮挡）
for idx in range(len(cone_ids)):
    plt.annotate(
        str(cone_ids[idx]),  # 要显示的序号（转换为字符串）
        xy=(cone_x[idx], cone_y[idx]),  # 点的坐标
        xytext=(5, 5),  # 标注文字相对于点的偏移量（右5，上5）
        textcoords='offset points',  # 偏移量基于点的像素
        fontsize=8,     # 序号字体大小
        color='black',  # 序号颜色（黑色清晰易读）
        weight='bold'   # 序号加粗
    )

# -------------------------- 3. 图表美化与显示 --------------------------
# 设置标题和坐标轴标签
plt.title('锥桶点分布可视化', fontsize=16, fontweight='bold', pad=20)
plt.xlabel('X 坐标', fontsize=12, fontweight='bold')
plt.ylabel('Y 坐标', fontsize=12, fontweight='bold')

# 添加网格（辅助读取坐标，alpha=0.3：半透明不遮挡数据）
plt.grid(True, alpha=0.3, linestyle='--')

# 调整布局（避免标签被截断）
plt.tight_layout()

# 显示图表（若需保存图片，可取消下方注释，图片会保存为当前文件夹下的png文件）
# plt.savefig('cone_visualization.png', dpi=300, bbox_inches='tight')
plt.show()