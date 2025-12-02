def read_frenet_data(file_path):
    """
    读取Frenet cones文件数据
    :param file_path: 文件路径
    :return: 列表形式的数据（每行元素：[序号(int), 第2列(float), 第3列(float), 第4列(float), 第5列(float)]）
    """
    data = []
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            for line in f:
                # 去除空行和首尾空格
                line = line.strip()
                if not line:
                    continue
                # 按空格分割列（支持多个空格分隔）
                parts = line.split()
                # 转换数据类型：第1列是序号（int），其余为数值（float）
                row = [int(parts[0])] + [float(p) for p in parts[1:]]
                data.append(row)
        return data
    except FileNotFoundError:
        print(f"错误：未找到文件 {file_path}，请检查路径是否正确！")
        exit()


def write_frenet_data(file_path, data):
    """
    将处理后的数据写入文件（自动重编序号）
    :param file_path: 目标文件路径
    :param data: 待写入的数据（每行包含除序号外的其他列数据）
    """
    with open(file_path, 'w', encoding='utf-8') as f:
        # 序号从1开始递增，数值保留4位小数（与原文件格式一致）
        for new_idx, row in enumerate(data, start=1):
            # 拼接行：新序号 + 原数据第2-5列（格式化为4位小数）
            line = f"{new_idx} " + " ".join([f"{x:.4f}" for x in row[1:]]) + "\n"
            f.write(line)


def main():
    # -------------------------- 1. 配置文件路径（请根据你的实际路径修改！） --------------------------
    left_file = "left_frenet_cones.txt"   # left文件路径
    right_file = "right_frenet_cones.txt" # right文件路径

    # -------------------------- 2. 读取原始数据 --------------------------
    print("正在读取原始文件...")
    left_data = read_frenet_data(left_file)
    right_data = read_frenet_data(right_file)
    print(f"原始Left文件：{len(left_data)}行")
    print(f"原始Right文件：{len(right_data)}行")

    # -------------------------- 3. 筛选需要移动的行 --------------------------
    # Left文件：筛选第三列<0的行（要移到Right），保留第三列>=0的行
    left_keep = [row for row in left_data if row[2] >= 0.0]  # Left保留行（第三列≥0）
    left_move = [row for row in left_data if row[2] < 0.0]   # Left移动行（第三列<0）

    # Right文件：筛选第三列>0的行（要移到Left），保留第三列<=0的行
    right_keep = [row for row in right_data if row[2] <= 0.0] # Right保留行（第三列≤0）
    right_move = [row for row in right_data if row[2] > 0.0]  # Right移动行（第三列>0）

    # -------------------------- 4. 合并新数据（保留行 + 从对方移动过来的行） --------------------------
    new_left = left_keep + right_move  # 新Left = Left保留行 + Right移动行
    new_right = right_keep + left_move # 新Right = Right保留行 + Left移动行

    # -------------------------- 5. 写入新文件（自动重编序号） --------------------------
    print("\n正在写入处理后的文件...")
    write_frenet_data(left_file, new_left)
    write_frenet_data(right_file, new_right)

    # -------------------------- 6. 输出处理结果 --------------------------
    print("\n处理完成！")
    print(f"\nLeft文件：")
    print(f"  - 保留行数：{len(left_keep)}")
    print(f"  - 从Right移入行数：{len(right_move)}")
    print(f"  - 最终总行数：{len(new_left)}")
    print(f"\nRight文件：")
    print(f"  - 保留行数：{len(right_keep)}")
    print(f"  - 从Left移入行数：{len(left_move)}")
    print(f"  - 最终总行数：{len(new_right)}")


if __name__ == "__main__":
    main()