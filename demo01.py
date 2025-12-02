import matplotlib.pyplot as plt
import numpy as np
import tkinter as tk
from tkinter import ttk, messagebox
import os
import threading

class ConeEditor:
    def __init__(self, cone_file, path_file):
        self.cone_file = cone_file
        self.path_file = path_file
        self.cones = []  # 存储格式：[(x, y, color, original_line_num), ...]
        self.path_xy = None
        self.fig, self.ax = None, None
        self.scatter = None
        self.annotations = []  # 存储所有序号注释
        self.dragged_index = None  # 当前拖动的锥桶索引
        self.annot_offset = (5, 5)  # 序号相对于锥桶的偏移（像素）
        self.root = None  # Tkinter主窗口
        
        # 初始化数据
        self.load_data()
        # 先创建可视化窗口（确保优先显示）
        self.create_visualization()
        # 在新线程中创建Tkinter控制窗口（避免阻塞）
        self.create_control_window_in_thread()
    
    def load_data(self):
        """加载锥桶和路径数据"""
        self.cones = self.read_cone_data()
        self.path_xy = self.read_path_data()
        
        if not self.cones:
            messagebox.showwarning("警告", "没有有效的锥桶数据可显示！")
    
    def read_cone_data(self):
        """读取锥桶数据文件"""
        cones = []
        if not os.path.exists(self.cone_file):
            messagebox.showerror("错误", f"锥桶文件 {self.cone_file} 不存在！")
            return cones
        
        with open(self.cone_file, 'r', encoding='utf-8') as f:
            for line_num, line in enumerate(f, start=1):
                parts = line.strip().split()
                if len(parts) < 3:
                    print(f"警告：第{line_num}行数据不完整，跳过")
                    continue
                
                try:
                    x = float(parts[0])
                    y = float(parts[1])
                    color_raw = parts[2].strip().lower()
                    color = 'blue' if color_raw == 'b' else 'red' if color_raw == 'r' else 'gray'
                    cones.append((x, y, color, line_num))
                except ValueError:
                    print(f"警告：第{line_num}行数据格式错误，跳过")
                    continue
        return cones
    
    def read_path_data(self):
        """读取路径数据文件"""
        path_data = []
        if not os.path.exists(self.path_file):
            messagebox.showwarning("警告", f"路径文件 {self.path_file} 不存在！")
            return None
        
        with open(self.path_file, 'r', encoding='utf-8') as f:
            for line_num, line in enumerate(f, start=1):
                parts = line.strip().split()
                if len(parts) < 2:
                    print(f"警告：路径文件第{line_num}行数据不完整，跳过")
                    continue
                
                try:
                    x = float(parts[0])
                    y = float(parts[1])
                    path_data.append([x, y])
                except ValueError:
                    print(f"警告：路径文件第{line_num}行数据格式错误，跳过")
                    continue
        
        return np.array(path_data) if path_data else None
    
    def update_cone_file(self):
        """更新锥桶数据文件（支持添加/删除/修改）"""
        # 读取原始文件内容
        with open(self.cone_file, 'r', encoding='utf-8') as f:
            lines = f.readlines()
        
        # 情况1：如果是删除操作，需要重新构建所有行
        if len(self.cones) < len(lines):
            new_lines = []
            for x, y, color, _ in self.cones:
                color_raw = 'b' if color == 'blue' else 'r' if color == 'red' else ''
                new_lines.append(f"{x:.6f} {y:.6f} {color_raw}\n")
            lines = new_lines
        # 情况2：修改或添加操作
        else:
            for idx, (x, y, color, original_line_num) in enumerate(self.cones):
                color_raw = 'b' if color == 'blue' else 'r' if color == 'red' else ''
                # 如果是新增的点（原始行号超过原有行数）
                if original_line_num > len(lines):
                    lines.append(f"{x:.6f} {y:.6f} {color_raw}\n")
                else:
                    # 更新原有行
                    parts = lines[original_line_num-1].strip().split()
                    if len(parts) >= 3:
                        parts[0] = f"{x:.6f}"
                        parts[1] = f"{y:.6f}"
                        parts[2] = color_raw
                        lines[original_line_num-1] = ' '.join(parts) + '\n'
                    else:
                        lines[original_line_num-1] = f"{x:.6f} {y:.6f} {color_raw}\n"
        
        # 写回文件
        with open(self.cone_file, 'w', encoding='utf-8') as f:
            f.writelines(lines)
        
        # 重新加载数据（更新序号）
        self.load_data()
        # 重新绘制
        self.redraw_visualization()
    
    def create_visualization(self):
        """创建可视化界面"""
        self.fig, self.ax = plt.subplots(figsize=(20, 16))
        self.ax.set_xlabel('X Coordinate (m)', fontsize=12)
        self.ax.set_ylabel('Y Coordinate (m)', fontsize=12)
        self.ax.set_title('Racing Track Cones and Path Visualization', fontsize=14, weight='bold')
        self.ax.axis('equal')
        
        # 设置网格（2单位间隔）
        self.set_grid_interval()
        self.fig.canvas.mpl_connect('draw_event', lambda e: self.set_grid_interval())
        
        # 绘制初始数据
        self.redraw_visualization()
        
        # 绑定鼠标事件
        self.fig.canvas.mpl_connect('pick_event', self.on_pick)
        self.fig.canvas.mpl_connect('motion_notify_event', self.on_motion)
        self.fig.canvas.mpl_connect('button_release_event', self.on_release)
        
        # 绑定matplotlib空闲事件，保持Tkinter窗口响应
        self.fig.canvas.mpl_connect('idle_event', self.update_tkinter)
    
    def redraw_visualization(self):
        """重新绘制所有元素（支持动态更新）"""
        # 清除原有内容
        self.ax.clear()
        self.annotations.clear()
        
        if not self.cones:
            return
        
        # 提取锥桶数据
        cone_coords = np.array([(x, y) for x, y, _, _ in self.cones])
        cone_colors = [color for _, _, color, _ in self.cones]
        cone_nums = [idx+1 for idx in range(len(self.cones))]  # 序号重新编排（1开始）
        
        # 绘制锥桶（圆形，小半径）
        self.scatter = self.ax.scatter(
            cone_coords[:, 0], cone_coords[:, 1],
            color=cone_colors, marker='o', s=30,
            edgecolors='black', linewidth=0.5,
            picker=5  # 拾取半径
        )
        
        # 绘制序号（关键修复：使用annotate的xy参数绑定锥桶位置）
        for i, (x, y, num) in enumerate(zip(cone_coords[:, 0], cone_coords[:, 1], cone_nums)):
            annot = self.ax.annotate(
                str(num),
                xy=(x, y),  # 绑定到锥桶的实际坐标
                xytext=self.annot_offset,
                textcoords='offset points',
                fontsize=8,
                color='black',
                weight='bold',
                bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.9),
                zorder=10
            )
            self.annotations.append(annot)
        
        # 绘制路径
        if self.path_xy is not None and len(self.path_xy) > 1:
            self.ax.plot(self.path_xy[:, 0], self.path_xy[:, 1], color='green', linewidth=2, label='Path', alpha=0.8)
            self.ax.scatter(self.path_xy[0, 0], self.path_xy[0, 1], color='darkgreen', marker='o', s=150, label='Start', zorder=5)
            self.ax.scatter(self.path_xy[-1, 0], self.path_xy[-1, 1], color='orange', marker='s', s=150, label='End', zorder=5)
        
        # 重置图形属性
        self.ax.legend(fontsize=10)
        self.set_grid_interval()
        self.fig.canvas.draw_idle()
    
    def set_grid_interval(self):
        """设置2单位间隔的网格"""
        xlim = self.ax.get_xlim()
        ylim = self.ax.get_ylim()
        
        x_start = np.floor(xlim[0] / 2) * 2
        y_start = np.floor(ylim[0] / 2) * 2
        
        self.ax.set_xticks(np.arange(x_start, xlim[1] + 2, 2))
        self.ax.set_yticks(np.arange(y_start, ylim[1] + 2, 2))
        self.ax.grid(True, alpha=0.3, linestyle='-', linewidth=0.5)
    
    # ---------------------- 拖动事件处理 ----------------------
    def on_pick(self, event):
        """拾取锥桶点"""
        if event.artist == self.scatter:
            ind = event.ind
            if len(ind) > 0:
                self.dragged_index = ind[0]
                # 高亮拖动的序号
                self.annotations[self.dragged_index].set_bbox(dict(boxstyle='round,pad=0.2', facecolor='yellow', alpha=0.9))
                self.fig.canvas.draw_idle()
    
    def on_motion(self, event):
        """拖动过程中更新位置"""
        if self.dragged_index is not None and event.inaxes == self.ax:
            x, y = event.xdata, event.ydata
            
            # 更新内存中的锥桶位置
            self.cones[self.dragged_index] = (
                x, y, 
                self.cones[self.dragged_index][2],
                self.cones[self.dragged_index][3]
            )
            
            # 实时更新散点位置
            cone_coords = np.array([(x, y) for x, y, _, _ in self.cones])
            self.scatter.set_offsets(cone_coords)
            
            # 实时更新序号位置（彻底修复：直接修改注释的xy坐标）
            self.annotations[self.dragged_index].set_xydata((x, y))
            
            self.fig.canvas.draw_idle()
    
    def on_release(self, event):
        """释放鼠标时保存更新"""
        if self.dragged_index is not None:
            cone_num = self.dragged_index + 1
            x, y = self.cones[self.dragged_index][0], self.cones[self.dragged_index][1]
            print(f"锥桶 {cone_num} 已更新位置: ({x:.6f}, {y:.6f})")
            
            # 恢复序号背景色
            self.annotations[self.dragged_index].set_bbox(dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.9))
            
            # 保存到文件
            self.update_cone_file()
            self.dragged_index = None
    
    # ---------------------- Tkinter控制窗口 ----------------------
    def create_control_window_in_thread(self):
        """在新线程中创建Tkinter窗口，避免阻塞matplotlib"""
        thread = threading.Thread(target=self.create_control_window)
        thread.daemon = True  # 守护线程，主程序退出时自动关闭
        thread.start()
    
    def create_control_window(self):
        """创建控制按钮窗口"""
        self.root = tk.Tk()
        self.root.title("锥桶编辑工具")
        self.root.geometry("300x150")
        self.root.attributes('-topmost', True)  # 控制窗口置顶，方便操作
        
        # 添加锥桶按钮
        add_btn = ttk.Button(self.root, text="添加锥桶点", command=self.show_add_window)
        add_btn.pack(pady=10, fill=tk.X, padx=20)
        
        # 删除锥桶按钮
        delete_btn = ttk.Button(self.root, text="删除锥桶点", command=self.show_delete_window)
        delete_btn.pack(pady=5, fill=tk.X, padx=20)
        
        # Tkinter主循环（在子线程中运行）
        self.root.mainloop()
    
    def update_tkinter(self, event):
        """保持Tkinter窗口响应"""
        if self.root:
            self.root.update_idletasks()
    
    def show_add_window(self):
        """显示添加锥桶窗口"""
        add_win = tk.Toplevel(self.root)
        add_win.title("添加锥桶点")
        add_win.geometry("300x250")
        add_win.attributes('-topmost', True)
        
        # 标签和输入框
        ttk.Label(add_win, text="输入两个锥桶序号（用中点创建新点）").pack(pady=10)
        
        ttk.Label(add_win, text="锥桶序号1：").pack(padx=20, anchor=tk.W)
        entry1 = ttk.Entry(add_win)
        entry1.pack(fill=tk.X, padx=20, pady=5)
        
        ttk.Label(add_win, text="锥桶序号2：").pack(padx=20, anchor=tk.W)
        entry2 = ttk.Entry(add_win)
        entry2.pack(fill=tk.X, padx=20, pady=5)
        
        # 确认按钮
        def confirm_add():
            try:
                num1 = int(entry1.get().strip())
                num2 = int(entry2.get().strip())
                
                # 验证序号有效性
                if num1 < 1 or num2 < 1 or num1 > len(self.cones) or num2 > len(self.cones):
                    messagebox.showerror("错误", f"序号必须在1-{len(self.cones)}之间！")
                    return
                
                if num1 == num2:
                    messagebox.showerror("错误", "两个序号不能相同！")
                    return
                
                # 获取两个点的坐标
                cone1 = self.cones[num1-1]
                cone2 = self.cones[num2-1]
                
                # 计算中点
                mid_x = (cone1[0] + cone2[0]) / 2
                mid_y = (cone1[1] + cone2[1]) / 2
                
                # 新点颜色默认与第一个点相同
                new_color = cone1[2]
                
                # 添加到内存（原始行号设为当前行数+1）
                new_original_line_num = len(self.cones) + 1
                self.cones.append((mid_x, mid_y, new_color, new_original_line_num))
                
                # 保存并更新
                self.update_cone_file()
                messagebox.showinfo("成功", f"已添加新锥桶点（序号{len(self.cones)}）：({mid_x:.6f}, {mid_y:.6f})")
                add_win.destroy()
                
            except ValueError:
                messagebox.showerror("错误", "请输入有效的整数序号！")
        
        ttk.Button(add_win, text="确认添加", command=confirm_add).pack(pady=15)
    
    def show_delete_window(self):
        """显示删除锥桶窗口"""
        delete_win = tk.Toplevel(self.root)
        delete_win.title("删除锥桶点")
        delete_win.geometry("300x250")
        delete_win.attributes('-topmost', True)
        
        # 标签和输入框
        ttk.Label(delete_win, text="输入要删除的锥桶序号").pack(pady=10)
        
        ttk.Label(delete_win, text="锥桶序号：").pack(padx=20, anchor=tk.W)
        entry = ttk.Entry(delete_win)
        entry.pack(fill=tk.X, padx=20, pady=5)
        
        # 确认按钮
        def confirm_delete():
            try:
                num = int(entry.get().strip())
                
                # 验证序号有效性
                if num < 1 or num > len(self.cones):
                    messagebox.showerror("错误", f"序号必须在1-{len(self.cones)}之间！")
                    return
                
                # 确认删除
                if not messagebox.askyesno("确认", f"确定要删除锥桶序号{num}吗？"):
                    return
                
                # 从内存中删除
                deleted_cone = self.cones.pop(num-1)
                print(f"已删除锥桶 {num}：({deleted_cone[0]:.6f}, {deleted_cone[1]:.6f})")
                
                # 保存并更新
                self.update_cone_file()
                messagebox.showinfo("成功", f"已删除锥桶序号{num}")
                delete_win.destroy()
                
            except ValueError:
                messagebox.showerror("错误", "请输入有效的整数序号！")
        
        ttk.Button(delete_win, text="确认删除", command=confirm_delete).pack(pady=15)

def main():
    # 配置文件路径（请根据实际情况修改）
    CONE_FILE_PATH = "cone_map.txt"
    PATH_FILE_PATH = "center_map.txt"
    
    # 检查文件是否存在，不存在则创建空文件
    for file_path in [CONE_FILE_PATH, PATH_FILE_PATH]:
        if not os.path.exists(file_path):
            with open(file_path, 'w', encoding='utf-8') as f:
                f.write("")
            print(f"已创建空文件：{file_path}")
    
    # 启动编辑器
    editor = ConeEditor(CONE_FILE_PATH, PATH_FILE_PATH)
    # 显示matplotlib窗口（非阻塞）
    plt.show(block=True)

if __name__ == "__main__":
    main()