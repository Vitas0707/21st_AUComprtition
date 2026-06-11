# =====================================================================
# 终极完美闭环版：ArrowClassifier.py
# =====================================================================
import ulab.numpy as np

def judge_arrow_direction(pl_img, box_coords):
    """
    专门针对 01Studio 竖屏 RGB 矩阵 (480x640) 进行坐标系自适应与像素计算
    """
    try:
        # 1. 强制坐标为标准整型
        x = int(box_coords[0])
        y = int(box_coords[1])
        w = int(box_coords[2])
        h = int(box_coords[3])
    except Exception as e:
        print("坐标解析失败:", e)
        return 'UNKNOWN'

    # 2. 动态捕获底层矩阵的真实形状 [高, 宽, 通道]
    try:
        shape = pl_img.shape
        img_h = shape[0] # 应该是 640
        img_w = shape[1] # 应该是 480
    except Exception as e:
        print("矩阵结构解析失败:", e)
        return 'UNKNOWN'

    # 3. 超强鲁棒性限幅保护：防止 YOLO 横屏坐标在竖屏矩阵上越界
    if x < 0: x = 0
    if y < 0: y = 0
    if x >= img_w: x = img_w - 5
    if y >= img_h: y = img_h - 5
    if x + w > img_w: w = img_w - x
    if y + h > img_h: h = img_h - y

    # 过滤无效尺寸
    if w <= 10 or h <= 10:
        return 'UNKNOWN'

    # 4. 纯数学矩阵切片与 RGB 降维对决
    try:
        # 找出宽度的中心分割点
        mid_x = x + (w // 2)

        # 核心修复：采用 [:, :] 语法切前两维（H轴和W轴），自动适配 R,G,B 三通道
        left_part = pl_img[y : y + h, x : mid_x]
        right_part = pl_img[y : y + h, mid_x : x + w]

        # ulab.numpy 的 mean 会自动将所有通道（三维）扁平化求出一个最终综合亮度均值
        left_mean = np.mean(left_part)
        right_mean = np.mean(right_part)

        # 5. 黑箭头白背景裁决逻辑
        # 画面中黑箭头部分更暗，值更低。大头在哪边，哪边均值就小。
        if left_mean < right_mean:
            return 'L'  # 左边更暗 -> 左箭头
        else:
            return 'R'  # 右边更暗 -> 右箭头

    except Exception as calc_err:
            import sys
            # 强行打印出最底层、最真实的报错堆栈信息
            sys.print_exception(calc_err)
            return 'UNKNOWN'
