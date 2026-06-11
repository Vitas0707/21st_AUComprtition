# Untitled - By: 13840 - Fri May 15 2026

# ============================
# grid_position.py
# 网格定位模块
# ============================

import math

# ============================
# 全局位置状态
# ============================

coord_x = 0
coord_y = 0

# 上一帧中心
prev_center = None

# 最近中心历史（抗抖）
center_history = []

# 网格间距估计
avg_grid_dist = 120

# 移动确认机制
pending_move = None
pending_count = 0
CONFIRM_FRAMES = 3

# 历史记录
move_history = []

# ============================
# 初始化
# ============================

def reset_position():
    global coord_x
    global coord_y
    global prev_center
    global center_history
    global avg_grid_dist
    global pending_move
    global pending_count
    global move_history

    coord_x = 0
    coord_y = 0

    prev_center = None
    center_history = []

    avg_grid_dist = 120

    pending_move = None
    pending_count = 0

    move_history = []

    print("[GRID] Reset Position -> (0,0)")

# ============================
# 平滑中心

def smooth_center(center):
    global center_history

    center_history.append(center)

    if len(center_history) > 5:
        center_history.pop(0)

    sx = 0
    sy = 0

    for c in center_history:
        sx += c[0]
        sy += c[1]

    sx = sx // len(center_history)
    sy = sy // len(center_history)

    return (sx, sy)

# ============================
# 计算距离
# ============================

def calc_distance(p1, p2):
    dx = p1[0] - p2[0]
    dy = p1[1] - p2[1]

    return math.sqrt(dx*dx + dy*dy)

# ============================
# 更新网格间距
# ============================

def update_grid_distance(blobs):
    global avg_grid_dist

    if len(blobs) < 2:
        return

    min_dist = 99999

    for i in range(len(blobs)):
        for j in range(i+1, len(blobs)):

            c1 = blobs[i]['center']
            c2 = blobs[j]['center']

            d = calc_distance(c1, c2)

            if d < min_dist:
                min_dist = d

    if min_dist > 20:
        avg_grid_dist = 0.7 * avg_grid_dist + 0.3 * min_dist

# ============================
# 找最近格子
    for b in blobs:

        d = calc_distance(b['center'], image_center)

        if d < best_dist:
            best_dist = d
            best = b

    return best

# ============================
# 核心定位函数
# ============================

def update_position(blobs, image_center):

    global coord_x
    global coord_y

    global prev_center

    global pending_move
    global pending_count

    global move_history

    # 没找到格子
    if not blobs:
        return None, "LOST"

    # 更新网格距离
    update_grid_distance(blobs)

    # 找主格子
    main = find_main_blob(blobs, image_center)

    if main is None:
        return None, "NO_MAIN"

    # 平滑中心
    current_center = smooth_center(main['center'])

    # 初始化
    if prev_center is None:

        prev_center = current_center

        return main, "INIT"

    # ============================
    # 位移计算
        # ============================

        dx = current_center[0] - prev_center[0]
        dy = current_center[1] - prev_center[1]

        dist = math.sqrt(dx*dx + dy*dy)

        # ============================
        # 没移动
        # ============================

        if dist < avg_grid_dist * 0.4:

            prev_center = current_center

            pending_move = None
            pending_count = 0

            return main, "SAME"

        # ============================
        # 正常移动
        # ============================

        if dist < avg_grid_dist * 1.5:

            move_x = 0
            move_y = 0

            # 横向为主
            if abs(dx) > abs(dy):

                if dx > 0:
                    move_x = -1
                else:
                    move_x = 1

            # 纵向为主
            else:

                if dy > 0:
                    move_y = 1
                else:
                    move_y = -1

            # ============================
            # 抗误判确认
                    # ============================

                    current_move = (move_x, move_y)

                    if pending_move == current_move:

                        pending_count += 1

                        # 确认移动成功
                        if pending_count >= CONFIRM_FRAMES:

                            old_x = coord_x
                            old_y = coord_y

                            coord_x += move_x
                            coord_y += move_y

                            move_history.append(
                                "(%d,%d)->(%d,%d)" % (
                                    old_x,
                                    old_y,
                                    coord_x,
                                    coord_y
                                )
                            )

                            if len(move_history) > 10:
                                move_history.pop(0)

                            prev_center = current_center

                            pending_move = None
                            pending_count = 0

                            print("[GRID] MOVE -> (%d,%d)" % (coord_x, coord_y))

                            return main, "MOVED"

                    else:

                        pending_move = current_move
                        pending_count = 1

                    return main, "PENDING"

                # ============================
                # 移动太大（可能误检）
                # ============================

                return main, "FAR"

            # ============================
            # 获取当前位置
            # ============================

            def get_position():
                return (coord_x, coord_y)

            # ============================
            # 获取历史
            # ============================

            def get_history():
                return move_history
