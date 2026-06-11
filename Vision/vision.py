'''
实验名称：摄像头使用
实验平台：01Studio CanMV K230
说明：实现摄像头图像采集显示
'''

'''
camera.py

功能：
1. 摄像头初始化
2. 左右ROI巡线
3. 中线计算
4. 偏移量输出
5. 状态判断
6. 图像显示

适用于：
K230
1920x1080
灰度巡线
'''

import time

from media.sensor import *
from media.display import *
from media.media import *

# =====================================================
# 全局变量
# =====================================================

sensor = None

clock = None

state = ""

center_history = []

# =====================================================
# 阈值
# =====================================================

threshold = (0, 60)

# =====================================================
# ROI区域
# =====================================================

# 左侧导航ROI
left_roi = (0, 600, 300, 480)

# 右侧导航ROI
right_roi = (1620, 600, 300, 480)

# 中间AI识别ROI
center_roi = (400, 300, 1120, 700)

# =====================================================
# 初始化摄像头
# =====================================================

def init_camera():

    global sensor
    global clock

    sensor = Sensor()

    sensor.reset()

    sensor.set_framesize(Sensor.FHD)

    sensor.set_pixformat(Sensor.RGB565)

    Display.init(
        Display.VIRT,
        sensor.width(),
        sensor.height(),
        to_ide=True
    )

    MediaManager.init()

    sensor.run()

    clock = time.clock()

    print("[CAMERA] INIT SUCCESS")

# =====================================================
# 获取一帧图像
# =====================================================

def capture_frame():

    global sensor
    global clock

    clock.tick()

    img = sensor.snapshot()

    # 转灰度
    img = img.to_grayscale()

    return img

# =====================================================
# 绘制ROI区域
# =====================================================

def draw_rois(img):

    # ROI框
    img.draw_rectangle(left_roi, color=0)
    img.draw_rectangle(right_roi, color=0)

    # ROI文字
    img.draw_string(30, 30, "L", color=0, scale=3)
    img.draw_string(1700, 30, "R", color=0, scale=3)
    img.draw_string(900, 30, "C", color=0, scale=3)

# =====================================================
# 检测左右黑线
# =====================================================

def find_lane_blobs(img):

    # 左侧
    left_blobs = img.find_blobs(
        [threshold],
        roi=left_roi,
        pixels_threshold=200,
        area_threshold=200,
        merge=True
    )

    # 右侧
    right_blobs = img.find_blobs(
        [threshold],
        roi=right_roi,
        pixels_threshold=200,
        area_threshold=200,
        merge=True
    )

    # 必须左右都有
    if not left_blobs or not right_blobs:

        return None, None

    # 最大blob
    left_blob = max(left_blobs, key=lambda b: b.pixels())

    right_blob = max(right_blobs, key=lambda b: b.pixels())

    return left_blob, right_blob

# =====================================================
# 绘制blob
# =====================================================

def draw_blobs(img, left_blob, right_blob):

    # 左
    img.draw_rectangle(left_blob.rect(), color=255)

    img.draw_cross(left_blob.cx(), left_blob.cy(), color=255)

    # 右
    img.draw_rectangle(right_blob.rect(), color=255)

    img.draw_cross(right_blob.cx(), right_blob.cy(), color=255)

# =====================================================
# 平滑中线
# =====================================================

def smooth_road_center(road_center):

    global center_history

    center_history.append(road_center)

    # 最多保存5帧
    if len(center_history) > 5:

        center_history.pop(0)

    smooth_center = sum(center_history) // len(center_history)

    return smooth_center

# =====================================================
# 计算偏移量
# =====================================================

def calc_offset(img, left_blob, right_blob):

    # 道路中心
    road_center = (left_blob.cx() + right_blob.cx()) // 2

    # 平滑
    road_center = smooth_road_center(road_center)

    # 图像中心
    image_center = img.width() // 2

    # 偏移量
    offset = road_center - image_center

    return offset, road_center, image_center

# =====================================================
# 绘制中心线
# =====================================================

def draw_center_lines(img, road_center, image_center):

    # 道路中心线
    img.draw_line(
        road_center,
        0,
        road_center,
        img.height(),
        color=255
    )

    # 图像中心线
    img.draw_line(
        image_center,
        0,
        image_center,
        img.height(),
        color=128
    )

# =====================================================
# 判断导航状态
# =====================================================

def get_navigation_state(offset):

    global state

    new_state = ""

    if offset < -50:

        new_state = "左转"

    elif offset > 50:

        new_state = "右转"

    else:

        new_state = "直行"

    # 状态变化才打印
    if new_state != state:

        print(new_state)

        state = new_state

    return state

# =====================================================
# 显示图像
# =====================================================

def show_image(img):

    Display.show_image(img)

# =====================================================
# 获取FPS
# =====================================================

def get_fps():

    global clock

    return clock.fps()

# =====================================================
# 核心处理函数
# 每调用一次：
# 处理一帧
# =====================================================

def process_frame():

    # 获取图像
    img = capture_frame()

    # 画ROI
    draw_rois(img)

    # 找blob
    left_blob, right_blob = find_lane_blobs(img)

    # 没找到
    if left_blob is None:

        show_image(img)

        return None

    # 绘制blob
    draw_blobs(img, left_blob, right_blob)

    # 计算偏移
    offset, road_center, image_center = calc_offset(
        img,
        left_blob,
        right_blob
    )

    # 画中线
    draw_center_lines(
        img,
        road_center,
        image_center
    )

    # 导航状态
    nav_state = get_navigation_state(offset)

    # 显示图像
    show_image(img)

    # 返回结果
    return {

        "img": img,

        "offset": offset,

        "state": nav_state,

        "road_center": road_center,

        "image_center": image_center,

        "left_blob": left_blob,

        "right_blob": right_blob,

        "fps": get_fps()
    }
