'''
camera.py (重构版：仅含纯算法逻辑)
说明：移除了所有硬件初始化，仅用于处理 PipeLine 传来的图像
'''

# 阈值 (灰度)
threshold = (0, 60)

# ROI区域 (基于 1920x1080 分辨率)
left_roi = (0, 600, 300, 480)
right_roi = (1620, 600, 300, 480)

# =====================================================
# 核心算法函数
# =====================================================

def find_lane_blobs(img):
    # img 应为调用者传入的灰度图像对象
    left_blobs = img.find_blobs([threshold], roi=left_roi, pixels_threshold=200, area_threshold=200, merge=True)
    right_blobs = img.find_blobs([threshold], roi=right_roi, pixels_threshold=200, area_threshold=200, merge=True)

    if not left_blobs or not right_blobs:
        return None, None

    left_blob = max(left_blobs, key=lambda b: b.pixels())
    right_blob = max(right_blobs, key=lambda b: b.pixels())
    return left_blob, right_blob

def calc_offset(img, left_blob, right_blob):
    road_center = (left_blob.cx() + right_blob.cx()) // 2
    image_center = img.width() // 2
    offset = road_center - image_center
    return offset, road_center, image_center

def draw_center_lines(img, road_center, image_center):
    # 在传入的 OSD 层 img 上绘制
    img.draw_line(road_center, 0, road_center, img.height(), color=255)
    img.draw_line(image_center, 0, image_center, img.height(), color=128)

#def draw_blobs(img, left_blob, right_blob):
#    img.draw_rectangle(left_blob.rect(), color=255)
#    img.draw_cross(left_blob.cx(), left_blob.cy(), color=255)
#    img.draw_rectangle(right_blob.rect(), color=255)
#    img.draw_cross(right_blob.cx(), right_blob.cy(), color=255)
