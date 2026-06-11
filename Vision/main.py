"""
实验名称：在线训练-YOLO图像检测: 基于摄像头
实验平台：01Studio CanMV K230/CanMV K230 mini
说明：可以通过display="xxx"参数选择"hdmi"、"lcd3_5"(3.5寸mipi屏)或"lcd2_4"(2.4寸mipi屏)显示方式
01科技（01Studio）在线训练平台：https://ai.01studio.cc
"""

from libs.PipeLine import PipeLine
from libs.YOLO import YOLO11
from libs.Utils import *
from media.sensor import *
import os, sys, gc
import ulab.numpy as np
import image
import time
from machine import FPIOA, UART
from libs.PlatTasks import ClassificationApp
# 这里为自动生成内容，自定义场景请修改为您自己的模型路径、标签名称、模型输入大小
kmodel_path="/sdcard/yolo11n_det_320.kmodel"
labels = {0: '0', 1: '1', 2: '2', 3: '2', 4: '3', 5: '3', 6: '4', 7: '4', 8: '5', 9: '6', 10: '6', 11: 'ANIMAL', 12: 'F', 13: 'FRUIT', 14: 'L', 15: 'PERSON', 16: 'R'}
model_input_size = [320, 320]
# 显示模式，可以选择"hdmi"、"lcd3_5"(3.5寸mipi屏)和"lcd2_4"(2.4寸mipi屏)
display = "lcd2_4"

if display == "hdmi":
    display_mode = "hdmi"
    display_size = [1920, 1080]

elif display == "lcd3_5":
    display_mode = "st7701"
    display_size = [800, 480]

elif display == "lcd2_4":
    display_mode = "st7701"
    display_size = [640, 480]

rgb888p_size = [640, 360]

# 初始化PipeLine
pl = PipeLine(
    rgb888p_size=rgb888p_size, display_size=display_size, display_mode=display_mode
)

if display == "lcd2_4":
    pl.create(sensor=Sensor(width=1280, height=960))  # 创建PipeLine实例，画面4:3

else:
    pl.create(sensor=Sensor(width=1920, height=1080))  # 创建PipeLine实例

display_size = pl.get_display_size()

# 初始化YOLO11实例
confidence_threshold = 0.6  # 置信度
nms_threshold = 0.45
yolo = YOLO11(
    task_type="detect",
    mode="video",
    kmodel_path=kmodel_path,
    labels=labels,
    rgb888p_size=rgb888p_size,
    model_input_size=model_input_size,
    display_size=display_size,
    conf_thresh=confidence_threshold,
    nms_thresh=nms_threshold,
    max_boxes_num=50,
    debug_mode=0,
)
yolo.config_preprocess()
# 修改为匹配类定义的参数名：使用 'mode' 代替 'inference_mode'
cls_app = ClassificationApp(
    mode="image",  # 必须改为 'mode'
    kmodel_path="/sdcard/mp_deployment_source/arrow.kmodel",
    labels=["L", "None", "R"],
    model_input_size=[224, 224],
    confidence_threshold=0.8,
    rgb888p_size=[640, 360],  # 确保这里匹配你的 PipeLine 设置
    display_size=[800, 480],
    debug_mode=0
)

# 现在可以调用了
cls_app.config_preprocess()
print("分类器初始化完成")
if __name__=='__main__':
    clock = time.clock()
    fpioa = FPIOA()
    fpioa.set_function(11,FPIOA.UART2_TXD)
    fpioa.set_function(12,FPIOA.UART2_RXD)
    uart=UART(UART.UART2,115200)

    STATE_STANDBY = 0  # 待机状态：只看画面，不跑 AI
    STATE_RUNNING = 1  # 运行状态：全速跑 YOLO 模型

    current_state = STATE_RUNNING # 开机默认处于待机状态 (也可改为 STATE_RUNNING)
    CONFIRM_THRESHOLD = 5     # 连续识别到相同结果 5 次才确认（可根据帧率修改）
    last_detected_msg = ""      # 记录上一次组合出来的指令
    consecutive_count = 0       # 连续相同次数的计数器
    has_sent_current = False    # 标记当前指令是否已经发送过（防刷屏）
    last_detected_ms = ""      # 记录上一次组合出来的指令
    has_sent_current_1 = False
    need_classify = False
    while True:

        clock.tick()
        cmd = uart.read(1)
        if cmd == b'1':
            current_state = STATE_RUNNING
            print(">>> 收到指令：切换为【运行状态】")
        elif cmd == b'0':
            current_state = STATE_STANDBY
            print(">>> 收到指令：切换为【待机状态】")
            pl.osd_img.clear() # 进入待机时，清空屏幕上的检测框

        # 逐帧推理
        img = pl.get_frame()

        if current_state == STATE_RUNNING:
            res = yolo.run(img)
            yolo.draw_result(res, pl.osd_img)


            # 业务逻辑：遍历识别结果，做出串口决策
            if res:
                #print("【YOLO 原始结果】:", res)
                class_ids = res[1]  # 所有的标签 ID
                num_id = None
                arrow_id = None
                photo_id = None
             # 3. 此时 label_id 已经是一个纯正的整数，可以放心查找字典
                for i in range(len(class_ids)):
                    # 精准拿到当前目标的 ID
                    label_id = class_ids[i]
                    label_name = labels.get(label_id, "UNKNOWN")
                    # 如果这个 ID 代表的是数字 (假设 0-9 代表数字1-9)
                    if label_name in ['0','1', '2', '3', '4', '5', '6']:
                        num_id = label_name  # 抓到了数字的 ID！
                        #print(num_id)
                    # 如果这个 ID 代表的是方向
                    elif label_name in ['L', 'R', 'F']:
                        arrow_id = label_name # 抓到了箭头的 ID！
                        #print(arrow_id)

                        if arrow_id == 'L' or arrow_id =='R':
                            need_classify = True
#                            print('start')
                        else :
                            need_classify = False
#                print("当前 need_classify 的值是:", need_classify) # 关键：看看循环结束后它是不是 True

                # 执行级联分类
                if need_classify:

                    res_cls = cls_app.run(img)
#                    print("已转化")
                    # 使用当前帧进行二次分类推理
#                    print(">>> 分类模型已返回，结果为:", res_cls)
                    if res_cls and isinstance(res_cls, dict):
                        # 从字典中获取 'label' 的值，如果获取不到则默认为 None
                        if res_cls.get('label')=='':
                            arrow_id = None
                        else:
                            arrow_id = res_cls.get('label')
#                print(num_id)
#                        print(arrow_id)
                    # 假设 res_cls 返回的是分类ID或标签
                    # 这里进行你的业务逻辑处理，例如拼接结果
                    # 修改第 165 行，直接解析返回的字典


#                    print(final_result)




            if num_id != None :
            # 【拼接核心代码】：使用 %s 占位符进行拼接
                current_msg = num_id
                # 1. 和上一帧比对
                if current_msg == last_detected_msg:
                    consecutive_count += 1  # 结果一样，计数器累加
                else:
                    # 结果变了！重新开始认领和计数
                    last_detected_msg = current_msg
                    consecutive_count = 1
                    has_sent_current = False # 新目标，允许发送

                # 2. 判断是否达到信任阈值，并且还没有发送过
                if consecutive_count >= CONFIRM_THRESHOLD and not has_sent_current:
                    uart.write('<')
                    uart.write('M')
                    uart.write(current_msg)
                    uart.write('>')
                    print(current_msg)
#                    current_state = STATE_STANDBY
#                    print(">>> 切换为【待机状态】")
#                    pl.osd_img.clear() # 进入待机时，清空屏幕上的检测框
#                    print(">>> [稳定识别] 连续确认，发送指令" )

                    # 标记为已发送，防止后续一直狂发同一句话卡死单片机
                    has_sent_current = True
                    need_classify = False
            elif num_id =='0':
                uart.write('<')
                uart.write('M')
                uart.write('O')
                uart.write('>')
            elif photo_id != None:
                current_msg = photo_id
                if current_msg == last_detected_msg:
                    consecutive_count += 1  # 结果一样，计数器累加
                else:
                    # 结果变了！重新开始认领和计数
                    last_detected_msg = current_msg
                    consecutive_count = 1
                    has_sent_current = False # 新目标，允许发送
            elif arrow_id != None :
            # 【拼接核心代码】：使用 %s 占位符进行拼接
                current_ms = arrow_id
                # 1. 和上一帧比对
                if current_ms == last_detected_ms:
                    consecutive_count_1 += 1  # 结果一样，计数器累加
                else:
                    # 结果变了！重新开始认领和计数
                    last_detected_ms = current_ms
                    consecutive_count_1 = 1
                    has_sent_current_1 = False # 新目标，允许发送

                # 2. 判断是否达到信任阈值，并且还没有发送过
                if consecutive_count_1 >= CONFIRM_THRESHOLD and not has_sent_current_1:
                    uart.write('<')
                    uart.write('M')
                    uart.write(current_ms)
                    uart.write('>')
                    print(current_ms)
#                    current_state = STATE_STANDBY
#                    print(">>> 切换为【待机状态】")
#                    pl.osd_img.clear() # 进入待机时，清空屏幕上的检测框
#                    print(">>> [稳定识别] 连续确认，发送指令" )

                    # 标记为已发送，防止后续一直狂发同一句话卡死单片机
                    has_sent_current_1 = True
                    need_classify = False

#                # 2. 判断是否达到信任阈值，并且还没有发送过
#                if consecutive_count >= CONFIRM_THRESHOLD and not has_sent_current:
#                    if photo_id =='ANIMAL':
#                        uart.write('<')
#                        uart.write('R')
#                        uart.write('0')
#                        uart.write('>')
#                    if photo_id =='PERSON':
#                        uart.write('<')
#                        uart.write('R')
#                        uart.write('1')
#                        uart.write('>')
#                    if photo_id =='FRUIT':
#                        uart.write('<')
#                        uart.write('R')
#                        uart.write('2')
#                        uart.write('>')
#                    current_state = STATE_STANDBY
#                    print(">>> 收到指令：切换为【待机状态】")
#                    pl.osd_img.clear() # 进入待机时，清空屏幕上的检测框

#                    print(">>> [稳定识别] 连续确认，发送指令" )
#            else:
#                # 如果当前帧没有同时看到数字和箭头（比如画面移开了）
#                # 重置计数器，非常严苛地要求必须连续看到
#                last_detected_msg = ""
#                consecutive_count = 0
#                has_sent_current = False

            pl.show_image()
            gc.collect()


    # 释放资源
    yolo.deinit()
    pl.destroy()
