# Untitled - By: 13840 - Mon May 18 2026

实验名称：数字识别 0~8 (屏蔽数字9) + 比赛串口输出
输出格式：0xFE M 数字 0xEF
平台：01Studio CanMV K230
串口引脚：IO3(TX1)、IO4(RX1)
'''

from libs.PipeLine import PipeLine, ScopedTiming
from libs.AIBase import AIBase
from libs.AI2D import Ai2d
import os
import ujson
from media.media import *
from media.sensor import *
from time import *
import nncase_runtime as nn
import ulab.numpy as np
import time
import image
import aicube
import random
import gc
import sys

# 串口初始化（改用IO3/TX1、IO4/RX1，适配硬件引脚）
from machine import UART, FPIOA


# 1. 配置IO3=TX1（功能号11）、IO4=RX1（功能号12）
fpioa = FPIOA()
fpioa.set_function(3, 11)   # IO3 → UART1_TX（功能号11）
fpioa.set_function(4, 12)   # IO4 → UART1_RX（功能号12）

# 2. 初始化UART1，波特率115200（对应IO3/IO4）

uart = UART(1, baudrate=115200, tx=3, rx=4)

# 比赛专用：发送 0xFE M 数字 0xEF
def send_m_packet(num):
    if num < 0 or num > 8:  # 只允许0~8
        return
    buf = bytearray()
    buf.append(0xFE)
    buf.append(ord('M'))
    buf.append(num)
    buf.append(0xEF)
    uart.write(buf)
    print("✅ 串口发送：0xFE M", num, "0xEF")

# 自定义OCR检测类

class OCRDetectionApp(AIBase):
    def __init__(self,kmodel_path,model_input_size,mask_threshold=0.5,box_threshold=0.2,rgb888p_size=[320,240],display_size=[1920,1080],debug_mode=0):
        super().__init__(kmodel_path,model_input_size,rgb888p_size,debug_mode)
        self.kmodel_path=kmodel_path
        self.model_input_size=model_input_size
        self.mask_threshold=mask_threshold
        self.box_threshold=box_threshold
        self.rgb888p_size=[ALIGN_UP(rgb888p_size[0],16),rgb888p_size[1]]
        self.display_size=[ALIGN_UP(display_size[0],16),display_size[1]]
        self.debug_mode=debug_mode
        self.ai2d=Ai2d(debug_mode)
        self.ai2d.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT,nn.ai2d_format.NCHW_FMT,np.uint8, np.uint8)

    def config_preprocess(self,input_image_size=None):
        with ScopedTiming("set preprocess config",self.debug_mode > 0):
            ai2d_input_size=input_image_size if input_image_size else self.rgb888p_size
            top,bottom,left,right=self.get_padding_param()
            self.ai2d.pad([0,0,0,0,top,bottom,left,right], 0, [0,0,0])
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
            self.ai2d.build([1,3,ai2d_input_size[1],ai2d_input_size[0]],[1,3,self.model_input_size[1],self.model_input_size[0]])

    def postprocess(self,results):
        with ScopedTiming("postprocess",self.debug_mode > 0):
            hwc_array=self.chw2hwc(self.cur_img)
            det_boxes = aicube.ocr_post_process(results[0][:,:,:,0].reshape(-1), hwc_array.reshape(-1),self.model_input_size,self.rgb888p_size, self.mask_threshold, self.box_threshold)
            return det_boxes

    def get_padding_param(self):
        dst_w = self.model_input_size[0]
        dst_h = self.model_input_size[1]
        input_width = self.rgb888p_size[0]
        input_high = self.rgb888p_size[1]
        ratio_w = dst_w / input_width
        ratio_h = dst_h / input_high
        if ratio_w < ratio_h:
            ratio = ratio_w
        else:
            ratio = ratio_h
        new_w = (int)(ratio * input_width)
        new_h = (int)(ratio * input_high)
        dw = (dst_w - new_w) / 2
        dh = (dst_h - new_h) / 2
        top = (int)(round(0))
        bottom = (int)(round(dh * 2 + 0.1))
        left = (int)(round(0))
        right = (int)(round(dw * 2 - 0.1))
        return  top, bottom, left, right

    def chw2hwc(self,features):
        ori_shape = (features.shape[0], features.shape[1], features.shape[2])
        c_hw_ = features.reshape((ori_shape[0], ori_shape[1] * ori_shape[2]))
        hw_c_ = c_hw_.transpose()
        new_array = hw_c_.copy()
        hwc_array = new_array.reshape((ori_shape[1], ori_shape[2], ori_shape[0]))
        del c_hw_
        del hw_c_
        del new_array
        return hwc_array

# 自定义OCR识别任务类
class OCRRecognitionApp(AIBase):
    def __init__(self,kmodel_path,model_input_size,dict_path,rgb888p_size=[320,240],display_size=[1920,1080],debug_mode=0):
        super().__init__(kmodel_path,model_input_size,rgb888p_size,debug_mode)
        self.kmodel_path=kmodel_path
        self.model_input_size=model_input_size
        self.dict_path=dict_path
        self.rgb888p_size=[ALIGN_UP(rgb888p_size[0],16),rgb888p_size[1]]
        self.display_size=[ALIGN_UP(display_size[0],16),display_size[1]]
        self.debug_mode=debug_mode
        self.dict_word=None
        self.read_dict()
        self.ai2d=Ai2d(debug_mode)
        self.ai2d.set_ai2d_dtype(nn.ai2d_format.RGB_packed,nn.ai2d_format.NCHW_FMT,np.uint8, np.uint8)

    def config_preprocess(self,input_image_size=None,input_np=None):
        with ScopedTiming("set preprocess config",self.debug_mode > 0):
            ai2d_input_size=input_image_size if input_image_size else self.rgb888p_size
            top,bottom,left,right=self.get_padding_param(ai2d_input_size,self.model_input_size)
            self.ai2d.pad([0,0,0,0,top,bottom,left,right], 0, [0,0,0])
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
            self.ai2d.build([input_np.shape[0],input_np.shape[1],input_np.shape[2],input_np.shape[3]],[1,3,self.model_input_size[1],self.model_input_size[0]])

    # 关键：只识别 0~8
    def postprocess(self,results):
        with ScopedTiming("postprocess",self.debug_mode > 0):
            preds = np.argmax(results[0], axis=2).reshape((-1))
            output_txt = ""
            allow_digits = {'0','1','2','3','4','5','6','7','8'}

            for i in range(len(preds)):
                if preds[i] != (len(self.dict_word) - 1) and (not (i > 0 and preds[i - 1] == preds[i])):
                    char = self.dict_word[preds[i]]
                    if char in allow_digits:
                        output_txt += char
            return output_txt

    def get_padding_param(self,src_size,dst_size):
        dst_w = dst_size[0]
        dst_h = dst_size[1]
        input_width = src_size[0]
        input_high = src_size[1]
        ratio_w = dst_w / input_width
        ratio_h = dst_h / input_high
        if ratio_w < ratio_h:
            ratio = ratio_w
        else:
            ratio = ratio_h
        new_w = (int)(ratio * input_width)
        new_h = (int)(ratio * input_high)
        dw = (dst_w - new_w) / 2
        dh = (dst_h - new_h) / 2
        top = (int)(round(0))
        bottom = (int)(round(dh * 2 + 0.1))
        left = (int)(round(0))
        right = (int)(round(dw * 2 - 0.1))
        return  top, bottom, left, right

    def read_dict(self):
        if self.dict_path!="":
            with open(self.dict_path, 'r') as file:
                line_one = file.read(100000)
                line_list = line_one.split("\r\n")
            self.dict_word = {num: char.replace("\r", "").replace("\n", "") for num, char in enumerate(line_list)}

class OCRDetRec:
    def __init__(self,ocr_det_kmodel,ocr_rec_kmodel,det_input_size,rec_input_size,dict_path,mask_threshold=0.25,box_threshold=0.3,rgb888p_size=[320,240],display_size=[1920,1080],debug_mode=0):
        self.ocr_det_kmodel=ocr_det_kmodel
        self.ocr_rec_kmodel=ocr_rec_kmodel
        self.det_input_size=det_input_size
        self.rec_input_size=rec_input_size
        self.dict_path=dict_path
        self.mask_threshold=mask_threshold
        self.box_threshold=box_threshold
        self.rgb888p_size=[ALIGN_UP(rgb888p_size[0],16),rgb888p_size[1]]
        self.display_size=[ALIGN_UP(display_size[0],16),display_size[1]]
        self.debug_mode=debug_mode
        self.ocr_det=OCRDetectionApp(self.ocr_det_kmodel,model_input_size=self.det_input_size,mask_threshold=self.mask_threshold,box_threshold=self.box_threshold,rgb888p_size=self.rgb888p_size,display_size=self.display_size,debug_mode=0)
        self.ocr_rec=OCRRecognitionApp(self.ocr_rec_kmodel,model_input_size=self.rec_input_size,dict_path=self.dict_path,rgb888p_size=self.rgb888p_size,display_size=self.display_size)
        self.ocr_det.config_preprocess()

    def run(self,input_np):
        det_res=self.ocr_det.run(input_np)
        boxes=[]
        ocr_res=[]
        for det in det_res:
            self.ocr_rec.config_preprocess(input_image_size=[det[0].shape[2],det[0].shape[1]],input_np=det[0])
            ocr_str=self.ocr_rec.run(det[0])
            if ocr_str.strip() != "":
                ocr_res.append(ocr_str)
                boxes.append(det[1])
            gc.collect()
        return boxes,ocr_res

    def draw_result(self,pl,det_res,rec_res):
        pl.osd_img.clear()
        if det_res:
            for j in range(len(det_res)):
                for i in range(4):
                    x1 = det_res[j][(i * 2)] / self.rgb888p_size[0] * self.display_size[0]
                    y1 = det_res[j][(i * 2 + 1)] / self.rgb888p_size[1] * self.display_size[1]
                    x2 = det_res[j][((i + 1) * 2) % 8] / self.rgb888p_size[0] * self.display_size[0]
                    y2 = det_res[j][((i + 1) * 2 + 1) % 8] / self.rgb888p_size[1] * self.display_size[1]
                    pl.osd_img.draw_line((int(x1), int(y1), int(x2), int(y2)), color=(0, 255, 0, 255), thickness=3)
                pl.osd_img.draw_string_advanced(int(x1),int(y1-30),32,rec_res[j],color=(255,255,0))

if __name__=="__main__":
    display="lcd3_5"

    if display=="hdmi":
        display_mode='hdmi'
        display_size=[1920,1080]
    elif display=="lcd3_5":
        display_mode= 'st7701'
        display_size=[800,480]
    elif display=="lcd2_4":
        display_mode= 'st7701'
        display_size=[640,480]

    rgb888p_size=[640,360]

    ocr_det_kmodel_path="/sdcard/examples/kmodel/ocr_det_int16.kmodel"
    ocr_rec_kmodel_path="/sdcard/examples/kmodel/ocr_rec_int16.kmodel"
    dict_path="/sdcard/examples/utils/dict.txt"

    ocr_det_input_size=[640,640]
    ocr_rec_input_size=[512,32]
    mask_threshold=0.25
    box_threshold=0.3

    pl=PipeLine(rgb888p_size=rgb888p_size,display_size=display_size,display_mode=display_mode)
    if display =="lcd2_4":
        pl.create(Sensor(width=1280, height=960))
    else:
        pl.create(Sensor(width=1920, height=1080))

    ocr=OCRDetRec(ocr_det_kmodel_path,ocr_rec_kmodel_path,det_input_size=ocr_det_input_size,rec_input_size=ocr_rec_input_size,dict_path=dict_path,mask_threshold=mask_threshold,box_threshold=box_threshold,rgb888p_size=rgb888p_size,display_size=display_size)

    clock = time.clock()

    # 等待单片机启动指令
    print("等待单片机发送 start 启动...")
    while True:
        if uart.any():
            cmd = uart.read().decode().strip()
            if 'M' in cmd:
                print("已启动数字识别！")
                break

    last_sent = -1  # 防止重复发送

    while True:
        clock.tick()
        img=pl.get_frame()
        det_res,rec_res=ocr.run(img)
        ocr.draw_result(pl,det_res,rec_res)
        pl.show_image()
        gc.collect()

# ===================== 比赛：数字识别串口发送 =====================
        if len(rec_res) > 0:
            num_str = rec_res[0]
            if num_str.isdigit():
                num = int(num_str)
                if 0 <= num <= 8 and num != last_sent:
                    send_m_packet(num)
                    last_sent = num
        # print("FPS：", clock.fps())
