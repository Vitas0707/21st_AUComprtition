
实验名称：箭头自分类识别 + 比赛串口输出
输出格式：0xFE J l/r/u/d 0xEF
启动指令：start222
串口：IO3(TX)  IO4(RX)
平台：01Studio CanMV K230

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
import gc
import sys

# ===================== 统一串口 IO3 + IO4 =====================
from machine import UART, FPIOA

fpioa = FPIOA()
fpioa.set_function(3, 11)   # IO3 → UART1_TX
fpioa.set_function(4, 12)   # IO4 → UART1_RX

uart = UART(1, baudrate=115200, tx=3, rx=4)

# 发送函数
def send_j_packet(cmd):
    buf = bytearray()
    buf.append(0xFE)
    buf.append(ord('J'))
    buf.append(ord(cmd))
    buf.append(0xEF)
    uart.write(buf)
    print("✅ 发送：0xFE J", cmd, "0xEF")

# ===================== 自学习识别类 =====================
class SelfLearningApp(AIBase):
    def __init__(self,kmodel_path,model_input_size,labels,top_k,threshold,database_path,rgb888p_size=[224,224],display_size=[1920,1080],debug_mode=0):
        super().__init__(kmodel_path,model_input_size,rgb888p_size,debug_mode)
        self.kmodel_path=kmodel_path
        self.model_input_size=model_input_size
        self.labels=labels
        self.database_path=database_path
        self.rgb888p_size=[ALIGN_UP(rgb888p_size[0],16),rgb888p_size[1]]
        self.display_size=[ALIGN_UP(display_size[0],16),display_size[1]]
        self.debug_mode=debug_mode
        self.threshold = threshold
        self.top_k = top_k

        self.crop_w = 400
        self.crop_h = 400
        self.crop_x = self.rgb888p_size[0] / 2.0 - self.crop_w / 2.0
        self.crop_y = self.rgb888p_size[1] / 2.0 - self.crop_h / 2.0
        self.crop_x_osd=0
        self.crop_y_osd=0
        self.crop_w_osd=0
        self.crop_h_osd=0

        self.ai2d=Ai2d(debug_mode)
        self.ai2d.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT,nn.ai2d_format.NCHW_FMT,np.uint8, np.uint8)
        self.data_init()
        self.last_send = ""  # 防重复发送

    def config_preprocess(self,input_image_size=None):
        with ScopedTiming("set preprocess config",self.debug_mode > 0):
            ai2d_input_size=input_image_size if input_image_size else self.rgb888p_size
            self.ai2d.crop(int(self.crop_x),int(self.crop_y),int(self.crop_w),int(self.crop_h))
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
            self.ai2d.build([1,3,ai2d_input_size[1],ai2d_input_size[0]],[1,3,self.model_input_size[1],self.model_input_size[0]])

    def postprocess(self,results):
        with ScopedTiming("postprocess",self.debug_mode > 0):
            return results[0][0]

    def draw_result(self, pl, feature):
        pl.osd_img.clear()
        pl.osd_img.draw_rectangle(self.crop_x_osd,self.crop_y_osd, self.crop_w_osd, self.crop_h_osd, color=(255,255,0,255), thickness=4)

        best_score = -1
        best_name = ""
        try:
            list_features = os.listdir(self.database_path)
            for feature_name in list_features:
                with open(self.database_path + feature_name, 'rb') as f:
                    data = f.read()
                save_vec = np.frombuffer(data, dtype=np.float32)
                score = self.getSimilarity(feature, save_vec)
                if score > self.threshold and score > best_score:
                    best_score = score
                    best_name = feature_name.split("_")[0]
        except:
            pass

        if best_name:
            pl.osd_img.draw_string_advanced(10, 10, 40, best_name, color=(255,0,0,255))

            # 比赛发送
            if best_name != self.last_send:
                if best_name == "左箭头":
                    send_j_packet('l')
                elif best_name == "右箭头":
                    send_j_packet('r')
                elif best_name == "上箭头":
                    send_j_packet('u')
                elif best_name == "下箭头":
                    send_j_packet('d')
                self.last_send = best_name

    def data_init(self):
        self.crop_x_osd = int(self.crop_x / self.rgb888p_size[0] * self.display_size[0])
        self.crop_y_osd = int(self.crop_y / self.rgb888p_size[1] * self.display_size[1])
        self.crop_w_osd = int(self.crop_w / self.rgb888p_size[0] * self.display_size[0])
        self.crop_h_osd = int(self.crop_h / self.rgb888p_size[1] * self.display_size[1])

    def getSimilarity(self,output_vec,save_vec):
        tmp = sum(output_vec * save_vec)
        mold_out = np.sqrt(sum(output_vec * output_vec))
        mold_save = np.sqrt(sum(save_vec * save_vec))
        return tmp / (mold_out * mold_out)

# ===================== 主程序 =====================
if __name__=="__main__":
    display="lcd2_4"

    if display=="hdmi":
        display_mode='hdmi'
        display_size=[1920,1080]
        rgb888p_size = [1920, 1080]
    elif display=="lcd3_5":
        display_mode= 'st7701'
        display_size=[800,480]
        rgb888p_size = [1920, 1080]
    elif display=="lcd2_4":
        display_mode= 'st7701'
        display_size=[640,480]
        rgb888p_size = [1280, 960]

    kmodel_path="/sdcard/examples/kmodel/recognition.kmodel"
    database_path="/sdcard/examples/utils/features/"
    model_input_size=[224,224]
    labels=["左箭头","上箭头","右箭头","下箭头"]
    top_k=4
    threshold=0.65

    pl=PipeLine(rgb888p_size=rgb888p_size,display_size=display_size,display_mode=display_mode)
    pl.create(Sensor(width=rgb888p_size[0], height=rgb888p_size[1]))
    sl=SelfLearningApp(kmodel_path,model_input_size=model_input_size,labels=labels,top_k=top_k,threshold=threshold,database_path=database_path,rgb888p_size=rgb888p_size,display_size=display_size,debug_mode=0)
    sl.config_preprocess()

    # ===================== 等待 start222 =====================
    print("等待 start222...")
    while True:
        if uart.any():
            try:
                cmd = uart.read().decode().strip()
                if 'start222' in cmd:
                    print("已启动箭头识别！")
                    break
            except:
                pass

    # 主循环
    while True:
        img=pl.get_frame()
        res=sl.run(img)
        sl.draw_result(pl,res)
        pl.show_image()
        gc.collect()
