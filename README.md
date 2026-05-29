# 使用器件

1. 48mm麦轮、联轴器、底盘
2. mg310减速编码器电机
3. 主控：STM32F407VET6
4. 电机驱动：L298N
5. OLED屏幕(0.96寸)、蓝牙模块(HC-05)


# 核心实现

## 蓝牙部分
只有透传功能，将手机发送的指令发送至单片机，实现任务项目的切换

## 摄像头通信部分
一样利用串口通信，利用帧头+数据+帧尾的数据传递数据
利用状态机编程，用中断处理数据并在数据处理完成后利用标志位控制小车运动
```c
void CAMERA_Init(void);//开启串口
void CAMERA_UART_Receive_Process(uint8_t data);//利用状态机一帧一帧处理数据
```

## 电机部分(核心)

### 实现逻辑
1. 封装电机结构体，将电机的所有参数用函数传入(OOP思想)，车子底盘同理，一些基本一样的参数可以用宏封装，如电机的减速比、线数、底盘的宽度等等
```c
void Motor_WheelInit(Motor_t* m,
                     TIM_HandleTypeDef* htim_pwm, uint32_t pwm_channel,
                     GPIO_TypeDef* in_port, uint16_t in_pin1, uint16_t in_pin2,
                     TIM_HandleTypeDef* htim_encoder,
                     float wheel_diameter_m, uint32_t encoder_ppr, float gear_ratio);

void Car_Init(Car_t* car, Motor_t* fl, Motor_t* fr, Motor_t* rl, Motor_t* rr, float track_width_m);
```

2. 基础底层实现：启动时基单元、开启编码器通道、开启PWM、关闭PWM、设置方向……
```c
void Motor_StartWheel(Motor_t* m, uint16_t pwm_percent, uint8_t direction);
void Motor_StopWheel(Motor_t* m);
void Motor_SetDirectionInverted(Motor_t* m, bool inverted);
```

3. PID闭环控制基础：更新编码器（读值、读速度、计算速度），计算PID输出（利用编码器更新函数读到的值，计算更新的out）（由于使用增量式pid，所以最终的pwm输出会存在结构体里面，然后在用到的时候读取）
```c
void Motor_UpdateEncoder(Motor_t* m);
void Motor_PIDInit(Motor_t* m, float kp, float ki, float kd, float output_limit) ;
void Motor_PIDSetParams(Motor_t* m, float kp, float ki, float kd);
float Motor_PIDIncrementalUpdate(Motor_t* m, float target_speed_m_s,float dt_s);
```

4. 用户具体实现（完成任务具体所用函数）：单轮控制（主要用于调pid参数）、整车直线移动、转圈、平移
```c
void Motor_ControlWheel(Motor_t* m, float target_speed_m_s) ;

int Car_DriveDistance(Car_t* car, CarDir_t dir, float distance_m, float speed_m_s);
int Car_StrafeDistance(Car_t* car, CarDir_t dir, float distance_m, float speed_m_s);
int Car_RotateAngle(Car_t* car, CarDir_t dir, float angle_deg, float speed_m_s) ;
```
### 电机部分经验汇总
1. PID调参可以使用串口将数据动态打印，并利用串口动态设置参数
2. 调参时先设置kp，让实际速度在目标速度周边小幅度震荡，然后增加kd，一直抖动，最后将kp、kd变成原来的70%-80%，增加ki，让曲线贴到目标
3. **注意**：四个轮子单独调试的时候，上升曲线要尽量控制一致，否则四轮启动的时候会偏航
4. 停止轮子的时候，要清空所有error和integral，否则下次启动轮子的时候会不准
