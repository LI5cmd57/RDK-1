# RDK-1
本作品为基于RDK和yolov8的智能轮腿盲道检测系统。发明专利已受理。作品用到了深度学习、目标检测、实例分割、Linux系统、机器视觉、轮腿平衡、万物互联（esp8266）、单片机控制等技术。近年来盲道占用现象愈发严重、盲道设计不合理问题突出（例如被井盖和柱子拦断），盲人生命安全受到威胁，且目前国内尚无不用佩戴在盲人身上、可以识别盲道与旁边路面同色系且可以及时广泛的通知城市管理人员清理盲道障碍和维护盲道缺陷的全自动智能化系统。本系统基于高性能计算平台RDK X5和先进的YOLOv8目标检测算法，设计了一套高效、精准的盲道检测解决方案。系统通过搭载摄像头实时采集路面图像，利用YOLOv8强大的视觉识别能力，快速检测并定位盲道区域，同时结合轮腿式移动平台，实现复杂环境下的稳定导航与避障功能。
##视觉端
```import math
import RPi.GPIO as GPIO
import cv2
import numpy
import numpy as np
import math
import serial
import time
from time import sleep
from ultralytics import YOLO
from cv2 import getTickCount, getTickFrequency
from PIL import Image
import asyncio
import datetime
ncnn_model = YOLO("/home/ly/Desktop/yolov8/ultralytics/runs/segment/train/weights/best.mnn")
model = YOLO("/home/ly/Desktop/yolov8/ultralytics/runs/detect/best.mnn")
CONTROL_PIN=11
CONTROL_PIN2=18
PWM_FREQ=20
STEP=15
duojiz=95
duojis=5
flag=0
maskflag=0
maskflag2=0
xuanzhuan=0
def angle_to_duty_cycle(angle=0):
    duty_clcle=(0.05*PWM_FREQ)+(0.19*PWM_FREQ*angle/180)
    pwm.ChangeDutyCycle(duty_clcle)
    sleep(0.3)
    pwm.ChangeDutyCycle(0) #清除当前占空比，使舵机停止抖动
    sleep(0.01)
    
def angle_to_duty_cycle2(angle=0):
    duty_clcle=(0.05*PWM_FREQ)+(0.19*PWM_FREQ*angle/180)
    pwm2.ChangeDutyCycle(duty_clcle)
    sleep(0.3)
    pwm2.ChangeDutyCycle(0) #清除当前占空比，使舵机停止抖动
    sleep(0.01)
                                                    
def main2():
    global maskflag
    global frame1
    global mask_raw
    success, frame = cap.read()
    i = 0
    timeF = 25
    j = 0
    if maskflag==0:
        while success:
            i = i + 1
            if (i % timeF == 0):
                j = j + 1
                frame1 = frame
                break
            success, frame1 = cap.read()
            #if success == False:  
             #   break
            if success:
                results = ncnn_model.predict(source=frame1) # 对当前帧进行目标检测并显示结果
            annotated_frame = results[0].plot()
            for result in results:
                if result.masks is not None:
                    mask_raw = result.masks[0].cpu().data.numpy().transpose(1,2,0)
                    maskflag=1
                    for mask in result.masks:
                        mask_raw += mask.cpu().data.numpy().transpose(1,2,0)
                        print(30)
                    #cv2.imshow("hb",mask_raw)
            if maskflag==1:
                break
            #cv2.imshow("hb",mask_raw)
async def main():
    global annotated_frame
    global annotated_frame1
    global duojiz
    global duojis
    global flag
    global maskflag
    global xuanzhuan
    you=0
    dayou=0
    zuo=0
    dazuo=0
    if success:
        results = ncnn_model.predict(source=frame1) # 对当前帧进行目标检测并显示结果
        #results_1 = model.predict(source=frame1)
    annotated_frame = results[0].plot()
    #annotated_frame_1 = results_1[0].plot()
    #show mask         zhegehanshuyidingyaozhixing!(shexiangtouxianzhaodaomangdao)
    for result in results:
        if result.masks is None:
                if flag==0 and maskflag==0:
                    duojiz=duojiz+30
                    angle_to_duty_cycle(duojiz)
                    main2()
                    xuanzhuan=1
                    if(maskflag==1 and xuanzhuan==1):
                        ser.write(('@'+str(10001)+'\r\n').encode())
                        angle_to_duty_cycle(90)#zuoyou
                        angle_to_duty_cycle2(0)#shangxia
                        xuanzhuan=6
                        #ser.write(('@'+str(10006)+'\r\n').encode())
                    print("舵机旋转挡位:",xuanzhuan)
                    if maskflag==0:
                        if duojiz>150:
                            angle_to_duty_cycle(90)
                            flag=1
                        elif duojiz<150:
                            duojiz=duojiz+30
                            angle_to_duty_cycle(duojiz)
                            main2()
                            xuanzhuan=2
                            if(maskflag==1 and xuanzhuan==2):
                                ser.write(('@'+str(10002)+'\r\n').encode())
                                angle_to_duty_cycle(90)#zuoyou
                                angle_to_duty_cycle2(0)#shangxia
                                xuanzhuan=6
                                #ser.write(('@'+str(10006)+'\r\n').encode())
                            print("舵机旋转挡位:",xuanzhuan)
                            if maskflag==0:
                                if duojiz>150:
                                    angle_to_duty_cycle(90)
                                    flag=1
                                    if flag==1:
                                        duojiz=90-30
                                        angle_to_duty_cycle(duojiz)
                                        main2()
                                        xuanzhuan=3
                                        if(maskflag==1 and xuanzhuan==3):
                                            ser.write(('@'+str(10003)+'\r\n').encode())
                                            angle_to_duty_cycle(90)#zuoyou
                                            angle_to_duty_cycle2(0)#shangxia
                                            xuanzhuan=6
                                            #ser.write(('@'+str(10006)+'\r\n').encode())
                                        print("舵机旋转挡位:",xuanzhuan)
                                        if maskflag==0:
                                            if(duojiz>5):
                                                duojiz=duojiz-30
                                                angle_to_duty_cycle(duojiz)
                                                main2()
                                                xuanzhuan=4
                                                if(maskflag==1 and xuanzhuan==4):
                                                    ser.write(('@'+str(10004)+'\r\n').encode())
                                                    angle_to_duty_cycle(90)#zuoyou
                                                    angle_to_duty_cycle2(0)#shangxia
                                                    xuanzhuan=6
                                                    #ser.write(('@'+str(10006)+'\r\n').encode())
                                                print("舵机旋转挡位:",xuanzhuan)
                                                if maskflag==0:
                                                    if(duojiz>5):
                                                        duojiz=duojiz-30
                                                        angle_to_duty_cycle(duojiz)
                                                        main2()
                                                        xuanzhuan=5
                                                        if(xuanzhuan==5 and maskflag==0):
                                                            ser.write(('@'+str(10005)+'\r\n').encode())
                                                        print("舵机旋转挡位:",xuanzhuan)
                                                        if maskflag==0:
                                                            if duojiz<20:
                                                                angle_to_duty_cycle(90)
                                                                flag=0
                                                                print(8)
                    

async def main1():
    global xuanzhuan
    global maskflag
    #ser.write(('@'+str(direction)+'\r\n').encode())
   # if ser.isOpen == False:
       # ser.open()               
   # try:
       # while True:
          #  size = ser.inWaiting()               
          #  if size != 0:
              #  response = ser.read(size)        # 读取内容并显示
              #  angle_to_duty_cycle2(0)#shangxia
               # print(response)        
            #ser.flushInput()                 # 清空接收缓存区
            #time.sleep(0.1)                  # 软件延时
   # except KeyboardInterrupt:
       # ser.close()
        
async def main3():
    global maskflag
    global maskflag2
    global frame1
    global mask_raw
    if success:
         results = ncnn_model.predict(source=frame1) # 对当前帧进行目标检测并显示结果
         results_1 = model.predict(source=frame1,classes=[1])
    annotated_frame = results[0].plot()
    annotated_frame_1 = results_1[0].plot()
         
    if len(results_1[0].boxes)>0:
        ser.write(('@'+str(20000)+'\r\n').encode())
        ser.write(('@'+str(20000)+'\r\n').encode())
        ser.write(('@'+str(20000)+'\r\n').encode())
        ser.write(('@'+str(20000)+'\r\n').encode())
        ser.write(('@'+str(20000)+'\r\n').encode())
    if len(results[0].boxes)==0:
        ser.write(('@'+str(20001)+'\r\n').encode())
        ser.write(('@'+str(20001)+'\r\n').encode())
        ser.write(('@'+str(20001)+'\r\n').encode())
        ser.write(('@'+str(20001)+'\r\n').encode())
        ser.write(('@'+str(20001)+'\r\n').encode())
    if len(results[0].boxes)>0:
        ser.write(('@'+str(20002)+'\r\n').encode())
        ser.write(('@'+str(20002)+'\r\n').encode())
        ser.write(('@'+str(20002)+'\r\n').encode())
        ser.write(('@'+str(20002)+'\r\n').encode())
        ser.write(('@'+str(20002)+'\r\n').encode())
        
    for result in results:
                if result.masks is not None:
                    mask_raw = result.masks[0].cpu().data.numpy().transpose(1,2,0)
                    maskflag=1
                    for mask in result.masks:
                        mask_raw += mask.cpu().data.numpy().transpose(1,2,0)
    cv2.imshow("zhangaiwu", annotated_frame_1)
    #print(results_1)
        




GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(CONTROL_PIN,GPIO.OUT)
GPIO.setup(CONTROL_PIN2,GPIO.OUT)

pwm=GPIO.PWM(CONTROL_PIN,PWM_FREQ)
pwm2=GPIO.PWM(CONTROL_PIN2,PWM_FREQ)
pwm.start(0)
pwm2.start(0)

angle_to_duty_cycle(90)#zuoyou
angle_to_duty_cycle2(0)#shangxia


ERROR = -999
run_flag = 0 
center = 320
direction=10000
GPIO.setmode(GPIO.BCM)
cap = cv2.VideoCapture(0)
ser=serial.Serial("/dev/ttyAMA0",9600,timeout=0.5) #使用树莓派的GPIO口连接串行口


while(1):
    global mask_raw
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 320)#tiaozheng fenbianlu
    #ret, frame = cap.read()
    success, frame = cap.read()
    i = 0
    timeF = 25
    j = 0
    while success:
        i = i + 1
        if (i % timeF == 0):
            j = j + 1
            frame1 = frame
            break
        success, frame = cap.read()
    if success == False:  
        break
    asyncio.run(main1())
   # print(results_1)
    if xuanzhuan != 6:
        asyncio.run(main())
        #print(results_1)
    elif xuanzhuan ==6 :
        asyncio.run(main3())
        #print(results_1)
    #size = ser.inWaiting() 
    #if size != 0:
     #   response = ser.read(size)        
      #  print(response)    
    # 膨胀，白区域变大
    dst = cv2.dilate(mask_raw, None, iterations=2)
    #dst = cv2.flip(dst, -1)
    # # 腐蚀，白区域变小
    #dst = cv2.erode(dst, None, iterations=6)
    #cv2.imshow("dst",dst)
    
    #cv2.imshow("zhangaiwu", annotated_frame_1)
    #cv2.imshow("annotated_frame",annotated_frame)
    color = dst[400]
    color1 = dst[200]
    color2 = dst[300]
    # 找到bai色的像素点个数
    #ser.write("@LED_ON".encode())
    black_count = np.sum(color != 0)
    # 找到黑色的像素点索引
    black_index = np.where(color != 0)
    # 防止black_count=0的报错
    #计算偏移的角度。
    black_count1_judge = np.sum(color1 == 0)#第200行如果全是白色的话就不计算角度了
    black_count2_judge = np.sum(color2 == 0)
    black_index1 = np.where(color1 != 0)
    black_index2 = np.where(color2 != 0)
    black_count1 = np.sum(color1 != 0)
    black_count2 = np.sum(color2 != 0)
    if black_count1 == 0:
        black_count1 = 1
    if black_count2 == 0:
        black_count2 = 1
    if black_count1_judge < 630 and black_count2_judge < 630:
        center1 = (black_index1[0][black_count1 - 1] + black_index1[0][0]) / 2#对应的是第200行
        direction1 = center1 - 302
        center2 = (black_index2[0][black_count2 - 1] + black_index2[0][0]) / 2#对应的是第300行
        direction2 = center2 - 302
        print("center1:",center1,"center2:",center2)
        angle = '%.2f'%(math.degrees(numpy.arctan(100/(direction2-direction1))))
        ser.write(('@'+str(angle)+'\r\n').encode())
        print("偏转角为：", angle)
        #ser.write("@"+angle+"\r\n".encode())
        cv2.line(annotated_frame,(int(center2),300), (int(center1),200),  color = (255,0,0), thickness = 3)  # 蓝色
        cv2.line(annotated_frame, (0, 300), (640, 300), color=(0, 0, 255), thickness=3)                      # 红色
        cv2.line(annotated_frame, (0, 200), (640, 200), color=(0, 0, 255), thickness=3)
        #cv2.imshow("frame", annotated_frame)
        pass
    if black_count1_judge >= 630 or black_count2_judge>= 630:  #如果没有发现第150行喝第300行的黑线
        angle = ERROR
        print("偏转角为：", angle)
        #print("偏转角为：", angle)
        #str(int(num)).encode()
        #angle=int(angle)
        #print("偏转角为：", angle)
        ser.write(('@'+str(angle)+'\r\n').encode())
        print("偏转角为：", angle)
        #ser.write("@"+angle+"\r\n".encode())
        pass
        # 防止black_count=0的报错
    if black_count == 0:
        black_count = 1
    # 找到黑色像素的中心点位置# 计算出center与标准中心点的偏移量
    
    try:
        center = (black_index[0][black_count - 1] + black_index[0][0]) / 2
    except IndexError:
        direction = center - 302 #在实际操作中，我发现当黑线处于小车车体正中央的时候应该减去302
        direction = int('%4d'%direction)
        print("方向为：",direction)
    #ser.write(('@'+str(direction)+'\r\n').encode())
    asyncio.run(main1())
    #cv2.imshow("frame", annotated_frame)
    
            
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    
cap.release()  
cv2.destroyAllWindows()  
ser.close() #关闭端口```






 










 







##轮腿代码
###cpu0_main.c
```/*********************************************************************************************************************
* TC387 Opensourec Library 即（TC387 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC387 开源库的一部分
*
* TC387 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          cpu0_main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.10.2
* 适用平台          TC387QP
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-11-04       pudding            first version
********************************************************************************************************************/
#include "zf_common_headfile.h"


#pragma section all "cpu0_dsram"
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

// **************************** 代码区域 ****************************

int core0_main(void)
{
    clock_init();                   // 获取时钟频率<务必保留>
    debug_init();                   // 初始化默认调试串口
    // 此处编写用户代码 例如外设初始化代码等
    gpio_init(P33_10, GPO, 0, GPO_PUSH_PULL);
    ips200_init(IPS200_TYPE_SPI);
    key_init(10);
    // 无线串口
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIRELESS_UART);
    seekfree_assistant_oscilloscope_data.channel_num = 2;
    wireless_uart_init();
    // 电机驱动
    small_driver_uart_init();
    imu660ra_init();
    Attitude_Init();
    mt9v03x_init ();   // 摄像头初始化
    Steer_Init();


    PID_Init();

//    KEY_init();
//    Multi_Button_All_Init();


    pit_ms_init(CCU60_CH0,1);
    pit_ms_init(CCU60_CH1,1);
    pit_ms_init(CCU61_CH0,1);

//    uart_init(UART_8,115200,UART8_TX_P33_7,UART8_RX_P33_6);
//    uart_rx_interrupt(UART_8, 1);

    // 此处编写用户代码 例如外设初始化代码等
    cpu_wait_event_ready();         // 等待所有核心初始化完毕
    while (TRUE)
    {
        // 此处编写需要循环执行的代码
        // 此处编写需要循环执行的代码
    }
}
#pragma section all restore
// **************************** 代码区域 ****************************```
###cpu0_main.h
```/**
 * \file Cpu0_Main.h
 * \brief System initialization and main program implementation.
 *
 * \version iLLD_Demos_1_0_1_11_0
 * \copyright Copyright (c) 2014 Infineon Technologies AG. All rights reserved.
 *
 *
 *                                 IMPORTANT NOTICE
 *
 *
 * Use of this file is subject to the terms of use agreed between (i) you or 
 * the company in which ordinary course of business you are acting and (ii) 
 * Infineon Technologies AG or its licensees. If and as long as no such 
 * terms of use are agreed, use of this file is subject to following:


 * Boost Software License - Version 1.0 - August 17th, 2003

 * Permission is hereby granted, free of charge, to any person or 
 * organization obtaining a copy of the software and accompanying 
 * documentation covered by this license (the "Software") to use, reproduce,
 * display, distribute, execute, and transmit the Software, and to prepare
 * derivative works of the Software, and to permit third-parties to whom the 
 * Software is furnished to do so, all subject to the following:

 * The copyright notices in the Software and this entire statement, including
 * the above license grant, this restriction and the following disclaimer, must
 * be included in all copies of the Software, in whole or in part, and all
 * derivative works of the Software, unless such copies or derivative works are
 * solely in the form of machine-executable object code generated by a source
 * language processor.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT
 * SHALL THE COPYRIGHT HOLDERS OR ANYONE DISTRIBUTING THE SOFTWARE BE LIABLE 
 * FOR ANY DAMAGES OR OTHER LIABILITY, WHETHER IN CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.

 *
 * \defgroup IfxLld_Demo_STMDemo_SrcDoc Source code documentation
 * \ingroup IfxLld_Demo_STMDemo
 *
 */

#ifndef CPU0_MAIN_H
#define CPU0_MAIN_H

/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/
#define TEST_T 0

#include "Cpu/Std/Ifx_Types.h"
/******************************************************************************/
/*-----------------------------------Macros-----------------------------------*/
/******************************************************************************/

/******************************************************************************/
/*------------------------------Type Definitions------------------------------*/
/******************************************************************************/

typedef struct
{
    float32 sysFreq;                /**< \brief Actual SPB frequency */
    float32 cpuFreq;                /**< \brief Actual CPU frequency */
    float32 pllFreq;                /**< \brief Actual PLL frequency */
    float32 stmFreq;                /**< \brief Actual STM frequency */
} AppInfo;

/** \brief Application information */
typedef struct
{
    AppInfo info;                               /**< \brief Info object */
} App_Cpu0;

/******************************************************************************/
/*------------------------------Global variables------------------------------*/
/******************************************************************************/

IFX_EXTERN App_Cpu0 g_AppCpu0;

#endif```
###cpu1_main.c
/*********************************************************************************************************************
* TC387 Opensourec Library 即（TC387 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC387 开源库的一部分
*
* TC387 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          cpu1_main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.10.2
* 适用平台          TC387QP
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-11-04       pudding            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#pragma section all "cpu1_dsram"
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU1的RAM中


// 工程导入到软件之后，应该选中工程然后点击refresh刷新一下之后再编译
// 工程默认设置为关闭优化，可以自己右击工程选择properties->C/C++ Build->Setting
// 然后在右侧的窗口中找到C/C++ Compiler->Optimization->Optimization level处设置优化等级
// 一般默认新建立的工程都会默认开2级优化，因此大家也可以设置为2级优化

// 对于TC系列默认是不支持中断嵌套的，希望支持中断嵌套需要在中断内使用 enableInterrupts(); 来开启中断嵌套
// 简单点说实际上进入中断后TC系列的硬件自动调用了 disableInterrupts(); 来拒绝响应任何的中断，因此需要我们自己手动调用 enableInterrupts(); 来开启中断的响应。


// **************************** 代码区域 ****************************
#define INCLUDE_BOUNDARY_TYPE   0


#define WIFI_SSID_TEST          "l"
#define WIFI_PASSWORD_TEST      "11112222"  // 如果需要连接的WIFI 没有密码则需要将 这里 替换为 NULL





// 边界的点数量远大于图像高度，便于保存回弯的情况
#define BOUNDARY_NUM            (MT9V03X_H * 3 / 2)

// 只有X边界
uint8 xy_x1_boundary[BOUNDARY_NUM], xy_x2_boundary[BOUNDARY_NUM], xy_x3_boundary[BOUNDARY_NUM];

// 只有Y边界
uint8 xy_y1_boundary[BOUNDARY_NUM], xy_y2_boundary[BOUNDARY_NUM], xy_y3_boundary[BOUNDARY_NUM];

// X Y边界都是单独指定的
uint8 x1_boundary[MT9V03X_H], x2_boundary[MT9V03X_H], x3_boundary[MT9V03X_H];
uint8 y1_boundary[MT9V03X_W], y2_boundary[MT9V03X_W], y3_boundary[MT9V03X_W];

// 图像备份数组，在发送前将图像备份再进行发送，这样可以避免图像出现撕裂的问题



void core1_main(void)
{

    disable_Watchdog();                     // 关闭看门狗
    interrupt_global_enable(0);             // 打开全局中断
//    #if(0 != INCLUDE_BOUNDARY_TYPE)
//        int32 i = 0;
//    #elif(3 == INCLUDE_BOUNDARY_TYPE)
//        int32 j = 0;
//    #endif
//
//        // 此处编写用户代码 例如外设初始化代码等
//        while(wifi_spi_init(WIFI_SSID_TEST, WIFI_PASSWORD_TEST))
//         {
//             printf("\r\n connect wifi failed. \r\n");
//             system_delay_ms(100);                                                   // 初始化失败 等待 100ms
//         }
//
//         printf("\r\n module version:%s",wifi_spi_version);                          // 模块固件版本
//         printf("\r\n module mac    :%s",wifi_spi_mac_addr);                         // 模块 MAC 信息
//         printf("\r\n module ip     :%s",wifi_spi_ip_addr_port);                     // 模块 IP 地址
//
//         // zf_device_wifi_spi.h 文件内的宏定义可以更改模块连接(建立) WIFI 之后，是否自动连接 TCP 服务器、创建 UDP 连接
//         if(1 != WIFI_SPI_AUTO_CONNECT)                                              // 如果没有开启自动连接 就需要手动连接目标 IP
//         {
//             while(wifi_spi_socket_connect(                                          // 向指定目标 IP 的端口建立 TCP 连接
//                 "TCP",                                                              // 指定使用TCP方式通讯
//                 WIFI_SPI_TARGET_IP,                                                 // 指定远端的IP地址，填写上位机的IP地址
//                 WIFI_SPI_TARGET_PORT,                                               // 指定远端的端口号，填写上位机的端口号，通常上位机默认是8080
//                 WIFI_SPI_LOCAL_PORT))                                               // 指定本机的端口号
//             {
//                 // 如果一直建立失败 考虑一下是不是没有接硬件复位
//                 printf("\r\n Connect TCP Servers error, try again.");
//                 system_delay_ms(100);                                               // 建立连接失败 等待 100ms
//             }
//         }
//
//         // 推荐先初始化摄像头，后初始化逐飞助手
//
//
//         // 逐飞助手初始化 数据传输使用高速WIFI SPI
//         seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
//
//         // 如果要发送图像信息，则务必调用seekfree_assistant_camera_information_config函数进行必要的参数设置
//         // 如果需要发送边线则还需调用seekfree_assistant_camera_boundary_config函数设置边线的信息
//
//     #if(0 == INCLUDE_BOUNDARY_TYPE)
//         // 发送总钻风图像信息(仅包含原始图像信息)
//         seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, image_copy[0], MT9V03X_W, MT9V03X_H);
//
//
//
//     #endif

    cpu_wait_event_ready();                 // 等待所有核心初始化完毕
    while (TRUE)
    {
        // 此处编写需要循环执行的代码
        if(mt9v03x_finish_flag)
         {
             mt9v03x_finish_flag = 0;
             memcpy(image_copy[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);
//                           seekfree_assistant_camera_send();
             system_start();

             Image_Process();
             Image_Time = system_getval();
         }

        // 此处编写需要循环执行的代码
    }
}
#pragma section all restore```
###cpu2_main.c
```/*********************************************************************************************************************
* TC387 Opensourec Library 即（TC387 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC387 开源库的一部分
*
* TC387 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          cpu2_main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.10.2
* 适用平台          TC387QP
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-11-04       pudding            first version
********************************************************************************************************************/
#include "zf_common_headfile.h"
#pragma section all "cpu2_dsram"
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU1的RAM中


// 工程导入到软件之后，应该选中工程然后点击refresh刷新一下之后再编译
// 工程默认设置为关闭优化，可以自己右击工程选择properties->C/C++ Build->Setting
// 然后在右侧的窗口中找到C/C++ Compiler->Optimization->Optimization level处设置优化等级
// 一般默认新建立的工程都会默认开2级优化，因此大家也可以设置为2级优化

// 对于TC系列默认是不支持中断嵌套的，希望支持中断嵌套需要在中断内使用 enableInterrupts(); 来开启中断嵌套
// 简单点说实际上进入中断后TC系列的硬件自动调用了 disableInterrupts(); 来拒绝响应任何的中断，因此需要我们自己手动调用 enableInterrupts(); 来开启中断的响应。


// **************************** 代码区域 ****************************

void core2_main(void)
{
    disable_Watchdog();                     // 关闭看门狗
    interrupt_global_enable(0);             // 打开全局中断
    // 此处编写用户代码 例如外设初始化代码等

    // 此处编写用户代码 例如外设初始化代码等
    cpu_wait_event_ready();                // 等待所有核心初始化完毕
    while (TRUE)
    {
        // 此处编写需要循环执行的代码
        ips200_show_gray_image(0, 0, image_copy[0], MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, 0);
        int aim_idx = clip(round(aim_distance / sample_dist), 0, rptsn_num - 1);
        for(uint8 ii = 0;ii < rptsn_num; ii ++)
        {
            ips200_draw_point((int)Limiter_float(rptsn[ii][0], 0, 187), (int)Limiter_float(rptsn[ii][1], 0, 119), RGB565_RED);
        }
        for(uint8 ii = 0;ii < 187; ii ++)
        {
            ips200_draw_point(ii, (int)rptsn[aim_idx][1], RGB565_YELLOW);
        }
        for(uint8 ii = 0;ii < 187; ii ++)
        {
            ips200_draw_point(ii, rptsn[55][1], RGB565_BLUE);
        }


        ips200_show_string(8 * 0,16 * 8,"Pitch:");
        ips200_show_float( 8 * 10,16 * 8,QEKF_INS.Pitch,3,3);//pitch
        ips200_show_string(8 * 0,16 * 9,"Roll:");
        ips200_show_float( 8 * 10,16 * 9,QEKF_INS.Roll,3,3);//roll
        ips200_show_string(8 * 0,16 * 10,"Yaw:");
        ips200_show_float( 8 * 10,16 * 10,QEKF_INS.Yaw,3,3);//yaw
        ips200_show_string(8 * 0,16 * 11,"gyro_y:");
        ips200_show_int(   8 * 10,16 * 11,imu660ra_gyro_y,7);
        ips200_show_string(8 * 0,16 * 12,"high:");
        ips200_show_int(   8 * 15,16 * 12,high,7);
        ips200_show_string(8 * 0,16 * 13,"rpts0s:");
        ips200_show_int(   8 * 10,16 * 13,rpts0s_num,7);
        ips200_show_string(8 * 0,16 * 14,"rpts1a:");
        ips200_show_int(   8 * 15,16 * 14,rpts1s_num,7);
        ips200_show_string(8 * 0,16 * 15,"Target_Speed:");
        ips200_show_int(   8 * 15,16 * 15,Target_Speed,5);
        ips200_show_string(8 * 0,16 * 16,"r:");
        ips200_show_float(   8 * 15,16 * 16,radius_of_curvature,3,3);
        ips200_show_string(8 * 0,16 * 17,"ERROR:");  ////40
        ips200_show_float(8 * 20,16 * 17,ERROR,4,3);
        ips200_show_string(8 * 0,16 * 18,"pure_angle:");  ////40
        ips200_show_float(8 * 15,16 * 18,pure_angle,4,3);
//        ips200_show_string(8 * 0,16 * 19,"Image_Time:");
//        ips200_show_uint(8 * 15,16 * 19,Image_Time,8);

        key_scanner();
        if(key_get_state(KEY_1) == 1)
        {
            high += 1;
        }
        else if(key_get_state(KEY_2) == 1)
        {

//            high += 1;
//            up_leg += 0.2;
//            leg_angle_limiting ++;
            up_leg += 1;
        }
        else if(key_get_state(KEY_3) == 1)
        {
//            go_single_side_bridge = true ;
            tiao = 1;
//            high += 1;
        }
        else if(key_get_state(KEY_4) == 1)
        {
//            single_bridge_flag = 40;
//            high += 0.5;
            track_flag = 1;
            run_flag ++;
            if(run_flag == 2)
                run_flag = 0;
        }

        key_clear_all_state();


        // 此处编写需要循环执行的代码
//        MENU_Page();
    }
}



#pragma section all restore```
###cpu3_main.c
```/*********************************************************************************************************************
* TC387 Opensourec Library 即（TC387 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC387 开源库的一部分
*
* TC387 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          cpu2_main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.10.2
* 适用平台          TC387QP
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-11-04       pudding            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#pragma section all "cpu3_dsram"
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU1的RAM中


// 工程导入到软件之后，应该选中工程然后点击refresh刷新一下之后再编译
// 工程默认设置为关闭优化，可以自己右击工程选择properties->C/C++ Build->Setting
// 然后在右侧的窗口中找到C/C++ Compiler->Optimization->Optimization level处设置优化等级
// 一般默认新建立的工程都会默认开2级优化，因此大家也可以设置为2级优化

// 对于TC系列默认是不支持中断嵌套的，希望支持中断嵌套需要在中断内使用 enableInterrupts(); 来开启中断嵌套
// 简单点说实际上进入中断后TC系列的硬件自动调用了 disableInterrupts(); 来拒绝响应任何的中断，因此需要我们自己手动调用 enableInterrupts(); 来开启中断的响应。


// **************************** 代码区域 ****************************


void core3_main(void)
{

    disable_Watchdog();                     // 关闭看门狗
    interrupt_global_enable(0);             // 打开全局中断
    // 此处编写用户代码 例如外设初始化代码等





    // 此处编写用户代码 例如外设初始化代码等
    cpu_wait_event_ready();                 // 等待所有核心初始化完毕
    while (TRUE)
    {
        // 此处编写需要循环执行的代码
        printf("%d, %d\r\n",Current_Speed,Target_Speed);
//        if(ALL_ERROR != 1.0){
//            printf("%d\r\n",Current_Speed);
//        }
//        image_thereshold = otsuThreshold2(Imgdata[0], image_w, image_h);
        // 此处编写需要循环执行的代码
    }
}



#pragma section all restore```
###isr.c
```/*********************************************************************************************************************
* TC387 Opensourec Library 即（TC387 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC387 开源库的一部分
*
* TC387 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          isr
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.10.2
* 适用平台          TC387QP
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-11-04       pudding            first version
********************************************************************************************************************/

#include "isr_config.h"
#include "isr.h"

// 对于TC系列默认是不支持中断嵌套的，希望支持中断嵌套需要在中断内使用 interrupt_global_enable(0); 来开启中断嵌套
// 简单点说实际上进入中断后TC系列的硬件自动调用了 interrupt_global_disable(); 来拒绝响应任何的中断，因此需要我们自己手动调用 interrupt_global_enable(0); 来开启中断的响应。

// **************************** PIT中断函数 ****************************
IFX_INTERRUPT(cc60_pit_ch0_isr, CCU6_0_CH0_INT_VECTAB_NUM, CCU6_0_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    pit_clear_flag(CCU60_CH0);
    Attitude_Calculate();
    Balance();

//    Steer_control();
}


IFX_INTERRUPT(cc60_pit_ch1_isr, CCU6_0_CH1_INT_VECTAB_NUM, CCU6_0_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    pit_clear_flag(CCU60_CH1);


}

IFX_INTERRUPT(cc61_pit_ch0_isr, CCU6_1_CH0_INT_VECTAB_NUM, CCU6_1_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    pit_clear_flag(CCU61_CH0);

    button_ticks();


}

IFX_INTERRUPT(cc61_pit_ch1_isr, CCU6_1_CH1_INT_VECTAB_NUM, CCU6_1_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    pit_clear_flag(CCU61_CH1);





}
// **************************** PIT中断函数 ****************************


// **************************** 外部中断函数 ****************************
IFX_INTERRUPT(exti_ch0_ch4_isr, EXTI_CH0_CH4_INT_VECTAB_NUM, EXTI_CH0_CH4_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    if(exti_flag_get(ERU_CH0_REQ0_P15_4))           // 通道0中断
    {
        exti_flag_clear(ERU_CH0_REQ0_P15_4);



    }

    if(exti_flag_get(ERU_CH4_REQ13_P15_5))          // 通道4中断
    {
        exti_flag_clear(ERU_CH4_REQ13_P15_5);




    }
}

IFX_INTERRUPT(exti_ch1_ch5_isr, EXTI_CH1_CH5_INT_VECTAB_NUM, EXTI_CH1_CH5_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套

    if(exti_flag_get(ERU_CH1_REQ10_P14_3))          // 通道1中断
    {
        exti_flag_clear(ERU_CH1_REQ10_P14_3);

        tof_module_exti_handler();                  // ToF 模块 INT 更新中断

    }

    if(exti_flag_get(ERU_CH5_REQ1_P15_8))           // 通道5中断
    {
        exti_flag_clear(ERU_CH5_REQ1_P15_8);



    }
}

// 由于摄像头pclk引脚默认占用了 2通道，用于触发DMA，因此这里不再定义中断函数
// IFX_INTERRUPT(exti_ch2_ch6_isr, EXTI_CH2_CH6_INT_VECTAB_NUM, EXTI_CH2_CH6_INT_PRIO)
// {
//  interrupt_global_enable(0);                     // 开启中断嵌套
//  if(exti_flag_get(ERU_CH2_REQ7_P00_4))           // 通道2中断
//  {
//      exti_flag_clear(ERU_CH2_REQ7_P00_4);
//  }
//  if(exti_flag_get(ERU_CH6_REQ9_P20_0))           // 通道6中断
//  {
//      exti_flag_clear(ERU_CH6_REQ9_P20_0);
//  }
// }

IFX_INTERRUPT(exti_ch3_ch7_isr, EXTI_CH3_CH7_INT_VECTAB_NUM, EXTI_CH3_CH7_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    if(exti_flag_get(ERU_CH3_REQ6_P02_0))           // 通道3中断
    {
        exti_flag_clear(ERU_CH3_REQ6_P02_0);
        camera_vsync_handler();                     // 摄像头触发采集统一回调函数
    }
    if(exti_flag_get(ERU_CH7_REQ16_P15_1))          // 通道7中断
    {
        exti_flag_clear(ERU_CH7_REQ16_P15_1);




    }
}
// **************************** 外部中断函数 ****************************


// **************************** DMA中断函数 ****************************
IFX_INTERRUPT(dma_ch5_isr, DMA_INT_VECTAB_NUM, DMA_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    camera_dma_handler();                           // 摄像头采集完成统一回调函数
}
// **************************** DMA中断函数 ****************************


// **************************** 串口中断函数 ****************************
// 串口0默认作为调试串口
IFX_INTERRUPT(uart0_tx_isr, UART0_INT_VECTAB_NUM, UART0_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}
IFX_INTERRUPT(uart0_rx_isr, UART0_INT_VECTAB_NUM, UART0_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套

#if DEBUG_UART_USE_INTERRUPT                        // 如果开启 debug 串口中断
        debug_interrupr_handler();                  // 调用 debug 串口接收处理函数 数据会被 debug 环形缓冲区读取
#endif                                              // 如果修改了 DEBUG_UART_INDEX 那这段代码需要放到对应的串口中断去
}


// 串口1默认连接到摄像头配置串口
IFX_INTERRUPT(uart1_tx_isr, UART1_INT_VECTAB_NUM, UART1_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套




}
IFX_INTERRUPT(uart1_rx_isr, UART1_INT_VECTAB_NUM, UART1_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    camera_uart_handler();                          // 摄像头参数配置统一回调函数
}

// 串口2默认连接到无线转串口模块
IFX_INTERRUPT(uart2_tx_isr, UART2_INT_VECTAB_NUM, UART2_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart2_rx_isr, UART2_INT_VECTAB_NUM, UART2_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    wireless_module_uart_handler();                 // 无线模块统一回调函数
    Receive_Interrupt_Processing();
//    uart2_control_callback();
//    Process_Serial_Packet();


}
// 串口3默认连接到GPS定位模块
IFX_INTERRUPT(uart3_tx_isr, UART3_INT_VECTAB_NUM, UART3_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart3_rx_isr, UART3_INT_VECTAB_NUM, UART3_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
//    gnss_uart_callback();                           // GNSS串口回调函数
    uart_control_callback();


}


IFX_INTERRUPT(uart4_tx_isr, UART4_INT_VECTAB_NUM, UART4_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart4_rx_isr, UART4_INT_VECTAB_NUM, UART4_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart5_tx_isr, UART5_INT_VECTAB_NUM, UART5_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart5_rx_isr, UART5_INT_VECTAB_NUM, UART5_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart6_tx_isr, UART6_INT_VECTAB_NUM, UART6_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart6_rx_isr, UART6_INT_VECTAB_NUM, UART6_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart8_tx_isr, UART8_INT_VECTAB_NUM, UART8_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart8_rx_isr, UART8_INT_VECTAB_NUM, UART8_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
//    uart8_control_callback();


}

IFX_INTERRUPT(uart9_tx_isr, UART9_INT_VECTAB_NUM, UART9_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart9_rx_isr, UART9_INT_VECTAB_NUM, UART9_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart10_tx_isr, UART10_INT_VECTAB_NUM, UART10_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart10_rx_isr, UART10_INT_VECTAB_NUM, UART10_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart11_tx_isr, UART11_INT_VECTAB_NUM, UART11_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart11_rx_isr, UART11_INT_VECTAB_NUM, UART11_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}
// 串口通讯错误中断
IFX_INTERRUPT(uart0_er_isr, UART0_INT_VECTAB_NUM, UART0_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart0_handle);
}
IFX_INTERRUPT(uart1_er_isr, UART1_INT_VECTAB_NUM, UART1_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart1_handle);
}
IFX_INTERRUPT(uart2_er_isr, UART2_INT_VECTAB_NUM, UART2_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart2_handle);
}
IFX_INTERRUPT(uart3_er_isr, UART3_INT_VECTAB_NUM, UART3_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart3_handle);
}
IFX_INTERRUPT(uart4_er_isr, UART4_INT_VECTAB_NUM, UART4_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart4_handle);
}
IFX_INTERRUPT(uart5_er_isr, UART5_INT_VECTAB_NUM, UART5_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart5_handle);
}
IFX_INTERRUPT(uart6_er_isr, UART6_INT_VECTAB_NUM, UART6_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart6_handle);
}
IFX_INTERRUPT(uart8_er_isr, UART8_INT_VECTAB_NUM, UART8_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart8_handle);
}
IFX_INTERRUPT(uart9_er_isr, UART9_INT_VECTAB_NUM, UART9_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart9_handle);
}
IFX_INTERRUPT(uart10_er_isr, UART10_INT_VECTAB_NUM, UART10_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart10_handle);
}
IFX_INTERRUPT(uart11_er_isr, UART11_INT_VECTAB_NUM, UART11_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart11_handle);
}
// **************************** 串口中断函数 ****************************```
###isr.h
```/*********************************************************************************************************************
* TC387 Opensourec Library 即（TC387 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC387 开源库的一部分
*
* TC387 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          isr
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.10.2
* 适用平台          TC387QP
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-11-04       pudding            first version
********************************************************************************************************************/


#ifndef _isr_h
#define _isr_h

#include "zf_common_headfile.h"











#endif```
###isr_config.h
```/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file            isr_config
 * @company         成都逐飞科技有限公司
 * @author          逐飞科技(QQ3184284598)
 * @version         查看doc内version文件 版本说明
 * @Software        ADS v1.2.2
 * @Target core     TC387QP
 * @Taobao          https://seekfree.taobao.com/
 * @date            2020-12-11
 ********************************************************************************************************************/

#ifndef _isr_config_h
#define _isr_config_h



//======================================================特别注意====================================================
// 中断优先级不能设置为相同值，所有中断优先级都必须设置为不一样的值
//======================================================特别注意====================================================
//======================================================特别注意====================================================
// 中断优先级不能设置为相同值，所有中断优先级都必须设置为不一样的值
//======================================================特别注意====================================================
//======================================================特别注意====================================================
// 中断优先级不能设置为相同值，所有中断优先级都必须设置为不一样的值
//======================================================特别注意====================================================

//ISR_PRIORITY：   TC387具有255个中断优先级可以设置 1-255，0优先级表示不开启中断，255为最高优先级
//INT_SERVICE：    宏定义决定中断由谁处理，也称为服务提供者（在TC387中，中断被叫做服务），可设置范围IfxSrc_Tos_cpu0 IfxSrc_Tos_cpu1 IfxSrc_Tos_cpu2 IfxSrc_Tos_cpu3 IfxSrc_Tos_dma  不可设置为其他值


//如果INT_SERVICE设置为IfxSrc_Tos_dma的话，ISR_PRIORITY的可设置范围则是0-127。

//================================================PIT中断参数相关定义===============================================
#define CCU6_0_CH0_INT_SERVICE  IfxSrc_Tos_cpu0     // 定义CCU6_0 PIT通道0中断服务类型，即中断是由谁响应处理 IfxSrc_Tos_cpu0 IfxSrc_Tos_cpu1 IfxSrc_Tos_dma  不可设置为其他值
#define CCU6_0_CH0_ISR_PRIORITY 50                  // 定义CCU6_0 PIT通道0中断优先级 优先级范围1-255 越大优先级越高 与平时使用的单片机不一样

#define CCU6_0_CH1_INT_SERVICE  IfxSrc_Tos_cpu0
#define CCU6_0_CH1_ISR_PRIORITY 51

#define CCU6_1_CH0_INT_SERVICE  IfxSrc_Tos_cpu2
#define CCU6_1_CH0_ISR_PRIORITY 52

#define CCU6_1_CH1_INT_SERVICE  IfxSrc_Tos_cpu0
#define CCU6_1_CH1_ISR_PRIORITY 53



//================================================GPIO中断参数相关定义===============================================
// 通道0与通道4是公用一个中断函数 在中断内部通过标志位判断是谁触发的中断
#define EXTI_CH0_CH4_INT_SERVICE IfxSrc_Tos_cpu0    // 定义ERU通道0和通道4中断服务类型，即中断是由谁响应处理 IfxSrc_Tos_cpu0 IfxSrc_Tos_cpu1 IfxSrc_Tos_dma  不可设置为其他值
#define EXTI_CH0_CH4_INT_PRIO   60                  // 定义ERU通道0和通道4中断优先级 优先级范围1-255 越大优先级越高 与平时使用的单片机不一样

// 通道1与通道5是公用一个中断函数 在中断内部通过标志位 判断是谁触发的中断
#define EXTI_CH1_CH5_INT_SERVICE IfxSrc_Tos_cpu0    // 定义ERU通道1和通道5中断服务类型，同上
#define EXTI_CH1_CH5_INT_PRIO   61                  // 定义ERU通道1和通道5中断优先级 同上

// 通道2与通道6是公用一个中断函数 在中断内部通过标志位 判断是谁触发的中断
#define EXTI_CH2_CH6_INT_SERVICE IfxSrc_Tos_dma     // 定义ERU通道2和通道6中断服务类型，同上
#define EXTI_CH2_CH6_INT_PRIO   5                   // 定义ERU通道2和通道6中断优先级 可设置范围为0-127(DMA响应)

// 通道3与通道7是公用一个中断函数 在中断内部通过标志位 判断是谁触发的中断
#define EXTI_CH3_CH7_INT_SERVICE IfxSrc_Tos_cpu1    // 定义ERU通道3和通道7中断服务类型，同上
#define EXTI_CH3_CH7_INT_PRIO   62                  // 定义ERU通道3和通道7中断优先级 同上


//===================================================DMA中断参数相关定义===============================================
#define DMA_INT_SERVICE         IfxSrc_Tos_cpu1     // ERU触发DMA中断服务类型，即中断是由谁响应处理 IfxSrc_Tos_cpu0 IfxSrc_Tos_cpu1 IfxSrc_Tos_dma  不可设置为其他值
#define DMA_INT_PRIO            70                  // ERU触发DMA中断优先级 优先级范围1-255 越大优先级越高 与平时使用的单片机不一样


//===================================================串口中断参数相关定义===============================================
#define UART0_INT_SERVICE       IfxSrc_Tos_cpu0     // 定义串口0中断服务类型，即中断是由谁响应处理 IfxSrc_Tos_cpu0 IfxSrc_Tos_cpu1 IfxSrc_Tos_dma  不可设置为其他值
#define UART0_TX_INT_PRIO       11                  // 定义串口0发送中断优先级 优先级范围1-255 越大优先级越高 与平时使用的单片机不一样
#define UART0_RX_INT_PRIO       10                  // 定义串口0接收中断优先级 优先级范围1-255 越大优先级越高 与平时使用的单片机不一样
#define UART0_ER_INT_PRIO       12                  // 定义串口0错误中断优先级 优先级范围1-255 越大优先级越高 与平时使用的单片机不一样

#define UART1_INT_SERVICE       IfxSrc_Tos_cpu0
#define UART1_TX_INT_PRIO       13
#define UART1_RX_INT_PRIO       14
#define UART1_ER_INT_PRIO       15

#define UART2_INT_SERVICE       IfxSrc_Tos_cpu0
#define UART2_TX_INT_PRIO       16
#define UART2_RX_INT_PRIO       17
#define UART2_ER_INT_PRIO       18

#define UART3_INT_SERVICE       IfxSrc_Tos_cpu3
#define UART3_TX_INT_PRIO       19
#define UART3_RX_INT_PRIO       20
#define UART3_ER_INT_PRIO       21

#define UART4_INT_SERVICE       IfxSrc_Tos_cpu0
#define UART4_TX_INT_PRIO       22
#define UART4_RX_INT_PRIO       23
#define UART4_ER_INT_PRIO       24

#define UART5_INT_SERVICE       IfxSrc_Tos_cpu0
#define UART5_TX_INT_PRIO       25
#define UART5_RX_INT_PRIO       26
#define UART5_ER_INT_PRIO       27

#define UART6_INT_SERVICE       IfxSrc_Tos_cpu0
#define UART6_TX_INT_PRIO       28
#define UART6_RX_INT_PRIO       29
#define UART6_ER_INT_PRIO       30

#define UART8_INT_SERVICE       IfxSrc_Tos_cpu0
#define UART8_TX_INT_PRIO       31
#define UART8_RX_INT_PRIO       32
#define UART8_ER_INT_PRIO       33

#define UART9_INT_SERVICE       IfxSrc_Tos_cpu0
#define UART9_TX_INT_PRIO       34
#define UART9_RX_INT_PRIO       35
#define UART9_ER_INT_PRIO       36

#define UART10_INT_SERVICE       IfxSrc_Tos_cpu0
#define UART10_TX_INT_PRIO       37
#define UART10_RX_INT_PRIO       38
#define UART10_ER_INT_PRIO       39

#define UART11_INT_SERVICE       IfxSrc_Tos_cpu0
#define UART11_TX_INT_PRIO       40
#define UART11_RX_INT_PRIO       41
#define UART11_ER_INT_PRIO       42







//------------中断向量表选择（不允许修改）------------
#define CCU6_0_CH0_INT_VECTAB_NUM    (int)CCU6_0_CH0_INT_SERVICE      > 0 ? (int)CCU6_0_CH0_INT_SERVICE    - 1 : (int)CCU6_0_CH0_INT_SERVICE
#define CCU6_0_CH1_INT_VECTAB_NUM    (int)CCU6_0_CH1_INT_SERVICE      > 0 ? (int)CCU6_0_CH1_INT_SERVICE    - 1 : (int)CCU6_0_CH1_INT_SERVICE
#define CCU6_1_CH0_INT_VECTAB_NUM    (int)CCU6_1_CH0_INT_SERVICE      > 0 ? (int)CCU6_1_CH0_INT_SERVICE    - 1 : (int)CCU6_1_CH0_INT_SERVICE
#define CCU6_1_CH1_INT_VECTAB_NUM    (int)CCU6_1_CH1_INT_SERVICE      > 0 ? (int)CCU6_1_CH1_INT_SERVICE    - 1 : (int)CCU6_1_CH1_INT_SERVICE

#define EXTI_CH0_CH4_INT_VECTAB_NUM  (int)EXTI_CH0_CH4_INT_SERVICE    > 0 ? (int)EXTI_CH0_CH4_INT_SERVICE  - 1 : (int)EXTI_CH0_CH4_INT_SERVICE
#define EXTI_CH1_CH5_INT_VECTAB_NUM  (int)EXTI_CH1_CH5_INT_SERVICE    > 0 ? (int)EXTI_CH1_CH5_INT_SERVICE  - 1 : (int)EXTI_CH1_CH5_INT_SERVICE
#define EXTI_CH2_CH6_INT_VECTAB_NUM  (int)EXTI_CH2_CH6_INT_SERVICE    > 0 ? (int)EXTI_CH2_CH6_INT_SERVICE  - 1 : (int)EXTI_CH2_CH6_INT_SERVICE
#define EXTI_CH3_CH7_INT_VECTAB_NUM  (int)EXTI_CH3_CH7_INT_SERVICE    > 0 ? (int)EXTI_CH3_CH7_INT_SERVICE  - 1 : (int)EXTI_CH3_CH7_INT_SERVICE

#define DMA_INT_VECTAB_NUM           (int)DMA_INT_SERVICE             > 0 ? (int)DMA_INT_SERVICE           - 1 : (int)DMA_INT_SERVICE

#define UART0_INT_VECTAB_NUM         (int)UART0_INT_SERVICE           > 0 ? (int)UART0_INT_SERVICE         - 1 : (int)UART0_INT_SERVICE
#define UART1_INT_VECTAB_NUM         (int)UART1_INT_SERVICE           > 0 ? (int)UART1_INT_SERVICE         - 1 : (int)UART1_INT_SERVICE
#define UART2_INT_VECTAB_NUM         (int)UART2_INT_SERVICE           > 0 ? (int)UART2_INT_SERVICE         - 1 : (int)UART2_INT_SERVICE
#define UART3_INT_VECTAB_NUM         (int)UART3_INT_SERVICE           > 0 ? (int)UART3_INT_SERVICE         - 1 : (int)UART3_INT_SERVICE
#define UART4_INT_VECTAB_NUM         (int)UART4_INT_SERVICE           > 0 ? (int)UART4_INT_SERVICE         - 1 : (int)UART4_INT_SERVICE
#define UART5_INT_VECTAB_NUM         (int)UART5_INT_SERVICE           > 0 ? (int)UART5_INT_SERVICE         - 1 : (int)UART5_INT_SERVICE
#define UART6_INT_VECTAB_NUM         (int)UART6_INT_SERVICE           > 0 ? (int)UART6_INT_SERVICE         - 1 : (int)UART6_INT_SERVICE
#define UART8_INT_VECTAB_NUM         (int)UART8_INT_SERVICE           > 0 ? (int)UART8_INT_SERVICE         - 1 : (int)UART8_INT_SERVICE
#define UART9_INT_VECTAB_NUM         (int)UART9_INT_SERVICE           > 0 ? (int)UART9_INT_SERVICE         - 1 : (int)UART9_INT_SERVICE
#define UART10_INT_VECTAB_NUM        (int)UART10_INT_SERVICE          > 0 ? (int)UART10_INT_SERVICE        - 1 : (int)UART10_INT_SERVICE
#define UART11_INT_VECTAB_NUM        (int)UART11_INT_SERVICE          > 0 ? (int)UART11_INT_SERVICE        - 1 : (int)UART11_INT_SERVICE

#endif```
```PID.C
/*
 * PID_control.c
 *
 *  Created on: 2025年1月15日
 *      Author: 张
 */

#include "PID_control.h"

/*!
  * @brief    限幅函数
  *
  * @param    amt   ： 参数
  * @param    low   ： 最低值
  * @param    high  ： 最高值
  *
  * @return   无
  *
  * @note     无
  *
  * @see      无
  *
  * @date     2025/1/15
  */

float Gyro_Y_Kp =  1.3, Gyro_Y_Ki = 0.014, Gyro_Y_Kd =     0;
float Pitch_Kp  =  285, Pitch_Ki  =     0, Pitch_Kd  = 0.005;
float Speed_Kp  = 0.12, Speed_Ki  =  0.00, Speed_kd  =     0;
float Turn_Kp1  = 30.f, Turn_Kp2  = 0.7f, Turn_Kd1   =  100.f, Turn_Kd2 = -0.30f;
//float Turn_Kp1  = 30.f, Turn_Kp2  = 0.7f, Turn_Kd1   =  0, Turn_Kd2 = -0.30f;



int int_min(int a, int b){
    if(a < b)
        return a;
    else
        return b;
}
float absValue(float num)
{
    if (num >= 0)
        return num;
     else
        return -num;
}


float Limiter_float(float aim, float low, float high)
{
    if(aim >= high)
        return high;
    else if(aim <= low)
        return low;
    else
        return aim;
}


// 初始化滤波器
void LowPassFilter_Init(LowPassFilter* filter, float alpha) {
    filter->alpha = alpha;
    filter->prev_output = 0.0f;
}

// 更新滤波器
float LowPassFilter_Update(LowPassFilter* filter, float input) {
    // 计算当前输出
    float output = filter->alpha * input + (1.0f - filter->alpha) * filter->prev_output;
    // 更新上一次的输出值
    filter->prev_output = output;
    return output;
}

// 初始化低通滤波器
LowPassFilter Filter_Gyro_Y,
              Filter_Gyro_Z,
              Filter_Speed_Output,
              Filter_Bend_Output;

PID_INCREMENTAL Control_Gyro_Y;
PID_PLACE Control_Pitch;
PID_PLACE Control_Speed;                 // 速度环
PID_STEER Control_Turn;                  // 转向
PID_STEER Control_Yaw;                   // 偏航角辅助
PID_INCREMENTAL Control_Roll;            // 翻滚角
PID_PLACE Control_bend;                  // 转向向心力补偿
PID_PLACE Control_Target_Speed;          // 速度规划

// 初始化角度解缠（unwrap）方法
YawUnwrapperState state = {
    .is_initialized = false,
    .prev_yaw = 0,
    .offset = 0
};

float Body_High = 0.0f;
float ERROR = 0;
float ALL_ERROR = 0;
int8 Speed_Compensation = 0;


////////////////////////////////////////////////////
int16 Target_Speed = 0, Current_Speed = 0;
// 编码器积分
int32 Enc_sum    = 0;
uint8 Enc_flag   = 0;


uint8 gyroscope_flag = 0;     // 偏航角辅助
uint8 track_flag     = 0;     // 图像偏差转向
uint8 run_flag       = 0;     // 开始跑
uint8 jump_flag      = 0;     // 开始跳
uint32 Time          = 0;
uint32 jump_time     = 0;


void Dynamic_Value_Control(void)
{
//    if(Time % 4000 == 0)
//    {
//        jump_flag = 1;
//    }
    if(tiao == 1){
        jump_flag = 1;
        gpio_set_level(P33_10, 1);
        tiao = 0;
    }
    else
        gpio_set_level(P33_10, 0);
    if(jump_flag)
    {
        Control_Gyro_Y.Kp = Gyro_Y_Kp / 2;
        Control_Gyro_Y.Ki = Gyro_Y_Ki / 2;
        Control_Pitch.Kp = Pitch_Kp / 2;
    }
    else
    {
        Control_Gyro_Y.Kp = Gyro_Y_Kp;
        Control_Gyro_Y.Ki = Gyro_Y_Ki;
        Control_Pitch.Kp = Pitch_Kp;
        if(single_side_bridge_type != SINGLE_SIDE_BRIDGE_NONE
                || traverse_type != TRAVERSE_NONE
                || cross_type != CROSS_NONE)  {
            Control_Turn.Kp2 = Turn_Kp2;
            Control_Turn.Kd1 = Turn_Kd1;
        }
        else
            Control_Turn.Kp2 = MINMAX(Turn_Kp2 + 0.2 * radius_of_curvature, 0.4, 0.8);
            Control_Turn.Kd1 = MINMAX(Turn_Kd1 + 50 * radius_of_curvature, 0, 100.f);

    }

}
float Target_Yaw = 0;


int16 pwm_ph1_l, pwm_ph4_l;
int16 pwm_ph1_r, pwm_ph4_r;

float leg_angle_limiting = 30;     // 速度环输出限幅
float left_high, right_high;
float high = 4.0f;

// 俯仰角、翻滚角机械零点
float mechanical_zero_of_pitch = Mechanical_Zero_Of_Ritch,
      mechanical_zero_of_roll  = Mechanical_Zero_Of_Roll;

// PID输出
float Control_Speed_Output  = 0;
float Turn_Output           = 0;
float Control_Yaw_Output    = 0;
float Control_Pitch_Output  = 0;
float Control_Roll_Output   = 0;
float Control_Gyro_Y_Output = 0;
float Control_Bend_Output   = 0;
float Control_Speed_Pitch_Output = 0;    // 单边桥减速补偿输出

int8 single_bridge_flag = 0;

float up_leg = 0;
void Motor_Control(){
    // 输出给电机
    int16 left_duty = (int16)(- Control_Gyro_Y_Output + Turn_Output + Control_Yaw_Output);
    int16 right_duty = (int16)( Control_Gyro_Y_Output + Turn_Output + Control_Yaw_Output);
    small_driver_set_duty(left_duty, right_duty);
}
void Servo_Control()
{
    left_high = high + Control_Roll_Output + Control_Bend_Output + up_leg;
    right_high = high - Control_Roll_Output - Control_Bend_Output - up_leg;

    Body_High = (left_high > right_high) ? left_high : right_high;
    // 限幅
    left_high = Limiter_float(left_high, 3.5, 14);
    right_high = Limiter_float(right_high, 3.5, 14);
    Control_Speed_Output = Limiter_float(Control_Speed_Output, - leg_angle_limiting, leg_angle_limiting);
    // 输出
    servo_control_table(left_high , Control_Speed_Output, &pwm_ph1_l, &pwm_ph4_l);    // 左边
    servo_control_table(right_high, Control_Speed_Output , &pwm_ph1_r, &pwm_ph4_r);    // 右边
    //限幅防卡死
    if(10000 == pwm_ph4_l || 10000 == pwm_ph1_l || 10000 == pwm_ph4_r || 10000 == pwm_ph1_r){
        zf_assert(10000 == pwm_ph4_l || 10000 == pwm_ph1_l || 10000 == pwm_ph4_r || 10000 == pwm_ph1_r);
        pwm_set_duty(SERVO_1, SERVO1_MID);
        pwm_set_duty(SERVO_2, SERVO2_MID);
        pwm_set_duty(SERVO_3, SERVO3_MID);
        pwm_set_duty(SERVO_4, SERVO4_MID);
    }
    //更新腿
    else {
        pwm_set_duty(SERVO_1, SERVO1_MID + pwm_ph4_l);
        pwm_set_duty(SERVO_2, SERVO2_MID - pwm_ph1_l);
        pwm_set_duty(SERVO_3, SERVO3_MID - pwm_ph4_r);
        pwm_set_duty(SERVO_4, SERVO4_MID + pwm_ph1_r);
    }
}

// 基准速度
int16 NORMAL_SPEED = 800.;
//速度限+  NORMAL_SPEED
int16 NORMAL_MAX_SPEED = 0, NORMAL_MIN_SPEED = -300;
void Speed_Control(void){

    if(single_side_bridge_type != SINGLE_SIDE_BRIDGE_NONE)  Target_Speed = 400;
    else if(traverse_type != TRAVERSE_NONE) Target_Speed = 750;
    else if (rptsn_num > 20) {
        float speed = PID_Position(&Control_Target_Speed, 0, radius_of_curvature);
        Target_Speed = (int16)MINMAX(NORMAL_SPEED + speed, NORMAL_SPEED + NORMAL_MIN_SPEED, NORMAL_SPEED + NORMAL_MAX_SPEED);
    }
    else if (rptsn_num > 5) {
        //点太少,不对劲直接慢速
        Target_Speed = NORMAL_SPEED + NORMAL_MIN_SPEED;
    }
}

uint32 time_stop = 0;

void Balance(void)
{
    Time ++;
    int16 Aim_Speed = 0;
    Dynamic_Value_Control();
    // 速度环
    if(Time % 20 == 0){
        Speed_Control();     // 速度规划
        if(run_flag == 1)   Aim_Speed = Target_Speed;
        Current_Speed = (motor_value.receive_left_speed_data - motor_value.receive_right_speed_data) / 2;
        Control_Speed_Output = PID_Position(&Control_Speed, Aim_Speed, Current_Speed);
        Control_Speed_Output = LowPassFilter_Update(&Filter_Speed_Output, Control_Speed_Output);
    }
    if(Time % 10 == 0)
    {
        // 图像偏差转向
        if(track_flag == 1 && jump_flag == 0 && !gyroscope_flag){
            Control_Yaw_Output = 0;    // 偏航角辅助转向输出清零
//            DynamicSteerParams steer_params = calculate_steer_params(radius_of_curvature, Current_Speed);
//            update_steer_pid_params(&Control_Turn, steer_params);
            float gyro_z = LowPassFilter_Update(&Filter_Gyro_Z, imu660ra_gyro_z);
            Turn_Output = Steer_PID(&Control_Turn, 0, pure_angle, gyro_z);
        }
        else
            Turn_Output = 0;
        // 切换偏航角辅助
        float corrected_yaw = unwrap_yaw(QEKF_INS.Yaw, &state);
        if(gyroscope_flag){
            Turn_Output = 0;           // 图像偏差转向输出清零
            float gyro_z = LowPassFilter_Update(&Filter_Gyro_Z, imu660ra_gyro_z);
            Control_Yaw_Output = Steer_PID(&Control_Yaw, Target_Yaw, corrected_yaw, gyro_z);
        }
        else
            Target_Yaw = corrected_yaw;
        if(Enc_flag == 1)    // 开启编码器积分，用于防止元素卡死
            Enc_sum += Current_Speed;
        else
            Enc_sum = 0;
    }
    if(Time % 5 == 0){
        Control_Pitch_Output = PID_Position(&Control_Pitch, mechanical_zero_of_pitch, QEKF_INS.Pitch);
        if(go_single_side_bridge == true && high == 6.0f){   //  翻滚角补偿
            Control_Bend_Output = 0;
            Control_Roll_Output = PID_Incremental(&Control_Roll, mechanical_zero_of_roll, QEKF_INS.Roll);
            if(single_side_bridge_start_offset_flag == 1){Target_Yaw -= 4; single_side_bridge_start_offset_flag = 0;}
            else if(single_side_bridge_start_offset_flag == 2){Target_Yaw += 4; single_side_bridge_start_offset_flag = 0;}

            if(Control_Roll_Output > 0.7 && single_bridge_flag == 0){
                single_bridge_flag = 1;
                Target_Yaw += 2;
//                gpio_set_level(P33_10, 1);
            }
            else if(Control_Roll_Output > 1.6 && single_bridge_flag == 1){
                single_bridge_flag = 2;
            }
            else if(Control_Roll_Output < 0.3 && single_bridge_flag == 2){
                single_bridge_flag = 0;
                Target_Yaw -= 0.3 *pure_angle;
                gpio_set_level(P33_10, 0);
            }
            else if(Control_Roll_Output < -0.7 && single_bridge_flag == 0){
                single_bridge_flag = -1;
                Target_Yaw -= 2;
//                gpio_set_level(P33_10, 1);
            }
            else if(Control_Roll_Output < -1.6 && single_bridge_flag == -1){
                single_bridge_flag = -2;
            }
            else if(Control_Roll_Output > -0.3 && single_bridge_flag == -2){
                single_bridge_flag = 0;
                Target_Yaw -= 0.3 *pure_angle;
                gpio_set_level(P33_10, 0);
            }
        }
        else {
            Control_Roll_Output = 0;
//            Control_Bend_Output = PID_Position(&Control_bend, 0, ya_angle);
//            Control_Bend_Output = LowPassFilter_Update(&Filter_Bend_Output, Control_Bend_Output);
//            ALL_ERROR = Control_Bend_Output;
        }
    }

    // 角速度环
    float gyro_y = LowPassFilter_Update(&Filter_Gyro_Y, imu660ra_gyro_y);
    Control_Gyro_Y_Output = PID_Incremental(&Control_Gyro_Y, Control_Pitch_Output, gyro_y);



    Motor_Control();
    if(!jump_flag)
        Servo_Control();
    // 跳跃控制
    dynamic_jump_control();
}



void PID_Init(void)
{
    // 滤波器初始化
    LowPassFilter_Init(&Filter_Gyro_Y,       0.1);
    LowPassFilter_Init(&Filter_Gyro_Z,       0.1);
    LowPassFilter_Init(&Filter_Speed_Output, 0.1);
    LowPassFilter_Init(&Filter_Bend_Output,  0.1);
    // PID参数初始化
    Incremental_PID_Init(&Control_Gyro_Y, Gyro_Y_Kp, Gyro_Y_Ki, Gyro_Y_Kd, 1);   //0.14
    Position_PID_Init   (& Control_Pitch,  Pitch_Kp,  Pitch_Ki,  Pitch_Kd, 1);   // 310
    Position_PID_Init   (& Control_Speed,  Speed_Kp,  Speed_Ki,  Speed_kd, 2);

    Position_PID_Init (&Control_Target_Speed, 157, 0, 5, 3);
    // 转向环
    Steer_PID_Init(& Control_Yaw, 30, 0.9,  0, -0.8f);
    Steer_PID_Init(&Control_Turn, Turn_Kp1, Turn_Kp2, Turn_Kd1, Turn_Kd2);
    // 舵机
    Incremental_PID_Init(&Control_Roll, -0.05, -0.006, -0.002, 2);
    Position_PID_Init(&Control_bend, -0.06, 0, -0.004, 4);
}

void Position_PID_Init(PID_PLACE *pid, float Kp, float Ki, float Kd, int type) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->prev_error = 0.0;
    pid->integral = 0.0;
    pid->type = type;
}

float PID_Position(PID_PLACE *pid, float setpoint, float current_value) {
    float error = setpoint - current_value;
    float derivative;  // 微分项
    float output;

    // 计算积分项
    pid->integral += error;

    // 计算微分项
    derivative = (error - pid->prev_error);
    // 计算输出
    output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
    if(pid->type == 4)
    {
        output = Limiter_float(output, -1, 1);
    }
    pid->prev_error = error;
    return output;
}
void Steer_PID_Init(PID_STEER *pid, float Kp1, float Kp2, float Kd1, float Kd2)
{
    pid->Kp1 = Kp1;
    pid->Kp2 = Kp2;
    pid->Kd1 = Kd1;
    pid->Kd2 = Kd2;
    pid->prev_error = 0;
}
float Steer_PID(PID_STEER *pid, float setpoint, float current_value, int16 gyro_z)
{
    float error = setpoint - current_value;

    float output;
    output= pid->Kp1 * error + pid->Kp2 * absValue(error) * error + pid->Kd1 * (error - pid->prev_error) + pid->Kd2 * gyro_z;

    output = Limiter_float(output, -PWM_Amplitude_Limiting, PWM_Amplitude_Limiting);
    pid->prev_error = error;
    return output;
}
//// 初始化PID结构体
void Incremental_PID_Init(PID_INCREMENTAL *pid, float Kp, float Ki, float Kd, int type) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->prev_error = 0.0;
    pid->prev_prev_error = 0.0;
    pid->integral = 0.0;
    pid->type = type;
}

// 增量式PID计算函数
float PID_Incremental(PID_INCREMENTAL *pid, float setpoint, float current_value) {
    float error = setpoint - current_value;
    pid->integral = pid->Ki * error;

    pid->output += pid->Kp * (error - pid->prev_error) + pid->integral + pid->Kd * (error - 2 * pid->prev_error + pid->prev_prev_error);

    // 输出限幅(内环)
    if(pid->type == 1)
    {
        pid->output = Limiter_float(pid->output, -PWM_Amplitude_Limiting, PWM_Amplitude_Limiting);
    }
    if(pid->type == 3)
    {
        pid->output = Limiter_float(pid->output, -1., 1.);
    }
    // 更新误差项
    pid->prev_prev_error = pid->prev_error;
    pid->prev_error = error;

    return pid->output;
}

// 增量式PID计算函数
float Steer_PID_Incremental(PID_INCREMENTAL *pid, float setpoint, float current_value, int16 gyro_z) {
    float error = setpoint - current_value;
    pid->integral = pid->Ki * error;

    pid->output += pid->Kp * (error - pid->prev_error) + pid->integral + pid->Kd * gyro_z;

    // 输出限幅(内环)
    if(pid->type == 1 && pid->type == 2)
    {
        pid->output = Limiter_float(pid->output, -PWM_Amplitude_Limiting, PWM_Amplitude_Limiting);
    }

    // 更新误差项
    pid->prev_error = error;

    return pid->output;
}





/**
 * 实时处理 yaw 跳变的函数
 * @param current_yaw  当前时刻的 yaw 角（单位：度，范围 [-180, 180]）
 * @param state        状态结构体指针（需持久化保存）
 * @return             修正后的连续 yaw 角
 */
float unwrap_yaw(float current_yaw, YawUnwrapperState *state) {
    if (!state->is_initialized) {
        // 初始化状态
        state->prev_yaw = current_yaw;
        state->offset = 0;
        state->is_initialized = true;
        return current_yaw;
    }

    // 计算当前角度与前一角度的差值
    float delta = current_yaw - state->prev_yaw;

    // 检测跳变并修正偏移量
    if (delta > 180.0f) {
        state->offset -= 360.0f;  // 正向跳变（例如 +179 → -179 → 修正为 +181）
    } else if (delta < -180.0f) {
        state->offset += 360.0f;  // 负向跳变（例如 -179 → +179 → 修正为 -181）
    }

    // 计算解缠后的角度
    float unwrapped_yaw = current_yaw + state->offset;

    // 更新状态
    state->prev_yaw = current_yaw;

    return unwrapped_yaw;
}



// 动态转向PID参数计算
DynamicSteerParams calculate_steer_params(float radius_of_curvature, float current_speed) {
    DynamicSteerParams params;
    float abs_curv = radius_of_curvature;

    // 基础参数值
    float base_kp1 = 32.0f;
    float base_kp2 = 0.9f;
    float base_kd1 = 35.0f;
    float base_kd2 = -0.25f;


    // 计算动态参数
    params.Kp1 = base_kp1 * abs_curv / current_speed;
    params.Kp2 = base_kp2 * abs_curv;
    params.Kd1 = base_kd1 * abs_curv * current_speed;
    params.Kd2 = base_kd2 * abs_curv;

    // 动态输出限制
    params.max_output = PWM_Amplitude_Limiting * (1.8f - 0.8f*current_speed);

    return params;
}

// 更新转向PID参数
void update_steer_pid_params(PID_STEER *pid, DynamicSteerParams params) {
    pid->Kp1 = params.Kp1;
    pid->Kp2 = params.Kp2;
    pid->Kd1 = params.Kd1;
    pid->Kd2 = params.Kd2;
}```
```PID.h
/*
 * PID_control.h
 *
 *  Created on: 2025年1月15日
 *      Author: 张
 */

#ifndef CODE_PID_CONTROL_H_
#define CODE_PID_CONTROL_H_

#include "zf_common_headfile.h"

#define PWM_Amplitude_Limiting 3000    // 占空比输出限制

// 俯仰角、翻滚角机械零点
#define Mechanical_Zero_Of_Ritch -3.3
#define Mechanical_Zero_Of_Roll -0



#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MINMAX(input, low, upper) MIN(MAX(input, low), upper)

// 低通滤波器结构体
typedef struct {
    float alpha;  // 滤波系数
    float prev_output;  // 上一次的输出值
} LowPassFilter;

// 定义PID结构体
typedef struct {
    float Kp;  // 比例系数
    float Ki;  // 积分系数
    float Kd;  // 微分系数
    float prev_error;  // 上一次的误差
    float prev_prev_error;  // 上上次的误差
    float integral;  // 积分项
    float output;   // 输出
//    float output_limiting;
    int type;      // 类（用户自定义）    1:角速度环 2:速度环
} PID_INCREMENTAL;
typedef struct {
    float Kp1;
    float Kp2;
    float Kd1;
    float Kd2;
    float prev_error;  // 上一次的误差
} PID_STEER;


typedef struct {
    float Kp;  // 比例系数
    float Ki;  // 积分系数
    float Kd;  // 微分系数
    float prev_error;  // 上一次的误差
    float integral;  // 积分项
//    float output_limiting;
    int type;      // 类（用户自定义）    1:角度环
} PID_PLACE;

// 状态结构体（保存角度解缠的中间状态）
typedef struct {
    bool is_initialized;  // 是否已初始化
    float prev_yaw;       // 前一时刻的 yaw 角
    float offset;         // 累计偏移量
} YawUnwrapperState;


// 动态转向PID参数结构体
typedef struct {
    float Kp1;          // 线性比例项
    float Kp2;          // 非线性比例项
    float Kd1;          // 微分项
    float Kd2;          // 陀螺仪阻尼项
    float max_output;   // 最大输出限制
} DynamicSteerParams;


extern float leg_angle_limiting;
extern float high;
extern float Body_High;
extern float ERROR;
extern float ALL_ERROR;
extern int16 pwm_ph1_l, pwm_ph4_l;
extern int16 pwm_ph1_r, pwm_ph4_r;

extern float mechanical_zero_of_pitch,
             mechanical_zero_of_roll;

extern int8 Speed_Compensation;
extern float Target_Yaw;
extern int16 Target_Speed, Current_Speed;
extern int32 Enc_sum;
extern uint8 Enc_flag;


extern float up_leg;

extern uint8 gyroscope_flag;
extern uint8 track_flag;
extern uint8 run_flag;
extern uint8 jump_flag;
extern uint32 jump_time;
extern int8 single_bridge_flag;
float absValue(float num);
int int_min(int a, int b);
void LowPassFilter_Init(LowPassFilter* filter, float alpha);
float LowPassFilter_Update(LowPassFilter* filter, float input);
float Limiter_float(float aim, float low, float high);
void Balance(void);
void PID_Init(void);
void Position_PID_Init(PID_PLACE *pid, float Kp, float Ki, float Kd, int type);
void Incremental_PID_Init(PID_INCREMENTAL *pid, float Kp, float Ki, float Kd, int type);
void Steer_PID_Init(PID_STEER *pid, float Kp1, float Kp2, float Kd1, float Kd2);
float Steer_PID(PID_STEER *pid, float setpoint, float current_value, int16 gyro_z);
float PID_Position(PID_PLACE *pid, float setpoint, float current_value);
float PID_Incremental(PID_INCREMENTAL *pid, float setpoint, float current_value);
float Steer_PID_Incremental(PID_INCREMENTAL *pid, float setpoint, float current_value, int16 gyro_z);
float unwrap_yaw(float current_yaw, YawUnwrapperState *state);

#endif /* CODE_PID_CONTROL_H_ */```
```uart.v
/*
 * Screen_display.h
 *
 *  Created on: 2025年1月26日
 *      Author: 张
 */

#include "uart.h"




char send_str[32] = {0};
uint8 tiao = 0;
void Receive_Interrupt_Processing(void)
{
    seekfree_assistant_data_analysis();
    for(uint8_t i = 0; i < SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT; i++)
    {
        // 更新标志位
        if(seekfree_assistant_parameter_update_flag[i])
        {
            seekfree_assistant_parameter_update_flag[i] = 0;
//            sprintf(send_str,"receive data channel : %d ", i);
//            wireless_uart_send_buffer((uint8 *)send_str,strlen(send_str));
//            sprintf(send_str,"data : %.2f \r\n", seekfree_assistant_parameter[i]);
//            wireless_uart_send_buffer((uint8 *)send_str,strlen(send_str));
            if(i == 0)
                Target_Speed = seekfree_assistant_parameter[i];
//
            else if(i == 1)
                pure_angle = seekfree_assistant_parameter[i];
            else if(i == 2)
                tiao = seekfree_assistant_parameter[i];
//            else if(i == 3)
//                Inner_Kp = seekfree_assistant_parameter[i];
//            else if(i == 4)
//                Inner_Ki = seekfree_assistant_parameter[i];
//            else if(i == 5)
//                Centre_Kp = seekfree_assistant_parameter[i];
//            else if(i == 6)
//                Outer_Kp = seekfree_assistant_parameter[i];
//            else if(i == 7)
//                Outer_Ki = seekfree_assistant_parameter[i];
        }
    }


}

Serial_Receive_Struct Serial_Rx;      // 接收状态结构体
uint8_t Serial_RxPacket[MAX_PACKET_LEN]; // 最终数据包存储

// 串口接收回调函数（UART8）
void uart2_control_callback(void)
{
    static uint8_t buffer[MAX_PACKET_LEN]; // 独立临时缓冲区
    static uint8_t index = 0;

    uint8_t receive_data;
    while(uart_query_byte(UART_2, &receive_data))
    {
        if(receive_data == '@')
        {
            index = 0; // 检测到新帧头时重置索引
            buffer[index++] = receive_data;
            continue;
        }

        if(index > 0 && index < MAX_PACKET_LEN-1)
        {
            buffer[index++] = receive_data;

            // 检测帧尾
            if(index >= 4 && buffer[index-2] == '\r' && buffer[index-1] == '\n')
            {
                // 复制到正式缓冲区前清空旧数据
                memset(Serial_RxPacket, 0, MAX_PACKET_LEN); // 关键清空操作
                memcpy(Serial_RxPacket, &buffer[1], index-3); // 去掉@和\r\n
                Serial_RxPacket[index-3] = '\0'; // 强制终止字符串

                Serial_Rx.packet_ready = 1;
                index = 0;
            }
        }
    }
}
// 全局变量定义
uint8 be_stop = 0;  // 控制标志位

// 串口数据包处理函数
void Process_Serial_Packet(void)
{
    if(Serial_Rx.packet_ready)
    {
        // 检查数据包内容
        if(Serial_RxPacket[0] == '1' && Serial_RxPacket[1] == '\0')
        {
            be_stop = 1;  // 设置停止标志
        }
        else if(Serial_RxPacket[0] == '2' && Serial_RxPacket[1] == '\0')
        {
            be_stop = 2;  // 清除停止标志
        }
        Serial_RxPacket[0] = '0';
        // 重置数据包就绪标志
        Serial_Rx.packet_ready = 0;
    }
}
//uart_init(UART_8,115200,UART8_TX_P33_7,UART8_RX_P33_6);
//uart_rx_interrupt(UART_8, 1);

//uint8_t line = 0;
//for (uint8_t i = 0; Serial_RxPacket[i] != '\0'; i += 20){
//    ips200_show_string(8 * 0, line * 16, (char*)&Serial_RxPacket[i]);
//    line++;
//}
//   printf("Received: %s\n", Serial_RxPacket);```
```uart.h
/*
 * uart.h
 *
 *  Created on: 2025年3月2日
 *      Author: 张
 */

#ifndef CODE_UART_H_
#define CODE_UART_H_

#include "zf_common_headfile.h"

#define MAX_PACKET_LEN 100  // 最大数据包长度

typedef struct {
    uint8_t buffer[MAX_PACKET_LEN];  // 接收缓冲区
    uint8_t index;                   // 当前存储位置
    uint8_t packet_ready;            // 数据包就绪标志
} Serial_Receive_Struct;

extern uint8 tiao;
extern uint8_t Serial_RxPacket[MAX_PACKET_LEN];
extern uint8 be_stop;
void Receive_Interrupt_Processing(void);
void uart2_control_callback(void);
void Process_Serial_Packet(void);


#endif /* CODE_UART_H_ */
、、、vmc.c
#include "vmc.h"

const float P_max = 14.200000;
const float P_min = 3.000000;
const float A_max = 30.000000;
const float A_min = -30.000000;
const float P_step = 0.030000;
const float A_step = 0.100000;

const int16 pwm_table_1[374][601] = {
    {10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 217, 213, 209, 205, 201, 197, 193, 189, 185, 180, 176, 172, 168, 164, 160, 156, 152, 147, 143, 139, 135, 131, 127, 123, 118, 114, 110, 106, 102, 97, 93, 89, 85, 80, 76, 72, 68, 63, 59, 55, 51, 46, 42, 38, 33, 29, 25, 20, 16, 12, 7, 3, -1, -6, -10, -14, -19, -23, -28, -32, -36, -41, -45, -50, -54, -59, -63, -67, -72, -76, -81, -85, -90, -94, -99, -103, -108, -113, -117, -122, -126, -131, -135, -140, -145, -149, -154, -158, -163, -168, -172, -177, -182, -186, -191, -196, -200, -205, -210, -214, -219, -224, -229, -233, -238, -243, -248, -252, -257, -262, -267, -272, -277, -281, -286, -291, -296, -301, -306, -311, -315, -320, -325, -330, -335, -340, -345, -350, -355, -360, -365, -370, -375, -380, -385, -390, -395, -400, -405, -411, -416, -421, -426, -431, -436, -441, -447, -452, -457, -462, -467, -473, -478, -483, -488, -494, -499, -504, -509, -515, -520, -525, -531, -536, -542, -547, -552, -558, -563, -569, -574, -580, -585, -591, -596, -602, -607, -613, -618, -624, -629, -635, -641, -646, -652, -658, -663, -669, -675, -680, -686, -692, -698, -703, -709, -715, -721, -727, -733, -738, -744, -750, -756, -762, -768, -774, -780, -786, -792, -798, -804, -810, -816, -823, -829, -835, -841, -847, -853, -860, -866, -872, -878, -885, -891, -897, -904, -910, -917, -923, -930, -936, -942, -949, -956, -962, -969, -975, -982, -989, -995, -1002, -1009, -1015, -1022, -1029, -1036, -1043, -1050, -1057, -1064, -1070, -1077, -1085, -1092, -1099, -1106, -1113, -1120, -1127, -1134, -1142, -1149, -1156, -1164, -1171, -1179, -1186, -1193, -1201, -1209, -1216, -1224, -1231, -1239, -1247, -1255, -1263, -1270, -1278, -1286, -1294, -1302, -1310, -1319, -1327, -1335, -1343, -1352, -1360, -1368, -1377, -1385, -1394, -1403, -1411, -1420, -1429, -1438, -1447, -1456, -1465, -1474, -1483, -1492, -1501, -1511, -1520, -1530, -1540, -1549, -1559, -1569, -1579, -1589, -1599, -1609, -1620, -1630, -1641, -1651, -1662, -1673, -1684, -1695, -1707, -1718, -1730, -1742, -1754, -1766, -1778, -1790, -1803, -1816, -1829, -1843, -1856, -1870, -1884, -1899, -1914, -1929, -1944, -1960, -1977, -1994, -2012, -2030, -2049, -2069, -2090, -2113, -2137, -2163, -2191, -2224, -2264, -2319, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000},
    {10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 263, 259, 255, 251, 247, 243, 239, 235, 231, 227, 223, 219, 215, 211, 207, 203, 199, 195, 191, 187, 183, 178, 174, 170, 166, 162, 158, 154, 150, 146, 141, 137, 133, 129, 125, 121, 116, 112, 108, 104, 100, 95, 91, 87, 83, 78, 74, 70, 66, 61, 57, 53, 49, 44, 40, 36, 31, 27, 23, 18, 14, 10, 5, 1, -3, -8, -12, -16, -21, -25, -30, -34, -39, -43, -47, -52, -56, -61, -65, -70, -74, -79, -83, -88, -92, -97, -101, -106, -110, -115, -119, -124, -128, -133, -138, -142, -147, -151, -156, -161, -165, -170, -174, -179, -184, -188, -193, -198, -202, -207, -212, -217, -221, -226, -231, -236, -240, -245, -250, -255, -259, -264, -269, -274, -279, -284, -288, -293, -298, -303, -308, -313, -318, -323, -327, -332, -337, -342, -347, -352, -357, -362, -367, -372, -377, -382, -387, -392, -397, -402, -408, -413, -418, -423, -428, -433, -438, -443, -449, -454, -459, -464, -469, -474, -480, -485, -490, -495, -501, -506, -511, -517, -522, -527, -533, -538, -543, -549, -554, -560, -565, -570, -576, -581, -587, -592, -598, -603, -609, -614, -620, -625, -631, -637, -642, -648, -653, -659, -665, -670, -676, -682, -688, -693, -699, -705, -711, -716, -722, -728, -734, -740, -746, -751, -757, -763, -769, -775, -781, -787, -793, -799, -805, -811, -817, -823, -829, -836, -842, -848, -854, -860, -866, -873, -879, -885, -891, -898, -904, -910, -917, -923, -930, -936, -943, -949, -956, -962, -969, -975, -982, -988, -995, -1002, -1008, -1015, -1022, -1029, -1035, -1042, -1049, -1056, -1063, -1070, -1076, -1083, -1090, -1097, -1104, -1112, -1119, -1126, -1133, -1140, -1147, -1155, -1162, -1169, -1177, -1184, -1191, -1199, -1206, -1214, -1221, -1229, -1236, -1244, -1252, -1259, -1267, -1275, -1283, -1291, -1299, -1307, -1315, -1323, -1331, -1339, -1347, -1355, -1364, -1372, -1380, -1389, -1397, -1406, -1414, -1423, -1432, -1440, -1449, -1458, -1467, -1476, -1485, -1494, -1503, -1512, -1522, -1531, -1541, -1550, -1560, -1570, -1579, -1589, -1599, -1609, -1619, -1629, -1640, -1650, -1661, -1671, -1682, -1693, -1704, -1715, -1726, -1738, -1749, -1761, -1773, -1785, -1797, -1809, -1822, -1835, -1848, -1861, -1874, -1888, -1902, -1916, -1931, -1946, -1961, -1977, -1993, -2010, -2027, -2045, -2063, -2083, -2103, -2124, -2147, -2171, -2197, -2227, -2260, -2301, -2359, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000},
    {10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 309, 305, 301, 297, 293, 289, 285, 281, 277, 273, 269, 265, 261, 257, 253, 249, 245, 241, 237, 233, 229, 225, 221, 217, 213, 209, 205, 201, 197, 193, 189, 185, 180, 176, 172, 168, 164, 160, 156, 152, 147, 143, 139, 135, 131, 127, 122, 118, 114, 110, 106, 102, 97, 93, 89, 85, 80, 76, 72, 68, 63, 59, 55, 51, 46, 42, 38, 33, 29, 25, 20, 16, 12, 7, 3, -1, -6, -10, -14, -19, -23, -28, -32, -36, -41, -45, -50, -54, -59, -63, -68, -72, -77, -81, -85, -90, -94, -99, -104, -108, -113, -117, -122, -126, -131, -135, -140, -145, -149, -154, -158, -163, -168, -172, -177, -182, -186, -191, -196, -200, -205, -210, -214, -219, -224, -228, -233, -238, -243, -247, -252, -257, -262, -267, -271, -276, -281, -286, -291, -296, -300, -305, -310, -315, -320, -325, -330, -335, -340, -345, -350, -355, -360, -365, -370, -375, -380, -385, -390, -395, -400, -405, -410, -415, -420, -425, -430, -435, -440, -446, -451, -456, -461, -466, -471, -477, -482, -487, -492, -498, -503, -508, -513, -519, -524, -529, -535, -540, -545, -551, -556, -562, -567, -572, -578, -583, -589, -594, -600, -605, -611, -616, -622, -627, -633, -638, -644, -650, -655, -661, -667, -672, -678, -684, -689, -695, -701, -706, -712, -718, -724, -729, -735, -741, -747, -753, -759, -765, -771, -776, -782, -788, -794, -800, -806, -812, -818, -824, -831, -837, -843, -849, -855, -861, -867, -874, -880, -886, -892, -899, -905, -911, -917, -924, -930, -937, -943, -949, -956, -962, -969, -975, -982, -989, -995, -1002, -1008, -1015, -1022, -1028, -1035, -1042, -1049, -1055, -1062, -1069, -1076, -1083, -1090, -1097, -1104, -1111, -1118, -1125, -1132, -1139, -1146, -1153, -1161, -1168, -1175, -1182, -1190, -1197, -1204, -1212, -1219, -1227, -1234, -1242, -1249, -1257, -1265, -1272, -1280, -1288, -1296, -1304, -1311, -1319, -1327, -1335, -1343, -1352, -1360, -1368, -1376, -1384, -1393, -1401, -1410, -1418, -1427, -1435, -1444, -1452, -1461, -1470, -1479, -1488, -1497, -1506, -1515, -1524, -1533, -1543, -1552, -1561, -1571, -1580, -1590, -1600, -1610, -1620, -1630, -1640, -1650, -1660, -1671, -1681, -1692, -1702, -1713, -1724, -1735, -1747, -1758, -1769, -1781, -1793, -1805, -1817, -1829, -1841, -1854, -1867, -1880, -1893, -1907, -1921, -1935, -1949, -1964, -1979, -1994, -2010, -2026, -2043, -2061, -2078, -2097, -2117, -2137, -2159, -2182, -2206, -2233, -2263, -2297, -2339, -2402, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000},
    {10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 354, 350, 346, 342, 338, 334, 330, 326, 322, 319, 315, 311, 307, 303, 299, 295, 291, 287, 283, 279, 275, 271, 267, 263, 259, 255, 251, 247, 243, 239, 235, 231, 227, 223, 219, 215, 211, 207, 203, 199, 194, 190, 186, 182, 178, 174, 170, 166, 162, 157, 153, 149, 145, 141, 137, 133, 128, 124, 120, 116, 112, 107, 103, 99, 95, 91, 86, 82, 78, 74, 69, 65, 61, 57, 52, 48, 44, 39, 35, 31, 26, 22, 18, 13, 9, 5, 0, -4, -8, -13, -17, -21, -26, -30, -35, -39, -43, -48, -52, -57, -61, -66, -70, -75, -79, -84, -88, -93, -97, -102, -106, -111, -115, -120, -124, -129, -133, -138, -143, -147, -152, -156, -161, -166, -170, -175, -180, -184, -189, -194, -198, -203, -208, -212, -217, -222, -226, -231, -236, -241, -245, -250, -255, -260, -264, -269, -274, -279, -284, -289, -293, -298, -303, -308, -313, -318, -323, -328, -332, -337, -342, -347, -352, -357, -362, -367, -372, -377, -382, -387, -392, -397, -402, -407, -412, -417, -423, -428, -433, -438, -443, -448, -453, -458, -464, -469, -474, -479, -484, -490, -495, -500, -505, -511, -516, -521, -526, -532, -537, -542, -548, -553, -559, -564, -569, -575, -580, -586, -591, -596, -602, -607, -613, -618, -624, -629, -635, -641, -646, -652, -657, -663, -669, -674, -680, -686, -691, -697, -703, -708, -714, -720, -726, -731, -737, -743, -749, -755, -760, -766, -772, -778, -784, -790, -796, -802, -808, -814, -820, -826, -832, -838, -844, -850, -856, -862, -869, -875, -881, -887, -893, -900, -906, -912, -918, -925, -931, -937, -944, -950, -957, -963, -970, -976, -983, -989, -996, -1002, -1009, -1015, -1022, -1029, -1035, -1042, -1049, -1056, -1062, -1069, -1076, -1083, -1090, -1096, -1103, -1110, -1117, -1124, -1131, -1138, -1145, -1153, -1160, -1167, -1174, -1181, -1189, -1196, -1203, -1210, -1218, -1225, -1233, -1240, -1248, -1255, -1263, -1270, -1278, -1286, -1293, -1301, -1309, -1317, -1325, -1333, -1340, -1348, -1356, -1365, -1373, -1381, -1389, -1397, -1406, -1414, -1422, -1431, -1439, -1448, -1456, -1465, -1474, -1482, -1491, -1500, -1509, -1518, -1527, -1536, -1545, -1554, -1564, -1573, -1582, -1592, -1602, -1611, -1621, -1631, -1641, -1651, -1661, -1671, -1681, -1692, -1702, -1713, -1723, -1734, -1745, -1756, -1767, -1778, -1790, -1801, -1813, -1825, -1837, -1849, -1861, -1874, -1887, -1900, -1913, -1926, -1940, -1954, -1968, -1982, -1997, -2012, -2028, -2044, -2060, -2077, -2095, -2113, -2132, -2151, -2172, -2194, -2217, -2242, -2269, -2300, -2335, -2379, -2449, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000},
    {10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 398, 394, 390, 386, 382, 378, 375, 371, 367, 363, 359, 355, 351, 347, 344, 340, 336, 332, 328, 324, 320, 316, 312, 308, 304, 300, 296, 293, 289, 285, 281, 277, 273, 269, 265, 261, 257, 253, 249, 245, 241, 237, 233, 229, 224, 220, 216, 212, 208, 204, 200, 196, 192, 188, 184, 180, 176, 171, 167, 163, 159, 155, 151, 147, 142, 138, 134, 130, 126, 122, 117, 113, 109, 105, 101, 96, 92, 88, 84, 79, 75, 71, 67, 62, 58, 54, 50, 45, 41, 37, 32, 28, 24, 19, 15, 11, 6, 2, -2, -7, -11, -15, -20, -24, -29, -33, -37, -42, -46, -51, -55, -60, -64, -69, -73, -77, -82, -86, -91, -95, -100, -104, -109, -114, -118, -123, -127, -132, -136, -141, -145, -150, -155, -159, -164, -169, -173, -178, -182, -187, -192, -196, -201, -206, -210, -215, -220, -225, -229, -234, -239, -244, -248, -253, -258, -263, -267, -272, -277, -282, -287, -291, -296, -301, -306, -311, -316, -321, -326, -330, -335, -340, -345, -350, -355, -360, -365, -370, -375, -380, -385, -390, -395, -400, -405, -410, -415, -420, -425, -430, -436, -441, -446, -451, -456, -461, -466, -472, -477, -482, -487, -492, -498, -503, -508, -513, -519, -524, -529, -534, -540, -545, -550, -556, -561, -567, -572, -577, -583, -588, -594, -599, -604, -610, -615, -621, -626, -632, -637, -643, -649, -654, -660, -665, -671, -677, -682, -688, -693, -699, -705, -711, -716, -722, -728, -734, -739, -745, -751, -757, -763, -768, -774, -780, -786, -792, -798, -804, -810, -816, -822, -828, -834, -840, -846, -852, -858, -864, -870, -876, -883, -889, -895, -901, -907, -914, -920, -926, -932, -939, -945, -951, -958, -964, -971, -977, -984, -990, -997, -1003, -1010, -1016, -1023, -1029, -1036, -1043, -1049, -1056, -1063, -1069, -1076, -1083, -1090, -1097, -1104, -1110, -1117, -1124, -1131, -1138, -1145, -1152, -1159, -1166, -1174, -1181, -1188, -1195, -1202, -1210, -1217, -1224, -1232, -1239, -1246, -1254, -1261, -1269, -1276, -1284, -1292, -1299, -1307, -1315, -1323, -1330, -1338, -1346, -1354, -1362, -1370, -1378, -1386, -1394, -1402, -1410, -1419, -1427, -1435, -1444, -1452, -1461, -1469, -1478, -1486, -1495, -1504, -1513, -1522, -1530, -1539, -1548, -1558, -1567, -1576, -1585, -1595, -1604, -1613, -1623, -1633, -1642, -1652, -1662, -1672, -1682, -1692, -1702, -1713, -1723, -1734, -1744, -1755, -1766, -1777, -1788, -1799, -1811, -1822, -1834, -1846, -1857, -1870, -1882, -1894, -1907, -1920, -1933, -1946, -1959, -1973, -1987, -2001, -2016, -2031, -2046, -2062, -2078, -2095, -2112, -2129, -2148, -2167, -2187, -2208, -2230, -2253, -2279, -2306, -2337, -2374, -2420, -2507, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000},
    {10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 437, 433, 430, 426, 422, 418, 414, 411, 407, 403, 399, 395, 391, 388, 384, 380, 376, 372, 368, 364, 361, 357, 353, 349, 345, 341, 337, 333, 329, 325, 322, 318, 314, 310, 306, 302, 298, 294, 290, 286, 282, 278, 274, 270, 266, 262, 258, 254, 250, 246, 242, 238, 234, 230, 226, 222, 218, 214, 210, 206, 201, 197, 193, 189, 185, 181, 177, 173, 169, 165, 160, 156, 152, 148, 144, 140, 135, 131, 127, 123, 119, 115, 110, 106, 102, 98, 94, 89, 85, 81, 77, 72, 68, 64, 60, 55, 51, 47, 42, 38, 34, 29, 25, 21, 16, 12, 8, 3, -1, -5, -10, -14, -18, -23, -27, -32, -36, -40, -45, -49, -54, -58, -63, -67, -72, -76, -81, -85, -90, -94, -99, -103, -108, -112, -117, -121, -126, -130, -135, -139, -144, -149, -153, -158, -162, -167, -172, -176, -181, -186, -190, -195, -200, -204, -209, -214, -218, -223, -228, -232, -237, -242, -247, -251, -256, -261, -266, -270, -275, -280, -285, -290, -295, -299, -304, -309, -314, -319, -324, -329, -334, -338, -343, -348, -353, -358, -363, -368, -373, -378, -383, -388, -393, -398, -403, -408, -413, -418, -423, -428, -433, -439, -444, -449, -454, -459, -464, -469, -475, -480, -485, -490, -495, -501, -506, -511, -516, -521, -527, -532, -537, -543, -548, -553, -559, -564, -569, -575, -580, -586, -591, -596, -602, -607, -613, -618, -624, -629, -635, -640, -646, -651, -657, -662, -668, -674, -679, -685, -690, -696, -702, -707, -713, -719, -724, -730, -736, -742, -747, -753, -759, -765, -771, -777, -782, -788, -794, -800, -806, -812, -818, -824, -830, -836, -842, -848, -854, -860, -866, -872, -878, -884, -891, -897, -903, -909, -915, -922, -928, -934, -940, -947, -953, -959, -966, -972, -978, -985, -991, -998, -1004, -1011, -1017, -1024, -1030, -1037, -1044, -1050, -1057, -1064, -1070, -1077, -1084, -1091, -1097, -1104, -1111, -1118, -1125, -1132, -1139, -1145, -1152, -1159, -1167, -1174, -1181, -1188, -1195, -1202, -1209, -1217, -1224, -1231, -1238, -1246, -1253, -1261, -1268, -1275, -1283, -1290, -1298, -1306, -1313, -1321, -1329, -1336, -1344, -1352, -1360, -1368, -1376, -1384, -1392, -1400, -1408, -1416, -1424, -1432, -1441, -1449, -1457, -1466, -1474, -1483, -1491, -1500, -1508, -1517, -1526, -1535, -1543, -1552, -1561, -1570, -1579, -1589, -1598, -1607, -1616, -1626, -1635, -1645, -1655, -1664, -1674, -1684, -1694, -1704, -1714, -1724, -1734, -1745, -1755, -1766, -1777, -1787, -1798, -1809, -1821, -1832, -1843, -1855, -1867, -1878, -1890, -1903, -1915, -1928, -1940, -1953, -1966, -1980, -1993, -2007, -2021, -2036, -2050, -2065, -2081, -2097, -2113, -2130, -2147, -2165, -2183, -2202, -2223, -2244, -2266, -2290, -2316, -2344, -2376, -2414, -2463, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000},
    {10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 480, 476, 473, 469, 465, 461, 458, 454, 450, 446, 442, 439, 435, 431, 427, 423, 420, 416, 412, 408, 404, 400, 397, 393, 389, 385, 381, 377, 374, 370, 366, 362, 358, 354, 350, 346, 342, 339, 335, 331, 327, 323, 319, 315, 311, 307, 303, 299, 295, 291, 287, 283, 279, 275, 271, 267, 263, 259, 255, 251, 247, 243, 239, 235, 231, 227, 223, 219, 215, 211, 207, 203, 199, 194, 190, 186, 182, 178, 174, 170, 166, 162, 157, 153, 149, 145, 141, 137, 132, 128, 124, 120, 116, 112, 107, 103, 99, 95, 90, 86, 82, 78, 73, 69, 65, 61, 56, 52, 48, 44, 39, 35, 31, 26, 22, 18, 13, 9, 5, 0, -4, -9, -13, -17, -22, -26, -30, -35, -39, -44, -48, -53, -57, -61, -66, -70, -75, -79, -84, -88, -93, -97, -102, -106, -111, -115, -120, -124, -129, -134, -138, -143, -147, -152, -156, -161, -166, -170, -175, -180, -184, -189, -193, -198, -203, -208, -212, -217, -222, -226, -231, -236, -240, -245, -250, -255, -259, -264, -269, -274, -279, -283, -288, -293, -298, -303, -308, -312, -317, -322, -327, -332, -337, -342, -347, -352, -357, -361, -366, -371, -376, -381, -386, -391, -396, -401, -406, -411, -416, -421, -427, -432, -437, -442, -447, -452, -457, -462, -467, -473, -478, -483, -488, -493, -498, -504, -509, -514, -519, -525, -530, -535, -540, -546, -551, -556, -562, -567, -572, -578, -583, -589, -594, -599, -605, -610, -616, -621, -627, -632, -638, -643, -649, -654, -660, -665, -671, -676, -682, -688, -693, -699, -705, -710, -716, -722, -727, -733, -739, -744, -750, -756, -762, -768, -773, -779, -785, -791, -797, -803, -808, -814, -820, -826, -832, -838, -844, -850, -856, -862, -868, -874, -880, -887, -893, -899, -905, -911, -917, -924, -930, -936, -942, -949, -955, -961, -967, -974, -980, -987, -993, -999, -1006, -1012, -1019, -1025, -1032, -1038, -1045, -1052, -1058, -1065, -1071, -1078, -1085, -1092, -1098, -1105, -1112, -1119, -1125, -1132, -1139, -1146, -1153, -1160, -1167, -1174, -1181, -1188, -1195, -1202, -1209, -1217, -1224, -1231, -1238, -1246, -1253, -1260, -1268, -1275, -1282, -1290, -1297, -1305, -1312, -1320, -1328, -1335, -1343, -1351, -1358, -1366, -1374, -1382, -1390, -1398, -1406, -1414, -1422, -1430, -1438, -1446, -1454, -1463, -1471, -1479, -1488, -1496, -1505, -1513, -1522, -1531, -1539, -1548, -1557, -1566, -1575, -1583, -1593, -1602, -1611, -1620, -1629, -1639, -1648, -1657, -1667, -1677, -1686, -1696, -1706, -1716, -1726, -1736, -1746, -1756, -1767, -1777, -1788, -1799, -1809, -1820, -1831, -1842, -1854, -1865, -1876, -1888, -1900, -1912, -1924, -1936, -1949, -1961, -1974, -1987, -2000, -2014, -2027, -2041, -2056, -2070, -2085, -2100, -2116, -2132, -2148, -2165, -2182, -2200, -2219, -2239, -2259, -2280, -2303, -2327, -2354, -2383, -2416, -2455, -2508, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000},
    {10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 522, 519, 515, 511, 507, 504, 500, 496, 493, 489, 485, 481, 478, 474, 470, 466, 463, 459, 455, 451, 447, 444, 440, 436, 432, 428, 425, 421, 417, 413, 409, 405, 402, 398, 394, 390, 386, 382, 379, 375, 371, 367, 363, 359, 355, 351, 347, 344, 340, 336, 332, 328, 324, 320, 316, 312, 308, 304, 300, 296, 292, 288, 284, 280, 276, 272, 268, 264, 260, 256, 252, 248, 244, 240, 236, 232, 228, 224, 220, 216, 212, 208, 204, 200, 195, 191, 187, 183, 179, 175, 171, 167, 163, 158, 154, 150, 146, 142, 138, 133, 129, 125, 121, 117, 113, 108, 104, 100, 96, 91, 87, 83, 79, 74, 70, 66, 62, 57, 53, 49, 45, 40, 36, 32, 27, 23, 19, 14, 10, 6, 1, -3, -8, -12, -16, -21, -25, -29, -34, -38, -43, -47, -52, -56, -60, -65, -69, -74, -78, -83, -87, -92, -96, -101, -105, -110, -114, -119, -123, -128, -132, -137, -142, -146, -151, -155, -160, -165, -169, -174, -178, -183, -188, -192, -197, -202, -206, -211, -216, -220, -225, -230, -235, -239, -244, -249, -253, -258, -263, -268, -273, -277, -282, -287, -292, -297, -301, -306, -311, -316, -321, -326, -331, -335, -340, -345, -350, -355, -360, -365, -370, -375, -380, -385, -390, -395, -400, -405, -410, -415, -420, -425, -430, -435, -440, -445, -450, -455, -461, -466, -471, -476, -481, -486, -492, -497, -502, -507, -512, -518, -523, -528, -533, -539, -544, -549, -554, -560, -565, -570, -576, -581, -587, -592, -597, -603, -608, -614, -619, -624, -630, -635, -641, -646, -652, -657, -663, -668, -674, -680, -685, -691, -696, -702, -708, -713, -719, -725, -730, -736, -742, -747, -753, -759, -765, -770, -776, -782, -788, -794, -800, -805, -811, -817, -823, -829, -835, -841, -847, -853, -859, -865, -871, -877, -883, -889, -895, -901, -907, -914, -920, -926, -932, -938, -945, -951, -957, -963, -970, -976, -982, -989, -995, -1001, -1008, -1014, -1021, -1027, -1034, -1040, -1047, -1053, -1060, -1066, -1073, -1080, -1086, -1093, -1100, -1106, -1113, -1120, -1127, -1133, -1140, -1147, -1154, -1161, -1168, -1175, -1182, -1189, -1196, -1203, -1210, -1217, -1224, -1231, -1239, -1246, -1253, -1260, -1268, -1275, -1282, -1290, -1297, -1304, -1312, -1319, -1327, -1335, -1342, -1350, -1357, -1365, -1373, -1381, -1388, -1396, -1404, -1412, -1420, -1428, -1436, -1444, -1452, -1460, -1469, -1477, -1485, -1493, -1502, -1510, -1519, -1527, -1536, -1544, -1553, -1562, -1570, -1579, -1588, -1597, -1606, -1615, -1624, -1633, -1643, -1652, -1661, -1671, -1680, -1690, -1699, -1709, -1719, -1728, -1738, -1748, -1758, -1769, -1779, -1789, -1800, -1810, -1821, -1832, -1842, -1853, -1864, -1876, -1887, -1898, -1910, -1922, -1934, -1946, -1958, -1970, -1983, -1995, -2008, -2021, -2035, -2048, -2062, -2076, -2090, -2105, -2120, -2135, -2151, -2167, -2184, -2201, -2218, -2236, -2255, -2275, -2296, -2317, -2341, -2365, -2392, -2422, -2456, -2497, -2555, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000},
    {10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 564, 560, 557, 553, 549, 546, 542, 538, 535, 531, 527, 523, 520, 516, 512, 509, 505, 501, 497, 494, 490, 486, 482, 479, 475, 471, 467, 464, 460, 456, 452, 448, 445, 441, 437, 433, 429, 426, 422, 418, 414, 410, 406, 403, 399, 395, 391, 387, 383, 379, 376, 372, 368, 364, 360, 356, 352, 348, 344, 341, 337, 333, 329, 325, 321, 317, 313, 309, 305, 301, 297, 293, 289, 285, 281, 277, 273, 269, 265, 261, 257, 253, 249, 245, 241, 237, 233, 229, 225, 221, 217, 213, 209, 205, 200, 196, 192, 188, 184, 180, 176, 172, 168, 163, 159, 155, 151, 147, 143, 138, 134, 130, 126, 122, 118, 113, 109, 105, 101, 96, 92, 88, 84, 79, 75, 71, 67, 62, 58, 54, 50, 45, 41, 37, 32, 28, 24, 19, 15, 11, 6, 2, -2, -7, -11, -15, -20, -24, -29, -33, -37, -42, -46, -51, -55, -60, -64, -69, -73, -77, -82, -86, -91, -95, -100, -104, -109, -113, -118, -123, -127, -132, -136, -141, -145, -150, -154, -159, -164, -168, -173, -178, -182, -187, -191, -196, -201, -205, -210, -215, -219, -224, -229, -234, -238, -243, -248, -252, -257, -262, -267, -272, -276, -281, -286, -291, -296, -300, -305, -310, -315, -320, -325, -329, -334, -339, -344, -349, -354, -359, -364, -369, -374, -379, -384, -389, -394, -399, -404, -409, -414, -419, -424, -429, -434, -439, -444, -449, -454, -459, -464, -469, -475, -480, -485, -490, -495, -500, -506, -511, -516, -521, -526, -532, -537, -542, -547, -553, -558, -563, -569, -574, -579, -585, -590, -595, -601, -606, -612, -617, -622, -628, -633, -639, -644, -650, -655, -661, -666, -672, -677, -683, -688, -694, -700, -705, -711, -717, -722, -728, -733, -739, -745, -751, -756, -762, -768, -774, -779, -785, -791, -797, -803, -808, -814, -820, -826, -832, -838, -844, -850, -856, -862, -868, -874, -880, -886, -892, -898, -904, -910, -916, -922, -928, -935, -941, -947, -953, -960, -966, -972, -978, -985, -991, -997, -1004, -1010, -1016, -1023, -1029, -1036, -1042, -1049, -1055, -1062, -1068, -1075, -1081, -1088, -1095, -1101, -1108, -1115, -1121, -1128, -1135, -1142, -1149, -1155, -1162, -1169, -1176, -1183, -1190, -1197, -1204, -1211, -1218, -1225, -1232, -1239, -1246, -1254, -1261, -1268, -1275, -1283, -1290, -1297, -1305, -1312, -1319, -1327, -1334, -1342, -1349, -1357, -1365, -1372, -1380, -1388, -1395, -1403, -1411, -1419, -1427, -1435, -1443, -1451, -1459, -1467, -1475, -1483, -1491, -1500, -1508, -1516, -1525, -1533, -1541, -1550, -1559, -1567, -1576, -1585, -1593, -1602, -1611, -1620, -1629, -1638, -1647, -1656, -1665, -1675, -1684, -1693, -1703, -1712, -1722, -1732, -1741, -1751, -1761, -1771, -1781, -1791, -1801, -1812, -1822, -1833, -1843, -1854, -1865, -1876, -1887, -1898, -1909, -1921, -1932, -1944, -1956, -1968, -1980, -1992, -2004, -2017, -2030, -2043, -2056, -2069, -2083, -2097, -2111, -2126, -2140, -2155, -2171, -2187, -2203, -2220, -2237, -2255, -2273, -2292, -2312, -2333, -2355, -2378, -2404, -2431, -2462, -2497, -2540, -2606, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000},
    {10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 605, 602, 598, 594, 591, 587, 583, 580, 576, 572, 569, 565, 561, 558, 554, 550, 547, 543, 539, 536, 532, 528, 524, 521, 517, 513, 510, 506, 502, 498, 495, 491, 487, 483, 480, 476, 472, 468, 464, 461, 457, 453, 449, 446, 442, 438, 434, 430, 426, 423, 419, 415, 411, 407, 403, 400, 396, 392, 388, 384, 380, 376, 373, 369, 365, 361, 357, 353, 349, 345, 341, 337, 333, 329, 326, 322, 318, 314, 310, 306, 302, 298, 294, 290, 286, 282, 278, 274, 270, 266, 262, 258, 254, 250, 246, 242, 238, 234, 230, 226, 221, 217, 213, 209, 205, 201, 197, 193, 189, 185, 181, 176, 172, 168, 164, 160, 156, 152, 147, 143, 139, 135, 131, 127, 122, 118, 114, 110, 106, 101, 97, 93, 89, 84, 80, 76, 72, 67, 63, 59, 55, 50, 46, 42, 37, 33, 29, 24, 20, 16, 11, 7, 3, -2, -6, -10, -15, -19, -24, -28, -32, -37, -41, -46, -50, -55, -59, -63, -68, -72, -77, -81, -86, -90, -95, -99, -104, -108, -113, -117, -122, -126, -131, -135, -140, -145, -149, -154, -158, -163, -168, -172, -177, -181, -186, -191, -195, -200, -205, -209, -214, -219, -223, -228, -233, -237, -242, -247, -252, -256, -261, -266, -271, -275, -280, -285, -290, -295, -299, -304, -309, -314, -319, -324, -329, -333, -338, -343, -348, -353, -358, -363, -368, -373, -378, -383, -388, -393, -397, -402, -407, -412, -418, -423, -428, -433, -438, -443, -448, -453, -458, -463, -468, -473, -478, -484, -489, -494, -499, -504, -509, -515, -520, -525, -530, -536, -541, -546, -551, -557, -562, -567, -572, -578, -583, -588, -594, -599, -605, -610, -615, -621, -626, -632, -637, -642, -648, -653, -659, -664, -670, -675, -681, -687, -692, -698, -703, -709, -714, -720, -726, -731, -737, -743, -748, -754, -760, -765, -771, -777, -783, -788, -794, -800, -806, -812, -818, -823, -829, -835, -841, -847, -853, -859, -865, -871, -877, -883, -889, -895, -901, -907, -913, -919, -925, -931, -938, -944, -950, -956, -962, -969, -975, -981, -987, -994, -1000, -1006, -1013, -1019, -1025, -1032, -1038, -1045, -1051, -1058, -1064, -1071, -1077, -1084, -1090, -1097, -1103, -1110, -1117, -1123, -1130, -1137, -1144, -1150, -1157, -1164, -1171, -1178, -1184, -1191, -1198, -1205, -1212, -1219, -1226, -1233, -1240, -1247, -1255, -1262, -1269, -1276, -1283, -1291, -1298, -1305, -1312, -1320, -1327, -1335, -1342, -1350, -1357, -1365, -1372, -1380, -1387, -1395, -1403, -1411, -1418, -1426, -1434, -1442, -1450, -1458, -1466, -1474, -1482, -1490, -1498, -1506, -1514, -1523, -1531, -1539, -1548, -1556, -1565, -1573, -1582, -1590, -1599, -1608, -1616, -1625, -1634, -1643, -1652, -1661, -1670, -1679, -1688, -1698, -1707, -1716, -1726, -1735, -1745, -1755, -1764, -1774, -1784, -1794, -1804, -1814, -1824, -1835, -1845, -1856, -1866, -1877, -1888, -1899, -1909, -1921, -1932, -1943, -1955, -1966, -1978, -1990, -2002, -2014, -2026, -2039, -2052, -2065, -2078, -2091, -2104, -2118, -2132, -2146, -2161, -2176, -2191, -2207, -2223, -2239, -2256, -2273, -2291, -2310, -2329, -2349, -2371, -2393, -2417, -2442, -2470, -2502, -2538, -2584, -2665, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000},
    {10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 642, 639, 635, 631, 628, 624, 621, 617, 613, 610, 606, 602, 599, 595, 592, 588, 584, 581, 577, 573, 570, 566, 562, 559, 555, 551, 547, 544, 540, 536, 533, 529, 525, 522, 518, 514, 510, 507, 503, 499, 495, 492, 488, 484, 480, 477, 473, 469, 465, 461, 458, 454, 450, 446, 442, 439, 435, 431, 427, 423, 419, 416, 412, 408, 404, 400, 396, 393, 389, 385, 381, 377, 373, 369, 365, 361, 358, 354, 350, 346, 342, 338, 334, 330, 326, 322, 318, 314, 310, 306, 302, 298, 294, 290, 286, 282, 278, 274, 270, 266, 262, 258, 254, 250, 246, 242, 238, 234, 230, 226, 222, 218, 214, 210, 206, 202, 197, 193, 189, 185, 181, 177, 173, 169, 165, 160, 156, 152, 148, 144, 140, 135, 131, 127, 123, 119, 114, 110, 106, 102, 98, 93, 89, 85, 81, 76, 72, 68, 64, 59, 55, 51, 46, 42, 38, 33, 29, 25, 20, 16, 12, 7, 3, -1, -6, -10, -14, -19, -23, -28, -32, -36, -41, -45, -50, -54, -58, -63, -67, -72, -76, -81, -85, -90, -94, -99, -103, -108, -112, -117, -121, -126, -130, -135, -140, -144, -149, -153, -158, -162, -167, -172, -176, -181, -185, -190, -195, -199, -204, -209, -213, -218, -223, -227, -232, -237, -242, -246, -251, -256, -261, -265, -270, -275, -280, -284, -289, -294, -299, -304, -308, -313, -318, -323, -328, -333, -338, -342, -347, -352, -357, -362, -367, -372, -377, -382, -387, -392, -397, -402, -407, -412, -417, -422, -427, -432, -437, -442, -447, -452, -457, -462, -467, -472, -477, -483, -488, -493, -498, -503, -508, -513, -519, -524, -529, -534, -540, -545, -550, -555, -561, -566, -571, -576, -582, -587, -592, -598, -603, -609, -614, -619, -625, -630, -636, -641, -646, -652, -657, -663, -668, -674, -679, -685, -690, -696, -701, -707, -713, -718, -724, -729, -735, -741, -746, -752, -758, -763, -769, -775, -781, -786, -792, -798, -804, -809, -815, -821, -827, -833, -839, -845, -850, -856, -862, -868, -874, -880, -886, -892, -898, -904, -910, -916, -922, -928, -935, -941, -947, -953, -959, -965, -972, -978, -984, -990, -996, -1003, -1009, -1015, -1022, -1028, -1034, -1041, -1047, -1054, -1060, -1067, -1073, -1080, -1086, -1093, -1099, -1106, -1112, -1119, -1126, -1132, -1139, -1146, -1152, -1159, -1166, -1173, -1180, -1186, -1193, -1200, -1207, -1214, -1221, -1228, -1235, -1242, -1249, -1256, -1263, -1270, -1277, -1284, -1292, -1299, -1306, -1313, -1321, -1328, -1335, -1343, -1350, -1358, -1365, -1373, -1380, -1388, -1395, -1403, -1410, -1418, -1426, -1434, -1441, -1449, -1457, -1465, -1473, -1481, -1489, -1497, -1505, -1513, -1521, -1529, -1538, -1546, -1554, -1563, -1571, -1579, -1588, -1596, -1605, -1614, -1622, -1631, -1640, -1649, -1657, -1666, -1675, -1684, -1693, -1703, -1712, -1721, -1730, -1740, -1749, -1759, -1768, -1778, -1788, -1798, -1807, -1817, -1827, -1837, -1848, -1858, -1868, -1879, -1889, -1900, -1911, -1922, -1932, -1944, -1955, -1966, -1977, -1989, -2001, -2012, -2024, -2037, -2049, -2061, -2074, -2087, -2100, -2113, -2126, -2140, -2154, -2168, -2182, -2197, -2212, -2227, -2243, -2259, -2276, -2293, -2310, -2328, -2347, -2367, -2387, -2409, -2431, -2455, -2482, -2510, -2542, -2580, -2630, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000},
    {10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 683, 679, 675, 672, 668, 665, 661, 658, 654, 650, 647, 643, 640, 636, 632, 629, 625, 621, 618, 614, 611, 607, 603, 600, 596, 592, 589, 585, 581, 578, 574, 570, 567, 563, 559, 556, 552, 548, 545, 541, 537, 533, 530, 526, 522, 518, 515, 511, 507, 504, 500, 496, 492, 488, 485, 481, 477, 473, 470, 466, 462, 458, 454, 451, 447, 443, 439, 435, 432, 428, 424, 420, 416, 412, 408, 405, 401, 397, 393, 389, 385, 381, 378, 374, 370, 366, 362, 358, 354, 350, 346, 342, 338, 334, 331, 327, 323, 319, 315, 311, 307, 303, 299, 295, 291, 287, 283, 279, 275, 271, 267, 263, 259, 255, 251, 247, 243, 239, 235, 230, 226, 222, 218, 214, 210, 206, 202, 198, 194, 190, 185, 181, 177, 173, 169, 165, 161, 157, 152, 148, 144, 140, 136, 131, 127, 123, 119, 115, 110, 106, 102, 98, 94, 89, 85, 81, 77, 72, 68, 64, 60, 55, 51, 47, 42, 38, 34, 29, 25, 21, 16, 12, 8, 3, -1, -5, -10, -14, -18, -23, -27, -32, -36, -40, -45, -49, -54, -58, -63, -67, -72, -76, -80, -85, -89, -94, -98, -103, -107, -112, -116, -121, -126, -130, -135, -139, -144, -148, -153, -157, -162, -167, -171, -176, -180, -185, -190, -194, -199, -204, -208, -213, -218, -222, -227, -232, -236, -241, -246, -251, -255, -260, -265, -270, -274, -279, -284, -289, -293, -298, -303, -308, -313, -318, -322, -327, -332, -337, -342, -347, -352, -357, -361, -366, -371, -376, -381, -386, -391, -396, -401, -406, -411, -416, -421, -426, -431, -436, -441, -446, -451, -456, -461, -466, -471, -477, -482, -487, -492, -497, -502, -507, -513, -518, -523, -528, -533, -539, -544, -549, -554, -560, -565, -570, -575, -581, -586, -591, -597, -602, -607, -613, -618, -623, -629, -634, -640, -645, -651, -656, -661, -667, -672, -678, -683, -689, -694, -700, -706, -711, -717, -722, -728, -733, -739, -745, -750, -756, -762, -767, -773, -779, -784, -790, -796, -802, -807, -813, -819, -825, -831, -837, -842, -848, -854, -860, -866, -872, -878, -884, -890, -896, -902, -908, -914, -920, -926, -932, -938, -944, -950, -956, -962, -969, -975, -981, -987, -993, -1000, -1006, -1012, -1019, -1025, -1031, -1037, -1044, -1050, -1057, -1063, -1069, -1076, -1082, -1089, -1095, -1102, -1108, -1115, -1122, -1128, -1135, -1141, -1148, -1155, -1162, -1168, -1175, -1182, -1189, -1195, -1202, -1209, -1216, -1223, -1230, -1237, -1244, -1251, -1258, -1265, -1272, -1279, -1286, -1293, -1300, -1307, -1315, -1322, -1329, -1336, -1344, -1351, -1358, -1366, -1373, -1381, -1388, -1396, -1403, -1411, -1418, -1426, -1434, -1441, -1449, -1457, -1465, -1473, -1480, -1488, -1496, -1504, -1512, -1520, -1528, -1537, -1545, -1553, -1561, -1569, -1578, -1586, -1595, -1603, -1611, -1620, -1629, -1637, -1646, -1655, -1663, -1672, -1681, -1690, -1699, -1708, -1717, -1726, -1735, -1745, -1754, -1763, -1773, -1782, -1792, -1802, -1811, -1821, -1831, -1841, -1851, -1861, -1871, -1881, -1892, -1902, -1913, -1923, -1934, -1945, -1956, -1967, -1978, -1989, -2001, -2012, -2024, -2035, -2047, -2059, -2072, -2084, -2096, -2109, -2122, -2135, -2148, -2162, -2175, -2189, -2204, -2218, -2233, -2248, -2264, -2279, -2296, -2312, -2329, -2347, -2365, -2384, -2404, -2425, -2447, -2470, -2494, -2521, -2550, -2584, -2623, -2676, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000},
    {10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 722, 719, 715, 712, 708, 705, 701, 698, 694, 690, 687, 683, 680, 676, 673, 669, 665, 662, 658, 655, 651, 647, 644, 640, 637, 633, 629, 626, 622, 618, 615, 611, 608, 604, 600, 597, 593, 589, 586, 582, 578, 575, 571, 567, 564, 560, 556, 552, 549, 545, 541, 538, 534, 530, 526, 523, 519, 515, 512, 508, 504, 500, 496, 493, 489, 485, 481, 478, 474, 470, 466, 462, 459, 455, 451, 447, 443, 440, 436, 432, 428, 424, 420, 417, 413, 409, 405, 401, 397, 393, 390, 386, 382, 378, 374, 370, 366, 362, 358, 354, 350, 347, 343, 339, 335, 331, 327, 323, 319, 315, 311, 307, 303, 299, 295, 291, 287, 283, 279, 275, 271, 267, 263, 259, 255, 251, 247, 243, 239, 235, 231, 227, 223, 218, 214, 210, 206, 202, 198, 194, 190, 186, 182, 177, 173, 169, 165, 161, 157, 153, 148, 144, 140, 136, 132, 127, 123, 119, 115, 111, 106, 102, 98, 94, 89, 85, 81, 77, 72, 68, 64, 60, 55, 51, 47, 42, 38, 34, 30, 25, 21, 17, 12, 8, 4, -1, -5, -10, -14, -18, -23, -27, -32, -36, -40, -45, -49, -54, -58, -63, -67, -71, -76, -80, -85, -89, -94, -98, -103, -107, -112, -116, -121, -125, -130, -134, -139, -144, -148, -153, -157, -162, -166, -171, -176, -180, -185, -190, -194, -199, -203, -208, -213, -217, -222, -227, -232, -236, -241, -246, -250, -255, -260, -265, -269, -274, -279, -284, -288, -293, -298, -303, -308, -312, -317, -322, -327, -332, -337, -341, -346, -351, -356, -361, -366, -371, -376, -381, -386, -391, -396, -400, -405, -410, -415, -420, -425, -430, -435, -441, -446, -451, -456, -461, -466, -471, -476, -481, -486, -491, -496, -502, -507, -512, -517, -522, -527, -533, -538, -543, -548, -553, -559, -564, -569, -575, -580, -585, -590, -596, -601, -606, -612, -617, -622, -628, -633, -639, -644, -649, -655, -660, -666, -671, -677, -682, -688, -693, -699, -704, -710, -715, -721, -726, -732, -738, -743, -749, -755, -760, -766, -771, -777, -783, -789, -794, -800, -806, -812, -817, -823, -829, -835, -841, -846, -852, -858, -864, -870, -876, -882, -888, -894, -900, -905, -911, -917, -924, -930, -936, -942, -948, -954, -960, -966, -972, -978, -985, -991, -997, -1003, -1009, -1016, -1022, -1028, -1034, -1041, -1047, -1053, -1060, -1066, -1073, -1079, -1085, -1092, -1098, -1105, -1111, -1118, -1124, -1131, -1138, -1144, -1151, -1158, -1164, -1171, -1178, -1184, -1191, -1198, -1205, -1211, -1218, -1225, -1232, -1239, -1246, -1253, -1260, -1267, -1274, -1281, -1288, -1295, -1302, -1309, -1316, -1323, -1331, -1338, -1345, -1352, -1360, -1367, -1374, -1382, -1389, -1397, -1404, -1412, -1419, -1427, -1434, -1442, -1450, -1457, -1465, -1473, -1481, -1488, -1496, -1504, -1512, -1520, -1528, -1536, -1544, -1552, -1560, -1568, -1577, -1585, -1593, -1602, -1610, -1618, -1627, -1635, -1644, -1652, -1661, -1670, -1678, -1687, -1696, -1705, -1714, -1723, -1732, -1741, -1750, -1759, -1769, -1778, -1787, -1797, -1806, -1816, -1825, -1835, -1845, -1855, -1865, -1875, -1885, -1895, -1905, -1915, -1926, -1936, -1947, -1958, -1968, -1979, -1990, -2001, -2013, -2024, -2035, -2047, -2059, -2070, -2082, -2094, -2107, -2119, -2132, -2145, -2157, -2171, -2184, -2198, -2211, -2225, -2240, -2254, -2269, -2285, -2300, -2316, -2332, -2349, -2367, -2384, -2403, -2422, -2442, -2463, -2485, -2509, -2534, -2561, -2591, -2625, -2666, -2724, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000},
    {10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 762, 758, 755, 751, 748, 744, 741, 737, 734, 730, 727, 723, 720, 716, 712, 709, 705, 702, 698, 695, 691, 688, 684, 680, 677, 673, 670, 666, 662, 659, 655, 652, 648, 644, 641, 637, 634, 630, 626, 623, 619, 615, 612, 608, 604, 601, 597, 593, 590, 586, 582, 579, 575, 571, 568, 564, 560, 557, 553, 549, 545, 542, 538, 534, 531, 527, 523, 519, 516, 512, 508, 504, 501, 497, 493, 489, 486, 482, 478, 474, 470, 467, 463, 459, 455, 451, 448, 444, 440, 436, 432, 428, 425, 421, 417, 413, 409, 405, 401, 397, 394, 390, 386, 382, 378, 374, 370, 366, 362, 359, 355, 351, 347, 343, 339, 335, 331, 327, 323, 319, 315, 311, 307, 303, 299, 295, 291, 287, 283, 279, 275, 271, 267, 263, 259, 255, 251, 247, 243, 239, 235, 231, 227, 223, 218, 214, 210, 206, 202, 198, 194, 190, 186, 182, 177, 173, 169, 165, 161, 157, 153, 148, 144, 140, 136, 132, 127, 123, 119, 115, 111, 106, 102, 98, 94, 89, 85, 81, 77, 72, 68, 64, 60, 55, 51, 47, 42, 38, 34, 30, 25, 21, 17, 12, 8, 3, -1, -5, -10, -14, -18, -23, -27, -32, -36, -40, -45, -49, -54, -58, -63, -67, -71, -76, -80, -85, -89, -94, -98, -103, -107, -112, -116, -121, -125, -130, -134, -139, -144, -148, -153, -157, -162, -166, -171, -176, -180, -185, -190, -194, -199, -203, -208, -213, -217, -222, -227, -231, -236, -241, -246, -250, -255, -260, -264, -269, -274, -279, -284, -288, -293, -298, -303, -307, -312, -317, -322, -327, -332, -336, -341, -346, -351, -356, -361, -366, -371, -376, -380, -385, -390, -395, -400, -405, -410, -415, -420, -425, -430, -435, -440, -445, -450, -455, -460, -465, -470, -476, -481, -486, -491, -496, -501, -506, -511, -517, -522, -527, -532, -537, -542, -548, -553, -558, -563, -569, -574, -579, -584, -590, -595, -600, -606, -611, -616, -622, -627, -632, -638, -643, -649, -654, -659, -665, -670, -676, -681, -687, -692, -698, -703, -709, -714, -720, -725, -731, -736, -742, -748, -753, -759, -765, -770, -776, -782, -787, -793, -799, -804, -810, -816, -822, -827, -833, -839, -845, -851, -856, -862, -868, -874, -880, -886, -892, -898, -904, -910, -915, -921, -927, -933, -940, -946, -952, -958, -964, -970, -976, -982, -988, -994, -1001, -1007, -1013, -1019, -1025, -1032, -1038, -1044, -1051, -1057, -1063, -1070, -1076, -1082, -1089, -1095, -1102, -1108, -1115, -1121, -1128, -1134, -1141, -1147, -1154, -1161, -1167, -1174, -1180, -1187, -1194, -1201, -1207, -1214, -1221, -1228, -1235, -1241, -1248, -1255, -1262, -1269, -1276, -1283, -1290, -1297, -1304, -1311, -1318, -1325, -1333, -1340, -1347, -1354, -1361, -1369, -1376, -1383, -1391, -1398, -1405, -1413, -1420, -1428, -1435, -1443, -1451, -1458, -1466, -1473, -1481, -1489, -1497, -1504, -1512, -1520, -1528, -1536, -1544, -1552, -1560, -1568, -1576, -1584, -1592, -1601, -1609, -1617, -1626, -1634, -1642, -1651, -1659, -1668, -1676, -1685, -1694, -1703, -1711, -1720, -1729, -1738, -1747, -1756, -1765, -1774, -1783, -1793, -1802, -1811, -1821, -1830, -1840, -1849, -1859, -1869, -1879, -1889, -1899, -1909, -1919, -1929, -1939, -1950, -1960, -1971, -1981, -1992, -2003, -2014, -2025, -2036, -2047, -2059, -2070, -2082, -2094, -2106, -2118, -2130, -2142, -2155, -2167, -2180, -2193, -2207, -2220, -2234, -2248, -2262, -2276, -2291, -2306, -2321, -2337, -2353, -2369, -2386, -2404, -2422, -2441, -2460, -2480, -2501, -2524, -2547, -2573, -2600, -2631, -2666, -2710, -2775, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000},
    {10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 801, 797, 794, 790, 787, 783, 780, 776, 773, 769, 766, 762, 759, 755, 752, 748, 745, 741, 738, 734, 731, 727, 724, 720, 717, 713, 709, 706, 702, 699, 695, 692, 688, 684, 681, 677, 674, 670, 667, 663, 659, 656, 652, 648, 645, 641, 638, 634, 630, 627, 623, 619, 616, 612, 608, 605, 601, 597, 594, 590, 586, 583, 579, 575, 572, 568, 564, 561, 557, 553, 549, 546, 542, 538, 535, 531, 527, 523, 520, 516, 512, 508, 505, 501, 497, 493, 489, 486, 482, 478, 474, 471, 467, 463, 459, 455, 451, 448, 444, 440, 436, 432, 428, 425, 421, 417, 413, 409, 405, 401, 398, 394, 390, 386, 382, 378, 374, 370, 366, 362, 359, 355, 351, 347, 343, 339, 335, 331, 327, 323, 319, 315, 311, 307, 303, 299, 295, 291, 287, 283, 279, 275, 271, 267, 263, 259, 255, 251, 247, 243, 239, 235, 231, 227, 222, 218, 214, 210, 206, 202, 198, 194, 190, 186, 181, 177, 173, 169, 165, 161, 157, 152, 148, 144, 140, 136, 131, 127, 123, 119, 115, 110, 106, 102, 98, 94, 89, 85, 81, 77, 72, 68, 64, 59, 55, 51, 47, 42, 38, 34, 29, 25, 21, 16, 12, 8, 3, -1, -5, -10, -14, -19, -23, -27, -32, -36, -41, -45, -49, -54, -58, -63, -67, -72, -76, -81, -85, -90, -94, -98, -103, -107, -112, -117, -121, -126, -130, -135, -139, -144, -148, -153, -157, -162, -167, -171, -176, -180, -185, -190, -194, -199, -204, -208, -213, -218, -222, -227, -232, -236, -241, -246, -250, -255, -260, -265, -269, -274, -279, -284, -288, -293, -298, -303, -307, -312, -317, -322, -327, -332, -336, -341, -346, -351, -356, -361, -366, -371, -375, -380, -385, -390, -395, -400, -405, -410, -415, -420, -425, -430, -435, -440, -445, -450, -455, -460, -465, -470, -475, -480, -486, -491, -496, -501, -506, -511, -516, -521, -527, -532, -537, -542, -547, -552, -558, -563, -568, -573, -579, -584, -589, -595, -600, -605, -610, -616, -621, -626, -632, -637, -643, -648, -653, -659, -664, -670, -675, -680, -686, -691, -697, -702, -708, -713, -719, -724, -730, -736, -741, -747, -752, -758, -763, -769, -775, -780, -786, -792, -797, -803, -809, -815, -820, -826, -832, -838, -843, -849, -855, -861, -867, -872, -878, -884, -890, -896, -902, -908, -914, -920, -926, -932, -938, -944, -950, -956, -962, -968, -974, -980, -986, -992, -998, -1005, -1011, -1017, -1023, -1029, -1036, -1042, -1048, -1054, -1061, -1067, -1073, -1080, -1086, -1092, -1099, -1105, -1112, -1118, -1125, -1131, -1138, -1144, -1151, -1157, -1164, -1170, -1177, -1184, -1190, -1197, -1204, -1210, -1217, -1224, -1231, -1237, -1244, -1251, -1258, -1265, -1272, -1279, -1286, -1293, -1299, -1306, -1314, -1321, -1328, -1335, -1342, -1349, -1356, -1363, -1371, -1378, -1385, -1392, -1400, -1407, -1414, -1422, -1429, -1437, -1444, -1452, -1459, -1467, -1474, -1482, -1490, -1497, -1505, -1513, -1521, -1528, -1536, -1544, -1552, -1560, -1568, -1576, -1584, -1592, -1600, -1608, -1617, -1625, -1633, -1641, -1650, -1658, -1667, -1675, -1684, -1692, -1701, -1709, -1718, -1727, -1736, -1744, -1753, -1762, -1771, -1780, -1789, -1799, -1808, -1817, -1826, -1836, -1845, -1855, -1864, -1874, -1883, -1893, -1903, -1913, -1923, -1933, -1943, -1953, -1963, -1974, -1984, -1995, -2005, -2016, -2027, -2038, -2049, -2060, -2071, -2083, -2094, -2106, -2117, -2129, -2141, -2153, -2165, -2178, -2190, -2203, -2216, -2229, -2243, -2256, -2270, -2284, -2298, -2313, -2327, -2343, -2358, -2374, -2390, -2407, -2424, -2441, -2460, -2478, -2498, -2518, -2540, -2562, -2586, -2612, -2640, -2672, -2708, -2754, -2829, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000},
    {10000, 10000, 10000, 839, 836, 833, 829, 826, 822, 819, 815, 812, 808, 805, 801, 798, 794, 791, 787, 784, 780, 777, 773, 770, 766, 763, 759, 756, 752, 749, 745, 742, 738, 735, 731, 728, 724, 721, 717, 713, 710, 706, 703, 699, 696, 692, 688, 685, 681, 678, 674, 670, 667, 663, 660, 656, 652, 649, 645, 642, 638, 634, 631, 627, 623, 620, 616, 612, 609, 605, 601, 598, 594, 590, 587, 583, 579, 576, 572, 568, 564, 561, 557, 553, 550, 546, 542, 538, 535, 531, 527, 523, 520, 516, 512, 508, 505, 501, 497, 493, 490, 486, 482, 478, 474, 471, 467, 463, 459, 455, 451, 448, 444, 440, 436, 432, 428, 425, 421, 417, 413, 409, 405, 401, 397, 394, 390, 386, 382, 378, 374, 370, 366, 362, 358, 354, 351, 347, 343, 339, 335, 331, 327, 323, 319, 315, 311, 307, 303, 299, 295, 291, 287, 283, 279, 275, 271, 267, 263, 259, 255, 251, 247, 243, 239, 234, 230, 226, 222, 218, 214, 210, 206, 202, 198, 194, 189, 185, 181, 177, 173, 169, 165, 160, 156, 152, 148, 144, 140, 135, 131, 127, 123, 119, 114, 110, 106, 102, 97, 93, 89, 85, 80, 76, 72, 68, 63, 59, 55, 51, 46, 42, 38, 33, 29, 25, 20, 16, 12, 7, 3, -1, -6, -10, -15, -19, -23, -28, -32, -37, -41, -45, -50, -54, -59, -63, -68, -72, -76, -81, -85, -90, -94, -99, -103, -108, -112, -117, -121, -126, -130, -135, -140, -144, -149, -153, -158, -162, -167, -172, -176, -181, -185, -190, -195, -199, -204, -209, -213, -218, -222, -227, -232, -237, -241, -246, -251, -255, -260, -265, -270, -274, -279, -284, -289, -293, -298, -303, -308, -312, -317, -322, -327, -332, -337, -341, -346, -351, -356, -361, -366, -371, -376, -380, -385, -390, -395, -400, -405, -410, -415, -420, -425, -430, -435, -440, -445, -450, -455, -460, -465, -470, -475, -480, -485, -491, -496, -501, -506, -511, -516, -521, -526, -532, -537, -542, -547, -552, -557, -563, -568, -573, -578, -584, -589, -594, -599, -605, -610, -615, -621, -626, -631, -637, -642, -647, -653, -658, -664, -669, -674, -680, -685, -691, -696, -702, -707, -713, -718, -724, -729, -735, -740, -746, -751, -757, -763, -768, -774, -779, -785, -791, -796, -802, -808, -814, -819, -825, -831, -836, -842, -848, -854, -860, -865, -871, -877, -883, -889, -895, -900, -906, -912, -918, -924, -930, -936, -942, -948, -954, -960, -966, -972, -978, -984, -990, -996, -1003, -1009, -1015, -1021, -1027, -1033, -1040, -1046, -1052, -1058, -1065, -1071, -1077, -1084, -1090, -1096, -1103, -1109, -1115, -1122, -1128, -1135, -1141, -1148, -1154, -1161, -1167, -1174, -1180, -1187, -1194, -1200, -1207, -1214, -1220, -1227, -1234, -1241, -1247, -1254, -1261, -1268, -1275, -1281, -1288, -1295, -1302, -1309, -1316, -1323, -1330, -1337, -1344, -1351, -1359, -1366, -1373, -1380, -1387, -1395, -1402, -1409, -1416, -1424, -1431, -1438, -1446, -1453, -1461, -1468, -1476, -1483, -1491, -1499, -1506, -1514, -1522, -1529, -1537, -1545, -1553, -1561, -1569, -1576, -1584, -1592, -1600, -1608, -1617, -1625, -1633, -1641, -1649, -1658, -1666, -1674, -1683, -1691, -1700, -1708, -1717, -1725, -1734, -1743, -1751, -1760, -1769, -1778, -1787, -1796, -1805, -1814, -1823, -1832, -1842, -1851, -1860, -1870, -1879, -1889, -1898, -1908, -1918, -1927, -1937, -1947, -1957, -1967, -1978, -1988, -1998, -2009, -2019, -2030, -2040, -2051, -2062, -2073, -2084, -2095, -2106, -2118, -2129, -2141, -2153, -2165, -2177, -2189, -2201, -2214, -2226, -2239, -2252, -2265, -2279, -2292, -2306, -2320, -2335, -2349, -2364, -2379, -2395, -2411, -2427, -2444, -2461, -2479, -2497, -2516, -2536, -2557, -2578, -2601, -2625, -2652, -2680, -2712, -2750, -2797, -2894, 10000, 10000, 10000},
    {861, 857, 854, 850, 847, 843, 840, 837, 833, 830, 826, 823, 819, 816, 812, 809, 805, 802, 798, 795, 791, 788, 784, 781, 777, 774, 770, 767, 763, 760, 756, 753, 749, 746, 742, 739, 735, 732, 728, 724, 721, 717, 714, 710, 707, 703, 699, 696, 692, 689, 685, 681, 678, 674, 671, 667, 663, 660, 656, 653, 649, 645, 642, 638, 634, 631, 627, 623, 620, 616, 612, 609, 605, 601, 598, 594, 590, 587, 583, 579, 576, 572, 568, 565, 561, 557, 553, 550, 546, 542, 538, 535, 531, 527, 523, 520, 516, 512, 508, 505, 501, 497, 493, 489, 486, 482, 478, 474, 470, 467, 463, 459, 455, 451, 447, 444, 440, 436, 432, 428, 424, 421, 417, 413, 409, 405, 401, 397, 393, 389, 386, 382, 378, 374, 370, 366, 362, 358, 354, 350, 346, 342, 338, 334, 331, 327, 323, 319, 315, 311, 307, 303, 299, 295, 291, 287, 283, 279, 275, 271, 267, 262, 258, 254, 250, 246, 242, 238, 234, 230, 226, 222, 218, 214, 210, 205, 201, 197, 193, 189, 185, 181, 177, 172, 168, 164, 160, 156, 152, 147, 143, 139, 135, 131, 127, 122, 118, 114, 110, 105, 101, 97, 93, 89, 84, 80, 76, 71, 67, 63, 59, 54, 50, 46, 41, 37, 33, 29, 24, 20, 16, 11, 7, 2, -2, -6, -11, -15, -19, -24, -28, -33, -37, -41, -46, -50, -55, -59, -64, -68, -72, -77, -81, -86, -90, -95, -99, -104, -108, -113, -117, -122, -126, -131, -135, -140, -145, -149, -154, -158, -163, -167, -172, -177, -181, -186, -190, -195, -200, -204, -209, -214, -218, -223, -228, -232, -237, -242, -246, -251, -256, -260, -265, -270, -275, -279, -284, -289, -294, -298, -303, -308, -313, -318, -322, -327, -332, -337, -342, -347, -351, -356, -361, -366, -371, -376, -381, -386, -391, -396, -400, -405, -410, -415, -420, -425, -430, -435, -440, -445, -450, -455, -460, -465, -470, -475, -480, -486, -491, -496, -501, -506, -511, -516, -521, -526, -532, -537, -542, -547, -552, -557, -563, -568, -573, -578, -584, -589, -594, -599, -605, -610, -615, -621, -626, -631, -636, -642, -647, -653, -658, -663, -669, -674, -680, -685, -690, -696, -701, -707, -712, -718, -723, -729, -734, -740, -745, -751, -756, -762, -768, -773, -779, -784, -790, -796, -801, -807, -813, -818, -824, -830, -836, -841, -847, -853, -859, -864, -870, -876, -882, -888, -893, -899, -905, -911, -917, -923, -929, -935, -941, -947, -953, -959, -965, -971, -977, -983, -989, -995, -1001, -1007, -1013, -1019, -1025, -1032, -1038, -1044, -1050, -1056, -1063, -1069, -1075, -1081, -1088, -1094, -1100, -1107, -1113, -1119, -1126, -1132, -1139, -1145, -1152, -1158, -1165, -1171, -1178, -1184, -1191, -1197, -1204, -1211, -1217, -1224, -1231, -1237, -1244, -1251, -1257, -1264, -1271, -1278, -1285, -1292, -1298, -1305, -1312, -1319, -1326, -1333, -1340, -1347, -1354, -1361, -1368, -1375, -1383, -1390, -1397, -1404, -1411, -1419, -1426, -1433, -1441, -1448, -1455, -1463, -1470, -1478, -1485, -1493, -1500, -1508, -1515, -1523, -1531, -1538, -1546, -1554, -1562, -1569, -1577, -1585, -1593, -1601, -1609, -1617, -1625, -1633, -1641, -1649, -1657, -1666, -1674, -1682, -1691, -1699, -1707, -1716, -1724, -1733, -1741, -1750, -1759, -1767, -1776, -1785, -1794, -1803, -1812, -1821, -1830, -1839, -1848, -1857, -1866, -1876, -1885, -1894, -1904, -1913, -1923, -1933, -1942, -1952, -1962, -1972, -1982, -1992, -2002, -2012, -2023, -2033, -2043, -2054, -2065, -2075, -2086, -2097, -2108, -2119, -2131, -2142, -2153, -2165, -2177, -2188, -2200, -2213, -2225, -2237, -2250, -2262, -2275, -2288, -2302, -2315, -2329, -2343, -2357, -2371, -2386, -2401, -2416, -2432, -2448, -2464, -2481, -2499, -2516, -2535, -2554, -2574, -2595, -2617, -2640, -2664, -2691},
    {871, 868, 865, 861, 858, 854, 851, 847, 844, 840, 837, 833, 830, 827, 823, 820, 816, 813, 809, 806, 802, 799, 795, 792, 788, 785, 781, 778, 774, 771, 767, 764, 760, 757, 753, 749, 746, 742, 739, 735, 732, 728, 725, 721, 717, 714, 710, 707, 703, 700, 696, 692, 689, 685, 682, 678, 674, 671, 667, 664, 660, 656, 653, 649, 645, 642, 638, 634, 631, 627, 623, 620, 616, 612, 609, 605, 601, 598, 594, 590, 587, 583, 579, 576, 572, 568, 564, 561, 557, 553, 549, 546, 542, 538, 535, 531, 527, 523, 519, 516, 512, 508, 504, 501, 497, 493, 489, 485, 482, 478, 474, 470, 466, 463, 459, 455, 451, 447, 443, 440, 436, 432, 428, 424, 420, 416, 412, 409, 405, 401, 397, 393, 389, 385, 381, 377, 373, 370, 366, 362, 358, 354, 350, 346, 342, 338, 334, 330, 326, 322, 318, 314, 310, 306, 302, 298, 294, 290, 286, 282, 278, 274, 270, 266, 262, 258, 254, 250, 246, 242, 238, 234, 230, 225, 221, 217, 213, 209, 205, 201, 197, 193, 188, 184, 180, 176, 172, 168, 164, 159, 155, 151, 147, 143, 139, 134, 130, 126, 122, 118, 113, 109, 105, 101, 96, 92, 88, 84, 79, 75, 71, 67, 62, 58, 54, 49, 45, 41, 37, 32, 28, 24, 19, 15, 11, 6, 2, -3, -7, -11, -16, -20, -24, -29, -33, -38, -42, -46, -51, -55, -60, -64, -69, -73, -78, -82, -86, -91, -95, -100, -104, -109, -113, -118, -122, -127, -132, -136, -141, -145, -150, -154, -159, -163, -168, -173, -177, -182, -186, -191, -196, -200, -205, -210, -214, -219, -224, -228, -233, -238, -242, -247, -252, -256, -261, -266, -270, -275, -280, -285, -289, -294, -299, -304, -309, -313, -318, -323, -328, -333, -337, -342, -347, -352, -357, -362, -367, -371, -376, -381, -386, -391, -396, -401, -406, -411, -416, -421, -426, -431, -436, -441, -446, -451, -456, -461, -466, -471, -476, -481, -486, -491, -496, -501, -506, -511, -516, -522, -527, -532, -537, -542, -547, -552, -558, -563, -568, -573, -578, -584, -589, -594, -599, -605, -610, -615, -621, -626, -631, -636, -642, -647, -652, -658, -663, -669, -674, -679, -685, -690, -696, -701, -707, -712, -717, -723, -728, -734, -739, -745, -751, -756, -762, -767, -773, -778, -784, -790, -795, -801, -806, -812, -818, -823, -829, -835, -841, -846, -852, -858, -864, -869, -875, -881, -887, -893, -898, -904, -910, -916, -922, -928, -934, -940, -945, -951, -957, -963, -969, -975, -981, -987, -993, -999, -1006, -1012, -1018, -1024, -1030, -1036, -1042, -1048, -1055, -1061, -1067, -1073, -1080, -1086, -1092, -1098, -1105, -1111, -1117, -1124, -1130, -1136, -1143, -1149, -1156, -1162, -1169, -1175, -1182, -1188, -1195, -1201, -1208, -1214, -1221, -1228, -1234, -1241, -1248, -1254, -1261, -1268, -1275, -1281, -1288, -1295, -1302, -1309, -1316, -1322, -1329, -1336, -1343, -1350, -1357, -1364, -1371, -1378, -1385, -1393, -1400, -1407, -1414, -1421, -1428, -1436, -1443, -1450, -1458, -1465, -1472, -1480, -1487, -1495, -1502, -1510, -1517, -1525, -1532, -1540, -1548, -1555, -1563, -1571, -1578, -1586, -1594, -1602, -1610, -1618, -1626, -1634, -1642, -1650, -1658, -1666, -1674, -1682, -1691, -1699, -1707, -1715, -1724, -1732, -1741, -1749, -1758, -1766, -1775, -1784, -1792, -1801, -1810, -1819, -1828, -1836, -1845, -1854, -1864, -1873, -1882, -1891, -1900, -1910, -1919, -1929, -1938, -1948, -1957, -1967, -1977, -1987, -1997, -2007, -2017, -2027, -2037, -2047, -2058, -2068, -2079, -2089, -2100, -2111, -2122, -2133, -2144, -2155, -2166, -2177, -2189, -2201, -2212, -2224, -2236, -2248, -2261, -2273, -2286, -2299, -2312, -2325, -2338, -2352, -2365, -2379, -2394, -2408, -2423, -2438, -2453, -2469, -2485, -2502, -2519, -2536, -2554},
    {882, 879, 875, 872, 868, 865, 861, 858, 855, 851, 848, 844, 841, 837, 834, 830, 827, 823, 820, 816, 813, 809, 806, 802, 799, 795, 792, 788, 785, 781, 778, 774, 771, 767, 764, 760, 757, 753, 750, 746, 742, 739, 735, 732, 728, 725, 721, 718, 714, 710, 707, 703, 700, 696, 692, 689, 685, 682, 678, 674, 671, 667, 663, 660, 656, 653, 649, 645, 642, 638, 634, 631, 627, 623, 620, 616, 612, 609, 605, 601, 598, 594, 590, 586, 583, 579, 575, 572, 568, 564, 560, 557, 553, 549, 546, 542, 538, 534, 530, 527, 523, 519, 515, 512, 508, 504, 500, 496, 493, 489, 485, 481, 477, 474, 470, 466, 462, 458, 455, 451, 447, 443, 439, 435, 431, 428, 424, 420, 416, 412, 408, 404, 400, 396, 393, 389, 385, 381, 377, 373, 369, 365, 361, 357, 353, 349, 345, 341, 338, 334, 330, 326, 322, 318, 314, 310, 306, 302, 298, 294, 290, 286, 282, 278, 274, 269, 265, 261, 257, 253, 249, 245, 241, 237, 233, 229, 225, 221, 217, 212, 208, 204, 200, 196, 192, 188, 184, 180, 175, 171, 167, 163, 159, 155, 150, 146, 142, 138, 134, 129, 125, 121, 117, 113, 108, 104, 100, 96, 91, 87, 83, 79, 74, 70, 66, 62, 57, 53, 49, 44, 40, 36, 31, 27, 23, 18, 14, 10, 5, 1, -3, -8, -12, -16, -21, -25, -30, -34, -38, -43, -47, -52, -56, -61, -65, -69, -74, -78, -83, -87, -92, -96, -101, -105, -110, -114, -119, -123, -128, -132, -137, -141, -146, -150, -155, -160, -164, -169, -173, -178, -183, -187, -192, -196, -201, -206, -210, -215, -220, -224, -229, -234, -238, -243, -248, -252, -257, -262, -266, -271, -276, -281, -285, -290, -295, -300, -304, -309, -314, -319, -324, -328, -333, -338, -343, -348, -353, -357, -362, -367, -372, -377, -382, -387, -392, -397, -401, -406, -411, -416, -421, -426, -431, -436, -441, -446, -451, -456, -461, -466, -471, -476, -481, -486, -491, -496, -502, -507, -512, -517, -522, -527, -532, -537, -542, -548, -553, -558, -563, -568, -574, -579, -584, -589, -594, -600, -605, -610, -615, -621, -626, -631, -637, -642, -647, -653, -658, -663, -669, -674, -679, -685, -690, -696, -701, -706, -712, -717, -723, -728, -734, -739, -745, -750, -756, -761, -767, -773, -778, -784, -789, -795, -800, -806, -812, -817, -823, -829, -834, -840, -846, -851, -857, -863, -869, -874, -880, -886, -892, -898, -903, -909, -915, -921, -927, -933, -939, -945, -950, -956, -962, -968, -974, -980, -986, -992, -998, -1004, -1010, -1016, -1023, -1029, -1035, -1041, -1047, -1053, -1059, -1066, -1072, -1078, -1084, -1090, -1097, -1103, -1109, -1116, -1122, -1128, -1134, -1141, -1147, -1154, -1160, -1166, -1173, -1179, -1186, -1192, -1199, -1205, -1212, -1218, -1225, -1232, -1238, -1245, -1252, -1258, -1265, -1272, -1278, -1285, -1292, -1299, -1305, -1312, -1319, -1326, -1333, -1340, -1347, -1354, -1361, -1367, -1375, -1382, -1389, -1396, -1403, -1410, -1417, -1424, -1431, -1439, -1446, -1453, -1460, -1468, -1475, -1482, -1490, -1497, -1504, -1512, -1519, -1527, -1534, -1542, -1549, -1557, -1565, -1572, -1580, -1588, -1596, -1603, -1611, -1619, -1627, -1635, -1643, -1651, -1659, -1667, -1675, -1683, -1691, -1699, -1707, -1716, -1724, -1732, -1740, -1749, -1757, -1766, -1774, -1783, -1791, -1800, -1809, -1817, -1826, -1835, -1844, -1853, -1862, -1871, -1880, -1889, -1898, -1907, -1916, -1925, -1935, -1944, -1954, -1963, -1973, -1982, -1992, -2002, -2012, -2022, -2031, -2042, -2052, -2062, -2072, -2082, -2093, -2103, -2114, -2124, -2135, -2146, -2157, -2168, -2179, -2190, -2202, -2213, -2225, -2236, -2248, -2260, -2272, -2285, -2297, -2309, -2322, -2335, -2348, -2361, -2375, -2388, -2402, -2416, -2430, -2445, -2460},
    {893, 889, 886, 882, 879, 875, 872, 869, 865, 862, 858, 855, 851, 848, 844, 841, 837, 834, 830, 827, 823, 820, 816, 813, 809, 806, 802, 799, 795, 792, 788, 785, 781, 778, 774, 771, 767, 764, 760, 757, 753, 750, 746, 742, 739, 735, 732, 728, 725, 721, 717, 714, 710, 707, 703, 699, 696, 692, 689, 685, 681, 678, 674, 671, 667, 663, 660, 656, 652, 649, 645, 641, 638, 634, 630, 627, 623, 619, 616, 612, 608, 605, 601, 597, 594, 590, 586, 582, 579, 575, 571, 568, 564, 560, 556, 553, 549, 545, 541, 538, 534, 530, 526, 523, 519, 515, 511, 507, 504, 500, 496, 492, 488, 485, 481, 477, 473, 469, 466, 462, 458, 454, 450, 446, 442, 439, 435, 431, 427, 423, 419, 415, 411, 408, 404, 400, 396, 392, 388, 384, 380, 376, 372, 368, 365, 361, 357, 353, 349, 345, 341, 337, 333, 329, 325, 321, 317, 313, 309, 305, 301, 297, 293, 289, 285, 281, 277, 273, 269, 265, 261, 257, 253, 248, 244, 240, 236, 232, 228, 224, 220, 216, 212, 208, 203, 199, 195, 191, 187, 183, 179, 175, 170, 166, 162, 158, 154, 150, 145, 141, 137, 133, 129, 124, 120, 116, 112, 108, 103, 99, 95, 91, 86, 82, 78, 74, 69, 65, 61, 56, 52, 48, 44, 39, 35, 31, 26, 22, 18, 13, 9, 5, 0, -4, -9, -13, -17, -22, -26, -30, -35, -39, -44, -48, -53, -57, -61, -66, -70, -75, -79, -84, -88, -93, -97, -102, -106, -111, -115, -120, -124, -129, -133, -138, -142, -147, -151, -156, -161, -165, -170, -174, -179, -183, -188, -193, -197, -202, -207, -211, -216, -220, -225, -230, -234, -239, -244, -248, -253, -258, -263, -267, -272, -277, -282, -286, -291, -296, -301, -305, -310, -315, -320, -324, -329, -334, -339, -344, -349, -353, -358, -363, -368, -373, -378, -383, -387, -392, -397, -402, -407, -412, -417, -422, -427, -432, -437, -442, -447, -452, -457, -462, -467, -472, -477, -482, -487, -492, -497, -502, -507, -512, -517, -522, -528, -533, -538, -543, -548, -553, -558, -564, -569, -574, -579, -584, -590, -595, -600, -605, -611, -616, -621, -626, -632, -637, -642, -648, -653, -658, -664, -669, -674, -680, -685, -690, -696, -701, -707, -712, -718, -723, -728, -734, -739, -745, -750, -756, -761, -767, -772, -778, -784, -789, -795, -800, -806, -812, -817, -823, -828, -834, -840, -845, -851, -857, -863, -868, -874, -880, -886, -891, -897, -903, -909, -915, -920, -926, -932, -938, -944, -950, -956, -962, -968, -973, -979, -985, -991, -997, -1003, -1009, -1015, -1022, -1028, -1034, -1040, -1046, -1052, -1058, -1064, -1070, -1077, -1083, -1089, -1095, -1101, -1108, -1114, -1120, -1127, -1133, -1139, -1145, -1152, -1158, -1165, -1171, -1177, -1184, -1190, -1197, -1203, -1210, -1216, -1223, -1229, -1236, -1242, -1249, -1256, -1262, -1269, -1276, -1282, -1289, -1296, -1303, -1309, -1316, -1323, -1330, -1337, -1343, -1350, -1357, -1364, -1371, -1378, -1385, -1392, -1399, -1406, -1413, -1420, -1427, -1434, -1442, -1449, -1456, -1463, -1470, -1478, -1485, -1492, -1500, -1507, -1514, -1522, -1529, -1537, -1544, -1552, -1559, -1567, -1574, -1582, -1590, -1597, -1605, -1613, -1621, -1628, -1636, -1644, -1652, -1660, -1668, -1676, -1684, -1692, -1700, -1708, -1716, -1724, -1732, -1741, -1749, -1757, -1766, -1774, -1782, -1791, -1799, -1808, -1817, -1825, -1834, -1843, -1851, -1860, -1869, -1878, -1887, -1896, -1905, -1914, -1923, -1932, -1941, -1951, -1960, -1969, -1979, -1988, -1998, -2007, -2017, -2027, -2037, -2047, -2057, -2067, -2077, -2087, -2097, -2107, -2118, -2128, -2139, -2149, -2160, -2171, -2182, -2193, -2204, -2215, -2226, -2238, -2249, -2261, -2272, -2284, -2296, -2308, -2321, -2333, -2346, -2358, -2371, -2384},
    {903, 900, 896, 893, 889, 886, 882, 879, 876, 872, 869, 865, 862, 858, 855, 851, 848, 844, 841, 837, 834, 830, 827, 823, 820, 816, 813, 809, 806, 802, 799, 795, 792, 788, 785, 781, 778, 774, 771, 767, 764, 760, 757, 753, 749, 746, 742, 739, 735, 732, 728, 724, 721, 717, 714, 710, 706, 703, 699, 696, 692, 688, 685, 681, 678, 674, 670, 667, 663, 659, 656, 652, 648, 645, 641, 637, 634, 630, 626, 623, 619, 615, 612, 608, 604, 601, 597, 593, 589, 586, 582, 578, 575, 571, 567, 563, 560, 556, 552, 548, 545, 541, 537, 533, 530, 526, 522, 518, 514, 511, 507, 503, 499, 495, 492, 488, 484, 480, 476, 473, 469, 465, 461, 457, 453, 450, 446, 442, 438, 434, 430, 426, 422, 419, 415, 411, 407, 403, 399, 395, 391, 387, 383, 380, 376, 372, 368, 364, 360, 356, 352, 348, 344, 340, 336, 332, 328, 324, 320, 316, 312, 308, 304, 300, 296, 292, 288, 284, 280, 276, 272, 268, 264, 260, 256, 252, 248, 244, 239, 235, 231, 227, 223, 219, 215, 211, 207, 203, 198, 194, 190, 186, 182, 178, 174, 169, 165, 161, 157, 153, 149, 144, 140, 136, 132, 128, 123, 119, 115, 111, 107, 102, 98, 94, 90, 85, 81, 77, 73, 68, 64, 60, 55, 51, 47, 42, 38, 34, 30, 25, 21, 17, 12, 8, 4, -1, -5, -10, -14, -18, -23, -27, -32, -36, -40, -45, -49, -54, -58, -62, -67, -71, -76, -80, -85, -89, -94, -98, -103, -107, -112, -116, -121, -125, -130, -134, -139, -143, -148, -152, -157, -162, -166, -171, -175, -180, -184, -189, -194, -198, -203, -208, -212, -217, -221, -226, -231, -235, -240, -245, -250, -254, -259, -264, -268, -273, -278, -282, -287, -292, -297, -301, -306, -311, -316, -321, -325, -330, -335, -340, -345, -349, -354, -359, -364, -369, -374, -379, -383, -388, -393, -398, -403, -408, -413, -418, -423, -428, -433, -438, -443, -448, -453, -458, -463, -468, -473, -478, -483, -488, -493, -498, -503, -508, -513, -518, -523, -528, -533, -539, -544, -549, -554, -559, -564, -569, -575, -580, -585, -590, -595, -601, -606, -611, -616, -622, -627, -632, -637, -643, -648, -653, -659, -664, -669, -675, -680, -685, -691, -696, -702, -707, -712, -718, -723, -729, -734, -740, -745, -751, -756, -762, -767, -773, -778, -784, -789, -795, -800, -806, -812, -817, -823, -828, -834, -840, -845, -851, -857, -862, -868, -874, -880, -885, -891, -897, -903, -908, -914, -920, -926, -932, -938, -943, -949, -955, -961, -967, -973, -979, -985, -991, -997, -1003, -1009, -1015, -1021, -1027, -1033, -1039, -1045, -1051, -1057, -1063, -1069, -1075, -1082, -1088, -1094, -1100, -1106, -1113, -1119, -1125, -1131, -1138, -1144, -1150, -1157, -1163, -1169, -1176, -1182, -1189, -1195, -1201, -1208, -1214, -1221, -1227, -1234, -1240, -1247, -1253, -1260, -1267, -1273, -1280, -1287, -1293, -1300, -1307, -1313, -1320, -1327, -1334, -1341, -1347, -1354, -1361, -1368, -1375, -1382, -1389, -1396, -1403, -1410, -1417, -1424, -1431, -1438, -1445, -1452, -1459, -1466, -1474, -1481, -1488, -1495, -1503, -1510, -1517, -1525, -1532, -1539, -1547, -1554, -1562, -1569, -1577, -1584, -1592, -1599, -1607, -1615, -1622, -1630, -1638, -1646, -1653, -1661, -1669, -1677, -1685, -1693, -1701, -1709, -1717, -1725, -1733, -1741, -1750, -1758, -1766, -1774, -1783, -1791, -1799, -1808, -1816, -1825, -1833, -1842, -1851, -1859, -1868, -1877, -1885, -1894, -1903, -1912, -1921, -1930, -1939, -1948, -1957, -1967, -1976, -1985, -1995, -2004, -2014, -2023, -2033, -2042, -2052, -2062, -2072, -2082, -2092, -2102, -2112, -2122, -2132, -2143, -2153, -2164, -2174, -2185, -2196, -2206, -2217, -2228, -2239, -2251, -2262, -2273, -2285, -2297, -2308, -2320},
    {913, 910, 906, 903, 900, 896, 893, 889, 886, 882, 879, 876, 872, 869, 865, 862, 858, 855, 851, 848, 844, 841, 837, 834, 830, 827, 823, 820, 816, 813, 809, 806, 802, 799, 795, 792, 788, 785, 781, 778, 774, 771, 767, 763, 760, 756, 753, 749, 746, 742, 738, 735, 731, 728, 724, 721, 717, 713, 710, 706, 703, 699, 695, 692, 688, 684, 681, 677, 674, 670, 666, 663, 659, 655, 652, 648, 644, 641, 637, 633, 630, 626, 622, 619, 615, 611, 607, 604, 600, 596, 593, 589, 585, 581, 578, 574, 570, 567, 563, 559, 555, 552, 548, 544, 540, 536, 533, 529, 525, 521, 518, 514, 510, 506, 502, 499, 495, 491, 487, 483, 480, 476, 472, 468, 464, 460, 456, 453, 449, 445, 441, 437, 433, 429, 426, 422, 418, 414, 410, 406, 402, 398, 394, 390, 387, 383, 379, 375, 371, 367, 363, 359, 355, 351, 347, 343, 339, 335, 331, 327, 323, 319, 315, 311, 307, 303, 299, 295, 291, 287, 283, 279, 275, 271, 267, 263, 259, 255, 251, 247, 243, 238, 234, 230, 226, 222, 218, 214, 210, 206, 202, 197, 193, 189, 185, 181, 177, 173, 168, 164, 160, 156, 152, 148, 143, 139, 135, 131, 127, 122, 118, 114, 110, 105, 101, 97, 93, 88, 84, 80, 76, 71, 67, 63, 59, 54, 50, 46, 41, 37, 33, 28, 24, 20, 15, 11, 7, 2, -2, -6, -11, -15, -20, -24, -28, -33, -37, -42, -46, -50, -55, -59, -64, -68, -73, -77, -81, -86, -90, -95, -99, -104, -108, -113, -117, -122, -126, -131, -135, -140, -144, -149, -154, -158, -163, -167, -172, -176, -181, -186, -190, -195, -199, -204, -209, -213, -218, -223, -227, -232, -237, -241, -246, -251, -255, -260, -265, -269, -274, -279, -284, -288, -293, -298, -303, -307, -312, -317, -322, -326, -331, -336, -341, -346, -351, -355, -360, -365, -370, -375, -380, -385, -389, -394, -399, -404, -409, -414, -419, -424, -429, -434, -439, -444, -449, -454, -459, -464, -469, -474, -479, -484, -489, -494, -499, -504, -509, -514, -519, -524, -529, -534, -539, -544, -550, -555, -560, -565, -570, -575, -581, -586, -591, -596, -601, -607, -612, -617, -622, -628, -633, -638, -643, -649, -654, -659, -665, -670, -675, -681, -686, -691, -697, -702, -707, -713, -718, -724, -729, -735, -740, -745, -751, -756, -762, -767, -773, -778, -784, -790, -795, -801, -806, -812, -817, -823, -829, -834, -840, -845, -851, -857, -862, -868, -874, -880, -885, -891, -897, -903, -908, -914, -920, -926, -931, -937, -943, -949, -955, -961, -967, -972, -978, -984, -990, -996, -1002, -1008, -1014, -1020, -1026, -1032, -1038, -1044, -1050, -1056, -1062, -1068, -1075, -1081, -1087, -1093, -1099, -1105, -1112, -1118, -1124, -1130, -1136, -1143, -1149, -1155, -1162, -1168, -1174, -1181, -1187, -1193, -1200, -1206, -1213, -1219, -1226, -1232, -1239, -1245, -1252, -1258, -1265, -1271, -1278, -1284, -1291, -1298, -1304, -1311, -1318, -1324, -1331, -1338, -1345, -1351, -1358, -1365, -1372, -1379, -1386, -1393, -1400, -1406, -1413, -1420, -1427, -1434, -1441, -1449, -1456, -1463, -1470, -1477, -1484, -1491, -1499, -1506, -1513, -1520, -1528, -1535, -1542, -1550, -1557, -1564, -1572, -1579, -1587, -1594, -1602, -1609, -1617, -1625, -1632, -1640, -1648, -1655, -1663, -1671, -1679, -1687, -1695, -1702, -1710, -1718, -1726, -1734, -1742, -1751, -1759, -1767, -1775, -1783, -1791, -1800, -1808, -1816, -1825, -1833, -1842, -1850, -1859, -1867, -1876, -1885, -1893, -1902, -1911, -1920, -1929, -1938, -1947, -1956, -1965, -1974, -1983, -1992, -2001, -2011, -2020, -2029, -2039, -2048, -2058, -2068, -2077, -2087, -2097, -2107, -2117, -2127, -2137, -2147, -2157, -2168, -2178, -2189, -2199, -2210, -2220, -2231, -2242, -2253, -2264},
    {924, 920, 917, 913, 910, 906, 903, 900, 896, 893, 889, 886, 882, 879, 875, 872, 868, 865, 862, 858, 855, 851, 848, 844, 841, 837, 834, 830, 827, 823, 820, 816, 813, 809, 806, 802, 799, 795, 791, 788, 784, 781, 777, 774, 770, 767, 763, 760, 756, 752, 749, 745, 742, 738, 734, 731, 727, 724, 720, 716, 713, 709, 706, 702, 698, 695, 691, 688, 684, 680, 677, 673, 669, 666, 662, 658, 655, 651, 647, 644, 640, 636, 633, 629, 625, 622, 618, 614, 611, 607, 603, 599, 596, 592, 588, 585, 581, 577, 573, 570, 566, 562, 558, 555, 551, 547, 543, 540, 536, 532, 528, 524, 521, 517, 513, 509, 505, 502, 498, 494, 490, 486, 483, 479, 475, 471, 467, 463, 459, 456, 452, 448, 444, 440, 436, 432, 429, 425, 421, 417, 413, 409, 405, 401, 397, 393, 389, 386, 382, 378, 374, 370, 366, 362, 358, 354, 350, 346, 342, 338, 334, 330, 326, 322, 318, 314, 310, 306, 302, 298, 294, 290, 286, 282, 278, 274, 270, 266, 262, 258, 254, 250, 246, 241, 237, 233, 229, 225, 221, 217, 213, 209, 205, 200, 196, 192, 188, 184, 180, 176, 171, 167, 163, 159, 155, 151, 146, 142, 138, 134, 130, 125, 121, 117, 113, 108, 104, 100, 96, 91, 87, 83, 79, 74, 70, 66, 62, 57, 53, 49, 44, 40, 36, 31, 27, 23, 18, 14, 10, 5, 1, -3, -8, -12, -16, -21, -25, -30, -34, -38, -43, -47, -52, -56, -60, -65, -69, -74, -78, -83, -87, -92, -96, -101, -105, -110, -114, -119, -123, -128, -132, -137, -141, -146, -150, -155, -159, -164, -169, -173, -178, -182, -187, -192, -196, -201, -205, -210, -215, -219, -224, -229, -233, -238, -243, -247, -252, -257, -261, -266, -271, -275, -280, -285, -290, -294, -299, -304, -309, -313, -318, -323, -328, -333, -337, -342, -347, -352, -357, -361, -366, -371, -376, -381, -386, -391, -395, -400, -405, -410, -415, -420, -425, -430, -435, -440, -445, -450, -455, -460, -465, -470, -475, -480, -485, -490, -495, -500, -505, -510, -515, -520, -525, -530, -535, -540, -545, -551, -556, -561, -566, -571, -576, -581, -587, -592, -597, -602, -607, -613, -618, -623, -628, -634, -639, -644, -649, -655, -660, -665, -671, -676, -681, -687, -692, -697, -703, -708, -714, -719, -724, -730, -735, -741, -746, -752, -757, -762, -768, -773, -779, -784, -790, -795, -801, -807, -812, -818, -823, -829, -835, -840, -846, -851, -857, -863, -868, -874, -880, -885, -891, -897, -903, -908, -914, -920, -926, -931, -937, -943, -949, -955, -961, -966, -972, -978, -984, -990, -996, -1002, -1008, -1014, -1020, -1026, -1032, -1038, -1044, -1050, -1056, -1062, -1068, -1074, -1080, -1086, -1092, -1098, -1105, -1111, -1117, -1123, -1129, -1136, -1142, -1148, -1154, -1161, -1167, -1173, -1179, -1186, -1192, -1198, -1205, -1211, -1218, -1224, -1231, -1237, -1243, -1250, -1256, -1263, -1269, -1276, -1283, -1289, -1296, -1302, -1309, -1316, -1322, -1329, -1336, -1342, -1349, -1356, -1363, -1369, -1376, -1383, -1390, -1397, -1404, -1411, -1417, -1424, -1431, -1438, -1445, -1452, -1459, -1466, -1474, -1481, -1488, -1495, -1502, -1509, -1516, -1524, -1531, -1538, -1545, -1553, -1560, -1568, -1575, -1582, -1590, -1597, -1605, -1612, -1620, -1627, -1635, -1642, -1650, -1658, -1665, -1673, -1681, -1689, -1696, -1704, -1712, -1720, -1728, -1736, -1744, -1752, -1760, -1768, -1776, -1784, -1792, -1801, -1809, -1817, -1825, -1834, -1842, -1850, -1859, -1867, -1876, -1884, -1893, -1902, -1910, -1919, -1928, -1937, -1945, -1954, -1963, -1972, -1981, -1990, -1999, -2008, -2018, -2027, -2036, -2046, -2055, -2064, -2074, -2084, -2093, -2103, -2113, -2122, -2132, -2142, -2152, -2162, -2172, -2183, -2193, -2203, -2214},
    {934, 930, 927, 923, 920, 917, 913, 910, 906, 903, 899, 896, 893, 889, 886, 882, 879, 875, 872, 868, 865, 861, 858, 854, 851, 847, 844, 840, 837, 833, 830, 826, 823, 819, 816, 812, 809, 805, 802, 798, 795, 791, 788, 784, 780, 777, 773, 770, 766, 763, 759, 755, 752, 748, 745, 741, 738, 734, 730, 727, 723, 720, 716, 712, 709, 705, 701, 698, 694, 691, 687, 683, 680, 676, 672, 669, 665, 661, 658, 654, 650, 647, 643, 639, 636, 632, 628, 625, 621, 617, 614, 610, 606, 602, 599, 595, 591, 587, 584, 580, 576, 573, 569, 565, 561, 558, 554, 550, 546, 542, 539, 535, 531, 527, 524, 520, 516, 512, 508, 505, 501, 497, 493, 489, 485, 482, 478, 474, 470, 466, 462, 459, 455, 451, 447, 443, 439, 435, 431, 428, 424, 420, 416, 412, 408, 404, 400, 396, 392, 388, 385, 381, 377, 373, 369, 365, 361, 357, 353, 349, 345, 341, 337, 333, 329, 325, 321, 317, 313, 309, 305, 301, 297, 293, 289, 285, 281, 277, 273, 269, 265, 261, 257, 252, 248, 244, 240, 236, 232, 228, 224, 220, 216, 211, 207, 203, 199, 195, 191, 187, 183, 178, 174, 170, 166, 162, 158, 153, 149, 145, 141, 137, 132, 128, 124, 120, 116, 111, 107, 103, 99, 94, 90, 86, 82, 77, 73, 69, 65, 60, 56, 52, 47, 43, 39, 34, 30, 26, 21, 17, 13, 8, 4, 0, -5, -9, -13, -18, -22, -27, -31, -35, -40, -44, -49, -53, -57, -62, -66, -71, -75, -80, -84, -89, -93, -98, -102, -107, -111, -116, -120, -125, -129, -134, -138, -143, -147, -152, -156, -161, -165, -170, -175, -179, -184, -188, -193, -198, -202, -207, -211, -216, -221, -225, -230, -235, -239, -244, -249, -253, -258, -263, -267, -272, -277, -282, -286, -291, -296, -300, -305, -310, -315, -320, -324, -329, -334, -339, -343, -348, -353, -358, -363, -368, -372, -377, -382, -387, -392, -397, -402, -407, -411, -416, -421, -426, -431, -436, -441, -446, -451, -456, -461, -466, -471, -476, -481, -486, -491, -496, -501, -506, -511, -516, -521, -526, -531, -536, -541, -547, -552, -557, -562, -567, -572, -577, -583, -588, -593, -598, -603, -608, -614, -619, -624, -629, -635, -640, -645, -650, -656, -661, -666, -672, -677, -682, -688, -693, -698, -704, -709, -714, -720, -725, -731, -736, -741, -747, -752, -758, -763, -769, -774, -780, -785, -791, -796, -802, -807, -813, -818, -824, -829, -835, -841, -846, -852, -857, -863, -869, -874, -880, -886, -891, -897, -903, -909, -914, -920, -926, -932, -937, -943, -949, -955, -961, -966, -972, -978, -984, -990, -996, -1002, -1008, -1014, -1020, -1025, -1031, -1037, -1043, -1049, -1055, -1061, -1067, -1074, -1080, -1086, -1092, -1098, -1104, -1110, -1116, -1122, -1129, -1135, -1141, -1147, -1153, -1160, -1166, -1172, -1178, -1185, -1191, -1197, -1204, -1210, -1216, -1223, -1229, -1236, -1242, -1249, -1255, -1261, -1268, -1274, -1281, -1288, -1294, -1301, -1307, -1314, -1320, -1327, -1334, -1340, -1347, -1354, -1360, -1367, -1374, -1381, -1388, -1394, -1401, -1408, -1415, -1422, -1429, -1436, -1443, -1449, -1456, -1463, -1470, -1478, -1485, -1492, -1499, -1506, -1513, -1520, -1527, -1535, -1542, -1549, -1556, -1564, -1571, -1578, -1586, -1593, -1600, -1608, -1615, -1623, -1630, -1638, -1645, -1653, -1660, -1668, -1676, -1683, -1691, -1699, -1707, -1714, -1722, -1730, -1738, -1746, -1754, -1762, -1770, -1778, -1786, -1794, -1802, -1810, -1818, -1826, -1835, -1843, -1851, -1859, -1868, -1876, -1885, -1893, -1902, -1910, -1919, -1927, -1936, -1945, -1954, -1962, -1971, -1980, -1989, -1998, -2007, -2016, -2025, -2034, -2043, -2053, -2062, -2071, -2081, -2090, -2100, -2109, -2119, -2128, -2138, -2148, -2158, -2168},
    {944, 940, 937, 934, 930, 927, 923, 920, 916, 913, 909, 906, 903, 899, 896, 892, 889, 885, 882, 878, 875, 871, 868, 864, 861, 857, 854, 850, 847, 843, 840, 836, 833, 829, 826, 822, 819, 815, 812, 808, 805, 801, 798, 794, 791, 787, 783, 780, 776, 773, 769, 766, 762, 758, 755, 751, 748, 744, 741, 737, 733, 730, 726, 723, 719, 715, 712, 708, 704, 701, 697, 694, 690, 686, 683, 679, 675, 672, 668, 664, 661, 657, 653, 650, 646, 642, 639, 635, 631, 628, 624, 620, 616, 613, 609, 605, 602, 598, 594, 590, 587, 583, 579, 575, 572, 568, 564, 560, 557, 553, 549, 545, 542, 538, 534, 530, 526, 523, 519, 515, 511, 507, 504, 500, 496, 492, 488, 484, 481, 477, 473, 469, 465, 461, 457, 454, 450, 446, 442, 438, 434, 430, 426, 423, 419, 415, 411, 407, 403, 399, 395, 391, 387, 383, 379, 375, 371, 368, 364, 360, 356, 352, 348, 344, 340, 336, 332, 328, 324, 320, 316, 312, 308, 304, 300, 296, 292, 288, 284, 280, 275, 271, 267, 263, 259, 255, 251, 247, 243, 239, 235, 231, 227, 222, 218, 214, 210, 206, 202, 198, 194, 189, 185, 181, 177, 173, 169, 165, 160, 156, 152, 148, 144, 139, 135, 131, 127, 123, 118, 114, 110, 106, 101, 97, 93, 89, 84, 80, 76, 72, 67, 63, 59, 54, 50, 46, 42, 37, 33, 29, 24, 20, 16, 11, 7, 3, -2, -6, -11, -15, -19, -24, -28, -33, -37, -41, -46, -50, -55, -59, -63, -68, -72, -77, -81, -86, -90, -95, -99, -104, -108, -113, -117, -122, -126, -131, -135, -140, -144, -149, -153, -158, -162, -167, -172, -176, -181, -185, -190, -194, -199, -204, -208, -213, -218, -222, -227, -232, -236, -241, -245, -250, -255, -260, -264, -269, -274, -278, -283, -288, -293, -297, -302, -307, -311, -316, -321, -326, -331, -335, -340, -345, -350, -355, -359, -364, -369, -374, -379, -384, -388, -393, -398, -403, -408, -413, -418, -423, -428, -433, -437, -442, -447, -452, -457, -462, -467, -472, -477, -482, -487, -492, -497, -502, -507, -512, -517, -522, -528, -533, -538, -543, -548, -553, -558, -563, -568, -573, -579, -584, -589, -594, -599, -604, -610, -615, -620, -625, -631, -636, -641, -646, -652, -657, -662, -667, -673, -678, -683, -689, -694, -699, -705, -710, -715, -721, -726, -731, -737, -742, -748, -753, -759, -764, -769, -775, -780, -786, -791, -797, -802, -808, -813, -819, -825, -830, -836, -841, -847, -852, -858, -864, -869, -875, -881, -886, -892, -898, -903, -909, -915, -920, -926, -932, -938, -943, -949, -955, -961, -967, -972, -978, -984, -990, -996, -1002, -1008, -1014, -1020, -1025, -1031, -1037, -1043, -1049, -1055, -1061, -1067, -1073, -1079, -1085, -1091, -1098, -1104, -1110, -1116, -1122, -1128, -1134, -1140, -1147, -1153, -1159, -1165, -1171, -1178, -1184, -1190, -1197, -1203, -1209, -1216, -1222, -1228, -1235, -1241, -1247, -1254, -1260, -1267, -1273, -1280, -1286, -1293, -1299, -1306, -1312, -1319, -1325, -1332, -1339, -1345, -1352, -1359, -1365, -1372, -1379, -1385, -1392, -1399, -1406, -1413, -1419, -1426, -1433, -1440, -1447, -1454, -1461, -1468, -1475, -1482, -1489, -1496, -1503, -1510, -1517, -1524, -1531, -1538, -1546, -1553, -1560, -1567, -1574, -1582, -1589, -1596, -1604, -1611, -1618, -1626, -1633, -1641, -1648, -1656, -1663, -1671, -1678, -1686, -1694, -1701, -1709, -1717, -1724, -1732, -1740, -1748, -1756, -1764, -1771, -1779, -1787, -1795, -1803, -1811, -1820, -1828, -1836, -1844, -1852, -1860, -1869, -1877, -1885, -1894, -1902, -1911, -1919, -1927, -1936, -1945, -1953, -1962, -1971, -1979, -1988, -1997, -2006, -2015, -2024, -2033, -2042, -2051, -2060, -2069, -2078, -2088, -2097, -2106, -2116, -2125},
    {954, 950, 947, 944, 940, 937, 933, 930, 926, 923, 919, 916, 913, 909, 906, 902, 899, 895, 892, 888, 885, 881, 878, 874, 871, 867, 864, 860, 857, 853, 850, 846, 843, 839, 836, 832, 829, 825, 822, 818, 815, 811, 808, 804, 801, 797, 794, 790, 786, 783, 779, 776, 772, 769, 765, 761, 758, 754, 751, 747, 743, 740, 736, 733, 729, 725, 722, 718, 715, 711, 707, 704, 700, 696, 693, 689, 685, 682, 678, 674, 671, 667, 663, 660, 656, 652, 649, 645, 641, 638, 634, 630, 627, 623, 619, 615, 612, 608, 604, 601, 597, 593, 589, 586, 582, 578, 574, 571, 567, 563, 559, 556, 552, 548, 544, 540, 537, 533, 529, 525, 521, 518, 514, 510, 506, 502, 499, 495, 491, 487, 483, 479, 476, 472, 468, 464, 460, 456, 452, 449, 445, 441, 437, 433, 429, 425, 421, 417, 413, 410, 406, 402, 398, 394, 390, 386, 382, 378, 374, 370, 366, 362, 358, 354, 350, 346, 342, 338, 334, 330, 326, 322, 318, 314, 310, 306, 302, 298, 294, 290, 286, 282, 278, 274, 270, 266, 262, 258, 254, 250, 246, 242, 237, 233, 229, 225, 221, 217, 213, 209, 205, 200, 196, 192, 188, 184, 180, 176, 171, 167, 163, 159, 155, 150, 146, 142, 138, 134, 129, 125, 121, 117, 113, 108, 104, 100, 96, 91, 87, 83, 79, 74, 70, 66, 61, 57, 53, 49, 44, 40, 36, 31, 27, 23, 18, 14, 10, 5, 1, -3, -8, -12, -17, -21, -25, -30, -34, -39, -43, -47, -52, -56, -61, -65, -70, -74, -78, -83, -87, -92, -96, -101, -105, -110, -114, -119, -123, -128, -132, -137, -141, -146, -150, -155, -160, -164, -169, -173, -178, -182, -187, -192, -196, -201, -205, -210, -215, -219, -224, -229, -233, -238, -242, -247, -252, -256, -261, -266, -271, -275, -280, -285, -289, -294, -299, -304, -308, -313, -318, -323, -327, -332, -337, -342, -347, -351, -356, -361, -366, -371, -376, -380, -385, -390, -395, -400, -405, -410, -414, -419, -424, -429, -434, -439, -444, -449, -454, -459, -464, -469, -474, -479, -484, -489, -494, -499, -504, -509, -514, -519, -524, -529, -534, -539, -544, -549, -554, -559, -565, -570, -575, -580, -585, -590, -595, -601, -606, -611, -616, -621, -627, -632, -637, -642, -647, -653, -658, -663, -669, -674, -679, -684, -690, -695, -700, -706, -711, -716, -722, -727, -733, -738, -743, -749, -754, -760, -765, -770, -776, -781, -787, -792, -798, -803, -809, -814, -820, -825, -831, -836, -842, -848, -853, -859, -864, -870, -876, -881, -887, -893, -898, -904, -910, -915, -921, -927, -932, -938, -944, -950, -955, -961, -967, -973, -979, -985, -990, -996, -1002, -1008, -1014, -1020, -1026, -1032, -1037, -1043, -1049, -1055, -1061, -1067, -1073, -1079, -1085, -1091, -1097, -1103, -1110, -1116, -1122, -1128, -1134, -1140, -1146, -1152, -1159, -1165, -1171, -1177, -1183, -1190, -1196, -1202, -1208, -1215, -1221, -1227, -1234, -1240, -1246, -1253, -1259, -1266, -1272, -1279, -1285, -1291, -1298, -1304, -1311, -1318, -1324, -1331, -1337, -1344, -1350, -1357, -1364, -1370, -1377, -1384, -1390, -1397, -1404, -1411, -1417, -1424, -1431, -1438, -1445, -1452, -1458, -1465, -1472, -1479, -1486, -1493, -1500, -1507, -1514, -1521, -1528, -1535, -1542, -1550, -1557, -1564, -1571, -1578, -1586, -1593, -1600, -1607, -1615, -1622, -1629, -1637, -1644, -1652, -1659, -1667, -1674, -1682, -1689, -1697, -1704, -1712, -1720, -1727, -1735, -1743, -1750, -1758, -1766, -1774, -1782, -1789, -1797, -1805, -1813, -1821, -1829, -1837, -1845, -1854, -1862, -1870, -1878, -1886, -1895, -1903, -1911, -1920, -1928, -1936, -1945, -1953, -1962, -1971, -1979, -1988, -1997, -2005, -2014, -2023, -2032, -2041, -2050, -2059, -2068, -2077, -2086},
    {964, 960, 957, 953, 950, 947, 943, 940, 936, 933, 929, 926, 922, 919, 916, 912, 909, 905, 902, 898, 895, 891, 888, 884, 881, 877, 874, 870, 867, 863, 860, 856, 853, 849, 846, 842, 839, 835, 832, 828, 825, 821, 818, 814, 811, 807, 803, 800, 796, 793, 789, 786, 782, 779, 775, 771, 768, 764, 761, 757, 753, 750, 746, 743, 739, 735, 732, 728, 725, 721, 717, 714, 710, 706, 703, 699, 695, 692, 688, 685, 681, 677, 674, 670, 666, 663, 659, 655, 651, 648, 644, 640, 637, 633, 629, 626, 622, 618, 614, 611, 607, 603, 600, 596, 592, 588, 585, 581, 577, 573, 570, 566, 562, 558, 554, 551, 547, 543, 539, 535, 532, 528, 524, 520, 516, 513, 509, 505, 501, 497, 494, 490, 486, 482, 478, 474, 470, 467, 463, 459, 455, 451, 447, 443, 439, 436, 432, 428, 424, 420, 416, 412, 408, 404, 400, 396, 392, 389, 385, 381, 377, 373, 369, 365, 361, 357, 353, 349, 345, 341, 337, 333, 329, 325, 321, 317, 313, 309, 305, 301, 297, 293, 289, 285, 281, 277, 273, 269, 264, 260, 256, 252, 248, 244, 240, 236, 232, 228, 224, 219, 215, 211, 207, 203, 199, 195, 191, 186, 182, 178, 174, 170, 166, 161, 157, 153, 149, 145, 140, 136, 132, 128, 124, 119, 115, 111, 107, 102, 98, 94, 90, 85, 81, 77, 73, 68, 64, 60, 55, 51, 47, 43, 38, 34, 30, 25, 21, 17, 12, 8, 4, -1, -5, -10, -14, -18, -23, -27, -32, -36, -40, -45, -49, -54, -58, -62, -67, -71, -76, -80, -85, -89, -94, -98, -103, -107, -112, -116, -121, -125, -130, -134, -139, -143, -148, -152, -157, -161, -166, -170, -175, -180, -184, -189, -193, -198, -203, -207, -212, -216, -221, -226, -230, -235, -240, -244, -249, -254, -258, -263, -268, -272, -277, -282, -286, -291, -296, -301, -305, -310, -315, -320, -324, -329, -334, -339, -344, -348, -353, -358, -363, -368, -372, -377, -382, -387, -392, -397, -401, -406, -411, -416, -421, -426, -431, -436, -441, -446, -451, -456, -460, -465, -470, -475, -480, -485, -490, -495, -500, -505, -510, -515, -520, -525, -531, -536, -541, -546, -551, -556, -561, -566, -571, -576, -581, -587, -592, -597, -602, -607, -612, -618, -623, -628, -633, -638, -644, -649, -654, -659, -665, -670, -675, -680, -686, -691, -696, -702, -707, -712, -718, -723, -728, -734, -739, -745, -750, -755, -761, -766, -772, -777, -782, -788, -793, -799, -804, -810, -815, -821, -826, -832, -837, -843, -849, -854, -860, -865, -871, -877, -882, -888, -893, -899, -905, -910, -916, -922, -927, -933, -939, -945, -950, -956, -962, -968, -973, -979, -985, -991, -997, -1003, -1008, -1014, -1020, -1026, -1032, -1038, -1044, -1050, -1056, -1062, -1067, -1073, -1079, -1085, -1091, -1097, -1104, -1110, -1116, -1122, -1128, -1134, -1140, -1146, -1152, -1158, -1165, -1171, -1177, -1183, -1189, -1196, -1202, -1208, -1214, -1221, -1227, -1233, -1239, -1246, -1252, -1259, -1265, -1271, -1278, -1284, -1291, -1297, -1303, -1310, -1316, -1323, -1329, -1336, -1343, -1349, -1356, -1362, -1369, -1376, -1382, -1389, -1396, -1402, -1409, -1416, -1422, -1429, -1436, -1443, -1450, -1456, -1463, -1470, -1477, -1484, -1491, -1498, -1505, -1512, -1519, -1526, -1533, -1540, -1547, -1554, -1561, -1568, -1575, -1582, -1590, -1597, -1604, -1611, -1618, -1626, -1633, -1640, -1648, -1655, -1663, -1670, -1677, -1685, -1692, -1700, -1707, -1715, -1723, -1730, -1738, -1745, -1753, -1761, -1769, -1776, -1784, -1792, -1800, -1808, -1815, -1823, -1831, -1839, -1847, -1855, -1863, -1871, -1880, -1888, -1896, -1904, -1912, -1921, -1929, -1937, -1946, -1954, -1963, -1971, -1979, -1988, -1997, -2005, -2014, -2023, -2031, -2040, -2049},
    {973, 970, 967, 963, 960, 956, 953, 950, 946, 943, 939, 936, 932, 929, 925, 922, 918, 915, 912, 908, 905, 901, 898, 894, 891, 887, 884, 880, 877, 873, 870, 866, 863, 859, 856, 852, 849, 845, 842, 838, 835, 831, 828, 824, 820, 817, 813, 810, 806, 803, 799, 796, 792, 788, 785, 781, 778, 774, 771, 767, 763, 760, 756, 753, 749, 745, 742, 738, 734, 731, 727, 724, 720, 716, 713, 709, 705, 702, 698, 694, 691, 687, 684, 680, 676, 673, 669, 665, 661, 658, 654, 650, 647, 643, 639, 636, 632, 628, 624, 621, 617, 613, 610, 606, 602, 598, 595, 591, 587, 583, 580, 576, 572, 568, 565, 561, 557, 553, 549, 546, 542, 538, 534, 530, 527, 523, 519, 515, 511, 508, 504, 500, 496, 492, 488, 484, 481, 477, 473, 469, 465, 461, 457, 454, 450, 446, 442, 438, 434, 430, 426, 422, 418, 415, 411, 407, 403, 399, 395, 391, 387, 383, 379, 375, 371, 367, 363, 359, 355, 351, 347, 343, 339, 335, 331, 327, 323, 319, 315, 311, 307, 303, 299, 295, 291, 287, 283, 279, 275, 271, 267, 263, 259, 255, 251, 246, 242, 238, 234, 230, 226, 222, 218, 214, 209, 205, 201, 197, 193, 189, 185, 180, 176, 172, 168, 164, 160, 155, 151, 147, 143, 139, 134, 130, 126, 122, 118, 113, 109, 105, 101, 96, 92, 88, 84, 79, 75, 71, 67, 62, 58, 54, 49, 45, 41, 36, 32, 28, 23, 19, 15, 10, 6, 2, -3, -7, -11, -16, -20, -25, -29, -33, -38, -42, -47, -51, -55, -60, -64, -69, -73, -78, -82, -87, -91, -95, -100, -104, -109, -113, -118, -122, -127, -131, -136, -140, -145, -150, -154, -159, -163, -168, -172, -177, -181, -186, -191, -195, -200, -204, -209, -214, -218, -223, -228, -232, -237, -241, -246, -251, -255, -260, -265, -270, -274, -279, -284, -288, -293, -298, -302, -307, -312, -317, -321, -326, -331, -336, -341, -345, -350, -355, -360, -365, -369, -374, -379, -384, -389, -394, -398, -403, -408, -413, -418, -423, -428, -433, -438, -442, -447, -452, -457, -462, -467, -472, -477, -482, -487, -492, -497, -502, -507, -512, -517, -522, -527, -532, -537, -542, -547, -552, -558, -563, -568, -573, -578, -583, -588, -593, -599, -604, -609, -614, -619, -624, -630, -635, -640, -645, -650, -656, -661, -666, -671, -677, -682, -687, -693, -698, -703, -708, -714, -719, -724, -730, -735, -740, -746, -751, -757, -762, -767, -773, -778, -784, -789, -795, -800, -806, -811, -817, -822, -828, -833, -839, -844, -850, -855, -861, -866, -872, -878, -883, -889, -894, -900, -906, -911, -917, -923, -928, -934, -940, -945, -951, -957, -963, -968, -974, -980, -986, -992, -997, -1003, -1009, -1015, -1021, -1027, -1032, -1038, -1044, -1050, -1056, -1062, -1068, -1074, -1080, -1086, -1092, -1098, -1104, -1110, -1116, -1122, -1128, -1134, -1140, -1146, -1152, -1158, -1165, -1171, -1177, -1183, -1189, -1195, -1202, -1208, -1214, -1220, -1227, -1233, -1239, -1245, -1252, -1258, -1264, -1271, -1277, -1283, -1290, -1296, -1303, -1309, -1316, -1322, -1329, -1335, -1342, -1348, -1355, -1361, -1368, -1374, -1381, -1388, -1394, -1401, -1407, -1414, -1421, -1428, -1434, -1441, -1448, -1455, -1461, -1468, -1475, -1482, -1489, -1496, -1502, -1509, -1516, -1523, -1530, -1537, -1544, -1551, -1558, -1565, -1572, -1580, -1587, -1594, -1601, -1608, -1615, -1623, -1630, -1637, -1644, -1652, -1659, -1666, -1674, -1681, -1688, -1696, -1703, -1711, -1718, -1726, -1733, -1741, -1749, -1756, -1764, -1771, -1779, -1787, -1795, -1802, -1810, -1818, -1826, -1834, -1842, -1850, -1857, -1865, -1873, -1881, -1890, -1898, -1906, -1914, -1922, -1930, -1939, -1947, -1955, -1963, -1972, -1980, -1989, -1997, -2006, -2014},
    {983, 980, 976, 973, 970, 966, 963, 959, 956, 952, 949, 945, 942, 939, 935, 932, 928, 925, 921, 918, 914, 911, 907, 904, 900, 897, 893, 890, 887, 883, 880, 876, 873, 869, 865, 862, 858, 855, 851, 848, 844, 841, 837, 834, 830, 827, 823, 820, 816, 812, 809, 805, 802, 798, 795, 791, 788, 784, 780, 777, 773, 770, 766, 762, 759, 755, 752, 748, 744, 741, 737, 733, 730, 726, 723, 719, 715, 712, 708, 704, 701, 697, 693, 690, 686, 682, 679, 675, 671, 668, 664, 660, 657, 653, 649, 646, 642, 638, 634, 631, 627, 623, 620, 616, 612, 608, 605, 601, 597, 593, 590, 586, 582, 578, 575, 571, 567, 563, 559, 556, 552, 548, 544, 540, 537, 533, 529, 525, 521, 518, 514, 510, 506, 502, 498, 495, 491, 487, 483, 479, 475, 471, 468, 464, 460, 456, 452, 448, 444, 440, 436, 433, 429, 425, 421, 417, 413, 409, 405, 401, 397, 393, 389, 385, 382, 378, 374, 370, 366, 362, 358, 354, 350, 346, 342, 338, 334, 330, 326, 322, 318, 314, 310, 306, 302, 298, 294, 289, 285, 281, 277, 273, 269, 265, 261, 257, 253, 249, 245, 241, 237, 232, 228, 224, 220, 216, 212, 208, 204, 199, 195, 191, 187, 183, 179, 175, 170, 166, 162, 158, 154, 149, 145, 141, 137, 133, 128, 124, 120, 116, 111, 107, 103, 99, 94, 90, 86, 82, 77, 73, 69, 65, 60, 56, 52, 47, 43, 39, 34, 30, 26, 21, 17, 13, 8, 4, 0, -5, -9, -13, -18, -22, -27, -31, -35, -40, -44, -49, -53, -57, -62, -66, -71, -75, -80, -84, -89, -93, -97, -102, -106, -111, -115, -120, -124, -129, -133, -138, -142, -147, -152, -156, -161, -165, -170, -174, -179, -183, -188, -193, -197, -202, -206, -211, -216, -220, -225, -230, -234, -239, -243, -248, -253, -257, -262, -267, -272, -276, -281, -286, -290, -295, -300, -304, -309, -314, -319, -323, -328, -333, -338, -343, -347, -352, -357, -362, -367, -371, -376, -381, -386, -391, -396, -400, -405, -410, -415, -420, -425, -430, -435, -439, -444, -449, -454, -459, -464, -469, -474, -479, -484, -489, -494, -499, -504, -509, -514, -519, -524, -529, -534, -539, -544, -549, -554, -559, -564, -570, -575, -580, -585, -590, -595, -600, -605, -611, -616, -621, -626, -631, -636, -642, -647, -652, -657, -663, -668, -673, -678, -684, -689, -694, -699, -705, -710, -715, -721, -726, -731, -737, -742, -747, -753, -758, -763, -769, -774, -780, -785, -791, -796, -801, -807, -812, -818, -823, -829, -834, -840, -845, -851, -856, -862, -868, -873, -879, -884, -890, -896, -901, -907, -912, -918, -924, -929, -935, -941, -946, -952, -958, -964, -969, -975, -981, -987, -992, -998, -1004, -1010, -1016, -1021, -1027, -1033, -1039, -1045, -1051, -1057, -1063, -1068, -1074, -1080, -1086, -1092, -1098, -1104, -1110, -1116, -1122, -1128, -1134, -1140, -1146, -1152, -1159, -1165, -1171, -1177, -1183, -1189, -1195, -1202, -1208, -1214, -1220, -1226, -1233, -1239, -1245, -1251, -1258, -1264, -1270, -1277, -1283, -1289, -1296, -1302, -1309, -1315, -1321, -1328, -1334, -1341, -1347, -1354, -1360, -1367, -1373, -1380, -1386, -1393, -1400, -1406, -1413, -1420, -1426, -1433, -1440, -1446, -1453, -1460, -1467, -1473, -1480, -1487, -1494, -1501, -1507, -1514, -1521, -1528, -1535, -1542, -1549, -1556, -1563, -1570, -1577, -1584, -1591, -1598, -1605, -1613, -1620, -1627, -1634, -1641, -1649, -1656, -1663, -1670, -1678, -1685, -1692, -1700, -1707, -1715, -1722, -1729, -1737, -1744, -1752, -1759, -1767, -1775, -1782, -1790, -1798, -1805, -1813, -1821, -1829, -1836, -1844, -1852, -1860, -1868, -1876, -1884, -1892, -1900, -1908, -1916, -1924, -1932, -1940, -1948, -1956, -1965, -1973, -1981},
    {993, 989, 986, 983, 979, 976, 972, 969, 965, 962, 959, 955, 952, 948, 945, 941, 938, 934, 931, 927, 924, 921, 917, 914, 910, 907, 903, 900, 896, 893, 889, 886, 882, 879, 875, 872, 868, 865, 861, 858, 854, 851, 847, 843, 840, 836, 833, 829, 826, 822, 819, 815, 812, 808, 804, 801, 797, 794, 790, 786, 783, 779, 776, 772, 769, 765, 761, 758, 754, 750, 747, 743, 740, 736, 732, 729, 725, 721, 718, 714, 711, 707, 703, 700, 696, 692, 689, 685, 681, 678, 674, 670, 666, 663, 659, 655, 652, 648, 644, 641, 637, 633, 629, 626, 622, 618, 614, 611, 607, 603, 599, 596, 592, 588, 584, 581, 577, 573, 569, 566, 562, 558, 554, 550, 547, 543, 539, 535, 531, 528, 524, 520, 516, 512, 508, 505, 501, 497, 493, 489, 485, 481, 478, 474, 470, 466, 462, 458, 454, 450, 447, 443, 439, 435, 431, 427, 423, 419, 415, 411, 407, 404, 400, 396, 392, 388, 384, 380, 376, 372, 368, 364, 360, 356, 352, 348, 344, 340, 336, 332, 328, 324, 320, 316, 312, 308, 304, 300, 296, 292, 288, 284, 280, 276, 271, 267, 263, 259, 255, 251, 247, 243, 239, 235, 231, 226, 222, 218, 214, 210, 206, 202, 198, 193, 189, 185, 181, 177, 173, 168, 164, 160, 156, 152, 147, 143, 139, 135, 131, 126, 122, 118, 114, 109, 105, 101, 97, 92, 88, 84, 80, 75, 71, 67, 63, 58, 54, 50, 45, 41, 37, 32, 28, 24, 19, 15, 11, 6, 2, -2, -7, -11, -15, -20, -24, -29, -33, -37, -42, -46, -51, -55, -59, -64, -68, -73, -77, -82, -86, -91, -95, -100, -104, -109, -113, -118, -122, -127, -131, -136, -140, -145, -149, -154, -158, -163, -167, -172, -176, -181, -186, -190, -195, -199, -204, -209, -213, -218, -222, -227, -232, -236, -241, -246, -250, -255, -260, -264, -269, -274, -278, -283, -288, -292, -297, -302, -307, -311, -316, -321, -326, -330, -335, -340, -345, -349, -354, -359, -364, -369, -373, -378, -383, -388, -393, -398, -402, -407, -412, -417, -422, -427, -432, -437, -441, -446, -451, -456, -461, -466, -471, -476, -481, -486, -491, -496, -501, -506, -511, -516, -521, -526, -531, -536, -541, -546, -551, -556, -561, -566, -571, -577, -582, -587, -592, -597, -602, -607, -612, -618, -623, -628, -633, -638, -643, -649, -654, -659, -664, -670, -675, -680, -685, -691, -696, -701, -706, -712, -717, -722, -728, -733, -738, -744, -749, -754, -760, -765, -770, -776, -781, -787, -792, -798, -803, -808, -814, -819, -825, -830, -836, -841, -847, -852, -858, -863, -869, -874, -880, -886, -891, -897, -902, -908, -914, -919, -925, -931, -936, -942, -948, -953, -959, -965, -970, -976, -982, -988, -993, -999, -1005, -1011, -1016, -1022, -1028, -1034, -1040, -1046, -1051, -1057, -1063, -1069, -1075, -1081, -1087, -1093, -1099, -1105, -1111, -1117, -1123, -1129, -1135, -1141, -1147, -1153, -1159, -1165, -1171, -1177, -1183, -1189, -1196, -1202, -1208, -1214, -1220, -1226, -1233, -1239, -1245, -1251, -1258, -1264, -1270, -1276, -1283, -1289, -1295, -1302, -1308, -1315, -1321, -1327, -1334, -1340, -1347, -1353, -1360, -1366, -1373, -1379, -1386, -1392, -1399, -1405, -1412, -1419, -1425, -1432, -1438, -1445, -1452, -1458, -1465, -1472, -1479, -1485, -1492, -1499, -1506, -1513, -1520, -1526, -1533, -1540, -1547, -1554, -1561, -1568, -1575, -1582, -1589, -1596, -1603, -1610, -1617, -1624, -1631, -1639, -1646, -1653, -1660, -1667, -1675, -1682, -1689, -1696, -1704, -1711, -1718, -1726, -1733, -1741, -1748, -1756, -1763, -1771, -1778, -1786, -1793, -1801, -1809, -1816, -1824, -1832, -1839, -1847, -1855, -1863, -1870, -1878, -1886, -1894, -1902, -1910, -1918, -1926, -1934, -1942, -1950},
    {1002, 999, 996, 992, 989, 985, 982, 978, 975, 972, 968, 965, 961, 958, 954, 951, 947, 944, 941, 937, 934, 930, 927, 923, 920, 916, 913, 909, 906, 902, 899, 895, 892, 888, 885, 881, 878, 874, 871, 867, 864, 860, 857, 853, 850, 846, 842, 839, 835, 832, 828, 825, 821, 818, 814, 810, 807, 803, 800, 796, 793, 789, 785, 782, 778, 775, 771, 767, 764, 760, 757, 753, 749, 746, 742, 738, 735, 731, 727, 724, 720, 717, 713, 709, 706, 702, 698, 695, 691, 687, 684, 680, 676, 672, 669, 665, 661, 658, 654, 650, 647, 643, 639, 635, 632, 628, 624, 620, 617, 613, 609, 606, 602, 598, 594, 590, 587, 583, 579, 575, 572, 568, 564, 560, 556, 553, 549, 545, 541, 537, 534, 530, 526, 522, 518, 515, 511, 507, 503, 499, 495, 491, 488, 484, 480, 476, 472, 468, 464, 460, 457, 453, 449, 445, 441, 437, 433, 429, 425, 421, 417, 414, 410, 406, 402, 398, 394, 390, 386, 382, 378, 374, 370, 366, 362, 358, 354, 350, 346, 342, 338, 334, 330, 326, 322, 318, 314, 310, 306, 302, 298, 294, 290, 286, 282, 278, 274, 270, 265, 261, 257, 253, 249, 245, 241, 237, 233, 229, 224, 220, 216, 212, 208, 204, 200, 196, 191, 187, 183, 179, 175, 171, 166, 162, 158, 154, 150, 145, 141, 137, 133, 129, 124, 120, 116, 112, 107, 103, 99, 95, 90, 86, 82, 78, 73, 69, 65, 60, 56, 52, 48, 43, 39, 35, 30, 26, 22, 17, 13, 9, 4, 0, -4, -9, -13, -18, -22, -26, -31, -35, -40, -44, -48, -53, -57, -62, -66, -71, -75, -79, -84, -88, -93, -97, -102, -106, -111, -115, -120, -124, -129, -133, -138, -142, -147, -151, -156, -160, -165, -170, -174, -179, -183, -188, -192, -197, -202, -206, -211, -215, -220, -225, -229, -234, -239, -243, -248, -252, -257, -262, -266, -271, -276, -281, -285, -290, -295, -299, -304, -309, -314, -318, -323, -328, -333, -337, -342, -347, -352, -356, -361, -366, -371, -376, -380, -385, -390, -395, -400, -405, -409, -414, -419, -424, -429, -434, -439, -444, -449, -453, -458, -463, -468, -473, -478, -483, -488, -493, -498, -503, -508, -513, -518, -523, -528, -533, -538, -543, -548, -553, -558, -563, -568, -573, -579, -584, -589, -594, -599, -604, -609, -614, -620, -625, -630, -635, -640, -645, -651, -656, -661, -666, -671, -677, -682, -687, -692, -698, -703, -708, -713, -719, -724, -729, -735, -740, -745, -751, -756, -761, -767, -772, -778, -783, -788, -794, -799, -805, -810, -815, -821, -826, -832, -837, -843, -848, -854, -859, -865, -870, -876, -882, -887, -893, -898, -904, -909, -915, -921, -926, -932, -938, -943, -949, -955, -960, -966, -972, -977, -983, -989, -994, -1000, -1006, -1012, -1018, -1023, -1029, -1035, -1041, -1047, -1052, -1058, -1064, -1070, -1076, -1082, -1088, -1094, -1100, -1106, -1111, -1117, -1123, -1129, -1135, -1141, -1147, -1153, -1159, -1166, -1172, -1178, -1184, -1190, -1196, -1202, -1208, -1214, -1220, -1227, -1233, -1239, -1245, -1251, -1258, -1264, -1270, -1276, -1283, -1289, -1295, -1302, -1308, -1314, -1321, -1327, -1333, -1340, -1346, -1353, -1359, -1366, -1372, -1379, -1385, -1392, -1398, -1405, -1411, -1418, -1424, -1431, -1438, -1444, -1451, -1457, -1464, -1471, -1477, -1484, -1491, -1498, -1504, -1511, -1518, -1525, -1532, -1539, -1545, -1552, -1559, -1566, -1573, -1580, -1587, -1594, -1601, -1608, -1615, -1622, -1629, -1636, -1643, -1650, -1657, -1665, -1672, -1679, -1686, -1693, -1701, -1708, -1715, -1723, -1730, -1737, -1745, -1752, -1759, -1767, -1774, -1782, -1789, -1797, -1804, -1812, -1820, -1827, -1835, -1842, -1850, -1858, -1866, -1873, -1881, -1889, -1897, -1905, -1913, -1920},
    {1012, 1008, 1005, 1002, 998, 995, 991, 988, 985, 981, 978, 974, 971, 967, 964, 960, 957, 954, 950, 947, 943, 940, 936, 933, 929, 926, 922, 919, 915, 912, 908, 905, 901, 898, 894, 891, 887, 884, 880, 877, 873, 870, 866, 863, 859, 856, 852, 848, 845, 841, 838, 834, 831, 827, 824, 820, 816, 813, 809, 806, 802, 799, 795, 791, 788, 784, 781, 777, 773, 770, 766, 763, 759, 755, 752, 748, 744, 741, 737, 733, 730, 726, 723, 719, 715, 712, 708, 704, 701, 697, 693, 690, 686, 682, 678, 675, 671, 667, 664, 660, 656, 653, 649, 645, 641, 638, 634, 630, 626, 623, 619, 615, 611, 608, 604, 600, 596, 593, 589, 585, 581, 578, 574, 570, 566, 562, 559, 555, 551, 547, 543, 540, 536, 532, 528, 524, 520, 517, 513, 509, 505, 501, 497, 494, 490, 486, 482, 478, 474, 470, 466, 463, 459, 455, 451, 447, 443, 439, 435, 431, 427, 424, 420, 416, 412, 408, 404, 400, 396, 392, 388, 384, 380, 376, 372, 368, 364, 360, 356, 352, 348, 344, 340, 336, 332, 328, 324, 320, 316, 312, 308, 304, 300, 296, 292, 288, 284, 280, 276, 272, 268, 263, 259, 255, 251, 247, 243, 239, 235, 231, 227, 222, 218, 214, 210, 206, 202, 198, 193, 189, 185, 181, 177, 173, 168, 164, 160, 156, 152, 147, 143, 139, 135, 131, 126, 122, 118, 114, 109, 105, 101, 97, 92, 88, 84, 80, 75, 71, 67, 63, 58, 54, 50, 45, 41, 37, 32, 28, 24, 19, 15, 11, 6, 2, -2, -7, -11, -15, -20, -24, -29, -33, -37, -42, -46, -51, -55, -60, -64, -68, -73, -77, -82, -86, -91, -95, -100, -104, -109, -113, -118, -122, -127, -131, -136, -140, -145, -149, -154, -158, -163, -167, -172, -176, -181, -186, -190, -195, -199, -204, -208, -213, -218, -222, -227, -232, -236, -241, -245, -250, -255, -259, -264, -269, -273, -278, -283, -288, -292, -297, -302, -306, -311, -316, -321, -325, -330, -335, -340, -344, -349, -354, -359, -363, -368, -373, -378, -383, -388, -392, -397, -402, -407, -412, -417, -421, -426, -431, -436, -441, -446, -451, -456, -461, -466, -470, -475, -480, -485, -490, -495, -500, -505, -510, -515, -520, -525, -530, -535, -540, -545, -550, -555, -560, -565, -571, -576, -581, -586, -591, -596, -601, -606, -611, -616, -622, -627, -632, -637, -642, -647, -653, -658, -663, -668, -673, -679, -684, -689, -694, -700, -705, -710, -715, -721, -726, -731, -737, -742, -747, -753, -758, -763, -769, -774, -779, -785, -790, -796, -801, -806, -812, -817, -823, -828, -834, -839, -845, -850, -856, -861, -867, -872, -878, -883, -889, -894, -900, -905, -911, -917, -922, -928, -933, -939, -945, -950, -956, -962, -967, -973, -979, -984, -990, -996, -1002, -1007, -1013, -1019, -1025, -1030, -1036, -1042, -1048, -1054, -1059, -1065, -1071, -1077, -1083, -1089, -1095, -1101, -1106, -1112, -1118, -1124, -1130, -1136, -1142, -1148, -1154, -1160, -1166, -1172, -1178, -1184, -1190, -1196, -1203, -1209, -1215, -1221, -1227, -1233, -1239, -1246, -1252, -1258, -1264, -1270, -1277, -1283, -1289, -1295, -1302, -1308, -1314, -1321, -1327, -1333, -1340, -1346, -1352, -1359, -1365, -1372, -1378, -1385, -1391, -1398, -1404, -1411, -1417, -1424, -1430, -1437, -1443, -1450, -1457, -1463, -1470, -1477, -1483, -1490, -1497, -1503, -1510, -1517, -1524, -1530, -1537, -1544, -1551, -1558, -1564, -1571, -1578, -1585, -1592, -1599, -1606, -1613, -1620, -1627, -1634, -1641, -1648, -1655, -1662, -1669, -1677, -1684, -1691, -1698, -1705, -1712, -1720, -1727, -1734, -1742, -1749, -1756, -1764, -1771, -1778, -1786, -1793, -1801, -1808, -1816, -1823, -1831, -1838, -1846, -1854, -1861, -1869, -1877, -1884, -1892},
    {1021, 1018, 1015, 1011, 1008, 1004, 1001, 997, 994, 991, 987, 984, 980, 977, 973, 970, 966, 963, 959, 956, 953, 949, 946, 942, 939, 935, 932, 928, 925, 921, 918, 914, 911, 907, 904, 900, 897, 893, 890, 886, 883, 879, 876, 872, 869, 865, 861, 858, 854, 851, 847, 844, 840, 837, 833, 829, 826, 822, 819, 815, 812, 808, 804, 801, 797, 794, 790, 786, 783, 779, 776, 772, 768, 765, 761, 758, 754, 750, 747, 743, 739, 736, 732, 728, 725, 721, 717, 714, 710, 706, 703, 699, 695, 692, 688, 684, 681, 677, 673, 670, 666, 662, 658, 655, 651, 647, 644, 640, 636, 632, 629, 625, 621, 617, 614, 610, 606, 602, 599, 595, 591, 587, 583, 580, 576, 572, 568, 565, 561, 557, 553, 549, 546, 542, 538, 534, 530, 526, 523, 519, 515, 511, 507, 503, 499, 496, 492, 488, 484, 480, 476, 472, 469, 465, 461, 457, 453, 449, 445, 441, 437, 433, 429, 426, 422, 418, 414, 410, 406, 402, 398, 394, 390, 386, 382, 378, 374, 370, 366, 362, 358, 354, 350, 346, 342, 338, 334, 330, 326, 322, 318, 314, 310, 306, 302, 298, 294, 290, 286, 282, 278, 274, 269, 265, 261, 257, 253, 249, 245, 241, 237, 233, 228, 224, 220, 216, 212, 208, 204, 200, 195, 191, 187, 183, 179, 175, 170, 166, 162, 158, 154, 149, 145, 141, 137, 133, 128, 124, 120, 116, 111, 107, 103, 99, 94, 90, 86, 82, 77, 73, 69, 65, 60, 56, 52, 47, 43, 39, 34, 30, 26, 21, 17, 13, 8, 4, 0, -5, -9, -13, -18, -22, -27, -31, -35, -40, -44, -49, -53, -57, -62, -66, -71, -75, -80, -84, -89, -93, -97, -102, -106, -111, -115, -120, -124, -129, -133, -138, -142, -147, -152, -156, -161, -165, -170, -174, -179, -183, -188, -193, -197, -202, -206, -211, -215, -220, -225, -229, -234, -239, -243, -248, -253, -257, -262, -267, -271, -276, -281, -285, -290, -295, -299, -304, -309, -313, -318, -323, -328, -332, -337, -342, -347, -352, -356, -361, -366, -371, -375, -380, -385, -390, -395, -400, -404, -409, -414, -419, -424, -429, -434, -438, -443, -448, -453, -458, -463, -468, -473, -478, -483, -488, -493, -498, -503, -507, -512, -517, -522, -527, -532, -537, -543, -548, -553, -558, -563, -568, -573, -578, -583, -588, -593, -598, -603, -608, -614, -619, -624, -629, -634, -639, -644, -650, -655, -660, -665, -670, -676, -681, -686, -691, -696, -702, -707, -712, -717, -723, -728, -733, -739, -744, -749, -755, -760, -765, -771, -776, -781, -787, -792, -797, -803, -808, -814, -819, -825, -830, -835, -841, -846, -852, -857, -863, -868, -874, -879, -885, -890, -896, -901, -907, -913, -918, -924, -929, -935, -941, -946, -952, -957, -963, -969, -974, -980, -986, -992, -997, -1003, -1009, -1014, -1020, -1026, -1032, -1037, -1043, -1049, -1055, -1061, -1066, -1072, -1078, -1084, -1090, -1096, -1102, -1108, -1113, -1119, -1125, -1131, -1137, -1143, -1149, -1155, -1161, -1167, -1173, -1179, -1185, -1191, -1197, -1203, -1209, -1215, -1222, -1228, -1234, -1240, -1246, -1252, -1258, -1265, -1271, -1277, -1283, -1289, -1296, -1302, -1308, -1315, -1321, -1327, -1333, -1340, -1346, -1352, -1359, -1365, -1372, -1378, -1384, -1391, -1397, -1404, -1410, -1417, -1423, -1430, -1436, -1443, -1449, -1456, -1463, -1469, -1476, -1482, -1489, -1496, -1502, -1509, -1516, -1523, -1529, -1536, -1543, -1550, -1556, -1563, -1570, -1577, -1584, -1591, -1597, -1604, -1611, -1618, -1625, -1632, -1639, -1646, -1653, -1660, -1667, -1674, -1681, -1689, -1696, -1703, -1710, -1717, -1724, -1732, -1739, -1746, -1753, -1761, -1768, -1775, -1783, -1790, -1797, -1805, -1812, -1820, -1827, -1835, -1842, -1850, -1857, -1865},
    {1031, 1027, 1024, 1020, 1017, 1014, 1010, 1007, 1003, 1000, 996, 993, 990, 986, 983, 979, 976, 972, 969, 965, 962, 958, 955, 952, 948, 945, 941, 938, 934, 931, 927, 924, 920, 917, 913, 910, 906, 903, 899, 896, 892, 889, 885, 881, 878, 874, 871, 867, 864, 860, 857, 853, 850, 846, 842, 839, 835, 832, 828, 825, 821, 817, 814, 810, 807, 803, 799, 796, 792, 789, 785, 781, 778, 774, 771, 767, 763, 760, 756, 752, 749, 745, 742, 738, 734, 731, 727, 723, 720, 716, 712, 709, 705, 701, 698, 694, 690, 686, 683, 679, 675, 672, 668, 664, 661, 657, 653, 649, 646, 642, 638, 634, 631, 627, 623, 619, 616, 612, 608, 604, 601, 597, 593, 589, 586, 582, 578, 574, 570, 567, 563, 559, 555, 551, 548, 544, 540, 536, 532, 528, 525, 521, 517, 513, 509, 505, 501, 498, 494, 490, 486, 482, 478, 474, 470, 467, 463, 459, 455, 451, 447, 443, 439, 435, 431, 427, 424, 420, 416, 412, 408, 404, 400, 396, 392, 388, 384, 380, 376, 372, 368, 364, 360, 356, 352, 348, 344, 340, 336, 332, 328, 324, 320, 316, 312, 308, 304, 300, 296, 292, 288, 284, 280, 275, 271, 267, 263, 259, 255, 251, 247, 243, 239, 234, 230, 226, 222, 218, 214, 210, 206, 201, 197, 193, 189, 185, 181, 176, 172, 168, 164, 160, 156, 151, 147, 143, 139, 134, 130, 126, 122, 118, 113, 109, 105, 101, 96, 92, 88, 84, 79, 75, 71, 66, 62, 58, 54, 49, 45, 41, 36, 32, 28, 23, 19, 15, 10, 6, 2, -3, -7, -12, -16, -20, -25, -29, -33, -38, -42, -47, -51, -56, -60, -64, -69, -73, -78, -82, -87, -91, -96, -100, -104, -109, -113, -118, -122, -127, -131, -136, -140, -145, -149, -154, -159, -163, -168, -172, -177, -181, -186, -190, -195, -200, -204, -209, -213, -218, -223, -227, -232, -236, -241, -246, -250, -255, -260, -264, -269, -274, -278, -283, -288, -292, -297, -302, -307, -311, -316, -321, -325, -330, -335, -340, -344, -349, -354, -359, -364, -368, -373, -378, -383, -388, -392, -397, -402, -407, -412, -417, -421, -426, -431, -436, -441, -446, -451, -456, -461, -465, -470, -475, -480, -485, -490, -495, -500, -505, -510, -515, -520, -525, -530, -535, -540, -545, -550, -555, -560, -565, -570, -575, -580, -585, -590, -595, -601, -606, -611, -616, -621, -626, -631, -636, -642, -647, -652, -657, -662, -667, -673, -678, -683, -688, -693, -699, -704, -709, -714, -720, -725, -730, -736, -741, -746, -751, -757, -762, -767, -773, -778, -783, -789, -794, -800, -805, -810, -816, -821, -827, -832, -837, -843, -848, -854, -859, -865, -870, -876, -881, -887, -892, -898, -903, -909, -914, -920, -926, -931, -937, -942, -948, -954, -959, -965, -970, -976, -982, -987, -993, -999, -1004, -1010, -1016, -1022, -1027, -1033, -1039, -1045, -1050, -1056, -1062, -1068, -1074, -1079, -1085, -1091, -1097, -1103, -1109, -1115, -1121, -1126, -1132, -1138, -1144, -1150, -1156, -1162, -1168, -1174, -1180, -1186, -1192, -1198, -1204, -1210, -1216, -1222, -1228, -1235, -1241, -1247, -1253, -1259, -1265, -1271, -1278, -1284, -1290, -1296, -1302, -1309, -1315, -1321, -1327, -1334, -1340, -1346, -1353, -1359, -1365, -1372, -1378, -1384, -1391, -1397, -1404, -1410, -1417, -1423, -1430, -1436, -1443, -1449, -1456, -1462, -1469, -1475, -1482, -1488, -1495, -1502, -1508, -1515, -1522, -1528, -1535, -1542, -1549, -1555, -1562, -1569, -1576, -1582, -1589, -1596, -1603, -1610, -1617, -1624, -1631, -1637, -1644, -1651, -1658, -1665, -1672, -1679, -1686, -1694, -1701, -1708, -1715, -1722, -1729, -1736, -1744, -1751, -1758, -1765, -1772, -1780, -1787, -1794, -1802, -1809, -1816, -1824, -1831, -1839},
    {1040, 1037, 1033, 1030, 1026, 1023, 1019, 1016, 1013, 1009, 1006, 1002, 999, 995, 992, 989, 985, 982, 978, 975, 971, 968, 964, 961, 957, 954, 950, 947, 943, 940, 936, 933, 929, 926, 922, 919, 915, 912, 908, 905, 901, 898, 894, 891, 887, 884, 880, 877, 873, 870, 866, 862, 859, 855, 852, 848, 845, 841, 838, 834, 830, 827, 823, 820, 816, 812, 809, 805, 802, 798, 794, 791, 787, 784, 780, 776, 773, 769, 765, 762, 758, 755, 751, 747, 744, 740, 736, 733, 729, 725, 722, 718, 714, 711, 707, 703, 700, 696, 692, 689, 685, 681, 677, 674, 670, 666, 663, 659, 655, 651, 648, 644, 640, 636, 633, 629, 625, 621, 618, 614, 610, 606, 603, 599, 595, 591, 587, 584, 580, 576, 572, 569, 565, 561, 557, 553, 549, 546, 542, 538, 534, 530, 526, 523, 519, 515, 511, 507, 503, 499, 496, 492, 488, 484, 480, 476, 472, 468, 465, 461, 457, 453, 449, 445, 441, 437, 433, 429, 425, 421, 418, 414, 410, 406, 402, 398, 394, 390, 386, 382, 378, 374, 370, 366, 362, 358, 354, 350, 346, 342, 338, 334, 330, 326, 322, 318, 314, 310, 306, 302, 298, 294, 289, 285, 281, 277, 273, 269, 265, 261, 257, 253, 249, 245, 240, 236, 232, 228, 224, 220, 216, 212, 207, 203, 199, 195, 191, 187, 182, 178, 174, 170, 166, 162, 157, 153, 149, 145, 141, 136, 132, 128, 124, 119, 115, 111, 107, 102, 98, 94, 90, 85, 81, 77, 73, 68, 64, 60, 55, 51, 47, 42, 38, 34, 29, 25, 21, 16, 12, 8, 3, -1, -5, -10, -14, -18, -23, -27, -32, -36, -40, -45, -49, -54, -58, -62, -67, -71, -76, -80, -85, -89, -94, -98, -103, -107, -112, -116, -120, -125, -129, -134, -139, -143, -148, -152, -157, -161, -166, -170, -175, -179, -184, -188, -193, -198, -202, -207, -211, -216, -221, -225, -230, -234, -239, -244, -248, -253, -258, -262, -267, -272, -276, -281, -286, -290, -295, -300, -304, -309, -314, -319, -323, -328, -333, -338, -342, -347, -352, -357, -361, -366, -371, -376, -381, -385, -390, -395, -400, -405, -409, -414, -419, -424, -429, -434, -439, -443, -448, -453, -458, -463, -468, -473, -478, -483, -488, -493, -498, -503, -507, -512, -517, -522, -527, -532, -537, -542, -547, -552, -557, -562, -568, -573, -578, -583, -588, -593, -598, -603, -608, -613, -618, -623, -629, -634, -639, -644, -649, -654, -659, -665, -670, -675, -680, -685, -691, -696, -701, -706, -711, -717, -722, -727, -732, -738, -743, -748, -754, -759, -764, -770, -775, -780, -786, -791, -796, -802, -807, -812, -818, -823, -829, -834, -839, -845, -850, -856, -861, -867, -872, -878, -883, -889, -894, -900, -905, -911, -916, -922, -927, -933, -939, -944, -950, -955, -961, -967, -972, -978, -983, -989, -995, -1000, -1006, -1012, -1018, -1023, -1029, -1035, -1040, -1046, -1052, -1058, -1064, -1069, -1075, -1081, -1087, -1093, -1098, -1104, -1110, -1116, -1122, -1128, -1134, -1140, -1145, -1151, -1157, -1163, -1169, -1175, -1181, -1187, -1193, -1199, -1205, -1211, -1217, -1223, -1229, -1235, -1242, -1248, -1254, -1260, -1266, -1272, -1278, -1284, -1291, -1297, -1303, -1309, -1315, -1322, -1328, -1334, -1340, -1347, -1353, -1359, -1366, -1372, -1378, -1385, -1391, -1397, -1404, -1410, -1417, -1423, -1430, -1436, -1442, -1449, -1455, -1462, -1468, -1475, -1482, -1488, -1495, -1501, -1508, -1514, -1521, -1528, -1534, -1541, -1548, -1555, -1561, -1568, -1575, -1581, -1588, -1595, -1602, -1609, -1615, -1622, -1629, -1636, -1643, -1650, -1657, -1664, -1671, -1678, -1685, -1692, -1699, -1706, -1713, -1720, -1727, -1734, -1741, -1748, -1756, -1763, -1770, -1777, -1784, -1792, -1799, -1806, -1814},
    {1049, 1046, 1042, 1039, 1036, 1032, 1029, 1025, 1022, 1018, 1015, 1012, 1008, 1005, 1001, 998, 994, 991, 987, 984, 980, 977, 974, 970, 967, 963, 960, 956, 953, 949, 946, 942, 939, 935, 932, 928, 925, 921, 918, 914, 911, 907, 904, 900, 897, 893, 889, 886, 882, 879, 875, 872, 868, 865, 861, 857, 854, 850, 847, 843, 840, 836, 832, 829, 825, 822, 818, 815, 811, 807, 804, 800, 797, 793, 789, 786, 782, 778, 775, 771, 768, 764, 760, 757, 753, 749, 746, 742, 738, 735, 731, 727, 724, 720, 716, 713, 709, 705, 702, 698, 694, 690, 687, 683, 679, 676, 672, 668, 664, 661, 657, 653, 650, 646, 642, 638, 635, 631, 627, 623, 620, 616, 612, 608, 604, 601, 597, 593, 589, 586, 582, 578, 574, 570, 567, 563, 559, 555, 551, 547, 544, 540, 536, 532, 528, 524, 521, 517, 513, 509, 505, 501, 497, 494, 490, 486, 482, 478, 474, 470, 466, 462, 459, 455, 451, 447, 443, 439, 435, 431, 427, 423, 419, 415, 411, 407, 403, 400, 396, 392, 388, 384, 380, 376, 372, 368, 364, 360, 356, 352, 348, 344, 340, 336, 332, 328, 324, 320, 315, 311, 307, 303, 299, 295, 291, 287, 283, 279, 275, 271, 267, 263, 259, 254, 250, 246, 242, 238, 234, 230, 226, 221, 217, 213, 209, 205, 201, 197, 192, 188, 184, 180, 176, 172, 167, 163, 159, 155, 151, 146, 142, 138, 134, 130, 125, 121, 117, 113, 108, 104, 100, 96, 91, 87, 83, 79, 74, 70, 66, 61, 57, 53, 48, 44, 40, 36, 31, 27, 23, 18, 14, 10, 5, 1, -4, -8, -12, -17, -21, -25, -30, -34, -39, -43, -47, -52, -56, -61, -65, -70, -74, -78, -83, -87, -92, -96, -101, -105, -110, -114, -119, -123, -128, -132, -137, -141, -146, -150, -155, -159, -164, -168, -173, -177, -182, -187, -191, -196, -200, -205, -210, -214, -219, -223, -228, -233, -237, -242, -246, -251, -256, -260, -265, -270, -274, -279, -284, -288, -293, -298, -302, -307, -312, -317, -321, -326, -331, -336, -340, -345, -350, -355, -359, -364, -369, -374, -378, -383, -388, -393, -398, -402, -407, -412, -417, -422, -427, -432, -436, -441, -446, -451, -456, -461, -466, -471, -476, -480, -485, -490, -495, -500, -505, -510, -515, -520, -525, -530, -535, -540, -545, -550, -555, -560, -565, -570, -575, -580, -585, -590, -595, -600, -606, -611, -616, -621, -626, -631, -636, -641, -646, -652, -657, -662, -667, -672, -677, -683, -688, -693, -698, -703, -709, -714, -719, -724, -730, -735, -740, -745, -751, -756, -761, -767, -772, -777, -783, -788, -793, -799, -804, -809, -815, -820, -825, -831, -836, -842, -847, -853, -858, -863, -869, -874, -880, -885, -891, -896, -902, -907, -913, -918, -924, -929, -935, -941, -946, -952, -957, -963, -968, -974, -980, -985, -991, -997, -1002, -1008, -1014, -1019, -1025, -1031, -1036, -1042, -1048, -1054, -1059, -1065, -1071, -1077, -1083, -1088, -1094, -1100, -1106, -1112, -1117, -1123, -1129, -1135, -1141, -1147, -1153, -1159, -1165, -1171, -1176, -1182, -1188, -1194, -1200, -1206, -1212, -1218, -1224, -1230, -1237, -1243, -1249, -1255, -1261, -1267, -1273, -1279, -1285, -1291, -1298, -1304, -1310, -1316, -1322, -1329, -1335, -1341, -1347, -1354, -1360, -1366, -1372, -1379, -1385, -1391, -1398, -1404, -1410, -1417, -1423, -1430, -1436, -1443, -1449, -1455, -1462, -1468, -1475, -1481, -1488, -1494, -1501, -1508, -1514, -1521, -1527, -1534, -1541, -1547, -1554, -1561, -1567, -1574, -1581, -1587, -1594, -1601, -1608, -1615, -1621, -1628, -1635, -1642, -1649, -1656, -1662, -1669, -1676, -1683, -1690, -1697, -1704, -1711, -1718, -1725, -1732, -1739, -1746, -1754, -1761, -1768, -1775, -1782, -1789},
    {1058, 1055, 1051, 1048, 1045, 1041, 1038, 1034, 1031, 1028, 1024, 1021, 1017, 1014, 1010, 1007, 1003, 1000, 997, 993, 990, 986, 983, 979, 976, 972, 969, 965, 962, 958, 955, 951, 948, 944, 941, 937, 934, 930, 927, 923, 920, 916, 913, 909, 906, 902, 899, 895, 892, 888, 884, 881, 877, 874, 870, 867, 863, 860, 856, 852, 849, 845, 842, 838, 835, 831, 827, 824, 820, 817, 813, 809, 806, 802, 799, 795, 791, 788, 784, 780, 777, 773, 769, 766, 762, 759, 755, 751, 748, 744, 740, 737, 733, 729, 726, 722, 718, 715, 711, 707, 703, 700, 696, 692, 689, 685, 681, 678, 674, 670, 666, 663, 659, 655, 651, 648, 644, 640, 636, 633, 629, 625, 621, 618, 614, 610, 606, 603, 599, 595, 591, 587, 584, 580, 576, 572, 568, 565, 561, 557, 553, 549, 545, 542, 538, 534, 530, 526, 522, 518, 515, 511, 507, 503, 499, 495, 491, 488, 484, 480, 476, 472, 468, 464, 460, 456, 452, 448, 445, 441, 437, 433, 429, 425, 421, 417, 413, 409, 405, 401, 397, 393, 389, 385, 381, 377, 373, 369, 365, 361, 357, 353, 349, 345, 341, 337, 333, 329, 325, 321, 317, 313, 309, 305, 301, 297, 293, 289, 285, 281, 277, 272, 268, 264, 260, 256, 252, 248, 244, 240, 236, 231, 227, 223, 219, 215, 211, 207, 202, 198, 194, 190, 186, 182, 177, 173, 169, 165, 161, 156, 152, 148, 144, 140, 135, 131, 127, 123, 118, 114, 110, 106, 102, 97, 93, 89, 84, 80, 76, 72, 67, 63, 59, 54, 50, 46, 41, 37, 33, 29, 24, 20, 16, 11, 7, 2, -2, -6, -11, -15, -19, -24, -28, -33, -37, -41, -46, -50, -55, -59, -63, -68, -72, -77, -81, -86, -90, -95, -99, -104, -108, -112, -117, -121, -126, -130, -135, -139, -144, -148, -153, -158, -162, -167, -171, -176, -180, -185, -189, -194, -199, -203, -208, -212, -217, -221, -226, -231, -235, -240, -245, -249, -254, -259, -263, -268, -272, -277, -282, -286, -291, -296, -301, -305, -310, -315, -319, -324, -329, -334, -338, -343, -348, -353, -357, -362, -367, -372, -376, -381, -386, -391, -396, -400, -405, -410, -415, -420, -425, -429, -434, -439, -444, -449, -454, -459, -464, -468, -473, -478, -483, -488, -493, -498, -503, -508, -513, -518, -523, -528, -533, -538, -543, -548, -553, -558, -563, -568, -573, -578, -583, -588, -593, -598, -603, -608, -613, -618, -623, -629, -634, -639, -644, -649, -654, -659, -664, -670, -675, -680, -685, -690, -696, -701, -706, -711, -716, -722, -727, -732, -737, -743, -748, -753, -758, -764, -769, -774, -780, -785, -790, -796, -801, -806, -812, -817, -822, -828, -833, -839, -844, -849, -855, -860, -866, -871, -877, -882, -888, -893, -898, -904, -909, -915, -921, -926, -932, -937, -943, -948, -954, -959, -965, -971, -976, -982, -987, -993, -999, -1004, -1010, -1016, -1021, -1027, -1033, -1038, -1044, -1050, -1055, -1061, -1067, -1073, -1078, -1084, -1090, -1096, -1102, -1107, -1113, -1119, -1125, -1131, -1137, -1142, -1148, -1154, -1160, -1166, -1172, -1178, -1184, -1190, -1196, -1202, -1208, -1214, -1220, -1226, -1232, -1238, -1244, -1250, -1256, -1262, -1268, -1274, -1280, -1286, -1292, -1299, -1305, -1311, -1317, -1323, -1329, -1336, -1342, -1348, -1354, -1361, -1367, -1373, -1379, -1386, -1392, -1398, -1405, -1411, -1417, -1424, -1430, -1436, -1443, -1449, -1456, -1462, -1469, -1475, -1481, -1488, -1494, -1501, -1508, -1514, -1521, -1527, -1534, -1540, -1547, -1554, -1560, -1567, -1573, -1580, -1587, -1594, -1600, -1607, -1614, -1621, -1627, -1634, -1641, -1648, -1655, -1661, -1668, -1675, -1682, -1689, -1696, -1703, -1710, -1717, -1724, -1731, -1738, -1745, -1752, -1759, -1766},
    {1067, 1064, 1061, 1057, 1054, 1050, 1047, 1043, 1040, 1037, 1033, 1030, 1026, 1023, 1019, 1016, 1013, 1009, 1006, 1002, 999, 995, 992, 988, 985, 981, 978, 974, 971, 967, 964, 960, 957, 953, 950, 946, 943, 939, 936, 932, 929, 925, 922, 918, 915, 911, 908, 904, 901, 897, 894, 890, 886, 883, 879, 876, 872, 869, 865, 862, 858, 854, 851, 847, 844, 840, 836, 833, 829, 826, 822, 818, 815, 811, 808, 804, 800, 797, 793, 790, 786, 782, 779, 775, 771, 768, 764, 760, 757, 753, 749, 746, 742, 738, 735, 731, 727, 724, 720, 716, 713, 709, 705, 702, 698, 694, 690, 687, 683, 679, 676, 672, 668, 664, 661, 657, 653, 649, 646, 642, 638, 634, 631, 627, 623, 619, 616, 612, 608, 604, 600, 597, 593, 589, 585, 581, 578, 574, 570, 566, 562, 559, 555, 551, 547, 543, 539, 536, 532, 528, 524, 520, 516, 512, 509, 505, 501, 497, 493, 489, 485, 481, 477, 474, 470, 466, 462, 458, 454, 450, 446, 442, 438, 434, 430, 427, 423, 419, 415, 411, 407, 403, 399, 395, 391, 387, 383, 379, 375, 371, 367, 363, 359, 355, 351, 347, 343, 339, 335, 331, 327, 323, 319, 315, 311, 307, 303, 298, 294, 290, 286, 282, 278, 274, 270, 266, 262, 258, 254, 249, 245, 241, 237, 233, 229, 225, 221, 216, 212, 208, 204, 200, 196, 191, 187, 183, 179, 175, 171, 166, 162, 158, 154, 150, 145, 141, 137, 133, 129, 124, 120, 116, 112, 107, 103, 99, 95, 90, 86, 82, 77, 73, 69, 65, 60, 56, 52, 47, 43, 39, 34, 30, 26, 21, 17, 13, 8, 4, 0, -5, -9, -13, -18, -22, -27, -31, -35, -40, -44, -49, -53, -57, -62, -66, -71, -75, -80, -84, -88, -93, -97, -102, -106, -111, -115, -120, -124, -129, -133, -138, -142, -147, -151, -156, -160, -165, -169, -174, -179, -183, -188, -192, -197, -201, -206, -211, -215, -220, -224, -229, -234, -238, -243, -247, -252, -257, -261, -266, -271, -275, -280, -285, -289, -294, -299, -303, -308, -313, -318, -322, -327, -332, -336, -341, -346, -351, -355, -360, -365, -370, -375, -379, -384, -389, -394, -398, -403, -408, -413, -418, -423, -427, -432, -437, -442, -447, -452, -457, -462, -466, -471, -476, -481, -486, -491, -496, -501, -506, -511, -516, -521, -526, -531, -536, -540, -545, -550, -555, -560, -565, -571, -576, -581, -586, -591, -596, -601, -606, -611, -616, -621, -626, -631, -636, -641, -647, -652, -657, -662, -667, -672, -677, -683, -688, -693, -698, -703, -709, -714, -719, -724, -729, -735, -740, -745, -750, -756, -761, -766, -772, -777, -782, -787, -793, -798, -803, -809, -814, -820, -825, -830, -836, -841, -846, -852, -857, -863, -868, -874, -879, -884, -890, -895, -901, -906, -912, -917, -923, -928, -934, -939, -945, -950, -956, -962, -967, -973, -978, -984, -989, -995, -1001, -1006, -1012, -1018, -1023, -1029, -1035, -1040, -1046, -1052, -1057, -1063, -1069, -1075, -1080, -1086, -1092, -1098, -1103, -1109, -1115, -1121, -1127, -1132, -1138, -1144, -1150, -1156, -1162, -1168, -1174, -1179, -1185, -1191, -1197, -1203, -1209, -1215, -1221, -1227, -1233、、、
、、、vmc.h
#include "zf_common_headfile.h"
#ifndef CODE_VMC_H_
#define CODE_VMC_H_


/*********************************************************************参数*********************************************************************/
//#define L1  6.0f    //左小腿长
//#define L2  9.0f    //左大腿长
//#define L3  9.0f    //右大腿长
//#define L4  6.0f    //右小腿长
//#define L5  3.7f    //舵机间距
/*********************************************************************参数*********************************************************************/


/*********************************************************************函数*********************************************************************/
void servo_control_table(float p, float angle, int16* pwm1, int16* pwm2);
/*********************************************************************函数*********************************************************************/


#endif /* CODE_VMC_H_ */、、、



