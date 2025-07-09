# RDK-1
本作品为基于RDK和yolov8的智能轮腿盲道检测系统。发明专利已受理。作品用到了深度学习、目标检测、实例分割、Linux系统、机器视觉、轮腿平衡、万物互联（esp8266）、单片机控制等技术。近年来盲道占用现象愈发严重、盲道设计不合理问题突出（例如被井盖和柱子拦断），盲人生命安全受到威胁，且目前国内尚无不用佩戴在盲人身上、可以识别盲道与旁边路面同色系且可以及时广泛的通知城市管理人员清理盲道障碍和维护盲道缺陷的全自动智能化系统。本系统基于高性能计算平台RDK X5和先进的YOLOv8目标检测算法，设计了一套高效、精准的盲道检测解决方案。系统通过搭载摄像头实时采集路面图像，利用YOLOv8强大的视觉识别能力，快速检测并定位盲道区域，同时结合轮腿式移动平台，实现复杂环境下的稳定导航与避障功能。
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

