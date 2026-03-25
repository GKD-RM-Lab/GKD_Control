/*************************************************************

RM自定义UI协议       基于RM2020学生串口通信协议V1.1

山东理工大学 齐奇战队 东东@Rjgawuie

**************************************************************/

#include "UI.hpp"

#include <chrono>
#include <climits>
#include <cstdarg>
#include <cstdio>
#include <string>
#include <thread>

static unsigned char UI_com[512];
static int UI_tot = 0;

UI_control_t UI_control;
Types::RC_ctrl_t rc_control;
uint16_t Robot_ID_Read = Robot_ID;
uint16_t Cilent_ID_Read = Cilent_ID;

unsigned char UI_Seq;  // 包序号

// 测试
#define AUTOAIM_LOST    0
#define AUTOAIM_LOCKED  1
#define AUTOAIM_OFFLINE 2
void vTaskDelay(int ticksToDelay);
void vTaskDelay(int ticksToDelay) {
    // 假设 1 tick = 1 ms
    std::this_thread::sleep_for(std::chrono::milliseconds(ticksToDelay));
}
void osDelay(int time);
void osDelay(int time) {
    std::this_thread::sleep_for(std::chrono::milliseconds(time));
}

/****************************************串口驱动映射************************************/

void UI_SendByte(unsigned char ch) {
    UI_com[UI_tot++] = ch;
}

void UI_Send(Device::Base *base_) {
    base_->serial_.write(UI_com, UI_tot);
    UI_tot = 0;
}

/********************************************删除操作*************************************
**参数：Del_Operate  对应头文件删除操作
        Del_Layer    要删除的层 取值0-9
*****************************************************************************************/

void UI_Delete(Device::Base *base_, u8 Del_Operate, u8 Del_Layer) {
    unsigned char *framepoint;  // 读写指针
    u16 frametail = 0xFFFF;     // CRC16校验值
    int loop_control;           // For函数循环控制

    UI_Packhead framehead;
    UI_Data_Operate datahead;
    UI_Data_Delete del;

    framepoint = (unsigned char *)&framehead;

    framehead.SOF = UI_SOF;
    framehead.Data_Length = 8;
    framehead.Seq = UI_Seq;
    framehead.CRC8 = base_->getCRC8CheckSum(framepoint, 4, 0xFF);
    framehead.CMD_ID = UI_CMD_Robo_Exchange;  // 填充包头数据

    datahead.Data_ID = UI_Data_ID_Del;
    datahead.Sender_ID = Robot_ID_Read;
    datahead.Receiver_ID = Cilent_ID_Read;  // 填充操作数据

    del.Delete_Operate = Del_Operate;
    del.Layer = Del_Layer;  // 控制信息

    frametail = base_->getCRC16CheckSum(framepoint, sizeof(framehead), frametail);
    framepoint = (unsigned char *)&datahead;
    frametail = base_->getCRC16CheckSum(framepoint, sizeof(datahead), frametail);
    framepoint = (unsigned char *)&del;
    frametail = base_->getCRC16CheckSum(framepoint, sizeof(del), frametail);  // CRC16校验值计算

    framepoint = (unsigned char *)&framehead;
    for (loop_control = 0; loop_control < sizeof(framehead); loop_control++) {
        UI_SendByte(*framepoint);
        framepoint++;
    }
    framepoint = (unsigned char *)&datahead;
    for (loop_control = 0; loop_control < sizeof(datahead); loop_control++) {
        UI_SendByte(*framepoint);
        framepoint++;
    }
    framepoint = (unsigned char *)&del;
    for (loop_control = 0; loop_control < sizeof(del); loop_control++) {
        UI_SendByte(*framepoint);
        framepoint++;
    }  // 发送所有帧
    framepoint = (unsigned char *)&frametail;
    for (loop_control = 0; loop_control < sizeof(frametail); loop_control++) {
        UI_SendByte(*framepoint);
        framepoint++;  // 发送CRC16校验值
    }
    UI_Send(base_);

    UI_Seq++;  // 包序号+1
}
/************************************************绘制直线*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Start_x、Start_x    开始坐标
        End_x、End_y   结束坐标
**********************************************************************************************************/

void Line_Draw(
    Graph_Data *image,
    std::string imagename,
    u32 Graph_Operate,
    u32 Graph_Layer,
    u32 Graph_Color,
    u32 Graph_Width,
    u32 Start_x,
    u32 Start_y,
    u32 End_x,
    u32 End_y) {
    int i;
    for (i = 0; i < 3 && imagename[i] != '\0'; i++)
        image->graphic_name[2 - i] = imagename[i];
    image->graphic_tpye = UI_Graph_Line;
    image->operate_tpye = Graph_Operate;
    image->layer = Graph_Layer;
    image->color = Graph_Color;
    image->width = Graph_Width;
    image->start_x = Start_x;
    image->start_y = Start_y;
    image->end_x = End_x;
    image->end_y = End_y;
}

/************************************************绘制矩形*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Start_x、Start_x    开始坐标
        End_x、End_y   结束坐标（对顶角坐标）
**********************************************************************************************************/

void Rectangle_Draw(
    Graph_Data *image,
    std::string imagename,
    u32 Graph_Operate,
    u32 Graph_Layer,
    u32 Graph_Color,
    u32 Graph_Width,
    u32 Start_x,
    u32 Start_y,
    u32 End_x,
    u32 End_y) {
    int i;
    for (i = 0; i < 3 && imagename[i] != '\0'; i++)
        image->graphic_name[2 - i] = imagename[i];
    image->graphic_tpye = UI_Graph_Rectangle;
    image->operate_tpye = Graph_Operate;
    image->layer = Graph_Layer;
    image->color = Graph_Color;
    image->width = Graph_Width;
    image->start_x = Start_x;
    image->start_y = Start_y;
    image->end_x = End_x;
    image->end_y = End_y;
}

/************************************************绘制整圆*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Start_x、Start_x    圆心坐标
        Graph_Radius  图形半径
**********************************************************************************************************/

void Circle_Draw(
    Graph_Data *image,
    std::string imagename,
    u32 Graph_Operate,
    u32 Graph_Layer,
    u32 Graph_Color,
    u32 Graph_Width,
    u32 Start_x,
    u32 Start_y,
    u32 Graph_Radius) {
    int i;
    for (i = 0; i < 3 && imagename[i] != '\0'; i++)
        image->graphic_name[2 - i] = imagename[i];
    image->graphic_tpye = UI_Graph_Circle;
    image->operate_tpye = Graph_Operate;
    image->layer = Graph_Layer;
    image->color = Graph_Color;
    image->width = Graph_Width;
    image->start_x = Start_x;
    image->start_y = Start_y;
    image->radius = Graph_Radius;
}

/************************************************绘制圆弧*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Graph_StartAngle,Graph_EndAngle    开始，终止角度
        Start_y,Start_y    圆心坐标
        x_Length,y_Length   x,y方向上轴长，参考椭圆
**********************************************************************************************************/

void Arc_Draw(
    Graph_Data *image,
    std::string imagename,
    u32 Graph_Operate,
    u32 Graph_Layer,
    u32 Graph_Color,
    u32 Graph_StartAngle,
    u32 Graph_EndAngle,
    u32 Graph_Width,
    u32 Start_x,
    u32 Start_y,
    u32 x_Length,
    u32 y_Length) {
    int i;

    for (i = 0; i < 3 && imagename[i] != '\0'; i++)
        image->graphic_name[2 - i] = imagename[i];
    image->graphic_tpye = UI_Graph_Arc;
    image->operate_tpye = Graph_Operate;
    image->layer = Graph_Layer;
    image->color = Graph_Color;
    image->width = Graph_Width;
    image->start_x = Start_x;
    image->start_y = Start_y;
    image->start_angle = Graph_StartAngle;
    image->end_angle = Graph_EndAngle;
    image->end_x = x_Length;
    image->end_y = y_Length;
}

/************************************************绘制浮点型数据*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Graph_Size     字号
        Graph_Digit    小数位数
        Start_x、Start_x    开始坐标
        Graph_Float   要显示的变量
**********************************************************************************************************/

void Float_Draw(
    Float_Data *image,
    std::string imagename,
    u32 Graph_Operate,
    u32 Graph_Layer,
    u32 Graph_Color,
    u32 Graph_Size,
    u32 Graph_Digit,
    u32 Graph_Width,
    u32 Start_x,
    u32 Start_y,
    float Graph_Float) {
    int i;

    for (i = 0; i < 3 && imagename[i] != '\0'; i++)
        image->graphic_name[2 - i] = imagename[i];
    image->graphic_tpye = UI_Graph_Float;
    image->operate_tpye = Graph_Operate;
    image->layer = Graph_Layer;
    image->color = Graph_Color;
    image->width = Graph_Width;
    image->start_x = Start_x;
    image->start_y = Start_y;
    image->start_angle = Graph_Size;
    image->end_angle = Graph_Digit;
    image->graph_Float = Graph_Float;
}

/************************************************绘制字符型数据*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Graph_Size     字号
        Graph_Digit    字符个数
        Start_x、Start_x    开始坐标
        *String_Data          待发送字符串开始地址
**********************************************************************************************************/

void String_Draw(
    String_Data *image,
    std::string imagename,
    u32 Graph_Operate,
    u32 Graph_Layer,
    u32 Graph_Color,
    u32 Graph_Size,
    u32 Graph_Digit,
    u32 Graph_Width,
    u32 Start_x,
    u32 Start_y,
    std::string String_Data) {
    int i;

    for (i = 0; i < 3 && imagename[i] != '\0'; i++)
        image->Graph_Control.graphic_name[2 - i] = imagename[i];
    image->Graph_Control.graphic_tpye = UI_Graph_Char;
    image->Graph_Control.operate_tpye = Graph_Operate;
    image->Graph_Control.layer = Graph_Layer;
    image->Graph_Control.color = Graph_Color;
    image->Graph_Control.width = Graph_Width;
    image->Graph_Control.start_x = Start_x;
    image->Graph_Control.start_y = Start_y;
    image->Graph_Control.start_angle = Graph_Size;
    image->Graph_Control.end_angle = Graph_Digit;

    for (i = 0; i < Graph_Digit; i++) {
        if (i < String_Data.length()) {
            image->show_Data[i] = String_Data[i];
        } else {
            break;
        }
    }
}

/************************************************UI推送函数（使更改生效）*********************************
**参数： cnt   图形个数
         ...   图形变量参数


Tips：：该函数只能推送1，2，5，7个图形，其他数目协议未涉及
**********************************************************************************************************/
int UI_ReFresh(Device::Base *base_, int cnt, ...) {
    int i, n;
    Graph_Data imageData;
    unsigned char *framepoint;  // 读写指针
    u16 frametail = 0xFFFF;     // CRC16校验值

    UI_Packhead framehead;
    UI_Data_Operate datahead;

    va_list ap;
    va_start(ap, cnt);

    framepoint = (unsigned char *)&framehead;
    framehead.SOF = UI_SOF;
    framehead.Data_Length = 6 + cnt * 15;
    framehead.Seq = UI_Seq;
    framehead.CRC8 = base_->getCRC8CheckSum(framepoint, 4, 0xFF);
    framehead.CMD_ID = UI_CMD_Robo_Exchange;  // 填充包头数据

    switch (cnt) {
        case 1: datahead.Data_ID = UI_Data_ID_Draw1; break;
        case 2: datahead.Data_ID = UI_Data_ID_Draw2; break;
        case 5: datahead.Data_ID = UI_Data_ID_Draw5; break;
        case 7: datahead.Data_ID = UI_Data_ID_Draw7; break;
        default: return (-1);
    }
    datahead.Sender_ID = Robot_ID_Read;
    datahead.Receiver_ID = Cilent_ID_Read;  // 填充操作数据

    framepoint = (unsigned char *)&framehead;
    frametail = base_->getCRC16CheckSum(framepoint, sizeof(framehead), frametail);
    framepoint = (unsigned char *)&datahead;
    frametail = base_->getCRC16CheckSum(
        framepoint, sizeof(datahead), frametail);  // CRC16校验值计算（部分）

    framepoint = (unsigned char *)&framehead;
    for (i = 0; i < sizeof(framehead); i++) {
        UI_SendByte(*framepoint);
        framepoint++;
    }
    framepoint = (unsigned char *)&datahead;
    for (i = 0; i < sizeof(datahead); i++) {
        UI_SendByte(*framepoint);
        framepoint++;
    }

    for (i = 0; i < cnt; i++) {
        imageData = va_arg(ap, Graph_Data);

        framepoint = (unsigned char *)&imageData;
        frametail = base_->getCRC16CheckSum(framepoint, sizeof(imageData), frametail);  // CRC16校验

        for (n = 0; n < sizeof(imageData); n++) {
            UI_SendByte(*framepoint);
            framepoint++;
        }  // 发送图片帧
    }
    framepoint = (unsigned char *)&frametail;
    for (i = 0; i < sizeof(frametail); i++) {
        UI_SendByte(*framepoint);
        framepoint++;  // 发送CRC16校验值
    }

    UI_Send(base_);

    va_end(ap);

    UI_Seq++;  // 包序号+1
    return 0;
}

/************************************************UI推送字符（使更改生效）*********************************
**参数： cnt   图形个数
         ...   图形变量参数


Tips：：该函数只能推送1，2，5，7个图形，其他数目协议未涉及
**********************************************************************************************************/
int String_ReFresh(Device::Base *base_, String_Data string_Data) {
    int i;
    String_Data imageData;
    unsigned char *framepoint;  // 读写指针
    u16 frametail = 0xFFFF;     // CRC16校验值

    UI_Packhead framehead;
    UI_Data_Operate datahead;
    imageData = string_Data;

    framepoint = (unsigned char *)&framehead;
    framehead.SOF = UI_SOF;
    framehead.Data_Length = 6 + 45;
    framehead.Seq = UI_Seq;
    framehead.CRC8 = base_->getCRC8CheckSum(framepoint, 4, 0xFF);
    framehead.CMD_ID = UI_CMD_Robo_Exchange;  // 填充包头数据

    datahead.Data_ID = UI_Data_ID_DrawString;

    datahead.Sender_ID = Robot_ID_Read;
    datahead.Receiver_ID = Cilent_ID_Read;  // 填充操作数据

    framepoint = (unsigned char *)&framehead;
    frametail = base_->getCRC16CheckSum(framepoint, sizeof(framehead), frametail);
    framepoint = (unsigned char *)&datahead;
    frametail = base_->getCRC16CheckSum(framepoint, sizeof(datahead), frametail);
    framepoint = (unsigned char *)&imageData;
    frametail = base_->getCRC16CheckSum(
        framepoint, sizeof(imageData), frametail);  // CRC16校验   //CRC16校验值计算（部分）

    framepoint = (unsigned char *)&framehead;
    for (i = 0; i < sizeof(framehead); i++) {
        UI_SendByte(*framepoint);
        framepoint++;
    }
    framepoint = (unsigned char *)&datahead;
    for (i = 0; i < sizeof(datahead); i++) {
        UI_SendByte(*framepoint);
        framepoint++;
    }  // 发送操作数据
    framepoint = (unsigned char *)&imageData;
    for (i = 0; i < sizeof(imageData); i++) {
        UI_SendByte(*framepoint);
        framepoint++;
    }  // 发送图片帧

    framepoint = (unsigned char *)&frametail;
    for (i = 0; i < sizeof(frametail); i++) {
        UI_SendByte(*framepoint);
        framepoint++;  // 发送CRC16校验值
    }

    UI_Send(base_);

    UI_Seq++;  // 包序号+1
    return 0;
}

/*****************************************************CRC8校验值计算**********************************************/
const unsigned char CRC8_INIT_UI = 0xff;
const unsigned char CRC8_TAB_UI[256] = {
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};

uint16_t CRC_INIT_UI = 0xffff;
const uint16_t wCRC_Table_UI[256] = {
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 0x8c48, 0x9dc1, 0xaf5a, 0xbed3,
    0xca6c, 0xdbe5, 0xe97e, 0xf8f7, 0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876, 0x2102, 0x308b, 0x0210, 0x1399,
    0x6726, 0x76af, 0x4434, 0x55bd, 0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c, 0xbdcb, 0xac42, 0x9ed9, 0x8f50,
    0xfbef, 0xea66, 0xd8fd, 0xc974, 0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3, 0x5285, 0x430c, 0x7197, 0x601e,
    0x14a1, 0x0528, 0x37b3, 0x263a, 0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 0xef4e, 0xfec7, 0xcc5c, 0xddd5,
    0xa96a, 0xb8e3, 0x8a78, 0x9bf1, 0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70, 0x8408, 0x9581, 0xa71a, 0xb693,
    0xc22c, 0xd3a5, 0xe13e, 0xf0b7, 0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 0x18c1, 0x0948, 0x3bd3, 0x2a5a,
    0x5ee5, 0x4f6c, 0x7df7, 0x6c7e, 0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd, 0xb58b, 0xa402, 0x9699, 0x8710,
    0xf3af, 0xe226, 0xd0bd, 0xc134, 0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, 0x4a44, 0x5bcd, 0x6956, 0x78df,
    0x0c60, 0x1de9, 0x2f72, 0x3efb, 0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a, 0xe70e, 0xf687, 0xc41c, 0xd595,
    0xa12a, 0xb0a3, 0x8238, 0x93b1, 0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 0x7bc7, 0x6a4e, 0x58d5, 0x495c,
    0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};
/*
** Descriptions: CRC16 checksum function
** Input: Data to check,Stream length, initialized checksum
** Output: CRC checksum
*/

void UI_init(UI_control_t *init) {
    // init->UI_rc_ctrl = get_remote_control_point();
    // init->UI_rc_ctrl = &rc_control;  // 待修改
    init->UI_open = 0;
    init->UI_mode = 0;
    init->fric_open = 0;
    init->fric_mode = 0;
    init->spin_open = 0;
    init->spin_mode = 0;
    memset(&init->fric_name, 0, sizeof(String_Data));
    memset(&init->spin_name, 0, sizeof(String_Data));
    memset(&init->fric_graph, 0, sizeof(Graph_Data));
    memset(&init->spin_graph, 0, sizeof(Graph_Data));
    memset(&init->accuracy_graph, 0, sizeof(Graph_Data));
    // Line_Draw(&init->accuracy_graph, "acc", UI_Graph_ADD, 9, UI_Color_Green, 5, 50, 50, 500,
    // 500);
}

void UI_open(Device::Base *base_, UI_control_t *UI) {
    UI->UI_mode = 1;
    String_Draw(
        &UI->fric_name,
        "frs",
        UI_Graph_ADD,
        9,
        UI_Color_Cyan,
        10,
        6,
        3,
        FRIC_NAME_START_X,
        FRIC_NAME_START_Y,
        "FRIC");
    String_Draw(
        &UI->spin_name,
        "sps",
        UI_Graph_ADD,
        9,
        UI_Color_Cyan,
        10,
        6,
        3,
        SPIN_NAME_START_X,
        SPIN_NAME_START_Y,
        "SPIN");
    Line_Draw(
        &UI->fric_graph,
        "frg",
        UI_Graph_ADD,
        9,
        UI->fric_open ? UI_Color_Green : UI_Color_Main,
        5,
        FRIC_NAME_START_X,
        FRIC_NAME_START_Y,
        FRIC_NAME_START_X + 20,
        FRIC_NAME_START_Y);
    Line_Draw(
        &UI->spin_graph,
        "sps",
        UI_Graph_ADD,
        9,
        UI->spin_open ? UI_Color_Green : UI_Color_Main,
        5,
        SPIN_NAME_START_X,
        SPIN_NAME_START_Y,
        SPIN_NAME_START_X + 20,
        SPIN_NAME_START_Y);
    Circle_Draw(&UI->accuracy_graph, "acc", UI_Graph_ADD, 9, UI_Color_Yellow, 10, 700, 500, 20);
    UI_ReFresh(base_, 2, UI->fric_graph, UI->spin_graph);
    UI_ReFresh(base_, 1, UI->accuracy_graph);
    String_ReFresh(base_, UI->fric_name);
    String_ReFresh(base_, UI->spin_name);
}

void UI_close(Device::Base *base_, UI_control_t *UI) {
    UI->UI_mode = 0;
    UI_Delete(base_, UI_Data_Del_ALL, 9);
}

void UI_set_fric(uint8_t mode) {
    UI_control.fric_open = true;
}

void UI_set_spin(uint8_t mode) {
    UI_control.spin_mode = true;
}

void UI_set_mode(UI_control_t *UI) {
    uint16_t last_key = 0;
    UI->UI_open = 1;
    // if ((UI->UI_rc_ctrl->key.v & UI_MODE_SWITCH_KEYBOARD) && !last_key) {
    //     UI->UI_open ^= 1;
    // }
    // last_key = (UI->UI_rc_ctrl->key.v & UI_MODE_SWITCH_KEYBOARD);
}

void UI_set_control(Device::Base *base_, UI_control_t *UI) {
    UI_open(base_, &UI_control);
    Line_Draw(
        &UI_control.fric_graph,
        "frg",
        UI_Graph_ADD,
        9,
        UI_control.fric_open ? UI_Color_Green : UI_Color_Cyan,
        5,
        FRIC_NAME_START_X,
        FRIC_NAME_START_Y,
        FRIC_NAME_START_X + 20,
        FRIC_NAME_START_Y);
    UI_control.fric_mode = UI_control.fric_open;

    Line_Draw(
        &UI_control.spin_graph,
        "sps",
        UI_Graph_ADD,
        9,
        UI_control.spin_open ? UI_Color_Green : UI_Color_Cyan,
        5,
        SPIN_NAME_START_X,
        SPIN_NAME_START_Y,
        SPIN_NAME_START_X + 20,
        SPIN_NAME_START_Y);
    UI_control.spin_mode = UI_control.spin_open;
    // if (UI_control.UI_mode != UI_control.UI_open) {
    //     if (UI_control.UI_open) {
    //     } else {
    UI_close(base_, &UI_control);
    //     }
    // }
    if (UI_control.fric_mode != UI_control.fric_open) {
    }
    if (UI_control.spin_mode != UI_control.spin_open) {
    }
}

void UI_task(Device::Base *base_) {
    UI_init(&UI_control);
    while (1) {
        // LOG_INFO("draw ui\n");
        UI_set_mode(&UI_control);
        UI_set_control(base_, &UI_control);
        vTaskDelay(10);
    }
}

/*custom_ui*/

Crosshair_Data_Type Crosshair_Data;
volatile UI_DisplayData_Type UI_Data;
State_Indicate_Type State_Data;
String_Data state_text_data;
String_Data fired_text_data;
String_Data warning_text_data;
String_Data warning_q_text_data;
Graph_Data shoot_distance_bar, cap_percentage, cap_full_frame, auto_aim_range;
Graph_Data still_cross_line[10];
char cap_text[30], auto_aim_text[10], fired_text[30], warning_text[16];
int count = 0;  // 计数器

namespace
{
    constexpr u32 kCapBarYOffset = 18;
    constexpr u32 kCapFrameWidth = 2;
    constexpr u32 kWarningTextSize = 30;
    constexpr u32 kWarningTextChars = 8;
    constexpr u32 kWarningTextY = 330;
    constexpr u32 kWarningQTextSize = 30;
    constexpr u32 kWarningQTextChars = 1;
    constexpr u32 kWarningQTextY = 370;
    constexpr int kUiPacketGapMs = 30;
    constexpr int kUiUpdateIntervalMs = 200;
    constexpr int kUiRetryIntervalMs = 100;
    constexpr int kUiFullRedrawIntervalMs = 10000;

    bool resolve_client_id(uint16_t robot_id, uint16_t *client_id) {
        if (client_id == nullptr) {
            return false;
        }

        switch (robot_id) {
            case UI_Data_RobotID_RHero:
                *client_id = UI_Data_CilentID_RHero;
                return true;
            case UI_Data_RobotID_REngineer:
                *client_id = UI_Data_CilentID_REngineer;
                return true;
            case UI_Data_RobotID_RStandard1:
                *client_id = UI_Data_CilentID_RStandard1;
                return true;
            case UI_Data_RobotID_RStandard2:
                *client_id = UI_Data_CilentID_RStandard2;
                return true;
            case UI_Data_RobotID_RStandard3:
                *client_id = UI_Data_CilentID_RStandard3;
                return true;
            case UI_Data_RobotID_RAerial:
                *client_id = UI_Data_CilentID_RAerial;
                return true;
            case UI_Data_RobotID_BHero:
                *client_id = UI_Data_CilentID_BHero;
                return true;
            case UI_Data_RobotID_BEngineer:
                *client_id = UI_Data_CilentID_BEngineer;
                return true;
            case UI_Data_RobotID_BStandard1:
                *client_id = UI_Data_CilentID_BStandard1;
                return true;
            case UI_Data_RobotID_BStandard2:
                *client_id = UI_Data_CilentID_BStandard2;
                return true;
            case UI_Data_RobotID_BStandard3:
                *client_id = UI_Data_CilentID_BStandard3;
                return true;
            case UI_Data_RobotID_BAerial:
                *client_id = UI_Data_CilentID_BAerial;
                return true;
            default:
                return false;
        }
    }

    bool update_ui_target_id(uint16_t robot_id) {
        uint16_t client_id = 0;
        if (!resolve_client_id(robot_id, &client_id)) {
            return false;
        }

        Robot_ID_Read = robot_id;
        Cilent_ID_Read = client_id;
        return true;
    }

    void draw_cap_energy_graph(
        Graph_Data *cap_bar,
        Graph_Data *cap_frame,
        u32 cap_bar_operate,
        u32 cap_frame_operate) {
        const u32 cap_bar_start_x = State_Data.cap_text_pos[0];
        const u32 cap_bar_center_y =
            State_Data.cap_text_pos[1] - State_Data.cap_text_size * 6 + kCapBarYOffset;
        const u32 cap_bar_end_x =
            cap_bar_start_x + ((State_Data.cap_display_length * State_Data.cap_percent) / 100);
        const u32 cap_frame_half_height = State_Data.cap_display_with / 2;

        Line_Draw(
            cap_bar,
            "cap",
            cap_bar_operate,
            1,
            State_Data.cap_bar_color,
            State_Data.cap_display_with,
            cap_bar_start_x,
            cap_bar_center_y,
            cap_bar_end_x,
            cap_bar_center_y);

        Rectangle_Draw(
            cap_frame,
            "cfr",
            cap_frame_operate,
            1,
            UI_Color_White,
            kCapFrameWidth,
            cap_bar_start_x,
            cap_bar_center_y - cap_frame_half_height,
            cap_bar_start_x + State_Data.cap_display_length,
            cap_bar_center_y + cap_frame_half_height);
    }

    void draw_auto_aim_graph(Graph_Data *auto_aim_graph, u32 operate_type) {
        u32 color = UI_Color_Cyan;
        if (UI_Data.auto_aim_state == AUTOAIM_LOST) {
            color = UI_Color_Green;
        } else if (UI_Data.auto_aim_state == AUTOAIM_LOCKED) {
            color = UI_Color_Purplish_red;
        } else if (UI_Data.auto_aim_state == AUTOAIM_OFFLINE) {
            color = UI_Color_Black;
        }

        Rectangle_Draw(
            auto_aim_graph,
            "aui",
            operate_type,
            0,
            color,
            3,
            Crosshair_Data.center[0] - (Crosshair_Data.cross_width + 50 / 2),
            300,
            Crosshair_Data.center[0] + (Crosshair_Data.cross_width + 50 / 2),
            800);
    }

    void build_state_text(u32 operate_type) {
        state_str(cap_text, State_Data.cap_percent, State_Data.spin_state, State_Data.fric_state);
        cap_text[21] = '\0';
        String_Draw(
            &state_text_data,
            "sta",
            operate_type,
            1,
            State_Data.cap_text_color,
            State_Data.cap_text_size,
            21,
            2,
            State_Data.cap_text_pos[0],
            State_Data.cap_text_pos[1],
            cap_text);
    }

    void build_fired_text(u32 operate_type) {
        std::snprintf(fired_text, sizeof(fired_text), "SHOT:%05d", UI_Data.fired_bullet_num);
        String_Draw(
            &fired_text_data,
            "sht",
            operate_type,
            1,
            UI_Color_Yellow,
            25,
            10,
            2,
            120,
            800,
            fired_text);
    }

    void build_warning_text(u32 operate_type) {
        std::snprintf(warning_text, sizeof(warning_text), "WARNING!");
        const u32 text_color = UI_Data.low_ammo_warning ? UI_Color_Purplish_red : UI_Color_Black;
        const u32 start_x =
            Crosshair_Data.center[0] - (kWarningTextSize * kWarningTextChars) / 4;
        String_Draw(
            &warning_text_data,
            "war",
            operate_type,
            1,
            text_color,
            kWarningTextSize,
            kWarningTextChars,
            3,
            start_x,
            kWarningTextY,
            warning_text);
    }

    void draw_warning_q_text(Device::Base *base_) {
        const u32 start_x =
            Crosshair_Data.center[0] - (kWarningQTextSize * kWarningQTextChars) / 4;
        String_Draw(
            &warning_q_text_data,
            "wrq",
            UI_Graph_ADD,
            1,
            UI_Color_Purplish_red,
            kWarningQTextSize,
            kWarningQTextChars,
            3,
            start_x,
            kWarningQTextY,
            "Q");
        String_ReFresh(base_, warning_q_text_data);
    }

    void draw_static_ui(Device::Base *base_) {
        if (UI_MODE == UI_HERO) {
            draw_crosshair_hero(base_);
        } else if (UI_MODE == UI_INFANTRY) {
            draw_crosshair_infantry(base_);
        }
        osDelay(kUiPacketGapMs);
        draw_warning_q_text(base_);
    }

    void refresh_dynamic_ui_full(Device::Base *base_, u32 graph_operate, u32 string_operate) {
        sync_parameter();

        draw_auto_aim_graph(&auto_aim_range, graph_operate);
        UI_ReFresh(base_, 1, auto_aim_range);
        osDelay(kUiPacketGapMs);

        draw_cap_energy_graph(&cap_percentage, &cap_full_frame, graph_operate, graph_operate);
        UI_ReFresh(base_, 2, cap_percentage, cap_full_frame);
        osDelay(kUiPacketGapMs);

        build_state_text(string_operate);
        String_ReFresh(base_, state_text_data);
        osDelay(kUiPacketGapMs);

        build_fired_text(string_operate);
        String_ReFresh(base_, fired_text_data);
        osDelay(kUiPacketGapMs);

        build_warning_text(string_operate);
        String_ReFresh(base_, warning_text_data);
        osDelay(kUiPacketGapMs);
    }

    void refresh_dynamic_ui_incremental(Device::Base *base_) {
        static int refresh_phase = 0;

        sync_parameter();

        draw_auto_aim_graph(&auto_aim_range, UI_Graph_Change);
        UI_ReFresh(base_, 1, auto_aim_range);
        osDelay(kUiPacketGapMs);

        switch (refresh_phase) {
            case 0:
                draw_cap_energy_graph(&cap_percentage, &cap_full_frame, UI_Graph_Change, UI_Graph_Change);
                UI_ReFresh(base_, 2, cap_percentage, cap_full_frame);
                break;
            case 1:
                build_state_text(UI_Graph_Change);
                String_ReFresh(base_, state_text_data);
                break;
            case 2:
                build_fired_text(UI_Graph_Change);
                String_ReFresh(base_, fired_text_data);
                break;
            default:
                build_warning_text(UI_Graph_Change);
                String_ReFresh(base_, warning_text_data);
                break;
        }
        osDelay(kUiPacketGapMs);

        refresh_phase = (refresh_phase + 1) % 4;
    }

    void full_redraw_ui(Device::Base *base_) {
        UI_clear(base_);
        ui_parameter_init();
        draw_static_ui(base_);
        osDelay(kUiPacketGapMs);
        refresh_dynamic_ui_full(base_, UI_Graph_ADD, UI_Graph_ADD);
    }
}  // namespace

void custom_ui_task(Device::Base *base_, uint8_t &robot_id_) {
    uint16_t last_robot_id = 0;
    auto last_full_redraw_time =
        std::chrono::steady_clock::now() - std::chrono::milliseconds(kUiFullRedrawIntervalMs);

    while (1) {
        const uint16_t current_robot_id = robot_id_;
        if (!base_->referee_data_is_online_ || !update_ui_target_id(current_robot_id)) {
            last_robot_id = 0;
            osDelay(kUiRetryIntervalMs);
            continue;
        }

        const auto now = std::chrono::steady_clock::now();
        const bool robot_id_changed = current_robot_id != last_robot_id;
        const bool redraw_due =
            now - last_full_redraw_time >= std::chrono::milliseconds(kUiFullRedrawIntervalMs);

        if (robot_id_changed || redraw_due) {
            full_redraw_ui(base_);
            last_full_redraw_time = std::chrono::steady_clock::now();
        } else {
            refresh_dynamic_ui_incremental(base_);
        }

        last_robot_id = current_robot_id;
        count++;
        osDelay(kUiUpdateIntervalMs);
    }
}

void update_ui_data(
    Device::Base *base_,
    bool fric_state,
    bool auto_aim_state,
    bool spin_state,
    float cap_state,
    uint32_t purchased_bullet_num,
    uint32_t remain_bullet_num,
    bool low_ammo_warning) {
    UI_Data.distance = 10;
    UI_Data.auto_aim_state = auto_aim_state ? AUTOAIM_LOCKED : AUTOAIM_LOST;
    UI_Data.fric_state = fric_state;
    UI_Data.shoot_speed = 10;
    UI_Data.spin_state = spin_state;
    UI_Data.Super_cap_percent = cap_state;
    const uint32_t fired_bullet_num =
        purchased_bullet_num >= remain_bullet_num ? purchased_bullet_num - remain_bullet_num : 0U;
    UI_Data.purchased_bullet_num = purchased_bullet_num > static_cast<uint32_t>(INT_MAX)
                                       ? INT_MAX
                                       : static_cast<int>(purchased_bullet_num);
    UI_Data.remain_bullet_num = remain_bullet_num > static_cast<uint32_t>(INT_MAX)
                                    ? INT_MAX
                                    : static_cast<int>(remain_bullet_num);
    UI_Data.fired_bullet_num = fired_bullet_num > static_cast<uint32_t>(INT_MAX)
                                   ? INT_MAX
                                   : static_cast<int>(fired_bullet_num);
    UI_Data.low_ammo_warning = low_ammo_warning ? 1 : 0;
}

/*画英雄的静止准星*/
void draw_crosshair_hero(Device::Base *base_) {
    char line_id[7][3] = { "L1", "L2", "L3", "L4", "L5", "L6", "L7" };
    const u32 ref_line_len_1 = (Crosshair_Data.cross_width * 4) / 5;
    const u32 ref_line_len_2 = (Crosshair_Data.cross_width * 3) / 5;
    const u32 ref_line_len_3 = (Crosshair_Data.cross_width * 2) / 5;
    const u32 ref_line_spacing = 35;
    const u32 ref_line_y_base = Crosshair_Data.center[1];
    /*准星*/
    // 横着的主准星
    Line_Draw(
        &still_cross_line[0],
        "L1",
        UI_Graph_ADD,
        0,
        Crosshair_Data.cross_colar,
        Crosshair_Data.line_width,
        Crosshair_Data.center[0] - (Crosshair_Data.cross_width / 2),
        Crosshair_Data.center[1],
        Crosshair_Data.center[0] + (Crosshair_Data.cross_width / 2),
        Crosshair_Data.center[1]);
    // 竖着的主准星
    Line_Draw(
        &still_cross_line[1],
        "L2",
        UI_Graph_ADD,
        0,
        Crosshair_Data.cross_colar,
        Crosshair_Data.line_width,
        Crosshair_Data.center[0],
        Crosshair_Data.center[1] + (Crosshair_Data.cross_high / 2) +
            Crosshair_Data.cross_high_offset,
        Crosshair_Data.center[0],
        Crosshair_Data.center[1] - (Crosshair_Data.cross_high / 2) +
            Crosshair_Data.cross_high_offset);
    // 5个标尺
    for (int i = 0; i < 5; i++) {
        Line_Draw(
            &still_cross_line[i + 2],
            line_id[i + 2],
            UI_Graph_ADD,
            0,
            Crosshair_Data.ruler_colar,
            Crosshair_Data.line_width,
            Crosshair_Data.center[0] - (Crosshair_Data.ruler_length[i] / 2),
            Crosshair_Data.center[1] - (Crosshair_Data.ballistic_ruler[i] / 2),
            Crosshair_Data.center[0] + (Crosshair_Data.ruler_length[i] / 2),
            Crosshair_Data.center[1] - (Crosshair_Data.ballistic_ruler[i] / 2));
    }
    Line_Draw(
        &still_cross_line[7],
        "B1",
        UI_Graph_ADD,
        0,
        Crosshair_Data.ruler_colar,
        Crosshair_Data.line_width,
        Crosshair_Data.center[0] - (ref_line_len_1 / 2),
        ref_line_y_base,
        Crosshair_Data.center[0] + (ref_line_len_1 / 2),
        ref_line_y_base);
    Line_Draw(
        &still_cross_line[8],
        "B2",
        UI_Graph_ADD,
        0,
        Crosshair_Data.ruler_colar,
        Crosshair_Data.line_width,
        Crosshair_Data.center[0] - (ref_line_len_2 / 2),
        ref_line_y_base - ref_line_spacing,
        Crosshair_Data.center[0] + (ref_line_len_2 / 2),
        ref_line_y_base - ref_line_spacing);
    Line_Draw(
        &still_cross_line[9],
        "B3",
        UI_Graph_ADD,
        0,
        Crosshair_Data.ruler_colar,
        Crosshair_Data.line_width,
        Crosshair_Data.center[0] - (ref_line_len_3 / 2),
        ref_line_y_base - ref_line_spacing * 2,
        Crosshair_Data.center[0] + (ref_line_len_3 / 2),
        ref_line_y_base - ref_line_spacing * 2);
    UI_ReFresh(
        base_,
        7,
        still_cross_line[0],
        still_cross_line[1],
        still_cross_line[2],
        still_cross_line[3],
        still_cross_line[4],
        still_cross_line[5],
        still_cross_line[6]);
    UI_ReFresh(base_, 2, still_cross_line[7], still_cross_line[8]);
    UI_ReFresh(base_, 1, still_cross_line[9]);

    osDelay(100);  // 确保间隔
    // 标尺
}

/*画步兵的静止准星*/
void draw_crosshair_infantry(Device::Base *base_) {
    const u32 ref_line_len_1 = (Crosshair_Data.cross_width * 4) / 5;
    const u32 ref_line_len_2 = (Crosshair_Data.cross_width * 3) / 5;
    const u32 ref_line_len_3 = (Crosshair_Data.cross_width * 2) / 5;
    const u32 ref_line_spacing = 35;
    const u32 ref_line_y_base = Crosshair_Data.center[1];
    /*准星*/
    // 横着的主准星
    Line_Draw(
        &still_cross_line[0],
        "L1",
        UI_Graph_ADD,
        0,
        Crosshair_Data.cross_colar,
        Crosshair_Data.line_width,
        Crosshair_Data.center[0] - (Crosshair_Data.cross_width / 2),
        Crosshair_Data.center[1],
        Crosshair_Data.center[0] + (Crosshair_Data.cross_width / 2),
        Crosshair_Data.center[1]);
    // 竖着的主准星
    Line_Draw(
        &still_cross_line[1],
        "L2",
        UI_Graph_ADD,
        0,
        Crosshair_Data.cross_colar,
        Crosshair_Data.line_width,
        Crosshair_Data.center[0],
        Crosshair_Data.center[1] + (Crosshair_Data.cross_high / 2) +
            Crosshair_Data.cross_high_offset,
        Crosshair_Data.center[0],
        Crosshair_Data.center[1] - (Crosshair_Data.cross_high / 2) +
            Crosshair_Data.cross_high_offset);
    Line_Draw(
        &still_cross_line[7],
        "B1",
        UI_Graph_ADD,
        0,
        Crosshair_Data.ruler_colar,
        Crosshair_Data.line_width,
        Crosshair_Data.center[0] - (ref_line_len_1 / 2),
        ref_line_y_base,
        Crosshair_Data.center[0] + (ref_line_len_1 / 2),
        ref_line_y_base);
    Line_Draw(
        &still_cross_line[8],
        "B2",
        UI_Graph_ADD,
        0,
        Crosshair_Data.ruler_colar,
        Crosshair_Data.line_width,
        Crosshair_Data.center[0] - (ref_line_len_2 / 2),
        ref_line_y_base - ref_line_spacing,
        Crosshair_Data.center[0] + (ref_line_len_2 / 2),
        ref_line_y_base - ref_line_spacing);
    Line_Draw(
        &still_cross_line[9],
        "B3",
        UI_Graph_ADD,
        0,
        Crosshair_Data.ruler_colar,
        Crosshair_Data.line_width,
        Crosshair_Data.center[0] - (ref_line_len_3 / 2),
        ref_line_y_base - ref_line_spacing * 2,
        Crosshair_Data.center[0] + (ref_line_len_3 / 2),
        ref_line_y_base - ref_line_spacing * 2);
    UI_ReFresh(
        base_,
        5,
        still_cross_line[0],
        still_cross_line[1],
        still_cross_line[7],
        still_cross_line[8],
        still_cross_line[9]);

    osDelay(100);  // 确保间隔
    // 标尺
}

void UI_init_draw(Device::Base *base_) {
}

/*刷新动态参数*/
void update_dynamic_paramater(Device::Base *base_) {
    refresh_dynamic_ui_incremental(base_);
}

/*刷新动态参数*/
// void update_dynamic_paramater()
// {
//     Graph_Data shoot_distance_bar, cap_percentage;
//     String_Data state_text_data;
//     char state_text[30];

//     //测距部分的刷新
//     Line_Draw(&shoot_distance_bar, "dst", UI_Graph_Change, 1, Crosshair_Data.shoot_bar_color,
//         Crosshair_Data.dist_display_width,
//         Crosshair_Data.center[0] + Crosshair_Data.dist_start_point[0],
//         Crosshair_Data.center[1] + Crosshair_Data.dist_start_point[1] -
//         Crosshair_Data.dist_display_width, Crosshair_Data.center[0] +
//         Crosshair_Data.dist_start_point[0] +
//         ((Crosshair_Data.dist_display_length*Crosshair_Data.shoot_dist_percent)/100),
//         Crosshair_Data.center[1] + Crosshair_Data.dist_start_point[1] -
//         Crosshair_Data.dist_display_width);

//     /*刷新超电部分*/
//     Line_Draw(&cap_percentage, "cap", UI_Graph_Change, 1, State_Data.cap_bar_color,
//     State_Data.cap_display_with,
//                 State_Data.cap_text_pos[0],
//                 State_Data.cap_text_pos[1] - State_Data.cap_text_size,
//                 State_Data.cap_text_pos[0] +
//                 ((State_Data.cap_display_length*State_Data.cap_percent)/100),
//                 State_Data.cap_text_pos[1] - State_Data.cap_text_size);
//     UI_ReFresh(2, shoot_distance_bar, cap_percentage);
//     state_str(&state_text, State_Data.cap_percent, State_Data.spin_state, State_Data.fric_state);
//     // osDelay(110);
//     // String_ReFresh(state_text_data);
//  }

void UI_clear(Device::Base *base_) {
    UI_Delete(base_, UI_Data_Del_ALL, 0);
    osDelay(100);
}

void custom_UI_init(Device::Base *base_) {
    UI_clear(base_);
    ui_parameter_init();
    draw_static_ui(base_);
}

void int_to_str(char *to_str, int number) {
    to_str[0] = ((number / 100) % 10) + 48;
    to_str[1] = ((number / 10) % 10) + 48;
    to_str[2] = (number % 10) + 48;
    to_str[3] = '\0';
}

// 生成超电百分比文字
void cap_text_format(char *to_str, int cap_percent) {
    to_str[0] = 'C';
    to_str[1] = 'A';
    to_str[2] = 'P';
    to_str[3] = ':';
    to_str[4] = ' ';
    to_str[5] = ((cap_percent / 100) % 10) + 48;
    to_str[6] = ((cap_percent / 10) % 10) + 48;
    to_str[7] = (cap_percent % 10) + 48;
    to_str[8] = '%';
    to_str[9] = '\0';
}

// 生成自旋字符串
void spin_state_str(char *to_str, int spin_state) {
    to_str[0] = 'S';
    to_str[1] = 'P';
    to_str[2] = 'I';
    to_str[3] = 'N';
    to_str[4] = ' ';
    if (spin_state == 0) {
        to_str[5] = 'O';
        to_str[6] = 'F';
        to_str[7] = 'F';
        to_str[8] = '\0';
    } else if (spin_state == 1) {
        to_str[5] = 'O';
        to_str[6] = 'N';
        to_str[7] = ' ';
        to_str[8] = '\0';
    }
}

// 生成摩擦轮字符串
void fric_state_str(char *to_str, int fric_state) {
    to_str[0] = 'F';
    to_str[1] = 'R';
    to_str[2] = 'I';
    to_str[3] = 'C';
    to_str[4] = ' ';
    if (fric_state == FRIC_OFF) {
        to_str[5] = 'O';
        to_str[6] = 'F';
        to_str[7] = 'F';
        to_str[8] = '\0';
    } else if (fric_state == FRIC_ON) {
        to_str[5] = 'O';
        to_str[6] = 'N';
        to_str[7] = ' ';
        to_str[8] = '\0';
    } else if (fric_state == FRIC_ACC) {
        to_str[5] = 'A';
        to_str[6] = 'C';
        to_str[7] = 'C';
        to_str[8] = '\0';
    }
}

void state_str(char *to_str, int cap_percent, int spin_state, int fric_state) {
    char cap_str[10];
    char spin_str[10];
    char fric_str[10];

    cap_text_format(cap_str, cap_percent);
    spin_state_str(spin_str, spin_state);
    fric_state_str(fric_str, fric_state);
    for (int i = 0; i < 8; i++)
        to_str[i] = fric_str[i];
    to_str[8] = '\n';
    for (int i = 9; i < 17; i++)
        to_str[i] = spin_str[i - 9];
    to_str[17] = '\n';

    // 字符串长度限制，改成电容只显示三位数字
    to_str[18] = ((cap_percent / 100) % 10) + 48;
    to_str[19] = ((cap_percent / 10) % 10) + 48;
    to_str[20] = (cap_percent % 10) + 48;
    to_str[21] = '\0';
}

// 同步UI_DisplayData_Type的状态到UI控制结构体
void sync_parameter() {
    Crosshair_Data.shoot_dist_percent = (u32)(UI_Data.distance * (100 / 8));
    State_Data.cap_percent = (u32)(UI_Data.Super_cap_percent);
    State_Data.fric_state = UI_Data.fric_state;
    State_Data.spin_state = UI_Data.spin_state;
    //  UI_Data.auto_aim_state = AutoAimData.auto_aim_status;
    //  测试
    //  UI_Data.auto_aim_state = 0;
}

// 从裁判系统读取机器人ID

void Read_Robot_ID(Device::Base *base_) {
    (void)base_;
    update_ui_target_id(Robot_ID_Read);
    // usart1_printf("robotid:%x, cilentid:%x\n", Robot_ID_Read, Cilent_ID_Read);
}

void ui_parameter_init() {
    /*准星参数初始化*/
    Crosshair_Data.center[0] = CROSS_CENTER_X;  // 中心X
    Crosshair_Data.center[1] = CROSS_CENTER_Y;  // 中心Y
    Crosshair_Data.cross_width = 300;           // 宽
    Crosshair_Data.cross_high = 300;            // 高
    Crosshair_Data.cross_high_offset = 0;       // 高偏移

    Crosshair_Data.ballistic_ruler[0] = CROSS_1M;  // 1m标尺
    Crosshair_Data.ballistic_ruler[1] = CROSS_2M;  // 2m标尺
    Crosshair_Data.ballistic_ruler[2] = CROSS_3M;  // 3m标尺
    Crosshair_Data.ballistic_ruler[3] = CROSS_4M;  // 4m标尺
    Crosshair_Data.ballistic_ruler[4] = CROSS_5M;  // 5m标尺

    Crosshair_Data.ruler_length[0] = 400;  // 1m标尺长度
    Crosshair_Data.ruler_length[1] = 300;  // 2m标尺长度
    Crosshair_Data.ruler_length[2] = 200;  // 3m标尺长度
    Crosshair_Data.ruler_length[3] = 100;  // 4m标尺长度
    Crosshair_Data.ruler_length[4] = 50;   // 5m标尺长度

    Crosshair_Data.line_width = 3;                 // 线宽度
    Crosshair_Data.cross_colar = UI_Color_Orange;  // 准星颜色
    Crosshair_Data.ruler_colar = UI_Color_Yellow;  // 标尺颜色

    Crosshair_Data.dist_indicate_color = UI_Color_Purplish_red;  // 测距尺颜色
    Crosshair_Data.dist_indicate_length = 150;                   // 测距尺长度
    Crosshair_Data.dist_indicate_width = 30;                     // 测距尺宽度

    Crosshair_Data.distance = 0;  // 默认距离=0

    /*测距&射速显示*/
    Crosshair_Data.dist_start_point[0] = 140;  // 测距起始X(相对中心)
    Crosshair_Data.dist_start_point[1] = 20;   // 测距起始Y(相对中心)
    Crosshair_Data.dist_display_length = 100;  // 测速条长度
    Crosshair_Data.dist_display_width = 8;     // 测速条宽度
    Crosshair_Data.dist_text_size = 15;        // 测速文字字号
    Crosshair_Data.shoot_dist_percent = 10;

    Crosshair_Data.speed_start_point[0] = 140;  // 测距起始X(相对中心)
    Crosshair_Data.speed_start_point[1] = -30;  // 测距起始Y(相对中心)
    Crosshair_Data.speed_display_length = 100;  // 射速条长度
    Crosshair_Data.speed_display_width = 8;     // 射速条宽度
    Crosshair_Data.speed_text_size = 15;        // 射速条字体大小
    Crosshair_Data.shoot_speed_percent = 10;

    Crosshair_Data.shoot_text_color = UI_Color_Orange;  // 字体颜色
    Crosshair_Data.shoot_bar_color = UI_Color_Cyan;     // 白分条颜色

    State_Data.cap_text_pos[0] = 1600;            // 超电字体X
    State_Data.cap_text_pos[1] = 800;             // 超电字体Y
    State_Data.cap_display_with = 30;             // 超电条宽度
    State_Data.cap_display_length = 250;          // 超电条长度
    State_Data.cap_text_size = 30;                // 字体大小
    State_Data.cap_text_color = UI_Color_Cyan;    // 文字颜色
    State_Data.cap_bar_color = UI_Color_Cyan;     // 百分条颜色
}
