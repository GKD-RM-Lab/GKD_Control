#ifndef UI_HPP
#define UI_HPP

#include "device/referee/referee_base.hpp"
#include "types.hpp"

#pragma pack(push, 1)  // 按1字节对齐

// #define NULL 0
#define __FALSE                 100
#define KEY_PRESSED_OFFSET_X    ((uint16_t)1 << 12)
#define UI_MODE_SWITCH_KEYBOARD KEY_PRESSED_OFFSET_X

/****************************开始标志*********************/
#define UI_SOF 0xA5
/****************************CMD_ID数据********************/
#define UI_CMD_Robo_Exchange 0x0301
/****************************内容ID数据********************/
#define UI_Data_ID_Del        0x100
#define UI_Data_ID_Draw1      0x101
#define UI_Data_ID_Draw2      0x102
#define UI_Data_ID_Draw5      0x103
#define UI_Data_ID_Draw7      0x104
#define UI_Data_ID_DrawString 0x110
/****************************红方机器人ID********************/
#define UI_Data_RobotID_RHero      1
#define UI_Data_RobotID_REngineer  2
#define UI_Data_RobotID_RStandard1 3
#define UI_Data_RobotID_RStandard2 4
#define UI_Data_RobotID_RStandard3 5
#define UI_Data_RobotID_RAerial    6
#define UI_Data_RobotID_RSentry    7
#define UI_Data_RobotID_RRadar     9
/****************************蓝方机器人ID********************/
#define UI_Data_RobotID_BHero      101
#define UI_Data_RobotID_BEngineer  102
#define UI_Data_RobotID_BStandard1 103
#define UI_Data_RobotID_BStandard2 104
#define UI_Data_RobotID_BStandard3 105
#define UI_Data_RobotID_BAerial    106
#define UI_Data_RobotID_BSentry    107
#define UI_Data_RobotID_BRadar     109
/**************************红方操作手ID************************/
#define UI_Data_CilentID_RHero      0x0101
#define UI_Data_CilentID_REngineer  0x0102
#define UI_Data_CilentID_RStandard1 0x0103
#define UI_Data_CilentID_RStandard2 0x0104
#define UI_Data_CilentID_RStandard3 0x0105
#define UI_Data_CilentID_RAerial    0x0106
/***************************蓝方操作手ID***********************/
#define UI_Data_CilentID_BHero      0x0165
#define UI_Data_CilentID_BEngineer  0x0166
#define UI_Data_CilentID_BStandard1 0x0167
#define UI_Data_CilentID_BStandard2 0x0168
#define UI_Data_CilentID_BStandard3 0x0169
#define UI_Data_CilentID_BAerial    0x016A
/***************************删除操作***************************/
#define UI_Data_Del_NoOperate 0
#define UI_Data_Del_Layer     1
#define UI_Data_Del_ALL       2
/***************************图形配置参数__图形操作********************/
#define UI_Graph_ADD    1
#define UI_Graph_Change 2
#define UI_Graph_Del    3
/***************************图形配置参数__图形类型********************/
#define UI_Graph_Line      0  // 直线
#define UI_Graph_Rectangle 1  // 矩形
#define UI_Graph_Circle    2  // 整圆
#define UI_Graph_Ellipse   3  // 椭圆
#define UI_Graph_Arc       4  // 圆弧
#define UI_Graph_Float     5  // 浮点型
#define UI_Graph_Int       6  // 整形
#define UI_Graph_Char      7  // 字符型
/***************************图形配置参数__图形颜色********************/
#define UI_Color_Main         0  // 红蓝主色
#define UI_Color_Yellow       1
#define UI_Color_Green        2
#define UI_Color_Orange       3
#define UI_Color_Purplish_red 4  // 紫红色
#define UI_Color_Pink         5
#define UI_Color_Cyan         6  // 青色
#define UI_Color_Black        7
#define UI_Color_White        8

#define FRIC_NAME_START_X 800
#define FRIC_NAME_START_Y 500
#define SPIN_NAME_START_X 850
#define SPIN_NAME_START_Y 550

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;

typedef struct
{
    u8 SOF;           // 起始字节,固定0xA5
    u16 Data_Length;  // 帧数据长度
    u8 Seq;           // 包序号
    u8 CRC8;          // CRC8校验值
    u16 CMD_ID;       // 命令ID
} UI_Packhead;        // 帧头

typedef struct
{
    u16 Data_ID;      // 内容ID
    u16 Sender_ID;    // 发送者ID
    u16 Receiver_ID;  // 接收者ID
} UI_Data_Operate;    // 操作定义帧

typedef struct
{
    u8 Delete_Operate;  // 删除操作
    u8 Layer;           // 删除图层
} UI_Data_Delete;       // 删除图层帧

typedef struct
{
    uint8_t graphic_name[3];
    uint32_t operate_tpye : 3;
    uint32_t graphic_tpye : 3;
    uint32_t layer : 4;
    uint32_t color : 4;
    uint32_t start_angle : 9;
    uint32_t end_angle : 9;
    uint32_t width : 10;
    uint32_t start_x : 11;
    uint32_t start_y : 11;
    float graph_Float;  // 浮点数据
} Float_Data;

typedef struct
{
    uint8_t graphic_name[3];
    uint32_t operate_tpye : 3;
    uint32_t graphic_tpye : 3;
    uint32_t layer : 4;
    uint32_t color : 4;
    uint32_t start_angle : 9;
    uint32_t end_angle : 9;
    uint32_t width : 10;
    uint32_t start_x : 11;
    uint32_t start_y : 11;
    uint32_t radius : 10;
    uint32_t end_x : 11;
    uint32_t end_y : 11;  // 图形数据
} Graph_Data;

typedef struct
{
    Graph_Data Graph_Control;
    uint8_t show_Data[30];
} String_Data;  // 打印字符串数据

// typedef struct{
//		uint8_t switch_open:1;
//		uint8_t switch_mode:1;
//		String_Data name;
//		Graph_Data graph;
//		uint32_t start_x:11;
//		uint32_t start_y:11;
//		uint32_t high:11;
//		uint32_t width:11;
// } switch_t;

typedef struct
{
    const Types::RC_ctrl_t *UI_rc_ctrl;
    uint8_t UI_open : 1;
    uint8_t UI_mode : 1;
    uint8_t fric_open : 1;
    uint8_t fric_mode : 1;
    uint8_t spin_open;
    uint8_t spin_mode;
    String_Data fric_name, spin_name;
    Graph_Data fric_graph, spin_graph, accuracy_graph;
} UI_control_t;

extern void UI_task(Device::Base *base_);
extern void UI_set_fric(uint8_t mode);
extern void UI_set_spin(uint8_t mode);
extern int String_ReFresh(Device::Base *base_, String_Data string_Data);
extern void String_Draw(
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
    std::string String_Data);
extern void UI_Delete(Device::Base *base_, u8 Del_Operate, u8 Del_Layer);
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
    u32 End_y);
int UI_ReFresh(Device::Base *base_, int cnt, ...);
void Circle_Draw(
    Graph_Data *image,
    std::string imagename,
    u32 Graph_Operate,
    u32 Graph_Layer,
    u32 Graph_Color,
    u32 Graph_Width,
    u32 Start_x,
    u32 Start_y,
    u32 Graph_Radius);
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
    u32 End_y);
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
    float Graph_Float);
void Char_Draw(
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
    std::string Char_Data);
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
    u32 y_Length);

extern uint16_t Robot_ID_Read;
extern uint16_t Cilent_ID_Read;

/*custom_ui*/
/*准星中心位置*/
#define CROSS_CENTER_X 960
#define CROSS_CENTER_Y 540

/*弹道标定*/
#define CROSS_1M 100
#define CROSS_2M 200
#define CROSS_3M 300
#define CROSS_4M 400
#define CROSS_5M 500

/*机器人角色*/
#define Robot_ID  UI_Data_RobotID_RHero
#define Cilent_ID UI_Data_CilentID_RHero

/*UI模式*/
#define UI_INFANTRY 1
#define UI_HERO     2
#define UI_MODE     UI_INFANTRY

#define RE_INIT_CYCLE 30

/*摩擦轮状态*/
#define FRIC_OFF 0
#define FRIC_ON  1
#define FRIC_ACC 2  // 加速状态

/*UI显示参数的结构体*/
typedef struct
{
    float distance;           // 目标距离
    float shoot_speed;        // 射速
    float Super_cap_percent;  // 超电百分比
    int spin_state;           // 小陀螺状态
    int fric_state;           // 摩擦轮状态
    int auto_aim_state;       // 自瞄状态
    int purchased_bullet_num; // 已购买弹丸数量
    int remain_bullet_num;    // 可发射弹丸数量
    int fired_bullet_num;     // 已发射弹丸数量
} UI_DisplayData_Type;

/*描述准星的结构体*/
typedef struct
{
    /*准星本体*/
    u32 center[2];           // 准星中心
    u32 ballistic_ruler[5];  // 弹道标尺，分别为1m~5m落点相对准星下降的距离
    u32 ruler_length[5];     // 标尺的长度
    u32 line_width;          // 线宽度
    u32 cross_width;
    u32 cross_high;            // 十字高度
    u32 cross_high_offset;     // 十字高度偏移
    u32 cross_colar;           // 准星颜色
    u32 ruler_colar;           // 标尺颜色
    u32 dist_indicate_color;   // 测距指示颜色
    u32 dist_indicate_length;  // 测距指示长度
    u32 dist_indicate_width;   // 测距指示宽度
    u32 distance;              // 测得距离

    /*测距&弹速显示*/
    u32 dist_start_point[2];  // 测距文字起始位置
    u32 dist_text_size;       // 测速文字字号
    u32 dist_display_length;  // 测速条的长度
    u32 dist_display_width;   // 测速条宽度

    u32 speed_start_point[2];  // 射速文字起始位置
    u32 speed_display_length;  // 射速条的长度
    u32 speed_display_width;   // 射速条宽度
    u32 speed_text_size;

    u32 shoot_text_color;  // 文字颜色
    u32 shoot_bar_color;   // 百分条颜色

    u32 shoot_speed_percent;
    u32 shoot_dist_percent;
} Crosshair_Data_Type;

/*描述右下角状态指示的结构体*/
typedef struct
{
    /* 超电部分 */
    u32 cap_text_pos[2];     // 超电字体位置，左下角
    u32 cap_display_length;  // 容量条长度
    u32 cap_display_with;    // 容量条宽度
    u32 cap_text_size;       // 超电字号
    u32 cap_text_color;      // 字体颜色
    u32 cap_bar_color;       // 百分条颜色
    int cap_percent;         // 超电百分比

    /* 状态部分 */
    u32 spin_state_pos[2];
    u32 fric_state_pos[2];
    int spin_state;
    int fric_state;
} State_Indicate_Type;

void custom_ui_task(Device::Base *base_, uint8_t &robot_id_);
extern void custom_UI_init(Device::Base *base_);
extern volatile UI_DisplayData_Type UI_Data;

void draw_crosshair_hero(Device::Base *base_);
void draw_crosshair_infantry(Device::Base *base_);
void update_dynamic_paramater(Device::Base *base_);
void UI_clear(Device::Base *base_);
void custom_UI_init(Device::Base *base_);
void int_to_str(char *to_str, int number);
void cap_text_format(char *to_str, int cap_percent);
void spin_state_str(char *to_str, int spin_state);
void fric_state_str(char *to_str, int fric_state);
void ui_parameter_init();
void cap_text_format(char *to_str, int cap_percent);
void sync_parameter();
void state_str(char *to_str, int cap_percent, int spin_state, int fric_state);
void UI_init_draw(Device::Base *base_);
void Read_Robot_ID(Device::Base *base_);

void update_ui_data(
    Device::Base *base_,
    bool fric_state,
    bool auto_aim_state,
    bool spin_state,
    float cap_state,
    uint32_t purchased_bullet_num,
    uint32_t remain_bullet_num);

#pragma pack(pop)
#endif
