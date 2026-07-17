/**
 * @file referee_UI.C
 * @author kidneygood (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-1-18
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "referee_UI.h"
#include "string.h"
#include "crc_ref.h"
#include "stdio.h"
#include "rm_referee.h"
#include "bsp_log.h"

#define REFEREE_UI_TX_BUFFER_SIZE 128U // 7个图形刷新最大约120字节,预留少量余量

/**
 * @brief 复制裁判系统图形名
 *
 * @note 裁判系统图形名固定3字节。协议要求低地址放末尾字符,因此这里保持旧版反向写入规则。
 */
static void UICopyGraphName(uint8_t graphic_name[3], const char* graphname)
{
    if (graphic_name == NULL)
        return;

    memset(graphic_name, 0, 3U);
    if (graphname == NULL)
        return;

    for (uint8_t i = 0; i < 3U && graphname[i] != '\0'; i++)
    {
        graphic_name[2U - i] = (uint8_t)graphname[i];
    }
}

// 包序号
/********************************************删除操作*************************************
**参数：_id 对应的id结构体
		Del_Operate  对应头文件删除操作
		Del_Layer    要删除的层 取值0-9
*****************************************************************************************/
HAL_StatusTypeDef UIDelete(referee_id_t* _id, uint8_t Del_Operate, uint8_t Del_Layer)
{
    UI_delete_t UI_delete_data = {0};
    HAL_StatusTypeDef status;
    uint8_t temp_datalength = Interactive_Data_LEN_Head + UI_Operate_LEN_Del; // 计算交互数据长度

    if (_id == NULL)
    {
        LOGERROR("[ref_ui] delete with null id");
        return HAL_ERROR;
    }

    UI_delete_data.FrameHeader.SOF = REFEREE_SOF;
    UI_delete_data.FrameHeader.DataLength = temp_datalength;
    UI_delete_data.FrameHeader.Seq = UI_Seq;
    UI_delete_data.FrameHeader.CRC8 = Get_CRC8_Check_Sum((uint8_t*)&UI_delete_data, LEN_CRC8, 0xFF);

    UI_delete_data.CmdID = ID_student_interactive;

    UI_delete_data.datahead.data_cmd_id = UI_Data_ID_Del;
    UI_delete_data.datahead.receiver_ID = _id->Cilent_ID;
    UI_delete_data.datahead.sender_ID = _id->Robot_ID;

    UI_delete_data.Delete_Operate = Del_Operate; // 删除操作
    UI_delete_data.Layer = Del_Layer;

    UI_delete_data.frametail = Get_CRC16_Check_Sum((uint8_t*)&UI_delete_data, LEN_HEADER + LEN_CMDID + temp_datalength,
                                                   0xFFFF);
    /* 填入0xFFFF,关于crc校验 */

    status = RefereeSend((uint8_t*)&UI_delete_data, LEN_HEADER + LEN_CMDID + temp_datalength + LEN_TAIL); // 发送
    if (status == HAL_OK)
        UI_Seq++; // 包序号+1

    return status;
}

/************************************************绘制直线*************************************************
**参数：*graph Graph_Data类型变量指针，用于存放图形数据
		graphname[3]   图片名称，用于标识更改
		Graph_Operate   图片操作，见头文件
		Graph_Layer    图层0-9
		Graph_Color    图形颜色
		Graph_Width    图形线宽
		Start_x、Start_y  起点xy坐标
		End_x、End_y   终点xy坐标
**********************************************************************************************************/

void UILineDraw(Graph_Data_t* graph, const char* graphname, uint32_t Graph_Operate, uint32_t Graph_Layer,
                uint32_t Graph_Color,
                uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t End_x, uint32_t End_y)
{
    if (graph == NULL)
        return;

    memset(graph, 0, sizeof(Graph_Data_t));
    UICopyGraphName(graph->graphic_name, graphname);

    graph->operate_tpye = Graph_Operate;
    graph->graphic_tpye = UI_Graph_Line;
    graph->layer = Graph_Layer;
    graph->color = Graph_Color;

    graph->start_angle = 0;
    graph->end_angle = 0;
    graph->width = Graph_Width;
    graph->start_x = Start_x;
    graph->start_y = Start_y;
    graph->radius = 0;
    graph->end_x = End_x;
    graph->end_y = End_y;
}

/************************************************绘制矩形*************************************************
**参数：*graph Graph_Data类型变量指针，用于存放图形数据
		graphname[3]   图片名称，用于标识更改
		Graph_Operate   图片操作，见头文件
		Graph_Layer    图层0-9
		Graph_Color    图形颜色
		Graph_Width    图形线宽
		Start_x、Start_y    起点xy坐标
		End_x、End_y        对角顶点xy坐标
**********************************************************************************************************/
void UIRectangleDraw(Graph_Data_t* graph, const char* graphname, uint32_t Graph_Operate, uint32_t Graph_Layer,
                     uint32_t Graph_Color,
                     uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t End_x, uint32_t End_y)
{
    if (graph == NULL)
        return;

    memset(graph, 0, sizeof(Graph_Data_t));
    UICopyGraphName(graph->graphic_name, graphname);

    graph->graphic_tpye = UI_Graph_Rectangle;
    graph->operate_tpye = Graph_Operate;
    graph->layer = Graph_Layer;
    graph->color = Graph_Color;

    graph->start_angle = 0;
    graph->end_angle = 0;
    graph->width = Graph_Width;
    graph->start_x = Start_x;
    graph->start_y = Start_y;
    graph->radius = 0;
    graph->end_x = End_x;
    graph->end_y = End_y;
}

/************************************************绘制整圆*************************************************
**参数：*graph Graph_Data类型变量指针，用于存放图形数据
		graphname[3]   图片名称，用于标识更改
		Graph_Operate   图片操作，见头文件
		Graph_Layer    图层0-9
		Graph_Color    图形颜色
		Graph_Width    图形线宽
		Start_x、Start_y    圆心xy坐标
		Graph_Radius  圆形半径
**********************************************************************************************************/

void UICircleDraw(Graph_Data_t* graph, const char* graphname, uint32_t Graph_Operate, uint32_t Graph_Layer,
                  uint32_t Graph_Color,
                  uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t Graph_Radius)
{
    if (graph == NULL)
        return;

    memset(graph, 0, sizeof(Graph_Data_t));
    UICopyGraphName(graph->graphic_name, graphname);

    graph->graphic_tpye = UI_Graph_Circle;
    graph->operate_tpye = Graph_Operate;
    graph->layer = Graph_Layer;
    graph->color = Graph_Color;

    graph->start_angle = 0;
    graph->end_angle = 0;
    graph->width = Graph_Width;
    graph->start_x = Start_x;
    graph->start_y = Start_y;
    graph->radius = Graph_Radius;
    graph->end_x = 0;
    graph->end_y = 0;
}

/************************************************绘制椭圆*************************************************
**参数：*graph Graph_Data类型变量指针，用于存放图形数据
		graphname[3]   图片名称，用于标识更改
		Graph_Operate   图片操作，见头文件
		Graph_Layer    图层0-9
		Graph_Color    图形颜色
		Graph_Width    图形线宽
		Start_x、Start_y    圆心xy坐标
		End_x、End_y        xy半轴长度
**********************************************************************************************************/
void UIOvalDraw(Graph_Data_t* graph, const char* graphname, uint32_t Graph_Operate, uint32_t Graph_Layer,
                uint32_t Graph_Color,
                uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t end_x, uint32_t end_y)
{
    if (graph == NULL)
        return;

    memset(graph, 0, sizeof(Graph_Data_t));
    UICopyGraphName(graph->graphic_name, graphname);

    graph->graphic_tpye = UI_Graph_Ellipse;
    graph->operate_tpye = Graph_Operate;
    graph->layer = Graph_Layer;
    graph->color = Graph_Color;
    graph->width = Graph_Width;

    graph->start_angle = 0;
    graph->end_angle = 0;
    graph->width = Graph_Width;
    graph->start_x = Start_x;
    graph->start_y = Start_y;
    graph->radius = 0;
    graph->end_x = end_x;
    graph->end_y = end_y;
}

/************************************************绘制圆弧*************************************************
**参数：*graph Graph_Data类型变量指针，用于存放图形数据
		graphname[3]   图片名称，用于标识更改
		Graph_Operate   图片操作，见头文件
		Graph_Layer    图层0-9
		Graph_Color    图形颜色
		Graph_StartAngle,Graph_EndAngle    起始终止角度
		Graph_Width    图形线宽
		Start_y,Start_y    圆心xy坐标
		x_Length,y_Length   xy半轴长度
**********************************************************************************************************/

void UIArcDraw(Graph_Data_t* graph, const char* graphname, uint32_t Graph_Operate, uint32_t Graph_Layer,
               uint32_t Graph_Color,
               uint32_t Graph_StartAngle, uint32_t Graph_EndAngle, uint32_t Graph_Width, uint32_t Start_x,
               uint32_t Start_y,
               uint32_t end_x, uint32_t end_y)
{
    if (graph == NULL)
        return;

    memset(graph, 0, sizeof(Graph_Data_t));
    UICopyGraphName(graph->graphic_name, graphname);

    graph->graphic_tpye = UI_Graph_Arc;
    graph->operate_tpye = Graph_Operate;
    graph->layer = Graph_Layer;
    graph->color = Graph_Color;

    graph->start_angle = Graph_StartAngle;
    graph->end_angle = Graph_EndAngle;
    graph->width = Graph_Width;
    graph->start_x = Start_x;
    graph->start_y = Start_y;
    graph->radius = 0;
    graph->end_x = end_x;
    graph->end_y = end_y;
}

/************************************************绘制浮点型数据*************************************************
**参数：*graph Graph_Data类型变量指针，用于存放图形数据
		graphname[3]   图片名称，用于标识更改
		Graph_Operate   图片操作，见头文件
		Graph_Layer    图层0-9
		Graph_Color    图形颜色
		Graph_Size     字号
		Graph_Digit    小数位数
		Graph_Width    图形线宽
		Start_x、Start_y    开始坐标
		radius=a&0x3FF;   a为浮点数乘以1000后的32位整型数
		end_x=(a>>10)&0x7FF;
		end_y=(a>>21)&0x7FF;
**********************************************************************************************************/

void UIFloatDraw(Graph_Data_t* graph, const char* graphname, uint32_t Graph_Operate, uint32_t Graph_Layer,
                 uint32_t Graph_Color,
                 uint32_t Graph_Size, uint32_t Graph_Digit, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y,
                 int32_t Graph_Float)
{
    if (graph == NULL)
        return;

    memset(graph, 0, sizeof(Graph_Data_t));
    UICopyGraphName(graph->graphic_name, graphname);
    graph->graphic_tpye = UI_Graph_Float;
    graph->operate_tpye = Graph_Operate;
    graph->layer = Graph_Layer;
    graph->color = Graph_Color;

    graph->width = Graph_Width;
    graph->start_x = Start_x;
    graph->start_y = Start_y;
    graph->start_angle = Graph_Size;
    graph->end_angle = Graph_Digit;

    graph->radius = Graph_Float & 0x3FF;
    graph->end_x = (Graph_Float >> 10) & 0x7FF;
    graph->end_y = (Graph_Float >> 21) & 0x7FF;
}

/************************************************绘制整型数据*************************************************
**参数：*graph Graph_Data类型变量指针，用于存放图形数据
		graphname[3]   图片名称，用于标识更改
		Graph_Operate   图片操作，见头文件
		Graph_Layer    图层0-9
		Graph_Color    图形颜色
		Graph_Size     字号
		Graph_Width    图形线宽
		Start_x、Start_y    开始坐标
		radius=a&0x3FF;   a为32位整型数
		end_x=(a>>10)&0x7FF;
		end_y=(a>>21)&0x7FF;
**********************************************************************************************************/
void UIIntDraw(Graph_Data_t* graph, const char* graphname, uint32_t Graph_Operate, uint32_t Graph_Layer,
               uint32_t Graph_Color,
               uint32_t Graph_Size, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, int32_t Graph_Integer)
{
    if (graph == NULL)
        return;

    memset(graph, 0, sizeof(Graph_Data_t));
    UICopyGraphName(graph->graphic_name, graphname);
    graph->graphic_tpye = UI_Graph_Int;
    graph->operate_tpye = Graph_Operate;
    graph->layer = Graph_Layer;
    graph->color = Graph_Color;

    graph->start_angle = Graph_Size;
    graph->end_angle = 0;
    graph->width = Graph_Width;
    graph->start_x = Start_x;
    graph->start_y = Start_y;
    graph->radius = Graph_Integer & 0x3FF;
    graph->end_x = (Graph_Integer >> 10) & 0x7FF;
    graph->end_y = (Graph_Integer >> 21) & 0x7FF;
}

/************************************************绘制字符型数据*************************************************
**参数：*graph Graph_Data类型变量指针，用于存放图形数据
		graphname[3]   图片名称，用于标识更改
		Graph_Operate   图片操作，见头文件
		Graph_Layer    图层0-9
		Graph_Color    图形颜色
		Graph_Size     字号
		Graph_Width    图形线宽
		Start_x、Start_y    开始坐标

**参数：*graph Graph_Data类型变量指针，用于存放图形数据
		fmt需要显示的字符串
		此函数的实现和具体使用类似于printf函数
**********************************************************************************************************/
void UICharDraw(String_Data_t* graph, const char* graphname, uint32_t Graph_Operate, uint32_t Graph_Layer,
                uint32_t Graph_Color,
                uint32_t Graph_Size, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, const char* fmt, ...)
{
    int show_len;

    if (graph == NULL || fmt == NULL)
        return;

    memset(graph, 0, sizeof(String_Data_t));
    UICopyGraphName(graph->Graph_Control.graphic_name, graphname);

    graph->Graph_Control.graphic_tpye = UI_Graph_Char;
    graph->Graph_Control.operate_tpye = Graph_Operate;
    graph->Graph_Control.layer = Graph_Layer;
    graph->Graph_Control.color = Graph_Color;

    graph->Graph_Control.width = Graph_Width;
    graph->Graph_Control.start_x = Start_x;
    graph->Graph_Control.start_y = Start_y;
    graph->Graph_Control.start_angle = Graph_Size;
    graph->Graph_Control.radius = 0;
    graph->Graph_Control.end_x = 0;
    graph->Graph_Control.end_y = 0;
    memset(graph->show_Data, 0, sizeof(graph->show_Data)); // 清空旧字符串残留,短字符串刷新时不会带出上一帧内容

    va_list ap;
    va_start(ap, fmt);
    show_len = vsnprintf((char*)graph->show_Data, sizeof(graph->show_Data), fmt, ap); // 限制写入长度,避免覆盖后续字段
    va_end(ap);
    if (show_len < 0)
    {
        graph->show_Data[0] = '\0';
        graph->Graph_Control.end_angle = 0;
    }
    else if ((uint32_t)show_len >= sizeof(graph->show_Data))
    {
        graph->Graph_Control.end_angle = sizeof(graph->show_Data) - 1U;
    }
    else
    {
        graph->Graph_Control.end_angle = (uint32_t)show_len;
    }
}

/* UI推送函数（使更改生效）
   参数： cnt   图形个数
			...   图形变量参数
   Tips：：该函数只能推送1，2，5，7个图形，其他数目协议未涉及
 */
HAL_StatusTypeDef UIGraphRefresh(referee_id_t* _id, int cnt, ...)
{
    UI_GraphReFresh_t UI_GraphReFresh_data = {0};
    Graph_Data_t graphData;
    HAL_StatusTypeDef status;
    uint16_t temp_datalength;
    uint8_t buffer[REFEREE_UI_TX_BUFFER_SIZE] = {0}; // 交互数据缓存,只在UI任务栈上短暂存在

    if (_id == NULL)
    {
        LOGERROR("[ref_ui] graph refresh with null id");
        return HAL_ERROR;
    }

    if (cnt != 1 && cnt != 2 && cnt != 5 && cnt != 7)
    {
        LOGERROR("[ref_ui] graph refresh count [%d] invalid", cnt);
        return HAL_ERROR;
    }

    temp_datalength = (uint16_t)(
        LEN_HEADER + LEN_CMDID + Interactive_Data_LEN_Head + UI_Operate_LEN_PerDraw * cnt + LEN_TAIL); // 计算整包长度
    if (temp_datalength > REFEREE_UI_TX_BUFFER_SIZE)
    {
        LOGERROR("[ref_ui] graph refresh length [%d] exceed buffer [%d]", temp_datalength, REFEREE_UI_TX_BUFFER_SIZE);
        return HAL_ERROR;
    }

    va_list ap; // 创建一个 va_list 类型变量
    va_start(ap, cnt); // 初始化 va_list 变量为一个参数列表

    UI_GraphReFresh_data.FrameHeader.SOF = REFEREE_SOF;
    UI_GraphReFresh_data.FrameHeader.DataLength = (uint16_t)(Interactive_Data_LEN_Head + cnt * UI_Operate_LEN_PerDraw);
    UI_GraphReFresh_data.FrameHeader.Seq = UI_Seq;
    UI_GraphReFresh_data.FrameHeader.CRC8 = Get_CRC8_Check_Sum((uint8_t*)&UI_GraphReFresh_data, LEN_CRC8, 0xFF);

    UI_GraphReFresh_data.CmdID = ID_student_interactive;

    switch (cnt)
    {
    case 1:
        UI_GraphReFresh_data.datahead.data_cmd_id = UI_Data_ID_Draw1;
        break;
    case 2:
        UI_GraphReFresh_data.datahead.data_cmd_id = UI_Data_ID_Draw2;
        break;
    case 5:
        UI_GraphReFresh_data.datahead.data_cmd_id = UI_Data_ID_Draw5;
        break;
    case 7:
        UI_GraphReFresh_data.datahead.data_cmd_id = UI_Data_ID_Draw7;
        break;
    }

    UI_GraphReFresh_data.datahead.receiver_ID = _id->Cilent_ID;
    UI_GraphReFresh_data.datahead.sender_ID = _id->Robot_ID;
    memcpy(buffer, (uint8_t*)&UI_GraphReFresh_data, LEN_HEADER + LEN_CMDID + Interactive_Data_LEN_Head);
    // 将帧头、命令码、交互数据帧头三部分复制到缓存中

    for (uint8_t i = 0; i < cnt; i++) // 发送交互数据的数据帧，并计算CRC16校验值
    {
        graphData = va_arg(ap, Graph_Data_t); // 访问参数列表中的每个项,第二个参数是你要返回的参数的类型,在取值时需要将其强制转化为指定类型的变量
        memcpy(buffer + (LEN_HEADER + LEN_CMDID + Interactive_Data_LEN_Head + UI_Operate_LEN_PerDraw * i),
               (uint8_t*)&graphData, UI_Operate_LEN_PerDraw);
    }
    Append_CRC16_Check_Sum(buffer, temp_datalength);
    status = RefereeSend(buffer, temp_datalength);
    if (status == HAL_OK)
        UI_Seq++;

    va_end(ap); // 结束可变参数的获取
    return status;
}

/************************************************UI推送字符（使更改生效）*********************************/
HAL_StatusTypeDef UICharRefresh(referee_id_t* _id, String_Data_t string_Data)
{
    UI_CharReFresh_t UI_CharReFresh_data = {0};
    HAL_StatusTypeDef status;
    uint8_t temp_datalength = Interactive_Data_LEN_Head + UI_Operate_LEN_DrawChar; // 计算交互数据长度

    if (_id == NULL)
    {
        LOGERROR("[ref_ui] char refresh with null id");
        return HAL_ERROR;
    }

    UI_CharReFresh_data.FrameHeader.SOF = REFEREE_SOF;
    UI_CharReFresh_data.FrameHeader.DataLength = temp_datalength;
    UI_CharReFresh_data.FrameHeader.Seq = UI_Seq;
    UI_CharReFresh_data.FrameHeader.CRC8 = Get_CRC8_Check_Sum((uint8_t*)&UI_CharReFresh_data, LEN_CRC8, 0xFF);

    UI_CharReFresh_data.CmdID = ID_student_interactive;

    UI_CharReFresh_data.datahead.data_cmd_id = UI_Data_ID_DrawChar;

    UI_CharReFresh_data.datahead.receiver_ID = _id->Cilent_ID;
    UI_CharReFresh_data.datahead.sender_ID = _id->Robot_ID;

    UI_CharReFresh_data.String_Data = string_Data;

    UI_CharReFresh_data.frametail = Get_CRC16_Check_Sum((uint8_t*)&UI_CharReFresh_data,
                                                        LEN_HEADER + LEN_CMDID + temp_datalength, 0xFFFF);

    status = RefereeSend((uint8_t*)&UI_CharReFresh_data, LEN_HEADER + LEN_CMDID + temp_datalength + LEN_TAIL); // 发送
    if (status == HAL_OK)
        UI_Seq++; // 包序号+1

    return status;
}
