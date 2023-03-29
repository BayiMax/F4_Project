/*
 *                        _oo0oo_
 *                       o8888888o
 *                       88" . "88
 *                       (| -_- |)
 *                       0\  =  /0
 *                     ___/`---'\___
 *                   .' \\|     |// '.
 *                  / \\|||  :  |||// \
 *                 / _||||| -:- |||||- \
 *                |   | \\\  - /// |   |
 *                | \_|  ''\---/''  |_/ |
 *                \  .-\__  '-'  ___/-. /
 *              ___'. .'  /--.--\  `. .'___
 *           ."" '<  `.___\_<|>_/___.' >' "".
 *          | | :  `- \`.;`\ _ /`;.`/ - ` : | |
 *          \  \ `_.   \_ __\ /__ _/   .-` /  /
 *      =====`-.____`.___ \_____/___.-`___.-'=====
 *                        `=---='
 *
 *
 *      ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *            佛祖保佑     永不宕机     永无BUG
 */

/**
 * @file oled.c
 * @author {白秉鑫}-{bbx20010518@outlook.com}
 * @brief
 * @version 0.1
 * @date 2023-03-26
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "oled.h"
#include "main.h"
#include "user.h"

extern I2C_HandleTypeDef hi2c1;

#define OLED_ADDR 0x78

typedef enum
{
  OLED_CMD = 0x00,
  OLED_DATA = 0x40
} WRITE_MODE;

typedef struct
{
  /**
   * @brief  写数据
   *
   * @param write_mod 数据格式
   * @param pdata 数据发送缓冲区
   * @param size 数据长度
   */
  void (*Write)(WRITE_MODE write_mode, char *pdata, uint16_t size);
  /**
   * @brief 更新显存
   *
   */
  void (*Refresh_Gram)(void);
  /**
   * @brief 设置坐标
   *
   * @param x
   * @param y
   */
  void (*Set_pos)(uint8_t x, uint8_t y);

  /**
   * @brief oled关显示
   *
   */
  void (*Display_OFF)(void);
  /**
   * @brief oled开显示
   *
   */
  void (*Display_ON)(void);
  /**
   * @brief 画点
   *
   * @param x x坐标
   * @param y y坐标
   * @param t 1填充/0清空
   */
  void (*DrawPoint)(uint8_t x, uint8_t y, uint8_t t);
  /**
   * @brief 填充屏幕
   *
   * @param x1 起点x
   * @param y1 起点y
   * @param x2 终点x
   * @param y2 终点y
   * @param dot 填充数据 0xff / 0
   */
  void (*fill)(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t dot);
  /**
   * @brief 画线
   *
   * @param x1 起点x
   * @param y1 起点y
   * @param x2 终点x
   * @param y2 终点y
   * @param dot 填充数据
   */
  void (*DrawLine)(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t dot);
  /**
   * @brief 画方
   *
   * @param x1 起点坐标
   * @param y1 起点坐标
   * @param x2 终点坐标
   * @param y2 终点坐标
   * @param dot 填充数据
   */
  void (*DrawRectangle)(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t dot);
  /**
   * @brief 画圆
   *
   * @param x0 圆心坐标
   * @param y0 圆心坐标
   * @param r 半径
   * @param dot 颜色
   */
  void (*DrawCircle)(uint8_t x0, uint8_t y0, uint8_t r, uint8_t dot);

} OLED_CONTROL;
/**
 * @brief olde控制结构体
 *
 */
OLED_CONTROL oled_control;

uint8_t oled_buf[4][128];
/**
 * @brief  写数据
 *
 * @param write_mod 数据格式
 * @param pdata 数据发送缓冲区
 * @param size 数据长度
 */
static void I2C_Write(WRITE_MODE write_mode, char *pdata, uint16_t size)
{
  HAL_I2C_Mem_Write(&hi2c1, (uint16_t)OLED_ADDR, write_mode, I2C_MEMADD_SIZE_8BIT, (uint8_t *)pdata, size, 100);
}
/**
 * @brief 更新显存
 *
 */
static void Refresh_Gram(void)
{
  uint8_t m;
  char buff[3] = {0, 0x00, 0x10};
  for (m = 0; m < 4; m++)
  {
    buff[0] = 0xb0 + m;
    oled_control.Write(OLED_CMD, buff, 3);
    oled_control.Write(OLED_DATA, (char *)oled_buf[m], 128);
  }
}
/**
 * @brief 设置坐标
 *
 * @param x
 * @param y
 */
static void Set_Pos(uint8_t x, uint8_t y)
{
  char buff[3];
  buff[0] = 0xb0 + y;
  buff[1] = ((x & 0xf0) >> 4) | 0x10;
  buff[2] = x & 0x0f;
  oled_control.Write(OLED_CMD, buff, 3);
}
/**
 * @brief oled开显示
 *
 */
static void Display_ON(void)
{
  char buff[3];
  buff[0] = 0x8D;
  buff[1] = 0x14;
  buff[2] = 0xaf;
  oled_control.Write(OLED_CMD, buff, 3);
}
/**
 * @brief oled关显示
 *
 */
static void Display_OFF(void)
{
  char buff[3];
  buff[0] = 0x8D;
  buff[1] = 0x10;
  buff[2] = 0xae;
  oled_control.Write(OLED_CMD, buff, 3);
}
/**
 * @brief 画点
 *
 * @param x x坐标
 * @param y y坐标
 * @param t 1填充/0清空
 */
static void DrawPoint(uint8_t x, uint8_t y, uint8_t t)
{
  uint8_t pos, bx, temp = 0;
  if (x > 127 || y > 32)
    return;
  pos = 3 - y / 8;
  bx = y % 8;
  temp = 1 << (7 - bx);
  if (t)
    oled_buf[pos][x] |= temp;
  else
    oled_buf[pos][x] &= ~temp;
}
/**
 * @brief 填充屏幕
 *
 * @param x1 起点x
 * @param y1 起点y
 * @param x2 终点x
 * @param y2 终点y
 * @param dot 填充数据 1 / 0
 */
static void fill(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t dot)
{
  uint8_t x, y;
  for (x = x1; x <= x2; x++)
  {
    for (y = y1; y <= y2; y++)
      oled_control.DrawPoint(x, y, dot);
  }
}
/**
 * @brief 画线
 *
 * @param x1 起点x
 * @param y1 起点y
 * @param x2 终点x
 * @param y2 终点y
 * @param dot 填充数据
 */
static void DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t dot)
{
  if (x1 > 128 || x2 > 128 || y1 > 32 || y2 > 32)
  {
    return;
  }
  short t;
  short xerr = 0, yerr = 0, delta_x, delta_y, distance;
  short incx, incy, uRow, uCol;
  delta_x = x2 - x1; // 计算坐标增量
  delta_y = y2 - y1;
  uRow = x1;
  uCol = y1;
  if (delta_x > 0)
    incx = 1; // 设置单步方向
  else if (delta_x == 0)
    incx = 0; // 垂直线
  else
  {
    incx = -1;
    delta_x = -delta_x;
  }
  if (delta_y > 0)
    incy = 1;
  else if (delta_y == 0)
    incy = 0; // 水平线
  else
  {
    incy = -1;
    delta_y = -delta_y;
  }
  if (delta_x > delta_y)
    distance = delta_x; // 选取基本增量坐标轴
  else
    distance = delta_y;
  for (t = 0; t <= distance + 1; t++) // 画线输出
  {
    oled_control.DrawPoint(uRow, uCol, dot); // 画点
    xerr += delta_x;
    yerr += delta_y;
    if (xerr > distance)
    {
      xerr -= distance;
      uRow += incx;
    }
    if (yerr > distance)
    {
      yerr -= distance;
      uCol += incy;
    }
  }
  return;
}
/**
 * @brief 画方
 *
 * @param x1 起点坐标
 * @param y1 起点坐标
 * @param x2 终点坐标
 * @param y2 终点坐标
 * @param dot 填充数据
 */
static void DrawRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t dot)
{
  oled_control.DrawLine(x1, y1, x2, y1, dot);
  oled_control.DrawLine(x1, y1, x1, y2, dot);
  oled_control.DrawLine(x1, y2, x2, y2, dot);
  oled_control.DrawLine(x2, y1, x2, y2, dot);
}
/**
 * @brief 画圆
 *
 * @param x0 圆心坐标
 * @param y0 圆心坐标
 * @param r 半径
 * @param dot 颜色
 */
static void DrawCircle(uint8_t x0, uint8_t y0, uint8_t r, uint8_t dot)
{
  short a, b;
  short di;
  a = 0;
  b = r;
  di = 3 - (r << 1); // 判断下个点位置的标志
  while (a <= b)
  {
    oled_control.DrawPoint(x0 + a, y0 - b, dot); // 5
    oled_control.DrawPoint(x0 + b, y0 - a, dot); // 0
    oled_control.DrawPoint(x0 + b, y0 + a, dot); // 4
    oled_control.DrawPoint(x0 + a, y0 + b, dot); // 6
    oled_control.DrawPoint(x0 - a, y0 + b, dot); // 1
    oled_control.DrawPoint(x0 - b, y0 + a, dot);
    oled_control.DrawPoint(x0 - a, y0 - b, dot); // 2
    oled_control.DrawPoint(x0 - b, y0 - a, dot); // 7
    a++;
    // 使用Bresenham算法画圆
    if (di < 0)
      di += 4 * a + 6;
    else
    {
      di += 10 + 4 * (a - b);
      b--;
    }
  }
}
/**
 * @brief 控制结构体初始化
 *
 */
void control_init(void)
{
  oled_control.Write = I2C_Write;
  oled_control.Refresh_Gram = Refresh_Gram;
  oled_control.Set_pos = Set_Pos;
  oled_control.Display_ON = Display_ON;
  oled_control.Display_OFF = Display_OFF;
  oled_control.DrawPoint = DrawPoint;
  oled_control.fill = fill;
  oled_control.DrawLine = DrawLine;
  oled_control.DrawRectangle = DrawRectangle;
  oled_control.DrawCircle = DrawCircle;
}
static char oled_init(void)
{

  HAL_Delay(200); // 等待

  char buff[23] = {0xAE, // 关显示

                   0x10, //
                   0xB0,
                   0xC8,
                   0x81, // 设置对比度
                   0xff,
                   0xa1,
                   0xa6,
                   0xa8, // 设置驱动路数
                   0x1f,
                   0xd3,
                   0x00,
                   0xd5,
                   0xf0,
                   0xd9,
                   0x22,
                   0xda,
                   0x02,
                   0xdb,
                   0x49,
                   0x8d,
                   0x14,
                   0xaf};

  oled_control.Write(OLED_CMD, buff, 23);

  oled_control.fill(0, 0, 128, 32, 0);

  return 1;
}

void Oled_show_tack(void const *argument)
{

  while (1)
  {

    osDelay(10);
  }
}
/**************旋转立方体******************/
#define ANGLE_STEP 0.08
typedef struct
{
  float x, y, z;
} Vector3f;
// 顶点坐标
const Vector3f vertices[8] = {
    {-0.5, -0.5, -0.5},
    {-0.5, 0.5, -0.5},
    {0.5, 0.5, -0.5},
    {0.5, -0.5, -0.5},
    {-0.5, -0.5, 0.5},
    {-0.5, 0.5, 0.5},
    {0.5, 0.5, 0.5},
    {0.5, -0.5, 0.5}};
// 面索引
const int indices[12][2] = {
    {0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6}, {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};

/**
 * @brief 向量旋转
 *
 * @param v
 * @param angle
 * @param axis
 */
static void rotate(Vector3f *v, float angle, char axis)
{
  float s = arm_sin_f32(angle);
  float c = arm_cos_f32(angle);

  switch (axis)
  {
  case 'x':
  {
    float y = v->y * c - v->z * s;
    float z = v->y * s + v->z * c;
    v->y = y;
    v->z = z;
  }
  break;

  case 'y':
  {
    float x = v->x * c + v->z * s;
    float z = -v->x * s + v->z * c;
    v->x = x;
    v->z = z;
  }
  break;

  case 'z':
  {
    float x = v->x * c - v->y * s;
    float y = v->x * s + v->y * c;
    v->x = x;
    v->y = y;
  }
  break;
  }
}
/**
 * @brief OLED更新显示任务
 *
 * @param argument 任务参数
 */
void Oled_Refresh_tack(void const *argument)
{
  oled_init();
  float angle_x = 0.0, angle_y = 0.0, angle_z = 0.0;
  Vector3f rotated_vertices[8];
  while (1)
  {
    /**/
    oled_control.fill(0, 0, 128, 32, 0);
    for (int i = 0; i < 8; i++)
    {
      rotated_vertices[i] = vertices[i];
      rotate(&rotated_vertices[i], angle_x, 'x');
      rotate(&rotated_vertices[i], angle_y, 'y');
      rotate(&rotated_vertices[i], angle_z, 'z');
    }
    for (int i = 0; i < 12; i++)
    {
      Vector3f v1 = rotated_vertices[indices[i][0]];
      Vector3f v2 = rotated_vertices[indices[i][1]];
      oled_control.DrawLine((char)(32 + v1.x * 16), (char)(16 + v1.y * 16), (char)(32 + v2.x * 16), (char)(16 + v2.y * 16), 1);
    }
    // 更新角度
    angle_x += (float)ANGLE_STEP;
    angle_y += (float)ANGLE_STEP;
    angle_z += (float)ANGLE_STEP;
    /**/
    oled_control.Refresh_Gram();
    osDelay(16);
  }
}
