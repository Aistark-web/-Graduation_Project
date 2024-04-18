#ifndef _XBOXSERIES_JOY_CONTROLLER_HPP
#define _XBOXSERIES_JOY_CONTROLLER_HPP

#include <cstdint>
#include "sensor_msgs/Joy.h"
class xboxseries_joy_controller{
public:
    float LX;                   // @brief 左摇杆X轴，初始为0，范围为[-1,1] 上为正，下为负
    float LY;                   // @brief 左摇杆Y轴，初始为0，范围为[-1,1] 左为正，右为负
    float RX;                   // @brief 右摇杆X轴，初始为0，范围为[-1,1] 上为正，下为负
    float RY;                   // @brief 右摇杆Y轴，初始为0，范围为[-1,1] 左为正，右为负
    float LT;                   // @brief 左扳机，初始为1，范围为[-1,1] 按下减小
    float RT;                   // @brief 右扳机，初始为1，范围为[-1,1] 按下减小
    int8_t DIR_X;               // @brief 方向键上下方向，初始为0，上为1，下为-1
    int8_t DIR_Y;               // @brief 方向键左右方向，初始为0，左为1，下为-1
    uint8_t LS :1;              // @brief 左摇杆键，初始为0，按下为1
    uint8_t RS :1;              // @brief 右摇杆键，初始为0，按下为1
    uint8_t LB :1;              // @brief 左缓冲键，初始为0，按下为1
    uint8_t RB :1;              // @brief 右缓冲键，初始为0，按下为1
    uint8_t A :1;               // @brief A键，初始为0，按下为1
    uint8_t B :1;               // @brief B键，初始为0，按下为1
    uint8_t X :1;               // @brief X键，初始为0，按下为1
    uint8_t Y :1;               // @brief Y键，初始为0，按下为1
    uint8_t VIEW :1;            // @brief VIEW键，初始为0，按下为1
    uint8_t MENU :1;            // @brief MENU键，初始为0，按下为1
    uint8_t XBOX :1;            // @brief XBOX键，初始为0，按下为1
    void transfer(const sensor_msgs::Joy::ConstPtr &msg);
};

#endif