#include "xboxseries_joy/xboxseries_joy_controller.hpp"

void xboxseries_joy_controller::transfer(const sensor_msgs::Joy::ConstPtr &msg)
{
    this->LY = msg->axes[0];
    this->LX = msg->axes[1];
    this->RY = msg->axes[2];
    this->RX = msg->axes[3];
    static bool LT_first_get,RT_first_get = false;
    if(!RT_first_get){
        if((int16_t)(msg->axes[4]*32767.0f) == 0){
            this->RT = 1.0f;
        }
        else{
            this->RT = msg->axes[4];
            RT_first_get = true;
        }
    }
    else{
        this->RT = msg->axes[4];
    }

    if(!LT_first_get){
        if((int16_t)(msg->axes[5]*32767.0f) == 0){
            this->LT = 1.0f;
        }
        else{
            this->LT = msg->axes[5];
            LT_first_get = true;
        }
    }
    else{
        this->LT = msg->axes[5];
    }
    this->DIR_Y = (int8_t)msg->axes[6];
    this->DIR_X = (int8_t)msg->axes[7];
    this->A = msg->buttons[0];
    this->B = msg->buttons[1];
    this->X = msg->buttons[3];
    this->Y = msg->buttons[4];
    this->LB = msg->buttons[6];
    this->RB = msg->buttons[7];
    this->VIEW = msg->buttons[10];
    this->MENU = msg->buttons[11];
    this->XBOX = msg->buttons[12];
    this->LS = msg->buttons[13];
    this->RS = msg->buttons[14];
}