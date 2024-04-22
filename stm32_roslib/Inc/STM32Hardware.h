/* 
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018, Kenta Yonekura (a.k.a. yoneken)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote prducts derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ROS_STM32_HARDWARE_H_
#define ROS_STM32_HARDWARE_H_

#define STM32F4xx  // Change for your device
#ifdef STM32F3xx
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_uart.h"
#endif /* STM32F3xx */
#ifdef STM32F4xx
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#endif /* STM32F4xx */
#ifdef STM32F7xx
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_uart.h"
#endif /* STM32F7xx */

/**
	@brief 回调函数使用(首先确保打开DMA回调和UART回调函数(HAL_UART_IRQHandler))
	只要使用本STM32Hardware.h文件，复制下面两个回调函数即可完成ROS通讯部分
	若使用

//接收回调
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	// 由于在STM32Hardware.h中默认rosserial相关串口为huart2，因此这样进行判断
	if(huart->Instance == USART2){
		//接收完全回调
		if(huart->RxXferSize == Size){
			nh.getHardware()->reset_rbuf(nh.getHardware()->rbuflen);
			nh.spinOnce();	//如果使用FreeRTOS可以选择把这行删去，通过任务通知的方式进行spinOnce()
		}
		// 判断UART IDLE中断是否关闭，从而判断是否由IDLE引起的中断
		else if(!__HAL_UART_GET_IT_SOURCE(huart,UART_IT_IDLE))
		{
			nh.getHardware()->reset_rbuf(huart->RxXferSize - Size);
			nh.spinOnce(); 	//如果使用FreeRTOS可以选择把这行删去，通过任务通知的方式进行spinOnce()
		}
	}
}

//发送回调
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	// 由于在STM32Hardware.h中默认rosserial相关串口为huart2，因此这样进行判断
	if(huart->Instance == USART2){
		nh.getHardware()->flush();
	}
}

**/

/* 设置rosserial相关串口，即与上位机通讯的串口，根据实际情况更改 */
/* 修改_huart指针所指通讯串口即可。例如修改为huart3，
   则改成
   extern UART_HandleTypeDef huart3;
   UART_HandleTypeDef *_huart= &huart3; 
   即可 */
extern UART_HandleTypeDef huart2;
UART_HandleTypeDef *_huart = &huart2;
class STM32Hardware {
	public:
		/* 接收缓冲大小，建议的最小缓冲长度为512，可根据实际情况更改 */
		const static uint16_t rbuflen = 2048;
  protected:
    UART_HandleTypeDef *huart;
    uint8_t rbuf[rbuflen];
    uint32_t rind;
    inline uint32_t getRdmaInd(void){ return (rbuflen - __HAL_DMA_GET_COUNTER(huart->hdmarx)) & (rbuflen - 1); }
		/* 发送缓冲大小，建议的最小缓冲长度为512，可根据实际情况更改 */
    const static uint16_t tbuflen = 2048;
    uint8_t tbuf[tbuflen];
    uint32_t twind, tfind;

  public:
    STM32Hardware():
      huart(_huart), rind(0), twind(0), tfind(0){
    }

    STM32Hardware(UART_HandleTypeDef *huart_):
      huart(huart_), rind(0), twind(0), tfind(0){
    }
  
    void init(){
      reset_rbuf(rbuflen);
    }
		
    void reset_rbuf(uint16_t rbufwlen){
			HAL_UARTEx_ReceiveToIdle_DMA(huart,&rbuf[rbuflen-rbufwlen],rbufwlen);
    }

    int read(){
      int c = -1;
      if(rind != getRdmaInd()){
        c = rbuf[rind++];
        rind &= rbuflen - 1;
      }
      return c;
    }
		
    void flush(void){
      static bool mutex = false;

      if((huart->gState == HAL_UART_STATE_READY) && !mutex){
        mutex = true;

        if(twind != tfind){
          uint16_t len = tfind < twind ? twind - tfind : tbuflen - tfind;
          HAL_UART_Transmit_DMA(huart, &(tbuf[tfind]), len);
          tfind = (tfind + len) & (tbuflen - 1);
        }
        mutex = false;
      }
    }

    void write(uint8_t* data, int length){
      int n = length;
      n = n <= tbuflen ? n : tbuflen;

      int n_tail = n <= tbuflen - twind ? n : tbuflen - twind;
      memcpy(&(tbuf[twind]), data, n_tail);
      twind = (twind + n) & (tbuflen - 1);

      if(n != n_tail){
        memcpy(tbuf, &(data[n_tail]), n - n_tail);
      }

      flush();
    }

    unsigned long time(){ return HAL_GetTick();; }

  protected:
};

#endif
