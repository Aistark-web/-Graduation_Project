/**
 * @file kalman.h
 * @author Liu heng
 * @date 27-August-2013
 * @brief һ�׿������˲���ģ��
 * @details
 * ʵ�ֹ�����ȫ��Ӳ���޹أ���ֱ�ӵ��ã�������ֲ��
 * ʹ��ʱ�ȶ���һ��kalman����Ȼ�����kalmanCreate()����һ���˲���
 * ÿ�ζ�ȡ�����������ݺ󼴿ɵ���KalmanFilter()�������ݽ����˲���
 *
 * kalman p;
 * float SersorData;
 * kalmanCreate(&p,20,200);
 * while(1)
 * {
 * SersorData = sersor();
 * SersorData = KalmanFilter(&p,SersorData);
 * printf("%2.2f",SersorData);
 * }
 *
	 * MPU6050�Ŀ������˲����ο����� Q��10 R��400
 */

#ifndef _KALMAN_H
#define _KALMAN_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief һ�׿������˲���
 */
typedef struct {
 float X_last; //��һʱ�̵����Ž��
 float X_mid; //��ǰʱ�̵�Ԥ����
 float X_now; //��ǰʱ�̵����Ž��
 float P_mid; //��ǰʱ��Ԥ������Э����
 float P_now; //��ǰʱ�����Ž����Э����
 float P_last; //��һʱ�����Ž����Э����
 float kg; //kalman����
 float A; //ϵͳ����
 float Q;
 float R;
 float H;
} kalman_filter_t;

/**
 * @brief ��ʼ��һ���������˲���
 * @param[out] p �˲���
 * @param[in] T_Q ϵͳ����Э����
 * @param[in] T_R ��������Э����
 */
void kalman_Init(kalman_filter_t *p, float T_Q, float T_R);

/**
 * @brief �������˲���
 * @param[in] p �˲���
 * @param[in] dat ���˲��ź�
 * @retval �˲�����ź�
 */
float Kalman_Filter(kalman_filter_t *p, float dat);

#ifdef __cplusplus
}
#endif

#endif