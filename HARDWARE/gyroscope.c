#include "gyroscope.h"

/*
@ link : http://wit-motion.cn

@ Function:
1. Power on automatic detection sensor
2. Read acceleration, angular velocity, angle and magnetic field data
3. Set switching baud rate parameters

USB-TTL                   STM32Core              		JY901s
VCC          -----           VCC        ----        	 VCC
TX           -----           RX1  (GPIOA_10)
RX           -----           TX1  (GPIOA_9)
GND          -----           GND    ----       			 GND
                             RX2  (GPIOA_3)  ----        TX
                             TX2  (GPIOA_2)  ----        RX
------------------------------------
*/

float fAcc[3], fGyro[3], fAngle[3];        // ����float����Ϊ������������׼��
int   i;

static volatile char s_cDataUpdate = 0, s_cCmd = 0xff;
const uint32_t       c_uiBaud[10] = {0, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};

void Gyro_read(void) {
    if (s_cDataUpdate)        // ��ע���ȡ���������ݻص�������Ա������и�ֵ
    {
        for (i = 0; i < 3; i++)        // forѭ����i����������ʱ����forѭ��
        {
            fAcc[i]   = sReg[AX + i] / 32768.0f * 16.0f;           // �㷨��ʽ
            fGyro[i]  = sReg[GX + i] / 32768.0f * 2000.0f;         // �㷨��ʽ
            fAngle[i] = sReg[Roll + i] / 32768.0f * 180.0f;        // �㷨��ʽ
        }
        if (s_cDataUpdate & ACC_UPDATE)        // �жϲ�Ϊ0��ִ����������
        {
            //				printf("acc:%.3f %.3f %.3f\r\n", fAcc[0], fAcc[1], fAcc[2]);//��ӡ��Ӧ�����ݳ���
            s_cDataUpdate &= ~ACC_UPDATE;        // s_cDataUpdate��~ACC_UPDATE�������ֵ��s_cDataUpdate
        }
        if (s_cDataUpdate & GYRO_UPDATE)        // ���¼��д���ͬ��
        {
            //				printf("gyro:%.3f %.3f %.3f\r\n", fGyro[0], fGyro[1], fGyro[2]);
            s_cDataUpdate &= ~GYRO_UPDATE;
        }
        if (s_cDataUpdate & ANGLE_UPDATE) {
            //				printf("angle:%.3f\r\n",fAngle[2]);
            s_cDataUpdate &= ~ANGLE_UPDATE;
        }
        if (s_cDataUpdate & MAG_UPDATE) {
            // printf("mag:%d %d %d\r\n", sReg[HX], sReg[HY], sReg[HZ]);
            s_cDataUpdate &= ~MAG_UPDATE;
        }
    }
}

void SensorUartSend(uint8_t *p_data, uint32_t uiSize)        // ���������ڷ���
{
    Uart2Send(p_data, uiSize);        // ����2����
}

void Delayms(uint16_t ucMs)        // ��ʱ����
{
    delay_ms(ucMs);
}

void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum)        // ��������������
{
    int i;
    for (i = 0; i < uiRegNum; i++) {
        switch (uiReg)        // �ж�uiReg��������ʲô������ѡ���Ӧ�Ĳ���
        {
            //            case AX:
            //            case AY:
        case AZ:
            s_cDataUpdate |= ACC_UPDATE;        // s_cDataUpdate������ACC_UPDATE�����������Ľ���ٰѽ����ֵ��s_cDataUpdate����s_cDataUpdate=s_cDataUpdate|��ACC_UPDATE
            break;
            //            case GX:
            //            case GY:
        case GZ:
            s_cDataUpdate |= GYRO_UPDATE;
            break;
            //            case HX:
            //            case HY:
        case HZ:
            s_cDataUpdate |= MAG_UPDATE;
            break;
            //            case Roll:
            //            case Pitch:
        case Yaw:
            s_cDataUpdate |= ANGLE_UPDATE;
            break;
        default:
            s_cDataUpdate |= READ_UPDATE;
            break;
        }
        uiReg++;
    }
}

void AutoScanSensor(void)        // ���ڲ����ʼ��
{
    int i, iRetry;

    for (i = 1; i < 10; i++)        // forѭ����i����������������ѭ��
    {
        Usart2Init(c_uiBaud[i]);        // ����2�����ʴ�С�����ѯ
        iRetry = 2;
        do        // do-while()ѭ�������ִ�����ж�
        {
            s_cDataUpdate = 0;
            WitReadReg(AX, 3);        // ���������е����β�
            delay_ms(100);            // ��ʱ100ms
            if (s_cDataUpdate != 0)        // �ж�s_cDataUpdate������0����ǲ�����0��ִ���������������
            {
                // printf("%d baud find sensor\r\n\r\n", c_uiBaud[i]);//��ӡ�ҵ��������ʹ�ӡ��Ӧ�Ĳ�����
                // ShowHelp();				//ִ�к�����ӡ����
                return;
            }
            iRetry--;        // �����Լ�
        } while (iRetry);        // while��Ϊ0��һֱִ��ѭ��������
    }
}
