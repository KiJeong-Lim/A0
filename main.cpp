#include "capstone.hpp"
#include "mbed2/299/drivers/CAN.h"

#define P_MIN   (-12.5f)
#define P_MAX   (12.5f)
#define V_MIN   (-45.0f)
#define V_MAX   (45.0f)
#define KP_MIN  (0.0f)
#define KP_MAX  (500.0f)
#define KD_MIN  (0.0f)
#define KD_MAX  (5.0f)
#define T_MIN   (-18.0f)
#define T_MAX   (18.0f)
#define I_MAX   (40.0f)

Serial      pc(PA_2, PA_3);

CAN         can1(PB_8, PB_9);
CAN         can2(PB_5, PB_6);

CANMessage  rxMsg1;
CANMessage  rxMsg2;
CANMessage  txMsg1;
CANMessage  txMsg2;
CANMessage  txMsg3;
CANMessage  txMsg4;
CANMessage  txMsg5;
CANMessage  txMsg6;

Timer       timer;
Ticker      sendCAN;

long int    x = 0;
int         obs = -1;
long int    logger = 0;

float theta[6], omega[6];

void pack_cmd(CANMessage *msg, float p_des, float v_des, float kp, float kd, float t_ff)
{
    p_des = fminf(fmaxf(P_MIN, p_des), P_MAX);                    
    v_des = fminf(fmaxf(V_MIN, v_des), V_MAX);
    kp = fminf(fmaxf(KP_MIN, kp), KP_MAX);
    kd = fminf(fmaxf(KD_MIN, kd), KD_MAX);
    t_ff = fminf(fmaxf(T_MIN, t_ff), T_MAX);
    int p_int  = float_to_uint(p_des, P_MIN, P_MAX, 16);            
    int v_int  = float_to_uint(v_des, V_MIN, V_MAX, 12);
    int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
    int t_int  = float_to_uint(t_ff, T_MIN, T_MAX, 12);
    msg->data[0] = p_int>>8;                                       
    msg->data[1] = p_int&0xFF;
    msg->data[2] = v_int>>4;
    msg->data[3] = ((v_int&0xF)<<4)|(kp_int>>8);
    msg->data[4] = kp_int&0xFF;
    msg->data[5] = kd_int>>4;
    msg->data[6] = ((kd_int&0xF)<<4)|(t_int>>8);
    msg->data[7] = t_int&0xff;
}

void onMsgReceived1(void)
{
    can1.read(rxMsg1);
    int id = rxMsg1.data[0];
    int p_int = (rxMsg1.data[1]<<8)|rxMsg1.data[2];
    int v_int = (rxMsg1.data[3]<<4)|(rxMsg1.data[4]>>4);
    int i_int = ((rxMsg1.data[4]&0xF)<<8)|rxMsg1.data[5];
    float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
    float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
    float i = uint_to_float(i_int, -I_MAX, I_MAX, 12);
    switch (id) {
    case 1:
        theta[0] = p;
        omega[0] = v;
        break;
    case 2:
        theta[1] = p;
        omega[1] = v;
        break;
    case 3:
        theta[2] = p;
        omega[2] = v;
        break;
    }
}

void onMsgReceived2(void)
{
    can2.read(rxMsg2);
    int id = rxMsg2.data[0];
    int p_int = (rxMsg2.data[1]<<8)|rxMsg2.data[2];
    int v_int = (rxMsg2.data[3]<<4)|(rxMsg2.data[4]>>4);
    int i_int = ((rxMsg2.data[4]&0xF)<<8)|rxMsg2.data[5];
    float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
    float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
    float i = uint_to_float(i_int, -I_MAX, I_MAX, 12);
    switch (id) {
    case 4:
        theta[3] = p;
        omega[3] = v;
        break;
    case 5:
        theta[4] = p;
        omega[4] = v;
        break;
    case 6:
        theta[5] = p;
        omega[5] = v;
        break;
    }
}

void serial_isr(void)
{
    if (x > 0) {
        if(0 < x && x < 99) {
            pack_cmd(&txMsg1, 0, 0, 0, 0, 0);
            pack_cmd(&txMsg2, 0, 0, 0, 0, 0);
            pack_cmd(&txMsg3, -0.20, 0, 4, 3, 0);    
            x++;
        }
        else if(99 < x && x < 199) {
            pack_cmd(&txMsg1, -0.1, 0, 18, 3.5, 0);
            pack_cmd(&txMsg2, -0.115, 0, 18, 3.5, 0);
            pack_cmd(&txMsg3, 0, 0, 15, 3, 0);
            x++;
        }
    }

    can1.write(txMsg1);
    can1.write(txMsg2);
    can1.write(txMsg3);
    can2.write(txMsg4);
    can2.write(txMsg5);
    can2.write(txMsg6);

    if (obs >= 0) {
        if ((obs + 1) % 20 == 0) {
            for (int i = 0; i < 6; i++) {
                printf("\rtheta%d(%ld)=%f; omega%d(%ld)=%f;\n", i, logger, theta[i], i, logger, omega[i]);
            }
            obs = 0;
            if (logger > 0)
                logger++;
        }
        else
            obs++;
    }
}

void command(void)
{
    while(pc.readable()) {
        const char c = pc.getc();
        switch (c) {
        case 27:
            printf("\n\r Exiting motor mode \n\r");
            txMsg1.data[0] = 0xFF;
            txMsg1.data[1] = 0xFF;
            txMsg1.data[2] = 0xFF;
            txMsg1.data[3] = 0xFF;
            txMsg1.data[4] = 0xFF;
            txMsg1.data[5] = 0xFF;
            txMsg1.data[6] = 0xFF;
            txMsg1.data[7] = 0xFD;
            txMsg2.data[0] = 0xFF;
            txMsg2.data[1] = 0xFF;
            txMsg2.data[2] = 0xFF;
            txMsg2.data[3] = 0xFF;
            txMsg2.data[4] = 0xFF;
            txMsg2.data[5] = 0xFF;
            txMsg2.data[6] = 0xFF;
            txMsg2.data[7] = 0xFD;
            txMsg3.data[0] = 0xFF;
            txMsg3.data[1] = 0xFF;
            txMsg3.data[2] = 0xFF;
            txMsg3.data[3] = 0xFF;
            txMsg3.data[4] = 0xFF;
            txMsg3.data[5] = 0xFF;
            txMsg3.data[6] = 0xFF;
            txMsg3.data[7] = 0xFD;
            txMsg4.data[0] = 0xFF;
            txMsg4.data[1] = 0xFF;
            txMsg4.data[2] = 0xFF;
            txMsg4.data[3] = 0xFF;
            txMsg4.data[4] = 0xFF;
            txMsg4.data[5] = 0xFF;
            txMsg4.data[6] = 0xFF;
            txMsg4.data[7] = 0xFD;
            txMsg5.data[0] = 0xFF;
            txMsg5.data[1] = 0xFF;
            txMsg5.data[2] = 0xFF;
            txMsg5.data[3] = 0xFF;
            txMsg5.data[4] = 0xFF;
            txMsg5.data[5] = 0xFF;
            txMsg5.data[6] = 0xFF;
            txMsg5.data[7] = 0xFD;
            txMsg6.data[0] = 0xFF;
            txMsg6.data[1] = 0xFF;
            txMsg6.data[2] = 0xFF;
            txMsg6.data[3] = 0xFF;
            txMsg6.data[4] = 0xFF;
            txMsg6.data[5] = 0xFF;
            txMsg6.data[6] = 0xFF;
            txMsg6.data[7] = 0xFD;
            break;

        case 'm':
            printf("\n\r Entering motor mode \n\r");
            txMsg1.data[0] = 0xFF;
            txMsg1.data[1] = 0xFF;
            txMsg1.data[2] = 0xFF;
            txMsg1.data[3] = 0xFF;
            txMsg1.data[4] = 0xFF;
            txMsg1.data[5] = 0xFF;
            txMsg1.data[6] = 0xFF;
            txMsg1.data[7] = 0xFC;
            txMsg2.data[0] = 0xFF;
            txMsg2.data[1] = 0xFF;
            txMsg2.data[2] = 0xFF;
            txMsg2.data[3] = 0xFF;
            txMsg2.data[4] = 0xFF;
            txMsg2.data[5] = 0xFF;
            txMsg2.data[6] = 0xFF;
            txMsg2.data[7] = 0xFC;
            txMsg3.data[0] = 0xFF;
            txMsg3.data[1] = 0xFF;
            txMsg3.data[2] = 0xFF;
            txMsg3.data[3] = 0xFF;
            txMsg3.data[4] = 0xFF;
            txMsg3.data[5] = 0xFF;
            txMsg3.data[6] = 0xFF;
            txMsg3.data[7] = 0xFC;
            txMsg4.data[0] = 0xFF;
            txMsg4.data[1] = 0xFF;
            txMsg4.data[2] = 0xFF;
            txMsg4.data[3] = 0xFF;
            txMsg4.data[4] = 0xFF;
            txMsg4.data[5] = 0xFF;
            txMsg4.data[6] = 0xFF;
            txMsg4.data[7] = 0xFC;
            txMsg5.data[0] = 0xFF;
            txMsg5.data[1] = 0xFF;
            txMsg5.data[2] = 0xFF;
            txMsg5.data[3] = 0xFF;
            txMsg5.data[4] = 0xFF;
            txMsg5.data[5] = 0xFF;
            txMsg5.data[6] = 0xFF;
            txMsg5.data[7] = 0xFC;
            txMsg6.data[0] = 0xFF;
            txMsg6.data[1] = 0xFF;
            txMsg6.data[2] = 0xFF;
            txMsg6.data[3] = 0xFF;
            txMsg6.data[4] = 0xFF;
            txMsg6.data[5] = 0xFF;
            txMsg6.data[6] = 0xFF;
            txMsg6.data[7] = 0xFC;
            break;

        case 'z':
            printf("\n\r Set zero \n\r");
            txMsg1.data[0] = 0xFF;
            txMsg1.data[1] = 0xFF;
            txMsg1.data[2] = 0xFF;
            txMsg1.data[3] = 0xFF;
            txMsg1.data[4] = 0xFF;
            txMsg1.data[5] = 0xFF;
            txMsg1.data[6] = 0xFF;
            txMsg1.data[7] = 0xFE;
            txMsg2.data[0] = 0xFF;
            txMsg2.data[1] = 0xFF;
            txMsg2.data[2] = 0xFF;
            txMsg2.data[3] = 0xFF;
            txMsg2.data[4] = 0xFF;
            txMsg2.data[5] = 0xFF;
            txMsg2.data[6] = 0xFF;
            txMsg2.data[7] = 0xFE;
            txMsg3.data[0] = 0xFF;
            txMsg3.data[1] = 0xFF;
            txMsg3.data[2] = 0xFF;
            txMsg3.data[3] = 0xFF;
            txMsg3.data[4] = 0xFF;
            txMsg3.data[5] = 0xFF;
            txMsg3.data[6] = 0xFF;
            txMsg3.data[7] = 0xFE;
            txMsg4.data[0] = 0xFF;
            txMsg4.data[1] = 0xFF;
            txMsg4.data[2] = 0xFF;
            txMsg4.data[3] = 0xFF;
            txMsg4.data[4] = 0xFF;
            txMsg4.data[5] = 0xFF;
            txMsg4.data[6] = 0xFF;
            txMsg4.data[7] = 0xFE;
            txMsg5.data[0] = 0xFF;
            txMsg5.data[1] = 0xFF;
            txMsg5.data[2] = 0xFF;
            txMsg5.data[3] = 0xFF;
            txMsg5.data[4] = 0xFF;
            txMsg5.data[5] = 0xFF;
            txMsg5.data[6] = 0xFF;
            txMsg5.data[7] = 0xFE;
            txMsg6.data[0] = 0xFF;
            txMsg6.data[1] = 0xFF;
            txMsg6.data[2] = 0xFF;
            txMsg6.data[3] = 0xFF;
            txMsg6.data[4] = 0xFF;
            txMsg6.data[5] = 0xFF;
            txMsg6.data[6] = 0xFF;
            txMsg6.data[7] = 0xFE;
            break;

        case '1':
            printf("\n\r 1st motor rest position \n\r");
            txMsg1.data[0] = 0x7F;
            txMsg1.data[1] = 0xFF;
            txMsg1.data[2] = 0x7F;
            txMsg1.data[3] = 0xf0;
            txMsg1.data[4] = 0x00;
            txMsg1.data[5] = 0x00;
            txMsg1.data[6] = 0x07;
            txMsg1.data[7] = 0xFF;
            break;

        case '2':
            printf("\n\r 2nd motor rest position \n\r");
            txMsg2.data[0] = 0x7F;
            txMsg2.data[1] = 0xFF;
            txMsg2.data[2] = 0x7F;
            txMsg2.data[3] = 0xF0;
            txMsg2.data[4] = 0x00;
            txMsg2.data[5] = 0x00;
            txMsg2.data[6] = 0x07;
            txMsg2.data[7] = 0xFF;               
            break;

        case '3':
            printf("\n\r 3rd motor rest position \n\r");
            txMsg3.data[0] = 0x7F;
            txMsg3.data[1] = 0xFF;
            txMsg3.data[2] = 0x7F;
            txMsg3.data[3] = 0xF0;
            txMsg3.data[4] = 0x00;
            txMsg3.data[5] = 0x00;
            txMsg3.data[6] = 0x07;
            txMsg3.data[7] = 0xFF;
            break;

        case '4':
            printf("\n\r 4th motor rest position \n\r");
            txMsg4.data[0] = 0x7F;
            txMsg4.data[1] = 0xFF;
            txMsg4.data[2] = 0x7F;
            txMsg4.data[3] = 0xF0;
            txMsg4.data[4] = 0x00;
            txMsg4.data[5] = 0x00;
            txMsg4.data[6] = 0x07;
            txMsg4.data[7] = 0xFF;
            break;

        case '5':
            printf("\n\r 5th motor rest position \n\r");
            txMsg5.data[0] = 0x7F;
            txMsg5.data[1] = 0xFF;
            txMsg5.data[2] = 0x7F;
            txMsg5.data[3] = 0xF0;
            txMsg5.data[4] = 0x00;
            txMsg5.data[5] = 0x00;
            txMsg5.data[6] = 0x07;
            txMsg5.data[7] = 0xFF;
            break;

        case '6':
            printf("\n\r 6th motor rest position \n\r");
            txMsg6.data[0] = 0x7F;
            txMsg6.data[1] = 0xFF;
            txMsg6.data[2] = 0x7F;
            txMsg6.data[3] = 0xF0;
            txMsg6.data[4] = 0x00;
            txMsg6.data[5] = 0x00;
            txMsg6.data[6] = 0x07;
            txMsg6.data[7] = 0xFF;
            break;

        case 'r':
            printf("\n\r run \n\r");
            x = 1;
            obs = 0;
            logger = 1;
            break;

        case 'o':
            printf("\n\r observe \n\r");
            x = 0;
            obs = 0;
            logger = 0;
            break;

        case 'b':
            pack_cmd(&txMsg1, 0, 0, 0, 0, 0);
            pack_cmd(&txMsg2, 0, 0, 0, 0, 0);
            pack_cmd(&txMsg3, 0, 0, 0, 0, 0);
            pack_cmd(&txMsg4, 0, 0, 0, 0, 0);
            pack_cmd(&txMsg5, 0, 0, 0, 0, 0);
            pack_cmd(&txMsg6, 0, 0, 0, 0, 0);
            can1.write(txMsg1);
            can1.write(txMsg2);
            can1.write(txMsg3);
            can2.write(txMsg4);
            can2.write(txMsg5);
            can2.write(txMsg6);
            x = 0;
            obs = -1;
            logger = 0;
            printf("\n\r break \n\r");
            break;
        }
    }
}

int main(void)
{
    printf("\n\r init \n\r");
    pc.baud(921600);
    pc.attach(&command);
    txMsg1.len = 8;
    txMsg2.len = 8;
    txMsg3.len = 8;
    txMsg4.len = 8;
    txMsg5.len = 8;
    txMsg6.len = 8;
    rxMsg1.len = 6;
    rxMsg2.len = 6;
    txMsg1.id = 1; 
    txMsg2.id = 2; 
    txMsg3.id = 3; 
    txMsg4.id = 4;
    txMsg5.id = 5;
    txMsg6.id = 6;
    can1.frequency(1000000);
    can1.attach(&onMsgReceived1);
    can1.filter(CAN_ID<<21, 0xFFE00004, CANStandard, 0);
    can2.frequency(1000000);
    can2.attach(&onMsgReceived2);
    can2.filter(CAN_ID<<21, 0xFFE00004, CANStandard, 0);
    sendCAN.attach(&serial_isr, 0.01);     
    pack_cmd(&txMsg1, 0, 0, 0, 0, 0);
    pack_cmd(&txMsg2, 0, 0, 0, 0, 0);
    pack_cmd(&txMsg3, 0, 0, 0, 0, 0);
    pack_cmd(&txMsg4, 0, 0, 0, 0, 0);
    pack_cmd(&txMsg5, 0, 0, 0, 0, 0);
    pack_cmd(&txMsg6, 0, 0, 0, 0, 0);
    timer.start();
}
