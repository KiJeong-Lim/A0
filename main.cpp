#include "capstone.hpp"

#define CAN_ID 0x01

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

CANMessage *txMsg[6] = { &txMsg1, &txMsg2, &txMsg3, &txMsg4, &txMsg5, &txMsg6, };

void pack_cmd(CANMessage &msg, float p_des, float v_des, float kp, float kd, float t_ff)
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
    UCh8 pack = { p_int>>8, p_int&0xFF, v_int>>4, ((v_int&0x0F)<<4)|(kp_int>>8), kp_int&0xFF, kd_int>>4, ((kd_int&0x0F)<<4)|(t_int>>8), t_int&0xFF, };
    for (int i = 0; i < 8; i++) {
        msg.data[i] = pack.data[i];
    }
}

void onMsgReceived1(void)
{
    can1.read(rxMsg1);
    int id = rxMsg1.data[0];
    int p_int = (rxMsg1.data[1]<<8)|rxMsg1.data[2];
    int v_int = (rxMsg1.data[3]<<4)|(rxMsg1.data[4]>>4);
    int i_int = ((rxMsg1.data[4]&0x0F)<<8)|rxMsg1.data[5];
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
    int i_int = ((rxMsg2.data[4]&0x0F)<<8)|rxMsg2.data[5];
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

bool operation(void)
{
    if (0 < x && x <= 99) {
        pack_cmd(txMsg1, 0, 0, 0, 0, 0);
        pack_cmd(txMsg2, 0, 0, 0, 0, 0);
        pack_cmd(txMsg3, 0.20, 0, 4, 3, 0);
        pack_cmd(txMsg4, 0, 0, 0, 0, 0);
        pack_cmd(txMsg5, 0, 0, 0, 0, 0);
        pack_cmd(txMsg6, 0.20, 0, 4, 3, 0);  
        return true;
    }
    if (99 < x && x <= 199) {
        pack_cmd(txMsg1, 0.1, 0, 18, 3.5, 0);
        pack_cmd(txMsg2, 0.115, 0, 18, 3.5, 0);
        pack_cmd(txMsg3, 0, 0, 15, 3, 0);
        pack_cmd(txMsg4, 0.1, 0, 18, 3.5, 0);
        pack_cmd(txMsg5, 0.115, 0, 18, 3.5, 0);
        pack_cmd(txMsg6, 0, 0, 15, 3, 0);    
        return true;
    }   
#if 0
    pack_cmd(txMsg1, 0, 0, 0, 0, 0);
    pack_cmd(txMsg2, 0, 0, 0, 0, 0);
    pack_cmd(txMsg3, 0, 0, 0, 0, 0);
    pack_cmd(txMsg4, 0, 0, 0, 0, 0);
    pack_cmd(txMsg5, 0, 0, 0, 0, 0);
    pack_cmd(txMsg6, 0, 0, 0, 0, 0);
#endif
    return false;
}

void serial_isr(void)
{
    if (x > 0) {
        const bool go_next = operation();
        if (go_next)
            x++;
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
                printf("\rtheta%d(%ld)=%f; omega%d(%ld)=%f;\n", i + 1, logger, theta[i], i + 1, logger, omega[i]);
            };
            obs = 0;
            if (logger > 0)
                logger++;
            printf("\n");
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
            for (int i = 0; i < 6; i++) {
                txMsg[i]->data[0] = 0xFF;
                txMsg[i]->data[1] = 0xFF;
                txMsg[i]->data[2] = 0xFF;
                txMsg[i]->data[3] = 0xFF;
                txMsg[i]->data[4] = 0xFF;
                txMsg[i]->data[5] = 0xFF;
                txMsg[i]->data[6] = 0xFF;
                txMsg[i]->data[7] = 0xFD;
            }
            printf("\n\rExiting motor mode\n\r");
            break;

        case 'm':
            for (int i = 0; i < 6; i++) {
                txMsg[i]->data[0] = 0xFF;
                txMsg[i]->data[1] = 0xFF;
                txMsg[i]->data[2] = 0xFF;
                txMsg[i]->data[3] = 0xFF;
                txMsg[i]->data[4] = 0xFF;
                txMsg[i]->data[5] = 0xFF;
                txMsg[i]->data[6] = 0xFF;
                txMsg[i]->data[7] = 0xFC;
            }
            can1.write(txMsg1);
            can1.write(txMsg2);
            can1.write(txMsg3);
            can2.write(txMsg4);
            can2.write(txMsg5);
            can2.write(txMsg6);
            pack_cmd(txMsg1, 0, 0, 0, 0, 0);
            pack_cmd(txMsg2, 0, 0, 0, 0, 0);
            pack_cmd(txMsg3, 0, 0, 0, 0, 0);
            pack_cmd(txMsg4, 0, 0, 0, 0, 0);
            pack_cmd(txMsg5, 0, 0, 0, 0, 0);
            pack_cmd(txMsg6, 0, 0, 0, 0, 0);
            can1.write(txMsg1);
            can1.write(txMsg2);
            can1.write(txMsg3);
            can2.write(txMsg4);
            can2.write(txMsg5);
            can2.write(txMsg6);
            printf("\n\rEntering motor mode\n\r");
            break;

        case 'z':
            for (int i = 0; i < 6; i++) {
                txMsg[i]->data[0] = 0xFF;
                txMsg[i]->data[1] = 0xFF;
                txMsg[i]->data[2] = 0xFF;
                txMsg[i]->data[3] = 0xFF;
                txMsg[i]->data[4] = 0xFF;
                txMsg[i]->data[5] = 0xFF;
                txMsg[i]->data[6] = 0xFF;
                txMsg[i]->data[7] = 0xFE;
            }
            can1.write(txMsg1);
            can1.write(txMsg2);
            can1.write(txMsg3);
            can2.write(txMsg4);
            can2.write(txMsg5);
            can2.write(txMsg6);
            printf("\n\rSet zero\n\r");
            break;

        case '1':
            txMsg1.data[0] = 0x7F;
            txMsg1.data[1] = 0xFF;
            txMsg1.data[2] = 0x7F;
            txMsg1.data[3] = 0xF0;
            txMsg1.data[4] = 0x00;
            txMsg1.data[5] = 0x00;
            txMsg1.data[6] = 0x07;
            txMsg1.data[7] = 0xFF;
            can1.write(txMsg1);
            printf("\n\r1st motor rest position\n\r");
            break;

        case '2':
            txMsg2.data[0] = 0x7F;
            txMsg2.data[1] = 0xFF;
            txMsg2.data[2] = 0x7F;
            txMsg2.data[3] = 0xF0;
            txMsg2.data[4] = 0x00;
            txMsg2.data[5] = 0x00;
            txMsg2.data[6] = 0x07;
            txMsg2.data[7] = 0xFF;
            
            can1.write(txMsg2);
            printf("\n\r2nd motor rest position\n\r");            
            break;

        case '3':
            txMsg3.data[0] = 0x7F;
            txMsg3.data[1] = 0xFF;
            txMsg3.data[2] = 0x7F;
            txMsg3.data[3] = 0xF0;
            txMsg3.data[4] = 0x00;
            txMsg3.data[5] = 0x00;
            txMsg3.data[6] = 0x07;
            txMsg3.data[7] = 0xFF;
            
            can1.write(txMsg3);
            printf("\n\r3rd motor rest position\n\r");
            break;

        case '4':
            txMsg4.data[0] = 0x7F;
            txMsg4.data[1] = 0xFF;
            txMsg4.data[2] = 0x7F;
            txMsg4.data[3] = 0xF0;
            txMsg4.data[4] = 0x00;
            txMsg4.data[5] = 0x00;
            txMsg4.data[6] = 0x07;
            txMsg4.data[7] = 0xFF;
        
            can2.write(txMsg4);
            printf("\n\r4th motor rest position\n\r");
            break;

        case '5':
            txMsg5.data[0] = 0x7F;
            txMsg5.data[1] = 0xFF;
            txMsg5.data[2] = 0x7F;
            txMsg5.data[3] = 0xF0;
            txMsg5.data[4] = 0x00;
            txMsg5.data[5] = 0x00;
            txMsg5.data[6] = 0x07;
            txMsg5.data[7] = 0xFF;
            
            can2.write(txMsg5);
            printf("\n\r5th motor rest position\n\r");
            break;

        case '6':
            txMsg6.data[0] = 0x7F;
            txMsg6.data[1] = 0xFF;
            txMsg6.data[2] = 0x7F;
            txMsg6.data[3] = 0xF0;
            txMsg6.data[4] = 0x00;
            txMsg6.data[5] = 0x00;
            txMsg6.data[6] = 0x07;
            txMsg6.data[7] = 0xFF;
            
            can2.write(txMsg6);
            printf("\n\r6th motor rest position\n\r");
            break;

        case 'r':
#if 0
            x = 1;
#else
            x = 0;
#endif
            obs = 0;
            logger = 1;

            printf("\n\rRun\n\r");
            break;

        case 'o':
            x = 0;
            obs = 0;
            logger = 0;
            printf("\n\rObserve\n\r");
            break;

        case 'b':
            pack_cmd(txMsg1, 0, 0, 0, 0, 0);
            pack_cmd(txMsg2, 0, 0, 0, 0, 0);
            pack_cmd(txMsg3, 0, 0, 0, 0, 0);
            pack_cmd(txMsg4, 0, 0, 0, 0, 0);
            pack_cmd(txMsg5, 0, 0, 0, 0, 0);
            pack_cmd(txMsg6, 0, 0, 0, 0, 0);
            can1.write(txMsg1);
            can1.write(txMsg2);
            can1.write(txMsg3);
            can2.write(txMsg4);
            can2.write(txMsg5);
            can2.write(txMsg6);
            x = 0;
            obs = -1;
            logger = 0;
            printf("\n\rBreak\n\r");
            break;

        case ' ':
            for (int i = 0; i < 6; i++) {
                txMsg[i]->data[0] = 0xFF;
                txMsg[i]->data[1] = 0xFF;
                txMsg[i]->data[2] = 0xFF;
                txMsg[i]->data[3] = 0xFF;
                txMsg[i]->data[4] = 0xFF;
                txMsg[i]->data[5] = 0xFF;
                txMsg[i]->data[6] = 0xFF;
                txMsg[i]->data[7] = 0xFD;
            }
            can1.write(txMsg1);
            can1.write(txMsg2);
            can1.write(txMsg3);
            can2.write(txMsg4);
            can2.write(txMsg5);
            can2.write(txMsg6);
            pack_cmd(txMsg1, 0, 0, 0, 0, 0);
            pack_cmd(txMsg2, 0, 0, 0, 0, 0);
            pack_cmd(txMsg3, 0, 0, 0, 0, 0);
            pack_cmd(txMsg4, 0, 0, 0, 0, 0);
            pack_cmd(txMsg5, 0, 0, 0, 0, 0);
            pack_cmd(txMsg6, 0, 0, 0, 0, 0);
            x = 0;
            obs = -1;
            logger = 0;
            
            can1.write(txMsg1);
            can1.write(txMsg2);
            can1.write(txMsg3);
            can2.write(txMsg4);
            can2.write(txMsg5); 
            can2.write(txMsg6);
            printf("\n\rEmergency stop\n\r");
            break;
        }
    }
}

int main(void)
{
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
    can1.attach(onMsgReceived1);
    can1.filter(CAN_ID<<21, 0xFFE00004, CANStandard, 0);
    can2.frequency(1000000);
    can2.attach(onMsgReceived2);
    can2.filter(CAN_ID<<21, 0xFFE00004, CANStandard, 0);
    sendCAN.attach(serial_isr, 0.01);     
    pack_cmd(txMsg1, 0, 0, 0, 0, 0);
    pack_cmd(txMsg2, 0, 0, 0, 0, 0);
    pack_cmd(txMsg3, 0, 0, 0, 0, 0);
    pack_cmd(txMsg4, 0, 0, 0, 0, 0);
    pack_cmd(txMsg5, 0, 0, 0, 0, 0);
    pack_cmd(txMsg6, 0, 0, 0, 0, 0);
    for (int i = 0; i < 6; i++) {
        theta[i] = 0.0f;
        omega[i] = 0.0f;
    }
    can1.write(txMsg1);
    can1.write(txMsg2);
    can1.write(txMsg3);
    can2.write(txMsg4);
    can2.write(txMsg5);
    can2.write(txMsg6);
    x = 0;
    obs = -1;
    logger = 0;
    timer.start();
    printf("\n\rINIT\n\r");
}
