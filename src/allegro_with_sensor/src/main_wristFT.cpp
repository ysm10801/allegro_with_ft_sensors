//
// 20141209: kcchang: changed window version to linux 

// myAllegroHand.cpp : Defines the entry point for the console application.
//
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <termios.h>  //_getch
#include <string.h>
#include <pthread.h>
#include "canAPI.h"
#include "rDeviceAllegroHandCANDef.h"
#include "RockScissorsPaper.h"
#include <BHand/BHand.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>

#define PEAKCAN (1)

#define TransF(x, y)   (x*256+y)/100.-300
#define TransT(x, y)   (x*256+y)/500.-50

typedef char    TCHAR;
#define _T(X)   X
#define _tcsicmp(x, y)   strcmp(x, y)

#define HAND_STOP 0
#define HAND_POSES 1
#define HAND_P_GAINS 2
#define HAND_D_GAINS 3 
#define GRAVITY_COMP 4
#define HAND_GRASP 5


// initial PD Gains

double kp[] = {
    500, 800, 900, 500,
    500, 800, 900, 500,
    500, 800, 900, 500,
    1000, 700, 600, 600
};
double kd[] = {
    25, 50, 55, 40,
    25, 50, 55, 40,
    25, 50, 55, 40,
    50, 50, 50, 40
};

using namespace std;
using namespace Eigen;

/////////////////////////////////////////////////////////////////////////////////////////
// for CAN communication
const double delT = 0.003;
int CAN_Ch = 0;
bool ioThreadRun = false;
pthread_t        hThread;
int recvNum = 0;
int sendNum = 0;
double statTime = -1.0;
AllegroHand_DeviceMemory_t vars;

double curTime = 0.0;

/////////////////////////////////////////////////////////////////////////////////////////
// for BHand library
BHand* pBHand = NULL;
double q[MAX_DOF];
double q_des[MAX_DOF];
double tau_des[MAX_DOF];
double cur_des[MAX_DOF];

// USER HAND CONFIGURATION
const bool	RIGHT_HAND = true;
const int	HAND_VERSION = 4;

const double tau_cov_const_v4 = 1200.0; // 1200.0 for SAH040xxxxx

const int FT_length = 6;

// FT_sensor_calibration
// double r_ee = 0.208;
// double r_s = 0.043;
double r_ee[3] = {0.0060, 0.0026, 0.07016};
double gripper_mass = 1.2004;

Matrix3d wRs = Matrix3d::Identity();
Matrix3d s_r_hat = Matrix3d::Identity();
Matrix3d eeRs = Matrix3d::Zero();
Vector3d gripper_grav = Vector3d::Zero();

class BilinearLPF {
public:
    BilinearLPF(double sampleRate, double cutoffFrequency) {
        this->sampleRate = sampleRate;
        this->cutoffFrequency = cutoffFrequency;
        for (int i=0 ; i<FT_length ; i++){
            prevInput1[i] = 0.0;
            prevOutput1[i] = 0.0;
        }
    }
    double* process(const double input[FT_length]) {
        static double output[FT_length];

        for (int i=0 ; i<FT_length ; i++){
            output[i] = (2 - cutoffFrequency / sampleRate) / (2 + cutoffFrequency / sampleRate) * prevOutput1[i]
                + (cutoffFrequency / sampleRate) / (2 + cutoffFrequency / sampleRate) * (input[i] + prevInput1[i]);
        }
        for (int i=0 ; i<FT_length ; i++){
            prevInput1[i] = input[i];
            prevOutput1[i] = output[i];
        }
        return output;
    }
private:
    double sampleRate;
    double cutoffFrequency;
    double prevInput1[FT_length];
    double prevOutput1[FT_length];
};

/////////////////////////////////////////////////////////////////////////////////////////
// Read keyboard input (one char) from stdin
char Getch()
{
    /*#include <unistd.h>   //_getch*/
    /*#include <termios.h>  //_getch*/
    char buf=0;
    struct termios old={0};
    fflush(stdout);
    if(tcgetattr(0, &old)<0)
        perror("tcsetattr()");
    old.c_lflag&=~ICANON;
    old.c_lflag&=~ECHO;
    old.c_cc[VMIN]=1;
    old.c_cc[VTIME]=0;
    if(tcsetattr(0, TCSANOW, &old)<0)
        perror("tcsetattr ICANON");
    if(read(0,&buf,1)<0)
        perror("read()");
    old.c_lflag|=ICANON;
    old.c_lflag|=ECHO;
    if(tcsetattr(0, TCSADRAIN, &old)<0)
        perror ("tcsetattr ~ICANON");
    printf("%c\n",buf);
    return buf;
}

// FT Sensor Value /////

double FT_calib[FT_length] = {0};
int calib_switch[2] = {0};
double FT[FT_length] = {0};
double FT_temp[FT_length] = {0};

double FT_filtered[FT_length] = {0};

BilinearLPF FT_lpf = BilinearLPF(200, 20);

bool bRun = true;
int c = 0;

/////////////////////////////////////////////////////////////////////////////////////////
// functions declarations
char Getch();
void PrintInstruction();
void MainLoop();
bool OpenCAN();
void CloseCAN();
int GetCANChannelIndex(const TCHAR* cname);
bool CreateBHandAlgorithm();
void DestroyBHandAlgorithm();
void ComputeTorque();

void joint_config_Callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    switch (msg->layout.data_offset)
    {
    case HAND_STOP:     // Force stop
        exit(0);
        break;

    case HAND_POSES:     // JOINT POSITIONS
        for (int i = 0; i < 16; i++)
        {
            q_des[i] = msg->data[i];
        }
        pBHand->SetMotionType(eMotionType_JOINT_PD);
	    pBHand->SetGainsEx(kp, kd);
        printf("Joint Configuration Moved\n");
        break;
    
    case HAND_P_GAINS:     // JOINT P gains
        for (int i = 0; i < 16; i++)
        {
            kp[i] = msg->data[i];
        }
        pBHand->SetMotionType(eMotionType_JOINT_PD);
	    pBHand->SetGainsEx(kp, kd);
        printf("P Gain Changed\n");
        break;
    
    case HAND_D_GAINS:     // JOINT D gains
        for (int i = 0; i < MAX_DOF; i++)
        {
            kd[i] = msg->data[i];
        }
        pBHand->SetMotionType(eMotionType_JOINT_PD);
	    pBHand->SetGainsEx(kp, kd);
        printf("D Gain Changed\n");
        break;
    
    case GRAVITY_COMP:
        pBHand->SetMotionType(eMotionType_GRAVITY_COMP);
        break;

    case HAND_GRASP:
        pBHand->SetMotionType(eMotionType_GRASP_4);
        break;

    default:
        break;
    }
    printf("\n");
}

void GetFTVal(int sens_num, unsigned char data[8])
{
    int s = sens_num*3;
    Matrix3d sRw = wRs.transpose();
    gripper_grav << 0, 0, -gripper_mass*9.81;

    Vector3d s_gripper_force = sRw * gripper_grav;
    Vector3d s_gripper_moment = s_r_hat*sRw * gripper_grav;

    for(int i=0 ; i<3 ; i++){
        if (sens_num==0){
            switch(i){
                case 0:
                    FT_temp[i+s]=TransF(data[i*2], data[i*2+1]) - s_gripper_force.x();
                    break;
                case 1:
                    FT_temp[i+s]=TransF(data[i*2], data[i*2+1]) - s_gripper_force.y();
                    break;
                case 2:
                    FT_temp[i+s]=TransF(data[i*2], data[i*2+1]) - s_gripper_force.z();
                    break;
            }
            // FT_temp[i+s]=TransF(data[i*2], data[i*2+1]);
        }
        else{
            switch(i){
                case 0:
                    FT_temp[i+s]=TransT(data[i*2], data[i*2+1]) - s_gripper_moment.x();
                    break;
                case 1:
                    FT_temp[i+s]=TransT(data[i*2], data[i*2+1]) - s_gripper_moment.y();
                    break;
                case 2:
                    FT_temp[i+s]=TransT(data[i*2], data[i*2+1]) - s_gripper_moment.z();
                    break;
            }
            // FT_temp[i+s]=TransT(data[i*2], data[i*2+1]);
        }
    }
    for(int i=0 ; i<3 ; i++){
        FT[i+s]=FT_temp[i+s]-FT_calib[i+s];
    }
    if(calib_switch[sens_num]<400 && s_gripper_force.x()!=0 ){
        if (calib_switch[sens_num]>=200){
            for(int k = s ; k<s+3 ; k++){
                printf("s_gripper_moment: %f, %f, %f\n", s_gripper_moment.x(), s_gripper_moment.y(), s_gripper_moment.z());
                printf("s_gripper_force: %f, %f, %f\n", s_gripper_force.x(), s_gripper_force.y(), s_gripper_force.z());

                FT_calib[k]+=FT_temp[k]/200;
            }
        }
        calib_switch[sens_num]++;
    }

    double* filtered_output = FT_lpf.process(FT);

    for (int i=0 ; i<FT_length ; i++){
        FT_filtered[i] = filtered_output[i];
    }
}

void ee_pose_update_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    double qx = msg->pose.orientation.x;
    double qy = msg->pose.orientation.y;
    double qz = msg->pose.orientation.z;
    double qw = msg->pose.orientation.w;

    Matrix3d wRee = Quaterniond(qw, qx, qy, qz).toRotationMatrix();
    // printf("wRee's diagonal: %f, %f, %f\n", wRee(0,0), wRee(1,1), wRee(2,2));
    
    s_r_hat << 0,       -r_ee[2],    r_ee[1],
               r_ee[2],        0,   -r_ee[0],
              -r_ee[1],  r_ee[0],         0;

    eeRs << 1,  0,  0,
            0,  1,  0,
            0,  0,  1;

    wRs = wRee * eeRs;

}
/////////////////////////////////////////////////////////////////////////////////////////
// CAN communication thread
static void* ioThreadProc(void* inst)
{
    int id;
    int len;
    unsigned char data[8];
    unsigned char data_return = 0;
    int i;

    // publisher and subscriber
    ros::NodeHandle nh;
    ros::Publisher ft_pub = nh.advertise<std_msgs::Float64MultiArray>("/ft_sensor_value", 7);
    ros::Publisher ft_filtered_pub = nh.advertise<std_msgs::Float64MultiArray>("/ft_sensor_filtered_value", 7);
    ros::Publisher allegro_config_pub = nh.advertise<std_msgs::Float64MultiArray>("/allegro_joint_configurations", 7);
    ros::Publisher allegro_torque_pub = nh.advertise<std_msgs::Float64MultiArray>("/allegro_joint_torques", 7);
    std_msgs::Float64MultiArray ft_value_msg;
    std_msgs::Float64MultiArray ft_filtered_value_msg;
    std_msgs::Float64MultiArray allegro_config_msg;
    std_msgs::Float64MultiArray allegro_torque_msg;
    while (ioThreadRun)
    {
        /* wait for the event */
        while (0 == get_message(CAN_Ch, &id, &len, data, FALSE))
        {
//////////////////////////FT_sensor_publish///////////////////////
            ft_value_msg.data.clear();
        
            for (size_t i = 0; i < FT_length; i++)
            {
                ft_value_msg.data.push_back(FT[i]);
            }
            ft_pub.publish(ft_value_msg);
            
            ft_filtered_value_msg.data.clear();
            for (size_t i = 0; i < FT_length; i++)
            {
                ft_filtered_value_msg.data.push_back(FT_filtered[i]);
            }
            ft_filtered_pub.publish(ft_filtered_value_msg);
//            printf(">CAN(%d): ", CAN_Ch);
//            for(int nd=0; nd<len; nd++)
//                printf("%02x ", data[nd]);
//            printf("\n");

/////////////////////////////////FT Sensor Value Prints//////////////////////////////////
            switch(id)
            {
                case 1:
                    GetFTVal(0, data);
                    break;
                case 2:
                    GetFTVal(1, data);
                    break;
                default:

/////////////////////////////////Allegro Hand Control////////////////////////////////////

                    id = (id & 0xfffffffc) >> 2;
                    switch (id)
                    {
                        case ID_RTR_HAND_INFO:
                        {
                            printf(">CAN(%d): AllegroHand hardware version: 0x%02x%02x\n", CAN_Ch, data[1], data[0]);
                            printf("                      firmware version: 0x%02x%02x\n", data[3], data[2]);
                            printf("                      hardware type: %d(%s)\n", data[4], (data[4] == 0 ? "right" : "left"));
                            printf("                      temperature: %d (celsius)\n", data[5]);
                            printf("                      status: 0x%02x\n", data[6]);
                            printf("                      servo status: %s\n", (data[6] & 0x01 ? "ON" : "OFF"));
                            printf("                      high temperature fault: %s\n", (data[6] & 0x02 ? "ON" : "OFF"));
                            printf("                      internal communication fault: %s\n", (data[6] & 0x04 ? "ON" : "OFF"));
                        }
                            break;
                        case ID_RTR_SERIAL:
                        {
                            printf(">CAN(%d): AllegroHand serial number: SAH0%d0 %c%c%c%c%c%c%c%c\n", CAN_Ch, HAND_VERSION
                                , data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
                            printf("DON'T PUSH ANY KEYS(FORCE STOP UNAVAILABLE WITH CTRL+C)\n");
                        }
                            break;
                        case ID_RTR_FINGER_POSE_1:
                        case ID_RTR_FINGER_POSE_2:
                        case ID_RTR_FINGER_POSE_3:
                        case ID_RTR_FINGER_POSE_4:
                        {
                            int findex = (id & 0x00000007);

                            vars.enc_actual[findex*4 + 0] = (short)(data[0] | (data[1] << 8));
                            vars.enc_actual[findex*4 + 1] = (short)(data[2] | (data[3] << 8));
                            vars.enc_actual[findex*4 + 2] = (short)(data[4] | (data[5] << 8));
                            vars.enc_actual[findex*4 + 3] = (short)(data[6] | (data[7] << 8));
                            data_return |= (0x01 << (findex));
                            recvNum++;

            //                printf(">CAN(%d): e[%d] Count : %6d %6d %6d %6d\n"
            //                    , CAN_Ch, findex
            //                    , vars.enc_actual[findex*4 + 0], vars.enc_actual[findex*4 + 1]
            //                    , vars.enc_actual[findex*4 + 2], vars.enc_actual[findex*4 + 3]);

                            if (data_return == (0x01 | 0x02 | 0x04 | 0x08))
                            {
                                // convert encoder count to joint angle and publish
                                for (i=0; i<MAX_DOF; i++)
                                {
                                    q[i] = (double)(vars.enc_actual[i])*(333.3/65536.0)*(3.141592/180.0);
                                }
                                allegro_config_msg.data.clear();

                                for (size_t i = 0; i < MAX_DOF; i++)
                                {
                                    allegro_config_msg.data.push_back(q[i]);
                                }
                                allegro_config_pub.publish(allegro_config_msg);

                                // // print joint angles
                                // for (int i=0; i<4; i++)
                                // {
                                //     printf(">CAN(%d): Joint[%d] Pos : %5.1f %5.1f %5.1f %5.1f\n"
                                //         , CAN_Ch, i, q[i*4+0]*RAD2DEG, q[i*4+1]*RAD2DEG, q[i*4+2]*RAD2DEG, q[i*4+3]*RAD2DEG);
                                // }

                                // compute joint torque
                                ComputeTorque();
                                
                                allegro_torque_msg.data.clear();

                                for (size_t i = 0; i < MAX_DOF; i++)
                                {
                                    allegro_torque_msg.data.push_back(tau_des[i]);
                                }
                                allegro_torque_pub.publish(allegro_torque_msg);

                                // convert desired torque to desired current and PWM count
                                for (int i=0; i<MAX_DOF; i++)
                                {
                                    cur_des[i] = tau_des[i];
                                    if (cur_des[i] > 1.0) cur_des[i] = 1.0;
                                    else if (cur_des[i] < -1.0) cur_des[i] = -1.0;
                                }

                                // send torques
                                for (int i=0; i<4;i++)
                                {
                                    vars.pwm_demand[i*4+0] = (short)(cur_des[i*4+0]*tau_cov_const_v4);
                                    vars.pwm_demand[i*4+1] = (short)(cur_des[i*4+1]*tau_cov_const_v4);
                                    vars.pwm_demand[i*4+2] = (short)(cur_des[i*4+2]*tau_cov_const_v4);
                                    vars.pwm_demand[i*4+3] = (short)(cur_des[i*4+3]*tau_cov_const_v4);

                                    command_set_torque(CAN_Ch, i, &vars.pwm_demand[4*i]);
                                    //usleep(5);
                                }
                                sendNum++;
                                curTime += delT;
                                data_return = 0;
                            }
                        }
                            break;
                        case ID_RTR_IMU_DATA:
                        {
                            printf(">CAN(%d): AHRS Roll : 0x%02x%02x\n", CAN_Ch, data[0], data[1]);
                            printf("               Pitch: 0x%02x%02x\n", data[2], data[3]);
                            printf("               Yaw  : 0x%02x%02x\n", data[4], data[5]);
                        }
                            break;
                        case ID_RTR_TEMPERATURE_1:
                        case ID_RTR_TEMPERATURE_2:
                        case ID_RTR_TEMPERATURE_3:
                        case ID_RTR_TEMPERATURE_4:
                        {
                            int sindex = (id & 0x00000007);
                            int celsius = (int)(data[0]      ) |
                                        (int)(data[1] << 8 ) |
                                        (int)(data[2] << 16) |
                                        (int)(data[3] << 24);
                            printf(">CAN(%d): Temperature[%d]: %d (celsius)\n", CAN_Ch, sindex, celsius);
                        }
                            break;
                        default:
                            continue;
                            // printf("");
                            /*for(int nd=0; nd<len; nd++)
                                printf("%d \n ", data[nd]);*/
                            //return;
                    }
            }
        }
    }
    return NULL;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Application main-loop. It handles the commands from rPanelManipulator and keyboard events
void MainLoop()
{
    while (bRun)
    {
        ros::NodeHandle nh_sub;
        ros::Subscriber allegro_config_sub = nh_sub.subscribe("/allegro_joint_desired", 100, joint_config_Callback);
        ros::Subscriber ee_pose_sub = nh_sub.subscribe("/ee_pose", 100, ee_pose_update_Callback);
        ros::Rate rate(1000);

        // c = Getch();
        c = 'q';
        switch (c)
        {
        case 'q':
            if (pBHand) pBHand->SetMotionType(eMotionType_NONE);
            bRun = false;
            break;
        }
//         case 'h':
//             if (pBHand) pBHand->SetMotionType(eMotionType_HOME);
//             break;

//         case 'r':
//             if (pBHand) pBHand->SetMotionType(eMotionType_READY);
//             break;

//         case 'g':
//             if (pBHand) pBHand->SetMotionType(eMotionType_GRASP_3);
//             break;

//         case 'k':
//             if (pBHand) pBHand->SetMotionType(eMotionType_GRASP_4);
//             break;

//         case 'p':
//             if (pBHand) pBHand->SetMotionType(eMotionType_PINCH_IT);
//             break;

//         case 'm':
//             if (pBHand) pBHand->SetMotionType(eMotionType_PINCH_MT);
//             break;

//         case 'a':
//             if (pBHand) pBHand->SetMotionType(eMotionType_GRAVITY_COMP);
//             break;

//         case 'e':
//             if (pBHand) pBHand->SetMotionType(eMotionType_ENVELOP);
//             break;

//         case 'f':
//             if (pBHand) pBHand->SetMotionType(eMotionType_NONE);
//             break;

//         case '1':
//             MotionRock();
//             break;

//         case '2':
//             MotionScissors();
//             break;

//         case '3':
//             MotionPaper();
//             break;
// /////////////////////motion_tests/////////////////////////
//         case '4':
//             // if (pBHand) pBHand->SetJointPosition();
//             break;
//         }
        ros::spin();
        // rate.sleep();
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
// Compute control torque for each joint using BHand library
void ComputeTorque()
{
    if (!pBHand) return;
    pBHand->SetJointPosition(q); // tell BHand library the current joint positions
    pBHand->SetJointDesiredPosition(q_des);
    pBHand->UpdateControl(0);
    pBHand->GetJointTorque(tau_des);

//    static int j_active[] = {
//        1, 1, 1, 1,
//        1, 1, 1, 1,
//        1, 1, 1, 1,
//        1, 1, 1, 1
//    };
//    for (int i=0; i<MAX_DOF; i++) {
//        if (j_active[i] == 0) {
//            tau_des[i] = 0;
//        }
//    }
}

/////////////////////////////////////////////////////////////////////////////////////////
// Open a CAN data channel
bool OpenCAN()
{
#if defined(PEAKCAN)
    CAN_Ch = GetCANChannelIndex(_T("USBBUS1"));
#elif defined(IXXATCAN)
    CAN_Ch = 1;
#elif defined(SOFTINGCAN)
    CAN_Ch = 1;
#else
    CAN_Ch = 1;
#endif
    printf(">CAN(%d): open\n", CAN_Ch);

    int ret = command_can_open(CAN_Ch);
    if(ret < 0)
    {
        printf("ERROR command_can_open !!! \n");
        return false;
    }

    //FT_calib_init
    for(int c;c<2;c++){
        calib_switch[c]=0;
    }

    // initialize CAN I/O thread
    ioThreadRun = true;
    /*int ioThread_error = */pthread_create(&hThread, NULL, ioThreadProc, 0);
    printf(">CAN: starts listening CAN frames\n");

    // query h/w information
    printf(">CAN: query system information\n");
    ret = request_hand_information(CAN_Ch);
    if(ret < 0)
    {
        printf("ERROR request_hand_information !!! \n");
        command_can_close(CAN_Ch);
        return false;
    }
    ret = request_hand_serial(CAN_Ch);
    if(ret < 0)
    {
        printf("ERROR request_hand_serial !!! \n");
        command_can_close(CAN_Ch);
        return false;
    }

    // set periodic communication parameters(period)
    printf(">CAN: Comm period set\n");
    short comm_period[3] = {3, 0, 0}; // millisecond {position, imu, temperature}
    ret = command_set_period(CAN_Ch, comm_period);
    if(ret < 0)
    {
        printf("ERROR command_set_period !!! \n");
        command_can_close(CAN_Ch);
        return false;
    }

    // servo on
    printf(">CAN: servo on\n");
    ret = command_servo_on(CAN_Ch);
    if(ret < 0)
    {
        printf("ERROR command_servo_on !!! \n");
        command_set_period(CAN_Ch, 0);
        command_can_close(CAN_Ch);
        return false;
    }

    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Close CAN data channel
void CloseCAN()
{
    printf(">CAN: stop periodic communication\n");
    int ret = command_set_period(CAN_Ch, 0);
    if(ret < 0)
    {
        printf("ERROR command_can_stop !!! \n");
    }

    if (ioThreadRun)
    {
        printf(">CAN: stoped listening CAN frames\n");
        ioThreadRun = false;
        int status;
        pthread_join(hThread, (void **)&status);
        hThread = 0;
    }

    printf(">CAN(%d): close\n", CAN_Ch);
    ret = command_can_close(CAN_Ch);
    if(ret < 0) printf("ERROR command_can_close !!! \n");
}

/////////////////////////////////////////////////////////////////////////////////////////
// Load and create grasping algorithm
bool CreateBHandAlgorithm()
{
    if (RIGHT_HAND)
        pBHand = bhCreateRightHand();
    else
        pBHand = bhCreateLeftHand();

    if (!pBHand) return false;
    pBHand->SetMotionType(eMotionType_NONE);
    pBHand->SetTimeInterval(delT);
    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Destroy grasping algorithm
void DestroyBHandAlgorithm()
{
    if (pBHand)
    {
#ifndef _DEBUG
        delete pBHand;
#endif
        pBHand = NULL;
    }
}

////////////////////////////////////////////////////////////////////////////////////////
// Print program information and keyboard instructions
void PrintInstruction()
{
    printf("--------------------------------------------------\n");
    printf("myAllegroHand: ");
    if (RIGHT_HAND) printf("Right Hand, v%i.x\n\n", HAND_VERSION); else printf("Left Hand, v%i.x\n\n", HAND_VERSION);

    printf("Keyboard Commands:\n");
    printf("H: Home Position (PD control)\n");
    printf("R: Ready Position (used before grasping)\n");
    printf("G: Three-Finger Grasp\n");
    printf("K: Four-Finger Grasp\n");
    printf("P: Two-finger pinch (index-thumb)\n");
    printf("M: Two-finger pinch (middle-thumb)\n");
    printf("E: Envelop Grasp (all fingers)\n");
    printf("A: Gravity Compensation\n\n");
    printf("F: Servos OFF (any grasp cmd turns them back on)\n");
    printf("Q: Quit this program\n");

    printf("--------------------------------------------------\n\n");
}

/////////////////////////////////////////////////////////////////////////////////////////
// Get channel index for Peak CAN interface
int GetCANChannelIndex(const TCHAR* cname)
{
    if (!cname) return 0;

    if (!_tcsicmp(cname, _T("0")) || !_tcsicmp(cname, _T("PCAN_NONEBUS")) || !_tcsicmp(cname, _T("NONEBUS")))
        return 0;
    else if (!_tcsicmp(cname, _T("1")) || !_tcsicmp(cname, _T("PCAN_ISABUS1")) || !_tcsicmp(cname, _T("ISABUS1")))
        return 1;
    else if (!_tcsicmp(cname, _T("2")) || !_tcsicmp(cname, _T("PCAN_ISABUS2")) || !_tcsicmp(cname, _T("ISABUS2")))
        return 2;
    else if (!_tcsicmp(cname, _T("3")) || !_tcsicmp(cname, _T("PCAN_ISABUS3")) || !_tcsicmp(cname, _T("ISABUS3")))
        return 3;
    else if (!_tcsicmp(cname, _T("4")) || !_tcsicmp(cname, _T("PCAN_ISABUS4")) || !_tcsicmp(cname, _T("ISABUS4")))
        return 4;
    else if (!_tcsicmp(cname, _T("5")) || !_tcsicmp(cname, _T("PCAN_ISABUS5")) || !_tcsicmp(cname, _T("ISABUS5")))
        return 5;
    else if (!_tcsicmp(cname, _T("7")) || !_tcsicmp(cname, _T("PCAN_ISABUS6")) || !_tcsicmp(cname, _T("ISABUS6")))
        return 6;
    else if (!_tcsicmp(cname, _T("8")) || !_tcsicmp(cname, _T("PCAN_ISABUS7")) || !_tcsicmp(cname, _T("ISABUS7")))
        return 7;
    else if (!_tcsicmp(cname, _T("8")) || !_tcsicmp(cname, _T("PCAN_ISABUS8")) || !_tcsicmp(cname, _T("ISABUS8")))
        return 8;
    else if (!_tcsicmp(cname, _T("9")) || !_tcsicmp(cname, _T("PCAN_DNGBUS1")) || !_tcsicmp(cname, _T("DNGBUS1")))
        return 9;
    else if (!_tcsicmp(cname, _T("10")) || !_tcsicmp(cname, _T("PCAN_PCIBUS1")) || !_tcsicmp(cname, _T("PCIBUS1")))
        return 10;
    else if (!_tcsicmp(cname, _T("11")) || !_tcsicmp(cname, _T("PCAN_PCIBUS2")) || !_tcsicmp(cname, _T("PCIBUS2")))
        return 11;
    else if (!_tcsicmp(cname, _T("12")) || !_tcsicmp(cname, _T("PCAN_PCIBUS3")) || !_tcsicmp(cname, _T("PCIBUS3")))
        return 12;
    else if (!_tcsicmp(cname, _T("13")) || !_tcsicmp(cname, _T("PCAN_PCIBUS4")) || !_tcsicmp(cname, _T("PCIBUS4")))
        return 13;
    else if (!_tcsicmp(cname, _T("14")) || !_tcsicmp(cname, _T("PCAN_PCIBUS5")) || !_tcsicmp(cname, _T("PCIBUS5")))
        return 14;
    else if (!_tcsicmp(cname, _T("15")) || !_tcsicmp(cname, _T("PCAN_PCIBUS6")) || !_tcsicmp(cname, _T("PCIBUS6")))
        return 15;
    else if (!_tcsicmp(cname, _T("16")) || !_tcsicmp(cname, _T("PCAN_PCIBUS7")) || !_tcsicmp(cname, _T("PCIBUS7")))
        return 16;
    else if (!_tcsicmp(cname, _T("17")) || !_tcsicmp(cname, _T("PCAN_PCIBUS8")) || !_tcsicmp(cname, _T("PCIBUS8")))
        return 17;
    else if (!_tcsicmp(cname, _T("18")) || !_tcsicmp(cname, _T("PCAN_USBBUS1")) || !_tcsicmp(cname, _T("USBBUS1")))
        return 18;
    else if (!_tcsicmp(cname, _T("19")) || !_tcsicmp(cname, _T("PCAN_USBBUS2")) || !_tcsicmp(cname, _T("USBBUS2")))
        return 19;
    else if (!_tcsicmp(cname, _T("20")) || !_tcsicmp(cname, _T("PCAN_USBBUS3")) || !_tcsicmp(cname, _T("USBBUS3")))
        return 20;
    else if (!_tcsicmp(cname, _T("21")) || !_tcsicmp(cname, _T("PCAN_USBBUS4")) || !_tcsicmp(cname, _T("USBBUS4")))
        return 21;
    else if (!_tcsicmp(cname, _T("22")) || !_tcsicmp(cname, _T("PCAN_USBBUS5")) || !_tcsicmp(cname, _T("USBBUS5")))
        return 22;
    else if (!_tcsicmp(cname, _T("23")) || !_tcsicmp(cname, _T("PCAN_USBBUS6")) || !_tcsicmp(cname, _T("USBBUS6")))
        return 23;
    else if (!_tcsicmp(cname, _T("24")) || !_tcsicmp(cname, _T("PCAN_USBBUS7")) || !_tcsicmp(cname, _T("USBBUS7")))
        return 24;
    else if (!_tcsicmp(cname, _T("25")) || !_tcsicmp(cname, _T("PCAN_USBBUS8")) || !_tcsicmp(cname, _T("USBBUS8")))
        return 25;
    else if (!_tcsicmp(cname, _T("26")) || !_tcsicmp(cname, _T("PCAN_PCCBUS1")) || !_tcsicmp(cname, _T("PCCBUS1")))
        return 26;
    else if (!_tcsicmp(cname, _T("27")) || !_tcsicmp(cname, _T("PCAN_PCCBUS2")) || !_tcsicmp(cname, _T("PCCBUS2")))
        return 27;
    else
        return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Program main
int main(int argc, TCHAR* argv[])
{
    PrintInstruction();
    ros::init(argc, argv, "CAN_devices_node");

    memset(&vars, 0, sizeof(vars));
    memset(q, 0, sizeof(q));
    memset(q_des, 0, sizeof(q_des));
    memset(tau_des, 0, sizeof(tau_des));
    memset(cur_des, 0, sizeof(cur_des));
    curTime = 0.0;

    if (CreateBHandAlgorithm() && OpenCAN()){
        MainLoop();
    }
            
    CloseCAN();
    DestroyBHandAlgorithm();

    return 0;
}
