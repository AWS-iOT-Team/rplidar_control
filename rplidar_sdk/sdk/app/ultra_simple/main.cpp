/*
 *  RPLIDAR
 *  Ultra Simple Data Grabber Demo App
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2019 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header
#include <string.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/types.h>

// for yolo
#include <sys/stat.h>
#define  FIFO_FROM_LIDAR   "/tmp/from_lidar_fifo"
#define  FIFO_TO_LIDAR     "/tmp/to_lidar_fifo"
#define  BUFF_SIZE   1024
#include <sys/stat.h>

#include <iostream>
#include <thread>
#include <chrono>

using namespace std;

int              counter = 0;
int              fd_from_yolo;
int              fd_to_yolo;
char             buff[BUFF_SIZE];
char             Buff[256];
char             buff_a[BUFF_SIZE]; 
int              handle;
struct  termios  oldtio,newtio;
const char*      TitleMessage = "Welcome Serial Port\r\n";
int              key;
int              fd;

// rate & resolution
char plus_minus_rate[1024];

int buff_size_chk;

float err_dist_rate = 0.0f;
float min_err_dist_rate = 0.0f;
float max_err_dist_rate = 0.0f;
float dst_distance = 0.0f;
float src_distance = 200.0f;
char min_err_dist_rate_buff[1024];
char max_err_dist_rate_buff[1024];

float err_angle_resolution = 0.0f;
float min_err_angle_resolution = 0.0f;
float max_err_angle_resolution = 0.0f;
float init_angle_resolution = 0.0f;
float prev_angle_resolution = 0.0f;
float dst_angle = 0.0f;
float src_angle = 6.0f;
float pi_angle = 360.0f;
char min_err_angle_resolution_buff[1024];
char max_err_angle_resolution_buff[1024];

int noise = 0;
vector<string> noise_angle;
////

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifdef _WIN32
#include <Windows.h>
#define delay(x)   ::Sleep(x)
#else
#include <unistd.h>
static inline void delay(_word_size_t ms){
    while (ms>=1000){
        usleep(1000*1000);
        ms-=1000;
    };
    if (ms!=0)
        usleep(ms*1000);
}
#endif

using namespace rp::standalone::rplidar;

double what_time_is_it_now()
{
    struct timeval time;
    if (gettimeofday(&time,NULL)){
        return 0;
    }
    return (double)time.tv_sec + (double)time.tv_usec * .000001;
}

bool checkRPLIDARHealth(RPlidarDriver * drv)
{
    u_result     op_result;
    rplidar_response_device_health_t healthinfo;


    op_result = drv->getHealth(healthinfo);
    if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        printf("RPLidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want rplidar to be reboot by software
            // drv->reset();
            return false;
        } else {
            return true;
        }

    } else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}

bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}

// 쓰레드 함수
void t_function()
{
    int fd_write;

    //from yolo
    while(1)
    {
        while(read(fd_from_yolo, buff, BUFF_SIZE) != -1)
        {
            Buff[0] = buff[0];
            fd_write = write( handle, Buff, 1 );
            if (fd_write < 0){
                cout << "write() error";
            }
            printf("%s", buff);
            memset(buff, 0x00, BUFF_SIZE);
        }
    }
}

int getch(int key)
{
    struct termios oldattr, newattr;
    int process_flag; 

    cout << "getch() fd : " << fd << endl;
    int get_chk = tcgetattr( 0, &oldattr);

    newattr = oldattr;
    newattr.c_lflag &= ~( ICANON | ECHO );
    
    // TCSANOW : 즉시 변경
    int set_chk = tcsetattr( 0, TCSANOW, &newattr );
    
    process_flag = key;
    
    set_chk = tcsetattr( 0, TCSANOW, &oldattr );
    
    int result_chk = get_chk + set_chk;
    if (result_chk < 0){
        cout << "tc() failed.\n";
        exit(-1);
    }

    return process_flag;
}

void rate_resolution_measurement(int pos){
    
    char dist_sign = '+';
    char angle_sign = '-';
    if (src_distance < dst_distance)
    {
        err_dist_rate = ((dst_distance - src_distance) / src_distance) * 100;
    }
    else
    {
        dist_sign = '-';
        err_dist_rate = ((src_distance - dst_distance) / src_distance) * 100;
    }

    // 거리 상에서 물체를 감지하지 못하면 각도 분해능 측정 x
    if(err_dist_rate <= 0.0 && err_dist_rate >= 100.0)
    {
        cout << "Couldn't detect" << endl;   
    }
    else if(err_dist_rate > 0.0 && err_dist_rate < 100.0)
    {
        // distance initialize
        if(min_err_dist_rate <= 0.0 && max_err_dist_rate <= 0.0)
        {
            max_err_dist_rate = min_err_dist_rate = err_dist_rate;
        }
        else
        {
            if (max_err_dist_rate < err_dist_rate)
            {
                max_err_dist_rate = err_dist_rate;
                cout << "err_dist_rate 1 " << err_dist_rate << endl;
                if((buff_size_chk = snprintf(max_err_dist_rate_buff, 1024, "%c %03.2f", dist_sign, max_err_dist_rate)) > 1024)
                {
                    cout << "overflow" << endl;
                    exit(-2);
                }
            }
            else if (min_err_dist_rate > err_dist_rate)
            {
                min_err_dist_rate = err_dist_rate;
                cout << "err_dist_rate 2 " << err_dist_rate << endl;
                if((buff_size_chk = snprintf(min_err_dist_rate_buff, 1024, "%c %03.2f", dist_sign, min_err_dist_rate)) > 1024)
                {
                    cout << "overflow" << endl;
                    exit(-2);
                }
            }   
        }
    }

    if (pos == 0)
    {
        prev_angle_resolution = 0.0f;
        cout << "init angle" << endl;
    }
    else
    {
        if (pos <= (int)src_angle - 1)
        {
            if (dst_distance == 0)
            {
                noise++;
                noise_angle.push_back(to_string(dst_angle));
            }
            else
            {
                prev_angle_resolution = dst_angle - prev_angle_resolution;
                err_angle_resolution = prev_angle_resolution;
                cout << "err_angle_resolution " << err_angle_resolution << endl;  
            }   
        }
        
        // degree initialize
        if(err_angle_resolution <= 0.0 && err_angle_resolution >= 100.0)
        {
            cout << "Couldn't detect" << endl;   
        }
        else if (err_angle_resolution > 0.0 && err_angle_resolution < 100.0)
        {
            if(min_err_angle_resolution <= 0.0 && max_err_angle_resolution <= 0.0)
            {
                max_err_angle_resolution = min_err_angle_resolution = err_angle_resolution;
            }
            else
            {
                if (min_err_angle_resolution > err_angle_resolution)
                {
                    min_err_angle_resolution = err_angle_resolution;
                    cout << "err_angle_resolution 1 " << err_angle_resolution << endl;
                    if((buff_size_chk = snprintf(min_err_angle_resolution_buff, 1024, "%c %03.2f", angle_sign, min_err_angle_resolution)) > 1024)
                    {
                        cout << "overflow" << endl;
                        exit(-2);
                    }
                }
                else if (max_err_angle_resolution < err_angle_resolution)
                {
                    max_err_angle_resolution = err_angle_resolution;
                    cout << "err_angle_resolution 2 " << err_angle_resolution << endl;
                    if((buff_size_chk = snprintf(min_err_angle_resolution_buff, 1024, "%c %03.2f", angle_sign, max_err_angle_resolution)) > 1024)
                    {
                        cout << "overflow" << endl;
                        exit(-2);
                    }
                }  
            }            
        }  
    }
}

// int main_menu(int key)
// {
//     char select_menu = ' ';

//     printf("\n\n");
//     printf("-------------------------------------------------\n");
//     printf("                    MAIN MENU\n");
//     printf("-------------------------------------------------\n");

//     printf(" a. Turn Left                                    \n");
//     printf(" b. Turn Right                                   \n");
//     printf(" c. Forward                                      \n");
//     printf(" d. backward                                     \n");
//     printf(" i. stop                                         \n");
//     printf(" I. speed up +10                                 \n");
//     printf(" D. speed down -10                               \n");


//     printf("-------------------------------------------------\n");
//     printf(" q. Motor Control application QUIT               \n");
//     printf("-------------------------------------------------\n");
  
//     switch (key)
//     {
//         case 0:
//             select_menu = 'i';
//             break;

//         default:
//             select_menu = 'c';
//             break;
//     }

//     select_menu = getch();
//     if (select_menu == NULL){
//         cout << "THE COMMAND NUMBER RESULT : " << key << "\n";
//         return key;
//     }
//     else
//     {
//         key = select_menu;
//         cout << "CHOOSED THE COMMAND NUMBER : " << key << "\n";
//         return key;
//     }

//     getch(select_menu);
//     cout << "CHOOSED THE COMMAND MENU : " << select_menu << "\n\n\n";
//     return key;
// }

int main(int argc, const char * argv[]) {
   
    //SECTION code is added -->
    struct termios oldtio, newtio;
    int title;
    thread t_f(t_function);
    vector<string>::iterator angle_iter;

    //SECTION code is added <--
 
    // from wifi thread
    if ( -1 == ( fd_from_yolo = open(FIFO_FROM_LIDAR, O_RDWR) ))
    {
        if ( -1 == mkfifo( FIFO_FROM_LIDAR, 0666))
        {
            perror( "mkfifo() run error\n");
            //exit( 1);
        }
        if ( -1 == ( fd_from_yolo = open( FIFO_FROM_LIDAR, O_RDWR)))
        {
            perror( "open() error\n");
            //exit( 1);
        }
    }
    // to wifi thread
    if ( -1 == ( fd_to_yolo = open( FIFO_TO_LIDAR, O_RDWR)))
    {
        if ( -1 == mkfifo( FIFO_TO_LIDAR, 0666))
        {
            perror( "mkfifo() run error\n");
            //exit( 1);
        }
        if ( -1 == ( fd_to_yolo = open( FIFO_TO_LIDAR, O_RDWR)))
        {
            perror( "open() error\n");
            //exit( 1);
        }
    }

    // 화일을 연다.
    // handle = open( "/dev/ttyUSB1", O_RDWR | O_NOCTTY );
    // handle = open( "/dev/ttyUSB0", O_RDWR | O_NOCTTY );
    handle = open( "/dev/ttyTHS2", O_RDWR | O_NOCTTY );
    cout << "handle : " << handle << endl;
    if( handle == -1) 
    {
        //화일 열기 실패
        // printf( "Serial Open Fail [/dev/ttyUSB0]\r\n "  );
        printf( "Serial Open Fail [/dev/ttyTHS2]\r\n "  );
        exit(0);
    }
    
    tcgetattr( handle, &oldtio );  // 현재 설정을 oldtio에 저장

    memset( &newtio, 0, sizeof(newtio) );
    newtio.c_cflag = B115200 | CS8 | CLOCAL | CREAD ; 
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    //set input mode (non-canonical, no echo,.....)
    newtio.c_lflag = 0;
    
    newtio.c_cc[VTIME] = 128;    // time-out 값으로 사용된다. time-out 값은 TIME*0.1초 이다.
    newtio.c_cc[VMIN]  = 0;     // MIN은 read가 리턴되기 위한 최소한의 문자 개수
    
    tcflush( handle, TCIFLUSH );
    tcsetattr( handle, TCSANOW, &newtio );
    
    // 타이틀 메세지를 표출한다. 
    title = write( handle, TitleMessage, strlen( TitleMessage ) );
    if (title == -1){
        cout << "write() failed.\n";
    }

    const char * opt_com_path = NULL;
    _u32         baudrateArray[2] = {115200, 256000};
    _u32         opt_com_baudrate = 0;
    u_result     op_result;
    
    bool useArgcBaudrate = false;  
    
    printf("Ultra simple LIDAR data grabber for RPLIDAR.\n"
           "Version: " RPLIDAR_SDK_VERSION "\n");

    // read serial port from the command line...
    if (argc>1) opt_com_path = argv[1]; // or set to a fixed value: e.g. "com3" 

    // read baud rate from the command line if specified...
    if (argc>2)
    {
        opt_com_baudrate = strtoul(argv[2], NULL, 10);
        useArgcBaudrate = true;
    }

    if (!opt_com_path) {
        #ifdef _WIN32
            // use default com port
            opt_com_path = "\\\\.\\com57";
        #elif __APPLE__
            opt_com_path = "/dev/tty.SLAB_USBtoUART";
        #else
            opt_com_path = "/dev/ttyUSB0";
        #endif
    }

    // create the driver instance
	RPlidarDriver * drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
    if (!drv) {
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }
    
    rplidar_response_device_info_t devinfo;
    bool connectSuccess = false;
    
    // make connection...
    if(useArgcBaudrate)
    {
        if(!drv)
            drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
        if (IS_OK(drv->connect(opt_com_path, opt_com_baudrate)))
        {
            op_result = drv->getDeviceInfo(devinfo);

            if (IS_OK(op_result)) 
            {
                connectSuccess = true;
            }
            else
            {
                delete drv;
                drv = NULL;
            }
        }
    }
    else
    {
        size_t baudRateArraySize = (sizeof(baudrateArray))/ (sizeof(baudrateArray[0]));
        for(size_t i = 0; i < baudRateArraySize; ++i)
        {
            if(!drv)
                drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
            if(IS_OK(drv->connect(opt_com_path, baudrateArray[i])))
            {
                op_result = drv->getDeviceInfo(devinfo);

                if (IS_OK(op_result)) 
                {
                    connectSuccess = true;
                    break;
                }
                else
                {
                    delete drv;
                    drv = NULL;
                }
            }
        }
    }
    if (!connectSuccess) {
        
        fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
            , opt_com_path);
        goto on_finished;
    }

    // print out the device serial number, firmware and hardware version number..
    printf("RPLIDAR S/N: ");
    for (int pos = 0; pos < 16 ;++pos) {
        printf("%02X", devinfo.serialnum[pos]);
    }

    printf("\n"
            "Firmware Ver: %d.%02d\n"
            "Hardware Rev: %d\n"
            , devinfo.firmware_version>>8
            , devinfo.firmware_version & 0xFF
            , (int)devinfo.hardware_version);

    // check health...
    if (!checkRPLIDARHealth(drv)) {
        goto on_finished;
    }

    signal(SIGINT, ctrlc);
    
    // start motor...
    drv->startMotor();

    // start scan...
    drv->startScan(0,1);

    // fetech result and print it out...
    while (1) 
    {
        rplidar_response_measurement_node_hq_t nodes[8192];
        size_t count = _countof(nodes);

        op_result = drv->grabScanDataHq(nodes, count);
        if (IS_OK(op_result)) 
        {
            drv->ascendScanData(nodes, count);

            // (nodes[pos].angle_z_q14 * 90.f / (1 << 14)) = 각도가 360을 넘기기 전까지 반복
            // 반복 조건문을 수정 시 보고 싶은 각도까지 설정 가능할 듯
            // for (int pos = 0; pos < (int)count; ++pos)
            for (int pos = 0; pos < (int)src_angle; ++pos) 
            {
                auto start = chrono::high_resolution_clock::now();

                // angle_z_q14 : deg(각도)
                // dist_mm_q2 : mm 단위 
                // quality : measurement quality; 반사된 레이저 펄스 강
                printf("%s theta: %03.2f Dist: %08.2f Q: %d \n", 
                    (nodes[pos].flag & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ?"S \n" : " ", 
                    (nodes[pos].angle_z_q14 * 90.f / (1 << 14)), 
                    nodes[pos].dist_mm_q2/4.0f,
                    nodes[pos].quality);
                    
                    dst_distance = nodes[pos].dist_mm_q2/4.0f;
                    if (dst_distance > 0.0f)
                    {
                        if (pos == 0)
                        {
                            prev_angle_resolution = (nodes[pos].angle_z_q14 * 90.f / (1 << 14));
                        }
                        else
                        {
                            dst_angle = (nodes[pos].angle_z_q14 * 90.f / (1 << 14));
                        }    
                        fprintf(stdout, "       prev_angle_resolution : %03.2f \n", prev_angle_resolution);
                    }
                    
                    auto finish = chrono::high_resolution_clock::now() - start;
                    long long microseconds = chrono::duration_cast<chrono::microseconds>(finish).count();
                    if (dst_distance > 0.0f){
                        cout << "microseconds : " << microseconds << endl;
                    }

                    // 오차 측정 함수
                    rate_resolution_measurement(pos);

                    fprintf(stdout, "  degree_count : %d \n", pos);
                    


                    // key = getch(nodes[pos].quality);
                
                    // switch(key)
                    // {               
                    //     case 0:
                    //         printf("0 == i => stop.\n");               
                    //         //Buff[0] = 'i';quality
                    //         fd = write(handle, Buff, 1 );
                    //         cout << "main fd : " << fd << endl;
                    //         if (fd == -1){
                    //             cout << "write() error\n";
                    //             exit(-1);
                    //         }
                            
                    //         break;

                    //     // case 113 || 'q': 
                    //     //     close(handle);
                    //     //     drv->stop();
                    //     //     drv->stopMotor();
                    //     //     drv = NULL;
                    //     //     exit(0);
                    //     //     break;

                    //     default :
                    //         printf("other == c => keep going.\n");               
                    //         //Buff[0] = 'c';
                    //         Buff[0] = 'c';
                    //         fd = write( handle, Buff, 1 );
                    //         cout << "main fd : " << fd << endl;
                    //         if (fd == -1){
                    //             cout << "write() error\n";
                    //             exit(-1);
                    //         }

                    //         break;
                    // }
                    // if (ctrl_c_pressed){ 
                    //     break;
                    // }
                
                
                if (ctrl_c_pressed)
                { 
                    fprintf(stdout, "  error rate minimum = %s %%\n", min_err_dist_rate_buff);
                    fprintf(stdout, "  error rate maximum = %s %%\n", max_err_dist_rate_buff);
                    fprintf(stdout, "  degree resolution minimum = %s ˚\n", min_err_angle_resolution_buff);
                    fprintf(stdout, "  degree resolution maximum = %s ˚\n", max_err_angle_resolution_buff);
                    for (angle_iter = noise_angle.begin(); angle_iter != noise_angle.end(); angle_iter++)
                    {
                        if (angle_iter == noise_angle.end() - 1)
                        {
                            cout << *angle_iter << "..." << endl;
                        }
                        else
                        {
                            cout << *angle_iter << ", ";
                        }          
                    }
                    cout << noise << " units" << endl; 
                    
                    break;
                }                 
            }
        }
        if (ctrl_c_pressed){ 
            break;
        }
    }
    tcsetattr( handle, TCSANOW, &oldtio );
    close( handle );

    drv->stop();
    drv->stopMotor();
    
    // done!
    on_finished:
        RPlidarDriver::DisposeDriver(drv);
    
    drv = NULL;
    
    return 0;
}

