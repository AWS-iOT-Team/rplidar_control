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

#include <thread>

int   counter = 0;
int   fd_from_yolo;
int   fd_to_yolo;
char  buff[BUFF_SIZE];
int     handle;
struct  termios  oldtio,newtio;
char    *TitleMessage = "Welcome Serial Port\r\n";
char    Buff[256];
int     RxCount;
int     loop;
int     ending;    
int     key;

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

using std::thread;
using namespace rp::standalone::rplidar;

double what_time_is_it_now()
{
    struct timeval time;
    if (gettimeofday(&time,NULL)){
        return 0;
    }
    return (double)time.tv_sec + (double)time.tv_usec * .000001;
}

// 쓰레드 함수
void t_function()
{
    int i = 0;

    //from yolo
    while(1)
    {
        while(read(fd_from_yolo, buff, BUFF_SIZE) != NULL)
        {
            if(buff[0] == 'a')
                Buff[0] = 'f';
            else if(buff[0] == 'b')
                Buff[0] = 'g';
            else if(buff[0] == 'c')
                Buff[0] = 'h';
            else if(buff[0] == 'd')
                Buff[0] = 'i';
            else if(buff[0] == 'i')
                Buff[0] = 'j';
            else
                Buff[0] = buff[0];
            write( handle, Buff, 1 );
            printf("%s", buff);
            memset(buff, 0x00, BUFF_SIZE);
        }
    }
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

int getch(void)
{
    struct termios oldattr, newattr;
    int ch;
    tcgetattr( STDIN_FILENO, &oldattr );
    newattr = oldattr;
    newattr.c_lflag &= ~( ICANON | ECHO );
    tcsetattr( STDIN_FILENO, TCSANOW, &newattr );
    ch = getchar();
    tcsetattr( STDIN_FILENO, TCSANOW, &oldattr );
    return ch;
}

int main_menu(void)
{
    int key;

    printf("\n\n");
    printf("-------------------------------------------------\n");
    printf("                    MAIN MENU\n");
    printf("-------------------------------------------------\n");

    printf(" a. Turn Left                                    \n");
    printf(" b. Turn Right                                   \n");
    printf(" c. Forward                                      \n");
    printf(" d. backward                                     \n");
    printf(" i. stop                                         \n");
    printf(" I. speed up +10                                 \n");
    printf(" D. speed down -10                               \n");


    printf("-------------------------------------------------\n");
    printf(" q. Motor Control application QUIT               \n");
    printf("-------------------------------------------------\n");
    printf("\n\n");
 
    printf("SELECT THE COMMAND NUMBER : ");

    key=getch();
 
    return key;
}

int main(int argc, const char * argv[]) {

    //SECTION code is added -->
    int thr_id;
    int a = 1;

    thread t1(t_function);

    // from wifi thread
    if ( -1 == ( fd_from_yolo = open(FIFO_FROM_LIDAR, O_RDWR) ))
    {
        if ( -1 == mkfifo( FIFO_FROM_LIDAR, 0666))
        {
            perror( "mkfifo() run error");
            //exit( 1);
        }
        if ( -1 == ( fd_from_yolo = open( FIFO_FROM_LIDAR, O_RDWR)))
        {
            perror( "open() error");
            //exit( 1);
        }
    }
    // to wifi thread
    if ( -1 == ( fd_to_yolo = open( FIFO_TO_LIDAR, O_RDWR)))
    {
        if ( -1 == mkfifo( FIFO_TO_LIDAR, 0666))
        {
            perror( "mkfifo() run error");
            //exit( 1);
        }
        if ( -1 == ( fd_to_yolo = open( FIFO_TO_LIDAR, O_RDWR)))
        {
            perror( "open() error");
            //exit( 1);
        }
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
    
    drv->startMotor();
    // start scan...
    drv->startScan(0,1);

    // fetech result and print it out...
    while (1) {
        rplidar_response_measurement_node_hq_t nodes[8192];
        size_t   count = _countof(nodes);

        op_result = drv->grabScanDataHq(nodes, count);
        if (IS_OK(op_result)) {
            drv->ascendScanData(nodes, count);

            // (nodes[pos].angle_z_q14 * 90.f / (1 << 14)) = 각도가 360을 넘기기 전까지 반복
            // 반복 조건문을 수정 시 보고 싶은 각도까지 설정 가능할 듯
            for (int pos = 0; pos < (int)count ; ++pos) {
                printf("%s theta: %03.2f Dist: %08.2f Q: %d \n", 
                    (nodes[pos].flag & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ?"S \n" : " ", 
                    (nodes[pos].angle_z_q14 * 90.f / (1 << 14)), 
                    nodes[pos].dist_mm_q2/4.0f,
                    nodes[pos].quality);   
                    fprintf(stdout, "  degree_count : %d \n", pos);
            }
        }

        if (ctrl_c_pressed){ 
            break;
        }
    }

    drv->stop();
    drv->stopMotor();
    // done!
on_finished:
    RPlidarDriver::DisposeDriver(drv);
    drv = NULL;
    return 0;
}

