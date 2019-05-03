#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <errno.h>
#include <math.h>
#include <drivers/drv_hrt.h>
#include <systemlib/err.h>
#include <fcntl.h>
#include <systemlib/mavlink_log.h>
#include <uORB/uORB.h>
#include <uORB/topics/optical_flow.h>
//#include <uORB/topics/vehicle_attitude.h>
#include <px4_defines.h>
#include <px4_config.h>
#include <px4_posix.h>
#include <px4_shutdown.h>
#include <px4_tasks.h>
#include <px4_time.h>



/**
 * @file flow_uart.c
 * @author Tonser <sundxfansky@sjtu.edu.cn>
 * v1.0 2019.3.24
 *
 * 优象光流驱动 通过串口传递数据 发布ORB_ID(optical_flow)
 *
 * This driver publish optical flow data.
 */

static bool thread_should_exit = false;
static bool thread_running = false;
static int daemon_task;


__EXPORT int flow_uart_main(int argc, char *argv[]);
int flow_uart_thread_main(int argc, char *argv[]);

static int uart_init(const char * uart_name);    //
static int set_uart_baudrate(const int fd, unsigned int baud); //static
static void usage(const char *reason);            //static
//extern orb_advert_t mavlink_log_pub;
orb_advert_t mavlink_log_pub = NULL;

int set_uart_baudrate(const int fd, unsigned int baud)
{
    int speed;

    switch (baud) {
        case 9600:   speed = B9600;   break;
        case 19200:  speed = B19200;  break;
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
        default:
            PX4_INFO("ERR: baudrate: %d\n", baud);
            return -EINVAL;
    }

    struct termios uart_config;

    int termios_state;


    tcgetattr(fd, &uart_config);

    uart_config.c_oflag &= ~ONLCR;

    uart_config.c_cflag &= ~(CSTOPB | PARENB);

    if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
        PX4_INFO("ERR: %d (cfsetispeed)\n", termios_state);
        return false;
    }

    if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
        PX4_INFO("ERR: %d (cfsetospeed)\n", termios_state);
        return false;
    }

    if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) {
        PX4_INFO("ERR: %d (tcsetattr)\n", termios_state);
        return false;
    }

    return true;
}


int uart_init(const char * uart_name)
{
    int serial_fd = open(uart_name, O_RDWR | O_NOCTTY);

    if (serial_fd < 0) {
        err(1, "failed to open port: %s", uart_name);
        return false;
    }
    return serial_fd;
}

static void usage(const char *reason)
{
    if (reason) {
        fprintf(stderr, "%s\n", reason);
    }

    fprintf(stderr, "WARN: lose para,use {start|stop|status} [param]\n\n");
    exit(1);
}

int flow_uart_main(int argc, char *argv[])
{

mavlink_log_info(&mavlink_log_pub,"[inav] upixels_flow_main on init");
// mavlink_log_critical(mavlink_log_pub, "test>>>>>>> %8.4f",9.9898);
			
    if (argc < 2) 
    {
        usage("[YCM]missing command");
    }

    if (!strcmp(argv[1], "start")) {
        if (thread_running) {
            PX4_INFO("[YCM]already running\n");
            exit(0);
        }

        thread_should_exit = false;
        daemon_task = px4_task_spawn_cmd("flow_uart",
                         SCHED_DEFAULT,
                         SCHED_PRIORITY_DEFAULT,
                         2500,
                         flow_uart_thread_main,
                         (argv) ? (char * const *)&argv[2] : (char * const *)NULL);
        return 0;
    }

    if (!strcmp(argv[1], "stop")) {
        thread_should_exit = true;
        thread_running = false;
        return 0;
    }
    if (!strcmp(argv[1], "status")) {
        if (thread_running) {
            warnx("[YCM]running");

        } else {
            warnx("[YCM]stopped");
        }

       return 0;
    }

    usage("unrecognized command");
    return 1;
}

int flow_uart_thread_main(int argc, char *argv[])
{
    mavlink_log_info(&mavlink_log_pub,"upixels_flow run ");
    char data = '0';
    //const char *uart_name = argv[1];
    char buffer[7] = "";
    float upixels_flow_x = -1;
    float upixels_flow_y = -1;
    float integration_timespan = -1;
    // long checksum = 0; 
    // uint8_t valid = 0;
    int uart_read = uart_init("/dev/ttyS6");

    if(false == uart_read)
    {
         mavlink_log_critical(&mavlink_log_pub,"[YCM]uart init is failed\n");
         return -1;
    }
    if(false == set_uart_baudrate(uart_read,19200)){
        // mavlink_log_critical(&mavlink_log_pub,"[YCM]set_uart_baudrate is failed\n");
        return -1;
    }
    mavlink_log_info(&mavlink_log_pub,"[YCM]uart init is successful\n");

    thread_running = true;

    // 定义话题结构
    struct optical_flow_s flow_data;
//    struct vehicle_attitude_s att;
//    memset(&att, 0, sizeof(att));
    // struct upixels_flow_s flow_data;
    // 初始化数据
    memset(&flow_data, 0 , sizeof(flow_data));
   
    //公告消息
    orb_advert_t flow_data_handle = orb_advertise(ORB_ID(optical_flow), &flow_data);//公告这个主题
    //  orb_advert_t flow_data_handle = orb_advertise(ORB_ID(upixels_flow), &flow_data);//公告这个主题

    // 测试订阅程序
    // int test_sub_handle = orb_subscribe(ORB_ID(upixels_flow));
    // struct upixels_flow_s test_sub;

    // int counter = 0;
    uint64_t _previous_collect_timestamp = hrt_absolute_time();
    uint64_t _flow_dt_sum_usec = 0;
    float scale = 1.3f;
//    int vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
    while(thread_running)
   {
        // PX4_WARN("warnxxxxxxxxxxxx!!!!");
        //解码 串口信息
    	read(uart_read,&data,1);
        if((data == 0xFE))
        {
            data = '0';
            read(uart_read,&data,1);
            if((data == 0x0A))
            {
                for(int k = 0;k < 6;++k)
                {
                data = '0';
                read(uart_read,&data,1);
                buffer[k] = data;
                }
                for(int k = 0;k < 3;++k)
                {
                    data = '0';
                    read(uart_read,&data,1);
                }
                buffer[6] = data;
                uint64_t timestamp = hrt_absolute_time();
                uint64_t dt_flow = timestamp - _previous_collect_timestamp;
                _previous_collect_timestamp = timestamp;
                _flow_dt_sum_usec += dt_flow;
                // debug 输出数据 十六进制
                // PX4_INFO("%X,%X,%X,%X,%X,%X,%X,",buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5],buffer[6]);
                //X 像素点累计时间内的累加位移,(radians*10000) [除以 10000 乘以高度(m)后为实际位移(m)]
                upixels_flow_x = (float)((int16_t)(buffer[1]<<8|buffer[0])/10000.0f);// 单位是m
                upixels_flow_y = (float)((int16_t)(buffer[3]<<8|buffer[2])/10000.0f);// 单位是m
                integration_timespan = (float)((uint16_t)(buffer[5]<<8|buffer[4]));// 单位是us
                //flow_data.pixel_flow_x_integral = (float)(upixels_flow_x*1000000.0f/integration_timespan);// rad/s
                //flow_data.pixel_flow_y_integral = (float)(upixels_flow_y*1000000.0f/integration_timespan);// rad/s
                flow_data.pixel_flow_x_integral = -upixels_flow_y*scale;
                flow_data.pixel_flow_y_integral = upixels_flow_x*scale;
//              orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &att);
                flow_data.gyro_x_rate_integral = 0.0f*integration_timespan;
                flow_data.gyro_y_rate_integral = 0.0f;
                flow_data.gyro_z_rate_integral = 0.0f;
                flow_data.min_ground_distance = 0.5;//与官方optical_flow不同，官方数据为0.7
                flow_data.max_ground_distance = 3;//与官方optical_flow相同
                flow_data.max_flow_rate = 2.5;//与官方optical_flow相同,最大限制角速度
                flow_data.sensor_id = 0;
                flow_data.timestamp = timestamp;
                flow_data.frame_count_since_last_readout = 1; //4;
                flow_data.integration_timespan = _flow_dt_sum_usec;
                _flow_dt_sum_usec = 0;
                if (buffer[6] == 0xF5) {
                    //数据可用
                    flow_data.quality = 255;
                    orb_publish(ORB_ID(optical_flow),flow_data_handle,  &flow_data);
                }
                else{
                    flow_data.quality = 0;
                    orb_publish(ORB_ID(optical_flow),flow_data_handle,&flow_data);
                }
            }
        }
   }
    thread_running = false;
    //取消订阅
    // orb_unsubscribe(test_sub_handle);
    close(uart_read);
    fflush(stdout);
    return 0;

}
