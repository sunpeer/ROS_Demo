#include "board.h"
#include "rtthread.h"
#include "ros.h"
#include "mbs_msgs/Velocities.h"
#include <geometry_msgs/Twist.h>
#include "moebus.h"
#include "mbs_msgs/RawImu.h"
#include "mpu6050.h"

#define DBG_TAG "ROS"
#define DBG_LVL DBG_INFO
#include "rtdbg.h"

//建立一个topic，一个速度topic，树莓派上的ROS发布topic，STM32上的node接受
#define ROS_CON_THREAD_SIZE 2 * 1024
#define ROS_CON_THREAD_PRIORITY 10
#define ROS_PARSE_THREAD_SIZE 3 * 1024
#define ROS_PARSE_THREAD_PRIORITY 8

#define ROS_SPIN_FREQUENCE 20
#define CMD_RATE 10
#define VEL_PUB_RATE 10
#define IMU_PUB_RATE 10
#define STOP_DELAY_MS 400

static ros::NodeHandle nh;

#ifdef CPLUSPLUS_VERSION
class MbsBase
{
public:
    MbsBase()
        : _velocities_subscriber("velocities", &MbsBase::set_spd_callback, this)
    {
    }
    void init(ros::NodeHandle &nh)
    {
        nh.subscribe(_velocities_subscriber);
    }
    void set_spd_callback(const mbs_msgs::Velocities &v)
    {
        set_spd(v.linear_x, v.linear_y, v.angular_z);
    }

private:
    ros::Subscriber<mbs_msgs::Velocities, MbsBase> _velocities_subscriber;
};

static MbsBase mb;

static void ros_thread_entry(void *paramter)
{
    nh.initNode();
    mb.init(nh);

    while (1)
    {
        nh.spinOnce();
        rt_thread_mdelay(500);
    }
}
#endif

static float required_linear_x = 0;
static float required_linear_y = 0;
static float required_angular_z = 0;
static rt_tick_t previous_command_time = 0;
static mbs_msgs::Velocities raw_vel_msg;
static mbs_msgs::Imu raw_imu_msg;

static void velocities_cb(const mbs_msgs::Velocities &v)
{
    set_spd(v.linear_x, v.linear_y, v.angular_z);
}

static void command_callback(const geometry_msgs::Twist &cmd_msg)
{

    required_linear_x = cmd_msg.linear.x * 1000;
    required_linear_y = cmd_msg.linear.y * 1000;
    required_angular_z = cmd_msg.angular.z;

    previous_command_time = rt_tick_get();
}

// static ros::Subscriber<mbs_msgs::Velocities> velocities_subscriber("velocities",&velocities_cb);
static ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", &command_callback);
static ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);
static ros::Publisher raw_imu_pub("raw_imu", &raw_imu_msg);
static void ros_con_thread_entry(void *parameter)
{
    nh.initNode();
    // nh.subscribe(velocities_subscriber);
    nh.subscribe(cmd_sub);
    nh.advertise(raw_vel_pub);
    nh.advertise(raw_imu_pub);
    //没连上一直连接
    // while(!nh.connected())
    // {
    //     rt_thread_delay(200);
    //     LOG_W("ros attempt to connect!");
    // }
    // LOG_E("ros connection succeed!");
    while (1)
    {
        nh.spinOnce();
        rt_thread_delay(RT_TICK_PER_SECOND / ROS_SPIN_FREQUENCE);
        // rt_thread_delay(RT_TICK_PER_SECOND/ROS_SPIN_FREQUENCE);
    }
}

static void publish_imu()
{
    measure_acceleration();
    raw_imu_msg.raw_linear_acceleration.x=data_acce.data.acce.x;
    raw_imu_msg.raw_linear_acceleration.y=data_acce.data.acce.y;
    raw_imu_msg.raw_linear_acceleration.z=data_acce.data.acce.z;
    measure_gryoscope();
    raw_imu_msg.raw_angular_velocity.x=data_gyro.data.gyro.x;
    raw_imu_msg.raw_angular_velocity.y=data_gyro.data.gyro.y;
    raw_imu_msg.raw_angular_velocity.z=data_gyro.data.gyro.z;
    raw_imu_pub.publish(&raw_imu_msg);
}

static void ros_parse_thread_entry(void *parameter)
{
    rt_tick_t previous_control_time = 0;
    rt_tick_t previous_vel_pub_time = 0;
    rt_tick_t previous_imu_pub_time = 0;
    while (1)
    {
        if (rt_tick_get() - previous_control_time > RT_TICK_PER_SECOND / CMD_RATE)
        {

            set_spd(required_linear_x, required_linear_y, required_angular_z);
            previous_control_time = rt_tick_get();
        }
        if (rt_tick_get() - previous_command_time > RT_TICK_PER_SECOND * STOP_DELAY_MS / 1000)
        {
            required_linear_x = 0;
            required_linear_y = 0;
            required_angular_z = 0;
            set_spd(required_linear_x, required_linear_y, required_angular_z);
        }
        if (rt_tick_get() - previous_vel_pub_time > RT_TICK_PER_SECOND / VEL_PUB_RATE)
        {
            //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            /*
    可能x，y的方向不太正确
*/
            //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            previous_vel_pub_time = rt_tick_get();
            velocities_t vel = get_velocities();
            raw_vel_msg.linear_x = vel.linear_x;
            raw_vel_msg.linear_y = vel.linear_y;
            raw_vel_msg.angular_z = vel.angular_z;
            raw_vel_pub.publish(&raw_vel_msg);
        }
        if (rt_tick_get() - previous_imu_pub_time > RT_TICK_PER_SECOND / IMU_PUB_RATE)
        {
            previous_imu_pub_time = rt_tick_get();
            publish_imu();
        }
        rt_thread_delay(50);
    }
}

static int ros_thread_init()
{
    //创建一个线程用来和ROS通信
    rt_thread_t con_thread = rt_thread_create("ROS_Con_Thread", ros_con_thread_entry, RT_NULL, ROS_CON_THREAD_SIZE, ROS_CON_THREAD_PRIORITY, 10);
    if (con_thread != RT_NULL)
    {
        rt_thread_startup(con_thread);
        LOG_I("ros con thread startup succeed!");
    }
    else
    {
        LOG_E("ros con thread startup failed!");
        return -1;
    }
    rt_thread_t parse_thread = rt_thread_create("ROS_Parse_Thread", ros_parse_thread_entry, RT_NULL, ROS_PARSE_THREAD_SIZE, ROS_PARSE_THREAD_PRIORITY, 10);
    if (parse_thread != RT_NULL)
    {
        rt_thread_startup(parse_thread);
        LOG_I("ros parse thread startup succeed!");
        return 0;
    }
    else
        LOG_E("ros parse thread startup failed!");
    return -1;
}
INIT_APP_EXPORT(ros_thread_init);