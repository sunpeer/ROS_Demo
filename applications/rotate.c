#include "rotate.h"

static rt_int32_t tar_heading;
static rt_int32_t cur_heading;
static float last_bias;
static float last_last_bias;
static float z_spd;

extern rt_thread_t rt_current_thread;

rt_timer_t heading_PID_Timer;

//该函数每20ms运行一次
static void heading_control(void *parameter)
{
    //读取航偏角
    // float heading=(rt_uint32_t)mag_read();
    //因为测出来的航偏角有一定的浮动，所以控制航偏值为2的倍数之内
    //float heading转为一个int，采用
    //pid控制确定要设置多大的速度
    //pid计算公式： y+=Kp*(d-d')+Ki*d+Kd*(d-d''+d')
    cur_heading=(rt_int32_t)mag_read();
    float cur_bias=tar_heading-cur_heading;

/* 这种方法没有什么效果 */

    //如果差1°就当做调整好了
    // if(cur_bias==confine(cur_bias,-1,1))
    //     cur_bias==0;


    z_spd+=heading_Kp*(cur_bias-last_bias)+heading_Ki*cur_bias;
    z_spd=confine(z_spd,-heading_MAX_Z_SPD,heading_MAX_Z_SPD);
    last_last_bias=last_bias;
    last_bias=cur_bias;
    //设置速度
    set_spd(0,0,z_spd*DEG_TO_RAD);
}

int heading_init()
{
    heading_PID_Timer=rt_timer_create("heading_PID_Timer",heading_control,RT_NULL,20,
                                        RT_TIMER_FLAG_PERIODIC|RT_TIMER_FLAG_SOFT_TIMER );
    if(heading_PID_Timer!=NULL)
        rt_timer_start(heading_PID_Timer);
    //初始化一下当前小车的航偏角
    cur_heading=(rt_uint32_t)mag_read();
    tar_heading=cur_heading;
    rt_kprintf("current heading = %d ",cur_heading);
    //初始化一下pid要用的这些数值
    last_bias=0;
    last_last_bias=0;
    z_spd=0;
    return 0;
}
MSH_CMD_EXPORT(heading_init,heading_control);


static int heading_control_demo(int argc,char** argv)
{
    int result=0;
    if(argc!=2)
    {
        rt_kprintf("Usage: heading_control_demo deg");
        result=-RT_ERROR;
        goto _exit;
    }
    tar_heading=atoi(argv[1]);
_exit:
    return result;
}
MSH_CMD_EXPORT(heading_control_demo,heading_control_demo deg);