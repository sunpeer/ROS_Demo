#include "moebus.h"

struct rt_device_pwm* pwm_device;

/*

      y
      ^
      |
      |
x<-----

*/
struct motor
{
    int pin_in1;
    int pin_in2;
    int pwm_channel;
    char* encoder_name;
    float spd;
    rt_device_t encoder_device;
    float cur_spd;
    unsigned long prev_time;
    long prev_encoder_ticks;
    rt_bool_t encoder_reverse;
    int pwm;
    float bias;
};

struct motor motors[4]=
{
    {LF_IN1,LF_IN2,LF_CHANNEL,"pulse2",0,RT_NULL,0,0,0,RT_FALSE,0,0},
    {RF_IN1,RF_IN2,RF_CHANNEL,"pulse3",0,RT_NULL,0,0,0,RT_TRUE,0,0},
    {LR_IN1,LR_IN2,LR_CHANNEL,"pulse4",0,RT_NULL,0,0,0,RT_FALSE,0,0},
    {RR_IN1,RR_IN2,RR_CHANNEL,"pulse5",0,RT_NULL,0,0,0,RT_TRUE,0,0}
};

void Motor_Init(void)
{
    pwm_device=(struct rt_device_pwm*)rt_device_find(PWM_NAME);
    rt_int8_t index;
    for(index=0;index<4;index++)
    {
        //设置PWM
        rt_pin_mode(motors[index].pin_in1,PIN_MODE_OUTPUT);
        rt_pin_mode(motors[index].pin_in2,PIN_MODE_OUTPUT);
        rt_pwm_set(pwm_device,motors[index].pwm_channel,PWM_PERIOD,0);
        rt_pwm_enable(pwm_device,motors[index].pwm_channel);
        //设置编码器
        motors[index].encoder_device=rt_device_find(motors[index].encoder_name);
        if(motors[index].encoder_device!=RT_NULL)
        rt_device_open(motors[index].encoder_device,RT_DEVICE_OFLAG_RDONLY);
    }
}

//单位是m/s ,r/s
velocities_t get_velocities()
{
    velocities_t vel;
    vel.linear_y=(motors[0].cur_spd+motors[1].cur_spd+motors[2].cur_spd+motors[3].cur_spd)/4; //rpm
    vel.linear_y=vel.linear_y/SDRM_PARAM/1000; //m/s
    vel.linear_x=(-motors[0].cur_spd+motors[1].cur_spd+motors[2].cur_spd-motors[3].cur_spd)/4; //rpm
    vel.linear_x=vel.linear_x/SDRM_PARAM/1000; //m/s
    vel.angular_z=(-motors[0].cur_spd+motors[1].cur_spd-motors[2].cur_spd+motors[3].cur_spd)/4;
    vel.angular_z=vel.angular_z/SDRM_PARAM; //r/s
    vel.angular_z=vel.angular_z/(a_PARAMETER+b_PARAMETER);
    return vel;
}

//单位是rpm
static void get_spd()
{
    unsigned long current_time=rt_tick_get();
    long current_encoder_ticks=0;
    for(unsigned short index=0;index<4;index++)
    {
        unsigned long dt=current_time-motors[index].prev_time; //单位是ms
        float dtm=dt/60000.0; //单位是分钟
        rt_device_read(motors[index].encoder_device,0,&current_encoder_ticks,1);
		current_encoder_ticks=motors[index].encoder_reverse?-current_encoder_ticks:current_encoder_ticks;
        long delta_ticks=current_encoder_ticks-motors[index].prev_encoder_ticks;
        float current_spd=(float)delta_ticks/PULSE_PER_ROTATE/dtm;
        if(current_spd-motors[index].cur_spd>500||motors[index].cur_spd-current_spd>500)
            current_spd=motors[index].cur_spd;
        else
            motors[index].cur_spd=current_spd;
        motors[index].prev_encoder_ticks=current_encoder_ticks;
        motors[index].prev_time=current_time;
    }
}

void set_spd(float x_vel, float y_vel, float z_rota) //mmps
{
    //关闭全局中断确保pid线程不会得到错误的速度值
    int level=rt_hw_interrupt_disable();
    z_rota *= (a_PARAMETER + b_PARAMETER);
    motors[0].spd = confine((y_vel - x_vel - z_rota) * SDRM_PARAM, -MAX_RPM, MAX_RPM); //rpm
    motors[1].spd = confine((y_vel + x_vel + z_rota) * SDRM_PARAM, -MAX_RPM, MAX_RPM); //rpm
    motors[2].spd = confine((y_vel + x_vel - z_rota) * SDRM_PARAM, -MAX_RPM, MAX_RPM); //rpm
    motors[3].spd = confine((y_vel - x_vel + z_rota) * SDRM_PARAM, -MAX_RPM, MAX_RPM); //rpm
    rt_hw_interrupt_enable(level);
}

void set_pwm(void)
{
    rt_int8_t index;
    for(index=0;index<4;index++)
    {
        if(motors[index].pwm > 0)
        {
            rt_pin_write(motors[index].pin_in1,PIN_LOW);
            rt_pin_write(motors[index].pin_in2,PIN_HIGH); 
            rt_pwm_set(pwm_device, motors[index].pwm_channel, PWM_PERIOD, motors[index].pwm);
        }
        else if(motors[index].pwm < 0)
        {
            rt_pin_write(motors[index].pin_in1, PIN_HIGH);
            rt_pin_write(motors[index].pin_in2, PIN_LOW);
            rt_pwm_set(pwm_device, motors[index].pwm_channel, PWM_PERIOD,-motors[index].pwm);
        }
        else
        {
            rt_pin_write(motors[index].pin_in1, PIN_LOW);
            rt_pin_write(motors[index].pin_in2, PIN_LOW);
            rt_pwm_set(pwm_device, motors[index].pwm_channel, PWM_PERIOD, 0);
        }
    }
}

void pid_update(void)
{
    for(unsigned short index=0;index<4;index++)
    {
        float bias=motors[index].spd-motors[index].cur_spd;
        motors[index].pwm+=(int)(Kp*(bias-motors[index].bias)+Ki*bias);
        motors[index].bias=bias;
        motors[index].pwm=confine(motors[index].pwm,-MAX_PWM,MAX_PWM);
    }
}

rt_timer_t PID_Timer;

static void PID_control(void *parameter)
{
    get_spd();
    pid_update();
	set_pwm();	//PID控制器
}

int car_init()
{
    PID_Timer=rt_timer_create("PID_Timer",PID_control,RT_NULL,10,RT_TIMER_FLAG_PERIODIC);
    if(PID_Timer!=NULL)
        rt_timer_start(PID_Timer);
    Motor_Init();
		return 0;
}
INIT_APP_EXPORT(car_init);


static int car_control_demo(int argc, char **argv)
{
    int result=0;
    if(argc!=5)
    {
        rt_kprintf("Usage: car_control_demo x_l y_l z_r");
        result=-RT_ERROR;
        goto _exit;
    }
    float x=(float)atoi(argv[2]);
    float y=(float)atoi(argv[3]);
    float z=(float)atoi(argv[4]);
    set_spd(x,y,z);
_exit:
    return result;
}
MSH_CMD_EXPORT(car_control_demo,car x_l y_l z_r);