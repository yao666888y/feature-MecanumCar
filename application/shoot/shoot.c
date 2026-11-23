#include "shoot.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include "servo_motor.h"

#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x))) // 限幅宏定义
#define LOADER_REVERSE_RATE 3.0f // 反转退弹速度, 等效"发/秒"(方向为负, 数值越小退得越慢)

/* 对于双发射机构的机器人,将下面的数据封装成结构体即可,生成两份shoot应用实例 */
static DJIMotorInstance  *loader; // 拨盘电机
static ServoInstance *friction_l, *friction_r; // 左右摩擦轮
// static servo_instance *lid; 需要增加弹舱盖

static Publisher_t *shoot_pub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv; // 来自cmd的发射控制信息
static Subscriber_t *shoot_sub;
static Shoot_Upload_Data_s shoot_feedback_data; // 来自cmd的发射控制信息

// dwt定时,计算冷却用
static float hibernate_time = 0, dead_time = 0;

void ShootInit()
{
    // 左摩擦轮
    Servo_Init_Config_s friction_config = {
        .pwm_init_config = {
            .htim = &htim1,
            .channel = TIM_CHANNEL_2,
            .period = 0.0020f,    // 2ms
            .dutyratio = 1.0f,     // 初始占空比1 最大油门行程对应的占空比
            .callback = NULL,
            .id = 0,
        },
        .servo_type = PWM_Servo,
    };
    friction_l = ServoInit(&friction_config);
    friction_config.pwm_init_config.channel = TIM_CHANNEL_3;

    // 右摩擦轮
    friction_r = ServoInit(&friction_config);
    
    ServoSetAngle(friction_l, 1.0f);
    ServoSetAngle(friction_r, 1.0f);
    DWT_Delay(2); // 等待电调自检
    ServoSetAngle(friction_l, 0.5f);
    ServoSetAngle(friction_r, 0.5f);

    // 油门行程启动电机 占空比 0.5 ~ 0.78
    for(int i=0;i<5;i++)
    {
        ServoSetAngle(friction_l, 0.5f+0.28f*i/4); 
        ServoSetAngle(friction_r, 0.5f+0.28f*i/4); 
        DWT_Delay(0.05f);
    }

    ServoSetAngle(friction_l, 0.5f);
    ServoSetAngle(friction_r, 0.5f);

    // 拨盘电机
    Motor_Init_Config_s loader_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 6,
        },
        .controller_param_init_config = {
            .angle_PID = {
                // 如果启用位置环来控制发弹,需要较大的I值保证输出力矩的线性度否则出现接近拨出的力矩大幅下降
                .Kp = 10, // 10
                .Ki = 0,
                .Kd = 0,
                .MaxOut = 5000,
            },
            .speed_PID = {
                .Kp = 10, // 10
                .Ki = 1, // 1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 8000,
                .MaxOut = 8000,
            },
            .current_PID = {
                .Kp = 2.0, // 0.7
                .Ki = 0.1, // 0.1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 8000,
                .MaxOut = 8000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED, 
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
            .close_loop_type = CURRENT_LOOP | SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 注意方向设置为拨盘的拨出的击发方向
        },
        .motor_type = M2006 // 英雄使用m3508
    };
    loader = DJIMotorInit(&loader_config);
    
    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
}
float setfric_PWM = 0.8f;
/* 机器人发射机构控制核心任务 */
void ShootTask()
{
    // 从cmd获取控制数据
    SubGetMessage(shoot_sub, &shoot_cmd_recv);

    // 对shoot mode等于SHOOT_STOP的情况特殊处理,直接停止所有电机(紧急停止)
    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF)
    {
        // DJIMotorStop(friction_l);
        // DJIMotorStop(friction_r);
        DJIMotorStop(loader);
    }
    else // 恢复运行
    {
        // DJIMotorEnable(friction_l);
        // DJIMotorEnable(friction_r);
        DJIMotorEnable(loader);
    }

    // 退弹模式不允许被单发/连发的不应期拦住
    if (shoot_cmd_recv.load_mode == LOAD_REVERSE)
    {
        hibernate_time = 0;
        dead_time = 0;
    }

    // 如果上一次触发单发或3发指令的时间加上不应期仍然大于当前时间(尚未休眠完毕),直接返回即可
    // 单发模式主要提供给能量机关激活使用(以及英雄的射击大部分处于单发)
    if (hibernate_time + dead_time > DWT_GetTimeline_ms())
        return;

    // 若不在休眠状态,根据robotCMD传来的控制模式进行拨盘电机参考值设定和模式切换
    switch (shoot_cmd_recv.load_mode)
    {
    // 停止拨盘
    case LOAD_STOP:
        DJIMotorOuterLoop(loader, SPEED_LOOP); // 切换到速度环
        DJIMotorSetRef(loader, 0);             // 同时设定参考值为0,这样停止的速度最快
        break;
    // 单发模式,根据鼠标按下的时间,触发一次之后需要进入不响应输入的状态(否则按下的时间内可能多次进入,导致多次发射)
    case LOAD_1_BULLET:                                                                     // 激活能量机关/干扰对方用,英雄用.
        DJIMotorOuterLoop(loader, ANGLE_LOOP);                                              // 切换到角度环
        DJIMotorChangeFeed(loader, ALL_THREE_LOOP, MOTOR_FEED);
        DJIMotorSetRef(loader, loader->measure.total_angle + ONE_BULLET_DELTA_ANGLE); // 控制量增加一发弹丸的角度
        hibernate_time = DWT_GetTimeline_ms();                                              // 记录触发指令的时间
        dead_time = 500;                                                                    // 完成1发弹丸发射的时间
        break;
    // 三连发,如果不需要后续可能删除
    case LOAD_3_BULLET:
        DJIMotorOuterLoop(loader, ANGLE_LOOP);                                                  // 切换到角度环
        DJIMotorChangeFeed(loader, ALL_THREE_LOOP, MOTOR_FEED);
        DJIMotorSetRef(loader, loader->measure.total_angle + 3 * ONE_BULLET_DELTA_ANGLE);       // 增加3发
        hibernate_time = DWT_GetTimeline_ms();                                                  // 记录触发指令的时间
        dead_time = 300;                                                                        // 完成3发弹丸发射的时间
        break;
    // 连发模式,对速度闭环,射频后续修改为可变,目前固定为1Hz
    case LOAD_BURSTFIRE:
        DJIMotorOuterLoop(loader, SPEED_LOOP);
        DJIMotorChangeFeed(loader, SPEED_AND_CURRENT_LOOP, MOTOR_FEED);
        DJIMotorSetRef(loader, shoot_cmd_recv.shoot_rate * ONE_BULLET_DELTA_ANGLE * REDUCTION_RATIO_LOADER);
        // x颗/秒换算成速度: 已知一圈的载弹量,由此计算出1s需要转的角度,注意换算角速度(DJIMotor的速度单位是angle per second)
        break;
    // 拨盘反转,对速度闭环,后续增加卡弹检测(通过裁判系统剩余热量反馈和电机电流)
    // 也有可能需要从switch-case中独立出来
    case LOAD_REVERSE:   // 反转退弹: 低速反转, 把卡住的弹丸退回弹仓(松开拨轮即停)
        DJIMotorOuterLoop(loader, SPEED_LOOP);
        DJIMotorChangeFeed(loader, SPEED_AND_CURRENT_LOOP, MOTOR_FEED);
        DJIMotorSetRef(loader, -LOADER_REVERSE_RATE * ONE_BULLET_DELTA_ANGLE * REDUCTION_RATIO_LOADER);
        break;
    default:
        while (1)
            ; // 未知模式,停止运行,检查指针越界,内存溢出等问题
    }
    // 确定是否开启摩擦轮,后续可能修改为键鼠模式下始终开启摩擦轮(上场时建议一直开启)
    if (shoot_cmd_recv.friction_mode == FRICTION_ON)
    {
        ServoSetAngle(friction_l, setfric_PWM);
        ServoSetAngle(friction_r, setfric_PWM);// 0.5f ~ 1.0f

        // 根据收到的弹速设置设定摩擦轮电机参考值,需实测后填入
        switch (shoot_cmd_recv.bullet_speed)
        {
        case SMALL_AMU_15:
            // DJIMotorSetRef(friction_l, 0);
            // DJIMotorSetRef(friction_r, 0);
            break;
        case SMALL_AMU_18:
            // DJIMotorSetRef(friction_l, 0);
            // DJIMotorSetRef(friction_r, 0);
            break;
        case SMALL_AMU_30:
            // DJIMotorSetRef(friction_l, 0);
            // DJIMotorSetRef(friction_r, 0);
            break;
        default: // 当前为了调试设定的默认值4000,因为还没有加入裁判系统无法读取弹速.
            // friction_control(friction_l, 0.65f); // 占空比根据测试调整
            // friction_control(friction_r, 0.65f);
            break;
        }
    }
    else // 关闭摩擦轮
    {
        ServoSetAngle(friction_l, 0.5f);
        ServoSetAngle(friction_r, 0.5f);
    }

    // 开关弹舱盖
    if (shoot_cmd_recv.lid_mode == LID_CLOSE)
    {
        //...
    }
    else if (shoot_cmd_recv.lid_mode == LID_OPEN)
    {
        //...
    }

    // 反馈数据,目前暂时没有要设定的反馈数据,后续可能增加应用离线监测以及卡弹反馈
    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}