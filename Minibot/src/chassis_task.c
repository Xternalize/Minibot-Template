#include "chassis_task.h"
#include "..\..\control-base\devices\inc\dji_motor.h"


#include "robot.h"
#include "remote.h"

extern Robot_State_t g_robot_state;
extern Remote_t g_remote;

DJI_Motor_Handle_t* motor_w[4];

#define WHEEL_ANG_OFFSET 0.78539816339f
#define WHEEL_ANG_OFFSET_COS cos(WHEEL_ANG_OFFSET)
#define WHEEL_ANG_OFFSET_SIN sin(WHEEL_ANG_OFFSET)

float chassis_rad = 1;
float wheel_rad = 1;

void Calc_Wheel_Ang_Velocs(float wheel_ang_velocs[], float x_veloc, float y_veloc, float omega)
{
    wheel_ang_velocs[0] = (- WHEEL_ANG_OFFSET_SIN * x_veloc + WHEEL_ANG_OFFSET_COS * y_veloc + chassis_rad * omega) / wheel_rad;
    wheel_ang_velocs[1] = (- WHEEL_ANG_OFFSET_COS * x_veloc - WHEEL_ANG_OFFSET_SIN * y_veloc + chassis_rad * omega) / wheel_rad;
    wheel_ang_velocs[2] = (+ WHEEL_ANG_OFFSET_SIN * x_veloc - WHEEL_ANG_OFFSET_COS * y_veloc + chassis_rad * omega) / wheel_rad;
    wheel_ang_velocs[3] = (+ WHEEL_ANG_OFFSET_COS * x_veloc + WHEEL_ANG_OFFSET_SIN * y_veloc + chassis_rad * omega) / wheel_rad;
}

void Chassis_Task_Init()
{
    Motor_Config_t chassis_w[4];

    for (int i = 0; i < 4; i++)
    {
        Motor_Config_t chassis_wi = {
            .can_bus = 1, // what can bus the motor is on
            .speed_controller_id = 1 + i, // identifier for each motor
            .offset = 0, // Initial offset of the motor (used for encoder)
            .control_mode = VELOCITY_CONTROL, // Control mode of the motor
            .motor_reversal = MOTOR_REVERSAL_NORMAL, // Direction of the motor
            .velocity_pid = // pid
                {
                    .kp = 500.0f,
                    .kd = 0.0f,
                    .kf = 0.0f,
                    .output_limit = M2006_MAX_CURRENT_INT, // m2006 is the motor
                },
        };

        motor_w[i] = DJI_Motor_Init(&chassis_wi, M2006); // Initializing motor
    }
}

void Chassis_Ctrl_Loop()
{
    g_robot_state.chassis.x_speed = g_remote.controller.left_stick.x / 660;
    g_robot_state.chassis.y_speed = g_remote.controller.left_stick.y / 660;
    g_robot_state.chassis.omega = g_remote.controller.right_stick.x / 660;

    float wheel_ang_velocs[4];
    Calc_Wheel_Ang_Velocs(wheel_ang_velocs, g_robot_state.chassis.x_speed, g_robot_state.chassis.x_speed, g_robot_state.chassis.omega);
    for (int i = 0; i < 4; i++)
        DJI_Motor_Set_Velocity(motor_w[i], wheel_ang_velocs[i]);
}