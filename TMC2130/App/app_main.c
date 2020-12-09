#include <stdlib.h>

#include "tmc2130_step_generator.h"
#include "tmc2130.h"
#include "delay_us.h"
#include "usart.h"
#include "main.h"

UART_HandleTypeDef *Log_UART = &huart2;

TMC2130TypeDef Motor_X;
TMC2130_Controller_t Motor_X_Controller = {.Motor = &Motor_X};

TMC2130TypeDef Motor_Y;
TMC2130_Controller_t Motor_Y_Controller = {.Motor = &Motor_Y};

void Log_UART_Send_Char(char data)
{
    Log_UART->Instance->DR = (data);
    while (__HAL_UART_GET_FLAG(Log_UART, UART_FLAG_TC) == 0)
        ;
}

void Log_UART_Send_String(char *data)
{
    while (*data)
    {
        Log_UART_Send_Char(*data++);
    }
}

void Log_UART_Send_Int(int32_t num)
{
    char int_to_str[10] = {0};
    itoa(num, int_to_str, 10);
    Log_UART_Send_String(int_to_str);
}

void Log_UART_Send_Int_Hex(int32_t num)
{
    char int_to_str[32] = {0};
    itoa(num, int_to_str, 16);
    Log_UART_Send_String(int_to_str);
}

void Motor_X_config()
{
    int32_t sts = 0;

    Motor_X.CS_Port = SPI_CS_X_GPIO_Port;
    Motor_X.CS_Pin = SPI_CS_X_Pin;

    Motor_X.Step_Port = STEP_X_GPIO_Port;
    Motor_X.Step_Pin = STEP_X_Pin;

    Motor_X.DIR_Port = DIR_X_GPIO_Port;
    Motor_X.DIR_Pin = DIR_X_Pin;

    Motor_X.Enable_Port = EN_X_GPIO_Port;
    Motor_X.Enable_Pin = EN_X_Pin;

    Motor_X.DIAG1_Port = DIAG0_X_GPIO_Port;
    Motor_X.DIAG1_Pin = DIAG0_X_Pin;

    TMC_Add(&Motor_X_Controller);

    // set step generator parameters
    TMC_Set_Acceleration(&Motor_X_Controller, 100000);
    TMC_Set_MAX_velocity(&Motor_X_Controller, 100000);

    // enable stall and set minimum velocity for stall activation
    TMC_Enable_Stall(&Motor_X_Controller, 100000);

    // set tmc2130 parameters
    TMC2130_Set_Chopper_Off_Time(&Motor_X, 4);
    TMC2130_Set_Chopper_Blank_Time(&Motor_X, 0b10);
    TMC2130_Set_Max_Current(&Motor_X, 5);
    TMC2130_Set_Standby_Current(&Motor_X, 5);
    TMC2130_Set_Microstep(&Motor_X, 256);
    TMC2130_Set_TCOOLTHRS(&Motor_X, 0xFFFFF);
    TMC2130_Set_THIGH(&Motor_X, 0);
    TMC2130_Set_SEMIN_I(&Motor_X, 5);
    TMC2130_Set_SEMAX(&Motor_X, 2);
    TMC2130_Set_SEDN_I(&Motor_X, 0b01);

    TMC2130_Set_Stall_Threshold(&Motor_X, 12);

    // config diag1 for stall
    TMC2130_Set_DIAG0_Attribute(&Motor_X, DIAG_STALL, 1);

    sts = TMC2130_Get_Microstep(&Motor_X);
    sts = TMC2130_Get_Max_Current(&Motor_X);
    sts = TMC2130_Get_Standby_Current(&Motor_X);

    UNUSED(sts);
}

void Motor_Y_config()
{
    int32_t sts = 0;

    Motor_Y.CS_Port = SPI_CS_Y_GPIO_Port;
    Motor_Y.CS_Pin = SPI_CS_Y_Pin;

    Motor_Y.Step_Port = STEP_Y_GPIO_Port;
    Motor_Y.Step_Pin = STEP_Y_Pin;

    Motor_Y.DIR_Port = DIR_Y_GPIO_Port;
    Motor_Y.DIR_Pin = DIR_Y_Pin;

    Motor_Y.Enable_Port = EN_Y_GPIO_Port;
    Motor_Y.Enable_Pin = EN_Y_Pin;

    Motor_Y.DIAG1_Port = DIAG0_Y_GPIO_Port;
    Motor_Y.DIAG1_Pin = DIAG0_Y_Pin;

    TMC_Add(&Motor_Y_Controller);

    // set step generator parameters
    TMC_Set_Acceleration(&Motor_Y_Controller, 130000);
    TMC_Set_MAX_velocity(&Motor_Y_Controller, 130000);

    // enable stall and set minimum velocity for stall activation
    TMC_Enable_Stall(&Motor_Y_Controller, 100000);

    // set tmc2130 parameters
    TMC2130_Set_Chopper_Off_Time(&Motor_Y, 4);
    TMC2130_Set_Chopper_Blank_Time(&Motor_Y, 0b10);
    TMC2130_Set_Max_Current(&Motor_Y, 5);
    TMC2130_Set_Standby_Current(&Motor_Y, 5);
    TMC2130_Set_Microstep(&Motor_Y, 256);
    TMC2130_Set_TCOOLTHRS(&Motor_Y, 0xFFFFF);
    TMC2130_Set_THIGH(&Motor_Y, 0);
    TMC2130_Set_SEMIN_I(&Motor_Y, 5);
    TMC2130_Set_SEMAX(&Motor_Y, 2);
    TMC2130_Set_SEDN_I(&Motor_Y, 0b01);

    TMC2130_Set_Stall_Threshold(&Motor_Y, 12);

    // config diag1 for stall
    TMC2130_Set_DIAG0_Attribute(&Motor_Y, DIAG_STALL, 1);

    sts = TMC2130_Get_Microstep(&Motor_Y);
    sts = TMC2130_Get_Max_Current(&Motor_Y);
    sts = TMC2130_Get_Standby_Current(&Motor_Y);

    UNUSED(sts);
}

void App_Main(void)
{
    Motor_X_config();
    Motor_Y_config();

    TMC_Enable_Driver(&Motor_X_Controller, 1);
    TMC_Enable_Driver(&Motor_Y_Controller, 1);

    TMC_TIM_Enable(1);

    //rotate at fix velocity rpm = (100000*60)/(200*256)
    TMC_Rotate(&Motor_X_Controller, 100000);

    // wait for stall, sensorless homing
    while (!(TMC_Get_Status(&Motor_X_Controller) & STATUS_STALLED))
        ;

    // reset postion
    TMC_Set_Actual_Position(&Motor_X_Controller, 0);

    //rotate at fix velocity rpm = (50000*60)/(200*256)
    //TMC_Rotate(&Motor_Y_Controller, 50000);

    // wait for stall, sensorless homing
    //while(!(TMC_Get_Status(&Motor_Y_Controller) & STATUS_STALLED));

    // reset postion
    //TMC_Set_Actual_Position(&Motor_Y_Controller, 0);

    HAL_Delay(2000);

    while (1)
    {
        TMC_Move(&Motor_X_Controller, -256 * 200 * 1); // move xx revolution
        TMC_Move(&Motor_Y_Controller, -256 * 200 * 2); // move xx revolution

        while (!(TMC_Get_Status(&Motor_X_Controller) & STATUS_TARGET_REACHED) &&
               !(TMC_Get_Status(&Motor_X_Controller) & STATUS_STALLED))
            ;

        while (!(TMC_Get_Status(&Motor_Y_Controller) & STATUS_TARGET_REACHED) &&
               !(TMC_Get_Status(&Motor_Y_Controller) & STATUS_STALLED))
            ;
        HAL_Delay(10);

        TMC_Move(&Motor_X_Controller, 256 * 200 * 1); // move xx revolution
        TMC_Move(&Motor_Y_Controller, 256 * 200 * 2); // move xx revolution

        while (!(TMC_Get_Status(&Motor_X_Controller) & STATUS_TARGET_REACHED) &&
               !(TMC_Get_Status(&Motor_X_Controller) & STATUS_STALLED))
            ;

        while (!(TMC_Get_Status(&Motor_Y_Controller) & STATUS_TARGET_REACHED) &&
               !(TMC_Get_Status(&Motor_Y_Controller) & STATUS_STALLED))
            ;

        HAL_Delay(10);
    }
}
