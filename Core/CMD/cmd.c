#include "cmd.h"
#include "../LED/MAIN_BOARD_RGB/ws2812.h"
//#include "../CALIBRATION/calibration.h"
//#include "../ICM20948/icm20948.h"
#include "../EEPROM/eeprom.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <main.h>
#include "stm32h7xx_hal.h"

//extern filter_state_t filter_state;

volatile uint8_t cmd_mode = 0; // Command mode flag (0 = normal, 1 = command mode)

uint8_t is_cmd_mode(void) {
    return cmd_mode;
}

void strip_brackets(char *str) {
    if (str[0] == '[') {
        memmove(str, str + 1, strlen(str));
    }
    char *end = strchr(str, ']');
    if (end) *end = '\0';
}

void process_command(char *cmd) {
    strip_brackets(cmd);

    if (strcmp(cmd, "CMD") == 0) {
        cmd_mode = 1;
        printf("\r\n\r\nEntered command mode\r\n\r\n");
        main_led(0, 255, 0, 255, 1);
        return;
    }

    if (!cmd_mode) {
        printf("Not in command mode\n");
        return;
    }

    if (strcmp(cmd, "exit") == 0) {
        cmd_mode = 0;
        printf("\r\nExiting command mode, rebooting...\r\n\r\n");
        // Wait for UART transmission to complete
        while (!LL_USART_IsActiveFlag_TC(USART6)); // Ensure transmission complete
        // Busy-wait instead of HAL_Delay
        for (volatile uint32_t i = 0; i < 1000000; i++); // ~200ms at 480MHz
        // Check for pending faults
        if (SCB->HFSR) {
            printf("Hard Fault pending: HFSR=0x%08lX\r\n", SCB->HFSR);
        }
        if (SCB->CFSR) {
            printf("Configurable Fault pending: CFSR=0x%08lX\r\n", SCB->CFSR);
        }
        printf("Disabling interrupts\r\n"); // Debug
        __disable_irq(); // Disable all interrupts
        printf("Triggering NVIC_SystemReset\r\n"); // Debug
        NVIC_SystemReset(); // Primary reset
        printf("NVIC_SystemReset failed, trying SCB reset\r\n"); // Debug (should not reach)
        SCB->AIRCR = (0x5FA << SCB_AIRCR_VECTKEY_Pos) | SCB_AIRCR_SYSRESETREQ_Msk;
        printf("SCB reset triggered\r\n"); // Debug (should not reach)
        while (true); // Trap if reset fails
    }
    else if (strcmp(cmd, "cal_accel_gyro") == 0) {
//        start_accel_and_gyro_calibration(&filter_state);
    }
    else if (strcmp(cmd, "cal_mag") == 0) {
//        start_mag_calibration(&filter_state);
    }
    else if (strcmp(cmd, "cal_all") == 0) {
//        start_full_calibration(&filter_state);
    }
    else if (strcmp(cmd, "cancel_cal") == 0) {
//        cancel_calibration();
    }
    else if (strcmp(cmd, "status") == 0) {
        printf("Status OK\n");
    }
    else if (strcmp(cmd, "main_led_ON") == 0) {
        printf("CMD: Executing main_led_ON command\n");
        main_led(0, 255, 0, 255, 1);
        printf("Main LED ON\n");
    }
    else if (strcmp(cmd, "main_led_OFF") == 0) {
        printf("CMD: Executing main_led_OFF command\n");
        main_led(0, 0, 0, 0, 0);
        printf("Main LED OFF\n");
    }
    else if (strncmp(cmd, "set_main_led", 12) == 0) {
        int index, r, g, b;
        float brightness;
        char *params_start = strchr(cmd, '(');
        if (params_start) {
            if (sscanf(params_start, "(%d,%d,%d,%d,%f)", &index, &r, &g, &b, &brightness) == 5) {
                main_led(index, r, g, b, brightness);
                printf("Main LED Set\n");
            } else {
                printf("Invalid Parameters\n");
            }
        }
    }
    else if (strncmp(cmd, "SetRollPID", 10) == 0) {
        float outer_kp, outer_ki, outer_kd, inner_kp, inner_ki, inner_kd;
        char *params_start = strchr(cmd, '(');
        if (params_start) {
            if (sscanf(params_start, "({%f,%f,%f},{%f,%f,%f})",
                       &outer_kp, &outer_ki, &outer_kd, &inner_kp, &inner_ki, &inner_kd) == 6) {
                DualPID_t roll_pid = {
                    .out = {outer_kp, outer_ki, outer_kd},
                    .in = {inner_kp, inner_ki, inner_kd}
                };
                if (EEPROM_SetRollPID(&roll_pid) == W25Qxx_OK) {
                    printf("Roll PID Set: Out Kp=%.3f, Ki=%.3f, Kd=%.3f; In Kp=%.3f, Ki=%.3f, Kd=%.3f\n",
                           outer_kp, outer_ki, outer_kd, inner_kp, inner_ki, inner_kd);
                } else {
                    printf("Failed to set Roll PID\n");
                }
            } else {
                printf("Invalid Roll PID Parameters\n");
            }
        } else {
            printf("Invalid Roll PID Command Format\n");
        }
    }
    else if (strncmp(cmd, "SetPitchPID", 11) == 0) {
        float outer_kp, outer_ki, outer_kd, inner_kp, inner_ki, inner_kd;
        char *params_start = strchr(cmd, '(');
        if (params_start) {
            if (sscanf(params_start, "({%f,%f,%f},{%f,%f,%f})",
                       &outer_kp, &outer_ki, &outer_kd, &inner_kp, &inner_ki, &inner_kd) == 6) {
                DualPID_t pitch_pid = {
                    .out = {outer_kp, outer_ki, outer_kd},
                    .in = {inner_kp, inner_ki, inner_kd}
                };
                if (EEPROM_SetPitchPID(&pitch_pid) == W25Qxx_OK) {
                    printf("Pitch PID Set: Out Kp=%.3f, Ki=%.3f, Kd=%.3f; In Kp=%.3f, Ki=%.3f, Kd=%.3f\n",
                           outer_kp, outer_ki, outer_kd, inner_kp, inner_ki, inner_kd);
                } else {
                    printf("Failed to set Pitch PID\n");
                }
            } else {
                printf("Invalid Pitch PID Parameters\n");
            }
        } else {
            printf("Invalid Pitch PID Command Format\n");
        }
    }
    else if (strncmp(cmd, "SetYawRatePID", 13) == 0) {
        float kp, ki, kd;
        char *params_start = strchr(cmd, '(');
        if (params_start) {
            if (sscanf(params_start, "({%f,%f,%f})", &kp, &ki, &kd) == 3) {
                PID_t yaw_rate_pid = {kp, ki, kd};
                if (EEPROM_SetYawRatePID(&yaw_rate_pid) == W25Qxx_OK) {
                    printf("Yaw Rate PID Set: Kp=%.3f, Ki=%.3f, Kd=%.3f\n", kp, ki, kd);
                } else {
                    printf("Failed to set Yaw Rate PID\n");
                }
            } else {
                printf("Invalid Yaw Rate PID Parameters\n");
            }
        } else {
            printf("Invalid Yaw Rate PID Command Format\n");
        }
    }
    else if (strcmp(cmd, "get_roll_pid") == 0) {
        DualPID_t roll_pid;
        if (EEPROM_GetRollPID(&roll_pid) == W25Qxx_OK) {
            printf("Roll PID: Out Kp=%.3f, Ki=%.3f, Kd=%.3f; In Kp=%.3f, Ki=%.3f, Kd=%.3f\n",
                   roll_pid.out.kp, roll_pid.out.ki, roll_pid.out.kd,
                   roll_pid.in.kp, roll_pid.in.ki, roll_pid.in.kd);
        } else {
            printf("Failed to get Roll PID\n");
        }
    }
    else if (strcmp(cmd, "get_pitch_pid") == 0) {
        DualPID_t pitch_pid;
        if (EEPROM_GetPitchPID(&pitch_pid) == W25Qxx_OK) {
            printf("Pitch PID: Out Kp=%.3f, Ki=%.3f, Kd=%.3f; In Kp=%.3f, Ki=%.3f, Kd=%.3f\n",
                   pitch_pid.out.kp, pitch_pid.out.ki, pitch_pid.out.kd,
                   pitch_pid.in.kp, pitch_pid.in.ki, pitch_pid.in.kd);
        } else {
            printf("Failed to get Pitch PID\n");
        }
    }
    else if (strcmp(cmd, "get_yaw_rate_pid") == 0) {
        PID_t yaw_rate_pid;
        if (EEPROM_GetYawRatePID(&yaw_rate_pid) == W25Qxx_OK) {
            printf("Yaw Rate PID: Kp=%.3f, Ki=%.3f, Kd=%.3f\n",
                   yaw_rate_pid.kp, yaw_rate_pid.ki, yaw_rate_pid.kd);
        } else {
            printf("Failed to get Yaw Rate PID\n");
        }
    }
    else {
        printf("Unknown Command\n");
    }
}


