/*
 * Closed-Loop Position Controller
 * v3.0  —  date: 2024
 * ServoX Controller Inc.
 *
 * Hardware:
 *   - STM32 (HAL-based)
 *   - Quadrature encoder on TIM2 (A→PA0, B→PA1)
 *   - 12-bit DAC output on DAC_CHANNEL_1 (PA4)
 *   - DAC output drives actuator amplifier (0–4095 = 0–3.3V)
 *
 * Control:
 *   - Setpoint: sinusoidal trajectory from SP_Sin_LUT[101]
 *   - Feedback: encoder position scaled to 0–4095
 *   - Algorithm: PD controller (proportional + derivative)
 *   - Loop rate: 10ms (100Hz)
 *
 * DAC output mapping:
 *   2048 = zero force (midpoint)
 *   >2048 = positive drive
 *   <2048 = negative drive
 */

#include "main.h"

DAC_HandleTypeDef hdac;
TIM_HandleTypeDef htim2;

/* -----------------------------------------------------------------------
 * Encoder scaling
 *   rotation: number of encoder counts per full actuator stroke
 *   TIM2 period = rotation * 4095 so CNT stays in range
 * ----------------------------------------------------------------------- */
#define ROTATION_COUNTS     10      /* encoder counts per full stroke */
#define DAC_MIDPOINT        2048    /* DAC value = zero actuator force */
#define DAC_MAX             4095
#define DAC_MIN             0
#define LUT_SIZE            101
#define LOOP_DELAY_MS       10      /* control loop update rate */

/* PD gains — tune these for your plant */
#define KP_NUMERATOR        1       /* Kp = KP_NUMERATOR / KP_DENOMINATOR */
#define KP_DENOMINATOR      1
#define KD_NUMERATOR        1       /* Kd = KD_NUMERATOR / KD_DENOMINATOR */
#define KD_DENOMINATOR      10

/*
 * Sinusoidal setpoint lookup table
 * 101 points, one full sine cycle mapped to 0–4095 (12-bit range)
 * Index 0 and 100 both = 0 (start and end of cycle)
 */
uint16_t SP_Sin_LUT[LUT_SIZE] = {
    0,    4,   16,   36,   64,
  100,  144,  195,  253,  319,  391,  470,  555,
  646,  742,  844,  950, 1061, 1176, 1294, 1415,
 1538, 1664, 1791, 1919,

 2048, 2176, 2304, 2431, 2557, 2680, 2801, 2919,
 3034, 3145, 3251, 3353, 3449, 3540, 3625, 3704,
 3776, 3842, 3900, 3951, 3995, 4031, 4059, 4079,
 4091, 4095,

 4091, 4079, 4059, 4031, 3995, 3951,
 3900, 3842, 3776, 3704, 3625, 3540, 3449, 3353,
 3251, 3145, 3034, 2919, 2801, 2680, 2557, 2431,
 2304, 2176, 2048,

 1919, 1791, 1664, 1538, 1415,
 1294, 1176, 1061,  950,  844,  742,  646,  555,
  470,  391,  319,  253,  195,  144,  100,   64,
   36,   16,    4,    0
};

/* -----------------------------------------------------------------------
 * Function prototypes
 * ----------------------------------------------------------------------- */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM2_Init(void);
static int32_t clamp(int32_t value, int32_t min, int32_t max);

/* -----------------------------------------------------------------------
 * main
 * ----------------------------------------------------------------------- */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DAC_Init();
    MX_TIM2_Init();

    /* Start encoder interface — TI12 mode counts both A and B edges */
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

    /* Start DAC output */
    HAL_DAC_Start(&hdac, DAC_CHANNEL_1);

    /* Reset encoder count to midpoint of TIM2 period
     * so that reverse motion (negative direction) counts
     * downward from midpoint instead of wrapping around */
    __HAL_TIM_SET_COUNTER(&htim2, (ROTATION_COUNTS * DAC_MAX) / 2);

    uint16_t lut_index  = 0;        /* current setpoint index in LUT */
    int32_t  setpoint   = 0;        /* current setpoint value 0–4095 */
    int32_t  position   = 0;        /* current encoder position 0–4095 */
    int32_t  error      = 0;        /* current error */
    int32_t  prev_error = 0;        /* previous error for derivative term */
    int32_t  d_term     = 0;        /* derivative term */
    int32_t  u_value    = 0;        /* DAC output value 0–4095 */

    while (1)
    {
        /* ------------------------------------------------------------------
         * 1. Read encoder
         *    TIM2->CNT is uint32. Cast to int32 so that counts below
         *    the reset midpoint correctly read as smaller values.
         *    Divide by ROTATION_COUNTS to scale to 0–4095 range.
         * ------------------------------------------------------------------ */
        position = (int32_t)(__HAL_TIM_GET_COUNTER(&htim2)) / ROTATION_COUNTS;

        /* Clamp position to valid DAC range in case of overshoot */
        position = clamp(position, DAC_MIN, DAC_MAX);

        /* ------------------------------------------------------------------
         * 2. Get setpoint from LUT
         * ------------------------------------------------------------------ */
        setpoint = (int32_t)SP_Sin_LUT[lut_index];

        /* ------------------------------------------------------------------
         * 3. Compute error
         *    error = setpoint - feedback
         *    positive error → position is behind setpoint → drive forward
         *    negative error → position is ahead of setpoint → drive back
         * ------------------------------------------------------------------ */
        error = setpoint - position;

        /* ------------------------------------------------------------------
         * 4. PD control law
         *
         *    u = Kp * error + Kd * (error - prev_error)
         *
         *    Using integer arithmetic with numerator/denominator scaling.
         *    Output is centered at DAC_MIDPOINT (2048):
         *      u_value = midpoint + control_output
         * ------------------------------------------------------------------ */
        d_term  = ((error - prev_error) * KD_NUMERATOR) / KD_DENOMINATOR;

        u_value = DAC_MIDPOINT
                + ((error * KP_NUMERATOR) / KP_DENOMINATOR)
                + d_term;

        /* Save error for next derivative calculation */
        prev_error = error;

        /* ------------------------------------------------------------------
         * 5. Clamp DAC output to valid 12-bit range (0–4095)
         *    Prevents integer overflow reaching the DAC register
         * ------------------------------------------------------------------ */
        u_value = clamp(u_value, DAC_MIN, DAC_MAX);

        /* ------------------------------------------------------------------
         * 6. Write to DAC
         * ------------------------------------------------------------------ */
        HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,
                         DAC_ALIGN_12B_R, (uint32_t)u_value);

        /* ------------------------------------------------------------------
         * 7. Advance LUT index — wraps at end of sine cycle
         * ------------------------------------------------------------------ */
        lut_index++;
        if (lut_index >= LUT_SIZE)
        {
            lut_index = 0;
        }

        /* ------------------------------------------------------------------
         * 8. Wait for next control loop cycle
         *    TODO: Replace HAL_Delay with timer interrupt for precise timing
         * ------------------------------------------------------------------ */
        HAL_Delay(LOOP_DELAY_MS);
    }
}

/* -----------------------------------------------------------------------
 * clamp — limits value to [min, max] range
 * ----------------------------------------------------------------------- */
static int32_t clamp(int32_t value, int32_t min, int32_t max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/* -----------------------------------------------------------------------
 * SystemClock_Config
 * Uses internal HSI oscillator, no PLL, 8MHz system clock
 * ----------------------------------------------------------------------- */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_NONE;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
        Error_Handler();
    }
}

/* -----------------------------------------------------------------------
 * MX_DAC_Init
 * DAC Channel 1 (PA4), software trigger, output buffer enabled
 * ----------------------------------------------------------------------- */
static void MX_DAC_Init(void)
{
    DAC_ChannelConfTypeDef sConfig = {0};

    hdac.Instance = DAC;
    if (HAL_DAC_Init(&hdac) != HAL_OK)
    {
        Error_Handler();
    }

    sConfig.DAC_Trigger      = DAC_TRIGGER_NONE;     /* software trigger */
    sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;

    if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
}

/* -----------------------------------------------------------------------
 * MX_TIM2_Init
 * TIM2 in quadrature encoder mode (TI12 — both A and B channels)
 * Period = ROTATION_COUNTS * DAC_MAX = full stroke range
 * Encoder input: PA0 (CH1/A), PA1 (CH2/B)
 * ----------------------------------------------------------------------- */
static void MX_TIM2_Init(void)
{
    TIM_Encoder_InitTypeDef  sConfig      = {0};
    TIM_MasterConfigTypeDef  sMasterConfig = {0};

    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 0;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = ROTATION_COUNTS * DAC_MAX; /* full stroke */
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    /* TI12 mode: counts both rising edges of CH1 and CH2
     * Gives direction detection and 4x resolution vs TI1 only */
    sConfig.EncoderMode   = TIM_ENCODERMODE_TI12;   /* FIX: was TI1 */

    sConfig.IC1Polarity   = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection  = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler  = TIM_ICPSC_DIV1;
    sConfig.IC1Filter     = 0;

    sConfig.IC2Polarity   = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection  = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler  = TIM_ICPSC_DIV1;
    sConfig.IC2Filter     = 0;

    if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;

    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

/* -----------------------------------------------------------------------
 * MX_GPIO_Init
 * Enable GPIOA clock — PA0/PA1 for encoder, PA4 for DAC
 * ----------------------------------------------------------------------- */
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
}

/* -----------------------------------------------------------------------
 * Error_Handler
 * TODO: Add LED blink or UART error log for debugging
 * ----------------------------------------------------------------------- */
void Error_Handler(void)
{
    /* Disable interrupts and halt */
    __disable_irq();
    while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(char *file, uint32_t line)
{
    /* TODO: log file and line number over UART for debug builds */
}
#endif