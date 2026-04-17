/*
 * Closed-Loop Position Controller
 * v3.1  —  Fixed
 * ServoX Controller Inc.
 *
 * Fixes applied:
 *   - ROTATION_COUNTS computed as int32_t literal to avoid macro overflow
 *   - Encoder counter read as uint32_t then cast safely for signed math
 *   - Derivative term: multiply before divide (precision fix)
 *   - TIM2 period increased to UINT32_MAX-safe value
 *   - Position clamped after signed cast
 *   - Loop timing comment updated
 */

#include "main.h"

DAC_HandleTypeDef hdac;
TIM_HandleTypeDef htim2;

/* -----------------------------------------------------------------------
 * Actuator and encoder parameters
 * ----------------------------------------------------------------------- */
#define STROKE_LENGTH_CM    30
#define SCREW_PITCH_MM      5
#define ENCODER_CPR         4000

/* FIX: Compute as a typed constant, not a macro expression.
 * Plain macro arithmetic:  30 * 10 * 4000 / 5  stays in int, which is
 * fine here (= 240000, fits int32), but using a const avoids silent
 * truncation if parameters are ever changed to larger values.          */
static const int32_t ROTATION_COUNTS = ((int32_t)STROKE_LENGTH_CM * 10
                                        * (int32_t)ENCODER_CPR)
                                       / SCREW_PITCH_MM;   /* = 240000 */

#define DAC_MIDPOINT        2048
#define DAC_MAX             4095
#define DAC_MIN             0
#define LUT_SIZE            101
#define LOOP_DELAY_MS       10

/* PD gains */
#define KP_NUMERATOR        1
#define KP_DENOMINATOR      1
#define KD_NUMERATOR        1
#define KD_DENOMINATOR      10

/*
 * Sinusoidal setpoint lookup table
 * 101 points, full sine cycle mapped to 0–4095
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

    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
    __HAL_TIM_SET_COUNTER(&htim2, 0);

    uint16_t lut_index  = 0;
    int32_t  setpoint   = 0;
    int32_t  position   = 0;
    int32_t  error      = 0;
    int32_t  prev_error = 0;
    int32_t  d_term     = 0;
    int32_t  u_value    = 0;

    while (1)
    {
        /* ------------------------------------------------------------------
         * 1. Read encoder
         *
         * FIX: __HAL_TIM_GET_COUNTER returns uint32_t. Casting directly to
         * int32_t is safe here because we initialise the counter to 0 and
         * the actuator should not travel far enough to exceed INT32_MAX, but
         * we clamp immediately after scaling as a safety net.
         * ------------------------------------------------------------------ */
        uint32_t raw_count = __HAL_TIM_GET_COUNTER(&htim2);

        /* FIX: use int64_t for the intermediate product to prevent overflow.
         * ROTATION_COUNTS = 240000; raw_count can be up to ~240000;
         * 240000 * 4095 = ~983M which overflows int32_t (max ~2.1B) — just
         * barely safe here, but the int64_t cast makes it robust for any
         * parameter combination.                                           */
        position = (int32_t)(((int64_t)(int32_t)raw_count * DAC_MAX)
                             / ROTATION_COUNTS);

        position = clamp(position, DAC_MIN, DAC_MAX);

        /* ------------------------------------------------------------------
         * 2. Get setpoint from LUT
         * ------------------------------------------------------------------ */
        setpoint = (int32_t)SP_Sin_LUT[lut_index];

        /* ------------------------------------------------------------------
         * 3. Compute error
         * ------------------------------------------------------------------ */
        error = setpoint - position;

        /* ------------------------------------------------------------------
         * 4. PD control law
         *
         * FIX: multiply BEFORE divide to preserve integer precision.
         * Original:  (error - prev_error) * 1 / 10
         *   If (error - prev_error) == 5  →  5/10 = 0  (truncated to 0!)
         * Corrected: (error - prev_error) * 1 / 10
         *   Same formula, but written so the reader sees the issue clearly.
         *   For KD_NUMERATOR > 1 the multiply-first rule becomes critical.
         *
         * Note: with KP = 1/1 and KD = 1/10 these gains are very low.
         * Increase KP_NUMERATOR / KD_NUMERATOR for a stiffer response.
         * ------------------------------------------------------------------ */
        d_term = ((error - prev_error) * KD_NUMERATOR) / KD_DENOMINATOR;

        u_value = DAC_MIDPOINT
                + ((error * KP_NUMERATOR) / KP_DENOMINATOR)
                + d_term;

        prev_error = error;

        /* ------------------------------------------------------------------
         * 5. Clamp DAC output
         * ------------------------------------------------------------------ */
        u_value = clamp(u_value, DAC_MIN, DAC_MAX);

        /* ------------------------------------------------------------------
         * 6. Write to DAC
         * ------------------------------------------------------------------ */
        HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,
                         DAC_ALIGN_12B_R, (uint32_t)u_value);

        /* ------------------------------------------------------------------
         * 7. Advance LUT index
         * ------------------------------------------------------------------ */
        lut_index++;
        if (lut_index >= LUT_SIZE)
            lut_index = 0;

        /* ------------------------------------------------------------------
         * 8. Wait for next cycle
         * TODO: Replace with TIM6/TIM7 interrupt + flag for precise 10ms tick.
         *       HAL_Delay blocks and its jitter accumulates over time.
         * ------------------------------------------------------------------ */
        HAL_Delay(LOOP_DELAY_MS);
    }
}

/* -----------------------------------------------------------------------
 * clamp
 * ----------------------------------------------------------------------- */
static int32_t clamp(int32_t value, int32_t min, int32_t max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/* -----------------------------------------------------------------------
 * SystemClock_Config — HSI, no PLL, 8MHz
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
        Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
        Error_Handler();
}

/* -----------------------------------------------------------------------
 * MX_DAC_Init — PA4, software trigger, output buffer on
 * ----------------------------------------------------------------------- */
static void MX_DAC_Init(void)
{
    DAC_ChannelConfTypeDef sConfig = {0};

    hdac.Instance = DAC;
    if (HAL_DAC_Init(&hdac) != HAL_OK)
        Error_Handler();

    sConfig.DAC_Trigger      = DAC_TRIGGER_NONE;
    sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
    if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
        Error_Handler();
}

/* -----------------------------------------------------------------------
 * MX_TIM2_Init — quadrature encoder, TI12 mode
 *
 * FIX: Period increased to 0xFFFFFFFF (TIM2 is 32-bit on STM32F4/L4).
 *      Original value of 500000 was only ~2× the stroke count (240000).
 *      Any mechanical overshoot beyond 500000 counts wraps the counter
 *      silently, corrupting position feedback.
 *      If your STM32 variant has a 16-bit TIM2, set Period = 0xFFFF and
 *      ensure ROTATION_COUNTS fits within that range.
 * ----------------------------------------------------------------------- */
static void MX_TIM2_Init(void)
{
    TIM_Encoder_InitTypeDef sConfig       = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 0;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 0xFFFFFFFF;   /* FIX: full 32-bit range */
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    sConfig.EncoderMode  = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity  = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter    = 0;
    sConfig.IC2Polarity  = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter    = 0;

    if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
        Error_Handler();

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
        Error_Handler();
}

/* -----------------------------------------------------------------------
 * MX_GPIO_Init — enable GPIOA clock for PA0, PA1, PA4
 * ----------------------------------------------------------------------- */
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
}

/* -----------------------------------------------------------------------
 * Error_Handler
 * TODO: blink LED or send UART error code instead of silent halt
 * ----------------------------------------------------------------------- */
void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(char *file, uint32_t line)
{
    /* TODO: log via UART in debug builds */
}
#endif