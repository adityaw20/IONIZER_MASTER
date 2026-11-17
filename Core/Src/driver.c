/**
 * @file    driver.c
 * @brief   Ionizer System — Master MCU Driver Implementation
 * @target  STM32F4xx / STM32F1xx (HAL)
 * @author  Ionizer Firmware Team
 * @version 1.0.0
 *
 * Covers:
 *  - GPIO / Relay / Alert control
 *  - RS485 Modbus RTU Master (FC03, FC06) with CRC-16
 *  - ADC — CT1 / CT2 current sensor reading
 *  - Inlet Water Flow Sensor (pulse-count via EXTI)
 *  - GSM AT command layer (timestamp, HTTP POST, CMD receive)
 *  - JSON payload builder
 *  - HMI display update (I2C LCD 16x2 example)
 *  - Ionizer set relay alternation logic
 *  - Independent Watchdog kick
 */

#include "driver.h"

/* ─────────────────────────────────────────────────────────────────────────
   Private Variables
───────────────────────────────────────────────────────────────────────── */

/* Inlet WFS pulse counter — incremented from EXTI ISR */
volatile uint32_t  s_wfs_pulse_count = 0;
static          uint32_t  s_wfs_last_tick   = 0;

/* GSM receive buffer */
static uint8_t  s_gsm_rx_buf[GSM_RESP_BUF_SIZE];

/* Modbus TX/RX buffers */
static uint8_t  s_mb_tx[8];
static uint8_t  s_mb_rx[32];


float flow_lpm=0;
float tds_f=0;
uint32_t time_interval=0,interval=1000;
static float s_total_volume_litres = 0.0f;
uint32_t currentMillis = 0;
uint32_t previousMillis = 0;


float calibrationFactor = 4.5;
volatile uint32_t pulseCount=0;
uint32_t pulse1Sec = 0;
static float litre=0.0f;
float flowRate=0.0f;
static float flowMilliLitres=0.0f;
static float  totalMilliLitres=0.0f;

static uint32_t s_previousMillis    = 0;
static float    s_totalMilliLitres  = 0.0f;
static float    s_litre             = 0.0f;

#define INTERVAL_MS         1000U     /* measure every 1 second    */
#define CALIBRATION_FACTOR  4.5f      /* YF-S201: 7.5 pulses/sec/LPM */
#define WFS_DEBOUNCE_MS     5U     /* ignore pulses faster than 5ms apart */

uint32_t litre_whole=0;
uint32_t milli_part  =0;
uint32_t rate_whole =0;
uint32_t rate_dec=0;



extern TIM_HandleTypeDef htim4;

uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0,duration=0;

uint8_t Is_First_Captured = 0;

float Distance = 0.0f;

float tdsValue=0.0f;
const float VREF = 3.3;      // ESP32 ADC reference
const int ADC_RES = 4095;    // 12-bit ADC

float temperature = 25.0;    // Water temperature
float voltage=0;
float adcAverage=0;
/* ─────────────────────────────────────────────────────────────────────────
   1.  DRIVER INITIALIZATION
───────────────────────────────────────────────────────────────────────── */

/**
 * @brief  Initialise driver layer; populate system handle with HAL handles.
 *         Call once from main() after all HAL peripherals are initialised.
 */
DRV_Status DRV_Init(IonizerSystem_t *sys)
{
    if (sys == NULL) return DRV_INVALID_PARAM;

    /* Attach HAL handles */
    sys->huart_rs485       = &huart1;
    sys->huart_gsm         = &huart2;
    sys->hadc              = &hadc1;
    sys->htim_alternate    = &htim1;
   // sys->hiwdg             = &hiwdg;

    /* Default state */
    sys->ionizer_on        = false;
    sys->active_set        = IONIZER_SET_NONE;
    sys->alternate_timer_ms = 0;
    sys->solenoid_status   = SOLENOID_CLOSE;

    /* Ensure relays OFF on startup */
    DRV_Relay1_Set(false);
    DRV_Relay2_Set(false);
 //   DRV_RedLED_Set(false);
 //   DRV_Buzzer_Set(false);

    /* RS485 in receive mode */
    RS485_RX_ENABLE();

    /* HMI init */
 //   DRV_HMI_Init();

    /* WFS timer reference */
    s_wfs_last_tick = HAL_GetTick();

    DBG_PRINT("Driver Init OK — FW v%d.%d.%d",
              FW_VERSION_MAJOR, FW_VERSION_MINOR, FW_VERSION_PATCH);

    return DRV_OK;
}

/* ─────────────────────────────────────────────────────────────────────────
   2.  GPIO / RELAY / ALERT CONTROL
───────────────────────────────────────────────────────────────────────── */

void DRV_Relay1_Set(bool state)
{
    HAL_GPIO_WritePin(RELAY1_PORT, RELAY1_PIN,
                      state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void DRV_Relay2_Set(bool state)
{
    HAL_GPIO_WritePin(RELAY2_PORT, RELAY2_PIN,
                      state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void DRV_RedLED_Set(bool state)
{
    HAL_GPIO_WritePin(RED_LED_PORT, RED_LED_PIN,
                      state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void DRV_Buzzer_Set(bool state)
{
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN,
                      state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief  Beep the buzzer N times.
 * @param  count   Number of beeps
 * @param  on_ms   ON duration per beep (ms)
 * @param  off_ms  OFF duration between beeps (ms)
 */
void DRV_Buzzer_Beep(uint8_t count, uint32_t on_ms, uint32_t off_ms)
{
    for (uint8_t i = 0; i < count; i++) {
        DRV_Buzzer_Set(true);
        HAL_Delay(on_ms);
        DRV_Buzzer_Set(false);
        if (i < count - 1) HAL_Delay(off_ms);
    }
}

/**
 * @brief  Read Tank A HIGH limit sensor.
 * @return true if HIGH limit is reached (float switch active)
 * @note   Sensor is active LOW — GPIO reads 0 when triggered.
 */
bool DRV_TankA_HighLimit(void)
{
    return (HAL_GPIO_ReadPin(TANK_A_HIGH_PORT, TANK_A_HIGH_PIN) == GPIO_PIN_RESET);
}

/**
 * @brief  Read Tank A LOW limit sensor.
 * @return true if LOW limit is reached
 */
bool DRV_TankA_LowLimit(void)
{
    return (HAL_GPIO_ReadPin(TANK_A_LOW_PORT, TANK_A_LOW_PIN) == GPIO_PIN_RESET);
}

/**
 * @brief  Read Booster Tank level sensor.
 * @return true if level is BELOW minimum threshold
 */
bool DRV_Booster_LevelLow(void)
{
    return (HAL_GPIO_ReadPin(BOOSTER_LEVEL_PORT, BOOSTER_LEVEL_PIN) == GPIO_PIN_RESET);
}




/**
 * @brief  Trigger AJ-SR04M and measure distance
 * @return Distance in cm, or -1 on timeout/error
 */
void JSN_SR04_Trigger(void)
{



	    /* Send trigger pulse */
	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	    delay_us(2);
	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
	    delay_us(10);
	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

}




void delay_us(uint16_t us)
{
	/* Enable DWT if not already */
	    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	    DWT->CYCCNT = 0;
	    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

	    uint32_t ticks = us * (SystemCoreClock / 1000000U);
	    uint32_t start = DWT->CYCCNT;
	    while ((DWT->CYCCNT - start) < ticks);
}

void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t Ultrasonic_ReadPulse(GPIO_TypeDef *port,
                              uint16_t pin,
                              uint32_t timeout_us)
{
    uint32_t startTick;

    /* Wait for pin to go HIGH */
    startTick = DWT->CYCCNT;
    while(HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET)
    {
        if(((DWT->CYCCNT - startTick) /
            (SystemCoreClock / 1000000U)) > timeout_us)
        {
            return 0;   // Timeout
        }
    }

    /* Start timing */
    uint32_t pulseStart = DWT->CYCCNT;

    /* Wait for pin to go LOW */
    while(HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET)
    {
        if(((DWT->CYCCNT - pulseStart) /
            (SystemCoreClock / 1000000U)) > timeout_us)
        {
            return 0;   // Timeout
        }
    }

    /* Pulse width in µs */
    uint32_t pulseWidth =
        (DWT->CYCCNT - pulseStart) /
        (SystemCoreClock / 1000000U);

    return pulseWidth;
}

void read_distance(IonizerSystem_t *sys)
{
	JSN_SR04_Trigger();

	 duration =
	    Ultrasonic_ReadPulse(GPIOB,
	                         GPIO_PIN_7,
	                         30000);

	if(duration > 0)
	{
	     Distance = (duration * 0.0343f) / 2.0f;
	    printf("Distance = %.2f cm\r\n", Distance);
	   if( Distance <= 10)
	   {
		   sys->Booster_level_percent= ((60 - Distance) * 100) / 60;
	   }
	   else if(Distance >= 10 &&  Distance <= 50)
	   {
		   sys->Booster_level_percent= ((60 - Distance) * 100) / 60;
	   }
	   else if(Distance >= 50)
	   {
		   sys->Booster_level_percent= ((60 - Distance) * 100) / 60;
	   }

	}
	else
	{
	    printf("No Echo\r\n");
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance != TIM4) return;

	    if (Is_First_Captured == 0)
	    {
	        /* Rising edge — record start time */
	        IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
	        Is_First_Captured = 1;

	        /* Switch to falling edge */
	        __HAL_TIM_SET_CAPTUREPOLARITY(htim,
	                                      TIM_CHANNEL_1,
	                                      TIM_INPUTCHANNELPOLARITY_FALLING);
	    }
	    else
	    {
	        /* Falling edge — record end time */
	        IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

	        /* Stop capture — measurement complete */
	        HAL_TIM_IC_Stop_IT(&htim4, TIM_CHANNEL_1);

	        /* Calculate with overflow handling */
	        Difference = (IC_Val2 >= IC_Val1)
	                     ? (IC_Val2 - IC_Val1)
	                     : (65535U - IC_Val1 + IC_Val2);

	        Distance = (Difference * 0.0343f) / 2.0f;

	        Is_First_Captured = 0;
	        /*  Do NOT restart here — let main loop call JSN_SR04_Trigger() */
	    }
}



/* ─────────────────────────────────────────────────────────────────────────
   3.  CRC-16 MODBUS
───────────────────────────────────────────────────────────────────────── */

/**
 * @brief  Calculate Modbus CRC-16 (polynomial 0xA001).
 * @param  data    Pointer to data bytes
 * @param  length  Number of bytes
 * @return 16-bit CRC value
 */
uint16_t DRV_CRC16_Modbus(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

/* ─────────────────────────────────────────────────────────────────────────
   4.  RS485 / MODBUS RTU MASTER
───────────────────────────────────────────────────────────────────────── */

/**
 * @brief  FC03 — Read Holding Registers from a Modbus slave.
 *
 * Request frame  : [Addr][FC03][RegHi][RegLo][NumHi][NumLo][CRCLo][CRCHi]
 * Response frame : [Addr][FC03][ByteCount][D0Hi][D0Lo]...[CRCLo][CRCHi]
 *
 * @param  huart       Pointer to RS485 UART handle
 * @param  slave_addr  Slave device address (0x01 or 0x02)
 * @param  start_reg   Starting register address
 * @param  num_regs    Number of registers to read (1–10)
 * @param  out_data    Buffer to store register values (uint16_t array)
 * @return DRV_OK on success, error code otherwise
 */
DRV_Status DRV_Modbus_ReadHR(
    UART_HandleTypeDef *huart,
    uint8_t   slave_addr,
    uint16_t  start_reg,
    uint8_t   num_regs,
    uint16_t *out_data)
{
    if (!huart || !out_data || num_regs == 0 || num_regs > 10)
        return DRV_INVALID_PARAM;

    /* ── Build request frame ── */
    s_mb_tx[0] = slave_addr;
    s_mb_tx[1] = FC03_READ_HR;
    s_mb_tx[2] = (uint8_t)(start_reg >> 8);
    s_mb_tx[3] = (uint8_t)(start_reg & 0xFF);
    s_mb_tx[4] = 0x00;
    s_mb_tx[5] = num_regs;
    uint16_t crc = DRV_CRC16_Modbus(s_mb_tx, 6);
    s_mb_tx[6] = (uint8_t)(crc & 0xFF);
    s_mb_tx[7] = (uint8_t)(crc >> 8);

    /* ── Transmit ── */
    RS485_TX_ENABLE();
    HAL_Delay(1);  /* DE/RE settling time */
    HAL_UART_Transmit(huart, s_mb_tx, 8, 100);
   // HAL_Delay(1);
    RS485_RX_ENABLE();
    HAL_Delay(1);
    /* ── Receive response ── */
    uint8_t  expected_len = 5 + (num_regs * 2);  /* header + data + CRC   */
    memset(s_mb_rx, 0, sizeof(s_mb_rx));
    HAL_Delay(1);
    HAL_StatusTypeDef rx_status = HAL_UART_Receive(huart, s_mb_rx, 32, MODBUS_TIMEOUT_MS);
    UART_Restart();
    if (rx_status == HAL_TIMEOUT) {
        DBG_PRINT("Modbus FC03 timeout | Slave=0x%02X Reg=0x%04X", slave_addr, start_reg);
 //       return DRV_MODBUS_TIMEOUT;
    }

    /* ── Validate address and function code ── */
  /*  if (s_mb_rx[0] != slave_addr || s_mb_rx[1] != FC03_READ_HR)
        return DRV_ERROR;
*/
    /* ── Validate CRC ── */
    uint16_t rx_crc    = ((uint16_t)s_mb_rx[expected_len - 1] << 8)
                        | (uint16_t)s_mb_rx[expected_len - 2];
    uint16_t calc_crc  = DRV_CRC16_Modbus(s_mb_rx, expected_len - 2);
    if (rx_crc != calc_crc) {
        DBG_PRINT("Modbus CRC error | Slave=0x%02X", slave_addr);
        return DRV_MODBUS_CRC_ERR;
    }

    /* ── Extract register values ── */
    for (uint8_t i = 0; i < num_regs; i++) {
        out_data[i] = ((uint16_t)s_mb_rx[3 + (i * 2)] << 8)
                     | (uint16_t)s_mb_rx[4 + (i * 2)];
    }

    return DRV_OK;
}

/**
 * @brief  FC06 — Write a Single Register to a Modbus slave.
 *
 * Request frame  : [Addr][FC06][RegHi][RegLo][ValHi][ValLo][CRCLo][CRCHi]
 * Response frame : echo of request (8 bytes)
 *
 * @param  huart       Pointer to RS485 UART handle
 * @param  slave_addr  Slave device address
 * @param  reg_addr    Register address to write
 * @param  value       Value to write
 * @return DRV_OK on success, error code otherwise
 */
DRV_Status DRV_Modbus_WriteSR(
    UART_HandleTypeDef *huart,
    uint8_t   slave_addr,
    uint16_t  reg_addr,
    uint16_t  value)
{
    if (!huart) return DRV_INVALID_PARAM;

    /* ── Build request frame ── */
    s_mb_tx[0] = slave_addr;
    s_mb_tx[1] = FC06_WRITE_SR;
    s_mb_tx[2] = (uint8_t)(reg_addr >> 8);
    s_mb_tx[3] = (uint8_t)(reg_addr & 0xFF);
    s_mb_tx[4] = (uint8_t)(value >> 8);
    s_mb_tx[5] = (uint8_t)(value & 0xFF);
    uint16_t crc = DRV_CRC16_Modbus(s_mb_tx, 6);
    s_mb_tx[6] = (uint8_t)(crc & 0xFF);
    s_mb_tx[7] = (uint8_t)(crc >> 8);

    /* ── Transmit ── */
    RS485_TX_ENABLE();
    HAL_Delay(1);
    HAL_UART_Transmit(huart, s_mb_tx, 8, 50);
    HAL_Delay(1);
    RS485_RX_ENABLE();

    /* ── Receive echo (8 bytes) ── */
    memset(s_mb_rx, 0, sizeof(s_mb_rx));
    HAL_StatusTypeDef rx_status =
        HAL_UART_Receive(huart, s_mb_rx, 8, MODBUS_TIMEOUT_MS);

    if (rx_status == HAL_TIMEOUT) {
        DBG_PRINT("Modbus FC06 timeout | Slave=0x%02X Reg=0x%04X", slave_addr, reg_addr);
        return DRV_MODBUS_TIMEOUT;
    }

    /* ── Validate echo ── */
    if (memcmp(s_mb_tx, s_mb_rx, 8) != 0)
        return DRV_ERROR;

    return DRV_OK;
}

/* ─────────────────────────────────────────────────────────────────────────
   5.  ADC — CT SENSOR READING
───────────────────────────────────────────────────────────────────────── */

/**
 * @brief  Read CT1 (Ionizer Set 1) current via ADC Channel 0.
 * @param  hadc     Pointer to ADC handle
 * @param  out_raw  Pointer to store raw 12-bit ADC value
 * @return DRV_OK on success
 */
DRV_Status DRV_ADC_ReadCT1(ADC_HandleTypeDef *hadc, uint16_t *out_raw)
{
    if (!hadc || !out_raw) return DRV_INVALID_PARAM;

    ADC_ChannelConfTypeDef cfg = {0};
    cfg.Channel      = CT1_ADC_CHANNEL;
    cfg.Rank         = 2;
    cfg.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    HAL_ADC_ConfigChannel(hadc, &cfg);

    HAL_ADC_Start(hadc);
    if (HAL_ADC_PollForConversion(hadc, 10) != HAL_OK) {
        HAL_ADC_Stop(hadc);
        return DRV_TIMEOUT;
    }
    *out_raw = (uint16_t)HAL_ADC_GetValue(hadc);
    HAL_ADC_Stop(hadc);
    return DRV_OK;
}

/**
 * @brief  Read CT2 (Ionizer Set 2) current via ADC Channel 1.
 */
DRV_Status DRV_ADC_ReadCT2(ADC_HandleTypeDef *hadc, uint16_t *out_raw)
{
    if (!hadc || !out_raw) return DRV_INVALID_PARAM;

    ADC_ChannelConfTypeDef cfg = {0};
    cfg.Channel      = CT2_ADC_CHANNEL;
    cfg.Rank         = 3;
    cfg.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    HAL_ADC_ConfigChannel(hadc, &cfg);

    HAL_ADC_Start(hadc);
    if (HAL_ADC_PollForConversion(hadc, 10) != HAL_OK) {
        HAL_ADC_Stop(hadc);
        return DRV_TIMEOUT;
    }
    *out_raw = (uint16_t)HAL_ADC_GetValue(hadc);
    HAL_ADC_Stop(hadc);
    return DRV_OK;
}

/**
 * @brief  Read CT3 (Ionizer Set 2) current via ADC Channel 1.
 */
DRV_Status DRV_ADC_ReadCT3(ADC_HandleTypeDef *hadc, uint16_t *out_raw)
{
    if (!hadc || !out_raw) return DRV_INVALID_PARAM;

    ADC_ChannelConfTypeDef cfg = {0};
    cfg.Channel      = CT3_ADC_CHANNEL;
    cfg.Rank         = 4;
    cfg.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    HAL_ADC_ConfigChannel(hadc, &cfg);

    HAL_ADC_Start(hadc);
    if (HAL_ADC_PollForConversion(hadc, 10) != HAL_OK) {
        HAL_ADC_Stop(hadc);
        return DRV_TIMEOUT;
    }
    *out_raw = (uint16_t)HAL_ADC_GetValue(hadc);
    HAL_ADC_Stop(hadc);
    return DRV_OK;
}

/**
 * @brief  Read CT4 (Ionizer Set 2) current via ADC Channel 1.
 */
DRV_Status DRV_ADC_ReadCT4(ADC_HandleTypeDef *hadc, uint16_t *out_raw)
{
    if (!hadc || !out_raw) return DRV_INVALID_PARAM;

    ADC_ChannelConfTypeDef cfg = {0};
    cfg.Channel      = CT4_ADC_CHANNEL;
    cfg.Rank         = 5;
    cfg.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    HAL_ADC_ConfigChannel(hadc, &cfg);

    HAL_ADC_Start(hadc);
    if (HAL_ADC_PollForConversion(hadc, 10) != HAL_OK) {
        HAL_ADC_Stop(hadc);
        return DRV_TIMEOUT;
    }
    *out_raw = (uint16_t)HAL_ADC_GetValue(hadc);
    HAL_ADC_Stop(hadc);
    return DRV_OK;
}


/**
 * @brief  Check whether a CT ADC reading is within normal operating range.
 * @param  raw  12-bit ADC value
 * @return true if fault (open circuit or overcurrent)
 */
bool DRV_CT_IsFault(uint16_t raw)
{
    return (raw < CT_MIN_THRESHOLD || raw > CT_MAX_THRESHOLD);
}

/* ─────────────────────────────────────────────────────────────────────────
   6.  INLET WATER FLOW SENSOR  (EXTI pulse counting)
───────────────────────────────────────────────────────────────────────── */

/**
 * @brief  Call this from the EXTI ISR for the inlet WFS GPIO pin.
 *         Example: In stm32f4xx_it.c inside EXTI15_10_IRQHandler().
 */
void DRV_WFS_PulseCallback(void)
{

	     /*    Startup stabilization
	        if (HAL_GetTick() < 2000U) return;

	         Debounce — ignore if pulse came too soon
	        static uint32_t last_pulse_tick = 0;
	        uint32_t now = HAL_GetTick();

	        if ((now - last_pulse_tick) < WFS_DEBOUNCE_MS) return;   noise → skip

	        last_pulse_tick   = now;*/
	        s_wfs_pulse_count++;

}

/**
 * @brief  Compute flow rate in L/min from accumulated pulses.
 *         Call periodically (e.g. every 1 second).
 *
 *         Calibration (typical Hall-effect sensor):
 *           Flow (L/min) = pulse_count / (7.5 * delta_t_sec)
 *         Adjust divisor based on your sensor datasheet.
 *
 * @return Flow rate in L/min; resets internal pulse counter.
 */

/*float DRV_WFS_GetFlowLPM(void)
{

      currentMillis = HAL_GetTick();
      if((currentMillis - previousMillis) >= interval)
      {
        pulse1Sec = s_wfs_pulse_count;
        s_wfs_pulse_count = 0;
        flowRate = (((1000 / (HAL_GetTick() - previousMillis)) * pulse1Sec) / calibrationFactor);
        previousMillis = currentMillis;
        flowMilliLitres = (flowRate / 60) * 1000;

        totalMilliLitres += flowMilliLitres;
        litre = totalMilliLitres / 1000;

      }
      return litre;
}*/

float DRV_WFS_GetFlowLPM(void)
{
    uint32_t currentMillis = HAL_GetTick();
    static bool s_first_reading_done = false;

    if ((currentMillis - s_previousMillis) >= INTERVAL_MS)
    {

        float delta_ms = (float)(currentMillis - s_previousMillis);

        __disable_irq();
        pulse1Sec  = s_wfs_pulse_count;
        s_wfs_pulse_count     = 0;
        __enable_irq();

        s_previousMillis = currentMillis;

        if (!s_first_reading_done)
                {
                    s_first_reading_done = true;
                    DBG_PRINT("WFS: First reading discarded (startup noise)");
                    HAL_Delay(10);
                    return 0.0f;
                }

        float delta_sec  = delta_ms / 1000.0f;
        flowRate   = 0;
        flowRate   = ((float)pulse1Sec / CALIBRATION_FACTOR) / delta_sec ;//* 60.0f;

        float flowMilliLitres  = (flowRate / 60.0f) * 1000.0f * delta_sec;
        s_totalMilliLitres    += flowMilliLitres;
        s_litre                = s_totalMilliLitres / 1000.0f;
       // s_litre                = s_litre/10;
    }

    return s_litre;
}

/*
float DRV_WFS_GetTotalLitres(bool reset)
{
	  uint32_t now       = HAL_GetTick();
	    uint32_t delta_ms  = now - s_wfs_last_tick;
	    s_wfs_last_tick    = now;

	    uint32_t pulses    = s_wfs_pulse_count;
	    s_wfs_pulse_count  = 0;

	    if (delta_ms == 0) return 0.0f;

	    float delta_sec = (float)delta_ms / 1000.0f;
	     Sensor factor: 7.5 pulses/sec = 1 L/min  (adjust per datasheet)
	     flow_lpm = (float)pulses / (7.5f * delta_sec);
	    return flow_lpm;





    if (reset) {
        s_total_volume_litres = 0.0f;
        return 0.0f;
    }

     Atomically grab and clear pulse count (ISR writes this)
    __disable_irq();
    uint32_t pulses   = s_wfs_pulse_count;
    s_wfs_pulse_count = 0;
    __enable_irq();

     YF-S201: 450 pulses = 1 Litre
    s_total_volume_litres += (float)pulses / 450.0f;

    return s_total_volume_litres;




}
*/




/* ─────────────────────────────────────────────────────────────────────────
   7.  GSM / AT COMMAND LAYER
───────────────────────────────────────────────────────────────────────── */

/**
 * @brief  Low-level helper — send AT command and wait for a keyword
 *         in the response.
 * @param  huart    GSM UART handle
 * @param  cmd      AT command string (null-terminated)
 * @param  expect   Expected response keyword (e.g. "OK", "+CCLK")
 * @param  timeout  Timeout in ms
 * @param  out_buf  Buffer to store full response (or NULL)
 * @param  out_len  Size of out_buf
 * @return DRV_OK if keyword found, DRV_GSM_NO_RESP otherwise
 */
static DRV_Status gsm_send_cmd(UART_HandleTypeDef *huart,
                                const char *cmd,
                                const char *expect,
                                uint32_t    timeout,
                                char       *out_buf,
                                uint16_t    out_len)
{
    memset(s_gsm_rx_buf, 0, sizeof(s_gsm_rx_buf));

    /* Transmit command */
    HAL_UART_Transmit(huart, (uint8_t *)cmd, strlen(cmd), 100);

    /* Receive response — non-blocking accumulation via polling */
    uint32_t start   = HAL_GetTick();
    uint16_t idx     = 0;
    uint8_t  byte    = 0;

    while ((HAL_GetTick() - start) < timeout) {
        if (HAL_UART_Receive(huart, &byte, 1, 5) == HAL_OK) {
            if (idx < GSM_RESP_BUF_SIZE - 1)
                s_gsm_rx_buf[idx++] = (char)byte;
        }
        if (strstr((char *)s_gsm_rx_buf, expect) != NULL) break;
    }

    if (out_buf && out_len > 0)
        strncpy(out_buf, (char *)s_gsm_rx_buf, out_len - 1);

    return (strstr((char *)s_gsm_rx_buf, expect) != NULL) ? DRV_OK : DRV_GSM_NO_RESP;
}

/**
 * @brief  Verify GSM module is alive with AT command.
 */
DRV_Status DRV_GSM_Check(UART_HandleTypeDef *huart)
{
    return gsm_send_cmd(huart, GSM_AT_CHECK, "OK",
                        GSM_TIMEOUT_SHORT_MS, NULL, 0);
}

/**
 * @brief  Read current timestamp from GSM module using AT+CCLK?
 *
 * Typical response: +CCLK: "24/03/16,10:30:45+22"
 *
 * @param  huart   GSM UART handle
 * @param  out_ts  Buffer to store extracted timestamp string
 * @param  len     Buffer length (min 32)
 * @return DRV_OK on success
 */
DRV_Status DRV_GSM_GetTimestamp(UART_HandleTypeDef *huart, char *out_ts, uint8_t len)
{
    char resp[GSM_RESP_BUF_SIZE] = {0};
    DRV_Status st = gsm_send_cmd(huart, GSM_AT_CCLK, "+CCLK",
                                  GSM_TIMEOUT_SHORT_MS, resp, sizeof(resp));
    if (st != DRV_OK) return st;

    /* Parse: extract string between first quote and last quote */
    char *start = strchr(resp, '"');
    if (start == NULL) return DRV_ERROR;
    start++;
    char *end = strchr(start, '"');
    if (end == NULL) return DRV_ERROR;

    uint8_t ts_len = (uint8_t)(end - start);
    if (ts_len >= len) ts_len = len - 1;
    strncpy(out_ts, start, ts_len);
    out_ts[ts_len] = '\0';
    return DRV_OK;
}

/**
 * @brief  Send JSON payload to cloud server via HTTP POST using GSM.
 *
 *    AT command sequence:
 *    AT+HTTPINIT
 *    AT+HTTPPARA="CID",1
 *    AT+HTTPPARA="URL","<url>"
 *    AT+HTTPPARA="CONTENT","application/json"
 *    AT+HTTPDATA=<len>,<timeout>
 *    <payload>
 *    AT+HTTPACTION=1     (POST)
 *    AT+HTTPREAD
 *    AT+HTTPTERM
 *
 * @param  huart        GSM UART handle
 * @param  url          Server URL string
 * @param  json_payload JSON string to POST
 * @return DRV_OK on success, DRV_GSM_HTTP_FAIL on error
 */
DRV_Status DRV_GSM_SendHTTP(UART_HandleTypeDef *huart,
                             const char *url,
                             const char *json_payload)
{
    char cmd[JSON_BUF_SIZE + 64];
    DRV_Status st;

    /* Init HTTP */
    st = gsm_send_cmd(huart, "AT+HTTPINIT\r\n", "OK",
                       GSM_TIMEOUT_SHORT_MS, NULL, 0);
    if (st != DRV_OK) goto http_fail;

    /* Set CID */
    gsm_send_cmd(huart, "AT+HTTPPARA=\"CID\",1\r\n", "OK",
                  GSM_TIMEOUT_SHORT_MS, NULL, 0);

    /* Set URL */
    snprintf(cmd, sizeof(cmd), "AT+HTTPPARA=\"URL\",\"%s\"\r\n", url);
    gsm_send_cmd(huart, cmd, "OK", GSM_TIMEOUT_SHORT_MS, NULL, 0);

    /* Set Content-Type */
    gsm_send_cmd(huart, "AT+HTTPPARA=\"CONTENT\",\"application/json\"\r\n",
                  "OK", GSM_TIMEOUT_SHORT_MS, NULL, 0);

    /* Upload data */
    uint16_t payload_len = (uint16_t)strlen(json_payload);
    snprintf(cmd, sizeof(cmd), "AT+HTTPDATA=%d,3000\r\n", payload_len);
    st = gsm_send_cmd(huart, cmd, "DOWNLOAD", GSM_TIMEOUT_SHORT_MS, NULL, 0);
    if (st != DRV_OK) goto http_fail;

    /* Send payload */
    HAL_UART_Transmit(huart, (uint8_t *)json_payload, payload_len, 500);
    HAL_Delay(200);

    /* Execute POST */
    st = gsm_send_cmd(huart, "AT+HTTPACTION=1\r\n", "+HTTPACTION",
                       GSM_TIMEOUT_LONG_MS, NULL, 0);
    if (st != DRV_OK) goto http_fail;

    /* Terminate */
    gsm_send_cmd(huart, "AT+HTTPTERM\r\n", "OK",
                  GSM_TIMEOUT_SHORT_MS, NULL, 0);

    DBG_PRINT("HTTP POST OK — %d bytes sent", payload_len);
    return DRV_OK;

http_fail:
    gsm_send_cmd(huart, "AT+HTTPTERM\r\n", "OK",
                  GSM_TIMEOUT_SHORT_MS, NULL, 0);
    DBG_PRINT("HTTP POST FAILED");
    return DRV_GSM_HTTP_FAIL;
}

/**
 * @brief  Check GSM receive buffer for a pending server command
 *         (e.g. solenoid OPEN / CLOSE instruction sent via SMS or HTTP push).
 *
 *  Expected format in received data: "CMD:OPEN" or "CMD:CLOSE"
 *
 * @param  huart    GSM UART handle
 * @param  out_cmd  Buffer to store extracted command
 * @param  len      Buffer length
 * @return DRV_OK if a command was received
 */
DRV_Status DRV_GSM_ReceiveCMD(UART_HandleTypeDef *huart, char *out_cmd, uint8_t len)
{
    char resp[GSM_RESP_BUF_SIZE] = {0};

    /* Non-blocking read — short timeout */
    uint8_t  byte  = 0;
    uint16_t idx   = 0;
    while (HAL_UART_Receive(huart, &byte, 1, 5) == HAL_OK && idx < sizeof(resp) - 1)
        resp[idx++] = (char)byte;

    if (idx == 0) return DRV_GSM_NO_RESP;

    /* Look for command token */
    char *token = strstr(resp, "CMD:");
    if (token == NULL) return DRV_GSM_NO_RESP;

    token += 4;  /* skip "CMD:" */
    strncpy(out_cmd, token, len - 1);
    out_cmd[len - 1] = '\0';

    /* Strip trailing whitespace / CR / LF */
    for (int8_t i = (int8_t)strlen(out_cmd) - 1; i >= 0; i--) {
        if (out_cmd[i] == '\r' || out_cmd[i] == '\n' || out_cmd[i] == ' ')
            out_cmd[i] = '\0';
        else
            break;
    }

    DBG_PRINT("GSM CMD received: %s", out_cmd);
    return DRV_OK;
}

/* ─────────────────────────────────────────────────────────────────────────
   8.  JSON PAYLOAD BUILDER
───────────────────────────────────────────────────────────────────────── */

/**
 * @brief  Serialise the IonizerSystem_t structure into a JSON string.
 *
 * Output format:
 * {
 *   "ts":"...",
 *   "inlet_flow":x.xx,
 *   "slave1":{"flow":x.xx},
 *   "slave2":{"flow":x.xx,"tds":xxx,"solenoid":x},
 *   "ionizer":{"active_set":x,"on":x,"ct1":xxxx,"ct2":xxxx},
 *   "tank_a":{"high":x,"low":x},
 *   "booster_low":x,
 *   "faults":{"tds":x,"boost":x,"tank_a":x,"ct1":x,"ct2":x,"mb_s1":x,"mb_s2":x}
 * }
 *
 * @param  sys       Pointer to system data structure
 * @param  out_buf   Output buffer
 * @param  buf_size  Buffer size (recommended >= JSON_BUF_SIZE)
 */
void DRV_JSON_Build(const IonizerSystem_t *sys, char *out_buf, uint16_t buf_size)
{
    if (!sys || !out_buf) return;

    snprintf(out_buf, buf_size,
        "{"
        "\"ts\":\"%s\","
        "\"inlet_flow\":%.2f,"
        "\"slave1\":{\"flow\":%.2f},"
        "\"slave2\":{\"flow\":%.2f,\"tds\":%u,\"solenoid\":%u},"
        "\"ionizer\":{\"active_set\":%d,\"on\":%d,\"ct1\":%u,\"ct2\":%u},"
        "\"tank_a\":{\"high\":%d,\"low\":%d},"
        "\"booster_low\":%d,"
        "\"faults\":{"
          "\"tds\":%d,\"boost\":%d,\"tank_a\":%d,"
          "\"ct1\":%d,\"ct2\":%d,\"mb_s1\":%d,\"mb_s2\":%d"
        "}"
        "}",
        sys->timestamp,
        (double)sys->inlet_flow_lpm,
        (double)sys->slave1_flow_lpm,
        (double)sys->slave2_flow_lpm,
        sys->tds_ppm,
        sys->solenoid_status,
        (int)sys->active_set,
        sys->ionizer_on ? 1 : 0,
        sys->ct1_raw,
        sys->ct2_raw,
        sys->tank_a_high_limit ? 1 : 0,
        sys->tank_a_low_limit  ? 1 : 0,
        sys->booster_level_low ? 1 : 0,
        sys->fault_tds_out_of_range ? 1 : 0,
        sys->fault_booster_low      ? 1 : 0,
        sys->fault_tank_a_limit     ? 1 : 0,
        sys->fault_ct1              ? 1 : 0,
        sys->fault_ct2              ? 1 : 0,
        sys->fault_modbus_s1        ? 1 : 0,
        sys->fault_modbus_s2        ? 1 : 0
    );


    snprintf(out_buf, buf_size,
        "{"
            "\"device\":{"
                "\"id\":\"ION001\","
                "\"type\":\"IONIZER\","
                "\"status\":\"ACTIVE\""
            "},"
            "\"timestamp\":\"%s\","
            "\"inlet\":{"
                "\"flow\":%.2f,"
                "\"tds\":%u"
            "},"
            "\"outlet\":{"
                "\"flow\":%.2f,"
                "\"tds\":%u"
            "},"
            "\"boosterTank\":{"
                "\"level\":%d,"
                "\"status\":\"MEDIUM\""
            "},"
            "\"userTank\":{"
                "\"level\":10,"
                "\"status\":\"LOW\","
                "\"solenoidValve\":\"OPEN\""
            "},"
            "\"user\":{"
                "\"outletFlow\":50,"
                "\"requestedWater\":10"
            "}"
        "}",sys->timestamp,(double)sys->inlet_flow_lpm,sys->tds_ppm,(double)sys->slave2_flow_lpm,sys->tds_ppm,(double)sys->slave1_flow_lpm,sys->Booster_level_percent);
}

/* ─────────────────────────────────────────────────────────────────────────
   9.  HMI DISPLAY (I2C LCD 16×2 example — adapt for TFT / UART display)
───────────────────────────────────────────────────────────────────────── */

/* I2C LCD constants (PCF8574 backpack — common with 0x27 or 0x3F address) */
#define LCD_ADDR        (0x27 << 1)
#define LCD_CMD         0x00
#define LCD_DATA        0x01

static void lcd_send_nibble(uint8_t nibble, uint8_t rs)
{
    uint8_t data[4];
    /* Backlight on (bit 3), EN toggle */
    data[0] = (nibble & 0xF0) | 0x08 | (rs ? 0x01 : 0x00) | 0x04;  /* EN=1 */
    data[1] = data[0];
    data[2] = (nibble & 0xF0) | 0x08 | (rs ? 0x01 : 0x00);          /* EN=0 */
    data[3] = data[2];
  //  extern I2C_HandleTypeDef hi2c1;
 //   HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, data, 4, 10);
}

static void lcd_write_byte(uint8_t byte, uint8_t rs)
{
    lcd_send_nibble(byte & 0xF0, rs);
    lcd_send_nibble((byte << 4) & 0xF0, rs);
}

static void lcd_set_cursor(uint8_t row, uint8_t col)
{
    uint8_t addr = (row == 0) ? 0x80 : 0xC0;
    addr += col;
    lcd_write_byte(addr, LCD_CMD);
}

static void lcd_print(const char *str)
{
    while (*str) {
        lcd_write_byte((uint8_t)*str++, LCD_DATA);
    }
}

/**
 * @brief  Initialise the I2C LCD display in 4-bit mode.
 */
DRV_Status DRV_HMI_Init(void)
{
    HAL_Delay(50);
    lcd_send_nibble(0x30, LCD_CMD); HAL_Delay(5);
    lcd_send_nibble(0x30, LCD_CMD); HAL_Delay(1);
    lcd_send_nibble(0x30, LCD_CMD); HAL_Delay(1);
    lcd_send_nibble(0x20, LCD_CMD); HAL_Delay(1);  /* 4-bit mode */

    lcd_write_byte(0x28, LCD_CMD);  /* 2-line, 5×8 font */
    lcd_write_byte(0x0C, LCD_CMD);  /* Display ON, cursor OFF */
    lcd_write_byte(0x06, LCD_CMD);  /* Entry mode: increment */
    lcd_write_byte(0x01, LCD_CMD);  /* Clear display */
    HAL_Delay(2);

    return DRV_OK;
}

/**
 * @brief  Update the HMI LCD with current system status.
 *         Line 1: TDS value and Ionizer Set state
 *         Line 2: Flow rate and alert summary
 */
void DRV_HMI_Update(const IonizerSystem_t *sys)
{
    char line[17] = {0};

    /* Line 1: TDS + Active Set */
    DRV_HMI_Clear();
    lcd_set_cursor(0, 0);
    snprintf(line, sizeof(line), "TDS:%3uppm S%d%s",
             sys->tds_ppm,
             (int)sys->active_set,
             sys->ionizer_on ? " ON " : " OFF");
    lcd_print(line);

    /* Line 2: Inlet flow + fault indicator */
    lcd_set_cursor(1, 0);
    memset(line, 0, sizeof(line));
    bool any_fault = sys->fault_tds_out_of_range ||
                     sys->fault_booster_low       ||
                     sys->fault_tank_a_limit       ||
                     sys->fault_ct1 || sys->fault_ct2;

    snprintf(line, sizeof(line), "F:%.1f %s",
             (double)sys->inlet_flow_lpm,
             any_fault ? "!FAULT" : "OK    ");
    lcd_print(line);
}

/**
 * @brief  Display a fault message on the HMI (scrolls on line 2).
 */
void DRV_HMI_ShowFault(const char *msg)
{
    lcd_set_cursor(1, 0);
    char line[17] = {0};
    snprintf(line, sizeof(line), "%-16s", msg);
    lcd_print(line);
}

void DRV_HMI_Clear(void)
{
    lcd_write_byte(0x01, LCD_CMD);
    HAL_Delay(2);
}

/* ─────────────────────────────────────────────────────────────────────────
   10. IONIZER RELAY CONTROL
───────────────────────────────────────────────────────────────────────── */

/**
 * @brief  Turn ON a specific ionizer set (relay control).
 *         Only one set should be ON at a time.
 * @param  set  IONIZER_SET1 or IONIZER_SET2
 */
void DRV_Ionizer_ON(IonizerSet_t set)
{
    if (set == IONIZER_SET1) {

        DRV_Relay1_Set(false);
        DRV_Relay2_Set(false);
        DBG_PRINT("Ionizer Set 1 ON");
    } else if (set == IONIZER_SET2) {

        DRV_Relay1_Set(true);
        DRV_Relay2_Set(true);
        DBG_PRINT("Ionizer Set 2 ON");
    }
}

/**
 * @brief  Turn OFF both ionizer sets (safety OFF).
 */
void DRV_Ionizer_OFF_All(void)
{
    DRV_Relay1_Set(false);
    DRV_Relay2_Set(false);
    DBG_PRINT("Ionizer ALL OFF");
}

/**
 * @brief  Manage automatic alternation between Set 1 and Set 2.
 *         Call this every main loop iteration.
 *         Uses HAL_GetTick() delta to track elapsed time.
 * @param  sys  Pointer to system data structure
 */
void DRV_Ionizer_Alternate(IonizerSystem_t *sys)
{
    static uint32_t last_tick = 0;
    uint32_t now   = HAL_GetTick();
    uint32_t delta = now - last_tick;
    last_tick      = now;

  //  if (!sys->ionizer_on) return;

    sys->alternate_timer_ms += delta;
           time_interval +=delta;
           //HAL_Delay(10000);
    if (sys->alternate_timer_ms >= IONIZER_ALTERNATE_MS) {
        sys->alternate_timer_ms = 0;
        time_interval=0;
        /* Switch set */
        if (sys->active_set == IONIZER_SET1) {
            sys->active_set = IONIZER_SET2;
        } else {
            sys->active_set = IONIZER_SET1;
        }
        DRV_Ionizer_ON(sys->active_set);
        DBG_PRINT("Ionizer alternated → Set %d", (int)sys->active_set);
    }
}

/* ─────────────────────────────────────────────────────────────────────────
   11. WATCHDOG
───────────────────────────────────────────────────────────────────────── */

/**
 * @brief  Kick (refresh) the Independent Watchdog.
 *         Call periodically to prevent MCU reset.
 */
/*
void DRV_WDT_Kick(IWDG_HandleTypeDef *hiwdg_handle)
{
    HAL_IWDG_Refresh(hiwdg_handle);
}
*/



/* ─────────────────────────────────────────────────────────────────────────
     TDS SENSOR
───────────────────────────────────────────────────────────────────────── */

/**
 * @brief  Read TDS sensor via ADC and convert raw value to ppm.
 *
 *   ADC1 Channel 9 (PA4), 12-bit resolution (0–4095)
 *
 *   Conversion:
 *     voltage_mv = (raw / 4095.0) * S2_TDS_VREF_MV
 *     tds_ppm    = (uint16_t)((voltage_mv / S2_TDS_VREF_MV) * S2_TDS_FULL_SCALE_PPM)
 *                + S2_TDS_OFFSET_PPM
 *
 *   Sensor fault:
 *     raw < TDS_ADC_MIN_VALID  → disconnected sensor
 *     raw > TDS_ADC_MAX_VALID  → shorted / saturated
 *
 * @param  sys  Pointer to Slave2System_t
 * @return S2_OK on success, S2_SENSOR_FAULT if ADC out of range
 */
DRV_Status DRV_TDS_Read(IonizerSystem_t *sys)
{
	  ADC_ChannelConfTypeDef cfg = {0};
	    cfg.Channel      = TDS_ADC_CHANNEL;
	    cfg.Rank         = 1;
	    cfg.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
	    HAL_ADC_ConfigChannel(sys->hadc, &cfg);

	    uint32_t sum = 0;
	    const uint32_t ADC_SAMPLES = 30;

	    for (uint32_t i = 0; i < ADC_SAMPLES; i++)
	    {
	        HAL_ADC_Start(sys->hadc);
	        if (HAL_ADC_PollForConversion(sys->hadc, 100) != HAL_OK)
	        {
	            HAL_ADC_Stop(sys->hadc);
	            sys->fault_tds = true;
	            printf("TDS ADC timeout\n");
	            return DRV_TIMEOUT;
	        }
	        sum += (uint16_t)HAL_ADC_GetValue(sys->hadc);
	        HAL_ADC_Stop(sys->hadc);
	        HAL_Delay(10);
	    }


	     adcAverage = (float)sum / ADC_SAMPLES;

	       voltage = adcAverage * VREF / ADC_RES;

	      // Temperature compensation
	      float compensationCoefficient =
	              1.0 + 0.02 * (temperature - 25.0);

	      float compensationVoltage =
	              voltage / compensationCoefficient;

	      // TDS conversion formula
	       tdsValue =
	          (133.42 * compensationVoltage * compensationVoltage * compensationVoltage
	          -255.86 * compensationVoltage * compensationVoltage
	          +857.39 * compensationVoltage) * 0.5;

	       sys->tds_ppm   = (uint16_t)tdsValue;


	    return DRV_OK;
}

 float readTDS(IonizerSystem_t *sys)
{
	        ADC_ChannelConfTypeDef cfg = {0};
		    cfg.Channel      = TDS_ADC_CHANNEL;
		    cfg.Rank         = 1;
		    cfg.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
		    HAL_ADC_ConfigChannel(sys->hadc, &cfg);

    const int samples = 30;
    uint32_t sum = 0;

    for(int i = 0; i < samples; i++)
    {
    	 HAL_ADC_Start(sys->hadc);
    		        if (HAL_ADC_PollForConversion(sys->hadc, 100) != HAL_OK)
    		        {
    		            HAL_ADC_Stop(sys->hadc);
    		            sys->fault_tds = true;
    		            printf("TDS ADC timeout\n");
    		            return DRV_TIMEOUT;
    		        }
    		        sum += (uint16_t)HAL_ADC_GetValue(sys->hadc);
    		        HAL_ADC_Stop(sys->hadc);
    		        HAL_Delay(1);
    }

    float adcAverage = (float)sum / samples;

    float voltage = adcAverage * VREF / ADC_RES;

    // Temperature compensation
    float compensationCoefficient =
            1.0 + 0.02 * (temperature - 25.0);

    float compensationVoltage =
            voltage / compensationCoefficient;

    // TDS conversion formula
     tdsValue =
        (133.42 * compensationVoltage * compensationVoltage * compensationVoltage
        -255.86 * compensationVoltage * compensationVoltage
        +857.39 * compensationVoltage) * 0.5;

    return tdsValue;
}




/* Get current microsecond tick */
static inline uint32_t DRV_Ultrasonic_GetUs(void)
{
    return __HAL_TIM_GET_COUNTER(&htim1);
}

/* Blocking microsecond delay */
static void DRV_Ultrasonic_DelayUs(uint32_t us)
{
    uint32_t start = DRV_Ultrasonic_GetUs();
    while ((DRV_Ultrasonic_GetUs() - start) < us);
}


void UART_Restart(void)
{
    HAL_UART_Abort(&huart1);

    HAL_UART_DeInit(&huart1);

    HAL_Delay(10);

    HAL_UART_Init(&huart1);


}
