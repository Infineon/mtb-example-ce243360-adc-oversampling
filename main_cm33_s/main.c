/*****************************************************************************
* File Name        : main.c
*
* Description      : Main CM33 secure core application demonstrating the PSOC
*                    Control C3 PPCA SAR ADC. It reads an on-board
*                    potentiometer and prints both the native 12-bit conversion
*                    and a 15-bit effective-resolution reading produced by the
*                    ADC's HARDWARE averaging filter (64x oversampling). A TCPWM
*                    timer, routed through the EPU trigger fabric to the ADC
*                    Start-Of-Conversion, drives the conversions that feed the
*                    filter. AREF, ADC (arbitrary trigger), averaging filter,
*                    TCPWM and EPU are all set up in the Device Configurator;
*                    the generated code is applied by cybsp_init() and used
*                    here.
*
* Related Document : See README.md
*
********************************************************************************
* (c) 2026, Infineon Technologies AG, or an affiliate of Infineon
* Technologies AG. All rights reserved.
* This software, associated documentation and materials ("Software") is
* owned by Infineon Technologies AG or one of its affiliates ("Infineon")
* and is protected by and subject to worldwide patent protection, worldwide
* copyright laws, and international treaty provisions. Therefore, you may use
* this Software only as provided in the license agreement accompanying the
* software package from which you obtained this Software. If no license
* agreement applies, then any use, reproduction, modification, translation, or
* compilation of this Software is prohibited without the express written
* permission of Infineon.
*
* Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
* IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
* INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF
* THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A
* SPECIFIC USE/PURPOSE OR MERCHANTABILITY.
* Infineon reserves the right to make changes to the Software without notice.
* You are responsible for properly designing, programming, and testing the
* functionality and safety of your intended application of the Software, as
* well as complying with any legal requirements related to its use. Infineon
* does not guarantee that the Software will be free from intrusion, data theft
* or loss, or other breaches ("Security Breaches"), and Infineon shall have
* no liability arising out of any Security Breaches. Unless otherwise
* explicitly approved by Infineon, the Software may not be used in any
* application where a failure of the Product or any consequences of the use
* thereof can reasonably be expected to result in personal injury.
*******************************************************************************/


/******************************************************************************
 * Header Files
 *****************************************************************************/

#include "cy_pdl.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include <stdio.h>

/*******************************************************************************
* Macros
*******************************************************************************/

/* The potentiometer is on AIN0P -> ADC Group 0, single-ended Channel 0 (the
 * primary POT on KIT_PSC3M8_EVK). Each ADC group supports up to 6 single-ended
 * channels if you want to add more analog inputs. */
#define POT_CHANNEL             (0U)

/* ---- Hardware oversampling (ADC averaging filter) -------------------------
 * The SAR ADC feeds a hardware digital averaging filter (ADC_FILT) that adds
 * resolution by accumulating OVERSAMPLE_SAMPLES conversions in hardware:
 * accumulating 4^K samples adds K bits, so 64 = 4^3 samples turn the native
 * 12-bit result into a 15-bit effective reading - with NO software loop.
 *
 * How the filter is fed: the averaging filter only accumulates samples that
 * arrive on the ADC Start-Of-Conversion (adc_soc) hardware signal. A TCPWM
 * timer generates a periodic trigger that the EPU routes to adc_soc, so the
 * ADC (in ARBITRARY trigger mode) converts the POT channel every PWM period
 * and every conversion flows into the filter. All of this is wired in the
 * Device Configurator (POT_ADC_TRIG_PWM -> POT_ADC_TRIG_PU -> POT_ADC_TRIG ->
 * adc_soc).
 *
 * Output scaling: the 12-bit ADC data is placed in bits [15:4] of the filter's
 * 16-bit input (<< 4), and the filter uses a SIGNED data path, so each sample
 * it accumulates is (raw12 << 4) - 32768. The filter output is the signed SUM
 * of n samples. We recover the unsigned 15-bit value (raw12 * 8) with
 *   15bit = (sum + n * 32768) / (2 * n)
 * which is exact for any accumulated sample count n. */
#define OVERSAMPLE_SAMPLES      (64U)
#define AVG_SAMPLE_MIDPOINT     (32768)    /* per-sample signed offset (2^15) */

#define ADC_12BIT_MAX           (4095U)     /* 2^12 - 1 */
#define ADC_15BIT_MAX           (32767U)    /* 2^15 - 1 */

/* Single-ended full-scale input range of the SAR ADC (approx. 3.6 V). */
#define ADC_FULLSCALE_MV        (3600U)

/* Bounded wait for the averaging filter to accumulate OVERSAMPLE_SAMPLES.
 * At the ~100 us PWM trigger period, 64 samples take ~6.4 ms; this bound is
 * generous so the demo never blocks if the hardware trigger is not running. */
#define AVG_SAMPLE_TIMEOUT      (2000000UL)

/* AREF bandgap settling time after enable. */
#define AREF_SETTLE_MS          (10U)

/* Main loop period (how often the live reading refreshes). */
#define LOOP_DELAY_MS           (50U)

/* Number of user LEDs used as the POT bar-graph "VU meter". */
#define NUM_LEDS                (6U)

/*******************************************************************************
* Global Variables
*******************************************************************************/

/* Debug UART variables */
static cy_stc_scb_uart_context_t    DEBUG_UART_context; /* DEBUG_UART context */
static mtb_hal_uart_t               DEBUG_UART_hal_obj; /* Debug UART HAL object */

/* The six user LEDs (LED1..LED6), used as a bar-graph VU meter of the POT
 * position. All are strong-drive outputs configured in the Device Configurator
 * and are active-low (drive 0 to light). */
static const struct
{
    GPIO_PRT_Type *port;
    uint32_t       pin;
} leds[NUM_LEDS] =
{
    { CYBSP_USER_LED1_PORT, CYBSP_USER_LED1_PIN },
    { CYBSP_USER_LED2_PORT, CYBSP_USER_LED2_PIN },
    { CYBSP_USER_LED3_PORT, CYBSP_USER_LED3_PIN },
    { CYBSP_USER_LED4_PORT, CYBSP_USER_LED4_PIN },
    { CYBSP_USER_LED5_PORT, CYBSP_USER_LED5_PIN },
    { CYBSP_USER_LED6_PORT, CYBSP_USER_LED6_PIN },
};

/*******************************************************************************
* Function Name: adc_read_12bit
********************************************************************************
* Summary:
*  Returns the latest native 12-bit result (0..4095) of the POT channel. The
*  ADC is triggered continuously by the TCPWM/EPU hardware path, so its data
*  register holds the most recent conversion - we simply read it.
*******************************************************************************/
static uint16_t adc_read_12bit(void)
{
    return Cy_PPCA_ADC_Read_ADC_Data(POT_ADC_HW, POT_CHANNEL);
}

/*******************************************************************************
* Function Name: counts_to_mv
********************************************************************************
* Summary:
*  Converts raw ADC counts to millivolts for a given full-scale count value.
*******************************************************************************/
static uint32_t counts_to_mv(uint32_t counts, uint32_t full_scale_counts)
{
    return (counts * ADC_FULLSCALE_MV) / full_scale_counts;
}

/*******************************************************************************
* Function Name: adc_read_15bit
********************************************************************************
* Summary:
*  Returns a 15-bit effective-resolution reading (0..32767) of the POT using
*  the ADC's HARDWARE averaging filter. It starts a fresh averaging window
*  (Start-Of-Filter), waits (bounded) until the filter has accumulated
*  OVERSAMPLE_SAMPLES conversions, then reads the signed accumulated sum and
*  the sample count and normalises to a 15-bit value. The filter uses a signed
*  data path (each sample = (raw12 << 4) - 32768), so the unsigned 15-bit value
*  is (sum + n * 32768) / (2 * n). No software sample loop is involved.
*******************************************************************************/
static uint16_t adc_read_15bit(void)
{
    uint32_t timeout = AVG_SAMPLE_TIMEOUT;

    /* Start a fresh averaging window: reset the hardware accumulator and begin
     * accumulating the next OVERSAMPLE_SAMPLES conversions. */
    Cy_PPCA_ADC_AVG_Filter_Trigger_Start(POT_ADC_FILTER_HW);

    /* Wait (bounded) until the hardware filter has accumulated all samples. */
    while ((Cy_PPCA_ADC_Filter_AVG_Get_Samples_Used(POT_ADC_FILTER_HW) < OVERSAMPLE_SAMPLES)
           && (timeout > 0UL))
    {
        timeout--;
    }

    /* Read the signed sum and the actual number of accumulated samples. */
    int32_t  sum = (int32_t)Cy_PPCA_ADC_Filter_AVG_Output(POT_ADC_FILTER_HW);
    uint32_t n   = Cy_PPCA_ADC_Filter_AVG_Get_Samples_Used(POT_ADC_FILTER_HW);
    if (n == 0U) { n = 1U; }

    /* Recover the unsigned 15-bit value from the signed accumulator.
     *
     * WHY: the filter runs a SIGNED data path. The 12-bit code is left-
     * justified into the 16-bit input (raw12 << 4) and the 2^15 midpoint is
     * subtracted, so every accumulated sample is (raw12 << 4) - 32768 and
     * 'sum' is the signed sum of n of them. To get an unsigned result we add
     * the midpoint back n times, then divide by 2*n. Dividing by 2 rescales
     * the 16-bit-justified average ((raw12 << 4) = raw12 * 16) down to a
     * 15-bit code (raw12 * 8), i.e. exactly avg(raw12) * 8 -> 12-bit becomes
     * 15-bit (+3 bits). This holds for any n, so a short/timed-out window
     * still yields the correct value.
     *
     * Worked example (n = 64, steady input raw12 = 2731):
     *   per-sample   = (2731 << 4) - 32768 = 43696 - 32768 = 10928
     *   sum          = 64 * 10928          = 699392
     *   unsigned_sum = 699392 + 64 * 32768 = 2796544
     *   val15        = 2796544 / (2 * 64)  = 21848   ( = 2731 * 8 )
     *   -> 21848 / 32767 * 3600 mV ~= 2400 mV, matching the 12-bit reading
     *      but with 3 extra bits of resolution and far less noise. */
    int32_t unsigned_sum = sum + (int32_t)(n * (uint32_t)AVG_SAMPLE_MIDPOINT);
    if (unsigned_sum < 0) { unsigned_sum = 0; }

    uint32_t val15 = (uint32_t)unsigned_sum / (2U * n);
    if (val15 > ADC_15BIT_MAX) { val15 = ADC_15BIT_MAX; }

    return (uint16_t)val15;
}

/*******************************************************************************
* Function Name: update_vu_leds
********************************************************************************
* Summary:
*  Lights the six user LEDs as a bar-graph "VU meter" proportional to the POT
*  position (LED1 first .. LED6 last). Driven from the raw 12-bit reading so it
*  works independently of the oversampling. Board LEDs are active-low (0 = on).
*******************************************************************************/
static void update_vu_leds(uint16_t raw12)
{
    /* Number of LEDs to light: 0..NUM_LEDS, rounded. */
    uint32_t lit = ((uint32_t)raw12 * NUM_LEDS + (ADC_12BIT_MAX / 2U)) / ADC_12BIT_MAX;
    if (lit > NUM_LEDS)
    {
        lit = NUM_LEDS;
    }

    for (uint32_t i = 0U; i < NUM_LEDS; i++)
    {
        Cy_GPIO_Write(leds[i].port, leds[i].pin, (i < lit) ? 0U : 1U);
    }
}

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  1. Initializes the board and UART (printf) output.
*  2. Enables the PPCA analog subsystem, then (re)applies and enables the
*     generated AREF / ADC / averaging-filter configuration.
*  3. Continuously refreshes the 12-bit and 15-bit readings and drives a
*     6-LED VU meter.
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals. */
    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize UART hardware for debug (printf) output. */
    Cy_SCB_UART_Init(DEBUG_UART_HW, &DEBUG_UART_config, &DEBUG_UART_context);
    Cy_SCB_UART_Enable(DEBUG_UART_HW);

    result = mtb_hal_uart_setup(&DEBUG_UART_hal_obj, &DEBUG_UART_hal_config,
                                &DEBUG_UART_context, NULL);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    result = cy_retarget_io_init(&DEBUG_UART_hal_obj);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    __enable_irq();

    /* Enable the PPCA subsystem that hosts the analog blocks.
     *
     * IMPORTANT: while the PPCA block is disabled its internal clocks are gated
     * and all its registers are held in reset. cybsp_init() applied the
     * generated analog configuration while the block was still disabled, so it
     * is lost the moment the block is enabled. We therefore RE-APPLY the
     * generated AREF / ADC / filter configuration here, after Cy_PPCA_Enable(),
     * using the same const config structures from the Device Configurator. */
    Cy_PPCA_CNFG_Init(PPCA_CNFG_HW, &PPCA_CNFG_config);
    Cy_PPCA_Enable(PPCA_CNFG_HW);

    /* Claim exclusive EPU access for this core before configuring the EPU. */
    Cy_PPCA_EPU_EnableExclusiveAccess(PPCA_EPU, true);

    /* Bring up the reference (with settling time), then the ADC. */
    Cy_PPCA_AREF_Init(POT_AREF_HW, &POT_AREF_config);
    Cy_PPCA_AREF_Enable(POT_AREF_HW);
    Cy_SysLib_Delay(AREF_SETTLE_MS);

    Cy_PPCA_ADC_Init(POT_ADC_HW, &POT_ADC_config);
    Cy_PPCA_ADC_Enable(POT_ADC_HW);

    /* Enable the hardware averaging filter (oversampling) on the POT channel. */
    Cy_PPCA_ADC_Filter_Init(POT_ADC_FILTER_HW, &POT_ADC_FILTER_config);

    /* TCPWM timer that periodically triggers an ADC conversion. */
    Cy_TCPWM_PWM_Init(POT_ADC_TRIG_PWM_HW, POT_ADC_TRIG_PWM_NUM, &POT_ADC_TRIG_PWM_config);
    Cy_TCPWM_PWM_Enable(POT_ADC_TRIG_PWM_HW, POT_ADC_TRIG_PWM_NUM);

    /* EPU: route the TCPWM trigger to the ADC Start-Of-Conversion input, so the
     * ADC converts the POT channel every PWM period and feeds the filter:
     *   POT_ADC_TRIG_PWM.tr_out0 -> POT_ADC_TRIG_PU -> POT_ADC_TRIG -> adc_soc[0] */
    Cy_PPCA_EPU_Enable(PPCA_EPU);
    Cy_PPCA_EPU_PU_T1_Configure(POT_ADC_TRIG_PU_HW, POT_ADC_TRIG_PU_INDEX,
                                &POT_ADC_TRIG_PU_put1_config);
    Cy_PPCA_EPU_PU_T1_Enable(POT_ADC_TRIG_PU_HW, POT_ADC_TRIG_PU_INDEX,
                             POT_ADC_TRIG_PU_ENABLE_MODE);
    Cy_PPCA_EPU_Combo_Configure(POT_ADC_TRIG_HW, POT_ADC_TRIG_INDEX,
                                &POT_ADC_TRIG_combo_config);

    /* Start the timer: the ADC now converts continuously and feeds the filter. */
    Cy_TCPWM_TriggerStart_Single(POT_ADC_TRIG_PWM_HW, POT_ADC_TRIG_PWM_NUM);

    /* Let the ADC/filter start producing data before the first read. */
    Cy_SysLib_Delay(10U);

    /* Banner. \x1b[2J\x1b[;H - ANSI ESC sequence to clear the screen. */
    printf("\x1b[2J\x1b[;H");
    printf("************************************************************\r\n");
    printf("PSOC Control C3M/P8: ADC 12-bit and 15-bit oversampling\r\n");
    printf("************************************************************\r\n\n");
    printf(" Reads the on-board potentiometer (AIN0P -> ADC0 ch0) and shows the\r\n");
    printf(" native 12-bit conversion together with a 15-bit effective-resolution\r\n");
    printf(" value produced by the ADC hardware averaging filter (%u-sample\r\n",
           OVERSAMPLE_SAMPLES);
    printf(" oversampling).\r\n");
    printf(" The six user LEDs form a bar-graph VU meter of the POT position.\r\n");
    printf("----------------------------------------------------------------------\r\n");

    printf("Live reading (refreshing):\r\n");

    for (;;)
    {
        uint16_t raw12 = adc_read_12bit();
        uint16_t val15 = adc_read_15bit();

        uint32_t mv12     = counts_to_mv(raw12, ADC_12BIT_MAX);
        uint32_t mv15_x10 = (((uint32_t)val15 * ADC_FULLSCALE_MV * 10U) + (ADC_15BIT_MAX / 2U))
                            / ADC_15BIT_MAX;

        /* \r returns to the start of the line, \x1b[K clears any leftovers. */
        printf("\r  12-bit %4u = %4lu mV   |   15-bit(x%u) %5u = %4lu.%lu mV\x1b[K",
               raw12, (unsigned long)mv12,
               OVERSAMPLE_SAMPLES, val15,
               (unsigned long)(mv15_x10 / 10U), (unsigned long)(mv15_x10 % 10U));

        /* LED bar-graph VU meter follows the POT position (from 12-bit). */
        update_vu_leds(raw12);

        Cy_SysLib_Delay(LOOP_DELAY_MS);
    }
}