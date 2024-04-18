/*
 * Copyright (c) Nordic Semiconductor ASA. All Rights Reserved.
 *
 * The information contained herein is confidential property of Nordic Semiconductor ASA.
 * The use, copying, transfer or disclosure of such information is prohibited except by
 * express written agreement with Nordic Semiconductor ASA.
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <nrf.h>
#include <hal/nrf_clock.h>
#include <hal/nrf_timer.h>
#include <hal/nrf_rtc.h>
#include <hal/nrf_gpiote.h>
#include <hal/nrf_egu.h>
#include <helpers/nrfx_gppi.h>
#include <assert.h>

// #define NRF_PLATFORM_LUMOS

#if !defined(NRF5340_XXAA_APPLICATION) && !defined(CONFIG_SOC_SERIES_BSIM_NRFXX)
#include <nrfx_temp.h>
#endif

#if defined(NRF_PLATFORM_LUMOS)
#define CLOCK NRF_CLOCK
#define RTC NRF_RTC30

#define CAL_TIMER NRF_TIMER21
#define REF_TIMER NRF_TIMER22
#define GPIOTE NRF_GPIOTE20
#define EGU NRF_EGU10

#define CLOCK_IRQ CLOCK_POWER_IRQn
#define EGU_IRQ EGU10_IRQn
#elif defined(NRF5340_XXAA_NETWORK)
#define CLOCK NRF_CLOCK_NS
#define RTC NRF_RTC1_NS

#define CAL_TIMER NRF_TIMER1_NS
#define REF_TIMER NRF_TIMER2_NS

#define GPIOTE NRF_GPIOTE_NS
#define EGU NRF_EGU0_NS

#define CLOCK_IRQ CLOCK_POWER_IRQn
#define EGU_IRQ EGU0_IRQn
#else
#define CLOCK NRF_CLOCK
#define RTC NRF_RTC1

#define CAL_TIMER NRF_TIMER1
#define REF_TIMER NRF_TIMER2

#define GPIOTE NRF_GPIOTE
#define EGU NRF_EGU0

#define CLOCK_IRQ POWER_CLOCK_IRQn
#define EGU_IRQ SWI0_EGU0_IRQn
#endif

#if defined(NRF_PLATFORM_LUMOS)
#define CAL_TIMER_IRQ TIMER21_IRQn
#define RTC_IRQ RTC30_IRQn
#else
#define CAL_TIMER_IRQ TIMER1_IRQn
#define RTC_IRQ RTC1_IRQn
#endif

#define RTC_EVENT_TOGGLE_PIN 3
#define RTC_RELOAD_VALUE 32768

#define RTC_EVT_INTERVAL_US 1000000

static uint8_t rtc_ppi_ch;
static uint8_t clock_ppi_ch;
static uint8_t timer_ppi_ch;
static uint8_t debug_ppi_ch;

typedef struct {
	struct k_work work;
	uint32_t count;
	uint32_t ref_time;
	uint32_t cal_events;
	int32_t temp;
	uint32_t drift;
} clk_accuracy_t;

static clk_accuracy_t clk_accuracy = { .count = 0,
				       .ref_time = 0,
				       .cal_events = 0,
				       .temp = 0,
				       .drift = 0 };

void clock_accuracy_handler(struct k_work *item)
{
	clk_accuracy_t *accuracy = CONTAINER_OF(item, clk_accuracy_t, work);
	printk("#%04d ref time:%-8d drift:%-4d cal_events:%-4d temperature:%d\n",
		    accuracy->count, accuracy->ref_time, accuracy->drift, accuracy->cal_events,
		    accuracy->temp);

	/* Verify the drift after first calibration event.*/
	if (accuracy->cal_events > 0) {
#if defined(NRF_PLATFORM_LUMOS)
		/* TODO DRGN-19212: Revisit this. The drift should not be this large. */
		__ASSERT_NO_MSG(accuracy->drift < 15000);
#else
		//__ASSERT_NO_MSG(accuracy->drift < 400);
#endif
	}
}

ISR_DIRECT_DECLARE(rtc_isr)
{
	nrf_rtc_event_clear(RTC, NRF_RTC_EVENT_COMPARE_0);
	uint32_t reload_value = RTC->CC[0] + RTC_RELOAD_VALUE;
	nrf_rtc_cc_set(RTC, 0, reload_value);

	uint32_t ref_time = nrf_timer_cc_get(REF_TIMER, 0);

	int32_t drift;
	if (ref_time > RTC_EVT_INTERVAL_US) {
		drift = ref_time - RTC_EVT_INTERVAL_US;
	} else {
		drift = RTC_EVT_INTERVAL_US - ref_time;
	}

	int32_t temp = 0;
#if !defined(NRF5340_XXAA_APPLICATION) && !defined(CONFIG_SOC_SERIES_BSIM_NRFXX)
	nrfx_temp_measure();
	/* Raw temperature is represented by 0.25[C] units, division by 4 is needed */
	temp = nrfx_temp_result_get() / 4;
#endif

	/* Ignore first reading to sync timings for timer and rtc.*/
	if (++clk_accuracy.count > 1) {
		clk_accuracy.ref_time = ref_time;
		clk_accuracy.drift = drift;
		clk_accuracy.temp = temp;
		k_work_submit(&clk_accuracy.work);
		//printk("ref time: %d, drift time: %d\n", ref_time, drift);
	}

	return 1;
}

ISR_DIRECT_DECLARE(egu_isr)
{
	if (EGU->EVENTS_TRIGGERED[0]) {
		nrf_egu_event_clear(EGU, NRF_EGU_EVENT_TRIGGERED0);
		clk_accuracy.cal_events++;
	}
	return 0;
}

void calibration_timer_init(uint32_t cal_interval_s)
{
	nrf_timer_task_trigger(CAL_TIMER, NRF_TIMER_TASK_STOP);
	nrf_timer_task_trigger(CAL_TIMER, NRF_TIMER_TASK_CLEAR);
	nrf_timer_mode_set(CAL_TIMER, NRF_TIMER_MODE_TIMER);
	nrf_timer_bit_width_set(CAL_TIMER, NRF_TIMER_BIT_WIDTH_32);
	nrf_timer_prescaler_set(CAL_TIMER, NRF_TIMER_PRESCALER_CALCULATE(
						   NRF_TIMER_BASE_FREQUENCY_GET(CAL_TIMER), 31250));
	nrf_timer_cc_set(CAL_TIMER, 0, cal_interval_s * 31250);
	nrf_timer_shorts_enable(CAL_TIMER, TIMER_SHORTS_COMPARE0_CLEAR_Msk);

	nrfx_err_t err;
	err = nrfx_gppi_channel_alloc(&timer_ppi_ch);
	__ASSERT_NO_MSG(err == NRFX_SUCCESS);

	nrfx_gppi_channel_endpoints_setup(
		timer_ppi_ch, nrf_timer_event_address_get(CAL_TIMER, NRF_TIMER_EVENT_COMPARE0),
		nrf_clock_task_address_get(CLOCK, NRF_CLOCK_TASK_CAL));
	nrfx_gppi_channels_enable(BIT(timer_ppi_ch));
}

void reference_timer_init(void)
{
	nrf_timer_task_trigger(REF_TIMER, NRF_TIMER_TASK_STOP);
	nrf_timer_task_trigger(REF_TIMER, NRF_TIMER_TASK_CLEAR);
	nrf_timer_mode_set(REF_TIMER, NRF_TIMER_MODE_TIMER);
	nrf_timer_bit_width_set(REF_TIMER, NRF_TIMER_BIT_WIDTH_32);
	nrf_timer_prescaler_set(
		REF_TIMER,
		NRF_TIMER_PRESCALER_CALCULATE(NRF_TIMER_BASE_FREQUENCY_GET(REF_TIMER), 1000000));
}

void clock_init(void)
{
	/* Initialize HFCLK to use in reference timer (REF_TIMER).*/
	nrf_clock_task_trigger(CLOCK, NRF_CLOCK_TASK_HFCLKSTART);
	while (!nrf_clock_hf_is_running(CLOCK, NRF_CLOCK_HFCLK_HIGH_ACCURACY)) {
		void arch_busy_wait(uint32_t);
		arch_busy_wait(10);
	}

	/* RTC uses LFCLK and its source is set as RC osc.*/
	//nrf_clock_task_trigger(CLOCK, NRF_CLOCK_TASK_LFCLKSTOP);
	//nrf_clock_lf_src_set(CLOCK, NRF_CLOCK_LFCLK_RC);
	//nrf_clock_lf_src_set(CLOCK, NRF_CLOCK_LFCLK_XTAL);
	//nrf_clock_event_clear(CLOCK, NRF_CLOCK_EVENT_LFCLKSTARTED);
	//nrf_clock_task_trigger(CLOCK, NRF_CLOCK_TASK_LFCLKSTART);
	//while (!nrf_clock_lf_is_running(CLOCK)) {
	//	void arch_busy_wait(uint32_t);
	//	arch_busy_wait(10);
	//}

	nrfx_err_t err;
	err = nrfx_gppi_channel_alloc(&clock_ppi_ch);
	__ASSERT_NO_MSG(err == NRFX_SUCCESS);

	nrfx_gppi_channel_endpoints_setup(clock_ppi_ch,
					  nrf_clock_event_address_get(CLOCK, NRF_CLOCK_EVENT_DONE),
					  nrf_egu_task_address_get(EGU, NRF_EGU_TASK_TRIGGER0));
	nrfx_gppi_channels_enable(BIT(clock_ppi_ch));

	nrf_egu_int_enable(EGU, 1 << 0);
	IRQ_DIRECT_CONNECT(EGU_IRQ, 0, egu_isr, 0);
	irq_enable(EGU_IRQ);
}

void rtc_init(void)
{
	nrf_rtc_task_trigger(RTC, NRF_RTC_TASK_STOP);
	nrf_rtc_task_trigger(RTC, NRF_RTC_TASK_CLEAR);
	nrf_rtc_prescaler_set(RTC, 0);
	nrf_rtc_event_enable(RTC, RTC_EVTENSET_COMPARE0_Msk);
	nrf_rtc_int_enable(RTC, RTC_INTENSET_COMPARE0_Msk);
	nrf_rtc_cc_set(RTC, 0, RTC_RELOAD_VALUE);

	/* When there is RTC compare event (in every sec), capture ref timer to compare in ISR.*/

	nrfx_err_t err;
	err = nrfx_gppi_channel_alloc(&rtc_ppi_ch);
	__ASSERT_NO_MSG(err == NRFX_SUCCESS);

	nrfx_gppi_channel_endpoints_setup(
		rtc_ppi_ch, nrf_rtc_event_address_get(RTC, NRF_RTC_EVENT_COMPARE_0),
		nrf_timer_task_address_get(REF_TIMER, NRF_TIMER_TASK_CAPTURE0));

	nrfx_gppi_fork_endpoint_setup(rtc_ppi_ch,
				      nrf_timer_task_address_get(REF_TIMER, NRF_TIMER_TASK_CLEAR));
	nrfx_gppi_channels_enable(BIT(rtc_ppi_ch));

	IRQ_DIRECT_CONNECT(RTC_IRQ, 0, rtc_isr, 0);
	irq_enable(RTC_IRQ);
}

void pin_debug_init(void)
{
	nrf_gpiote_task_configure(GPIOTE, 0, RTC_EVENT_TOGGLE_PIN, NRF_GPIOTE_POLARITY_TOGGLE,
				  NRF_GPIOTE_INITIAL_VALUE_LOW);

	nrfx_err_t err;
	err = nrfx_gppi_channel_alloc(&debug_ppi_ch);
	__ASSERT_NO_MSG(err == NRFX_SUCCESS);

#if defined(DPPIC_PRESENT)
	/* RTC compare event is already published to rtc_ppi_ch.*/
	(void)debug_ppi_ch;
	nrf_gpiote_subscribe_set(GPIOTE, NRF_GPIOTE_TASK_OUT_0, rtc_ppi_ch);
#else
	nrfx_gppi_channel_endpoints_setup(
		debug_ppi_ch, nrf_rtc_event_address_get(RTC, NRF_RTC_EVENT_COMPARE_0),
		nrf_gpiote_task_address_get(GPIOTE, NRF_GPIOTE_TASK_OUT_0));
	nrfx_gppi_channels_enable(BIT(debug_ppi_ch));
#endif
}

static void check_result(uint32_t expected_cal_events)
{
	nrfx_gppi_channels_disable_all();

	irq_disable(RTC_IRQ);
	irq_disable(EGU_IRQ);

	printk("clk_accuracy cal_events %d\n\n", clk_accuracy.cal_events);
	__ASSERT_NO_MSG(clk_accuracy.cal_events >= expected_cal_events);
}

void test_lfclk_accuracy(uint32_t test_duration_s, uint32_t cal_interval_s)
{
	/* Initialize HFCLK and LFCLK.*/
	clock_init();

	/* Initialize RTC to create compare event on each seconds.*/
	rtc_init();

	/* Initialize a timer (CAL_TIMER) to schedule calibrations.*/
	calibration_timer_init(cal_interval_s);
	/* Initialize a reference timer(REF_TIMER) to compare with RTC timing.*/
	reference_timer_init();
	/* Initialize debug pin toggle to check timings for RTC compare events.*/
	//pin_debug_init();

/* Initialize temperature sensor to read in RTC events.*/
#if !defined(NRF5340_XXAA_APPLICATION) && !defined(CONFIG_SOC_SERIES_BSIM_NRFXX)
	nrfx_temp_config_t temp_config = { .interrupt_priority = 0 };
	nrfx_temp_init(&temp_config, NULL);
#else
	printk("Temperature sensor is not available for current board.");
#endif
	/* Handle accuracy data in system work queue thread to not block ISR.*/
	k_work_init(&clk_accuracy.work, clock_accuracy_handler);

	/* Start RTC and timers.*/
	nrf_rtc_task_trigger(RTC, NRF_RTC_TASK_START);
	nrf_timer_task_trigger(REF_TIMER, NRF_TIMER_TASK_START);
	nrf_timer_task_trigger(CAL_TIMER, NRF_TIMER_TASK_START);
	void arch_busy_wait(uint32_t);
	arch_busy_wait(test_duration_s * 1000 * 1000);
	//check_result(test_duration_s / cal_interval_s);
}

int main(void)
{
	/*
	duration_s = 60
        self.dut.tca.communication_timeout = duration_s
        self.dut.tca.function_execution_timeout = duration_s
        calibration_interval_s = 8
	*/
	test_lfclk_accuracy(60, 8);

	printk("main DONE\n");
	return 0;
}