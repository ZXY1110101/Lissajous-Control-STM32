#ifndef __LISSAJOUS_PLL_H
#define __LISSAJOUS_PLL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ad9833.h"

#define LISSAJOUS_PLL_TIMER_CLOCK_HZ       84000000UL
#define LISSAJOUS_PLL_CAPTURE_WINDOW_MS    100UL
#define LISSAJOUS_PLL_CAPTURE_EDGE_DIVISOR 8U
#define LISSAJOUS_PLL_DEFAULT_MULTIPLIER    2U
#define LISSAJOUS_PLL_REF_SAMPLES          2049U
#define LISSAJOUS_PLL_FB_MAX_SAMPLES       4097U
#define LISSAJOUS_PLL_RAW_SAMPLES          64U

typedef enum
{
  LISSAJOUS_PLL_MODE_FREQUENCY_FOLLOW = 0U, /* 诊断：测得多少就命令多少 */
  LISSAJOUS_PLL_MODE_QUADRATURE = 1U,       /* 第二问：同频、相差 90 度 */
  LISSAJOUS_PLL_MODE_DOUBLE     = 2U  /* 第三问：输出为输入的 2 倍频 */
} LissajousPLL_Mode;

typedef struct
{
  volatile uint32_t input_frequency_hz;
  volatile uint32_t input_frequency_millihz;
  volatile uint32_t target_frequency_millihz;
  volatile uint32_t command_frequency_hz;
  volatile uint32_t output_frequency_hz;
  volatile uint32_t output_frequency_millihz;
  volatile uint32_t estimated_mclk_hz;
  volatile uint32_t apb1_clock_hz;
  volatile uint32_t timer_counter_hz;
  volatile uint32_t timer_prescaler;
  volatile uint32_t input_measurement_periods;
  volatile uint32_t feedback_measurement_periods;
  volatile uint32_t input_captured_edges;
  volatile uint32_t capture_window_ms;
  volatile uint32_t capture_elapsed_ms;
  volatile uint32_t capture_edge_divisor;
  volatile uint32_t input_span_ticks;
  volatile uint32_t output_span_ticks;
  volatile uint32_t input_period_ticks;
  volatile uint32_t output_period_ticks;
  volatile int32_t  frequency_error_ticks;
  volatile int32_t  frequency_error_millihz;
  volatile int32_t  nominal_frequency_error_millihz;
  volatile int32_t  filtered_nominal_frequency_error_millihz;
  volatile int32_t  integrator_frequency_error_millihz;
  volatile uint32_t frequency_spike_rejected;
  volatile uint32_t frequency_spike_rejections;
  volatile int32_t  frequency_error_delta_millihz;
  volatile uint32_t control_target_frequency_millihz;
  volatile int32_t  phase_frequency_offset_millihz;
  volatile uint32_t phase_loop_enabled;
  volatile uint32_t phase_loop_active;
  volatile uint32_t phase_escape_active;
  volatile int32_t  phase_escape_sign;
  volatile uint32_t phase_escape_entry_count;
  volatile uint32_t phase_escape_exit_count;
  volatile int32_t  phase_normal_limit_millihz;
  volatile int32_t  phase_escape_offset_limit_millihz;
  volatile int32_t  phase_error_ticks;
  volatile int32_t  phase_error_millidegrees;
  volatile uint32_t phase_measurement_valid;
  volatile uint32_t phase_fit_used;
  volatile uint32_t phase_fit_points;
  volatile uint32_t measured_phase_ticks;
  volatile int32_t  measured_phase_millidegrees;
  volatile uint32_t target_phase_millidegrees;
  volatile int32_t  phase_tracking_error_millidegrees;
  volatile uint32_t frequency_word;
  volatile uint16_t phase_word;
  volatile uint8_t  multiplier;
  volatile uint8_t  locked;
  volatile uint8_t  good_batches;
  volatile uint8_t  capture_running;
  volatile uint8_t  data_valid;
  volatile uint32_t completed_batches;
  volatile uint32_t last_batch_tick_ms;
  volatile uint32_t capture_timeouts;
  volatile uint32_t rejected_batches;
  volatile uint32_t dma_start_errors;
  volatile uint32_t dds_errors;
  volatile uint32_t dither_test_enabled;
  volatile uint32_t dither_test_mode;
  volatile uint32_t dither_capture_guard_us;
  volatile uint32_t dither_test_delta_ftw;
  volatile uint32_t feedback_captured_edges;
  volatile uint32_t dither_switches;
  volatile uint32_t dither_control_writes;
  volatile int32_t  feedback_excess_edges;
  volatile uint32_t raw_output_frequency_millihz;
  volatile uint32_t dither_blank_windows;
  volatile uint32_t feedback_kept_edges;
  volatile uint32_t feedback_dropped_edges;
  volatile uint32_t feedback_reconstructed_cycles;
  volatile uint32_t feedback_blanking_threshold_ticks;
} LissajousPLL_Status;

/* 可直接加入 Keil Watch/Live Watch 观察锁定过程。 */
extern volatile LissajousPLL_Status g_lissajous_pll_status;
/* Runtime frequency multiplier: 1=circle, 2=1:2 Lissajous figure-eight. */
extern volatile uint8_t M;

HAL_StatusTypeDef LissajousPLL_Init(TIM_HandleTypeDef *htim,
                                    SPI_HandleTypeDef *hspi,
                                    LissajousPLL_Mode mode);
void LissajousPLL_Task(void);
void LissajousPLL_SetMode(LissajousPLL_Mode mode);
LissajousPLL_Mode LissajousPLL_GetMode(void);
HAL_StatusTypeDef LissajousPLL_SetMultiplier(uint8_t multiplier);
uint8_t LissajousPLL_GetMultiplier(void);
const volatile LissajousPLL_Status *LissajousPLL_GetStatus(void);
uint8_t LissajousPLL_RawSnapshotReady(void);
const uint32_t *LissajousPLL_GetRawReference(uint32_t *count);
const uint32_t *LissajousPLL_GetRawFeedback(uint32_t *count);

#ifdef __cplusplus
}
#endif

#endif /* __LISSAJOUS_PLL_H */
