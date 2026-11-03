#include "lissajous_pll.h"

#define PLL_DDS_MCLK_HZ              AD9833_DEFAULT_MCLK_HZ
#define PLL_FIXED_FREQUENCY_TEST     0U
#define PLL_FIXED_TEST_FREQUENCY_HZ  10000UL
#define PLL_START_FREQUENCY_HZ       PLL_FIXED_TEST_FREQUENCY_HZ
#define PLL_FTW_PER_HZ                10.73741824f

/* Step-1 false-edge isolation: A=0, B=1, C=2. */
#define PLL_DITHER_EDGE_TEST           0U
#define PLL_DITHER_EDGE_TEST_MODE_A    0U
#define PLL_DITHER_EDGE_TEST_MODE_B    1U
#define PLL_DITHER_EDGE_TEST_MODE_C    2U
#define PLL_DITHER_EDGE_TEST_MODE      PLL_DITHER_EDGE_TEST_MODE_C
#define PLL_DITHER_EDGE_TEST_FTW       10737UL
#define PLL_DITHER_EDGE_TEST_DELTA_FTW ((PLL_DITHER_EDGE_TEST_MODE == PLL_DITHER_EDGE_TEST_MODE_B) ? 1U : 0U)
#define PLL_FOLLOW_MIN_SAFE_HZ        800.0f
#define PLL_FOLLOW_MAX_SAFE_HZ        220000.0f
#define PLL_FOLLOW_MIN_SAFE_FTW       (PLL_FOLLOW_MIN_SAFE_HZ * PLL_FTW_PER_HZ)
#define PLL_FOLLOW_MAX_SAFE_FTW       (PLL_FOLLOW_MAX_SAFE_HZ * PLL_FTW_PER_HZ)
#define PLL_FOLLOW_INPUT_CHANGE_MHZ   5000UL
#if PLL_DITHER_EDGE_TEST
#define PLL_FREQUENCY_I_GAIN          0.0f
#else
#define PLL_FREQUENCY_I_GAIN          0.08f
#endif
#define PLL_FREQUENCY_I_REFERENCE_MS  100.0f
#define PLL_FREQUENCY_MAX_STEP_FTW    3.0f
#define PLL_FOLLOW_REACQUIRE_ERROR_HZ 5.0f
#define PLL_FREQUENCY_DEADBAND_HZ     0.02f
#define PLL_ERROR_SPIKE_REJECT_MILLIHZ 100L
#define PLL_ERROR_SPIKE_CONFIRM_MILLIHZ 50L
#define PLL_ERROR_SPIKE_CONFIRM_BATCHES 2U
#define PLL_NOMINAL_ERROR_AVG_SAMPLES 8U
#define PLL_LOCK_THRESHOLD_HZ         0.05f
#define PLL_FTW_MAX_SAFE_FLOAT        268435440.0f
#define PLL_DITHER_INTERVAL_MS       10UL
#define PLL_FAST_DITHER_ENABLE       1U
#define PLL_DITHER_CAPTURE_GUARD_US  50UL
#define PLL_DITHER_BLANK_MAX_WINDOWS 160U
#define PLL_SOFTWARE_EDGE_BLANK_ENABLE 1U
#define PLL_SOFTWARE_EDGE_MIN_KEEP    4U
#define PLL_SOFTWARE_EDGE_TOL_DIVISOR 3U


#define PLL_CAPTURE_TIMEOUT_MS       250UL
#define PLL_RESTART_DELAY_MS         2UL
#define PLL_INPUT_PERIOD_MIN_TICKS   700UL
#define PLL_INPUT_PERIOD_MAX_TICKS   168000UL
#define PLL_OUTPUT_PERIOD_MIN_TICKS  350UL
#define PLL_OUTPUT_PERIOD_MAX_TICKS  168000UL
#define PLL_REACQUIRE_DIVISOR        100UL
#define PLL_MAX_TRIM_DIVISOR         100UL
#define PLL_FREQ_GAIN_DIVISOR        2L
#define PLL_MAX_FREQ_STEP_DIVISOR    2000UL
#define PLL_PHASE_DEADBAND_TICKS     2L
#define PLL_PHASE_TARGET_CIRCLE_MDEG  90000L
#define PLL_PHASE_TARGET_FIGURE8_MDEG     0L
#define PLL_PHASE_LOOP_ENABLE          1U
#define PLL_PHASE_P_MILLIHZ_PER_DEG    1L
#define PLL_PHASE_SCHEDULE_BASE_MILLIHZ 2000000UL
#define PLL_PHASE_NORMAL_LIMIT_MIN_MILLIHZ 200L
#define PLL_PHASE_NORMAL_LIMIT_MAX_MILLIHZ 2000L
#define PLL_PHASE_ESCAPE_OFFSET_MIN_MILLIHZ 500L
#define PLL_PHASE_ESCAPE_OFFSET_MAX_MILLIHZ 4000L
#define PLL_PHASE_ENTRY_ERROR_MILLIHZ  500L
#define PLL_PHASE_DEADBAND_MDEG        2000L
#define PLL_PHASE_ESCAPE_ENTRY_MDEG  150000L
#define PLL_PHASE_ESCAPE_EXIT_MDEG   110000L
#define PLL_PHASE_ESCAPE_ENTRY_CONFIRM_BATCHES 2U
#define PLL_PHASE_ESCAPE_EXIT_CONFIRM_BATCHES  3U
#define PLL_LOCK_PHASE_THRESHOLD_MDEG 3000L
#define PLL_MAX_PHASE_STEP_WORDS     128L
#define PLL_LOCK_BATCHES             5U
typedef struct
{
  TIM_HandleTypeDef *htim;
  AD9833_HandleTypeDef dds;
  LissajousPLL_Mode mode;
  uint32_t ref_capture[LISSAJOUS_PLL_REF_SAMPLES];
  uint32_t fb_capture[LISSAJOUS_PLL_FB_MAX_SAMPLES];
  uint32_t raw_ref_snapshot[LISSAJOUS_PLL_RAW_SAMPLES];
  uint32_t raw_fb_snapshot[LISSAJOUS_PLL_RAW_SAMPLES];
  volatile uint8_t ref_ready;
  volatile uint8_t fb_ready;
  volatile uint8_t raw_snapshot_ready;
  uint32_t ref_capture_count;
  uint32_t fb_capture_count;
  uint32_t raw_ref_snapshot_count;
  uint32_t raw_fb_snapshot_count;
  uint32_t capture_elapsed_ms;
  uint8_t initialized;
  uint8_t follow_settling;
  uint8_t follow_reacquire_armed;
  uint8_t nominal_valid;
  uint8_t phase_escape_active;
  int8_t phase_escape_sign;
  uint8_t phase_escape_entry_count;
  uint8_t phase_escape_exit_count;
  uint8_t error_filter_valid;
  uint8_t error_spike_candidate_valid;
  uint8_t error_spike_candidate_count;
  int32_t accepted_nominal_error_millihz;
  int32_t error_spike_candidate_millihz;
  int32_t nominal_error_history[PLL_NOMINAL_ERROR_AVG_SAMPLES];
  int64_t nominal_error_sum_millihz;
  uint8_t nominal_error_history_count;
  uint8_t nominal_error_history_index;
  uint32_t nominal_frequency_word;
  float ftw_accumulator;
  float frequency_error_hz;
  float dither_accumulator;
  uint8_t dither_pair_valid;
  uint32_t dither_floor_word;
  uint32_t follow_reacquire_input_millihz;
  uint32_t last_input_period;
  uint32_t capture_started_ms;
  uint32_t next_capture_ms;
  uint32_t next_dither_ms;
  volatile uint32_t dither_switches_total;
  uint32_t capture_start_dither_switches;
  volatile uint32_t fb_end_dither_switches;
  volatile uint32_t dither_control_writes_total;
  uint32_t capture_start_dither_control_writes;
  volatile uint32_t fb_end_dither_control_writes;
  uint32_t dither_blank_start[PLL_DITHER_BLANK_MAX_WINDOWS];
  uint32_t dither_blank_end[PLL_DITHER_BLANK_MAX_WINDOWS];
  volatile uint16_t dither_blank_window_count;
} LissajousPLL_Context;

static LissajousPLL_Context pll;
volatile LissajousPLL_Status g_lissajous_pll_status;
volatile uint8_t M = LISSAJOUS_PLL_DEFAULT_MULTIPLIER;

static uint32_t Timer5CounterClockHz(TIM_HandleTypeDef *htim)
{
  uint32_t peripheral_clock;
  uint32_t timer_kernel_clock;
  uint32_t prescaler;

  if ((htim == NULL) || (htim->Instance != TIM5))
  {
    return 0U;
  }

  peripheral_clock = HAL_RCC_GetPCLK1Freq();
  timer_kernel_clock = peripheral_clock;
  if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0U)
  {
    timer_kernel_clock *= 2U;
  }
  prescaler = htim->Instance->PSC + 1U;
  return timer_kernel_clock / prescaler;
}

static uint32_t AbsoluteDifferenceU32(uint32_t a, uint32_t b)
{
  return (a >= b) ? (a - b) : (b - a);
}

static uint32_t AbsoluteSigned32(int32_t value)
{
  if (value >= 0)
  {
    return (uint32_t)value;
  }
  return (uint32_t)(-(int64_t)value);
}

static int32_t ClampSigned32(int32_t value, int32_t minimum, int32_t maximum)
{
  if (value < minimum)
  {
    return minimum;
  }
  if (value > maximum)
  {
    return maximum;
  }
  return value;
}

static uint32_t ClampFrequencyWord(int64_t value)
{
  if (value < 1)
  {
    return 1U;
  }
  if (value > (int64_t)AD9833_FTW_MASK)
  {
    return AD9833_FTW_MASK;
  }
  return (uint32_t)value;
}

static uint32_t TargetFrequencyMilliHz(uint32_t input_frequency_millihz)
{
  uint32_t multiplier;
  uint64_t target;

  multiplier = g_lissajous_pll_status.multiplier;
  if ((multiplier != 1U) && (multiplier != 2U))
  {
    multiplier = 1U;
  }

  target = (uint64_t)input_frequency_millihz *
           (uint64_t)multiplier;
  if (target > 0xFFFFFFFFULL)
  {
    return 0xFFFFFFFFUL;
  }
  return (uint32_t)target;
}

static uint16_t WrapPhaseWord(int32_t value)
{
  while (value < 0)
  {
    value += (int32_t)AD9833_PHASE_MODULUS;
  }
  while (value >= (int32_t)AD9833_PHASE_MODULUS)
  {
    value -= (int32_t)AD9833_PHASE_MODULUS;
  }
  return (uint16_t)value;
}

static int32_t WrapSignedPeriod(int32_t value, uint32_t period)
{
  int32_t half_period;

  if (period == 0U)
  {
    return 0;
  }

  half_period = (int32_t)(period / 2U);
  while (value > half_period)
  {
    value -= (int32_t)period;
  }
  while (value < -half_period)
  {
    value += (int32_t)period;
  }
  return value;
}

static int32_t WrapSignedMilliDegrees(int32_t value)
{
  while (value > 180000L)
  {
    value -= 360000L;
  }
  while (value < -180000L)
  {
    value += 360000L;
  }
  return value;
}

static int32_t PhaseTargetForMultiplier(uint8_t multiplier)
{
  return (multiplier == 2U) ?
         PLL_PHASE_TARGET_FIGURE8_MDEG :
         PLL_PHASE_TARGET_CIRCLE_MDEG;
}

static void StoreMeasuredPhase(int32_t phase_millidegrees,
                               uint32_t phase_ticks,
                               uint32_t fit_used,
                               uint32_t fit_points)
{
  int32_t phase_error;
  int32_t target_phase_millidegrees;

  while (phase_millidegrees < 0)
  {
    phase_millidegrees += 360000L;
  }
  while (phase_millidegrees >= 360000L)
  {
    phase_millidegrees -= 360000L;
  }
  target_phase_millidegrees =
      PhaseTargetForMultiplier(
          g_lissajous_pll_status.multiplier);
  phase_error = WrapSignedMilliDegrees(
      target_phase_millidegrees - phase_millidegrees);

  g_lissajous_pll_status.measured_phase_ticks = phase_ticks;
  g_lissajous_pll_status.measured_phase_millidegrees =
      phase_millidegrees;
  g_lissajous_pll_status.target_phase_millidegrees =
      (uint32_t)target_phase_millidegrees;
  g_lissajous_pll_status.phase_tracking_error_millidegrees =
      phase_error;
  g_lissajous_pll_status.phase_fit_used = fit_used;
  g_lissajous_pll_status.phase_fit_points = fit_points;
  g_lissajous_pll_status.phase_measurement_valid = 1U;
}

static void UpdateMeasuredPhase(uint32_t reference_timestamp,
                                uint32_t feedback_timestamp,
                                uint32_t feedback_period)
{
  int32_t signed_delta;
  int32_t phase_ticks;
  int32_t phase_millidegrees;

  g_lissajous_pll_status.phase_measurement_valid = 0U;
  if ((feedback_period == 0U) ||
      (feedback_period > 0x7FFFFFFFUL))
  {
    return;
  }

  /* Unsigned subtraction followed by int32 conversion preserves a TIM5
   * wrap as long as the two selected edges are less than 2^31 ticks apart. */
  signed_delta = (int32_t)(feedback_timestamp - reference_timestamp);
  phase_ticks = signed_delta % (int32_t)feedback_period;
  if (phase_ticks < 0)
  {
    phase_ticks += (int32_t)feedback_period;
  }

  phase_millidegrees =
      (int32_t)(((int64_t)phase_ticks * 360000LL +
                 (int64_t)feedback_period / 2LL) /
                (int64_t)feedback_period);
  StoreMeasuredPhase(phase_millidegrees,
                     (uint32_t)phase_ticks,
                     0U,
                     1U);
}

static void ResetPhaseEscapeState(void)
{
  pll.phase_escape_active = 0U;
  pll.phase_escape_sign = 0;
  pll.phase_escape_entry_count = 0U;
  pll.phase_escape_exit_count = 0U;
  g_lissajous_pll_status.phase_escape_active = 0U;
  g_lissajous_pll_status.phase_escape_sign = 0;
  g_lissajous_pll_status.phase_escape_entry_count = 0U;
  g_lissajous_pll_status.phase_escape_exit_count = 0U;
}
static void ResetFrequencyErrorFilterState(void)
{
  uint32_t index;

  pll.error_filter_valid = 0U;
  pll.error_spike_candidate_valid = 0U;
  pll.error_spike_candidate_count = 0U;
  pll.accepted_nominal_error_millihz = 0;
  pll.error_spike_candidate_millihz = 0;
  pll.nominal_error_sum_millihz = 0LL;
  pll.nominal_error_history_count = 0U;
  pll.nominal_error_history_index = 0U;
  for (index = 0U; index < PLL_NOMINAL_ERROR_AVG_SAMPLES; ++index)
  {
    pll.nominal_error_history[index] = 0;
  }
  g_lissajous_pll_status.frequency_spike_rejected = 0U;
  g_lissajous_pll_status.frequency_error_delta_millihz = 0;
  g_lissajous_pll_status.filtered_nominal_frequency_error_millihz = 0;
  g_lissajous_pll_status.integrator_frequency_error_millihz = 0;
}

static uint8_t FrequencyErrorSampleAccepted(
    int32_t nominal_error_millihz)
{
  int32_t error_delta_millihz;
  int32_t candidate_delta_millihz;

  g_lissajous_pll_status.frequency_spike_rejected = 0U;
  g_lissajous_pll_status.frequency_error_delta_millihz = 0;

  if (pll.error_filter_valid == 0U)
  {
    pll.error_filter_valid = 1U;
    pll.accepted_nominal_error_millihz = nominal_error_millihz;
    pll.error_spike_candidate_valid = 0U;
    pll.error_spike_candidate_count = 0U;
    return 1U;
  }

  error_delta_millihz =
      nominal_error_millihz -
      pll.accepted_nominal_error_millihz;
  g_lissajous_pll_status.frequency_error_delta_millihz =
      error_delta_millihz;

  if (AbsoluteSigned32(error_delta_millihz) <=
      PLL_ERROR_SPIKE_REJECT_MILLIHZ)
  {
    pll.accepted_nominal_error_millihz = nominal_error_millihz;
    pll.error_spike_candidate_valid = 0U;
    pll.error_spike_candidate_count = 0U;
    return 1U;
  }

  if (pll.error_spike_candidate_valid != 0U)
  {
    candidate_delta_millihz =
        nominal_error_millihz -
        pll.error_spike_candidate_millihz;
    if (AbsoluteSigned32(candidate_delta_millihz) <=
        PLL_ERROR_SPIKE_CONFIRM_MILLIHZ)
    {
      if (pll.error_spike_candidate_count < 0xFFU)
      {
        ++pll.error_spike_candidate_count;
      }
      if (pll.error_spike_candidate_count >=
          PLL_ERROR_SPIKE_CONFIRM_BATCHES)
      {
        pll.accepted_nominal_error_millihz =
            nominal_error_millihz;
        pll.error_spike_candidate_valid = 0U;
        pll.error_spike_candidate_count = 0U;
        return 1U;
      }
    }
    else
    {
      pll.error_spike_candidate_millihz =
          nominal_error_millihz;
      pll.error_spike_candidate_count = 1U;
    }
  }
  else
  {
    pll.error_spike_candidate_valid = 1U;
    pll.error_spike_candidate_millihz =
        nominal_error_millihz;
    pll.error_spike_candidate_count = 1U;
  }

  g_lissajous_pll_status.frequency_spike_rejected = 1U;
  ++g_lissajous_pll_status.frequency_spike_rejections;
  return 0U;
}

static int32_t FilteredNominalFrequencyErrorMilliHz(
    int32_t nominal_error_millihz,
    uint8_t sample_accepted)
{
  uint32_t count;
  int64_t average;

  if (sample_accepted != 0U)
  {
    if (pll.nominal_error_history_count <
        PLL_NOMINAL_ERROR_AVG_SAMPLES)
    {
      pll.nominal_error_history[
          pll.nominal_error_history_index] =
          nominal_error_millihz;
      pll.nominal_error_sum_millihz +=
          (int64_t)nominal_error_millihz;
      ++pll.nominal_error_history_count;
    }
    else
    {
      pll.nominal_error_sum_millihz -=
          (int64_t)pll.nominal_error_history[
              pll.nominal_error_history_index];
      pll.nominal_error_history[
          pll.nominal_error_history_index] =
          nominal_error_millihz;
      pll.nominal_error_sum_millihz +=
          (int64_t)nominal_error_millihz;
    }

    ++pll.nominal_error_history_index;
    if (pll.nominal_error_history_index >=
        PLL_NOMINAL_ERROR_AVG_SAMPLES)
    {
      pll.nominal_error_history_index = 0U;
    }
  }

  count = pll.nominal_error_history_count;
  if (count == 0U)
  {
    return nominal_error_millihz;
  }

  average = pll.nominal_error_sum_millihz;
  if (average >= 0LL)
  {
    average = (average + (int64_t)count / 2LL) /
              (int64_t)count;
  }
  else
  {
    average = (average - (int64_t)count / 2LL) /
              (int64_t)count;
  }
  return (int32_t)average;
}

static int32_t ScheduledPhasePermissionMilliHz(int32_t minimum_millihz,
                                                int32_t maximum_millihz)
{
  uint64_t scaled;
  uint32_t target_millihz;

  target_millihz = g_lissajous_pll_status.target_frequency_millihz;
  scaled = (uint64_t)(uint32_t)minimum_millihz *
           (uint64_t)target_millihz;
  scaled = (scaled + PLL_PHASE_SCHEDULE_BASE_MILLIHZ / 2ULL) /
           PLL_PHASE_SCHEDULE_BASE_MILLIHZ;

  if (scaled < (uint64_t)(uint32_t)minimum_millihz)
  {
    scaled = (uint64_t)(uint32_t)minimum_millihz;
  }
  if (scaled > (uint64_t)(uint32_t)maximum_millihz)
  {
    scaled = (uint64_t)(uint32_t)maximum_millihz;
  }
  return (int32_t)scaled;
}

static int32_t CalculatePhaseFrequencyOffsetMilliHz(
    int32_t nominal_frequency_error_millihz)
{
  int32_t phase_error_millidegrees;
  uint32_t absolute_phase_error_millidegrees;
  int32_t normal_limit_millihz;
  int32_t escape_offset_millihz;
  int64_t numerator;
  int32_t offset_millihz;

  normal_limit_millihz = ScheduledPhasePermissionMilliHz(
      PLL_PHASE_NORMAL_LIMIT_MIN_MILLIHZ,
      PLL_PHASE_NORMAL_LIMIT_MAX_MILLIHZ);
  escape_offset_millihz = ScheduledPhasePermissionMilliHz(
      PLL_PHASE_ESCAPE_OFFSET_MIN_MILLIHZ,
      PLL_PHASE_ESCAPE_OFFSET_MAX_MILLIHZ);
  g_lissajous_pll_status.phase_normal_limit_millihz =
      normal_limit_millihz;
  g_lissajous_pll_status.phase_escape_offset_limit_millihz =
      escape_offset_millihz;
  g_lissajous_pll_status.phase_loop_enabled = PLL_PHASE_LOOP_ENABLE;
  g_lissajous_pll_status.phase_loop_active = 0U;
  g_lissajous_pll_status.phase_escape_active =
      pll.phase_escape_active;
  g_lissajous_pll_status.phase_escape_sign =
      (int32_t)pll.phase_escape_sign;
  g_lissajous_pll_status.phase_escape_entry_count =
      pll.phase_escape_entry_count;
  g_lissajous_pll_status.phase_escape_exit_count =
      pll.phase_escape_exit_count;

  if ((PLL_PHASE_LOOP_ENABLE == 0U) ||
      (g_lissajous_pll_status.phase_measurement_valid == 0U) ||
      ((pll.phase_escape_active == 0U) &&
       (AbsoluteSigned32(nominal_frequency_error_millihz) >
        PLL_PHASE_ENTRY_ERROR_MILLIHZ)))
  {
    ResetPhaseEscapeState();
    return 0;
  }

  phase_error_millidegrees =
      g_lissajous_pll_status.phase_tracking_error_millidegrees;
  absolute_phase_error_millidegrees =
      AbsoluteSigned32(phase_error_millidegrees);

  /* Latch one escape direction near the wrapped-error ambiguity. Both
   * transitions require consecutive batches to reject high-frequency noise. */
  if (pll.phase_escape_active != 0U)
  {
    pll.phase_escape_entry_count = 0U;
    if (absolute_phase_error_millidegrees <=
        PLL_PHASE_ESCAPE_EXIT_MDEG)
    {
      if (pll.phase_escape_exit_count < 0xFFU)
      {
        ++pll.phase_escape_exit_count;
      }
      g_lissajous_pll_status.phase_escape_exit_count =
          pll.phase_escape_exit_count;
      if (pll.phase_escape_exit_count >=
          PLL_PHASE_ESCAPE_EXIT_CONFIRM_BATCHES)
      {
        ResetPhaseEscapeState();
      }
      else
      {
        g_lissajous_pll_status.phase_loop_active = 1U;
        return (int32_t)pll.phase_escape_sign *
               escape_offset_millihz;
      }
    }
    else
    {
      pll.phase_escape_exit_count = 0U;
      g_lissajous_pll_status.phase_escape_exit_count = 0U;
      g_lissajous_pll_status.phase_loop_active = 1U;
      return (int32_t)pll.phase_escape_sign *
             escape_offset_millihz;
    }
  }
  else
  {
    pll.phase_escape_exit_count = 0U;
    if (absolute_phase_error_millidegrees >=
        PLL_PHASE_ESCAPE_ENTRY_MDEG)
    {
      if (pll.phase_escape_entry_count < 0xFFU)
      {
        ++pll.phase_escape_entry_count;
      }
      g_lissajous_pll_status.phase_escape_entry_count =
          pll.phase_escape_entry_count;
      if (pll.phase_escape_entry_count >=
          PLL_PHASE_ESCAPE_ENTRY_CONFIRM_BATCHES)
      {
        pll.phase_escape_active = 1U;
        pll.phase_escape_entry_count = 0U;
        /* Measured polarity: increasing FB frequency decreases MEAS. */
        pll.phase_escape_sign =
            (phase_error_millidegrees >= 0) ? -1 : 1;
        g_lissajous_pll_status.phase_escape_active = 1U;
        g_lissajous_pll_status.phase_escape_sign =
            (int32_t)pll.phase_escape_sign;
        g_lissajous_pll_status.phase_escape_entry_count = 0U;
        g_lissajous_pll_status.phase_loop_active = 1U;
        return (int32_t)pll.phase_escape_sign *
               escape_offset_millihz;
      }
    }
    else
    {
      pll.phase_escape_entry_count = 0U;
      g_lissajous_pll_status.phase_escape_entry_count = 0U;
    }
  }

  if (absolute_phase_error_millidegrees <=
      PLL_PHASE_DEADBAND_MDEG)
  {
    return 0;
  }

  /* Measured plant polarity: increasing FB frequency decreases MEAS. */
  numerator = -(int64_t)phase_error_millidegrees *
              (int64_t)PLL_PHASE_P_MILLIHZ_PER_DEG;
  if (numerator >= 0)
  {
    offset_millihz = (int32_t)((numerator + 500LL) / 1000LL);
  }
  else
  {
    offset_millihz = (int32_t)((numerator - 500LL) / 1000LL);
  }
  offset_millihz = ClampSigned32(
      offset_millihz,
      -normal_limit_millihz,
      normal_limit_millihz);
  g_lissajous_pll_status.phase_loop_active = 1U;
  return offset_millihz;
}

static uint32_t MultiPeriodAverageTicks(const uint32_t *timestamps,
                                        uint32_t count,
                                        uint32_t cycles_per_capture)
{
  uint32_t span_ticks;
  uint32_t period_count;

  if ((timestamps == NULL) || (count < 2U) ||
      (count > LISSAJOUS_PLL_FB_MAX_SAMPLES) ||
      (cycles_per_capture == 0U))
  {
    return 0U;
  }

  period_count = (count - 1U) * cycles_per_capture;
  span_ticks = timestamps[count - 1U] - timestamps[0U];
  if (span_ticks == 0U)
  {
    return 0U;
  }

  return (span_ticks + period_count / 2U) / period_count;
}

static uint32_t MultiPeriodFrequencyMilliHz(const uint32_t *timestamps,
                                            uint32_t count,
                                            uint32_t timer_clock_hz,
                                            uint32_t cycles_per_capture)
{
  uint32_t span_ticks;
  uint32_t period_count;
  uint64_t numerator;

  if ((timestamps == NULL) || (count < 2U) ||
      (count > LISSAJOUS_PLL_FB_MAX_SAMPLES) ||
      (timer_clock_hz == 0U) ||
      (cycles_per_capture == 0U))
  {
    return 0U;
  }

  period_count = (count - 1U) * cycles_per_capture;
  span_ticks = timestamps[count - 1U] - timestamps[0U];
  if (span_ticks == 0U)
  {
    return 0U;
  }

  numerator = (uint64_t)period_count *
              (uint64_t)timer_clock_hz * 1000ULL;
  return (uint32_t)((numerator + span_ticks / 2U) / span_ticks);
}

static uint32_t ExpectedFeedbackPeriodTicks(void)
{
  uint32_t frequency_word;
  uint64_t numerator;
  uint64_t denominator;

  frequency_word = (uint32_t)(pll.ftw_accumulator + 0.5f);
  if (frequency_word < 1U)
  {
    frequency_word = 1U;
  }

  numerator = (uint64_t)g_lissajous_pll_status.timer_counter_hz *
              (1ULL << 28);
  denominator = (uint64_t)PLL_DDS_MCLK_HZ *
                (uint64_t)frequency_word;
  if (denominator == 0ULL)
  {
    return 0U;
  }

  return (uint32_t)((numerator + denominator / 2ULL) / denominator);
}

static uint8_t TimestampInDitherWindow(uint32_t timestamp)
{
  uint16_t index;
  uint32_t window_width;

  for (index = 0U; index < pll.dither_blank_window_count; ++index)
  {
    window_width = pll.dither_blank_end[index] -
                   pll.dither_blank_start[index];
    if ((uint32_t)(timestamp - pll.dither_blank_start[index]) <=
        window_width)
    {
      return 1U;
    }
  }
  return 0U;
}

static HAL_StatusTypeDef CleanFeedbackCapture(
    const uint32_t *timestamps,
    uint32_t count,
    uint32_t timer_clock_hz,
    uint32_t target_frequency_millihz,
    uint32_t reference_timestamp,
    uint32_t *clean_span_ticks,
    uint32_t *clean_average_ticks,
    uint32_t *clean_frequency_millihz,
    uint32_t *kept_edges,
    uint32_t *dropped_edges,
    uint32_t *reconstructed_cycles,
    uint32_t *blanking_threshold_ticks,
    uint32_t *last_valid_timestamp)
{
  uint32_t index;
  uint32_t expected_period;
  uint32_t target_period;
  uint32_t minimum_delta;
  uint32_t tolerance;
  uint32_t first_timestamp;
  uint32_t last_timestamp;
  uint32_t candidate;
  uint32_t delta;
  uint32_t cycles;
  uint32_t kept;
  uint32_t dropped;
  uint64_t modeled_delta;
  uint64_t residual;
  uint64_t total_cycles;
  uint64_t numerator;
  uint64_t relative_time;
  uint64_t sum_cycle;
  uint64_t sum_time;
  uint64_t sum_cycle_squared;
  uint64_t sum_cycle_time;
  int64_t fit_covariance;
  int64_t fit_variance;
  double fitted_period;
  double fitted_intercept;
  double reference_delta;
  double phase_delta;
  double fitted_phase_ticks;
  int32_t fit_quotient;
  int32_t fitted_phase_millidegrees;
  uint32_t fitted_phase_ticks_rounded;

  *clean_span_ticks = 0U;
  *clean_average_ticks = 0U;
  *clean_frequency_millihz = 0U;
  *kept_edges = 0U;
  *dropped_edges = 0U;
  *reconstructed_cycles = 0U;
  *blanking_threshold_ticks = 0U;
  *last_valid_timestamp = 0U;

  if ((timestamps == NULL) || (count < 2U) ||
      (timer_clock_hz == 0U) ||
      (target_frequency_millihz == 0U))
  {
    return HAL_ERROR;
  }

  expected_period = ExpectedFeedbackPeriodTicks();
  if (expected_period < 2U)
  {
    return HAL_ERROR;
  }
  target_period = (uint32_t)(
      ((uint64_t)timer_clock_hz * 1000ULL +
       (uint64_t)target_frequency_millihz / 2ULL) /
      (uint64_t)target_frequency_millihz);
  minimum_delta = (target_period * 3U + 2U) / 5U;
  if (minimum_delta < 1U)
  {
    minimum_delta = 1U;
  }
  *blanking_threshold_ticks = minimum_delta;
  tolerance = expected_period / PLL_SOFTWARE_EDGE_TOL_DIVISOR;

  first_timestamp = 0U;
  last_timestamp = 0U;
  kept = 0U;
  dropped = 0U;
  total_cycles = 0ULL;
  sum_cycle = 0ULL;
  sum_time = 0ULL;
  sum_cycle_squared = 0ULL;
  sum_cycle_time = 0ULL;

  for (index = 0U; index < count; ++index)
  {
    candidate = timestamps[index];
    if (TimestampInDitherWindow(candidate) != 0U)
    {
      ++dropped;
      continue;
    }

    if (kept == 0U)
    {
      first_timestamp = candidate;
      last_timestamp = candidate;
      kept = 1U;
      continue;
    }

    delta = candidate - last_timestamp;
    if (delta < minimum_delta)
    {
      ++dropped;
      continue;
    }

    cycles = (delta + expected_period / 2U) / expected_period;
    if (cycles < 1U)
    {
      cycles = 1U;
    }
    modeled_delta = (uint64_t)cycles * (uint64_t)expected_period;
    residual = (modeled_delta >= (uint64_t)delta) ?
               (modeled_delta - (uint64_t)delta) :
               ((uint64_t)delta - modeled_delta);
    if (residual > (uint64_t)tolerance)
    {
      ++dropped;
      continue;
    }

    total_cycles += (uint64_t)cycles;
    relative_time = (uint64_t)(candidate - first_timestamp);
    sum_cycle += total_cycles;
    sum_time += relative_time;
    sum_cycle_squared += total_cycles * total_cycles;
    sum_cycle_time += total_cycles * relative_time;
    last_timestamp = candidate;
    ++kept;
  }

  *kept_edges = kept;
  *dropped_edges = dropped;
  if (total_cycles > 0xFFFFFFFFULL)
  {
    return HAL_ERROR;
  }
  *reconstructed_cycles = (uint32_t)total_cycles;

  if ((kept < PLL_SOFTWARE_EDGE_MIN_KEEP) ||
      (total_cycles == 0ULL))
  {
    return HAL_ERROR;
  }

  *clean_span_ticks = last_timestamp - first_timestamp;
  if (*clean_span_ticks == 0U)
  {
    return HAL_ERROR;
  }
  *clean_average_ticks =
      (uint32_t)(((uint64_t)(*clean_span_ticks) + total_cycles / 2ULL) /
                 total_cycles);
  numerator = total_cycles * (uint64_t)timer_clock_hz * 1000ULL;
  *clean_frequency_millihz =
      (uint32_t)((numerator + (*clean_span_ticks) / 2U) /
                 (*clean_span_ticks));
  *last_valid_timestamp = last_timestamp;

  fit_covariance =
      (int64_t)((uint64_t)kept * sum_cycle_time) -
      (int64_t)(sum_cycle * sum_time);
  fit_variance =
      (int64_t)((uint64_t)kept * sum_cycle_squared) -
      (int64_t)(sum_cycle * sum_cycle);
  if ((fit_covariance > 0) && (fit_variance > 0))
  {
    fitted_period =
        (double)fit_covariance / (double)fit_variance;
    fitted_intercept =
        ((double)sum_time -
         fitted_period * (double)sum_cycle) /
        (double)kept;
    reference_delta =
        (double)(int32_t)(reference_timestamp - first_timestamp);
    phase_delta = fitted_intercept - reference_delta;
    fit_quotient = (int32_t)(phase_delta / fitted_period);
    fitted_phase_ticks =
        phase_delta - (double)fit_quotient * fitted_period;
    while (fitted_phase_ticks < 0.0)
    {
      fitted_phase_ticks += fitted_period;
    }
    while (fitted_phase_ticks >= fitted_period)
    {
      fitted_phase_ticks -= fitted_period;
    }
    fitted_phase_millidegrees =
        (int32_t)(fitted_phase_ticks * 360000.0 /
                  fitted_period + 0.5);
    fitted_phase_ticks_rounded =
        (uint32_t)(fitted_phase_ticks + 0.5);
    StoreMeasuredPhase(fitted_phase_millidegrees,
                       fitted_phase_ticks_rounded,
                       1U,
                       kept);
  }
  return HAL_OK;
}

static uint32_t FrequencyWordFromInputPeriod(uint32_t input_period,
                                             uint32_t multiplier)
{
  uint64_t numerator;
  uint64_t denominator;

  numerator = (uint64_t)LISSAJOUS_PLL_TIMER_CLOCK_HZ *
              (uint64_t)multiplier * (1ULL << 28);
  denominator = (uint64_t)input_period * (uint64_t)PLL_DDS_MCLK_HZ;
  if (denominator == 0ULL)
  {
    return 1U;
  }

  return ClampFrequencyWord((int64_t)((numerator + denominator / 2ULL) /
                                      denominator));
}

static uint32_t FeedbackBufferCapacity(void)
{
  return LISSAJOUS_PLL_FB_MAX_SAMPLES;
}

static void ResetLockState(void)
{
  g_lissajous_pll_status.locked = 0U;
  g_lissajous_pll_status.good_batches = 0U;
  g_lissajous_pll_status.phase_error_ticks = 0;
  g_lissajous_pll_status.phase_error_millidegrees = 0;
  g_lissajous_pll_status.frequency_error_ticks = 0;
  g_lissajous_pll_status.frequency_error_millihz = 0;
  pll.frequency_error_hz = 0.0f;
}

static void ApplyFrequencyMultiplier(uint8_t multiplier)
{
  g_lissajous_pll_status.multiplier = multiplier;
  g_lissajous_pll_status.target_phase_millidegrees =
      (uint32_t)PhaseTargetForMultiplier(multiplier);
  g_lissajous_pll_status.target_frequency_millihz = 0U;
  g_lissajous_pll_status.phase_frequency_offset_millihz = 0;
  g_lissajous_pll_status.frequency_spike_rejected = 0U;
  g_lissajous_pll_status.frequency_error_delta_millihz = 0;
  g_lissajous_pll_status.phase_loop_active = 0U;
  pll.follow_settling = 0U;
  pll.follow_reacquire_armed = 1U;
  ResetPhaseEscapeState();
  ResetFrequencyErrorFilterState();
  pll.follow_reacquire_input_millihz = 0U;
  pll.nominal_valid = 0U;
  ResetLockState();
}

static void ResetDmaHandle(DMA_HandleTypeDef *hdma)
{
  if (hdma == NULL)
  {
    return;
  }

  if (hdma->State == HAL_DMA_STATE_BUSY)
  {
    (void)HAL_DMA_Abort(hdma);
  }
  else
  {
    __HAL_DMA_DISABLE(hdma);
  }

  hdma->State = HAL_DMA_STATE_READY;
  hdma->ErrorCode = HAL_DMA_ERROR_NONE;
  hdma->Lock = HAL_UNLOCKED;
}

static uint32_t DmaCapturedCount(DMA_HandleTypeDef *hdma,
                                 uint32_t capacity,
                                 uint8_t buffer_full)
{
  uint32_t remaining;

  if (buffer_full != 0U)
  {
    return capacity;
  }
  if (hdma == NULL)
  {
    return 0U;
  }

  remaining = __HAL_DMA_GET_COUNTER(hdma);
  if (remaining > capacity)
  {
    return 0U;
  }
  return capacity - remaining;
}

static void FreezeAndSnapshotCapture(uint32_t now)
{
  if (pll.htim == NULL)
  {
    return;
  }

  __HAL_TIM_DISABLE_DMA(pll.htim, TIM_DMA_CC2 | TIM_DMA_CC3);
  TIM_CCxChannelCmd(pll.htim->Instance, TIM_CHANNEL_2, TIM_CCx_DISABLE);
  TIM_CCxChannelCmd(pll.htim->Instance, TIM_CHANNEL_3, TIM_CCx_DISABLE);
  __HAL_TIM_DISABLE(pll.htim);

  pll.ref_capture_count = DmaCapturedCount(
      pll.htim->hdma[TIM_DMA_ID_CC2],
      LISSAJOUS_PLL_REF_SAMPLES,
      pll.ref_ready);
  pll.fb_capture_count = DmaCapturedCount(
      pll.htim->hdma[TIM_DMA_ID_CC3],
      FeedbackBufferCapacity(),
      pll.fb_ready);
  pll.capture_elapsed_ms = now - pll.capture_started_ms;
  pll.fb_end_dither_switches = pll.dither_switches_total;
  pll.fb_end_dither_control_writes =
      pll.dither_control_writes_total;
}

static void StopCapture(void)
{
  if (pll.htim != NULL)
  {
    __HAL_TIM_DISABLE_DMA(pll.htim, TIM_DMA_CC2 | TIM_DMA_CC3);
    TIM_CCxChannelCmd(pll.htim->Instance, TIM_CHANNEL_2, TIM_CCx_DISABLE);
    TIM_CCxChannelCmd(pll.htim->Instance, TIM_CHANNEL_3, TIM_CCx_DISABLE);
    __HAL_TIM_DISABLE(pll.htim);

    ResetDmaHandle(pll.htim->hdma[TIM_DMA_ID_CC2]);
    ResetDmaHandle(pll.htim->hdma[TIM_DMA_ID_CC3]);
    TIM_CHANNEL_STATE_SET(pll.htim, TIM_CHANNEL_2,
                          HAL_TIM_CHANNEL_STATE_READY);
    TIM_CHANNEL_N_STATE_SET(pll.htim, TIM_CHANNEL_2,
                            HAL_TIM_CHANNEL_STATE_READY);
    TIM_CHANNEL_STATE_SET(pll.htim, TIM_CHANNEL_3,
                          HAL_TIM_CHANNEL_STATE_READY);
    TIM_CHANNEL_N_STATE_SET(pll.htim, TIM_CHANNEL_3,
                            HAL_TIM_CHANNEL_STATE_READY);
  }
  g_lissajous_pll_status.capture_running = 0U;
}

static HAL_StatusTypeDef StartCapture(void)
{
  HAL_StatusTypeDef result;

  StopCapture();
  pll.ref_ready = 0U;
  pll.fb_ready = 0U;
  pll.ref_capture_count = 0U;
  pll.fb_capture_count = 0U;
  pll.capture_elapsed_ms = 0U;
  pll.dither_blank_window_count = 0U;
  pll.capture_start_dither_switches = pll.dither_switches_total;
  pll.capture_start_dither_control_writes =
      pll.dither_control_writes_total;
  pll.fb_end_dither_switches = pll.dither_switches_total;
  pll.fb_end_dither_control_writes =
      pll.dither_control_writes_total;
  __HAL_TIM_SET_COUNTER(pll.htim, 0U);
  __HAL_TIM_CLEAR_FLAG(pll.htim, TIM_FLAG_CC2 | TIM_FLAG_CC3 |
                                 TIM_FLAG_CC2OF | TIM_FLAG_CC3OF);

  result = HAL_TIM_IC_Start_DMA(pll.htim, TIM_CHANNEL_2,
                                pll.ref_capture,
                                LISSAJOUS_PLL_REF_SAMPLES);
  if (result != HAL_OK)
  {
    StopCapture();
    return result;
  }

  result = HAL_TIM_IC_Start_DMA(pll.htim, TIM_CHANNEL_3,
                                pll.fb_capture,
                                (uint16_t)FeedbackBufferCapacity());
  if (result != HAL_OK)
  {
    StopCapture();
    return result;
  }

  pll.capture_started_ms = HAL_GetTick();
  g_lissajous_pll_status.capture_running = 1U;
  return HAL_OK;
}

static HAL_StatusTypeDef ReacquireFrequency(uint32_t input_period)
{
  uint32_t word;

  word = FrequencyWordFromInputPeriod(input_period,
                                      g_lissajous_pll_status.multiplier);
  pll.nominal_frequency_word = word;
  pll.nominal_valid = 1U;
  pll.last_input_period = input_period;

  if (AD9833_SetFrequencyWord(&pll.dds, word) != HAL_OK)
  {
    ++g_lissajous_pll_status.dds_errors;
    return HAL_ERROR;
  }

  g_lissajous_pll_status.frequency_word = word;
  ResetLockState();
  return HAL_OK;
}

static HAL_StatusTypeDef CorrectFrequency(uint32_t input_period,
                                         uint32_t output_period)
{
  int32_t error;
  int64_t correction;
  int32_t maximum_step;
  int64_t new_word;
  int64_t minimum_word;
  int64_t maximum_word;

  error = (int32_t)((uint64_t)output_period *
                    g_lissajous_pll_status.multiplier) -
          (int32_t)input_period;
  g_lissajous_pll_status.frequency_error_ticks = error;

  correction = ((int64_t)g_lissajous_pll_status.frequency_word *
                (int64_t)error) /
               ((int64_t)input_period * PLL_FREQ_GAIN_DIVISOR);

  maximum_step = (int32_t)(g_lissajous_pll_status.frequency_word /
                           PLL_MAX_FREQ_STEP_DIVISOR);
  if (maximum_step < 1)
  {
    maximum_step = 1;
  }
  correction = ClampSigned32((int32_t)correction,
                             -maximum_step, maximum_step);

  minimum_word = (int64_t)pll.nominal_frequency_word -
                 (int64_t)(pll.nominal_frequency_word /
                           PLL_MAX_TRIM_DIVISOR);
  maximum_word = (int64_t)pll.nominal_frequency_word +
                 (int64_t)(pll.nominal_frequency_word /
                           PLL_MAX_TRIM_DIVISOR);
  new_word = (int64_t)g_lissajous_pll_status.frequency_word + correction;
  if (new_word < minimum_word)
  {
    new_word = minimum_word;
  }
  if (new_word > maximum_word)
  {
    new_word = maximum_word;
  }

  if ((uint32_t)new_word != g_lissajous_pll_status.frequency_word)
  {
    if (AD9833_SetFrequencyWord(&pll.dds, (uint32_t)new_word) != HAL_OK)
    {
      ++g_lissajous_pll_status.dds_errors;
      return HAL_ERROR;
    }
    g_lissajous_pll_status.frequency_word = (uint32_t)new_word;
  }

  return HAL_OK;
}

static HAL_StatusTypeDef CorrectPhase(uint32_t input_period,
                                     uint32_t ref_timestamp,
                                     uint32_t fb_timestamp)
{
  uint32_t target_period;
  uint32_t desired_offset;
  uint32_t observed_offset;
  int32_t phase_error;
  int32_t phase_step;
  uint16_t new_phase_word;

  target_period = (input_period +
                   g_lissajous_pll_status.multiplier / 2U) /
                  g_lissajous_pll_status.multiplier;
  if (target_period == 0U)
  {
    return HAL_ERROR;
  }

  desired_offset = (pll.mode == LISSAJOUS_PLL_MODE_QUADRATURE) ?
                   (target_period / 4U) : 0U;
  observed_offset = (fb_timestamp - ref_timestamp) % target_period;
  phase_error = WrapSignedPeriod((int32_t)observed_offset -
                                 (int32_t)desired_offset,
                                 target_period);

  g_lissajous_pll_status.phase_error_ticks = phase_error;
  g_lissajous_pll_status.phase_error_millidegrees =
      (int32_t)(((int64_t)phase_error * 360000L) /
                (int64_t)target_period);

  if (AbsoluteSigned32(phase_error) <= PLL_PHASE_DEADBAND_TICKS)
  {
    return HAL_OK;
  }

  /*
   * AD9833 的正相位字会让正弦过零沿提前；反馈沿偏晚时，
   * phase_error 为正，因此增加相位字。
   */
  phase_step = (int32_t)(((int64_t)phase_error *
                          AD9833_PHASE_MODULUS) /
                         (int64_t)target_period);
  phase_step = ClampSigned32(phase_step,
                            -PLL_MAX_PHASE_STEP_WORDS,
                            PLL_MAX_PHASE_STEP_WORDS);
  if (phase_step == 0)
  {
    phase_step = (phase_error > 0) ? 1 : -1;
  }

  new_phase_word = WrapPhaseWord(
      (int32_t)g_lissajous_pll_status.phase_word + phase_step);
  if (AD9833_SetPhaseWord(&pll.dds, new_phase_word) != HAL_OK)
  {
    ++g_lissajous_pll_status.dds_errors;
    return HAL_ERROR;
  }
  g_lissajous_pll_status.phase_word = new_phase_word;
  return HAL_OK;
}

static void UpdateLockState(uint32_t input_period)
{
  uint32_t target_period;
  uint32_t frequency_limit;
  uint32_t phase_limit;

  target_period = input_period / g_lissajous_pll_status.multiplier;
  frequency_limit = input_period / 2000U;
  phase_limit = target_period / 100U;
  if (frequency_limit < 1U)
  {
    frequency_limit = 1U;
  }
  if (phase_limit < 2U)
  {
    phase_limit = 2U;
  }

  if ((AbsoluteSigned32(g_lissajous_pll_status.frequency_error_ticks) <=
       frequency_limit) &&
      (AbsoluteSigned32(g_lissajous_pll_status.phase_error_ticks) <=
       phase_limit))
  {
    if (g_lissajous_pll_status.good_batches < PLL_LOCK_BATCHES)
    {
      ++g_lissajous_pll_status.good_batches;
    }
  }
  else
  {
    g_lissajous_pll_status.good_batches = 0U;
  }

  g_lissajous_pll_status.locked =
      (g_lissajous_pll_status.good_batches >= PLL_LOCK_BATCHES) ? 1U : 0U;
}

static HAL_StatusTypeDef LoadDitherFrequencyPair(uint32_t low_word,
                                                 uint8_t discard_next_batch)
{
  uint32_t high_word;
  uint32_t inactive_word;
  uint32_t active_word;
  uint8_t active_register;
  uint8_t inactive_register;

#if PLL_DITHER_EDGE_TEST
  high_word = low_word + PLL_DITHER_EDGE_TEST_DELTA_FTW;
#else
  high_word = (low_word < AD9833_FTW_MASK) ?
              (low_word + 1U) : low_word;
#endif
  active_register = pll.dds.selected_frequency_register;
  inactive_register = (active_register == AD9833_FREQ_REGISTER_0) ?
                      AD9833_FREQ_REGISTER_1 : AD9833_FREQ_REGISTER_0;
  inactive_word = (inactive_register == AD9833_FREQ_REGISTER_0) ?
                  low_word : high_word;
  active_word = (active_register == AD9833_FREQ_REGISTER_0) ?
                low_word : high_word;

  /*
   * Never rewrite the selected register. Load the inactive register first,
   * switch to it with FSELECT, then load the old active register.
   */
  if (AD9833_WriteFrequencyRegister(&pll.dds, inactive_register,
                                    inactive_word) != HAL_OK)
  {
    goto load_failed;
  }
  if (AD9833_SelectFrequencyRegister(&pll.dds,
                                     inactive_register) != HAL_OK)
  {
    goto load_failed;
  }
  g_lissajous_pll_status.frequency_word = pll.dds.frequency_word;

  if (AD9833_WriteFrequencyRegister(&pll.dds, active_register,
                                    active_word) != HAL_OK)
  {
    goto load_failed;
  }

  if (AD9833_SelectFrequencyRegister(&pll.dds,
                                     AD9833_FREQ_REGISTER_0) != HAL_OK)
  {
    goto load_failed;
  }

  pll.dither_floor_word = low_word;
  pll.dither_pair_valid = 1U;
  pll.dither_accumulator = 0.0f;
  g_lissajous_pll_status.frequency_word = pll.dds.frequency_word;
  if (discard_next_batch != 0U)
  {
    pll.follow_settling = 1U;
    g_lissajous_pll_status.locked = 0U;
    g_lissajous_pll_status.good_batches = 0U;
  }
  return HAL_OK;

load_failed:
  pll.dither_pair_valid = 0U;
  ++g_lissajous_pll_status.dds_errors;
  ResetLockState();
  return HAL_ERROR;
}

static uint16_t BeginDitherTimestampWindow(void)
{
  uint16_t window_index;

  if ((pll.htim == NULL) ||
      (g_lissajous_pll_status.capture_running == 0U) ||
      (pll.fb_ready != 0U) ||
      (pll.dither_blank_window_count >= PLL_DITHER_BLANK_MAX_WINDOWS))
  {
    return PLL_DITHER_BLANK_MAX_WINDOWS;
  }

  window_index = pll.dither_blank_window_count;
  pll.dither_blank_start[window_index] =
      __HAL_TIM_GET_COUNTER(pll.htim);
  pll.dither_blank_end[window_index] =
      pll.dither_blank_start[window_index];
  pll.dither_blank_window_count = window_index + 1U;
  return window_index;
}

static void EndDitherTimestampWindow(uint16_t window_index)
{
  uint32_t guard_ticks;

  if (window_index >= PLL_DITHER_BLANK_MAX_WINDOWS)
  {
    return;
  }

  guard_ticks =
      (g_lissajous_pll_status.timer_counter_hz / 1000000UL) *
      PLL_DITHER_CAPTURE_GUARD_US;
  pll.dither_blank_end[window_index] =
      __HAL_TIM_GET_COUNTER(pll.htim) + guard_ticks;
}
static HAL_StatusTypeDef ApplyFrequencyDither(void)
{
  HAL_StatusTypeDef write_status;
  uint8_t previous_register;
  uint8_t control_word_written;
  uint16_t timestamp_window;
#if !(PLL_DITHER_EDGE_TEST && (PLL_DITHER_EDGE_TEST_MODE == PLL_DITHER_EDGE_TEST_MODE_C))
  uint8_t selected_register;
#endif
#if !PLL_DITHER_EDGE_TEST
  float fractional_word;
#endif

  if (pll.dither_pair_valid == 0U)
  {
    return HAL_ERROR;
  }

  previous_register = pll.dds.selected_frequency_register;
  control_word_written = 0U;
  timestamp_window = PLL_DITHER_BLANK_MAX_WINDOWS;
  write_status = HAL_OK;

#if PLL_DITHER_EDGE_TEST && (PLL_DITHER_EDGE_TEST_MODE == PLL_DITHER_EDGE_TEST_MODE_C)
  /* Test C: rewrite the same control word and record its SPI time window. */
  timestamp_window = BeginDitherTimestampWindow();
  write_status = AD9833_RewriteControlWord(&pll.dds);
  EndDitherTimestampWindow(timestamp_window);
  if (write_status != HAL_OK)
  {
    ++g_lissajous_pll_status.dds_errors;
    ResetLockState();
    return HAL_ERROR;
  }
  control_word_written = 1U;
#else
#if PLL_DITHER_EDGE_TEST
  selected_register =
      (previous_register == AD9833_FREQ_REGISTER_0) ?
      AD9833_FREQ_REGISTER_1 : AD9833_FREQ_REGISTER_0;
#else
  fractional_word =
      pll.ftw_accumulator - (float)pll.dither_floor_word;
  if (fractional_word < 0.0f)
  {
    fractional_word = 0.0f;
  }
  if (fractional_word >= 1.0f)
  {
    fractional_word = 0.999999f;
  }

  pll.dither_accumulator += fractional_word;
  selected_register = AD9833_FREQ_REGISTER_0;
  if (pll.dither_accumulator >= 1.0f)
  {
    pll.dither_accumulator -= 1.0f;
    selected_register = AD9833_FREQ_REGISTER_1;
  }
#endif

  if (selected_register != previous_register)
  {
    timestamp_window = BeginDitherTimestampWindow();
    write_status = AD9833_SelectFrequencyRegister(&pll.dds,
                                                  selected_register);
    EndDitherTimestampWindow(timestamp_window);
    if (write_status != HAL_OK)
    {
      ++g_lissajous_pll_status.dds_errors;
      ResetLockState();
      return HAL_ERROR;
    }
    control_word_written = 1U;
  }
#endif

  if (pll.dds.selected_frequency_register != previous_register)
  {
    ++pll.dither_switches_total;
  }
  if (control_word_written != 0U)
  {
    ++pll.dither_control_writes_total;
  }
  g_lissajous_pll_status.frequency_word = pll.dds.frequency_word;
  return HAL_OK;
}
static void FollowInputFrequency(uint32_t input_period,
                                 uint32_t output_period,
                                 uint32_t input_frequency_millihz,
                                 uint32_t target_frequency_millihz,
                                 uint32_t output_frequency_millihz)
{
  int32_t error_millihz;
  int32_t nominal_error_millihz;
  int32_t phase_offset_millihz;
  int32_t filtered_nominal_error_millihz;
  int32_t integrator_error_millihz;
  int64_t control_target_signed;
  uint32_t control_target_millihz;
  uint32_t target_period;
  uint32_t new_floor_word;
  float error_hz;
  float nominal_error_hz;
  float integrator_error_hz;
  float step_ftw;
  float gain_scale;
  float reacquired_ftw;
  uint8_t error_sample_accepted;

  nominal_error_millihz =
      (int32_t)target_frequency_millihz -
      (int32_t)output_frequency_millihz;
  nominal_error_hz = (float)nominal_error_millihz / 1000.0f;
  phase_offset_millihz =
      CalculatePhaseFrequencyOffsetMilliHz(nominal_error_millihz);
  control_target_signed =
      (int64_t)target_frequency_millihz +
      (int64_t)phase_offset_millihz;
  if (control_target_signed < 1)
  {
    control_target_signed = 1;
  }
  if (control_target_signed > 0xFFFFFFFFLL)
  {
    control_target_signed = 0xFFFFFFFFLL;
  }
  control_target_millihz = (uint32_t)control_target_signed;
  error_millihz =
      (int32_t)control_target_millihz -
      (int32_t)output_frequency_millihz;
  error_hz = (float)error_millihz / 1000.0f;
  pll.frequency_error_hz = error_hz;
  g_lissajous_pll_status.frequency_error_millihz = error_millihz;
  g_lissajous_pll_status.nominal_frequency_error_millihz =
      nominal_error_millihz;
  g_lissajous_pll_status.phase_frequency_offset_millihz =
      phase_offset_millihz;
  g_lissajous_pll_status.control_target_frequency_millihz =
      control_target_millihz;
  target_period = (input_period +
                   g_lissajous_pll_status.multiplier / 2U) /
                  g_lissajous_pll_status.multiplier;
  g_lissajous_pll_status.frequency_error_ticks =
      (int32_t)output_period - (int32_t)target_period;
  g_lissajous_pll_status.phase_error_ticks = 0;
  g_lissajous_pll_status.phase_error_millidegrees = 0;
  g_lissajous_pll_status.command_frequency_hz =
      (control_target_millihz + 500U) / 1000U;

#if PLL_DITHER_EDGE_TEST
  /* Diagnostic mode: measure only; never change the frozen test FTW. */
  g_lissajous_pll_status.locked = 0U;
  g_lissajous_pll_status.good_batches = 0U;
  g_lissajous_pll_status.estimated_mclk_hz = 0U;
  return;
#endif

  /* Arm a new coarse jump only after the previous target was reached, or
   * when the measured input itself has moved by more than 5 Hz. */
  if ((nominal_error_hz <= PLL_FOLLOW_REACQUIRE_ERROR_HZ) &&
      (nominal_error_hz >= -PLL_FOLLOW_REACQUIRE_ERROR_HZ))
  {
    pll.follow_reacquire_armed = 1U;
  }

  /* Coarse acquisition uses IN only. A bad FB batch can trigger this branch,
   * but it can no longer collapse the commanded FTW through an IN/FB ratio. */
  if (((nominal_error_hz > PLL_FOLLOW_REACQUIRE_ERROR_HZ) ||
       (nominal_error_hz < -PLL_FOLLOW_REACQUIRE_ERROR_HZ)) &&
      ((pll.follow_reacquire_armed != 0U) ||
       (pll.follow_reacquire_input_millihz == 0U) ||
       (AbsoluteDifferenceU32(input_frequency_millihz,
                              pll.follow_reacquire_input_millihz) >
        PLL_FOLLOW_INPUT_CHANGE_MHZ)))
  {
    reacquired_ftw = (float)target_frequency_millihz *
                     (PLL_FTW_PER_HZ / 1000.0f);
    if (reacquired_ftw < PLL_FOLLOW_MIN_SAFE_FTW)
    {
      reacquired_ftw = PLL_FOLLOW_MIN_SAFE_FTW;
    }
    if (reacquired_ftw > PLL_FOLLOW_MAX_SAFE_FTW)
    {
      reacquired_ftw = PLL_FOLLOW_MAX_SAFE_FTW;
    }

    pll.ftw_accumulator = reacquired_ftw;
    pll.follow_reacquire_armed = 0U;
    pll.follow_reacquire_input_millihz = input_frequency_millihz;
    new_floor_word = (uint32_t)reacquired_ftw;
    pll.nominal_frequency_word = new_floor_word;
    pll.nominal_valid = 1U;
    pll.last_input_period = input_period;
    g_lissajous_pll_status.locked = 0U;
    g_lissajous_pll_status.good_batches = 0U;
    (void)LoadDitherFrequencyPair(new_floor_word, 1U);
    ResetPhaseEscapeState();
    ResetFrequencyErrorFilterState();
    return;
  }

  if (AbsoluteSigned32(nominal_error_millihz) >
      PLL_PHASE_ENTRY_ERROR_MILLIHZ)
  {
    pll.error_filter_valid = 0U;
    pll.error_spike_candidate_valid = 0U;
    pll.error_spike_candidate_count = 0U;
    pll.nominal_error_sum_millihz = 0LL;
    pll.nominal_error_history_count = 0U;
    pll.nominal_error_history_index = 0U;
    g_lissajous_pll_status.frequency_spike_rejected = 0U;
    g_lissajous_pll_status.frequency_error_delta_millihz = 0;
    error_sample_accepted = 1U;
    filtered_nominal_error_millihz = nominal_error_millihz;
  }
  else
  {
    error_sample_accepted =
        FrequencyErrorSampleAccepted(nominal_error_millihz);
    filtered_nominal_error_millihz =
        FilteredNominalFrequencyErrorMilliHz(
            nominal_error_millihz, error_sample_accepted);
  }
  integrator_error_millihz =
      filtered_nominal_error_millihz + phase_offset_millihz;
  integrator_error_hz =
      (float)integrator_error_millihz / 1000.0f;
  g_lissajous_pll_status.filtered_nominal_frequency_error_millihz =
      filtered_nominal_error_millihz;
  g_lissajous_pll_status.integrator_frequency_error_millihz =
      integrator_error_millihz;

  if ((g_lissajous_pll_status.phase_loop_active != 0U) ||
      (integrator_error_hz > PLL_FREQUENCY_DEADBAND_HZ) ||
      (integrator_error_hz < -PLL_FREQUENCY_DEADBAND_HZ))
  {
    gain_scale = (float)pll.capture_elapsed_ms /
                 PLL_FREQUENCY_I_REFERENCE_MS;
    step_ftw = PLL_FREQUENCY_I_GAIN * gain_scale *
               integrator_error_hz * PLL_FTW_PER_HZ;
    if (step_ftw > PLL_FREQUENCY_MAX_STEP_FTW)
    {
      step_ftw = PLL_FREQUENCY_MAX_STEP_FTW;
    }
    if (step_ftw < -PLL_FREQUENCY_MAX_STEP_FTW)
    {
      step_ftw = -PLL_FREQUENCY_MAX_STEP_FTW;
    }

    pll.ftw_accumulator += step_ftw;
    if (pll.ftw_accumulator < PLL_FOLLOW_MIN_SAFE_FTW)
    {
      pll.ftw_accumulator = PLL_FOLLOW_MIN_SAFE_FTW;
    }
    if (pll.ftw_accumulator > PLL_FOLLOW_MAX_SAFE_FTW)
    {
      pll.ftw_accumulator = PLL_FOLLOW_MAX_SAFE_FTW;
    }
  }

  if ((integrator_error_hz <= PLL_LOCK_THRESHOLD_HZ) &&
      (integrator_error_hz >= -PLL_LOCK_THRESHOLD_HZ) &&
      (g_lissajous_pll_status.phase_measurement_valid != 0U) &&
      (g_lissajous_pll_status.phase_escape_active == 0U) &&
      (AbsoluteSigned32(
           g_lissajous_pll_status.phase_tracking_error_millidegrees) <=
       PLL_LOCK_PHASE_THRESHOLD_MDEG))
  {
    if (g_lissajous_pll_status.good_batches < PLL_LOCK_BATCHES)
    {
      ++g_lissajous_pll_status.good_batches;
    }
  }
  else
  {
    g_lissajous_pll_status.good_batches = 0U;
  }
  g_lissajous_pll_status.locked =
      (g_lissajous_pll_status.good_batches >= PLL_LOCK_BATCHES) ? 1U : 0U;

  new_floor_word = (uint32_t)pll.ftw_accumulator;
  if ((pll.dither_pair_valid == 0U) ||
      (new_floor_word != pll.dither_floor_word))
  {
    /* Fine tracking reloads between capture batches, so no batch discard. */
    (void)LoadDitherFrequencyPair(new_floor_word, 0U);
  }

  pll.nominal_frequency_word = new_floor_word;
  pll.nominal_valid = 1U;
  pll.last_input_period = input_period;
  g_lissajous_pll_status.estimated_mclk_hz = 0U;
}

static void ProcessCaptureBatch(void)
{
  HAL_StatusTypeDef feedback_clean_status;
  uint32_t input_period;
  uint32_t output_period;
  uint32_t input_span_ticks;
  uint32_t output_span_ticks;
  uint32_t raw_output_span_ticks;
  uint32_t input_frequency_millihz;
  uint32_t target_frequency_millihz;
  uint32_t output_frequency_millihz;
  uint32_t raw_output_frequency_millihz;
  uint32_t blanking_threshold_ticks;
  uint32_t last_valid_fb_timestamp;
  uint32_t feedback_kept_edges;
  uint32_t feedback_dropped_edges;
  uint32_t feedback_reconstructed_cycles;
  uint32_t input_change_limit;
  uint32_t i;
  uint32_t ref_count;
  uint32_t fb_count;
#if PLL_DITHER_EDGE_TEST
  uint32_t expected_feedback_millihz;
  uint32_t expected_feedback_cycles;
  uint64_t expected_numerator;
  uint64_t expected_denominator;
#endif

  ref_count = pll.ref_capture_count;
  fb_count = pll.fb_capture_count;
  g_lissajous_pll_status.phase_measurement_valid = 0U;
  g_lissajous_pll_status.phase_fit_used = 0U;
  g_lissajous_pll_status.phase_fit_points = 0U;
  g_lissajous_pll_status.phase_loop_active = 0U;
  g_lissajous_pll_status.phase_frequency_offset_millihz = 0;
  g_lissajous_pll_status.frequency_spike_rejected = 0U;
  g_lissajous_pll_status.frequency_error_delta_millihz = 0;
  g_lissajous_pll_status.input_captured_edges = ref_count;
  g_lissajous_pll_status.feedback_captured_edges = fb_count;
  g_lissajous_pll_status.capture_window_ms =
      LISSAJOUS_PLL_CAPTURE_WINDOW_MS;
  g_lissajous_pll_status.capture_elapsed_ms =
      pll.capture_elapsed_ms;
  g_lissajous_pll_status.capture_edge_divisor =
      LISSAJOUS_PLL_CAPTURE_EDGE_DIVISOR;

  if ((ref_count < 2U) || (fb_count < 2U))
  {
    ++g_lissajous_pll_status.rejected_batches;
    g_lissajous_pll_status.data_valid = 0U;
    ResetLockState();
    return;
  }

  if (pll.raw_snapshot_ready == 0U)
  {
    pll.raw_ref_snapshot_count =
        (ref_count < LISSAJOUS_PLL_RAW_SAMPLES) ?
        ref_count : LISSAJOUS_PLL_RAW_SAMPLES;
    pll.raw_fb_snapshot_count =
        (fb_count < LISSAJOUS_PLL_RAW_SAMPLES) ?
        fb_count : LISSAJOUS_PLL_RAW_SAMPLES;
    for (i = 0U; i < pll.raw_ref_snapshot_count; ++i)
    {
      pll.raw_ref_snapshot[i] = pll.ref_capture[i];
    }
    for (i = 0U; i < pll.raw_fb_snapshot_count; ++i)
    {
      pll.raw_fb_snapshot[i] = pll.fb_capture[i];
    }
    pll.raw_snapshot_ready = 1U;
  }

  input_span_ticks = pll.ref_capture[ref_count - 1U] -
                     pll.ref_capture[0U];
  raw_output_span_ticks = pll.fb_capture[fb_count - 1U] -
                          pll.fb_capture[0U];
  input_period = MultiPeriodAverageTicks(
      pll.ref_capture,
      ref_count,
      LISSAJOUS_PLL_CAPTURE_EDGE_DIVISOR);
  input_frequency_millihz = MultiPeriodFrequencyMilliHz(
      pll.ref_capture,
      ref_count,
      g_lissajous_pll_status.timer_counter_hz,
      LISSAJOUS_PLL_CAPTURE_EDGE_DIVISOR);
  target_frequency_millihz =
      TargetFrequencyMilliHz(input_frequency_millihz);
  raw_output_frequency_millihz = MultiPeriodFrequencyMilliHz(
      pll.fb_capture,
      fb_count,
      g_lissajous_pll_status.timer_counter_hz,
      LISSAJOUS_PLL_CAPTURE_EDGE_DIVISOR);

  feedback_clean_status = HAL_OK;
  blanking_threshold_ticks = 0U;
  last_valid_fb_timestamp = pll.fb_capture[fb_count - 1U];
  feedback_kept_edges = fb_count;
  feedback_dropped_edges = 0U;
  feedback_reconstructed_cycles =
      (fb_count - 1U) * LISSAJOUS_PLL_CAPTURE_EDGE_DIVISOR;
#if PLL_SOFTWARE_EDGE_BLANK_ENABLE
  if (pll.mode == LISSAJOUS_PLL_MODE_FREQUENCY_FOLLOW)
  {
    feedback_clean_status = CleanFeedbackCapture(
        pll.fb_capture,
        fb_count,
        g_lissajous_pll_status.timer_counter_hz,
        target_frequency_millihz,
        pll.ref_capture[ref_count - 1U],
        &output_span_ticks,
        &output_period,
        &output_frequency_millihz,
        &feedback_kept_edges,
        &feedback_dropped_edges,
        &feedback_reconstructed_cycles,
        &blanking_threshold_ticks,
        &last_valid_fb_timestamp);
  }
  else
  {
    output_span_ticks = raw_output_span_ticks;
    output_period = MultiPeriodAverageTicks(
        pll.fb_capture,
        fb_count,
        LISSAJOUS_PLL_CAPTURE_EDGE_DIVISOR);
    output_frequency_millihz = raw_output_frequency_millihz;
  }
#else
  output_span_ticks = raw_output_span_ticks;
  output_period = MultiPeriodAverageTicks(
      pll.fb_capture,
      fb_count,
      LISSAJOUS_PLL_CAPTURE_EDGE_DIVISOR);
  output_frequency_millihz = raw_output_frequency_millihz;
#endif

  g_lissajous_pll_status.input_span_ticks = input_span_ticks;
  g_lissajous_pll_status.output_span_ticks = output_span_ticks;
  g_lissajous_pll_status.input_period_ticks = input_period;
  g_lissajous_pll_status.output_period_ticks = output_period;
  g_lissajous_pll_status.input_frequency_millihz =
      input_frequency_millihz;
  g_lissajous_pll_status.target_frequency_millihz =
      target_frequency_millihz;
  g_lissajous_pll_status.output_frequency_millihz =
      output_frequency_millihz;
  g_lissajous_pll_status.raw_output_frequency_millihz =
      raw_output_frequency_millihz;
  g_lissajous_pll_status.dither_capture_guard_us =
      PLL_DITHER_CAPTURE_GUARD_US;
  g_lissajous_pll_status.dither_blank_windows =
      pll.dither_blank_window_count;
  g_lissajous_pll_status.feedback_kept_edges = feedback_kept_edges;
  g_lissajous_pll_status.feedback_dropped_edges = feedback_dropped_edges;
  g_lissajous_pll_status.feedback_reconstructed_cycles =
      feedback_reconstructed_cycles;
  g_lissajous_pll_status.feedback_blanking_threshold_ticks =
      blanking_threshold_ticks;
  g_lissajous_pll_status.input_measurement_periods =
      (ref_count - 1U) * LISSAJOUS_PLL_CAPTURE_EDGE_DIVISOR;
  g_lissajous_pll_status.feedback_measurement_periods =
      feedback_reconstructed_cycles;
  g_lissajous_pll_status.dither_switches =
      pll.fb_end_dither_switches - pll.capture_start_dither_switches;
  g_lissajous_pll_status.dither_control_writes =
      pll.fb_end_dither_control_writes -
      pll.capture_start_dither_control_writes;

#if PLL_DITHER_EDGE_TEST
  g_lissajous_pll_status.dither_test_enabled = 1U;
  g_lissajous_pll_status.dither_test_mode =
      PLL_DITHER_EDGE_TEST_MODE;
  g_lissajous_pll_status.dither_capture_guard_us =
      PLL_DITHER_CAPTURE_GUARD_US;
  g_lissajous_pll_status.dither_test_delta_ftw =
      PLL_DITHER_EDGE_TEST_DELTA_FTW;

  expected_numerator =
      ((uint64_t)(2UL * PLL_DITHER_EDGE_TEST_FTW +
                  PLL_DITHER_EDGE_TEST_DELTA_FTW) *
       (uint64_t)PLL_DDS_MCLK_HZ * 1000ULL);
  expected_denominator = 2ULL * (1ULL << 28);
  expected_feedback_millihz =
      (uint32_t)((expected_numerator + expected_denominator / 2ULL) /
                 expected_denominator);

  expected_numerator =
      (uint64_t)raw_output_span_ticks *
      (uint64_t)expected_feedback_millihz;
  expected_denominator =
      (uint64_t)g_lissajous_pll_status.timer_counter_hz * 1000ULL;
  expected_feedback_cycles =
      (uint32_t)((expected_numerator + expected_denominator / 2ULL) /
                 expected_denominator);
  g_lissajous_pll_status.feedback_excess_edges =
      (int32_t)((fb_count - 1U) *
                LISSAJOUS_PLL_CAPTURE_EDGE_DIVISOR) -
      (int32_t)expected_feedback_cycles;
#endif

  if ((feedback_clean_status != HAL_OK) ||
      (input_period < PLL_INPUT_PERIOD_MIN_TICKS) ||
      (input_period > PLL_INPUT_PERIOD_MAX_TICKS) ||
      (output_period < PLL_OUTPUT_PERIOD_MIN_TICKS) ||
      (output_period > PLL_OUTPUT_PERIOD_MAX_TICKS))
  {
    ++g_lissajous_pll_status.rejected_batches;
    g_lissajous_pll_status.data_valid = 0U;
    g_lissajous_pll_status.phase_measurement_valid = 0U;
    g_lissajous_pll_status.phase_fit_used = 0U;
    g_lissajous_pll_status.phase_fit_points = 0U;
    ResetLockState();
    return;
  }

  if (g_lissajous_pll_status.phase_measurement_valid == 0U)
  {
    UpdateMeasuredPhase(pll.ref_capture[ref_count - 1U],
                        last_valid_fb_timestamp,
                        output_period);
  }

  g_lissajous_pll_status.input_frequency_hz =
      (input_frequency_millihz + 500U) / 1000U;
  g_lissajous_pll_status.output_frequency_hz =
      (output_frequency_millihz + 500U) / 1000U;
  ++g_lissajous_pll_status.completed_batches;
  g_lissajous_pll_status.last_batch_tick_ms = HAL_GetTick();
  g_lissajous_pll_status.data_valid = 1U;

  if ((pll.mode == LISSAJOUS_PLL_MODE_FREQUENCY_FOLLOW) &&
      (pll.follow_settling != 0U))
  {
    /* Keep diagnostics coherent while skipping only the control update. */
    pll.frequency_error_hz =
        (float)((int32_t)target_frequency_millihz -
                (int32_t)output_frequency_millihz) / 1000.0f;
    g_lissajous_pll_status.frequency_error_millihz =
        (int32_t)target_frequency_millihz -
        (int32_t)output_frequency_millihz;
    g_lissajous_pll_status.nominal_frequency_error_millihz =
        g_lissajous_pll_status.frequency_error_millihz;
    g_lissajous_pll_status.phase_frequency_offset_millihz = 0;
    g_lissajous_pll_status.frequency_spike_rejected = 0U;
    g_lissajous_pll_status.frequency_error_delta_millihz = 0;
    g_lissajous_pll_status.phase_loop_enabled = PLL_PHASE_LOOP_ENABLE;
    g_lissajous_pll_status.phase_loop_active = 0U;
    g_lissajous_pll_status.control_target_frequency_millihz =
        target_frequency_millihz;
    g_lissajous_pll_status.frequency_error_ticks =
        (int32_t)output_period -
        (int32_t)((input_period +
                   g_lissajous_pll_status.multiplier / 2U) /
                  g_lissajous_pll_status.multiplier);
    g_lissajous_pll_status.command_frequency_hz =
        (target_frequency_millihz + 500U) / 1000U;
    g_lissajous_pll_status.phase_error_ticks = 0;
    g_lissajous_pll_status.phase_error_millidegrees = 0;
    pll.follow_settling = 0U;
    g_lissajous_pll_status.data_valid = 0U;
    return;
  }

  if (pll.mode == LISSAJOUS_PLL_MODE_FREQUENCY_FOLLOW)
  {
    FollowInputFrequency(input_period,
                         output_period,
                         input_frequency_millihz,
                         target_frequency_millihz,
                         output_frequency_millihz);
    return;
  }

  input_change_limit = input_period / PLL_REACQUIRE_DIVISOR;
  if (input_change_limit < 2U)
  {
    input_change_limit = 2U;
  }

  if ((!pll.nominal_valid) ||
      (AbsoluteDifferenceU32(input_period, pll.last_input_period) >
       input_change_limit))
  {
    (void)ReacquireFrequency(input_period);
    return;
  }

  pll.last_input_period = input_period;
  if (CorrectFrequency(input_period, output_period) != HAL_OK)
  {
    ResetLockState();
    return;
  }

  if (CorrectPhase(input_period,
                   pll.ref_capture[ref_count - 1U],
                   pll.fb_capture[fb_count - 1U]) != HAL_OK)
  {
    ResetLockState();
    return;
  }

  UpdateLockState(input_period);
}

HAL_StatusTypeDef LissajousPLL_Init(TIM_HandleTypeDef *htim,
                                    SPI_HandleTypeDef *hspi,
                                    LissajousPLL_Mode mode)
{
  uint32_t i;
  volatile uint32_t *status_words;

  if ((htim == NULL) || (hspi == NULL) ||
      ((mode != LISSAJOUS_PLL_MODE_FREQUENCY_FOLLOW) &&
       (mode != LISSAJOUS_PLL_MODE_QUADRATURE) &&
       (mode != LISSAJOUS_PLL_MODE_DOUBLE)))
  {
    return HAL_ERROR;
  }

  status_words = (volatile uint32_t *)&g_lissajous_pll_status;
  for (i = 0U; i < (sizeof(g_lissajous_pll_status) / sizeof(uint32_t)); ++i)
  {
    status_words[i] = 0U;
  }

  pll.htim = htim;
  pll.mode = mode;
  pll.ref_ready = 0U;
  pll.fb_ready = 0U;
  pll.ref_capture_count = 0U;
  pll.fb_capture_count = 0U;
  pll.capture_elapsed_ms = 0U;
  pll.dither_blank_window_count = 0U;
  pll.raw_snapshot_ready = 0U;
  pll.raw_ref_snapshot_count = 0U;
  pll.raw_fb_snapshot_count = 0U;
  pll.initialized = 0U;
  pll.follow_settling = 0U;
  pll.follow_reacquire_armed = 1U;
  ResetPhaseEscapeState();
  ResetFrequencyErrorFilterState();
  pll.follow_reacquire_input_millihz = 0U;
  pll.nominal_valid = 0U;
  pll.nominal_frequency_word = 0U;
  pll.dither_pair_valid = 0U;
  pll.dither_floor_word = 0U;
  pll.last_input_period = 0U;
  pll.next_capture_ms = HAL_GetTick();
  pll.dither_switches_total = 0U;
  pll.capture_start_dither_switches = 0U;
  pll.fb_end_dither_switches = 0U;
  pll.dither_control_writes_total = 0U;
  pll.capture_start_dither_control_writes = 0U;
  pll.fb_end_dither_control_writes = 0U;

  g_lissajous_pll_status.apb1_clock_hz =
      HAL_RCC_GetPCLK1Freq();
  g_lissajous_pll_status.timer_prescaler =
      htim->Instance->PSC;
  g_lissajous_pll_status.timer_counter_hz =
      Timer5CounterClockHz(htim);
  g_lissajous_pll_status.input_measurement_periods = 0U;
  g_lissajous_pll_status.feedback_measurement_periods = 0U;
  g_lissajous_pll_status.capture_window_ms =
      LISSAJOUS_PLL_CAPTURE_WINDOW_MS;
  g_lissajous_pll_status.capture_edge_divisor =
      LISSAJOUS_PLL_CAPTURE_EDGE_DIVISOR;
  g_lissajous_pll_status.dither_capture_guard_us =
      PLL_DITHER_CAPTURE_GUARD_US;
  g_lissajous_pll_status.phase_loop_enabled =
      PLL_PHASE_LOOP_ENABLE;
  g_lissajous_pll_status.phase_normal_limit_millihz =
      PLL_PHASE_NORMAL_LIMIT_MIN_MILLIHZ;
  g_lissajous_pll_status.phase_escape_offset_limit_millihz =
      PLL_PHASE_ESCAPE_OFFSET_MIN_MILLIHZ;
  if (mode == LISSAJOUS_PLL_MODE_DOUBLE)
  {
    M = 2U;
  }
  else if (mode == LISSAJOUS_PLL_MODE_QUADRATURE)
  {
    M = 1U;
  }
  else if ((M != 1U) && (M != 2U))
  {
    M = LISSAJOUS_PLL_DEFAULT_MULTIPLIER;
  }
  g_lissajous_pll_status.multiplier = M;
  g_lissajous_pll_status.target_phase_millidegrees =
      (uint32_t)PhaseTargetForMultiplier(M);

  if (AD9833_Init(&pll.dds, hspi, PLL_DDS_MCLK_HZ,
                  PLL_START_FREQUENCY_HZ) != HAL_OK)
  {
    ++g_lissajous_pll_status.dds_errors;
    return HAL_ERROR;
  }

#if PLL_DITHER_EDGE_TEST
  pll.ftw_accumulator = (float)PLL_DITHER_EDGE_TEST_FTW;
#else
  pll.ftw_accumulator = (float)pll.dds.frequency_word;
#endif
  pll.frequency_error_hz = 0.0f;
  pll.dither_accumulator = 0.0f;
  pll.dither_pair_valid = 0U;
  pll.dither_floor_word = 0U;
  if (LoadDitherFrequencyPair((uint32_t)pll.ftw_accumulator, 0U) != HAL_OK)
  {
    return HAL_ERROR;
  }
  pll.next_dither_ms = HAL_GetTick();
  g_lissajous_pll_status.frequency_word = pll.dds.frequency_word;
#if PLL_DITHER_EDGE_TEST
  g_lissajous_pll_status.command_frequency_hz = 1000U;
  g_lissajous_pll_status.dither_test_enabled = 1U;
  g_lissajous_pll_status.dither_test_mode =
      PLL_DITHER_EDGE_TEST_MODE;
  g_lissajous_pll_status.dither_capture_guard_us =
      PLL_DITHER_CAPTURE_GUARD_US;
  g_lissajous_pll_status.dither_test_delta_ftw =
      PLL_DITHER_EDGE_TEST_DELTA_FTW;
#else
  g_lissajous_pll_status.command_frequency_hz = PLL_START_FREQUENCY_HZ;
#endif
  g_lissajous_pll_status.phase_word = pll.dds.phase_word;
  pll.initialized = 1U;
  return HAL_OK;
}

void LissajousPLL_SetMode(LissajousPLL_Mode mode)
{
  if ((mode != LISSAJOUS_PLL_MODE_FREQUENCY_FOLLOW) &&
      (mode != LISSAJOUS_PLL_MODE_QUADRATURE) &&
      (mode != LISSAJOUS_PLL_MODE_DOUBLE))
  {
    return;
  }

  if (pll.mode == mode)
  {
    return;
  }

  StopCapture();
  pll.mode = mode;
  pll.ftw_accumulator = (float)g_lissajous_pll_status.frequency_word;
  pll.frequency_error_hz = 0.0f;
  pll.dither_accumulator = 0.0f;
  pll.dither_pair_valid = 0U;
  pll.dither_floor_word = 0U;
  pll.next_dither_ms = HAL_GetTick();
  pll.nominal_valid = 0U;
  pll.follow_settling = 0U;
  pll.follow_reacquire_armed = 1U;
  ResetPhaseEscapeState();
  ResetFrequencyErrorFilterState();
  pll.follow_reacquire_input_millihz = 0U;
  pll.next_capture_ms = HAL_GetTick() + PLL_RESTART_DELAY_MS;
  g_lissajous_pll_status.data_valid = 0U;
  g_lissajous_pll_status.estimated_mclk_hz = 0U;
  if (mode == LISSAJOUS_PLL_MODE_DOUBLE)
  {
    M = 2U;
  }
  else if (mode == LISSAJOUS_PLL_MODE_QUADRATURE)
  {
    M = 1U;
  }
  ApplyFrequencyMultiplier(M);
  if ((mode == LISSAJOUS_PLL_MODE_FREQUENCY_FOLLOW) &&
      (LoadDitherFrequencyPair((uint32_t)pll.ftw_accumulator, 0U) != HAL_OK))
  {
    g_lissajous_pll_status.data_valid = 0U;
  }
  ResetLockState();
}

LissajousPLL_Mode LissajousPLL_GetMode(void)
{
  return pll.mode;
}

HAL_StatusTypeDef LissajousPLL_SetMultiplier(uint8_t multiplier)
{
  if ((multiplier != 1U) && (multiplier != 2U))
  {
    return HAL_ERROR;
  }

  M = multiplier;
  if ((pll.initialized != 0U) &&
      (g_lissajous_pll_status.multiplier != multiplier))
  {
    ApplyFrequencyMultiplier(multiplier);
  }
  return HAL_OK;
}

uint8_t LissajousPLL_GetMultiplier(void)
{
  return g_lissajous_pll_status.multiplier;
}

const volatile LissajousPLL_Status *LissajousPLL_GetStatus(void)
{
  return &g_lissajous_pll_status;
}

uint8_t LissajousPLL_RawSnapshotReady(void)
{
  return pll.raw_snapshot_ready;
}

const uint32_t *LissajousPLL_GetRawReference(uint32_t *count)
{
  if ((count == NULL) || (pll.raw_snapshot_ready == 0U))
  {
    if (count != NULL)
    {
      *count = 0U;
    }
    return NULL;
  }

  *count = pll.raw_ref_snapshot_count;
  return pll.raw_ref_snapshot;
}

const uint32_t *LissajousPLL_GetRawFeedback(uint32_t *count)
{
  if ((count == NULL) || (pll.raw_snapshot_ready == 0U))
  {
    if (count != NULL)
    {
      *count = 0U;
    }
    return NULL;
  }

  *count = pll.raw_fb_snapshot_count;
  return pll.raw_fb_snapshot;
}

void LissajousPLL_Task(void)
{
  uint32_t now;
  uint32_t capture_elapsed;

  if (!pll.initialized)
  {
    return;
  }

  if (((M == 1U) || (M == 2U)) &&
      (M != g_lissajous_pll_status.multiplier))
  {
    ApplyFrequencyMultiplier(M);
  }

  now = HAL_GetTick();
  if ((PLL_FAST_DITHER_ENABLE != 0U) &&
      (pll.mode == LISSAJOUS_PLL_MODE_FREQUENCY_FOLLOW) &&
      ((int32_t)(now - pll.next_dither_ms) >= 0))
  {
    pll.next_dither_ms = now + PLL_DITHER_INTERVAL_MS;
    (void)ApplyFrequencyDither();
  }

  if (g_lissajous_pll_status.capture_running)
  {
    capture_elapsed = now - pll.capture_started_ms;
    if (capture_elapsed >= LISSAJOUS_PLL_CAPTURE_WINDOW_MS)
    {
      FreezeAndSnapshotCapture(now);
      StopCapture();
      if (capture_elapsed >= PLL_CAPTURE_TIMEOUT_MS)
      {
        g_lissajous_pll_status.data_valid = 0U;
        ++g_lissajous_pll_status.capture_timeouts;
        ResetLockState();
      }
      else
      {
        ProcessCaptureBatch();
      }
      pll.next_capture_ms = now + PLL_RESTART_DELAY_MS;
    }
    else if ((pll.ref_ready != 0U) || (pll.fb_ready != 0U))
    {
      /* A DMA buffer filled before 100 ms: frequency/noise is out of range. */
      FreezeAndSnapshotCapture(now);
      StopCapture();
      g_lissajous_pll_status.data_valid = 0U;
      ++g_lissajous_pll_status.capture_timeouts;
      ResetLockState();
      pll.next_capture_ms = now + PLL_RESTART_DELAY_MS;
    }
  }
  else if ((int32_t)(now - pll.next_capture_ms) >= 0)
  {
    if (StartCapture() != HAL_OK)
    {
      StopCapture();
      g_lissajous_pll_status.data_valid = 0U;
      ++g_lissajous_pll_status.dma_start_errors;
      pll.next_capture_ms = now + PLL_RESTART_DELAY_MS;
    }
  }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim != pll.htim)
  {
    return;
  }

  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
  {
    pll.ref_ready = 1U;
  }
  else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
  {
    pll.fb_ready = 1U;
  }
}
