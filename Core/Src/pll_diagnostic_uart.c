#include "pll_diagnostic_uart.h"

#define DIAG_UART_BAUD_RATE       115200UL
#define DIAG_UART_PCLK_HZ         84000000UL
#define DIAG_PRINT_INTERVAL_MS    250UL

static uint32_t last_print_ms;
static uint8_t raw_dump_printed;

static char *AppendText(char *destination, const char *text)
{
  while (*text != '\0')
  {
    *destination++ = *text++;
  }
  return destination;
}

static char *AppendU32(char *destination, uint32_t value)
{
  char digits[10];
  uint32_t count = 0U;

  do
  {
    digits[count++] = (char)('0' + (value % 10U));
    value /= 10U;
  } while ((value != 0U) && (count < sizeof(digits)));

  while (count > 0U)
  {
    *destination++ = digits[--count];
  }
  return destination;
}


static char *AppendFrequencyMilliHz(char *destination, uint32_t value)
{
  uint32_t remainder;

  destination = AppendU32(destination, value / 1000U);
  *destination++ = '.';
  remainder = value % 1000U;
  *destination++ = (char)('0' + (remainder / 100U));
  *destination++ = (char)('0' + ((remainder / 10U) % 10U));
  *destination++ = (char)('0' + (remainder % 10U));

  return destination;
}
static char *AppendSignedFrequencyMilliHz(char *destination, int32_t value)
{
  uint32_t magnitude;

  if (value < 0)
  {
    *destination++ = '-';
    magnitude = (uint32_t)(-(int64_t)value);
  }
  else
  {
    *destination++ = '+';
    magnitude = (uint32_t)value;
  }
  return AppendFrequencyMilliHz(destination, magnitude);
}


static void WriteBuffer(const char *data, uint32_t length)
{
  uint32_t index;

  for (index = 0U; index < length; ++index)
  {
    while ((USART1->SR & USART_SR_TXE) == 0U)
    {
    }
    USART1->DR = (uint8_t)data[index];
  }

  while ((USART1->SR & USART_SR_TC) == 0U)
  {
  }
}

static void WriteText(const char *text)
{
  const char *end = text;

  while (*end != '\0')
  {
    ++end;
  }
  WriteBuffer(text, (uint32_t)(end - text));
}

static void WriteRawChannel(const char *name,
                            const uint32_t *samples,
                            uint32_t count)
{
  uint32_t index;
  uint32_t delta;
  char line[96];
  char *cursor;

  if ((name == NULL) || (samples == NULL))
  {
    return;
  }

  for (index = 0U; index < count; ++index)
  {
    delta = (index == 0U) ? 0U : (samples[index] - samples[index - 1U]);
    cursor = line;
    cursor = AppendText(cursor, "RAW ");
    cursor = AppendText(cursor, name);
    cursor = AppendText(cursor, " i=");
    cursor = AppendU32(cursor, index);
    cursor = AppendText(cursor, " CCR=");
    cursor = AppendU32(cursor, samples[index]);
    cursor = AppendText(cursor, " D=");
    cursor = AppendU32(cursor, delta);
    cursor = AppendText(cursor, "\r\n");
    WriteBuffer(line, (uint32_t)(cursor - line));
  }
}

static void WriteRawSnapshot(void)
{
  const uint32_t *reference;
  const uint32_t *feedback;
  uint32_t reference_count;
  uint32_t feedback_count;
  uint32_t input_filter;
  uint32_t feedback_filter;
  char line[96];
  char *cursor;

  reference = LissajousPLL_GetRawReference(&reference_count);
  feedback = LissajousPLL_GetRawFeedback(&feedback_count);
  if ((reference == NULL) || (feedback == NULL))
  {
    return;
  }

  input_filter = (TIM5->CCMR1 & TIM_CCMR1_IC2F) >>
                 TIM_CCMR1_IC2F_Pos;
  feedback_filter = (TIM5->CCMR2 & TIM_CCMR2_IC3F) >>
                    TIM_CCMR2_IC3F_Pos;
  cursor = line;
  cursor = AppendText(cursor, "\r\nRAW SNAPSHOT IC2F=");
  cursor = AppendU32(cursor, input_filter);
  cursor = AppendText(cursor, " IC3F=");
  cursor = AppendU32(cursor, feedback_filter);
  cursor = AppendText(cursor, " D=CCR[i]-CCR[i-1]\r\n");
  WriteBuffer(line, (uint32_t)(cursor - line));

  WriteRawChannel("IN", reference, reference_count);
  WriteRawChannel("FB", feedback, feedback_count);
  WriteText("RAW SNAPSHOT END\r\n\r\n");
}

void PLL_DiagnosticUart_Init(void)
{
  uint32_t baud_divider;

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_USART1_CLK_ENABLE();

  GPIOA->MODER = (GPIOA->MODER & ~(3UL << (9U * 2U))) |
                 (2UL << (9U * 2U));
  GPIOA->OTYPER &= ~GPIO_PIN_9;
  GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~(3UL << (9U * 2U))) |
                   (2UL << (9U * 2U));
  GPIOA->PUPDR = (GPIOA->PUPDR & ~(3UL << (9U * 2U))) |
                 (1UL << (9U * 2U));
  GPIOA->AFR[1] = (GPIOA->AFR[1] & ~(0xFUL << 4U)) |
                  (7UL << 4U);

  USART1->CR1 = 0U;
  USART1->CR2 = 0U;
  USART1->CR3 = 0U;
  baud_divider = (DIAG_UART_PCLK_HZ + DIAG_UART_BAUD_RATE / 2U) /
                 DIAG_UART_BAUD_RATE;
  USART1->BRR = baud_divider;
  USART1->CR1 = USART_CR1_TE | USART_CR1_UE;

  last_print_ms = HAL_GetTick();
  raw_dump_printed = 0U;
  WriteText("\r\nPLL xM normal: M=2 default, 100ms window, dynamic blanking\r\n");
}

void PLL_DiagnosticUart_Task(void)
{
  const volatile LissajousPLL_Status *status;
  uint32_t now;
  char line[300];
  char detail[720];
  char *cursor;

  now = HAL_GetTick();
  if ((raw_dump_printed == 0U) &&
      (LissajousPLL_RawSnapshotReady() != 0U))
  {
    WriteRawSnapshot();
    raw_dump_printed = 1U;
    last_print_ms = now;
    return;
  }

  if ((uint32_t)(now - last_print_ms) < DIAG_PRINT_INTERVAL_MS)
  {
    return;
  }
  last_print_ms = now;

  status = LissajousPLL_GetStatus();
  cursor = line;
  cursor = AppendText(cursor, "IN=");
  cursor = AppendFrequencyMilliHz(cursor, status->input_frequency_millihz);
  cursor = AppendText(cursor, "Hz M=");
  cursor = AppendU32(cursor, status->multiplier);
  cursor = AppendText(cursor, " TGT=");
  cursor = AppendFrequencyMilliHz(
      cursor, status->target_frequency_millihz);
  cursor = AppendText(cursor, "Hz CMD=");
  cursor = AppendU32(cursor, status->command_frequency_hz);
  cursor = AppendText(cursor, "Hz ERR=");
  cursor = AppendSignedFrequencyMilliHz(cursor, status->frequency_error_millihz);
  cursor = AppendText(cursor, "Hz FB=");
  cursor = AppendFrequencyMilliHz(cursor, status->output_frequency_millihz);
  cursor = AppendText(cursor, "Hz MCLK_EST=");
  cursor = AppendU32(cursor, status->estimated_mclk_hz);
  cursor = AppendText(cursor, "Hz FTW=");
  cursor = AppendU32(cursor, status->frequency_word);
  cursor = AppendText(cursor, " LOCK=");
  cursor = AppendU32(cursor, status->locked);
  cursor = AppendText(cursor, " VALID=");
  cursor = AppendU32(cursor, status->data_valid);
  cursor = AppendText(cursor, " AGE=");
  cursor = AppendU32(cursor, now - status->last_batch_tick_ms);
  cursor = AppendText(cursor, "ms BATCH=");
  cursor = AppendU32(cursor, status->completed_batches);
  cursor = AppendText(cursor, " TIMEOUT=");
  cursor = AppendU32(cursor, status->capture_timeouts);
  cursor = AppendText(cursor, " REJECT=");
  cursor = AppendU32(cursor, status->rejected_batches);
  cursor = AppendText(cursor, " DMAERR=");
  cursor = AppendU32(cursor, status->dma_start_errors);
  cursor = AppendText(cursor, " DDSERR=");
  cursor = AppendU32(cursor, status->dds_errors);
  cursor = AppendText(cursor, "\r\n");

  WriteBuffer(line, (uint32_t)(cursor - line));

  cursor = detail;
  cursor = AppendText(cursor, "CLK: PCLK1=");
  cursor = AppendU32(cursor, status->apb1_clock_hz);
  cursor = AppendText(cursor, "Hz TIM5_CNT=");
  cursor = AppendU32(cursor, status->timer_counter_hz);
  cursor = AppendText(cursor, "Hz CFG=");
  cursor = AppendU32(cursor, LISSAJOUS_PLL_TIMER_CLOCK_HZ);
  cursor = AppendText(cursor, "Hz PSC=");
  cursor = AppendU32(cursor, status->timer_prescaler);
  cursor = AppendText(cursor, " N_IN=");
  cursor = AppendU32(cursor, status->input_measurement_periods);
  cursor = AppendText(cursor, " N_FB=");
  cursor = AppendU32(cursor, status->feedback_measurement_periods);
  cursor = AppendText(cursor, " DT=");
  cursor = AppendU32(cursor, status->capture_elapsed_ms);
  cursor = AppendText(cursor, "ms ICP=/");
  cursor = AppendU32(cursor, status->capture_edge_divisor);
  cursor = AppendText(cursor, "\r\nCAP: IN_EDGE=");
  cursor = AppendU32(cursor, status->input_captured_edges);
  cursor = AppendText(cursor, " FB_EDGE=");
  cursor = AppendU32(cursor, status->feedback_captured_edges);
  cursor = AppendText(cursor, " IN_SPAN=");
  cursor = AppendU32(cursor, status->input_span_ticks);
  cursor = AppendText(cursor, " IN_TAVG=");
  cursor = AppendU32(cursor, status->input_period_ticks);
  cursor = AppendText(cursor, " FB_SPAN=");
  cursor = AppendU32(cursor, status->output_span_ticks);
  cursor = AppendText(cursor, " FB_TAVG=");
  cursor = AppendU32(cursor, status->output_period_ticks);
  cursor = AppendText(cursor, "\r\nDITH: GUARD=");
  cursor = AppendU32(cursor, status->dither_capture_guard_us);
  cursor = AppendText(cursor, "us FSEL=");
  cursor = AppendU32(cursor, status->dither_switches);
  cursor = AppendText(cursor, " CTRL=");
  cursor = AppendU32(cursor, status->dither_control_writes);
  cursor = AppendText(cursor, " RAWFB=");
  cursor = AppendFrequencyMilliHz(
      cursor, status->raw_output_frequency_millihz);
  cursor = AppendText(cursor, "Hz BLANK=");
  cursor = AppendU32(cursor, status->dither_blank_windows);
  cursor = AppendText(cursor, " KEEP=");
  cursor = AppendU32(cursor, status->feedback_kept_edges);
  cursor = AppendText(cursor, " DROP=");
  cursor = AppendU32(cursor, status->feedback_dropped_edges);
  cursor = AppendText(cursor, " CYC=");
  cursor = AppendU32(cursor, status->feedback_reconstructed_cycles);
  cursor = AppendText(cursor, " THR=");
  cursor = AppendU32(
      cursor, status->feedback_blanking_threshold_ticks);
  cursor = AppendText(cursor, "t");
  cursor = AppendText(cursor, "\r\nPHASE: VALID=");
  cursor = AppendU32(cursor, status->phase_measurement_valid);
  cursor = AppendText(cursor, " MEAS=");
  cursor = AppendFrequencyMilliHz(
      cursor, (uint32_t)status->measured_phase_millidegrees);
  cursor = AppendText(cursor, "deg TARGET=");
  cursor = AppendFrequencyMilliHz(
      cursor, status->target_phase_millidegrees);
  cursor = AppendText(cursor, "deg ERR=");
  cursor = AppendSignedFrequencyMilliHz(
      cursor, status->phase_tracking_error_millidegrees);
  cursor = AppendText(cursor, "deg PTICK=");
  cursor = AppendU32(cursor, status->measured_phase_ticks);
  cursor = AppendText(cursor, " FIT=");
  cursor = AppendU32(cursor, status->phase_fit_used);
  cursor = AppendText(cursor, " POINTS=");
  cursor = AppendU32(cursor, status->phase_fit_points);
  cursor = AppendText(cursor, "\r\nPLOOP: EN=");
  cursor = AppendU32(cursor, status->phase_loop_enabled);
  cursor = AppendText(cursor, " ACTIVE=");
  cursor = AppendU32(cursor, status->phase_loop_active);
  cursor = AppendText(cursor, " ESC=");
  cursor = AppendU32(cursor, status->phase_escape_active);
  cursor = AppendText(cursor, " ESIGN=");
  if (status->phase_escape_sign < 0)
  {
    cursor = AppendText(cursor, "-1");
  }
  else if (status->phase_escape_sign > 0)
  {
    cursor = AppendText(cursor, "+1");
  }
  else
  {
    cursor = AppendText(cursor, "0");
  }
  cursor = AppendText(cursor, " ECNT=");
  cursor = AppendU32(cursor, status->phase_escape_entry_count);
  cursor = AppendText(cursor, "/");
  cursor = AppendU32(cursor, status->phase_escape_exit_count);
  cursor = AppendText(cursor, " PLIM=");
  cursor = AppendFrequencyMilliHz(
      cursor, (uint32_t)status->phase_normal_limit_millihz);
  cursor = AppendText(cursor, "Hz EOFF=");
  cursor = AppendFrequencyMilliHz(
      cursor, (uint32_t)status->phase_escape_offset_limit_millihz);
  cursor = AppendText(cursor, "Hz");
  cursor = AppendText(cursor, " NERR=");
  cursor = AppendSignedFrequencyMilliHz(
      cursor, status->nominal_frequency_error_millihz);
  cursor = AppendText(cursor, "Hz FNERR=");
  cursor = AppendSignedFrequencyMilliHz(
      cursor, status->filtered_nominal_frequency_error_millihz);
  cursor = AppendText(cursor, "Hz IERR=");
  cursor = AppendSignedFrequencyMilliHz(
      cursor, status->integrator_frequency_error_millihz);
  cursor = AppendText(cursor, "Hz FREJ=");
  cursor = AppendU32(cursor, status->frequency_spike_rejected);
  cursor = AppendText(cursor, " FDELTA=");
  cursor = AppendSignedFrequencyMilliHz(
      cursor, status->frequency_error_delta_millihz);
  cursor = AppendText(cursor, "Hz FRTOT=");
  cursor = AppendU32(cursor, status->frequency_spike_rejections);
  cursor = AppendText(cursor, " POFF=");
  cursor = AppendSignedFrequencyMilliHz(
      cursor, status->phase_frequency_offset_millihz);
  cursor = AppendText(cursor, "Hz CTGT=");
  cursor = AppendFrequencyMilliHz(
      cursor, status->control_target_frequency_millihz);
  cursor = AppendText(cursor, "Hz\r\n");

  WriteBuffer(detail, (uint32_t)(cursor - detail));
}
