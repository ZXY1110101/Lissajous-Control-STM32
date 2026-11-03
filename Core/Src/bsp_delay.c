#include "bsp_delay.h"

void BSP_DelayUs(uint32_t microseconds)
{
  uint32_t start_cycles;
  uint32_t wait_cycles;

  if (microseconds == 0U)
  {
    return;
  }

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  wait_cycles = (SystemCoreClock / 1000000U) * microseconds;
  start_cycles = DWT->CYCCNT;
  while ((uint32_t)(DWT->CYCCNT - start_cycles) < wait_cycles)
  {
  }
}
