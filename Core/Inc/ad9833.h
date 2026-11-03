#ifndef AD9833_H
#define AD9833_H

#include "main.h"

#define AD9833_DEFAULT_MCLK_HZ 25000000UL
#define AD9833_PHASE_MODULUS    4096U
#define AD9833_FTW_MASK         0x0FFFFFFFUL
#define AD9833_FREQ_REGISTER_0  0U
#define AD9833_FREQ_REGISTER_1  1U

typedef struct
{
    SPI_HandleTypeDef *hspi;
    uint32_t mclk_hz;
    uint32_t frequency_word;
    uint32_t frequency_words[2];
    uint16_t phase_word;
    uint16_t control_word;
    uint8_t selected_frequency_register;
    HAL_StatusTypeDef last_status;
} AD9833_HandleTypeDef;

HAL_StatusTypeDef AD9833_Init(AD9833_HandleTypeDef *device,
                              SPI_HandleTypeDef *hspi,
                              uint32_t mclk_hz,
                              uint32_t initial_frequency_hz);
HAL_StatusTypeDef AD9833_SetFrequencyWord(AD9833_HandleTypeDef *device,
                                         uint32_t frequency_word);
HAL_StatusTypeDef AD9833_WriteFrequencyRegister(
    AD9833_HandleTypeDef *device,
    uint8_t register_index,
    uint32_t frequency_word);
HAL_StatusTypeDef AD9833_SelectFrequencyRegister(
    AD9833_HandleTypeDef *device,
    uint8_t register_index);
HAL_StatusTypeDef AD9833_RewriteControlWord(
    AD9833_HandleTypeDef *device);
HAL_StatusTypeDef AD9833_ReprogramFrequencyWord(
    AD9833_HandleTypeDef *device,
    uint32_t frequency_word);
HAL_StatusTypeDef AD9833_SetFrequencyHz(AD9833_HandleTypeDef *device,
                                       uint32_t frequency_hz);
HAL_StatusTypeDef AD9833_SetPhaseWord(AD9833_HandleTypeDef *device,
                                     uint16_t phase_word);
uint32_t AD9833_FrequencyHzToWord(uint32_t frequency_hz, uint32_t mclk_hz);

#endif
