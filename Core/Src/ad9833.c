#include "ad9833.h"

#define AD9833_CONTROL_B28       0x2000U
#define AD9833_CONTROL_FSELECT   0x0800U
#define AD9833_CONTROL_RESET     0x0100U
#define AD9833_FREQ0_WRITE       0x4000U
#define AD9833_FREQ1_WRITE       0x8000U
#define AD9833_PHASE0_WRITE      0xC000U
#define AD9833_SPI_TIMEOUT_MS    10U

static HAL_StatusTypeDef AD9833_WriteWord(AD9833_HandleTypeDef *device,
                                         uint16_t word)
{
    uint8_t bytes[2];
    HAL_StatusTypeDef status;

    if ((device == NULL) || (device->hspi == NULL))
    {
        return HAL_ERROR;
    }

    bytes[0] = (uint8_t)(word >> 8);
    bytes[1] = (uint8_t)word;

    HAL_GPIO_WritePin(DDS_FSYNC_GPIO_Port, DDS_FSYNC_Pin, GPIO_PIN_RESET);
    status = HAL_SPI_Transmit(device->hspi,
                             bytes,
                             (uint16_t)sizeof(bytes),
                             AD9833_SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(DDS_FSYNC_GPIO_Port, DDS_FSYNC_Pin, GPIO_PIN_SET);

    device->last_status = status;
    return status;
}

static HAL_StatusTypeDef AD9833_WriteControlWord(
    AD9833_HandleTypeDef *device,
    uint16_t control_word)
{
    HAL_StatusTypeDef status;

    status = AD9833_WriteWord(device, control_word);
    if (status == HAL_OK)
    {
        device->control_word = control_word;
        device->selected_frequency_register =
            ((control_word & AD9833_CONTROL_FSELECT) != 0U) ?
            AD9833_FREQ_REGISTER_1 : AD9833_FREQ_REGISTER_0;
        device->frequency_word =
            device->frequency_words[device->selected_frequency_register];
    }
    return status;
}

uint32_t AD9833_FrequencyHzToWord(uint32_t frequency_hz, uint32_t mclk_hz)
{
    uint64_t numerator;

    if ((frequency_hz == 0U) || (mclk_hz == 0U))
    {
        return 0U;
    }

    numerator = ((uint64_t)frequency_hz << 28) + ((uint64_t)mclk_hz / 2U);
    return (uint32_t)(numerator / mclk_hz) & AD9833_FTW_MASK;
}

HAL_StatusTypeDef AD9833_WriteFrequencyRegister(
    AD9833_HandleTypeDef *device,
    uint8_t register_index,
    uint32_t frequency_word)
{
    HAL_StatusTypeDef status;
    uint32_t word;
    uint16_t address;

    if ((device == NULL) || (register_index > AD9833_FREQ_REGISTER_1))
    {
        return HAL_ERROR;
    }

    word = frequency_word & AD9833_FTW_MASK;
    address = (register_index == AD9833_FREQ_REGISTER_0) ?
              AD9833_FREQ0_WRITE : AD9833_FREQ1_WRITE;

    status = AD9833_WriteWord(
        device, (uint16_t)(address | (word & 0x3FFFU)));
    if (status != HAL_OK)
    {
        return status;
    }

    status = AD9833_WriteWord(
        device, (uint16_t)(address | ((word >> 14) & 0x3FFFU)));
    if (status == HAL_OK)
    {
        device->frequency_words[register_index] = word;
        if (register_index == device->selected_frequency_register)
        {
            device->frequency_word = word;
        }
    }
    return status;
}

HAL_StatusTypeDef AD9833_SelectFrequencyRegister(
    AD9833_HandleTypeDef *device,
    uint8_t register_index)
{
    uint16_t control_word;

    if ((device == NULL) || (register_index > AD9833_FREQ_REGISTER_1))
    {
        return HAL_ERROR;
    }
    if (register_index == device->selected_frequency_register)
    {
        return HAL_OK;
    }

    control_word = device->control_word;
    if (register_index == AD9833_FREQ_REGISTER_1)
    {
        control_word |= AD9833_CONTROL_FSELECT;
    }
    else
    {
        control_word &= (uint16_t)~AD9833_CONTROL_FSELECT;
    }
    return AD9833_WriteControlWord(device, control_word);
}

HAL_StatusTypeDef AD9833_RewriteControlWord(
    AD9833_HandleTypeDef *device)
{
    if (device == NULL)
    {
        return HAL_ERROR;
    }

    return AD9833_WriteControlWord(device, device->control_word);
}
HAL_StatusTypeDef AD9833_SetFrequencyWord(AD9833_HandleTypeDef *device,
                                         uint32_t frequency_word)
{
    if (device == NULL)
    {
        return HAL_ERROR;
    }

    return AD9833_WriteFrequencyRegister(
        device, device->selected_frequency_register, frequency_word);
}

HAL_StatusTypeDef AD9833_ReprogramFrequencyWord(
    AD9833_HandleTypeDef *device,
    uint32_t frequency_word)
{
    HAL_StatusTypeDef status;
    uint32_t word;
    uint16_t phase_word;

    if (device == NULL)
    {
        return HAL_ERROR;
    }

    word = frequency_word & AD9833_FTW_MASK;
    phase_word = device->phase_word & 0x0FFFU;

    status = AD9833_WriteControlWord(
        device, (uint16_t)(AD9833_CONTROL_B28 | AD9833_CONTROL_RESET));
    if (status != HAL_OK)
    {
        return status;
    }

    status = AD9833_WriteFrequencyRegister(
        device, AD9833_FREQ_REGISTER_0, word);
    if (status != HAL_OK)
    {
        return status;
    }

    status = AD9833_WriteWord(
        device, (uint16_t)(AD9833_PHASE0_WRITE | phase_word));
    if (status != HAL_OK)
    {
        return status;
    }

    return AD9833_WriteControlWord(device, AD9833_CONTROL_B28);
}

HAL_StatusTypeDef AD9833_SetFrequencyHz(AD9833_HandleTypeDef *device,
                                       uint32_t frequency_hz)
{
    if (device == NULL)
    {
        return HAL_ERROR;
    }

    return AD9833_SetFrequencyWord(
        device,
        AD9833_FrequencyHzToWord(frequency_hz, device->mclk_hz));
}

HAL_StatusTypeDef AD9833_SetPhaseWord(AD9833_HandleTypeDef *device,
                                     uint16_t phase_word)
{
    HAL_StatusTypeDef status;
    uint16_t word;

    if (device == NULL)
    {
        return HAL_ERROR;
    }

    word = phase_word & 0x0FFFU;
    status = AD9833_WriteWord(device, (uint16_t)(AD9833_PHASE0_WRITE | word));
    if (status == HAL_OK)
    {
        device->phase_word = word;
    }

    return status;
}

HAL_StatusTypeDef AD9833_Init(AD9833_HandleTypeDef *device,
                              SPI_HandleTypeDef *hspi,
                              uint32_t mclk_hz,
                              uint32_t initial_frequency_hz)
{
    HAL_StatusTypeDef status;

    if ((device == NULL) || (hspi == NULL) || (mclk_hz == 0U))
    {
        return HAL_ERROR;
    }

    device->hspi = hspi;
    device->mclk_hz = mclk_hz;
    device->frequency_word = 0U;
    device->frequency_words[0] = 0U;
    device->frequency_words[1] = 0U;
    device->phase_word = 0U;
    device->control_word =
        (uint16_t)(AD9833_CONTROL_B28 | AD9833_CONTROL_RESET);
    device->selected_frequency_register = AD9833_FREQ_REGISTER_0;
    device->last_status = HAL_OK;

    HAL_GPIO_WritePin(DDS_FSYNC_GPIO_Port, DDS_FSYNC_Pin, GPIO_PIN_SET);

    status = AD9833_WriteControlWord(
        device, (uint16_t)(AD9833_CONTROL_B28 | AD9833_CONTROL_RESET));
    if (status != HAL_OK)
    {
        return status;
    }

    status = AD9833_SetFrequencyHz(device, initial_frequency_hz);
    if (status != HAL_OK)
    {
        return status;
    }

    status = AD9833_SetPhaseWord(device, 0U);
    if (status != HAL_OK)
    {
        return status;
    }

    return AD9833_WriteControlWord(device, AD9833_CONTROL_B28);
}
