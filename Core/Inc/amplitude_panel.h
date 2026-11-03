#ifndef AMPLITUDE_PANEL_H
#define AMPLITUDE_PANEL_H

#include "main.h"

typedef enum
{
    AMP_LEVEL_1 = 0U,
    AMP_LEVEL_2,
    AMP_LEVEL_3,
    AMP_LEVEL_4,
    AMP_LEVEL_NONE = 0xFFU
} AmpLevel;

void AmpPanel_Init(AmpLevel default_level);
void AmpPanel_Task(void);
void AmpPanel_SetLevel(AmpLevel level);
void AmpPanel_Disable(void);
AmpLevel AmpPanel_GetLevel(void);

#endif
