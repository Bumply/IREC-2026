/**
 * @file pyro.c
 * @brief Pyrotechnic Channel Driver Implementation
 * @author Zenith Rocket Team
 * @date 2025
 */

#include "pyro.h"
#include <string.h>

/*============================================================================
 * PRIVATE FUNCTIONS
 *============================================================================*/

static void Pyro_SetOutput(Pyro_Channel_t *ch, bool state)
{
    HAL_GPIO_WritePin(ch->fire_port, ch->fire_pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/*============================================================================
 * PUBLIC FUNCTIONS
 *============================================================================*/

HAL_StatusTypeDef Pyro_Init(Pyro_Handle_t *dev)
{
    if (dev == NULL) {
        return HAL_ERROR;
    }
    
    /* Clear handle */
    memset(dev, 0, sizeof(Pyro_Handle_t));
    
    /* Initialize all channels to safe state */
    for (int i = 0; i < PYRO_NUM_CHANNELS; i++) {
        dev->channels[i].state = PYRO_STATE_SAFE;
        dev->channels[i].continuity = false;
        dev->channels[i].fire_duration_ms = PYRO_FIRE_DURATION_MS;
        dev->channels[i].has_continuity_sense = false;
    }
    
    dev->system_armed = false;
    dev->initialized = true;
    
    return HAL_OK;
}

HAL_StatusTypeDef Pyro_ConfigureChannel(Pyro_Handle_t *dev, uint8_t channel,
                                        GPIO_TypeDef *fire_port, uint16_t fire_pin)
{
    if (dev == NULL || channel >= PYRO_NUM_CHANNELS) {
        return HAL_ERROR;
    }
    
    dev->channels[channel].fire_port = fire_port;
    dev->channels[channel].fire_pin = fire_pin;
    
    /* Ensure output is OFF */
    Pyro_SetOutput(&dev->channels[channel], false);
    
    return HAL_OK;
}

HAL_StatusTypeDef Pyro_ConfigureContinuity(Pyro_Handle_t *dev, uint8_t channel,
                                           GPIO_TypeDef *cont_port, uint16_t cont_pin,
                                           ADC_HandleTypeDef *hadc, uint32_t adc_channel)
{
    if (dev == NULL || channel >= PYRO_NUM_CHANNELS) {
        return HAL_ERROR;
    }
    
    dev->channels[channel].cont_port = cont_port;
    dev->channels[channel].cont_pin = cont_pin;
    dev->channels[channel].hadc = hadc;
    dev->channels[channel].adc_channel = adc_channel;
    dev->channels[channel].has_continuity_sense = true;
    
    return HAL_OK;
}

HAL_StatusTypeDef Pyro_ConfigureArmSwitch(Pyro_Handle_t *dev,
                                          GPIO_TypeDef *port, uint16_t pin,
                                          bool active_high)
{
    if (dev == NULL) {
        return HAL_ERROR;
    }
    
    dev->arm_switch_port = port;
    dev->arm_switch_pin = pin;
    dev->arm_switch_active_high = active_high;
    
    return HAL_OK;
}

void Pyro_SetFireDuration(Pyro_Handle_t *dev, uint8_t channel, uint16_t duration_ms)
{
    if (dev == NULL || channel >= PYRO_NUM_CHANNELS) {
        return;
    }
    
    dev->channels[channel].fire_duration_ms = duration_ms;
}

bool Pyro_IsArmSwitchActive(Pyro_Handle_t *dev)
{
    if (dev == NULL || dev->arm_switch_port == NULL) {
        return false;
    }
    
    GPIO_PinState state = HAL_GPIO_ReadPin(dev->arm_switch_port, dev->arm_switch_pin);
    
    if (dev->arm_switch_active_high) {
        return state == GPIO_PIN_SET;
    } else {
        return state == GPIO_PIN_RESET;
    }
}

HAL_StatusTypeDef Pyro_Arm(Pyro_Handle_t *dev)
{
    if (dev == NULL || !dev->initialized) {
        return HAL_ERROR;
    }
    
    /* Check arm switch if configured */
    if (dev->arm_switch_port != NULL) {
        if (!Pyro_IsArmSwitchActive(dev)) {
            return HAL_ERROR;  /* Arm switch not active */
        }
    }
    
    /* Arm all channels that haven't fired */
    for (int i = 0; i < PYRO_NUM_CHANNELS; i++) {
        if (dev->channels[i].state == PYRO_STATE_SAFE) {
            dev->channels[i].state = PYRO_STATE_ARMED;
        }
    }
    
    dev->system_armed = true;
    
    return HAL_OK;
}

void Pyro_Disarm(Pyro_Handle_t *dev)
{
    if (dev == NULL) {
        return;
    }
    
    /* Stop any firing channels */
    for (int i = 0; i < PYRO_NUM_CHANNELS; i++) {
        Pyro_SetOutput(&dev->channels[i], false);
        
        if (dev->channels[i].state == PYRO_STATE_ARMED ||
            dev->channels[i].state == PYRO_STATE_FIRING) {
            dev->channels[i].state = PYRO_STATE_SAFE;
        }
    }
    
    dev->system_armed = false;
}

bool Pyro_IsArmed(Pyro_Handle_t *dev)
{
    if (dev == NULL) {
        return false;
    }
    return dev->system_armed;
}

bool Pyro_CheckContinuity(Pyro_Handle_t *dev, uint8_t channel)
{
    if (dev == NULL || channel >= PYRO_NUM_CHANNELS) {
        return false;
    }
    
    Pyro_Channel_t *ch = &dev->channels[channel];
    
    if (!ch->has_continuity_sense) {
        return false;  /* No continuity sensing configured */
    }
    
    if (ch->hadc != NULL) {
        /* Analog continuity check (ADC) */
        ADC_ChannelConfTypeDef config = {0};
        config.Channel = ch->adc_channel;
        config.Rank = 1;
        config.SamplingTime = ADC_SAMPLETIME_56CYCLES;
        
        HAL_ADC_ConfigChannel(ch->hadc, &config);
        HAL_ADC_Start(ch->hadc);
        
        if (HAL_ADC_PollForConversion(ch->hadc, 10) == HAL_OK) {
            uint32_t adc_value = HAL_ADC_GetValue(ch->hadc);
            ch->continuity = (adc_value > PYRO_CONTINUITY_THRESH);
        }
        
        HAL_ADC_Stop(ch->hadc);
    } else {
        /* Digital continuity check (GPIO) */
        GPIO_PinState state = HAL_GPIO_ReadPin(ch->cont_port, ch->cont_pin);
        ch->continuity = (state == GPIO_PIN_SET);
    }
    
    return ch->continuity;
}

uint8_t Pyro_CheckAllContinuity(Pyro_Handle_t *dev)
{
    uint8_t mask = 0;
    
    if (dev == NULL) {
        return 0;
    }
    
    for (int i = 0; i < PYRO_NUM_CHANNELS; i++) {
        if (Pyro_CheckContinuity(dev, i)) {
            mask |= (1 << i);
        }
    }
    
    return mask;
}

HAL_StatusTypeDef Pyro_Fire(Pyro_Handle_t *dev, uint8_t channel)
{
    if (dev == NULL || channel >= PYRO_NUM_CHANNELS) {
        return HAL_ERROR;
    }
    
    Pyro_Channel_t *ch = &dev->channels[channel];
    
    /* Must be armed to fire */
    if (ch->state != PYRO_STATE_ARMED) {
        return HAL_ERROR;
    }
    
    /* Check global arm */
    if (!dev->system_armed) {
        return HAL_ERROR;
    }
    
    /* Fire! */
    ch->state = PYRO_STATE_FIRING;
    ch->fire_start_tick = HAL_GetTick();
    Pyro_SetOutput(ch, true);
    
    return HAL_OK;
}

void Pyro_Update(Pyro_Handle_t *dev)
{
    if (dev == NULL || !dev->initialized) {
        return;
    }
    
    uint32_t now = HAL_GetTick();
    
    for (int i = 0; i < PYRO_NUM_CHANNELS; i++) {
        Pyro_Channel_t *ch = &dev->channels[i];
        
        if (ch->state == PYRO_STATE_FIRING) {
            /* Check if fire duration has elapsed */
            if ((now - ch->fire_start_tick) >= ch->fire_duration_ms) {
                /* Stop firing */
                Pyro_SetOutput(ch, false);
                ch->state = PYRO_STATE_FIRED;
            }
        }
    }
}

Pyro_State_t Pyro_GetState(Pyro_Handle_t *dev, uint8_t channel)
{
    if (dev == NULL || channel >= PYRO_NUM_CHANNELS) {
        return PYRO_STATE_ERROR;
    }
    
    return dev->channels[channel].state;
}

bool Pyro_HasFired(Pyro_Handle_t *dev, uint8_t channel)
{
    if (dev == NULL || channel >= PYRO_NUM_CHANNELS) {
        return false;
    }
    
    return dev->channels[channel].state == PYRO_STATE_FIRED;
}

void Pyro_EmergencyStop(Pyro_Handle_t *dev)
{
    if (dev == NULL) {
        return;
    }
    
    /* Immediately disable all outputs */
    for (int i = 0; i < PYRO_NUM_CHANNELS; i++) {
        Pyro_SetOutput(&dev->channels[i], false);
        dev->channels[i].state = PYRO_STATE_SAFE;
    }
    
    dev->system_armed = false;
}

void Pyro_ResetChannel(Pyro_Handle_t *dev, uint8_t channel)
{
    if (dev == NULL || channel >= PYRO_NUM_CHANNELS) {
        return;
    }
    
    Pyro_Channel_t *ch = &dev->channels[channel];
    
    Pyro_SetOutput(ch, false);
    ch->state = PYRO_STATE_SAFE;
    ch->fire_start_tick = 0;
}
