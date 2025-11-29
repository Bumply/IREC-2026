/**
 * @file pyro.h
 * @brief Pyrotechnic Channel Driver for STM32
 * @author Zenith Rocket Team
 * @date 2025
 * 
 * Controls 2 pyro channels for parachute deployment:
 * - Channel 1: Drogue chute (at apogee)
 * - Channel 2: Main chute (at target altitude)
 * 
 * Hardware: IRFU120 N-Channel MOSFETs
 * E-match ignition: Typical 1A for 50ms
 * 
 * SAFETY FEATURES:
 * - Arming switch required
 * - Continuity check before flight
 * - Configurable fire duration
 * - Hardware timeout protection
 */

#ifndef PYRO_H
#define PYRO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/*============================================================================
 * CONFIGURATION
 *============================================================================*/

/* Number of pyro channels */
#define PYRO_NUM_CHANNELS       2

/* Channel indices */
#define PYRO_CHANNEL_DROGUE     0
#define PYRO_CHANNEL_MAIN       1

/* Default fire duration (ms) */
#define PYRO_FIRE_DURATION_MS   100

/* Continuity check threshold (ADC counts) */
#define PYRO_CONTINUITY_THRESH  100

/*============================================================================
 * CHANNEL STATE
 *============================================================================*/

typedef enum {
    PYRO_STATE_SAFE = 0,        /* Not armed, cannot fire */
    PYRO_STATE_ARMED,           /* Armed, ready to fire */
    PYRO_STATE_FIRING,          /* Currently firing */
    PYRO_STATE_FIRED,           /* Has been fired */
    PYRO_STATE_ERROR            /* Error detected */
} Pyro_State_t;

/*============================================================================
 * CHANNEL STRUCTURE
 *============================================================================*/

typedef struct {
    GPIO_TypeDef *fire_port;    /* Fire GPIO port */
    uint16_t fire_pin;          /* Fire GPIO pin */
    
    GPIO_TypeDef *cont_port;    /* Continuity sense port (optional) */
    uint16_t cont_pin;          /* Continuity sense pin */
    
    ADC_HandleTypeDef *hadc;    /* ADC for continuity (optional) */
    uint32_t adc_channel;       /* ADC channel */
    
    Pyro_State_t state;         /* Current state */
    bool continuity;            /* Continuity detected */
    uint32_t fire_start_tick;   /* When firing started */
    uint16_t fire_duration_ms;  /* Fire pulse duration */
    bool has_continuity_sense;  /* Whether continuity sensing is available */
} Pyro_Channel_t;

/*============================================================================
 * HANDLE STRUCTURE
 *============================================================================*/

typedef struct {
    Pyro_Channel_t channels[PYRO_NUM_CHANNELS];
    
    /* Global arming */
    GPIO_TypeDef *arm_switch_port;  /* Arming switch input port */
    uint16_t arm_switch_pin;        /* Arming switch input pin */
    bool arm_switch_active_high;    /* Switch polarity */
    
    bool system_armed;              /* Global arm state */
    bool initialized;
    
} Pyro_Handle_t;

/*============================================================================
 * FUNCTION PROTOTYPES
 *============================================================================*/

/**
 * @brief Initialize pyro system
 * @param dev Pointer to Pyro handle
 * @return HAL_OK on success
 */
HAL_StatusTypeDef Pyro_Init(Pyro_Handle_t *dev);

/**
 * @brief Configure a pyro channel
 * @param dev Pointer to Pyro handle
 * @param channel Channel index (0 or 1)
 * @param fire_port Fire GPIO port
 * @param fire_pin Fire GPIO pin
 * @return HAL_OK on success
 */
HAL_StatusTypeDef Pyro_ConfigureChannel(Pyro_Handle_t *dev, uint8_t channel,
                                        GPIO_TypeDef *fire_port, uint16_t fire_pin);

/**
 * @brief Configure continuity sensing for a channel
 * @param dev Pointer to Pyro handle
 * @param channel Channel index
 * @param cont_port Continuity sense GPIO port
 * @param cont_pin Continuity sense GPIO pin
 * @param hadc ADC handle for analog sensing (NULL for digital)
 * @param adc_channel ADC channel number
 * @return HAL_OK on success
 */
HAL_StatusTypeDef Pyro_ConfigureContinuity(Pyro_Handle_t *dev, uint8_t channel,
                                           GPIO_TypeDef *cont_port, uint16_t cont_pin,
                                           ADC_HandleTypeDef *hadc, uint32_t adc_channel);

/**
 * @brief Configure arming switch
 * @param dev Pointer to Pyro handle
 * @param port Switch GPIO port
 * @param pin Switch GPIO pin
 * @param active_high true if switch is active high
 * @return HAL_OK on success
 */
HAL_StatusTypeDef Pyro_ConfigureArmSwitch(Pyro_Handle_t *dev,
                                          GPIO_TypeDef *port, uint16_t pin,
                                          bool active_high);

/**
 * @brief Set fire duration for a channel
 * @param dev Pointer to Pyro handle
 * @param channel Channel index
 * @param duration_ms Fire duration in milliseconds
 */
void Pyro_SetFireDuration(Pyro_Handle_t *dev, uint8_t channel, uint16_t duration_ms);

/**
 * @brief Check if arming switch is enabled
 * @param dev Pointer to Pyro handle
 * @return true if arm switch is active
 */
bool Pyro_IsArmSwitchActive(Pyro_Handle_t *dev);

/**
 * @brief Arm the pyro system (requires arm switch active)
 * @param dev Pointer to Pyro handle
 * @return HAL_OK if armed successfully
 */
HAL_StatusTypeDef Pyro_Arm(Pyro_Handle_t *dev);

/**
 * @brief Disarm the pyro system
 * @param dev Pointer to Pyro handle
 */
void Pyro_Disarm(Pyro_Handle_t *dev);

/**
 * @brief Check if system is armed
 * @param dev Pointer to Pyro handle
 * @return true if armed
 */
bool Pyro_IsArmed(Pyro_Handle_t *dev);

/**
 * @brief Check continuity on a channel
 * @param dev Pointer to Pyro handle
 * @param channel Channel index
 * @return true if continuity detected (e-match connected)
 */
bool Pyro_CheckContinuity(Pyro_Handle_t *dev, uint8_t channel);

/**
 * @brief Check continuity on all channels
 * @param dev Pointer to Pyro handle
 * @return Bitmask of channels with continuity
 */
uint8_t Pyro_CheckAllContinuity(Pyro_Handle_t *dev);

/**
 * @brief Fire a pyro channel
 * @param dev Pointer to Pyro handle
 * @param channel Channel index
 * @return HAL_OK if fire command accepted
 */
HAL_StatusTypeDef Pyro_Fire(Pyro_Handle_t *dev, uint8_t channel);

/**
 * @brief Update pyro state (call regularly to handle fire timing)
 * @param dev Pointer to Pyro handle
 */
void Pyro_Update(Pyro_Handle_t *dev);

/**
 * @brief Get channel state
 * @param dev Pointer to Pyro handle
 * @param channel Channel index
 * @return Channel state
 */
Pyro_State_t Pyro_GetState(Pyro_Handle_t *dev, uint8_t channel);

/**
 * @brief Check if channel has fired
 * @param dev Pointer to Pyro handle
 * @param channel Channel index
 * @return true if channel has fired
 */
bool Pyro_HasFired(Pyro_Handle_t *dev, uint8_t channel);

/**
 * @brief Emergency stop - disable all channels immediately
 * @param dev Pointer to Pyro handle
 */
void Pyro_EmergencyStop(Pyro_Handle_t *dev);

/**
 * @brief Reset channel state (for testing)
 * @param dev Pointer to Pyro handle
 * @param channel Channel index
 */
void Pyro_ResetChannel(Pyro_Handle_t *dev, uint8_t channel);

#ifdef __cplusplus
}
#endif

#endif /* PYRO_H */
