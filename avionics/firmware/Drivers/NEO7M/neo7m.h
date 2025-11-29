/**
 * @file neo7m.h
 * @brief u-blox NEO-7M GPS Module Driver for STM32
 * @author Zenith Rocket Team
 * @date 2025
 * 
 * NEO-7M is a GPS receiver that outputs NMEA sentences via UART
 * Default baud rate: 9600
 * Update rate: Up to 5 Hz
 * 
 * This driver parses NMEA sentences:
 * - GPGGA: Fix data (position, altitude, fix quality)
 * - GPRMC: Recommended minimum data (position, speed, course, date/time)
 * - GPGSA: DOP and active satellites
 * - GPGSV: Satellites in view
 */

#ifndef NEO7M_H
#define NEO7M_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/*============================================================================
 * CONFIGURATION
 *============================================================================*/

/* Maximum NMEA sentence length */
#define NEO7M_MAX_SENTENCE_LEN      128

/* RX buffer size (should hold multiple sentences) */
#define NEO7M_RX_BUFFER_SIZE        512

/* Default baud rate */
#define NEO7M_DEFAULT_BAUD          9600

/*============================================================================
 * FIX QUALITY DEFINITIONS
 *============================================================================*/

typedef enum {
    NEO7M_FIX_INVALID = 0,     /* No fix */
    NEO7M_FIX_GPS = 1,         /* GPS fix */
    NEO7M_FIX_DGPS = 2,        /* Differential GPS fix */
    NEO7M_FIX_PPS = 3,         /* PPS fix */
    NEO7M_FIX_RTK = 4,         /* Real Time Kinematic */
    NEO7M_FIX_FLOAT_RTK = 5,   /* Float RTK */
    NEO7M_FIX_ESTIMATED = 6,   /* Estimated (dead reckoning) */
    NEO7M_FIX_MANUAL = 7,      /* Manual input */
    NEO7M_FIX_SIMULATION = 8   /* Simulation mode */
} NEO7M_FixQuality_t;

typedef enum {
    NEO7M_FIX_TYPE_NONE = 1,   /* No fix */
    NEO7M_FIX_TYPE_2D = 2,     /* 2D fix */
    NEO7M_FIX_TYPE_3D = 3      /* 3D fix */
} NEO7M_FixType_t;

/*============================================================================
 * DATA STRUCTURES
 *============================================================================*/

typedef struct {
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint16_t millisecond;
} NEO7M_Time_t;

typedef struct {
    uint8_t day;
    uint8_t month;
    uint16_t year;
} NEO7M_Date_t;

typedef struct {
    /* UART handle */
    UART_HandleTypeDef *huart;
    
    /* RX buffer and state */
    uint8_t rx_buffer[NEO7M_RX_BUFFER_SIZE];
    volatile uint16_t rx_head;
    volatile uint16_t rx_tail;
    
    /* Current sentence being parsed */
    char sentence[NEO7M_MAX_SENTENCE_LEN];
    uint8_t sentence_idx;
    bool sentence_started;
    
    /* Position data */
    double latitude;          /* Degrees (positive = North) */
    double longitude;         /* Degrees (positive = East) */
    float altitude_msl;       /* Altitude above mean sea level (meters) */
    float geoid_separation;   /* Geoid separation (meters) */
    
    /* Navigation data */
    float speed_knots;        /* Ground speed in knots */
    float speed_kmh;          /* Ground speed in km/h */
    float course;             /* Course over ground (degrees) */
    
    /* Time and date */
    NEO7M_Time_t time;
    NEO7M_Date_t date;
    
    /* Fix information */
    NEO7M_FixQuality_t fix_quality;
    NEO7M_FixType_t fix_type;
    uint8_t satellites_used;
    uint8_t satellites_in_view;
    
    /* Dilution of Precision */
    float hdop;               /* Horizontal DOP */
    float vdop;               /* Vertical DOP */
    float pdop;               /* Position DOP */
    
    /* Status flags */
    bool data_valid;          /* A=valid, V=invalid in GPRMC */
    bool fix_available;       /* True if fix_quality > 0 */
    
    /* Update timestamps */
    uint32_t last_gga_tick;   /* Tick when last GGA was received */
    uint32_t last_rmc_tick;   /* Tick when last RMC was received */
    uint32_t last_update_tick;
    
    /* Statistics */
    uint32_t sentences_parsed;
    uint32_t checksum_errors;
    
    /* Initialization status */
    bool initialized;
    
} NEO7M_Handle_t;

/*============================================================================
 * FUNCTION PROTOTYPES
 *============================================================================*/

/**
 * @brief Initialize NEO-7M GPS module
 * @param dev Pointer to NEO7M handle
 * @param huart Pointer to UART handle (must be pre-configured)
 * @return HAL_OK on success
 */
HAL_StatusTypeDef NEO7M_Init(NEO7M_Handle_t *dev, UART_HandleTypeDef *huart);

/**
 * @brief Start receiving GPS data (enables UART interrupt)
 * @param dev Pointer to NEO7M handle
 * @return HAL_OK on success
 */
HAL_StatusTypeDef NEO7M_StartReceiving(NEO7M_Handle_t *dev);

/**
 * @brief Process received UART data (call from UART RX callback)
 * @param dev Pointer to NEO7M handle
 * @param data Received byte
 */
void NEO7M_ProcessByte(NEO7M_Handle_t *dev, uint8_t data);

/**
 * @brief Parse all complete sentences in buffer (call from main loop)
 * @param dev Pointer to NEO7M handle
 * @return Number of sentences parsed
 */
uint8_t NEO7M_ParseBuffer(NEO7M_Handle_t *dev);

/**
 * @brief Check if GPS has a valid fix
 * @param dev Pointer to NEO7M handle
 * @return true if valid fix available
 */
bool NEO7M_HasFix(NEO7M_Handle_t *dev);

/**
 * @brief Check if GPS data is fresh (received within timeout)
 * @param dev Pointer to NEO7M handle
 * @param timeout_ms Maximum age of data in milliseconds
 * @return true if data is fresh
 */
bool NEO7M_IsDataFresh(NEO7M_Handle_t *dev, uint32_t timeout_ms);

/**
 * @brief Get latitude
 * @param dev Pointer to NEO7M handle
 * @return Latitude in degrees (positive = North)
 */
double NEO7M_GetLatitude(NEO7M_Handle_t *dev);

/**
 * @brief Get longitude
 * @param dev Pointer to NEO7M handle
 * @return Longitude in degrees (positive = East)
 */
double NEO7M_GetLongitude(NEO7M_Handle_t *dev);

/**
 * @brief Get altitude above mean sea level
 * @param dev Pointer to NEO7M handle
 * @return Altitude in meters
 */
float NEO7M_GetAltitude(NEO7M_Handle_t *dev);

/**
 * @brief Get ground speed
 * @param dev Pointer to NEO7M handle
 * @return Speed in km/h
 */
float NEO7M_GetSpeed(NEO7M_Handle_t *dev);

/**
 * @brief Get course over ground
 * @param dev Pointer to NEO7M handle
 * @return Course in degrees (0-360)
 */
float NEO7M_GetCourse(NEO7M_Handle_t *dev);

/**
 * @brief Get number of satellites used
 * @param dev Pointer to NEO7M handle
 * @return Number of satellites
 */
uint8_t NEO7M_GetSatellites(NEO7M_Handle_t *dev);

/**
 * @brief Get horizontal dilution of precision
 * @param dev Pointer to NEO7M handle
 * @return HDOP value (lower is better, <2 is excellent)
 */
float NEO7M_GetHDOP(NEO7M_Handle_t *dev);

/**
 * @brief Get fix quality
 * @param dev Pointer to NEO7M handle
 * @return Fix quality enum value
 */
NEO7M_FixQuality_t NEO7M_GetFixQuality(NEO7M_Handle_t *dev);

/**
 * @brief Get current UTC time from GPS
 * @param dev Pointer to NEO7M handle
 * @return Time structure
 */
NEO7M_Time_t NEO7M_GetTime(NEO7M_Handle_t *dev);

/**
 * @brief Get current UTC date from GPS
 * @param dev Pointer to NEO7M handle
 * @return Date structure
 */
NEO7M_Date_t NEO7M_GetDate(NEO7M_Handle_t *dev);

/**
 * @brief Calculate distance between two GPS coordinates (Haversine formula)
 * @param lat1 Latitude of point 1 (degrees)
 * @param lon1 Longitude of point 1 (degrees)
 * @param lat2 Latitude of point 2 (degrees)
 * @param lon2 Longitude of point 2 (degrees)
 * @return Distance in meters
 */
float NEO7M_CalculateDistance(double lat1, double lon1, double lat2, double lon2);

/**
 * @brief Calculate bearing from point 1 to point 2
 * @param lat1 Latitude of point 1 (degrees)
 * @param lon1 Longitude of point 1 (degrees)
 * @param lat2 Latitude of point 2 (degrees)
 * @param lon2 Longitude of point 2 (degrees)
 * @return Bearing in degrees (0-360)
 */
float NEO7M_CalculateBearing(double lat1, double lon1, double lat2, double lon2);

/**
 * @brief Send UBX configuration command
 * @param dev Pointer to NEO7M handle
 * @param data Command data
 * @param len Command length
 * @return HAL_OK on success
 */
HAL_StatusTypeDef NEO7M_SendUBX(NEO7M_Handle_t *dev, uint8_t *data, uint16_t len);

/**
 * @brief Configure GPS update rate
 * @param dev Pointer to NEO7M handle
 * @param rate_hz Update rate in Hz (1-5)
 * @return HAL_OK on success
 */
HAL_StatusTypeDef NEO7M_SetUpdateRate(NEO7M_Handle_t *dev, uint8_t rate_hz);

/**
 * @brief Configure which NMEA sentences are output
 * @param dev Pointer to NEO7M handle
 * @param gga Enable GGA (position)
 * @param rmc Enable RMC (recommended minimum)
 * @param gsa Enable GSA (DOP and satellites)
 * @param gsv Enable GSV (satellites in view)
 * @return HAL_OK on success
 */
HAL_StatusTypeDef NEO7M_ConfigureNMEA(NEO7M_Handle_t *dev, bool gga, bool rmc, bool gsa, bool gsv);

/**
 * @brief Put GPS into power save mode
 * @param dev Pointer to NEO7M handle
 * @return HAL_OK on success
 */
HAL_StatusTypeDef NEO7M_EnterPowerSave(NEO7M_Handle_t *dev);

/**
 * @brief Wake GPS from power save mode
 * @param dev Pointer to NEO7M handle
 * @return HAL_OK on success
 */
HAL_StatusTypeDef NEO7M_ExitPowerSave(NEO7M_Handle_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* NEO7M_H */
