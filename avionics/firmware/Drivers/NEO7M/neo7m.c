/**
 * @file neo7m.c
 * @brief u-blox NEO-7M GPS Module Driver Implementation
 * @author Zenith Rocket Team
 * @date 2025
 */

#include "neo7m.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

/*============================================================================
 * PRIVATE DEFINES
 *============================================================================*/

#define KNOTS_TO_KMH    1.852f
#define DEG_TO_RAD      0.01745329251f
#define RAD_TO_DEG      57.2957795131f
#define EARTH_RADIUS_M  6371000.0f

/*============================================================================
 * PRIVATE VARIABLES
 *============================================================================*/

/* Single byte RX buffer for interrupt mode */
static uint8_t rx_byte;

/*============================================================================
 * PRIVATE FUNCTION PROTOTYPES
 *============================================================================*/

static bool NEO7M_ValidateChecksum(const char *sentence);
static void NEO7M_ParseGGA(NEO7M_Handle_t *dev, char *sentence);
static void NEO7M_ParseRMC(NEO7M_Handle_t *dev, char *sentence);
static void NEO7M_ParseGSA(NEO7M_Handle_t *dev, char *sentence);
static void NEO7M_ParseGSV(NEO7M_Handle_t *dev, char *sentence);
static double NEO7M_ParseCoordinate(const char *coord, const char *dir);
static char* NEO7M_GetField(char *sentence, uint8_t field_num);

/*============================================================================
 * PUBLIC FUNCTIONS
 *============================================================================*/

HAL_StatusTypeDef NEO7M_Init(NEO7M_Handle_t *dev, UART_HandleTypeDef *huart)
{
    if (dev == NULL || huart == NULL) {
        return HAL_ERROR;
    }
    
    /* Clear handle */
    memset(dev, 0, sizeof(NEO7M_Handle_t));
    
    /* Store UART handle */
    dev->huart = huart;
    
    /* Initialize buffer indices */
    dev->rx_head = 0;
    dev->rx_tail = 0;
    
    /* Initialize sentence parser state */
    dev->sentence_idx = 0;
    dev->sentence_started = false;
    
    /* Initialize fix state */
    dev->fix_quality = NEO7M_FIX_INVALID;
    dev->fix_type = NEO7M_FIX_TYPE_NONE;
    dev->fix_available = false;
    dev->data_valid = false;
    
    /* Initialize DOP to worst case */
    dev->hdop = 99.99f;
    dev->vdop = 99.99f;
    dev->pdop = 99.99f;
    
    dev->initialized = true;
    
    return HAL_OK;
}

HAL_StatusTypeDef NEO7M_StartReceiving(NEO7M_Handle_t *dev)
{
    if (dev == NULL || !dev->initialized) {
        return HAL_ERROR;
    }
    
    /* Start UART receive in interrupt mode */
    return HAL_UART_Receive_IT(dev->huart, &rx_byte, 1);
}

void NEO7M_ProcessByte(NEO7M_Handle_t *dev, uint8_t data)
{
    if (dev == NULL || !dev->initialized) {
        return;
    }
    
    /* Add to circular buffer */
    uint16_t next_head = (dev->rx_head + 1) % NEO7M_RX_BUFFER_SIZE;
    
    if (next_head != dev->rx_tail) {
        dev->rx_buffer[dev->rx_head] = data;
        dev->rx_head = next_head;
    }
    /* else: buffer full, drop byte */
    
    /* Restart UART receive */
    HAL_UART_Receive_IT(dev->huart, &rx_byte, 1);
}

uint8_t NEO7M_ParseBuffer(NEO7M_Handle_t *dev)
{
    if (dev == NULL || !dev->initialized) {
        return 0;
    }
    
    uint8_t sentences_parsed = 0;
    
    /* Process all available bytes */
    while (dev->rx_tail != dev->rx_head) {
        uint8_t c = dev->rx_buffer[dev->rx_tail];
        dev->rx_tail = (dev->rx_tail + 1) % NEO7M_RX_BUFFER_SIZE;
        
        if (c == '$') {
            /* Start of new NMEA sentence */
            dev->sentence_idx = 0;
            dev->sentence_started = true;
        }
        else if (dev->sentence_started) {
            if (c == '\r' || c == '\n') {
                /* End of sentence */
                if (dev->sentence_idx > 0) {
                    dev->sentence[dev->sentence_idx] = '\0';
                    
                    /* Validate checksum */
                    if (NEO7M_ValidateChecksum(dev->sentence)) {
                        /* Parse based on sentence type */
                        if (strncmp(dev->sentence, "GPGGA", 5) == 0 ||
                            strncmp(dev->sentence, "GNGGA", 5) == 0) {
                            NEO7M_ParseGGA(dev, dev->sentence);
                            dev->last_gga_tick = HAL_GetTick();
                            sentences_parsed++;
                        }
                        else if (strncmp(dev->sentence, "GPRMC", 5) == 0 ||
                                 strncmp(dev->sentence, "GNRMC", 5) == 0) {
                            NEO7M_ParseRMC(dev, dev->sentence);
                            dev->last_rmc_tick = HAL_GetTick();
                            sentences_parsed++;
                        }
                        else if (strncmp(dev->sentence, "GPGSA", 5) == 0 ||
                                 strncmp(dev->sentence, "GNGSA", 5) == 0) {
                            NEO7M_ParseGSA(dev, dev->sentence);
                            sentences_parsed++;
                        }
                        else if (strncmp(dev->sentence, "GPGSV", 5) == 0 ||
                                 strncmp(dev->sentence, "GNGSV", 5) == 0) {
                            NEO7M_ParseGSV(dev, dev->sentence);
                            sentences_parsed++;
                        }
                        
                        dev->sentences_parsed++;
                        dev->last_update_tick = HAL_GetTick();
                    }
                    else {
                        dev->checksum_errors++;
                    }
                }
                dev->sentence_started = false;
            }
            else if (dev->sentence_idx < NEO7M_MAX_SENTENCE_LEN - 1) {
                dev->sentence[dev->sentence_idx++] = c;
            }
        }
    }
    
    return sentences_parsed;
}

bool NEO7M_HasFix(NEO7M_Handle_t *dev)
{
    if (dev == NULL || !dev->initialized) {
        return false;
    }
    return dev->fix_available && (dev->fix_quality > NEO7M_FIX_INVALID);
}

bool NEO7M_IsDataFresh(NEO7M_Handle_t *dev, uint32_t timeout_ms)
{
    if (dev == NULL || !dev->initialized) {
        return false;
    }
    
    uint32_t now = HAL_GetTick();
    return (now - dev->last_update_tick) < timeout_ms;
}

double NEO7M_GetLatitude(NEO7M_Handle_t *dev)
{
    if (dev == NULL) return 0.0;
    return dev->latitude;
}

double NEO7M_GetLongitude(NEO7M_Handle_t *dev)
{
    if (dev == NULL) return 0.0;
    return dev->longitude;
}

float NEO7M_GetAltitude(NEO7M_Handle_t *dev)
{
    if (dev == NULL) return 0.0f;
    return dev->altitude_msl;
}

float NEO7M_GetSpeed(NEO7M_Handle_t *dev)
{
    if (dev == NULL) return 0.0f;
    return dev->speed_kmh;
}

float NEO7M_GetCourse(NEO7M_Handle_t *dev)
{
    if (dev == NULL) return 0.0f;
    return dev->course;
}

uint8_t NEO7M_GetSatellites(NEO7M_Handle_t *dev)
{
    if (dev == NULL) return 0;
    return dev->satellites_used;
}

float NEO7M_GetHDOP(NEO7M_Handle_t *dev)
{
    if (dev == NULL) return 99.99f;
    return dev->hdop;
}

NEO7M_FixQuality_t NEO7M_GetFixQuality(NEO7M_Handle_t *dev)
{
    if (dev == NULL) return NEO7M_FIX_INVALID;
    return dev->fix_quality;
}

NEO7M_Time_t NEO7M_GetTime(NEO7M_Handle_t *dev)
{
    NEO7M_Time_t empty = {0, 0, 0, 0};
    if (dev == NULL) return empty;
    return dev->time;
}

NEO7M_Date_t NEO7M_GetDate(NEO7M_Handle_t *dev)
{
    NEO7M_Date_t empty = {0, 0, 0};
    if (dev == NULL) return empty;
    return dev->date;
}

float NEO7M_CalculateDistance(double lat1, double lon1, double lat2, double lon2)
{
    /* Haversine formula */
    double lat1_rad = lat1 * DEG_TO_RAD;
    double lat2_rad = lat2 * DEG_TO_RAD;
    double dlat = (lat2 - lat1) * DEG_TO_RAD;
    double dlon = (lon2 - lon1) * DEG_TO_RAD;
    
    double a = sin(dlat / 2.0) * sin(dlat / 2.0) +
               cos(lat1_rad) * cos(lat2_rad) *
               sin(dlon / 2.0) * sin(dlon / 2.0);
    
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    
    return (float)(EARTH_RADIUS_M * c);
}

float NEO7M_CalculateBearing(double lat1, double lon1, double lat2, double lon2)
{
    double lat1_rad = lat1 * DEG_TO_RAD;
    double lat2_rad = lat2 * DEG_TO_RAD;
    double dlon = (lon2 - lon1) * DEG_TO_RAD;
    
    double y = sin(dlon) * cos(lat2_rad);
    double x = cos(lat1_rad) * sin(lat2_rad) -
               sin(lat1_rad) * cos(lat2_rad) * cos(dlon);
    
    float bearing = (float)(atan2(y, x) * RAD_TO_DEG);
    
    /* Normalize to 0-360 */
    if (bearing < 0) {
        bearing += 360.0f;
    }
    
    return bearing;
}

HAL_StatusTypeDef NEO7M_SendUBX(NEO7M_Handle_t *dev, uint8_t *data, uint16_t len)
{
    if (dev == NULL || data == NULL || !dev->initialized) {
        return HAL_ERROR;
    }
    
    return HAL_UART_Transmit(dev->huart, data, len, 100);
}

HAL_StatusTypeDef NEO7M_SetUpdateRate(NEO7M_Handle_t *dev, uint8_t rate_hz)
{
    if (dev == NULL || !dev->initialized) {
        return HAL_ERROR;
    }
    
    if (rate_hz < 1) rate_hz = 1;
    if (rate_hz > 5) rate_hz = 5;
    
    /* UBX-CFG-RATE command */
    /* Measurement period in ms */
    uint16_t period_ms = 1000 / rate_hz;
    
    uint8_t ubx_cfg_rate[] = {
        0xB5, 0x62,             /* UBX sync chars */
        0x06, 0x08,             /* Class: CFG, ID: RATE */
        0x06, 0x00,             /* Length: 6 bytes */
        (uint8_t)(period_ms & 0xFF),        /* measRate LSB */
        (uint8_t)(period_ms >> 8),          /* measRate MSB */
        0x01, 0x00,             /* navRate = 1 */
        0x01, 0x00,             /* timeRef = GPS */
        0x00, 0x00              /* Checksum (calculated below) */
    };
    
    /* Calculate UBX checksum */
    uint8_t ck_a = 0, ck_b = 0;
    for (int i = 2; i < 12; i++) {
        ck_a += ubx_cfg_rate[i];
        ck_b += ck_a;
    }
    ubx_cfg_rate[12] = ck_a;
    ubx_cfg_rate[13] = ck_b;
    
    return NEO7M_SendUBX(dev, ubx_cfg_rate, sizeof(ubx_cfg_rate));
}

HAL_StatusTypeDef NEO7M_ConfigureNMEA(NEO7M_Handle_t *dev, bool gga, bool rmc, bool gsa, bool gsv)
{
    if (dev == NULL || !dev->initialized) {
        return HAL_ERROR;
    }
    
    /* Send PMTK commands to enable/disable sentences */
    /* PMTK314 - Set NMEA output frequencies */
    /* Format: GLL, RMC, VTG, GGA, GSA, GSV, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, ZDA */
    
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "$PMTK314,0,%d,0,%d,%d,%d,0,0,0,0,0,0,0,0,0,0,0,0,0",
             rmc ? 1 : 0,
             gga ? 1 : 0,
             gsa ? 1 : 0,
             gsv ? 1 : 0);
    
    /* Calculate NMEA checksum */
    uint8_t checksum = 0;
    for (int i = 1; cmd[i] != '\0'; i++) {
        checksum ^= cmd[i];
    }
    
    char full_cmd[80];
    snprintf(full_cmd, sizeof(full_cmd), "%s*%02X\r\n", cmd, checksum);
    
    return HAL_UART_Transmit(dev->huart, (uint8_t*)full_cmd, strlen(full_cmd), 100);
}

HAL_StatusTypeDef NEO7M_EnterPowerSave(NEO7M_Handle_t *dev)
{
    if (dev == NULL || !dev->initialized) {
        return HAL_ERROR;
    }
    
    /* UBX-CFG-RXM command for power save mode */
    uint8_t ubx_cfg_rxm[] = {
        0xB5, 0x62,             /* UBX sync chars */
        0x06, 0x11,             /* Class: CFG, ID: RXM */
        0x02, 0x00,             /* Length: 2 bytes */
        0x00,                   /* reserved1 */
        0x01,                   /* lpMode = 1 (Power Save Mode) */
        0x00, 0x00              /* Checksum */
    };
    
    /* Calculate UBX checksum */
    uint8_t ck_a = 0, ck_b = 0;
    for (int i = 2; i < 8; i++) {
        ck_a += ubx_cfg_rxm[i];
        ck_b += ck_a;
    }
    ubx_cfg_rxm[8] = ck_a;
    ubx_cfg_rxm[9] = ck_b;
    
    return NEO7M_SendUBX(dev, ubx_cfg_rxm, sizeof(ubx_cfg_rxm));
}

HAL_StatusTypeDef NEO7M_ExitPowerSave(NEO7M_Handle_t *dev)
{
    if (dev == NULL || !dev->initialized) {
        return HAL_ERROR;
    }
    
    /* UBX-CFG-RXM command for continuous mode */
    uint8_t ubx_cfg_rxm[] = {
        0xB5, 0x62,             /* UBX sync chars */
        0x06, 0x11,             /* Class: CFG, ID: RXM */
        0x02, 0x00,             /* Length: 2 bytes */
        0x00,                   /* reserved1 */
        0x00,                   /* lpMode = 0 (Continuous) */
        0x00, 0x00              /* Checksum */
    };
    
    /* Calculate UBX checksum */
    uint8_t ck_a = 0, ck_b = 0;
    for (int i = 2; i < 8; i++) {
        ck_a += ubx_cfg_rxm[i];
        ck_b += ck_a;
    }
    ubx_cfg_rxm[8] = ck_a;
    ubx_cfg_rxm[9] = ck_b;
    
    return NEO7M_SendUBX(dev, ubx_cfg_rxm, sizeof(ubx_cfg_rxm));
}

/*============================================================================
 * PRIVATE FUNCTIONS
 *============================================================================*/

static bool NEO7M_ValidateChecksum(const char *sentence)
{
    if (sentence == NULL) return false;
    
    /* Find checksum marker '*' */
    const char *star = strchr(sentence, '*');
    if (star == NULL) return false;
    
    /* Calculate checksum (XOR of all chars between $ and *) */
    uint8_t calc_checksum = 0;
    for (const char *p = sentence; p < star; p++) {
        calc_checksum ^= *p;
    }
    
    /* Parse provided checksum */
    uint8_t provided_checksum = (uint8_t)strtol(star + 1, NULL, 16);
    
    return calc_checksum == provided_checksum;
}

static char* NEO7M_GetField(char *sentence, uint8_t field_num)
{
    uint8_t current_field = 0;
    char *p = sentence;
    
    /* Skip sentence identifier (e.g., "GPGGA") */
    while (*p != ',' && *p != '\0') p++;
    if (*p == ',') p++;
    
    while (current_field < field_num && *p != '\0') {
        if (*p == ',') {
            current_field++;
        }
        if (current_field < field_num) {
            p++;
        }
    }
    
    if (*p == ',') p++;
    
    return p;
}

static double NEO7M_ParseCoordinate(const char *coord, const char *dir)
{
    if (coord == NULL || dir == NULL || *coord == '\0') {
        return 0.0;
    }
    
    /* NMEA format: DDDMM.MMMM or DDMM.MMMM */
    double value = atof(coord);
    
    /* Extract degrees and minutes */
    int degrees = (int)(value / 100.0);
    double minutes = value - (degrees * 100.0);
    
    /* Convert to decimal degrees */
    double result = degrees + (minutes / 60.0);
    
    /* Apply direction */
    if (*dir == 'S' || *dir == 'W') {
        result = -result;
    }
    
    return result;
}

static void NEO7M_ParseGGA(NEO7M_Handle_t *dev, char *sentence)
{
    /* Make a working copy for strtok */
    char buffer[NEO7M_MAX_SENTENCE_LEN];
    strncpy(buffer, sentence, NEO7M_MAX_SENTENCE_LEN - 1);
    buffer[NEO7M_MAX_SENTENCE_LEN - 1] = '\0';
    
    /* $GPGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
     * Field 0: Sentence type (GPGGA)
     * Field 1: Time (hhmmss.ss)
     * Field 2: Latitude
     * Field 3: N/S
     * Field 4: Longitude
     * Field 5: E/W
     * Field 6: Fix quality
     * Field 7: Number of satellites
     * Field 8: HDOP
     * Field 9: Altitude (MSL)
     * Field 10: M
     * Field 11: Geoid separation
     * Field 12: M
     */
    
    char *token;
    uint8_t field = 0;
    char lat_str[16] = "", lat_dir[2] = "";
    char lon_str[16] = "", lon_dir[2] = "";
    
    token = strtok(buffer, ",");
    while (token != NULL) {
        switch (field) {
            case 1: /* Time */
                if (strlen(token) >= 6) {
                    dev->time.hour = (token[0] - '0') * 10 + (token[1] - '0');
                    dev->time.minute = (token[2] - '0') * 10 + (token[3] - '0');
                    dev->time.second = (token[4] - '0') * 10 + (token[5] - '0');
                    if (strlen(token) > 6 && token[6] == '.') {
                        dev->time.millisecond = (uint16_t)(atof(&token[6]) * 1000);
                    }
                }
                break;
            case 2: /* Latitude */
                strncpy(lat_str, token, sizeof(lat_str) - 1);
                break;
            case 3: /* N/S */
                strncpy(lat_dir, token, sizeof(lat_dir) - 1);
                break;
            case 4: /* Longitude */
                strncpy(lon_str, token, sizeof(lon_str) - 1);
                break;
            case 5: /* E/W */
                strncpy(lon_dir, token, sizeof(lon_dir) - 1);
                break;
            case 6: /* Fix quality */
                dev->fix_quality = (NEO7M_FixQuality_t)atoi(token);
                dev->fix_available = (dev->fix_quality > NEO7M_FIX_INVALID);
                break;
            case 7: /* Satellites used */
                dev->satellites_used = (uint8_t)atoi(token);
                break;
            case 8: /* HDOP */
                dev->hdop = (float)atof(token);
                break;
            case 9: /* Altitude MSL */
                dev->altitude_msl = (float)atof(token);
                break;
            case 11: /* Geoid separation */
                dev->geoid_separation = (float)atof(token);
                break;
        }
        field++;
        token = strtok(NULL, ",");
    }
    
    /* Parse coordinates after collecting all parts */
    if (lat_str[0] != '\0' && lat_dir[0] != '\0') {
        dev->latitude = NEO7M_ParseCoordinate(lat_str, lat_dir);
    }
    if (lon_str[0] != '\0' && lon_dir[0] != '\0') {
        dev->longitude = NEO7M_ParseCoordinate(lon_str, lon_dir);
    }
}

static void NEO7M_ParseRMC(NEO7M_Handle_t *dev, char *sentence)
{
    char buffer[NEO7M_MAX_SENTENCE_LEN];
    strncpy(buffer, sentence, NEO7M_MAX_SENTENCE_LEN - 1);
    buffer[NEO7M_MAX_SENTENCE_LEN - 1] = '\0';
    
    /* $GPRMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,ddmmyy,x.x,a*hh
     * Field 0: Sentence type (GPRMC)
     * Field 1: Time
     * Field 2: Status (A=valid, V=invalid)
     * Field 3: Latitude
     * Field 4: N/S
     * Field 5: Longitude
     * Field 6: E/W
     * Field 7: Speed (knots)
     * Field 8: Course (degrees)
     * Field 9: Date (ddmmyy)
     */
    
    char *token;
    uint8_t field = 0;
    char lat_str[16] = "", lat_dir[2] = "";
    char lon_str[16] = "", lon_dir[2] = "";
    
    token = strtok(buffer, ",");
    while (token != NULL) {
        switch (field) {
            case 1: /* Time */
                if (strlen(token) >= 6) {
                    dev->time.hour = (token[0] - '0') * 10 + (token[1] - '0');
                    dev->time.minute = (token[2] - '0') * 10 + (token[3] - '0');
                    dev->time.second = (token[4] - '0') * 10 + (token[5] - '0');
                }
                break;
            case 2: /* Status */
                dev->data_valid = (token[0] == 'A');
                break;
            case 3: /* Latitude */
                strncpy(lat_str, token, sizeof(lat_str) - 1);
                break;
            case 4: /* N/S */
                strncpy(lat_dir, token, sizeof(lat_dir) - 1);
                break;
            case 5: /* Longitude */
                strncpy(lon_str, token, sizeof(lon_str) - 1);
                break;
            case 6: /* E/W */
                strncpy(lon_dir, token, sizeof(lon_dir) - 1);
                break;
            case 7: /* Speed (knots) */
                dev->speed_knots = (float)atof(token);
                dev->speed_kmh = dev->speed_knots * KNOTS_TO_KMH;
                break;
            case 8: /* Course */
                if (strlen(token) > 0) {
                    dev->course = (float)atof(token);
                }
                break;
            case 9: /* Date */
                if (strlen(token) >= 6) {
                    dev->date.day = (token[0] - '0') * 10 + (token[1] - '0');
                    dev->date.month = (token[2] - '0') * 10 + (token[3] - '0');
                    dev->date.year = 2000 + (token[4] - '0') * 10 + (token[5] - '0');
                }
                break;
        }
        field++;
        token = strtok(NULL, ",");
    }
    
    /* Parse coordinates */
    if (lat_str[0] != '\0' && lat_dir[0] != '\0') {
        dev->latitude = NEO7M_ParseCoordinate(lat_str, lat_dir);
    }
    if (lon_str[0] != '\0' && lon_dir[0] != '\0') {
        dev->longitude = NEO7M_ParseCoordinate(lon_str, lon_dir);
    }
}

static void NEO7M_ParseGSA(NEO7M_Handle_t *dev, char *sentence)
{
    char buffer[NEO7M_MAX_SENTENCE_LEN];
    strncpy(buffer, sentence, NEO7M_MAX_SENTENCE_LEN - 1);
    buffer[NEO7M_MAX_SENTENCE_LEN - 1] = '\0';
    
    /* $GPGSA,A,3,04,05,...,2.5,1.3,2.1*hh
     * Field 0: Sentence type (GPGSA)
     * Field 1: Mode (M=Manual, A=Automatic)
     * Field 2: Fix type (1=none, 2=2D, 3=3D)
     * Fields 3-14: Satellite PRNs
     * Field 15: PDOP
     * Field 16: HDOP
     * Field 17: VDOP
     */
    
    char *token;
    uint8_t field = 0;
    
    token = strtok(buffer, ",");
    while (token != NULL) {
        switch (field) {
            case 2: /* Fix type */
                dev->fix_type = (NEO7M_FixType_t)atoi(token);
                break;
            case 15: /* PDOP */
                dev->pdop = (float)atof(token);
                break;
            case 16: /* HDOP */
                dev->hdop = (float)atof(token);
                break;
            case 17: /* VDOP - may have checksum attached */
                {
                    char *star = strchr(token, '*');
                    if (star) *star = '\0';
                    dev->vdop = (float)atof(token);
                }
                break;
        }
        field++;
        token = strtok(NULL, ",");
    }
}

static void NEO7M_ParseGSV(NEO7M_Handle_t *dev, char *sentence)
{
    char buffer[NEO7M_MAX_SENTENCE_LEN];
    strncpy(buffer, sentence, NEO7M_MAX_SENTENCE_LEN - 1);
    buffer[NEO7M_MAX_SENTENCE_LEN - 1] = '\0';
    
    /* $GPGSV,2,1,08,01,40,083,46,...*hh
     * Field 0: Sentence type (GPGSV)
     * Field 1: Total messages
     * Field 2: Message number
     * Field 3: Satellites in view
     */
    
    char *token;
    uint8_t field = 0;
    
    token = strtok(buffer, ",");
    while (token != NULL && field <= 3) {
        if (field == 3) {
            dev->satellites_in_view = (uint8_t)atoi(token);
            break;
        }
        field++;
        token = strtok(NULL, ",");
    }
}
