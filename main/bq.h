#pragma once

// ──────────────────────────────────────────────────────────────────────────────
//  Generic WORD helper
// ──────────────────────────────────────────────────────────────────────────────
/**
 * A *word* in the SBS spec is a 16‑bit little‑endian value returned by the
 * gauge. Many commands (Voltage, Temperature, Current…) simply map that WORD to
 * engineering units through a linear transform:   result = raw * scaling + offset.
 *
 * Define a `bq_entry` describing each command you care about and call
 * `bq_generic_word()` to fetch/print it – or use it inside console commands.
 */

typedef enum
{
    BQ40Z555_TYPE_BYTE = 0,
    BQ40Z555_TYPE_WORD_FLOAT,
    BQ40Z555_TYPE_WORD_INTEGER,
    BQ40Z555_TYPE_WORD_HEX,
    BQ40Z555_TYPE_BLOCK_ASCII,
    BQ40Z555_TYPE_BLOCK_HEX,
    BQ40Z555_TYPE_BLOCK_BITS
} bq_data_type;


// ──────────────────────────────────────────────────────────────────────────────
//  Self‑describing **bit‑field** support
// ──────────────────────────────────────────────────────────────────────────────
/**
 * Describe a sub‑field within a WORD (bit‑mapped flags, enums…).
 *
 * `bit`   – least‑significant bit position (0 = LSB)
 * `width` – number of bits (1 for a single flag)
 * `desc`  – short human‑readable description
 */
typedef struct {
    uint8_t bit;
    uint8_t width;
    const char *desc;
    const char *long_desc;
} bq_bit_desc_t;


typedef struct bq_entry
{
    uint8_t reg;      ///< SBS command code (0x00‑0xFF)
    const char *name; ///< Human‑readable name (for printf & logging)
    const char *unit; ///< Engineering unit string (e.g. "V", "°C")
    float offset;     ///< Additive offset after scaling (e.g. –273.15 for K→°C)
    float scaling;    ///< Multiplier applied to RAW word before offset
    bq_data_type type;
    const bq_bit_desc_t *bits;
    uint8_t bits_count;
} bq_entry;

#define BQ40Z555_CMD_MANUFACTURER_ACCESS 0x00
#define BQ40Z555_CMD_REMAINING_CAPACITY_ALARM 0x01
#define BQ40Z555_CMD_REMAINING_TIME_ALARM 0x02
#define BQ40Z555_CMD_BATTERY_MODE 0x03
#define BQ40Z555_CMD_AT_RATE 0x04
#define BQ40Z555_CMD_AT_RATE_TIME_TO_FULL 0x05
#define BQ40Z555_CMD_AT_RATE_TIME_TO_EMPTY 0x06
#define BQ40Z555_CMD_AT_RATE_OK 0x07
#define BQ40Z555_CMD_TEMPERATURE 0x08
#define BQ40Z555_CMD_VOLTAGE 0x09
#define BQ40Z555_CMD_CURRENT 0x0A
#define BQ40Z555_CMD_AVERAGE_CURRENT 0x0B
#define BQ40Z555_CMD_MAX_ERROR 0x0C
#define BQ40Z555_CMD_RELATIVE_STATE_OF_CHARGE 0x0D
#define BQ40Z555_CMD_ABSOLUTE_STATE_OF_CHARGE 0x0E
#define BQ40Z555_CMD_REMAINING_CAPACITY 0x0F
#define BQ40Z555_CMD_FULL_CHARGE_CAPACITY 0x10
#define BQ40Z555_CMD_RUN_TIME_TO_EMPTY 0x11
#define BQ40Z555_CMD_AVERAGE_TIME_TO_EMPTY 0x12
#define BQ40Z555_CMD_AVERAGE_TIME_TO_FULL 0x13
#define BQ40Z555_CMD_CHARGING_CURRENT 0x14
#define BQ40Z555_CMD_CHARGING_VOLTAGE 0x15
#define BQ40Z555_CMD_BATTERY_STATUS 0x16
#define BQ40Z555_CMD_CYCLE_COUNT 0x17
#define BQ40Z555_CMD_DESIGN_CAPACITY 0x18
#define BQ40Z555_CMD_DESIGN_VOLTAGE 0x19
#define BQ40Z555_CMD_SPECIFICATION_INFO 0x1A
#define BQ40Z555_CMD_MANUFACTURER_DATE 0x1B
#define BQ40Z555_CMD_SERIAL_NUMBER 0x1C
#define BQ40Z555_CMD_MANUFACTURER_NAME 0x20
#define BQ40Z555_CMD_DEVICE_NAME 0x21
#define BQ40Z555_CMD_DEVICE_CHEMISTRY 0x22
#define BQ40Z555_CMD_MANUFACTURER_DATA 0x23
#define BQ40Z555_CMD_AUTHENTICATE 0x2F
#define BQ40Z555_CMD_CELL_VOLTAGE4 0x3C // Cell 3 → index order per TI
#define BQ40Z555_CMD_CELL_VOLTAGE3 0x3D
#define BQ40Z555_CMD_CELL_VOLTAGE2 0x3E
#define BQ40Z555_CMD_CELL_VOLTAGE1 0x3F
#define BQ40Z555_CMD_STATE_OF_HEALTH 0x4F
#define BQ40Z555_CMD_SAFETY_ALERT 0x50
#define BQ40Z555_CMD_SAFETY_STATUS 0x51
#define BQ40Z555_CMD_PF_ALERT 0x52
#define BQ40Z555_CMD_PF_STATUS 0x53
#define BQ40Z555_CMD_OPERATION_STATUS 0x54
#define BQ40Z555_CMD_CHARGING_STATUS 0x55
#define BQ40Z555_CMD_GAUGING_STATUS 0x56
#define BQ40Z555_CMD_MANUFACTURING_STATUS 0x57
#define BQ40Z555_CMD_AFE_REGISTERS 0x58
#define BQ40Z555_CMD_TURBO_POWER 0x59
#define BQ40Z555_CMD_TURBO_FINAL 0x5A
#define BQ40Z555_CMD_TURBO_PACK_R 0x5B
#define BQ40Z555_CMD_TURBO_SYS_R 0x5C
#define BQ40Z555_CMD_MIN_SYS_V 0x5D
#define BQ40Z555_CMD_TURBO_CURRENT 0x5E
#define BQ40Z555_CMD_LIFETIME_DATA1 0x60
#define BQ40Z555_CMD_LIFETIME_DATA2 0x61
#define BQ40Z555_CMD_LIFETIME_DATA3 0x62
#define BQ40Z555_CMD_MANUFACTURER_INFO 0x70
#define BQ40Z555_CMD_VOLTAGES 0x71
#define BQ40Z555_CMD_TEMPERATURES 0x72
#define BQ40Z555_CMD_IT_STATUS1 0x73
#define BQ40Z555_CMD_IT_STATUS2 0x74
// (ManufacturerAccess() sub‑commands 0x0000‑0x0074, 0x01yy, 0xF080‑… remain
//  word/block writes through CMD 0x00 and are therefore *not* enumerated here.)

// ──────────────────────────────────────────────────────────────────────────────
//  Configuration
// ──────────────────────────────────────────────────────────────────────────────

#ifndef BQ40Z555_I2C_ADDR
/// SMBus/I²C 7‑bit slave address of the BQ40Z555 (TI default 0x0B)
#define BQ40Z555_I2C_ADDR 0x0B
#endif

void bq_start();
