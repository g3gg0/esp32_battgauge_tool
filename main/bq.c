// bq.c – ESP-IDF esp_console command(s) for TI BQ30Z555 gas-gauge
//
// This module registers commands that talk to a BQ30Z555 over SMBus/I²C using
// the helper function below provided by board support code:
//      int i2c_write_read(uint8_t addr,
//                         const uint8_t *wdata, size_t wlen,
//                         uint8_t *rdata, size_t rlen);
//
// Copyright (c) 2025  <Your Company / Name>
// SPDX-License-Identifier: MIT

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "esp_console.h"
#include "esp_log.h"
#include "argtable3/argtable3.h"
#include "bq.h"

#define COUNT(x) (sizeof(x) / sizeof((x)[0]))

// ──────────────────────────────────────────────────────────────────────────────
//  Utility
// ──────────────────────────────────────────────────────────────────────────────
static inline uint16_t le16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

// ──────────────────────────────────────────────────────────────────────────────
//  Configuration
// ──────────────────────────────────────────────────────────────────────────────

static const char *TAG = "bq";

// Declaration of BSP helper (implemented elsewhere)
int i2c_write_read(uint8_t addr, const uint8_t *wdata, size_t wlen,
                   uint8_t *rdata, size_t rlen);

// ──────────────────────────────────────────────────────────────────────────────
//  SafetyAlert() bit descriptions (global, can be shared by other tables)
// ──────────────────────────────────────────────────────────────────────────────
static const bq_bit_desc_t SAFETY_ALERT_BITS[] = {
    {0, 1, .desc = "OCC", .long_desc = "Over-Charge Current"},
    {1, 1, .desc = "OCD", .long_desc = "Over-Discharge Current"},
    {2, 1, .desc = "COV", .long_desc = "Cell Over-Voltage"},
    {3, 1, .desc = "CUV", .long_desc = "Cell Under-Voltage"},
    {4, 1, .desc = "OTC", .long_desc = "Over-Temp Charge"},
    {5, 1, .desc = "OTD", .long_desc = "Over-Temp Discharge"},
    {6, 1, .desc = "SCD", .long_desc = "Short-Circuit Discharge"},
    {7, 1, .desc = "OLD", .long_desc = "Overload Protection"},
    {8, 1, .desc = "RSVD8", .long_desc = "Reserved"},
    {9, 1, .desc = "RSVD9", .long_desc = "Reserved"},
    {10, 1, .desc = "RSVD10", .long_desc = "Reserved"},
    {11, 1, .desc = "RSVD11", .long_desc = "Reserved"},
    {12, 1, .desc = "RSVD12", .long_desc = "Reserved"},
    {13, 1, .desc = "PF", .long_desc = "Permanent Fail"},
    {14, 1, .desc = "SLEEP", .long_desc = "Sleep"},
    {15, 1, .desc = "RSVD15", .long_desc = "Reserved"}};

// ──────────────────────────────────────────────────────────────────────────────
// SafetyStatus (0x51)
// ──────────────────────────────────────────────────────────────────────────────
static const bq_bit_desc_t BITS_SAFETY_STATUS[] = {
    {0, 1, .desc = "CUV", .long_desc = "Cell UnderVoltage"},
    {1, 1, .desc = "COV", .long_desc = "Cell Overvoltage"},
    {2, 1, .desc = "OCC1", .long_desc = "Overcurrent in Charge 1st Tier"},
    {3, 1, .desc = "OCC2", .long_desc = "Overcurrent in Charge 2nd Tier"},
    {4, 1, .desc = "OCD1", .long_desc = "Overcurrent in Discharge 1st Tier"},
    {5, 1, .desc = "OCD2", .long_desc = "Overcurrent in Discharge 2nd Tier"},
    {6, 1, .desc = "OLD", .long_desc = "Overload in discharge"},
    {7, 1, .desc = "OLDL", .long_desc = "Overload in discharge latch"},
    {8, 1, .desc = "SCC", .long_desc = "Short circuit in charge"},
    {9, 1, .desc = "SCCL", .long_desc = "Short circuit in charge latch"},
    {10, 1, .desc = "SCD", .long_desc = "Short circuit in discharge"},
    {11, 1, .desc = "SCDL", .long_desc = "Short circuit in discharge latch"},
    {12, 1, .desc = "OTC", .long_desc = "Overtemperature in charge"},
    {13, 1, .desc = "OTD", .long_desc = "Overtemperature in discharge"},
    {14, 1, .desc = "CUVC", .long_desc = "I*R compensated CUV"},
    {15, 1, .desc = "RSVD15", .long_desc = "Reserved"},
    {16, 1, .desc = "OTF", .long_desc = "FET overtemperature"},
    {17, 1, .desc = "HWD", .long_desc = "SBS Host watchdog timeout"},
    {18, 1, .desc = "PTO", .long_desc = "Precharging timeout"},
    {19, 1, .desc = "RSVD19", .long_desc = "Reserved"},
    {20, 1, .desc = "CTO", .long_desc = "Charging timeout"},
    {21, 1, .desc = "RSVD21", .long_desc = "Reserved"},
    {22, 1, .desc = "OC", .long_desc = "Overcharge"},
    {23, 1, .desc = "CHGC", .long_desc = "Charging Current higher than requested"},
    {24, 1, .desc = "CHGV", .long_desc = "Charging Voltage higher than requested"},
    {25, 1, .desc = "RSVD25", .long_desc = "Reserved"},
    {26, 1, .desc = "RSVD26", .long_desc = "Reserved"},
    {27, 1, .desc = "RSVD27", .long_desc = "Reserved"},
    {28, 1, .desc = "RSVD28", .long_desc = "Reserved"},
    {29, 1, .desc = "RSVD29", .long_desc = "Reserved"},
    {30, 1, .desc = "RSVD30", .long_desc = "Reserved"},
    {31, 1, .desc = "RSVD31", .long_desc = "Reserved"}};

// ──────────────────────────────────────────────────────────────────────────────
// PFAlert (0x52) – Permanent Failure flags (latched alert)
// ──────────────────────────────────────────────────────────────────────────────
static const bq_bit_desc_t BITS_PF_ALERT[] = {
    {0, 1, .desc = "CUV", .long_desc = "CUV Latched"},
    {1, 1, .desc = "COV", .long_desc = "COV Latched"},
    {2, 1, .desc = "CUDEP", .long_desc = "Copper deposition"},
    {3, 1, .desc = "RSVD3", .long_desc = "Reserved"},
    {4, 1, .desc = "OTCE", .long_desc = "Overtemperature"},
    {5, 1, .desc = "RSVD5", .long_desc = "Reserved"},
    {6, 1, .desc = "OTF", .long_desc = "Overtemperature FET"},
    {7, 1, .desc = "QIM", .long_desc = "QMAX Imbalance"},
    {8, 1, .desc = "CB", .long_desc = "Cell balancing"},
    {9, 1, .desc = "IMP", .long_desc = "Cell impedance"},
    {10, 1, .desc = "CD", .long_desc = "Capacity Deterioration"},
    {11, 1, .desc = "VIMR", .long_desc = "Voltage imbalance at Rest"},
    {12, 1, .desc = "VIMA", .long_desc = "Voltage imbalance at Rest"},
    {13, 1, .desc = "RSVD13", .long_desc = "Reserved"},
    {14, 1, .desc = "RSVD14", .long_desc = "Reserved"},
    {15, 1, .desc = "RSVD15", .long_desc = "Reserved"},
    {16, 1, .desc = "CFETF", .long_desc = "Charge FET"},
    {17, 1, .desc = "DFET", .long_desc = "Discharge FET"},
    {18, 1, .desc = "THERM", .long_desc = "Thermistor"},
    {19, 1, .desc = "FUSE", .long_desc = "Fuse"},
    {20, 1, .desc = "AFER", .long_desc = "AFE Register"},
    {21, 1, .desc = "AFEC", .long_desc = "AFE Communication"},
    {22, 1, .desc = "2LVL", .long_desc = "FUSE input indicating fuse trigger by external 2nd level protection"},
    {23, 1, .desc = "RSVD23", .long_desc = "Reserved"},
    {24, 1, .desc = "RSVD24", .long_desc = "Reserved"},
    {25, 1, .desc = "OCECO", .long_desc = "Open VCx"},
    {26, 1, .desc = "RSVD26", .long_desc = "Reserved"},
    {27, 1, .desc = "RSVD27", .long_desc = "Reserved"},
    {28, 1, .desc = "RSVD28", .long_desc = "Reserved"},
    {29, 1, .desc = "RSVD29", .long_desc = "Reserved"},
    {30, 1, .desc = "RSVD30", .long_desc = "Reserved"},
    {31, 1, .desc = "RSVD31", .long_desc = "Reserved"}};

// ──────────────────────────────────────────────────────────────────────────────
// PFStatus (0x53)
// ──────────────────────────────────────────────────────────────────────────────
static const bq_bit_desc_t BITS_PF_STATUS[] = {
    {0, 1, .desc = "CUV", .long_desc = "CUV Latched"},
    {1, 1, .desc = "COV", .long_desc = "COV Latched"},
    {2, 1, .desc = "CUDEP", .long_desc = "Copper deposition"},
    {3, 1, .desc = "RSVD3", .long_desc = "Reserved"},
    {4, 1, .desc = "OTCE", .long_desc = "Overtemperature"},
    {5, 1, .desc = "RSVD5", .long_desc = "Reserved"},
    {6, 1, .desc = "OTF", .long_desc = "Overtemperature FET"},
    {7, 1, .desc = "QIM", .long_desc = "QMAX Imbalance"},
    {8, 1, .desc = "CB", .long_desc = "Cell balancing"},
    {9, 1, .desc = "IMP", .long_desc = "Cell impedance"},
    {10, 1, .desc = "CD", .long_desc = "Capacity Deterioration"},
    {11, 1, .desc = "VIMR", .long_desc = "Voltage imbalance at Rest"},
    {12, 1, .desc = "VIMA", .long_desc = "Voltage imbalance at Rest"},
    {13, 1, .desc = "RSVD13", .long_desc = "Reserved"},
    {14, 1, .desc = "RSVD14", .long_desc = "Reserved"},
    {15, 1, .desc = "RSVD15", .long_desc = "Reserved"},
    {16, 1, .desc = "CFETF", .long_desc = "Charge FET"},
    {17, 1, .desc = "DFET", .long_desc = "Discharge FET"},
    {18, 1, .desc = "THERM", .long_desc = "Thermistor"},
    {19, 1, .desc = "FUSE", .long_desc = "Fuse"},
    {20, 1, .desc = "AFER", .long_desc = "AFE Register"},
    {21, 1, .desc = "AFEC", .long_desc = "AFE Communication"},
    {22, 1, .desc = "2LVL", .long_desc = "FUSE input indicating fuse trigger by external 2nd level protection"},
    {23, 1, .desc = "RSVD23", .long_desc = "Reserved"},
    {24, 1, .desc = "RSVD24", .long_desc = "Reserved"},
    {25, 1, .desc = "OCECO", .long_desc = "Open VCx"},
    {26, 1, .desc = "RSVD26", .long_desc = "Reserved"},
    {27, 1, .desc = "RSVD27", .long_desc = "Reserved"},
    {28, 1, .desc = "RSVD28", .long_desc = "Reserved"},
    {29, 1, .desc = "RSVD29", .long_desc = "Reserved"},
    {30, 1, .desc = "RSVD30", .long_desc = "Reserved"},
    {31, 1, .desc = "RSVD31", .long_desc = "Reserved"}};

// ──────────────────────────────────────────────────────────────────────────────
// OperationStatus (0x54)
// ──────────────────────────────────────────────────────────────────────────────
static const bq_bit_desc_t BITS_OPERATION_STATUS[] = {
    {0, 1, .desc = "PRES", .long_desc = "PRES input (active = low detected)"},
    {1, 1, .desc = "DSG", .long_desc = "Discharge FET Enabled"},
    {2, 1, .desc = "CHG", .long_desc = "Charge FET Enabled"},
    {3, 1, .desc = "PCHG", .long_desc = "PCHG FET Enabled"},
    {4, 1, .desc = "GPOD", .long_desc = "GPOD FET Enabled"},
    {5, 1, .desc = "FUSE", .long_desc = "Fuse Input High"},
    {6, 1, .desc = "CB", .long_desc = "Cell Balancing Active"},
    {7, 1, .desc = "RSVD7", .long_desc = "Reserved"},
    {8, 2, .desc = "SEC0/1", .long_desc = "Security Mode (0: Reserved, 1: Full access, 2: Unsealed, 3: Sealed)"},
    {10, 1, .desc = "CAL", .long_desc = "Cal mode active"},
    {11, 1, .desc = "SS", .long_desc = "SafetyStatus active"},
    {12, 1, .desc = "PF", .long_desc = "Permanent Failure active"},
    {13, 1, .desc = "XDSG", .long_desc = "Discharging Disabled"},
    {14, 1, .desc = "XCHG", .long_desc = "Charging Disabled"},
    {15, 1, .desc = "SLEEP", .long_desc = "Sleep condition met"},
    {16, 1, .desc = "SDM", .long_desc = "Shutdown via MfgAccess"},
    {17, 1, .desc = "RSVD17", .long_desc = "Reserved"},
    {18, 1, .desc = "AUTH", .long_desc = "Authentication ongoing"},
    {19, 1, .desc = "AWD", .long_desc = "AFE Watchdog failure"},
    {20, 1, .desc = "FVS", .long_desc = "Fast Voltage Sampling"},
    {21, 1, .desc = "CALO", .long_desc = "Raw ADC/CC offset active"},
    {22, 1, .desc = "SDV", .long_desc = "Shutdown via voltage"},
    {23, 1, .desc = "SLEEPM", .long_desc = "Sleep via MfgAccess"},
    {24, 1, .desc = "INIT", .long_desc = "Init after full reset"},
    {25, 1, .desc = "SMBLCAL", .long_desc = "CC auto offset cal"},
    {26, 1, .desc = "SLEEPQMAX", .long_desc = "QMAX update in sleep"},
    {27, 1, .desc = "SLEEPC", .long_desc = "Current check in sleep"},
    {28, 1, .desc = "XLSBS", .long_desc = "Fast SBS mode"},
    {29, 1, .desc = "RSVD29", .long_desc = "Reserved"},
    {30, 1, .desc = "RSVD30", .long_desc = "Reserved"},
    {31, 1, .desc = "RSVD31", .long_desc = "Reserved"}};

// ──────────────────────────────────────────────────────────────────────────────
// ChargingStatus (0x55)
// ──────────────────────────────────────────────────────────────────────────────
static const bq_bit_desc_t BITS_CHARGING_STATUS[] = {
    {0, 1, .desc = "UT", .long_desc = "Under Temp"},
    {1, 1, .desc = "LT", .long_desc = "Low Temp"},
    {2, 1, .desc = "STL", .long_desc = "Std Low Temp"},
    {3, 1, .desc = "RT", .long_desc = "Recommended Temp"},
    {4, 1, .desc = "ST", .long_desc = "Std High Temp"},
    {5, 1, .desc = "HT", .long_desc = "High Temp"},
    {6, 1, .desc = "OT", .long_desc = "Over Temp"},
    {7, 1, .desc = "PV", .long_desc = "Precharge Voltage"},
    {8, 1, .desc = "LV", .long_desc = "Low Voltage Range"},
    {9, 1, .desc = "MV", .long_desc = "Mid Voltage Range"},
    {10, 1, .desc = "HV", .long_desc = "High Voltage Range"},
    {11, 1, .desc = "IN", .long_desc = "Charge Inhibit"},
    {12, 1, .desc = "SU", .long_desc = "Charge Suspend"},
    {13, 1, .desc = "CCR", .long_desc = "Charging Current Rate"},
    {14, 1, .desc = "CVR", .long_desc = "Charging Voltage Rate"},
    {15, 1, .desc = "CCC", .long_desc = "Charging Current Comp"}};

// ──────────────────────────────────────────────────────────────────────────────
//  GaugingStatus (0x56) - algorithm + QMAX + mode flags
// ──────────────────────────────────────────────────────────────────────────────
static const bq_bit_desc_t BITS_GAUGING_STATUS[] = {
    {0, 1, .desc = "RESTDOD0", .long_desc = "OCV/QMAX Updated"},
    {1, 1, .desc = "DSG", .long_desc = "Discharging Detected"},
    {2, 1, .desc = "RU", .long_desc = "Resistance Update Enabled"},
    {3, 1, .desc = "VOK", .long_desc = "Voltage OK for QMAX"},
    {4, 1, .desc = "QEN", .long_desc = "QMAX Updates Enabled"},
    {5, 1, .desc = "FD", .long_desc = "Fully Discharged detected"},
    {6, 1, .desc = "FC", .long_desc = "Fully Charged detected"},
    {7, 1, .desc = "NSFM", .long_desc = "Negative Scale Factor Mode"},
    {8, 1, .desc = "VDQ", .long_desc = "Qualified Discharge"},
    {9, 1, .desc = "QMAX", .long_desc = "QMAX Updated"},
    {10, 1, .desc = "RX", .long_desc = "Resistance Updated"},
    {11, 1, .desc = "LDMD", .long_desc = "Load Mode (0 = CC, 1 = CP)"},
    {12, 1, .desc = "OCVFR", .long_desc = "OCV in Flat Region"},
    {13, 1, .desc = "TDA", .long_desc = "Terminate Discharge Alarm"},
    {14, 1, .desc = "TCA", .long_desc = "Terminate Charge Alarm"},
    {15, 1, .desc = "LPF", .long_desc = "LiPh Relax (0x400)"}};

// ──────────────────────────────────────────────────────────────────────────────
//  ManufacturingStatus (0x57)
// ──────────────────────────────────────────────────────────────────────────────
static const bq_bit_desc_t BITS_MANUFACTURING_STATUS[] = {
    {0, 1, .desc = "PCHG", .long_desc = "Precharge FET"},
    {1, 1, .desc = "CHG", .long_desc = "Charge FET"},
    {2, 1, .desc = "DSG", .long_desc = "Discharge FET"},
    {3, 1, .desc = "GAUGE", .long_desc = "Gauging"},
    {4, 1, .desc = "FET", .long_desc = "FET Action"},
    {5, 1, .desc = "LF", .long_desc = "Lifetime Data"},
    {6, 1, .desc = "PF", .long_desc = "Permanent Fail"},
    {7, 1, .desc = "BBR", .long_desc = "Black Box Recorder"},
    {8, 1, .desc = "FUSE", .long_desc = "Fuse Action"},
    {9, 1, .desc = "RSVD9", .long_desc = "Reserved"},
    {10, 1, .desc = "RSVD10", .long_desc = "Reserved"},
    {11, 1, .desc = "RSVD11", .long_desc = "Reserved"},
    {12, 1, .desc = "RSVD12", .long_desc = "Reserved"},
    {13, 1, .desc = "RSVD13", .long_desc = "Reserved"},
    {14, 1, .desc = "RSVD14", .long_desc = "Reserved"},
    {15, 1, .desc = "CAL", .long_desc = "Cal Mode ADC/CC"}};

// ──────────────────────────────────────────────────────────────────────────────
//  BatteryStatus (0x16) - alarms, state flags, and error code
// ──────────────────────────────────────────────────────────────────────────────
static const bq_bit_desc_t BITS_BATTERY_STATUS[] = {
    {0, 4, .desc = "ERR", .long_desc = "Error Code (0: OK, 1: Busy, 2: Reserved command, 3: Unsupported command, 4: Access denied, 5: Over/underflow, 6: Bad size, 7: Unknown)"},
    {4, 1, .desc = "FD", .long_desc = "Fully Discharged"},
    {5, 1, .desc = "FC", .long_desc = "Fully Charged"},
    {6, 1, .desc = "DSG", .long_desc = "Discharging"},
    {7, 1, .desc = "INIT", .long_desc = "Initialization Active"},
    {8, 1, .desc = "RTA", .long_desc = "Remaining Time Alarm"},
    {9, 1, .desc = "RCA", .long_desc = "Remaining Capacity Alarm"},
    {10, 1, .desc = "RSVD10", .long_desc = "Reserved"},
    {11, 1, .desc = "TDA", .long_desc = "Terminate Discharge Alarm"},
    {12, 1, .desc = "OTA", .long_desc = "Overtemperature Alarm"},
    {13, 1, .desc = "RSVD13", .long_desc = "Reserved"},
    {14, 1, .desc = "TCA", .long_desc = "Terminate Charge Alarm"},
    {15, 1, .desc = "OCA", .long_desc = "Overcharged Alarm"}};

static const bq_entry bq_commands[] = {
    {BQ30Z555_CMD_SERIAL_NUMBER, "SerialNumber", "", 0.0f, 1.0f, .type = BQ30Z555_TYPE_WORD_INTEGER},
    {BQ30Z555_CMD_MANUFACTURER_NAME, "ManufacturerName", "", 0.0f, 1.0f, .type = BQ30Z555_TYPE_BLOCK_ASCII},
    {BQ30Z555_CMD_DEVICE_NAME, "DeviceName", "", 0.0f, 1.0f, .type = BQ30Z555_TYPE_BLOCK_ASCII},
    {BQ30Z555_CMD_DEVICE_CHEMISTRY, "DeviceChemistry", "", 0.0f, 1.0f, .type = BQ30Z555_TYPE_BLOCK_ASCII},
    {BQ30Z555_CMD_MANUFACTURER_DATA, "ManufacturerData", "", 0.0f, 1.0f, .type = BQ30Z555_TYPE_BLOCK_ASCII},
    {BQ30Z555_CMD_MANUFACTURER_DATE, "ManufacturerDate", "", 0.0f, 1.0f, .type = BQ30Z555_TYPE_WORD_HEX},
    {BQ30Z555_CMD_VOLTAGE, "Voltage", "V", 0.0f, 0.001f, .type = BQ30Z555_TYPE_WORD_FLOAT},            // mV → V
    {BQ30Z555_CMD_TEMPERATURE, "Temperature", "°C", -273.15f, 0.1f, .type = BQ30Z555_TYPE_WORD_FLOAT}, // 0.1 K → °C
    {BQ30Z555_CMD_CURRENT, "Current", "A", 0.0f, 0.001f, .type = BQ30Z555_TYPE_WORD_FLOAT},
    {BQ30Z555_CMD_CELL_VOLTAGE1, "Cell1Voltage", "V", 0.0f, 0.001f, .type = BQ30Z555_TYPE_WORD_FLOAT},
    {BQ30Z555_CMD_CELL_VOLTAGE2, "Cell2Voltage", "V", 0.0f, 0.001f, .type = BQ30Z555_TYPE_WORD_FLOAT},
    {BQ30Z555_CMD_CELL_VOLTAGE3, "Cell3Voltage", "V", 0.0f, 0.001f, .type = BQ30Z555_TYPE_WORD_FLOAT},
    {BQ30Z555_CMD_CELL_VOLTAGE4, "Cell4Voltage", "V", 0.0f, 0.001f, .type = BQ30Z555_TYPE_WORD_FLOAT},
    {BQ30Z555_CMD_CYCLE_COUNT, "CycleCount", "cycles", 0.0f, 1.0f, .type = BQ30Z555_TYPE_WORD_INTEGER},
    {BQ30Z555_CMD_CHARGING_VOLTAGE, "ChargingVoltage", "V", 0.0f, 0.001f, .type = BQ30Z555_TYPE_WORD_FLOAT},
    {BQ30Z555_CMD_DESIGN_VOLTAGE, "DesignVoltage", "V", 0.0f, 0.001f, .type = BQ30Z555_TYPE_WORD_FLOAT},
    {BQ30Z555_CMD_MIN_SYS_V, "MinSystemVoltage", "V", 0.0f, 0.001f, .type = BQ30Z555_TYPE_WORD_FLOAT},
    {BQ30Z555_CMD_AVERAGE_CURRENT, "AverageCurrent", "A", 0.0f, 0.001f, .type = BQ30Z555_TYPE_WORD_FLOAT},
    {BQ30Z555_CMD_CHARGING_CURRENT, "ChargingCurrent", "A", 0.0f, 0.001f, .type = BQ30Z555_TYPE_WORD_FLOAT},
    {BQ30Z555_CMD_TURBO_CURRENT, "TurboCurrent", "A", 0.0f, 0.001f, .type = BQ30Z555_TYPE_WORD_FLOAT},
    {BQ30Z555_CMD_RELATIVE_STATE_OF_CHARGE, "RelativeSoC", "%", 0.0f, 1.0f, .type = BQ30Z555_TYPE_WORD_INTEGER},
    {BQ30Z555_CMD_ABSOLUTE_STATE_OF_CHARGE, "AbsoluteSoC", "%", 0.0f, 1.0f, .type = BQ30Z555_TYPE_WORD_INTEGER},
    {BQ30Z555_CMD_STATE_OF_HEALTH, "State of Health", "%", 0.0f, 1.0f, .type = BQ30Z555_TYPE_WORD_INTEGER},
    {BQ30Z555_CMD_REMAINING_CAPACITY, "RemainingCapacity", "mAh", 0.0f, 1.0f, .type = BQ30Z555_TYPE_WORD_INTEGER},
    {BQ30Z555_CMD_FULL_CHARGE_CAPACITY, "FullChargeCapacity", "mAh", 0.0f, 1.0f, .type = BQ30Z555_TYPE_WORD_INTEGER},
    {BQ30Z555_CMD_DESIGN_CAPACITY, "DesignCapacity", "mAh", 0.0f, 1.0f, .type = BQ30Z555_TYPE_WORD_INTEGER},
    {BQ30Z555_CMD_RUN_TIME_TO_EMPTY, "RunTimeToEmpty", "min", 0.0f, 1.0f, .type = BQ30Z555_TYPE_WORD_INTEGER},
    {BQ30Z555_CMD_AVERAGE_TIME_TO_EMPTY, "AvgTimeToEmpty", "min", 0.0f, 1.0f, .type = BQ30Z555_TYPE_WORD_INTEGER},
    {BQ30Z555_CMD_AVERAGE_TIME_TO_FULL, "AvgTimeToFull", "min", 0.0f, 1.0f, .type = BQ30Z555_TYPE_WORD_INTEGER},
    {BQ30Z555_CMD_BATTERY_STATUS, "BatteryStatus", "", .type = BQ30Z555_TYPE_BLOCK_BITS, .bits = BITS_BATTERY_STATUS, .bits_count = COUNT(BITS_BATTERY_STATUS)},
    {BQ30Z555_CMD_SAFETY_ALERT, "SafetyAlert", "", .type = BQ30Z555_TYPE_BLOCK_BITS, .bits = SAFETY_ALERT_BITS, .bits_count = COUNT(SAFETY_ALERT_BITS)},
    {BQ30Z555_CMD_SAFETY_STATUS, "SafetyStatus", .type = BQ30Z555_TYPE_BLOCK_BITS, .bits = BITS_SAFETY_STATUS, .bits_count = COUNT(BITS_SAFETY_STATUS)},
    {BQ30Z555_CMD_PF_ALERT, "PFAlert", .type = BQ30Z555_TYPE_BLOCK_BITS, .bits = BITS_PF_ALERT, .bits_count = COUNT(BITS_PF_ALERT)},
    {BQ30Z555_CMD_PF_STATUS, "PFStatus", .type = BQ30Z555_TYPE_BLOCK_BITS, .bits = BITS_PF_STATUS, .bits_count = COUNT(BITS_PF_STATUS)},
    {BQ30Z555_CMD_OPERATION_STATUS, "OperationStatus", .type = BQ30Z555_TYPE_BLOCK_BITS, .bits = BITS_OPERATION_STATUS, .bits_count = COUNT(BITS_OPERATION_STATUS)},
    {BQ30Z555_CMD_CHARGING_STATUS, "ChargingStatus", .type = BQ30Z555_TYPE_BLOCK_BITS, .bits = BITS_CHARGING_STATUS, .bits_count = COUNT(BITS_CHARGING_STATUS)},
    {BQ30Z555_CMD_GAUGING_STATUS, "GaugingStatus", .type = BQ30Z555_TYPE_BLOCK_BITS, .bits = BITS_GAUGING_STATUS, .bits_count = COUNT(BITS_GAUGING_STATUS)},
    {BQ30Z555_CMD_MANUFACTURING_STATUS, "ManufacturingStatus", .type = BQ30Z555_TYPE_BLOCK_BITS, .bits = BITS_MANUFACTURING_STATUS, .bits_count = COUNT(BITS_MANUFACTURING_STATUS)},

};

// ──────────────────────────────────────────────────────────────────────────────
//  Bit-extraction helper
// ──────────────────────────────────────────────────────────────────────────────
/**
 * Extract an arbitrary bit-field spanning one or more bytes.
 *
 * @param data       Buffer with little-endian byte order (LSB = byte[0]).
 * @param len        Length of buffer.
 * @param lsb_index  Index of least-significant bit to extract (0 = bit0 of byte0).
 * @param width      Width of field in bits (1-32 supported).
 * @return           Extracted value right-aligned (bit0 = LSB of return).
 */
static uint32_t bq_extract_bits(const uint8_t *data, size_t len, uint16_t lsb_index, uint8_t width)
{
    uint32_t value = 0;
    for (uint8_t i = 0; i < width; ++i)
    {
        uint16_t bit_idx = lsb_index + i;
        size_t byte_idx = bit_idx >> 3; // /8
        if (byte_idx >= len)
            break;                            // out-of-range safety
        uint8_t bit_in_byte = bit_idx & 0x07; // %8
        if (data[byte_idx] & (1u << bit_in_byte))
        {
            value |= (1u << i); // place into result
        }
    }
    return value;
}

// ──────────────────────────────────────────────────────────────────────────────
//  Generic bit-field printer (buffer-aware, arbitrary size)
// ──────────────────────────────────────────────────────────────────────────────
static int bq_print_bits_from_buffer(const bq_entry *e,
                                     const uint8_t *data, size_t data_len)
{
    if (!e || !data || data_len == 0 || e->type != BQ30Z555_TYPE_BLOCK_BITS)
    {
        return ESP_ERR_INVALID_ARG;
    }

    printf("%s:\n", e->name);
    for (size_t i = 0; i < e->bits_count; ++i)
    {
        const bq_bit_desc_t *d = &e->bits[i];
        uint32_t field = bq_extract_bits(data, data_len, d->bit, d->width);
        if (d->width == 1)
        {
            printf("  %s%10s%s [%s] \033[90m(%s)\033[0m\n",
                   field ? "\033[32m" : "",
                   d->desc,
                   field ? "\033[0m" : "",
                   field ? "\033[32mX\033[0m" : " ",
                   d->long_desc ? d->long_desc : "");
        }
        else
        {
            printf("  %10s [\033[32m%" PRIu32 "\033[0m] \033[90m(%s)\033[0m\n",
                   d->desc,
                   field,
                   d->long_desc ? d->long_desc : "");
        }
    }
    return 0;
}

/**
 * @brief Fetch an SBS WORD and print it.
 *
 * Prints a single line:   "<name>: <value> <unit>".
 * Returns 0 on success or the I²C error code.
 */
int bq_generic_dump(const bq_entry *entry)
{
    if (!entry)
        return ESP_ERR_INVALID_ARG;

    uint8_t cmd = entry->reg;

    switch (entry->type)
    {
    case BQ30Z555_TYPE_BLOCK_BITS:
    {
        uint8_t resp_len[1] = {0};
        int err = i2c_write_read(BQ30Z555_I2C_ADDR, &cmd, sizeof(cmd), resp_len, sizeof(resp_len));
        if (err)
        {
            ESP_LOGE(TAG, "%s: i2c_write_read failed (err=%d)", entry->name, err);
            return err;
        }

        uint8_t len = resp_len[0];
        uint8_t resp_data[256];
        err = i2c_write_read(BQ30Z555_I2C_ADDR, &cmd, sizeof(cmd), resp_data, 1 + len);
        if (err)
        {
            ESP_LOGE(TAG, "%s: i2c_write_read failed (err=%d)", entry->name, err);
            return err;
        }

        bq_print_bits_from_buffer(entry, &resp_data[1], len);

        break;
    }
    case BQ30Z555_TYPE_BLOCK_ASCII:
    case BQ30Z555_TYPE_BLOCK_HEX:
    {
        uint8_t resp_len[1] = {0};
        int err = i2c_write_read(BQ30Z555_I2C_ADDR, &cmd, sizeof(cmd), resp_len, sizeof(resp_len));
        if (err)
        {
            ESP_LOGE(TAG, "%s: i2c_write_read failed (err=%d)", entry->name, err);
            return err;
        }

        uint8_t len = resp_len[0];
        uint8_t resp_data[256];
        err = i2c_write_read(BQ30Z555_I2C_ADDR, &cmd, sizeof(cmd), resp_data, 1 + len);
        if (err)
        {
            ESP_LOGE(TAG, "%s: i2c_write_read failed (err=%d)", entry->name, err);
            return err;
        }

        for (int pos = 0; pos < len; pos++)
        {
            if (resp_data[1 + pos] < 0x20 || resp_data[1 + pos] >= 0x80)
            {
                resp_data[1 + pos] = '.';
            }
        }
        switch (entry->type)
        {
        case BQ30Z555_TYPE_BLOCK_ASCII:
        {
            printf("%-32s: '%.*s' %s\n", entry->name, len, &resp_data[1], entry->unit);
            break;
        }
        case BQ30Z555_TYPE_BLOCK_HEX:
        {
            printf("%-32s: '", entry->name);
            for (int pos = 0; pos < len; pos++)
            {
                char tmp_buf[4];
                sprintf(tmp_buf, "%02X ", resp_data[1 + pos]);
                printf("%s", tmp_buf);
            }
            printf("' %s\n", entry->unit);
            break;
        }
        default:
            break;
        }

        break;
    }
    case BQ30Z555_TYPE_WORD_HEX:
    case BQ30Z555_TYPE_WORD_FLOAT:
    case BQ30Z555_TYPE_WORD_INTEGER:
    {
        uint8_t resp[2] = {0};
        int err = i2c_write_read(BQ30Z555_I2C_ADDR, &cmd, sizeof(cmd), resp, sizeof(resp));
        if (err)
        {
            ESP_LOGE(TAG, "%s: i2c_write_read failed (err=%d)", entry->name, err);
            return err;
        }

        uint16_t raw = (uint16_t)resp[0] | ((uint16_t)resp[1] << 8);

        switch (entry->type)
        {
        case BQ30Z555_TYPE_WORD_HEX:
        {
            printf("%-32s: 0x%08X %s\n", entry->name, raw, entry->unit);
            break;
        }
        case BQ30Z555_TYPE_WORD_FLOAT:
        {
            float val = raw * entry->scaling + entry->offset;
            printf("%-32s: %02.3f %s\n", entry->name, val, entry->unit);
            break;
        }
        case BQ30Z555_TYPE_WORD_INTEGER:
        {
            float val = raw * entry->scaling + entry->offset;
            printf("%-32s: %d %s\n", entry->name, (int)val, entry->unit);
            break;
        }
        default:
            break;
        }
        break;
    }
    default:
        break;
    }
    return 0;
}

/**
 * @brief Decode and print Lifetime Data Block n (n = 1‥3).
 *
 * Mapping – **only fields that are 16-bit voltages or currents** are interpreted.
 * Everything else is shown as raw hex words to keep the output compact yet
 * useful.
 *
 *  Block 1 (0x60)
 *  ────────────────
 *   Word Idx | Description                 | Unit
 *   ---------|-----------------------------|------
 *        0-3 | Max Cell Voltage 1-4        | mV
 *        4-7 | Min Cell Voltage 1-4        | mV
 *          8 | Max Delta Cell Voltage      | mV
 *          9 | Max Charge Current          | mA
 *         10 | Max Discharge Current       | mA
 *         11 | Max Avg Discharge Current   | mA
 *
 *  Block 2 (0x61) - no voltage/current *word* fields; printed raw.
 *  Block 3 (0x62) - time counters only; printed raw.
 */
int bq_print_lifetime_block_decoded(int n)
{
    if (n < 1 || n > 3)
    {
        ESP_LOGE(TAG, "LifetimeData block index %d out of range (1..3)", n);
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t cmd = (uint8_t)(BQ30Z555_CMD_LIFETIME_DATA1 + (n - 1));

    uint8_t resp_len[1] = {0};
    int err = i2c_write_read(BQ30Z555_I2C_ADDR, &cmd, sizeof(cmd), resp_len, sizeof(resp_len));
    if (err)
    {
        ESP_LOGE(TAG, "LifetimeData%d: i2c I/O err %d", n, err);
        return err;
    }

    uint8_t len = resp_len[0];

    uint8_t resp[256] = {0};

    err = i2c_write_read(BQ30Z555_I2C_ADDR, &cmd, sizeof(cmd), resp, 1 + len);
    if (err)
    {
        ESP_LOGE(TAG, "LifetimeData%d: i2c I/O err %d", n, err);
        return err;
    }

    if (n == 1)
    {
        int offset = 1;
        puts("LifetimeData1 decoded (voltages in V, currents in A):");
        for (int i = 0; i < 4; ++i)
        {
            float v = le16(&resp[offset]) / 1000.0f; // mV → V
            printf("  Max Cell Voltage  %d: %.3f V\n", i + 1, v);
            offset += 2;
        }
        for (int i = 0; i < 4; ++i)
        {
            float v = le16(&resp[offset]) / 1000.0f;
            printf("  Min Cell Voltage  %d: %.3f V\n", i + 1, v);
            offset += 2;
        }
        printf("  Max Δ Cell Voltage : %.3f V\n", le16(&resp[offset]) / 1000.0f);
        offset += 2;
        printf("  Max Charge Current : %.3f A\n", le16(&resp[offset]) / 1000.0f);
        offset += 2;
        printf("  Max Disch  Current : %.3f A\n", le16(&resp[offset]) / 1000.0f);
        offset += 2;
        printf("  Max Avg   Current  : %.3f A\n", le16(&resp[offset]) / 1000.0f);
        offset += 2;
        printf("  Max Avg Disch Power: %d W\n", resp[offset]);
        offset += 1;
        return 0;
    }

    // Blocks 2 & 3: show raw words for reference
    printf("LifetimeData%d raw words:\n", n);
    for (int i = 0; i < len; ++i)
    {
        printf("  0x%02x: 0x%02X\n", i, resp[1 + i]);
    }
    return 0;
}

// ──────────────────────────────────────────────────────────────────────────────
//  Voltage command implementation
// ──────────────────────────────────────────────────────────────────────────────

/**
 * @brief Read pack voltage from the BQ30Z555.
 *
 * The SBS Voltage() command returns a *word* (little-endian 16 bit) that
 * represents the pack voltage in millivolts. We issue a single-byte write of
 * `0x09`, then read back two bytes and assemble them into a host-endian `uint16_t`.
 */
static int cmd_bq_dump(int argc, char **argv)
{
    (void)argc;
    (void)argv; // Unused

    for (int pos = 0; pos < COUNT(bq_commands); pos++)
    {
        bq_generic_dump(&bq_commands[pos]);
    }

    return 0;
}
static int cmd_bq_lifetime(int argc, char **argv)
{
    int block = 1; // default
    if (argc == 2)
    {
        block = atoi(argv[1]);
    }
    return bq_print_lifetime_block_decoded(block);
}
// ──────────────────────────────────────────────────────────────────────────────
//  Command registration helper
// ──────────────────────────────────────────────────────────────────────────────

void register_bq_commands(void)
{
    const esp_console_cmd_t dump_cmd = {
        .command = "bq_show",
        .help = "Read all known fields",
        .hint = NULL,
        .func = &cmd_bq_dump,
        .argtable = NULL,
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&dump_cmd));
    const esp_console_cmd_t lifetime_cmd = {
        .command = "bq_lifetime",
        .help = "Show Lifetime Data block 1-3 (default 1)",
        .hint = NULL,
        .func = &cmd_bq_lifetime,
        .argtable = NULL, // simple argv parsing
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&lifetime_cmd));
}

void bq_start(void)
{
    register_bq_commands();
}
