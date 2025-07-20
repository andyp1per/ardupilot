/*
  MCU tables for STM32H5
 */

#if defined(STM32H5)

// Define device IDs for STM32H5 series
// These values are typically found in the DBGMCU_IDCODE register.
// It's recommended to verify these against the latest ST Microelectronics reference manuals.
#define STM32_UNKNOWN 0
#define STM32_H56x    0x484  // STM32H56x/57x lines
#define STM32_H503    0x472  // STM32H503 line

/**
 * @brief Array of MCU descriptions for the STM32H5 family.
 * Maps the device ID code to a human-readable string.
 */
mcu_des_t mcu_descriptions[] = {
    { STM32_UNKNOWN, "STM32H5???" },
    { STM32_H56x,    "STM32H56x/57x" },
    { STM32_H503,    "STM32H503" },
};

/**
 * @brief Array of known silicon revisions for the STM32H5 family.
 * The revision is read from the REV_ID bits of the DBGMCU_IDCODE register.
 */
const mcu_rev_t silicon_revs[] = {
    // Revision codes for STM32H5 series
    { 0x1000, 'V' },
    { 0x1001, 'B' },
    { 0x1002, 'Z' },
    // Add other revisions as they become available
};

#endif // STM32H5
