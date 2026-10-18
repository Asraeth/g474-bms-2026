#include "bms_command_counter.h"

BMS_CounterCommand BMS_CC_Classify(const uint8_t command[2])
{
    uint16_t code = ((uint16_t)command[0] << 8) | command[1];
    /* ADCV, ADSV, ADAX and ADAX2: mask only the option bits in Table 50.
     * Poll commands also increment, once per command, not per polled byte. */
    if ((code & 0xFE68U) == 0x0260U || (code & 0xFF6CU) == 0x0168U
            || (code & 0xFE30U) == 0x0410U || (code & 0xFFF0U) == 0x0400U)
        return BMS_CC_INCREMENT;
    switch (code) {
    case 0x002E: case 0x0027: return BMS_CC_RESET; // RSTCC / SRST
    case 0x0001: case 0x0024: // WRCFGA/B
    case 0x0020: case 0x0021: // WRPWMA/B
    case 0x0717: case 0x0715: // CLRFLAG / CLOVUV require data PEC
        return BMS_CC_WRITE;
    case 0x0028: case 0x0029: case 0x002D: case 0x002F: // MUTE/UNMUTE/SNAP/UNSNAP
    case 0x0711: case 0x0712: case 0x0714: case 0x0716: // clear result registers
    case 0x0718: case 0x071C: case 0x071D: case 0x071E: case 0x071F:
        return BMS_CC_INCREMENT;
    case 0x0002: case 0x0026: case 0x0022: case 0x0023: case 0x002C:
    case 0x0004: case 0x0006: case 0x0008: case 0x000A: case 0x0009: case 0x000B:
    case 0x0044: case 0x0046: case 0x0048: case 0x004A: case 0x0049: case 0x004B:
    case 0x0003: case 0x0005: case 0x0007: case 0x000D: case 0x000E: case 0x000F:
    case 0x0012: case 0x0013: case 0x0014: case 0x0015: case 0x0016: case 0x0017:
    case 0x0019: case 0x001A: case 0x001B: case 0x001F:
    case 0x001C: case 0x001D: case 0x001E: case 0x0025:
    case 0x0030: case 0x0031: case 0x0032: case 0x0072: case 0x0033: case 0x0034:
        return BMS_CC_READ;
    default:
        // Fail closed for unused legacy/unknown opcodes, LPCM, retention,
        // COMM and variable-length read-all protocols. Add with protocol tests.
        return BMS_CC_UNSUPPORTED;
    }
}

void BMS_CC_Reset(BMS_CommandCounterChain *chain)
{
    for (unsigned int i = 0; i < TOTAL_IC; i++)
        chain->device[i] = (BMS_CommandCounterDevice){.synchronized = true};
}

void BMS_CC_Advance(BMS_CommandCounterChain *chain)
{
    for (unsigned int i = 0; i < TOTAL_IC; i++) {
        BMS_CommandCounterDevice *device = &chain->device[i];
        if (device->synchronized)
            device->expected = device->expected == 63 ? 1 : device->expected + 1;
    }
}

void BMS_CC_Invalidate(BMS_CommandCounterChain *chain)
{
    // A failed transmit might have reached only part of the chain. Guessing
    // whether it was accepted would permit stale data or create false faults.
    for (unsigned int i = 0; i < TOTAL_IC; i++) {
        chain->device[i].synchronized = false;
        chain->device[i].fault = true;
    }
}

bool BMS_CC_Check(BMS_CommandCounterChain *chain, const uint8_t received[TOTAL_IC],
        const bool pec_errors[TOTAL_IC], bool errors[TOTAL_IC])
{
    bool any = false;
    for (unsigned int i = 0; i < TOTAL_IC; i++) {
        BMS_CommandCounterDevice *device = &chain->device[i];
        bool pec_error = pec_errors[i]; // Allow input/output arrays to alias.
        if (!pec_error) {
            device->received = received[i];
            device->received_valid = true;
            if (!device->synchronized || received[i] != device->expected)
                device->fault = true;
        }
        errors[i] = pec_error || device->fault;
        any |= errors[i];
    }
    return any;
}
