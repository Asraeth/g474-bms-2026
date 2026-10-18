#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "bms_command_counter.h"

static void classify(uint16_t code, BMS_CounterCommand expected)
{
    uint8_t command[2] = {code >> 8, code};
    assert(BMS_CC_Classify(command) == expected);
}

int main(void)
{
    const uint16_t reads[] = {0x2,0x26,0x22,0x23,0x2C,0x4,0x6,0x8,0xA,0x9,0xB,
        0x44,0x46,0x48,0x4A,0x49,0x4B,0x3,0x5,0x7,0xD,0xE,0xF,
        0x12,0x13,0x14,0x15,0x16,0x17,0x19,0x1A,0x1B,0x1F,
        0x1C,0x1D,0x1E,0x25,0x30,0x31,0x32,0x72,0x33,0x34};
    const uint16_t increments[] = {0x28,0x29,0x2D,0x2F,0x711,0x712,0x714,0x716,
        0x718,0x71C,0x71D,0x71E,0x71F};
    const uint16_t writes[] = {0x1,0x24,0x20,0x21,0x717,0x715};
    for (unsigned int i = 0; i < sizeof reads / sizeof *reads; i++) classify(reads[i], BMS_CC_READ);
    for (unsigned int i = 0; i < sizeof increments / sizeof *increments; i++) classify(increments[i], BMS_CC_INCREMENT);
    for (unsigned int i = 0; i < sizeof writes / sizeof *writes; i++) classify(writes[i], BMS_CC_WRITE);
    classify(0x27, BMS_CC_RESET); classify(0x2E, BMS_CC_RESET);
    // Enumerate every option-bit combination of the four conversion commands.
    const uint16_t fixed[] = {0x260,0x168,0x410,0x400};
    const uint16_t options[] = {0x197,0x093,0x1CF,0x00F};
    for (unsigned int family = 0; family < 4; family++)
        for (unsigned int bits = 0; bits <= options[family]; bits++)
            if (!(bits & ~options[family])) classify(fixed[family] | bits, BMS_CC_INCREMENT);
    classify(0, BMS_CC_UNSUPPORTED); classify(0xFFFF, BMS_CC_UNSUPPORTED);
    classify(0x719, BMS_CC_UNSUPPORTED); classify(0xC, BMS_CC_UNSUPPORTED);
    puts("PASS datasheet INC classification: all supported reads/writes/polls/resets, conversion options and unsupported opcode rejection");

    BMS_CommandCounterChain chain = {0}, other = {0};
    uint8_t received[TOTAL_IC] = {0}; bool pec[TOTAL_IC] = {false}, errors[TOTAL_IC];
    assert(BMS_CC_Check(&chain, received, pec, errors)); // No arbitrary first-frame synchronization.
    BMS_CC_Reset(&chain);
    assert(!BMS_CC_Check(&chain, received, pec, errors));
    assert(!BMS_CC_Check(&chain, received, pec, errors)); // Reads legally repeat.
    for (int count = 1; count <= 63; count++) {
        BMS_CC_Advance(&chain); memset(received, count, sizeof received);
        assert(!BMS_CC_Check(&chain, received, pec, errors));
    }
    BMS_CC_Advance(&chain); memset(received, 1, sizeof received);
    assert(!BMS_CC_Check(&chain, received, pec, errors)); // 63 -> 1, never 0.
    puts("PASS normal per-device progression, unchanged counters across reads and 63-to-1 rollover");

    BMS_CC_Advance(&chain); memset(received, 2, sizeof received); received[13] = 1;
    assert(BMS_CC_Check(&chain, received, pec, errors) && errors[13] && !errors[0]);
    received[13] = 2;
    assert(BMS_CC_Check(&chain, received, pec, errors) && errors[13]); // Latches.
    BMS_CC_Reset(&chain); BMS_CC_Advance(&chain); memset(received, 1, sizeof received);
    received[7] = 2;
    assert(BMS_CC_Check(&chain, received, pec, errors) && errors[7] && !errors[13]);
    BMS_CC_Reset(&chain); BMS_CC_Advance(&chain); received[7] = 37;
    assert(BMS_CC_Check(&chain, received, pec, errors) && errors[7]);
    puts("PASS duplicate after an INC command, skipped/unexpected counters, per-device localization and fault latching");

    BMS_CC_Reset(&chain); BMS_CC_Advance(&chain); memset(received, 1, sizeof received);
    received[0] = 0; assert(BMS_CC_Check(&chain, received, pec, errors)); // Unexpected device reset/sleep.
    BMS_CC_Reset(&chain); memset(received, 0, sizeof received);
    assert(!BMS_CC_Check(&chain, received, pec, errors));
    BMS_CC_Invalidate(&chain); BMS_CC_Advance(&chain);
    assert(BMS_CC_Check(&chain, received, pec, errors) && !chain.device[0].synchronized);
    BMS_CC_Reset(&chain); assert(!BMS_CC_Check(&chain, received, pec, errors));
    BMS_CC_Reset(&other); BMS_CC_Advance(&other);
    assert(chain.device[0].expected == 0 && other.device[0].expected == 1);
    puts("PASS explicit reset/recovery without false sequence faults, ambiguous-send desynchronization and independent chain instances");

    BMS_CC_Advance(&chain); memset(received, 1, sizeof received);
    pec[4] = true; received[4] = 37;
    assert(BMS_CC_Check(&chain, received, pec, errors) && errors[4]);
    assert(chain.device[4].received == 0 && !chain.device[4].fault);
    pec[4] = false; received[4] = 1;
    assert(!BMS_CC_Check(&chain, received, pec, errors));
    received[4] = 64; assert(BMS_CC_Check(&chain, received, pec, errors));
    puts("PASS PEC-first counter trust, later valid read and out-of-range counter rejection");
    puts("ALL ADBMS COMMAND COUNTER TESTS PASSED");
    return 0;
}
