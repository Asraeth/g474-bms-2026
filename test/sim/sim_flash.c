/*
 * sim_flash.c
 *
 * The last 4 KB of the STM32G474RE's flash (the runtime settings area) and the HAL calls that erase and
 * program it. Dual-bank mode by default, like the chip from the factory: 2 KB pages, bank 2 from
 * 0x08040000. Erasing takes ~20 ms of simulated time with interrupts running (bank 2 is erased while the
 * program runs from bank 1). Anything outside the settings area fails the test.
 */

#include "sim.h"

#include <string.h>

FLASH_TypeDef sim_FLASH;

#define AREA_START      0x0807F000u
#define AREA_SIZE       0x1000u
#define BANK2_START     0x08040000u
#define ERASE_TIME_US   20000

static uint8_t  area[AREA_SIZE];
static bool     initialised = false;
static bool     unlocked = false;
static bool     writeError = false;
static int      erases = 0;

static void init_once(void)
{
    if (!initialised)
    {
        memset(area, 0xFF, sizeof(area));
        sim_FLASH.OPTR |= FLASH_OPTR_DBANK;
        initialised = true;
    }
}

const void* sim_flash_ptr(uint32_t address)
{
    init_once();
    if (address < AREA_START || address >= AREA_START + AREA_SIZE)
    {
        sim_fail("flash read at 0x%08lX, outside the settings area", (unsigned long)address);
        return area;
    }
    return &area[address - AREA_START];
}

HAL_StatusTypeDef HAL_FLASH_Unlock(void) { init_once(); unlocked = true; return HAL_OK; }
HAL_StatusTypeDef HAL_FLASH_Lock(void)   { unlocked = false; return HAL_OK; }

HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef* erase, uint32_t* pageError)
{
    init_once();
    *pageError = 0xFFFFFFFFu;
    if (!unlocked)
    {
        sim_fail("flash erase while locked");
        return HAL_ERROR;
    }

    bool dual = (sim_FLASH.OPTR & FLASH_OPTR_DBANK) != 0;
    uint32_t pageSize = dual ? 0x800u : 0x1000u;
    for (uint32_t i = 0; i < erase->NbPages; i++)
    {
        uint32_t page = erase->Page + i;
        uint32_t start;
        if (dual)
        {
            start = (erase->Banks == FLASH_BANK_2 ? BANK2_START : FLASH_BASE) + page * pageSize;
        }
        else
        {
            start = FLASH_BASE + page * pageSize;
        }
        if (erase->TypeErase != FLASH_TYPEERASE_PAGES || start < AREA_START || start + pageSize > AREA_START + AREA_SIZE)
        {
            sim_fail("flash erase of bank %lu page %lu (0x%08lX), outside the settings area",
                     (unsigned long)erase->Banks, (unsigned long)page, (unsigned long)start);
            *pageError = page;
            return HAL_ERROR;
        }
        memset(&area[start - AREA_START], 0xFF, pageSize);
        erases++;
    }
    sim_advance_us(ERASE_TIME_US);
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FLASH_Program(uint32_t typeProgram, uint32_t address, uint64_t data)
{
    init_once();
    if (!unlocked || typeProgram != FLASH_TYPEPROGRAM_DOUBLEWORD || address % 8 != 0 ||
        address < AREA_START || address + 8 > AREA_START + AREA_SIZE)
    {
        sim_fail("flash program at 0x%08lX (type %lu, %s)", (unsigned long)address, (unsigned long)typeProgram,
                 unlocked ? "unlocked" : "locked");
        return HAL_ERROR;
    }
    uint8_t* p = &area[address - AREA_START];
    for (int i = 0; i < 8; i++)
    {
        if (p[i] != 0xFF)
        {
            return HAL_ERROR;           // PROGERR: double word not erased
        }
    }
    if (writeError)
    {
        return HAL_ERROR;
    }
    memcpy(p, &data, 8);
    sim_advance_us(90);
    return HAL_OK;
}

void sim_flash_set_dual_bank(bool dual)
{
    init_once();
    if (dual) sim_FLASH.OPTR |= FLASH_OPTR_DBANK;
    else      sim_FLASH.OPTR &= ~FLASH_OPTR_DBANK;
}

void sim_flash_corrupt(uint32_t address)
{
    init_once();
    area[address - AREA_START] ^= 0x01;
}

void sim_flash_set_write_error(bool error) { writeError = error; }
int  sim_flash_erases(void)                { return erases; }
