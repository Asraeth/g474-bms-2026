/*
 * uartDMA.c
 *
 * printf to the debug UART (LPUART1, 1 Mbaud) without waiting: messages go into a ring buffer, and DMA sends
 * it in the background, one contiguous run at a time (up to the end of the buffer, then from its start).
 */

#include "uartDMA.h"
#include "main.h"

#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>

#define uartHandle      hlpuart1
#define BUFFER_SIZE     10000
#define MESSAGE_MAX     256

static char         buffer[BUFFER_SIZE];
static volatile int head = 0;           // Next byte written
static volatile int tail = 0;           // Next byte to send
static bool         isWrapped = false;  // head has wrapped to the start, tail hasn't yet
static int          tailDma = 0;        // Where tail moves when the running transfer is done

volatile uint32_t droppedMessages = 0;  // Didn't fit in the buffer


static int getDataLen(void)
{
    int t = tail;
    return head >= t ? head - t : BUFFER_SIZE - t + head;
}


// Sends from tail to head, or to the end of the buffer if head has wrapped. Interrupts off or in the
// DMA interrupt, with the UART idle and data waiting.
static void startUartDmaTx(void)
{
    int dataLen = isWrapped ? BUFFER_SIZE - tail : head - tail;
    tailDma = tail + dataLen;
    HAL_UART_Transmit_DMA(&uartHandle, (uint8_t*)(buffer + tail), (uint16_t)dataLen);
}


// The HAL marks the UART ready just before calling this. Every interrupt has the same priority, so no other
// one can print in between and start a transfer from the old tail.
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
    if (huart == &uartHandle)
    {
        tail = tailDma;
        if (tail == BUFFER_SIZE)
        {
            tail = 0;
            isWrapped = false;
        }
        if (head != tail)       // More was written meanwhile (also when head has wrapped)
        {
            startUartDmaTx();
        }
    }
}


// Safe to call from interrupts, but there without %f / %e / %g: newlib's float formatting allocates and keeps
// state shared with the main loop's printing, which an interrupt could corrupt. Messages are cut at
// MESSAGE_MAX - 1 characters; one that doesn't fit in the buffer is dropped (droppedMessages).
int printfDma(const char *format, ...)
{
    char message[MESSAGE_MAX];
    va_list args;
    va_start(args, format);
    int written = vsnprintf(message, MESSAGE_MAX, format, args);
    va_end(args);

    if (written < 0)
    {
        return -1;
    }
    if (written >= MESSAGE_MAX)
    {
        written = MESSAGE_MAX - 1;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    // Never completely full: head == tail means empty
    if (written >= BUFFER_SIZE - getDataLen())
    {
        droppedMessages++;
        __set_PRIMASK(primask);
        return 0;
    }

    for (int i = 0; i < written; i++)
    {
        buffer[head++] = message[i];
        if (head == BUFFER_SIZE)
        {
            isWrapped = true;
            head = 0;
        }
    }

    if (written > 0 && HAL_UART_GetState(&uartHandle) == HAL_UART_STATE_READY)
    {
        startUartDmaTx();
    }

    __set_PRIMASK(primask);
    return written;
}
