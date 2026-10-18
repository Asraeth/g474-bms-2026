/*
 * uartDMA.h
 *
 * Non-blocking printf to the debug UART, see uartDMA.c.
 */

#ifndef INC_UARTDMA_H_
#define INC_UARTDMA_H_

int printfDma(const char *format, ...) __attribute__((format(printf, 1, 2)));

#endif /* INC_UARTDMA_H_ */
