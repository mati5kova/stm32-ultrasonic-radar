#include "redirect.h"

int _write(int file, char* ptr, int len)
{
    (void)file;
    for (int i = 0; i < len; i++)
    {
        // cakaj dokler transmit buffer ni prazen
        while (!(USART3->ISR & USART_ISR_TXE_TXFNF))
        {
        }
        USART3->TDR = (uint8_t)ptr[i]; // zapisi character v transmit data register
    }

    // cakamo dokler se celotna vsebina ne poslje
    while (!(USART3->ISR & USART_ISR_TC))
    {
    }

    USART3->ICR = USART_ICR_TCCF; // v interrupt clear register zapisemo transmition complete flag

    return len;
}
