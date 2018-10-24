#include <pbdrv/time.h>

void mp_hal_set_interrupt_char(int c);
#define mp_hal_ticks_ms pbdrv_time_get_msec
#define mp_hal_ticks_us pbdrv_time_get_usec
#define mp_hal_delay_us pbdrv_time_delay_usec
#define mp_hal_ticks_cpu() 0
