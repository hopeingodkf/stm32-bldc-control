#ifndef CRITICAL_H
#define CRITICAL_H

#include "main.h"

/*
 * Критична секція зі збереженням попереднього стану переривань.
 *
 * Пара __disable_irq() / __enable_irq() вмикає переривання на виході
 * беззастережно. Якщо секцію викликано з місця, де вони вже були вимкнені,
 * вихід із неї увімкне їх передчасно. Ці макроси запам'ятовують PRIMASK на
 * вході й відновлюють його на виході.
 */

#define CRITICAL_ENTER()   uint32_t __saved_primask = __get_PRIMASK(); __disable_irq()
#define CRITICAL_EXIT()    __set_PRIMASK(__saved_primask)

#endif
