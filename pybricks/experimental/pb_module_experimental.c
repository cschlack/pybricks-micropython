// SPDX-License-Identifier: MIT
// Copyright (c) 2018-2021 The Pybricks Authors

#include "py/mpconfig.h"

#if PYBRICKS_PY_EXPERIMENTAL

#include "py/mphal.h"
#include "py/obj.h"
#include "py/runtime.h"

#include <pybricks/util_mp/pb_obj_helper.h>
#include <pybricks/util_mp/pb_kwarg_helper.h>

#include <pybricks/experimental.h>
#include <pybricks/robotics.h>

#include <pbdrv/clock.h>

#include <stm32f4xx_hal_pcd.h>
#include <stm32f4xx_hal_pcd_ex.h>

#if PYBRICKS_HUB_EV3BRICK
#if !MICROPY_MODULE_BUILTIN_INIT
#error "pybricks.experimental module requires that MICROPY_MODULE_BUILTIN_INIT is enabled"
#endif

#include <signal.h>

#include "py/mpthread.h"

STATIC void sighandler(int signum) {
    // we just want the signal to interrupt system calls
}

STATIC mp_obj_t mod_experimental___init__(void) {
    struct sigaction sa;
    sa.sa_flags = 0;
    sa.sa_handler = sighandler;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGUSR2, &sa, NULL);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mod_experimental___init___obj, mod_experimental___init__);

STATIC mp_obj_t mod_experimental_pthread_raise(mp_obj_t thread_id_in, mp_obj_t ex_in) {
    mp_uint_t thread_id = mp_obj_int_get_truncated(thread_id_in);
    if (ex_in != mp_const_none && !mp_obj_is_exception_instance(ex_in)) {
        mp_raise_TypeError(MP_ERROR_TEXT("must be an exception or None"));
    }
    return mp_obj_new_int(mp_thread_schedule_exception(thread_id, ex_in));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(mod_experimental_pthread_raise_obj, mod_experimental_pthread_raise);
#endif // PYBRICKS_HUB_EV3BRICK

void HAL_PCDEx_BCD_VBUSDetectHACKED(PCD_HandleTypeDef *hpcd) {
    USB_OTG_GlobalTypeDef *USBx = hpcd->Instance;
    uint32_t tickstart = pbdrv_clock_get_ms();


    PCD_BCD_MsgTypeDef result = PCD_BCD_DISCOVERY_COMPLETED;

    /* Start BCD When device is connected */

    // HACK: Be lazy and call it in a loop instead of interrupt.
    // But this way we can debug print with ease...

    if (1 /* USBx_DEVICE->DCTL & USB_OTG_DCTL_SDIS */) {
        /* Enable DCD : Data Contact Detect */
        USBx->GCCFG |= USB_OTG_GCCFG_DCDEN;

        /* Wait Detect flag or a timeout is happen*/
        while ((USBx->GCCFG & USB_OTG_GCCFG_DCDET) == 0U) {
            /* Check for the Timeout */
            if ((pbdrv_clock_get_ms() - tickstart) > 1000U) {
                mp_printf(&mp_plat_print, "open cable\n");
                return;
            }
        }

        /* Right response got */
        mp_hal_delay_ms(100U);

        /* Check Detect flag*/
        if (USBx->GCCFG & USB_OTG_GCCFG_DCDET) {
            result = PCD_BCD_CONTACT_DETECTION;
        }
        else {
            mp_printf(&mp_plat_print, "Nothing\n");
            return;
        }

        /*Primary detection: checks if connected to Standard Downstream Port
        (without charging capability) */
        USBx->GCCFG &= ~USB_OTG_GCCFG_DCDEN;
        USBx->GCCFG |= USB_OTG_GCCFG_PDEN;
        mp_hal_delay_ms(100U);

        if (!(USBx->GCCFG & USB_OTG_GCCFG_PDET)) {
            /* Case of Standard Downstream Port */
            result = PCD_BCD_STD_DOWNSTREAM_PORT;
        } else {
            /* start secondary detection to check connection to Charging Downstream
            Port or Dedicated Charging Port */
            USBx->GCCFG &= ~USB_OTG_GCCFG_PDEN;
            USBx->GCCFG |= USB_OTG_GCCFG_SDEN;
            mp_hal_delay_ms(100U);

            if ((USBx->GCCFG) & USB_OTG_GCCFG_SDET) {
                /* case Dedicated Charging Port  */
                result = PCD_BCD_DEDICATED_CHARGING_PORT;
            } else {
                /* case Charging Downstream Port  */
                result = PCD_BCD_CHARGING_DOWNSTREAM_PORT;
            }
        }

        if (result) {
            mp_printf(&mp_plat_print, "Detected: ");
            switch (result)
            {
            case PCD_BCD_STD_DOWNSTREAM_PORT:
                mp_printf(&mp_plat_print, "PCD_BCD_STD_DOWNSTREAM_PORT\n");
                break;
            case PCD_BCD_CHARGING_DOWNSTREAM_PORT:
                mp_printf(&mp_plat_print, "PCD_BCD_CHARGING_DOWNSTREAM_PORT\n");
                break;
            case PCD_BCD_DEDICATED_CHARGING_PORT:
                mp_printf(&mp_plat_print, "PCD_BCD_DEDICATED_CHARGING_PORT\n");
                break;
            default:
                mp_printf(&mp_plat_print, "Unknown\n");
                break;
            }
        }
        else {
            mp_printf(&mp_plat_print, "???\n");
        }
    }
}

STATIC mp_obj_t experimental_detect(void) {

    extern PCD_HandleTypeDef hpcd;

    // If we don't do this then detection only works
    // the very first time. Maybe there is a particular
    // flag that needs to be unset by another HAL function.
    // or maybe that HAL function is just broken, but we'll
    // be rewriting our own process anyway.
    hpcd.Instance->GCCFG = 0;

    // Not sure we need call this every time
    HAL_PCDEx_ActivateBCD(&hpcd);
    
    HAL_PCDEx_BCD_VBUSDetectHACKED(&hpcd);

    // Not sure we need call this every time
    HAL_PCDEx_DeActivateBCD(&hpcd);

    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_0(experimental_detect_obj, experimental_detect);


// pybricks.experimental.hello_world
STATIC mp_obj_t experimental_hello_world(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    PB_PARSE_ARGS_FUNCTION(n_args, pos_args, kw_args,
        // Add up to 8 arguments below, all separated by one comma.
        // You can choose from:
        //  PB_ARG_REQUIRED(name), This is a required argument.
        //  PB_ARG_DEFAULT_INT(name, value), Keyword argument with default int value.
        //  PB_ARG_DEFAULT_OBJ(name, value), Keyword argument with default MicroPython object.
        //  PB_ARG_DEFAULT_QSTR(name, value), Keyword argument with default string value, without quotes.
        //  PB_ARG_DEFAULT_FALSE(name), Keyword argument with default False value.
        //  PB_ARG_DEFAULT_TRUE(name), Keyword argument with default True value.
        //  PB_ARG_DEFAULT_NONE(name), Keyword argument with default None value.
        PB_ARG_REQUIRED(foo),
        PB_ARG_DEFAULT_NONE(bar));

    // This function can be used in the following ways:
    // from pybricks.experimental import hello_world
    // y = hello_world(5)
    // y = hello_world(foo=5)
    // y = hello_world(5, 6)
    // y = hello_world(foo=5, bar=6)

    // All input arguments are available as MicroPython objects with _in post-fixed to the name.
    // Use MicroPython functions or Pybricks helper functions to unpack them into C types:
    mp_int_t foo = pb_obj_get_int(foo_in);

    // You can check if an argument is none.
    if (bar_in == mp_const_none) {
        // Example of how to print.
        mp_printf(&mp_plat_print, "Bar was not given. Foo is: %d\n", foo);
    }

    // Example of returning an object. Here we return the square of the input argument.
    return mp_obj_new_int(foo * foo);
}
// See also experimental_globals_table below. This function object is added there to make it importable.
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(experimental_hello_world_obj, 0, experimental_hello_world);


STATIC const mp_rom_map_elem_t experimental_globals_table[] = {
    #if PYBRICKS_HUB_EV3BRICK
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_experimental_c) },
    { MP_ROM_QSTR(MP_QSTR___init__), MP_ROM_PTR(&mod_experimental___init___obj) },
    { MP_ROM_QSTR(MP_QSTR_pthread_raise), MP_ROM_PTR(&mod_experimental_pthread_raise_obj) },
    #else
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_experimental) },
    #endif // PYBRICKS_HUB_EV3BRICK
    { MP_ROM_QSTR(MP_QSTR_hello_world), MP_ROM_PTR(&experimental_hello_world_obj) },
    { MP_ROM_QSTR(MP_QSTR_detect), MP_ROM_PTR(&experimental_detect_obj) },
};
STATIC MP_DEFINE_CONST_DICT(pb_module_experimental_globals, experimental_globals_table);

const mp_obj_module_t pb_module_experimental = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&pb_module_experimental_globals,
};

#endif // PYBRICKS_PY_EXPERIMENTAL
