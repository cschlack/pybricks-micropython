#define PYBRICKS_BRICK_MOVEHUB
#define PYBRICKS_BRICK_NAME             "BOOST Move Hub"


// You can disable the built-in MicroPython compiler by setting the following
// config option to 0.  If you do this then you won't get a REPL prompt, but you
// will still be able to execute pre-compiled scripts, compiled with mpy-cross.
// Requires about 19K (19568) of flash
#define MICROPY_ENABLE_COMPILER         (1)

// Set to (1) to enable basic motor functionality such as setting the duty cycle, braking, and coasting.
#define PYBRICKS_HW_ENABLE_MOTORS       (1)

// Set to (1) to enable motor encoder functionality and speed control. Has an effect only if PYBRICKS_HW_ENABLE_MOTORS is enabled.
#define PYBRICKS_HW_ENABLE_ENCODERS     (0)

extern const struct _mp_obj_module_t mp_module_hub;
extern const struct _mp_obj_module_t mp_module_constants;

#define PYBRICKS_PORT_BUILTIN_MODULES \
    { MP_OBJ_NEW_QSTR(MP_QSTR_hub), (mp_obj_t)&mp_module_hub },  \
    { MP_OBJ_NEW_QSTR(MP_QSTR__constants), (mp_obj_t)&mp_module_constants },  
