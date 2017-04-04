px4_add_module(
    MODULE modules__pid_att_control
    MAIN pid_att_control
    COMPILE_FLAGS
    STACK_MAIN 1200
    SRCS
        pid_att_control.cpp
    DEPENDS
        platforms__common
    )
