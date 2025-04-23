#include "global_lib/simple_fsm.h"
#include "global_lib/custom_time.h"
#include "stdio.h"
#include <unistd.h>

int main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    MachineState state;

    state.resetUptimeTimeout();

    while (1)
    {
        switch (state.value)
        {
        case 0:
            printf("HAL_gpio State 0\n");
            state.timeout(1, 2.5);
            break;

        case 1:
            printf("HAL_gpio State 1\n");
            state.timeout(2, 2.5);
            break;

        case 2:
            printf("HAL_gpio State 2\n");
            state.timeout(0, 2.5);
            break;
        }

        usleep(100000); // Sleep for 100ms
    }

    return 0;
}