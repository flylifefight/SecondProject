#include "bsp_misc.h"
#include "misc.h"

void misc_config(void)
{
    NVIC_Priority_Group_Set(NVIC_PER4_SUB0_PRIORITYGROUP);
}

