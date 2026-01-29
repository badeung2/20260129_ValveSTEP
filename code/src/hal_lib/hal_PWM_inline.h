#ifndef HAL_LIB_PWM_INLINE_H
#define HAL_LIB_PWM_INLINE_H
#include "../AppBuild.h"
#include <atomic.h>

static __inline__ void HAL_PWM_MasterIrqDisable(void)
{
	ENTER_SECTION(SYSTEM_MODE);
	IO_MLX16_ITC_MASK1_S  &=  ~B_MLX16_ITC_MASK1_PWM_MASTER1_END;
	EXIT_SECTION();
}

static __inline__ void HAL_PWM_MasterIrqEnable(void)
{
	ENTER_SECTION(SYSTEM_MODE);
	IO_MLX16_ITC_PEND1_S = B_MLX16_ITC_PEND1_PWM_MASTER1_END;
	IO_MLX16_ITC_MASK1_S  |=  B_MLX16_ITC_MASK1_PWM_MASTER1_END;
	EXIT_SECTION();
}

static __inline__ void HAL_PWM_MasterPendClear(void)
{
	ENTER_SECTION(SYSTEM_MODE);
	IO_MLX16_ITC_PEND1_S = B_MLX16_ITC_PEND1_PWM_MASTER1_END;
	EXIT_SECTION();
}

static __inline__ void HAL_PWM_MasterPendWait(void)
{
	while((IO_MLX16_ITC_PEND1_S & B_MLX16_ITC_PEND1_PWM_MASTER1_END)  ==  0U){}
}

#endif
