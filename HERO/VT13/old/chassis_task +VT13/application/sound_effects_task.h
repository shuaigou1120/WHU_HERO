/**
  *************************(C) COPYRIGHT 2020 LionHeart*************************
  * @file       sound_effects.c/h
  * @brief      实现蜂鸣器鸣响效果音的功能。程序内置了12种蜂鸣器效果音，可由用户简
  *             单调用。在机器人开/关功能、状态切换、识别目标等时鸣响对应的效果音，
  *             可在日常研发中辅助开发人员识别机器人状态、判断程序的运行进度，以及
  *             在赛场备赛中辅助队员快速调整机器人。
  *             本程序调用了RoboMaster-C型开发板的板载蜂鸣器，程序简洁、轻便，占用
  *             芯片资源极少，适合用于各种开发场合。
  *
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Sep-19-2020     LionHeart       1. done
  *
  @verbatim
  ==============================================================================
  使用说明：

	1.任务维护：
		使用FreeRTOS维护任务函数：buzzer_effects_task(void const *argument)，保证
		该任务得到较高的优先级。
		需要包含头文件：#include "sound_effects_task.h" 来索引上述函数。

	2.功能调用（举例）：
		假设在另一源文件“xxx.c"中需要触发蜂鸣器音效，则首先需要包含头文件：
			#include "sound_effects_task.h"
		然后需要创建一指针，使其指向sound_effects_task中的buzzer_control，此处直接
		调用函数 get_buzzer_effect_point() 来获取地址，例如：
			buzzer_t *buzzer = get_buzzer_effect_point();
		若需要在程序进行到某阶段时，触发指定的音效，则直接操作指针的sound_effect即
		可。以触发“系统启动音效”为例：
			......
			buzzer->sound_effect = SYSTEM_START_BEEP;
			......
		或：
			......
			if (*(buzzer->is_busy) == FALSE)
				buzzer->sound_effect = SYSTEM_START_BEEP;
			......
		若其他任务中调用蜂鸣器，与此任务产生冲突，导致蜂鸣器音效不正常，可通过操作：
			buzzer->work = FALSE;
		来停用蜂鸣器音效操作。此时，蜂鸣器仍会完成当前正在鸣响的音效，然后才会停止。
		有关各种音效的说明，详见sound_effects_task.h中的sound_effects_t枚举类型。

	3.环境配置：
		本程序还包含buzzer_TIM_init.c/h和bsp_buzzer_driver.c/h两组文件。这些文件皆
		移植自RM2020官方步兵开源程序。本程序依赖HAL库“stm32f4xx_hal.c/h”工作，请确
		保移植的目标工程使用HAL库，否则需要重新设计buzzer_init.c/h和bsp_buzzer.c/h
		中的部分函数。也可由HAL直接产生初始化代码，详细操作方法请参照RoboMaster官方
		的GitHub开源文档：“RoboMaster开发板C型嵌入式软件教程文档.pdf”第71页。
		以RM2020官方开源程序为例，若要将此程序移植进RM2020官方步兵程序，则需要：
			·将源文件和头文件复制到工程目录下，并在工程中添加源文件
			·在freertos.c中包含头文件：#include "sound_effects_task.h"
			·创建蜂鸣器音效任务：
				osThreadDef(buzr, buzzer_effects_task, osPriorityNormal, 0, 128);
				testHandle = osThreadCreate(osThread(buzr), NULL);
			·在test_task.c中引入头文件，并在任务中合适的位置上添加：
				buzzer_t *buzzer = get_buzzer_effect_point();
				......
				buzzer->work = FALSE;
				......
				buzzer->work = TRUE;
			 以消除官方代码中的模块离线提示音与任务冲突造成的音效异常（其实不操作也
			 没有大问题，只是声音难听一点而已）
			·按照需求，在其他源文件中执行功能调用（上述）步骤。
		移植进其他工程中，则需要根据具体情况自行做出调整。
  ==============================================================================
  @endverbatim
  *************************(C) COPYRIGHT 2020 LionHeart*************************
  */

#ifndef __SOUND_EFFECTS_TASK_H
#define __SOUND_EFFECTS_TASK_H
#ifdef __cplusplus
extern "C" {
#endif

// 先包含标准头文件，避免类型未定义
#include "stdint.h"
#include "cmsis_os.h"
// 依赖头文件（确保工程中存在，路径正确）
#include "buzzer_TIM_init.h"
#include "bsp_buzzer_driver.h"

// 蜂鸣器音效任务间隔，10ms。建议不要高于30。
#define BUZZER_TASK_CONTROL_TIME 10

// ----- 蜂鸣器音效枚举（修改命名：去掉下划线结尾，避免编译器识别异常）
typedef enum
{
    SOUND_STOP = 0,                // 停止。蜂鸣器不鸣响。
    SOUND_SYSTEM_START_BEEP,       // 类似于DJI产品开机的三段蜂鸣音
    SOUND_B_SHORT,                 // 一声短促的高音（替代原B_）
    SOUND_B_DOUBLE_SHORT,          // 两声短促的高音（替代原B_B_）
    SOUND_B_TRIPLE_SHORT,          // 三声短促的高音（替代原B_B_B_）
    SOUND_B_LONG,                  // 一声悠长的高音（替代原B___）
    SOUND_B_CONTINUE,              // 连续短促的高音
    SOUND_D_SHORT,                 // 一声短促的低音（替代原D_）
    SOUND_D_DOUBLE_SHORT,          // 两声短促的低音（替代原D_D_）
    SOUND_D_TRIPLE_SHORT,          // 三声短促的低音（替代原D_D_D_）
    SOUND_D_LONG,                  // 一声悠长的低音（替代原D___）
    SOUND_D_CONTINUE,              // 连续短促的低音
    SOUND_D_B_B_SHORT              // 一声低音加上两声高音（替代原D_B_B_）
} sound_effects_t;

// 布尔类型（改名避免与标准库冲突，用uint8_t更兼容）
typedef uint8_t bool_check_t;
#define FALSE 0x00U
#define TRUE  0x01U

// 蜂鸣器操作接口结构体（修改is_busy为非指针，解决类型兼容问题）
typedef struct
{
    const bool_check_t *is_busy;           // 蜂鸣器正忙标志（去掉const和指针，直接存储）
    bool_check_t work;             // 蜂鸣器工作使能
    sound_effects_t sound_effect;  // 蜂鸣器音效选择
} buzzer_t;

/**
  * @brief          蜂鸣器音效任务（10ms间隔执行）
  * @param[in]      argument: 任务入参（空）
  * @retval         none
  */
extern void buzzer_effects_task(void const *argument);

/**
  * @brief          返回蜂鸣器任务数据指针
  * @param[in]      none
  * @retval         buzzer_t* 蜂鸣器任务数据指针
  */
extern buzzer_t *get_buzzer_effect_point(void);

#ifdef __cplusplus
}
#endif
#endif /* __SOUND_EFFECTS_TASK_H */