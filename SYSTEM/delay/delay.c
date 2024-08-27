#include "delay.h"

static u8  fac_us;        // us延时倍乘数
static u16 fac_ms;        // ms延时倍乘数

/**
 * @brief  初始化延迟函数
 * @param  无
 * @retval 无
 * @note		SYSTICK的时钟频率为外部晶振八倍频
 */
void delay_init() {
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);        // 选择外部时钟  HCLK/8
    fac_us = SystemCoreClock / 8 / 1000000;                      // 为系统时钟的1/8，SysTick的计数频率为 72/8 = 9MHz，即每1us计数9次
    fac_ms = SystemCoreClock / 8 / 1000;                         // 代表每个ms需要的systick时钟数
}

/**
 * @brief  基于SysTick的微秒延时函数
 * @param  nus 要延时的微秒数
 * @retval 无
 * @note		nus < 0xffffff / fac_us = 1,864,135
 */
void delay_us(u32 nus) {
    u32 temp;
    SysTick->LOAD  = nus * fac_us;                   // 时间加载
    SysTick->VAL   = 0x00;                           // 清空计数器
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;        // 开始倒数
    do {
        temp = SysTick->CTRL;
    } while ((temp & 0x01) && !(temp & (1 << 16)));        // 等待时间到达
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;        // 关闭计数器
    SysTick->VAL   = 0X00;                            // 清空计数器
}

/**
 * @brief  基于SysTick的豪秒延时函数
 * @param  nms 要延时的豪秒数，最大65,535ms
 * @retval 无
 * @note		ms < 0xffffff / fac_ms = 1,864 即可
 */
void delay_ms(u16 nms) {
    u16 tt = nms >> 10;         // nms = 1024 * tt + nms
    u16 ms = nms & 1023;        // 此处 ms 一定会 < 1024

    u32 temp;
    while (tt--) {
        SysTick->LOAD  = 1024 * fac_ms;                  // 时间加载，每次延时1024ms，循环tt次
        SysTick->VAL   = 0x00;                           // 清空计数器
        SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;        // 开始倒数
        do {
            temp = SysTick->CTRL;
        } while ((temp & 0x01) && !(temp & (1 << 16)));        // 等待时间到达
        SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;        // 关闭计数器
    }

    SysTick->LOAD  = ms * fac_ms;                    // 时间加载，延时nms
    SysTick->VAL   = 0x00;                           // 清空计数器
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;        // 开始倒数
    do {
        temp = SysTick->CTRL;
    } while ((temp & 0x01) && !(temp & (1 << 16)));        // 等待时间到达
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;        // 关闭计数器
    SysTick->VAL   = 0X00;                            // 清空计数器
}
