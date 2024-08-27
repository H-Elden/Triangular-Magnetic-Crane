/*----- HC-SR04 超声波传感器的板级支持包-----*/

#include "HC-SR04.h"

/* ----- 引脚定义 ----- */
static GPIO_TypeDef *Echo_GPIO[3] = {GPIOG, GPIOE, GPIOE};
static u16           Echo_Pin[3]  = {GPIO_Pin_0, GPIO_Pin_7, GPIO_Pin_9};
static GPIO_TypeDef *Trig_GPIO[3] = {GPIOG, GPIOE, GPIOE};
static u16           Trig_Pin[3]  = {GPIO_Pin_1, GPIO_Pin_8, GPIO_Pin_10};

// 超声波是否开启测距
uint8_t Sensor_open;

// 超声波待测的目标距离，下标取[0,2]
static u8 Goaldis_min[3];
static u8 Goaldis_max[3];
// 第i个传感器开始测距时的定时器时间
uint16_t Tim7_begin[3];
// 是否要在检测到目标后就停车
uint8_t isStop;

float dist[3];

/**
 * @brief  基本定时器7初始化函数
 * @param  无
 * @retval 无
 */
void TIM7_Init() {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStrue;        // 定义一个定时中断的结构体

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);        // 使能基本定时器7时钟

    TIM_TimeBaseInitStrue.TIM_Prescaler     = 72 - 1;                    // 预分频系数：每1us计数一次
    TIM_TimeBaseInitStrue.TIM_Period        = 0xffff;                    // 自动重装载寄存器：最多计时 65535 us
    TIM_TimeBaseInitStrue.TIM_CounterMode   = TIM_CounterMode_Up;        // 计数模式：向上计数
    TIM_TimeBaseInitStrue.TIM_ClockDivision = TIM_CKD_DIV1;              // 一般不使用，默认TIM_CKD_DIV1
    TIM_TimeBaseInit(TIM7, &TIM_TimeBaseInitStrue);                      // 根据TIM_TimeBaseInitStrue的参数初始化定时器TIM7

    TIM_Cmd(TIM7, ENABLE);        // 使能定时器TIM7
}

void HCSR04_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct;

    // 使能GPIOE和GPIOA和GPIOG时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    // 配置Echo0_Pin至Echo4_Pin为输入模式
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPD;        // 下拉输入
    for (u8 i = 0; i < 3; i++) {
        GPIO_InitStruct.GPIO_Pin = Echo_Pin[i];
        GPIO_Init(Echo_GPIO[i], &GPIO_InitStruct);
    }

    // 配置Trig1_Pin至Trig4_Pin为输出模式
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;        // 推挽输出
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    for (u8 i = 0; i < 3; i++) {
        GPIO_InitStruct.GPIO_Pin = Trig_Pin[i];
        GPIO_Init(Trig_GPIO[i], &GPIO_InitStruct);
        Trig_GPIO[i]->BRR = Trig_Pin[i];        // 设置初始输出电平为低电平
    }

    // 使能复用时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    // 定义外部中断结构体
    NVIC_InitTypeDef NVIC_InitStructure;

    // 配置外部中断
    NVIC_InitStructure.NVIC_IRQChannel                   = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannel            = EXTI9_5_IRQn;
    NVIC_Init(&NVIC_InitStructure);

    EXTI_InitTypeDef EXTI_InitStructure;

    // 配置 EXTI_Line
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOG, GPIO_PinSource0);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource7);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource9);
    EXTI_InitStructure.EXTI_Line    = EXTI_Line0 | EXTI_Line7 | EXTI_Line9;
    EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
}

void Getdis() {
    for (int i = 0; i < 3; i++) {
        if (Sensor_open & (1 << i)) {
            u16 delay = 72 * 20;                            // 软延时，约20us
            GPIO_SetBits(Trig_GPIO[i], Trig_Pin[i]);        // 设置为高电平
            while (delay--);        // 此处不要使用定时器，因为修改了CNT寄存器值，会影响到别的进程的延时功能
            GPIO_ResetBits(Trig_GPIO[i], Trig_Pin[i]);        // 清除为低电平
        }
    }
}

void SetGoaldis(u8 i, u8 _min, u8 _max) {
    Goaldis_max[i] = _max;
    Goaldis_min[i] = _min;
}

/**
 * @brief  超声波传感器i的中断服务函数
 * @param  i  传感器的编号[0,2]
 * @retval 无
 */
uint8_t obj[8];         // i点是否存在砝码，6、7号对应F线的两个砝码
char    isStore;        // 存储哪条线上的砝码信息 '\0'表示不存储，'C'表示存储C线
void    EXTI_Sensor(uint8_t i) {
    if (!(Sensor_open & (1 << i)))        // 如果i传感器没开，但产生了中断，说明是电平抖动，直接return
        return;
    if (GPIO_ReadInputDataBit(Echo_GPIO[i], Echo_Pin[i]) != Bit_RESET) {        // 接收到的是上升沿引起的中断
        Tim7_begin[i] = TIM7->CNT;                                              // 存下开始时间点
    } else {                                                                    // 接收到的是下降沿引起的中断
        // 获取时间，转换为距离，计算一次为1us，获取计算次数cnt
        uint16_t cnt = TIM7->CNT;

        if (cnt > Tim7_begin[i])
            cnt -= Tim7_begin[i];
        else
            cnt += ((uint16_t)0xffff - Tim7_begin[i]);

        if (cnt < 10)        // 0表示无效
            dist[i] = 0;
        else
            dist[i] = 0.0001 * cnt * 340 / 2;        // 单位cm
        // printf("dis %d = %.3f\r\n", i, dist[i]);

        // 近15次测距中有效值(即满足dis在Goaldis_min和Goaldis_max之间)不少于3次 就 停车
        if (countEffect(i, dist[i] > Goaldis_min[i] && dist[i] < Goaldis_max[i]) >= 3) {
            // 关闭传感器 + 置零测距值
            SensorOFF(i);
            if (i & 1) {
                LED_GREEN = 0;        // 左找到开绿灯
            } else {
                LED_RED = 0;        // 右找到开红灯
            }
            dist[i] = 0;
            if (isStore == 'B') {
                obj[0] = 1;
            } else if (isStore == 'C') {
                obj[i] = 1;
                if (!obj[0]) way = 1;        // 中间砝码在C抓
            } else if (isStore == 'E') {
                obj[i + 2] = 1;
            } else if (isStore == 'F') {
                if (i)
                    obj[i + 5] = 1;
                else {
                    obj[5] = 1;
                    return;        // 防止因为测到5而停车
                }
            }

            // 走一段距离 + 停车
            if (isStop) {
                Con_Stop(fabs(Con_Dis - Run_Dis));        // 继续走多少mm停车
                isStop = 0;
            }
        }
    }
}

/**
 * @brief  根据数据是否有效，统计最近的 CYCLES 次数据中有几次是有效数据
 * @param  i 表示传感器编号
 * @param  input 0表示无效，非0表示有效
 * @retval 无
 */
uint32_t ones[3];        // 存储1的位数
uint8_t  countEffect(uint8_t i, uint8_t input) {
    static uint8_t pos[3];        // 当前输入的位置
    // 每 CYCLES 个输入循环一次
    if (pos[i] == CYCLES) pos[i] = 0;

    ones[i] &= ~(1 << pos[i]);           // 移除最旧的输入的位
    if (input)                           // 如果输入是1
        ones[i] |= (1 << pos[i]);        // 把第shift位置1

    // 移动到下一个位置
    pos[i]++;

    // 编译器内置函数，返回64位长整型的二进制中1的个数
    return __builtin_popcountll(ones[i]);
}

/**
 * @brief  超声波Echo引脚的中断回调函数
 * @param  无
 * @retval 无
 */
void EXTI0_IRQHandler() {        // 超声波0的数据:PG0
    if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
        EXTI_ClearITPendingBit(EXTI_Line0);        // 清除中断标志位
        EXTI_Sensor(0);
    }
}

void EXTI9_5_IRQHandler() {
    if (EXTI_GetITStatus(EXTI_Line7) != RESET) {        // 超声波1的数据:PE7
        EXTI_ClearITPendingBit(EXTI_Line7);             // 清除中断标志位
        EXTI_Sensor(1);
    }
    if (EXTI_GetITStatus(EXTI_Line9) != RESET) {        // 超声波2的数据:PE9
        EXTI_ClearITPendingBit(EXTI_Line9);             // 清除中断标志位
        EXTI_Sensor(2);
    }
}
