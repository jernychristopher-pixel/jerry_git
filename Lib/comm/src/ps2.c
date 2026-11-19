#include "ps2.h"

/* ── 全局数据 ── */
uint8_t  PS2_Data[9] = {0};
uint16_t PS2_Key = 0;

static const uint16_t MASK[] = {
    PSB_SELECT, PSB_L3, PSB_R3, PSB_START,
    PSB_PAD_UP, PSB_PAD_RIGHT, PSB_PAD_DOWN, PSB_PAD_LEFT,
    PSB_L2, PSB_R2, PSB_L1, PSB_R1,
    PSB_TRIANGLE, PSB_CIRCLE, PSB_CROSS, PSB_SQUARE
};

/* ── 微秒延时 (DWT 硬件周期计数, 精确) ── */
static void ps2_delay_us(uint32_t us) {
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (HAL_RCC_GetHCLKFreq() / 1000000UL);
    while ((DWT->CYCCNT - start) < ticks) { __NOP(); }
}

/* ── GPIO 原子操作 ── */
#define DI_READ()   HAL_GPIO_ReadPin(PS2_DI_PORT, PS2_DI_PIN)
#define DO_H()      HAL_GPIO_WritePin(PS2_DO_PORT, PS2_DO_PIN, GPIO_PIN_SET)
#define DO_L()      HAL_GPIO_WritePin(PS2_DO_PORT, PS2_DO_PIN, GPIO_PIN_RESET)
#define CS_H()      HAL_GPIO_WritePin(PS2_CS_PORT, PS2_CS_PIN, GPIO_PIN_SET)
#define CS_L()      HAL_GPIO_WritePin(PS2_CS_PORT, PS2_CS_PIN, GPIO_PIN_RESET)
#define CLK_H()     HAL_GPIO_WritePin(PS2_CLK_PORT, PS2_CLK_PIN, GPIO_PIN_SET)
#define CLK_L()     HAL_GPIO_WritePin(PS2_CLK_PORT, PS2_CLK_PIN, GPIO_PIN_RESET)

/* ── 初始化 GPIO ── */
void PS2_Init(void) {
    GPIO_InitTypeDef s = {0};

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* DO, CS, CLK → 推挽输出 */
    s.Mode = GPIO_MODE_OUTPUT_PP;
    s.Pull = GPIO_NOPULL;
    s.Speed = GPIO_SPEED_FREQ_HIGH;
    s.Pin = PS2_DO_PIN;
    HAL_GPIO_Init(PS2_DO_PORT, &s);
    s.Pin = PS2_CS_PIN;
    HAL_GPIO_Init(PS2_CS_PORT, &s);
    s.Pin = PS2_CLK_PIN;
    HAL_GPIO_Init(PS2_CLK_PORT, &s);

    /* DI → 下拉输入 */
    s.Mode = GPIO_MODE_INPUT;
    s.Pull = GPIO_PULLDOWN;
    s.Pin = PS2_DI_PIN;
    HAL_GPIO_Init(PS2_DI_PORT, &s);
}

/* ── 发送一字节命令，同时读回一字节数据 ── */
static void PS2_Cmd(uint8_t cmd) {
    volatile uint16_t bit = 0x01;
    PS2_Data[1] = 0;
    for (bit = 0x01; bit < 0x0100; bit <<= 1) {
        if (cmd & bit) DO_H(); else DO_L();
        CLK_H(); ps2_delay_us(5);
        CLK_L(); ps2_delay_us(5);
        CLK_H();
        if (DI_READ())
            PS2_Data[1] |= bit;
    }
    ps2_delay_us(16);
}

/* ── 判断是否红灯(模拟)模式: 帧有效(0x5A)且非数字模式 ── */
static uint8_t PS2_RedLight(void) {
    uint8_t id, ack;
    CS_L();
    PS2_Cmd(0x01);
    PS2_Cmd(0x42);
    id = PS2_Data[1];
    PS2_Cmd(0x00);
    ack = PS2_Data[1];
    PS2_Cmd(0x00);
    PS2_Cmd(0x00);
    PS2_Cmd(0x00);
    PS2_Cmd(0x00);
    PS2_Cmd(0x00);
    PS2_Cmd(0x00);
    CS_H();
    return ((ack == 0x5A) && ((id & 0xF0) != 0x40)) ? 1 : 0;
}

/* ── 读取完整一帧 (9 字节), Data[1] 保留 ID 字节 ── */
void PS2_ReadData(void) {
    uint8_t i, id;
    CS_L();
    PS2_Cmd(0x01);
    PS2_Cmd(0x42);
    id = PS2_Data[1];
    for (i = 2; i < 9; i++) {
        PS2_Cmd(0x00);
        PS2_Data[i] = PS2_Data[1];
    }
    PS2_Cmd(0x00);
    CS_H();
    PS2_Data[1] = id;
}

/* ── 读取按键值 ── */
uint8_t PS2_DataKey(void) {
    uint8_t i;
    PS2_Key = 0;
    PS2_ReadData();                     // 读一帧
    for (i = 0; i < 16; i++) {
        if (PS2_Data[3] & (1 << i))     // Data[3]: 低 8 键 (SELECT~LEFT)
            PS2_Key |= MASK[i];
        if (PS2_Data[4] & (1 << i))     // Data[4]: 高 8 键 (L2~□)
            PS2_Key |= MASK[i + 8];
    }
    return PS2_Key;
}

/* ── 读取摇杆模拟值 (0~255) ── */
uint8_t PS2_AnologData(uint8_t channel) {
    PS2_ReadData();
    return PS2_Data[channel];
}

/* ── 配置手柄为模拟模式 ── */
void PS2_SetInit(void) {
    CS_L();
    PS2_Cmd(0x01);
    PS2_Cmd(0x43);
    PS2_Cmd(0x00);
    PS2_Cmd(0x01);
    PS2_Cmd(0x00);
    CS_H();
}

/* ── 上电自动进入模拟(红灯)模式: 发送配置 + 校验 + 重试 ── */
uint8_t PS2_EnableAnalog(void) {
    uint8_t tries;
    for (tries = 0; tries < 10; tries++) {
        PS2_SetInit();
        HAL_Delay(50);
        if (PS2_RedLight()) {
            return 1;
        }
        HAL_Delay(100);
    }
    return 0;
}