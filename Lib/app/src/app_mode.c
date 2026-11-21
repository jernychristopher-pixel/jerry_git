#include "app_mode.h"

#include <string.h>

#include "kinematics.h"
#include "m42_stream.h"
#include "vofa.h"

volatile uint8_t g_app_mode = APP_MODE_PROTOCOL;

/* 主循环要补做的副作用。APP_MODE_PROTOCOL 的值就是 0, 不能拿 0 表示"没有",
 * 所以单独用一个标志; 要切到哪个模式直接看 g_app_mode。 */
static volatile uint8_t s_pending_valid;

/* AA55 检测只认连续两字节, 避免线上噪声里的单个 0xAA 误切 */
static uint8_t s_prev_byte;

/* "#M42 1\n" / "#M42 0\n" 滑窗, 含结束换行共 7 字节 */
static char    s_cmd[8];
static uint8_t s_cmd_len;

#define APP_CMD_LEN   7u

static const char APP_CMD_M42_ON[APP_CMD_LEN]  = {'#', 'M', '4', '2', ' ', '1', '\n'};
static const char APP_CMD_M42_OFF[APP_CMD_LEN] = {'#', 'M', '4', '2', ' ', '0', '\n'};

void AppMode_Init(uint8_t boot_mode)
{
    g_app_mode      = boot_mode;
    s_pending_valid = 0U;
    s_prev_byte     = 0U;
    s_cmd_len       = 0U;
    memset(s_cmd, 0, sizeof(s_cmd));
}

/* 中断里调用: 只做字节级判断, 切模式本身留给主循环 */
void AppMode_FeedByte(uint8_t byte)
{
    /* --- 1. 二进制帧头: AA 55 --- */
    if ((s_prev_byte == 0xAAu) && (byte == 0x55u) &&
        (g_app_mode == APP_MODE_M42)) {
        g_app_mode = APP_MODE_PROTOCOL;
        s_pending_valid = 1U;
    }
    s_prev_byte = byte;

    /* --- 2. ASCII 命令: 两种模式都收, 否则切进采集模式后就切不回来了 ---
     * 只从 '#' 开始对齐, 逐字节比前缀; 一旦对不上就整段丢掉重来。
     * 两条命令前 5 字节相同, 只在第 6 个字节分叉。 */
    if (s_cmd_len == 0U && byte != (uint8_t)'#') {
        return;
    }
    if (s_cmd_len >= APP_CMD_LEN) {
        s_cmd_len = 0U;
        return;
    }

    if ((byte != (uint8_t)APP_CMD_M42_ON[s_cmd_len]) &&
        (byte != (uint8_t)APP_CMD_M42_OFF[s_cmd_len])) {
        s_cmd_len = 0U;
        return;
    }

    s_cmd[s_cmd_len++] = (char)byte;
    if (s_cmd_len < APP_CMD_LEN) {
        return;
    }
    s_cmd_len = 0U;

    if (memcmp(s_cmd, APP_CMD_M42_ON, APP_CMD_LEN) == 0) {
        g_app_mode      = APP_MODE_M42;
        s_pending_valid = 1U;
    } else if (memcmp(s_cmd, APP_CMD_M42_OFF, APP_CMD_LEN) == 0) {
        g_app_mode      = APP_MODE_PROTOCOL;
        s_pending_valid = 1U;
    }
}

void AppMode_Service(void)
{
    if (s_pending_valid == 0U) {
        return;
    }
    s_pending_valid = 0U;

    if (g_app_mode == APP_MODE_M42) {
        /* 切进采集模式: 先把轮子停住, 否则 PWM 停在上一条指令上 */
        Chassis_Set_Wheel_Rpm(0.0f, 0.0f);
        /* 命令本身的 7 个字节可能已经进了协议环形缓冲, 一起清掉 */
        Vofa_Reset_Parser();
        /* M42_Init() 里要静止 2 s 估零偏, 只能在主循环里阻塞 */
        M42_Init();
    } else {
        /* 切回协议: 清掉残留文本, 让解析器从帧头重新开始 */
        M42_Stop();
        Vofa_Reset_Parser();
        Chassis_Set_Wheel_Rpm(0.0f, 0.0f);
    }
}
