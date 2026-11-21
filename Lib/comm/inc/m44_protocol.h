#ifndef M44_PROTOCOL_H
#define M44_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>

#define M44_HEAD1              0xAAu
#define M44_HEAD2              0x55u
#define M44_MAX_PAYLOAD        32u
#define M44_FRAME_OVERHEAD     7u
#define M44_RB_SIZE            256u
#define M44_FRAME_TIMEOUT_MS   50u
#define M44_CMD_TIMEOUT_MS     200u

#define M44_MSG_VELOCITY       0x01u
#define M44_MSG_STATUS         0x02u
#define M44_MSG_CLEAR          0x03u

#define M44_STATUS_LINK_ALIVE  0x01u
#define M44_STATUS_IMU_OK      0x02u
#define M44_STATUS_MOTOR_ON    0x04u

typedef struct {
    uint8_t msg_id;
    uint8_t seq;
    uint8_t len;
    uint8_t payload[M44_MAX_PAYLOAD];
} M44_Frame;

typedef void (*M44_WriteFn)(const uint8_t *data, uint16_t len);
typedef void (*M44_VelocityFn)(int16_t left_rpm, int16_t right_rpm);
typedef void (*M44_ClearFn)(void);

typedef enum {
    M44_ST_WAIT_H1 = 0,
    M44_ST_WAIT_H2,
    M44_ST_WAIT_LEN,
    M44_ST_WAIT_ID,
    M44_ST_WAIT_SEQ,
    M44_ST_WAIT_PAYLOAD,
    M44_ST_WAIT_CRC1,
    M44_ST_WAIT_CRC2
} M44_ParserState;

typedef struct {
    volatile uint16_t head;
    volatile uint16_t tail;
    uint8_t buf[M44_RB_SIZE];
    volatile uint32_t overflow_cnt;
} M44_RingBuf;

typedef struct {
    M44_RingBuf rb;

    M44_ParserState state;
    uint8_t len;
    uint8_t id;
    uint8_t seq;
    uint8_t idx;
    uint8_t payload[M44_MAX_PAYLOAD];
    uint8_t crc_lo;
    uint32_t last_byte_ms;

    uint16_t bad_frame_cnt;
    uint16_t timeout_cnt;
    uint16_t heartbeat;
    uint8_t  tx_seq;

    uint32_t last_cmd_ms;
    uint8_t  have_cmd;
    uint8_t  link_alive;

    M44_WriteFn    write;
    M44_VelocityFn on_velocity;
    M44_ClearFn    on_clear;
} M44_Context;

void M44_Init(M44_Context *ctx, M44_WriteFn write,
              M44_VelocityFn on_velocity, M44_ClearFn on_clear);

/* UART 接收中断里调用: 只入环形缓冲, 不做解析 */
void M44_RxByte_ISR(M44_Context *ctx, uint8_t byte);

/* 主循环周期调用: 出缓冲 -> 状态机 -> 超时检查 -> 命令超时停车 */
void M44_Poll(M44_Context *ctx, uint32_t now_ms);

/* 每 50ms 调一次: 组 0x02 状态帧并发送 */
void M44_SendStatus(M44_Context *ctx, int16_t left_rpm, int16_t right_rpm,
                    int16_t pitch_001deg, int16_t yaw_001deg,
                    uint8_t status_bits);

void M44_ClearCounters(M44_Context *ctx);

/* UART 硬件错误回调: 帧错误/噪声错误/溢出。
 * 波特率不匹配时 HAL 会连续报 FE, 这里折算成坏帧计数。 */
void M44_OnUartError(M44_Context *ctx, uint8_t is_overrun);

uint16_t M44_CRC16(const uint8_t *data, size_t len);

#endif
