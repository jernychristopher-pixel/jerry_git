#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

/*
 * 上下位机二进制协议 v1.0（M4-4）
 *
 * 帧格式（小端）:
 *   AA 55 | payload_len(1) | msg_id(1) | seq(1) | payload(N) | CRC16(2)
 *   CRC = CRC-16/MODBUS, 初值 0xFFFF, 反射多项式 0xA001,
 *   覆盖帧头到载荷末尾（不含 CRC 本身），低字节先发。
 *
 * 消息 ID:
 *   0x01 上位机 -> MCU  速度指令  left_rpm(int16), right_rpm(int16)
 *   0x02 MCU -> 上位机  状态      24 字节, 见 Protocol_Status
 *   0x03 上位机 -> MCU  清计数    magic=0x5A
 *   0x10 MCU -> 上位机  遥测      target/actual/pwm 左右轮, 20ms 一帧
 *   0x11 上位机 -> MCU  PID 预设  0=P, 1=PI, 2=PID
 *   0x12 上位机 -> MCU  PID 增益  6 x int16, 单位 0.001
 */

#define PROTO_HEAD1              0xAAu
#define PROTO_HEAD2              0x55u
#define PROTO_MAX_PAYLOAD        32u
#define PROTO_FRAME_OVERHEAD     7u
#define PROTO_RB_SIZE            256u
#define PROTO_FRAME_TIMEOUT_MS   50u
#define PROTO_CMD_TIMEOUT_MS     200u

#define PROTO_MSG_VELOCITY       0x01u
#define PROTO_MSG_STATUS         0x02u
#define PROTO_MSG_CLEAR          0x03u
#define PROTO_MSG_TELEMETRY      0x10u
#define PROTO_MSG_PID_PRESET     0x11u
#define PROTO_MSG_PID_GAINS      0x12u

#define PROTO_STATUS_LEN         24u
#define PROTO_TELEMETRY_LEN      12u
#define PROTO_PID_GAINS_LEN      12u

/* 状态帧 status 位定义 */
#define PROTO_STATUS_LINK_ALIVE  0x01u  /* 200ms 内收到过合法速度帧 */
#define PROTO_STATUS_IMU_OK      0x02u
#define PROTO_STATUS_MOTOR_ON    0x04u

/* 0x02 状态帧的字段（不直接对应线上字节，发送时按小端展开） */
typedef struct {
    int16_t  left_rpm;              /* 实测左轮 r/min */
    int16_t  right_rpm;             /* 实测右轮 r/min */
    int16_t  pitch_001deg;          /* 俯仰角, 0.01 度 */
    int16_t  yaw_001deg;            /* 偏航角, 0.01 度 */
    uint16_t battery_mv;            /* 电池电压 mV, 0=未接 */
    int16_t  yaw_rate_mrad_s;       /* 偏航角速率, mrad/s */
    int16_t  yaw_hold_out_mrad_s;   /* 航向环输出, mrad/s */
    uint8_t  status_bits;           /* PROTO_STATUS_* 位或 */
} Protocol_Status;

/* 0x10 遥测帧: M4-3 画 target/actual/pwm 曲线用 */
typedef struct {
    int16_t target_rpm_L;
    int16_t actual_rpm_L;
    int16_t pwm_L;
    int16_t target_rpm_R;
    int16_t actual_rpm_R;
    int16_t pwm_R;
} Protocol_Telemetry;

typedef struct {
    uint8_t msg_id;
    uint8_t seq;
    uint8_t len;
    uint8_t payload[PROTO_MAX_PAYLOAD];
} Protocol_Frame;

typedef struct {
    volatile uint16_t head;
    volatile uint16_t tail;
    uint8_t buf[PROTO_RB_SIZE];
    volatile uint32_t overflow_cnt;
} Protocol_RingBuf;

typedef enum {
    PROTO_ST_WAIT_H1 = 0,
    PROTO_ST_WAIT_H2,
    PROTO_ST_WAIT_LEN,
    PROTO_ST_WAIT_ID,
    PROTO_ST_WAIT_SEQ,
    PROTO_ST_WAIT_PAYLOAD,
    PROTO_ST_WAIT_CRC1,
    PROTO_ST_WAIT_CRC2
} Protocol_ParserState;

typedef void (*Protocol_WriteFn)(const uint8_t *data, uint16_t len);
typedef void (*Protocol_VelocityFn)(int16_t left_rpm, int16_t right_rpm);
typedef void (*Protocol_PresetFn)(uint8_t preset);
typedef void (*Protocol_GainsFn)(const int16_t gains[6]);

typedef struct {
    Protocol_RingBuf rb;

    Protocol_ParserState state;
    uint8_t len;
    uint8_t id;
    uint8_t seq;
    uint8_t idx;
    uint8_t payload[PROTO_MAX_PAYLOAD];
    uint8_t crc_lo;
    uint32_t last_byte_ms;

    uint16_t bad_frame_cnt;
    uint16_t timeout_cnt;
    uint16_t heartbeat;
    uint8_t  tx_seq;

    uint32_t last_cmd_ms;
    uint8_t  have_cmd;
    uint8_t  link_alive;

    Protocol_WriteFn    write;
    Protocol_VelocityFn on_velocity;
    Protocol_PresetFn   on_preset;
    Protocol_GainsFn    on_gains;
} Protocol_Context;

void Protocol_Init(Protocol_Context *ctx, Protocol_WriteFn write,
                   Protocol_VelocityFn on_velocity,
                   Protocol_PresetFn on_preset,
                   Protocol_GainsFn on_gains);

/* UART 接收中断里调用: 只入环形缓冲, 不做解析 */
void Protocol_RxByte_ISR(Protocol_Context *ctx, uint8_t byte);

/* 主循环周期调用: 出缓冲 -> 状态机 -> 超时检查 -> 命令超时停车 */
void Protocol_Poll(Protocol_Context *ctx, uint32_t now_ms);

/* 每 50ms 调一次 */
void Protocol_SendStatus(Protocol_Context *ctx, const Protocol_Status *st);

/* 每 20ms 调一次, 给 M4-3 采集 target/actual/pwm */
void Protocol_SendTelemetry(Protocol_Context *ctx,
                            const Protocol_Telemetry *tm);

void Protocol_ClearCounters(Protocol_Context *ctx);

/* 模式切换时用: 丢掉半截帧/残留文本, 让状态机从帧头重新开始 */
void Protocol_Reset_Parser(Protocol_Context *ctx);

/* UART 硬件错误回调: 帧错误/噪声错误/溢出 */
void Protocol_OnUartError(Protocol_Context *ctx, uint8_t is_overrun);

uint16_t Protocol_CRC16(const uint8_t *data, size_t len);

/* 把浮点物理量限幅后转 int16, 防止 IMU 异常值在组帧时溢出 */
int16_t Protocol_ClampI16(float v);

#endif /* __PROTOCOL_H */
