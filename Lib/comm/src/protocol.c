#include "protocol.h"

#include <string.h>

static uint16_t crc16_update(uint16_t crc, uint8_t byte)
{
    crc ^= byte;
    for (uint8_t i = 0; i < 8; i++) {
        if (crc & 0x0001u) {
            crc = (uint16_t)((crc >> 1) ^ 0xA001u);
        } else {
            crc = (uint16_t)(crc >> 1);
        }
    }
    return crc;
}

uint16_t Protocol_CRC16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc = crc16_update(crc, data[i]);
    }
    return crc;
}

static uint16_t frame_crc(uint8_t len, uint8_t id, uint8_t seq,
                          const uint8_t *payload)
{
    uint16_t crc = 0xFFFFu;
    crc = crc16_update(crc, PROTO_HEAD1);
    crc = crc16_update(crc, PROTO_HEAD2);
    crc = crc16_update(crc, len);
    crc = crc16_update(crc, id);
    crc = crc16_update(crc, seq);
    for (uint8_t i = 0; i < len; i++) {
        crc = crc16_update(crc, payload[i]);
    }
    return crc;
}

static void put_i16_le(uint8_t *p, int16_t v)
{
    p[0] = (uint8_t)((uint16_t)v & 0xFFu);
    p[1] = (uint8_t)(((uint16_t)v >> 8) & 0xFFu);
}

static void put_u16_le(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static int16_t get_i16_le(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

/* uint16 计数器饱和在 0xFFFF, 不回绕。
 * 长时间垃圾流下回绕会把"坏帧很多"显示成"坏帧 0", 现场无法判断。 */
static void cnt_inc(uint16_t *c)
{
    if (*c != 0xFFFFu) {
        (*c)++;
    }
}

static void rb_init(Protocol_RingBuf *rb)
{
    rb->head = 0;
    rb->tail = 0;
    rb->overflow_cnt = 0;
}

static void rb_push(Protocol_RingBuf *rb, uint8_t byte)
{
    uint16_t next = (uint16_t)((rb->head + 1u) % PROTO_RB_SIZE);
    if (next == rb->tail) {
        /* 丢最旧: 保证解析器拿到的是最新字节, 半截帧由帧头重新同步 */
        rb->tail = (uint16_t)((rb->tail + 1u) % PROTO_RB_SIZE);
        rb->overflow_cnt++;
    }
    rb->buf[rb->head] = byte;
    rb->head = next;
}

static uint8_t rb_pop(Protocol_RingBuf *rb, uint8_t *byte)
{
    if (rb->head == rb->tail) {
        return 0;
    }
    *byte = rb->buf[rb->tail];
    rb->tail = (uint16_t)((rb->tail + 1u) % PROTO_RB_SIZE);
    return 1;
}

/* 出错时如果当前字节恰好是 0xAA, 它可能就是下一帧的帧头, 不要直接吞掉 */
static void parser_resync(Protocol_Context *ctx, uint8_t byte)
{
    ctx->state = (byte == PROTO_HEAD1) ? PROTO_ST_WAIT_H2 : PROTO_ST_WAIT_H1;
}

static void handle_velocity(Protocol_Context *ctx, const Protocol_Frame *f,
                            uint32_t now_ms)
{
    int16_t left = get_i16_le(&f->payload[0]);
    int16_t right = get_i16_le(&f->payload[2]);

    ctx->last_cmd_ms = now_ms;
    ctx->have_cmd = 1u;
    ctx->link_alive = 1u;
    if (ctx->on_velocity != NULL) {
        ctx->on_velocity(left, right);
    }
}

static void handle_frame(Protocol_Context *ctx, const Protocol_Frame *f,
                         uint32_t now_ms)
{
    switch (f->msg_id) {
    case PROTO_MSG_VELOCITY:
        if (f->len == 4u) {
            handle_velocity(ctx, f, now_ms);
        }
        break;

    case PROTO_MSG_CLEAR:
        if (f->len == 1u && f->payload[0] == 0x5Au) {
            Protocol_ClearCounters(ctx);
        }
        break;

    case PROTO_MSG_PID_PRESET:
        if (f->len == 1u && f->payload[0] <= 2u &&
            ctx->on_preset != NULL) {
            ctx->on_preset(f->payload[0]);
        }
        break;

    case PROTO_MSG_PID_GAINS:
        if (f->len == PROTO_PID_GAINS_LEN && ctx->on_gains != NULL) {
            int16_t gains[6];
            for (uint8_t i = 0; i < 6u; i++) {
                gains[i] = get_i16_le(&f->payload[i * 2u]);
            }
            ctx->on_gains(gains);
        }
        break;

    default:
        /* 未知消息忽略, 不当作坏帧: 方便以后加新 ID 而不破坏旧固件 */
        break;
    }
}

static void parser_feed(Protocol_Context *ctx, uint8_t byte, uint32_t now_ms)
{
    /* 如果上一帧还没收完就隔了太久, 先判超时再处理这个新字节 */
    if (ctx->state != PROTO_ST_WAIT_H1 &&
        (uint32_t)(now_ms - ctx->last_byte_ms) > PROTO_FRAME_TIMEOUT_MS) {
        cnt_inc(&ctx->timeout_cnt);
        ctx->state = PROTO_ST_WAIT_H1;
    }

    ctx->last_byte_ms = now_ms;

    switch (ctx->state) {
    case PROTO_ST_WAIT_H1:
        if (byte == PROTO_HEAD1) {
            ctx->state = PROTO_ST_WAIT_H2;
        }
        break;

    case PROTO_ST_WAIT_H2:
        if (byte == PROTO_HEAD2) {
            ctx->state = PROTO_ST_WAIT_LEN;
        } else if (byte != PROTO_HEAD1) {
            ctx->state = PROTO_ST_WAIT_H1;
        }
        break;

    case PROTO_ST_WAIT_LEN:
        if (byte > PROTO_MAX_PAYLOAD) {
            cnt_inc(&ctx->bad_frame_cnt);
            parser_resync(ctx, byte);
        } else {
            ctx->len = byte;
            ctx->state = PROTO_ST_WAIT_ID;
        }
        break;

    case PROTO_ST_WAIT_ID:
        ctx->id = byte;
        ctx->state = PROTO_ST_WAIT_SEQ;
        break;

    case PROTO_ST_WAIT_SEQ:
        ctx->seq = byte;
        ctx->idx = 0;
        ctx->state = (ctx->len == 0u) ? PROTO_ST_WAIT_CRC1
                                      : PROTO_ST_WAIT_PAYLOAD;
        break;

    case PROTO_ST_WAIT_PAYLOAD:
        ctx->payload[ctx->idx++] = byte;
        if (ctx->idx >= ctx->len) {
            ctx->state = PROTO_ST_WAIT_CRC1;
        }
        break;

    case PROTO_ST_WAIT_CRC1:
        ctx->crc_lo = byte;
        ctx->state = PROTO_ST_WAIT_CRC2;
        break;

    case PROTO_ST_WAIT_CRC2: {
        uint16_t crc_rx = (uint16_t)((uint16_t)ctx->crc_lo |
                                     ((uint16_t)byte << 8));
        uint16_t crc_calc = frame_crc(ctx->len, ctx->id, ctx->seq,
                                      ctx->payload);
        if (crc_rx == crc_calc) {
            Protocol_Frame f;
            f.msg_id = ctx->id;
            f.seq = ctx->seq;
            f.len = ctx->len;
            memcpy(f.payload, ctx->payload, ctx->len);
            handle_frame(ctx, &f, now_ms);
        } else {
            cnt_inc(&ctx->bad_frame_cnt);
        }
        ctx->state = PROTO_ST_WAIT_H1;
        break;
    }

    default:
        ctx->state = PROTO_ST_WAIT_H1;
        break;
    }
}

void Protocol_Init(Protocol_Context *ctx, Protocol_WriteFn write,
                   Protocol_VelocityFn on_velocity,
                   Protocol_PresetFn on_preset,
                   Protocol_GainsFn on_gains)
{
    memset(ctx, 0, sizeof(*ctx));
    rb_init(&ctx->rb);
    ctx->state = PROTO_ST_WAIT_H1;
    ctx->write = write;
    ctx->on_velocity = on_velocity;
    ctx->on_preset = on_preset;
    ctx->on_gains = on_gains;
}

void Protocol_RxByte_ISR(Protocol_Context *ctx, uint8_t byte)
{
    rb_push(&ctx->rb, byte);
}

/*
 * 只在主循环、且已经确定不会再往缓冲里塞旧模式数据时调用。
 * 从 M4-2 文本流切回来时, 缓冲里可能躺着半行 CSV 或 ASCII 命令,
 * 不清掉会把它们当成帧头发下去。计数器和 seq 保留, 便于观察累计坏帧。
 */
void Protocol_Reset_Parser(Protocol_Context *ctx)
{
    ctx->rb.head = 0;
    ctx->rb.tail = 0;

    ctx->state       = PROTO_ST_WAIT_H1;
    ctx->len         = 0;
    ctx->id          = 0;
    ctx->seq         = 0;
    ctx->idx         = 0;
    ctx->crc_lo      = 0;
    ctx->last_byte_ms = 0;
}

void Protocol_ClearCounters(Protocol_Context *ctx)
{
    ctx->bad_frame_cnt = 0;
    ctx->timeout_cnt = 0;
    ctx->rb.overflow_cnt = 0;
}

void Protocol_OnUartError(Protocol_Context *ctx, uint8_t is_overrun)
{
    /* 硬件层面已经判定这一串波形不合法, 当前半帧直接作废。
     * 波特率错时 FE 会连续触发, 坏帧计数随之快速增长并饱和。 */
    cnt_inc(&ctx->bad_frame_cnt);
    if (is_overrun != 0u && ctx->rb.overflow_cnt != 0xFFFFFFFFu) {
        ctx->rb.overflow_cnt++;
    }
    ctx->state = PROTO_ST_WAIT_H1;
}

void Protocol_Poll(Protocol_Context *ctx, uint32_t now_ms)
{
    uint8_t byte;

    while (rb_pop(&ctx->rb, &byte)) {
        parser_feed(ctx, byte, now_ms);
    }

    /* 半截帧超时: 必须由周期任务检查, 不能只等新字节 */
    if (ctx->state != PROTO_ST_WAIT_H1 &&
        (uint32_t)(now_ms - ctx->last_byte_ms) > PROTO_FRAME_TIMEOUT_MS) {
        cnt_inc(&ctx->timeout_cnt);
        ctx->state = PROTO_ST_WAIT_H1;
    }

    /* 上位机断线: 200ms 没有合法速度帧就自动停车, 只报一次 */
    if (ctx->have_cmd &&
        (uint32_t)(now_ms - ctx->last_cmd_ms) > PROTO_CMD_TIMEOUT_MS) {
        if (ctx->link_alive) {
            ctx->link_alive = 0u;
            if (ctx->on_velocity != NULL) {
                ctx->on_velocity(0, 0);
            }
        }
    }
}

static void send_frame(Protocol_Context *ctx, uint8_t msg_id,
                       const uint8_t *payload, uint8_t len)
{
    uint8_t frame[PROTO_MAX_PAYLOAD + PROTO_FRAME_OVERHEAD];
    uint16_t crc;

    frame[0] = PROTO_HEAD1;
    frame[1] = PROTO_HEAD2;
    frame[2] = len;
    frame[3] = msg_id;
    frame[4] = ctx->tx_seq++;
    memcpy(&frame[5], payload, len);
    crc = frame_crc(len, msg_id, frame[4], payload);
    frame[5u + len] = (uint8_t)(crc & 0xFFu);
    frame[6u + len] = (uint8_t)((crc >> 8) & 0xFFu);

    if (ctx->write != NULL) {
        ctx->write(frame, (uint16_t)(len + PROTO_FRAME_OVERHEAD));
    }
}

void Protocol_SendStatus(Protocol_Context *ctx, const Protocol_Status *st)
{
    uint8_t payload[PROTO_STATUS_LEN];

    put_i16_le(&payload[0], st->left_rpm);
    put_i16_le(&payload[2], st->right_rpm);
    put_i16_le(&payload[4], st->pitch_001deg);
    put_i16_le(&payload[6], st->yaw_001deg);
    put_u16_le(&payload[8], ctx->heartbeat++);
    put_u16_le(&payload[10], ctx->bad_frame_cnt);
    put_u16_le(&payload[12], ctx->timeout_cnt);
    put_u16_le(&payload[14], (ctx->rb.overflow_cnt > 0xFFFFu)
                                 ? 0xFFFFu
                                 : (uint16_t)ctx->rb.overflow_cnt);
    payload[16] = (uint8_t)(st->status_bits |
                            (ctx->link_alive ? PROTO_STATUS_LINK_ALIVE : 0u));
    payload[17] = 0u;
    put_u16_le(&payload[18], st->battery_mv);
    put_i16_le(&payload[20], st->yaw_rate_mrad_s);
    put_i16_le(&payload[22], st->yaw_hold_out_mrad_s);

    send_frame(ctx, PROTO_MSG_STATUS, payload, PROTO_STATUS_LEN);
}

void Protocol_SendTelemetry(Protocol_Context *ctx,
                            const Protocol_Telemetry *tm)
{
    uint8_t payload[PROTO_TELEMETRY_LEN];

    put_i16_le(&payload[0], tm->target_rpm_L);
    put_i16_le(&payload[2], tm->actual_rpm_L);
    put_i16_le(&payload[4], tm->pwm_L);
    put_i16_le(&payload[6], tm->target_rpm_R);
    put_i16_le(&payload[8], tm->actual_rpm_R);
    put_i16_le(&payload[10], tm->pwm_R);

    send_frame(ctx, PROTO_MSG_TELEMETRY, payload, PROTO_TELEMETRY_LEN);
}

int16_t Protocol_ClampI16(float v)
{
    /* IMU 异常/丢帧时数值可能离谱, 组帧前统一限幅, 防止 int16 溢出翻符号 */
    if (v >  32767.0f) return  32767;
    if (v < -32768.0f) return -32768;
    return (int16_t)v;
}
