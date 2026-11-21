#include "m44_protocol.h"

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

uint16_t M44_CRC16(const uint8_t *data, size_t len)
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
    crc = crc16_update(crc, M44_HEAD1);
    crc = crc16_update(crc, M44_HEAD2);
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

static void rb_init(M44_RingBuf *rb)
{
    rb->head = 0;
    rb->tail = 0;
    rb->overflow_cnt = 0;
}

static void rb_push(M44_RingBuf *rb, uint8_t byte)
{
    uint16_t next = (uint16_t)((rb->head + 1u) % M44_RB_SIZE);
    if (next == rb->tail) {
        /* 丢最旧: 保证解析器拿到的是最新字节, 半截帧由帧头重新同步 */
        rb->tail = (uint16_t)((rb->tail + 1u) % M44_RB_SIZE);
        rb->overflow_cnt++;
    }
    rb->buf[rb->head] = byte;
    rb->head = next;
}

static uint8_t rb_pop(M44_RingBuf *rb, uint8_t *byte)
{
    if (rb->head == rb->tail) {
        return 0;
    }
    *byte = rb->buf[rb->tail];
    rb->tail = (uint16_t)((rb->tail + 1u) % M44_RB_SIZE);
    return 1;
}

/* 出错时如果当前字节恰好是 0xAA, 它可能就是下一帧的帧头, 不要直接吞掉 */
static void parser_resync(M44_Context *ctx, uint8_t byte)
{
    ctx->state = (byte == M44_HEAD1) ? M44_ST_WAIT_H2 : M44_ST_WAIT_H1;
}

static void handle_frame(M44_Context *ctx, const M44_Frame *f, uint32_t now_ms)
{
    if (f->msg_id == M44_MSG_VELOCITY && f->len == 4u) {
        int16_t left = get_i16_le(&f->payload[0]);
        int16_t right = get_i16_le(&f->payload[2]);

        ctx->last_cmd_ms = now_ms;
        ctx->have_cmd = 1u;
        ctx->link_alive = 1u;
        if (ctx->on_velocity != NULL) {
            ctx->on_velocity(left, right);
        }
    } else if (f->msg_id == M44_MSG_CLEAR &&
               f->len == 1u && f->payload[0] == 0x5Au) {
        M44_ClearCounters(ctx);
        if (ctx->on_clear != NULL) {
            ctx->on_clear();
        }
    }
}

static void parser_feed(M44_Context *ctx, uint8_t byte, uint32_t now_ms)
{
    /* 如果上一帧还没收完就隔了太久, 先判超时再处理这个新字节 */
    if (ctx->state != M44_ST_WAIT_H1 &&
        (uint32_t)(now_ms - ctx->last_byte_ms) > M44_FRAME_TIMEOUT_MS) {
        cnt_inc(&ctx->timeout_cnt);
        ctx->state = M44_ST_WAIT_H1;
    }

    ctx->last_byte_ms = now_ms;

    switch (ctx->state) {
    case M44_ST_WAIT_H1:
        if (byte == M44_HEAD1) {
            ctx->state = M44_ST_WAIT_H2;
        }
        break;

    case M44_ST_WAIT_H2:
        if (byte == M44_HEAD2) {
            ctx->state = M44_ST_WAIT_LEN;
        } else if (byte != M44_HEAD1) {
            ctx->state = M44_ST_WAIT_H1;
        }
        break;

    case M44_ST_WAIT_LEN:
        if (byte > M44_MAX_PAYLOAD) {
            cnt_inc(&ctx->bad_frame_cnt);
            parser_resync(ctx, byte);
        } else {
            ctx->len = byte;
            ctx->state = M44_ST_WAIT_ID;
        }
        break;

    case M44_ST_WAIT_ID:
        ctx->id = byte;
        ctx->state = M44_ST_WAIT_SEQ;
        break;

    case M44_ST_WAIT_SEQ:
        ctx->seq = byte;
        ctx->idx = 0;
        ctx->state = (ctx->len == 0u) ? M44_ST_WAIT_CRC1 : M44_ST_WAIT_PAYLOAD;
        break;

    case M44_ST_WAIT_PAYLOAD:
        ctx->payload[ctx->idx++] = byte;
        if (ctx->idx >= ctx->len) {
            ctx->state = M44_ST_WAIT_CRC1;
        }
        break;

    case M44_ST_WAIT_CRC1:
        ctx->crc_lo = byte;
        ctx->state = M44_ST_WAIT_CRC2;
        break;

    case M44_ST_WAIT_CRC2: {
        uint16_t crc_rx = (uint16_t)((uint16_t)ctx->crc_lo |
                                     ((uint16_t)byte << 8));
        uint16_t crc_calc = frame_crc(ctx->len, ctx->id, ctx->seq, ctx->payload);
        if (crc_rx == crc_calc) {
            M44_Frame f;
            f.msg_id = ctx->id;
            f.seq = ctx->seq;
            f.len = ctx->len;
            memcpy(f.payload, ctx->payload, ctx->len);
            handle_frame(ctx, &f, now_ms);
        } else {
            cnt_inc(&ctx->bad_frame_cnt);
        }
        ctx->state = M44_ST_WAIT_H1;
        break;
    }

    default:
        ctx->state = M44_ST_WAIT_H1;
        break;
    }
}

void M44_Init(M44_Context *ctx, M44_WriteFn write,
              M44_VelocityFn on_velocity, M44_ClearFn on_clear)
{
    memset(ctx, 0, sizeof(*ctx));
    rb_init(&ctx->rb);
    ctx->state = M44_ST_WAIT_H1;
    ctx->write = write;
    ctx->on_velocity = on_velocity;
    ctx->on_clear = on_clear;
}

void M44_RxByte_ISR(M44_Context *ctx, uint8_t byte)
{
    rb_push(&ctx->rb, byte);
}

void M44_ClearCounters(M44_Context *ctx)
{
    ctx->bad_frame_cnt = 0;
    ctx->timeout_cnt = 0;
    ctx->rb.overflow_cnt = 0;
}

void M44_OnUartError(M44_Context *ctx, uint8_t is_overrun)
{
    /* 硬件层面已经判定这一串波形不合法, 当前半帧直接作废。
     * 波特率错时 FE 会连续触发, 坏帧计数随之快速增长并饱和。 */
    cnt_inc(&ctx->bad_frame_cnt);
    if (is_overrun != 0u && ctx->rb.overflow_cnt != 0xFFFFFFFFu) {
        ctx->rb.overflow_cnt++;
    }
    ctx->state = M44_ST_WAIT_H1;
}

void M44_Poll(M44_Context *ctx, uint32_t now_ms)
{
    uint8_t byte;

    while (rb_pop(&ctx->rb, &byte)) {
        parser_feed(ctx, byte, now_ms);
    }

    /* 半截帧超时: 必须由周期任务检查, 不能只等新字节 */
    if (ctx->state != M44_ST_WAIT_H1 &&
        (uint32_t)(now_ms - ctx->last_byte_ms) > M44_FRAME_TIMEOUT_MS) {
        cnt_inc(&ctx->timeout_cnt);
        ctx->state = M44_ST_WAIT_H1;
    }

    /* 上位机断线: 200ms 没有合法速度帧就自动停车, 只报一次 */
    if (ctx->have_cmd &&
        (uint32_t)(now_ms - ctx->last_cmd_ms) > M44_CMD_TIMEOUT_MS) {
        if (ctx->link_alive) {
            ctx->link_alive = 0u;
            if (ctx->on_velocity != NULL) {
                ctx->on_velocity(0, 0);
            }
        }
    }
}

void M44_SendStatus(M44_Context *ctx, int16_t left_rpm, int16_t right_rpm,
                    int16_t pitch_001deg, int16_t yaw_001deg,
                    uint8_t status_bits)
{
    uint8_t payload[18];
    uint8_t frame[18 + M44_FRAME_OVERHEAD];
    uint16_t crc;

    put_i16_le(&payload[0], left_rpm);
    put_i16_le(&payload[2], right_rpm);
    put_i16_le(&payload[4], pitch_001deg);
    put_i16_le(&payload[6], yaw_001deg);
    put_u16_le(&payload[8], ctx->heartbeat++);
    put_u16_le(&payload[10], ctx->bad_frame_cnt);
    put_u16_le(&payload[12], ctx->timeout_cnt);
    put_u16_le(&payload[14], (ctx->rb.overflow_cnt > 0xFFFFu)
                                 ? 0xFFFFu
                                 : (uint16_t)ctx->rb.overflow_cnt);
    payload[16] = (uint8_t)(status_bits |
                            (ctx->link_alive ? M44_STATUS_LINK_ALIVE : 0u));
    payload[17] = 0u;

    frame[0] = M44_HEAD1;
    frame[1] = M44_HEAD2;
    frame[2] = 18u;
    frame[3] = M44_MSG_STATUS;
    frame[4] = ctx->tx_seq++;
    memcpy(&frame[5], payload, sizeof(payload));
    crc = frame_crc(18u, M44_MSG_STATUS, frame[4], payload);
    frame[23] = (uint8_t)(crc & 0xFFu);
    frame[24] = (uint8_t)((crc >> 8) & 0xFFu);

    if (ctx->write != NULL) {
        ctx->write(frame, (uint16_t)sizeof(frame));
    }
}
