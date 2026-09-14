#include <string.h>

#include "foc_link_protocol.h"

uint16_t foc_link_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF; /* CRC-16/CCITT-FALSE */

    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static void put_le16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)(v >> 8);
}

static void put_le32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)(v >> 24);
}

void foc_link_encode(uint8_t *out, const foc_link_pkt_t *pkt)
{
    put_le16(&out[0], pkt->magic);
    out[2]  = pkt->version;
    out[3]  = pkt->role;
    put_le32(&out[4], pkt->receiver_id);
    out[8]  = pkt->cmd;
    out[9]  = pkt->session;
    put_le16(&out[10], pkt->sequence);
    put_le32(&out[12], pkt->nonce);
    put_le16(&out[16], pkt->arg);
    put_le16(&out[18], foc_link_crc16(out, FOC_LINK_CRC_COVER_LEN));
}

bool foc_link_decode(const uint8_t *in, size_t len,
                     foc_link_pkt_t *pkt, bool *crc_ok)
{
    if (in == NULL || pkt == NULL || len < FOC_LINK_PKT_LEN) {
        return false;
    }

    pkt->magic       = (uint16_t)(in[0] | (in[1] << 8));
    pkt->version     = in[2];
    pkt->role        = in[3];
    pkt->receiver_id = (uint32_t)in[4] | ((uint32_t)in[5] << 8) |
                       ((uint32_t)in[6] << 16) | ((uint32_t)in[7] << 24);
    pkt->cmd         = in[8];
    pkt->session     = in[9];
    pkt->sequence    = (uint16_t)(in[10] | (in[11] << 8));
    pkt->nonce       = (uint32_t)in[12] | ((uint32_t)in[13] << 8) |
                       ((uint32_t)in[14] << 16) | ((uint32_t)in[15] << 24);
    pkt->arg         = (uint16_t)(in[16] | (in[17] << 8));
    pkt->crc16       = (uint16_t)(in[18] | (in[19] << 8));

    /* 先判"是不是本协议", 再判 CRC —— 两者语义不同, 不能合并成一个 false */
    if (pkt->magic != FOC_LINK_MAGIC || pkt->version != FOC_LINK_VERSION) {
        return false;
    }

    if (crc_ok != NULL) {
        *crc_ok = (foc_link_crc16(in, FOC_LINK_CRC_COVER_LEN) == pkt->crc16);
    }
    return true;
}

foc_link_rx_result_t foc_link_rx_filter(foc_link_rx_state_t *st,
                                        const foc_link_pkt_t *pkt,
                                        uint32_t my_receiver_id)
{
    /* receiver_id 0 = 广播; 否则必须命中本板 */
    if (pkt->receiver_id != 0 && pkt->receiver_id != my_receiver_id) {
        return FOC_LINK_NOT_FOR_ME;
    }

    if (!st->valid) {
        st->session  = pkt->session;
        st->sequence = pkt->sequence;
        st->valid    = true;
        return FOC_LINK_ACCEPT;
    }

    if (pkt->session != st->session) {
        /* session 用 uint8 环形比较: (int8_t)(新 - 旧) > 0 即"新 session 更靠后"。
         * 不能用 !=, 否则发送端重启后回绕到旧值会被永久拒收。 */
        if ((int8_t)(pkt->session - st->session) <= 0) {
            return FOC_LINK_OLD_SESSION;
        }
        /* 新 session: 无条件接受, 并重置 sequence 基线 */
        st->session  = pkt->session;
        st->sequence = pkt->sequence;
        return FOC_LINK_ACCEPT;
    }

    /* 同 session: sequence 必须严格前进 */
    if ((int16_t)(pkt->sequence - st->sequence) <= 0) {
        return FOC_LINK_DUP;
    }
    st->sequence = pkt->sequence;
    return FOC_LINK_ACCEPT;
}

uint8_t foc_link_peek_version(const uint8_t *in, size_t len)
{
    if (in == NULL || len < 3) {
        return 0;
    }
    uint16_t magic = (uint16_t)(in[0] | (in[1] << 8));
    if (magic != FOC_LINK_MAGIC) {
        return 0;
    }
    return in[2];
}
