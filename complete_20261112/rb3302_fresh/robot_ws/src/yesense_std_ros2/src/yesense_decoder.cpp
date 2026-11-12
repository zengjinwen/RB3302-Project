#include <iostream>
#include <string.h>
#include "yesense_decoder_comm.h"
#include "yesense_decoder.h"

// ===========================================================================


// ===========================================================================
namespace yesense{

yesense_decoder::yesense_decoder()
{
    memset(decode_data, 0, DATA_BUF_SIZE);
    decode_buf_len = 0u;
    std_out_decoder = new yis_std_out_decoder();

    cmd_info.flg        = 0u;
    cmd_info.result     = 0x00u;
    cmd_info.data_class = 0x00u;
}

yesense_decoder::~yesense_decoder()
{
    delete std_out_decoder;
}

int yesense_decoder::data_proc(unsigned char *data, unsigned int len, yis_out_data_t *result)
{   
    if (!result || (len && !data)) return para_err;
    result->content.valid_flg = 0;
    // Keep the newest bytes on overflow, so the stream can always recover.
    if (len >= DATA_BUF_SIZE) {
        data += len - DATA_BUF_SIZE;
        len = DATA_BUF_SIZE;
        decode_buf_len = 0;
    } else if (len > DATA_BUF_SIZE - decode_buf_len) {
        clear_buf_data(0, len - (DATA_BUF_SIZE - decode_buf_len) - 1);
    }
    if (len) memcpy(decode_data + decode_buf_len, data, len);
    decode_buf_len += len;

    while (decode_buf_len) {
        unsigned int start = 0;
        while (start + 1 < decode_buf_len &&
               !(decode_data[start] == 0x59 && decode_data[start + 1] == 0x53)) ++start;
        if (start + 1 >= decode_buf_len) {
            // Preserve a partial header across serial reads.
            const bool partial = decode_data[decode_buf_len - 1] == 0x59;
            decode_buf_len = partial ? 1 : 0;
            if (partial) decode_data[0] = 0x59;
            return no_header;
        }
        if (start) clear_buf_data(0, start - 1);
        int ret = std_out_decoder->data_proc(decode_data, decode_buf_len, result);
        if (ret == data_len_err) return ret;
        if (ret == analysis_ok) {
            clear_buf_data(0, std_out_decoder->msg_idx_obt()->end_idx);
            return ret;
        }
        // Bad CRC/payload: skip one byte and search again, never use stale indices.
        clear_buf_data(0, 0);
        result->content.valid_flg = 0;
    }
    return no_header;
}

int yesense_decoder::clear_buf_data(unsigned int st_idx, unsigned int end_idx)
{
    if (st_idx > end_idx || end_idx >= decode_buf_len) return para_err;
    const unsigned int count = end_idx - st_idx + 1;
    memmove(decode_data + st_idx, decode_data + end_idx + 1,
            decode_buf_len - end_idx - 1);
    decode_buf_len -= count;

    return analysis_ok;
}

}
