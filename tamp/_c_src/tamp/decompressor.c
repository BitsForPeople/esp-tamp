#include "decompressor.h"
#include "common.h"

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

#define FLUSH 15


#define TAMP_ASSUME(x) if(!(x)) { __builtin_unreachable(); }

/**
 * This array was generated with tools/huffman_jump_table.py
 *
 * The idea is that the resulting code is smaller/faster as a lookup table than a bunch of if/else statements.
 *
 * Of each element:
 *  * The upper 4 bits express the number of bits to decode.
 *  * The lower 4 bits express the decoded value, with FLUSH being represented as 0b1111
 */
static const uint8_t HUFFMAN_TABLE[128] = {50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 85, 85, 85, 85, 122, 123, 104, 104, 86, 86, 86, 86, 93, 93, 93, 93, 68, 68, 68, 68, 68, 68, 68, 68, 105, 105, 124, 127, 87, 87, 87, 87, 51, 51, 51, 51, 51, 51, 51, 51, 51, 51, 51, 51, 51, 51, 51, 51, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17};


/**
 * @brief Decode a huffman match-size symbol from the decompressor's bit_buffer.
 *
 * Internally updates bit_buffer and bit_buffer_pos.
 *
 * bit_buffer MUST have at least 8 bits prior to calling.
 *
 * @returns Decoded match_size
 */
static inline int8_t huffman_decode(uint32_t *bit_buffer, uint8_t *bit_buffer_pos){
    uint8_t code;
    uint8_t bit_len;

    (*bit_buffer_pos)--;
    code = *bit_buffer >> 31;
    *bit_buffer <<= 1;
    if(TAMP_LIKELY(code == 0))
        return 0;

    code = *bit_buffer >> (32 - 7);
    code = HUFFMAN_TABLE[code];
    bit_len = code >> 4;
    *bit_buffer <<= bit_len;
    (*bit_buffer_pos) -= bit_len;

    return code & 0xF;
}

tamp_res tamp_decompressor_read_header(TampConf *conf, const unsigned char *input, size_t input_size, size_t *input_consumed_size) {
    if(input_consumed_size)
        (*input_consumed_size) = 0;
    if(input_size == 0)
        return TAMP_INPUT_EXHAUSTED;
    if(input[0] & 0x2)
        return TAMP_INVALID_CONF;  // Reserved
    if(input[0] & 0x1)
        return TAMP_INVALID_CONF;  // Currently only a single header byte is supported.
    if(input_consumed_size)
        (*input_consumed_size)++;

    conf->window = ((input[0] >> 5) & 0x7) + 8;
    conf->literal = ((input[0] >> 3) & 0x3) + 5;
    conf->use_custom_dictionary = ((input[0] >> 2) & 0x1);

    return TAMP_OK;
}

/**
 * Populate the rest of the decompressor structure after the following fields have been populated:
 *   * conf
 *   * window
 */
static tamp_res tamp_decompressor_populate_from_conf(TampDecompressor *decompressor, uint8_t conf_window, uint8_t conf_literal, uint8_t conf_use_custom_dictionary){
    if(conf_window < 8 || conf_window > 15)
        return TAMP_INVALID_CONF;
    if(conf_literal < 5 || conf_literal > 8)
        return TAMP_INVALID_CONF;
    if(!conf_use_custom_dictionary)
        tamp_initialize_dictionary(decompressor->window, (1 << conf_window));

    decompressor->conf_window = conf_window;
    decompressor->conf_literal = conf_literal;

    decompressor->min_pattern_size = tamp_compute_min_pattern_size(conf_window, conf_literal);
    decompressor->configured = true;

    return TAMP_OK;
}

tamp_res tamp_decompressor_init(TampDecompressor *decompressor, const TampConf *conf, unsigned char *window){
    tamp_res res = TAMP_OK;
    for(uint8_t i=0; i < sizeof(TampDecompressor); i++)  // Zero-out the struct
        ((unsigned char *)decompressor)[i] = 0;
    decompressor->window = window;
    if(conf){
        res = tamp_decompressor_populate_from_conf(decompressor, conf->window, conf->literal, conf->use_custom_dictionary);
    }

    return res;
}

/**
 * @brief Copies 16 bytes from \p src to \p dst
 * 
 * @param src 
 * @param dst 
 */
static inline void cpy16(const uint8_t* src, uint8_t* dst) {
    #if TAMP_ESP32
        *(uint64_t*)dst = *(const uint64_t*)src;
        *(uint64_t*)(dst+8) = *(const uint64_t*)(src+8);
    #else 
        for(unsigned i = 0; i < 16; ++i) {
            dst[i] = src[i];
        }
    #endif
}

/**
 * @brief Copies 0...16 bytes from \p src to \p dst
 * 
 * @param src 
 * @param dst 
 * @param cnt number of bytes to copy, max. 16
 */
static inline void cpy(const uint8_t* src, uint8_t* dst, size_t cnt) {
    #if TAMP_ESP32
        if(TAMP_LIKELY(cnt < 16)) {
            if(cnt & 8) {
                *(uint64_t*)dst = *(const uint64_t*)src;
                dst += 8;
                src += 8;
            }
            if(cnt & 4) {
                *(uint32_t*)dst = *(const uint32_t*)src;
                dst += 4;
                src += 4;
            }
            if(cnt & 2) {
                *(uint16_t*)dst = *(const uint16_t*)src;
                dst += 2;
                src += 2;
            }
            if(cnt & 1) {
                *(uint8_t*)dst = *(const uint8_t*)src;
                // dst += 1;
                // src += 1;
            }
        } else {
            cpy16(src,dst);
        }
    #else
    for(unsigned i = 0; i < cnt; ++i) {
        *dst++ = *src++;
    }
    #endif
}

tamp_res tamp_decompressor_decompress(
        TampDecompressor *decompressor,
        unsigned char *output,
        size_t output_size,
        size_t *output_written_size,
        const unsigned char *input,
        size_t input_size,
        size_t *input_consumed_size
        ){
    size_t input_consumed_size_proxy;
    size_t output_written_size_proxy;
    tamp_res res;
    const unsigned char *input_end = input + input_size;
    const unsigned char *output_end = output + output_size;

    if(!output_written_size)
        output_written_size = &output_written_size_proxy;
    if(!input_consumed_size)
        input_consumed_size = &input_consumed_size_proxy;

    *input_consumed_size = 0;
    *output_written_size = 0;

    if(!decompressor->configured){
        //Read in header
        size_t header_consumed_size;
        TampConf conf;
        res = tamp_decompressor_read_header(&conf, input, input_end - input, &header_consumed_size);
        if(res != TAMP_OK)
            return res;

        res = tamp_decompressor_populate_from_conf(decompressor, conf.window, conf.literal, conf.use_custom_dictionary);
        if(res != TAMP_OK)
            return res;

        input += header_consumed_size;
        (*input_consumed_size) += header_consumed_size;
    }
    const uint16_t window_mask = (1 << decompressor->conf_window) - 1;
    while(input != input_end || decompressor->bit_buffer_pos){
        // Populate the bit buffer
        while(input != input_end && decompressor->bit_buffer_pos <= 24){
            decompressor->bit_buffer_pos += 8;
            decompressor->bit_buffer |=  *input << (32 - decompressor->bit_buffer_pos);
            input++;
            (*input_consumed_size)++;
        }

        if(TAMP_UNLIKELY(decompressor->bit_buffer_pos == 0))
            return TAMP_INPUT_EXHAUSTED;

        if(TAMP_UNLIKELY(output == output_end))
            return TAMP_OUTPUT_FULL;

        // Hint that patterns are more likely than literals
        if(TAMP_UNLIKELY(decompressor->bit_buffer >> 31)){
            // is literal
            if(TAMP_UNLIKELY(decompressor->bit_buffer_pos < (1 + decompressor->conf_literal)))
                return TAMP_INPUT_EXHAUSTED;
            decompressor->bit_buffer <<= 1;  // shift out the is_literal flag

            // Copy literal to output
            *output = decompressor->bit_buffer >> (32 - decompressor->conf_literal);
            decompressor->bit_buffer <<= decompressor->conf_literal;
            decompressor->bit_buffer_pos -= (1 + decompressor->conf_literal);

            // Update window
            decompressor->window[decompressor->window_pos] = *output;
            decompressor->window_pos = (decompressor->window_pos + 1) & window_mask;

            output++;
            (*output_written_size)++;
        }
        else{
            // is token; attempt a decode
            /* copy the bit buffers so that we can abort at any time */
            uint32_t bit_buffer = decompressor->bit_buffer;
            uint16_t window_offset;
            uint16_t window_offset_skip;
            uint8_t bit_buffer_pos = decompressor->bit_buffer_pos;
            uint8_t match_size;
            uint8_t match_size_skip;

            // shift out the is_literal flag
            bit_buffer <<= 1;
            bit_buffer_pos--;

            // There must be at least 8 bits, otherwise no possible decoding.
            if(TAMP_UNLIKELY(bit_buffer_pos < 8))
                return TAMP_INPUT_EXHAUSTED;

            match_size = huffman_decode(&bit_buffer, &bit_buffer_pos);
            if(TAMP_UNLIKELY(match_size == FLUSH)){
                // flush bit_buffer to the nearest byte and skip the remainder of decoding
                decompressor->bit_buffer = bit_buffer << (bit_buffer_pos & 7);
                decompressor->bit_buffer_pos = bit_buffer_pos & ~7;  // Round bit_buffer_pos down to nearest multiple of 8.
                continue;
            }
            if(TAMP_UNLIKELY(bit_buffer_pos < decompressor->conf_window)){
                // There are not enough bits to decode window offset
                return TAMP_INPUT_EXHAUSTED;
            }
            match_size += decompressor->min_pattern_size;

            window_offset = bit_buffer >> (32 - decompressor->conf_window);

            // Apply skip_bytes
            match_size_skip = match_size - decompressor->skip_bytes;
            window_offset_skip = window_offset + decompressor->skip_bytes;

            // Check if we are output-buffer-limited, and if so to set skip_bytes
            // Otherwise, update the decompressor buffers
            size_t remaining = output_end - output;
            if(TAMP_UNLIKELY((uint8_t)match_size_skip > remaining)){
                decompressor->skip_bytes += remaining;
                match_size_skip = remaining;
            }
            else {
                decompressor->skip_bytes = 0;
                decompressor->bit_buffer = bit_buffer << decompressor->conf_window;
                decompressor->bit_buffer_pos = bit_buffer_pos - decompressor->conf_window;
            }

            // Copy pattern to output
            {
                const uint8_t* src = &(decompressor->window[window_offset_skip]);
                cpy(src,output,match_size_skip);
                output += match_size_skip;
            }

            (*output_written_size) += match_size_skip;

            if(TAMP_LIKELY(decompressor->skip_bytes == 0)){
                // Perform window update;
                uint8_t tmp_buf[16];

                const uint32_t w_pos = decompressor->window_pos;
                const uint8_t* src = &(decompressor->window[window_offset]);
                uint8_t* dst = &(decompressor->window[w_pos]);

                if(TAMP_UNLIKELY(((src < dst) && ((src + match_size) >= dst)))) {
                    // src and dst overlap.
                    // Copy src data into tmp buffer
                    #if TAMP_ESP32
                        cpy16(src,tmp_buf);
                    #else
                        cpy(src,tmp_buf,match_size);
                    #endif
                    src = &(tmp_buf[0]);
                }
                {
                    const uint32_t window_size = window_mask + 1;

                    uint32_t l1 = window_size - w_pos;
                    l1 = (l1 <= match_size) ? l1 : match_size;

                    cpy(src,dst,l1);

                    if(TAMP_UNLIKELY(l1 < match_size)) {
                        // Need to wrap around.
                        src += l1;
                        const uint32_t l2 = match_size - l1;
                        dst = &(decompressor->window[0]);
                        cpy(src,dst,l2);
                    }
                    decompressor->window_pos = (w_pos + match_size) & window_mask;
                }
            }
        }

    }
    return TAMP_INPUT_EXHAUSTED;
}
