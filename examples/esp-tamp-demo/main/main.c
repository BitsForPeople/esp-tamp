#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "tamp/compressor.h"
#include "tamp/decompressor.h"

static const char* const TAG = "tamp-demo";

extern const uint8_t espressif_logo_bmp_start[] asm("_binary_espressif_logo_bmp_start");
extern const uint8_t espressif_logo_bmp_end[] asm("_binary_espressif_logo_bmp_end");

static inline uint32_t min(const uint32_t a, const uint32_t b) {
    return (a<b) ? a : b;
}

typedef struct mem_span {
    union {
        uint8_t* data;
        const uint8_t* cdata;
    };
    size_t size;
} mem_span_t;

static tamp_res compress_block(TampCompressor* const comp, mem_span_t* const input, mem_span_t* const output) {
    const size_t consumed = tamp_compressor_sink( comp, input->cdata, input->size );
    tamp_res r = TAMP_OK;
    input->cdata += consumed;
    input->size -= consumed;
    if(input->size != 0) {
        // If not all input was consumed, the compressor's input buffer is full -> we can poll compressed data.
        size_t produced = 0;
        r = tamp_compressor_compress_poll( comp, output->data, output->size, &produced );
        output->data += produced;
        output->size -= produced;
    }
    return r;
}

/**
 * @brief Flushes and outputs any remaining data from the compressor's buffers.
 * 
 * @param[in,out] comp \c TampCompressor instance
 * @param[in,out] output buffer where any output is to be put; will be adjusted to reflect the data produced upon return.
 * @return tamp_res 
 */
static tamp_res flush_compressor(TampCompressor* const comp, mem_span_t* const output) {
    size_t bytesOutput = 0;
    // Once all input is passed to the compressor, we have to finish the compression:
    tamp_res r = tamp_compressor_flush(comp,output->data,output->size,&bytesOutput,false);
    output->data += bytesOutput;
    output->size -= bytesOutput;
    return r;
}

/**
 * @brief Compresses the input data by incrementally sending it to the compressor, demonstrating
 * incremental compression which can be used when input data may successively become available over time
 * instead of it all being available in a single buffer.
 * 
 * @param input 
 * @param input_len 
 * @param window_bits 
 * @param output 
 * @param max_output 
 * @param out_time_us (optional) if not \c NULL will receive the number of microseconds the compression took.
 * @return number of bytes placed in \p output 
 */
static size_t compress_incremental(const uint8_t* input, size_t input_len, unsigned window_bits, uint8_t* output, size_t max_output, uint32_t* const out_time_us) {

    const TampConf cfg = {
        .window = window_bits,
        .literal = 8,
        .use_custom_dictionary = 0
    };


    uint8_t* const window_mem = (uint8_t*)malloc(1 << window_bits);

    if(!window_mem) {
        ESP_LOGE(TAG, "Failed to allocate %d bytes for compressor window.", (1<<cfg.window));
        return 0;
    }

    TampCompressor comp;
    tamp_compressor_init(&comp, &cfg, window_mem);

    /* For demonstration, we'll send input to the compressor in small blocks,
     * although in this case we actually have all input available right from the start.
     */
    const size_t blocksize = 10; 

    const uint8_t* const input_end = input + input_len;

    mem_span_t inp = {
        .cdata = input,
        .size = min(input_len,blocksize)
    };

    mem_span_t outp = {
        .data = output,
        .size = max_output
    };

    uint32_t tstart = esp_timer_get_time();

    // Keep passing input into the compressor until we run out of input (or output space).
    while(inp.size != 0 && outp.size != 0) {
        tamp_res r = compress_block(&comp, &inp, &outp);
        if(r == TAMP_OUTPUT_FULL) {
            ESP_LOGE(TAG, "Not enough space in output buffer!");
            free(window_mem);
            return 0;
        }
        inp.size = min(input_end-inp.data,blocksize);
    }

    // Once all input is passed to the compressor, we have to finish the compression:
    tamp_res r = flush_compressor(&comp,&outp);

    uint32_t tend = esp_timer_get_time();
    if(out_time_us) {
        *out_time_us = tend - tstart;
    }

    free(window_mem);

    if(r == TAMP_OUTPUT_FULL) {
        ESP_LOGE(TAG, "Not enough space in output buffer!");
    }

    size_t compressed_size = outp.data - output;

    return compressed_size;

}

/**
 * @brief Uses the "one-stop" convenience function \c tamp_compressor_compress_and_flush() to compress the full
 * input in one go.
 * 
 * @param input 
 * @param input_len 
 * @param window_bits 
 * @param output 
 * @param max_output 
 * @param out_time_us (optional) if not \c NULL will receive the number of microseconds the compression took.
 * @return number of bytes placed in \p output
 */
static size_t compress(const uint8_t* input, size_t input_len, unsigned window_bits, uint8_t* output, size_t max_output, uint32_t* const out_time_us) {

    const TampConf cfg = {
        .window = window_bits,
        .literal = 8,
        .use_custom_dictionary = 0
    };


    uint8_t* const window_mem = (uint8_t*)malloc(1 << window_bits);

    if(!window_mem) {
        ESP_LOGE(TAG, "Failed to allocate %d bytes for compressor window.", (1<<window_bits));
        return 0;
    }

    TampCompressor comp;
    tamp_compressor_init(&comp, &cfg, window_mem);

    uint32_t tstart = esp_timer_get_time();

    size_t produced = 0;
    tamp_res r = tamp_compressor_compress_and_flush(&comp, output, max_output, &produced, input, input_len, NULL, false);

    uint32_t tend = esp_timer_get_time();
    if(out_time_us) {
        *out_time_us = tend-tstart;
    }

    free(window_mem);

    if(r == TAMP_OUTPUT_FULL) {
        ESP_LOGE(TAG, "Not enough space in output buffer!");
    } else
    if(r != TAMP_OK) {
        ESP_LOGE(TAG, "Compression returned error: %d", (int)r);
    }

    return produced;
}


static size_t test_decompress(const uint8_t* compressed, size_t compressed_len, uint8_t* output, size_t max_output) {
    TampConf cfg;

    {
        // Read the compressor configuration used from the compressed data:
        size_t consumed = 0;
        tamp_res r = tamp_decompressor_read_header(&cfg,compressed,compressed_len,&consumed);
        if(r != TAMP_OK) {
            ESP_LOGE(TAG, "Failed to parse header of compressed data.");
            return 0;
        }
        compressed += consumed;
        compressed_len -= consumed;
    }

    uint8_t* const window_mem = (uint8_t*)malloc(1 << cfg.window);
    if(!window_mem) {
        ESP_LOGE(TAG, "Failed to allocate %d bytes for decompressor window.", (1<<cfg.window));
        return 0;
    }

    TampDecompressor decomp;
    tamp_decompressor_init(&decomp,&cfg,window_mem);

    size_t consumed = 0;
    size_t produced = 0;

    tamp_res r = tamp_decompressor_decompress(&decomp,output,max_output,&produced, compressed, compressed_len, &consumed);

    free(window_mem);

    if(r == TAMP_OUTPUT_FULL) {
        ESP_LOGE(TAG, "Output buffer exhausted.");
    }

    return produced;
}

static void compare(const uint8_t* input, const size_t input_len, const uint8_t* const decompressed, const size_t decompressed_len) {
    // Compare decompressed data with original:
    if(decompressed_len != input_len) {
        ESP_LOGE(TAG, "Decompressed to %d bytes while %d bytes were expected.",(int)decompressed_len, (int)input_len);
    }
    unsigned i;
    for(i = 0; i < input_len; ++i) {
        if(decompressed[i] != input[i]) {
            ESP_LOGE(TAG, "Difference @ %d: %" PRIu8 " != %" PRIu8, (int)i, input[i], espressif_logo_bmp_start[i]);
            break;
        }
    }
    if(i >= input_len) {
        ESP_LOGI(TAG, "Decompressed data matches input data.");
    }
}


void app_main(void)
{
    ESP_LOGI(TAG, "Tamp demo");
    ESP_LOGI(TAG, "[ESP32-optimized: "
        #if TAMP_ESP32
            "YES"
        #else
            "NO"
        #endif
        "]"
    );

    static const unsigned WINDOW_SIZE_LOG2 = 10;

    const uint8_t* const input = espressif_logo_bmp_start;
    const size_t input_len = espressif_logo_bmp_end - espressif_logo_bmp_start;

    // We don't know the final size of the compressed data, so allocate for the 'worst case'.
    const size_t max_output = input_len;

    uint8_t* const compressed = malloc(max_output);
    uint8_t* const decompressed = malloc(input_len+1);

    uint32_t micros = 0;

    ESP_LOGI(TAG, "Testing incremental compression.");

    // Perform compression:
    ESP_LOGI(TAG, "Compressing %d bytes of input using a window of %d bytes.", (int)input_len, (int)(1<<WINDOW_SIZE_LOG2));

    size_t compressed_len = compress_incremental(
            input,
            input_len,
            WINDOW_SIZE_LOG2,
            compressed,
            max_output,
            &micros);

    ESP_LOGI(TAG, "Compressed %d -> %d bytes (%.1f%%) in %.1fms.", 
        input_len, compressed_len, (compressed_len * 100.f) / input_len, (micros*0.001f));



    // Perform decompression of the compressed data:

    micros = esp_timer_get_time();

    size_t decompressed_len = test_decompress(compressed, compressed_len, decompressed, input_len+1);

    micros = esp_timer_get_time() - micros;

    ESP_LOGI(TAG, "Decompressed to %d bytes (%.1fms).", (int)decompressed_len,(micros*0.001f));


    // Compare decompressed data with original:
    compare(input,input_len,decompressed,decompressed_len);



    ESP_LOGI(TAG, "Testing one-step compression.");

    compressed_len = compress(input,input_len,WINDOW_SIZE_LOG2,compressed,max_output,&micros);

    ESP_LOGI(TAG, "Compressed %d -> %d bytes (%.1f%%) in %.1fms.", 
        input_len, compressed_len, (compressed_len * 100.f) / input_len, (micros*0.001f));



    micros = esp_timer_get_time();

    decompressed_len = test_decompress(compressed, compressed_len, decompressed, input_len+1);

    micros = esp_timer_get_time() - micros;

    ESP_LOGI(TAG, "Decompressed to %d bytes (%.1fms).", (int)decompressed_len,(micros*0.001f));

    // Compare decompressed data with original:
    compare(input,input_len,decompressed,decompressed_len);



    free(decompressed);
    free(compressed);
}
