#ifndef TAMP_COMPRESSOR_H
#define TAMP_COMPRESSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "common.h"

typedef struct TampCompressor{
    unsigned char *window;
    TampConf conf;
    unsigned char input[16];
    uint32_t bit_buffer;
    uint32_t bit_buffer_pos:6;
    uint32_t min_pattern_size:2;
    uint32_t window_pos:15;
    uint32_t input_size:5;
    uint32_t input_pos:4;
} TampCompressor;


/**
 * @brief Initialize Tamp Compressor object.
 *
 * @param[out] compressor Object to initialize.
 * @param[in] conf Compressor configuration. Set to NULL for default.
 * @param[in] window Pre-allocated window buffer. Size must agree with conf->window.
 *                   If conf.use_custom_dictionary is true, then the window must be
 *                   externally initialized.
 *
 * @return Tamp Status Code.
 */
tamp_res tamp_compressor_init(TampCompressor *compressor, const TampConf *conf, unsigned char *window);

/**
 * @brief Sink data into input buffer.
 *
 * @param[in,out] compressor TampCompressor object to perform compression with.
 * @param[in] input Pointer to the input data to be sinked into compressor.
 * @param[in] input_size Size of input.
 * @param[out] consumed_size Number of bytes of input consumed. May be NULL.
 *
 * @return Tamp Status Code.
 */
void tamp_compressor_sink(
        TampCompressor *compressor,
        const unsigned char *input,
        size_t input_size,
        size_t *consumed_size
        );

/**
 * @brief Run a single compression iteration on the internal input buffer.
 *
 * @param[in,out] compressor TampCompressor object to perform compression with.
 * @param[out] output Pointer to a pre-allocated buffer to hold the output compressed data.
 * @param[in] output_size Size of the pre-allocated buffer. Will compress up-to this many bytes.
 * @param[out] output_written_size Number of bytes written to output. May be NULL.
 *
 * @return Tamp Status Code.
 */
tamp_res tamp_compressor_compress_poll(
        TampCompressor *compressor,
        unsigned char *output,
        size_t output_size,
        size_t *output_written_size
        );

/**
 * @brief Completely flush the internal bit buffer. Makes output "complete".
 *
 * @param[in,out] compressor TampCompressor object to flush.
 * @param[out] output Pointer to a pre-allocated buffer to hold the output compressed data.
 * @param[in] output_size Size of the pre-allocated buffer. Will compress up-to this many bytes.
 * @param[out] output_written_size Number of bytes written to output. May be NULL.
 * @param[in] write_token Write the FLUSH token, if appropriate. Set to true if you want to continue using the compressor. Set to false if you are done with the compressor, usually at the end of a stream.
 *
 * @return Tamp Status Code.
 */
tamp_res tamp_compressor_flush(
                TampCompressor *compressor,
                unsigned char *output,
                size_t output_size,
                size_t *output_written_size,
                bool write_token
                );

/**
 * @brief Compress a chunk of data until input or output buffer is exhausted.
 *
 * @param[in,out] compressor TampCompressor object to perform compression with.
 * @param[out] output Pointer to a pre-allocated buffer to hold the output compressed data.
 * @param[in] output_size Size of the pre-allocated buffer. Will compress up-to this many bytes.
 * @param[out] output_written_size Number of bytes written to output. May be NULL.
 * @param[in] input Pointer to the input data to be compressed.
 * @param[in] input_size Number of bytes in input data.
 * @param[out] input_consumed_size Number of bytes of input data consumed. May be NULL.
 *
 * @return Tamp Status Code.
 */
tamp_res tamp_compressor_compress(
        TampCompressor *compressor,
        unsigned char *output,
        size_t output_size,
        size_t *output_written_size,
        const unsigned char *input,
        size_t input_size,
        size_t *input_consumed_size
        );

/**
 * @brief Compress a chunk of data until input or output buffer is exhausted.
 *
 * While all buffers are flushed, no flush token is will be written to the output stream.
 * If the output buffer is full, flushing will not be performed and TAMP_OUTPUT_FULL will be returned.
 *
 * @param[in,out] compressor TampCompressor object to perform compression with.
 * @param[out] output Pointer to a pre-allocated buffer to hold the output compressed data.
 * @param[in] output_size Size of the pre-allocated buffer. Will compress up-to this many bytes.
 * @param[out] output_written_size Number of bytes written to output. May be NULL.
 * @param[in] input Pointer to the input data to be compressed.
 * @param[in] input_size Number of bytes in input data.
 * @param[out] input_consumed_size Number of bytes of input data consumed. May be NULL.
 *
 * @return Tamp Status Code.
 */
tamp_res tamp_compressor_compress_and_flush(
        TampCompressor *compressor,
        unsigned char *output,
        size_t output_size,
        size_t *output_written_size,
        const unsigned char *input,
        size_t input_size,
        size_t *input_consumed_size
        );


#ifdef __cplusplus
}
#endif

#endif