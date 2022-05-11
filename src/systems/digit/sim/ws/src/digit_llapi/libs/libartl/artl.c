/*
 * Copyright 2020 Agility Robotics
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

/*
 * Version history
 *
 * 1.0 (4/30/2020)
 *   Initial public release
 *
 * 1.1 (7/27/2020)
 *   Fixed an issue that prevented reconnecting to subscribers on some systems
 */

#include "artl.h"
#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
#include <zstd.h>
#include "artl_internal.h"
#include "crc32c.h"


/*******************************************************************************
 * ARTL reading functions
 ******************************************************************************/

int artl_read(const void *artl_data, size_t artl_data_size,
              artl_description_t *desc, void **data, size_t *data_size)
{
    // Check inputs
    if (!desc || !data || !data_size) {
        error("Null pointer passed as input\n");
        return -1;
    }

    // Track number of chunks read
    int nchunks = 0;

    // Initialize description
    artl_clear_description(desc);

    // Initialize data output
    *data = NULL;
    *data_size = 0;

    // Read chunks
    while (artl_data_size) {
        // Read a chunk
        artl_chunk_type_t chunk_type;
        const void *chunk_data;
        size_t chunk_data_size;
        const int64_t chunk_size =
            artl_read_chunk(artl_data, artl_data_size,
                            &chunk_type, &chunk_data, &chunk_data_size);
        if (chunk_size < 0)
            return -1;

        // Advance through buffer
        artl_data += chunk_size;
        artl_data_size -= chunk_size;
        ++nchunks;

        // Check chunk validity
        if (chunk_size < 0)
            continue;

        // Parse chunk
        size_t buffer_size_required;
        bool desc_changed;
        artl_stream_output_options_t out = {
            .desc = desc,
            .buffer_size_required = &buffer_size_required,
            .desc_changed = &desc_changed
        };
        if (artl_read_stream(chunk_type, chunk_data,
                             chunk_data_size, &out) < 0 || desc_changed) {
            return -1;
        }

        // If data is available, run another pass to get it
        if (buffer_size_required) {
            // Increase output buffer size
            *data = realloc(*data, *data_size + buffer_size_required);

            // Parse chunk again
            memset(&out, 0, sizeof out);
            out.buffer = *data + *data_size;
            if (artl_read_stream(chunk_type, chunk_data,
                                 chunk_data_size, &out) < 0) {
                return -1;
            }

            *data_size += buffer_size_required;
        }
    }

    info("Read %d chunks\n", nchunks);
    return 0;
}


int artl_read_partial(int fd, artl_description_t *desc,
                      void **data, size_t *data_size)
{
    // Initialize data outputs
    if (data)
        *data = NULL;
    if (data_size)
        *data_size = 0;

    // Read chunk from file descriptor
    artl_chunk_type_t chunk_type;
    void *chunk_data;
    size_t chunk_data_size;
    const int64_t ret =
        artl_read_chunk_fd(fd, &chunk_type, &chunk_data, &chunk_data_size);
    if (ret < 0)
        return -1;
    else if (ret == 0)
        return 0;

    // Workaround for matlab being unable to pass NULL to a void**
    if (!data_size)
        data = NULL;

    // Parse chunk
    bool desc_changed;
    artl_stream_output_options_t out = {
        .desc = desc,
        .buffer_ptr = data,
        .buffer_size = data_size,
        .desc_changed = &desc_changed
    };
    if (artl_read_stream(chunk_type, chunk_data,
                         chunk_data_size, &out) < 0 || desc_changed) {
        // An error occured or the description changed
        if (desc_changed)
            error("Message description changed partway through the file\n");
        free(chunk_data);
        if (data)
            free(*data);
        if (data_size)
            *data_size = 0;
        return -1;
    }

    free(chunk_data);
    return 1;
}


/*******************************************************************************
 * ARTL writing
 ******************************************************************************/

int64_t artl_write_description(int fd, const artl_description_t *desc)
{
    // Create description chunks
    size_t nchunks;
    void **chunks;
    size_t *chunk_sizes;
    artl_create_description_chunks(desc, &nchunks, &chunks, &chunk_sizes);

    // Write chunks to file
    size_t total_size = 0;
    for (size_t i = 0; i < nchunks; ++i) {
        void *pos = chunks[i];
        size_t size = chunk_sizes[i];
        do {
            const ssize_t nbytes = write(fd, pos, size);
            if (-1 == nbytes) {
                error("Write to file descriptor failed\n");
                artl_free_description_chunks(nchunks, chunks, chunk_sizes);
                return -1;
            }
            size -= nbytes;
        } while (size);
        total_size += chunk_sizes[i];
    }

    // Free chunks
    artl_free_description_chunks(nchunks, chunks, chunk_sizes);

    // Return bytes written
    return total_size;
}


int64_t artl_write_udat(int fd, const void *data, size_t data_size)
{
    // Allocate chunk buffer
    int64_t chunk_size = data_size + 128;
    void *chunk = malloc(chunk_size);

    // Create chunk
    chunk_size = artl_create_udat_chunk(chunk, chunk_size, data, data_size);
    if (chunk_size < 0) {
        free(chunk);
        error("Failed to create UDAT chunk\n");
        return -1;
    }

    // Write chunk
    void *pos = chunk;
    size_t size = chunk_size;
    do {
        const ssize_t nbytes = write(fd, pos, size);
        if (-1 == nbytes) {
            error("Write to file descriptor failed\n");
            free(chunk);
            return -1;
        }
        size -= nbytes;
    } while (size);

    // Free chunk memory and return chunk size
    free(chunk);
    return chunk_size;
}


int64_t artl_write_cdat(int fd, const void *data, size_t data_size)
{
    // Allocate chunk buffer
    int64_t chunk_size = ZSTD_compressBound(data_size) + 128;
    void *chunk = malloc(chunk_size);

    // Create chunk
    chunk_size = artl_create_cdat_chunk(chunk, chunk_size, data, data_size);
    if (chunk_size < 0) {
        free(chunk);
        error("Failed to create CDAT chunk\n");
        return -1;
    }

    // Write chunk
    void *pos = chunk;
    size_t size = chunk_size;
    do {
        const ssize_t nbytes = write(fd, pos, size);
        if (-1 == nbytes) {
            error("Write to file descriptor failed\n");
            free(chunk);
            return -1;
        }
        size -= nbytes;
    } while (size);

    // Free chunk memory and return chunk size
    free(chunk);
    return chunk_size;

}


void artl_create_description_chunks(const artl_description_t *desc,
                                    size_t *nchunks, void ***chunks,
                                    size_t **chunk_sizes)
{
    // Create description chunks
    // Use one DESC chunk and one chunk per enum
    const size_t nchunks_total = 3 + desc->nenums + !!desc->comment;
    *nchunks = 0;
    *chunks = malloc(nchunks_total * sizeof (void *));
    *chunk_sizes = malloc(nchunks_total * sizeof (size_t));
    uint32_t desc_crc = 0;

    // Create STRT chunk
    (*chunk_sizes)[*nchunks] = 12;
    (*chunks)[*nchunks] = malloc((*chunk_sizes)[*nchunks]);
    artl_create_strt_chunk((*chunks)[*nchunks], (*chunk_sizes)[*nchunks]);
    ++(*nchunks);

    // Create ENUM chunks
    for (int64_t i = 0; i < desc->nenums; ++i) {
        const artl_enum_t *enm = &desc->enums[i];

        // Start chunk
        artl_chunk_writer_t *cw = artl_enum_chunk_start(
            enm->type, enm->underlying_type);

        // Add labels
        for (int64_t j = 0; j < enm->nlabels; ++j)
            artl_enum_chunk_add_label(
                cw, enm->labels[j].value, enm->labels[j].text);

        // Finish chunk
        desc_crc = artl_chunk_end(cw, desc_crc, &(*chunks)[*nchunks],
                                  &(*chunk_sizes)[*nchunks]);

        // Directly free chunk writer to "steal" its allocated memory instead
        free(cw);
        ++(*nchunks);
    }

    // Create DESC chunk
    artl_chunk_writer_t *cw = artl_desc_chunk_start();
    for (int64_t i = 0; i < desc->nfields; ++i) {
        const artl_field_t *field = &desc->fields[i];
        artl_desc_chunk_add_field(cw, field->name, field->type,
                                  field->nrows, field->ncols);
    }
    desc_crc = artl_chunk_end(cw, desc_crc, &(*chunks)[*nchunks],
                              &(*chunk_sizes)[*nchunks]);
    free(cw);
    ++(*nchunks);

    // If a comment is present, create CMNT chunk
    if (desc->comment) {
        artl_chunk_writer_t *cw =
            artl_cmnt_chunk_start(desc->comment_data,
                                  desc->comment->message_size);
        for (int64_t i = 0; i < desc->comment->nfields; ++i) {
            const artl_field_t *field = &desc->comment->fields[i];
            artl_desc_chunk_add_field(cw, field->name, field->type,
                                      field->nrows, field->ncols);
        }
        desc_crc = artl_chunk_end(cw, desc_crc, &(*chunks)[*nchunks],
                                  &(*chunk_sizes)[*nchunks]);
        free(cw);
        ++(*nchunks);
    }

    // Create DEND chunk
    (*chunk_sizes)[*nchunks] = 16;
    (*chunks)[*nchunks] = malloc((*chunk_sizes)[*nchunks]);
    artl_create_dend_chunk((*chunks)[*nchunks],
                           (*chunk_sizes)[*nchunks], desc_crc);
    ++(*nchunks);
}


void artl_free_description_chunks(size_t nchunks, void **chunks,
                                  size_t *chunk_sizes)
{
    // Free description chunks and size array
    for (size_t i = 0; i < nchunks; ++i)
        free(chunks[i]);
    free(chunks);
    free(chunk_sizes);
}


int64_t artl_create_udat_chunk(void *buffer, size_t buffer_size,
                               const void *data, size_t data_size)
{
    // Check data size
    if (data_size > UINT32_MAX) {
        error("Data is too large for a single UDAT chunk\n");
        return -1;
    }

    // Compute size needed for chunk
    const size_t chunk_size =
        sizeof (artl_chunk_header_t) + data_size + sizeof (uint32_t);

    // If provided buffer is not large enough, return -1
    if (buffer_size < chunk_size) {
        error("Insufficient buffer size for UDAT chunk\n");
        return -1;
    }

    // Add chunk header
    artl_chunk_header_t *chunk_header = buffer;
    chunk_header->data_size = data_size;
    chunk_header->type = ARTL_CHUNK_UDAT;

    // Copy data
    void *chunk_data = buffer + sizeof *chunk_header;
    memcpy(chunk_data, data, data_size);

    // Compute CRC32c of chunk type and data
    uint32_t crc = crc32c(0, &chunk_header->type,
                          sizeof chunk_header->type + data_size);

    // Append CRC
    uint32_t *crc_addr = chunk_data + data_size;
    *crc_addr = crc;

    return chunk_size;
}


int64_t artl_create_cdat_chunk(void *buffer, size_t buffer_size,
                               const void *data, size_t data_size)
{
    // If provided buffer is not large enough for the chunk header, return -1
    if (buffer_size < sizeof (artl_chunk_header_t)) {
        error("Insufficient buffer size for CDAT chunk\n");
        return -1;
    }

    // Add chunk header
    artl_chunk_header_t *chunk_header = buffer;
    chunk_header->type = ARTL_CHUNK_CDAT;

    // Get buffer size available for compressed data
    size_t compressed_max_size =
        buffer_size - sizeof *chunk_header - sizeof (uint32_t);

    // Compress data
    void *compressed_data = buffer + sizeof *chunk_header;
    size_t compressed_size =
        ZSTD_compress(compressed_data, compressed_max_size, data, data_size, 3);

    // Check for compression errors (including insufficient buffer_size)
    if (ZSTD_isError(compressed_size)) {
        error("CDAT compression error: %s\n",
              ZSTD_getErrorName(compressed_size));
        return -1;
    }

    // Set compressed data size
    chunk_header->data_size = compressed_size;
    if (compressed_size > UINT32_MAX) {
        error("Compressed data is too large for a single CDAT chunk\n");
        return -1;
    }

    // Compute CRC32c of chunk type and data
    uint32_t crc = crc32c(0, &chunk_header->type,
                          sizeof chunk_header->type + compressed_size);

    // Append CRC
    uint32_t *crc_addr = compressed_data + compressed_size;
    *crc_addr = crc;

    return sizeof *chunk_header + compressed_size + sizeof crc;
}


/*******************************************************************************
 * Structure mapping functions
 ******************************************************************************/

artl_map_t *artl_map_init(const artl_description_t *desc)
{
    artl_map_t *map = calloc(1, sizeof (artl_map_t));

    // Save description pointer for more concise field mapping calls
    map->description = desc;

    // Save message size directly in the map so description isn't used
    // when mapping data
    map->message_size = desc->message_size;

    return map;
}


bool artl_map_field(artl_map_t *map, const char *field,
                    int64_t output_offset, artl_type_t output_type)
{
    // Get field
    const artl_field_t *f = artl_find_field(map->description, field);
    if (!f) {
        info("Requested field does not exist in description\n");
        return false;
    }

    // Map entire field
    const int64_t nelements = f->ncols * f->nrows;
    return artl_map_partial_field_impl(map, field, output_offset,
                                       output_type, 0, nelements, false);
}


bool artl_map_partial_field(artl_map_t *map, const char *field,
                            int64_t output_offset, artl_type_t output_type,
                            int64_t istart, int64_t nelements)
{
    return artl_map_partial_field_impl(map, field, output_offset,
                                       output_type, istart, nelements, false);
}

bool artl_map_field_abs(artl_map_t *map, const char *field,
                        void *output_address, artl_type_t output_type)
{
    // Get field
    const artl_field_t *f = artl_find_field(map->description, field);
    if (!f) {
        info("Requested field does not exist in description\n");
        return false;
    }

    // Map entire field
    const int64_t nelements = f->ncols * f->nrows;
    return artl_map_partial_field_impl(map, field, (int64_t) output_address,
                                       output_type, 0, nelements, true);
}


bool artl_map_partial_field_abs(artl_map_t *map, const char *field,
                                void *output_address, artl_type_t output_type,
                                int64_t istart, int64_t nelements)
{
    return artl_map_partial_field_impl(map, field, (int64_t) output_address,
                                       output_type, istart, nelements, true);
}


int64_t artl_map_get_array(const artl_map_t *map, void *output,
                           size_t output_size, size_t output_stride,
                           const void *buffer, size_t buffer_size)
{
    // Handle bad inputs
    if (!output_stride) {
        info("Output stride cannot be zero\n");
        return 0;
    }

    // Track number of mapped structs
    int64_t nstructs = 0;

    // Map structs until the source or destination are depleted
    while (output_size >= output_stride &&
           buffer_size >= map->message_size) {
        // Copy each mapped region
        for (int i = 0; i < map->nregions; ++i) {
            const artl_map_region_t *region = &map->regions[i];
            void *output_addr = region->output_absolute ?
                region->output_address : output + region->output_offset;
            if (artl_convert(buffer + region->buffer_offset,
                             region->buffer_size, output_addr,
                             region->buffer_type, region->output_type) < 0) {
                error("Invalid conversion attempted (%d to %d)\n",
                      region->buffer_type, region->output_type);
                return nstructs;
            }
        }

        // Advance source and destination pointers
        output += output_stride;
        output_size -= output_stride;
        buffer += map->message_size;
        buffer_size -= map->message_size;

        info("Mapped struct %ld in array\n", nstructs);
        ++nstructs;
    }

    return nstructs;
}


void artl_map_free(artl_map_t *map)
{
    // Free regions
    if (map)
        free(map->regions);

    // Free map struct
    free(map);
}


bool artl_check_type_conversion(artl_type_t from, artl_type_t to,
                                int *num, int *den)
{
    // Initialize numerator and denominator for no conversion
    *num = 1;
    *den = 1;

    // ARTL_ANY signifies no conversion
    if (ARTL_ANY == to)
        return true;

    // Can only convert to base types
    if (artl_classify_type(to) != ARTL_TYPE_CLASS_BASE)
        return false;

    // Get size ratio
    *den = artl_find_type_size(NULL, from);
    *num = artl_find_type_size(NULL, to);
    if (*num < 1 || *den < 1)
        return false;

    // Check integer type compatibility
    // Booleans can be converted to integers and vice-versa
    if ((artl_check_type_match(NULL, from, ARTL_INT) || from == ARTL_BOOL) &&
        (artl_check_type_match(NULL, to, ARTL_INT) || to == ARTL_BOOL))
        return true;

    // Check float type compatibility
    if (artl_check_type_match(NULL, from, ARTL_FLOAT) &&
        artl_check_type_match(NULL, to, ARTL_FLOAT))
        return true;

    // Check byte type compatibility
    if (artl_check_type_match(NULL, from, ARTL_BYTE) &&
        artl_check_type_match(NULL, to, ARTL_BYTE))
        return true;

    return false;
}


// memcpy-like with type conversion
#define COPY_CONVERT(from, to) do {                                 \
        const from *src = buffer;                                   \
        to *dst = output;                                           \
        for (size_t i = 0; i < buffer_size / sizeof (from); ++i)    \
            dst[i] = src[i];                                        \
        return 0;                                                   \
    } while (0)
#define INT_CONVERT_FROM(fromartl, fromtype) do {                   \
        if (from == fromartl) {                                     \
            if (to == ARTL_U8) COPY_CONVERT(fromtype, uint8_t);     \
            if (to == ARTL_U16) COPY_CONVERT(fromtype, uint16_t);   \
            if (to == ARTL_U32) COPY_CONVERT(fromtype, uint32_t);   \
            if (to == ARTL_U64) COPY_CONVERT(fromtype, uint64_t);   \
            if (to == ARTL_I8) COPY_CONVERT(fromtype, int8_t);      \
            if (to == ARTL_I16) COPY_CONVERT(fromtype, int16_t);    \
            if (to == ARTL_I32) COPY_CONVERT(fromtype, int32_t);    \
            if (to == ARTL_I64) COPY_CONVERT(fromtype, int64_t);    \
        }} while (0)
#define INT_CONVERT_FROM_SIZE(bits) do {                        \
        INT_CONVERT_FROM(ARTL_U ## bits, uint ## bits ## _t);   \
        INT_CONVERT_FROM(ARTL_I ## bits, int ## bits ## _t);    \
    } while (0)

int artl_convert(const void *buffer, size_t buffer_size, void *output,
                 artl_type_t from, artl_type_t to)
{
    // No conversion
    if (from == to || ARTL_ANY == to || to == ARTL_BIN ||
        artl_find_type_size(NULL, from) == artl_find_type_size(NULL, to)) {
        memcpy(output, buffer, buffer_size);
        return 0;
    }

    // Treat booleans like U8
    if (from == ARTL_BOOL)
        from = ARTL_U8;
    if (to == ARTL_BOOL)
        to = ARTL_U8;

    // Integers, all combinations of signed and unsigned
    INT_CONVERT_FROM_SIZE(8);
    INT_CONVERT_FROM_SIZE(16);
    INT_CONVERT_FROM_SIZE(32);
    INT_CONVERT_FROM_SIZE(64);

    // Floats
    if (from == ARTL_F32 && to == ARTL_F64) COPY_CONVERT(float, double);
    if (from == ARTL_F64 && to == ARTL_F32) COPY_CONVERT(double, float);

    // No valid conversion
    return -1;
}

#undef INT_CONVERT_FROM_SIZE
#undef INT_CONVERT_FROM
#undef COPY_CONVERT


/*******************************************************************************
 * ARTL over UDP communication
 ******************************************************************************/

#ifdef _WIN32

// Winsock compatibility
#include <winsock2.h>

#define poll WSAPoll
#define close closesocket
#define ioctl ioctlsocket
#define perror(str) fprintf(stderr, str ": %d\n", WSAGetLastError())
#define SOCKETS_INIT                                    \
    do {                                                \
        WSADATA wsaData;                                \
        int res = WSAStartup(MAKEWORD(2, 2), &wsaData); \
        if (res) {                                      \
            printf("WSAStartup failed: %d\n", res);     \
            return NULL;                                \
        }                                               \
    } while (0)
#define SOCKETS_CLEANUP WSACleanup()
typedef u_long ioctl_arg_t;
typedef char sockopt_t;
typedef struct ip_mreq ip_mreq_t;

static int inet_aton(const char *cp, struct in_addr *addr)
{
    addr->s_addr = inet_addr(cp);
    return (addr->s_addr == INADDR_NONE) ? 0 : 1;
}

#else

// Linux sockets
#include <arpa/inet.h>
#include <poll.h>
#include <sys/ioctl.h>

#define SOCKETS_INIT
#define SOCKETS_CLEANUP
typedef int ioctl_arg_t;
typedef int sockopt_t;
typedef struct ip_mreqn ip_mreq_t;

#endif // _WIN32


artl_publisher_t *artl_publisher_init(const artl_description_t *desc,
                                      const char *address, uint16_t port)
{
    // Platform-specific socket library initialization
    SOCKETS_INIT;

    // Create zeroed publisher struct
    artl_publisher_t *pub = calloc(1, sizeof (artl_publisher_t));

    // Set publisher options to default values
    pub->interval_ms = 100;
    pub->compressed = false;

    // Record message size
    pub->message_size = desc->message_size;

    // Create socket
    pub->sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (-1 == pub->sock) {
        error("Failed to open publisher socket\n");
        goto error_free_pub;
    }

    // Enable broadcast on socket
    sockopt_t opt = 1;
    setsockopt(pub->sock, SOL_SOCKET, SO_BROADCAST, &opt, sizeof opt);

    // Set destination address
    pub->address.sin_family = AF_INET;
    pub->address.sin_port = htons(port);
    if (!inet_aton(address, &pub->address.sin_addr)) {
        error("Invalid publisher address: %s\n", address);
        goto error_free_pub;
    }

    // Create description chunks
    artl_create_description_chunks(desc, &pub->nchunks,
                                   &pub->chunks, &pub->chunk_sizes);

    // Send description chunks
    for (size_t i = 0; i < pub->nchunks; ++i) {
        if (sendto(pub->sock, pub->chunks[i], pub->chunk_sizes[i],
                   0, (struct sockaddr *) &pub->address,
                   sizeof pub->address) == -1) {
            error("Failed to send description chunk\n");
            goto error_free_pub;
        }
    }
    pub->last_ms = artl_clock_getms();

    return pub;

error_free_pub:
    artl_publisher_free(pub);
    return NULL;
}


int artl_publisher_update(artl_publisher_t *pub, const void *message)
{
    if (!pub)
        return -1;

    if (message) {
        // Create data chunk
        const size_t buffer_size = pub->message_size + 128;
        void *buffer = malloc(buffer_size);
        int64_t chunk_size;
        if (pub->compressed) {
            chunk_size = artl_create_cdat_chunk(buffer, buffer_size,
                                                message, pub->message_size);
        } else {
            chunk_size = artl_create_udat_chunk(buffer, buffer_size,
                                                 message, pub->message_size);
        }
        if (chunk_size < 0) {
            error("Failed to create UDAT chunk\n");
            free(buffer);
            return -1;
        }

        // Send data chunk
        ssize_t nbytes = sendto(pub->sock, buffer, chunk_size, 0,
                                (struct sockaddr *) &pub->address,
                                sizeof pub->address);
        free(buffer);
        if (-1 == nbytes) {
            error("Failed to send UDAT chunk\n");
            return -1;
        }
    }

    // If data is null or a description chunk hasn't been sent in a
    // while, send one
    if (!message ||
        (pub->interval_ms &&
         artl_clock_getms() >= pub->last_ms + pub->interval_ms)) {
        // Send one of the description chunks
        ssize_t nbytes = sendto(pub->sock,
                                pub->chunks[pub->ichunk],
                                pub->chunk_sizes[pub->ichunk], 0,
                                (struct sockaddr *) &pub->address,
                                sizeof pub->address);
        if (-1 == nbytes) {
            error("Failed to send description chunk\n");
            return -1;
        }

        // Increment description chunk index
        ++pub->ichunk;
        pub->ichunk %= pub->nchunks;

        // Update last description send time
        pub->last_ms = artl_clock_getms();
    }

    return 0;
}


void artl_publisher_free(artl_publisher_t *pub)
{
    if (!pub)
        return;

    // Close socket
    if (-1 != pub->sock)
        close(pub->sock);

    // Free description chunks
    artl_free_description_chunks(pub->nchunks, pub->chunks, pub->chunk_sizes);

    // Free publisher struct
    free(pub);

    // Platform-specific socket library cleanup
    SOCKETS_CLEANUP;
}


artl_subscriber_t *artl_subscriber_init(const char *address, uint16_t port)
{
    // Platform-specific socket library initialization
    SOCKETS_INIT;

    // Create zeroed subscriber struct
    artl_subscriber_t *sub = calloc(1, sizeof (artl_subscriber_t));

    // Set publisher options to default values
    sub->update_timeout_us = 1000;
    sub->disconnect_timeout_ms = 100;
    sub->output_buffer_size = SIZE_MAX;

    // Create socket
    sub->sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (-1 == sub->sock) {
        error("Failed to open subscriber socket\n");
        goto error_free_sub;
    }

    // Enable broadcast and address reuse on socket
    sockopt_t opt = 1;
    setsockopt(sub->sock, SOL_SOCKET, SO_BROADCAST, &opt, sizeof opt);
    setsockopt(sub->sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof opt);

    // Create address struct
    struct sockaddr_in sockaddr;
    sockaddr.sin_family = AF_INET;
    sockaddr.sin_port = htons(port);
    if (!inet_aton(address, &sockaddr.sin_addr)) {
        error("Invalid subscriber address: %s\n", address);
        goto error_free_sub;
    }

    // Bind to address
    if (bind(sub->sock, (struct sockaddr *) &sockaddr, sizeof sockaddr)) {
        error("Failed to bind to subscriber address\n");
        goto error_free_sub;
    }

    // Add group membership if address is multicast
    if (IN_MULTICAST(ntohl(sockaddr.sin_addr.s_addr))) {
        ip_mreq_t group = { .imr_multiaddr = sockaddr.sin_addr };
        setsockopt(sub->sock, IPPROTO_IP, IP_ADD_MEMBERSHIP,
                   (char*) &group, sizeof (group));
    }

    return sub;

error_free_sub:
    artl_subscriber_free(sub);
    return NULL;
}


int artl_subscriber_update(artl_subscriber_t *sub, void *output)
{
    if (!sub)
        return -1;

    // Start timer
    const int64_t time_limit = artl_clock_getus() + sub->update_timeout_us;

    // Exit early if the stream was marked invalid
    if (sub->invalid)
        return -1;

    // Flag for new output data
    int new_output = 0;

    // Loop through RX buffer until it is empty or the timeout is reached
    struct pollfd fd = {.fd = sub->sock, .events = POLLIN, .revents = 0};
    while (poll(&fd, 1, 0) && artl_clock_getus() < time_limit) {
        // Get packet size
        ioctl_arg_t nbytes_avail;
        ioctl(sub->sock, FIONREAD, &nbytes_avail);

        // Get packet
        const size_t buffer_size = nbytes_avail;
        void *buffer = malloc(buffer_size);
        struct sockaddr_storage recv_addr;
        socklen_t recv_addr_size = sizeof recv_addr;
        const ssize_t nbytes =
            recvfrom(sub->sock, buffer, buffer_size, 0,
                     (struct sockaddr *) &recv_addr, &recv_addr_size);
        if (nbytes < 0) {
            error("Error receiving packet\n");
            free(buffer);
            continue;
        }

        // If already connected to a publisher, make sure this packet is from
        // the correct publisher address. This is done instead of calling
        // connect() on the socket because of strange behavior observed on some
        // systems where reconnecting to a subscriber did not work correctly.
        if (sub->connected) {
            if (recv_addr_size != sub->publisher_address_size ||
                memcmp(&sub->publisher_address, &recv_addr, recv_addr_size)) {
                // Packet is from a different source, ignore it
                free(buffer);
                continue;
            }
        }

        // Read chunk
        artl_chunk_type_t chunk_type;
        const void *chunk_data;
        size_t chunk_data_size;
        if (artl_read_chunk(buffer, nbytes,
                            &chunk_type, &chunk_data, &chunk_data_size) < 0) {
            info("Received invalid ARTL chunk\n");
            free(buffer);
            continue;
        }

        // Parse chunk
        bool desc_changed;
        artl_stream_output_options_t out = {
            .desc = &sub->description,
            .buffer = output,
            .buffer_size = &sub->output_buffer_size,
            .map = &sub->map,
            .mapfun = sub->mapfun,
            .mapfun_arg = sub->mapfun_arg,
            .desc_changed = &desc_changed
        };
        const int ret = artl_read_stream(chunk_type, chunk_data,
                                         chunk_data_size, &out);

        // Check return value of artl_read_stream
        switch (ret) {
        case 1: // New output is available
            // Valid packet, update last received time
            sub->last_received_ms = artl_clock_getms();

            // Set new output flag
            sub->received_message = true;
            new_output = 1;
            break;
        case 0: // No new output but packet is valid
            // Valid packet, update last received time
            sub->last_received_ms = artl_clock_getms();

            // Check for STRT chunk and connect to the sending address
            if (!sub->connected &&
                artl_read_chunk_type(buffer, buffer_size) == ARTL_CHUNK_STRT) {
                memcpy(&sub->publisher_address, &recv_addr, recv_addr_size);
                sub->publisher_address_size = recv_addr_size;
                sub->connected = true;
                ++sub->connect_count;
            }
            break;
        case -1: // Invalid chunk
            info("Received invalid ARTL chunk\n");
            break;
        }

        // Check for description format change
        if (desc_changed) {
            if (sub->disconnect_timeout_ms) {
                // Disconnect if disconnecting is enabled
                artl_subscriber_disconnect(sub);
            } else {
                // Disconnecting is disabled, mark connection as invalid
                sub->invalid = true;
                free(buffer);
                return -1;
            }
        }

        // Free chunk data
        free(buffer);
    }

    // Check disconnect timeout, but only if the last call to update did not
    // produce any output. This prevents spurious disconnections when the
    // subscriber update function is called too slowly.
    if (sub->connected && sub->disconnect_timeout_ms &&
        !new_output && sub->last_retval == 0 &&
        artl_clock_getms() > sub->last_received_ms + sub->disconnect_timeout_ms)
        artl_subscriber_disconnect(sub);

    // Save and return the new output flag
    sub->last_retval = new_output;
    return new_output;
}


void artl_subscriber_disconnect(artl_subscriber_t *sub)
{
    if (!sub)
        return;

    // Clear invalid flag
    sub->invalid = false;

    // Zero last returned value
    sub->last_retval = 0;

    // Free and clear map
    artl_map_free(sub->map);
    sub->map = NULL;

    // Clear connected and received_message flags
    sub->connected = false;
    sub->received_message = false;

    // Clear description
    artl_clear_description(&sub->description);

    // Clear publisher address
    memset(&sub->publisher_address, 0, sizeof sub->publisher_address);
    sub->publisher_address_size = 0;
}


void artl_subscriber_free(artl_subscriber_t *sub)
{
    if (!sub)
        return;

    // Close socket
    if (-1 != sub->sock)
        close(sub->sock);

    // Free map
    artl_map_free(sub->map);

    // Clear description
    artl_clear_description(&sub->description);

    // Free subscriber struct
    free(sub);

    // Platform-specific socket library cleanup
    SOCKETS_CLEANUP;
}


/*******************************************************************************
 * Description creation
 ******************************************************************************/

artl_description_t *artl_description_init()
{
    return calloc(1, sizeof (artl_description_t));
}


void artl_description_free(artl_description_t *desc)
{
    artl_clear_description(desc);
    free(desc);
}


artl_field_t *artl_add_field(artl_description_t *desc, const char *name,
                             artl_type_t type, uint16_t nrows, uint16_t ncols)
{
    // Get base size of field type
    int base_size = artl_find_type_size(desc, type);
    if (base_size < 0) {
        error("Unknown or invalid base type %d encountered in "
              "field %s\n", type, name);
        return NULL;
    }

    // Add field to description
    ++desc->nfields;
    desc->fields =
        realloc(desc->fields,
                desc->nfields * sizeof (artl_field_t));

    // Set field data
    artl_field_t *field = &desc->fields[desc->nfields - 1];
    field->type = type;
    field->ncols = ncols;
    field->nrows = nrows;
    if (name)
        field->name = strdup(name);
    else
        field->name = strdup("");

    // Set field size and offset and update entry size
    field->size = base_size * field->nrows * field->ncols;
    field->offset = desc->message_size;
    desc->message_size += field->size;
    return field;
}


artl_enum_t *artl_add_enum(artl_description_t *desc,
                           artl_type_t type,
                           artl_type_t underlying_type)
{
    // Choose a type index automatically
    if (-1 == type) {
        type = 256;
        for (int64_t i = 0; i < desc->nenums; ++i)
            if (desc->enums[i].type >= type)
                type = desc->enums[i].type + 1;
    }

    // Check types
    if (artl_classify_type(type) != ARTL_TYPE_CLASS_EXTENDED) {
        error("Invalid enum type index: %d\n", type);
        return NULL;
    }
    if (!artl_check_type_match(NULL, underlying_type, ARTL_INT)) {
        error("Invalid enum underlying type index: %d\n", type);
        return NULL;
    }

    // Add enum to description
    ++desc->nenums;
    desc->enums =
        realloc(desc->enums,
                desc->nenums * sizeof (artl_enum_t));
    artl_enum_t *enm = &desc->enums[desc->nenums - 1];
    enm->type = type;
    enm->underlying_type = underlying_type;
    enm->labels = NULL;
    enm->nlabels = 0;

    // If the description has a comment, update the comment enums
    if (desc->comment) {
        desc->comment->enums = desc->enums;
        desc->comment->nenums = desc->nenums;
    }

    return enm;
}


artl_enum_label_t *artl_add_label(artl_description_t *desc,
                                  artl_type_t type,
                                  int64_t value, const char *text)
{
    // Find enum in description
    artl_enum_t *enm;
    bool found_enum = false;
    for (int i = 0; i < desc->nenums; ++i) {
        enm = &desc->enums[i];
        // Check if enum types match
        if (enm->type == type) {
            found_enum = true;
            break;
        }
    }

    if (!found_enum) {
        error("Specified enum type does not exist in description: %d\n", type);
        return NULL;
    }

    // Add label to enum
    ++enm->nlabels;
    enm->labels =
        realloc(enm->labels, enm->nlabels * sizeof (artl_enum_label_t));
    artl_enum_label_t *label = &enm->labels[enm->nlabels - 1];
    label->value = value;
    label->text = strdup(text);
    return label;
}


artl_field_t *artl_add_comment(artl_description_t *desc, const void *data,
                               const char *name, artl_type_t type,
                               uint16_t nrows, uint16_t ncols)
{
    // If comment sub-description does not exist, create it
    if (!desc->comment) {
        desc->comment = artl_description_init();
        desc->comment->enums = desc->enums;
        desc->comment->nenums = desc->nenums;
    }

    // Record current comment message size
    const size_t old_comment_message_size = desc->comment->message_size;

    // Add field to comment description
    artl_field_t *field =
        artl_add_field(desc->comment, name, type, nrows, ncols);
    if (!field)
        return NULL;

    // Add data
    desc->comment_data = realloc(desc->comment_data,
                                 desc->comment->message_size);
    memcpy(desc->comment_data + old_comment_message_size,
           data, field->size);

    return field;
}


artl_field_t *artl_add_comment_string(artl_description_t *desc,
                                      const char *string, const char *name)
{
    // Use strlen to get string length and call artl_add_comment
    return artl_add_comment(desc, string, name, ARTL_CHAR, strlen(string), 1);
}


void artl_clear_description(artl_description_t *desc)
{
    if (!desc)
        return;

    // Free field names
    for (int64_t i = 0; i < desc->nfields; ++i)
        free((void *) desc->fields[i].name);

    // Free enums
    for (int64_t i = 0; i < desc->nenums; ++i) {
        // Free enum labels
        for (int64_t j = 0; j < desc->enums[i].nlabels; ++j)
            free((void *) desc->enums[i].labels[j].text);
        free(desc->enums[i].labels);
    }

    // Free comment
    if (desc->comment) {
        desc->comment->enums = NULL;
        desc->comment->nenums = 0;
        artl_description_free(desc->comment);
    }


    // Free fields, enums, and comment data
    free(desc->fields);
    free(desc->enums);
    free(desc->comment_data);

    // Clear the description struct
    memset(desc, 0, sizeof *desc);
}


/*******************************************************************************
 * Description querying
 ******************************************************************************/

bool artl_check_field(const artl_description_t *desc,
                      const char *name, artl_type_t type,
                      int nrows, int ncols)
{
    // Get field
    const artl_field_t *field = artl_find_field(desc, name);
    if (!field)
        return false;

    // Check field information
    return (artl_check_type_match(desc, field->type, type) &&
            (nrows == -1 || nrows == field->nrows) &&
            (ncols == -1 || ncols == field->ncols));
}


bool artl_check_optional_field(const artl_description_t *desc,
                               const char *name, artl_type_t type,
                               int nrows, int ncols)
{
    // Get field
    const artl_field_t *field = artl_find_field(desc, name);
    if (!field)
        return true;

    // Check field information
    return (artl_check_type_match(desc, field->type, type) &&
            (nrows == -1 || nrows == field->nrows) &&
            (ncols == -1 || ncols == field->ncols));
}


const artl_field_t *artl_find_field(const artl_description_t *desc,
                                    const char *name)
{
    if (!desc)
        return NULL;

    // Find field index with given name
    for (int i = 0; i < desc->nfields; ++i) {
        if (!strcmp(name, desc->fields[i].name))
            return &desc->fields[i];
    }

    return NULL;
}


const artl_enum_t *artl_find_enum(const artl_description_t *desc,
                                  artl_type_t type)
{
    if (!desc)
        return NULL;

    // Check that type is a valid enum index
    if (artl_classify_type(type) != ARTL_TYPE_CLASS_EXTENDED)
        return NULL;

    // Find enum with given type index
    for (int i = 0; i < desc->nenums; ++i) {
        if (type == desc->enums[i].type)
            return &desc->enums[i];
    }

    return NULL;
}


const char *artl_find_label(const artl_enum_t *enm, int64_t value)
{
    if (!enm)
        return NULL;

    // Find label with value in enum
    for (int i = 0; i < enm->nlabels; ++i) {
        if (value == enm->labels[i].value)
            return enm->labels[i].text;
    }

    return NULL;
}


const char *artl_find_enum_label(const artl_description_t *desc,
                                 artl_type_t type, int64_t value)
{
    return artl_find_label(artl_find_enum(desc, type), value);
}


int64_t artl_find_field_size(const artl_description_t *desc, const char *name)
{
    // Get field
    const artl_field_t *field = artl_find_field(desc, name);
    if (!field) {
        info("Requested field %s does not exist in description\n", name);
        return -1;
    }

    // Get field information
    const int type_size = artl_find_type_size(desc, field->type);
    return type_size * field->nrows * field->ncols;
}


artl_type_t artl_find_base_type(const artl_description_t *desc,
                                artl_type_t type)
{
    switch (artl_classify_type(type)) {
    case ARTL_TYPE_CLASS_BASE:
        return type;
    case ARTL_TYPE_CLASS_EXTENDED: {
        const artl_enum_t *enm = artl_find_enum(desc, type);
        if (!enm) {
            error("Extended type %d is not defined\n", type);
            return -1;
        }
        type = enm->underlying_type;
        if (artl_classify_type(type) != ARTL_TYPE_CLASS_BASE) {
            error("Invalid enum underlying type index %d\n", type);
            return -1;
        }
        return type;
    }
    case ARTL_TYPE_CLASS_INVALID:
        error("Invalid type index %d\n", type);
        return -1;
    case ARTL_TYPE_CLASS_RESERVED:
        error("Reserved type index %d\n", type);
        return -1;
    default:
        return -1;
    }
}


bool artl_check_type_match(const artl_description_t *desc,
                           artl_type_t type, artl_type_t match)
{
    if (type == match)
        return true;

    switch (match) {
    case ARTL_ANY:
        return true;
    case ARTL_INT:
        return type >= ARTL_U8 && type <= ARTL_I64;
    case ARTL_UINT:
        return type >= ARTL_U8 && type <= ARTL_U64;
    case ARTL_SINT:
        return type >= ARTL_I8 && type <= ARTL_I64;
    case ARTL_FLOAT:
        return type == ARTL_F32 || type == ARTL_F64;
    case ARTL_BYTE:
        return artl_find_type_size(desc, type) == 1;
    case ARTL_ENUM:
        return artl_find_enum(desc, type);
    case ARTL_INT_ENUM:
        return artl_check_type_match(desc, type, ARTL_INT) ||
            artl_check_type_match(desc, type, ARTL_ENUM);
    default:
        return false;
    }
}


int artl_find_type_size(const artl_description_t *desc,
                        artl_type_t type)
{
    // If description is not provided, skip looking up base types and
    // just make sure that the type already is a base type
    if (desc) {
        type = artl_find_base_type(desc, type);
        if (type < 0)
            return -1;
    } else if (artl_classify_type(type) != ARTL_TYPE_CLASS_BASE) {
        return -1;
    }

    return artl_type_size[type];
}


/*******************************************************************************
 * Helper functions for languages with a limited C interface
 ******************************************************************************/

const artl_field_t *artl_get_field(const artl_description_t *description,
                                   int64_t ifield)
{
    if (description && ifield >= 0 && ifield < description->nfields)
        return &description->fields[ifield];
    return NULL;
}


const artl_enum_t *artl_get_enum(const artl_description_t *description,
                                 int64_t ienum)
{
    if (description && ienum >= 0 && ienum < description->nenums)
        return &description->enums[ienum];
    return NULL;
}


const artl_enum_label_t *artl_get_label(const artl_enum_t *enm, int64_t ilabel)
{
    if (enm && ilabel>= 0 && ilabel < enm->nlabels)
        return &enm->labels[ilabel];
    return NULL;
}


int artl_file_open(const char *path)
{
#ifndef _WIN32
    return open(path, O_RDONLY);
#else
    return _open(path, O_RDONLY | O_BINARY);
#endif // _WIN32
}


int artl_file_open_write(const char *path)
{
#ifndef _WIN32
    return open(path, O_WRONLY | O_CREAT | O_TRUNC | O_SYNC, 0644);
#else
    return _open(path, O_WRONLY | O_CREAT | O_TRUNC | O_BINARY, _S_IWRITE);
#endif // _WIN32
}


// Windows should use _lseeki64 for large file support
#ifdef _WIN32
#define lseek _lseeki64
#endif // _WIN32


int64_t artl_file_size(int fd)
{
    const off_t off = lseek(fd, 0, SEEK_CUR);
    const off_t size = lseek(fd, 0, SEEK_END);
    lseek(fd, off, SEEK_SET);
    return size;
}


int64_t artl_file_tell(int fd)
{
    return lseek(fd, 0, SEEK_CUR);
}


void artl_file_seek(int fd, size_t position)
{
    lseek(fd, position, SEEK_SET);
}


void artl_file_close(int fd)
{
    close(fd);
}


void artl_free(void *p)
{
    free(p);
}


/*******************************************************************************
 * Library version symbols
 ******************************************************************************/

const int artl_version_major = 1;
const int artl_version_minor = 1;
const char artl_version_extra[] = "";
