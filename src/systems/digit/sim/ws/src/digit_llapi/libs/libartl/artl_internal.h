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

#ifndef ARTL_INTERNAL_H
#define ARTL_INTERNAL_H

// This file contains declarations for lower-level structures and
// functions used internally by the high-level ARTL API.

#include "artl.h"


/*******************************************************************************
 * Debug printing
 ******************************************************************************/

// Debugging printf macros
#include <stdio.h>
#define error(...) fprintf(stderr, __VA_ARGS__)
// #define info(...) fprintf(stderr, __VA_ARGS__)

#ifndef error
#define error(...)
#endif

#ifndef info
#define info(...)
#endif


/*******************************************************************************
 * Chunk type enumeration
 ******************************************************************************/

typedef enum {
    ARTL_CHUNK_INVALID = 0,

    // Standard casing for recognized chunk types
    ARTL_CHUNK_STRT = 'S' <<  0 | 'T' <<  8 | 'R' << 16 | 'T' << 24,
    ARTL_CHUNK_DESC = 'D' <<  0 | 'E' <<  8 | 'S' << 16 | 'c' << 24,
    ARTL_CHUNK_ENUM = 'E' <<  0 | 'N' <<  8 | 'U' << 16 | 'm' << 24,
    ARTL_CHUNK_CMNT = 'C' <<  0 | 'M' <<  8 | 'N' << 16 | 't' << 24,
    ARTL_CHUNK_DEND = 'D' <<  0 | 'E' <<  8 | 'N' << 16 | 'D' << 24,
    ARTL_CHUNK_UDAT = 'U' <<  0 | 'D' <<  8 | 'A' << 16 | 'T' << 24,
    ARTL_CHUNK_CDAT = 'C' <<  0 | 'D' <<  8 | 'A' << 16 | 'T' << 24,

    // Case flags
    ARTL_CHUNK_CASE0 = 1 << 5,
    ARTL_CHUNK_CASE1 = 1 << 13,
    ARTL_CHUNK_CASE2 = 1 << 21,
    ARTL_CHUNK_CASE3 = 1 << 29,
    ARTL_CHUNK_DESCRIPTION_CRC = ARTL_CHUNK_CASE3,
    ARTL_CHUNK_DOWNCASE = (ARTL_CHUNK_CASE0 | ARTL_CHUNK_CASE1 |
                           ARTL_CHUNK_CASE2 | ARTL_CHUNK_CASE3),
} artl_chunk_type_t;


// Standardizes casing for recognized chunk types.
artl_chunk_type_t artl_standardize_chunk_type(artl_chunk_type_t type);


/*******************************************************************************
 * Data format structs
 ******************************************************************************/

typedef struct __attribute__((packed)) {
    uint32_t data_size;
    artl_chunk_type_t type;
} artl_chunk_header_t;

typedef struct __attribute__((packed)) {
    uint16_t type;
    uint16_t nrows;
    uint16_t ncols;
    uint16_t name_size;
    char name[];
} artl_field_descriptor_t;

typedef struct __attribute__((packed)) {
    uint16_t type;
    uint16_t underlying_type;
} artl_enum_header_t;


/*******************************************************************************
 * Timing
 ******************************************************************************/

// Returns the current time in microseconds. Useful for timing intervals.
int64_t artl_clock_getus(void);

// Returns the current time in milliseconds. Useful for timing intervals.
int64_t artl_clock_getms(void);


/*******************************************************************************
 * ARTL type utilities
 ******************************************************************************/

// Enum for classes of ARTL type indices
typedef enum {
    ARTL_TYPE_CLASS_BASE,
    ARTL_TYPE_CLASS_EXTENDED,
    ARTL_TYPE_CLASS_INVALID = -1,
    ARTL_TYPE_CLASS_RESERVED = -2
} artl_type_class_t;

// Size in bytes of each base type
extern const int artl_type_size[];

// Returns the class of a given type index.
artl_type_class_t artl_classify_type(artl_type_t type);


/*******************************************************************************
 * Description creation
 ******************************************************************************/

// Adds information from a DESC or ENUM chunk a description
// struct. Returns 0 on success, or -1 on failure.
int artl_add_chunk_to_description(artl_description_t *description,
                                  artl_chunk_type_t chunk_type,
                                  const void *chunk_data,
                                  size_t chunk_data_size);

// Validates a description against the information in a DEND
// chunk. Returns 0 if the description and DEND chunk are valid, or -1
// otherwise.
int artl_end_description(artl_description_t *description,
                         artl_chunk_type_t chunk_type,
                         const void *chunk_data, size_t chunk_data_size);


/*******************************************************************************
 * Structure mapping
 ******************************************************************************/

// Reads a single log entry from a chunk and copies it to the output
// struct using the provided field map. Returns 0 on success, or -1 on
// failure.
int artl_get_mapped_chunk(const artl_map_t *map, void *output,
                          artl_chunk_type_t chunk_type,
                          const void *chunk_data, size_t chunk_data_size);

// Core implementation for all of the field mapping functions.
bool artl_map_partial_field_impl(artl_map_t *map, const char *field,
                                 int64_t output_offset, artl_type_t output_type,
                                 int64_t istart, int64_t nelements,
                                 bool output_absolute);


/*******************************************************************************
 * ARTL reading
 ******************************************************************************/

// Reads a single chunk from a buffer. On success, returns the total
// length of the chunk and passes back the type of the chunk (or
// ARTL_UNKNOWN if the chunk is valid but not a supported type), a
// pointer to the chunk data section, and the length of the chunk data
// section. If the chunk is invalid, returns -1.
int64_t artl_read_chunk(const void *artl_data, size_t artl_data_size,
                        artl_chunk_type_t *chunk_type,
                        const void **chunk_data, size_t *chunk_data_size);

// Similar to artl_read_chunk, but reads from a file descriptor
// instead of a buffer. The chunk data is returned in a buffer
// allocated with malloc. If a partial or invalid chunk is
// encountered, chunk_data is NULL and the function returns -1. If the
// end of file is encountered immediately, chunk_data is NULL and the
// function returns 0. If a chunk is successfully read, the function
// returns the total length of the chunk.
int64_t artl_read_chunk_fd(int fd, artl_chunk_type_t *chunk_type,
                           void **chunk_data, size_t *chunk_data_size);

// Reads the chunk type from a chunk in a buffer.
artl_chunk_type_t artl_read_chunk_type(const void *artl_data,
                                       size_t artl_data_size);

// Gets data from a UDAT or CDAT chunk. Returns the size of the
// (uncompressed) data, or -1 on failure.
int64_t artl_get_data(void *buffer, size_t buffer_size,
                      artl_chunk_type_t chunk_type,
                      const void *chunk_data, size_t chunk_data_size);

// Returns the size of the (uncompressed) data in a UDAT or CDAT
// chunk, or -1 if the chunk type is incorrect or a decompression
// error occured.
int64_t artl_get_data_size(artl_chunk_type_t chunk_type,
                           const void *chunk_data, size_t chunk_data_size);

// Structure for artl_read_stream output options.
// If desc is provided, it will be filled out until complete.
// Data outputs can be:
//   buffer alone, which will not check for overflow.
//   buffer and buffer_size, which errors if the write would overflow.
//   buffer_ptr and buffer_size, which allocates a buffer and returns its size.
// map and mapfun require each other along with desc and a data output.
// mapfun_arg is provided as the second argument to mapfun.
// desc_changed requires desc.
typedef struct {
    artl_description_t *desc;
    void **buffer_ptr;
    void *buffer;
    size_t *buffer_size;
    artl_map_t **map;
    artl_map_t *(*mapfun)(const artl_description_t *, void *);
    void *mapfun_arg;
    bool *desc_changed;
    size_t *buffer_size_required;
} artl_stream_output_options_t;

// Reads reads a chunk from a stream. Can fill out a description,
// output raw data, output mapped data, and signal various conditions,
// depending on which output options are set. Returns 1 if new message
// data is available in the output, 0 if the chunk was valid but did
// not result in new message data, or -1 if an error occured.
int artl_read_stream(artl_chunk_type_t chunk_type, const void *chunk_data,
                     size_t chunk_data_size, artl_stream_output_options_t *out);

// Adds information in a DESC chunk to a description struct. Returns 0
// on success, or -1 on failure.
int artl_add_desc_chunk(artl_description_t *description,
                        const void *chunk_data, size_t chunk_data_size);

// Adds information in an ENUM chunk to a description struct. Returns
// 0 on success, or -1 on failure.
int artl_add_enum_chunk(artl_description_t *description,
                        const void *chunk_data, size_t chunk_data_size);

// Adds information in a CMNT chunk to a description struct. Returns 0
// on success, or -1 on failure.
int artl_add_cmnt_chunk(artl_description_t *desc,
                        const void *chunk_data, size_t chunk_data_size);

// Unpacks data from a UDAT chunk into a fixed size buffer and returns
// the unpacked size, or -1 on failure.
int64_t artl_read_udat(void *buffer, size_t buffer_size,
                       const void *chunk_data, size_t chunk_data_size);

// Unpacks data from a CDAT chunk into a fixed size buffer and returns
// the unpacked size, or -1 on failure.
int64_t artl_read_cdat(void *buffer, size_t buffer_size,
                       const void *chunk_data, size_t chunk_data_size);

// Returns the size of the decompressed data in a CDAT chunk, or -1 if
// the chunk is invalid.
int64_t artl_scan_cdat(const void *chunk_data, size_t chunk_data_size);


/*******************************************************************************
 * ARTL writing
 ******************************************************************************/

// Chunk writer utility struct.
typedef struct {
    void *data;
    size_t size;
} artl_chunk_writer_t;

// Writes a STRT chunk indicating the start of an ARTL file or
// stream. Returns the size of the chunk, or -1 if the output buffer
// was not large enough.
int64_t artl_create_strt_chunk(void *buffer, size_t buffer_size);

// Writes a DEND chunk indicating the end of a complete
// description. Takes in the crc returned by the last call to
// artl_end_chunk. Returns the size of the chunk, or -1 if the output
// buffer was not large enough.
int64_t artl_create_dend_chunk(void *buffer, size_t buffer_size,
                               uint32_t description_crc);

// Starts a description chunk. Returns a chunk writer object used to
// add fields to the description. Must be freed with
// artl_chunk_writer_free().
artl_chunk_writer_t *artl_desc_chunk_start(void);

// Adds a field to a description chunk.
void artl_desc_chunk_add_field(artl_chunk_writer_t *writer,
                               const char *name, artl_type_t type,
                               uint16_t nrows, uint16_t ncols);

// Starts an enum type definition chunk. Returns a chunk writer object
// used to add value/label pairs to the enum.
artl_chunk_writer_t *artl_enum_chunk_start(artl_type_t type,
                                           artl_type_t underlying_type);

// Adds a value/label text pair to an enum definition chunk.
void artl_enum_chunk_add_label(artl_chunk_writer_t *writer,
                               int64_t value, const char *label);

// Starts a comment chunk. Adds the comment data to the chunk, then
// eturns a chunk writer object used to add fields to the comment
// description. Must be freed with artl_chunk_writer_free().
artl_chunk_writer_t *artl_cmnt_chunk_start(void *data, uint32_t data_size);

// Adds a field to a comment chunk.
void artl_cmnt_chunk_add_field(artl_chunk_writer_t *writer,
                               const char *name, artl_type_t type,
                               uint16_t nrows, uint16_t ncols);

// Finalizes a description or enumeration chunk. Takes in the CRC
// returned by the last call to artl_end_chunk, or 0 if this is the
// first DESC or ENUM chunk. Returns the accumulated CRC of this and
// the preceeding DESC and ENUM chunks. Output parameters return a
// pointer to the chunk data and the length of the chunk.
uint32_t artl_chunk_end(artl_chunk_writer_t *writer,
                        uint32_t description_crc,
                        void **chunk_data, size_t *chunk_size);

// Returns an initialized chunk writer object.
artl_chunk_writer_t *artl_chunk_writer_init(void);

// Frees the memory used by a chunk writer object. Does nothing if
// writer is null.
void artl_chunk_writer_free(artl_chunk_writer_t *writer);


#endif // ARTL_INTERNAL_H
