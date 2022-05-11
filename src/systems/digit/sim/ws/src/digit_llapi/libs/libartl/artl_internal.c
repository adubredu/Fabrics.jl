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

#include "artl_internal.h"
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <zstd.h>
#include "crc32c.h"


/*******************************************************************************
 * Chunk type enumeration
 ******************************************************************************/

artl_chunk_type_t artl_standardize_chunk_type(artl_chunk_type_t type)
{
    // Standardize casing for recognized types
#define STANDARDIZE(TYPE) case TYPE | ARTL_CHUNK_DOWNCASE: type = TYPE; break
    switch (type | ARTL_CHUNK_DOWNCASE) {
        STANDARDIZE(ARTL_CHUNK_STRT);
        STANDARDIZE(ARTL_CHUNK_DESC);
        STANDARDIZE(ARTL_CHUNK_ENUM);
        STANDARDIZE(ARTL_CHUNK_DEND);
        STANDARDIZE(ARTL_CHUNK_UDAT);
        STANDARDIZE(ARTL_CHUNK_CDAT);
    default:
        break;
    }

    return type;
}


/*******************************************************************************
 * Timing
 ******************************************************************************/

int64_t artl_clock_getus(void)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return now.tv_sec * 1000000 + now.tv_nsec / 1000;
}


int64_t artl_clock_getms(void)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return now.tv_sec * 1000 + now.tv_nsec / 1000000;
}


/*******************************************************************************
 * ARTL type utilities
 ******************************************************************************/

const int artl_type_size[] = {1, 2, 4, 8, 1, 2, 4, 8, 4, 8, 1, 1, 1};

int artl_classify_type(artl_type_t type)
{
    // Invalid indices
    if (type < 0)
        return ARTL_TYPE_CLASS_INVALID;

    // Base types
    if (type < (int) (sizeof artl_type_size / sizeof artl_type_size[0]))
        return ARTL_TYPE_CLASS_BASE;

    // Reserved indices
    if (type < UINT8_MAX)
        return ARTL_TYPE_CLASS_RESERVED;

    // Enum types
    if (type < INT16_MAX)
        return ARTL_TYPE_CLASS_EXTENDED;

    // Reserved indices
    if (type < UINT16_MAX)
        return ARTL_TYPE_CLASS_RESERVED;

    return ARTL_TYPE_CLASS_INVALID;
}


/*******************************************************************************
 * Description creation
 ******************************************************************************/

int artl_add_chunk_to_description(artl_description_t *description,
                                  artl_chunk_type_t chunk_type,
                                  const void *chunk_data,
                                  size_t chunk_data_size)
{
    // Check chunk type
    switch (artl_standardize_chunk_type(chunk_type)) {
    case ARTL_CHUNK_DESC:
        if (artl_add_desc_chunk(description, chunk_data, chunk_data_size) < 0)
            return -1;
        break;
    case ARTL_CHUNK_ENUM:
        if (artl_add_enum_chunk(description, chunk_data, chunk_data_size) < 0)
            return -1;
        break;
    case ARTL_CHUNK_CMNT:
        if (artl_add_cmnt_chunk(description, chunk_data, chunk_data_size) < 0)
            return -1;
        break;
    default:
        break;
    }

    // Accumulate description CRC
    if (artl_standardize_chunk_type(chunk_type) & ARTL_CHUNK_DESCRIPTION_CRC) {
        description->crc = crc32c(description->crc,
                                  &chunk_type, sizeof chunk_type);
        description->crc = crc32c(description->crc,
                                  chunk_data, chunk_data_size);
    }

    return 0;
}


int artl_end_description(artl_description_t *description,
                         artl_chunk_type_t chunk_type,
                         const void *chunk_data, size_t chunk_data_size)
{
    // Check chunk type
    if (chunk_type != ARTL_CHUNK_DEND) {
        error("Wrong chunk type\n");
        return -1;
    }

    info("Reading DEND chunk at %p, size %lu\n", chunk_data, chunk_data_size);

    // Get CRC from DEND chunk
    uint32_t dend_crc;
    if (chunk_data_size != sizeof dend_crc) {
        error("Invalid DEND chunk size\n");
        return -1;
    }
    memcpy(&dend_crc, chunk_data, sizeof dend_crc);

    // Check the the description CRC
    if (description->crc != dend_crc) {
        info("Invalid description CRC\n");
        return -1;
    }

    // Set complete flag
    description->complete = true;

    info("Finished reading DEND chunk\n");

    return 0;
}


/*******************************************************************************
 * Structure mapping
 ******************************************************************************/

int artl_get_mapped_chunk(const artl_map_t *map, void *output,
                          artl_chunk_type_t chunk_type,
                          const void *chunk_data, size_t chunk_data_size)
{
    // Get size required for data buffer
    const int64_t buffer_size =
        artl_get_data_size(chunk_type, chunk_data, chunk_data_size);
    if (buffer_size < 0)
        return -1;

    // Check that data size is the same size as the expected message
    if ((size_t) buffer_size != map->description->message_size) {
        error("Received message size not equal to expected message size "
              "(expected %zu, received %zu)\n",
              map->description->message_size, (size_t) buffer_size);
        return -1;
    }

    // Allocate decompressed data buffer and copy data
    void *buffer = malloc(buffer_size);
    const int64_t data_size = artl_get_data(buffer, buffer_size, chunk_type,
                                            chunk_data, chunk_data_size);
    if (data_size < 0) {
        free(buffer);
        return -1;
    }

    // Copy one struct into the output
    artl_map_get_array(map, output, 1, 1, buffer, buffer_size);

    // Free decompressed data buffer
    free(buffer);

    return 0;
}


bool artl_map_partial_field_impl(artl_map_t *map, const char *field,
                                 int64_t output_offset, artl_type_t output_type,
                                 int64_t istart, int64_t nelements,
                                 bool output_absolute)
{
    // Get field
    const artl_field_t *f = artl_find_field(map->description, field);
    if (!f) {
        info("Requested field %s does not exist in description\n", field);
        return false;
    }

    // Validate partial field indices
    const int64_t max_num = f->ncols * f->nrows;
    if (istart < 0 || nelements < 0) {
        error("Invalid values passed to artl_map_partial_field\n");
        return false;
    }
    if (max_num < istart + nelements) {
        info("Field %s is not large enough for requested mapping\n", field);
        return false;
    }

    // Get field size and source offset for field subindices
    const int buffer_element_size =
        artl_find_type_size(map->description, f->type);
    if (buffer_element_size < 0) {
        error("Invalid field type for %s in description\n", field);
        return false;
    }
    const size_t buffer_size = buffer_element_size * nelements;
    const size_t buffer_offset = f->offset + buffer_element_size * istart;

    // If destination offset was not supplied, put it after the last
    // mapped field (or zero)
    if (!output_absolute && output_offset < 0) {
        if (!map->nregions) {
            output_offset = 0;
        } else {
            output_offset = map->regions[map->nregions - 1].output_offset +
                map->regions[map->nregions - 1].output_size;
        }
    }

    // Check type conversion
    const artl_type_t buffer_type =
        artl_find_base_type(map->description, f->type);
    int num, den;
    if (!artl_check_type_conversion(buffer_type, output_type, &num, &den)) {
        error("Invalid type conversion for %s (%d to %d)\n",
              field, buffer_type, output_type);
        return false;
    }

    // Output field size after conversion
    const size_t output_size = (buffer_size * num) / den;

    // Check for overlapping map regions
    for (int i = 0; i < map->nregions; ++i) {
        const artl_map_region_t *region = &map->regions[i];
        // Compare two relative offsets
        if (!output_absolute && !region->output_absolute) {
            size_t off = output_offset;
            if ((off <= region->output_offset &&
                 off + output_size > region->output_offset) ||
                (region->output_offset <= off &&
                 region->output_offset + region->output_size > off)) {
                error("Requested field mapping for %s overlaps with "
                      "previously mapped field\n", field);
                return false;
            }
        }
        // Compare two absolute addresses
        if (output_absolute && region->output_absolute) {
            void *addr = (void *) output_offset;
            if ((addr <= region->output_address &&
                 addr + output_size > region->output_address) ||
                (region->output_address <= addr &&
                 region->output_address + region->output_size > addr)) {
                error("Requested field mapping for %s overlaps with "
                      "previously mapped field\n", field);
                return false;
            }
        }
    }

    // Add map region
    ++map->nregions;
    map->regions = realloc(map->regions,
                           map->nregions * sizeof (artl_map_region_t));
    artl_map_region_t *region = &map->regions[map->nregions - 1];
    region->buffer_offset = buffer_offset;
    region->buffer_size = buffer_size;
    region->buffer_type = buffer_type;
    region->output_size = output_size;
    region->output_type = output_type;
    region->output_absolute = output_absolute;
    if (output_absolute)
        region->output_address = (void *) output_offset;
    else
        region->output_offset = output_offset;

    // Update required minimum required size of the output buffer
    if (!output_absolute) {
        const size_t map_end = output_offset + output_size;
        if (map_end > map->min_output_size)
            map->min_output_size = map_end;
    }

    return true;
}


/*******************************************************************************
 * ARTL reading
 ******************************************************************************/

int64_t artl_read_chunk(const void *artl_data, size_t artl_data_size,
                        artl_chunk_type_t *chunk_type,
                        const void **chunk_data, size_t *chunk_data_size)
{
    // Initialize output variables
    *chunk_type = ARTL_CHUNK_INVALID;
    *chunk_data = NULL;
    *chunk_data_size = 0;

    // Check remaining size
    if (artl_data_size < sizeof (artl_chunk_header_t)) {
        error("Partial chunk encountered\n");
        return -1;
    }

    // Read chunk header
    const artl_chunk_header_t *header = artl_data;

    // Check chunk size
    const size_t chunk_size =
        sizeof *header + header->data_size + sizeof (uint32_t);
    if (chunk_size > artl_data_size) {
        error("Partial chunk encountered\n");
        return -1;
    }

    // Check chunk CRC
    const size_t crc_size =
        sizeof header->type + header->data_size;
    const uint32_t crc = crc32c(0, &header->type, crc_size);
    const uint32_t *crc_addr =
        (void *)&header->type + sizeof header->type + header->data_size;
    if (crc != *crc_addr) {
        error("Bad chunk CRC\n");
        return -1;
    }

    // Set chunk type, data, and size outputs
    *chunk_type = header->type;
    *chunk_data = artl_data + sizeof *header;
    *chunk_data_size = header->data_size;

    return sizeof *header + *chunk_data_size + sizeof crc;
}


int64_t artl_read_chunk_fd(int fd, artl_chunk_type_t *chunk_type,
                           void **chunk_data, size_t *chunk_data_size)
{
    // Initialize output variables
    *chunk_type = ARTL_CHUNK_INVALID;
    *chunk_data = NULL;
    *chunk_data_size = 0;

    // Read chunk header
    artl_chunk_header_t header;
    const ssize_t header_nbytes = read(fd, &header, sizeof header);
    if (header_nbytes < 0) {
        error("Error reading from file descriptor\n");
        return -1;
    } else if (0 == header_nbytes) {
        return 0; // Indicate end of file
    } else if (header_nbytes != (ssize_t) sizeof header) {
        error("Partial chunk encountered\n");
        return -1;
    }

    // Read chunk data
    *chunk_data = malloc(header.data_size);
    if (!*chunk_data) {
        error("Failed to allocate chunk data memory\n");
        return -1;
    }
    const ssize_t data_nbytes = read(fd, *chunk_data, header.data_size);
    if (data_nbytes != header.data_size) {
        error("Partial chunk encountered\n");
        goto error_free_chunk_data;
    }

    // Check chunk CRC
    uint32_t crc_expected = crc32c(0, &header.type, sizeof header.type);
    crc_expected = crc32c(crc_expected, *chunk_data, header.data_size);
    uint32_t crc;
    const ssize_t crc_nbytes = read(fd, &crc, sizeof crc);
    if (crc_nbytes != sizeof crc) {
        error("Partial chunk encountered\n");
        goto error_free_chunk_data;
    }
    if (crc != crc_expected) {
        error("Bad chunk CRC\n");
        goto error_free_chunk_data;
    }

    // Set chunk type and size outputs
    *chunk_type = header.type;
    *chunk_data_size = header.data_size;

    // Return total chunk length read from file descriptor
    return header_nbytes + data_nbytes + crc_nbytes;

error_free_chunk_data:
    free(*chunk_data);
    *chunk_data = NULL;
    return -1;
}


artl_chunk_type_t artl_read_chunk_type(const void *artl_data,
                                       size_t artl_data_size)
{
    // Check that chunk data is large enough for a header
    if (artl_data_size < sizeof (artl_chunk_header_t))
        return ARTL_CHUNK_INVALID;

    // Read chunk header
    const artl_chunk_header_t *header = artl_data;
    return artl_standardize_chunk_type(header->type);
}


int64_t artl_get_data(void *buffer, size_t buffer_size,
                      artl_chunk_type_t chunk_type,
                      const void *chunk_data, size_t chunk_data_size)
{
    switch (artl_standardize_chunk_type(chunk_type)) {
    case ARTL_CHUNK_UDAT:
        return artl_read_udat(buffer, buffer_size, chunk_data, chunk_data_size);
    case ARTL_CHUNK_CDAT:
        return artl_read_cdat(buffer, buffer_size, chunk_data, chunk_data_size);
    default:
        error("Wrong chunk type\n");
        return -1;
    }
}


int64_t artl_get_data_size(artl_chunk_type_t chunk_type,
                           const void *chunk_data, size_t chunk_data_size)
{
    switch (artl_standardize_chunk_type(chunk_type)) {
    case ARTL_CHUNK_UDAT:
        return chunk_data_size;
    case ARTL_CHUNK_CDAT:
        return artl_scan_cdat(chunk_data, chunk_data_size);
    default:
        error("Wrong chunk type\n");
        return -1;
    }
}


int artl_read_stream(artl_chunk_type_t chunk_type, const void *chunk_data,
                     size_t chunk_data_size, artl_stream_output_options_t *out)
{
    // Unpack output options struct
    artl_description_t *desc = out->desc;
    void **buffer_ptr = out->buffer_ptr;
    void *buffer = out->buffer;
    size_t *buffer_size = out->buffer_size;
    artl_map_t **map = out->map;
    artl_map_t *(*mapfun)(const artl_description_t *, void *) = out->mapfun;
    void *mapfun_arg = out->mapfun_arg;
    bool *desc_changed = out->desc_changed;
    size_t *buffer_size_required = out->buffer_size_required;

    // Validate output options
    const bool malloc_output = buffer_ptr && buffer_size;
    const bool direct_output = buffer && !malloc_output;
    const bool has_output = direct_output || malloc_output;
    const bool has_map = mapfun && map && has_output && desc;
    if ((malloc_output && buffer) ||
        (!has_output && (buffer || buffer_ptr || buffer_size)) ||
        (!has_map && mapfun)) {
        error("Invalid data output options provided to artl_read_stream\n");
        return -1;
    }

    // Initialize outputs
    if (malloc_output) {
        *buffer_ptr = NULL;
        *buffer_size = 0;
    }
    if (desc_changed)
        *desc_changed = false;
    if (buffer_size_required)
        *buffer_size_required = 0;

    // Set pointers for missing outputs used by state machine
    size_t buffer_size_default = INT64_MAX;
    if (!buffer_size)
        buffer_size = &buffer_size_default;
    size_t buffer_size_required_default = 0;
    if (!buffer_size_required)
        buffer_size_required = &buffer_size_required_default;

    // Check chunk type
    switch (artl_standardize_chunk_type(chunk_type)) {
    case ARTL_CHUNK_STRT:
        // Prepare to read description and create a new map if a
        // complete valid description has not yet been received
        info("Reading STRT chunk at %p, size %lu\n",
             chunk_data, chunk_data_size);
        info("Finished reading STRT chunk\n");
        if (desc && !desc->complete) {
            artl_clear_description(desc);
            if (has_map) {
                artl_map_free(*map);
                *map = NULL;
            }
        }
        if (desc)
            desc->started = true;
        break;
    default:
        // For all other chunks, try adding them to the description
        // Unrecognized chunks will be included in the CRC if they
        // have the correct case bit set
        if (desc && desc->started && !desc->complete) {
            if (artl_add_chunk_to_description(desc, chunk_type, chunk_data,
                                              chunk_data_size) < 0) {
                // Bad chunk added to description
                return -1;
            }
        }
        break;
    case ARTL_CHUNK_DEND:
        // Finalize and check message description
        if (desc && desc->started && !desc->complete &&
            artl_end_description(desc, chunk_type,
                                 chunk_data, chunk_data_size) == 0) {
            // Try to create mapping if a mapping function was provided
            if (has_map) {
                *map = mapfun(desc, mapfun_arg);

                // Check that the output size is sufficient for the mapping
                if (buffer_size && *map &&
                    (*map)->min_output_size > *buffer_size) {
                    error("Insufficient output size for struct mapping\n");
                    *map = NULL;
                    return -1;
                }
            }
        } else if (desc && desc_changed && desc->complete &&
                   artl_end_description(desc, chunk_type,
                                        chunk_data, chunk_data_size) < 0) {
            // Check for a new conflicting message description
            *desc_changed = true;
        }
        break;
    case ARTL_CHUNK_UDAT:
    case ARTL_CHUNK_CDAT:
        // Get required buffer size for output
        if (has_map) {
            if (*map)
                *buffer_size_required = (*map)->min_output_size;
        } else {
            const int64_t size =
                artl_get_data_size(chunk_type, chunk_data, chunk_data_size);
            if (size < 0)
                return -1;
            *buffer_size_required = size;
        }

        // Skip rest if no output, mapping failed, or description exists
        // but isn't complete
        if ((!has_output) ||
            (has_map && !*map) ||
            (desc && !desc->complete))
            break;

        // Check that buffer is large enough when using direct output
        if (direct_output && *buffer_size_required > *buffer_size) {
            info("Output buffer is not large enough for data\n");
            return 0;
        }

        // Allocate buffer when using malloc output
        if (malloc_output) {
            *buffer_size = *buffer_size_required;
            buffer = malloc(*buffer_size);
            if (!buffer) {
                error("Failed to allocate decompressed data buffer\n");
                return -1;
            }
            *buffer_ptr = buffer;
        }

        // Get data
        if (has_map) {
            // Get mapped data
            if (artl_get_mapped_chunk(*map, buffer, chunk_type, chunk_data,
                                      chunk_data_size) < 0) {
                goto error_free_malloc_output;
            }
        } else {
            // Get raw data
            if ((int64_t) *buffer_size_required !=
                artl_get_data(buffer, *buffer_size, chunk_type,
                              chunk_data, chunk_data_size)) {
                goto error_free_malloc_output;
            }
        }
        return 1;
    }

    // If control reaches here, no new output data was produced
    return 0;

error_free_malloc_output:
    // If malloc output is used, free and clear the pointer and size outputs
    if (malloc_output) {
        free(*buffer_ptr);
        *buffer_ptr = NULL;
        buffer_size = 0;
    }
    return -1;
}


int artl_add_desc_chunk(artl_description_t *desc,
                        const void *chunk_data, size_t chunk_data_size)
{
    info("Reading DESC chunk at %p, size %lu\n", chunk_data, chunk_data_size);

    // Read fields
    while (chunk_data_size) {
        const artl_field_descriptor_t *field_desc = chunk_data;

        // Check field descriptor size before name bytes
        if (sizeof *field_desc > chunk_data_size) {
            error("Insufficient field descriptor size\n");
            return -1;
        }

        // Check total field descriptor size
        const size_t field_size =
            sizeof *field_desc + field_desc->name_size;
        if (field_size > chunk_data_size) {
            error("Insufficient field descriptor size\n");
            return -1;
        }

        // Add field to description
        ++desc->nfields;
        desc->fields =
            realloc(desc->fields, desc->nfields * sizeof (artl_field_t));

        // Set field data
        artl_field_t *field = &desc->fields[desc->nfields - 1];
        field->type = field_desc->type;
        field->ncols = field_desc->ncols;
        field->nrows = field_desc->nrows;
        char *name = malloc(field_desc->name_size + 1);
        memcpy(name, field_desc->name, field_desc->name_size);
        name[field_desc->name_size] = '\0';
        field->name = name;

        // Set field size and offset and update entry size
        int base_size = artl_find_type_size(desc, field_desc->type);
        if (base_size < 0) {
            base_size = 0;
            error("Unknown or invalid base type (%d) encountered in "
                  "field %s\n", field_desc->type, field->name);
        }
        field->size = base_size * field->nrows * field->ncols;
        field->offset = desc->message_size;
        desc->message_size += field->size;

        // Go to next field
        chunk_data += field_size;
        chunk_data_size -= field_size;
    }

    info("Finished reading DESC chunk\n");
    return 0;
}


int artl_add_enum_chunk(artl_description_t *desc,
                        const void *chunk_data, size_t chunk_data_size)
{
    info("Reading ENUM chunk at %p, size %lu\n", chunk_data, chunk_data_size);

    // Check enum header validity
    const artl_enum_header_t *header = chunk_data;
    if (sizeof *header > chunk_data_size) {
        error("Insufficient ENUM chunk data size\n");
        return -1;
    }
    if (artl_classify_type(header->type) != ARTL_TYPE_CLASS_EXTENDED) {
        error("Bad enum type (%d)\n", header->type);
        return -1;
    }
    if (!artl_check_type_match(NULL, header->underlying_type, ARTL_INT)) {
        error("Bad enum underlying type (%d)\n",
              header->underlying_type);
        return -1;
    }

    // Get underlying type size
    const size_t enum_type_size =
        artl_type_size[header->underlying_type];

    // Add enum to description
    // First, try to find matching existing enum in description
    artl_enum_t *enm;
    bool found_enum = false;
    for (int i = 0; i < desc->nenums; ++i) {
        enm = &desc->enums[i];
        // Check if enum types match
        if (enm->type == header->type) {
            if (enm->underlying_type != header->underlying_type)
                error("Underlying type of new ENUM chunk is "
                      "inconsistent with previous ENUM chunks\n");
            found_enum = true;
            break;
        }
    }

    // Enum with specified type not found, add a new enum
    if (!found_enum)
        enm = artl_add_enum(desc, header->type, header->underlying_type);

    // Go to first label
    chunk_data += sizeof *header;
    chunk_data_size -= sizeof *header;

    while (chunk_data_size) {
        // Check size without label text
        if (enum_type_size + sizeof (uint16_t) > chunk_data_size) {
            error("Insufficient enum label size\n");
            return -1;
        }

        // Get correctly sign-extended enum value
        int64_t value = 0;
        switch (header->underlying_type) {
        case ARTL_U8: value = *(uint8_t *) chunk_data; break;
        case ARTL_U16: value = *(uint16_t *) chunk_data; break;
        case ARTL_U32: value = *(uint32_t *) chunk_data; break;
        case ARTL_U64: value = *(uint64_t *) chunk_data; break;
        case ARTL_I8: value = *(int8_t *) chunk_data; break;
        case ARTL_I16: value = *(int16_t *) chunk_data; break;
        case ARTL_I32: value = *(int32_t *) chunk_data; break;
        case ARTL_I64: value = *(int64_t *) chunk_data; break;
        }

        // Check label entry size
        const size_t text_size = *((uint16_t *)(chunk_data + enum_type_size));
        const size_t label_entry_size =
            enum_type_size + sizeof (uint16_t) + text_size;
        if (label_entry_size > chunk_data_size) {
            error("Insufficient enum label size\n");
            return -1;
        }

        // Add label to enum
        ++enm->nlabels;
        enm->labels =
            realloc(enm->labels, enm->nlabels * sizeof (artl_enum_label_t));
        artl_enum_label_t *label = &enm->labels[enm->nlabels - 1];
        label->value = value;
        char *text = malloc(text_size + 1);
        memcpy(text,
               chunk_data + enum_type_size + sizeof (uint16_t), text_size);
        text[text_size] = '\0';
        label->text = text;

        // Go to next label
        chunk_data += label_entry_size;
        chunk_data_size -= label_entry_size;
    }

    info("Finished reading ENUM chunk\n");
    return 0;
}


int artl_add_cmnt_chunk(artl_description_t *desc,
                        const void *chunk_data, size_t chunk_data_size)
{
    info("Reading CMNT chunk at %p, size %lu\n", chunk_data, chunk_data_size);

    // Read data size
    const uint32_t *data_size = chunk_data;
    if (sizeof *data_size > chunk_data_size) {
        error("Insufficent comment data length size\n");
        return -1;
    }
    chunk_data += sizeof *data_size;
    chunk_data_size -= sizeof *data_size;

    // Read chunk data
    if (*data_size > chunk_data_size) {
        error("Insufficient comment data size\n");
        return -1;
    }
    const void *data = chunk_data;
    chunk_data += *data_size;
    chunk_data_size -= *data_size;

    // If comment sub-description does not exist, create it
    if (!desc->comment) {
        desc->comment = artl_description_init();
        desc->comment->enums = desc->enums;
        desc->comment->nenums = desc->nenums;
    }

    // Record current and new comment message size
    const size_t old_comment_message_size = desc->comment->message_size;
    const size_t new_comment_data_size = old_comment_message_size + *data_size;

    // Add fields to comment sub-description
    if (artl_add_desc_chunk(desc->comment, chunk_data, chunk_data_size) < 0)
        return -1;

    // Check that fields are consistent with data length
    if (desc->comment->message_size != new_comment_data_size) {
        error("Comment chunk data length is inconsistent "
              "with comment field size\n");
        return -1;
    }

    // Add comment data to description
    desc->comment_data = realloc(desc->comment_data, new_comment_data_size);
    memcpy(desc->comment_data + old_comment_message_size, data, *data_size);

    info("Finished reading CMNT chunk\n");
    return 0;
}


int64_t artl_read_udat(void *buffer, size_t buffer_size,
                       const void *chunk_data, size_t chunk_data_size)
{
    info("Reading UDAT chunk at %p, size %lu\n", chunk_data, chunk_data_size);

    // Check data buffer size
    if (buffer_size < chunk_data_size) {
        error("Insufficient buffer size for UDAT data\n");
        return -1;
    }

    // Copy uncompressed data
    memcpy(buffer, chunk_data, chunk_data_size);

    info("Finished reading UDAT chunk\n");
    return chunk_data_size;
}


int64_t artl_read_cdat(void *buffer, size_t buffer_size,
                       const void *chunk_data, size_t chunk_data_size)
{
    info("Reading CDAT chunk at %p, size %lu\n", chunk_data, chunk_data_size);

    // Decompress data
    const size_t decompressed_size =
        ZSTD_decompress(buffer, buffer_size, chunk_data, chunk_data_size);

    // Check for errors
    if (ZSTD_isError(decompressed_size)) {
        error("CDAT decompression error: %s\n",
              ZSTD_getErrorName(decompressed_size));
        return -1;
    }

    info("Finished reading CDAT chunk\n");
    return decompressed_size;
}


int64_t artl_scan_cdat(const void *chunk_data, size_t chunk_data_size)
{
    // Get decompressed data size
    size_t size = ZSTD_getFrameContentSize(chunk_data, chunk_data_size);

    // Check for errors
    if (size == ZSTD_CONTENTSIZE_UNKNOWN || size == ZSTD_CONTENTSIZE_ERROR) {
        error("Failed to read CDAT decompressed data size\n");
        return -1;
    }

    return size;
}


/*******************************************************************************
 * ARTL writing
 ******************************************************************************/

int64_t artl_create_strt_chunk(void *buffer, size_t buffer_size)
{
    // STRT chunks have no data
    static const size_t chunk_size =
        sizeof (artl_chunk_header_t) + sizeof (uint32_t);

    // Check buffer size
    if (buffer_size < chunk_size) {
        error("Insufficient buffer size for STRT chunk\n");
        return -1;
    }

    // Write chunk header
    artl_chunk_header_t *chunk_header = buffer;
    chunk_header->data_size = 0;
    chunk_header->type = ARTL_CHUNK_STRT;

    // Compute CRC32c of chunk type and data
    uint32_t crc = crc32c(0, &chunk_header->type, sizeof chunk_header->type);

    // Append CRC
    uint32_t *crc_addr = buffer + sizeof *chunk_header;
    *crc_addr = crc;

    return chunk_size;
}


int64_t artl_create_dend_chunk(void *buffer, size_t buffer_size,
                               uint32_t desc_crc)
{
    // Compute size needed for chunk
    const size_t chunk_size = sizeof (artl_chunk_header_t) +
        sizeof desc_crc + sizeof (uint32_t);

    // If provided buffer is not large enough, return -1
    if (buffer_size < chunk_size) {
        error("Insufficient buffer size for DEND chunk\n");
        return -1;
    }

    // Add chunk header
    artl_chunk_header_t *chunk_header = buffer;
    chunk_header->data_size = sizeof desc_crc;
    chunk_header->type = ARTL_CHUNK_DEND;

    // Copy data
    void *chunk_data = buffer + sizeof *chunk_header;
    memcpy(chunk_data, &desc_crc, sizeof desc_crc);

    // Compute CRC32c of chunk type and data
    uint32_t crc = crc32c(0, &chunk_header->type,
                          sizeof chunk_header->type + sizeof desc_crc);

    // Append CRC
    uint32_t *crc_addr = chunk_data + sizeof desc_crc;
    *crc_addr = crc;

    return chunk_size;
}


artl_chunk_writer_t *artl_desc_chunk_start()
{
    // Create chunk writer
    artl_chunk_writer_t *writer = artl_chunk_writer_init();

    // Chunk header
    artl_chunk_header_t *chunk_header = writer->data;
    // data_size is written after labels have been added
    chunk_header->type = ARTL_CHUNK_DESC & ~ARTL_CHUNK_DOWNCASE;
    writer->size = sizeof *chunk_header;

    return writer;
}


void artl_desc_chunk_add_field(artl_chunk_writer_t *writer,
                               const char *name, artl_type_t type,
                               uint16_t nrows, uint16_t ncols)
{
    // Validate type
    if (artl_classify_type(type) < 0) {
        error("Invalid field type (%d)\n", type);
        return;
    }

    // Allow null pointers for name
    if (!name)
        name = "";

    // Get field name size
    uint16_t name_size = strnlen(name, UINT16_MAX);
    if (name_size == UINT16_MAX)
        error("Field name too long, truncated to %d characters\n", UINT16_MAX);

    // Add field descriptor size to buffer
    const size_t field_desc_size = sizeof (artl_field_descriptor_t) + name_size;
    writer->data = realloc(writer->data, writer->size + field_desc_size);

    // Set field descriptor values
    artl_field_descriptor_t *field = writer->data + writer->size;
    field->type = type;
    field->nrows = nrows;
    field->ncols = ncols;
    field->name_size = name_size;
    memcpy(field->name, name, name_size);
    writer->size += field_desc_size;
}


artl_chunk_writer_t *artl_enum_chunk_start(artl_type_t type,
                                           artl_type_t underlying_type)
{
    // Validate enum type
    if (artl_classify_type(type) != ARTL_TYPE_CLASS_EXTENDED) {
        error("Invalid enum type (%d)\n", type);
        return NULL;
    }

    // Validate underlying data type
    if (artl_classify_type(underlying_type) != ARTL_TYPE_CLASS_BASE ||
        underlying_type > ARTL_I64) {
        error("Invalid underlying enum type (%d)\n", underlying_type);
        return NULL;
    }

    // Create chunk writer
    artl_chunk_writer_t *writer = artl_chunk_writer_init();

    // Chunk header
    artl_chunk_header_t *chunk_header = writer->data;
    // data_size is written after labels have been added
    chunk_header->type = ARTL_CHUNK_ENUM & ~ARTL_CHUNK_DOWNCASE;
    writer->size = sizeof *chunk_header;

    // Enum header
    artl_enum_header_t *enum_header = writer->data + writer->size;
    enum_header->type = type;
    enum_header->underlying_type = underlying_type;
    writer->size += sizeof *enum_header;

    return writer;
}


void artl_enum_chunk_add_label(artl_chunk_writer_t *writer,
                               int64_t value, const char *label)
{
    // Get size of underlying type
    artl_enum_header_t *enum_header =
        writer->data + sizeof (artl_chunk_header_t);
    if (enum_header->underlying_type > ARTL_I64) {
        error("Invalid underlying type in enum header\n");
        return;
    }
    size_t enum_type_size = artl_type_size[enum_header->underlying_type];

    // Get label text size
    uint16_t label_size = strnlen(label, UINT16_MAX);
    if (label_size == UINT16_MAX)
        error("Label text too long, truncated to %d characters\n", UINT16_MAX);

    // Add label definition size to buffer
    const size_t label_def_size =
        enum_type_size + sizeof label_size + label_size;
    writer->data = realloc(writer->data, writer->size + label_def_size);
    enum_header = writer->data + sizeof (artl_chunk_header_t);

    // Add label definition
    // Exploits little-endian truncation properties to handle different types
    memcpy(writer->data + writer->size, &value, enum_type_size);
    writer->size += enum_type_size;
    memcpy(writer->data + writer->size, &label_size, sizeof label_size);
    writer->size += sizeof label_size;
    memcpy(writer->data + writer->size, label, label_size);
    writer->size += label_size;
}


artl_chunk_writer_t *artl_cmnt_chunk_start(void *data, uint32_t data_size)
{
    // Create chunk writer
    artl_chunk_writer_t *writer = artl_chunk_writer_init();

    // Chunk header
    artl_chunk_header_t *chunk_header = writer->data;
    // data_size is written after labels have been added
    chunk_header->type = ARTL_CHUNK_CMNT;
    writer->size = sizeof *chunk_header;

    // Comment data
    writer->data =
        realloc(writer->data, writer->size + sizeof data_size + data_size);
    memcpy(writer->data + writer->size, &data_size, sizeof data_size);
    writer->size += sizeof data_size;
    memcpy(writer->data + writer->size, data, data_size);
    writer->size += data_size;

    return writer;
}


void artl_cmnt_chunk_add_field(artl_chunk_writer_t *writer,
                               const char *name, artl_type_t type,
                               uint16_t nrows, uint16_t ncols)
{
    // Same field format as DESC chunks
    artl_desc_chunk_add_field(writer, name, type, nrows, ncols);
}


uint32_t artl_chunk_end(artl_chunk_writer_t *writer,
                        uint32_t desc_crc,
                        void **chunk_data, size_t *chunk_size)
{
    // Fill in chunk data size
    const size_t chunk_data_size = writer->size - sizeof (artl_chunk_header_t);
    artl_chunk_header_t *chunk_header = writer->data;
    chunk_header->data_size = chunk_data_size;

    // Compute CRC32c of chunk type and data
    uint32_t chunk_crc = crc32c(0, &chunk_header->type,
                                sizeof chunk_header->type + chunk_data_size);
    desc_crc = crc32c(desc_crc, &chunk_header->type,
                      sizeof chunk_header->type + chunk_data_size);

    // Append chunk CRC
    writer->data = realloc(writer->data, writer->size + sizeof chunk_crc);
    uint32_t *crc_addr = writer->data + writer->size;
    *crc_addr = chunk_crc;
    writer->size += sizeof chunk_crc;

    // Return chunk data size and address
    *chunk_data = writer->data;
    *chunk_size = writer->size;

    // Return accumulated description CRC
    return desc_crc;
}


artl_chunk_writer_t *artl_chunk_writer_init()
{
    artl_chunk_writer_t *writer = calloc(1, sizeof (artl_chunk_writer_t));
    writer->data = malloc(1024);
    return writer;
}


void artl_chunk_writer_free(artl_chunk_writer_t *writer)
{
    // Free data buffer
    if (writer)
        free(writer->data);

    // Free writer struct
    free(writer);
}

