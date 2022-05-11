/*
 * Copyright Agility Robotics
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


// This file tests writing and reading an unstructured log, which contains no
// fields in the description but still contains data chunks. This type of file
// can be useful for storing non-tabular binary data with header info and chunk
// boundaries.


#include "artl.h"
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>


/*******************************************************************************
 * Writing
 ******************************************************************************/

static int write_log_file(const char* filepath)
{
    // Open log file
    int fd = creat(filepath, 0644);
    if (-1 == fd) {
        printf("Failed to create log file\n");
        return -1;
    }

    // Create message description
    artl_description_t *desc = artl_description_init();

    // No ordinary description fields

    // Add comments
    int64_t timestamp = time(NULL);
    artl_add_comment(desc, &timestamp, "timestamp", ARTL_I64, 1, 1);
    const char str[] = "Comment string";
    artl_add_comment(desc, str, "string", ARTL_CHAR, sizeof str, 1);

    // Write description to file
    if (artl_write_description(fd, desc) < 0) {
        printf("Failed to write description to file\n");
        return -1;
    }
    artl_description_free(desc);

    // Write several compressed and uncompressed chunks of different lengths
    char* data = malloc(4096);
    memset(data, 1, 4096);
    artl_write_udat(fd, &data, 1024);
    memset(data, 2, 4096);
    artl_write_cdat(fd, &data, 2048);
    memset(data, 3, 4096);
    artl_write_udat(fd, &data, 3072);
    memset(data, 4, 4096);
    artl_write_cdat(fd, &data, 4096);
    free(data);

    // Close file
    close(fd);

    return 0;
}


/*******************************************************************************
 * Reading
 ******************************************************************************/

static const char *artl_type_string(artl_type_t type)
{
    switch (type) {
    case ARTL_U8: return "u8";
    case ARTL_U16: return "u16";
    case ARTL_U32: return "u32";
    case ARTL_U64: return "u64";
    case ARTL_I8: return "i8";
    case ARTL_I16: return "i16";
    case ARTL_I32: return "i32";
    case ARTL_I64: return "i64";
    case ARTL_F32: return "f32";
    case ARTL_F64: return "f64";
    case ARTL_BOOL: return "bool";
    case ARTL_CHAR: return "char";
    case ARTL_BIN: return "bin";
    default: break;
    }

    static char out_buf[16];
    sprintf(out_buf, "%d", type);
    return out_buf;
}


static void print_field(const artl_description_t *desc,
                        int64_t ifield, const void *message_data)
{
    const artl_field_t *field = &desc->fields[ifield];
    const artl_enum_t *enm = artl_find_enum(desc, field->type);
    const artl_type_t base_type = artl_find_base_type(desc, field->type);
    if (base_type < 0) {
        printf("Invalid type\n");
        return;
    }

    // Enum types
    if (enm) {
        int64_t *data = malloc(field->nrows * field->ncols * sizeof (int64_t));
        artl_convert(message_data + field->offset, field->size,
                     data, field->type, ARTL_I64);
        for (int i = 0; i < field->nrows * field->ncols; ++i)
            puts(artl_find_label(enm, data[i]));
        free(data);
        return;
    }

    // Integral and boolean types
    if (artl_check_type_match(NULL, base_type, ARTL_INT) ||
        ARTL_BOOL == base_type) {
        int64_t *data = malloc(field->nrows * field->ncols * sizeof (int64_t));
        artl_convert(message_data + field->offset, field->size,
                     data, field->type, ARTL_I64);
        for (int i = 0; i < field->nrows; ++i) {
            for (int j = 0; j < field->ncols; ++j)
                printf("%ld ", data[j * field->nrows + i]);
            putchar('\n');
        }
        free(data);
        return;
    }

    // Float types
    if (artl_check_type_match(NULL, base_type, ARTL_FLOAT)) {
        double *data = malloc(field->nrows * field->ncols * sizeof (double));
        artl_convert(message_data + field->offset, field->size,
                     data, field->type, ARTL_F64);
        for (int i = 0; i < field->nrows; ++i) {
            for (int j = 0; j < field->ncols; ++j)
                printf("%f ", data[j * field->nrows + i]);
            putchar('\n');
        }
        free(data);
        return;
    }

    // Strings
    if (ARTL_CHAR == base_type) {
        fwrite(message_data + field->offset, 1, field->size, stdout);
        putchar('\n');
        return;
    }

    // Strings
    if (ARTL_BIN == base_type) {
        for (size_t i = 0; i < field->size; ++i)
            printf("%02x ", *((char *)(message_data + field->offset + i)));
        putchar('\n');
        return;
    }

    printf("INVALID OR UNHANDLED TYPE\n");
}


static void print_description(const artl_description_t *desc)
{
    // Print enum information
    for (int i = 0; i < desc->nenums; ++i) {
        printf("Enum type %d, underlying type %s\n", desc->enums[i].type,
               artl_type_string(desc->enums[i].underlying_type));
        for (int j = 0; j < desc->enums[i].nlabels; ++j) {
            printf("  %ld: %s\n", desc->enums[i].labels[j].value,
                   desc->enums[i].labels[j].text);
        }
    }

    // Print fields
    printf("Fields:\n");
    for (int i = 0; i < desc->nfields; ++i) {
        const artl_field_t *field = &desc->fields[i];
        printf("  %s: %dx%d, type %s\n", field->name, field->nrows,
               field->ncols, artl_type_string(field->type));
    }

    // Print comments
    if (desc->comment) {
        printf("Comments:\n");
        for (int i = 0; i < desc->comment->nfields; ++i) {
            const artl_field_t *field = &desc->comment->fields[i];
            printf("  %s: %dx%d, type %s\n", field->name, field->nrows,
                   field->ncols, artl_type_string(field->type));
            print_field(desc->comment, i, desc->comment_data);
        }
    }
}


static int read_log_file(const char* filepath)
{
    // Open file
    int fd = open(filepath, O_RDONLY);
    if (-1 == fd) {
        printf("Failed to open file: %s\n", filepath);
        return -1;
    }

    artl_description_t *desc = artl_description_init();
    void *data;
    size_t data_size;
    bool printed_desc = false;
    long nchunks = 0;

    while (true) {
        // Read a chunk from the file
        const int ret = artl_read_partial(fd, desc, &data, &data_size);
        if (0 == ret) {
            printf("End of file reached, %ld chunks read\n", nchunks);
            break;
        } else if (ret < 0) {
            printf("Error encountered while reading file, %ld chunks read\n",
                   nchunks);
            break;
        }

        if (data) {
            printf("Read data chunk: %zd bytes\n", data_size);
            free(data);
        } else {
            printf("Read chunk\n");
        }

        if (!printed_desc && desc->complete) {
            print_description(desc);
            printed_desc = true;
        }

        ++nchunks;
    }

    return 0;
}


/*******************************************************************************
 * Run file example
 ******************************************************************************/

int main(int argc, char *argv[])
{
    // Open file named test.log by default, or accept file path as an argument
    const char *filepath = "./unstructured.log";
    if (argc >= 2)
        filepath = argv[1];

    if (write_log_file(filepath) < 0)
        return EXIT_FAILURE;
    if (read_log_file(filepath) < 0)
        return EXIT_FAILURE;

    return EXIT_SUCCESS;
}
