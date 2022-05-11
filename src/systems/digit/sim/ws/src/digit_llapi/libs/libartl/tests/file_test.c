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


// This file tests writing and reading log files as well as the integer
// conversion functionality.


#include "artl.h"
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>


/*******************************************************************************
 * Writing
 ******************************************************************************/

typedef struct __attribute__((packed)) {
    uint64_t a[2];
    uint32_t b[2];
    uint16_t c[2];
    uint8_t d[2];
    int64_t e[2];
    int32_t f[2];
    int16_t g[2];
    int8_t h[2];
} test_struct_t;


static int write_log_file(void)
{
    // Open log file
    int fd = creat("test.log", 0644);
    if (-1 == fd) {
        printf("Failed to create log file\n");
        return -1;
    }

    // Create message description
    artl_description_t *desc = artl_description_init();

    // Add fields
    artl_add_field(desc, "a", ARTL_U64, 2, 1);
    artl_add_field(desc, "b", ARTL_U32, 2, 1);
    artl_add_field(desc, "c", ARTL_U16, 2, 1);
    artl_add_field(desc, "d", ARTL_U8, 2, 1);
    artl_add_field(desc, "e", ARTL_I64, 2, 1);
    artl_add_field(desc, "f", ARTL_I32, 2, 1);
    artl_add_field(desc, "g", ARTL_I16, 2, 1);
    artl_add_field(desc, "h", ARTL_I8, 2, 1);

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

    // Write log data
    test_struct_t data = {
        .a = {-1, 1},
        .b = {-2, 2},
        .c = {-3, 3},
        .d = {-4, 4},
        .e = {-5, 5},
        .f = {-6, 6},
        .g = {-7, 7},
        .h = {-8, 8},
    };

    // Uncompressed
    artl_write_udat(fd, &data, sizeof data);

    // // Compressed
    artl_write_cdat(fd, &data, sizeof data);

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


typedef struct {
    uint64_t test_U64[16];
    uint32_t test_U32[16];
    uint16_t test_U16[16];
    uint8_t test_U8[16];
    int64_t test_I64[16];
    int32_t test_I32[16];
    int16_t test_I16[16];
    int8_t test_I8[16];
} test_struct_subset_t;


static int read_log_file(void)
{
    // Open file
    int fd = open("test.log", O_RDONLY);

    // Read file into buffer
    off_t file_size = lseek(fd, 0, SEEK_END);
    lseek(fd, 0, SEEK_SET);
    void *file_data = malloc(file_size);
    read(fd, file_data, file_size);
    close(fd);

    // Read file
    artl_description_t *desc = artl_description_init();
    void *log_data;
    size_t log_data_size;
    int ret = artl_read(file_data, file_size, desc, &log_data, &log_data_size);
    free(file_data);
    if (ret < 0) {
        printf("Failed to read log file\n");
        return -1;
    }

    // Print description
    print_description(desc);

    // Map log data to desired format
    artl_map_t *map = artl_map_init(desc);

    // Integer conversion tests
#define MAP_INT_TESTS(totype) do {                                      \
        artl_map_field(map, "a", offsetof(test_struct_subset_t, test_ ## totype), ARTL_ ## totype); \
        artl_map_field(map, "b", -1, ARTL_ ## totype);                  \
        artl_map_field(map, "c", -1, ARTL_ ## totype);                  \
        artl_map_field(map, "d", -1, ARTL_ ## totype);                  \
        artl_map_field(map, "e", -1, ARTL_ ## totype);                  \
        artl_map_field(map, "f", -1, ARTL_ ## totype);                  \
        artl_map_field(map, "g", -1, ARTL_ ## totype);                  \
        artl_map_field(map, "h", -1, ARTL_ ## totype);                  \
    } while (0)
    MAP_INT_TESTS(U64);
    MAP_INT_TESTS(U32);
    MAP_INT_TESTS(U16);
    MAP_INT_TESTS(U8);
    MAP_INT_TESTS(I64);
    MAP_INT_TESTS(I32);
    MAP_INT_TESTS(I16);
    MAP_INT_TESTS(I8);

    // Copy log data into formatted struct array
    long nentries = log_data_size / desc->message_size;
    long data_size = nentries * sizeof (test_struct_subset_t);
    test_struct_subset_t *data = malloc(data_size);
    nentries = artl_map_get_array(
        map,
        data, data_size, sizeof data[0],
        log_data, log_data_size);
    artl_map_free(map);
    free(log_data);
    printf("Read %ld log entries\n", nentries);

    // Print struct data
    for (int i = 0; i < nentries; ++i) {
        printf("data[%d]:\n", i);
#define PRINT_INT_TEST(tobits) do {                                     \
            for (int j = 0; j < 16; ++j)                                \
                printf("  test_U" #tobits "[%d]: %lu\n", j,             \
                       (uint64_t) data[i].test_U ## tobits [j]);  \
            for (int j = 0; j < 16; ++j)                                \
                printf("  test_I" #tobits "[%d]: %ld\n", j,             \
                       (int64_t) data[i].test_I ## tobits [j]);   \
        } while (0)
        PRINT_INT_TEST(64);
        PRINT_INT_TEST(32);
        PRINT_INT_TEST(16);
        PRINT_INT_TEST(8);
    }
    free(data);

    return 0;
}


/*******************************************************************************
 * Run file example
 ******************************************************************************/

int main()
{
    if (write_log_file() < 0)
        return EXIT_FAILURE;
    if (read_log_file() < 0)
        return EXIT_FAILURE;

    return EXIT_SUCCESS;
}
