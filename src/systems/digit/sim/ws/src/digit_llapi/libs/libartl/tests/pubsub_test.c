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


// This file creates publisher and subscriber threads and streams data for a
// short while.


#include "artl.h"
#include "artl_internal.h"
#include <pthread.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#define RETURN_SUCCESS ((void *) 0)
#define RETURN_FAILURE ((void *) -1)


/*******************************************************************************
 * Publishing
 ******************************************************************************/

typedef struct __attribute__((packed)) {
    int64_t a;
    double b[3];
    bool c;
} test_struct_t;


static void *publisher(void *arg)
{
    (void) arg;

    // Create message description
    artl_description_t *desc = artl_description_init();

    // Add fields
    artl_add_field(desc, "a", ARTL_I64, 1, 1);
    artl_add_field(desc, "b", ARTL_F64, 3, 1);
    artl_add_field(desc, "c", ARTL_BOOL, 1, 1);

    // Add comments
    int64_t timestamp = time(NULL);
    artl_add_comment(desc, &timestamp, "timestamp", ARTL_I64, 1, 1);
    const char str[] = "Comment string";
    artl_add_comment(desc, str, "string", ARTL_CHAR, sizeof str, 1);

    // Initialize publisher
    artl_publisher_t *pub = artl_publisher_init(desc, "127.0.0.1", 25500);
    artl_description_free(desc);
    if (!pub) {
        printf("Failed to create publisher\n");
        return RETURN_FAILURE;
    }

    // Publish a certain number of messages
    for (int i = 0; i < 10000; ++i) {
        // Message data
        test_struct_t data = {
            .a = i,
            .b = {0.1, 0.2, 0.3},
            .c = (i % 2),
        };

        // Update publisher
        if (artl_publisher_update(pub, &data) < 0) {
            printf("Error publishing data to stream\n");
            return RETURN_FAILURE;
        }
    }

    return RETURN_SUCCESS;
}


/*******************************************************************************
 * Receiving
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


static artl_map_t *mapfun(const artl_description_t *desc, void *arg)
{
    (void) arg;

    // Print description
    print_description(desc);

    // Validate field sizes and types
    if (!artl_check_field(desc, "a", ARTL_INT, 1, 1) ||
        !artl_check_field(desc, "b", ARTL_FLOAT, 3, 1) ||
        !artl_check_field(desc, "c", ARTL_BOOL, 1, 1)) {
        printf("Description failed validation\n");
        return NULL;
    }

    // Map message data to desired format
    artl_map_t *map = artl_map_init(desc);
    artl_map_field(map, "a", -1, ARTL_I64);
    artl_map_field(map, "b", -1, ARTL_F64);
    artl_map_field(map, "c", -1, ARTL_BOOL);

    return map;
}


static void *subscriber(void *arg)
{
    (void) arg;

    test_struct_t data;

    // Initialize subscriber
    artl_subscriber_t *sub = artl_subscriber_init("127.0.0.1", 25500);
    if (!sub) {
        printf("Failed to create subscriber\n");
        return RETURN_FAILURE;
    }
    sub->mapfun = mapfun;
    sub->output_buffer_size = sizeof data;

    // Record start time
    long start_ms = artl_clock_getms();

    // Update subscriber and get data
    bool printed_message = false;
    while (true) {
        // Update stream and check for new data
        int ret = artl_subscriber_update(sub, &data);
        bool new_data = ret > 0;
        if (ret < 0) {
            printf("Failed to update subscriber\n");
            return RETURN_FAILURE;
        }

        if (new_data && !printed_message) {
            // Print struct data
            printf("data:\n");
            printf("  a: %ld\n", data.a);
            for (int j = 0; j < 3; ++j)
                printf("  b[%d]: %f\n", j, data.b[j]);
            printf("  c: %s\n", data.c ? "true" : "false");
            printed_message = true;
        }

        // Stop successfully when subscriber disconnects after a timeout
        if (!sub->connected && sub->connect_count > 0) {
            artl_subscriber_free(sub);
            return RETURN_SUCCESS;
        }

        // Abort if subscriber never connects
        if (!sub->connected && artl_clock_getms() - start_ms > 1000) {
            printf("Failed to connect to publisher\n");
            artl_subscriber_free(sub);
            return RETURN_FAILURE;
        }
    }

    return RETURN_FAILURE;
}


/*******************************************************************************
 * Run stream example
 ******************************************************************************/

int main()
{
    // Create publishing and subscribing threads
    pthread_t pub_thread, sub_thread;
    pthread_create(&pub_thread, NULL, publisher, NULL);
    pthread_create(&sub_thread, NULL, subscriber, NULL);

    // Wait for threads to terminate
    void *pubret, *subret;
    pthread_join(pub_thread, &pubret);
    pthread_join(sub_thread, &subret);

    // Check for failure
    if (pubret || subret)
        return EXIT_FAILURE;

    return EXIT_SUCCESS;
}
