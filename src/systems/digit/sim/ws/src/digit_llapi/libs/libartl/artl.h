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

// This is the only header needed when using the precompiled library
// (libartl.so on linux, artl.dll on windows)

#ifndef ARTL_H
#define ARTL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Data type indices
typedef enum {
    ARTL_U8,
    ARTL_U16,
    ARTL_U32,
    ARTL_U64,
    ARTL_I8,
    ARTL_I16,
    ARTL_I32,
    ARTL_I64,
    ARTL_F32,
    ARTL_F64,
    ARTL_BOOL,
    ARTL_CHAR,
    ARTL_BIN,

    // Used for validation functions
    ARTL_INT = UINT16_MAX + 1,
    ARTL_UINT,
    ARTL_SINT,
    ARTL_FLOAT,
    ARTL_BYTE,
    ARTL_ENUM,
    ARTL_INT_ENUM,
    ARTL_ANY = -1
} artl_type_t;

// Information about a field in a log format descriptor
typedef struct {
    artl_type_t type;
    int nrows;
    int ncols;
    const char *name;
    size_t size;
    size_t offset;
} artl_field_t;

// Value/label text pair for an enum type
typedef struct {
    int64_t value;
    const char *text;
} artl_enum_label_t;

// Information for an enum type in a log format descriptor
typedef struct {
    artl_type_t type;
    artl_type_t underlying_type;
    artl_enum_label_t *labels;
    int64_t nlabels;
} artl_enum_t;

// Information parsed from a log format descriptor
typedef struct artl_description artl_description_t;
struct artl_description {
    artl_field_t *fields;
    int64_t nfields;
    artl_enum_t *enums;
    int64_t nenums;
    artl_description_t *comment;
    void *comment_data;
    size_t message_size;
    uint32_t crc;
    bool started;
    bool complete;
};

#ifdef __cplusplus
extern "C" {
#endif


/*******************************************************************************
 * ARTL reading
 ******************************************************************************/

// Reads an ARTL file from a buffer. The value of *data returns a
// pointer to a buffer allocated with malloc containing *data_size
// bytes of data. Returns 0 on success and -1 if any errors occured.
int artl_read(const void *artl_data, size_t artl_data_size,
              artl_description_t *description, void **data, size_t *data_size);

// Reads an ARTL file chunk-by-chunk from a file descriptor. Each call
// to this function attempts to read a single chunk and either fills
// out the description, returns decompressed data in a buffer
// allocated with malloc, or indicates an error or end of stream. The
// description should be zero-initialized or cleared before the first
// call. If description is null, the function will skip filling out
// the description. If data and data_size are null, the function will
// skip decoding and returning data. When all output pointers are
// provided, data is only returned after the description is complete.
// Returns 1 if there may be data left in the stream, 0 if the end of
// the file was reached, or -1 if an error was encountered.
int artl_read_partial(int fd, artl_description_t *description,
                      void **data, size_t *data_size);


/*******************************************************************************
 * ARTL writing
 ******************************************************************************/

// Writes a complete message description to a file descriptor. Returns
// the total number of bytes written, or -1 if an error occured.
int64_t artl_write_description(int fd, const artl_description_t *description);

// Writes uncompressed message data to a file descriptor. Returns the
// total number of bytes written, or -1 if an error occured.
int64_t artl_write_udat(int fd, const void *data, size_t data_size);

// Writes compressed message data to a file descriptor. Returns the
// total number of bytes written, or -1 if an error occured.
int64_t artl_write_cdat(int fd, const void *data, size_t data_size);

// Converts a description struct to a series of chunks. All returned
// memory is allocated with malloc and must be freed by the client.
void artl_create_description_chunks(const artl_description_t *description,
                                    size_t *nchunks, void ***chunks,
                                    size_t **chunk_sizes);

// Frees memory allocated by artl_create_description_chunks.
void artl_free_description_chunks(size_t nchunks, void **chunks,
                                  size_t *chunk_sizes);

// Creates an uncompressed data chunk in the given buffer. Returns the
// size of the chunk, or -1 if the output buffer was not large enough.
int64_t artl_create_udat_chunk(void *buffer, size_t buffer_size,
                               const void *data, size_t data_size);

// Creates a compressed data chunk in the given buffer. Returns the
// size of the chunk, or -1 if the output buffer was not large enough
// or a compression error occured.
int64_t artl_create_cdat_chunk(void *buffer, size_t buffer_size,
                               const void *data, size_t data_size);


/*******************************************************************************
 * Structure mapping and data conversion
 ******************************************************************************/

// Data for a single mapped region in a mapping object.
typedef struct {
    size_t buffer_offset;
    union {
        size_t output_offset;
        void *output_address;
    };
    size_t buffer_size;
    size_t output_size;
    artl_type_t buffer_type;
    artl_type_t output_type;
    bool output_absolute;
} artl_map_region_t;

// Object used to map log data to an output structure.
typedef struct {
    const artl_description_t *description;
    size_t message_size;
    artl_map_region_t *regions;
    int64_t nregions;
    size_t min_output_size;
} artl_map_t;

// Creates a structure mapping object that can be used with the given
// description. The description pointer must remain valid as long as
// the fields are being added to the map, but can be invalid when
// artl_map_get_array is called. Must be freed with artl_free_map().
artl_map_t *artl_map_init(const artl_description_t *description);

// Records information needed to map log data from a field in the
// description to a particular offset in an output struct. A type
// conversion can be given, which will succeed only if both types are
// either integers or floats, or -1 can be provided to skip type
// conversion. If offset is -1, the offset will be automatically
// computed to come after the last mapped field. Mapped fields are not
// allowed to overlap previously mapped fields in the output struct.
// Returns true if mapping was successful, or false on failure.
bool artl_map_field(artl_map_t *map, const char *field,
                    int64_t output_offset, artl_type_t output_type);

// Similar to artl_map_field, but can map just part of a vector or
// matrix field. Takes the starting linear index into the field and
// the number of elements to copy. Fails if the field has fewer than
// (istart + nelements) elements.
bool artl_map_partial_field(artl_map_t *map, const char *field,
                            int64_t output_offset, artl_type_t output_type,
                            int64_t istart, int64_t nelements);

// Similar to artl_map_field, but maps to an absolute address instead
// of an offset into a struct. Useful for mapping into data structures
// containing pointers to other regions in memory.
bool artl_map_field_abs(artl_map_t *map, const char *field,
                        void *output_address, artl_type_t output_type);

// Similar to artl_map_partial_field, but maps to an absolute address
// instead of an offset into a struct. Useful for mapping into data
// structures containing pointers to other regions in memory.
bool artl_map_partial_field_abs(artl_map_t *map, const char *field,
                                void *output_address, artl_type_t output_type,
                                int64_t istart, int64_t nelements);

// Maps a buffer of log data to an array of output structs. Returns
// the number of structs that were mapped.
int64_t artl_map_get_array(const artl_map_t *map, void *output,
                           size_t output_size, size_t output_stride,
                           const void *buffer, size_t buffer_size);

// Frees the memory used by a structure mapping object. Does nothing
// if map is null.
void artl_map_free(artl_map_t *map);

// Checks whether the first given type can be converted to the second
// given type using artl_convert. Returns true if the conversion is
// valid, or false otherwise.
bool artl_check_type_conversion(artl_type_t from, artl_type_t to,
                                int *num, int *den);

// Copies data from one buffer to another while performing type
// conversions. Returns 0 on success, or -1 if the conversion is
// invalid.
int artl_convert(const void *buffer, size_t buffer_size, void *output,
                 artl_type_t from, artl_type_t to);


/*******************************************************************************
 * ARTL over UDP communication
 ******************************************************************************/

#ifdef _WIN32
#undef _WIN32_WINNT
#define _WIN32_WINNT 0x0601
#include <ws2tcpip.h>
#else
#include <netinet/ip.h>
#endif

// Settings and state for a UDP stream publisher.
//
// The following fields can be modified after artl_publisher_init()
// is called to change publisher settings:
//   interval_ms:
//     Time between sending out description chunks along with data chunks.
//     If set to zero, no description chunks are sent after initialization.
//     Default value is 100.
//   compressed:
//     If true, message data is sent compressed.
//     Default value is false.
typedef struct {
    // Internal fields, should not be modified or inspected
    int sock;
    struct sockaddr_in address;
    size_t message_size;
    size_t nchunks;
    void **chunks;
    size_t *chunk_sizes;
    size_t ichunk;
    int64_t last_ms;

    // Setting fields, these can be modified after initialization
    int interval_ms;
    bool compressed;
} artl_publisher_t;


// Creates a publisher struct. Must be provided with a description
// struct filled out with the fields and enums in the sent message and
// an address and port to send to. Returns NULL on failure.
artl_publisher_t *artl_publisher_init(const artl_description_t *description,
                                      const char *address, uint16_t port);

// Sends new data over UDP while also managing repeated transmission
// of the description chunks. If the message pointer is null, a
// description chunk is sent instead of a data chunk. Returns 0 on
// success, -1 on failure.
int artl_publisher_update(artl_publisher_t *publisher, const void *message);

// Closes the socket and frees memory associated with a publisher struct.
void artl_publisher_free(artl_publisher_t *publisher);

// Settings and state for a UDP stream subscriber.
//
// The following fields provide potentially useful information,
// but should not be modified directly:
//   description:
//     A description that is filled out by the incoming stream.
//     Before the description is marked complete, data chunks are ignored.
//   connected:
//     True if the subscriber is connected to a publisher.
//   received_message:
//     True if the subscriber has received a valid message
//     since connecting to the publisher.
//   connect_count:
//     Counts the number of times the connection has been (re)established.
//   publisher_address:
//     Address of the connected publisher.
//   publisher_address_size:
//     Size in bytes of the address of the connected publisher.
//
// The following fields can be modified after artl_subscriber_init()
// is called to change subscriber settings:
//   update_timeout_us:
//     Maximum time spent reading packets before the function returns.
//     Default value is 1000 (1 ms).
//   disconnect_timeout_ms:
//     If connected to a publisher but no valid chunks are received within
//     the timeout period, the subscriber disconnects and becomes available
//     for a new connection. The subscriber also disconnects if a valid
//     but different description is received. Note that a connection can
//     be kept alive without sending actual message data if valid
//     description chunks are continually sent. If set to zero, restarting
//     is disabled, even when a different description is received.
//     Default value is 100.
//   output_buffer_size:
//     Limits the number of bytes written to the output pointer by
//     artl_subscriber_update().
//     Default value is SIZE_MAX.
//   mapfun:
//     Function pointer taking a description pointer and a void pointer
//     and returning a pointer to a map. Intended to validate and map an
//     input message to an output struct. Called once the description
//     becomes complete. Can return NULL to indicate an invalid message.
//     If null, artl_subscriber_update outputs the raw message data instead.
//     Default value is NULL.
//   mapfun_arg:
//     Arbitrary pointer passed as the second argument to mapfun.
//     Allows mapfun to have side effects without using global state.
//     Default value is NULL.
typedef struct {
    // Internal fields, should not be modified or inspected
    int sock;
    int last_retval;
    artl_map_t *map;
    int64_t last_received_ms;
    bool invalid;

    // Informational fields, these should not be modified by the
    // client but can provide useful information
    artl_description_t description;
    bool connected;
    bool received_message;
    unsigned int connect_count;
    struct sockaddr_storage publisher_address;
    socklen_t publisher_address_size;

    // Setting fields, these can be modified after initialization
    unsigned int update_timeout_us;
    unsigned int disconnect_timeout_ms;
    size_t output_buffer_size;
    artl_map_t *(*mapfun)(const artl_description_t *, void *);
    void *mapfun_arg;
} artl_subscriber_t;

// Creates a subscriber struct. Must be provided with an address and
// port to listen at. Several settings can be applied to the returned
// struct, see artl_subscriber_t comments for details. Returns NULL on
// failure.
artl_subscriber_t *artl_subscriber_init(const char *address, uint16_t port);

// Reads a stream of ARTL data from a UDP socket, filling out a
// description and obtaining the newest valid mapped message data.
// When a valid STRT chunk is received, the socket will be connected
// to the sender and all other senders will be ignored until the
// subscriber disconnects due to the disconnect_timeout_ms field.
// Returns 1 if new message data is available in *output, 0 if no new
// message data is available, or -1 if an error was detected.
int artl_subscriber_update(artl_subscriber_t *subscriber, void *output);

// Manually disconnects a subscriber, resetting its state to as it was
// before it received any packets. The connect_count field is preserved.
void artl_subscriber_disconnect(artl_subscriber_t *subscriber);

// Closes the socket and frees memory associated with a subscriber struct.
void artl_subscriber_free(artl_subscriber_t *subscriber);


/*******************************************************************************
 * Description creation
 ******************************************************************************/

// Allocates an empty description struct.
artl_description_t *artl_description_init(void);

// Clears and frees a description struct created with artl_description_init.
void artl_description_free(artl_description_t *description);

// Adds a field to a description struct. Returns a pointer to the
// field, or NULL on failure.
artl_field_t *artl_add_field(artl_description_t *description,
                             const char *name, artl_type_t type,
                             uint16_t nrows, uint16_t ncols);

// Adds an enum to a description struct. If the given enum type is -1,
// an enum type index is chosen automatically. Returns a pointer to
// the enum, or NULL on failure.
artl_enum_t *artl_add_enum(artl_description_t *description,
                           artl_type_t type,
                           artl_type_t underlying_type);

// Adds an enum label to a description struct. Returns a pointer to the
// label, or NULL on failure.
artl_enum_label_t *artl_add_label(artl_description_t *description,
                                  artl_type_t type,
                                  int64_t value, const char *text);

// Adds a comment field and data to a description struct. Returns a
// pointer to the field, or NULL on failure.
artl_field_t *artl_add_comment(artl_description_t *description,
                               const void *data,
                               const char *name, artl_type_t type,
                               uint16_t nrows, uint16_t ncols);

// Abbreviated form of artl_add_comment for null-terminated strings.
artl_field_t *artl_add_comment_string(artl_description_t *description,
                                      const char *string, const char *name);

// Clears the contents of the given description struct. The struct can
// be reused. artl_description_init and artl_description_free should
// be used instead.
void artl_clear_description(artl_description_t *description);


/*******************************************************************************
 * Description querying
 ******************************************************************************/

// Checks for the existence, type, and dimensions of a field. Returns
// true if the field exists and has the specified type and size, or
// false otherwise. Passing -1 (ARTL_ANY) for type, nrows, or ncols
// disables checking for that attribute.
bool artl_check_field(const artl_description_t *description,
                      const char *name, artl_type_t type,
                      int nrows, int ncols);

// Similar to artl_check_field, but returns true instead of false if
// the field does not exist.
bool artl_check_optional_field(const artl_description_t *description,
                               const char *name, artl_type_t type,
                               int nrows, int ncols);

// Returns a pointer to the field matching the given field name, or
// NULL if the field does not exist in the description.
const artl_field_t *artl_find_field(const artl_description_t *description,
                                    const char *name);

// Returns a pointer to the enum matching the given enum type index,
// or NULL if the enum does not exist in the description.
const artl_enum_t *artl_find_enum(const artl_description_t *description,
                                  artl_type_t type);

// Returns a pointer to the label string matching the given enum value, or
// NULL if the enum does not have a label for the given value.
const char *artl_find_label(const artl_enum_t *enm, int64_t value);

// Returns a pointer to the label string matching the given enum type
// and value, or NULL if the type or value were invalid.
const char *artl_find_enum_label(const artl_description_t *description,
                                 artl_type_t type, int64_t value);

// Returns the size in bytes of the given field, or -1 if the field
// does not exist in the description.
int64_t artl_find_field_size(const artl_description_t *description,
                             const char *name);

// Returns the base type corresponding to the given type, or -1 if the
// type index is invalid.
artl_type_t artl_find_base_type(const artl_description_t *description,
                                artl_type_t type);

// Checks whether two given type indices are considered to match. The
// second given type can be a category such as ARTL_INT instead of a
// specific type. The description pointer can be null, in which case
// type must be a base type. Returns true if the types match, or false
// otherwise.
bool artl_check_type_match(const artl_description_t *description,
                           artl_type_t type, artl_type_t match);

// Returns the size in bytes required to store the type with the given
// index, or -1 if the type index is invalid. The description pointer
// can be null, in which case type must be a base type.
int artl_find_type_size(const artl_description_t *description,
                        artl_type_t type);


/*******************************************************************************
 * Helper functions for languages with a limited C interface (e.g. Matlab)
 ******************************************************************************/

// Returns a pointer to the field with the given index, or NULL if the
// index is invalid.
const artl_field_t *artl_get_field(const artl_description_t *description,
                                   int64_t ifield);

// Returns a pointer to the enum with the given index, or NULL if the
// index is invalid.
const artl_enum_t *artl_get_enum(const artl_description_t *description,
                                 int64_t ienum);

// Returns a pointer to the label with the given index, or NULL if the
// index is invalid.
const artl_enum_label_t *artl_get_label(const artl_enum_t *enm, int64_t ilabel);

// Opens a file as read-only and returns a file descriptor. Returns -1
// on failure.
int artl_file_open(const char *path);

// Opens a file as write-only and returns a file descriptor. Creates the
// file if it doesn't exist, and overwrites the file if it does. Returns
// -1 on failure.
int artl_file_open_write(const char *path);

// Returns the size of the given file, or -1 on failure.
int64_t artl_file_size(int fd);

// Returns the position from the start in the given file, or -1 on failure.
int64_t artl_file_tell(int fd);

// Seeks to a position in the given file.
void artl_file_seek(int fd, size_t position);

// Closes the given file.
void artl_file_close(int fd);

// Frees memory allocated with malloc.
void artl_free(void *p);


/*******************************************************************************
 * Library version symbols
 ******************************************************************************/

extern const int artl_version_major;
extern const int artl_version_minor;
extern const char artl_version_extra[];


#ifdef __cplusplus
}
#endif

#endif // ARTL_H
