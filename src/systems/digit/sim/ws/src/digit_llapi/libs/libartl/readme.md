# The Agility Real-Time Log Format

The Agility Real-Time Log (ARTL) format is used to encode data-intensive time series logs on Agility Robotics machines. It is also used in some cases for real-time communication over a network link.


## Building the library

On a Unix-like system, run `make` to build the ARTL library code as both a static and dynamic library. Either of these libraries can then be used with just the `artl.h` header. This library has been tested on Ubuntu 18.04. A Windows build can be produced by having `mingw-w64` installed and running `make PLATFORM=WIN`.

The rest of this document discusses details of the log format and is not required reading for using the library.


## Goals

Why did we make a proprietary binary log format?

We wanted the following attributes:

- No external schema is needed to decode the log file
- Maximal space efficiency for highly-compressible time series data
- Less than a second of data lost if power is cut without warning

Together these attributes disqualified many common log formats (including CSV, SQLite, LCM, raw binary dumps, Protocol Buffers, HDF5), so we developed the ARTL format to support our use case. Most users of the robot will not need to directly interact with logs in this format, but the code and format description is provided in case it becomes useful.

When we needed to implement a low-latency network communication channel for sending raw torque commands to the robot, we found that ARTL over UDP worked adequately for this purpose. Hopefully the provided sample code makes working with a proprietary format in this context relatively painless.


## Type names

Some definitions of abbreviations used for type names in this document:

- 'u8', 'i32', etc. fields are integers (little endian)
- 'f32', 'f64' fields are floating point values (little endian)
- 'bool' fields are boolean values occupying one byte each
- 'char' fields are text bytes encoded as ASCII
- 'bin' fields are arbitrary bytes not interpreted as numeric or text

This document describes parts of the ARTL format as ordered lists of fields with no implicit padding between fields. Strings are not null terminated; instead, all variable length fields used for strings have an associated length field. All indices start at zero.


## File format

An ARTL log file is organized into chunks, which are contiguous binary sections of varying size, with no padding between chunks. The first chunk is always a single STRT chunk, signifying the start of the data format description. Any number of ENUM, DESC, and CMNT chunks may follow. A single DEND chunk must follow, signifying the end of the description. Any number of UDAT and CDAT chunks may follow until the end of the file.

ARTL logs and streams are usually used to store or communicate tabular data, consisting of a series of fixed-length and fixed-format table rows (referred to as "messages"). The format of the message is described by the DESC chunks as if the message was a packed C struct. The boundaries between separate UDAT or CDAT chunks should be ignored and all of the data in the file should be treated as a contiguous block. The message format is fixed for the entirety of a file or stream.

### Non-tabular data files

In a few special cases, ARTL files will define no fields in the description, but will still contain data in UDAT or CDAT chunks. In this case, the data is not tabular data, and general log file loading tools should not attempt to interpret it as such. Log file loading tools must check the description for a message size of zero and treat the file as if it contains no data if so. Specialized tools for loading these non-tabular files must know the meaning of the data chunks in advance, and may treat the chunk boundaries as significant. Tools that use non-tabular files should use CMNT data to identify the type of non-tabular data in the file and avoid erroneously processing the wrong type of data.

While tabular data is the primary focus of the ARTL format, non-tabular files can be used to store a series of variable-sized chunks of binary data, with each chunk potentially representing a separate data object. Using ARTL for this purpose instead of a custom binary format has the advantages of supporting file metadata in CMNT chunks, built-in data framing, and optional data compression.


## Packet stream format

Streams operate over packet-oriented protocols (nominally UDP), where each chunk is a separate packet. Unlike in a log file, different chunk types can be expected to arrive in any order. A STRT chunk indicates that a sequence of ENUM, DESC, or CMNT chunks completely describing the data format will follow before the next DEND chunk. UDAT and CDAT chunks must contain a single complete message each. UDAT and CDAT chunks may be sent at any time, but should be discarded if a valid DEND chunk has not yet been received or the uncompressed data size doesn't match the expected message size. The sequence of STRT, ENUM, DESC, CMNT, and DEND chunks should be re-transmitted periodically to allow new listeners to synchronize to the stream. The stream is expected to be unreliable in terms of both out-of-order and dropped packets.


## Chunk format

- u32: chunk data length
- char[4]: chunk type
- bin[length]: chunk data, interpretation depends on type
- u32: CRC32c on chunk type and data (see example code)

The length of the binary section is equal to the value of the length field, and does not include the length of the chunk type or CRC fields. Note that the CRC used is not the ether net/deflate polynomial, but the iSCSI polynomial implemented by the Intel CRC32C instruction.


## Chunk types

- 'STRT': marker for the start of a message description
- 'DESc': descriptor for messages in following message data chunks
- 'ENUm': enumeration type definition (must come before affected DESc chunks)
- 'CMNt': comment descriptor/data
- 'DEND': marker for the end of a message description
- 'UDAT': uncompressed message data
- 'CDAT': compressed message data

Chunk types are generally referred to with all uppercase names in this document. Decoders should treat chunk type IDs case insensitively for recognized chunk types. However, the case of the characters should be used to determine how to handle unrecognized chunk types as follows:

- chunk type[0-2]: reserved, should be upper case
- chunk type[3]: lower case indicates that the chunk should be included in the DEND CRC

Encoders should use the casing convention shown above when writing files. The case convention is intended to allow future revisions to introduce new chunk types while allowing old decoders to ignore the content of the unrecognized chunks and still decode the rest of the information in the file.


### STRT chunk data format

STRT chunks contain no data, so the chunk data length is always zero. As a result, valid STRT chunks always have the following form: {0x00, 0x00, 0x00, 0x00, 0x53, 0x54, 0x52, 0x54, 0xca, 0x3f, 0xb6, 0x30}. If the first 12 bytes of a file are not as above, the file is not a valid ARTL log file.


### DESC chunk data format

- Ordered sequence of field descriptors

Each descriptor lists the data type and field name for a range of bytes in each message. Each field can be a 2D array of basic numeric data types. A complete message description may be split into multiple DESC chunks. A DESC chunk with a field referencing an extended data type that has not been defined is considered invalid.

Note that a DESC chunk may be empty and a file may not contain any DESC chunks at all. A file that has no DESC chunks or only empty DESC chunks might be used if it only contains comment data in CMNT chunks or if it contains non-tabular data in its UDAT and CDAT chunks.


#### Field descriptor format

- u16: base data type
- u16: nrows
- u16: ncols
- u16: field name length
- char[length]: field name

The nrows and ncols fields allow fields to be defined as fixed-size vectors or matrices. Zero is allowed for nrows and ncols and causes the field to have zero total length. Multiple field descriptors with the same field name across all of the the DESC chunks in the log file are not allowed. An exception is made if the field name is empty (zero length), in which case the field should be treated as anonymous padding. The same restriction applies to the field names across the CMNT chunks in a file, but there can be a DESC and a CMNT field with the same name.


##### Field data types

- 0: u8
- 1: u16
- 2: u32
- 3: u64
- 4: i8
- 5: i16
- 6: i32
- 7: i64
- 8: f32
- 9: f64
- 10: bool
- 11: char
- 12: bin
- 13-255: reserved
- 256-32767: extended types, can be defined by ENUM chunks
- 32768-65535: reserved


### ENUM chunk data format

- u16: enum extended data type
- u16: underlying enum data type, must be an integral type
- Unordered sequence of enum label descriptors

The extended data type for an enum must be within the range reserved for extended types. Labels for a single enum may be split across multiple ENUM chunks, but the label for a given value can not be redefined. All of the ENUM chunks fully defining an enum must come before any DESC or CMNT chunks that use the enum definition.


#### Enum label descriptor format

- underlying data type: value
- u16: label length
- char[length]: label


### CMNT chunk data format

- u32: uncompressed data length
- bin[length]: uncompressed comment data
- Ordered sequence of field descriptors describing the comment data

CMNT chunks are meant to store static metadata about the file or stream. Field descriptors in CMNT chunks are identical to field descriptors in DESC chunks. CMNT fields can use any previously defined extended data types. There can be multiple CMNT chunks.


### DEND chunk data format

- u32: CRC32c of the chunk type and chunk data fields of the preceding DESC, ENUM, and CMNT chunks concatenated in order

A DEND chunk marks the end of a complete message description and allows the complete description to be checked for validity. This is important for streams, in which swapping the order of two DESC chunks would be otherwise undetectable. Any unknown chunk types with a lowercase fourth chunk type ID character should also be included in the CRC calculation.


### UDAT chunk data format

- bin[chunk data length]: raw bytes corresponding to zero or more complete messages

Messages use the format described by the DESC and ENUM chunks between the last set of STRT and DEND chunks. The bytes of each field are arranged sequentially in the order that the fields were defined in. Matrices are column-major (Fortran/Matlab layout, not C layout). No extra padding is inserted. A data chunk must start and end on a message boundary. Data chunks can be read and interpreted independently from preceding and following data chunks.


### CDAT chunk data format

- bin[chunk data length]: data as in UDAT, but compressed with zstd

See https://github.com/facebook/zstd for information on zstd compression.


## Conventions

- Fields representing mathematical vectors should be column matrices
- Field names represent nested struct fields with '.' as a separator
- Data that is not supposed to change should be represented in a CMNT chunk, not as a field in the message
- Streams periodically resend the STRT, ENUM, DESC, CMNT, and DEND chunks to accommodate new listeners and compensate for dropped packets
- During real-time operation, data is split into multiple roughly-equal-sized chunks that are compressed and written to disk as a unit
