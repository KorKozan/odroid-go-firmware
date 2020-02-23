/***
 *
 * This tool is a replacement for the mkfw tool at: https://github.com/OtherCrashOverride/odroid-go-firmware
 * This code is in the Public Domain.
 * Alternatively, if Public Domain is not feasible in your jurisdiction or if it doesn't meet your needs,
 * you can use it under the terms and conditions of the Zero Clause BSD license:
 *
 * Copyright (C) 2020 by KorKozan <kozankor@gmail.com>
 *
 * Permission to use, copy, modify, and/or distribute this software for any purpose with or without fee is hereby granted.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT,
 * INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN
 * AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 *
 ***/

#ifdef __linux__
#   define _DEFAULT_SOURCE
#endif

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <limits.h>
#include <errno.h>
#include <assert.h>
#include <getopt.h>

#if defined (__APPLE__)

#   include <machine/endian.h>
#	include <libkern/OSByteOrder.h>

#	define htobe16(x) OSSwapHostToBigInt16(x)
#	define htole16(x) OSSwapHostToLittleInt16(x)
#	define be16toh(x) OSSwapBigToHostInt16(x)
#	define le16toh(x) OSSwapLittleToHostInt16(x)

#	define htobe32(x) OSSwapHostToBigInt32(x)
#	define htole32(x) OSSwapHostToLittleInt32(x)
#	define be32toh(x) OSSwapBigToHostInt32(x)
#	define le32toh(x) OSSwapLittleToHostInt32(x)

#	define htobe64(x) OSSwapHostToBigInt64(x)
#	define htole64(x) OSSwapHostToLittleInt64(x)
#	define be64toh(x) OSSwapBigToHostInt64(x)
#	define le64toh(x) OSSwapLittleToHostInt64(x)

#elif defined (__linux__)
#   include <endian.h>
#else
#   include <sys/endian.h>
#endif

// implemented in crc32.c
unsigned long crc32(unsigned long crc, const unsigned char* buf, unsigned int len);

typedef struct odroid_partition_t
{
    uint8_t     type;
    uint8_t     subtype;
    uint8_t     _reserved0;
    uint8_t     _reserved1;

    uint8_t     label[16];

    uint32_t    flags;
    uint32_t    length;
} odroid_partition_t;
static_assert (sizeof(odroid_partition_t) == 28, "The partition header should be exactly 28 bytes");

// app return codes
enum return_code_t
{
    RET_OK              =  0, // all OK
    RET_INVALID_PARAMS  = -1, // parameters are invalid
    RET_READ_ERROR      = -2, // I/O error while reading from a file
    RET_WRITE_ERROR     = -3, // I/O error while writing to the output file
};

// values for predefined partition types
enum partition_type_t
{
    PARTITION_APP   = 0, // 'app' partition type
    PARTITION_DATA  = 1, // 'data' partition type
};

// values for predefined partition subtypes
enum partition_subtype_t
{
    APP_FACTORY     =    0, // 'factory' app partition subtype
    APP_TEST        = 0x20, // 'test' app partition subtype
    DATA_OTA        =    0, // 'ota' data partition subtype
    DATA_PHY        =    1, // 'phy' data partition subtype
    DATA_NVS        =    2, // 'nvs' data partition subtype
};
// 'ota_0' to 'ota_15' app partition subtypes
#define APP_OTA(X) (0x10 + ((X) & 0x0f))

// clean up and return from app
#define RETURN(x) do { result = (x); goto cleanup; } while (false)

// current version
static const char* kVersion = "1.0.0 (20200222)";

static const struct option kLongOptions[] = {
//   NAME       ARGUMENT           FLAG  SHORTNAME
    {"help",    no_argument,       NULL, 'h'},
    {"header",  required_argument, NULL, 'H'},
    {"out",     required_argument, NULL, 'o'},
    {"quiet",   no_argument,       NULL, 'q'},
    {"version", no_argument,       NULL, 'V'},
    {NULL,      0,                 NULL, 0}
};

enum
{   // constants
    kHeaderSize = 24,
    kFirmwareDescriptionSize = 40,
    kTileSize = 86 * 48 * 2,
    kBufferSize = 10 * 1024 * 1024,
};

// default firmware file name
static const char* kDefaultFirmwareName = "firmware.fw";

// default firmware header
static const char* kDefaultHeader = "ODROIDGO_FIRMWARE_V00_01";
static char sHeader[kHeaderSize + 1];

static char sFirmwareDescription[kFirmwareDescriptionSize];

// ffmpeg -i tile.png -f rawvideo -pix_fmt rgb565 tile.raw
// 8KB: 86x48 pixels, 16bpp

// 10 MB for the buffer, make sure this is at least as large as kTileSize
static_assert (kBufferSize >= kTileSize, "The buffer should be at least as large as the tile size");

static uint8_t sBuffer[kBufferSize];

// write bytes to the file and also accumulate the CRC32 for the written bytes
// returns true on success, false otherwise
static bool writeAndCrc(const void* ptr, size_t size, FILE* stream, uint32_t* crc)
{
    *crc = crc32(*crc, ptr, size);
    return (fwrite(ptr, size, 1, stream) == 1);
}

int main(int argc, char *argv[])
{
    int result = RET_OK;
    uint32_t checksum = 0;
    const char* firmwareName = kDefaultFirmwareName;
    memset(sHeader, 0, sizeof(sHeader));
    strncpy(sHeader, kDefaultHeader, kHeaderSize);
    bool quiet = false;

    {
        int opt = 0;
        int optionIndex = 0;
        while ((opt = getopt_long(argc, argv, "hH:o:qV", kLongOptions, &optionIndex)) != -1) {
            switch (opt) {
            case 'h': // help
                // tabulation guidelines to keep each line less than 80 characters
                //     |         1         2         3         4         5         6         7         8|
                //     |12345678901234567890123456789012345678901234567890123456789012345678901234567890|
                printf("Usage: mkfw [ options ] [ description tile partition... ]\n");
                printf("  partition is a tuple of 5 parameters: type subtype length label binary\n");
                printf("Version: %s\n\n", kVersion);
                printf("  description   firmware description; truncated to %d bytes\n", kFirmwareDescriptionSize);
                printf("  tile          86x48 pixels raw bit pixmap in RGB565 format (%d bytes)\n", kTileSize);
                printf("  type          partition type; valid values are:\n");
                printf("                  app, 0 or 0x00\n");
                printf("                  data, 1, or 0x01\n");
                printf("                  a custom type in the range 0x40 - 0xfe\n");
                printf("                types 0x02 - 0x3f are reserved for esp-idf core functions\n");
                printf("                type can be specified as one of the app or data literals,\n");
                printf("                  a decimal or a hex number starting with 0x/0X\n");
                printf("  subtype       partition subtype; valid values are:\n");
                printf("                  app partitions:\n");
                printf("                    factory/0/0x00, ota_0/0x10, ... ota_15/0x1f\n");
                printf("                  data partitions:\n");
                printf("                    ota/0/0x00, phy/1/0x01, nvs/2/0x02\n");
                printf("                  custom partitions:\n");
                printf("                    any number between 0x00 and 0xff\n");
                printf("                subtype can be specified either as one of the literals,\n");
                printf("                  a decimal or a hex number starting with 0x/0X\n");
                printf("  length        partition length, should be a multiple of 64K; valid values:\n");
                printf("                  auto: calculate partition size based on the actual file size\n");
                printf("                  decimal number or hex number starting with 0x/0X followed by\n");
                printf("                    an optional size multiplier suffix:\n");
                printf("                      k, K: KB (1024 bytes)\n");
                printf("                      m, M: MB (1024 * 1024 bytes)\n");
                printf("  label         partition label; truncated to 16 bytes\n");
                printf("  binary        file that contains the actual partition data\n");
                printf("\n");
                printf("  -h, --help    print this help message and exit\n");
                printf("  -H header, --header=header\n");
                printf("                use the specified header instead of the default one\n");
                printf("                the header will be truncated to %d bytes\n", kHeaderSize);
                printf("  -o firmware, --out=firmware\n");
                printf("                output to the specified file instead of '%s'\n", kDefaultFirmwareName);
                printf("  -q, --quiet   quiet operation\n");
                printf("  -V, --version print version and exit\n\n");
                printf("Exit status:    mkfw exits with one of the following status codes:\n");
                printf("   0            no error occured\n");
                printf("  -1            invalid argument(s)\n");
                printf("  -2            error reading from file\n");
                printf("  -3            error writing to file\n\n");
                return RET_OK;
            case 'H':
                memset(sHeader, 0, sizeof(sHeader));
                strncpy(sHeader, optarg, kHeaderSize);
                break;
            case 'o':
                firmwareName = optarg;
                break;
            case 'q':
                quiet = true;
                break;
            case 'V':
                printf("%s\n", kVersion);
                return RET_OK;
            //case ':':
            //    fprintf(stderr, "Missing argument to option %s\n", argv[optind - 1]);
            //    return RET_INVALID_PARAMS;
            default:
                //fprintf(stderr, "Unknown option: %s\n", argv[optind - 1]);
                return RET_INVALID_PARAMS;
                break;
            }
        }
    }

    int arg = optind;
    if (arg >= argc) {
        fprintf(stderr, "Missing firmware description\n");
        return RET_INVALID_PARAMS;
    }
    memset(sFirmwareDescription, 0, kFirmwareDescriptionSize);
    strncpy(sFirmwareDescription, argv[arg], kFirmwareDescriptionSize);

    ++arg;
    if (arg >= argc) {
        fprintf(stderr, "Missing tile file\n");
        return RET_INVALID_PARAMS;
    }
    const char* tileFileName = argv[arg];

    FILE* file = fopen(firmwareName, "wb");
    if (!file) {
        perror("Error");
        fprintf(stderr, "Could not create firmware file: %s\n", firmwareName);
        return RET_READ_ERROR;
    }

    {   // block: write the header
        if (!writeAndCrc(sHeader, kHeaderSize, file, &checksum)) {
            perror("Error");
            fprintf(stderr, "Could not write header '%s' to firmware file: %s\n", sHeader, firmwareName);
            RETURN (RET_WRITE_ERROR);
        }
        if (!quiet) {
            printf("Header: '%s'\n", sHeader);
        }
    }

    {   // block: write the firmware description

        if (!writeAndCrc(sFirmwareDescription, kFirmwareDescriptionSize, file, &checksum)) {
            perror("Error");
            fprintf(stderr, "Could not write firmware description '%s' to firmware file: %s\n", sFirmwareDescription, firmwareName);
            RETURN (RET_WRITE_ERROR);
        }
        if (!quiet) {
            printf("Firmware Description: '%s'\n", sFirmwareDescription);
        }
    }

    {   // block: copy the tile
        FILE* tileFile = fopen(tileFileName, "rb");
        if (!tileFile) {
            perror("Error");
            fprintf(stderr, "Tile file not found: %s", tileFileName);
            RETURN (RET_READ_ERROR);
        }

        const size_t count = fread(sBuffer, 1, kTileSize, tileFile);
        if (count != kTileSize) {
            if (feof(tileFile)) {
                fprintf(stderr, "Tile file is too small. Expected %d bytes, got %zu: %s\n", kTileSize, count, tileFileName);
                fclose(tileFile);
                RETURN (RET_INVALID_PARAMS);
            }
            perror("Error");
            fprintf(stderr, "Could not read tile file: %s", tileFileName);
            fclose(tileFile);
            RETURN (RET_READ_ERROR);
        }
        if (fgetc(tileFile) != EOF) {
            fprintf(stderr, "Tile file is too large. Expected %d bytes: %s\n", kTileSize, tileFileName);
            fclose(tileFile);
            RETURN (RET_INVALID_PARAMS);
        }
        fclose(tileFile);

        if (!writeAndCrc(sBuffer, kTileSize, file, &checksum)) {
            perror("Error");
            fprintf(stderr, "Could not write tile to firmware file: %s\n", firmwareName);
            RETURN (RET_WRITE_ERROR);
        }
        if (!quiet) {
            printf("Tile: wrote %zu bytes.\n", count);
        }
    }

    int partCount = 0;
    for (++arg; arg < argc; ++arg, ++partCount) {
        odroid_partition_t part;
        memset(&part, 0, sizeof(odroid_partition_t));

        {   // block: parse partition type
            if (strcmp(argv[arg], "app") == 0) {
                part.type = PARTITION_APP;
            } else if (strcmp(argv[arg], "data") == 0) {
                part.type = PARTITION_DATA;
            } else {
                char* endPtr = NULL;
                const unsigned long value = strtoul(argv[arg], &endPtr, 0);
                if (!value) {
                    if (endPtr == argv[arg]) {
                        fprintf(stderr, "Could not parse partition %d type: %s\n", partCount, argv[arg]);
                        RETURN (RET_INVALID_PARAMS);
                    }
                } else if (value == ULONG_MAX) {
                    if (errno == ERANGE) {
                        perror("Error");
                        fprintf(stderr, "Partition %d type is too large: %s\n", partCount, argv[arg]);
                        RETURN (RET_INVALID_PARAMS);
                    }
                }
                part.type = value;
                if (value != (unsigned long) part.type) {
                    fprintf(stderr, "Partition %d type is too large: %s\n", partCount, argv[arg]);
                    RETURN (RET_INVALID_PARAMS);
                }
            }
            if ((part.type != PARTITION_APP) && (part.type != PARTITION_DATA) && (part.type < 0x40)) {
                fprintf(stderr, "Partition type %s is reserved, please use app (0), data (1) or a type >= 0x40 for partition %d\n", argv[arg], partCount);
                RETURN (RET_INVALID_PARAMS);
            }
        }

        ++arg;
        if (arg >= argc) {
            fprintf(stderr, "Missing partition %d subtype\n", partCount);
            return (RET_INVALID_PARAMS);
        }
        {   // block: parse partion subtype
            bool numericSubtype = true;
            if (part.type == PARTITION_APP) {
                if (strcmp(argv[arg], "factory") == 0) {
                    part.subtype = APP_FACTORY;
                    numericSubtype = false;
                //} else if (strcmp(argv[i], "test") == 0) {
                //    part.subtype = APP_TEST;
                //    numericSubtype = false;
                } else if (strncmp(argv[arg], "ota_", 4) == 0) {
                    bool invalidOta = false;
                    // check the first digit
                    if ((argv[arg][4] < '0') || (argv[arg][4] > '9')) {
                        invalidOta = true;
                    } else {
                        part.subtype = argv[arg][4] - '0';
                        if (argv[arg][5]) {
                            // we have a second digit
                            if (argv[arg][6] || (part.subtype != 1) || (argv[arg][5] < '0') || (argv[arg][5] > '5')) {
                                invalidOta = true;
                            } else {
                                part.subtype = part.subtype * 10 + argv[arg][5] - '0';
                            }
                        }
                    }
                    if (invalidOta) {
                        fprintf(stderr, "Invalid OTA partition subtype for partition %d, should be one of ota_0, ..., ota_15: %s\n", partCount, argv[arg]);
                        RETURN (RET_INVALID_PARAMS);
                    }
                    part.subtype = APP_OTA(part.subtype);
                    numericSubtype = false;
                }
            } else if (part.type == PARTITION_DATA) {
                if (strcmp(argv[arg], "ota") == 0) {
                    part.subtype = DATA_OTA;
                    numericSubtype = false;
                } else if (strcmp(argv[arg], "phy") == 0) {
                    part.subtype = DATA_PHY;
                    numericSubtype = false;
                } else if (strcmp(argv[arg], "nvs") == 0) {
                    part.subtype = DATA_NVS;
                    numericSubtype = false;
                }
            }
            if (numericSubtype) {
                char* endPtr = NULL;
                const unsigned long value = strtoul(argv[arg], &endPtr, 0);
                if (!value) {
                    if (endPtr == argv[arg]) {
                        fprintf(stderr, "Could not parse partition %d subtype: %s\n", partCount, argv[arg]);
                        RETURN (RET_INVALID_PARAMS);
                    }
                } else if (value == LONG_MAX) {
                    if (errno == ERANGE) {
                        perror("Error");
                        fprintf(stderr, "Partition %d subtype is too large: %s\n", partCount, argv[arg]);
                        RETURN (RET_INVALID_PARAMS);
                    }
                }
                part.subtype = value;
                if (endPtr && *endPtr) {
                    fprintf(stderr, "Could not parse partition %d subtype: %s\n", partCount, argv[arg]);
                    RETURN (RET_INVALID_PARAMS);
                }
                if (value != (unsigned long) part.subtype) {
                    fprintf(stderr, "Partition %d subtype is too large: %s\n", partCount, argv[arg]);
                    RETURN (RET_INVALID_PARAMS);
                }
                if (part.type == PARTITION_APP) {
                    if ((part.subtype != APP_FACTORY) && ((part.subtype < 0x10) || (part.subtype > 0x1f))) {
                        fprintf(stderr, "Partition %d APP subtypes can only be 'factory' (0x0), 'ota_0' (0x10) to 'ota_15' (0x1f): %s\n", partCount, argv[arg]);
                        RETURN (RET_INVALID_PARAMS);
                    }
                } else if (part.type == PARTITION_DATA) {
                    if (part.subtype > 2) {
                        fprintf(stderr, "Partition %d DATA subtypes can only be 'ota' (0), 'phy' (1) or 'nvs' (2): %s\n", partCount, argv[arg]);
                        RETURN (RET_INVALID_PARAMS);
                    }
                }
            }
        }

        ++arg;
        if (arg >= argc) {
            fprintf(stderr, "Missing partition %d size\n", partCount);
            return (RET_INVALID_PARAMS);
        }
        const bool autoPartitionSize = (strcmp(argv[arg], "auto") == 0);
        if (!autoPartitionSize) {   // block: parse partition size
            char* endPtr = NULL;
            const unsigned long value = strtoul(argv[arg], &endPtr, 0);
            if (!value) {
                if (endPtr == argv[arg]) {
                    fprintf(stderr, "Could not parse partition %d size: %s\n", partCount, argv[arg]);
                    RETURN (RET_INVALID_PARAMS);
                }
            } else if (value == LONG_MAX) {
                if (errno == ERANGE) {
                    perror("Error");
                    fprintf(stderr, "Partition %d size is too large: %s\n", partCount, argv[arg]);
                    RETURN (RET_INVALID_PARAMS);
                }
            }
            if (endPtr) {
                switch (*endPtr) {
                //case 'g':
                //case 'G':
                //    part.length *= 1024;
                    // fallthrough
                case 'm':
                case 'M':
                    part.length *= 1024;
                    // fallthrough
                case 'k':
                case 'K':
                    part.length *= 1024;
                    // fallthrough
                case 0:
                    break;
                default:
                    fprintf(stderr, "Could not parse partition %d size: %s\n", partCount, argv[arg]);
                    RETURN (RET_INVALID_PARAMS);
                }
            }
            part.length = value;
            if (value != (unsigned long) part.length) {
                fprintf(stderr, "Partition %d size is too large: %s\n", partCount, argv[arg]);
                RETURN (RET_INVALID_PARAMS);
            }
            // the size should be 64K aligned
            if (value & 0xffff) {
                fprintf(stderr, "Partition %d size is not aligned to 64K: %s\n", partCount, argv[arg]);
                RETURN (RET_INVALID_PARAMS);
            }
        }

        ++arg;
        if (arg >= argc) {
            fprintf(stderr, "Missing partition %d label\n", partCount);
            RETURN (RET_INVALID_PARAMS);
        }
        strncpy((char*) part.label, argv[arg], sizeof(part.label));

        ++arg;
        if (arg >= argc) {
            fprintf(stderr, "Missing partition %d file name\n", partCount);
            RETURN (RET_INVALID_PARAMS);
        }
        {   // block: copy the partition
            FILE* binary = fopen(argv[arg], "rb");
            if (!binary) {
                perror("Error");
                fprintf(stderr, "Partition %d file not found: %s\n", partCount, argv[arg]);
                RETURN (RET_READ_ERROR);
            }

            // get the file size
            if (fseek(binary, 0, SEEK_END)) {
                perror("Error");
                fprintf(stderr, "Could not get partition %d file size: %s\n", partCount, argv[arg]);
                fclose(binary);
                RETURN (RET_READ_ERROR);
            }
            const long fileSize = ftell(binary);
            if (fileSize == -1) {
                perror("Error");
                fprintf(stderr, "Could not get partition %d file size: %s\n", partCount, argv[arg]);
                fclose(binary);
                RETURN (RET_READ_ERROR);
            }
            if (fseek(binary, 0, SEEK_SET)) {
                perror("Error");
                fprintf(stderr, "Could not read partition %d file: %s\n", partCount, argv[arg]);
                fclose(binary);
                RETURN (RET_READ_ERROR);
            }

            uint32_t length = (uint32_t)fileSize;
            if (autoPartitionSize) {
                part.length = length;
                if (part.length & 0xffff) {
                    part.length = (part.length | 0xffff) + 1;
                }
            }
            if (!quiet) {
                printf("[%d] type=%d, subtype=%d, length=%d, label=%-16s\n", partCount, part.type, part.subtype, part.length, part.label);
            }

            if (length > part.length) {
                fprintf(stderr, "Actual partition size: %u is larger than the declared size: %u for partition %d\n", length, part.length, partCount);
                fclose(binary);
                RETURN (RET_INVALID_PARAMS);
            }

            // make sure the entry fields have the correct byte order (LE)
            part.length = htole32(part.length);
            // part.flags = htole32(part.flags);
            // write the entry
            if (!writeAndCrc(&part, sizeof(part), file, &checksum)) {
                perror("Error");
                fprintf(stderr, "Could not write partition %d entry\n", partCount);
                fclose(binary);
                RETURN (RET_WRITE_ERROR);
            }

            length = htole32(length);
            if (!writeAndCrc(&length, sizeof(length), file, &checksum)) {
                perror("Error");
                fprintf(stderr, "Could not write partition %d size\n", partCount);
                fclose(binary);
                RETURN (RET_WRITE_ERROR);
            }

            while (!feof(binary)) {
                const size_t count = fread(sBuffer, 1, kBufferSize, binary);
                if (ferror(binary)) {
                    perror("Error");
                    fprintf(stderr, "Could not read partition %d data\n", partCount);
                    fclose(binary);
                    RETURN (RET_READ_ERROR);
                }
                if (!writeAndCrc(sBuffer, count, file, &checksum)) {
                    perror("Error");
                    fprintf(stderr, "Could not write partition %d data\n", partCount);
                    fclose(binary);
                    RETURN (RET_WRITE_ERROR);
                }
            }
            fclose(binary);

            if (!quiet) {
                printf("part=%d, length=%d, data=%s\n", partCount, length, argv[arg]);
            }
        }
    }

    if (!quiet) {
        printf("checksum: %#010x\n", checksum);
    }

    checksum = htole32(checksum);
    if (fwrite(&checksum, sizeof(checksum), 1, file) != 1) {
        perror("Could not write checksum");
        RETURN (RET_WRITE_ERROR);
    }

cleanup:
    fclose(file);
    return result;
}
