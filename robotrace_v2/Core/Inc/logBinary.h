#ifndef LOG_BINARY_H_
#define LOG_BINARY_H_
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <limits.h>

#define LOG_BINARY_HEADER_BYTES 512U
#define LOG_BINARY_VERSION 1U

typedef struct {
    uint32_t number, profile, recordBytes, rows, metadataBytes;
    uint32_t dataCrc, metadataCrc, schema;
} LogBinaryHeader;

/////////////////////////////////////////////////////////////////////
// モジュール名 logBinaryCrcUpdate
// 処理概要     CRC32/IEEEを更新する（開始値と終了XORは0xffffffff）
// 引数         crc: 更新前CRC, data: データ, length: 長さ[B]
// 戻り値       更新後CRC
/////////////////////////////////////////////////////////////////////
static inline uint32_t logBinaryCrcUpdate(uint32_t crc, const uint8_t *data, size_t length)
{
    while (length--) {
        crc ^= *data++;
        for (unsigned bit = 0; bit < 8U; bit++) crc = (crc >> 1) ^ (0xedb88320U & (0U - (crc & 1U)));
    }
    return crc;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logBinaryPut32
// 処理概要     コンテナ整数をリトルエンディアンで格納する
// 引数         p: 格納先, value: 値
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static inline void logBinaryPut32(uint8_t *p, uint32_t value)
{
    for (unsigned i = 0; i < 4U; i++) p[i] = (uint8_t)(value >> (8U * i));
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logBinaryGet32
// 処理概要     コンテナ整数をリトルエンディアンで復元する
// 引数         p: 読込元
// 戻り値       復元値
/////////////////////////////////////////////////////////////////////
static inline uint32_t logBinaryGet32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logBinaryEncodeHeader
// 処理概要     512 Bヘッダへ識別情報とCRCを格納する
// 引数         bytes: 出力先, header: 保存情報
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static inline void logBinaryEncodeHeader(uint8_t bytes[LOG_BINARY_HEADER_BYTES], const LogBinaryHeader *header)
{
    memset(bytes, 0, LOG_BINARY_HEADER_BYTES);
    memcpy(bytes, "RLOG", 4U);
    logBinaryPut32(bytes + 4, LOG_BINARY_VERSION);
    const uint32_t values[] = {header->number, header->profile, header->recordBytes, header->rows,
        header->metadataBytes, header->dataCrc, header->metadataCrc, header->schema};
    for (unsigned i = 0; i < 8U; i++) logBinaryPut32(bytes + 8U + i * 4U, values[i]);
    logBinaryPut32(bytes + 40, ~logBinaryCrcUpdate(UINT32_MAX, bytes, 40U));
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logBinaryDecodeHeader
// 処理概要     バージョン・ヘッダCRC・サイズを検証して復元する
// 引数         bytes: ヘッダ, fileBytes: ファイル長[B], header: 復元先
// 戻り値       true: 有効 false: 不正
/////////////////////////////////////////////////////////////////////
static inline bool logBinaryDecodeHeader(const uint8_t bytes[LOG_BINARY_HEADER_BYTES], uint64_t fileBytes, LogBinaryHeader *header)
{
    if (memcmp(bytes, "RLOG", 4U) || logBinaryGet32(bytes + 4) != LOG_BINARY_VERSION ||
        logBinaryGet32(bytes + 40) != ~logBinaryCrcUpdate(UINT32_MAX, bytes, 40U)) return false;
    header->number = logBinaryGet32(bytes + 8);
    header->profile = logBinaryGet32(bytes + 12);
    header->recordBytes = logBinaryGet32(bytes + 16);
    header->rows = logBinaryGet32(bytes + 20);
    header->metadataBytes = logBinaryGet32(bytes + 24);
    header->dataCrc = logBinaryGet32(bytes + 28);
    header->metadataCrc = logBinaryGet32(bytes + 32);
    header->schema = logBinaryGet32(bytes + 36);
    return header->number > 0U && header->number <= INT16_MAX && header->profile <= 1U &&
        header->recordBytes > 0U && header->recordBytes <= 512U && header->rows <= UINT16_MAX &&
        header->metadataBytes > 0U && header->metadataBytes < 4096U &&
        fileBytes == LOG_BINARY_HEADER_BYTES + (uint64_t)header->recordBytes * header->rows + header->metadataBytes;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logBinaryName
// 処理概要     ログ番号と拡張子からファイル名を生成する
// 引数         name: 出力先, capacity: 容量[B], number: ログ番号, extension: 拡張子
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static inline void logBinaryName(char *name, size_t capacity, uint16_t number, const char *extension)
{
    (void)snprintf(name, capacity, "%u.%s", number, extension);
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logBinaryParseName
// 処理概要     数字だけのログ番号と指定拡張子を厳密に検証する
// 引数         name: ファイル名, extension: 拡張子, number: 番号出力
// 戻り値       true: 対象ファイル false: 対象外
/////////////////////////////////////////////////////////////////////
static inline bool logBinaryParseName(const char *name, const char *extension, uint16_t *number)
{
    uint32_t value = 0U;
    if (*name < '0' || *name > '9') return false;
    do {
        value = value * 10U + (uint32_t)(*name++ - '0');
        if (value > INT16_MAX) return false;
    } while (*name >= '0' && *name <= '9');
    if (*name++ != '.' || value == 0U) return false;
    while (*extension) {
        char c = *name++;
        if (c >= 'A' && c <= 'Z') c = (char)(c + ('a' - 'A'));
        if (c != *extension++) return false;
    }
    if (*name) return false;
    *number = (uint16_t)value;
    return true;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logBinaryReservedName
// 処理概要     保存済み・作業中ログに予約された番号を取得する
// 引数         name: ファイル名, number: 番号出力
// 戻り値       true: 予約番号 false: 対象外
/////////////////////////////////////////////////////////////////////
static inline bool logBinaryReservedName(const char *name, uint16_t *number)
{
    return logBinaryParseName(name, "csv", number) || logBinaryParseName(name, "bin", number) ||
        logBinaryParseName(name, "tmp", number) || logBinaryParseName(name, "part", number);
}
#endif
