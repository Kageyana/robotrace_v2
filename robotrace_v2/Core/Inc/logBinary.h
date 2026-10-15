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
    static const uint32_t table[256] = {
        0x00000000U, 0x77073096U, 0xee0e612cU, 0x990951baU, 0x076dc419U, 0x706af48fU, 0xe963a535U, 0x9e6495a3U,
        0x0edb8832U, 0x79dcb8a4U, 0xe0d5e91eU, 0x97d2d988U, 0x09b64c2bU, 0x7eb17cbdU, 0xe7b82d07U, 0x90bf1d91U,
        0x1db71064U, 0x6ab020f2U, 0xf3b97148U, 0x84be41deU, 0x1adad47dU, 0x6ddde4ebU, 0xf4d4b551U, 0x83d385c7U,
        0x136c9856U, 0x646ba8c0U, 0xfd62f97aU, 0x8a65c9ecU, 0x14015c4fU, 0x63066cd9U, 0xfa0f3d63U, 0x8d080df5U,
        0x3b6e20c8U, 0x4c69105eU, 0xd56041e4U, 0xa2677172U, 0x3c03e4d1U, 0x4b04d447U, 0xd20d85fdU, 0xa50ab56bU,
        0x35b5a8faU, 0x42b2986cU, 0xdbbbc9d6U, 0xacbcf940U, 0x32d86ce3U, 0x45df5c75U, 0xdcd60dcfU, 0xabd13d59U,
        0x26d930acU, 0x51de003aU, 0xc8d75180U, 0xbfd06116U, 0x21b4f4b5U, 0x56b3c423U, 0xcfba9599U, 0xb8bda50fU,
        0x2802b89eU, 0x5f058808U, 0xc60cd9b2U, 0xb10be924U, 0x2f6f7c87U, 0x58684c11U, 0xc1611dabU, 0xb6662d3dU,
        0x76dc4190U, 0x01db7106U, 0x98d220bcU, 0xefd5102aU, 0x71b18589U, 0x06b6b51fU, 0x9fbfe4a5U, 0xe8b8d433U,
        0x7807c9a2U, 0x0f00f934U, 0x9609a88eU, 0xe10e9818U, 0x7f6a0dbbU, 0x086d3d2dU, 0x91646c97U, 0xe6635c01U,
        0x6b6b51f4U, 0x1c6c6162U, 0x856530d8U, 0xf262004eU, 0x6c0695edU, 0x1b01a57bU, 0x8208f4c1U, 0xf50fc457U,
        0x65b0d9c6U, 0x12b7e950U, 0x8bbeb8eaU, 0xfcb9887cU, 0x62dd1ddfU, 0x15da2d49U, 0x8cd37cf3U, 0xfbd44c65U,
        0x4db26158U, 0x3ab551ceU, 0xa3bc0074U, 0xd4bb30e2U, 0x4adfa541U, 0x3dd895d7U, 0xa4d1c46dU, 0xd3d6f4fbU,
        0x4369e96aU, 0x346ed9fcU, 0xad678846U, 0xda60b8d0U, 0x44042d73U, 0x33031de5U, 0xaa0a4c5fU, 0xdd0d7cc9U,
        0x5005713cU, 0x270241aaU, 0xbe0b1010U, 0xc90c2086U, 0x5768b525U, 0x206f85b3U, 0xb966d409U, 0xce61e49fU,
        0x5edef90eU, 0x29d9c998U, 0xb0d09822U, 0xc7d7a8b4U, 0x59b33d17U, 0x2eb40d81U, 0xb7bd5c3bU, 0xc0ba6cadU,
        0xedb88320U, 0x9abfb3b6U, 0x03b6e20cU, 0x74b1d29aU, 0xead54739U, 0x9dd277afU, 0x04db2615U, 0x73dc1683U,
        0xe3630b12U, 0x94643b84U, 0x0d6d6a3eU, 0x7a6a5aa8U, 0xe40ecf0bU, 0x9309ff9dU, 0x0a00ae27U, 0x7d079eb1U,
        0xf00f9344U, 0x8708a3d2U, 0x1e01f268U, 0x6906c2feU, 0xf762575dU, 0x806567cbU, 0x196c3671U, 0x6e6b06e7U,
        0xfed41b76U, 0x89d32be0U, 0x10da7a5aU, 0x67dd4accU, 0xf9b9df6fU, 0x8ebeeff9U, 0x17b7be43U, 0x60b08ed5U,
        0xd6d6a3e8U, 0xa1d1937eU, 0x38d8c2c4U, 0x4fdff252U, 0xd1bb67f1U, 0xa6bc5767U, 0x3fb506ddU, 0x48b2364bU,
        0xd80d2bdaU, 0xaf0a1b4cU, 0x36034af6U, 0x41047a60U, 0xdf60efc3U, 0xa867df55U, 0x316e8eefU, 0x4669be79U,
        0xcb61b38cU, 0xbc66831aU, 0x256fd2a0U, 0x5268e236U, 0xcc0c7795U, 0xbb0b4703U, 0x220216b9U, 0x5505262fU,
        0xc5ba3bbeU, 0xb2bd0b28U, 0x2bb45a92U, 0x5cb36a04U, 0xc2d7ffa7U, 0xb5d0cf31U, 0x2cd99e8bU, 0x5bdeae1dU,
        0x9b64c2b0U, 0xec63f226U, 0x756aa39cU, 0x026d930aU, 0x9c0906a9U, 0xeb0e363fU, 0x72076785U, 0x05005713U,
        0x95bf4a82U, 0xe2b87a14U, 0x7bb12baeU, 0x0cb61b38U, 0x92d28e9bU, 0xe5d5be0dU, 0x7cdcefb7U, 0x0bdbdf21U,
        0x86d3d2d4U, 0xf1d4e242U, 0x68ddb3f8U, 0x1fda836eU, 0x81be16cdU, 0xf6b9265bU, 0x6fb077e1U, 0x18b74777U,
        0x88085ae6U, 0xff0f6a70U, 0x66063bcaU, 0x11010b5cU, 0x8f659effU, 0xf862ae69U, 0x616bffd3U, 0x166ccf45U,
        0xa00ae278U, 0xd70dd2eeU, 0x4e048354U, 0x3903b3c2U, 0xa7672661U, 0xd06016f7U, 0x4969474dU, 0x3e6e77dbU,
        0xaed16a4aU, 0xd9d65adcU, 0x40df0b66U, 0x37d83bf0U, 0xa9bcae53U, 0xdebb9ec5U, 0x47b2cf7fU, 0x30b5ffe9U,
        0xbdbdf21cU, 0xcabac28aU, 0x53b39330U, 0x24b4a3a6U, 0xbad03605U, 0xcdd70693U, 0x54de5729U, 0x23d967bfU,
        0xb3667a2eU, 0xc4614ab8U, 0x5d681b02U, 0x2a6f2b94U, 0xb40bbe37U, 0xc30c8ea1U, 0x5a05df1bU, 0x2d02ef8dU,
    };
    while (length--) {
        crc = (crc >> 8) ^ table[(crc ^ *data++) & 0xffU];
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
