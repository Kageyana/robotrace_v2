#include "logMetadata.h"
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

/////////////////////////////////////////////////////////////////////
// モジュール名 logMetadataReadInt
// 処理概要     走行メタデータから重複のない整数相当値を範囲検証して取得する
// 引数         line: key=valueのメタデータ, key: 項目名, value: 成功時の格納先
// 戻り値       true: 取得成功 false: 欠落・重複・端数・範囲外または不正入力
/////////////////////////////////////////////////////////////////////
bool logMetadataReadInt(const char *line, const char *key, int32_t *value)
{
    if (line == NULL || key == NULL || value == NULL || *key == '\0' ||
        strpbrk(key, ",=\r\n") != NULL) return false;
    const size_t length = strlen(key);
    bool found = false;
    int32_t parsed = 0;
    for (const char *field = line; field != NULL && *field != '\0'; )
    {
        if (strncmp(field, key, length) == 0 && field[length] == '=')
        {
            char *end;
            errno = 0;
            const double number = strtod(field + length + 1U, &end);
            if (found || end == field + length + 1U || errno == ERANGE ||
                (*end != ',' && *end != '\r' && *end != '\n' && *end != '\0') ||
                !isfinite(number) || number < INT32_MIN || number > INT32_MAX ||
                number != trunc(number)) return false;
            parsed = (int32_t)number;
            found = true;
        }
        const char *next = strchr(field, ',');
        field = next != NULL ? next + 1 : NULL;
    }
    if (found) *value = parsed;
    return found;
}
