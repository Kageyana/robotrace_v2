#ifndef LOG_METADATA_H_
#define LOG_METADATA_H_

#include <stdbool.h>
#include <stdint.h>

// 停止中のログ解析用。整数相当の小数表記を許容し、欠落・重複・不正値を拒否する。
bool logMetadataReadInt(const char *line, const char *key, int32_t *value);

#endif // LOG_METADATA_H_
