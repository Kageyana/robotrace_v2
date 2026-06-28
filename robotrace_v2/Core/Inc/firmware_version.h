#ifndef FIRMWARE_VERSION_H_
#define FIRMWARE_VERSION_H_

// ログヘッダに残す手動更新用のファームウェア版。
#define FW_VERSION "v2-dev"

#ifndef GIT_COMMIT
#define GIT_COMMIT "unknown"
#endif

#ifndef GIT_BRANCH
#define GIT_BRANCH "unknown"
#endif

#define BUILD_DATE __DATE__
#define BUILD_TIME __TIME__

#endif // FIRMWARE_VERSION_H_
