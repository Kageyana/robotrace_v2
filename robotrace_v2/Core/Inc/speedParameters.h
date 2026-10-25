#ifndef SPEED_PARAMETERS_H_
#define SPEED_PARAMETERS_H_

// SD保存順・既定値・UI表示・調整幅・有効範囲・単位・ログ出力対象の共通定義。
// X(メンバー, 既定値, 表示名, 調整幅, 最小値, 最大値, 表示単位, メタデータ出力1/0)
// bst500の既定値はPARAM_BOOST_500を使用する。
#define SPEED_PARAM_FIELDS(X) \
    X(search, PARAM_SEARCH, "STRAIGHT", 0.1F, 0.0F, 10.0F, "m/s", 0) \
    X(stop, PARAM_STOP, "STOP", 0.1F, 0.0F, 10.0F, "m/s", 0) \
    X(bstStraight, PARAM_BOOST_STRAIGHT, "BST STRT", 0.1F, 0.0F, 10.0F, "m/s", 1) \
    X(bst1500, PARAM_BOOST_1500, "BST 1500", 0.1F, 0.0F, 10.0F, "m/s", 1) \
    X(bst1300, PARAM_BOOST_1300, "BST 1300", 0.1F, 0.0F, 10.0F, "m/s", 1) \
    X(bst1000, PARAM_BOOST_1000, "BST 1000", 0.1F, 0.0F, 10.0F, "m/s", 1) \
    X(bst800, PARAM_BOOST_800, "BST 800", 0.1F, 0.0F, 10.0F, "m/s", 1) \
    X(bst700, PARAM_BOOST_700, "BST 700", 0.1F, 0.0F, 10.0F, "m/s", 1) \
    X(bst600, PARAM_BOOST_600, "BST 600", 0.1F, 0.0F, 10.0F, "m/s", 1) \
    X(bst500, PARAM_BOOST_500, "BST 500", 0.1F, 0.0F, 10.0F, "m/s", 1) \
    X(bst400, PARAM_BOOST_400, "BST 400", 0.1F, 0.0F, 10.0F, "m/s", 1) \
    X(bst300, PARAM_BOOST_300, "BST 300", 0.1F, 0.0F, 10.0F, "m/s", 1) \
    X(bst200, PARAM_BOOST_200, "BST 200", 0.1F, 0.0F, 10.0F, "m/s", 1) \
    X(bst100, PARAM_BOOST_100, "BST 100", 0.1F, 0.0F, 10.0F, "m/s", 1) \
    X(acceleF, MACHINEACCELE, "BST acceleF", 0.1F, 0.0F, 20.0F, "m/ss", 1) \
    X(acceleD, MACHINEDECREACE, "BST acceleD", 0.1F, 0.0F, 20.0F, "m/ss", 1) \
    X(shortCut, PARAM_SHORTCUT, "BST shortCut", 0.1F, 0.0F, 10.0F, "m/s", 0) \
    X(decelLeadMm, PARAM_DECEL_LEAD_MM, "BST dLead", 1.0F, 0.0F, 99.0F, "mm", 1) \
    X(pathReplay, PARAM_PATH_REPLAY, "PATH L0", 0.1F, 0.0F, 10.0F, "m/s", 0)

typedef struct
{
#define SPEED_PARAM_MEMBER(member, initial, label, step, minimum, maximum, unit, metadata) float member;
    SPEED_PARAM_FIELDS(SPEED_PARAM_MEMBER)
#undef SPEED_PARAM_MEMBER
} speedParam;

typedef enum
{
#define SPEED_PARAM_INDEX(member, initial, label, step, minimum, maximum, unit, metadata) SPEED_PARAM_##member,
    SPEED_PARAM_FIELDS(SPEED_PARAM_INDEX)
#undef SPEED_PARAM_INDEX
    TARGET_SPEED_PARAM_COUNT
} SpeedParamIndex;

#define SPEED_PARAM_STORAGE_SCALE 100.0F
#define SPEED_PARAM_STORAGE_FORMAT "%04d,"
#define SPEED_PARAM_STORAGE_READ_FORMAT "%04hd,"
#define SPEED_PARAM_STORAGE_FIELD_BYTES 5U

_Static_assert(TARGET_SPEED_PARAM_COUNT == 19, "targetSpeeds.txt must retain its 19-field format");
_Static_assert(sizeof(speedParam) == 19U * sizeof(float), "speed parameter layout changed");

#endif // SPEED_PARAMETERS_H_
