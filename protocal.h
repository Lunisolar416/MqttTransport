#ifndef PROTOCOL_H_

#define PROTOCOL_H_

#include <cstdint>
#include <nlohmann/json.hpp>
#include <string>
#include <sys/types.h>
#include <vector>

constexpr double DEG_TO_INT = (1LL << 31) / 180.0;
// 经纬度转换系数 (2^31)/180
constexpr double LONLAT_SCALE = static_cast<double>(1LL << 31) / 180.0;

// -1 对应 uint16_t 最大值，转换时用65535表示无效长度/宽度
constexpr uint16_t INVALID_LENGTH_WIDTH = 65535;

struct Header
{
    uint32_t packageSeq;   // 报文序号，递增序列 [4 bytes]
    uint16_t packageType;  // 报文类型：0=标准协议，1=应答 [2 bytes]
    uint16_t infoUnitId;   // 信息单元标识，参见协议定义 [2 bytes]
    uint64_t time;         // 时间戳，从1970-01-01起的毫秒数 [8 bytes]
    uint16_t sendPlatform; // 发送平台编码 [2 bytes]
    uint16_t recvPlatform; // 接收平台编码 [2 bytes]
};

// 水域类型上报协议0041
struct Environmental
{
    uint8_t type; // 水域环境类型,0：禁航区，1：宽阔水域，2：复杂水域
};
// 组件状态协议 0051 心跳报文结构体定义
enum SensorErrorCode : uint16_t
{
    ERROR_NONE = 0x0000,
    ERROR_0111_RADAR_TIMEOUT = 0x0101,
    ERROR_0131_LIDAR_TIMEOUT = 0x0102,
    ERROR_0411_AIS_TIMEOUT = 0x0103,
};

enum ComponentID : uint16_t
{
    COMPONENT_THIS = 0x0141, // 本组件：态势融合
    COMPONENT_0421 = 0x0421, // AIS
    COMPONENT_0111 = 0x0111, // 雷达
    COMPONENT_0131 = 0x0131  // 激光雷达
};

struct ComponentStatus
{
    // 组件编码（2字节）
    // 例如：0411：卫星导航，0412：MRU 等
    uint16_t component_id;

    // 组件状态（1字节）
    // 0：异常，1：正常
    uint8_t state;

    // 错误数量（1字节）
    uint8_t error_count;

    // 错误码列表（每个2字节）
    // 0000为无效，范围0001~0FFF具体错误码由各组件定义
    std::vector<SensorErrorCode> error_codes;
};

// 单个目标数据结构
struct RadarTarget
{
    uint16_t batch;      // 批号，0-65534
    uint16_t absAzimuth; // 绝对方位，单位0.1度，范围0~3600（0~360度 * 10）
    uint32_t distance;   // 距离，单位0.1米
    uint16_t absSpeed;   // 绝对航速，单位0.1节
    uint16_t absCourse;  // 绝对航向，单位0.1度，范围0~3600
    int32_t longitude;   // 经度，32位带符号整数，单位度，东经为正，最低有效位180/2^31
    int32_t latitude;    // 纬度，32位带符号整数，单位度，北纬为正，最低有效位180/2^31
    uint8_t state;       // 状态，0：跟踪；1：询问；2：丢失；3：不稳定跟踪
};

// 导航雷达目标协议0111消息整体结构
struct RadarTargetsMsg
{
    uint16_t count;                   // 目标数目
    std::vector<RadarTarget> targets; // 目标数组
};

RadarTargetsMsg jsonToRadarTargets(const nlohmann::json &input);

// 目标识别协议 0121
// 目标识别协议 0121
struct TargetRecognition
{
    int batch;
    int sign;
    int source;
    int type;
    int color;

    char code[16]; // 识别舷号：16字节，0=无效，全1=不明（如用 memset 全1）

    uint16_t confidence;  // 置信度，实际值 = value * 0.1；范围0~1000，单位0.1
    uint16_t height;      // 距离，实际值 = value * 0.1 米
    uint16_t orientation; // 水平方位，实际值 = value * 0.1 度
    uint16_t pitch;       // 垂直方位，实际值 = value * 0.1 度
};

// 激光雷达目标协议 0131
struct LidarTarget
{
    uint16_t batch;      // 批号
    uint16_t absAzimuth; // 方位
    uint32_t distance;   // 距离
    uint16_t absSpeed;   // 绝对速度，单位0，1节
    uint16_t absCourse;  // 绝对航向，单位0.1度，范围0~3600
    uint16_t heading;    // 朝向
    int32_t longitude;   // 经度，32位带符号整数，单位度，东经为正
    int32_t latitude;    // 纬度，32位带符号整数，单位度，北纬为正
    uint8_t state;       // 状态，0：跟踪；1：询问；2：丢失；3：不稳定跟踪
    uint16_t length;     // 长度，单位0.1米,-1为无效
    uint16_t width;      // 宽度，单位0.1米,-1为无效
};
struct LidarTargetsMsg
{
    uint16_t count;
    std::vector<LidarTarget> targets; // 目标数组
};

LidarTargetsMsg jsonToLidarTargets(const nlohmann::json &input);

// 单艇融合目标信息0141
struct VesselFusionTarget
{
    uint16_t batch;     // 批号：唯一目标编号
    uint16_t absSpeed;  // 绝对航速：单位0.1节
    uint16_t absCourse; // 绝对航向：单位0.1度，0~3600
    int16_t heading;    // 船首向：单位0.1度，-1代表未知

    int32_t longitude; // 经度：32位有符号整数，单位：度 × (2^31 / 180)
    int32_t latitude;  // 纬度：32位有符号整数，单位：度 × (2^31 / 180)

    uint16_t distance;   // 相对距离：单位0.1米
    uint16_t absAzimuth; // 绝对方位：单位0.1度
    uint16_t relAzimuth; // 相对方位：单位0.1度

    uint8_t sign;   // 属性：0~6（敌/我/友等）
    uint8_t type;   // 类型：0未知，1船只，2浮标
    uint8_t color;  // 颜色：0未知，1红，2黄...
    uint8_t status; // 状态：0~8

    uint16_t threatingRadius; // 威胁半径：单位0.1米

    char code[16]; // 船舶识别舷号，最长16字节字符串（0为无效）
    uint8_t model; // 型号（用于区分我方船）

    char name[16]; // 船舶名称（最长16字节）

    uint16_t shipLength; // 船长：单位0.1米
    uint16_t shipWidth;  // 船宽：单位0.1米
    uint16_t shipHeight; // 船高：单位0.1米

    uint32_t MMSI; // 船舶识别号（MMSI），0为无效
};
struct VesselFusionTargetsMsg
{
    uint16_t count;
    std::vector<VesselFusionTarget> targets; // 目标数组
};
nlohmann::json vesselFusionTargetsMsgToJson(const VesselFusionTargetsMsg &msg);
// 目标信息修订协议 0151
struct TargetRevision
{
    uint16_t batch;           // 批号：唯一目标编号
    uint8_t type;             // 类型：0未知，1船只，2浮标
    uint8_t color;            // 颜色：0未知，1红，2黄...
    uint16_t threatingRadius; // 威胁半径：单位0.1米
    char code[16];            // 船舶识别舷号，0为无效
};

// Basic Load Protocol for Single Vessels

// 单艇导航协议 0411
struct VesselNavigation
{
    int32_t longitude;  // 经度，单位：度 × (2^31 / 180)
    int32_t latitude;   // 纬度，单位：度 × (2^31 / 180)
    uint16_t height;    // 高度（深度），单位：0.1m
    int16_t speed;      // 绝对航速，单位：0.1kn
    int16_t course;     // 绝对航向，单位：0.01度
    int16_t heading;    // 船首向，单位：0.01度
    int16_t headingAcc; // 船首向加速度，单位：0.01度/s^2

    int16_t eastSpeed;     // 东向速度，单位：0.1kn，允许负值
    int16_t northSpeed;    // 北向速度，单位：0.1kn，允许负值
    int16_t verticalSpeed; // 垂直速度，单位：0.1kn，允许负值

    int16_t pitch;   // 纵摇，单位：度 × (2^15 / 180)
    int16_t rolling; // 横摇，单位：度 × (2^15 / 180)

    int16_t angularX; // 姿态角速度（X），单位：0.01°/s
    int16_t angularY; // 姿态角速度（Y），单位：0.01°/s
    int16_t angularZ; // 姿态角速度（Z），单位：0.01°/s

    int16_t accX; // X向加速度，单位：0.005 m/s²
    int16_t accY; // Y向加速度，单位：0.005 m/s²
    int16_t accZ; // Z向加速度，单位：0.005 m/s²

    uint8_t deviceState; // 导航设备状态：1-正常，2-卫导丢信号，3-无罗经信号，4-无信号
};

// AIS协议 0421

struct AISMsg
{
    uint16_t abs_speed;  // 目标绝对航速，单位：0.1节
    uint16_t abs_course; // 目标绝对航向，单位：0.1度
    int16_t heading;     // 船首向，单位：0.1度，-1 表示未知

    int32_t longitude; // 目标经度，单位：度 × (2^31 / 180)
    int32_t latitude;  // 目标纬度，单位：度 × (2^31 / 180)

    uint8_t type; // 类型：0=不明，1=船只，2=浮标

    char name[16]; // 船名（定长16字节，0表示不明）

    uint16_t ship_length; // 船长，单位：0.1米，0表示不明
    uint16_t ship_width;  // 船宽，单位：0.1米，0表示不明
    uint16_t ship_height; // 船高，单位：0.1米，0表示不明

    uint32_t MMSI; // 船只识别号，0 表示不明
};

// 集群平台类协议

// 导航共享协议0A11
struct NavigationShare
{
    int32_t longitude; // 经度，单位：度 × (2^31 / 180)，东正西负
    int32_t latitude;  // 纬度，单位：度 × (2^31 / 180)，北正南负

    uint16_t height; // 深度，单位：0.1m，全1 (0xFFFF) 表示不明

    int16_t abs_speed;   // 绝对航速，单位：0.1节，负数表示反向（范围：-200~2000）
    uint16_t abs_course; // 绝对航向，单位：0.1度，范围：0~3600
    uint16_t heading;    // 艏向角，单位：0.1度，范围：0~3600
};

// 人工增加目标协议0152

struct ManualTarget
{
    int id;           // targetId
    float absSpeed;   // 单位：节
    float absCourse;  // 单位：度
    double longitude; // 单位：度
    double latitude;  // 单位：度
    uint8_t type;     // 0不明，1船只，2浮标
    std::chrono::steady_clock::time_point lastUpdateTime;
};

// 集群态势协议

// 集群融合目标信息 0B11

struct Target0B11
{
    uint16_t batch;             // 批号 [2 bytes]
    uint16_t absSpeed;          // 绝对航速 0.1节 [2 bytes]
    uint16_t absCourse;         // 绝对航向 0.1度 [2 bytes]
    int16_t heading;            // 艏向 0.1度，有符号，-1代表未知 [2 bytes]
    int32_t longitude;          // 经度，单位：180/2^31 度 [4 bytes]
    int32_t latitude;           // 纬度，单位：180/2^31 度 [4 bytes]
    uint16_t distance;          // 相对距离 0.1米 [2 bytes]
    uint16_t absAzimuth;        // 绝对方位 0.1度 [2 bytes]
    uint16_t relAzimuth;        // 相对方位 0.1度 [2 bytes]
    uint8_t sign;               // 属性 [1 byte]
    uint8_t type;               // 类型 [1 byte]
    uint8_t color;              // 颜色 [1 byte]
    uint8_t status;             // 状态 [1 byte]
    uint16_t threateningRadius; // 威胁半径 0.1米 [2 bytes]
    char code[16];              // 船舶识别舷号 [16 bytes]
    uint16_t model;             // 型号 [2 bytes]
    char name[16];              // 船舶名称 [16 bytes]
    uint16_t shipLength;        // 船长 0.1米 [2 bytes]
    uint16_t shipWidth;         // 船宽 0.1米 [2 bytes]
    uint16_t shipHeight;        // 船高 0.1米 [2 bytes]
    uint32_t MMSI;              // 识别号 [4 bytes]
};

struct ClusterFusionTargetInfo
{
    uint16_t count;                  // 数目 [2 bytes]
    std::vector<Target0B11> targets; // 动态目标列表
};

// 目标批号对照表 0B12

struct TargetSourceMapping
{
    uint16_t targetSourcePlatformId;       // 平台编码
    std::vector<uint16_t> targetSourceIds; // 本平台上与fusedId一一对应的原始目标批号（0表示无映射）
};

struct TargetBatchNumberTable
{
    uint16_t targetNumber;         // 集群融合目标数量 n
    std::vector<uint16_t> fusedId; // 集群融合目标批号列表 [n * 2 bytes]

    uint8_t platformNumber;                        // 平台节点数量 m
    std::vector<TargetSourceMapping> idComparison; // m组节点对应信息
};

#endif // PROTOCOL_H_
