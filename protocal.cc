#include "protocal.h"
#include <nlohmann/json.hpp>
#include <unordered_map>
#include <unordered_set>
std::string getCurrentTimeString()
{
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %H:%M:%S");
    ss << "." << std::setfill('0') << std::setw(3) << ms.count();
    return ss.str();
}
RadarTargetsMsg jsonToRadarTargets(const nlohmann::json &input)
{
    RadarTargetsMsg msg;
    msg.count = static_cast<uint16_t>(input["content"].value("count", 0));

    for (const auto &t : input["content"]["targets"])
    {
        RadarTarget target;

        target.batch = static_cast<uint16_t>(t.value("batch", 0));

        // 按0.1单位乘以10并四舍五入到整数
        target.absAzimuth = static_cast<uint16_t>(std::round(t.value("absAzimuth", 0.0) * 10));
        target.distance = static_cast<uint32_t>(std::round(t.value("distance", 0.0) * 10));
        target.absSpeed = static_cast<uint16_t>(std::round(t.value("absSpeed", 0.0) * 10));
        target.absCourse = static_cast<uint16_t>(std::round(t.value("absCourse", 0.0) * 10));

        // 经纬度乘比例并截断为int32_t
        double lon = t.value("longitude", 0.0);
        double lat = t.value("latitude", 0.0);
        target.longitude = static_cast<int32_t>(lon * DEG_TO_INT);
        target.latitude = static_cast<int32_t>(lat * DEG_TO_INT);

        target.state = static_cast<uint8_t>(t.value("state", 0));

        msg.targets.push_back(target);
    }

    return msg;
}

// 0131
LidarTargetsMsg jsonToLidarTargets(const nlohmann::json &input)
{
    LidarTargetsMsg msg;
    msg.count = input["content"].value("count", 0);

    for (const auto &t : input["content"]["targets"])
    {
        LidarTarget target;

        target.batch = t.value("batch", 0);
        target.absAzimuth = static_cast<uint16_t>(t.value("absAzimuth", 0.0) * 10.0); // 0.1度单位
        target.distance = static_cast<uint32_t>(t.value("distance", 0.0) * 10.0);     // 0.1米单位
        target.absSpeed = static_cast<uint16_t>(t.value("absSpeed", 0.0) * 10.0);     // 0.1节单位
        target.absCourse = static_cast<uint16_t>(t.value("absCourse", 0.0) * 10.0);   // 0.1度单位

        double heading_raw = t.value("heading", -1.0);
        if (heading_raw < 0)
            target.heading = INVALID_LENGTH_WIDTH; // 无效用65535代替
        else
            target.heading = static_cast<uint16_t>(heading_raw * 10.0); // 0.1度单位

        // 经纬度映射为整数，直接乘系数，截断
        double lon = t.value("longitude", 0.0);
        double lat = t.value("latitude", 0.0);
        target.longitude = static_cast<int32_t>(lon * LONLAT_SCALE);
        target.latitude = static_cast<int32_t>(lat * LONLAT_SCALE);

        target.state = static_cast<uint8_t>(t.value("state", 0));

        // 长度、宽度，-1无效
        double length_raw = t.value("length", -1.0);
        if (length_raw < 0)
            target.length = INVALID_LENGTH_WIDTH;
        else
            target.length = static_cast<uint16_t>(length_raw * 10.0);

        double width_raw = t.value("width", -1.0);
        if (width_raw < 0)
            target.width = INVALID_LENGTH_WIDTH;
        else
            target.width = static_cast<uint16_t>(width_raw * 10.0);

        msg.targets.push_back(target);
    }

    return msg;
}

// 0141

nlohmann::json vesselFusionTargetsMsgToJson(const VesselFusionTargetsMsg &msg)
{
    nlohmann::json output;

    // head
    output["head"] = {{"packageSeq", 0}, {"packageType", 0}, {"time", getCurrentTimeString().substr(11)}};

    // content
    nlohmann::json content;
    content["count"] = msg.count;
    content["targets"] = nlohmann::json::array();

    for (const auto &t : msg.targets)
    {
        nlohmann::json target_json;

        target_json["batch"] = t.batch;
        target_json["absSpeed"] = t.absSpeed;
        target_json["absCourse"] = t.absCourse;
        target_json["heading"] = t.heading;
        target_json["longitude"] = t.longitude;
        target_json["latitude"] = t.latitude;
        target_json["distance"] = t.distance;
        target_json["absAzimuth"] = t.absAzimuth;
        target_json["relAzimuth"] = t.relAzimuth;
        target_json["sign"] = t.sign;
        target_json["type"] = t.type;
        target_json["color"] = t.color;
        target_json["status"] = t.status;
        target_json["threatingRadius"] = t.threatingRadius;

        // 字符串字段需要从char数组转换成string，注意去掉结尾多余的0
        target_json["code"] = std::string(t.code, strnlen(t.code, sizeof(t.code)));
        target_json["model"] = t.model;
        target_json["name"] = std::string(t.name, strnlen(t.name, sizeof(t.name)));

        target_json["shipLength"] = t.shipLength;
        target_json["shipWidth"] = t.shipWidth;
        target_json["shipHeight"] = t.shipHeight;
        target_json["MMSI"] = t.MMSI;

        content["targets"].push_back(target_json);
    }

    output["content"] = content;

    return output;
}
