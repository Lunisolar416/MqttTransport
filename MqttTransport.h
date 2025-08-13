#ifndef MQTTSUBSCRIBER_H
#define MQTTSUBSCRIBER_H

#include "GeoUtils.h"
#include "polygon_point_singleton.h"
#include "protocal.h"
#include <Eigen/Dense>
#include <GeographicLib/LocalCartesian.hpp>
#include <cstdint>
#include <functional>
#include <mqtt/async_client.h>
#include <mqtt/callback.h>
#include <mqtt/client.h>
#include <mqtt/connect_options.h>
#include <mqtt/token.h>
#include <nlohmann/json.hpp>
#include <sys/types.h>
#include <unordered_map>
#include <unordered_set>
// 处理融合结果
struct TargetCache
{
    nlohmann::json part0;
    nlohmann::json part1;
    nlohmann::json part2;
    std::chrono::steady_clock::time_point last_update;
    bool has0 = false;
    bool has1 = false;
    bool has2 = false;
};
struct ClusterTargetCache
{
    nlohmann::json part0;
    nlohmann::json part1;
    nlohmann::json part2;
    std::chrono::steady_clock::time_point last_update;
    bool has0 = false;
    bool has1 = false;
    bool has2 = false;
};

struct Point
{
    double lat; // 纬度
    double lon; // 经度
    Point(double lat, double lon) : lat(lat), lon(lon)
    {
    }
};
struct Area
{
    int areasFuncId;
    int polygonNumber;
    std::vector<Point> polygon;

    std::vector<std::pair<double, double>> polygonXY;
    GeographicLib::LocalCartesian proj;
    double minLat, maxLat, minLon, maxLon;

    Area() : proj(0, 0, 0), minLat(0), maxLat(0), minLon(0), maxLon(0)
    {
    }
};
// 这个类就作为订阅者和发布者
// 订阅外部消息，再该类中转化成新的协议并发到usfs的brige的mqttsubscriber中
class MqttTransport : public virtual mqtt::callback, public virtual mqtt::iaction_listener
{
  public:
    MqttTransport(const std::string &subclientaddress, const std::string &pubclientaddress,
                  const std::string &subclient_id, const std::string pubclient_id);
    ~MqttTransport();

    bool connect();
    void subscribe(const std::vector<std::string> &topics, const std::vector<int> &qos);
    void publish(const std::string &topic, const nlohmann::json &json_msg);
    void getNewMessage();
    void message_arrived(mqtt::const_message_ptr msg) override;

    void on_success(const mqtt::token &tok) override;
    void on_failure(const mqtt::token &tok) override;

  private:
    std::string getCurrentTimeString();
    void loadConfig();
    // 处理接收到的消息
    // 接受0041协议也就是水域类型上报协议，确定水域类型
    void handle_0041(const nlohmann::json &json_msg, const std::string &topic);
    // 区域过滤
    void handle_0042(const nlohmann::json &json_msg, const std::string &topic);
    // 接受0051协议也就是组件状态协议，并以1HZ上报自身状态
    void handle_0051();
    void heartbeatsLoop();

    // 接受导航雷达目标协议0111
    void handle_0111(const nlohmann::json &json_msg, const std::string &topic);
    // 接受AIS协议0421
    void handle_0421(const nlohmann::json &json_msg, const std::string &topic);
    // 接受激光雷达目标协议0131
    void handle_0131(const nlohmann::json &json_msg, const std::string &topic);
    // 接受目标识别协议0121
    void handle_0121(const nlohmann::json &json_msg, const std::string &topic);
    // 接受目标信息修订协议0151，然后可以修改指定批号目标的类型等等
    void handle_0151(const nlohmann::json &json_msg, const std::string &topic);
    // 接受导航共享协议0A11，接收其他个平台发来的位置信息
    void handle_0A11(const nlohmann::json &json_msg, const std::string &topic);
    // 接收单艇导航协议0411，获取本艇的位置姿态信息
    void handle_0411(const nlohmann::json &json_msg, const std::string &topic);
    // 接收单艇融合目标信息0141
    void handle_0141(const nlohmann::json &json_msg, const std::string &topic);
    // 人工增加目标协议0152
    void handle_0152(const nlohmann::json &json_msg, const std::string &topic);

    void handle_0E20H0(const nlohmann::json &json_msg, const std::string &topic);
    void handle_0E20H1(const nlohmann::json &json_msg, const std::string &topic);
    void handle_0E20H2(const nlohmann::json &json_msg, const std::string &topic);

    void handle_underwater_motion(const nlohmann::json &json_msg, const std::string &topic);
    void handle_underwater_identification(const nlohmann::json &json_msg, const std::string &topic);
    void handle_underwater_name(const nlohmann::json &json_msg, const std::string &topic);

    // 融合
    void tryMerge();
    nlohmann::json filterTargetFromPart(const nlohmann::json &part, int targetId);
    void mergeAndOutput(const std::vector<int> &targetIds);
    // 清理过期或者不完整的融合
    void cleanupExpiredCaches();

    // 集群融合
    void handle_0220H0(const nlohmann::json &json_msg, const std::string &topic);
    void handle_0220H1(const nlohmann::json &json_msg, const std::string &topic);
    void handle_0220H2(const nlohmann::json &json_msg, const std::string &topic);

    void cleanupExpiredClusterCaches();
    void tryClusterMerge();
    void clustermergeAndOutput(const std::vector<int> &targetIds);

    // distance

    // // position
    // bool isPointInPolygonXY(double x, double y, const std::vector<std::pair<double, double>> &polygonXY);
    // bool isPointInPolygonGeo(const Point &pt, const std::vector<Point> &polygon);
    // bool isInArea(const Point &pt);
    // double distancePointToSegment(double px, double py, double x1, double y1, double x2, double y2);
    // // bool isPointInPolygonGeoBuffered(const Point& pt,const std::vector<Point>& polygon,double toleranceMeters);
    // bool isPointInPolygonGeoBuffered(const Point &pt, const Area &area, double toleranceMeters);

    // bool isInAreaBuffered(const Point &pt, double toleranceMeters);

    // void prepareAreaCache();

  private:
    // subscribe to receive outenv data
    mqtt::client client_;             // subscribe client
    mqtt::connect_options conn_opts_; // client options

    // publish to send data
    mqtt::async_client pub_client_;

    // subscibe to receive result
    mqtt::client sub_client_;
    // publisher to send result
    mqtt::async_client pubR_client_;
    // publisher to send origin result
    mqtt::async_client pubOr_client_;
    // 处理协议
    std::unordered_map<std::string, std::function<void(const nlohmann::json &, const std::string)>> topic_handlers_;
    // 单艇融合
    std::unordered_map<uint16_t, TargetCache> target_cache;
    // 集群融合
    std::unordered_map<uint16_t, ClusterTargetCache> cluster_target_cache;
    // 0041
    bool is_simple_waterarea = false;

    // 0121
    std::unordered_map<int, TargetRecognition> target_recongnition_;

    // 0151
    std::unordered_map<uint16_t, TargetRevision> target_revisions_;

    // 0152
    std::unordered_map<uint16_t, ManualTarget> manual_targets_;

    // 0051
    std::unordered_map<uint16_t, std::chrono::steady_clock::time_point> sensor_last_recv_time_;

    // Longitude
    double self_longitude_;
    // Latitude
    double self_latitude_;
    // heading
    double self_heading_;

    // Course
    std::unordered_map<uint16_t, double> courses_;
};

#endif // MQTTSUBSCRIBER_H