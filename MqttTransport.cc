#include "MqttTransport.h"
#include "protocal.h"
#include <exception>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <utility>
#include <yaml-cpp/yaml.h>
MqttTransport::MqttTransport(const std::string &subclientaddress, const std::string &pubclientaddress,
                             const std::string &subclient_id, const std::string pubclient_id)
    : client_(subclientaddress, subclient_id), pub_client_(pubclientaddress, pubclient_id),
      sub_client_(pubclientaddress, "result"), pubR_client_(subclientaddress, "sendRes"),
      pubOr_client_(subclientaddress, "sendOres")
{
    loadConfig();
    // heartbeatsLoop();
    conn_opts_.set_clean_session(true);

    // setting callback
    pub_client_.set_callback(*this);
    client_.set_callback(*this);
    sub_client_.set_callback(*this);
    pubR_client_.set_callback(*this);
    pubOr_client_.set_callback(*this);
}

MqttTransport::~MqttTransport()
{
    try
    {
        client_.disconnect();
        pub_client_.disconnect();
        sub_client_.disconnect();
        pubR_client_.disconnect();
        pubOr_client_.disconnect();
    }
    catch (const mqtt::exception &e)
    {
        std::cerr << "Disconnection failed: " << e.what() << std::endl;
    }
}

bool MqttTransport::connect()
{
    try
    {
        pub_client_.connect(conn_opts_, nullptr, *this);
        pubR_client_.connect(conn_opts_, nullptr, *this);
        client_.connect(conn_opts_);
        sub_client_.connect(conn_opts_);
        pubOr_client_.connect(conn_opts_);
        return true;
    }
    catch (const mqtt::exception &e)
    {
        std::cerr << "Connection failed: " << e.what() << std::endl;
        return false;
    }
}

void MqttTransport::subscribe(const std::vector<std::string> &topics, const std::vector<int> &qos)
{
    try
    {
        for (auto &t : topics)
        {
            std::cout << "topic " << t << std::endl;
        }
        client_.subscribe(topics, qos);
        sub_client_.subscribe({"0E20H-0", "0E20H-1", "0E20H-2", "0220H-0", "0220H-1", "0220H-2"}, {0, 0, 0, 0, 0, 0});
    }
    catch (const mqtt::exception &e)
    {
        std::cerr << "Subscription failed: " << e.what() << std::endl;
    }
}
void MqttTransport::getNewMessage()
{
    try
    {
        client_.consume_message(); // 阻塞等待新消息,触发回调函数
        sub_client_.consume_message();
    }
    catch (const mqtt::exception &e)
    {
        std::cerr << "can't consume message: " << e.what() << std::endl;
    }
}

void MqttTransport::message_arrived(mqtt::const_message_ptr msg)
{
    auto topic = msg->get_topic();
    auto payload = msg->get_payload();
    nlohmann::json json_data = nlohmann::json::parse(payload);
    // std::cout << "Received message on topic " << topic << ": " << json_data.dump(4) << std::endl;
    auto it = topic_handlers_.find(topic);
    if (it != topic_handlers_.end())
        it->second(json_data, topic);
    else
        std::cerr << "No handler for topic: " << topic << std::endl;
}

void MqttTransport::on_success(const mqtt::token &tok)
{
#ifdef DEBUG
    if (tok.get_type() == mqtt::token::Type::CONNECT)
        std::cout << "[MQTT] pub_client_ connected successfully\n";
    else
        std::cout << "[MQTT] publish success\n";
#endif
}

void MqttTransport::on_failure(const mqtt::token &tok)
{
#ifdef DEBUG
    std::cout << "[MQTT] publish failure " << std::endl;
#endif
}

void MqttTransport::loadConfig()
{
    YAML::Node config = YAML::LoadFile("szo.yml");

    std::cout << "\n[MQTT Topics from driver.mqtt_topics]:\n";
    auto driver_topics = config["usfs_bridge"]["driver"]["mqtt_topics"];
    auto reporter_topic_root = config["usfs_bridge"]["reporter"]["object_status"]["mqtt_topics"];
    auto reporter_topic = reporter_topic_root["fused"];

    // init other
    topic_handlers_["0041"] =
        std::bind(&MqttTransport::handle_0041, this, std::placeholders::_1, std::placeholders::_2);
    topic_handlers_["0121"] =
        std::bind(&MqttTransport::handle_0121, this, std::placeholders::_1, std::placeholders::_2);
    topic_handlers_["0151"] =
        std::bind(&MqttTransport::handle_0151, this, std::placeholders::_1, std::placeholders::_2);
    topic_handlers_["0152"] =
        std::bind(&MqttTransport::handle_0152, this, std::placeholders::_1, std::placeholders::_2);
    topic_handlers_["0411"] =
        std::bind(&MqttTransport::handle_0411, this, std::placeholders::_1, std::placeholders::_2);
    topic_handlers_["0A11"] =
        std::bind(&MqttTransport::handle_0A11, this, std::placeholders::_1, std::placeholders::_2);

    if (driver_topics)
    {
        for (const auto &item : driver_topics)
        {
            std::string key = item.first.as<std::string>();
            const auto &val = item.second;

            if (val.IsScalar())
            {
                std::string topic = val.as<std::string>();
                std::cout << "  " << key << ": " << topic << std::endl;

                if (key == "vessel_site_info")
                    topic_handlers_[topic] =
                        std::bind(&MqttTransport::handle_0411, this, std::placeholders::_1, std::placeholders::_2);
                else if (key == "nav_radar")
                    topic_handlers_[topic] =
                        std::bind(&MqttTransport::handle_0111, this, std::placeholders::_1, std::placeholders::_2);
                else if (key == "ais_moving_target")
                    topic_handlers_[topic] =
                        std::bind(&MqttTransport::handle_0421, this, std::placeholders::_1, std::placeholders::_2);
                else if (key == "laser_radar")
                    topic_handlers_[topic] =
                        std::bind(&MqttTransport::handle_0131, this, std::placeholders::_1, std::placeholders::_2);
            }
            else if (val.IsSequence()) // single_vessel_underwater, single_vessel_surface
            {
                std::cout << "  " << key << " (sequence):\n";
                for (const auto &sub : val)
                {
                    if (!sub["name"] || !sub["topic"])
                        continue;

                    std::string name = sub["name"].as<std::string>();
                    std::string topic = sub["topic"].as<std::string>();
                    int id = sub["id"] ? sub["id"].as<int>() : 0;
                    int extent_id = sub["extent_id"] ? sub["extent_id"].as<int>() : 0;

                    std::cout << "    - " << name << " | topic: " << topic << " | id: " << id
                              << " | extent_id: " << extent_id << std::endl;

                    if (name == "single_vessel_surface_motion_attributes")
                        topic_handlers_[topic] = std::bind(&MqttTransport::handle_0E20H0, this, std::placeholders::_1,
                                                           std::placeholders::_2);
                    else if (name == "single_vessel_surface_identification")
                        topic_handlers_[topic] = std::bind(&MqttTransport::handle_0E20H1, this, std::placeholders::_1,
                                                           std::placeholders::_2);
                    else if (name == "single_vessel_surface_name")
                        topic_handlers_[topic] = std::bind(&MqttTransport::handle_0E20H2, this, std::placeholders::_1,
                                                           std::placeholders::_2);
                    else if (name == "single_vessel_underwater_motion_attributes")
                        topic_handlers_[topic] = std::bind(&MqttTransport::handle_underwater_motion, this,
                                                           std::placeholders::_1, std::placeholders::_2);
                    else if (name == "single_vessel_underwater_identification")
                        topic_handlers_[topic] = std::bind(&MqttTransport::handle_underwater_identification, this,
                                                           std::placeholders::_1, std::placeholders::_2);
                    else if (name == "single_vessel_underwater_name")
                        topic_handlers_[topic] = std::bind(&MqttTransport::handle_underwater_name, this,
                                                           std::placeholders::_1, std::placeholders::_2);
                }
            }
        }
    }
    if (reporter_topic && reporter_topic.IsSequence())
    {
        std::cout << "\n[MQTT Topics from reporter.object_status.mqtt_topics.fused]:\n";
        for (const auto &item : reporter_topic)
        {
            if (!item["name"] || !item["topic"])
                continue;

            std::string name = item["name"].as<std::string>();
            std::string topic = item["topic"].as<std::string>();
            int id = item["id"] ? item["id"].as<int>() : 0;
            int extent_id = item["extent_id"] ? item["extent_id"].as<int>() : 0;

            std::cout << "  - " << name << " | topic: " << topic << " | id: " << id << " | extent_id: " << extent_id
                      << std::endl;

            if (name == "cluster_surface_motion_attributes")
                topic_handlers_[topic] =
                    std::bind(&MqttTransport::handle_0220H0, this, std::placeholders::_1, std::placeholders::_2);
            else if (name == "cluster_surface_identification")
                topic_handlers_[topic] =
                    std::bind(&MqttTransport::handle_0220H1, this, std::placeholders::_1, std::placeholders::_2);
            else if (name == "cluster_surface_name")
                topic_handlers_[topic] =
                    std::bind(&MqttTransport::handle_0220H2, this, std::placeholders::_1, std::placeholders::_2);
            //
        }
    }
    else
    {
        std::cerr << "[ERROR] mqtt_topics.fused is missing or not a sequence\n";
    }
}

void MqttTransport::handle_0041(const nlohmann::json &json_msg, const std::string &topic)
{
    const auto &content = json_msg.at("content");

    uint8_t type = content.value("type", 0);

    if (type == 1)
        is_simple_waterarea = true;
    else
        is_simple_waterarea = false;
}
// 组件协议
void MqttTransport::handle_0051()
{
    nlohmann::json msg;
    msg["content"]["Id"] = COMPONENT_THIS;
    msg["content"]["state"] = 1; // 默认正常
    msg["content"]["info"] = nlohmann::json::array();
    int error_count = 0;

    auto now = std::chrono::steady_clock::now();

    auto check_timeout = [&](uint16_t comp_id, SensorErrorCode err_code) {
        auto it = sensor_last_recv_time_.find(comp_id);
        if (it == sensor_last_recv_time_.end() ||
            std::chrono::duration_cast<std::chrono::milliseconds>(now - it->second).count() > 2000)
        {
            msg["content"]["info"].push_back({{"errorCode", err_code}});
            ++error_count;
        }
    };

    check_timeout(COMPONENT_0421, ERROR_0411_AIS_TIMEOUT);
    check_timeout(COMPONENT_0111, ERROR_0111_RADAR_TIMEOUT);
    check_timeout(COMPONENT_0131, ERROR_0131_LIDAR_TIMEOUT);

    msg["content"]["count"] = error_count;
    if (error_count > 0)
    {
        msg["content"]["state"] = 0; // 异常
    }

    mqtt::message_ptr pmsg = mqtt::make_message("0051", msg.dump());
    pubR_client_.publish(pmsg, nullptr, *this);
}
void MqttTransport::heartbeatsLoop()
{
    std::thread([this]() {
        while (true)
        {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            this->handle_0051();
        }
    }).detach();
}

// 导航雷达
void MqttTransport::handle_0111(const nlohmann::json &json_msg, const std::string &topic)
{

    sensor_last_recv_time_[COMPONENT_0111] = std::chrono::steady_clock::now();
    try
    {
        const auto &head_in = json_msg.at("head");
        const auto &content_in = json_msg.at("content");
        const auto &targets_in = content_in.at("targets");

        nlohmann::json output;

        // ========== head ==========
        output["head"] = {{"packageSerialNumber", head_in.value("packageSeq", 0)},
                          {"packageId", 0},
                          {"packageConfirmNumber", 0},
                          {"sendIP", head_in.value("sendIP", "")},
                          {"destIP", head_in.value("destIP", "")}};

        // ========== content[0] ==========
        nlohmann::json unit;

        unit["infoUnitHead"] = {{"destPlatformId", 1793},
                                {"infoSourceTypeId", 0},
                                {"infoUnitCreateTime", getCurrentTimeString().substr(11)}, // "HH:MM:SS.xxx"
                                {"infoUnitExtentId", 0},
                                {"infoUnitID", 0x03},
                                {"infoUnitLength", 0},
                                {"secondInfoUnitId", 3619},
                                {"sourcePlatformId", 3073}};

        nlohmann::json content_out;
        content_out["targetNumber"] = content_in.value("count", 0);
        content_out["targets"] = nlohmann::json::array();

        for (const auto &t : targets_in)
        {
            nlohmann::json tgt;

            tgt["targetId"] = t.value("batch", 0);
            tgt["coordinateSystemTaype"] = 0;

            // 单位换算和类型转换?
            tgt["targetDirection"] = t.value("absAzimuth", 0.0);
            tgt["targetDistance"] = t.value("distance", 0.0);
            tgt["targetAbsoluteSpeed"] = static_cast<float>(t.value("absSpeed", 0));
            tgt["targetAbsoluteDirection"] = t.value("absCourse", 0);
            tgt["targetLongitude"] = static_cast<float>(t.value("longitude", 0.0));
            tgt["targetLatitude"] = static_cast<float>(t.value("latitude", 0.0));
            tgt["targetState"] = t.value("state", 0);

            // 其他字段默认值
            tgt["targetCreateTime"] = 0;
            tgt["targetDcpa"] = 0;
            tgt["targetEdgeLength"] = 0;
            tgt["targetEdgeWidth"] = 0;
            tgt["targetHeight"] = 0;
            tgt["targetRelativeDirection"] = 0;
            tgt["targetRelativeSpeed"] = 0;
            tgt["targetSimulateSign"] = 0;
            tgt["targetSourcePlatformId"] = 3073;
            tgt["targetTcpa"] = 0;
            tgt["trackQualityNumber"] = 0;

            content_out["targets"].push_back(tgt);
        }

        unit["infoUnitContent"] = content_out;
        output["content"] = nlohmann::json::array({unit});

        publish("0E23H-0", output);
        // mqtt::message_ptr msg = mqtt::make_message("0E23H-0", output.dump());
        // pubOr_client_.publish(msg, nullptr, *this);
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error in handle_0111: " << e.what() << std::endl;
    }
}
// 接受AIS协议0421
void MqttTransport::handle_0421(const nlohmann::json &json_msg, const std::string &topic)
{
    sensor_last_recv_time_[COMPONENT_0421] = std::chrono::steady_clock::now();
    nlohmann::json output;
    try
    {
        // 构造 head
        const auto &head_in = json_msg.at("head");
        output["head"]["destIP"] = "";
        output["head"]["sendIP"] = "";
        output["head"]["packageSerialNumber"] = head_in.value("packageSeq", 0);
        output["head"]["packageConfirmNumber"] = 0;
        output["head"]["packageId"] = 0;

        // 构造 content
        const auto &content_in = json_msg.at("content");
        nlohmann::json unit;

        nlohmann::json infoUnitContent;
        infoUnitContent["COG"] = content_in.value("absCourse", 0.0);
        infoUnitContent["SOG"] = content_in.value("absSpeed", 0.0);
        infoUnitContent["ShipHead"] = content_in.value("heading", 511);
        infoUnitContent["Longitude"] = content_in.value("longitude", 0.0);
        infoUnitContent["Latitude"] = content_in.value("latitude", 0.0);
        infoUnitContent["MMSI"] = content_in.value("MMSI", 0);
        infoUnitContent["aisType"] = content_in.value("type", -1);

        // 设置默认字段
        infoUnitContent["ForwardFlag"] = 0;
        infoUnitContent["NavStatus"] = 0;
        infoUnitContent["PositionPrecision"] = 1;
        infoUnitContent["ROT"] = 128;
        infoUnitContent["TimeMark"] = 30;

        unit["infoUnitContent"] = infoUnitContent;

        // 构造 infoUnitHead（固定值或零）
        nlohmann::json infoUnitHead;
        infoUnitHead["destPlatformId"] = 1739;
        infoUnitHead["sourcePlatformId"] = 3073;
        infoUnitHead["infoUnitCreateTime"] = getCurrentTimeString().substr(11); // eg. "11:31:36.300"
        infoUnitHead["infoUnitExtentId"] = 1;
        infoUnitHead["infoUnitID"] = 0x03;
        infoUnitHead["infoUnitLength"] = 0;
        infoUnitHead["secondInfoUnitId"] = 3632;
        infoUnitHead["infoSourceTypeId"] = 3073;

        unit["infoUnitHead"] = infoUnitHead;

        output["content"] = nlohmann::json::array({unit});

        publish("0E30H-1", output);
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
    }
}
// 接受激光雷达目标协议0131
void MqttTransport::handle_0131(const nlohmann::json &json_msg, const std::string &topic)
{
    sensor_last_recv_time_[COMPONENT_0131] = std::chrono::steady_clock::now();
    nlohmann::json output;
    try
    {
        // head
        const auto &head_in = json_msg.at("head");
        output["head"] = {{"packageSerialNumber", head_in.value("packageSeq", 0)},
                          {"packageId", 0},
                          {"packageConfirmNumber", 0},
                          {"sendIP", ""},
                          {"destIP", ""}};
        nlohmann::json info_unit;
        nlohmann::json content_out;
        const auto &content_in = json_msg.at("content");
        const auto &targets = content_in.at("targets");

        content_out["targetNumber"] = static_cast<int>(targets.size());
        content_out["targets"] = nlohmann::json::array();

        for (const auto &t : targets)
        {
            nlohmann::json target;

            target["targetId"] = t.value("batch", 0);
            std::cout << "targetId: " << target["targetId"] << std::endl;
            target["targetAbsoluteDirection"] = t.value("absAzimuth", 0.0);
            target["targetDistance"] = t.value("distance", 0.0);
            target["targetAbsoluteSpeed"] = t.value("absSpeed", 0.0);
            target["targetDirection"] = t.value("absCourse", 0.0);
            target["targetRelativeDirection"] = t.value("heading", 0.0);
            target["targetLatitude"] = t.value("latitude", 0.0);
            target["targetLongitude"] = t.value("longitude", 0.0);
            target["targetState"] = t.value("state", 0);
            target["targetEdgeLength"] = t.value("length", 0.0);
            target["targetEdgeWidth"] = t.value("width", 0.0);

            // 缺失字段
            target["targetCreateTime"] = 0;
            target["targetRelativeSpeed"] = 0;
            target["targetSimulateSign"] = 0;
            target["targetSourcePlatformId"] = 3073;
            target["targetHeight"] = 0;
            target["coordinateSystemTaype"] = 0;

            content_out["targets"].push_back(target);
        }

        info_unit["infoUnitContent"] = content_out;

        // infoUnitHead 置零
        info_unit["infoUnitHead"] = {
            {"infoUnitID", 0x03},       {"secondInfoUnitId", 3621},
            {"sourcePlatformId", 3073}, {"destPlatformId", 1793},
            {"infoUnitLength", 0},      {"infoUnitExtentId", 2},
            {"infoSourceTypeId", 0},    {"infoUnitCreateTime", getCurrentTimeString().substr(11)} // 仅保留 HH:mm:ss.sss
        };

        output["content"] = nlohmann::json::array({info_unit});

        // 如果是开阔水域将不再上报激光雷达信息
        // if (is_simple_waterarea == false)
        // {
        //     publish("0E25H-2", output);
        // }

        publish("0E25H-2", output);
    }
    catch (const std::exception &e)
    {
        std::cout << "Error: " << e.what() << std::endl;
    }
}
// 接受目标识别协议0121 光电相关
void MqttTransport::handle_0121(const nlohmann::json &json_msg, const std::string &topic)
{
    // std::cout<<"[0121] "<<json_msg.dump()<<std::endl;
    try
    {
        TargetRecognition targetRec_;
        const auto &content = json_msg.at("content");
        targetRec_.batch = content.value("batch", 0.0);
        targetRec_.sign = content.value("sign", 0);
        targetRec_.source = content.value("source", 0);
        targetRec_.type = content.value("type", 0.0);
        {
            std::string code_str = content.value("code", "");
            std::strncpy(targetRec_.code, code_str.c_str(), sizeof(targetRec_.code) - 1);
            targetRec_.code[sizeof(targetRec_.code) - 1] = '\0';
        }
        targetRec_.confidence = content.value("confidence", 0.0);
        targetRec_.height = content.value("height", 0.0);
        targetRec_.orientation = content.value("orientation", 0.0);
        targetRec_.pitch = content.value("pitch", 0.0);

        target_recongnition_[targetRec_.batch] = targetRec_;
    }
    catch (const std::exception &e)
    {
        std::cout << "Error handle_0121:" << e.what() << std::endl;
    }
}
// 接受目标信息修订协议0151，然后可以修改指定批号目标的类型等等
void MqttTransport::handle_0151(const nlohmann::json &json_msg, const std::string &topic)
{
    try
    {
        TargetRevision trv;
        const auto &content = json_msg.at("content");

        trv.batch = content.value("batch", 0);
        trv.color = content.value("color", 0);
        trv.type = content.value("type", 0);
        trv.threatingRadius = content.value("threatingRadius", 0.0);
        std::string code_str = content.value("code", "");
        std::strncpy(trv.code, code_str.c_str(), sizeof(trv.code) - 1);
        trv.code[sizeof(trv.code) - 1] = '\0';

        target_revisions_[trv.batch] = trv;
    }
    catch (const std::exception &e)
    {
        std::cout << "Handle_0151 Error : " << e.what() << std::endl;
    }
}
// 接受导航共享协议0A11，接收其他个平台发来的位置信息
void MqttTransport::handle_0A11(const nlohmann::json &json_msg, const std::string &topic)
{
    nlohmann::json output;
    try
    {
        // head
        const auto &head_in = json_msg.at("head");
        output["head"] = {{"packageId", 0},
                          {"packageSerialNumber", head_in.value("packageSeq", 0)},
                          {"packageConfirmNumber", 0},
                          {"sendIP", ""},
                          {"destIP", ""}};

        nlohmann::json unit;
        // content
        const auto &content = json_msg.at("content");
        // infoUnitContent
        nlohmann::json infoUnitContent;
        infoUnitContent["longitude"] = content.value("longitude", 0);
        infoUnitContent["latitude"] = content.value("latitude", 0);
        infoUnitContent["height"] = content.value("height", 0);
        infoUnitContent["absSpeed"] = content.value("absSpeed", 0);
        infoUnitContent["absCourse"] = content.value("absCourse", 0);
        infoUnitContent["heading"] = content.value("heading", 0);

        // infoUnitHead
        nlohmann::json infoUnitHead;
        infoUnitHead["sourcePlatformId"] = 3073;
        infoUnitHead["destPlatformId"] = 0;
        infoUnitHead["infoSourceTypeId"] = 3073;
        infoUnitHead["infoUnitExtentId"] = 0;
        infoUnitHead["infoUnitID"] = 0x03;
        infoUnitHead["secondInfoUnitId"] = 0;
        infoUnitHead["infoUnitLength"] = 0;
        infoUnitHead["infoUnitCreateTime"] = getCurrentTimeString().substr(11);

        unit["infoUnitContent"] = infoUnitContent;
        unit["infoUnitHead"] = infoUnitHead;
        output["content"] = nlohmann::json::array({unit});
        std::cout << "[0A11 output]" << output.dump(4) << std::endl;
        publish("0E23H-0", output);
    }
    catch (const std::exception &e)
    {
        std::cout << "Error : " << e.what() << std::endl;
    }
}
// 接收单艇导航协议0411，获取本艇的位置姿态信息
void MqttTransport::handle_0411(const nlohmann::json &json_msg, const std::string &topic)
{
    // std::cout<<"[0411] "<<json_msg.dump(4)<<std::endl;
    nlohmann::json output;
    try
    {
        // head
        const auto &head_in = json_msg.at("head");
        output["head"] = {{"packageId", 0},
                          {"packageSerialNumber", head_in.value("packageSeq", 0)},
                          {"packageConfirmNumber", 0},
                          {"sendIP", ""},
                          {"destIP", ""}};

        // content
        nlohmann::json unit;
        const auto &content = json_msg.at("content");

        nlohmann::json infoUnitContent;
        infoUnitContent["targetLongitude"] = content.value("longitude", 0.0);
        infoUnitContent["targetLatitude"] = content.value("latitude", 0.0);
        self_longitude_ = infoUnitContent["targetLongitude"];
        self_latitude_ = infoUnitContent["targetLatitude"];
        std::cout << "self longitude " << self_longitude_ << std::endl;
        std::cout << "self latitude " << self_latitude_ << std::endl;
        infoUnitContent["targetHeight"] = content.value("height", 0.0);
        infoUnitContent["targetAbsoluteSpeed"] = content.value("speed", 0.0);
        infoUnitContent["targetAbsoluteDirection"] = content.value("course", 0.0);
        infoUnitContent["headingAngle"] = content.value("heading", 0.0);
        self_heading_ = infoUnitContent["headingAngle"];
        std::cout << "self heading " << self_heading_ << std::endl;
        infoUnitContent["eastSpeed"] = content.value("eastSpeed", 0.0);
        infoUnitContent["northSpeed"] = content.value("northSpeed", 0.0);
        infoUnitContent["heavingSpeed"] = content.value("verticalSpeed", 0.0);
        infoUnitContent["pitch"] = content.value("pitch", 0.0);
        infoUnitContent["rolling"] = content.value("rolling", 0.0);
        infoUnitContent["angularSpeedX"] = content.value("angularX", 0.0);
        infoUnitContent["angularSpeedY"] = content.value("angularY", 0.0);
        infoUnitContent["angularSpeedZ"] = content.value("angularZ", 0.0);
        infoUnitContent["accelerationX"] = content.value("accX", 0.0);
        infoUnitContent["accelerationY"] = content.value("accY", 0.0);
        infoUnitContent["accelerationZ"] = content.value("accZ", 0.0);
        infoUnitContent["headingAngleAcceleration"] = content.value("headingAcc", 0.0);
        infoUnitContent["simulationRatio"] = 1;
        infoUnitContent["targetSimulateSign"] = 0;
        infoUnitContent["cruiseMode"] = 1;
        // infoUnitHead
        nlohmann::json infoUnitHead;
        infoUnitHead["sourcePlatformId"] = 3073;
        infoUnitHead["destPlatformId"] = 1793;
        infoUnitHead["infoSourceTypeId"] = 0;
        infoUnitHead["infoUnitExtentId"] = 3;
        infoUnitHead["infoUnitID"] = 0x03;
        infoUnitHead["secondInfoUnitId"] = 3741;
        infoUnitHead["infoUnitLength"] = 0;
        infoUnitHead["infoUnitCreateTime"] = getCurrentTimeString().substr(11);

        unit["infoUnitContent"] = infoUnitContent;
        unit["infoUnitHead"] = infoUnitHead;

        output["content"] = nlohmann::json::array({unit});

        publish("0E9DH-3", output);
    }
    catch (const std::exception &e)
    {
        std::cerr << "[handle_0411] Error: " << e.what() << std::endl;
    }
}

// 接收人工增加目标协议0152
void MqttTransport::handle_0152(const nlohmann::json &json_msg, const std::string &topic)
{
    std::cout << "[Handle_0152]add target 0152" << std::endl;
    std::cout << "json :" << json_msg.dump(4) << std::endl;
    try
    {
        const auto content = json_msg.at("content");
        const auto &targets = content.at("targets");

        for (const auto &t : targets)
        {
            ManualTarget mt;
            mt.id = t.value("batch", 0);
            mt.absSpeed = t.value("absSpeed", 0.0);
            mt.absCourse = t.value("absCourse", 0.0);
            mt.longitude = t.value("longitude", 0.0);
            mt.latitude = t.value("latitude", 0.0);
            mt.type = t.value("type", 0);
            mt.lastUpdateTime = std::chrono::steady_clock::now(); // 更新时间戳
            std::cout << "id:" << mt.id << " absspeed:" << mt.absSpeed << " absCourse:" << mt.absCourse
                      << " longitude:" << mt.longitude << " latitude:" << mt.latitude << " type:" << mt.type
                      << std::endl;
            manual_targets_[mt.id] = mt;
        }
    }
    catch (const std::exception &e)
    {
        std::cout << "handle_0152 Error : " << e.what() << std::endl;
    }
}

void MqttTransport::handle_0E20H0(const nlohmann::json &json_msg, const std::string &topic)
{
    std::cout << "[0E20H-0 ]" << json_msg.dump(4) << std::endl;
    // mqtt::message_ptr msg = mqtt::make_message("0E20H-0", json_msg.dump());
    // pubOr_client_.publish(msg, nullptr, *this);
    try
    {
        if (!json_msg.contains("content") || !json_msg["content"].is_array() || json_msg["content"].empty())
        {
            std::cerr << "[0E20H-0] Invalid JSON: missing or empty 'content'" << std::endl;
            return;
        }
        const auto &content = json_msg["content"][0];
        if (!content.contains("infoUnitContent") || !content["infoUnitContent"].contains("targets") ||
            !content["infoUnitContent"]["targets"].is_array())
        {
            std::cerr << "[0E20H-0] Invalid JSON: missing 'infoUnitContent.targets'" << std::endl;
            return;
        }

        for (const auto &target : content["infoUnitContent"]["targets"])
        {
            int targetId = target.value("targetId", -1);
            std::cout << "[targetId ]" << targetId << std::endl;
            if (targetId == -1)
            {
                std::cerr << "[0E20H-0] Invalid targetId" << std::endl;
                continue;
            }

            auto &cache = target_cache[targetId];
            cache.part0 = target;
            cache.has0 = true;
            cache.last_update = std::chrono::steady_clock::now();
        }
        std::cout << "try merge" << std::endl;
        tryMerge();
    }
    catch (const nlohmann::json::type_error &e)
    {
        std::cerr << "[0E20H-0] Exception: " << e.what() << std::endl;
    }
}

void MqttTransport::handle_0E20H1(const nlohmann::json &json_msg, const std::string &topic)
{
    // mqtt::message_ptr msg = mqtt::make_message("0E20H-1", json_msg.dump());
    // pubOr_client_.publish(msg, nullptr, *this);
    try
    {
        if (!json_msg.contains("content") || !json_msg["content"].is_array() || json_msg["content"].empty())
        {
            std::cerr << "[0E20H-1] Invalid JSON: missing or empty 'content'" << std::endl;
            return;
        }
        const auto &content = json_msg["content"][0];
        if (!content.contains("infoUnitContent") || !content["infoUnitContent"].contains("targets") ||
            !content["infoUnitContent"]["targets"].is_array())
        {
            std::cerr << "[0E20H-1] Invalid JSON: missing 'infoUnitContent.targets'" << std::endl;
            return;
        }

        for (const auto &target : content["infoUnitContent"]["targets"])
        {
            int targetId = target.value("targetId", -1);
            if (targetId == -1)
            {
                std::cerr << "[0E20H-1] Invalid targetId" << std::endl;
                continue;
            }

            auto &cache = target_cache[targetId];
            cache.part1 = target;
            cache.has1 = true;
            cache.last_update = std::chrono::steady_clock::now();
        }
        tryMerge();
    }
    catch (const nlohmann::json::type_error &e)
    {
        std::cerr << "[0E20H-1] Exception: " << e.what() << std::endl;
    }
}

void MqttTransport::handle_0E20H2(const nlohmann::json &json_msg, const std::string &topic)
{
    // mqtt::message_ptr msg = mqtt::make_message("0E20H-2", json_msg.dump());
    // pubOr_client_.publish(msg, nullptr, *this);
    try
    {
        if (!json_msg.contains("content") || !json_msg["content"].is_array() || json_msg["content"].empty())
        {
            std::cerr << "[0E20H-2] Invalid JSON: missing or empty 'content'" << std::endl;
            return;
        }
        const auto &content = json_msg["content"][0];
        if (!content.contains("infoUnitContent") || !content["infoUnitContent"].contains("targets") ||
            !content["infoUnitContent"]["targets"].is_array())
        {
            std::cerr << "[0E20H-2] Invalid JSON: missing 'infoUnitContent.targets'" << std::endl;
            return;
        }

        for (const auto &target : content["infoUnitContent"]["targets"])
        {
            int targetId = target.value("targetId", -1);
            if (targetId == -1)
            {
                std::cerr << "[0E20H-2] Invalid targetId" << std::endl;
                continue;
            }

            auto &cache = target_cache[targetId];
            cache.part2 = target;
            cache.has2 = true;
            cache.last_update = std::chrono::steady_clock::now();
        }
        tryMerge();
    }
    catch (const nlohmann::json::type_error &e)
    {
        std::cerr << "[0E20H-2] Exception: " << e.what() << std::endl;
    }
}

void MqttTransport::handle_underwater_motion(const nlohmann::json &json_msg, const std::string &topic)
{
    try
    {
        if (!json_msg.contains("content") || !json_msg["content"].is_array() || json_msg["content"].empty())
        {
            std::cerr << "[0E21H-0] Invalid JSON: missing or empty 'content'" << std::endl;
            return;
        }
        const auto &content = json_msg["content"][0];
        if (!content.contains("infoUnitContent"))
        {
            std::cerr << "[0E21H-0] Invalid JSON: missing 'infoUnitContent'" << std::endl;
            return;
        }
        const auto &info = content["infoUnitContent"];
        if (!info.contains("targets") || !info["targets"].is_array() || info["targets"].empty())
        {
            std::cerr << "[0E21H-0] Invalid JSON: missing or empty 'targets'" << std::endl;
            return;
        }

        int targetId = info["targets"][0].value("targetId", -1);
        if (targetId == -1)
        {
            std::cerr << "[0E21H-0] Invalid JSON: 'targetId' missing or invalid" << std::endl;
            return;
        }

        auto &cache = target_cache[targetId];
        cache.part0 = json_msg;
        cache.has0 = true;
        cache.last_update = std::chrono::steady_clock::now();
    }
    catch (const std::exception &e)
    {
        std::cerr << "[0E20H-1] Exception: " << e.what() << std::endl;
    }
}

void MqttTransport::handle_underwater_identification(const nlohmann::json &json_msg, const std::string &topic)
{
    try
    {
        if (!json_msg.contains("content") || !json_msg["content"].is_array() || json_msg["content"].empty())
        {
            std::cerr << "[0E21H-1] Invalid JSON: missing or empty 'content'" << std::endl;
            return;
        }
        const auto &content = json_msg["content"][0];
        if (!content.contains("infoUnitContent"))
        {
            std::cerr << "[0E21H-1] Invalid JSON: missing 'infoUnitContent'" << std::endl;
            return;
        }
        const auto &info = content["infoUnitContent"];
        if (!info.contains("targets") || !info["targets"].is_array() || info["targets"].empty())
        {
            std::cerr << "[0E21H-1] Invalid JSON: missing or empty 'targets'" << std::endl;
            return;
        }

        int targetId = info["targets"][0].value("targetId", -1);
        if (targetId == -1)
        {
            std::cerr << "[0E21H-1] Invalid JSON: 'targetId' missing or invalid" << std::endl;
            return;
        }

        auto &cache = target_cache[targetId];
        cache.part0 = json_msg;
        cache.has0 = true;
        cache.last_update = std::chrono::steady_clock::now();
    }
    catch (const std::exception &e)
    {
        std::cerr << "[0E21H-1] Exception: " << e.what() << std::endl;
    }
}
void MqttTransport::handle_underwater_name(const nlohmann::json &json_msg, const std::string &topic)
{
    try
    {
        if (!json_msg.contains("content") || !json_msg["content"].is_array() || json_msg["content"].empty())
        {
            std::cerr << "[0E21H-2] Invalid JSON: missing or empty 'content'" << std::endl;
            return;
        }
        const auto &content = json_msg["content"][0];
        if (!content.contains("infoUnitContent"))
        {
            std::cerr << "[0E20H-2] Invalid JSON: missing 'infoUnitContent'" << std::endl;
            return;
        }
        const auto &info = content["infoUnitContent"];
        if (!info.contains("targets") || !info["targets"].is_array() || info["targets"].empty())
        {
            std::cerr << "[0E21H-2] Invalid JSON: missing or empty 'targets'" << std::endl;
            return;
        }

        int targetId = info["targets"][0].value("targetId", -1);
        if (targetId == -1)
        {
            std::cerr << "[0E21H-2] Invalid JSON: 'targetId' missing or invalid" << std::endl;
            return;
        }

        auto &cache = target_cache[targetId];
        cache.part0 = json_msg;
        cache.has0 = true;
        cache.last_update = std::chrono::steady_clock::now();
    }
    catch (const std::exception &e)
    {
        std::cerr << "[0E21H-2] Exception: " << e.what() << std::endl;
    }
}

std::string MqttTransport::getCurrentTimeString()
{
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %H:%M:%S");
    ss << "." << std::setfill('0') << std::setw(3) << ms.count();
    return ss.str();
}

void MqttTransport::publish(const std::string &topic, const nlohmann::json &json_msg)
{
    try
    {
        mqtt::message_ptr msg = mqtt::make_message(topic, json_msg.dump());
        pub_client_.publish(msg, nullptr, *this);
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
    }
}

void MqttTransport::tryMerge()
{
    // clean expiredcaches
    cleanupExpiredCaches();

    std::vector<int> readyIds;
    for (const auto &[id, cache] : target_cache)
    {
        if (cache.has0 && cache.has1 && cache.has2)
            readyIds.push_back(id);
        else
            std::cout << "Don't have at least one target" << std::endl;
    }
    for (const auto &i : readyIds)
    {
        std::cout << "[TargetID] " << i << std::endl;
    }
    if (!readyIds.empty())
    {
        mergeAndOutput(readyIds);
    }
}

void MqttTransport::mergeAndOutput(const std::vector<int> &targetIds)
{
    nlohmann::json merged;
    merged["head"]["packageSeq"] = 1;
    merged["head"]["packageType"] = 0;
    merged["head"]["time"] = getCurrentTimeString();

    nlohmann::json targets = nlohmann::json::array();

    for (int id : targetIds)
    {
        std::cout << "merged Id " << id << std::endl;
        auto it = target_cache.find(id);
        if (it == target_cache.end())
            continue;

        const auto &cache = it->second;
        if (!(cache.has0 && cache.has1 && cache.has2))
            continue;

        const auto &t0 = cache.part0;
        const auto &t1 = cache.part1;
        const auto &t2 = cache.part2;

        nlohmann::json tgt;
        tgt["batch"] = id;
        tgt["absSpeed"] = t0.value("targetAbsoluteSpeed", 0.0);
        tgt["absCourse"] = t0.value("targetAbsoluteDirection", 0.0);
        tgt["heading"] = t0.value("targetRelativeDirection", 0.0);
        tgt["longitude"] = t0.value("targetLongitude", 0.0);
        tgt["latitude"] = t0.value("targetLatitude", 0.0);
        double longitude = tgt["longitude"];
        double latitude = tgt["latitude"];
        double distance = GeoUtils::computeDistance(self_longitude_, self_latitude_, longitude, latitude);
        // std::cout << "距离: " << distance << " 米" << std::endl;

        double absAzimuth = GeoUtils::computeBearing(self_longitude_, self_latitude_, longitude, latitude);
        // std::cout<<"absAzimuth" <<absAzimuth<<std::endl;

        double relAzimuth = GeoUtils::computeRelBearing(absAzimuth, self_heading_);
        // std::cout<<" [relAzimuth]"<<relAzimuth<<std::endl;
        /*tgt["distance"] = distance;*/
        tgt["absAzimuth"] = absAzimuth;
        tgt["relAzimuth"] = relAzimuth;

        tgt["distance"] = t0.value("targetDistance", 0.0);
        // 存疑
        //  tgt["absAzimuth"] = t0.value("targetDirection", 0.0);
        //  tgt["relAzimuth"] = t0.value("targetRelativeDirection", 0.0);

        int sign = t1.value("targetIFFSign", 0);
        if (sign == 6)
            sign == 65535;
        if (sign == 3)
            sign == 0;
        tgt["sign"] = sign;
        // tgt["shipLength"] = t0.value("targetEdgeLength", 0.0);
        // tgt["shipWidth"] = t0.value("targetEdgeWidth", 0.0);
        // tgt["shipHeight"] = t0.value("targetHeight", 0.0);
        tgt["type"] = t1.value("targetType", 0);
        tgt["color"] = 0;
        tgt["state"] = t1.value("targetStatus", 0);
        tgt["threatingRadius"] = t1.value("targetThreatingRadius", 0.0);

        tgt["code"] = t2.value("targetPennantNumber", "");
        tgt["model"] = t2.value("targetShipType", 0);
        tgt["name"] = t2.value("targetName", "");

        tgt["MMSI"] = 0;
        tgt["source"] = t0.value("trackQualityNumber", 0.0);

        // 0121
        auto rec_it = target_recongnition_.find(id);
        if (rec_it != target_recongnition_.end())
        {
            const auto &rec = rec_it->second;
            tgt["sign"] = rec.sign;
            tgt["source"] = rec.source;
            tgt["type"] = rec.type;
            tgt["color"] = rec.color;
            tgt["code"] = rec.code;
        }
        else
            std::cerr << "cant't find id for 0121" << std::endl;

        // 目标修订协议新增
        // ✅ 尝试从 target_revisions_ 获取识别信息
        auto rev_it = target_revisions_.find(id);
        if (rev_it != target_revisions_.end())
        {
            const auto &rev = rev_it->second;
            tgt["type"] = rev.type;
            tgt["color"] = rev.color;
            tgt["threatingRadius"] = rev.threatingRadius;
            tgt["code"] = rev.code;
        }
        targets.push_back(tgt);
        target_cache.erase(it); // 清除已处理的
    }

    // 如果要人工增加目标
    using clock = std::chrono::steady_clock;
    auto now = clock::now();
    auto expireDuration = std::chrono::seconds(3); // 保活 3 秒
    for (auto it = manual_targets_.begin(); it != manual_targets_.end();)
    {
        if (now - it->second.lastUpdateTime > expireDuration)
        {
            std::cout << "[ManualTarget] Expired: " << it->first << std::endl;
            it = manual_targets_.erase(it);
            continue;
        }
        const auto &mt = it->second;
        nlohmann::json tgt;
        tgt["batch"] = mt.id;
        tgt["absSpeed"] = mt.absSpeed;
        tgt["absCourse"] = mt.absCourse;
        tgt["longitude"] = mt.longitude;
        tgt["latitude"] = mt.latitude;

        double distance = GeoUtils::computeDistance(self_longitude_, self_latitude_, mt.longitude, mt.latitude);
        // std::cout << "距离: " << distance << " 米" << std::endl;

        double absAzimuth = GeoUtils::computeBearing(self_longitude_, self_latitude_, mt.longitude, mt.latitude);
        // std::cout<<"absAzimuth" <<absAzimuth<<std::endl;

        double relAzimuth = GeoUtils::computeRelBearing(absAzimuth, self_heading_);
        // std::cout<<" [relAzimuth]"<<relAzimuth<<std::endl;
        tgt["type"] = mt.type;
        tgt["heading"] = 0;
        tgt["distance"] = distance;
        tgt["absAzimuth"] = absAzimuth;
        tgt["relAzimuth"] = relAzimuth;
        tgt["sign"] = 0;
        tgt["color"] = 0;
        tgt["state"] = 0;
        tgt["threatingRadius"] = 0;

        tgt["code"] = "";
        tgt["model"] = 0;
        tgt["name"] = "";
        tgt["shipLength"] = 0;
        tgt["shipWidth"] = 0;
        tgt["shipHeight"] = 0;
        tgt["MMSI"] = 0;
        tgt["source"] = 0;

        // 0121
        auto rec_it = target_recongnition_.find(mt.id);
        if (rec_it != target_recongnition_.end())
        {
            const auto &rec = rec_it->second;
            tgt["sign"] = rec.sign;
            tgt["source"] = rec.source;
            tgt["type"] = rec.type;
            tgt["color"] = rec.color;
            tgt["code"] = rec.code;
        }
        else
            std::cerr << "cant't find id for 0121" << std::endl;

        // 如果有目标修订协议？
        auto rev_it = target_revisions_.find(mt.id);
        if (rev_it != target_revisions_.end())
        {
            const auto &rev = rev_it->second;
            tgt["type"] = rev.type;
            tgt["color"] = rev.color;
            tgt["threatingRadius"] = rev.threatingRadius;
            tgt["code"] = rev.code;
        }
        targets.push_back(tgt);
        ++it;
    }
    // 应该是增加完一次就清除容器？
    // manual_targets_.clear();
    merged["content"]["count"] = targets.size();
    merged["content"]["targets"] = targets;
    std::cout << "[0141 Result] " << merged.dump(4) << std::endl;
    if (!targets.empty())
    {
        mqtt::message_ptr msg = mqtt::make_message("0141", merged.dump());
        pubR_client_.publish(msg, nullptr, *this);
        std::cout << "[0141] Published merged targets count: " << targets.size() << std::endl;
    }
}

void MqttTransport::saveMergedJsonToFile(const nlohmann::json &merged, const std::string &filename)
{
    std::ofstream file(filename, std::ios::app);
    if (!file.is_open())
    {
        std::cerr << "Failed to open file for writing: " << filename << std::endl;
        return;
    }

    file << std::setw(4) << merged << std::endl;
    file.close();
    std::cout << "Merged JSON saved to: " << filename << std::endl;
}

void MqttTransport::cleanupExpiredCaches()
{
    using namespace std::chrono;
    auto now = steady_clock::now();
    for (auto it = target_cache.begin(); it != target_cache.end();)
    {
        if (duration_cast<seconds>(now - it->second.last_update).count() > 30) // 超过30秒没更新
        {
            it = target_cache.erase(it);
        }
        else
        {
            ++it;
        }
    }
}

void MqttTransport::handle_0220H0(const nlohmann::json &json_msg, const std::string &topic)
{
    std::cout << "[0220H-0]" << json_msg.dump(4) << std::endl;
    try
    {
        if (!json_msg.contains("content") || !json_msg["content"].is_array() || json_msg["content"].empty())
        {
            std::cerr << "[0220H-0] Invalid JSON: missing or empty 'content'" << std::endl;
            return;
        }
        const auto &content = json_msg["content"][0];
        if (!content.contains("infoUnitContent") || !content["infoUnitContent"].contains("targets") ||
            !content["infoUnitContent"]["targets"].is_array())
        {
            std::cerr << "[0220H-0] Invalid JSON: missing 'infoUnitContent.targets'" << std::endl;
            return;
        }

        for (const auto &target : content["infoUnitContent"]["targets"])
        {
            int targetId = target.value("targetId", -1);
            std::cout << "[targetId ]" << targetId << std::endl;
            if (targetId == -1)
            {
                std::cerr << "[0220H-0] Invalid targetId" << std::endl;
                continue;
            }

            auto &cache = cluster_target_cache[targetId];
            cache.part0 = target;
            cache.has0 = true;
            cache.last_update = std::chrono::steady_clock::now();
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "[0E20H-0] Exception: " << e.what() << std::endl;
    }
}

void MqttTransport::handle_0220H1(const nlohmann::json &json_msg, const std::string &topic)
{
    std::cout << "[0220H-1]" << json_msg.dump(4) << std::endl;
    try
    {
        if (!json_msg.contains("content") || !json_msg["content"].is_array() || json_msg["content"].empty())
        {
            std::cerr << "[0220H-1] Invalid JSON: missing or empty 'content'" << std::endl;
            return;
        }
        const auto &content = json_msg["content"][0];
        if (!content.contains("infoUnitContent") || !content["infoUnitContent"].contains("targets") ||
            !content["infoUnitContent"]["targets"].is_array())
        {
            std::cerr << "[0220H-1] Invalid JSON: missing 'infoUnitContent.targets'" << std::endl;
            return;
        }

        for (const auto &target : content["infoUnitContent"]["targets"])
        {
            int targetId = target.value("targetId", -1);
            std::cout << "[targetId ]" << targetId << std::endl;
            if (targetId == -1)
            {
                std::cerr << "[0220H-1] Invalid targetId" << std::endl;
                continue;
            }

            auto &cache = cluster_target_cache[targetId];
            cache.part1 = target;
            cache.has1 = true;
            cache.last_update = std::chrono::steady_clock::now();
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "[0E20H-1] Exception: " << e.what() << std::endl;
    }
}
void MqttTransport::handle_0220H2(const nlohmann::json &json_msg, const std::string &topic)
{
    std::cout << "[0220H-2]" << json_msg.dump(4) << std::endl;
    try
    {
        if (!json_msg.contains("content") || !json_msg["content"].is_array() || json_msg["content"].empty())
        {
            std::cerr << "[0220H-2] Invalid JSON: missing or empty 'content'" << std::endl;
            return;
        }
        const auto &content = json_msg["content"][0];
        if (!content.contains("infoUnitContent") || !content["infoUnitContent"].contains("targets") ||
            !content["infoUnitContent"]["targets"].is_array())
        {
            std::cerr << "[0220H-2] Invalid JSON: missing 'infoUnitContent.targets'" << std::endl;
            return;
        }

        for (const auto &target : content["infoUnitContent"]["targets"])
        {
            int targetId = target.value("targetId", -1);
            std::cout << "[targetId ]" << targetId << std::endl;
            if (targetId == -1)
            {
                std::cerr << "[0220H-2] Invalid targetId" << std::endl;
                continue;
            }

            auto &cache = cluster_target_cache[targetId];
            cache.part2 = target;
            cache.has2 = true;
            cache.last_update = std::chrono::steady_clock::now();
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "[0E20H-2] Exception: " << e.what() << std::endl;
    }
}

void MqttTransport::tryClusterMerge()
{
    // clean expiredcaches
    cleanupExpiredClusterCaches();

    std::vector<int> readyIds;
    for (const auto &[id, cache] : cluster_target_cache)
    {
        if (cache.has0 && cache.has1 && cache.has2)
            readyIds.push_back(id);
        else
            std::cout << "Don't have at least one target" << std::endl;
    }
    for (const auto &i : readyIds)
    {
        std::cout << "[TargetID] " << i << std::endl;
    }
    if (!readyIds.empty())
    {
        clustermergeAndOutput(readyIds);
    }
}

void MqttTransport::clustermergeAndOutput(const std::vector<int> &targetIds)
{
    nlohmann::json merged;
    merged["head"]["packageSeq"] = 1;
    merged["head"]["packageType"] = 0;
    merged["head"]["time"] = getCurrentTimeString();

    nlohmann::json targets = nlohmann::json::array();

    for (int id : targetIds)
    {
        std::cout << "merged Id " << id << std::endl;
        auto it = cluster_target_cache.find(id);
        if (it == cluster_target_cache.end())
            continue;

        const auto &cache = it->second;
        if (!(cache.has0 && cache.has1 && cache.has2))
            continue;

        const auto &t0 = cache.part0;
        const auto &t1 = cache.part1;
        const auto &t2 = cache.part2;

        nlohmann::json tgt;
        tgt["batch"] = id;
        tgt["absSpeed"] = t0.value("targetAbsoluteSpeed", 0.0);
        tgt["absCourse"] = t0.value("targetAbsoluteDirection", 0.0);
        tgt["heading"] = t0.value("targetRelativeDirection", 0.0);
        tgt["longitude"] = t0.value("targetLongitude", 0.0);
        tgt["latitude"] = t0.value("targetLatitude", 0.0);
        tgt["distance"] = t0.value("targetDistance", 0.0);
        tgt["absAzimuth"] = t0.value("targetAbsoluteDirection", 0.0);
        tgt["relAzimuth"] = t0.value("targetDirection", 0.0);
        tgt["sign"] = t0.value("targetSimulateSign", 0);

        tgt["type"] = t1.value("targetType", 0);
        tgt["color"] = 0;
        tgt["state"] = t1.value("targetStatus", 0);
        tgt["threatingRadius"] = t1.value("targetThreatingRadius", 0.0);

        tgt["code"] = t2.value("targetPennantNumber", "");
        tgt["model"] = t2.value("targetShipType", 0);
        tgt["name"] = t2.value("targetName", "");
        tgt["shipLength"] = t2.value("targetShipLength", 0.0);
        tgt["shipWidth"] = t2.value("targetShipWidth", 0.0);
        tgt["shipHeight"] = t2.value("targetShipHeight", 0.0);
        tgt["MMSI"] = 0;

        targets.push_back(tgt);
        cluster_target_cache.erase(it); // 清除已处理的
    }

    merged["content"]["count"] = targets.size();
    merged["content"]["targets"] = targets;
    std::cout << "[0B11 Result] " << merged.dump(4) << std::endl;
    if (!targets.empty())
    {
        mqtt::message_ptr msg = mqtt::make_message("0B11", merged.dump());
        pubR_client_.publish(msg, nullptr, *this);
        std::cout << "[0B11] Published merged targets count: " << targets.size() << std::endl;
    }
}

void MqttTransport::cleanupExpiredClusterCaches()
{
    using namespace std::chrono;
    auto now = steady_clock::now();
    for (auto it = cluster_target_cache.begin(); it != cluster_target_cache.end();)
    {
        if (duration_cast<seconds>(now - it->second.last_update).count() > 20) // 超过30秒没更新
        {
            it = cluster_target_cache.erase(it);
        }
        else
        {
            ++it;
        }
    }
}
