# 导航雷达

| 0111       | 0E23H-0                 |      |
| ---------- | ----------------------- | ---- |
| batch      | targetId                |      |
| absAzimuth | targetDirection         |      |
| distance   | targetDistance          |      |
| absSpeed   | targetAbsoluteSpeed     |      |
| absCourse  | targetAbsoluteDirection |      |
| longitude  | targetLongitude         |      |
| latitude   | targetLatitude          |      |
| state      | targetState             |      |

```cpp
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

        unit["infoUnitHead"] = {{"destPlatformId", head_in.value("destPlatformId", 0)},
                                {"infoSourceTypeId", 0},
                                {"infoUnitCreateTime", getCurrentTimeString().substr(11)}, // "HH:MM:SS.xxx"
                                {"infoUnitExtentId", 0},
                                {"infoUnitID", 0},
                                {"infoUnitLength", 0},
                                {"secondInfoUnitId", head_in.value("secondPlatformId", 0)},
                                {"sourcePlatformId", head_in.value("sourcePlatformId", 0)}};

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
            tgt["targetSourcePlatformId"] = 0;
            tgt["targetTcpa"] = 0;
            tgt["trackQualityNumber"] = 0;

            content_out["targets"].push_back(tgt);
        }

        unit["infoUnitContent"] = content_out;
        output["content"] = nlohmann::json::array({unit});

        publish("0E23H-0", output);
        mqtt::message_ptr msg = mqtt::make_message("0E23H-0", output.dump());
        pubOr_client_.publish(msg, nullptr, *this);
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error in handle_0111: " << e.what() << std::endl;
    }
}
```

# 激光雷达 

| 0131        | 0E25H-2                 |      |
| ----------- | ----------------------- | ---- |
| batch:,     | targetId                |      |
| absAzimuth: | targetAbsoluteDirection |      |
| distance:,  | targetDistance          |      |
| absSpeed    | targetAbsoluteSpeed     |      |
| absCourse:, | targetDirection         |      |
| heading     | targetRelativeDirection |      |
| longitude   | targetLatitude          |      |
| latitude    | targetLongitude         |      |
| state       | targetState             |      |
| length:     | targetEdgeLength        |      |
| width       | targetEdgeWidth         |      |

```cpp
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
            target["targetSourcePlatformId"] = 0;
            target["targetHeight"] = 0;
            target["coordinateSystemTaype"] = 0;

            content_out["targets"].push_back(target);
        }

        info_unit["infoUnitContent"] = content_out;

        // infoUnitHead 置零
        info_unit["infoUnitHead"] = {
            {"infoUnitID", 0},       {"secondInfoUnitId", 0},
            {"sourcePlatformId", 0}, {"destPlatformId", 0},
            {"infoUnitLength", 0},   {"infoUnitExtentId", 0},
            {"infoSourceTypeId", 0}, {"infoUnitCreateTime", getCurrentTimeString().substr(11)} // 仅保留 HH:mm:ss.sss
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
```

# AIS

| 0421         | 0E30H-1   |      |
| ------------ | --------- | ---- |
| absSpeed:,   | COG       |      |
| absCourse:,  | SOG       |      |
| heading:     | ShipHead  |      |
| longitude:,  | Longitude |      |
| latitude:,   | Latitude  |      |
| type         | aisType   |      |
| name         |           |      |
| shipLength:, |           |      |
| shipWidth    |           |      |
| shipHeight:  |           |      |
| MMSI         | MMSI      |      |

```cpp
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
        infoUnitHead["destPlatformId"] = 0;
        infoUnitHead["sourcePlatformId"] = 0;
        infoUnitHead["infoUnitCreateTime"] = ""; // eg. "11:31:36.300"
        infoUnitHead["infoUnitExtentId"] = 0;
        infoUnitHead["infoUnitID"] = 0;
        infoUnitHead["infoUnitLength"] = 0;
        infoUnitHead["secondInfoUnitId"] = 0;
        infoUnitHead["infoSourceTypeId"] = 0;

        unit["infoUnitHead"] = infoUnitHead;

        output["content"] = nlohmann::json::array({unit});

        publish("0E30H-1", output);
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
    }
}
```

# 单艇导航协议

| 0411                                  | 0E9DH-3                  |      |
| ------------------------------------- | ------------------------ | ---- |
| longitude:, // 经度number             | targetLongitude          |      |
| latitude:,  // 纬度number             | targetLatitude           |      |
| height:,  // 高度number               | targetHeight             |      |
| speed:,  // 航速number                | targetAbsoluteSpeed      |      |
| course:,  // 航向number               | targetAbsoluteDirection  |      |
| heading:,  // 艏向角number            | headingAngle             |      |
| headingAcc:, // 艏向角加速度number    | headingAngleAcceleration |      |
| eastSpeed:,  // 东向速度number        | eastSpeed                |      |
| northSpeed:,  // 北向速度number       | northSpeed               |      |
| verticalSpeed:,  // 垂直速度number    | heavingSpeed             |      |
| pitch:,   // 纵摇number               | pitch                    |      |
| rolling:,   // 横摇number             | rolling                  |      |
| angularX:,  // 姿态角速度（X）number  | angularSpeedX            |      |
| angularY:,   // 姿态角速度（Y）number | angularSpeedY            |      |
| angularZ:,  // 姿态角速度（Z）number  | angularSpeedZ            |      |
| accX:,   // X向加速度number           | accelerationX            |      |
| accY:,   // Y向加速度number           | accelerationY            |      |
| accZ:,   // Z向加速度number           | accelerationZ            |      |
| deviceState:                          | cruiseMode               |      |

```cpp
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

        publish("0E9DH-3", output);
    }
    catch (const std::exception &e)
    {
        std::cerr << "[handle_0411] Error: " << e.what() << std::endl;
    }
}
```

# 单艇融合目标信息

| 0141                                 |                         |      |
| ------------------------------------ | ----------------------- | ---- |
| batch:,   // 批号 number             | targetId                |      |
| absSpeed:,  // 绝对航速 number       | targetAbsoluteSpeed     |      |
| absCourse:,  // 绝对航向 number      | targetAbsoluteDirection |      |
| heading:,   // 艏向number            | targetRelativeDirection |      |
| longitude:,  // 经度 number          | targetLongitude         |      |
| latitude:,   // 纬度 number          | targetLatitude          |      |
| distance:,   // 相对距离number       | targetDistance          |      |
| absAzimuth:,  // 绝对方位number      | targetDirection         |      |
| relAzimuth:,  // 相对方位number      | targetRelativeDirection |      |
| sign:,   // 属性 number              | targetIFFSign           |      |
| type:,   // 类型 number              | targetType              |      |
| color:,   // 颜色number              | 0                       |      |
| status:,   // 状态 number            | targetStatus            |      |
| threatingRadius:, // 威胁半径 number | targetThreatingRadius   |      |
| code:,   // 船舶识别舷号 string      | targetPennantNumber     |      |
| model:,   // 型号number              | targetShipType          |      |
| name:"",   // 船舶名称 string        | targetName              |      |
| shipLength:,  // 船长 number         | targetEdgeLength        |      |
| shipWidth:,  // 船宽 number          | targetEdgeWidth         |      |
| shipHeight:,  // 船高 number         | targetHeight            |      |
| MMSI:   // 识别号 number             | 0                       |      |

