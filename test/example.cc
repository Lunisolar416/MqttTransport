#include "mqtt/async_client.h"
#include "nlohmann/json.hpp"
#include "polygon_point_singleton.h"
#include <chrono>
#include <iostream>
#include <string>
#include <thread>

class Callback : public virtual mqtt::callback
{
  public:
    void message_arrived(mqtt::const_message_ptr msg) override
    {
        std::cout << "Message arrived on topic: " << msg->get_topic() << std::endl;
        try
        {
            auto json_msg = nlohmann::json::parse(msg->get_payload());
            handle_0042(json_msg, msg->get_topic());
        }
        catch (const std::exception &e)
        {
            std::cerr << "Failed to parse JSON: " << e.what() << std::endl;
        }
    }

    void connection_lost(const std::string &cause) override
    {
        std::cerr << "Connection lost: " << cause << std::endl;
    }

  private:
    void handle_0042(const nlohmann::json &json_msg, const std::string &topic)
    {
        std::vector<Eigen::Vector2d> polygon_lonlat;

        if (!json_msg.contains("content") || !json_msg["content"].is_object())
        {
            std::cerr << "handle_0042: missing or invalid content\n";
            return;
        }

        const auto &content = json_msg["content"];

        if (!content.contains("polygon") || !content["polygon"].is_array())
        {
            std::cerr << "handle_0042: missing or invalid polygon array\n";
            return;
        }

        for (const auto &point : content["polygon"])
        {
            if (!point.contains("areaLatitude") || !point.contains("areaLongitude"))
            {
                std::cerr << "handle_0042: polygon point missing lat or lon\n";
                continue;
            }

            double lat = point["areaLatitude"].get<double>();
            double lon = point["areaLongitude"].get<double>();
            polygon_lonlat.emplace_back(lon, lat);
        }

        if (!POLYGON_TESTER->LoadPolygon(polygon_lonlat))
        {
            std::cerr << "Failed to load polygon\n";
            return;
        }
        std::cout << "Loaded polygon with " << polygon_lonlat.size() << " points.\n";

        // 立即测试示例点是否在多边形内
        Eigen::Vector2d test_point_inside(109.480, 18.230);
        Eigen::Vector2d test_point_outside(109.460, 18.220);

        bool inside1 = POLYGON_TESTER->isPointInPolygon(test_point_inside);
        bool inside2 = POLYGON_TESTER->isPointInPolygon(test_point_outside);

        std::cout << "Point " << test_point_inside.transpose()
                  << (inside1 ? " is inside the polygon\n" : " is outside the polygon\n");

        std::cout << "Point " << test_point_outside.transpose()
                  << (inside2 ? " is inside the polygon\n" : " is outside the polygon\n");
    }
};

int main()
{
    const std::string SERVER_ADDRESS{"tcp://localhost:1883"}; // 修改为你的MQTT broker
    const std::string CLIENT_ID{"subscriber_0042"};
    const std::string TOPIC{"0042"};

    mqtt::async_client client(SERVER_ADDRESS, CLIENT_ID);
    Callback cb;
    client.set_callback(cb);

    mqtt::connect_options connOpts;
    connOpts.set_clean_session(true);

    try
    {
        std::cout << "Connecting to MQTT broker..." << std::endl;
        client.connect(connOpts)->wait();
        std::cout << "Connected." << std::endl;

        client.start_consuming();
        client.subscribe(TOPIC, 1)->wait();
        std::cout << "Subscribed to topic: " << TOPIC << std::endl;

        // 主循环，保持运行
        while (true)
        {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        client.disconnect()->wait();
    }
    catch (const mqtt::exception &exc)
    {
        std::cerr << "MQTT error: " << exc.what() << std::endl;
        return 1;
    }

    return 0;
}
