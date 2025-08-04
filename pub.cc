#include <mqtt/client.h>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>
int main()
{
    const std::string address = "tcp://192.168.50.129:1883";
    const std::string client_id = "cpp_pub";
    const std::string topic = "0E23H-0";

    mqtt::client client(address, client_id);
    mqtt::connect_options conn_opts;
    conn_opts.set_user_name("third_client");
    conn_opts.set_password("third_client");

    client.connect(conn_opts);

    std::string json_str = R"({
  "head": {
    "packageSeq": 1,
    "packageType": 0,
    "time": "2025-07-23 14:07:47.543"
  },
  "content": {
    "count": 2,
    "targets": [
      {
        "batch": 1001,
        "absAzimuth": 75.3,
        "distance": 1200.5,
        "absSpeed": 12.8,
        "absCourse": 90.0,
        "longitude": 121.123456,
        "latitude": 31.987654,
        "state": "1"
      },
      {
        "batch": 1001,
        "absAzimuth": 120.0,
        "distance": 850.0,
        "absSpeed": 8.3,
        "absCourse": 135.0,
        "longitude": 121.124321,
        "latitude": 31.988888,
        "state": "0"
      }
    ]
  }
}
)";
    nlohmann::json json_msg = nlohmann::json::parse(json_str);

    mqtt::message_ptr msg = mqtt::make_message(topic, json_msg.dump());
    msg->set_qos(0);

    client.publish(msg);

    client.disconnect();
    return 0;
}