#include <mqtt/async_client.h>
#include <mqtt/callback.h>
#include <mqtt/client.h>
#include <mqtt/connect_options.h>
#include <yaml-cpp/yaml.h>
#include <nlohmann/json.hpp>
void loadconfig(std::vector<std::string>& topics)
{
    try
    {
        YAML::Node config = YAML::LoadFile("szo_sanya_vessel.yml");
        YAML::Node mqtt_topics = config["usfs_bridge"]["driver"]["mqtt_topics"];
        if (!mqtt_topics)
        {
            std::cerr << "mqtt_topics not found in config" << std::endl;
            return;
        }

        for (auto it = mqtt_topics.begin(); it != mqtt_topics.end(); ++it)
        {
            const auto& value = it->second;
            if (value.IsScalar())
            {
                topics.push_back(value.as<std::string>());
            }
            else if (value.IsSequence())
            {
                for (const auto& item : value)
                {
                    if (item["topic"])
                    {
                        topics.push_back(item["topic"].as<std::string>());
                    }
                }
            }
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error Loading Config: " << e.what() << std::endl;
    }
}

class callback : public virtual mqtt::callback
{
   public:
    void message_arrived(mqtt::const_message_ptr msg) override
    {
        std::cout << "Received on topic '" << msg->get_topic();
        std::string payload = msg->to_string();
        nlohmann::json j = nlohmann::json::parse(payload);
        std::cout << "Message: " << j.dump(4) << std::endl;
    }
};
int main()
{
    const std::string address = "tcp://192.168.50.129:2883";
    const std::string client_id = "cpp-sub";

    std::vector<std::string> topics;
    loadconfig(topics);

    mqtt::client client(address, client_id);
    callback cb;
    client.set_callback(cb);

    mqtt::connect_options conn_opts;
    client.connect(conn_opts);

    client.subscribe(topics, std::vector<int>(topics.size(), 1));
    while (true)
    {
        client.consume_message();  // 消费新消息（触发回调）
    }
    client.disconnect();
    return 0;
}