#include "MqttTransport.h"
#include <exception>
#include <iostream>
#include <set>
#include <string>
#include <vector>
#include <yaml-cpp/node/parse.h>
#include <yaml-cpp/yaml.h>

void loadconfig(std::vector<std::string> &topics)
{
    std::set<std::string> unique_topics;
    try
    {
        YAML::Node config = YAML::LoadFile("szo.yml");
        auto driver_topics = config["usfs_bridge"]["driver"]["mqtt_topics"];
        if (driver_topics && driver_topics.IsMap())
        {
            for (auto it = driver_topics.begin(); it != driver_topics.end(); ++it)
            {
                const YAML::Node &value = it->second;
                if (value.IsScalar())
                {
                    unique_topics.insert(value.as<std::string>());
                }
            }
        }

        topics.assign(unique_topics.begin(), unique_topics.end());
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error Loading Config: " << e.what() << std::endl;
    }
}

int main(int argc, char **argv)
{
    std::vector<std::string> topics;
    loadconfig(topics);

    MqttTransport sub("tcp://192.1.18.8:1883", "tcp://192.1.18.8:1883", "sub_brige_client", "pub_brige_client");
    sub.connect();
    topics.insert(topics.end(), {"0041", "0121", "0151", "0152", "0411", "0A11"});

    sub.subscribe(topics, std::vector<int>(topics.size(), 0));

    while (true)
    {
        sub.getNewMessage();
    }

    return 0;
}