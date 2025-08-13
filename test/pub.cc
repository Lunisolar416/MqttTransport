#include "mqtt/async_client.h"
#include "nlohmann/json.hpp"
#include <chrono>
#include <iostream>
#include <string>
#include <thread>

int main()
{
    const std::string SERVER_ADDRESS{"tcp://localhost:1883"}; // 修改为你的MQTT broker地址
    const std::string CLIENT_ID{"publisher_0042"};
    const std::string TOPIC{"0042"};

    mqtt::async_client client(SERVER_ADDRESS, CLIENT_ID);

    mqtt::connect_options connOpts;
    connOpts.set_clean_session(true);

    try
    {
        std::cout << "Connecting to MQTT broker..." << std::endl;
        client.connect(connOpts)->wait();
        std::cout << "Connected." << std::endl;
    }
    catch (const mqtt::exception &exc)
    {
        std::cerr << "Error connecting: " << exc.what() << std::endl;
        return 1;
    }

    // 构建 JSON 数据
    nlohmann::json msg_json = {
        {"content",
         {{"polygon",
           {{{"areaLatitude", 18.231961894780397}, {"areaLongitude", 109.46637332439423}},
            {{"areaLatitude", 18.226930238306522}, {"areaLongitude", 109.47083928622305}},
            {{"areaLatitude", 18.226782297715545}, {"areaLongitude", 109.47353951632977}},
            {{"areaLatitude", 18.23038324713707}, {"areaLongitude", 109.48371757753193}},
            {{"areaLatitude", 18.23378697037697}, {"areaLongitude", 109.49233769439161}},
            {{"areaLatitude", 18.234333219006658}, {"areaLongitude", 109.49520681984723}},
            {{"areaLatitude", 18.233198644593358}, {"areaLongitude", 109.49943909421563}},
            {{"areaLatitude", 18.233421351760626}, {"areaLongitude", 109.50029907748103}},
            {{"areaLatitude", 18.231264520436525}, {"areaLongitude", 109.50245976448059}},
            {{"areaLatitude", 18.23121632449329}, {"areaLongitude", 109.50284055434167}},
            {{"areaLatitude", 18.229896426200867}, {"areaLongitude", 109.5045679807663}},
            {{"areaLatitude", 18.228587675839663}, {"areaLongitude", 109.50535655021667}},
            {{"areaLatitude", 18.227578159421682}, {"areaLongitude", 109.50505102984607}},
            {{"areaLatitude", 18.226665537804365}, {"areaLongitude", 109.50619348324835}},
            {{"areaLatitude", 18.22793573141098}, {"areaLongitude", 109.5083874464035}},
            {{"areaLatitude", 18.229333497583866}, {"areaLongitude", 109.50756124220788}},
            {{"areaLatitude", 18.23010521940887}, {"areaLongitude", 109.50530558824539}},
            {{"areaLatitude", 18.231705240905285}, {"areaLongitude", 109.5043103210628}},
            {{"areaLatitude", 18.23327978141606}, {"areaLongitude", 109.50546375475824}},
            {{"areaLatitude", 18.234245125204325}, {"areaLongitude", 109.50616112910211}},
            {{"areaLatitude", 18.234555842354894}, {"areaLongitude", 109.50537264347076}},
            {{"areaLatitude", 18.233626037836075}, {"areaLongitude", 109.50450897216797}},
            {{"areaLatitude", 18.23270369321108}, {"areaLongitude", 109.50335285626352}},
            {{"areaLatitude", 18.23292807675898}, {"areaLongitude", 109.50221291743219}},
            {{"areaLatitude", 18.236055616289377}, {"areaLongitude", 109.49681865982711}},
            {{"areaLatitude", 18.238033577799797}, {"areaLongitude", 109.49774988926947}},
            {{"areaLatitude", 18.23999016545713}, {"areaLongitude", 109.49729927815497}},
            {{"areaLatitude", 18.238604050129652}, {"areaLongitude", 109.49476718902588}},
            {{"areaLatitude", 18.238746458664536}, {"areaLongitude", 109.4930290337652}},
            {{"areaLatitude", 18.23976586572826}, {"areaLongitude", 109.49217072688043}},
            {{"areaLatitude", 18.23913403786719}, {"areaLongitude", 109.48933839797974}},
            {{"areaLatitude", 18.237401833757758}, {"areaLongitude", 109.48620557785034}},
            {{"areaLatitude", 18.234344786033034}, {"areaLongitude", 109.4803261756897}},
            {{"areaLatitude", 18.230676194652915}, {"areaLongitude", 109.47382441721857}},
            {{"areaLatitude", 18.23053085245192}, {"areaLongitude", 109.471555352211}},
            {{"areaLatitude", 18.23393457569182}, {"areaLongitude", 109.46838774718344}},
            {{"areaLatitude", 18.235587067902088}, {"areaLongitude", 109.46586923673749}}}},
          {"polygonCount", 37},
          {"type", 1}}},
        {"head", {{"packageSeq", 10251}, {"packageType", 0}, {"time", "2025-08-13 23:19:45.620"}}}};

    while (true)
    {
        std::string payload = msg_json.dump();
        mqtt::message_ptr pubmsg = mqtt::make_message(TOPIC, payload);
        pubmsg->set_qos(1);

        try
        {
            client.publish(pubmsg)->wait_for(std::chrono::seconds(1));
            std::cout << "Published 0042 message." << std::endl;
        }
        catch (const mqtt::exception &exc)
        {
            std::cerr << "Error publishing: " << exc.what() << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    client.disconnect()->wait();
    return 0;
}
