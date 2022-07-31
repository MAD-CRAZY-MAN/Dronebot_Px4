//
//
// Author: Sihyun Noh <awake_noh@thepeach.kr>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <future>
#include <memory>
#include <thread>
#include <string>

//opencv
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

using namespace mavsdk;
using std::chrono::seconds;

static bool start_capture = false;

void usage(const std::string& bin_name);
static void process_command_long(const mavlink_message_t& message, uint8_t our_sysid);

std::shared_ptr<System> get_system(Mavsdk& mavsdk)
{
    std::cout << "Waiting to discover system...\n";
    auto prom = std::promise<std::shared_ptr<System>>{};
    auto fut = prom.get_future();

    // We wait for new systems to be discovered, once we find one that has an
    // autopilot, we decide to use it.
    Mavsdk::NewSystemHandle handle = mavsdk.subscribe_on_new_system([&mavsdk, &prom, &handle]() {
        auto system = mavsdk.systems().back();

        if (system->has_autopilot()) {
            std::cout << "Discovered autopilot\n";

            // Unsubscribe again as we only want to find one system.
            mavsdk.unsubscribe_on_new_system(handle);
            prom.set_value(system);
        }
    });

    // We usually receive heartbeats at 1Hz, therefore we should find a
    // system after around 3 seconds max, surely.
    if (fut.wait_for(seconds(3)) == std::future_status::timeout) {
        std::cerr << "No autopilot found.\n";
        return {};
    }

    // Get discovered system now.
    return fut.get();
}

int main(int argc, char** argv)
{
    if (argc != 2) {
        usage(argv[0]);
        return 1;
    }

    Mavsdk mavsdk;
    ConnectionResult connection_result = mavsdk.add_any_connection(argv[1]);

    if (connection_result != ConnectionResult::Success) {
        std::cerr << "Connection failed: " << connection_result << '\n';
        return 1;
    }

    auto system = get_system(mavsdk);
    if (!system) {
        return 1;
    }
    uint8_t our_sysid = system->get_system_id(); 

    auto mavlink_passthrough = MavlinkPassthrough{system};

    mavlink_passthrough.subscribe_message(
        MAVLINK_MSG_ID_COMMAND_LONG, [&our_sysid](const mavlink_message_t& message) {
            process_command_long(message, our_sysid);
        });

    VideoCapture cap1(0);
    if(!cap1.isOpened())
        std::cout <<"fail" << std::endl;
    Mat image;
    int image_cnt = 0;
    string image_name;

    while (true) {
        if(start_capture) {
            cap1 >> image;
            image_name = to_string(image_cnt) + ".jpg";
            
            imwrite(image_name, image);
            start_capture = false;
            image_cnt++;
        }
        //std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}

static void process_command_long(const mavlink_message_t& message, uint8_t our_sysid)
{
    mavlink_command_long_t command_long;
    mavlink_msg_command_long_decode(&message, &command_long);

    // Only listen to image_start_capture commands.
    if (command_long.command != MAV_CMD_IMAGE_START_CAPTURE) {
        return;
    }

    // Check if it's meant for us.
    if (command_long.target_system != 0 && command_long.target_system != our_sysid) {
        return;
    }

    if (command_long.target_component != 0 &&
        command_long.target_component != 100) {
        return;
    }

    if(command_long.param3 == 1) {
        start_capture = true;
        std::cout << "take one shot" << std::endl;
    }
    else if (command_long.param3 == 0) {
        return;
    }
}

void usage(const std::string& bin_name)
{
    std::cerr << "Usage : " << bin_name << " <connection_url>\n"
              << "Connection URL format should be :\n"
              << " For TCP : tcp://[server_host][:server_port]\n"
              << " For UDP : udp://[bind_host][:bind_port]\n"
              << " For Serial : serial:///path/to/serial/dev[:baudrate]\n"
              << "For example, to connect to the simulator use URL: udp://:14540\n";
}
