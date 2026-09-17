/**
ProVer-Cert Source Code

Copyright 2026 Carnegie Mellon University.

NO WARRANTY. THIS CARNEGIE MELLON UNIVERSITY AND SOFTWARE ENGINEERING INSTITUTE MATERIAL IS FURNISHED ON AN "AS-IS" BASIS. CARNEGIE MELLON UNIVERSITY MAKES NO WARRANTIES OF ANY KIND, EITHER EXPRESSED OR IMPLIED, AS TO ANY MATTER INCLUDING, BUT NOT LIMITED TO, WARRANTY OF FITNESS FOR PURPOSE OR MERCHANTABILITY, EXCLUSIVITY, OR RESULTS OBTAINED FROM USE OF THE MATERIAL. CARNEGIE MELLON UNIVERSITY DOES NOT MAKE ANY WARRANTY OF ANY KIND WITH RESPECT TO FREEDOM FROM PATENT, TRADEMARK, OR COPYRIGHT INFRINGEMENT.

Licensed under a BSD (SEI)-style license, please see license.txt or contact permission@sei.cmu.edu for full terms.

[DISTRIBUTION STATEMENT A] This material has been approved for public release and unlimited distribution.  Please see Copyright notice for non-US Government use and distribution.

This Software includes and/or makes use of Third-Party Software each subject to its own license.

DM26-0076
*/

//
// Demonstrates how to add and fly Waypoint missions using the MAVSDK.
//

#include <mavsdk/mavsdk.hpp>
#include <mavsdk/plugins/shell/shell.hpp>
#include <mavsdk/plugins/action/action.hpp>
#include <mavsdk/plugins/mission/mission.hpp>
#include <mavsdk/plugins/telemetry/telemetry.hpp>
#include <mavsdk/plugins/param/param.hpp>


#include <chrono>
#include <functional>
#include <future>
#include <iostream>
#include <thread>
#include <cstring>

#include <cmath>
#include <iomanip>

#include <random>

#include <csignal>
#include <atomic>
#include <unistd.h>



using namespace mavsdk;
using std::chrono::seconds;
using std::this_thread::sleep_for;

std::atomic<bool> keepRunning(true);

// The signal handler function executed when Ctrl-C is pressed
void signalHandler(int signum) {
    std::cout << "\n[Signal] Captured Ctrl-C (Signal number: " << signum << ")\n";
    
    // Set flag to false to break out of any processing loops safely
    keepRunning = false; 
}


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct Point {
    double lat;
    double lon;
};

// Helper to convert degrees to radians
double toRadians(double degree) {
    return degree * (M_PI / 180.0);
}

// Earth's mean radius in meters
const double EARTH_RADIUS = 6371000.0; 

Point project_point(double startLat, double startLon, double distance, double bearingDeg) {
    // 1. Convert degrees to radians
    double lat1 = startLat * M_PI / 180.0;
    double lon1 = startLon * M_PI / 180.0;
    double brng = bearingDeg * M_PI / 180.0;
    double d_r = distance / EARTH_RADIUS; // Angular distance

    // 2. Calculate new latitude
    double lat2 = asin(sin(lat1) * cos(d_r) + 
                       cos(lat1) * sin(d_r) * cos(brng));

    // 3. Calculate new longitude
    double lon2 = lon1 + atan2(sin(brng) * sin(d_r) * cos(lat1),
                               cos(d_r) - sin(lat1) * sin(lat2));

    // 4. Convert back to degrees and normalize longitude
    Point result;
    result.lat = lat2 * 180.0 / M_PI;
    result.lon = fmod((lon2 * 180.0 / M_PI) + 540.0, 360.0) - 180.0;

    return result;
}

/**
 * Calculates distance between two points in meters
 * @param lat1, lon1: Coordinates of first point (degrees)
 * @param lat2, lon2: Coordinates of second point (degrees)
 */
double haversineDistance(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371000.0; // Earth's radius in meters

    double phi1 = toRadians(lat1);
    double phi2 = toRadians(lat2);
    double deltaPhi = toRadians(lat2 - lat1);
    double deltaLambda = toRadians(lon2 - lon1);

    double a = std::pow(std::sin(deltaPhi / 2.0), 2) +
               std::cos(phi1) * std::cos(phi2) *
               std::pow(std::sin(deltaLambda / 2.0), 2);
    
    double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));

    return R * c;
}

Mission::MissionItem make_mission_item(
    double latitude_deg,
    double longitude_deg,
    float relative_altitude_m,
    float speed_m_s,
    bool is_fly_through,
    float gimbal_pitch_deg,
    float gimbal_yaw_deg,
    Mission::MissionItem::CameraAction camera_action)
{
    Mission::MissionItem new_item{};
    new_item.latitude_deg = latitude_deg;
    new_item.longitude_deg = longitude_deg;
    new_item.relative_altitude_m = relative_altitude_m;
    new_item.speed_m_s = speed_m_s;
    new_item.is_fly_through = is_fly_through;
    new_item.gimbal_pitch_deg = gimbal_pitch_deg;
    new_item.gimbal_yaw_deg = gimbal_yaw_deg;
    new_item.camera_action = camera_action;
    return new_item;
}

void usage(const std::string& bin_name)
{
    std::cerr << "Usage : " << bin_name << " <connection_url>  [missionrounds=<int> roundlength=<int> addlanding=y maxturnshift=<degrees>]\n"
              << "Connection URL format should be :\n"
              << " For TCP server: tcpin://<our_ip>:<port>\n"
              << " For TCP client: tcpout://<remote_ip>:<port>\n"
              << " For UDP server: udpin://<our_ip>:<port>\n"
              << " For UDP client: udpout://<remote_ip>:<port>\n"
              << " For Serial : serial://</path/to/serial/dev>:<baudrate>]\n"
              << "For example, to connect to the simulator use URL: udpin://0.0.0.0:14540\n";
}

struct params_t {
    int missionrounds;
    int roundlength;
    bool addlanding;
    int maxturnshift;
    bool shutdown;
} params;

int main(int argc, char** argv)
{
    params.missionrounds=1;
    params.roundlength=10;
    params.addlanding = false;
    params.maxturnshift = 30;
    params.shutdown = false;

    if (argc < 2) {
        usage(argv[0]);
        return 1;
    }

    if (argc >3){
        for (int i=2 ; i<argc; i++){
            char *name = std::strtok(argv[i],"=") ;
            char *value = std::strtok(nullptr,"=");
            if (value != nullptr && name != nullptr){
                if (std::strcmp(name,"missionrounds")==0){
                    params.missionrounds = atoi(value);
                } else if (std::strcmp(name,"roundlength")==0){
                    params.roundlength = atoi(value);
                } else if (std::strcmp(name,"addlanding")==0){
                    if (value[0] == 'y' || value[0] == 'Y'){
                        params.addlanding = true;
                    }
                } else if (std::strcmp(name,"maxturnshift")==0){
                    params.maxturnshift = atoi(value);
                } else if (std::strcmp(name,"shutdown")==0){
                    params.shutdown= true;
                }
            }
        }
    }

    // Setup the Ctl-C interrupt handler
    struct sigaction sigIntHandler;

    // Direct incoming SIGINT signals to our custom handler function
    sigIntHandler.sa_handler = signalHandler;
    
    // Clear any active masking options during the signal handling process
    sigemptyset(&sigIntHandler.sa_mask);
    
    // Set option flags to 0 for standard signal response behavior
    sigIntHandler.sa_flags = 0;

    // Bind our configuration to the SIGINT (Ctrl-C) event
    if (sigaction(SIGINT, &sigIntHandler, nullptr) == -1) {
        std::cerr << "Error registering signal handler." << std::endl;
        return 1;
    }

    Mavsdk mavsdk{Mavsdk::Configuration{ComponentType::GroundStation}};
    ConnectionResult connection_result = mavsdk.add_any_connection(argv[1]);

    if (connection_result != ConnectionResult::Success) {
        std::cerr << "Connection failed: " << connection_result << '\n';
        return 1;
    }

    auto system = mavsdk.first_autopilot(3.0);
    if (!system) {
        std::cerr << "Timed out waiting for system\n";
        return 1;
    }

    auto action = Action{system.value()};
    auto mission = Mission{system.value()};
    auto telemetry = Telemetry{system.value()};

    // reduce workload of telemetry
    telemetry.set_rate_position(1.0);
    telemetry.set_rate_imu(1.0);

    auto shell = Shell{system.value()};

    // disable the max first waypoint warning
    auto param = Param(system.value());
    Param::Result result = param.set_param_float("MIS_DIST_1WP",0);

    if (result == Param::Result::Success) {
        std::cout << "Successfully updated MIS_DIST_1WP !\n";
    } else {
        std::cerr << "Failed to set parameter: " << result << '\n';
    }

    // disable battery draining
    result = param.set_param_float("SIM_BAT_DRAIN",0);

    if (result == Param::Result::Success) {
        std::cout << "Successfully disabled battery draining !\n";
    } else {
        std::cerr << "Failed to set parameter: " << result << '\n';
    }

    // Disable the magnetometer gate breaker to void errors like "magnetic interference"
    result = param.set_param_int("COM_ARM_MAG_STR",0);

    if (result == Param::Result::Success) {
        std::cout << "Successfully disabled magnetometer breaker \n";
    } else {
        std::cerr << "Failed to set parameter: " << result << '\n';
    }

    result = param.set_param_int("NAV_DLL_ACT",0);

    if (result == Param::Result::Success) {
        std::cout << "Successfully disabled waiting for ground control station \n";
    } else {
        std::cerr << "Failed to set parameter: " << result << '\n';
    }

    // ALLOW ARMING WITHOUT GPS
    // result = param.set_param_int("COM_ARM_WO_GPS",1);

    // if (result == Param::Result::Success) {
    //     std::cout << "Successfully enabled arming without GPS \n";
    // } else {
    //     std::cerr << "Failed to set parameter: " << result << '\n';
    // }


    int i=0;
    while (!telemetry.health_all_ok() && i<5) {
    //while (!telemetry.health().is_armable && i <10){
        std::cout << "Waiting for system to be ready\n";
        sleep_for(seconds(1));
        i++;
    }

    mavsdk::Telemetry::Position initial_position; 

    if (telemetry.health_all_ok()){
    //if (telemetry.health().is_armable){
        auto origin_result = telemetry.get_gps_global_origin();
        if (origin_result.first == Telemetry::Result::Success) {
            std::cout << "Initial GPS origin: " << origin_result.second << '\n';
        }

        // Get initial position
        initial_position = telemetry.position();
    } else {
        // Set a default position
        initial_position.latitude_deg =  47.398170327054473;
        initial_position.longitude_deg = 8.5456490218639658;
    }




    // // Define the desired delay to loitering (-1)
    // const std::string param_name = "RTL_DESCEND_DELAY";
    // const float delay_seconds = -1.0f;
    // Set the parameter (this is a blocking call)
    // Param::Result result = param.set_param_float(param_name, delay_seconds);

    std::cout << "System ready\n";
    std::cout << "Creating and uploading mission\n";

    std::vector<Mission::MissionItem> mission_items;

    std::random_device rd; 
    std::mt19937 gen(rd()); 
    std::uniform_real_distribution<double> dis(0.0, 1.0);
    std::uniform_int_distribution<int> disbin(0,1); 
    //double random_val = dis(gen);


    Point prevPoint,nextPoint;

    prevPoint.lat = initial_position.latitude_deg;
    prevPoint.lon = initial_position.longitude_deg;

    double direction = 90.0;// due east
    double difDirection=0.0;
    int leftrightturn=1; // 1 = left, -1 = right

    double minDirection = 0.0;
    double maxDirection = 360.0;

    std::cout << "Commanding takeoff...\n";
    const Action::Result takeoff_result = action.takeoff(); 
    if (takeoff_result != Action::Result::Success) {
        std::cout << "Failed to command takeoff: " << takeoff_result << '\n';
        std::cout << "\tThe program will exit after resetting parameters. Try again after that. \n\tIf this fails:\n";
        std::cout << "\tTry Running QGroundControl and \n";
        std::cout << "\tgo to Configure->Parameters->SITL\n";
        std::cout << "\tPress the 'Tools' button  and select\n";
        std::cout << "\t'reset all to firmware defaults'\n";
        shell.send("param reset_all");
        sleep_for(seconds(1));
        shell.send("param save");
        sleep_for(seconds(1));
        action.reboot();
        return -1;
    }
    std::cout << "Commanded takeoff.\n";

    bool first_time=true;


    for (int j=0;j<params.missionrounds && keepRunning;j++){

        std::cout << "Mission round: " << j <<"\n";

        mission_items.clear();

        // try to keep it close to the launch point
        // if ((j % 2) == 0){
        //     minDirection = 0.0;
        //     maxDirection = 180;
        // } else {
        //     minDirection = 180.0;
        //     maxDirection = 360.0;
        // }

        if ((j%2) == 0){
            direction = 90.0; //due east
        } else {
            direction = 270.0; // due west
        }

        for (int i=0;i<params.roundlength && keepRunning;i++){

            // to generate a Browninan motion (random walk) we generate the direction
            // based on the uniform distribution
            leftrightturn = disbin(gen);
            difDirection = (params.maxturnshift * dis(gen));
            difDirection = (leftrightturn == 0 ? -1 : 1) * difDirection;
            direction = direction + difDirection; //((maxDirection - minDirection) * dis(gen)) + minDirection;
            if (direction > 360) {
                direction = direction - 360.0;
            } else if (direction <0){
                direction=direction * -1;
            }

            auto item = make_mission_item(
                prevPoint.lat,
                prevPoint.lon,
                10.0f,
                5.0f,
                true,//false, // fly through
                20.0f,
                60.0f,
                Mission::MissionItem::CameraAction::None);

            // add landing to the last item if requested
            if (i == (params.roundlength -1)  && j == (params.missionrounds-1) && params.addlanding){
                item.vehicle_action = Mission::MissionItem::VehicleAction::Land;
                std::cout<<"Added landing to mission\n";
            }

            mission_items.push_back(item);

            nextPoint = project_point(
                prevPoint.lat, 
                prevPoint.lon, 
                20.0 , // meter forward
                direction
            );
            prevPoint = nextPoint;
        }

    // repeat mission multiple times 
    // we should later generate multiple missions and execute them as well
        std::cout << "Uploading mission...\n";
        Mission::MissionPlan mission_plan{};
        mission_plan.mission_items = mission_items;

        const Mission::Result upload_result = mission.upload_mission(mission_plan);

        if (upload_result != Mission::Result::Success) {
            std::cerr << "Mission upload failed: " << upload_result << ", exiting.\n";
            return -1;
        }

        if (first_time){
            first_time = false;
            std::cout << "Arming...\n";
            const Action::Result arm_result = action.arm();
            if (arm_result != Action::Result::Success) {
                std::cerr << "Arming failed: " << arm_result << '\n';
                return -1;
            }
            std::cout << "Armed.\n";
        //}
        
            std::atomic<bool> want_to_pause{false};
            // Before starting the mission, we want to be sure to subscribe to the mission progress.
            mission.subscribe_mission_progress([&want_to_pause](Mission::MissionProgress mission_progress) {
                std::cout << "Mission status update: " << mission_progress.current << " / "
                        << mission_progress.total << '\n';
            });

        }

        Mission::Result start_mission_result = mission.start_mission();
        if (start_mission_result != Mission::Result::Success) {
            std::cerr << "Starting mission failed: " << start_mission_result << '\n';
            return -1;
        }

        while (!mission.is_mission_finished().second) {
            if (!keepRunning){
                mission.pause_mission();
                action.land();
                action.disarm();
                break;
            }
            sleep_for(seconds(1));
        }

        // we will see if clearing the mission allow us to run 
        // for longer
        // This does not seem to work because it switches out of mission mode
        //mission.clear_mission();


        // TRY TO SEE IF WE CAN AVOID GOING TO THE BASE POSITION. THIS IS POLLUTING THE RESULTS.
        // We will return to the launch position if we cannot control the direction in the random walk
        //
        // Action::Result return_to_first = action.goto_location(initial_position.latitude_deg, initial_position.longitude_deg,10.0,0);
        // if (return_to_first != Action::Result::Success){
        //     std::cout << "Failed to return to first location "<< return_to_first << "\n";
        // }

        // Telemetry::Position pos = telemetry.position();
        // while (haversineDistance(pos.latitude_deg, pos.longitude_deg, initial_position.latitude_deg,initial_position.longitude_deg) > 2.0 && keepRunning){
        //     sleep_for(seconds(1));
        //     pos = telemetry.position();
        // }

        // prevPoint.lat = initial_position.latitude_deg;
        // prevPoint.lon = initial_position.longitude_deg;

    }

    // Done. Land in place
    std::cout << "Commanding Land...\n";
    Action::Result rtl_result = action.return_to_launch(); //action.land(); 
    if (rtl_result != Action::Result::Success) {
        std::cout << "Failed to command return to launch: " << rtl_result << '\n';
        return -1;
    }
    std::cout << "Commanded Land.\n";

    // We need to wait a bit, otherwise the armed state might not be correct yet.
    sleep_for(seconds(2));

    while(telemetry.landed_state() != Telemetry::LandedState::OnGround && keepRunning){
        sleep_for(seconds(1));
    }

    sleep_for(seconds(2));

    rtl_result = action.disarm();

    if (rtl_result != Action::Result::Success) {
        std::cout << "Failed to disarm: " << rtl_result << '\n';
    }

    while (telemetry.armed() && keepRunning) {
        // Wait until we're done.
        sleep_for(seconds(1));
    }
    std::cout << "Disarmed, clearing mission\n";

    //mission.clear_mission();

    sleep_for(seconds(1));

    if (params.shutdown){
        //shell.send("shutdown");
        std::cout << "Shutting down drone\n";
        action.shutdown();
        sleep_for(seconds(1));
    }

    std::cout << "Mission cleared. Existing\n";

}
