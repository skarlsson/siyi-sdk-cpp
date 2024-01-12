#include <siyi-sdk-cpp/siyi.h>
#include <chrono>
#include <glog/logging.h>

using namespace std::chrono_literals;
#define SERVER_PORT 37260
#define SERVER_IP "192.168.144.25"

int main(int argc, char *argv[]) {
  siyi_connection gimbal(SERVER_IP, SERVER_PORT);
  std::this_thread::sleep_for(1s);
  uint16_t seq=0;
  //gimbal.send_packet(make_req_heartbeat_packet(seq++, 1));
  //std::this_thread::sleep_for(100ms);
  gimbal.send_packet(make_get_firmware_version_packet(seq++));
  std::this_thread::sleep_for(1000ms);
  gimbal.send_packet(make_get_hardware_id_packet(seq++));
  std::this_thread::sleep_for(1000ms);
  gimbal.send_packet(make_acquire_max_zoom_packet(seq++));


  //Gimbal Rotation
  //gimbal.send_packet(make_gi_packet(seq++));

  //Center
  //gimbal.send_packet(make__packet(seq++));


  //Acquire Gimbal Configuration Information
  std::this_thread::sleep_for(1000ms);
  gimbal.send_packet(make_acquire_gimbal_configuration_information_packet(seq++));


  /*
   3: Motion – Lock Mode
   4: Motion – Follow Mode
   5: Motion – FPV Mode
   */
  std::this_thread::sleep_for(1000ms);

  /*
  LOG(INFO) << "Set gimbal mode to LOCK_MODE";
  gimbal.send_packet(make_set_gimbal_motion_mode_packet(seq, LOCK_MODE));
  std::this_thread::sleep_for(2s);
  gimbal.send_packet(make_acquire_gimbal_configuration_information_packet(seq++));
  std::this_thread::sleep_for(2s);

  LOG(INFO) << "Set gimbal mode to FOLLOW_MODE";
  gimbal.send_packet(make_set_gimbal_motion_mode_packet(seq, FOLLOW_MODE));
  std::this_thread::sleep_for(2s);
  gimbal.send_packet(make_acquire_gimbal_configuration_information_packet(seq++));
  std::this_thread::sleep_for(2s);

  LOG(INFO) << "Set gimbal mode to FPV_MODE";
  gimbal.send_packet(make_set_gimbal_motion_mode_packet(seq, FPV_MODE));
  std::this_thread::sleep_for(2s);
  gimbal.send_packet(make_acquire_gimbal_configuration_information_packet(seq++));
  std::this_thread::sleep_for(2s);
  */

  LOG(INFO) << "Set gimbal mode to LOCK_MODE";
  gimbal.send_packet(make_set_gimbal_motion_mode_packet(seq, FPV_MODE));
  std::this_thread::sleep_for(2s);
  gimbal.send_packet(make_acquire_gimbal_configuration_information_packet(seq++));
  std::this_thread::sleep_for(2s);

  // rotate the gimbal from left to right
  std::vector<int16_t> vyaw = { -100, -45, 0, 45, 100 };
  std::vector<int16_t> vpitch =  { -90, -45, -25, 0, 25 };

  // loop over those positions and wait 2s for each step
  /*
  for (auto yaw : vyaw) {
    for (auto pitch : vpitch) {
      LOG(INFO) << "Set gimbal angle to yaw: " << yaw << " pitch: " << pitch;
      gimbal.send_packet(make_set_gimbal_control_angle_packet(seq++, {yaw, pitch}));
      std::this_thread::sleep_for(2s);
    }
  }
  */

  // rotate the gimbal from left to right
  {
  std::vector<double> vyaw = { -100.0, -45.0, 0.0, 45.0, 100.0 };
  std::vector<double> vpitch =  { -30 };

  // loop over those positions and wait 2s for each step
  for (auto yaw : vyaw) {
    for (auto pitch : vpitch) {
      LOG(INFO) << "Set gimbal angle to yaw: " << yaw << " pitch: " << pitch;
      gimbal.send_packet(make_set_gimbal_control_angle_packet(seq++, {yaw, pitch}));
      for (auto i = 0; i < 10; i++) {
        gimbal.send_packet(make_acquire_gimbal_attitude_packet(seq++));
        std::this_thread::sleep_for(1000ms);
      }
    }
  }
  }

  LOG(INFO) << "center the gimbal";
  gimbal.send_packet(make_center_packet(seq++));

  while(true) {
    gimbal.send_packet(make_acquire_gimbal_attitude_packet(seq++));
    std::this_thread::sleep_for(1000ms);
  }

  //std::this_thread::sleep_for(100ms);
  //gimbal.send_packet(make_set_gimbal_control_angle_packet(seq++, {0, 0}));
}
