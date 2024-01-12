#include <cstdint>
#include <string>
#include <vector>
#include <cstring>
#include <netinet/in.h>
#include <stdexcept>
#include <thread>
#include <nonstd/decimal.h>
#pragma once

using decimal31 = nonstd::decimal<int16_t, 3, 1>;

enum command_id_t {
  Heartbeat = 0x00,
  Acquire_Firmware_Version = 0x01,
  Acquire_Hardware_ID = 0x02,
  Auto_Focus = 0x04,
  Manual_Zoom_and_Auto_Focus = 0x05,
  Manual_Focus = 0x06,
  Gimbal_Rotation = 0x07,
  Center_the_Gimbal = 0x08,
  Acquire_Gimbal_Configuration_Information = 0x0A,
  Function_Feedback_Information = 0x0B,
  Photo_and_Video = 0x0C,
  Acquire_Gimbal_Attitude = 0x0D,
  Set_Gimbal_Control_Angle = 0x0E,
  Absolute_Zoom_and_Auto_Focus = 0x0F,
  Acquire_Camera_Image_Type = 0x10,
  Set_Camera_Image_Type = 0x11,
  Read_Temperature_of_a_Point = 0x12,
  Read_Temperature_of_a_Box_on_Screen = 0x13,
  Read_Temperature_of_the_Full_Screen = 0x14,
  Read_Range_from_Laser_Rangefinder = 0x15,
  Acquire_Max_Zoom = 0x16
};
enum gimbal_motion_mode_t {
  LOCK_MODE = 0,
  FOLLOW_MODE = 1,
  FPV_MODE = 2
};

enum gimbal_mounting_dir_t {
  MOUNTING_DIR_UNDEFINED= 0,
  MOUNTING_DIR_NORMAL= 1,
  MOUNTING_DIR_UPSIDE_DOWN = 2
};

std::string to_string(gimbal_motion_mode_t mode);
std::string to_string(gimbal_mounting_dir_t mode);

uint16_t CRC16_cal(uint8_t *ptr, uint32_t len, uint16_t crc_init);

//uint8_t crc_check_16bites(uint8_t* pbuf, uint32_t len,uint32_t* p_result);
//std::vector<uint8_t> hex_to_bytes(const std::string& hex);
std::vector<uint8_t> encode(uint8_t cmd_id, const std::vector<uint8_t>& data, uint16_t seq, bool need_ack);

bool verify_crc(const std::vector<uint8_t>& packet);

//convinience functions
uint16_t get_seq(const std::vector<uint8_t>& packet);
size_t get_first_packet_length(const std::vector<uint8_t>& packet);
command_id_t get_cmd_id(const std::vector<uint8_t>& packet);
std::vector<uint8_t> get_data(const std::vector<uint8_t>& packet);

struct FirmwareVersions {
  std::string camera_firmware_ver;
  std::string gimbal_firmware_ver;
  std::string zoom_firmware_ver;
};

struct HardwareID {
  std::string id;
};

/*
struct GimbalAttitude0 {
  int16_t yaw;
  int16_t pitch;
  int16_t roll;
  int16_t yaw_velocity;
  int16_t pitch_velocity;
  int16_t roll_velocity;
};
*/

struct GimbalAttitude {
  decimal31 yaw;
  decimal31 pitch;
  decimal31 roll;
  decimal31 yaw_velocity;
  decimal31 pitch_velocity;
  decimal31 roll_velocity;
};

struct GimbalConfirmation {
  uint8_t reserved0;
  uint8_t hdr_sta;
  uint8_t reserved1;
  uint8_t record_sta;
  gimbal_motion_mode_t gimbal_motion_mode;
  gimbal_mounting_dir_t gimbal_mounting_dir;
  uint8_t video_hdmi_or_cvbs;
};

struct GimbalRotation {
  int8_t turn_yaw;
  /* -100~0~100: Negative and positive represent two directions,
  higher or lower the number is away from
  0, faster the rotation speed is.
  Send 0 when released from
  control command and gimbal
  stops rotation.
  */
  int8_t turn_pitch; /* same as turn_yaw */
};


struct GimbalControlAngle {
  decimal31 yaw;
  decimal31 pitch;
};

struct GimbalCurrentAngles {
  decimal31 yaw;
  decimal31 pitch;
  decimal31 roll;
};

struct ZoomAndAutoFocusCommand {
  int8_t zoom;            // 1 for zooming in, 0 for stop, -1 for zooming out
};

struct ZoomMultipleAck {
  uint16_t zoom_multiple; // Current zoom multiple
};

struct MaxZoomValue {
  uint8_t zoom_max_int;   // Integer part of zoom multiple (0x01 to 0x1E)
  uint8_t zoom_max_float; // Fractional part of zoom multiple (0x00 to 0x09)
};

struct AbsoluteZoomAndAutoFocusCommand {
  uint8_t absolute_movement_int;   // Integer part of target zoom multiple (0x01 to 0x1E)
  uint8_t absolute_movement_float; // Fractional part of target zoom multiple (0x00 to 0x09)
};

std::string to_string(command_id_t cmd_id);

inline std::vector<uint8_t> make_req_heartbeat_packet(uint16_t seq, uint8_t payload) {
  return encode(Heartbeat, {payload}, seq, true);
}

inline std::vector<uint8_t> make_get_firmware_version_packet(uint16_t seq) {
  return encode(Acquire_Firmware_Version, {}, seq, true);
}

inline std::vector<uint8_t> make_get_hardware_id_packet(uint16_t seq) {
  return encode(Acquire_Hardware_ID, {} , seq, true);
}

inline std::vector<uint8_t> make_set_auto_focus_packet(uint16_t seq) {
  return encode(Auto_Focus, {1}, seq, true);
}

inline std::vector<uint8_t> make_set_manual_zoom_and_auto_focus_packet(uint16_t seq, const ZoomAndAutoFocusCommand& command) {
  return encode(Manual_Zoom_and_Auto_Focus, {(uint8_t) command.zoom}, seq, true);
}

inline std::vector<uint8_t> make_set_absolute_zoom_and_auto_focus_packet(uint16_t seq, const AbsoluteZoomAndAutoFocusCommand& command) {
  return encode(Absolute_Zoom_and_Auto_Focus, {command.absolute_movement_int, command.absolute_movement_float}, seq, true);
}

inline std::vector<uint8_t> make_acquire_max_zoom_packet(uint16_t seq) {
  return encode(Acquire_Max_Zoom, {}, seq, true);
}

//1: Long shot, 0 Stop manual focus, -1 Short shot todo fix enum
inline std::vector<uint8_t> make_manual_focus_packet(uint16_t seq, int8_t focus) {
  return encode(Manual_Focus, {(uint8_t) focus}, seq, true);
}

inline std::vector<uint8_t> make_gimbal_rotation_packet(uint16_t seq, GimbalRotation rotation) {
  return encode(Gimbal_Rotation, {(uint8_t) rotation.turn_yaw, (uint8_t) rotation.turn_pitch}, seq, true);
}

inline std::vector<uint8_t> make_center_packet(uint16_t seq) {
  return encode(Center_the_Gimbal, {1}, seq, true);
}

inline std::vector<uint8_t> make_acquire_gimbal_configuration_information_packet(uint16_t seq) {
  return encode(Acquire_Gimbal_Configuration_Information, {}, seq, true);
}

// addon
inline std::vector<uint8_t> make_set_gimbal_motion_mode_packet(uint16_t seq, gimbal_motion_mode_t mode) {
  switch (mode) {
    case LOCK_MODE:
      return encode(Photo_and_Video, {3}, seq, false);
    case FOLLOW_MODE:
      return encode(Photo_and_Video, {4}, seq, false);
    case FPV_MODE:
      return encode(Photo_and_Video, {5}, seq, false);
    default:
      throw std::runtime_error("Invalid gimbal motion mode");
  }
}

inline std::vector<uint8_t> make_photo_and_video_packet_internal(uint16_t seq, uint8_t func_type) {
  return encode(Photo_and_Video, {func_type}, seq, false);
}

inline std::vector<uint8_t> make_acquire_gimbal_attitude_packet(uint16_t seq) {
  return encode(Acquire_Gimbal_Attitude, {}, seq, true);
}

inline std::vector<uint8_t> make_set_gimbal_control_angle_packet(uint16_t seq, const GimbalControlAngle& angles) {
  std::vector<uint8_t> data(sizeof(GimbalControlAngle));
  // this does not look like ntohs is needed
  //int16_t yaw_network = htons(angles.yaw);
  //int16_t pitch_network = htons(angles.pitch);

  int16_t yaw_network = (int16_t) (double) (angles.yaw * 10.00);
  int16_t pitch_network = (int16_t) (double) angles.pitch * 10.0;

  memcpy(&data[0], &yaw_network, sizeof(yaw_network));
  memcpy(&data[2], &pitch_network, sizeof(pitch_network));
  // Encode the packet
  return encode(Set_Gimbal_Control_Angle, data, seq, true);
}



FirmwareVersions parse_firmware_versions_ack(const std::vector<uint8_t>& packet);
HardwareID parse_hardware_id_ack(const std::vector<uint8_t>& packet);

GimbalAttitude parse_gimbal_attitude(const std::vector<uint8_t>& packet);
GimbalCurrentAngles parse_gimbal_control_angle_ack(const std::vector<uint8_t>& packet);
ZoomMultipleAck parse_zoom_multiple_ack(const std::vector<uint8_t>& packet);
bool parse_auto_focus_ack(const std::vector<uint8_t>& packet);
MaxZoomValue parse_acquire_max_zoom_ack(const std::vector<uint8_t>& packet);
GimbalConfirmation parse_gimbal_configuration_ack(const std::vector<uint8_t>& packet);




class siyi_connection {
public:
  siyi_connection(std::string ip, uint16_t port);
  ~siyi_connection();
  bool send_packet(const std::vector<uint8_t>& packet);
private:
  std::vector<uint8_t> recv();
  void handle_packet(const std::vector<uint8_t>& packet);
  void recv_thread();
  bool start_ = false;
  bool exit_ = false;
  std::thread thread_;
  int socket_;
  struct sockaddr_in send_addr_;
  struct sockaddr_in recv_addr_;
};



