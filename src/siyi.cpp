#include <siyi-sdk-cpp/siyi.h>

#include <cstring>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <glog/logging.h>
#include <vector>
#include <cstdint>

#include <chrono>
using namespace std::chrono_literals;

static const uint16_t crc16_tab[256] = {
  0x0, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
  0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
  0x1231, 0x210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
  0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
  0x2462, 0x3443, 0x420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
  0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
  0x3653, 0x2672, 0x1611, 0x630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
  0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
  0x48c4, 0x58e5, 0x6886, 0x78a7, 0x840, 0x1861, 0x2802, 0x3823,
  0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
  0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0xa50, 0x3a33, 0x2a12,
  0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
  0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0xc60, 0x1c41,
  0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
  0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0xe70,
  0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
  0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
  0x1080, 0xa1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
  0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
  0x2b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
  0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
  0x34e2, 0x24c3, 0x14a0, 0x481, 0x7466, 0x6447, 0x5424, 0x4405,
  0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
  0x26d3, 0x36f2, 0x691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
  0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
  0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x8e1, 0x3882, 0x28a3,
  0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
  0x4a75, 0x5a54, 0x6a37, 0x7a16, 0xaf1, 0x1ad0, 0x2ab3, 0x3a92,
  0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
  0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0xcc1,
  0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
  0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0xed1, 0x1ef0
};


static std::string to_hex(const std::vector<uint8_t> &bytes) {
  std::string hex;
  hex.reserve(bytes.size() * 2);
  for (auto byte: bytes) {
    char buf[3];
    sprintf(buf, "%02x", byte);
    hex += buf;
  }
  return hex;
}

/***********************************************************
CRC16 Coding & Decoding G(X) = X^16+X^12+X^5+1
***********************************************************/
uint16_t CRC16_cal(uint8_t *ptr, uint32_t len, uint16_t crc_init) {
  uint16_t oldcrc16 = 0;
  uint16_t crc = crc_init;
  while (len-- != 0) {
    uint8_t temp = (crc >> 8) & 0xff;
    oldcrc16 = crc16_tab[*ptr ^ temp];
    crc = (crc << 8) ^ oldcrc16;
    ptr++;
  }
  //crc=~crc;
  return (crc);
  //??
}

/*
uint8_t crc_check_16bites(uint8_t *pbuf, uint32_t len, uint32_t *p_result) {
  uint16_t crc_result = 0;
  crc_result = CRC16_cal(pbuf, len, 0);
  *p_result = crc_result;
  return 2;
}
*/

std::string to_string(command_id_t cmd_id) {
  switch (cmd_id) {
    case Heartbeat:
      return "Heartbeat";
    case Acquire_Firmware_Version: return "Acquire_Firmware_Version";
    case Acquire_Hardware_ID: return "Acquire_Hardware_ID";
    case Auto_Focus: return "Auto_Focus";
    case Manual_Zoom_and_Auto_Focus: return "Manual_Zoom_and_Auto_Focus";
    case Manual_Focus: return "Manual_Focus";
    case Gimbal_Rotation: return "Gimbal_Rotation";
    case Center_the_Gimbal: return "Center_the_Gimbal";
    case Acquire_Gimbal_Configuration_Information: return "Acquire_Gimbal_Configuration_Information";
    case Function_Feedback_Information: return "Function_Feedback_Information";
    case Photo_and_Video: return "Photo_and_Video";
    case Acquire_Gimbal_Attitude: return "Acquire_Gimbal_Attitude";
    case Set_Gimbal_Control_Angle: return "Set_Gimbal_Control_Angle";
    case Absolute_Zoom_and_Auto_Focus: return "Absolute_Zoom_and_Auto_Focus";
    case Acquire_Camera_Image_Type: return "Acquire_Camera_Image_Type";
    case Set_Camera_Image_Type: return "Set_Camera_Image_Type";
    case Read_Temperature_of_a_Point: return "Read_Temperature_of_a_Point";
    case Read_Temperature_of_a_Box_on_Screen: return "Read_Temperature_of_a_Box_on_Screen";
    case Read_Temperature_of_the_Full_Screen: return "Read_Temperature_of_the_Full_Screen";
    case Read_Range_from_Laser_Rangefinder: return "Read_Range_from_Laser_Rangefinder";
    case Acquire_Max_Zoom: return "Acquire_Max_Zoom";
  }
  return "Unknown";
}

std::string to_string(gimbal_motion_mode_t mode) {
  switch (mode) {
    case LOCK_MODE: return "LOCK_MODE";
    case FOLLOW_MODE: return "FOLLOW_MODE";
    case FPV_MODE: return "FPV_MODE";
  }
  return "Unknown";
}

std::string to_string(gimbal_mounting_dir_t mode) {
  switch (mode) {
    case MOUNTING_DIR_UNDEFINED: return "MOUNTING_DIR_UNDEFINED";
    case MOUNTING_DIR_NORMAL: return "MOUNTING_DIR_NORMAL";
    case MOUNTING_DIR_UPSIDE_DOWN: return "MOUNTING_DIR_UPSIDE_DOWN";
  }
  return "Unknown";
}

std::vector<uint8_t> encode(uint8_t cmd_id, const std::vector<uint8_t> &data, uint16_t seq, bool need_ack) {
  std::vector<uint8_t> packet;

  size_t packet_size = 10 + data.size();
  packet.reserve(packet_size);
  packet.resize(packet_size);

  // STX
  packet[0] = 0x55;
  packet[1] = 0x66;

  // CTRL
  uint8_t ctrl = 0;
  if (need_ack) ctrl |= 0x01;
  //if (ack_pack) ctrl |= 0x02;
  packet[2] = ctrl;

  // Data_len
  uint16_t data_len = data.size();
  packet[3] = (data_len & 0xFF);
  packet[4] = ((data_len >> 8) & 0xFF);

  // SEQ
  packet[5] = (seq & 0xFF);
  packet[6] = ((seq >> 8) & 0xFF);

  // CMD_ID
  packet[7] = cmd_id;

  // DATA
  for (uint16_t i = 0; i < data_len; i++) {
    packet[8 + i] = data[i];
  }

  // CRC
  uint16_t crc = CRC16_cal(packet.data(), packet_size - 2, 0);
  packet[packet_size - 2] = (crc & 0xFF);
  packet[packet_size - 1] = ((crc >> 8) & 0xFF);

  return packet;
}

size_t get_first_packet_length(const std::vector<uint8_t> &packet) {
  uint16_t data_len = packet[3] | (packet[4] << 8);
  return 8 + data_len + 2; // header + data + crc
}

std::vector<uint8_t> get_first_packet(const std::vector<uint8_t> &packet) {
  auto first_packet_length = get_first_packet_length(packet);
  return std::vector<uint8_t>(packet.begin(), packet.begin() + first_packet_length);
}

bool verify_crc(const std::vector<uint8_t> &packet) {
  if (packet.size() < 3) {
    // Minimum size (including CRC)
    return false;
  }

  // Extract the CRC from the last two bytes of the packet
  uint16_t expected_crc = (packet[packet.size() - 2]) | (packet[packet.size() - 1] << 8);

  // Calculate the CRC for the packet excluding the last two bytes
  uint16_t calculated_crc = CRC16_cal(const_cast<uint8_t *>(packet.data()), packet.size() - 2, 0);

  return calculated_crc == expected_crc;
}

uint16_t get_seq(const std::vector<uint8_t> &packet) {
  if (packet.size() < 7) {
    // Packet is too short to contain a complete sequence number
    throw std::runtime_error("Invalid packet size for extracting sequence number.");
  }

  // Extract the 2-byte sequence number
  // Assuming little-endian format as per your example
  uint16_t seq = packet[5] | (packet[6] << 8);
  return seq;
}

command_id_t get_cmd_id(const std::vector<uint8_t> &packet) {
  if (packet.size() < 8) {
    // Packet is too short to contain a CMD_ID
    throw std::runtime_error("Invalid packet size for extracting CMD_ID.");
  }

  // Extract the CMD_ID
  uint8_t cmd_id = packet[7];
  return (command_id_t) cmd_id;
}

std::vector<uint8_t> get_data(const std::vector<uint8_t> &packet) {
  if (packet.size() < 8) {
    // Packet is too short to even contain headers
    throw std::runtime_error("Invalid packet size for extracting data.");
  }

  // Extract the length of the data field
  uint16_t data_len = packet[3] | (packet[4] << 8);
  if (packet.size() < (size_t) (8 + data_len)) {
    // Packet is too short to contain the specified amount of data
    throw std::runtime_error("Packet size is too small for the specified data length.");
  }

  // Extract the data
  std::vector<uint8_t> data(packet.begin() + 8, packet.begin() + 8 + data_len);
  return data;
}

// parsing stuff

FirmwareVersions parse_firmware_versions_ack(const std::vector<uint8_t> &packet) {
  auto data = get_data(packet);
  //LOG(INFO) << "data:   " << to_hex(data) << ", sz: " << data.size();
  FirmwareVersions versions;
  //
  if (data.size() >= 8) {
    // this is weird formatting
    // 0x6E030203 --> firmware version v3.2.3

    // camera_firmware
    {
      uint8_t ignored, major, minor, patch;
      memcpy(&ignored, &data[0], 1);
      memcpy(&major, &data[1], 1);
      memcpy(&minor, &data[2], 1);
      memcpy(&patch, &data[3], 1);
      char buf[32];
      sprintf(buf, "v%d.%d.%d", major, minor, patch);
      versions.camera_firmware_ver = buf;
    }

    //gimbal firmware
    {
      uint8_t ignored, major, minor, patch;
      memcpy(&ignored, &data[4], 1);
      memcpy(&major, &data[5], 1);
      memcpy(&minor, &data[6], 1);
      memcpy(&patch, &data[7], 1);
      char buf[32];
      sprintf(buf, "v%d.%d.%d", major, minor, patch);
      versions.gimbal_firmware_ver = buf;
    }
  }

  // we might also have a zoom firmware version
  if (data.size() == 12) {
    uint8_t ignored, major, minor, patch;
    memcpy(&ignored, &data[8], 1);
    memcpy(&major, &data[9], 1);
    memcpy(&minor, &data[10], 1);
    memcpy(&patch, &data[11], 1);
    char buf[32];
    sprintf(buf, "v%d.%d.%d", major, minor, patch);
    versions.zoom_firmware_ver = buf;
  }
  return versions;
}

HardwareID parse_hardware_id_ack(const std::vector<uint8_t> &packet) {
  auto data = get_data(packet);
  HardwareID hardware_id;
  for (auto c: data) {
    if (c == 0) {
      break;
    }
    hardware_id.id += c;
  }
  return hardware_id;
}

MaxZoomValue parse_acquire_max_zoom_ack(const std::vector<uint8_t> &packet) {
  auto data = get_data(packet);
  if (data.size() != 2) {
    throw std::runtime_error("Invalid packet size for parsing acquire_max_zoom");
  }

  MaxZoomValue max_zoom;
  memcpy(&max_zoom.zoom_max_int, &data[0], 1);
  memcpy(&max_zoom.zoom_max_float, &data[1], 1);
  return max_zoom;
}

GimbalConfirmation parse_gimbal_configuration_ack(const std::vector<uint8_t> &packet) {
  auto data = get_data(packet);
  GimbalConfirmation confirmation;
  memset(&confirmation, 0, sizeof(confirmation));

  memcpy(&confirmation.hdr_sta, &data[1], 1);
  memcpy(&confirmation.record_sta, &data[3], 1);
  memcpy(&confirmation.gimbal_motion_mode, &data[4], 1);
  memcpy(&confirmation.gimbal_mounting_dir, &data[5], 1);
  memcpy(&confirmation.video_hdmi_or_cvbs, &data[6], 1);
  return confirmation;
}

GimbalAttitude parse_gimbal_attitude(const std::vector<uint8_t> &packet) {
  auto data = get_data(packet);
  if (data.size() != 12) {
    throw std::runtime_error("Invalid packet size for parsing gimbal attitude.");
  }
  GimbalAttitude attitude;
  // Extract the data and convert to host byte order
  memcpy(&attitude.yaw, &data[0], 2);
  //attitude.yaw = ntohs(attitude.yaw);
  memcpy(&attitude.pitch, &data[2], 2);
  //attitude.pitch = ntohs(attitude.pitch);
  memcpy(&attitude.roll, &data[4], 2);
  //attitude.roll = ntohs(attitude.roll);
  memcpy(&attitude.yaw_velocity, &data[6], 2);
  //attitude.yaw_velocity = ntohs(attitude.yaw_velocity);
  memcpy(&attitude.pitch_velocity, &data[8], 2);
  //attitude.pitch_velocity = ntohs(attitude.pitch_velocity);
  memcpy(&attitude.roll_velocity, &data[10], 2);
  //attitude.roll_velocity = ntohs(attitude.roll_velocity);
  return attitude;
}

GimbalCurrentAngles parse_gimbal_control_angle_ack(const std::vector<uint8_t> &packet) {
  /*
  if (packet.size() < 15) {
    // Header (9 bytes) + Data (3 * 2 bytes)
    throw std::runtime_error("Invalid packet size for parsing gimbal control angle ACK.");
  }*/
  auto data = get_data(packet);

  GimbalCurrentAngles angles;
  // Assuming the data starts at byte 9 and the system uses little-endian format
  // Extract and convert each value from network byte order (big-endian) to host byte order
  memcpy(&angles.yaw, &data[0], 2);
  //angles.yaw = ntohs(angles.yaw);
  memcpy(&angles.pitch, &data[2], 2);
  //angles.pitch = ntohs(angles.pitch);
  memcpy(&angles.roll, &data[4], 2);
  //angles.roll = ntohs(angles.roll);

  return angles;
}


ZoomMultipleAck parse_zoom_multiple_ack(const std::vector<uint8_t> &packet) {
  if (packet.size() < 11) {
    // Header (9 bytes) + Zoom Multiple (2 bytes)
    throw std::runtime_error("Invalid packet size for parsing zoom multiple ACK.");
  }

  ZoomMultipleAck ack;
  // Assuming the zoom multiple starts at byte 9 and is in network byte order
  memcpy(&ack.zoom_multiple, &packet[9], 2);
  ack.zoom_multiple = ntohs(ack.zoom_multiple); // Convert from network to host byte order

  return ack;
}

bool parse_auto_focus_ack(const std::vector<uint8_t> &packet) {
  if (packet.size() < 10) {
    // Header (9 bytes) + Auto Focus Status (1 byte)
    throw std::runtime_error("Invalid packet size for parsing auto focus ACK.");
  }
  // Assuming the auto focus status is at byte 9
  return packet[9] == 1; // 1 for success, 0 for failure
}


siyi_connection::siyi_connection(std::string ip, uint16_t port) : thread_(&siyi_connection::recv_thread, this) {
  memset(&send_addr_, 0, sizeof(send_addr_));
  send_addr_.sin_family = AF_INET;
  send_addr_.sin_addr.s_addr = inet_addr(ip.c_str());
  send_addr_.sin_port = htons(port);
  socket_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_ < 0) {
    LOG(ERROR) << "failed to create socket";
  }
  start_ = true;
}

siyi_connection::~siyi_connection() {
  exit_ = true;
  thread_.join();
  close(socket_);
}

void siyi_connection::recv_thread() {
  while (start_ == false && exit_ == false) {
    std::this_thread::sleep_for(100ms);
  }
  LOG(INFO) << "recv_thread start";
  while (!exit_) {
    auto packet = recv();
    if (packet.size()>=8) {
      // should we split packet first
      auto remaining = packet.size();
      while(remaining) {
          auto first_packet = get_first_packet(packet);
          handle_packet(first_packet);
          remaining -= first_packet.size();
          if (remaining) {
            LOG(INFO) << "splitting big packet";
            packet.erase(packet.begin(), packet.begin() + first_packet.size());
          }
      }
    }
  }

  LOG(INFO) << "recv_thread exit";
}

bool siyi_connection::send_packet(const std::vector<uint8_t> &packet) {
  //LOG(INFO)<< "send packet, sz: " << packet.size() << ", " << to_hex(packet) << ", cmd: " << to_string(get_cmd_id(packet));
  if (sendto(socket_, packet.data(), packet.size(), 0, (struct sockaddr *) &send_addr_,
             sizeof(struct sockaddr_in)) < 0) {
    LOG(ERROR) << "sendto failed";
    return false;
  }
  return true;
}

std::vector<uint8_t> siyi_connection::recv() {
  fd_set read_fds;
  FD_ZERO(&read_fds);
  FD_SET(socket_, &read_fds);

  struct timeval tv;
  tv.tv_sec = 1; // Timeout interval
  tv.tv_usec = 0; // Timeout interval

  int retval = select(1, &read_fds, nullptr, nullptr, &tv);
  if (retval == -1) {
    LOG(INFO) << "no data to read";
    return {};
    // Error occurred
  }

  std::vector<uint8_t> recv_buf(64);
  struct sockaddr_in recv_addr{};
  socklen_t addr_len = sizeof(struct sockaddr_in);
  auto recv_len = recvfrom(socket_, recv_buf.data(), 64, 0, (struct sockaddr *) &recv_addr, &addr_len);
  if (recv_len < 0) {
    LOG(ERROR) << "recvfrom failed";
    return {};
  }
  recv_buf.resize(recv_len);
  //LOG(INFO) << "recvfrom " << inet_ntoa(recv_addr.sin_addr) << ":" << ntohs(recv_addr.sin_port) << ", " << to_hex(recv_buf) <<", sz: " << recv_len;
  return recv_buf;
}

void siyi_connection::handle_packet(const std::vector<uint8_t> &packet) {
  if (!verify_crc(packet)) {
    LOG(ERROR) << "CRC verification failed, " << to_hex(packet) << ", cmd: " << to_string(get_cmd_id(packet)) <<
        ", payload: " << to_hex(get_data(packet));
    return;
  }
  uint16_t seq = get_seq(packet);
  auto cmd_id = get_cmd_id(packet);
  auto data = get_data(packet);
  //LOG(INFO) << "handle_packet: " << to_string(cmd_id) << ", seq: " << seq << ", data: " << to_hex(data);
  switch (cmd_id) {
    case Heartbeat: {
      LOG(INFO) << "heartbeat";
      break;
    }

    case Acquire_Firmware_Version: {
      auto versions = parse_firmware_versions_ack(packet);
      LOG(INFO) << "firmware [camera: " << versions.camera_firmware_ver << ", gimbal: " << versions.gimbal_firmware_ver
          << ", zoom: " << versions.zoom_firmware_ver << "]";
      break;
    }

    case Acquire_Hardware_ID: {
      auto hardware_id = parse_hardware_id_ack(packet);
      LOG(INFO) << "hardware_id: " << to_hex(data) << ", " << hardware_id.id;
      break;
    }

    case Auto_Focus: {
      LOG(INFO) << "auto focus";
      break;
    }

    case Manual_Zoom_and_Auto_Focus: {
      LOG(INFO) << "Manual_Zoom_and_Auto_Focus";
      break;
    }

    case Absolute_Zoom_and_Auto_Focus: {
      LOG(INFO) << "Absolute_Zoom_and_Auto_Focus";
      break;
    }

    case Acquire_Max_Zoom: {
      //LOG(INFO) << "Acquire_Max_Zoom";
      auto maxzoom = parse_acquire_max_zoom_ack(packet);
      LOG(INFO) << "MaxZoom: " << (int) maxzoom.zoom_max_int << "." << (int) maxzoom.zoom_max_float;
      break;
    }

    case Manual_Focus: {
      LOG(INFO) << "Manual_Focus";
      break;
    }

    case Gimbal_Rotation: {
      LOG(INFO) << "Gimbal_Rotation";
      break;
    }

    case Center_the_Gimbal: {
      LOG(INFO) << "Center_the_Gimbal";
      break;
    }

    case Acquire_Gimbal_Configuration_Information: {
      //LOG(INFO) << "Acquire_Gimbal_Configuration_Information";
      GimbalConfirmation config = parse_gimbal_configuration_ack(packet);
      LOG(INFO) << "Gimbal_Configuration, hdr_sta: " << (int) config.hdr_sta << ", record_sta: " << (int) config.
          record_sta << ", gimbal_motion_mode: " << to_string(config.gimbal_motion_mode) << ", gimbal_mounting_dir:" <<
          to_string(config.gimbal_mounting_dir) << ", video_hdmi_or_cvbs: " << (int) config.video_hdmi_or_cvbs;
      break;
    }

    case Function_Feedback_Information: {
      LOG(INFO) << "Function_Feedback_Information";
      break;
    }

    case Photo_and_Video: {
      LOG(INFO) << "Photo_and_Video";
      break;
    }

    case Acquire_Gimbal_Attitude: {
      auto attitude = parse_gimbal_attitude(packet);
      LOG(INFO) << "Acquire_Gimbal_Attitude pos: [" <<
          (double) attitude.yaw << ", " << (double) attitude.pitch << ", " << (double) attitude.roll << "] "
          "velocity: [" << (double) attitude.yaw_velocity << ", " << (double) attitude.pitch_velocity << ", " << (double) attitude.
          roll_velocity << "]";
      break;
    }

    case Set_Gimbal_Control_Angle: {
      auto angles = parse_gimbal_control_angle_ack(packet);
      LOG(INFO) << "Set_Gimbal_Control_Angle" << ", yaw: " << (double) angles.yaw << ", pitch: " << (double) angles.pitch
          << ", roll: " << (double) angles.roll;
      break;
    }

    case Acquire_Camera_Image_Type: {
      LOG(INFO) << "Acquire_Camera_Image_Type";
      break;
    }

    case Set_Camera_Image_Type: {
      LOG(INFO) << "Set_Camera_Image_Type";
      break;
    }

    case Read_Temperature_of_a_Point: {
      LOG(INFO) << "Read_Temperature_of_a_Point";
      break;
    }

    case Read_Temperature_of_a_Box_on_Screen: {
      LOG(INFO) << "Read_Temperature_of_a_Box_on_Screen";
      break;
    }

    case Read_Temperature_of_the_Full_Screen: {
      LOG(INFO) << "Read_Temperature_of_the_Full_Screen";
      break;
    }

    case Read_Range_from_Laser_Rangefinder: {
      LOG(INFO) << "Read_Range_from_Laser_Rangefinder";
      break;
    }
  }
}
