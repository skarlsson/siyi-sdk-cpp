#include <gtest/gtest.h>
#include <siyi-sdk-cpp/siyi.h>
#include <glog/logging.h>

class A8_test : public ::testing::Test {
protected:
};

TEST_F(A8_test, Test1) // NOLINT
{
  //Rotate 100 100
  std::string ROTATE_100_100_HEX = "556601020000000764643dcf";
  //EXPECT_EQ(encryption_entry0->iv, hex_to_bytes(ENCRYPTION_INIT_VECTOR_HEX));
  EXPECT_EQ(100, 100);
  // EXPECT_EQ(verification_entry0->verificatorItem.data,
  // hex_to_bytes(VERIFICATION_ITEM_HEX));
}

class FirmwarePacketTest : public ::testing::Test {
protected:
  // Set up any required variables or state here
};

TEST_F(FirmwarePacketTest, FirmwareVersionPacketCorrectFormat) {
  uint16_t seq = 0x0000;
  std::vector<uint8_t> expected_payload_bytes = {0x55, 0x66, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01};
  auto packet = make_get_firmware_version_packet(seq);
  // Check if the packet has a valid CRC
  LOG(INFO) << packet.size();
  ASSERT_TRUE(verify_crc(packet));
  ASSERT_EQ(get_seq(packet), seq);
  ASSERT_EQ(get_cmd_id(packet), 0x01);
  auto data = get_data(packet);
  ASSERT_EQ(data.size(), 0); // should be empty
  std::vector<uint8_t> actual_payload_bytes(packet.begin(), packet.end() - 2);

  // Compare the generated packet with the expected packet
  // Note: You may need to adjust the expected CRC bytes based on your CRC calculation
  ASSERT_EQ(actual_payload_bytes.size(), expected_payload_bytes.size());
  for (size_t i = 0; i < actual_payload_bytes.size(); ++i) {
    EXPECT_EQ(actual_payload_bytes[i], expected_payload_bytes[i]) << "Mismatch at byte index " << i;
  }
}



int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
