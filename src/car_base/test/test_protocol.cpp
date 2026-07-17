// C++ 协议编解码单元测试,用《总体方案设计 v1.2.1》6.5 节示例值断言。
#include <gtest/gtest.h>
#include <array>
#include <cstring>
#include <vector>

#include "car_base/protocol.hpp"

using namespace car_base;

// 按协议拼一个合法 24 字节反馈帧(自动算 BCC)
static std::array<uint8_t, kFbFrameLen> make_fb(
  int16_t vx, int16_t vy, int16_t vz,
  int16_t ax, int16_t ay, int16_t az,
  int16_t gx, int16_t gy, int16_t gz,
  uint16_t batt_mv)
{
  std::array<uint8_t, kFbFrameLen> f{};
  f[0] = kFrameHead;
  f[1] = 0;
  write_be_i16(&f[2], vx);
  write_be_i16(&f[4], vy);
  write_be_i16(&f[6], vz);
  write_be_i16(&f[8], ax);
  write_be_i16(&f[10], ay);
  write_be_i16(&f[12], az);
  write_be_i16(&f[14], gx);
  write_be_i16(&f[16], gy);
  write_be_i16(&f[18], gz);
  f[20] = static_cast<uint8_t>((batt_mv >> 8) & 0xFF);
  f[21] = static_cast<uint8_t>(batt_mv & 0xFF);
  f[22] = bcc(f.data(), 22);
  f[23] = kFrameTail;
  return f;
}

TEST(Protocol, CtrlFrameStructure) {
  auto f = build_ctrl_frame(0.257, 0.0);
  EXPECT_EQ(f.size(), kCtrlFrameLen);
  EXPECT_EQ(f[0], kFrameHead);
  EXPECT_EQ(f[10], kFrameTail);
  EXPECT_EQ(f[9], bcc(f.data(), 9));
  // 0.257 m/s -> 257 = 0x0101 大端
  EXPECT_EQ(f[2], 0x01);
  EXPECT_EQ(f[3], 0x01);
}

TEST(Protocol, CtrlFrameNegative) {
  auto f = build_ctrl_frame(-0.257, 0.0);
  EXPECT_EQ(read_be_i16(&f[2]), -257);
}

TEST(Protocol, CtrlWzSign) {
  auto fp = build_ctrl_frame(0.0, 0.5, 1);
  auto fn = build_ctrl_frame(0.0, 0.5, -1);
  EXPECT_EQ(read_be_i16(&fp[6]), 500);
  EXPECT_EQ(read_be_i16(&fn[6]), -500);
}

TEST(Protocol, CtrlVxSign) {
  // 第4个参数 cmd_vx_sign:-1 应把正 vx 编码成负值
  auto fp = build_ctrl_frame(0.2, 0.0, 1, 1);
  auto fn = build_ctrl_frame(0.2, 0.0, 1, -1);
  EXPECT_EQ(read_be_i16(&fp[2]), 200);
  EXPECT_EQ(read_be_i16(&fn[2]), -200);
}

TEST(Protocol, ParseFeedbackDocExample) {
  // X 速度 0x0101=257 -> 0.257 m/s;Z 加速度 0x4080=16512 -> /1672=9.8756;
  // 电池 0x5838=22584 mV
  auto f = make_fb(0x0101, 0, 0, 0, 0, 0x4080, 0, 0, 0, 0x5838);
  auto fb = parse_feedback(f.data(), f.size());
  ASSERT_TRUE(fb.has_value());
  EXPECT_NEAR(fb->vx, 0.257, 1e-6);
  EXPECT_NEAR(fb->acc[2], 9.8756, 1e-3);
  EXPECT_NEAR(fb->voltage_raw, 22.584, 1e-6);
}

TEST(Protocol, ParseFeedbackNegative) {
  // 0xFE96=-362 -> /1672=-0.2165;0xFFFB=-5 -> /3753=-0.00133
  auto f = make_fb(0, 0, 0,
    static_cast<int16_t>(0xFE96), 0, 0,
    0, 0, static_cast<int16_t>(0xFFFB), 12000);
  auto fb = parse_feedback(f.data(), f.size());
  ASSERT_TRUE(fb.has_value());
  EXPECT_NEAR(fb->acc[0], -0.2165, 1e-3);
  EXPECT_NEAR(fb->gyro[2], -0.00133, 1e-4);
}

TEST(Protocol, BccDetectsCorruption) {
  auto f = make_fb(1, 0, 0, 0, 0, 0, 0, 0, 0, 12000);
  f[5] ^= 0xFF;  // 破坏数据字节
  auto fb = parse_feedback(f.data(), f.size());
  EXPECT_FALSE(fb.has_value());
}

TEST(Protocol, FrameReaderResyncWithGarbage) {
  auto good = make_fb(0x0101, 0, 0, 0, 0, 0, 0, 0, 0, 12000);
  FrameReader reader;
  // 垃圾字节 + 一整帧
  std::vector<uint8_t> stream = {0x00, 0xFF, 0xAB};
  stream.insert(stream.end(), good.begin(), good.end());
  // 加下一帧前 10 字节
  stream.insert(stream.end(), good.begin(), good.begin() + 10);
  auto out = reader.feed(stream.data(), stream.size());
  ASSERT_EQ(out.size(), 1u);
  EXPECT_NEAR(out[0].vx, 0.257, 1e-6);
  // 喂入后半段,拼出第二帧
  auto out2 = reader.feed(good.data() + 10, kFbFrameLen - 10);
  ASSERT_EQ(out2.size(), 1u);
  EXPECT_EQ(reader.ok, 2u);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
