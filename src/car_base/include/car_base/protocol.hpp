// STM32 底盘串口协议编解码(纯 C++,无 ROS 依赖)。
//
// 依据《总体方案设计 v1.2.1》第 6 章 / 小车手册第 6 章:
//   - 控制帧 (Orange Pi -> STM32): 11 字节
//   - 反馈帧 (STM32 -> Orange Pi): 24 字节
//   - 多字节字段为 int16 大端(网络字节序),补码
//   - BCC 校验 = 帧内数据字节逐字节异或
//
// 实测标定(见 docs/protocols/car_base_标定记录.md):
//   - cmd_wz_sign = +1, odom_wz_sign = +1
//   - 反馈角速度幅值疑似约 3 倍,用 odom_wz_scale 修正
//   - IMU 未到货,反馈帧字节 8..19 暂为噪声
#ifndef CAR_BASE_PROTOCOL_HPP_
#define CAR_BASE_PROTOCOL_HPP_

#include <array>
#include <cstdint>
#include <cstddef>
#include <deque>
#include <optional>
#include <vector>

namespace car_base
{

constexpr uint8_t kFrameHead = 0x7B;
constexpr uint8_t kFrameTail = 0x7D;
constexpr size_t kCtrlFrameLen = 11;
constexpr size_t kFbFrameLen = 24;

// IMU 原始计数 -> 物理量换算(手册 6.2 明确值)
constexpr double kAccCountPerMs2 = 1672.0;      // ±2G 量程
constexpr double kGyroCountPerRadps = 3753.0;   // ±500°/s 量程

// 逐字节异或校验
inline uint8_t bcc(const uint8_t * data, size_t len)
{
  uint8_t c = 0;
  for (size_t i = 0; i < len; ++i) {
    c ^= data[i];
  }
  return c;
}

// 大端读取 int16(补码)
inline int16_t read_be_i16(const uint8_t * p)
{
  return static_cast<int16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
}

// 大端读取 uint16
inline uint16_t read_be_u16(const uint8_t * p)
{
  return static_cast<uint16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
}

// 大端写入 int16(补码),带饱和
inline void write_be_i16(uint8_t * p, int32_t v)
{
  if (v > 32767) {v = 32767;}
  if (v < -32768) {v = -32768;}
  const uint16_t u = static_cast<uint16_t>(static_cast<int16_t>(v));
  p[0] = static_cast<uint8_t>((u >> 8) & 0xFF);
  p[1] = static_cast<uint8_t>(u & 0xFF);
}

// 构造 11 字节控制帧。
//   vx: 线速度 m/s;  wz: 角速度 rad/s;  cmd_wz_sign: 角速度符号参数。
inline std::array<uint8_t, kCtrlFrameLen> build_ctrl_frame(
  double vx, double wz, int cmd_wz_sign = 1,
  uint8_t flag_stop = 0, uint8_t reserved = 0)
{
  std::array<uint8_t, kCtrlFrameLen> f{};
  f[0] = kFrameHead;
  f[1] = flag_stop;
  write_be_i16(&f[2], static_cast<int32_t>(vx * 1000.0));        // X 速度
  write_be_i16(&f[4], 0);                                        // Y 速度(差速车恒 0)
  write_be_i16(&f[6], static_cast<int32_t>(cmd_wz_sign * wz * 1000.0));  // Z 角速度
  f[8] = reserved;
  f[9] = bcc(f.data(), 9);        // 字节 1..9
  f[10] = kFrameTail;
  return f;
}

// 解析后的反馈数据(已换算物理单位)
struct Feedback
{
  uint8_t flag_stop = 0;
  double vx = 0.0;              // m/s
  double vy = 0.0;              // m/s
  double wz = 0.0;             // rad/s(未套 odom_wz_sign/scale,由上层处理)
  double acc[3] = {0, 0, 0};   // m/s^2
  double gyro[3] = {0, 0, 0};  // rad/s
  double voltage_raw = 0.0;    // 原始值/1000(未标定,上层用 voltage_scale 处理)
  int16_t raw_vz = 0;          // Z 角速度原始计数,便于调试
};

// 解析 24 字节反馈帧;校验/长度错误返回 nullopt。
inline std::optional<Feedback> parse_feedback(const uint8_t * f, size_t len)
{
  if (len != kFbFrameLen) {return std::nullopt;}
  if (f[0] != kFrameHead) {return std::nullopt;}
  if (f[kFbFrameLen - 1] != kFrameTail) {return std::nullopt;}
  if (bcc(f, 22) != f[22]) {return std::nullopt;}   // 字节 1..22

  Feedback fb;
  fb.flag_stop = f[1];
  fb.vx = read_be_i16(&f[2]) / 1000.0;
  fb.vy = read_be_i16(&f[4]) / 1000.0;
  fb.raw_vz = read_be_i16(&f[6]);
  fb.wz = fb.raw_vz / 1000.0;
  fb.acc[0] = read_be_i16(&f[8]) / kAccCountPerMs2;
  fb.acc[1] = read_be_i16(&f[10]) / kAccCountPerMs2;
  fb.acc[2] = read_be_i16(&f[12]) / kAccCountPerMs2;
  fb.gyro[0] = read_be_i16(&f[14]) / kGyroCountPerRadps;
  fb.gyro[1] = read_be_i16(&f[16]) / kGyroCountPerRadps;
  fb.gyro[2] = read_be_i16(&f[18]) / kGyroCountPerRadps;
  fb.voltage_raw = read_be_u16(&f[20]) / 1000.0;
  return fb;
}

// 字节流帧同步器:串口 read() 不保证整帧,按帧头累积、校验、重同步。
class FrameReader
{
public:
  // 诊断计数
  uint64_t ok = 0;
  uint64_t bcc_err = 0;
  uint64_t resync = 0;

  // 喂入字节,返回本次解析出的所有 Feedback
  std::vector<Feedback> feed(const uint8_t * data, size_t len)
  {
    std::vector<Feedback> out;
    for (size_t i = 0; i < len; ++i) {
      buf_.push_back(data[i]);
    }
    while (true) {
      // 找帧头
      while (!buf_.empty() && buf_.front() != kFrameHead) {
        buf_.pop_front();
        ++resync;
      }
      if (buf_.size() < kFbFrameLen) {break;}
      // 拷出一帧
      uint8_t frame[kFbFrameLen];
      for (size_t i = 0; i < kFbFrameLen; ++i) {frame[i] = buf_[i];}
      auto fb = parse_feedback(frame, kFbFrameLen);
      if (fb) {
        out.push_back(*fb);
        ++ok;
        for (size_t i = 0; i < kFbFrameLen; ++i) {buf_.pop_front();}
      } else {
        // 帧头后单字节跳过,重新同步
        buf_.pop_front();
        if (frame[22] != bcc(frame, 22)) {++bcc_err;}
        ++resync;
      }
    }
    return out;
  }

private:
  std::deque<uint8_t> buf_;
};

}  // namespace car_base

#endif  // CAR_BASE_PROTOCOL_HPP_
