#ifndef LINKTRACKTAGFRAME0_H
#define LINKTRACKTAGFRAME0_H

#include "../../NProtocol/nprotocolbase.h"
#include "ncommon.h"

namespace LinkTrack {

class NTagFrame0 : public NProtocolBase {
public:
  explicit NTagFrame0();

  void unpackData(const std::string &byteArray) override;

#pragma pack(1)
  struct Frame_t : public FrameBase<128> {
    char header[2]{0x55, 0x01};
    uint8_t id{};
    uint8_t reserved0[1]{};
    DataType::Position position;
    DataType::Velocity velocity;
    DataType::Distance distances[8];
    DataType::GYRO imuGyro;
    DataType::ACC imuAcc;
    uint8_t reserved1[12]{};
    DataType::Euler angle;
    float quaternions[4]{};
    uint8_t reserved2[8]{};
    uint32_t systemTime{};
    struct {
      uint8_t : 4;
      uint8_t accWorkNormal : 1;
      uint8_t accIsOnline : 1;
      uint8_t gyroWorkNormal : 1;
      uint8_t gyroIsOnline : 1;
    } sensorStatus{};
    DataType::EOP eop{};
    DataType::SupplyVoltage supplyVoltage{};
    uint8_t reserved3[5]{};
    uint8_t checkSum{};
  };
  static_assert((sizeof(Frame_t) == Frame_t::skLength), "Error");
#pragma pack()

  Frame_t data() const;

private:
  Frame_t data_;
};

} // namespace LinkTrack

#endif // LINKTRACKTAGFRAME0_H
