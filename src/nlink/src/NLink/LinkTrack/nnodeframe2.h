#ifndef LINKTRACKNODEFRAME2_H
#define LINKTRACKNODEFRAME2_H

#include "../../NProtocol/nprotocolbase.h"
#include "ncommon.h"
namespace LinkTrack {

class NNodeFrame2 : public NProtocolBase {
public:
  explicit NNodeFrame2();

  void unpackData(const std::string &byteArray) override;

#pragma pack(1)
  struct Node_t {
    DataType::Role role;
    uint8_t id{};
    DataType::Distance distance{};
    DataType::RSSI fpRssi{};
    DataType::RSSI rxRssi{};
    uint32_t systemTime{};
    uint8_t reserved1[2]{};
  };

  struct Frame_t {
    char header[2]{0x55, 0x04};
    uint16_t frameLength{};
    DataType::Role role{};
    uint8_t id{};
    uint32_t systemTime{};
    DataType::EOP eop{};
    DataType::Position position{};
    DataType::Velocity velocity{};
    uint8_t reserved1[9]{};
    DataType::GYRO imuGyro{};
    DataType::ACC imuAcc{};
    uint8_t reserved2[12]{};
    DataType::Euler angle{};
    float quaternions[4]{};
    uint8_t reserved3[18]{};
    DataType::SupplyVoltage supplyVoltage{};
    uint8_t validNodeCount{};
    //... nodes
    //        uint8_t checkSum{};
  };
#pragma pack()

  Frame_t data() const;

  std::vector<Node_t> currentNodes() const;

private:
  static const size_t skSizeOfCheckSum_ = sizeof(uint8_t);

  static const size_t skFrontFixedPartLength_ = 119; //不包含nodes,sumCheck

  static_assert((sizeof(Frame_t) == skFrontFixedPartLength_), "Error");

  Frame_t data_;
  std::vector<Node_t> currentNodes_; //当前数据帧中节点
};

} // namespace LinkTrack

#endif // LINKTRACKNODEFRAME2_H
