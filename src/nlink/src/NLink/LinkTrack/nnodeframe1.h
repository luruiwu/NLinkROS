#ifndef LINKTRACKNODEFRAME1_H
#define LINKTRACKNODEFRAME1_H

#include "../../NProtocol/nprotocolbase.h"
#include "ncommon.h"

namespace LinkTrack {

class NNodeFrame1 : public NProtocolBase {
public:
  explicit NNodeFrame1();

  void unpackData(const std::string &byteArray) override;

#pragma pack(1)
  struct Node_t {
    DataType::Role role;
    uint8_t id{};
    DataType::Position position;
    uint8_t reserved0[9]{};
  };

  struct Frame_t {
    char header[2]{0x55, 0x03};
    uint16_t frameLength{};
    DataType::Role role;
    uint8_t id{};
    uint32_t systemTime{};
    uint8_t reserved0[14]{};
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

  static const size_t skFrontFixedPartLength_ = 27; //不包含nodes,sumCheck

  static_assert((sizeof(Frame_t) == skFrontFixedPartLength_), "Error");

  Frame_t data_;
  std::vector<Node_t> currentNodes_; //当前数据帧中节点
};

} // namespace LinkTrack
#endif // LINKTRACKNODEFRAME1_H
