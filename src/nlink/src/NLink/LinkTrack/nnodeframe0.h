#ifndef LINKTRACKNODEFRAME0_H
#define LINKTRACKNODEFRAME0_H

#include "../../NProtocol/nprotocolbase.h"
#include "../../NProtocol/nprotocolitembase.h"
#include "ncommon.h"

namespace LinkTrack {

class NNodeFrame0 : public NProtocolBase {
public:
  explicit NNodeFrame0();

  void unpackData(const std::string &byteArray) override;

#pragma pack(1)

  struct Node_t {
    DataType::Role role;
    uint8_t id{};
    uint16_t dataLength;
    std::string dataArray;
  };

  struct Frame_t {
    char header[2]{0x55, 0x02};
    uint16_t frameLength{};
    DataType::Role role;
    uint8_t id{};
    uint8_t reserved0[4]{};
    uint8_t validNodeCount{};
    //... nodes
    //        uint8_t checkSum{};
  };

#pragma pack()

  Frame_t data() const;

  std::vector<Node_t> currentNodes() const;

private:
  static const size_t skBytesOfNodeFixedPart_ =
      sizeof(Node_t) - sizeof(std::string);

  static const size_t skFrontFixedPartLength_ = 11; //不包含nodes,sumCheck

  static_assert((sizeof(Frame_t) == skFrontFixedPartLength_), "Error");

  static const size_t skSizeOfCheckSum_ = sizeof(uint8_t);

  Frame_t data_;

  //当前数据帧有效节点
  std::vector<Node_t> currentNodes_;

  //更新groupitem长度时进行的赋值，此时还未通过完整数据帧校验
  std::vector<Node_t> nodesWaitVerify_;

  friend class NNodeFrame0GroupsItem;
};

class NNodeFrame0GroupsItem : public NProtocolItemBase {
public:
  explicit NNodeFrame0GroupsItem(NNodeFrame0 *protocol);
  bool updateItemBytes(const char *byte) override;

private:
  NNodeFrame0 *protocol_;
};

} // namespace LinkTrack
#endif // LINKTRACKNODEFRAME0_H
