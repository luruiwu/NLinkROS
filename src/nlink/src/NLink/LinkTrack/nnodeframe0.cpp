#include "nnodeframe0.h"
#include <iostream>

namespace LinkTrack {

NNodeFrame0GroupsItem::NNodeFrame0GroupsItem(NNodeFrame0 *protocol)
    : NProtocolItemBase(), protocol_(protocol) {
  setBytes(sizeof(protocol_->data_.validNodeCount));
}

bool NNodeFrame0GroupsItem::updateItemBytes(const char *byte) {
  protocol_->nodesWaitVerify_.clear();
  memcpy(&protocol_->data_, byte, sizeof(protocol_->data_));

  auto currentAddress = address() + sizeof(protocol_->data_.validNodeCount);
  size_t allGroupSize = 0;
  const size_t maxGroupSize = protocol_->data_.frameLength -
                              NNodeFrame0::skFrontFixedPartLength_ -
                              NNodeFrame0::skSizeOfCheckSum_;
  NNodeFrame0::Node_t node;
  for (int i = 0;
       i < protocol_->data_.validNodeCount && allGroupSize <= maxGroupSize;
       ++i) {
    memcpy(&node, byte + currentAddress, NNodeFrame0::skBytesOfNodeFixedPart_);
    if (currentAddress + NNodeFrame0::skBytesOfNodeFixedPart_ +
            node.dataLength <=
        protocol_->data_.frameLength) {
      node.dataArray.clear();
      node.dataArray.append(byte + currentAddress +
                                NNodeFrame0::skBytesOfNodeFixedPart_,
                            node.dataLength);
      protocol_->nodesWaitVerify_.push_back(node);
      auto currentGroupSize =
          NNodeFrame0::skBytesOfNodeFixedPart_ + node.dataLength;
      allGroupSize += currentGroupSize;
      currentAddress += currentGroupSize;
    } else {
      std::cerr
          << "Protocol_I_UWB_LPS_Node_Frame0::NGroupItem::updateBytes error";
      setBytes(sizeof(protocol_->data_.validNodeCount));
      return false;
    }
  }
  setBytes(sizeof(protocol_->data_.validNodeCount) + allGroupSize);
  return true;
}

NNodeFrame0::NNodeFrame0() : NProtocolBase() {
  appendItem(new NConstantByteArrayItem(
      std::string(data_.header, sizeof(data_.header))));
  appendItem(new NBytesOfProtocolItem(sizeof(data_.frameLength)));
  appendItem(new NPaddingItem(skFrontFixedPartLength_ - sizeof(data_.header) -
                              sizeof(data_.frameLength) -
                              sizeof(data_.validNodeCount)));

  appendItem(new NNodeFrame0GroupsItem(this));

  appendItem(new NSumCheckItem(skSizeOfCheckSum_));

  assert(totalBytes() == skFrontFixedPartLength_ + skSizeOfCheckSum_);
}

void NNodeFrame0::unpackData(const std::string &byteArray) {
  memcpy(&data_, byteArray.data(), sizeof(data_));
  currentNodes_ = nodesWaitVerify_;
}

NNodeFrame0::Frame_t NNodeFrame0::data() const { return data_; }

std::vector<NNodeFrame0::Node_t> NNodeFrame0::currentNodes() const {
  return currentNodes_;
}

} // namespace LinkTrack
