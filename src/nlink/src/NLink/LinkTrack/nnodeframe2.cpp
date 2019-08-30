#include "nnodeframe2.h"

namespace LinkTrack {
NNodeFrame2::NNodeFrame2() : NProtocolBase() {
  appendItem(new NConstantByteArrayItem(
      std::string(data_.header, sizeof(data_.header))));
  appendItem(new NBytesOfProtocolItem(sizeof(data_.frameLength)));
  appendItem(new NPaddingItem(skFrontFixedPartLength_ - sizeof(data_.header) -
                              sizeof(data_.frameLength) -
                              sizeof(data_.validNodeCount)));
  appendItem(new NGroupsItem(sizeof(data_.validNodeCount), sizeof(Node_t)));
  //中间是变长的group
  auto checkSumBytes = sizeof(uint8_t);
  appendItem(new NSumCheckItem(checkSumBytes));

  assert(totalBytes() == skFrontFixedPartLength_ + checkSumBytes);
}

void NNodeFrame2::unpackData(const std::string &byteArray) {
  memcpy(&data_, byteArray.data(), sizeof(data_));
  currentNodes_.clear();
  Node_t node;
  for (size_t i = 0; i < data_.validNodeCount; ++i) {
    memcpy(&node,
           byteArray.data() + skFrontFixedPartLength_ + i * sizeof(Node_t),
           sizeof(Node_t));
    currentNodes_.push_back(node);
  }
}

NNodeFrame2::Frame_t NNodeFrame2::data() const { return data_; }

std::vector<NNodeFrame2::Node_t> NNodeFrame2::currentNodes() const {
  return currentNodes_;
}

} // namespace LinkTrack
