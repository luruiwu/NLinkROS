#include "nnodeframe1.h"

namespace LinkTrack {

NNodeFrame1::NNodeFrame1() : NProtocolBase() {
  appendItem(new NConstantByteArrayItem(
      std::string(data_.header, sizeof(data_.header))));
  appendItem(new NBytesOfProtocolItem(sizeof(data_.frameLength)));
  appendItem(new NPaddingItem(skFrontFixedPartLength_ - sizeof(data_.header) -
                              sizeof(data_.frameLength) -
                              sizeof(data_.validNodeCount)));
  appendItem(new NGroupsItem(sizeof(data_.validNodeCount), sizeof(Node_t)));
  //中间是变长的group
  appendItem(new NSumCheckItem(skSizeOfCheckSum_));

  assert(totalBytes() == skFrontFixedPartLength_ + skSizeOfCheckSum_);
}

void NNodeFrame1::unpackData(const std::string &byteArray) {
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

NNodeFrame1::Frame_t NNodeFrame1::data() const { return data_; }

std::vector<NNodeFrame1::Node_t> NNodeFrame1::currentNodes() const {
  return currentNodes_;
}

} // namespace LinkTrack
