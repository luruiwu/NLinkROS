#include "nframe0.h"

namespace TOFSense {

NFrame0::NFrame0() : NProtocolBase() {
  appendItem(new NConstantByteArrayItem(
      std::string(data_.header, sizeof(data_.header))));
  appendItem(new NPaddingItem(Frame_t::skLength - sizeof(data_.header) -
                              sizeof(data_.checkSum)));
  appendItem(new NSumCheckItem(sizeof(data_.checkSum)));

  assert(totalBytes() == sizeof(Frame_t));
}

void NFrame0::unpackData(const std::string &byteArray) {
  data_.update(byteArray.data());
}

NFrame0::Frame_t NFrame0::data() const { return data_; }

} // namespace TOFSense
