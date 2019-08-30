#include "nanchorframe0.h"

namespace LinkTrack {

NAnchorFrame0::NAnchorFrame0() : NProtocolBase() {
  appendItem(new NConstantByteArrayItem(
      std::string(data_.header, sizeof(data_.header))));
  appendItem(new NPaddingItem(sizeof(Frame_t) - sizeof(data_.header) -
                              sizeof(data_.check)));
  appendItem(new NConstantByteArrayItem(
      std::string(data_.check, sizeof(data_.check))));

  assert(totalBytes() == sizeof(Frame_t));
}

void NAnchorFrame0::unpackData(const std::string &byteArray) {
  memcpy(&data_, byteArray.data(), sizeof(data_));
}

NAnchorFrame0::Frame_t NAnchorFrame0::data() const { return data_; }

} // namespace LinkTrack
