#include "nparameter.h"

namespace LinkTrack {
NParameter::NParameter() : NProtocolBase() {
  appendItem(new NConstantByteArrayItem(
      std::string(data_.header, sizeof(data_.header))));
  appendItem(new NPaddingItem(sizeof(Frame_t) - sizeof(data_.header) -
                              sizeof(data_.checkSum)));
  appendItem(new NSumCheckItem(sizeof(data_.checkSum)));

  assert(totalBytes() == sizeof(Frame_t));
}

void NParameter::unpackData(const std::string &byteArray) {
  memcpy(&data_, byteArray.data(), sizeof(data_));
}

std::string NParameter::updateCommand() {
  data_.updateCheckSum();
  return data_.byteArray();
}

NParameter::Frame_t NParameter::data() const { return data_; }

} // namespace LinkTrack
