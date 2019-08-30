#include "ntagframe0.h"
namespace LinkTrack {

NTagFrame0::NTagFrame0() : NProtocolBase() {
  appendItem(new NConstantByteArrayItem(
      std::string(data_.header, sizeof(data_.header))));
  appendItem(new NPaddingItem(sizeof(Frame_t) - sizeof(data_.header) -
                              sizeof(data_.checkSum)));
  appendItem(new NSumCheckItem(sizeof(data_.checkSum)));

  assert(totalBytes() == sizeof(Frame_t));
}

void NTagFrame0::unpackData(const std::string &byteArray) {
  data_.update(byteArray.data());
}

NTagFrame0::Frame_t NTagFrame0::data() const { return data_; }

} // namespace LinkTrack
