#include "nsysteminfo.h"
#include <cassert>

namespace NLink {

NSystemInfo::NSystemInfo() : NProtocolBase() {
  appendItem(new NConstantByteArrayItem(
      std::string(data_.header, sizeof(data_.header))));
  appendItem(new NPaddingItem(sizeof(Frame_t) - sizeof(data_.header) -
                              sizeof(data_.checkSum)));
  appendItem(new NSumCheckItem(sizeof(data_.checkSum)));

  assert(totalBytes() == sizeof(Frame_t));
}

void NSystemInfo::unpackData(const std::string &byteArray) {
  memcpy(&data_, byteArray.data(), sizeof(data_));
  //  qDebug() << "NSystemInfo::handleFrameResolved" << byteArray.toHex(' ');
}

std::string NSystemInfo::updateCommand() {
  data_.updateCheckSum();
  return data_.byteArray();
}

std::string NSystemInfo::readCommand() {
  if (readCommand_.empty()) {
    Frame_t data;
    data.mix.readParam = 1;
    data.updateCheckSum();
    readCommand_ = data.byteArray();
  }
  return readCommand_;
}

} // namespace NLink
