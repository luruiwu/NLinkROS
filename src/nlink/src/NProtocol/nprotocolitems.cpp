#include "nprotocolitems.h"
#include <cassert>
#include <string.h>
NBytesOfProtocolItem::NBytesOfProtocolItem(size_t bytesOfValue)
    : NProtocolItemBase(), NCountInterface(bytesOfValue) {
  setBytes(bytesOfValue);
}

NPaddingItem::NPaddingItem(size_t bytes) : NProtocolItemBase() {
  setBytes(bytes);
}

NConstantByteArrayItem::NConstantByteArrayItem(const std::string &byteArray)
    : NProtocolItemBase(), byteArray_(byteArray) {
  assert(!byteArray.empty());
  setBytes(byteArray.size());
}

NSumCheckItem::NSumCheckItem(size_t bytes) : NProtocolItemBase() {
  setBytes(bytes);
  assert(bytes <= 8); //最长uint64_t
}

bool NSumCheckItem::verifyData(const char *byte) {
  uint64_t sum = 0;
  auto uByte = reinterpret_cast<const uint8_t *>(byte);
  sum = std::accumulate(uByte, uByte + address(), sum);
  return 0 == memcmp(uByte + address(), &sum, bytes());
}

NGroupsItem::NGroupsItem(size_t bytesOfGroupCountValue,
                         size_t bytesOfSingleGroup)
    : NProtocolItemBase(), NCountInterface(bytesOfGroupCountValue),
      kBytesOfGroupCountValue_(bytesOfGroupCountValue),
      kBytesOfSingleGroup_(bytesOfSingleGroup) {
  setBytes(kBytesOfGroupCountValue_);
}

bool NGroupsItem::updateItemBytes(const char *byte) {
  updateValue(byte + address());
  auto groupCount = value();
  setBytes(kBytesOfGroupCountValue_ + groupCount * kBytesOfSingleGroup_);
  return true;
}

NCountInterface::NCountInterface(size_t bytes) : bytes_(bytes) {}

void NCountInterface::updateValue(const char *byte) {
  value_ = 0;
  memcpy(&value_, byte, bytes_);
}
