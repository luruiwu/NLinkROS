#ifndef NPROTOCOLITEMS_H
#define NPROTOCOLITEMS_H

#include "nprotocolitembase.h"
#include <algorithm>
#include <string.h>
#include <string>

class NConstantByteArrayItem : public NProtocolItemBase {
public:
  explicit NConstantByteArrayItem(const std::string &byteArray);
  bool verifyData(const char *byte) override {
    return 0 == memcmp(byteArray_.data(), byte + address(), byteArray_.size());
    //        return
    //        std::equal(byteArray_.cbegin(),byteArray().cend(),byte+address());
  }
  std::string byteArray() const { return byteArray_; }

private:
  const std::string byteArray_;
};

class NSumCheckItem : public NProtocolItemBase {
public:
  explicit NSumCheckItem(size_t bytes);
  bool verifyData(const char *byte) override;
};

class NPaddingItem : public NProtocolItemBase {
public:
  explicit NPaddingItem(size_t bytes);
};

class NCountInterface {
public:
  explicit NCountInterface(size_t bytes);
  void updateValue(const char *byte);
  inline uint64_t value() const { return value_; }

private:
  uint64_t value_ = 0;
  const size_t bytes_;
};

class NBytesOfProtocolItem : public NProtocolItemBase, public NCountInterface {
public:
  explicit NBytesOfProtocolItem(size_t bytesOfValue);
};

class NGroupsItem : public NProtocolItemBase, public NCountInterface {
public:
  NGroupsItem(size_t bytesOfGroupCountValue, size_t bytesOfSingleGroup);

  bool updateItemBytes(const char *byte) override;

private:
  const size_t kBytesOfGroupCountValue_;
  const size_t kBytesOfSingleGroup_;
};

#endif // NPROTOCOLITEMS_H
