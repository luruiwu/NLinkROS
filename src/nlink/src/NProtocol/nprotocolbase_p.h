#ifndef NPROTOCOLBASE_P_H
#define NPROTOCOLBASE_P_H

#include "nprotocolitems.h"
#include <functional>
#include <vector>

class NProtocolBase;
class NProtocolBase_P {
public:
  explicit NProtocolBase_P() = default;
  ~NProtocolBase_P();

  bool isLengthVariable() const { return bytesOfProtocolItem_; }

  bool updateItemsAddressAndBytes(const char *byte);

  void setTotalBytes(size_t totalBytes) { totalBytes_ = totalBytes; }

  NConstantByteArrayItem *headerItem() const { return headerItem_; }

  NBytesOfProtocolItem *bytesOfProtocolItem() const {
    return bytesOfProtocolItem_;
  }

  size_t totalBytes_ = 0;

  std::vector<NProtocolItemBase *> items_;

  NConstantByteArrayItem *headerItem_;

  NBytesOfProtocolItem *bytesOfProtocolItem_ = nullptr;

  std::function<void()> dataHandle_;
};
#endif // NPROTOCOLBASE_P_H
