#include "nprotocolbase.h"
#include "nprotocolbase_p.h"
#include <algorithm>
#include <cassert>

NProtocolBase::NProtocolBase() : d(new NProtocolBase_P()) {}

NProtocolBase::~NProtocolBase() { delete d; }

size_t NProtocolBase::totalBytes() const { return d->totalBytes_; }

void NProtocolBase::appendItem(NProtocolItemBase *item) {
  assert(
      (std::find(d->items_.begin(), d->items_.end(), item) == d->items_.end()));

  if (d->items_.empty()) {
    auto firstItem = dynamic_cast<NConstantByteArrayItem *>(item);
    assert(firstItem);
    d->headerItem_ = firstItem;

    item->setAddress(0);
    d->totalBytes_ = 0;
  } else {
    item->setAddress(d->items_.back()->address() + d->items_.back()->bytes());
  }
  if (!d->bytesOfProtocolItem_) {
    d->bytesOfProtocolItem_ = dynamic_cast<NBytesOfProtocolItem *>(item);
  }
  d->items_.push_back(item);
  d->totalBytes_ += item->bytes();
}

std::vector<NProtocolItemBase *> NProtocolBase::items() const {
  return d->items_;
}

void NProtocolBase::setDataHandle(std::function<void()> handle) {
  d->dataHandle_ = handle;
}

NProtocolBase_P::~NProtocolBase_P() {
  for (auto item : items_) {
    delete item;
  }
}

bool NProtocolBase_P::updateItemsAddressAndBytes(const char *byte) {
  NProtocolItemBase *prevItem = items_.front();
  totalBytes_ = prevItem->bytes();
  for (size_t i = 1; i < items_.size(); ++i) {
    items_.at(i)->setAddress(prevItem->address() + prevItem->bytes());
    if (items_.at(i)->updateItemBytes(byte)) {
      prevItem = items_.at(i);
      totalBytes_ += prevItem->bytes();
    } else {
      return false;
    }
  }
  return true;
}
