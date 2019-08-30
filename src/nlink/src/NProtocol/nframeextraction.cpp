#include "nframeextraction.h"
#include "nframeextraction_p.h"
#include "nprotocolbase.h"
#include "nprotocolbase_p.h"
#include <cassert>
#include <string>
#include <vector>

using std::string;
NFrameExtraction::NFrameExtraction() : d(new NFrameExtraction_P()) {}

NFrameExtraction::~NFrameExtraction() { delete d; }

void NFrameExtraction::unpackData(const string &data) {

  if (data.empty())
    return;
  string tempArray(d->prevTempArray_ + data);
  d->prevTempArray_.clear();

  d->lackIndex_ = tempArray.size();
  d->sortInfos_.clear();

  auto protocols = d->protocols_;
  for (size_t i = 0; i < protocols.size(); ++i) {
    NProtocolBase *protocol = protocols.at(i);

    const auto headerArray = protocol->d->headerItem()->byteArray();

    size_t address = 0;
    for (;;) {
      //同一个帧头可能存在多次，找到后继续找
      auto headerIndex = tempArray.find(headerArray, address);
      address = headerIndex + headerArray.size();
      if (headerIndex != string::npos) {
        d->sortInfos_.push_back(NFrameExtraction_P::SortInfo{
            i, headerIndex, protocol->totalBytes()});
      } else
        break;
    }
  }

  std::sort(d->sortInfos_.begin(), d->sortInfos_.end(),
            [](NFrameExtraction_P::SortInfo a, NFrameExtraction_P::SortInfo b) {
              if (a.headerIndex < b.headerIndex)
                return true;
              else if (a.headerIndex == b.headerIndex) {
                if (a.protocolBytes < b.protocolBytes) {
                  return true;
                }
              }
              return false;
            });

  for (size_t sortIndex = 0; sortIndex < d->sortInfos_.size(); ++sortIndex) {
    const NFrameExtraction_P::SortInfo &sortInfo = d->sortInfos_.at(sortIndex);
    NProtocolBase *protocol = protocols.at(sortInfo.protocolIndex);

    if (protocol->d->isLengthVariable()) {
      NBytesOfProtocolItem *bytesOfProtocolItem =
          protocol->d->bytesOfProtocolItem();
      if (sortInfo.headerIndex + bytesOfProtocolItem->address() +
              bytesOfProtocolItem->bytes() <=
          tempArray.size()) {
        bytesOfProtocolItem->updateValue(tempArray.data() +
                                         sortInfo.headerIndex +
                                         bytesOfProtocolItem->address());
        protocol->d->setTotalBytes(bytesOfProtocolItem->value());
      } else {
        if (d->lackIndex_ == tempArray.size()) {
          d->lackIndex_ = sortInfo.headerIndex;
          continue;
        }
      }
    }
    if (tempArray.size() - sortInfo.headerIndex < protocol->totalBytes()) {
      if (d->lackIndex_ == tempArray.size()) {
        d->lackIndex_ = sortInfo.headerIndex;
      }
      continue;
    }

    if (protocol->d->isLengthVariable()) {
      protocol->d->updateItemsAddressAndBytes(tempArray.data() +
                                              sortInfo.headerIndex);
      auto totalBytes = protocol->d->bytesOfProtocolItem()->value();
      auto calculateBytes = protocol->totalBytes();
      if (totalBytes != calculateBytes) {
        continue;
      }
    }
    bool succeed = true;
    for (NProtocolItemBase *item : protocol->items()) {
      succeed = item->verifyData(tempArray.data() + sortInfo.headerIndex);
      if (!succeed)
        break;
    }

    if (succeed) {
      //本次成功后 将帧头在本次协议帧长范围内的都移除
      d->lackIndex_ = tempArray.size();
      auto iterBegin = d->sortInfos_.begin() + sortIndex + 1;
      auto iterEnd = d->sortInfos_.end();
      for (auto iter = iterBegin; iter != iterEnd; ++iter) {
        if ((*iter).headerIndex - sortInfo.headerIndex >=
            protocol->totalBytes()) {
          iterEnd = iter;
          break;
        }
      }
      d->sortInfos_.erase(iterBegin, iterEnd);

      auto protocolByteArray =
          tempArray.substr(sortInfo.headerIndex, protocol->totalBytes());

      protocol->unpackData(protocolByteArray);
      protocol->d->dataHandle_();
    }
  }

  if (d->lackIndex_ < tempArray.size()) {
    d->prevTempArray_ =
        tempArray.substr(d->lackIndex_, tempArray.size() - d->lackIndex_);
  } else {
    d->prevTempArray_.push_back(tempArray.back());
  }
}

std::vector<NProtocolBase *> NFrameExtraction::protocols() const {
  return d->protocols_;
}

void NFrameExtraction::appendProtocol(NProtocolBase *protocol) {

  assert(std::find(d->protocols_.begin(), d->protocols_.end(), protocol) ==
         d->protocols_.end());
  d->protocols_.push_back(protocol);
}

NFrameExtraction_P::~NFrameExtraction_P() {
  for (auto protocol : protocols_) {
    delete protocol;
  }
}
