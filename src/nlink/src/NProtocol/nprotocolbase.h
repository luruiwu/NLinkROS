#ifndef NPROTOCOLBASE_H
#define NPROTOCOLBASE_H

#include "nprotocolitems.h"
#include <functional>
#include <string>
#include <vector>

class NProtocolBase_P;

class NProtocolBase {
public:
  explicit NProtocolBase();
  virtual ~NProtocolBase();

  size_t totalBytes() const;

  std::vector<NProtocolItemBase *> items() const;

  virtual std::string updateCommand() { return ""; }

  void newPocketResolved();

  void setDataHandle(std::function<void()> handle);

protected:
  void appendItem(NProtocolItemBase *item);
  virtual void unpackData(const std::string &) = 0;

private:
  NProtocolBase_P *d;
  friend class NFrameExtraction;
};

#endif // NPROTOCOLBASE_H
