#ifndef NPROTOCOLITEMBASE_H
#define NPROTOCOLITEMBASE_H

#include <cstddef>
class NProtocolItemBase {
public:
  explicit NProtocolItemBase() = default;
  virtual ~NProtocolItemBase() = default;

  virtual bool verifyData(const char *);

  virtual bool updateItemBytes(const char *);

  inline size_t address() const { return address_; }
  inline void setAddress(size_t address) { address_ = address; }

  inline size_t bytes() const { return bytes_; }
  inline void setBytes(size_t bytes) { bytes_ = bytes; }

private:
  size_t address_;
  size_t bytes_;
};

#endif // NPROTOCOLITEMBASE_H
