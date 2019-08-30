#ifndef NFRAMEEXTRACTION_H
#define NFRAMEEXTRACTION_H

//#include <memory>
#include <string>
#include <vector>

class NProtocolBase;
class NFrameExtraction_P;

class NFrameExtraction {
public:
  explicit NFrameExtraction();
  ~NFrameExtraction();

  std::vector<NProtocolBase *> protocols() const;

  void appendProtocol(NProtocolBase *protocol);

  //  void removeProtocol(NProtocolBase *protocol);

  void unpackData(const std::string &data);

private:
  NFrameExtraction_P *d;
  //  NFrameExtraction_P *d;
  //  Q_DISABLE_COPY(NFrameExtraction)
};

#endif // NFRAMEEXTRACTION_H
