#include "init.h"
#include "../NLink/LinkTrack/nanchorframe0.h"
#include "../NLink/LinkTrack/nnodeframe0.h"
#include "../NLink/LinkTrack/nnodeframe1.h"
#include "../NLink/LinkTrack/nnodeframe2.h"
#include "../NLink/LinkTrack/ntagframe0.h"
#include "../NProtocol/ncommon.h"
#include "../NProtocol/nframeextraction.h"
#include <nlink/anchor_frame0.h>
#include <nlink/node_frame0.h>
#include <nlink/node_frame1.h>
#include <nlink/node_frame2.h>
#include <nlink/tag_frame0.h>
#include <ros/ros.h>

#define ASSIGN_VECTOR_3D(dst, src)                                             \
  dst.x = src.x();                                                             \
  dst.y = src.y();                                                             \
  dst.z = src.z();

#define COUNT_OF(array) sizeof(array) / sizeof(array[0])

namespace LinkTrack {

Init::Init(NFrameExtraction *frameExtraction) {

  initAnchorFrame0(frameExtraction);
  initTagFrame0(frameExtraction);
  initNodeFrame0(frameExtraction);
  initNodeFrame1(frameExtraction);
  initNodeFrame2(frameExtraction);

  //  test(frameExtraction);
}

void Init::initAnchorFrame0(NFrameExtraction *frameExtraction) {
  auto protocol = new NAnchorFrame0;
  frameExtraction->appendProtocol(protocol);
  protocol->setDataHandle([=] {
    if (!publishers_[protocol]) {
      ros::NodeHandle nodeHandle;
      publishers_[protocol] = nodeHandle.advertise<nlink::anchor_frame0>(
          "nlink_linktrack_anchorframe0", 50);
    }
    auto data = protocol->data();
    nlink::anchor_frame0 msgData;

    msgData.id = data.id;
    msgData.supplyVoltage = data.supplyVoltage();
    msgData.systemTime = data.systemTime;
    auto &msgTags = msgData.tags;
    const auto &tags = data.tags;
    msgTags.clear();
    decltype(msgData.tags)::value_type msgTag;
    for (size_t i = 0, icount = COUNT_OF(tags); i < icount; ++i) {
      if (tags[i].id == 0xff)
        continue;
      msgTag.id = tags[i].id;
      ASSIGN_VECTOR_3D(msgTag.position, tags[i].position)
      for (size_t j = 0, jcount = COUNT_OF(tags[0].anchors); j < jcount; ++j) {

        msgTag.anchors[j].id = tags[i].anchors[j].id;
        msgTag.anchors[j].distance = tags[i].anchors[j].distance();
      }

      msgTags.push_back(msgTag);
    }

    publishers_.at(protocol).publish(msgData);
  });
}

void Init::initTagFrame0(NFrameExtraction *frameExtraction) {
  auto protocol = new NTagFrame0;
  frameExtraction->appendProtocol(protocol);
  protocol->setDataHandle([=] {
    if (!publishers_[protocol]) {
      ros::NodeHandle nodeHandle;
      publishers_[protocol] = nodeHandle.advertise<nlink::tag_frame0>(
          "nlink_linktrack_tagframe0", 50);
    }
    const auto data = protocol->data();
    nlink::tag_frame0 msgData;

    msgData.id = data.id;
    ASSIGN_VECTOR_3D(msgData.position, data.position)
    ASSIGN_VECTOR_3D(msgData.velocity, data.velocity)
    for (size_t i = 0, count = COUNT_OF(data.distances); i < count; ++i) {
      msgData.distances[i] = data.distances[i]();
    }
    ASSIGN_VECTOR_3D(msgData.imuGyro, data.imuGyro)
    ASSIGN_VECTOR_3D(msgData.imuAcc, data.imuAcc)
    ASSIGN_VECTOR_3D(msgData.angle, data.angle)

    for (size_t i = 0, count = COUNT_OF(data.quaternions); i < count; ++i) {
      msgData.quaternions[i] = data.quaternions[i];
    }
    msgData.systemTime = data.systemTime;
    msgData.accWorkNormal = data.sensorStatus.accWorkNormal;
    msgData.accIsOnLine = data.sensorStatus.accIsOnline;
    msgData.gyroWorkNormal = data.sensorStatus.gyroWorkNormal;
    msgData.gyroIsOnLine = data.sensorStatus.gyroIsOnline;
    ASSIGN_VECTOR_3D(msgData.eop, data.eop)
    msgData.supplyVoltage = data.supplyVoltage();

    publishers_.at(protocol).publish(msgData);
  });
}

void Init::initNodeFrame0(NFrameExtraction *frameExtraction) {
  auto protocol = new NNodeFrame0;
  frameExtraction->appendProtocol(protocol);
  protocol->setDataHandle([=] {
    if (!publishers_[protocol]) {
      ros::NodeHandle nodeHandle;
      publishers_[protocol] = nodeHandle.advertise<nlink::node_frame0>(
          "nlink_linktrack_nodeframe0", 50);
    }
    const auto data = protocol->data();
    const auto nodes = protocol->currentNodes();
    nlink::node_frame0 msgData;
    auto &msgNodes = msgData.nodes;

    msgData.role = data.role;
    msgData.id = data.id;

    msgNodes.resize(nodes.size());
    for (size_t i = 0, count = nodes.size(); i < count; ++i) {
      msgNodes[i].id = nodes.at(i).id;
      msgNodes[i].role = nodes.at(i).role;
      msgNodes[i].dataArray = nodes.at(i).dataArray;
    }

    publishers_.at(protocol).publish(msgData);
  });
}

void Init::initNodeFrame1(NFrameExtraction *frameExtraction) {
  auto protocol = new NNodeFrame1;
  frameExtraction->appendProtocol(protocol);
  protocol->setDataHandle([=] {
    if (!publishers_[protocol]) {
      ros::NodeHandle nodeHandle;
      publishers_[protocol] = nodeHandle.advertise<nlink::node_frame1>(
          "nlink_linktrack_nodeframe1", 50);
    }
    const auto data = protocol->data();
    const auto nodes = protocol->currentNodes();
    nlink::node_frame1 msgData;
    auto &msgNodes = msgData.nodes;

    msgData.role = data.role;
    msgData.id = data.id;
    msgData.systemTime = data.systemTime;
    msgData.supplyVoltage = data.supplyVoltage();

    msgNodes.resize(nodes.size());
    for (size_t i = 0, count = nodes.size(); i < count; ++i) {
      msgNodes[i].id = nodes.at(i).id;
      msgNodes[i].role = nodes.at(i).role;
      ASSIGN_VECTOR_3D(msgNodes[i].position, nodes.at(i).position)
    }

    publishers_.at(protocol).publish(msgData);
  });
}

void Init::initNodeFrame2(NFrameExtraction *frameExtraction) {

  auto protocol = new NNodeFrame2;
  frameExtraction->appendProtocol(protocol);
  protocol->setDataHandle([=] {
    if (!publishers_[protocol]) {
      ros::NodeHandle nodeHandle;
      publishers_[protocol] = nodeHandle.advertise<nlink::node_frame2>(
          "nlink_linktrack_nodeframe2", 50);
    }
    const auto data = protocol->data();
    const auto nodes = protocol->currentNodes();
    nlink::node_frame2 msgData;
    auto &msgNodes = msgData.nodes;

    msgData.role = data.role;
    msgData.id = data.id;
    msgData.systemTime = data.systemTime;
    ASSIGN_VECTOR_3D(msgData.eop, data.eop)
    ASSIGN_VECTOR_3D(msgData.position, data.position)
    ASSIGN_VECTOR_3D(msgData.velocity, data.velocity)

    ASSIGN_VECTOR_3D(msgData.imuGyro, data.imuGyro)
    ASSIGN_VECTOR_3D(msgData.imuAcc, data.imuAcc)
    ASSIGN_VECTOR_3D(msgData.angle, data.angle)

    for (size_t i = 0, count = COUNT_OF(data.quaternions); i < count; ++i) {
      msgData.quaternions[i] = data.quaternions[i];
    }

    msgData.supplyVoltage = data.supplyVoltage();

    msgNodes.resize(nodes.size());
    for (size_t i = 0, count = nodes.size(); i < count; ++i) {
      msgNodes[i].id = nodes.at(i).id;
      msgNodes[i].role = nodes.at(i).role;
      msgNodes[i].distance = nodes.at(i).distance();
      msgNodes[i].fpRssi = nodes.at(i).fpRssi();
      msgNodes[i].rxRssi = nodes.at(i).rxRssi();
      msgNodes[i].systemTime = nodes.at(i).systemTime;
    }

    publishers_.at(protocol).publish(msgData);
  });
}

void Init::test(NFrameExtraction *frameExtraction) {
  //测试数据
  frameExtraction->unpackData(
      NCommon::StringToHex("55 00 00 00 a4 09 00 2a 0a 00 ff ff ff 00 fe 0d "
                           "00 01 c7 0b 00 02 ef 0c "
                           "00 03 84 0e 00 ff 00 01 0e 44 12 b0 f8 ee 61 b5 "
                           "c8 e0 a4 3b 6b 89 56 bb "
                           "5a bc ad 3b 4b 67 e3 c0 02 00 58 09 00 24 0a 00 "
                           "e8 03 00 00 6b 0e 00 01 "
                           "ad 0b 00 03 ae 0e 00 ff b2 0e 00 ff 5b 3f dd c0 "
                           "ab be 22 c6 b9 3e 8b a1 "
                           "0d 3e e4 11 e7 ef eb 50 4c 82 51 ab 7b 5b ff 9b "
                           "20 00 8a c0 60 13 00 87 "
                           "31 56 92 60 56 73 c5 e8 d2 91 04 33 9f 97 77 7d "
                           "43 ff e6 f2 dc 5b 29 79 "
                           "c4 16 01 e0 b9 2e 73 3f 99 c5 6f 52 e2 2a c1 22 "
                           "b1 b9 48 09 ff 1b e7 76 "
                           "c9 e0 1e b8 db 33 fd b7 1d 05 8b 78 f7 e7 63 7e "
                           "c9 18 19 19 e2 fa c7 ff "
                           "11 06 5e 22 76 dd fb 37 04 a0 80 10 af 9f b8 ac "
                           "51 27 a4 2a dd 0f 23 eb "
                           "8c ac ff 55 0c f7 6c a3 3e 58 8a 9b de db fa 5b "
                           "05 26 e2 4c de ae 5e 5d "
                           "c0 00 08 48 7f ff 83 39 27 01 00 a2 2d 9f ff f5 "
                           "11 9a 36 1b 65 9d fd 9c "
                           "bc e2 10 24 2a e6 5b ff ff 01 40 49 bf 1f fe 4e "
                           "94 04 94 00 5a 3c 4d 39 "
                           "24 09 06 d8 72 4b 78 ff 26 08 6d ff c5 b6 f2 40 "
                           "95 85 28 21 d7 03 b1 3b "
                           "24 7c ae b3 3e 44 bb 1c 08 20 c2 64 25 97 ff e7 "
                           "80 80 02 fc 33 c6 f8 73 "
                           "d8 ed 49 18 0a bf aa 69 d0 98 0c 00 69 7f de 59 "
                           "68 ff 06 28 40 a7 15 19 "
                           "ac 89 a4 42 74 ac bc 8e cd a4 08 b8 bf 2f f7 2e "
                           "90 31 00 62 ff 2f 8e 7d "
                           "80 89 16 b0 c1 c2 8e 34 d8 ae 64 82 e9 de 63 f6 "
                           "51 16 51 a4 2f ee d3 ff "
                           "18 00 db a0 27 1c 01 ac 44 c2 44 a0 cf 25 0e 53 "
                           "ce c4 a5 d1 92 7f 5d 19 "
                           "2b a3 ff 95 ff a3 c3 2f 5b 40 71 40 7b 13 e7 f0 "
                           "05 80 a3 00 e9 fc cd 3d "
                           "18 2c 52 9b bf ff f0 b3 4e b0 3f dc 7e 9f bb dd "
                           "a8 65 e8 03 28 a9 37 91 "
                           "10 d0 1a 35 ff c3 e6 f9 ff 28 81 a4 b6 e3 0f 7e "
                           "61 57 08 21 cf 57 a6 e5 "
                           "6e a8 41 02 da ef 11 74 1c c0 00 ff 16 fa c5 93 "
                           "3a 64 20 90 f7 29 11 b3 "
                           "44 50 70 00 ab 96 b7 36 8e 25 97 82 9e ed ff 2c "
                           "93 42 5d 48 cd eb af db "
                           "01 ab 4a 25 e7 fb 95 ff 5b 94 04 43 eb b5 bd 5b "
                           "ab ff c6 83 9f 7b 3b e7 "
                           "65 95 02 04 8c fe d5 93 72 a0 00 e0 fb b5 e5 72 "
                           "05 00 98 92 ff 8b 7d 7f "
                           "94 c2 87 2a fe ac f4 3d 09 d9 d0 c7 96 fd 22 07 "
                           "f1 a0 52 c0 f6 58 c6 ff "
                           "08 ed 91 80 eb 37 0f df 58 35 e6 c0 d7 bf 51 f2 "
                           "c2 01 00 80 fe 3e c9 0c "
                           "79 22 ff e2 e4 e3 46 55 8b a3 ca 92 cf 3c eb 66 "
                           "78 72 61 9b fb 7c 9c dc "
                           "a3 2d 43 09 7b ff f8 7b 0b 01 0e 86 7b fc 72 1f "
                           "12 50 1d 76 1d e4 d7 6f "
                           "44 fc 10 b1 be 7b 64 98 ff 24 82 86 5d 2f eb bf "
                           "22 8a e7 36 9b 6a 52 ee "
                           "00 98 1e 17 e5 ff b5 f2 48 6d 09 ff f3 53 bb 49 "
                           "99 67 64 06 2d 39 7f 75 "
                           "32 8d 02 c7 5e da f6 59 2a a8 00 c9 db f8 ff f6 "
                           "18 58 0c b5 7e df 40 db "
                           "6b 2e 8d c7 cf f7 b1 fe 0a 18 7d bb 3b 1d 7f f9 "
                           "60 ff 09 0a e5 ff b6 67 "
                           "92 85 24 8c 5b 57 9e 37 87 82 68 06 d5 75 ff ed "
                           "20 ac a1 83 ff 6b 36 96 "
                           "21 8e 20 71 dc af ce fd ac 30 60 29 6c 7d ba 8a "
                           "35 31 28 b1 d4 96 b5 ff "
                           "92 aa db 10 7e 53 ff fc 90 81 64 03 ba 3f 7a b3 "
                           "42 20 e0 41 9e f7 85 f1 "
                           "ca 64 ff 96 f9 d5 4e 5d 88 1d 91 8d 3e 1f ad f8 "
                           "08 de 86 10 c3 85 aa 4a "
                           "13 7c 2d 01 00 00 7e ee"));

  frameExtraction->unpackData(
      NCommon::StringToHex("55 01 00 ab 5e 0b 00 e6 0e 00 ec fb ff 83 ff ff "
                           "43 01 00 00 00 00 0d 11 "
                           "00 7b 0a 00 42 02 00 5c 10 00 00 00 00 00 00 00 "
                           "ff ff ff 00 00 00 9f d2 "
                           "12 3e 94 b0 02 3e f6 6d 41 3f 58 4b 93 c0 9a e0 "
                           "46 3e bf 37 0e 41 c2 cd "
                           "f3 3f ae aa 87 40 00 00 00 00 33 00 b3 07 bd 44 "
                           "e8 72 10 3d f8 6c 2f be "
                           "51 b3 35 3c 69 99 7b 3f 00 00 00 00 00 00 00 00 "
                           "5c 58 02 00 f0 1a 21 ff "
                           "67 13 82 02 56 73 d3 2b"));

  frameExtraction->unpackData(NCommon::StringToHex(
      "55 02 42 00 01 00 d1 2c c3 88 02 02 00 09 00 11 22 33 44 55 66 77 "
      "88 "
      "99 "
      "02 02 25 00 11 12 23 22 32 44 34 54 55 65 67 76 67 87 77 99 aa a2 "
      "13 "
      "45 "
      "57 65 56 56 56 56 57 78 43 33 34 44 44 44 44 46 76 0d"));

  frameExtraction->unpackData(NCommon::StringToHex(
      "55 03 44 00 03 00 54 64 01 00 ff ff ff 00 a7 0e 00 02 82 0c 02 02 "
      "42 "
      "09 "
      "43 13 02 02 00 c2 07 00 51 08 00 ff ff ff 61 b5 c8 e0 a4 3b 6b 89 "
      "56 "
      "02 "
      "02 8c 07 00 43 07 00 e8 03 00 34 09 00 89 07 00 e8 03 00 cd"));

  frameExtraction->unpackData(
      NCommon::StringToHex("55 04 ac 00 02 00 ff c1 88 00 02 03 03 50 07"
                           "00 "
                           "95 0c 00 7a 01 00 aa ed "
                           "ff 36 03 00 00 00 00 b1 a2 00 00 00 00 ee 60"
                           "b5 "
                           "ad b2 96 bf 60 d1 26 be "
                           "b6 6a a4 3f 53 9d ae 3d 82 60 ae 40 f5 fa 1a"
                           "41 "
                           "55 1b 10 3f 9d 2e 6f 40 "
                           "00 00 00 00 aa 08 b1 06 10 cc 5c 8c c5 3e ee"
                           "6d "
                           "57 3e 22 df ee bd dd 85 "
                           "63 bf a0 10 e7 ef eb 50 4c 8a 52 8b 7b 5b bf"
                           "9b "
                           "10 00 8a 40 62 13 04 01 "
                           "00 03 0f 00 ab 9d eb c1 88 00 0c 33 01 01 2d"
                           "08 "
                           "00 a3 9d eb c1 88 00 79 "
                           "c4 01 02 d6 0c 00 a7 a1 eb c1 88 00 e0 2a 01"
                           "03 "
                           "c1 11 00 a6 9f eb c1 88 "
                           "00 80 1e 3e"));
}

} // namespace LinkTrack
