#ifndef JSONWRITER_H
#define JSONWRITER_H

#include <jsoncpp/json/json.h>
#include <jsoncpp/json/value.h>
#include <fstream>
#include <string>
#include <vector>

#include <System.h>

namespace ORB_SLAM2{

class JSONwriter {

public:
  JSONwriter();
  void AddMapPoint(MapPoint& mp);
  void AddKeyFrame(KeyFrame& kf);
  void WriteToFile(std::string fname);

private:
  Json::Value root;
  Json::Value mappoints;
  Json::Value keyframes;

};

}

#endif