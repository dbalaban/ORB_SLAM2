#include "JSONwriter.h"
namespace ORB_SLAM2
{
JSONwriter::JSONwriter() : mappoints(Json::arrayValue),
                           keyframes(Json::arrayValue) {}

void JSONwriter::AddMapPoint(MapPoint& mp) {
  Json::Value jmp;
  Json::Value jpos(Json::arrayValue);
  Json::Value jframes(Json::arrayValue);

  cv::Mat pos = mp.GetWorldPos();
  jpos.append(Json::Value(pos.at<float>(0,0)));
  jpos.append(Json::Value(pos.at<float>(1,0)));
  jpos.append(Json::Value(pos.at<float>(2,0)));
  jmp["position"] = jpos;

  std::map<KeyFrame*,size_t> keyframes = mp.GetObservations();
  for (auto kf : keyframes) {
    size_t id = kf.first->id_;
    size_t idx = kf.second;
    jframes["id"] = Json::Value((unsigned) id);
    jframes["index"] = Json::Value((unsigned) idx);
  }
  jmp["keyframes"] = jframes;

  mappoints.append(jmp);
}

void JSONwriter::AddKeyFrame(KeyFrame& kf) {
  Json::Value jkf;
  Json::Value jquat(Json::arrayValue);
  Json::Value jpos(Json::arrayValue);

  cv::Mat R = kf.GetRotation().t();
  std::vector<float> q = Converter::toQuaternion(R);
  cv::Mat t = kf.GetCameraCenter();

  jkf["id"] = Json::Value((unsigned) kf.id_);
  jkf["timestamp"] = Json::Value(kf.mTimeStamp);

  jquat.append(Json::Value(q[0]));
  jquat.append(Json::Value(q[1]));
  jquat.append(Json::Value(q[2]));
  jquat.append(Json::Value(q[3]));
  jkf["quaternion"] = jquat;

  jpos.append(Json::Value(t.at<float>(0)));
  jpos.append(Json::Value(t.at<float>(1)));
  jpos.append(Json::Value(t.at<float>(2)));
  jkf["position"] = jpos;

  keyframes.append(jkf);
}

void JSONwriter::WriteToFile(string fname) {
  root["keyframes"] = keyframes;
  root["map points"] = mappoints;

  ofstream f;
  f.open(fname.c_str());
  f << root;
  f.close();
}

}