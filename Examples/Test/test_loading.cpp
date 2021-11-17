#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <jsoncpp/json/json.h>
#include <jsoncpp/json/value.h>
#include <fstream>

#include <opencv2/core/core.hpp>

#include <System.h>

using namespace std;

void LoadImages(const string read_f,
                vector<string>& fnames,
                vector<double>& timestamps) {
    ifstream fmap;
    string line;
    string delim = " ";

    fmap.open(read_f);
    while (getline(fmap, line)) {
        if (line.at(0) == '#') {
            continue;
        }
        size_t pos = line.find(delim);
        const double time = std::stod(line.substr(0, pos));
        line.erase(0, pos + delim.length());

        fnames.push_back(line);
        timestamps.push_back(time);
    }
    fmap.close();
}

int main(int argc, char **argv)
{
    string directory = "/home/david/VideoSegmentationAnnotator/ORB_SLAM2/";
    string vocab = directory + "Vocabulary/ORBvoc.txt";
    string settings = directory + "Examples/Monocular/TUM1.yaml";
    string sequence = directory + "rgbd_dataset_freiburg1_xyz";

     // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    string strFile = sequence + "/rgb.txt";
    LoadImages(strFile, vstrImageFilenames, vTimestamps);

    int nImages = vstrImageFilenames.size();

    cout << "starting SLAM system\n";
    ORB_SLAM2::System SLAM(vocab,settings,ORB_SLAM2::System::MONOCULAR,true);
    cout << "loading map\n";
    SLAM.LoadMapMonocular("keyframe_map.json", sequence+"/rgb");
    for(unsigned int i = 0; i < 290; i++) {
        SLAM.drawKeyFrame(i);
        usleep(1e4);
    }

    for (size_t i = 0; i < vTimestamps.size(); i++) {
      cv::Mat im = cv::imread(sequence+"/"+vstrImageFilenames[i],CV_LOAD_IMAGE_UNCHANGED);
      double tframe = vTimestamps[i];
      cv::Mat track = SLAM.TrackMonocular(im, tframe);
      std::cout << "frame " << i+1 << " / " << vTimestamps.size() << " pose:\n" << track << std::endl;
      usleep(1e5);
    }

    while(true) {
        sleep(1);
    }

    return 0;
}