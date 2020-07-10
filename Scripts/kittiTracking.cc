/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if(argc != 2)
    {
        cerr << endl << "Usage: ./kittiTracking sequence_id" << endl;
        return 1;
    }

    remove("Outputs/IniMatches.txt");
    remove("Outputs/KeyPoints.txt");
    remove("Outputs/PrevMatched.txt");
    remove("Outputs/Poses.txt");
    remove("Outputs/MaskKeyPoints.txt");

    string vocabFile = "Vocabulary/ORBvoc.txt";
    string settingsFile = "Examples/Monocular/KITTI03.yaml";
    string imageDir = "/home/andylai2/group-data/kitti_data/data_tracking_image_2/training/image_02/";
    string maskDir = "/home/andylai2/group-data/kitti_data/data_tracking_image_2/training/masks/";

    // Retrieve paths to images
    string sequenceId = argv[1];
    string strFile = imageDir + "/" + sequenceId + "/" + sequenceId + ".txt";
    vector<string> vstrImageFilenames;
    vector<string> vstrMaskFilenames;
    vector<double> vTimestamps;
    LoadImages(strFile, vstrImageFilenames, vTimestamps);

    int nImages = vstrImageFilenames.size();


    // cv::Mat mask = cv::imread(maskFile,0);
    // vector<uint> uniqueObjs = matUnique(mask, true);
    // cout << mask.at<uint>(0,0) << endl;
    // cout << "Mask Unique Id's: ";
    // for(size_t i = 0; i < uniqueObjs.size(); i++)
    // {
    //     cout << uniqueObjs[i] << " ";
    // }
    // cout << endl;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(vocabFile,settingsFile,ORB_SLAM2::System::MONOCULAR,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat im;
    cv::Mat mask;
    cout << "Starting main loop" << endl << endl;

    int frameIndices[] = {200,201};
    for(size_t i=0; i<sizeof(frameIndices); i++)
    // for(int ni = 0; ni < 50; ni++);
    {
        // Read image from file

        int ni = frameIndices[i];
        // im = cv::imread(string(argv[1])+"/"+vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        im = cv::imread(imageDir+sequenceId+"/"+vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        mask = cv::imread(maskDir+sequenceId+"/"+vstrImageFilenames[ni],0);
        double tframe = vTimestamps[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << imageDir << sequenceId << "/" << vstrImageFilenames[ni] << endl;
            return 1;
        }
        if(mask.empty())
        {
            cerr << endl << "Failed to load mask at: "
                 << maskDir << sequenceId << "/" << vstrImageFilenames[ni] << endl;
            return 1;
        }


#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        cout << "-------" << endl;
        cout << "Processing frame " << ni << endl;
        // Pass the image to the SLAM system
        SLAM.TrackMonocularMasked(im,mask,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}


void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream f;
    f.open(strFile.c_str());

    while(!f.eof())
    {
        string s;
        getline(f,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenames.push_back(sRGB);
        }
    }
}
