#include "ObjectTracker.h"

using namespace std;

namespace ORB_SLAM2
{
void ObjectTracker::Register(const pair<int,int> &Centroid, DBoW2::BowVector &BowVec)
{
	auto ObjParameters = make_tuple(Centroid.first, Centroid.second, BowVec);
	// mdObjects[mnNextObj];
}


}