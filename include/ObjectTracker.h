#ifndef OBJECTTRACKER_H
#define OBJECTTRACKER_H

// Include list TBD
// Do we need to incl something for DBOW2?
#include "System.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"

namespace ORB_SLAM2
{
class ObjectTracker
{
public:
	/*public function:
	update(new_object_in_some_representation)
	getNextId()?
	*/
	//Constructor with ?? parameters
	ObjectTracker();
	void Update(const std::vector< std::pair<int,int> > &vCentroids, vector<DBoW2::BowVector> vBowVecs);

private:
	/* 
	Private functions:
	register(centroid)
	deregister(centroid)

	Private attributes:
	int nextObjectID
	dict objects
	dict disappeared
	*/

	// After which frame we stop tracking said object
	int MaxDisappeared;
	std::map<int, std::tuple<int, int, DBoW2::BowVector> > mdObjects;
	std::map<int, int> mdDisappeaered; 

	int mnNextObjId;

	void Register(const std::pair<int,int> &Centroid, DBoW2::BowVector &BowVec);
	void Deregister(const int objId);

};


}

#endif //OBJECTTRACKER_H