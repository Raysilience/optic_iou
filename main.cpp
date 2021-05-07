#include "iou_tracker.h"
#include <iostream>
#include <vector>
#include <set>

using namespace iou_tracker;

int main() {

	std::cout << "start test" << std::endl;
	IOUTracker *my_tracker = new IOUTracker();

	std::vector<BBox> detectResults;
	detectResults.push_back({100,200,300,300});

	my_tracker->StartTrack(detectResults);
	my_tracker->println();

	detectResults.clear();
	detectResults.push_back({100,400,300,500});
	detectResults.push_back({110,210,310,310});

	my_tracker->StartTrack(detectResults);
	// my_tracker->println();

	// my_tracker->StartTrack(detectResults);
	// detectResults.clear();
	// detectResults.push_back({100,200,300,300});
	// my_tracker->println();
}