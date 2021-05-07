#include <algorithm>	
#include <iostream>
#include "iou_tracker.h"

namespace iou_tracker{

void BBox::toString(){
	std::cout << "BBox:\n\txmin:" << xmin <<  
	"\tymin: " << ymin << 
	"\txmax: " << xmax <<
	"\tymax: " << ymax << std::endl; 
}

void Tracklet::toString(){
	std::cout << "Tracklet:\n" << std::endl;
	last_box.toString();
	std::cout << "Tracklet:\n" <<
	"\n\tid: "<< id << 
	"\n\tinvisible_frame: " << invisible_frame << std::endl; 
}

IOUTracker::IOUTracker() {
}

IOUTracker::~IOUTracker() {}


void IOUTracker::Record::toString(){
	std::cout << "iou: " << iou << std::endl;
	ptr_tracklet->toString();
	ptr_bbox->toString();
}

float IOUTracker::calc_iou(BBox b1, BBox b2) {
	if ((b1.xmax <= b2.xmin) || (b1.ymax <= b2.ymin) || (b1.ymin >= b2.ymax) || (b1.xmin >= b2.xmax)) {
		return 0;
	}
	else {
		float left = std::max(b1.xmin, b2.xmin);
		float right = std::min(b1.xmax, b2.xmax);
		float up = std::max(b1.ymin, b2.ymin);
		float down = std::min(b1.ymax, b2.ymax);
		float intersection = abs((right - left)*(up - down));
		float total_area = abs((b1.xmax - b1.xmin)*(b1.ymax - b1.ymin)) + abs((b2.xmax - b2.xmin)*(b2.ymax - b2.ymin));
		return 1/(total_area/intersection - 1);
	}
}

void IOUTracker::StartTrack(std::vector<BBox> &detectResults) {
	// 1. calculate all ious between existing tracklets and detect boxes
	if (!tracklets.empty() && !detectResults.empty()){
		std::cout << "calculate iou table" << std::endl;
		for (BBox &bbox: detectResults){
			for (Tracklet* tracklet_ptr: tracklets){
				float iou = calc_iou(bbox, tracklet_ptr->last_box);
				if (iou > IOU_THRES){
					bbox.toString();
					table_iou.push_back(Record{iou, tracklet_ptr, &bbox});
				}
			}
		}
	}
	for (Record &record: table_iou){
		record.toString();
	}
	std::sort(table_iou.begin(), table_iou.end());

	// 2. update existing trackers
	for (Record record: table_iou){
		std::cout << "update existing trackers" << std::endl;

		if ((updated_tracklets.find(record.ptr_tracklet) == updated_tracklets.end()) && 
		(updated_bboxs.find(record.ptr_bbox) == updated_bboxs.end())){

			updated_tracklets.insert(record.ptr_tracklet);
			updated_bboxs.insert(record.ptr_bbox);
			// update
			record.ptr_tracklet->last_box = *(record.ptr_bbox);
			record.ptr_tracklet->state = Tracklet::TrackletState::COMFIRMED;
			record.ptr_tracklet->invisible_frame = 0;
		}
	}

	// 3. change state of unupdated trackers
	std::set<Tracklet*>::iterator iter_tracklet = tracklets.begin();
	while (iter_tracklet != tracklets.end()){
		std::cout << "change state of unupdated trackers" << std::endl;

		if (updated_tracklets.find(*iter_tracklet) == updated_tracklets.end()){
			(*iter_tracklet)->invisible_frame++;
			(*iter_tracklet)->state = Tracklet::TrackletState::INVISIBLE;
			// remove disappeared trackers
			if ((*iter_tracklet)->invisible_frame > MAX_INVISIBLE_FRAME){
				(*iter_tracklet)->state = Tracklet::TrackletState::DELETED;
				delete *iter_tracklet;
				std::cout << "delete tracklet" << std::endl;

				tracklets.erase(iter_tracklet++);
			}
		} else {
			iter_tracklet++;
		}
	}
	
	// 4. add non-updated boxes to new tracklets
	for (BBox &bbox: detectResults){
		std::cout << "add unupdated boxes to new tracklets" << std::endl;
		if (updated_bboxs.find(&bbox) == updated_bboxs.end()){
			tracklets.insert(new Tracklet{bbox, id_cnt++, 0});
		}
	} 

	updated_bboxs.clear();
	updated_tracklets.clear();
	table_iou.clear();
}

void IOUTracker::println() {
	for(auto ptr_tracklet: tracklets){
		ptr_tracklet->toString();
		}

}

} // namespace iou_tracker