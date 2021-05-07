#ifndef IOU_TRACKER_H
#define IOU_TRACKER_H

#include <vector>
#include <set>
#include <string>

namespace iou_tracker {

struct BBox{
	float xmin;
	float ymin;
	float xmax;
	float ymax;
	float score;
	int id;
	void toString();
};

struct Tracklet{
	enum TrackletState{
		// 可能跟踪状态
		TENTATIVE = 0,
		// 确认跟踪状态
		COMFIRMED = 1,
		// 消失状态
		INVISIBLE = 2,
		// 删除状态
		DELETED = 3
	};
	BBox last_box;
	int id;
	int invisible_frame;
	TrackletState state = TENTATIVE;
	
	void toString();
};

class IOUTracker {
public:
	IOUTracker();
	~IOUTracker();
	void println();
	void StartTrack(std::vector<BBox> &detectResults);

private:
	struct Record{
		float iou;
		Tracklet *ptr_tracklet;
		BBox *ptr_bbox;
		bool operator < (const Record &record) const{
			return iou < record.iou;
		}
		void toString();
	};
	// 最大追踪轨迹个数
	int MAX_NUM_TRACKLET = 10;
	// 最大消失时间
	int MAX_INVISIBLE_FRAME = 3;
	// iou阈值
	float IOU_THRES = 0.3f;
	// todo：ID回收
	int id_cnt = 0;

	// iou 轨迹映射表
	std::vector<Record> table_iou;
	// 该帧更新过的轨迹集合
	std::set<Tracklet*> updated_tracklets;
	// 该帧更新过的框集合
	std::set<BBox*> updated_bboxs;
	// 轨迹集合
	std::set<Tracklet*> tracklets;
	// std::vector<Tracklet> tracklets;

	float calc_iou(BBox b1, BBox b2);

};

} // namespace iou_tracker

#endif