
#ifndef RVO_OBSTACLE_H_
#define RVO_OBSTACLE_H_


#include "Definitions.h"

namespace RVO {

	class Obstacle {
	private:

		Obstacle() : isConvex_(false), nextObstacle_(NULL), prevObstacle_(NULL), id_(0) { }

		bool isConvex_;
		Obstacle *nextObstacle_;
		Vec2 point_;
		Obstacle *prevObstacle_;
		Vec2 unitDir_;

		size_t id_;

		friend class Agent;
		friend class KdTree;
		friend class RVOSimulator;
	};
}

#endif /* RVO_OBSTACLE_H_ */
