
#ifndef RVO_OBSTACLE_H_
#define RVO_OBSTACLE_H_


#include "Definitions.h"

namespace RVO {

	class Obstacle 
    {
    public:
		Obstacle() : isConvex_(false), nextObstacle_(NULL), prevObstacle_(NULL), id_(0) { }

		bool isConvex_;
		Obstacle *nextObstacle_;
		Vec2 point_;
		Obstacle *prevObstacle_;
		Vec2 unitDir_;

		int id_;
	};
}

#endif /* RVO_OBSTACLE_H_ */
