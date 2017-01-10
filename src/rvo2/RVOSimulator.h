
#ifndef RVO_RVO_SIMULATOR_H_
#define RVO_RVO_SIMULATOR_H_


#include <cstddef>
#include <limits>
#include <vector>

#include "../Vec2.h"
#include "KdTree.h"

namespace RVO {
	/**
	 * \brief       Error value.
	 *
	 * A value equal to the largest unsigned integer that is returned in case
	 * of an error by functions in RVO::RVOSimulator.
	 */
	const size_t RVO_ERROR = std::numeric_limits<size_t>::max();

	/**
	 * \brief      Defines a directed line.
	 */
	class Line {
	public:
		/**
		 * \brief     A point on the directed line.
		 */
		Vec2 point;

		/**
		 * \brief     The direction of the directed line.
		 */
		Vec2 direction;
	};

	class Agent;
	class KdTree;
	class Obstacle;

	/**
	 * \brief      Defines the simulation.
	 *
	 * The main class of the library that contains all simulation functionality.
	 */
	class RVOSimulator 
    {
	public:
		RVOSimulator();
		~RVOSimulator();
        void clearAgents();
        void clearObstacles();

		/**
		 * \brief      Adds a new obstacle to the simulation.
		 * \param      vertices        List of the vertices of the polygonal
		 *             obstacle in counterclockwise order.
		 * \return     The number of the first vertex of the obstacle,
		 *             or RVO::RVO_ERROR when the number of vertices is less than two.
		 * \note       To add a "negative" obstacle, e.g. a bounding polygon around
		 *             the environment, the vertices should be listed in clockwise
		 *             order.
		 */
		size_t addObstacle(const std::vector<Vec2> &vertices);

        void addAgent(Agent* agent);

		void doStep(float timeStep);


		void processObstacles();



        void setupBlocks();
        void setPreferredVelocities(bool rnd);

		std::vector<Agent *> agents_;
		//Agent *defaultAgent_;
		float globalTime_;
		KdTree kdTree_;
		std::vector<Obstacle *> obstacles_;
		//float timeStep_;

		friend class Agent;
		friend class KdTree;
		friend class Obstacle;
	};
}

#endif /* RVO_RVO_SIMULATOR_H_ */
