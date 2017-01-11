
#ifndef RVO_KD_TREE_H_
#define RVO_KD_TREE_H_


#include "Definitions.h"

namespace RVO {
	/**
	 * \brief      Defines <i>k</i>d-trees for agents and static obstacles in the
	 *             simulation.
	 */
	class KdTree 
    {
	public:
		class AgentTreeNode {
		public:
			int begin;
			int end;
			int left;
			float maxX;
			float maxY;
			float minX;
			float minY;
			int right;
		};

		class ObstacleTreeNode {
		public:
			ObstacleTreeNode *left;
			const Obstacle *obstacle;
			ObstacleTreeNode *right;
		};

		explicit KdTree(RVOSimulator *sim);
        void clear();

		~KdTree();
		void buildAgentTree();
		void buildAgentTreeRecursive(int begin, int end, int node);
		void buildObstacleTree();

		ObstacleTreeNode *buildObstacleTreeRecursive(const std::vector<Obstacle *> &obstacles);

		/**
		 * \brief      Computes the agent neighbors of the specified agent.
		 * \param      agent           A pointer to the agent for which agent
		 *                             neighbors are to be computed.
		 * \param      rangeSq         The squared range around the agent.
		 */
		void computeAgentNeighbors(Agent *agent, float &rangeSq) const;

		/**
		 * \brief      Computes the obstacle neighbors of the specified agent.
		 * \param      agent           A pointer to the agent for which obstacle
		 *                             neighbors are to be computed.
		 * \param      rangeSq         The squared range around the agent.
		 */
		void computeObstacleNeighbors(Agent *agent, float rangeSq) const;

		/**
		 * \brief      Deletes the specified obstacle tree node.
		 * \param      node            A pointer to the obstacle tree node to be
		 *                             deleted.
		 */
		void deleteObstacleTree(ObstacleTreeNode *node);

		void queryAgentTreeRecursive(Agent *agent, float &rangeSq,
									 size_t node) const;

		void queryObstacleTreeRecursive(Agent *agent, float rangeSq,
										const ObstacleTreeNode *node) const;

		/**
		 * \brief      Queries the visibility between two points within a
		 *             specified radius.
		 * \param      q1              The first point between which visibility is
		 *                             to be tested.
		 * \param      q2              The second point between which visibility is
		 *                             to be tested.
		 * \param      radius          The radius within which visibility is to be
		 *                             tested.
		 * \return     True if q1 and q2 are mutually visible within the radius;
		 *             false otherwise.
		 */
		bool queryVisibility(const Vec2 &q1, const Vec2 &q2,
							 float radius) const;

		bool queryVisibilityRecursive(const Vec2 &q1, const Vec2 &q2,
									  float radius,
									  const ObstacleTreeNode *node) const;

		std::vector<Agent *> agents_;
		std::vector<AgentTreeNode> agentTree_;
		ObstacleTreeNode *obstacleTree_;
		RVOSimulator *sim_;

		static const size_t MAX_LEAF_SIZE = 10;

		friend class Agent;
		friend class RVOSimulator;
	};
}

#endif /* RVO_KD_TREE_H_ */
