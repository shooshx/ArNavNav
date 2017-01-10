

#include "Agent.h"

#include "KdTree.h"
#include "Obstacle.h"

namespace RVO 
{
    void Agent::setTrivialPlan(bool goalIsReachable)
    {
        m_plan.setEnd(m_endGoalPos.p, m_endGoalPos.radius);
        m_indexInPlan = 0;
        m_curGoalPos = m_plan.m_d[0];
        m_goalIsReachable = goalIsReachable;
    }

    size_t linearProgram2(const std::vector<Line> &lines, float radius, const Vec2 &optVelocity, bool directionOpt, Vec2 &result);
    void linearProgram3(const std::vector<Line> &lines, size_t numObstLines, size_t beginLine, float radius, Vec2 &result);

	void Agent::computeNeighbors(KdTree& kdTree)
	{
		obstacleNeighbors_.clear();
		float rangeSq = sqr(timeHorizonObst_ * maxSpeed_ + m_radius);
		kdTree.computeObstacleNeighbors(this, rangeSq);

		agentNeighbors_.clear();

		if (maxNeighbors_ > 0) {
			rangeSq = sqr(neighborDist_);
			kdTree.computeAgentNeighbors(this, rangeSq);
		}
	}

    void Agent::computePreferredVelocity(float deltaTime)
    {
        if (m_reached && m_endGoalPos.type == GOAL_ATTACK) {
            prefVelocity_ = Vec2(0,0);
            return;
        }

        //Vec2 toGoal = m_curGoalPos.p - m_position;
        Vec2 goalPnt = m_curGoalPos->getDest(m_position);
        Vec2 toGoal = goalPnt - m_position;

	//    const float distSqToGoal = absSq(toGoal);

	 /*   if (m_curGoalPos->shouldTaper() && sqr(maxSpeed_ * deltaTime) > distSqToGoal) 
        { // close to the goal? the point goal is the final one and we should not overshoot it
		    prefVelocity_ = (toGoal) / deltaTime;
	    }
	    else 
        { // in segment goals we can overshoot the line, that's intended.
		    prefVelocity_ = (maxSpeed_ / std::sqrt(distSqToGoal)) * toGoal;
	    }*/

        prefVelocity_ = toGoal;
		if (absSq(prefVelocity_) > 1.0f) {
			prefVelocity_ = normalize(prefVelocity_);
		}
    }



	/* Search for the best new velocity. */
	void Agent::computeNewVelocity(float timeStep)
	{
		orcaLines_.clear();

		const float invTimeHorizonObst = 1.0f / timeHorizonObst_;

		/* Create obstacle ORCA lines. */
		for (size_t i = 0; i < obstacleNeighbors_.size(); ++i) {

			const Obstacle *obstacle1 = obstacleNeighbors_[i].second;
			const Obstacle *obstacle2 = obstacle1->nextObstacle_;

			const Vec2 relativePosition1 = obstacle1->point_ - m_position;
			const Vec2 relativePosition2 = obstacle2->point_ - m_position;

			/*
			 * Check if velocity obstacle of obstacle is already taken care of by
			 * previously constructed obstacle ORCA lines.
			 */
			bool alreadyCovered = false;

			for (size_t j = 0; j < orcaLines_.size(); ++j) {
				if (det(invTimeHorizonObst * relativePosition1 - orcaLines_[j].point, orcaLines_[j].direction) - invTimeHorizonObst * m_radius >= -RVO_EPSILON && det(invTimeHorizonObst * relativePosition2 - orcaLines_[j].point, orcaLines_[j].direction) - invTimeHorizonObst * m_radius >=  -RVO_EPSILON) {
					alreadyCovered = true;
					break;
				}
			}

			if (alreadyCovered) {
				continue;
			}

			/* Not yet covered. Check for collisions. */

			const float distSq1 = absSq(relativePosition1);
			const float distSq2 = absSq(relativePosition2);

			const float radiusSq = sqr(m_radius);

			const Vec2 obstacleVector = obstacle2->point_ - obstacle1->point_;
			const float s = (-relativePosition1 * obstacleVector) / absSq(obstacleVector);
			const float distSqLine = absSq(-relativePosition1 - s * obstacleVector);

			Line line;

			if (s < 0.0f && distSq1 <= radiusSq) {
				/* Collision with left vertex. Ignore if non-convex. */
				if (obstacle1->isConvex_) {
					line.point = Vec2(0.0f, 0.0f);
					line.direction = normalize(Vec2(-relativePosition1.y, relativePosition1.x));
					orcaLines_.push_back(line);
				}

				continue;
			}
			else if (s > 1.0f && distSq2 <= radiusSq) {
				/* Collision with right vertex. Ignore if non-convex
				 * or if it will be taken care of by neighoring obstace */
				if (obstacle2->isConvex_ && det(relativePosition2, obstacle2->unitDir_) >= 0.0f) {
					line.point = Vec2(0.0f, 0.0f);
					line.direction = normalize(Vec2(-relativePosition2.y, relativePosition2.x));
					orcaLines_.push_back(line);
				}

				continue;
			}
			else if (s >= 0.0f && s < 1.0f && distSqLine <= radiusSq) {
				/* Collision with obstacle segment. */
				line.point = Vec2(0.0f, 0.0f);
				line.direction = -obstacle1->unitDir_;
				orcaLines_.push_back(line);
				continue;
			}

			/*
			 * No collision.
			 * Compute legs. When obliquely viewed, both legs can come from a single
			 * vertex. Legs extend cut-off line when nonconvex vertex.
			 */

			Vec2 leftLegDirection, rightLegDirection;

			if (s < 0.0f && distSqLine <= radiusSq) {
				/*
				 * Obstacle viewed obliquely so that left vertex
				 * defines velocity obstacle.
				 */
				if (!obstacle1->isConvex_) {
					/* Ignore obstacle. */
					continue;
				}

				obstacle2 = obstacle1;

				const float leg1 = std::sqrt(distSq1 - radiusSq);
				leftLegDirection = Vec2(relativePosition1.x * leg1 - relativePosition1.y * m_radius, relativePosition1.x * m_radius + relativePosition1.y * leg1) / distSq1;
				rightLegDirection = Vec2(relativePosition1.x * leg1 + relativePosition1.y * m_radius, -relativePosition1.x * m_radius + relativePosition1.y * leg1) / distSq1;
			}
			else if (s > 1.0f && distSqLine <= radiusSq) {
				/*
				 * Obstacle viewed obliquely so that
				 * right vertex defines velocity obstacle.
				 */
				if (!obstacle2->isConvex_) {
					/* Ignore obstacle. */
					continue;
				}

				obstacle1 = obstacle2;

				const float leg2 = std::sqrt(distSq2 - radiusSq);
				leftLegDirection = Vec2(relativePosition2.x * leg2 - relativePosition2.y * m_radius, relativePosition2.x * m_radius + relativePosition2.y * leg2) / distSq2;
				rightLegDirection = Vec2(relativePosition2.x * leg2 + relativePosition2.y * m_radius, -relativePosition2.x * m_radius + relativePosition2.y * leg2) / distSq2;
			}
			else {
				/* Usual situation. */
				if (obstacle1->isConvex_) {
					const float leg1 = std::sqrt(distSq1 - radiusSq);
					leftLegDirection = Vec2(relativePosition1.x * leg1 - relativePosition1.y * m_radius, relativePosition1.x * m_radius + relativePosition1.y * leg1) / distSq1;
				}
				else {
					/* Left vertex non-convex; left leg extends cut-off line. */
					leftLegDirection = -obstacle1->unitDir_;
				}

				if (obstacle2->isConvex_) {
					const float leg2 = std::sqrt(distSq2 - radiusSq);
					rightLegDirection = Vec2(relativePosition2.x * leg2 + relativePosition2.y * m_radius, -relativePosition2.x * m_radius + relativePosition2.y * leg2) / distSq2;
				}
				else {
					/* Right vertex non-convex; right leg extends cut-off line. */
					rightLegDirection = obstacle1->unitDir_;
				}
			}

			/*
			 * Legs can never point into neighboring edge when convex vertex,
			 * take cutoff-line of neighboring edge instead. If velocity projected on
			 * "foreign" leg, no constraint is added.
			 */

			const Obstacle *const leftNeighbor = obstacle1->prevObstacle_;

			bool isLeftLegForeign = false;
			bool isRightLegForeign = false;

			if (obstacle1->isConvex_ && det(leftLegDirection, -leftNeighbor->unitDir_) >= 0.0f) {
				/* Left leg points into obstacle. */
				leftLegDirection = -leftNeighbor->unitDir_;
				isLeftLegForeign = true;
			}

			if (obstacle2->isConvex_ && det(rightLegDirection, obstacle2->unitDir_) <= 0.0f) {
				/* Right leg points into obstacle. */
				rightLegDirection = obstacle2->unitDir_;
				isRightLegForeign = true;
			}

			/* Compute cut-off centers. */
			const Vec2 leftCutoff = invTimeHorizonObst * (obstacle1->point_ - m_position);
			const Vec2 rightCutoff = invTimeHorizonObst * (obstacle2->point_ - m_position);
			const Vec2 cutoffVec = rightCutoff - leftCutoff;

			/* Project current velocity on velocity obstacle. */

			/* Check if current velocity is projected on cutoff circles. */
			const float t = (obstacle1 == obstacle2 ? 0.5f : ((m_velocity - leftCutoff) * cutoffVec) / absSq(cutoffVec));
			const float tLeft = ((m_velocity - leftCutoff) * leftLegDirection);
			const float tRight = ((m_velocity - rightCutoff) * rightLegDirection);

			if ((t < 0.0f && tLeft < 0.0f) || (obstacle1 == obstacle2 && tLeft < 0.0f && tRight < 0.0f)) {
				/* Project on left cut-off circle. */
				const Vec2 unitW = normalize(m_velocity - leftCutoff);

				line.direction = Vec2(unitW.y, -unitW.x);
				line.point = leftCutoff + m_radius * invTimeHorizonObst * unitW;
				orcaLines_.push_back(line);
				continue;
			}
			else if (t > 1.0f && tRight < 0.0f) {
				/* Project on right cut-off circle. */
				const Vec2 unitW = normalize(m_velocity - rightCutoff);

				line.direction = Vec2(unitW.y, -unitW.x);
				line.point = rightCutoff + m_radius * invTimeHorizonObst * unitW;
				orcaLines_.push_back(line);
				continue;
			}

			/*
			 * Project on left leg, right leg, or cut-off line, whichever is closest
			 * to velocity.
			 */
			const float distSqCutoff = ((t < 0.0f || t > 1.0f || obstacle1 == obstacle2) ? std::numeric_limits<float>::infinity() : absSq(m_velocity - (leftCutoff + t * cutoffVec)));
			const float distSqLeft = ((tLeft < 0.0f) ? std::numeric_limits<float>::infinity() : absSq(m_velocity - (leftCutoff + tLeft * leftLegDirection)));
			const float distSqRight = ((tRight < 0.0f) ? std::numeric_limits<float>::infinity() : absSq(m_velocity - (rightCutoff + tRight * rightLegDirection)));

			if (distSqCutoff <= distSqLeft && distSqCutoff <= distSqRight) {
				/* Project on cut-off line. */
				line.direction = -obstacle1->unitDir_;
				line.point = leftCutoff + m_radius * invTimeHorizonObst * Vec2(-line.direction.y, line.direction.x);
				orcaLines_.push_back(line);
				continue;
			}
			else if (distSqLeft <= distSqRight) {
				/* Project on left leg. */
				if (isLeftLegForeign) {
					continue;
				}

				line.direction = leftLegDirection;
				line.point = leftCutoff + m_radius * invTimeHorizonObst * Vec2(-line.direction.y, line.direction.x);
				orcaLines_.push_back(line);
				continue;
			}
			else {
				/* Project on right leg. */
				if (isRightLegForeign) {
					continue;
				}

				line.direction = -rightLegDirection;
				line.point = rightCutoff + m_radius * invTimeHorizonObst * Vec2(-line.direction.y, line.direction.x);
				orcaLines_.push_back(line);
				continue;
			}
		}

		const size_t numObstLines = orcaLines_.size();

		const float invTimeHorizon = 1.0f / timeHorizon_;

		/* Create agent ORCA lines. */
		for (size_t i = 0; i < agentNeighbors_.size(); ++i) {
			const Agent *const other = agentNeighbors_[i].second;

			const Vec2 relativePosition = other->m_position - m_position;
			const Vec2 relativeVelocity = m_velocity - other->m_velocity;
			const float distSq = absSq(relativePosition);
			const float combinedRadius = m_radius + other->m_radius;
			const float combinedRadiusSq = sqr(combinedRadius);

			Line line;
			Vec2 u;

			if (distSq > combinedRadiusSq) {
				/* No collision. */
				const Vec2 w = relativeVelocity - invTimeHorizon * relativePosition;
				/* Vector from cutoff center to relative velocity. */
				const float wLengthSq = absSq(w);

				const float dotProduct1 = w * relativePosition;

				if (dotProduct1 < 0.0f && sqr(dotProduct1) > combinedRadiusSq * wLengthSq) {
					/* Project on cut-off circle. */
					const float wLength = std::sqrt(wLengthSq);
					const Vec2 unitW = w / wLength;

					line.direction = Vec2(unitW.y, -unitW.x);
					u = (combinedRadius * invTimeHorizon - wLength) * unitW;
				}
				else {
					/* Project on legs. */
					const float leg = std::sqrt(distSq - combinedRadiusSq);

					if (det(relativePosition, w) > 0.0f) {
						/* Project on left leg. */
						line.direction = Vec2(relativePosition.x * leg - relativePosition.y * combinedRadius, relativePosition.x * combinedRadius + relativePosition.y * leg) / distSq;
					}
					else {
						/* Project on right leg. */
						line.direction = -Vec2(relativePosition.x * leg + relativePosition.y * combinedRadius, -relativePosition.x * combinedRadius + relativePosition.y * leg) / distSq;
					}

					const float dotProduct2 = relativeVelocity * line.direction;

					u = dotProduct2 * line.direction - relativeVelocity;
				}
			}
			else {
				/* Collision. Project on cut-off circle of time timeStep. */
				const float invTimeStep = 1.0f / timeStep;

				/* Vector from cutoff center to relative velocity. */
				const Vec2 w = relativeVelocity - invTimeStep * relativePosition;

				const float wLength = length(w);
				const Vec2 unitW = w / wLength;

				line.direction = Vec2(unitW.y, -unitW.x);
				u = (combinedRadius * invTimeStep - wLength) * unitW;
			}

			line.point = m_velocity + 0.5f * u;
			orcaLines_.push_back(line);
		}

		size_t lineFail = linearProgram2(orcaLines_, maxSpeed_, prefVelocity_, false, newVelocity_);

		if (lineFail < orcaLines_.size()) {
			linearProgram3(orcaLines_, numObstLines, lineFail, maxSpeed_, newVelocity_);
		}
	}

	void Agent::insertAgentNeighbor(const Agent *agent, float &rangeSq)
	{
		if (this != agent) {
			const float distSq = absSq(m_position - agent->m_position);

			if (distSq < rangeSq) {
				if (agentNeighbors_.size() < maxNeighbors_) {
					agentNeighbors_.push_back(std::make_pair(distSq, agent));
				}

				size_t i = agentNeighbors_.size() - 1;

				while (i != 0 && distSq < agentNeighbors_[i - 1].first) {
					agentNeighbors_[i] = agentNeighbors_[i - 1];
					--i;
				}

				agentNeighbors_[i] = std::make_pair(distSq, agent);

				if (agentNeighbors_.size() == maxNeighbors_) {
					rangeSq = agentNeighbors_.back().first;
				}
			}
		}
	}

	void Agent::insertObstacleNeighbor(const Obstacle *obstacle, float rangeSq)
	{
		const Obstacle *const nextObstacle = obstacle->nextObstacle_;

		const float distSq = distSqPointLineSegment(obstacle->point_, nextObstacle->point_, m_position);

		if (distSq < rangeSq) {
			obstacleNeighbors_.push_back(std::make_pair(distSq, obstacle));

			size_t i = obstacleNeighbors_.size() - 1;

			while (i != 0 && distSq < obstacleNeighbors_[i - 1].first) {
				obstacleNeighbors_[i] = obstacleNeighbors_[i - 1];
				--i;
			}

			obstacleNeighbors_[i] = std::make_pair(distSq, obstacle);
		}
	}

	bool Agent::update(float timeStep)
	{
		m_velocity = newVelocity_;
		m_position += m_velocity * timeStep;

        // goal analysis
        bool reachedEnd = false;
        if (m_curGoalPos->isPassed(m_position)) 
        {
            if (m_indexInPlan + 1 < m_plan.m_d.size()) {
                //cout << qui::g_curFrame << ": Agent " << index << " passed " << m_indexInPlan << endl;
                ++m_indexInPlan;
                m_curGoalPos = m_plan.m_d[m_indexInPlan];
            }
            else {
                reachedEnd = true;
              /*  m_curGoalPos = nullptr;
                m_indexInPlan = -1;
                m_velocity = Vec2();*/
            }
        }

        return reachedEnd;

	}

    	
    /**
	 * \relates    Agent
	 * \brief      Solves a one-dimensional linear program on a specified line
	 *             subject to linear constraints defined by lines and a circular
	 *             constraint.
	 * \param      lines         Lines defining the linear constraints.
	 * \param      lineNo        The specified line constraint.
	 * \param      radius        The radius of the circular constraint.
	 * \param      optVelocity   The optimization velocity.
	 * \param      directionOpt  True if the direction should be optimized.
	 * \param      result        A reference to the result of the linear program.
	 * \return     True if successful.
	 */
	bool linearProgram1(const std::vector<Line> &lines, size_t lineNo, float radius, const Vec2 &optVelocity, bool directionOpt, Vec2 &result)
	{
		const float dotProduct = lines[lineNo].point * lines[lineNo].direction;
		const float discriminant = sqr(dotProduct) + sqr(radius) - absSq(lines[lineNo].point);

		if (discriminant < 0.0f) {
			/* Max speed circle fully invalidates line lineNo. */
			return false;
		}

		const float sqrtDiscriminant = std::sqrt(discriminant);
		float tLeft = -dotProduct - sqrtDiscriminant;
		float tRight = -dotProduct + sqrtDiscriminant;

		for (size_t i = 0; i < lineNo; ++i) {
			const float denominator = det(lines[lineNo].direction, lines[i].direction);
			const float numerator = det(lines[i].direction, lines[lineNo].point - lines[i].point);

			if (std::fabs(denominator) <= RVO_EPSILON) {
				/* Lines lineNo and i are (almost) parallel. */
				if (numerator < 0.0f) {
					return false;
				}
				else {
					continue;
				}
			}

			const float t = numerator / denominator;

			if (denominator >= 0.0f) {
				/* Line i bounds line lineNo on the right. */
				tRight = std::min(tRight, t);
			}
			else {
				/* Line i bounds line lineNo on the left. */
				tLeft = std::max(tLeft, t);
			}

			if (tLeft > tRight) {
				return false;
			}
		}

		if (directionOpt) {
			/* Optimize direction. */
			if (optVelocity * lines[lineNo].direction > 0.0f) {
				/* Take right extreme. */
				result = lines[lineNo].point + tRight * lines[lineNo].direction;
			}
			else {
				/* Take left extreme. */
				result = lines[lineNo].point + tLeft * lines[lineNo].direction;
			}
		}
		else {
			/* Optimize closest point. */
			const float t = lines[lineNo].direction * (optVelocity - lines[lineNo].point);

			if (t < tLeft) {
				result = lines[lineNo].point + tLeft * lines[lineNo].direction;
			}
			else if (t > tRight) {
				result = lines[lineNo].point + tRight * lines[lineNo].direction;
			}
			else {
				result = lines[lineNo].point + t * lines[lineNo].direction;
			}
		}

		return true;
	}

	/**
	 * \relates    Agent
	 * \brief      Solves a two-dimensional linear program subject to linear
	 *             constraints defined by lines and a circular constraint.
	 * \param      lines         Lines defining the linear constraints.
	 * \param      radius        The radius of the circular constraint.
	 * \param      optVelocity   The optimization velocity.
	 * \param      directionOpt  True if the direction should be optimized.
	 * \param      result        A reference to the result of the linear program.
	 * \return     The number of the line it fails on, and the number of lines if successful.
	 */
	size_t linearProgram2(const std::vector<Line> &lines, float radius, const Vec2 &optVelocity, bool directionOpt, Vec2 &result)
	{
		if (directionOpt) {
			/*
			 * Optimize direction. Note that the optimization velocity is of unit
			 * length in this case.
			 */
			result = optVelocity * radius;
		}
		else if (absSq(optVelocity) > sqr(radius)) {
			/* Optimize closest point and outside circle. */
			result = normalize(optVelocity) * radius;
		}
		else {
			/* Optimize closest point and inside circle. */
			result = optVelocity;
		}

		for (size_t i = 0; i < lines.size(); ++i) {
			if (det(lines[i].direction, lines[i].point - result) > 0.0f) {
				/* Result does not satisfy constraint i. Compute new optimal result. */
				const Vec2 tempResult = result;

				if (!linearProgram1(lines, i, radius, optVelocity, directionOpt, result)) {
					result = tempResult;
					return i;
				}
			}
		}

		return lines.size();
	}

	/**
	 * \relates    Agent
	 * \brief      Solves a two-dimensional linear program subject to linear
	 *             constraints defined by lines and a circular constraint.
	 * \param      lines         Lines defining the linear constraints.
	 * \param      numObstLines  Count of obstacle lines.
	 * \param      beginLine     The line on which the 2-d linear program failed.
	 * \param      radius        The radius of the circular constraint.
	 * \param      result        A reference to the result of the linear program.
	 */
	void linearProgram3(const std::vector<Line> &lines, size_t numObstLines, size_t beginLine, float radius, Vec2 &result)
	{
		float distance = 0.0f;
    	std::vector<Line> projLines;
        projLines.reserve(numObstLines * 2); 
        projLines.insert(projLines.begin(), lines.begin(), lines.begin() + static_cast<ptrdiff_t>(numObstLines));
        int initProjSize = projLines.size();        

		for (size_t i = beginLine; i < lines.size(); ++i) 
        {
			if (det(lines[i].direction, lines[i].point - result) > distance) 
            {
                projLines.resize(initProjSize);
				/* Result does not satisfy constraint of line i. */
               // projLines.reserve(projLines.size() + (i - numObstLines)); // maximum additional lines that can be added

				for (size_t j = numObstLines; j < i; ++j) 
                {
					Line line;

					float determinant = det(lines[i].direction, lines[j].direction);

					if (std::fabs(determinant) <= RVO_EPSILON) {
						/* Line i and line j are parallel. */
						if (lines[i].direction * lines[j].direction > 0.0f) {
							/* Line i and line j point in the same direction. */
							continue;
						}
						else {
							/* Line i and line j point in opposite direction. */
							line.point = 0.5f * (lines[i].point + lines[j].point);
						}
					}
					else {
						line.point = lines[i].point + (det(lines[j].direction, lines[i].point - lines[j].point) / determinant) * lines[i].direction;
					}

					line.direction = normalize(lines[j].direction - lines[i].direction);
					projLines.push_back(line);
				}

				const Vec2 tempResult = result;

				if (linearProgram2(projLines, radius, Vec2(-lines[i].direction.y, lines[i].direction.x), true, result) < projLines.size()) {
					/* This should in principle not happen.  The result is by definition
					 * already in the feasible region of this linear program. If it fails,
					 * it is due to small floating point error, and the current result is
					 * kept.
					 */
					result = tempResult;
				}

				distance = det(lines[i].direction, lines[i].point - result);
			}
		}
	}
}
