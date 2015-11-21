#include "Agent.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
#include <iostream>

#include "BihTree.h"
#include "mtrig.h"

using namespace std;


void Agent::computeNewVelocity(VODump* dump)
{
#ifdef USE_MY_TRIG
    if (!mtrig::g_wasInited) {
        mtrig::build_table();
        mtrig::g_wasInited = true;
    }
#endif
    m_voStore.clear();
    vector<VelocityObstacle>& velocityObstacles = m_voStore;
	velocityObstacles.reserve(m_neighbors.c.size());

    m_neighbors.sort();

    // create VOs
	for (auto iter = m_neighbors.c.begin(); iter != m_neighbors.c.end(); ++iter)
    {
        VelocityObstacle vo;
		const Object* otherObj = iter->second;
        if (otherObj->m_type == TypeCircle || otherObj->m_type == TypeAgent)
        {
            const Circle* otherCirc = static_cast<const Circle*>(otherObj);
            const Agent* otherAgent = static_cast<const Agent*>(otherCirc); // may not be valid if type is not TypeAgent

		    if (absSq(otherCirc->m_position - m_position) > sqr(otherCirc->m_radius + m_radius)) // not intersecting
		    {
                Vec2 posDiff = otherCirc->m_position - m_position;
			    const float angle = mtrig::atan2(posDiff.y, posDiff.x);
			    const float openingAngle = mtrig::asin((otherCirc->m_radius + m_radius) / length(otherCirc->m_position - m_position));

			    vo.m_side1 = Vec2(mtrig::cos(angle - openingAngle), mtrig::sin(angle - openingAngle));
			    vo.m_side2 = Vec2(mtrig::cos(angle + openingAngle), mtrig::sin(angle + openingAngle));

			    const float d = 2.0f * mtrig::sin(openingAngle) * mtrig::cos(openingAngle);

                if (otherObj->m_type == TypeAgent) 
                {
                    if (det(otherAgent->m_position - m_position, m_prefVelocity - otherAgent->m_prefVelocity) > 0.0f)
                    {
                        const float s = 0.5f * det(m_velocity - otherAgent->m_velocity, vo.m_side2) / d;
                        vo.m_apex = otherAgent->m_velocity + s * vo.m_side1; 
                    }
                    else 
                    {
                        const float s = 0.5f * det(m_velocity - otherAgent->m_velocity, vo.m_side1) / d;
                        vo.m_apex = otherAgent->m_velocity + s * vo.m_side2;
                    }
                }
                else {
                    vo.m_apex = Vec2(0.0f, 0.0f); // non moving neighbor the apes is 0
                }

		    }
		    else 
            {
                if (otherObj->m_type == TypeAgent) {
				    vo.m_apex = 0.5f * (otherAgent->m_velocity + m_velocity); 
                }
                else {
                    vo.m_apex = Vec2(0.0f, 0.0f);
                }
			    vo.m_side1 = normal(m_position, otherCirc->m_position);
			    vo.m_side2 = -vo.m_side1;
			    
		    }
        }
        else  // not a circle
        { 
            vo.m_apex = Vec2(0.0f, 0.0f);

            Vec2 p1, p2;
            if (otherObj->spanningPoints(m_position, m_radius, &p1, &p2)) //m_radius
            {// outside
                vo.m_side1 = normalize(p1 - m_position);
                vo.m_side2 = normalize(p2 - m_position);
                if (det(vo.m_side1, vo.m_side2) < 0)
                    continue; // ignore
            }
            else {
            //    velocityObstacle.m_side1 = normal(m_position, otherAab->m_position);
            //    velocityObstacle.m_side2 = -velocityObstacle.m_side1;

                // alternate segment inside handling
                vo.m_side1 = normalize(p1 - m_position);
                vo.m_side2 = normalize(p2 - m_position);

            }
            vo.p1 = p1;
            vo.p2 = p2;
            vo.fromObstacle = true;
        }
        // search existing vos for unification
        bool foundUni = false;
        if (vo.p1.isValid())
        {
            for(auto& evo: velocityObstacles) {
                if (evo.p1 == vo.p2) {
                    evo.isBig = true;
                    evo.m_sideMid = evo.m_side1;
                    evo.p1 = vo.p1;
                    evo.m_side1 = vo.m_side1;
                    foundUni = true;
                    break;
                }
                if (evo.p2 == vo.p1) {
                    evo.isBig = true;
                    evo.m_sideMid = evo.m_side2; // TBD - set real mid in case of multiple like this
                    evo.p2 = vo.p2;
                    evo.m_side2 = vo.m_side2;
                    foundUni = true;
                    break;
                }
            }
        }
        if (!foundUni)
            velocityObstacles.push_back(vo);
	}

    Candidate candidate;

    Vec2 minCandidateVel;
    float minScore = FLT_MAX;

    Vec2 bestInvalidVel;
    int bestInvalidVOIndex = -1; //(int optimal)
    float bestInvalidMinScore = FLT_MAX;

    auto checkCandidate = [&]() {
        float score = absSq(m_prefVelocity - candidate.m_position);
        if (score >= minScore)
            return;
        
        int invalidatingIndex = -1;
        bool invalidatedByObstacle = false;
        for (int j = 0; j < (int)velocityObstacles.size(); ++j) 
        {
            if (j == candidate.m_velocityObstacle1 || j == candidate.m_velocityObstacle2) 
                continue; // don't check VOs that this candidate is part of

            bool isValid = true;
            auto& evo = velocityObstacles[j];
            Vec2 topos = candidate.m_position - evo.m_apex;
            if (!evo.isBig) {
                float d1 = det(evo.m_side2, topos); 
                float d2 = det(evo.m_side1, topos); 
                if (d1 < 0.0f && d2 > 0.0f)
                    isValid = false;
            }
            else {
                float d1 = det(evo.m_side2, topos); 
                float dmid = det(evo.m_sideMid, topos); 
                float d2 = det(evo.m_side1, topos); 
                if ((d1 < 0.0f && dmid > 0.0f) || (dmid < 0.0f && d2 > 0.0f))
                    isValid = false;
            }

            if (!isValid)
            {
                if (invalidatingIndex == -1) {
                    invalidatingIndex = j;
                }
                if (evo.fromObstacle) {
                    invalidatedByObstacle = true;
                }
            }

        }

        if (invalidatingIndex != -1) // it's invalid
        {
            // its not an obstacle vo, (it can be entered into if there's no alternative)
            // and its in the VO of the furthest neighbor and the best score among those of the furthest neighbor
            if (!invalidatedByObstacle && ((invalidatingIndex > bestInvalidVOIndex || 
                                           (invalidatingIndex == bestInvalidVOIndex && score < bestInvalidMinScore)))) 
            {
                bestInvalidVel = candidate.m_position;
                bestInvalidVOIndex = invalidatingIndex;
                bestInvalidMinScore = score;
            }
        }
        else 
        {
            minCandidateVel = candidate.m_position;
            minScore = score;
        }
    };

    //std::multimap<float, Candidate> candidates;


	candidate.m_velocityObstacle1 = INT_MAX;
	candidate.m_velocityObstacle2 = INT_MAX;

	if (absSq(m_prefVelocity) < m_maxSpeed * m_maxSpeed) {
		candidate.m_position = m_prefVelocity;
	}
	else {
		candidate.m_position = m_maxSpeed * normalize(m_prefVelocity);
	}
    // first candidate, the preferred velocity capped to the max speed
    checkCandidate();
	//candidates.insert(std::make_pair(absSq(m_prefVelocity - candidate.m_position), candidate));

    // project perf velocity on sides of all vos
	for (int i = 0; i < (int)velocityObstacles.size(); ++i) 
    {
		candidate.m_velocityObstacle1 = INT_MAX; //i;
		candidate.m_velocityObstacle2 = i;

		const float dotProduct1 = (m_prefVelocity - velocityObstacles[i].m_apex) * velocityObstacles[i].m_side1;
		const float dotProduct2 = (m_prefVelocity - velocityObstacles[i].m_apex) * velocityObstacles[i].m_side2;

		if (dotProduct1 > 0.0f && det(velocityObstacles[i].m_side1, m_prefVelocity - velocityObstacles[i].m_apex) > 0.0f) 
        {
			candidate.m_position = velocityObstacles[i].m_apex + dotProduct1 * velocityObstacles[i].m_side1;

			if (absSq(candidate.m_position) < m_maxSpeed * m_maxSpeed) {
				//candidates.insert(std::make_pair(absSq(m_prefVelocity - candidate.m_position), candidate));
                checkCandidate();
			}
		}

        candidate.m_velocityObstacle1 = i; //i;
        candidate.m_velocityObstacle2 = INT_MAX;

		if (dotProduct2 > 0.0f && det(velocityObstacles[i].m_side2, m_prefVelocity - velocityObstacles[i].m_apex) < 0.0f) 
        {
			candidate.m_position = velocityObstacles[i].m_apex + dotProduct2 * velocityObstacles[i].m_side2;

			if (absSq(candidate.m_position) < m_maxSpeed * m_maxSpeed) {
				//candidates.insert(std::make_pair(absSq(m_prefVelocity - candidate.m_position), candidate));
                checkCandidate();
			}
		}
	}

	for (int j = 0; j < (int)velocityObstacles.size(); ++j) 
    {
		candidate.m_velocityObstacle1 = INT_MAX;
		candidate.m_velocityObstacle2 = j;

		float discriminant = m_maxSpeed * m_maxSpeed - sqr(det(velocityObstacles[j].m_apex, velocityObstacles[j].m_side1));

		if (discriminant > 0.0f)
        {
            float dsqrt = std::sqrt(discriminant);
			const float t1 = -(velocityObstacles[j].m_apex * velocityObstacles[j].m_side1) + dsqrt;
			const float t2 = -(velocityObstacles[j].m_apex * velocityObstacles[j].m_side1) - dsqrt;

			if (t1 >= 0.0f) {
				candidate.m_position = velocityObstacles[j].m_apex + t1 * velocityObstacles[j].m_side1;
				//candidates.insert(std::make_pair(absSq(m_prefVelocity - candidate.m_position), candidate));
                checkCandidate();
			}

			if (t2 >= 0.0f) {
				candidate.m_position = velocityObstacles[j].m_apex + t2 * velocityObstacles[j].m_side1;
				//candidates.insert(std::make_pair(absSq(m_prefVelocity - candidate.m_position), candidate));
                checkCandidate();
			}
		}

        candidate.m_velocityObstacle1 = j;
        candidate.m_velocityObstacle2 = INT_MAX;

		discriminant = m_maxSpeed * m_maxSpeed - sqr(det(velocityObstacles[j].m_apex, velocityObstacles[j].m_side2));

		if (discriminant > 0.0f)
        {
            float dsqrt = std::sqrt(discriminant);
			const float t1 = -(velocityObstacles[j].m_apex * velocityObstacles[j].m_side2) + dsqrt;
			const float t2 = -(velocityObstacles[j].m_apex * velocityObstacles[j].m_side2) - dsqrt;

			if (t1 >= 0.0f) {
				candidate.m_position = velocityObstacles[j].m_apex + t1 * velocityObstacles[j].m_side2;
				//candidates.insert(std::make_pair(absSq(m_prefVelocity - candidate.m_position), candidate));
                checkCandidate();
			}

			if (t2 >= 0.0f) {
				candidate.m_position = velocityObstacles[j].m_apex + t2 * velocityObstacles[j].m_side2;
				//candidates.insert(std::make_pair(absSq(m_prefVelocity - candidate.m_position), candidate));
                checkCandidate();
			}
		}
	}


    // intersections of two VOs
	for (int i = 0; i < (int)velocityObstacles.size() - 1; ++i) 
    {
		for (int j = i + 1; j < static_cast<int>(velocityObstacles.size()); ++j) 
        {
			candidate.m_velocityObstacle1 = i;
			candidate.m_velocityObstacle2 = j;

			float d = det(velocityObstacles[i].m_side1, velocityObstacles[j].m_side1);

			if (d != 0.0f) {
                Vec2 dapex = velocityObstacles[j].m_apex - velocityObstacles[i].m_apex;
				const float s = det(dapex, velocityObstacles[j].m_side1) / d;
				const float t = det(dapex, velocityObstacles[i].m_side1) / d;

				if (s >= 0.0f && t >= 0.0f) 
                {
					candidate.m_position = velocityObstacles[i].m_apex + s * velocityObstacles[i].m_side1;

					if (absSq(candidate.m_position) < m_maxSpeed * m_maxSpeed) {
						//candidates.insert(std::make_pair(absSq(m_prefVelocity - candidate.m_position), candidate));
                        checkCandidate();
					}
				}
			}

			d = det(velocityObstacles[i].m_side2, velocityObstacles[j].m_side1);

			if (d != 0.0f) {
                Vec2 dapex = velocityObstacles[j].m_apex - velocityObstacles[i].m_apex;
				const float s = det(dapex, velocityObstacles[j].m_side1) / d;
				const float t = det(dapex, velocityObstacles[i].m_side2) / d;

				if (s >= 0.0f && t >= 0.0f) {
					candidate.m_position = velocityObstacles[i].m_apex + s * velocityObstacles[i].m_side2;

					if (absSq(candidate.m_position) < m_maxSpeed * m_maxSpeed) {
						//candidates.insert(std::make_pair(absSq(m_prefVelocity - candidate.m_position), candidate));
                        checkCandidate();
					}
				}
			}

			d = det(velocityObstacles[i].m_side1, velocityObstacles[j].m_side2);

			if (d != 0.0f) {
                Vec2 dapex = velocityObstacles[j].m_apex - velocityObstacles[i].m_apex;
				const float s = det(dapex, velocityObstacles[j].m_side2) / d;
				const float t = det(dapex, velocityObstacles[i].m_side1) / d;

				if (s >= 0.0f && t >= 0.0f) {
					candidate.m_position = velocityObstacles[i].m_apex + s * velocityObstacles[i].m_side1;

					if (absSq(candidate.m_position) < m_maxSpeed * m_maxSpeed) {
						//candidates.insert(std::make_pair(absSq(m_prefVelocity - candidate.m_position), candidate));
                        checkCandidate();
					}
				}
			}

			d = det(velocityObstacles[i].m_side2, velocityObstacles[j].m_side2);

			if (d != 0.0f) {
                Vec2 dapex = velocityObstacles[j].m_apex - velocityObstacles[i].m_apex;
				const float s = det(dapex, velocityObstacles[j].m_side2) / d;
				const float t = det(dapex, velocityObstacles[i].m_side2) / d;

				if (s >= 0.0f && t >= 0.0f) {
					candidate.m_position = velocityObstacles[i].m_apex + s * velocityObstacles[i].m_side2;

					if (absSq(candidate.m_position) < m_maxSpeed * m_maxSpeed) {
						//candidates.insert(std::make_pair(absSq(m_prefVelocity - candidate.m_position), candidate));
                        checkCandidate();
					}
				}
			}
		}
	}


    m_newVelocity = Vec2();
    if (minScore != FLT_MAX) {
        m_newVelocity = minCandidateVel;
    }
    else if (bestInvalidVOIndex != -1) {
        m_newVelocity = bestInvalidVel;
    }

    if (dump != nullptr) {
        dump->vos = velocityObstacles;
//        dump->candidates = candidates;
        dump->selected = m_newVelocity;
    }

}



void Agent::computePreferredVelocity(float deltaTime)
{
   /* if (m_reached && m_endGoalPos.type == GOAL_ATTACK) {
        m_prefVelocity = Vec2(0,0);
        return;
    }*/

    //Vec2 toGoal = m_curGoalPos.p - m_position;
    Vec2 goalPnt = m_curGoalPos->getDest(m_position);
    Vec2 toGoal = goalPnt - m_position;

	float distSqToGoal = absSq(toGoal);
    float distToGoal = std::sqrt(distSqToGoal);

    bool isPointGoal = m_curGoalPos->isPoint(); // assumes this means its the final goal

    if (m_endGoalPos.type == GOAL_ATTACK) {
        float dstRadius = m_endGoalPos.radius*0.9;
        float newDist = distToGoal - dstRadius;
        toGoal = toGoal * (newDist / distToGoal);
        distToGoal = newDist;
    }


    if (isPointGoal && m_prefSpeed * deltaTime > distToGoal) 
    { // close to the goal? the point goal is the final one and we should not overshoot it
        m_prefVelocity = toGoal / deltaTime;
    }
    else 
    { // in segment goals we can overshoot the line, that's intended.
        m_prefVelocity = (m_prefSpeed / distToGoal) * toGoal;
    }

    //    Vec2 toGoalRadius = toGoal * ((distToGoal - m_endGoalPos.radius*0.9) / distToGoal);
        // minus because I want it close to me, not far


}


void Agent::computeNeighbors(BihTree& bihTree)
{
    m_neighbors.clear();

    float adjustingRangeSq = m_neighborDist * m_neighborDist;
    bihTree.query(m_position, m_neighborDist, [&](Object* obj) {
        insertNeighbor(obj, adjustingRangeSq);
    });
}



void Agent::insertNeighbor(Object* otherObj, float &rangeSq)
{
	if (this == otherObj) 
        return;


    // distance to where it touches me (to the point on the surface of the object closest to me), squared
    // there's no way to save this sqrt
    float checkSq = otherObj->distSqToSurface(m_position);


    if (checkSq < rangeSq) 
    {
		//m_neighbors.insert(std::make_pair(checkSq, otherObj));
        m_neighbors.qpush(std::make_pair(checkSq, otherObj));       

		if (m_neighbors.c.size() > m_maxNeighbors) {
			//rangeSq = (--m_neighbors.end())->first;
            m_neighbors.qpop();
            rangeSq = m_neighbors.top().first;
        }
	}
}

namespace qui {
extern int g_curFrame;
}

#define MAX_ANGULAR_SPEED 0.5  // rad/sec
#define I_PI (3.1415926535897932384626433832795)


bool Agent::update(float deltaTime)
{
	const float dv = length(m_newVelocity - m_velocity);

	if (dv < m_maxAccel * deltaTime) {
		m_velocity = m_newVelocity;
	}
	else {
		m_velocity = (1.0f - (m_maxAccel * deltaTime / dv)) * m_velocity + (m_maxAccel * deltaTime / dv) * m_newVelocity;
	}

	m_position += m_velocity * deltaTime;

    //bool reachedGoal = ( (absSq(m_curGoalPos.p - m_position) < m_goalRadius * m_goalRadius) );
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

    float prevo = m_orientation;
	float nexto = mtrig::atan2(m_prefVelocity.y, m_prefVelocity.x);
    float d = prevo - nexto;
    float maxd = MAX_ANGULAR_SPEED * deltaTime;
    float absd = iabs(d);
    if (absd > maxd) {
        //OUT("orientDelta=" << d << " o=" << m_orientation)
        if ((d > 0) != (absd > I_PI))
            m_orientation = prevo - maxd;
        else 
            m_orientation = prevo + maxd;

        // the range of atan2
        if (m_orientation < -I_PI)
            m_orientation += 2 * I_PI;
        if (m_orientation > I_PI)
            m_orientation -= 2 * I_PI;
    }
    else {
        m_orientation = nexto;
    }
    return reachedEnd;
}

