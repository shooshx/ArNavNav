#include "Agent.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
#include <iostream>

#include "BihTree.h"

using namespace std;



namespace mtrig 
{

// from http://http.developer.nvidia.com/Cg/atan2.html
float atan2(float y, float x)
{
    float t0, t1, t2, t3, t4;

    t3 = iabs(x);
    t1 = iabs(y);
    t0 = imax(t3, t1);
    t1 = imin(t3, t1);
    t3 = float(1) / t0;
    t3 = t1 * t3;

    t4 = t3 * t3;
    t0 =         - float(0.013480470);
    t0 = t0 * t4 + float(0.057477314);
    t0 = t0 * t4 - float(0.121239071);
    t0 = t0 * t4 + float(0.195635925);
    t0 = t0 * t4 - float(0.332994597);
    t0 = t0 * t4 + float(0.999995630);
    t3 = t0 * t3;

    t3 = (abs(y) > abs(x)) ? float(1.570796327) - t3 : t3;
    t3 = (x < 0) ?  float(3.141592654) - t3 : t3;
    t3 = (y < 0) ? -t3 : t3;

    return t3;
}

float asin(float x) {
    float negate = float(x < 0);
    x = iabs(x);
    float ret = -0.0187293;
    ret *= x;
    ret += 0.0742610;
    ret *= x;
    ret -= 0.2121144;
    ret *= x;
    ret += 1.5707288;
    ret = 3.14159265358979*0.5 - sqrt(1.0 - x)*ret;
    return ret - 2 * negate * ret;
}


// from http://www.flipcode.com/archives/Fast_Trigonometry_Functions_Using_Lookup_Tables.shtml
#define MAX_CIRCLE_ANGLE      512
#define HALF_MAX_CIRCLE_ANGLE (MAX_CIRCLE_ANGLE/2)
#define QUARTER_MAX_CIRCLE_ANGLE (MAX_CIRCLE_ANGLE/4)
#define MASK_MAX_CIRCLE_ANGLE (MAX_CIRCLE_ANGLE - 1)
#define PI 3.14159265358979323846f

static bool g_wasInited = false;
static float fast_cossin_table[MAX_CIRCLE_ANGLE];

void build_table() {
    // Build cossin table
    for (int i = 0 ; i < MAX_CIRCLE_ANGLE ; i++)
    {
        fast_cossin_table[i] = (float)sin((double)i * PI / HALF_MAX_CIRCLE_ANGLE);
    }
}

inline float cos(float n)
{
    float f = n * HALF_MAX_CIRCLE_ANGLE / PI;
    int i = (int)f;
    if (i < 0)
    {
        return fast_cossin_table[((-i) + QUARTER_MAX_CIRCLE_ANGLE) & MASK_MAX_CIRCLE_ANGLE];
    }
    else
    {
        return fast_cossin_table[(i + QUARTER_MAX_CIRCLE_ANGLE) & MASK_MAX_CIRCLE_ANGLE];
    }
}

inline float sin(float n)
{
    float f = n * HALF_MAX_CIRCLE_ANGLE / PI;
    int i = (int)f;
    if (i < 0)
    {
        return fast_cossin_table[(-((-i) & MASK_MAX_CIRCLE_ANGLE)) + MAX_CIRCLE_ANGLE];
    }
    else
    {
        return fast_cossin_table[i & MASK_MAX_CIRCLE_ANGLE];
    }
}

}

void Agent::computeNewVelocity(VODump* dump)
{
    if (!mtrig::g_wasInited) {
        mtrig::build_table();
        mtrig::g_wasInited = true;
    }
    m_voStore.clear();
    vector<VelocityObstacle>& velocityObstacles = m_voStore;
	velocityObstacles.reserve(m_neighbors.c.size());

    m_neighbors.sort();

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

                if (otherObj->m_type == TypeAgent && otherAgent->m_isMobile) 
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
                if (otherObj->m_type == TypeAgent && otherAgent->m_isMobile) {
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
        }
        // search existing vos for unification
        bool foundUni = false;
        if (vo.p1.isValid())
        {
            for(auto& evo: velocityObstacles) {
                if (evo.p1 == vo.p2) {
                    evo.p1 = vo.p1;
                    evo.m_side1 = vo.m_side1;
                    foundUni = true;
                    break;
                }
                if (evo.p2 == vo.p1) {
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

    Candidate minCandidate;
    float minScore = FLT_MAX;

    auto checkCandidate = [&]() {
        float score = absSq(m_prefVelocity - candidate.m_position);
        if (score >= minScore)
            return;

        for (int j = 0; j < (int)velocityObstacles.size(); ++j) 
        {

            if (j != candidate.m_velocityObstacle1 && j != candidate.m_velocityObstacle2) 
            { 
                float d1 = det(velocityObstacles[j].m_side2, candidate.m_position - velocityObstacles[j].m_apex); 
                float d2 = det(velocityObstacles[j].m_side1, candidate.m_position - velocityObstacles[j].m_apex); 
                if (d1 < 0.0f && d2 > 0.0f)
                {
                    return;
                }
            }
            // avoid points that are in the middle between two VOs
          /*  if (candidate.m_velocityObstacle1 == INT_MAX && j != candidate.m_velocityObstacle2) {
                float d2 = det(velocityObstacles[j].m_side2, candidate.m_position - velocityObstacles[j].m_apex); 
                if (iabs(d2) < 0.0001)
                    return;
            }

            if (candidate.m_velocityObstacle2 == INT_MAX && j != candidate.m_velocityObstacle1) {
                float d1 = det(velocityObstacles[j].m_side1, candidate.m_position - velocityObstacles[j].m_apex); 
                if (iabs(d1) < 0.0001)
                    return;
            }*/


        }
        minCandidate = candidate;
        minScore = score;
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

#if 1
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
#endif
#if 1
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
#endif
#if 1

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
#endif

	//int optimal = -1;

    m_newVelocity = Vec2();
    if (minScore != FLT_MAX)
        m_newVelocity = minCandidate.m_position;
/*	for (auto iter = candidates.begin(); iter != candidates.end(); ++iter) 
    {
		candidate = iter->second;

		bool valid = true;

		for (int j = 0; j < (int)velocityObstacles.size(); ++j) 
        {
			if (j != candidate.m_velocityObstacle1 && j != candidate.m_velocityObstacle2 && 
                det(velocityObstacles[j].m_side2, candidate.m_position - velocityObstacles[j].m_apex) < 0.0f && 
                det(velocityObstacles[j].m_side1, candidate.m_position - velocityObstacles[j].m_apex) > 0.0f) 
            {
				valid = false;

				if (j > optimal) {
					optimal = j;
                    // I don't know WTF is this shit but it's not good.
    		        //m_newVelocity = candidate.m_position;
				}

				break;
			}
		}

		if (valid) {
    	    m_newVelocity = candidate.m_position;
			break;
		}
	}*/

    if (dump != nullptr) {
        dump->vos = velocityObstacles;
//        dump->candidates = candidates;
        dump->selected = m_newVelocity;
    }

}



void Agent::computePreferredVelocity(float deltaTime)
{
    if (m_reached) {
        m_prefVelocity = Vec2(0,0);
        return;
    }

    //Vec2 toGoal = m_curGoalPos.p - m_position;
    Vec2 goalPnt = m_curGoalPos->getDest(m_position);
    Vec2 toGoal = goalPnt - m_position;

	const float distSqToGoal = absSq(toGoal);

	if (m_curGoalPos->shouldTaper() && sqr(m_prefSpeed * deltaTime) > distSqToGoal) 
    { // close to the goal? the point goal is the final one and we should not overshoot it
		m_prefVelocity = (toGoal) / deltaTime;
	}
	else 
    { // in segment goals we can overshoot the line, that's intended.
		m_prefVelocity = (m_prefSpeed / std::sqrt(distSqToGoal)) * toGoal;
	}
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
    else {
        // did we backtrack?
        // this is a bad idea since it can cause loops see  back_and_forth_stuck.txt
   /*     if (m_indexInPlan > 0 && !m_plan.m_d[m_indexInPlan - 1]->isPassed(m_position)  ) {
            --m_indexInPlan;
           // cout << "BACK! " << index << endl;
            m_curGoalPos = m_plan.m_d[m_indexInPlan];
        }
        */

    }

	/*if (!reachedEnd) {
		m_orientation = atan(m_prefVelocity);
	}*/
    return reachedEnd;
}

