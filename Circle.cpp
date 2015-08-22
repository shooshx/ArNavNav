#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>

using namespace std;

#include "Simulator.h"
#include "Agent.h"



const float HRVO_TWO_PI = 6.283185307179586f;

#define COUNT 1
#define RADIUS 6.0
#define ANG_OFFST 0


int xxmain()
{
	Simulator simulator;

    //ofstream outf("/Users/shyshalom/Projects/collision/hrvo_test/output.txt");
    ofstream outf("C:\\projects\\nav\\output.txt");
    if (!outf.good()) {
        cout << "Could not open file";
        return 0;
    }

    outf << "data = {\nradiuses: [";

    stringstream colors;
    //Vec2 obs[] = { Vec2(0, 0) };
    //Vec2 obs[] = { Vec2(40, 0), Vec2(-40,0), Vec2(-79,0), Vec2(79,0)};//, Vec2(-120,0), Vec2(120,0) };
    Vec2 obs[] = { Vec2(0, 19), Vec2(0,-19) };

    int agentIndex = 0;
    for(auto i = 0; i < _countof(obs); ++i)
    {
        simulator.addObject(new Circle(obs[i], 20, agentIndex++));
        //simulator.addObject(new AABB(obs[i], Vec2(40+12,50+12), agentIndex++));
        //outf << "[40,50],";
        outf << "20, ";
        colors << "\"#222222\", ";
    }


    float d = 1.0/COUNT;

	for (int i = 0; i < COUNT; ++i) 
    {
		Vec2 pos = 100.0f * Vec2(std::cos(d * i * HRVO_TWO_PI + ANG_OFFST), std::sin(d * i * HRVO_TWO_PI + ANG_OFFST));
        Vec2 goal = -pos;


        //Vec2 pos((i%20)*15 - 150, 70+(i/20)*15);
        //Vec2 goal = pos - Vec2(0, 140);

        simulator.addObject(new Agent(agentIndex++, pos,
                goal, // goal 
                15.0, //15.0, // nei dist
                10, // max nei
                RADIUS, // radius
                1.5f, // goal radius
                1.0f, // pref speed
                2.0f)); // max speed

        outf << RADIUS << ", ";

        string c = "#ff0000";
        if (i == 0)
            c = "#0000ff";
        else if (i == 10)
            c = "#00eeee";
        else if (i == 30)
            c = "#CC33FF";
        colors << "\"" << c << "\", ";
	}
    outf << "],\ncolors: [" << colors.str() << "],\nframes: [";

    int frameCount = 0;
	do {
		//outf << simulator.getGlobalTime();
        outf << "[";
		for (int i = 0; i < simulator.getNumAgents(); ++i) {
            auto p = simulator.getObject(i)->m_position;
			outf << p.x << "," << p.y << ", ";
		}
		outf << "], //" << frameCount << endl;
        if (frameCount == 259) {
            cout << "here";
        }

        simulator.doStep(0.25f);

        ++frameCount;
        if ((frameCount % 50) == 0) {
            cout << frameCount << endl;
        }
        if (frameCount > 5000)
            break;
    }
	while (!simulator.haveReachedGoals());
    cout << frameCount << endl;
    outf << "]}";
    outf.close();

	return 0;
}
