#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include "../include/BeliefVoxel.h"
#include "../include/BeliefMap.h"
#include "../include/TrueMap.h"
#include "../include/Robot.hpp"

#include "../include/TruncatedGaussianDistribution.hpp"
#include "../include/Visualizer.h"

using namespace std;
using namespace octomap;

int main(int argc, char **argv)
{
    BeliefMap beliefMap;
    TrueMap trueMap = TrueMap::generate();
    //trueMap.writeBinary("simple_tree.bt");

    Robot<> robot(Parameters::Vec3Type(Parameters::voxelSize/2.f, Parameters::voxelSize/2.f, 0), Parameters::Vec3Type(1, 0, 0));

    Visualizer *visualizer = new Visualizer(argc, argv);
    //trueMap.subscribe(std::bind(&Visualizer::publishTrueMap, visualizer, std::placeholders::_1));
    trueMap.subscribe(std::bind(&Visualizer::publishTrueMap2dSlice, visualizer, std::placeholders::_1, 0));
    beliefMap.subscribe(std::bind(&Visualizer::publishBeliefMap, visualizer, std::placeholders::_1));
    robot.sensor().subscribe(std::bind(&Visualizer::publishSensor, visualizer, std::placeholders::_1));

    for (auto rad = 0.; /*rad <= 6 * M_PI */; rad += 8. * M_PI / 180.)
    {
        trueMap.publish();
        robot.setOrientation(Parameters::Vec3Type(std::cos(rad), std::sin(rad), 0));
        ROS_INFO("Rotating sensor by %g degrees.", rad * 180.0/M_PI);
        Observation o = robot.observe(trueMap);
        beliefMap.update(o, trueMap);
        //visualizer->publishRay(trueMap, robot.sensor());
    }
    beliefMap.publish();

    visualizer->render();

    return EXIT_SUCCESS;
}