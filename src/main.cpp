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


void print_query_info(point3d query, OcTreeNode *node)
{
    if (node != NULL)
    {
        cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << endl;
    }
    else
        cout << "occupancy probability at " << query << ":\t is unknown" << endl;
}

int main(int argc, char **argv)
{
    cout << endl;
    cout << "generating example map" << endl;
    BeliefMap beliefMap;
    cout << beliefMap.calcNumNodes() << std::endl;

    TruncatedGaussianDistribution g(3, 5, -100, 100);
    cout << "g = TruncatedGaussianDistribution(3, 5, -100, 100)" << endl;
    cout << "g.cdfValue(4) .. 0.5" << endl;
    cout << g.cdfValue(4) << endl;
    cout << "g.pdfValue(4) .. 0.079788456080286549" << endl;
    cout << g.pdfValue(4) << endl;
    for (int i = 0; i < 15; ++i)
    {
        cout << "g.sample():  " << g.sample() << endl;
    }

    TrueMap trueMap = TrueMap::generate();
    //trueMap.writeBinary("simple_tree.bt");

    //return 0;

    Robot<> robot(Parameters::Vec3Type(Parameters::voxelSize/2.f, Parameters::voxelSize/2.f, 0), Parameters::Vec3Type(1, 0, 0));
    //Observation o = robot.observe(trueMap);

    //cout << endl << "Update succeeds? " << std::boolalpha << beliefMap.updateVisualization(o, trueMap) << endl;

    Visualizer *visualizer = new Visualizer(argc, argv);
    trueMap.subscribe(std::bind(&Visualizer::publishTrueMap, visualizer, std::placeholders::_1));
    beliefMap.subscribe(std::bind(&Visualizer::publishBeliefMap, visualizer, std::placeholders::_1));

    robot.sensor().subscribe(std::bind(&Visualizer::publishSensor, visualizer, std::placeholders::_1));

    trueMap.publish();
    //beliefMap.publish();
    robot.sensor().publish();

    for (auto rad = 0.; rad < 2 * M_PI; rad += 15. * M_PI / 180.)
    {
        trueMap.publish();
        robot.setOrientation(Parameters::Vec3Type(std::cos(rad), std::sin(rad), 0));
        ROS_INFO("Rotating sensor by %f degrees.", rad * 180.0/M_PI);
        Observation o = robot.observe(trueMap);
        beliefMap.update(o, trueMap);
        //visualizer->publishRay(trueMap, robot.sensor());
        //beliefMap.publish(); // force
    }
    delete beliefMap.icm;
    beliefMap.icm = NULL;
    beliefMap.publish();

    visualizer->render();

    return EXIT_SUCCESS;
}


int main2(int argc, char **argv)
{

    cout << endl;
    cout << "generating example map" << endl;

    OcTree tree(0.1);  // create empty tree with resolution 0.1


    // insert some _measurements of occupied cells

    for (int x = -20; x < 20; x++)
    {
        for (int y = -20; y < 20; y++)
        {
            for (int z = -20; z < 20; z++)
            {
                point3d endpoint((float) x * 0.05f, (float) y * 0.05f, (float) z * 0.05f);
                tree.updateNode(endpoint, true); // integrate 'occupied' measurement
            }
        }
    }

    // insert some _measurements of free cells

    for (int x = -30; x < 30; x++)
    {
        for (int y = -30; y < 30; y++)
        {
            for (int z = -30; z < 30; z++)
            {
                point3d endpoint((float) x * 0.02f - 1.0f, (float) y * 0.02f - 1.0f, (float) z * 0.02f - 1.0f);
                tree.updateNode(endpoint, false);  // integrate 'free' measurement
            }
        }
    }

    cout << endl;
    cout << "performing some queries:" << endl;

    point3d query(0., 0., 0.);
    OcTreeNode *result = tree.search(query);
    print_query_info(query, result);

    query = point3d(-1., -1., -1.);
    result = tree.search(query);
    print_query_info(query, result);

    query = point3d(1., 1., 1.);
    result = tree.search(query);
    print_query_info(query, result);


    cout << endl;
    tree.writeBinary("simple_tree.bt");
    cout << "wrote example file simple_tree.bt" << endl << endl;
    cout << "now you can use octovis to visualize: octovis simple_tree.bt" << endl;
    cout << "Hint: hit 'F'-key in viewer to see the freespace" << endl << endl;

}
