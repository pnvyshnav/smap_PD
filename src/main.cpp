/*
 * OctoMap - An Efficient Probabilistic 3D Mapping Framework Based on Octrees
 * http://octomap.github.com/
 *
 * Copyright (c) 2009-2013, K.M. Wurm and A. Hornung, University of Freiburg
 * All rights reserved.
 * License: New BSD
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include "../include/BeliefVoxel.h"
#include "../include/BeliefMap.h"
#include "../include/TrueMap.h"
#include "../include/Robot.hpp"

#include "../include/TruncatedGaussianDistribution.hpp"

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
    trueMap.writeBinary("simple_tree.bt");

    Robot<> robot(Parameters::Vec3Type(0, 0, 0), Parameters::Vec3Type(1, 0, 0));
    Observation o = robot.observe(trueMap);

    return 0;
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