#include <GL/freeglut.h>
#include <string.h>
#include <math.h>

#include "../include/Parameters.hpp"
#include "../include/Visualizer.h"

extern "C"
{
    TrueMap trueMap = TrueMap::generateRandomCorridor(); // use a fixed seed value

    MapType map;
    LogOddsMap logOddsMap;
    FakeRobot<> robot(
            Parameters::Vec3Type(Parameters::xCenter,
                                 Parameters::yCenter,
                                 Parameters::zCenter),
            Parameters::Vec3Type(0, 1, 0),
            trueMap,
            map);

    Visualizer *visualizer;

    std::vector<Parameters::Vec3Type> lastPositions;

    unsigned int episode = 0;
    double lastAvgError = 0.5 * Parameters::voxelsTotal;

    void handleObservation(const Observation &observation)
    {
        map.update(observation, trueMap);
        Visualizer::updateMapView();
    }

    std::vector<float> observation;

    /**
     * Initializes environment.
     */
    void initialize(int skipFrame = 1)
    {
        char *myargv[1];
        int myargc = 1;
        myargv[0] = strdup("SMAP Gym Environment");
        glutInit(&myargc, myargv);

        episode = 0;

        visualizer = new Visualizer(&trueMap, &map, &robot, true, skipFrame, true);

        robot.setPosition(Parameters::Vec3Type(0, 0, 0));
        robot.setYaw(M_PI / 2.);
        robot.registerObserver(&handleObservation);
        robot.run();
    }

    /**
     * Resets the environment.
     */
    void reset()
    {
        visualizer->update();

        episode += 1;
        lastPositions.clear();
        visualizer->setEpisode(episode);

//        std::cout << "Resetting environment..." << std::endl;
        map.reset();

        float difficulty = std::max(1.f, 500.f / episode);
        int easiestRadius = 10;
        int easiestBranches = 50;
        int hardestRadius = 2;
        int hardestBranches = 20;

        // compute corridor map parameterized by difficulty
        int radius = (int) ((1.f - difficulty) * easiestRadius + difficulty * hardestRadius);
        int branches = (int) ((1.f - difficulty) * easiestBranches + difficulty * hardestBranches);

        //trueMap = TrueMap::generateRandomCorridor();
//        std::cout << "Shuffling environment..." << std::endl;
        trueMap.shuffleCorridor(radius, branches, difficulty);

        robot.setPosition(Parameters::Vec3Type(0, 0, 0));
        robot.setYaw(M_PI / 2.);
//        std::cout << "Running robot..." << std::endl;
        robot.run();
//        std::cout << "Finished reset" << std::endl;
    }

    /**
     * Returns reachability from the omni-directional perspective at
     * the current position, centered around the current orientation.
     * @param rays Total number of rays to shoot.
     * @return Reachability of each ray.
     */
    float* observeLocal(int rays)
    {
        observation = robot.getOmniDirectionalReachability((unsigned int) rays);
        return observation.data();

//        for (int i = 0; i < rays; ++i)
//            std::cout << observation[i] << " ";
//        std::cout << std::endl;
    }

    /**
     * Returns complete map view.
     * @return Voxel occupancies of whole map.
     */
    float* observeGlobal()
    {
        return visualizer->mapView();
    }

    /**
     * Returns the width of the map view.
     * @return Width in pixels.
     */
    int mapWidth()
    {
        return visualizer->mapWidth();
    }

    /**
     * Returns the height of the map view.
     * @return Height in pixels.
     */
    int mapHeight()
    {
        return visualizer->mapHeight();
    }

    /**
     * Estimate the reachability at the current position of the robot.
     * @return Interpolated reachability at robot position.
     */
    float reachability()
    {
        return (float) map.filteredReachability(robot.position().x(), robot.position().y(), robot.position().z());
    }

    /**
     * Determines whether robot is inside the map's boundaries.
     * @return True if robot is on the map.
     */
    bool inside()
    {
        return !(robot.position().x() < Parameters::xMin || robot.position().x() > Parameters::xMax ||
        robot.position().y() < Parameters::yMin || robot.position().y() > Parameters::yMax ||
        robot.position().z() < Parameters::zMin || robot.position().z() > Parameters::zMax);
    }

    /**
     * Makes the robot perform a translational and rotational movement
     * and returns the reward of such action.
     * @param velocity Translational velocity (positive is forward).
     * @param angularVelocity Rotational velocity (positive is right turn).
     * @return Reward.
     */
    float act(float velocity, float angularVelocity)
    {
        visualizer->setVelocity(velocity);
        visualizer->setAngularVelocity(angularVelocity);
        auto p = robot.position();
        robot.setYaw(robot.yaw() - angularVelocity);
        robot.setPosition(p + robot.orientation() * velocity);
        visualizer->registerPosition(robot.position());
        robot.run();

        auto stats = map.stats(trueMap);
        auto errors = VoxelStatistics::selectError(stats);
        double avg_error = 0;
        for (double e : errors)
            avg_error += 1.-e;
        //std::cout << avg_error << std::endl;
        //avg_error /= errors.size();

        double diff = std::abs(avg_error - lastAvgError);
        lastAvgError = avg_error;

        auto v = trueMap.query(robot.position());
        if (!inside() || v.type != GEOMETRY_VOXEL || trueMap.getVoxelMean(v) > 0.5)
            return -1000;

        // multiply by the distance to closest last position
        // to reinforce exploration of new areas
        float closestDistance = 1;
        for (auto &pos : lastPositions)
        {
            float dist = pos.distance(robot.position());
            if (dist < closestDistance)
                closestDistance = dist;
        }
        lastPositions.push_back(robot.position());


        //if (reachability() < 0.5)
         //   return (float) (-std::abs(diff) * reachability());
        return (float) diff * 10.f * closestDistance;
    }

    void destroy()
    {
        delete visualizer;
//        std::exit(1);
    }
}

