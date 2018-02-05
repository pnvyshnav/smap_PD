#include <octomap/octomap.h>

#include <ros/ros.h>

#include <ecl/time/stopwatch.hpp>

#undef MANY_STEPS

#include "../include/BeliefVoxel.h"
#include "../include/BeliefMap.h"
#include "../include/LogOddsMap.h"
#include "../include/GaussianProcessMap.h"
#include "../include/FakeRobot.hpp"
#include "../include/Visualizer.h"
#include "../include/Drone.h"
#include "../include/Statistics.hpp"
#include "../include/PointCloud.h"
#include "../include/Trajectory.hpp"
#include "../include/ICLNUIMLoader.hpp"





//
//Observation allObservations;
//
//int updated = 0;
//void handleObservation(const Observation &observation)
//{
//    allObservations.append(observation);
//#ifdef GP_RUNS
//    return;
//#endif
//#ifndef SLIM_STATS
//    stats->registerMeasurements((int)observation.measurements().size());
//    std::valarray<Parameters::NumType> rayLengths(observation.measurements().size());
//    unsigned int i = 0;
//    for (auto &measurement : observation.measurements())
//    {
//        rayLengths[i++] = measurement.value;
//    }
//    stats->registerRayStatistics(rayLengths.min(), rayLengths.max(), rayLengths.sum() / i);
//
//    stopWatch.restart();
//    beliefMap.update(observation, trueMap);
//    stats->registerStepTimeBelief(stopWatch.elapsed());
//    stopWatch.restart();
//    logOddsMap.update(observation, trueMap);
//    stats->registerStepTimeLogOdds(stopWatch.elapsed());
//
////    stopWatch.restart();
////    gaussianProcessMap.update(observation);
////    stats->registerStepTimeGP(stopWatch.elapsed());
//
//    stats->update(logOddsMap, beliefMap, gaussianProcessMap, robot);
//#else
//    beliefMap.update(observation, trueMap);
//    logOddsMap.update(observation, trueMap);
//    gaussianProcessMap.update(observation);
//#endif
////
////#ifdef REAL_3D
////    if (updated > 0 && updated % 25 == 0)
////    {
////#ifdef SLIM_STATS
////        stats->update(logOddsMap, beliefMap, robot);
////#endif
////
////        // save stats continually
////        stats->saveToFile(homedir + "/catkin_ws/src/smap/stats/stats_real3d_"
////                          + std::to_string(updated) + ".bag");
////#ifdef SLIM_STATS
////        stats->reset();
////#endif
////    }
////#endif
////    ++updated;
////
////#if defined(FAKE_2D) || defined(FAKE_3D)
////    trueMap.publish();
////    robot.publish();
////    gaussianProcessMap.publish();
////    if (!ros::ok())
////        robot.stop();
////#endif
////}

int main(int argc, char **argv)
{
//    PointCloud cloud;
//    cloud.loadPly("~/catkin_ws/src/smap/dataset/V1_01_easy/groundtruth_pcl.ply");
//    cloud.visualize();

    ros::init(argc, argv, "SMAP");
    ros::Time::init();

    // intrinsic matrix
    Eigen::Matrix3d K(3, 3);
    K << 481.2,	   0.0, 319.5,
           0.0, -480.0, 239.5,
           0.0,	   0.0,   1.0;

    double fx = K(0, 0);
    double fy = K(1, 1);
    double u0 = K(0, 2);
    double v0 = K(1, 2);

    ICLNUIMLoader loader;
//    std::string trajFilename = homedir + "/catkin_ws/src/smap/dataset/iclnuim_livingroom1/traj0/livingRoom0n.gt.sim";
//    traj.loadFromFile(trajFilename);

    int trajectory = 3;

    ROS_INFO("Loading ...");
    bool result = loader.load("/media/eric/data/Mapping-Datasets/smap_datasets/iclnuim_livingroom1", trajectory, 1800, 3, 10); //1500); //1510);
    ROS_INFO("Loaded traj0 successfully? %i", (int)result);
    assert(result);

    auto *visualizer = new Visualizer;
//    cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);

    // TODO these parameters are registered too late (helper function still refer to old coords)
    Parameters::xMin = -2.7;
    Parameters::yMin = -0.1;
    Parameters::zMin = -4.8;
    Parameters::xMax = 2.8;
    Parameters::yMax = 2.9;
    Parameters::zMax = 4.2;
    Parameters::voxelSize = 0.0625; //0.125;

    std::string plyFilename = "/media/eric/data/Mapping-Datasets/smap_datasets/iclnuim_livingroom1/livingroom.ply";
    TrueMap trueMap = TrueMap::generateFromPointCloud(plyFilename);
    BeliefMap beliefMap;
    LogOddsMap logOddsMap;
    GaussianProcessMap gaussianProcessMap;
    FakeRobot<> robot(
            Parameters::center(),
            Parameters::Vec3Type(0, 1, 0),
            trueMap,
            beliefMap);

    Statistics<> *stats = new Statistics<>(trueMap);

    auto allObservations = loader.allObservations();
//    allObservations.translate(-0.06, 0, 0);
//    allObservations.translate(0.015, 0, 0); // traj0
//    allObservations.scale(1.03, 1.03, 1.03); // traj0
    for (int i = 0; i < 1; ++i)
    {
        visualizer->publishTrueMap(&trueMap);
        visualizer->publishObservation(&allObservations, false, true, 20);
        visualizer->sleep(200);
    }

    ecl::StopWatch stopWatch;
    int cnt = 0;
    for (auto observation : loader.frames)
    {
        ROS_INFO("Updating frame %i out of %i (%i measurements)...", cnt,
                 (int)loader.frames.size(), (int)observation.measurements().size());
//            observation.translate(0.025, 0, 0); // traj1
//        observation.translate(0.015, 0, 0); // traj0
//        observation.scale(1.03, 1.03, 1.03); // traj0
        //TODO potentially dangerous if this measurement is not a voxel?
        robot.setPosition(observation.measurements().front().sensor.position);
//        robot.setOrientation(observation.measurements()[observation.measurements().size()/2].sensor.orientation);

        stats->registerMeasurements((int)observation.measurements().size());
        std::valarray<Parameters::NumType> rayLengths(observation.measurements().size());
        unsigned int i = 0;
        for (auto &measurement : observation.measurements())
        {
            rayLengths[i++] = measurement.value;
        }
        stats->registerRayStatistics(rayLengths.min(), rayLengths.max(), rayLengths.sum() / i);

        stopWatch.restart();
        beliefMap.update(observation, trueMap);
        stats->registerStepTimeBelief(stopWatch.elapsed());
        stopWatch.restart();
        logOddsMap.update(observation, trueMap);
        stats->registerStepTimeLogOdds(stopWatch.elapsed());

//    stopWatch.restart();
//    gaussianProcessMap.update(observation);
//    stats->registerStepTimeGP(stopWatch.elapsed());

        stats->update(logOddsMap, beliefMap, gaussianProcessMap, robot);
        ++cnt;
    }

    std::string homedir = getenv("HOME");
    stats->saveToFile(homedir + "/catkin_ws/src/smap/stats/iclnuim/stats_traj" + std::to_string(trajectory) + ".bag");
    stats->reset();

//    for (int step = 0; step < traj.poses.size(); ++ step)
//    {
//        cv::Mat image;
//        std::string filename = homedir + "/catkin_ws/src/smap/dataset/iclnuim_livingroom1/traj0/depth/" + std::to_string(step) + ".png";
//        image = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
//
//        if (!image.data)                              // Check for invalid input
//        {
//            std::cout << "Could not open or find the image" << std::endl;
//            return -1;
//        }
//
//        allObservations.clear();
//        srand(1234);
//        for (int i = 0; i < 20; ++i)
//        {
//            // pixels to query
//            int v = i % 5 * 640/5; //rand() % 640;
//            int u = i / 5 * 480/5; //rand() % 480;
//
//            unsigned char c = image.at<unsigned char>(u, v);
//            std::cout << "Pixel (" << u << "," << v << "): " << (int) c << std::endl;
//            image.at<unsigned char>(u, v) = 255;
//
//            endianSwap(&c);
//            auto z = (double) c;
//            if ((int) c == 255)
//            {
//                std::cerr << "Skipping depth of 255 (infinity)." << std::endl;
//                continue;
//            }
//            double depth = z / 255.;
//            z /= 255.;
//
//            double u_u0_by_fx = (u - u0) / fx;
//            double v_v0_by_fy = (v - v0) / fy;
//
//            z = z / std::sqrt(std::pow(u_u0_by_fx, 2.) + std::pow(v_v0_by_fy, 2.) + 1);
//
//            double x = ((u - u0) / fx) * z;
//            double y = ((v - v0) / fy) * z;
//
//            std::cout << "z: " << z << std::endl;
//
//            Eigen::Vector3d cameraPosition = traj.poses[step].block(0, 3, 3, 1);
//            //TODO what about camera rotation?
//            Eigen::Matrix3d cameraRotation = traj.poses[step].block(0, 0, 3, 3);
//            Eigen::Vector3d orientation(3);
//            orientation << x, y, z;
//            orientation = cameraRotation * orientation;
//
//            SensorRay ray(Parameters::Vec3Type(cameraPosition[0], cameraPosition[1], cameraPosition[2]),
//                          Parameters::Vec3Type(orientation[0], orientation[1], orientation[2]),
//                          depth);
//            Measurement m = Measurement::voxel(ray, depth);
//            allObservations.append(m);
//
//            beliefMap.update(m, trueMap);
//
//            imshow("Display window", image);
//            cv::waitKey(1);
//        }
//
//        for (int i = 0; i < 10; ++i)
//            visualizer->publishObservation(&allObservations);
//
//        visualizer->publishBeliefMap(&beliefMap);
//
//        visualizer->sleep(100);
//    }

//    cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);// Create a window for display.
//    imshow("Display window", image);                   // Show our image inside it.

//    visualizer->render();

//    cv::waitKey(0);                                          // Wait for a keystroke in the window
    return EXIT_SUCCESS;

//    std::vector<unsigned char> image;
//    unsigned width;
//    unsigned height;
//    std::cout << "PNG decoding status: " << lodepng::decode(image, width, height,
//                    homedir + "/catkin_ws/src/smap/dataset/iclnuim_livingroom1/depth/00000.png",
//                    LodePNGColorType::LCT_GREY, 16) << std::endl;
//    std::cout << "width: " << width << "  height: " << height << "  pixels: " << image.size() << std::endl;
//    std::vector<unsigned int> imageDepth;
//    for (unsigned int i = 0; i < image.size()-1; i += 2)
//    {
//        imageDepth.push_back(image[i] << 4 | image[i+1]);
//        std::cout << imageDepth.back() << std::endl;
//    }
//    return 0;

//#ifdef PLANNER_2D_TEST
//    TrajectoryPlanner planner(trueMap, beliefMap, logOddsMap);
//#endif
//
//#ifdef ENABLE_VISUALIZATION
//    Visualizer *visualizer = new Visualizer;
//    trueMap.subscribe(std::bind(&Visualizer::publishTrueMap2dSlice, visualizer, std::placeholders::_1, 0));
//    for (int i = 0; i < 20; ++i)
//        trueMap.publish();
//#ifndef REAL_3D
//    robot.subscribe(std::bind(&Visualizer::publishFakeRobot, visualizer, std::placeholders::_1, &trueMap));
//#endif
////    beliefMap.subscribe(std::bind(&Visualizer::publishBeliefMap, visualizer, std::placeholders::_1));
////
////    //trueMap.subscribe(std::bind(&Visualizer::publishTrueMap2dSlice, visualizer, std::placeholders::_1, 0));
////    trueMap.subscribe(std::bind(&Visualizer::publishTrueMap, visualizer, std::placeholders::_1));
////
////    for (int j = 0; j < 20; ++j)
////        trueMap.publish();
////
////    visualizer->render();
////    return 0;
//
//#ifdef PLANNER_2D_TEST
//    planner.subscribe(std::bind(&Visualizer::publishTrajectoryPlanner, visualizer, std::placeholders::_1));
//#endif
//#ifdef FAKE_2D
//    double k_inconsistency = 1;
////    beliefMap.subscribe(std::bind(&Visualizer::publishBeliefMapFull,
////                                  visualizer, std::placeholders::_1,
////                                  true));
//    beliefMap.subscribe(std::bind(&Visualizer::publishBeliefInconsistencyMapFull,
//                                  visualizer, std::placeholders::_1,
//                                  trueMap,
//                                  k_inconsistency));
////    logOddsMap.subscribe(std::bind(&Visualizer::publishLogOddsMapFull,
////                                   visualizer, std::placeholders::_1,
////                                   true));
//    logOddsMap.subscribe(std::bind(&Visualizer::publishLogOddsInconsistencyMapFull,
//                                   visualizer, std::placeholders::_1,
//                                   trueMap,
//                                   k_inconsistency));
////    gaussianProcessMap.subscribe(std::bind(&Visualizer::publishGaussianProcessMapFull,
////                                           visualizer, std::placeholders::_1,
////                                           true));
//    gaussianProcessMap.subscribe(std::bind(&Visualizer::publishGaussianProcessInconsistencyMapFull,
//                                           visualizer, std::placeholders::_1,
//                                           trueMap,
//                                           k_inconsistency));
//#else
//    // todo reactivate
////    beliefMap.subscribe(std::bind(&Visualizer::publishBeliefMapFull, visualizer, std::placeholders::_1));
////    logOddsMap.subscribe(std::bind(&Visualizer::publishLogOddsMapFull, visualizer, std::placeholders::_1));
//#endif
//    robot.sensor().subscribe(std::bind(&Visualizer::publishStereoCameraSensor, visualizer, std::placeholders::_1));
//#endif
//
//    robot.registerObserver(&handleObservation);
//
//
//    Trajectory trajectory = {
//            TrajectoryPoint(Eigen::Vector3f(-0.92f, -0.25f, 0.f), (float) (0.f * M_PI / 180.f)),
//            TrajectoryPoint(Eigen::Vector3f(-0.77f, -0.25f, 0.f), (float) (0.f * M_PI / 180.f)),
//            TrajectoryPoint(Eigen::Vector3f(-0.61f, -0.25f, 0.f), (float) (0.f * M_PI / 180.f)),
//            TrajectoryPoint(Eigen::Vector3f(-0.46f, -0.25f, 0.f), (float) (0.f * M_PI / 180.f)),
//            TrajectoryPoint(Eigen::Vector3f(-0.30f, -0.25f, 0.f), (float) (15.f * M_PI / 180.f)),
//            TrajectoryPoint(Eigen::Vector3f(-0.16f, -0.21f, 0.f), (float) (60.f * M_PI / 180.f)),
//            TrajectoryPoint(Eigen::Vector3f(-0.08f, -0.07f, 0.f), (float) (80.f * M_PI / 180.f)),
//            TrajectoryPoint(Eigen::Vector3f(-0.05f, 0.09f, 0.f), (float) (90.f * M_PI / 180.f)),
//            TrajectoryPoint(Eigen::Vector3f(-0.05f, 0.24f, 0.f), (float) (80.f * M_PI / 180.f)),
//            TrajectoryPoint(Eigen::Vector3f(-0.03f, 0.39f, 0.f), (float) (85.f * M_PI / 180.f)),
//            TrajectoryPoint(Eigen::Vector3f(-0.01f, 0.56f, 0.f), (float) (30.f * M_PI / 180.f)),
//            TrajectoryPoint(Eigen::Vector3f(0.13f, 0.63f, 0.f), (float) (-10.f * M_PI / 180.f)),
//            TrajectoryPoint(Eigen::Vector3f(0.30f, 0.61f, 0.f), (float) (-10.f * M_PI / 180.f)),
//            TrajectoryPoint(Eigen::Vector3f(0.45f, 0.58f, 0.f), (float) (5.f * M_PI / 180.f)),
//            TrajectoryPoint(Eigen::Vector3f(0.62f, 0.58f, 0.f), (float) (-25.f * M_PI / 180.f)),
//            TrajectoryPoint(Eigen::Vector3f(0.74f, 0.51f, 0.f), (float) (-70.f * M_PI / 180.f)),
//            TrajectoryPoint(Eigen::Vector3f(0.77f, 0.36f, 0.f), (float) (-85.f * M_PI / 180.f)),
//            TrajectoryPoint(Eigen::Vector3f(0.77f, 0.20f, 0.f), (float) (-90.f * M_PI / 180.f)),
//            TrajectoryPoint(Eigen::Vector3f(0.77f, 0.05f, 0.f), (float) (-100.f * M_PI / 180.f)),
//            TrajectoryPoint(Eigen::Vector3f(0.74f, -0.11f, 0.f), (float) (-140.f * M_PI / 180.f)),
//            TrajectoryPoint(Eigen::Vector3f(0.61f, -0.17f, 0.f), (float) (-170.f * M_PI / 180.f)),
//            TrajectoryPoint(Eigen::Vector3f(0.46f, -0.17f, 0.f), (float) (-175.f * M_PI / 180.f)),
//            TrajectoryPoint(Eigen::Vector3f(0.30f, -0.17f, 0.f), (float) (-180.f * M_PI / 180.f))
//    };
//
//#if defined(ISM_RUNS)
//    std::vector<double> increments = {0.05, 0.25, 0.4};
//    std::vector<double> rampSizes = {0.025, 0.05, 0.1, 0.3};
//    std::vector<double> topSizes = {0.025, 0.05, 0.1, 0.3};
//    unsigned int round = 0;
//#if defined(REAL_2D)
//    Drone drone;
//    drone.registerObserver(&handleObservation);
//    visualizer->publishTrueMap2dSlice(&trueMap, 0);
//    std::cout << "Running CARMEN file: " << carmenFile << std::endl;
//#endif
//    for (auto increment : increments)
//    {
//        for (auto rampSize : rampSizes)
//        {
//            for (auto topSize : topSizes)
//            {
//                ROS_INFO("Running with ISM parameters: increment=%f rampSize=%f topSize=%f",
//                         increment, rampSize, topSize);
//                LogOddsMap::parameters.increment = increment;
//                LogOddsMap::parameters.rampSize = rampSize;
//                LogOddsMap::parameters.topSize = topSize;
//                beliefMap.reset();
//                logOddsMap.reset();
//#if defined(REAL_2D)
//                drone.runCarmenFile(carmenFile, "FLASER", true, 10);//, "ROBOTLASER1", false);
//#else
//                robot.run(trajectory);
//#endif
//
//                stats->saveToFile(
//                        homedir + "/catkin_ws/src/smap/stats/ism_albert_runs/stats_" + std::to_string(round++) +
//                        ".bag");
//                stats->reset();
//            }
//        }
//    }
//#elif defined(GP_RUNS)
//    // obtain observations
//        robot.run(trajectory);
//
//        std::vector<double> p1s = {-2, -2.5, -3};
//        std::vector<double> p2s = {-2.5, -3.5, -4.5};
//        std::vector<double> p3s = {-0.5, -1, -1.5};
//        unsigned int round = 0;
//        for (auto p1 : p1s)
//        {
//            for (auto p2 : p2s)
//            {
//                for (auto p3 : p3s)
//                {
//                    gaussianProcessMap.reset();
//                    ROS_INFO("Running with GP parameters: p1=%f p2=%f p3=%f",
//                             p1, p2, p3);
//                    GaussianProcessMap::parameters.parameter1 = p1;
//                    GaussianProcessMap::parameters.parameter2 = p2;
//                    GaussianProcessMap::parameters.parameter3 = p3;
//                    gaussianProcessMap.updateParameters();
//                    gaussianProcessMap.update(allObservations);
//
//                    stats->update(logOddsMap, beliefMap, gaussianProcessMap, robot);
//
//                    stats->saveToFile(homedir + "/catkin_ws/src/smap/stats/gp_runs/stats_" + std::to_string(round++) + ".bag");
//                    stats->reset();
//                }
//            }
//        }
//        return EXIT_SUCCESS;
//#endif
//
//    // TODO compute Hilbert map using all measurements
////    gaussianProcessMap.update(allObservations);
////    gaussianProcessMap.publish();
////    stats->update(logOddsMap, beliefMap, gaussianProcessMap, robot);
//
//    std::stringstream ss;
//    ss << Parameters::sensorNoiseStd;
//    stats->saveToFile(homedir + "/catkin_ws/src/smap/stats/stats_carmen_std_" + ss.str() + ".bag");
//#ifdef ENABLE_VISUALIZATION
//#if defined(REAL_2D) || defined(REAL_3D)
//    trajectory = drone.poseHistory();
//#endif
//    std::cout << "Trajectory bounding box: " << trajectory.boundingBox().str() << std::endl;
//    for (int i = 0; i < 1; i++)
//    {
//        visualizer->publishObservation(&allObservations);
//        visualizer->publishTrajectory(&trajectory);
//        visualizer->publishTrueMap2dSlice(&trueMap, 0);
//        visualizer->sleep(1);
//        visualizer->publishBeliefMapFull(&beliefMap);
//        visualizer->publishLogOddsMapFull(&logOddsMap);
//    }
//#endif
//    std::cout << allObservations.measurements().size()
//              << " measurements were taken in total." << std::endl;
//
//#ifndef PLANNER_2D_TEST
////    visualizer->publishBeliefMapFull(&beliefMap);
////    visualizer->publishLogOddsMapFull(&logOddsMap);
////    visualizer->publishTrueMap(&trueMap);
//#endif
//
//    delete stats;
//
//#ifdef ENABLE_VISUALIZATION
//    visualizer->render();
//#endif

    return EXIT_SUCCESS;
}
