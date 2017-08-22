#pragma once

/**********************************************************************************************
 * This code comes from                                                                       *
 * https://github.com/qianyizh/ElasticReconstruction/blob/master/BuildCorrespondence/Helper.h *
 **********************************************************************************************/

#include <vector>
#include <fstream>
#include <Eigen/Core>

typedef std::pair< int, int > IntPair;

struct FramedTransformation
{
    int id1;
    int id2;
    int frame;
    Eigen::Matrix4d transformation;

    FramedTransformation(int id1, int id2, int f, Eigen::Matrix4d t)
            : id1(id1), id2(id2), frame(f), transformation(t)
    {
    }
};

struct ICLNUIMTrajectory
{
    std::vector <FramedTransformation> data;
    int index;

    void loadFromFile(std::string filename)
    {
        data.clear();
        index = 0;
        int id1, id2, frame;
        Eigen::Matrix4d trans;
        FILE *f = fopen(filename.c_str(), "r");
        if (f != NULL)
        {
            char buffer[1024];
            while (fgets(buffer, 1024, f) != NULL)
            {
                if (strlen(buffer) > 0 && buffer[0] != '#')
                {
                    sscanf(buffer, "%d %d %d", &id1, &id2, &frame);
                    fgets(buffer, 1024, f);
                    sscanf(buffer, "%lf %lf %lf %lf", &trans(0, 0), &trans(0, 1), &trans(0, 2), &trans(0, 3));
                    fgets(buffer, 1024, f);
                    sscanf(buffer, "%lf %lf %lf %lf", &trans(1, 0), &trans(1, 1), &trans(1, 2), &trans(1, 3));
                    fgets(buffer, 1024, f);
                    sscanf(buffer, "%lf %lf %lf %lf", &trans(2, 0), &trans(2, 1), &trans(2, 2), &trans(2, 3));
                    fgets(buffer, 1024, f);
                    sscanf(buffer, "%lf %lf %lf %lf", &trans(3, 0), &trans(3, 1), &trans(3, 2), &trans(3, 3));
                    data.push_back(FramedTransformation(id1, id2, frame, trans));
                }
            }
            fclose(f);
        }
    }

    void saveToFile(std::string filename)
    {
        FILE *f = fopen(filename.c_str(), "w");
        for (int i = 0; i < (int) data.size(); i++)
        {
            Eigen::Matrix4d &trans = data[i].transformation;
            fprintf(f, "%d\t%d\t%d\n", data[i].id1, data[i].id2, data[i].frame);
            fprintf(f, "%.8f %.8f %.8f %.8f\n", trans(0, 0), trans(0, 1), trans(0, 2), trans(0, 3));
            fprintf(f, "%.8f %.8f %.8f %.8f\n", trans(1, 0), trans(1, 1), trans(1, 2), trans(1, 3));
            fprintf(f, "%.8f %.8f %.8f %.8f\n", trans(2, 0), trans(2, 1), trans(2, 2), trans(2, 3));
            fprintf(f, "%.8f %.8f %.8f %.8f\n", trans(3, 0), trans(3, 1), trans(3, 2), trans(3, 3));
        }
        fclose(f);
    }
};
