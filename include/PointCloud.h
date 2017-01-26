#include <string>
#include "Parameters.hpp"

class PointCloud
{
public:
    void loadPly(std::string filename);

    void visualize() const;

    pcl::PointCloud<Parameters::PointType> &cloud()
    {
        return _cloud;
    }

private:
    pcl::PointCloud<Parameters::PointType> _cloud;
};

