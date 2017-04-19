#include "../include/Visualizer.h"

#include <GL/freeglut.h>
#include <ecl/time/stopwatch.hpp>

TrueMap *_trueMap;
MapType *_map;
FakeRobot<> *_robot;

unsigned int _episode = 0;
float _velocity = 0;
float _angularVelocity = 0;

std::vector<float> _observation;

std::vector<Parameters::Vec3Type> _positions;

GLuint map_tex_id, obs_tex_id;

unsigned int frame = 0;

int _window = 0;

bool _gymMode = false;
int _skipFrame = 10;
bool _egoCentric = false;
bool _egoCentricScaling = false;

int _egoCentricResolutionScaling = 4;

void draw()
{
    if (frame++ % _skipFrame != 0)
        return;

    glLoadIdentity();

    glClearColor(0.1f, 0.1f, 0.1f, 1.f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // render map
    std::vector<float> occupancies;
    unsigned int width = Parameters::voxelsPerDimensionX;
    unsigned int height = Parameters::voxelsPerDimensionY;
    if (!_egoCentric)
    {
        for (unsigned int y = 0; y < height; ++y)
        {
            for (unsigned int x = 0; x < width; ++x)
            {
                for (unsigned int z = 0; z < 1; ++z)
                {
                    auto _x = Parameters::xMin + x * Parameters::voxelSize;
                    auto _y = Parameters::yMin + y * Parameters::voxelSize;
                    auto _z = Parameters::zMin + z * Parameters::voxelSize;
                    auto voxel = _map->query(_x, _y, _z);
                    occupancies.push_back(1.f - (float) _map->getVoxelMean(voxel));
                }
            }
        }
    }
    else
    {
        width *= _egoCentricResolutionScaling;
        height *= _egoCentricResolutionScaling;
        for (unsigned int y = 0; y < height; ++y)
        {
            for (unsigned int x = 0; x < width; ++x)
            {
                for (unsigned int z = 0; z < 1; ++z)
                {
                    auto _x = Parameters::xMin + x * Parameters::voxelSize / _egoCentricResolutionScaling;
                    auto _y = Parameters::yMin + y * Parameters::voxelSize / _egoCentricResolutionScaling;
                    auto _z = Parameters::zMin + z * Parameters::voxelSize;
                    auto direction = Eigen::Vector3f(_x, _y, _z);
                    auto rotHorizontal = Eigen::AngleAxis<float>(
                            (float) (_robot->yaw() - M_PI / 2.), Eigen::Vector3f(0, 0, 1));
                    Eigen::Vector3f rotated = rotHorizontal * (direction);
                    double distx = std::abs(rotated[0]);
                    double disty = std::abs(rotated[1]);
                    double xfactor = 1, yfactor = 1;
                    if (_egoCentricScaling)
                    {
                        xfactor = std::pow(distx, 2.);
                        yfactor = std::pow(disty, 2.);
                    }
                    double r = _map->filteredReachability(
                            rotated[0] * xfactor + _robot->position().x(),
                            rotated[1] * yfactor + _robot->position().y(),
                            rotated[2]);
//                    double r = _map->filteredReachability(_x, _y, _z);
                    occupancies.push_back((float) r);


//                    auto voxel = _map->query(_x, _y, _z);
//                    float m = (float) (voxel.type == GEOMETRY_VOXEL ? _map->getVoxelMean(voxel) : Parameters::priorMean);
//                    occupancies.push_back(1.f - m);
                }
            }
        }
    }
    float *map_tex = occupancies.data();

    glBindTexture(GL_TEXTURE_2D, map_tex_id);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE,
                 width, height,
                 0, GL_LUMINANCE, GL_FLOAT, map_tex);

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, map_tex_id);

    glBegin(GL_QUADS);
    float x1 = -1, x2 = 1, y1 = -.84f, y2 = 1;
    glTexCoord2f(0, 0);
    glVertex3f(  x1, y1, 0.f);
    glTexCoord2f(1, 0);
    glVertex3f(  x2, y1, 0.f);
    glTexCoord2f(1, 1);
    glVertex3f(  x2, y2, 0.f);
    glTexCoord2f(0, 1);
    glVertex3f(  x1, y2, 0.f);
    glEnd();


    // render observation
    if (!_observation.empty())
    {
        glBindTexture(GL_TEXTURE_2D, obs_tex_id);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE,
                     _observation.size(), 1, 0, GL_LUMINANCE, GL_FLOAT, _observation.data());

        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, obs_tex_id);

        glBegin(GL_QUADS);
        x1 = -1; x2 = 1; y1 = -.92; y2 = -.86f;
        glTexCoord2f(0, 0);
        glVertex3f(  x1, y1, 0.f);
        glTexCoord2f(1, 0);
        glVertex3f(  x2, y1, 0.f);
        glTexCoord2f(1, 1);
        glVertex3f(  x2, y2, 0.f);
        glTexCoord2f(0, 1);
        glVertex3f(  x1, y2, 0.f);
        glEnd();
    }

    glDisable(GL_TEXTURE_2D);

    // render position trace
    if (_egoCentric)
    {
        glColor3f(1, .5f, 0);
        glBegin(GL_LINE_STRIP);
        for (auto &p : _positions)
        {
            auto direction = Eigen::Vector3f(p.x(), p.y(), p.z());
            auto rotHorizontal = Eigen::AngleAxis<float>(
                    (float) (_robot->yaw() - M_PI / 2.), Eigen::Vector3f(0, 0, 1));
            Eigen::Vector3f rotated = rotHorizontal * (direction);
            double distx = std::abs(rotated[0]);
            double disty = std::abs(rotated[1]);
            double xfactor = 1, yfactor = 1;
            if (_egoCentricScaling)
            {
                xfactor = std::pow(distx, 2.);
                yfactor = std::pow(disty, 2.);
            }
            glVertex2f((GLfloat) (rotated[0] * xfactor + _robot->position().x()),
                       (GLfloat) (rotated[1] * yfactor + _robot->position().y()));
        }
        glEnd();
    }
    else
    {
        glColor3f(1, .5f, 0);
        glBegin(GL_LINE_STRIP);
        for (auto &p : _positions)
        {
            glVertex2f(p.x(), (GLfloat) ((p.y() + 1.) * (1.84 / 2.) - 0.84));
        }
        glEnd();
    }

    glColor3f(1, 0, 0);
    glBegin(GL_POINTS);
    Parameters::Vec3Type p;
    if (!_egoCentric)
        p = _robot->position();
    glVertex2f(p.x(), (GLfloat) ((p.y() + 1.) * (1.84 / 2.) - 0.84));
    glEnd();
    glBegin(GL_LINES);
    Eigen::Vector3f direction = Eigen::Vector3f(0, 1, 0);
    if (!_egoCentric)
        direction = Eigen::Vector3f(_robot->orientation().x(), _robot->orientation().y(), _robot->orientation().z());
    const double hFactor = Parameters::StereoCameraHorizontalFOV / (Parameters::StereoCameraHorizontalPixels-1);
    for (unsigned int hp = 0; hp < Parameters::StereoCameraHorizontalPixels; ++hp)
    {
        double angleH = -Parameters::StereoCameraHorizontalFOV/2. + hp * hFactor;
        auto rotHorizontal = Eigen::AngleAxis<float>((float) angleH, Eigen::Vector3f(0, 0, 1));
        Eigen::Vector3f rotated = rotHorizontal * (direction);

        glVertex2f(p.x(), (GLfloat) ((p.y() + 1.) * (1.84 / 2.) - 0.84));
        float y = (float) (p.y() + rotated[1] * Parameters::sensorRange);
        glVertex2f((GLfloat) (p.x() + rotated[0] * Parameters::sensorRange),
                   (GLfloat) ((y + 1.) * (1.84 / 2.) - 0.84));
    }
    glEnd();

    // render velocity
    glColor3f(0, 1, 0);
    glBegin(GL_QUADS);
    x1 = std::min(0.f, _velocity) * 3.f - 0.5f;
    x2 = std::max(0.f, _velocity) * 3.f - 0.5f;
    y1 = -1.f; y2 = -.94f;
    glVertex2f(x1, y1);
    glVertex2f(x2, y1);
    glVertex2f(x2, y2);
    glVertex2f(x1, y2);
    glEnd();
    glColor3f(1, 1, 1);
    glBegin(GL_LINES);
    glVertex2f(-0.5f, -1);
    glVertex2f(-0.5f, -.94f);
    glEnd();

    // render angular velocity
    glColor3f(0, 0, 1);
    glBegin(GL_QUADS);
    x1 = std::min(0.f, _angularVelocity) * 3.f + 0.5f;
    x2 = std::max(0.f, _angularVelocity) * 3.f + 0.5f;
    y1 = -1.f; y2 = -.94f;
    glVertex2f(x1, y1);
    glVertex2f(x2, y1);
    glVertex2f(x2, y2);
    glVertex2f(x1, y2);
    glEnd();
    glColor3f(1, 1, 1);
    glBegin(GL_LINES);
    glVertex2f(0.5f, -1);
    glVertex2f(0.5f, -.94f);
    glEnd();

    glColor3f(1, 1, 1);

    if (_gymMode)
    {
        // print episode number
        glRasterPos2f(-0.92f, 0.9f);
        const unsigned char *epistr = (const unsigned char *) std::to_string(_episode).c_str();
        glutBitmapString(GLUT_BITMAP_HELVETICA_18, epistr);
    }

    glutSwapBuffers();
}

void keyboard(unsigned char key, int x, int y)
{
//    std::cout << "Key: " << key << std::endl;
    _velocity = 0.f;
    _angularVelocity = 0.f;
    _positions.push_back(_robot->position());
    switch (key)
    {
        case 'w': case 'W':
            _robot->setPosition(_robot->position() + _robot->orientation() * 0.02);
            _robot->run();
            _velocity = 0.05f;
            break;
        case 's': case 'S':
            _robot->setPosition(_robot->position() - _robot->orientation() * 0.02);
            _robot->run();
            _velocity = -0.05f;
            break;
        case 'a': case 'A':
            _robot->setYaw(_robot->yaw() + 0.1);
            _robot->run();
            _angularVelocity = 0.1f;
            break;
        case 'd': case 'D':
            _robot->setYaw(_robot->yaw() - 0.1);
            _robot->run();
            _angularVelocity = -0.1f;
            break;
        case 'r': case 'R':
            _map->reset();
            _trueMap->shuffleCorridor();
            break;
        default:
            break;
    }
    _observation = _robot->getOmniDirectionalReachability();
    glutPostRedisplay();
}

void Visualizer::render()
{
    glutMainLoop();
}

Visualizer::Visualizer(TrueMap *trueMap, MapType *map, FakeRobot<> *robot, bool gymMode, int skipFrame, bool egoCentric)
{
    _trueMap = trueMap;
    _map = map;
    _robot = robot;

    _gymMode = gymMode;
    _skipFrame = skipFrame;
    _egoCentric = egoCentric;

    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);

    glutInitWindowSize(350, 380);
    glutInitWindowPosition(100, 100);

    _window = glutCreateWindow("SMAP Gym Environment");

    glEnable(GL_POINT_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glPointSize(6.0);

    glGenTextures(1, &map_tex_id);
    glGenTextures(1, &obs_tex_id);

    glutDisplayFunc(draw);
    glutKeyboardFunc(keyboard);

//    ecl::StopWatch watch;
//    for (int i = 0; i < 10000; ++i)
//    {
//        _robot->setYaw(_robot->yaw() + 0.1);
//        _observation = _robot->getOmniDirectionalReachability();
//        if (i % 500 == 0)
//            _map->reset();
//        _robot->run();
////        glutPostRedisplay();
////        glutMainLoopEvent();
//        std::cout << watch.elapsed() << std::endl;
//        watch.restart();
//    }
//    std::exit(0);
}

Visualizer::~Visualizer()
{
    glutDestroyWindow(_window);
}

void Visualizer::update()
{
    glutMainLoopEvent();
    glutPostRedisplay();
}

void Visualizer::setEpisode(unsigned int episode)
{
    _episode = episode;
    _positions.clear();
}
void Visualizer::setVelocity(float velocity)
{
    _velocity = velocity;
}
void Visualizer::setAngularVelocity(float angularVelocity)
{
    _angularVelocity = angularVelocity;
}

void Visualizer::setObservation(std::vector<float> observation)
{
    _observation = observation;
    _positions.push_back(_robot->position());
}
