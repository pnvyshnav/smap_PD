#include "../include/Visualizer.h"

#include <GL/freeglut.h>

TrueMap *_trueMap;
BeliefMap *_beliefMap;
FakeRobot<> *_robot;

GLuint map_tex_id, obs_tex_id;

void draw()
{
    glLoadIdentity();

    glClearColor(0.1f, 0.1f, 0.1f, 1.f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // render map
    std::vector<float> occupancies;
    for (unsigned int y = 0; y < Parameters::voxelsPerDimensionY; ++y)
    {
        for (unsigned int x = 0; x < Parameters::voxelsPerDimensionX; ++x)
        {
            for (unsigned int z = 0; z < 1; ++z)
            {
                auto _x = Parameters::xMin + x * Parameters::voxelSize;
                auto _y = Parameters::yMin + y * Parameters::voxelSize;
                auto _z = Parameters::zMin + z * Parameters::voxelSize;
                QBeliefVoxel voxel = _beliefMap->query(_x, _y, _z);
                occupancies.push_back(1.f - (float) _beliefMap->getVoxelMean(voxel));
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
                 Parameters::voxelsPerDimensionX,
                 Parameters::voxelsPerDimensionY,
                 0, GL_LUMINANCE, GL_FLOAT, map_tex);

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, map_tex_id);

    glBegin(GL_QUADS);
    float x1 = -1, x2 = 1, y1 = -.9f, y2 = 1;
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
    unsigned int rays = 64;
    float *obs_tex = _robot->getOmniDirectionalReachability(rays).data();

    glBindTexture(GL_TEXTURE_2D, obs_tex_id);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE,
                 rays, 1, 0, GL_LUMINANCE, GL_FLOAT, obs_tex);

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, obs_tex_id);

    glBegin(GL_QUADS);
    x1 = -1; x2 = 1; y1 = -1; y2 = -.92f;
    glTexCoord2f(0, 0);
    glVertex3f(  x1, y1, 0.f);
    glTexCoord2f(1, 0);
    glVertex3f(  x2, y1, 0.f);
    glTexCoord2f(1, 1);
    glVertex3f(  x2, y2, 0.f);
    glTexCoord2f(0, 1);
    glVertex3f(  x1, y2, 0.f);
    glEnd();

    glDisable(GL_TEXTURE_2D);
    glColor4f(1, 0, 0, 1);
    glBegin(GL_POINTS);
    glVertex2f(_robot->position().x(), _robot->position().y());
    glEnd();
    glBegin(GL_LINES);
    auto direction = Eigen::Vector3f(_robot->orientation().x(), _robot->orientation().y(), _robot->orientation().z());
    const double hFactor = Parameters::StereoCameraHorizontalFOV / (Parameters::StereoCameraHorizontalPixels-1);
    for (unsigned int hp = 0; hp < Parameters::StereoCameraHorizontalPixels; ++hp)
    {
        double angleH = -Parameters::StereoCameraHorizontalFOV/2. + hp * hFactor;
        auto rotHorizontal = Eigen::AngleAxis<float>((float) angleH, Eigen::Vector3f(0, 0, 1));
        Eigen::Vector3f rotated = rotHorizontal * (direction);

        glVertex2f(_robot->position().x(), _robot->position().y());
        glVertex2f((GLfloat) (_robot->position().x() + rotated[0] * Parameters::sensorRange),
                   (GLfloat) (_robot->position().y() + rotated[1] * Parameters::sensorRange));
    }
    glEnd();

    glColor4f(1, 1, 1, 1);

    glFinish();
    glutSwapBuffers();
}

void keyboard(unsigned char key, int x, int y)
{
//    std::cout << "Key: " << key << std::endl;
    switch (key)
    {
        case 'w': case 'W':
            _robot->setPosition(_robot->position() + _robot->orientation() * 0.02);
            _robot->run();
            break;
        case 's': case 'S':
            _robot->setPosition(_robot->position() - _robot->orientation() * 0.02);
            _robot->run();
            break;
        case 'a': case 'A':
            _robot->setYaw(_robot->yaw() + 0.1);
            _robot->run();
            break;
        case 'd': case 'D':
            _robot->setYaw(_robot->yaw() - 0.1);
            _robot->run();
            break;

        default:
            break;
    }
    glutPostRedisplay();
}

void Visualizer::render()
{
    glutMainLoop();
}

Visualizer::Visualizer(TrueMap *trueMap, BeliefMap *beliefMap, FakeRobot<> *robot)
{
    _trueMap = trueMap;
    _beliefMap = beliefMap;
    _robot = robot;

    glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);

    glutInitWindowSize(350, 380);
    glutInitWindowPosition(100, 100);

    glutCreateWindow("SMAP Gym Environment");

    glEnable(GL_POINT_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glPointSize(6.0);

    glGenTextures(1, &map_tex_id);
    glGenTextures(1, &obs_tex_id);

    glutDisplayFunc(draw);
    glutKeyboardFunc(keyboard);
}

Visualizer::~Visualizer()
{
}

void Visualizer::update()
{
    glutMainLoopEvent();
}
