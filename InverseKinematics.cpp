#include <vector>
#include <iostream>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <stdexcept>
#include <algorithm>
#include <iterator>
#include <vector>
#include <time.h>
#include <limits>



#ifdef _WIN32
#include <windows.h>
#else
#include <sys/time.h>
#endif

#ifdef OSX
#include <GLUT/glut.h>
#include <OpenGL/glu.h>
#else
#include <GL/glut.h>
#include <GL/glu.h>
#endif


#include <Eigen/Dense>
#include <Eigen/StdVector>

using namespace std;
using namespace Eigen;


const double THETA_CHANGE = 0.001;


class Viewport {
public:
    int w, h; // width and height
};


class Joint {
public:
    double length;
    Vector3d theta;
    Matrix3d rotationMatrix;
    Vector3d basePoint, endPoint;
}



class System {
private:
    vector<Joint> joints;
public:

    System(vector<Joint> inJoints, Vector3d inBasePoint, Vector3d inEndPoint);
    MatrixXd getJacobian();

}


System::System(vector<Joint> inJoints, Vector3d inBasePoint, Vector3d inEndPoint) {
    joints = inJoints;
    basePoint = inBasePoint;
    endPoint = inEndPoint;
}


Vector3d rotateX(const Vector3d& p, double theta) {
    Matrix3d rotateX;
    rotateX <<
        1, 0, 0,
        0, cos(theta), -sin(theta),
        0, sin(theta), sin(theta);

    return rotateX * p;
}

Vector3d rotateY(const Vector3d& p, double theta) {
    Matrix3d rotateY;
    rotateY <<
        cos(theta), 0, sin(theta),
        0, 1, 0,
        -sin(theta), 0, cos(theta);

    return rotateY * p;
}

Vector3d rotateZ(const Vector3d& p, double theta) {
    Matrix3d rotateZ;
    rotateZ <<
        cos(theta), -sin(theta), 0,
        sin(theta), sin(theta), 0,
        0, 0, 1;

    return rotateZ * p;
}


Vector3d translate(const Vector3d& p, const Vector3d& to) {
    Matrix4d t;
    t <<
        0, 0, 0, -to[0]
        0, 0, 0, -to[1]
        0, 0, 0, -to[2]
        0, 0, 0, 1;

    Vector4d pAffine(p[0], p[1], p[2], 1);

    Vector4d result = t * pAffine;

    return Vector3d(new[0], new[1], new[2]);
}


MatrixXd System::getJacobian() {
    MatrixXd result;

    Vector3d deltaTheta(1, 1, 1);
    deltaTheta *= THETA_CHANGE;

    for (int i = 0; i < joints.size(); i++) {
        Vector3d newTheta = joints[i].theta + deltaTheta;

        Vector3d original = translate(joints[i].endPoint, joints[i].basePoint);

        Vector3d p = rotateX(original, joints[i].theta[0]);
        Vector3d newP = rotateX(original, newTheta[0]);

        Vector3d col1 = (p - newP) / THETA_CHANGE;

        result(i * 3, 0) = col1[0];
        result(i * 3, 1) = col1[1];
        result(i * 3, 2) = col1[2];

        p = rotateY(original, joints[i].theta[1]);
        newP = rotateY(original, newTheta[1]);

        Vector3d col2 = (p - newP) / THETA_CHANGE;

        result(i * 3 + 1, 0) = col2[0];
        result(i * 3 + 1, 1) = col2[1];
        result(i * 3 + 1, 2) = col2[2];

        p = rotateZ(original, joints[i].theta[2]);
        newP = rotateZ(original, newTheta[2]);

        Vector3d col3 = (p - newP) / THETA_CHANGE;

        result(i * 3 + 2, 0) = col3[0];
        result(i * 3 + 2, 1) = col3[1];
        result(i * 3 + 2, 2) = col3[2];
    }

    return result;
}



bool update(Vector3f g) {
    g_sys = g - system.basepoint
    if I cant reach the goal:
        g = new goal that can be reached
    dp = g - system.endpoint
    if dp.norm() > eps:
        J = system.getJ()
        svd(J)
        dtheta = svd.solve(dp)

        system.updateAngles(dtheta)
        system.updateEndpoint
        return false

    return true
}



Viewport viewport;


void myReshape(int w, int h) {
    viewport.w = w;
    viewport.h = h;

    glViewport(0, 0, viewport.w, viewport.h);// sets the rectangle that will be the window
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();                // loading the identity matrix for the screen

    //----------- setting the projection -------------------------
    // glOrtho sets left, right, bottom, top, zNear, zFar of the chord system
    // glOrtho(-1, 1 + (w-400)/200.0 , -1 -(h-400)/200.0, 1, 1, -1); // resize type = add
     glOrtho(-w/400.0, w/400.0, -h/400.0, h/400.0, 2, -2); // resize type = center

    //glOrtho(-4, 4, -4, 4, 2, -2);    // resize type = stretch

    //------------------------------------------------------------
}




void initScene(){
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f); // Clear to black, fully transparent

    myReshape(viewport.w, viewport.h);
}




void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();


    glShadeModel(GL_FLAT);

    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);


    glFlush();
    glutSwapBuffers();
}

//****************************************************
// called by glut when there are no messages to handle
//****************************************************
void myFrameMove() {
    //nothing here for now
#ifdef _WIN32
    Sleep(10);                                   //give ~10ms back to OS (so as not to waste the CPU)
#endif
    glutPostRedisplay(); // forces glut to call the display function (myDisplay())
}


int main(int argc, char *argv[]) {

    glutInit(&argc, argv);

    //This tells glut to use a double-buffered window with red, green, and blue channels
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);


    // Initalize theviewport size
    viewport.w = 800;
    viewport.h = 800;

    //The size and position of the window
    glutInitWindowSize(viewport.w, viewport.h);
    glutInitWindowPosition(0, 0);
    glutCreateWindow("CS184 Assignment 4");

    initScene();                                 // quick function to set up scene

    glEnable(GL_DEPTH_TEST);

    glutDisplayFunc(display);                    // function to run when its time to draw something
    glutReshapeFunc(myReshape);                  // function to run when the window gets resized
    glutIdleFunc(myFrameMove);                   // function to run when not handling any other task
    glutMainLoop();                              // infinite loop that will keep drawing and resizing and whatever else

    return 0;
}