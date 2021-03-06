// CS184 IK Solver
#include <vector>
#include <iostream>
#include <fstream>
#include <cmath>

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

#include <time.h>
#include <math.h>

#include <stdio.h>
#include <string.h>
#include <cmath>

#include <sstream>
#include <string>

#include "Eigen/Dense"

#ifdef _WIN32
static DWORD lastTime;
#else
static struct timeval lastTime;
#endif

using namespace std;
using namespace Eigen;

#define PI 3.14159265359

int iteration = 0;

//****************************************************
// Some Classes
//****************************************************
class Viewport {
  public:
    int w, h; // width and height
};

class Joint {
  public:
    float length;
    Vector3f end;
    Vector3f ri;
    Matrix3f Ri;
    int index;

    //Constructor
    Joint(float length, int i): length(length) {
      Matrix3f R(3, 3);
      R << 1, 0, 0, 0, 1, 0, 0, 0, 1;
      Ri = R;
      ri = Vector3f(0, 0, 0);
      calculateEnd();
      index = i;
    };

    /* Calculates end point of the joint. */
    Vector3f calculateEnd() {
      MatrixXf len(3, 1);
      len << length, 0, 0;
      MatrixXf endPoint = Ri * len;
      end = Vector3f(endPoint(0, 0), endPoint(1, 0), endPoint(2, 0));
      return end;
    }

    /* Adds a rotation to the current configuration */
    void addRotation(float rx, float ry, float rz) {
      ri += Vector3f(rx, ry, rz);
      Vector3f nri = ri.normalized();

      float theta = ri.norm();
      MatrixXf rot(3, 1);
      rot << nri.x(), nri.y(), nri.z();

      Matrix3f matrix;
      matrix << 0, -nri.z(), nri.y(), nri.z(), 0, -nri.x(), -nri.y(), nri.x(), 0;

      Matrix3f identity;
      identity << 1, 0, 0, 0, 1, 0, 0, 0, 1;
      Ri = identity + sin(theta) * matrix + (1 - cos(theta)) * (matrix * matrix);
      calculateEnd();
    }

    /* Gets the joint transformation matrix */
    Matrix4f getTransformationMatrix() {
      Matrix4f X = Matrix4f::Zero();
      X.block(0, 0, 3, 3) << Ri(0, 0), Ri(0, 1), Ri(0, 2), Ri(1, 0), Ri(1, 1), Ri(1, 2),
        Ri(2, 0), Ri(2, 1), Ri(2, 2);
      X.block(0, 3, 4, 1) << end.x(), end.y(), end.z(), 1;
      return X;
    }

    /* Composes rotation matrixs up to this joint */
    Matrix3f getTotalRotationMatrix(vector<Joint> jnts) {
      Matrix3f composite;
      composite << 1, 0, 0, 0, 1, 0, 0, 0, 1;
      for (int i = index; i >= 0; i--) {
        composite *= jnts[i].Ri;
      }
      return composite;
    }

    void draw(Vector3f start) {
      //Draw arm segment
      glLineWidth(3); 
      glColor3f(1.0, 1.0, 0.0);
      glBegin(GL_LINES);
      glVertex3f(start.x(), start.y(), start.z());
      glVertex3f(start.x() + end.x(), start.y() + end.y(), start.z() + end.z());
      glEnd();

      //Draw Ball Joint
      GLUquadric *quad;
      quad = gluNewQuadric();
      glTranslatef(start.x(), start.y(), start.z());
      gluSphere(quad,0.02,100,20);

      glLoadIdentity();
    };
};

//****************************************************
// Global Variables
//****************************************************
vector<Joint> joints;
Viewport viewport;
Vector3f pe;
Vector3f goal;
float step;
float epsilon;
float accum = 0.0f;

//****************************************************
// General Functions
//****************************************************

/* [v] */
Matrix3f getCrossProductMatrix(Vector3f v) {
  Matrix3f matrix;
  matrix <<      0, -v.z(),  v.y(),
             v.z(),      0, -v.x(),
            -v.y(),  v.x(),      0;
  return matrix;
}

/* Gets J = (J1' | J2' | ... | JN') */
MatrixXf getJacobian() {
  MatrixXf jacobian(3, 3*joints.size());
  Matrix4f Xcurrent;
  Xcurrent << 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1;

  /* Apply to last node */
  Joint joint = joints[joints.size() - 1];
  Matrix3f totalRotation = joint.getTotalRotationMatrix(joints);
  Vector4f homoEnd = Vector4f(joint.end.x(), joint.end.y(), joint.end.z(), 1);
  Vector4f xpn = Xcurrent * homoEnd;
  Vector3f point = Vector3f(xpn[0] / xpn[3], xpn[1] / xpn[3], xpn[2] / xpn[3]);
  Matrix3f Ji = (-1 * totalRotation) * getCrossProductMatrix(point);
  jacobian.block(0, 3*joints.size() - 3, 3, 3) << Ji(0, 0), Ji(0, 1), Ji(0, 2),
                                                  Ji(1, 0), Ji(1, 1), Ji(1, 2),
                                                  Ji(2, 0), Ji(2, 1), Ji(2, 2);
  Xcurrent = joint.getTransformationMatrix();

  /* Loop through the rest */
  for (int i = joints.size() - 2; i >= 0; i--) {
    joint = joints[i];
    totalRotation = joint.getTotalRotationMatrix(joints);
    homoEnd = Vector4f(joint.end.x(), joint.end.y(), joint.end.z(), 1);
    xpn = Xcurrent * homoEnd;
    point = Vector3f(xpn[0] / xpn[3], xpn[1] / xpn[3], xpn[2] / xpn[3]);
    Ji = totalRotation * getCrossProductMatrix(point);
    jacobian.block(0, 3*i, 3, 3) << Ji(0, 0), Ji(0, 1), Ji(0, 2),
                                    Ji(1, 0), Ji(1, 1), Ji(1, 2),
                                    Ji(2, 0), Ji(2, 1), Ji(2, 2);
    Xcurrent = joint.getTransformationMatrix() * Xcurrent;
  }

  return jacobian;
}

/* Recalculates the end effector in pe */
void getEndEffector() {
  Vector4f origin = Vector4f(0, 0, 0, 1);
  for (int i = 0; i < joints.size(); i += 1) {
    origin = joints[i].getTransformationMatrix() * origin;
  }
  pe = Vector3f(origin[0] / origin[3], origin[1] / origin[3], origin[2] / origin[3]);
}

/* Solves IK */
void solveIK() {
  MatrixXf jacobian = getJacobian();
  getEndEffector();
  Vector3f dp = pe + step*(goal - pe);
  MatrixXf dr = jacobian.jacobiSvd(Eigen::ComputeThinU|Eigen::ComputeThinV).solve(dp);
  int start = 0;
  for (vector<Joint>::size_type i = 0; i != joints.size(); i++) {
    start = dr.rows() - 3*(i + 1);
    joints[i].addRotation(dr(start, 0), dr(start + 1, 0), dr(start + 2, 0));
  }
  getEndEffector();
}

/* Step IK */
void stepIK() {
  if ((goal - pe).norm() > epsilon) {
    solveIK();
  }
}


//****************************************************
// reshape viewport if the window is resized
//****************************************************
void myReshape(int w, int h) {
  viewport.w = w;
  viewport.h = h;

  glViewport(0,0,viewport.w,viewport.h);// sets the rectangle that will be the window
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();                // loading the identity matrix for the screen

  //----------- setting the projection -------------------------
  // glOrtho sets left, right, bottom, top, zNear, zFar of the chord system


  //glOrtho(-1, 1 + (w-400)/200.0 , -1 -(h-400)/200.0, 1, 1, -1); // resize type = add
  //glOrtho(-w/400.0, w/400.0, -h/400.0, h/400.0, 1, -1); // resize type = center

  glOrtho(-1, 1, -1, 1, 1, -1);    // resize type = stretch

  //------------------------------------------------------------
}


//****************************************************
// sets the window up
//****************************************************
void initScene(){
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f); // Clear to black, fully transparent

  myReshape(viewport.w,viewport.h);
}

//***************************************************
// function that does the actual drawing
//***************************************************

void draw() {

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);                 // clear the color buffer (sets everything to black)
  glMatrixMode(GL_MODELVIEW);                  // indicate we are specifying camera transformations
  glLoadIdentity();                            // make sure transformation is "zero'd"

  //----------------------- code to draw objects --------------------------

  stepIK();

  Vector3f start = Vector3f(0, 0, 0);
  for (vector<Joint>::size_type i = 0; i != joints.size(); i++) {
    joints[i].draw(start);
    start += joints[i].end;
  }

  /* Draw end effector */
  GLUquadric *quad;
  quad = gluNewQuadric();
  glTranslatef(goal.x(), goal.y(), goal.z());
  gluSphere(quad,0.02,100,20);
  
  glFlush();
  glutSwapBuffers();                           // swap buffers (we earlier set double buffer)
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

//****************************************************
// called by glut when a key is pressed
//****************************************************

void keyboardHandle(unsigned char key, int x, int y) { // Funtion to call the correct function when a key is pressed
  if(key == 32) {
    exit(0);
  }
}

void specialKey(int key, int x, int y) {
}

//****************************************************
// parse input file
//****************************************************

/* Splits a string by whitespace. */
std::vector<std::string> split(const std::string &str) {
  std::vector <std::string> tokens;
  std::stringstream stream(str);
  std::string temp;
  
  while (stream >> temp) {
    tokens.push_back(temp);
  }
  return tokens;
}

/* Displays an error if too many args */
void argumentError(std::string command, int expected) {
  string err = "Too many arguments in \"" + command + "\". Expected: " + to_string(expected) + "\n";
  fprintf(stderr, "%s", err.c_str());
}

/* Parses a line of floats */
vector<float> parseLine(vector<string> tokens, int expected, string command) {
  vector<float> data;
  int depth = 0;
  for (vector<string>::size_type i = 1; i != tokens.size(); i++) {
    if (tokens[i].compare("") != 0) {
      if (depth == expected) { argumentError(command, expected); break; }
      data.push_back(stof(tokens[i]));
      depth += 1;
    }
  }
  return data;
}

void parseJoint(vector<string> tokens) { 
  vector<float> data = parseLine(tokens, 1, "joint");
  Joint *j = new Joint(data[0], joints.size());
  joints.push_back(*j); 
  pe += j->calculateEnd();
}

void parseGoal(vector<string> tokens) {
  vector<float> data = parseLine(tokens, 3, "end");
  goal = Vector3f(data[0], data[1], data[2]);
}

void parseStepSize(vector<string> tokens) {
  vector<float> data = parseLine(tokens, 1, "step");
  step = data[0];
}

void parseEpsilon(vector<string> tokens) {
  vector<float> data = parseLine(tokens, 1, "epsilon");
  epsilon = data[0];
}

void parseInput(int argc, char** argv) {
  string line;
  while (getline(cin, line)) {
    vector<string> tokens = split(line);
    if (tokens.size() == 0) { continue; }
    if (tokens[0].compare("joint") == 0) { parseJoint(tokens); }
    if (tokens[0].compare("end") == 0) { parseGoal(tokens); }
    if (tokens[0].compare("step") == 0) { parseStepSize(tokens); }
    if (tokens[0].compare("epsilon") == 0) { parseEpsilon(tokens); }
  }
}

//****************************************************
// the usual stuff, nothing exciting here
//****************************************************
int main(int argc, char *argv[]) {
  
  //This initializes glut
  glutInit(&argc, argv);

  //read command line arguments 
  pe = Vector3f(0, 0, 0);
  parseInput(argc, argv);

  //This tells glut to use a double-buffered window with red, green, and blue channels 
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

  // Initalize theviewport sizes
  viewport.w = 1000;
  viewport.h = 1000;

  //The size and position of the window
  glutInitWindowSize(viewport.w, viewport.h);
  glutInitWindowPosition(0, 0);
  glutCreateWindow("CS184 - IKSolver");

  initScene();                                 // quick function to set up scene

  glutDisplayFunc(draw);                       // function to run when its time to draw something
  glutReshapeFunc(myReshape);                  // function to run when the window gets resized
  glutIdleFunc(myFrameMove);                   // function to run when not handling any other task
  glutKeyboardFunc(keyboardHandle);          // function to read keyboard input
  glutSpecialFunc(specialKey);             // function to read special keyboard input - arrow keys

  glutMainLoop();                              // infinite loop that will keep drawing and resizing and whatever else
  return 0;
}