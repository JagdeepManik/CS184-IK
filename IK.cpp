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

#include <sstream>
#include <string>
#include "Eigen/Dense"

#include <cstdio>
#include <iterator>

#ifdef _WIN32
static DWORD lastTime;
#else
static struct timeval lastTime;
#endif

using namespace std;
using namespace Eigen;

#define PI 3.14159265359

//****************************************************
// Classes
//****************************************************
class Viewport {
  public:
    int w, h; // width and height
};

class Joint {
  public:
    float length;
    Matrix3f rotation;
    MatrixXf colRotate;
    Vector3f end;
    Matrix3f jacobian;

    //Constructor
    Joint(Matrix3f rotation, float length, Vector3f end): length(length), rotation(rotation), end(end) {
    	findJacobian();
    };

    void findJacobian() {
    	jacobian << 0,  -end.z(),  end.y(),
    				end.z(),         0, -end.x(),
    				-end.y(),  end.x(),        0;   
    };

    void rotate(float rx, float ry, float rz) {
      Vector3f rvec = Vector3f(rx, ry, rz);
      MatrixXf rot(3, 1);
      float theta = rvec.norm();
      rvec.normalize();
      rot << rvec.x(), rvec.y(), rvec.z();

      Matrix3f matrx; 
      matrx << 0, -rvec.z(), rvec.y(), rvec.z(), 0, -rvec.x(), -rvec.y(), rvec.x(), 0;

      Matrix3f identity;
      identity << 1, 0, 0, 0, 1, 0, 0, 0, 1;
      rotation = identity + sin(theta) * matrx + (1 - cos(theta)) * (matrx * matrx);
      end = rotation * end;
      colRotate = rot;
    }

    void draw(Vector3f start) {

      //Draw arm segment
      glLineWidth(3); 
      glColor3f(1.0, 1.0, 0.0);
      glBegin(GL_LINES);
      glVertex3f(start.x(), start.y(), start.z());
      glVertex3f(end.x(), end.y(), end.z());
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
MatrixXf    *systemJacobian;
Viewport    viewport;
Vector3f    effector;
Vector3f    goal;
float       step;

vector<Joint> joints;
float accum = 0.0f;
int numJoints;

// jacobian composition
void composeJacobian() {
  Matrix3f composition;
  composition << 1, 0, 0, 0, 1, 0, 0, 0, 1;
  for (vector<Joint>::size_type i = 0; i != joints.size(); i++) {
    composition *= joints[i].rotation;
    joints[i].jacobian *= composition;
    Matrix3f ji = joints[i].jacobian;
    systemJacobian->block(3*joints.size() - 3*(i+1), 0, 3, 3) << ji(0, 0), ji(0, 1), 
      ji(0, 2), ji(1, 0), ji(1, 1), ji(1, 2), ji(2, 0), ji(2, 1), ji(2, 2); 
  }
  systemJacobian->transposeInPlace();
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

  Vector3f start = Vector3f(0, 0, 0);
  for (vector<Joint>::size_type i = 0; i != joints.size(); i++) {
    joints[i].draw(start);
    start = joints[i].end;
  }

  GLUquadric *quad;
  quad = gluNewQuadric();
  glTranslatef(goal.x(), goal.y(), goal.z());
  gluSphere(quad,0.02,100,20);

  //composeJacobian();
  
  glFlush();
  glutSwapBuffers();                           // swap buffers (we earlier set double buffer)
}

void ikSolve() {
  composeJacobian();
  Vector3f dp = effector + step*(goal - effector);
  MatrixXf dr = systemJacobian->jacobiSvd(Eigen::ComputeThinU|Eigen::ComputeThinV).solve(dp);
  int start = 0;
  for (vector<Joint>::size_type i = 0; i != joints.size(); i++) {
    start = dr.rows() - 3*(i + 1);
    joints[i].rotate(dr(start, 0), dr(start + 1, 0), dr(start + 2, 0));
  }
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
  Matrix3f rotation;
  rotation << 1, 0, 0, 0, 1, 0, 0, 0, 1;
  accum += data[0];
  Vector3f end = Vector3f(accum, 0, 0);
  Joint *j = new Joint(rotation, data[0], end);
  effector = j->end;
  joints.push_back(*j); 
}

void parseGoal(vector<string> tokens) {
  vector<float> data = parseLine(tokens, 3, "end");
  goal = Vector3f(data[0], data[1], data[2]);
}

void parseStepSize(vector<string> tokens) {
  vector<float> data = parseLine(tokens, 1, "step");
  step = data[0];
}

void parseInput(int argc, char** argv) {
  string line;
  while (getline(cin, line)) {
    vector<string> tokens = split(line);
    if (tokens.size() == 0) { continue; }
    if (tokens[0].compare("joint") == 0) { parseJoint(tokens); }
    if (tokens[0].compare("end") == 0) { parseGoal(tokens); }
    if (tokens[0].compare("step") == 0) { parseStepSize(tokens); }
  }
} 

//****************************************************
// the usual stuff, nothing exciting here
//****************************************************
int main(int argc, char *argv[]) {
  //This initializes glut
  glutInit(&argc, argv);

  //read command line arguments 
  parseInput(argc, argv);
  MatrixXf mat(3*joints.size(), 3);
  systemJacobian = &mat;

  ikSolve();

  //This tells glut to use a double-buffered window with red, green, and blue channels 
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

  // Initalize theviewport sizes
  viewport.w = 1300;
  viewport.h = 1000;

  //The size and position of the window
  glutInitWindowSize(viewport.w, viewport.h);
  glutInitWindowPosition(0, 0);
  glutCreateWindow("CS184 - IKSolver");

  initScene();                                 // quick function to set up scene

  glutDisplayFunc(draw);                       // function to run when its time to draw something
  glutReshapeFunc(myReshape);                  // function to run when the window gets resized
  glutIdleFunc(myFrameMove);                   // function to run when not handling any other task
  glutKeyboardFunc(keyboardHandle);   		   // function to read keyboard input
  glutSpecialFunc(specialKey);   		       // function to read special keyboard input - arrow keys

  glutMainLoop();                              // infinite loop that will keep drawing and resizing and whatever else
  return 0;
}








