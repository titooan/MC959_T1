#ifndef ROBOT_H
#define ROBOT_H

#define LOG_FILE    "output.txt"
#define NUM_SONARS  16
#define PI 3.14159

#include <fstream>
#include <stdio.h>
#include <math.h>

#include <Eigen/Dense>
using Eigen::MatrixXd;


extern "C" {
   #include "extApi.h"
   #include "v_repLib.h"
}
class Robot
{
public:    
    Robot(int clientID,const char* name);
    void update();
    void printPosition();
    void updateInfo();
    void writeGT();
    void writeSonars();
    void changeCoordinateToOrigin(float *localFrame, float *transformedFrame);
    void detectedPosition(simxFloat** position);
    void move(float vLeft, float vRight);
    int blockedFront();
    int blockedRight();
    int blockedLeft();
    void moveForward();
    void updatePosition();
    void moveInCircle();
    int frenteLivre();

    int clientID;
    simxInt handle;                                        // robot handle
    simxFloat velocity[2] = {1,1};                         // wheels' speed
    simxInt sonarHandle[16];                               // handle for sonars
    simxInt motorHandle[2] = {0,0};                        // [0]-> leftMotor [1]->rightMotor
    simxInt encoderHandle[2] = {0,0};
    simxFloat encoder[2] = {0,0};
    simxFloat lastEncoder[2] = {0,0};

    float angularVelocity[2] = {0,0};

    /* Robot Position  */
    simxFloat robotPosition[3] = {0,0,0};                    // current robot position
    simxFloat robotOrientation[3] = {0,0,0};                 // current robot orientation
    float initialPose[3] = {0,0,0};
    simxFloat robotLastPosition[3] = {0,0,0};                // last robot position
    float sonarReadings[NUM_SONARS];

    void multiplyMatrix();
    MatrixXd translationMatrix(int dx, int dy);
    MatrixXd rotationMatrix(double alfa);

    const float R = 0.097;      // raio da roda em cm
    const float L = 0.19;       // metade da distancia entre as rodas em cm

};

#endif // ROBOT_H
