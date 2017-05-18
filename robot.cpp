#include "robot.h"
#include <iostream>
#include <Eigen/Dense>
using Eigen::MatrixXd;

MatrixXd coords(3,1);   // [x; y; 1]
MatrixXd pose(1,3);     // [x,y,teta]

MatrixXd velocity(3,1); // [Vx, Vy, w]

float sensorAngle[16] = {PI/2.0,(5.0/18.0)*PI,PI/6.0,PI/18.0,-PI/18.0,-PI/6.0,-(5.0/18.0)*PI,-PI/2.0,-PI/2.0,-(13.0/18.0)*PI,-(5.0/6.0)*PI,-(17.0/18.0)*PI,(17.0/18.0)*PI,(5.0/6.0)*PI,(13.0/18.0)*PI,PI/2.0};
int start = 1;
int objectRight, objectLeft = 0;
int turnRight = 1; // 0 pra vira esquerda e 1 pra direita
int timeInCircle = 0;
int lastTurn = 0;   // 0 se virou pra direita, 1 se virou pra esquerda
float leftVelocity, rightVelocity = 0;

float xPosOdometry = -2.97;
float yPosOdometry = -0.0976959;
float tetaOdometry = 0;
float gyroData = 0;
float tetaPlot[5] = {0,0,0,0,0};
float xPlot[5] = {-2.97,-2.97,-2.97,-2.97,-2.97};
float yPlot[5] = {-0.0976959,-0.0976959,-0.0976959,-0.0976959,-0.0976959};

//MatrixXd destination(2);
//MatrixXd power(2);          // power to apply on each motor

int stoppedCounter = 0;

const float base_speed = 1.5;



MatrixXd Robot::translationMatrix(int dx, int dy) {
    MatrixXd t(3,3);
    t << 1, 0, dx, 0, 1, dy, 0, 0, 1;
    std::cout << "translation matrix: \n" << t << std::endl;
    return t;
}

// alfa em radianos!!!
MatrixXd Robot::rotationMatrix(double alfa) {
    MatrixXd r(3,3);
    r << cos(alfa), -sin(alfa), 0, sin(alfa), cos(alfa), 0, 0, 0, 1;
    std::cout << "rotation matrix:\n" << r << std::endl;
    return r;
}




Robot::Robot(int clientID, const char* name) {
    pose  << 0, 0, 0;
    coords << 0, 0, 1;
//    std::cout << pose << std::endl << coords << std::endl;

    translationMatrix(-2,4);
    rotationMatrix(-1.57);

    FILE *data =  fopen("gt.txt", "wt");
    if (data!=NULL)
        fclose(data);

    data =  fopen("sonar.txt", "wt");
    if (data!=NULL)
        fclose(data);

    this->clientID = clientID;
    simxGetObjectHandle(clientID, name, &handle, simx_opmode_blocking);


    /* Get handles of sensors and actuators */
    simxGetObjectHandle(clientID, "Pioneer_p3dx_leftWheel",&encoderHandle[0], simx_opmode_oneshot_wait);
    simxGetObjectHandle(clientID, "Pioneer_p3dx_rightWheel",&encoderHandle[1], simx_opmode_oneshot_wait);
    std::cout << encoderHandle[0] << std::endl;
    std::cout << encoderHandle[1] << std::endl;

    /* Get handles of sensors and actuators */
    simxGetObjectHandle(clientID, "Pioneer_p3dx_leftMotor",&motorHandle[0], simx_opmode_oneshot_wait);
    simxGetObjectHandle(clientID, "Pioneer_p3dx_rightMotor",&motorHandle[1], simx_opmode_oneshot_wait);
    std::cout << motorHandle[0] << std::endl;
    std::cout << motorHandle[1] << std::endl;

    simxChar sensorName[31];
    /* Connect to sonar sensors. Requires a handle per sensor. Sensor name: Pioneer_p3dx_ultrasonicSensorX, where
     * X is the sensor number, from 1 - 16 */
    for(int i = 0; i < 16; i++)
    {
        sprintf(sensorName,"%s%d","Pioneer_p3dx_ultrasonicSensor",i+1);
        if (simxGetObjectHandle(clientID, sensorName, &sonarHandle[i], simx_opmode_oneshot_wait) != simx_return_ok)
            std::cout << "Error on connecting to sensor " + i+1;
        else
        {
            std::cout << "Connected to sensor\n";
            /* Reads current value */
            simxReadProximitySensor(clientID,sonarHandle[i],NULL,NULL,NULL,NULL,simx_opmode_streaming);
        }
    }
    /* Get the robot current absolute position */
    simxGetObjectPosition(clientID,handle,-1,robotPosition,simx_opmode_streaming);
    simxGetObjectOrientation(clientID,handle,-1,robotOrientation,simx_opmode_streaming);
    initialPose[0]=robotPosition[0];
    initialPose[1]=robotPosition[1];
    initialPose[2]=robotOrientation[2];
    for(int i = 0; i < 3; i++)
    {
        robotLastPosition[i] = robotPosition[i];
    }
    /* Get the encoder data */
    /*simxGetJointPosition(clientID,motorHandle[0], &encoder[0],simx_opmode_streaming);  // left
    simxGetJointPosition(clientID,motorHandle[1], &encoder[1],simx_opmode_streaming);  // right
    std::cout << encoder[0] << std::endl;
    std::cout << encoder[1] << std::endl;*/

}

bool Robot::frenteLivre() {
    return ((sonarReadings[3]==-1 || sonarReadings[3]>0.75) && (sonarReadings[4]==-1 || sonarReadings[4]>0.75));
}

bool Robot::esquerdaLivre() {
    return (sonarReadings[0]==-1 && sonarReadings[15]==-1);
}

bool Robot::direitaLivre() {
    return (sonarReadings[7]==-1 && sonarReadings[8]==-1);
}

void Robot::detectedPosition(simxFloat** position){
    int i =0;
    simxFloat posX,posY,angle,robotRay;
    robotRay = 0.21;
    angle = robotOrientation[2];
    //std::cout<<"positionRobot"<<i<<" values, x = "<<robotPosition[0]<<" y = "<<robotPosition[1]<<" gamma = "<<angle<<" bot ray"<<robotRay<<std::endl;
    for(i;i<8;i++){
        //std::cout<<"sensor"<<i<<" values, sonar = "<<sonarReadings[i]<<" sensorAngle = "<<sensorAngle[i]<<std::endl;
        if(sonarReadings[i]==-1){
            posX = -5000;
            posY = -5000;
        }else{
            posX = robotPosition[0]+(sonarReadings[i]+robotRay)*cos(angle+sensorAngle[i]);
            posY = robotPosition[1]+(sonarReadings[i]+robotRay)*sin(angle+sensorAngle[i]);
        }
        position[i][0] = posX;
        position[i][1] = posY;
        //std::cout<<"sensor"<<i<<" seen obstacle, x = "<<posX<<" y = "<<posY<<std::endl;
    }
}


void Robot::updateInfo() {
//    std::cout << "leftMotor: " << motorHandle[0] << std::endl;
//    std::cout << "joint: " << encoder[0] << std::endl;
//    std::cout << "rightMotor: " << motorHandle[1] << std::endl;
//    std::cout << "joint: " << encoder[1] << std::endl;
    /* Update sonars */
    for(int i = 0; i < NUM_SONARS; i++)
    {
        simxUChar state;       // sensor state
        simxFloat coord[3];    // detected point coordinates [only z matters]

        /* simx_opmode_streaming -> Non-blocking mode */

        /* read the proximity sensor
         * detectionState: pointer to the state of detection (0=nothing detected)
         * detectedPoint: pointer to the coordinates of the detected point (relative to the frame of reference of the sensor) */
        if (simxReadProximitySensor(clientID,sonarHandle[i],&state,coord,NULL,NULL,simx_opmode_buffer)==simx_return_ok)
        {
            if(state > 0)
                sonarReadings[i] = coord[2];
            else
                sonarReadings[i] = -1;
        }
    }
    for(int i = 0; i < 3; i++)
    {
        robotLastPosition[i] = robotPosition[i];
    }
    /*std::cout << robotPosition[0] << "\t";
    std::cout << robotPosition[1] << "\t";
    std::cout << robotPosition[2] << std::endl;*/

    /* Get the robot current position and orientation */
    simxGetObjectPosition(clientID,handle,-1,robotPosition,simx_opmode_streaming);
    simxGetObjectOrientation(clientID,handle,-1,robotOrientation,simx_opmode_streaming);



    /* Get the encoder data */
    simxGetJointPosition(clientID,motorHandle[0], &encoder[0],simx_opmode_streaming);
//        std::cout << "ok left enconder = "<< encoder[0] << std::endl;  // left
    simxGetJointPosition(clientID,motorHandle[1], &encoder[1],simx_opmode_streaming);
//        std::cout << "ok right enconder  = "<< encoder[1] << std::endl;  // right

    updateOdometry();

    lastEncoder[0] = encoder[0];
    lastEncoder[1] = encoder[1];
}

//void Robot::calculateDest() {

//}


void Robot::avoidObstacles() {
    std::cout << "avoid!!" << std::endl;
    float pLeft,pRight;
    if (frenteLivre()) {
        pLeft = 3;
        pRight = 3;
//        destX = robotPosition[0] + 2*cos(robotOrientation[2]);
//        destY = robotPosition[1] + 2*sin(robotOrientation[2]);
//        proportionalController();
    } else /*if (lastTurn)*/ {
        if (direitaLivre()){
            pLeft = 1;
            pRight = -1;
            lastTurn = 0;
        } /*else if (esquerdaLivre()){
            pLeft = -1;
            pRight = 1;
            lastTurn = 1;
        }*/ else {
            pLeft = 1;
            pRight = -1;
            lastTurn = 1;
        }
//    } else {
//        if (esquerdaLivre()){
//            pLeft = -1;
//            pRight = 1;
//            lastTurn = 1;
////        } else if (direitaLivre()){
////            pLeft = 1;
////            pRight = -1;
////            lastTurn = 0;
//        } else {
//            pLeft = 1;
//            pRight = -1;
//            lastTurn = 0;
//        }
    }

    move(pLeft,pRight);
}


//void Robot::proportionalController() {
//    float Kp = 0.5;
//    float error[2] = {destX-robotPosition[0], destY-robotPosition[1]};
//    power[2] = {Kp*error[0], Kp*error[1]};
//}

void Robot::updateOdometry() {
    float dTeta;
    float tetaVelocity;
    float curveRadius;
    float deltaTeta;
    float dS;
    float dX;
    float dY;
    float dXPlot[5];
    float dYPlot[5];

    //*****************************************************
    // calculo velocidade angular em cada roda
    if (leftVelocity >= 0)
        angularVelocity[0] = (encoder[0]>=lastEncoder[0] ? encoder[0]-lastEncoder[0] : 2*PI-lastEncoder[0]+encoder[0]);
    else
        angularVelocity[0] = (encoder[0]<=lastEncoder[0] ? encoder[0]-lastEncoder[0] : encoder[0]-lastEncoder[0]-2*PI);
    if (abs(angularVelocity[0]) > 2*PI-1)
        angularVelocity[0] = 0;


    if (rightVelocity >= 0)
        angularVelocity[1] = (encoder[1]>=lastEncoder[1] ? encoder[1]-lastEncoder[1] : 2*PI-lastEncoder[1]+encoder[1]);
    else
        angularVelocity[1] = (encoder[1]<=lastEncoder[1] ? encoder[1]-lastEncoder[1] : encoder[1]-lastEncoder[1]-2*PI);
    if (abs(angularVelocity[1]) > 2*PI-1)
        angularVelocity[1] = 0;
    //*****************************************************


    leftVelocity = angularVelocity[0]*R;
    rightVelocity = angularVelocity[1]*R;

//        curveRadius = (rightVelocity != leftVelocity ? L*(rightVelocity+leftVelocity)/(rightVelocity-leftVelocity) : 0) ;
//        tetaVelocity = curveRadius*(angularVelocity[1]-angularVelocity[0])/(2*L);
//        dTeta = tetaVelocity*0.05;
//    dX = (angularVelocity[0]+angularVelocity[1])*R/2;
//        dS = 0.05*((angularVelocity[0]+angularVelocity[1])*curveRadius)/2;

    dTeta = (rightVelocity-leftVelocity)/L;

    simxGetFloatSignal(clientID,"gyroZ",&gyroData,simx_opmode_streaming);
//    std::cout << "gyroData = " << gyroData << "  " << gyroData*0.05 << " // dTeta (rodas) = " << dTeta << std::endl;

    dS = (leftVelocity+rightVelocity)/2;

    for (int i=0; i<5; ++i) {
        tetaPlot[i] += ((1+i)*gyroData*0.05 + dTeta)/(2+i);
        xPlot[i] += dS*cos(tetaPlot[i]+(((1+i)*gyroData*0.05 + dTeta)/(2+i))/2);
        yPlot[i] += dS*sin(tetaPlot[i]+(((1+i)*gyroData*0.05 + dTeta)/(2+i))/2);
    }

//    deltaTeta = (leftVelocity*rightVelocity > 0 ? (4*dTeta+gyroData*0.05)/5 : (gyroData*0.05 + dTeta)/2);//(2.5*gyroData*0.05 + dTeta)/3.5;
    deltaTeta = (9*dTeta+gyroData*0.05)/10;

    tetaOdometry += deltaTeta;
    if (tetaOdometry > PI)
        tetaOdometry = -PI+(tetaOdometry-PI);
    if (tetaOdometry < -PI)
        tetaOdometry = PI-(tetaOdometry+PI);

    dX = dS*cos(tetaOdometry+deltaTeta/2);
    dY = dS*sin(tetaOdometry+deltaTeta/2);

    xPosOdometry += dX;
    yPosOdometry += dY;

//    std::cout << " odometry           -->   [" << xPosOdometry << "," << yPosOdometry << "," << tetaOdometry << "]"  << std::endl;
//    std::cout << " groundTruth [x,y,teta] = [" << robotPosition[0] << "," << robotPosition[1] << "," << robotOrientation[2] << "]" << std::endl;
}



void Robot::update() {


//    float vRight,vLeft;
//    if (blockedFront()) {
////        std::cout << "--> FRENTE BLOQUEADA"<< std::endl;
//        vRight = 0.5;
//        vLeft = -0.5;
//    } else {
////        std::cout << "--> FRENTE LIVRE"<< std::endl;
//        vRight = 3;
//        vLeft = 3;
//    }

////    move(20,-10);
//    move(vRight,vLeft);


//    simxSetJointTargetVelocity(clientID, motorHandle[0], velocity, simx_opmode_streaming);
//    simxSetJointTargetVelocity(clientID, motorHandle[1], velocity, simx_opmode_streaming);

    /* Update sonars */
    for(int i = 0; i < NUM_SONARS; i++)
    {
        simxUChar state;       // sensor state
        simxFloat coord[3];    // detected point coordinates [only z matters]

        /* simx_opmode_streaming -> Non-blocking mode */

        /* read the proximity sensor
         * detectionState: pointer to the state of detection (0=nothing detected)
         * detectedPoint: pointer to the coordinates of the detected point (relative to the frame of reference of the sensor) */
        if (simxReadProximitySensor(clientID,sonarHandle[i],&state,coord,NULL,NULL,simx_opmode_buffer)==simx_return_ok)
        {
            if(state > 0)
                sonarReadings[i] = coord[2];
            else
                sonarReadings[i] = -1;
        }
    }
    for(int i = 0; i < 3; i++)
    {
        robotLastPosition[i] = robotPosition[i];
    }
    std::cout << robotPosition[0] << "\t";
    std::cout << robotPosition[1] << "\t";
    std::cout << robotPosition[2] << std::endl;

    /* Get the robot current position and orientation */
    simxGetObjectPosition(clientID,handle,-1,robotPosition,simx_opmode_streaming);
    simxGetObjectOrientation(clientID,handle,-1,robotOrientation,simx_opmode_streaming);

    lastEncoder[0] = encoder[0];
    lastEncoder[1] = encoder[1];

    /* Get the encoder data */
    if (simxGetJointPosition(clientID,motorHandle[0], &encoder[0],simx_opmode_streaming) == simx_return_ok)
        std::cout << "ok left enconder"<< encoder[0] << std::endl;  // left
    if (simxGetJointPosition(clientID,motorHandle[1], &encoder[1],simx_opmode_streaming) == simx_return_ok)
        std::cout << "ok right enconder"<< encoder[1] << std::endl;  // right



}

bool Robot::obstacleFront(){
    return
            sonarReadings[4] != -1 && sonarReadings[4] < 0.4
            ||
            sonarReadings[3] != -1 && sonarReadings[3] < 0.4;
}

int Robot::blockedFront() {
    return ((sonarReadings[2]!=-1 && sonarReadings[2]<0.3) || sonarReadings[3]!=-1 || sonarReadings[4]!=-1 || (sonarReadings[5]!=-1 && sonarReadings[5]<0.3));
}

int Robot::blockedRight() {
    turnRight = 1;
    return ((sonarReadings[6]!=-1 && sonarReadings[6]<0.3) || sonarReadings[7]!=-1 || sonarReadings[8]!=-1 || (sonarReadings[9]!=-1 && sonarReadings[9]<0.3));
}

int Robot::blockedLeft() {
    turnRight = 0;
    return ((sonarReadings[14]!=-1 && sonarReadings[14]<0.3) || sonarReadings[15]!=-1 || sonarReadings[0]!=-1 || (sonarReadings[1]!=-1 && sonarReadings[1]<0.3));
}

//int Robot::blockedLeft() {
//    return ((sonarReadings[15]!=-1 && sonarReadings[15]<0.75) || (sonarReadings[0]!=-1 && sonarReadings[0]<0.75));
//}

void Robot::pid(){
    if (!frenteLivre() || (esquerdaLivre() && direitaLivre()))
        avoidObstacles();
    else {
        if (!direitaLivre())
            pid(sonarReadings[8], sonarReadings[7]);
        else
            pid(sonarReadings[0], sonarReadings[15]);
    }
}

float Robot::pid(float distance1, float distance2){
    //if no wall detected, move forward until find a wall
    //when a wall is close, adapt individual wheels speed to start following it
    //if there is an obstacle in front of me, find a way to avoid it
    std::cout << "PID" << std::endl;
    if(distance1 == -1 && distance2 == -1 ){
        error_i = 0;
        last_error = 0;
        std::cout << "PID - NO WALL" << std::endl;
        move(base_speed, base_speed);
        return base_speed;
    }

    const float minAcceptedDiff = 0.005;
    const float maxDistToWall = 1;
    const float infiniteDistance = 3;
    float rightwheelSpeed = base_speed ;
    float leftWheelSpeed;

    float diff = std::abs( distance1 - distance2 );

    if(diff >  minAcceptedDiff ){
        std::cout << "PID - NORMAL" << std::endl;

        if(distance1 == -1) distance1 = infiniteDistance;
        if(distance2 == -1) distance2 = infiniteDistance;

        float Kp = 0.2;
         float Ki = 0.5;
         float Kd = 0.8;

        float error = distance2 - distance1;  //error is difference between two side sensors
        float diff_error = error - last_error;
        error_i += error;

        leftWheelSpeed = base_speed + Kp * error + Ki * error_i + Kd * diff_error;

        leftWheelSpeed = std::min(leftWheelSpeed, 2.3f);

        last_error = error;

       // rightwheelSpeed = base_speed; //Keep right wheel speed constant

    } else if( distance1 >  maxDistToWall) {
        std::cout << "PID - BIG DISTANCE TO WALL" << std::endl;
        leftWheelSpeed = base_speed ;
    } else {
        std::cout << "PID - ELSE" << std::endl;
        leftWheelSpeed = base_speed;
    }

    move(leftWheelSpeed, rightwheelSpeed);
    return leftWheelSpeed;

}


float Robot::pid2(float distance1, float distance2){
    //if no wall detected, move forward until find a wall
    //when a wall is close, adapt individual wheels speed to start following it
    //if there is an obstacle in front of me, find a way to avoid it

    if(stoppedCounter > 5){
        if(stoppedCounter == 6)
            stoppedCounter += 51;
        else
            stoppedCounter -=2;
        move(-1, -1);
        return -1;
    }


    //first test if there is something in front of me
    if(false && obstacleFront()){
        float distanceFromObstacle = std::min(std::abs(sonarReadings[4]), std::abs(sonarReadings[4] ) );
        float rightWheelSpeed = base_speed;
        float leftWheelSpeed = base_speed;

        if(distanceFromObstacle < 0.1){
            stoppedCounter ++;
        }

        if(distanceFromObstacle < 0.25){
            rightWheelSpeed = 3;
            leftWheelSpeed = 0;
        }

        float coef = 4*distanceFromObstacle;
        leftWheelSpeed = 1/coef* base_speed;
        move(leftWheelSpeed, rightWheelSpeed);
        return coef*base_speed;
        /*
        if(std::min(distance1, distance2) < 0.05){
            move(0.5* base_speed, base_speed);
            return 0.5*base_speed;
        } else {
            move(1.5*base_speed, base_speed);
            return 1.5*base_speed;
        }*/
    }

    if(distance1 == -1 && distance2 == -1 ){
        move(2*base_speed, 2*base_speed);
        return 2*base_speed;
    }

    const float minAcceptedDiff = 0.005;
    const float maxDistToWall = 0.0;
    const float infiniteDistance = 3;
    float rightwheelSpeed = base_speed ;
    float leftWheelSpeed;


    float diff = std::abs( distance1 - distance2 );

    if(diff >  minAcceptedDiff && std::min(distance2 , distance1) < 4*maxDistToWall){

        if(distance1 == -1) distance1 = infiniteDistance;
        if(distance2 == -1) distance2 = infiniteDistance;

        float Kp = 4;

        float error = distance2 - distance1;  //error is difference between two side sensors
        leftWheelSpeed = base_speed + Kp * error;

       // rightwheelSpeed = base_speed; //Keep right wheel speed constant

    } else if( distance1 >  maxDistToWall) {
        leftWheelSpeed = base_speed + distance1 ;
    } else {
        leftWheelSpeed = base_speed;
    }

    move(leftWheelSpeed, rightwheelSpeed);
    return leftWheelSpeed;

}

void Robot::moveForward() {
    timeInCircle = 0;
    float vRight,vLeft;
    if (blockedFront()) {
//        std::cout << "--> FRENTE BLOQUEADA"<< std::endl;
        vRight = -1;
        vLeft = 1;
    } else {
//        std::cout << "--> FRENTE LIVRE"<< std::endl;
        vRight = 2;
        vLeft = 2;
    }
    move(vLeft,vRight);
}

void Robot::moveInCircle() {
    float vRight,vLeft;
    if (blockedFront()) {
//        std::cout << "--> CIRCULANDO FRENTE BLOQUEADA"<< std::endl;
        vRight = -2;
        vLeft = 2;
    } else {
//        std::cout << "--> CIRCULANDO FRENTE LIVRE"<< std::endl;
        if (turnRight) {
//            std::cout << "--> CIRCULANDO --> turnRight"<< std::endl;
            vRight = 1.5;
            vLeft = 3;
        } else {
//            std::cout << "--> CIRCULANDO --> turn left!"<< std::endl;
            vRight = 3;
            vLeft = 1.5;
        }

        timeInCircle++;
    }
    move(vLeft,vRight);
}


void Robot::updatePosition() {
    objectRight = blockedRight();
    objectLeft = blockedLeft();

    if (start) {
//        std::cout << "--> TO PROCURANDOO A PAREDE..."<< std::endl;
        moveForward();
        if (objectRight || objectLeft)
            start = 0;
    } else {
        if (objectRight || objectLeft) {
//            std::cout << "--> Paredinha aqui do lado..."<< std::endl;
            moveForward();
        } else {
//            std::cout << "--> CADE A PAREDE??"<< std::endl;
            moveInCircle();
            if (timeInCircle > 2000)
                start = 1;
        }
    }

}

void Robot::writeGT() {
    /* write data to file */
    /* file format: robotPosition[x] robotPosition[y] robotPosition[z] robotLastPosition[x] robotLastPosition[y] robotLastPosition[z]
     *              encoder[0] encoder[1] lastEncoder[0] lastEncoder[1] */
    float posX;
    float posY;
    float robotX;
    float robotY;
    FILE *data =  fopen("gt.txt", "at");
    if (data!=NULL)
    {
        for (int i=0; i<3; ++i)
            fprintf(data, "%.2f\t",robotPosition[i]);
        for (int i=0; i<3; ++i)
            fprintf(data, "%.2f\t",robotLastPosition[i]);
        for (int i=0; i<3; ++i)
            fprintf(data, "%.2f\t",robotOrientation[i]);
        for (int i=0; i<2; ++i)
            fprintf(data, "%.2f\t",encoder[i]);
        for (int i=0; i<2; ++i)
            fprintf(data, "%.2f\t",lastEncoder[i]);

        fprintf(data, "%.2f\t",xPosOdometry);
        fprintf(data, "%.2f\t",yPosOdometry);
        fprintf(data, "%.2f\t",tetaOdometry);

        for (int i=0; i<5; ++i) {
            fprintf(data, "%.2f\t",xPlot[i]);
            fprintf(data, "%.2f\t",yPlot[i]);
        }

        for (int i=0; i<NUM_SONARS; ++i) {
            robotX = xPosOdometry;      // robotPosition[0] ou xPosOdometry
            robotY = yPosOdometry;      // robotPosition[1] ou yPosOdometry
            posX = (sonarReadings[i] > 0.2 && sonarReadings[i] < 0.95 ? robotX+(sonarReadings[i]+0.2)*cos(robotOrientation[2]+sensorAngle[i]) : 4);      // obstacle X
            fprintf(data, "%.2f\t",posX);
            posY = (sonarReadings[i] > 0.2 && sonarReadings[i] < 0.95 ? robotPosition[1]+(sonarReadings[i]+0.2)*sin(robotOrientation[2]+sensorAngle[i]) : 4);      // obstacle Y
            fprintf(data, "%.2f\t",posY);
//            if (i == 9)
//                std::cout << "r = " << robotX << "," << robotPosition[1] << "\t s = " << posX << "," << posY << " angle = " << sensorAngle[i] << " sin = " << sin(robotOrientation[2]+sensorAngle[i]) << std::endl;
        }

        fprintf(data, "\n");
        fflush(data);
        fclose(data);
      }
      else
        std::cout << "Unable to open file";
}



void Robot::writeSonars() {
    /* write data to file */
    /* file format: robotPosition[x] robotPosition[y] robotPosition[z] robotLastPosition[x] robotLastPosition[y] robotLastPosition[z]
     *              encoder[0] encoder[1] lastEncoder[0] lastEncoder[1] */
    FILE *data =  fopen("sonar.txt", "at");
    if (data!=NULL)
    {
        if (data!=NULL)
        {
            for (int i=0; i<NUM_SONARS; ++i) {
                fprintf(data, "%.2f\t",sonarReadings[i]);
            }
            fprintf(data, "\n");
            std::cout << std::endl;
            fflush(data);
            fclose(data);
        }
    }
}

void Robot::printPosition() {
  simxFloat position[3];
  //simxGetObjectPosition(_clientID, _handle, -1, position, simx_opmode_streaming);
  //std::cout << "[" << position[0] << ", " << position[1] << ", " << position[2] << "]" << std::endl;
}

void Robot::move(float vLeft, float vRight) {
    simxSetJointTargetVelocity(clientID, motorHandle[0], vLeft, simx_opmode_streaming);
    simxSetJointTargetVelocity(clientID, motorHandle[1], vRight, simx_opmode_streaming);
//    std::cout << "*****> vLeft = " << vLeft << " angularLeft = " << angularVelocity[0] << " leftVelocity = " << leftVelocity << std::endl;
}


void Robot::changeCoordinateToOrigin(float *localFrame, float *transformedFrame) {

}


//void Robot::multiplyMatrix() {
//    int i,j,k;
//    for(i = 0; i < rowsA; ++i)
//        for(j = 0; j < colsA; ++j)
//        {
////            if (i==j)
//                a[i][j] = 1;
////            else
////                a[i][j] = 0;
//        }

//    for(i = 0; i < rowsB; ++i)
//        for(j = 0; j < colsB; ++j)
//        {
//            b[i][j] = 1;
//        }

//    for(i = 0; i < rowsA; ++i)
//        for(j = 0; j < colsB; ++j)
//        {
//            mult[i][j] = 0;
//        }

//// Multiplying matrix a and b and storing in array mult.
//    for(i = 0; i < rowsA; ++i) {
//        for(j = 0; j < colsB; ++j) {
//            for(k = 0; k < colsA; ++k)
//            {
//                mult[i][j] += a[i][k] * b[k][j];
//            }
//            std::cout << "[" << mult[i][j] << "]   ";
//        }
//        std::cout << std::endl;
//    }
//}

//Acelerômetro
//simxReadForceSensor(clientID,Acelerometro,NULL,Forca,NULL,simx_opmode_streaming);
//simxGetObjectFloatParameter(clientID,MassaAcelerometro,3005,MassaSensor,simx_opmode_streaming);
//for(int i=0;i<3;i++)
//  Acel[i]=Forca[i]/MassaSensor[0];
/***************************************************************
 * This method rotates the robot to a global coordinate system
 ***************************************************************/
/*void Robot::changeCoordinatesToOrigin(float* datax, float* datay, float theta0)
{
    //The best estimate of the robot angle is simply theta = thetaold + deltatheta.
    //X and Y coordinates must be rotated based on this angle.

    //Floats that represent the robot start position, whith respect to the origin
    float x0,y0; //,theta0;
    float RoLoc, ThetaLoc, RoRot, ThetaRot, RoTrans, XRot, YRot, XTrans, YTrans;

    // Get original positions of robots
    x0 = startPosition[0];
    y0 = startPosition[1];
    //theta0 = startDirectionZ.ToDouble();
    //theta0 = theta0 - *deltatheta;

    //Calculate point in polar coordinates
    RoLoc = sqrt(pow(*datax,2)+pow(*datay,2));
    if ( (*datax==0) && (*datay==0)) {
        ThetaLoc = 0;
    }else if ((*datay == 0) && (*datax>0)) {
        ThetaLoc = 0;
    } else if ((*datay==0) && (*datax<0)) {
        ThetaLoc = 3.1415;
    } else if ((*datax==0) && (*datay>0)) {
        ThetaLoc = 3.1415/2;
    } else if ((*datax==0) && (*datay<0)) {
        ThetaLoc = -3.1415/2;
    } else {
        ThetaLoc = atan(*datax/ *datay);
        // There are many angles with the same atan.
        // Some corrections must be made.
        if ((*datax<0) && (*datay<0)) {
                ThetaLoc = ThetaLoc + 3.1415;
        }
        if ((*datax>0) && (*datay<0)) {
                ThetaLoc = ThetaLoc + 3.1415;
        }
    }


    //Calculate Rotation of this reference system
    ThetaRot = ThetaLoc+theta0;
    RoRot = RoLoc;
    XRot = RoRot*sin(ThetaRot);
    YRot = RoRot*cos(ThetaRot);

    //Claculate Translation of this reference system
    XTrans = XRot + x0;
    YTrans = YRot + y0;

    *datax = XTrans;
    *datay = YTrans;
}
*/
/*
void Robot::rotate(float xPGlobal, float yPGlobal, float zPGlobal, float xDGlobal, float yDGlobal, float zDGlobal)
{
    // Floats that represent the robot start position
    float xP,yP,zP,xD,yD,zD;

    // R1: robot slave (must be rotated)
    xP = startPosition[0];
    yP = startPosition[1];
    zP = startPosition[2];

    xD = startOrientation[0];
    yD = startOrientation[1];
    zD = startOrientation[2];

    // R = [x0, y0, z0, pitch0, roll0, yaw0];
    float deltax = xP - xPGlobal;
    float deltay = yP - yPGlobal;
    float deltaz = zP - zPGlobal;

    // Pitch - y-axis
    float deltapitch = yD - yDGlobal;
    // Roll - x-axis
    float deltaroll = xD - xDGlobal;
    // Yaw - z-axis
    float deltayaw = zD - zDGlobal;

    // R1: new coordinates
    xP = (xP*cos(deltapitch)+(yP*sin(deltaroll)+zP*cos(deltaroll))*sin(deltapitch))*cos(deltayaw)-(yP*cos(deltaroll)-zP*sin(deltaroll))*sin(deltayaw);
    yP = (xP*cos(deltapitch)+(yP*sin(deltaroll)+zP*cos(deltaroll))*sin(deltapitch))*sin(deltayaw)+(yP*cos(deltaroll)-zP*sin(deltaroll))*cos(deltayaw);
    zP = -xP*sin(deltapitch)+(yP*sin(deltaroll)+zP*cos(deltaroll))*cos(deltapitch);

    xD = deltaroll;
    yD = deltapitch;
    zD = deltayaw;

    // Checar aqui a transformação para String
    startPosition[0] = xP;
    startPosition[1] = yP;
    startPosition[2] = zP;

  //  startDirectionX = xD;
  //  startDirectionY = yD;
  //  startDirectionZ = zD;
}*/
