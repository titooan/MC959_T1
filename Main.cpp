#include <QCoreApplication>
#include "robot.h"
#include "Simulator.h"
#include <iostream>
#include <unistd.h>
#include <valarray>
#define SIZE_AMBIENTE 15000
#define STEP 5
extern "C" {
   #include "extApi.h"
    #include "v_repLib.h"
}



//indexes : {line,column}
void coordinatesToMatrixIndex(simxFloat* coord,simxInt* indexes){
    indexes[0] = static_cast<int>((coord[0]+7.5)*100);
    indexes[1] = static_cast<int>((coord[1]+7.5)*100);
    std::cout<<indexes[0]<<","<<indexes[1]<<std::endl;
}

int main(int argc, char *argv[])
{
    int numberLines = (SIZE_AMBIENTE/STEP);

    std::valarray<int> ambiente(numberLines*numberLines);
    Robot *robot;
    Simulator *vrep = new Simulator("127.0.0.1", 25000);
    simxInt indexes[2];
    simxFloat* positiondetected[16];
    for(int i=0;i<8;i++){
        simxFloat newPos[2];
        positiondetected[i] = newPos;
    }
    int id;
    if (id = vrep->connect() ==-1){
        std::cout << "Failed to Connect" << std::endl;
        return 0;
    }

    robot = new Robot(id, "Pioneer_p3dx");

    for (int i=0; i<3000; ++i)
    {
        //std::cout << "Here we go... " << i << std::endl;
        robot->updateInfo();
        std::cout << "pos : "<< robot->robotPosition[0]<< ","<< robot->robotPosition[1]<< ","<< robot->robotPosition[2]<< std::endl;
        coordinatesToMatrixIndex(robot->robotPosition,indexes);
        std::cout<<"IGUAL "<<indexes[0]<<","<<indexes[1]<<std::endl;
        robot->detectedPosition(positiondetected);
        /*
        for(int j=0;j<8;j++){
           std::cout<<"sensor"<<j<<" seen obstacle, x = "<<positiondetected[j][0]<<" y = "<<positiondetected[j][1]<<std::endl;
        }*/
        //robot->writeGT();
        //robot->writeSonars();
        extApi_sleepMs(5000);
    }
    vrep->disconnect();
    exit(0);
}
