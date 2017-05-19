#include <QCoreApplication>
#include "robot.h"
#include "Simulator.h"
#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <valarray>
#define SIZE_AMBIENTE 1500
#define STEP 1

extern "C" {
   #include "extApi.h"
    #include "v_repLib.h"
}


void writeMatrix(std::valarray<int>* va, int num) {
    /* write data to file */
    /* file format: robotPosition[x] robotPosition[y] robotPosition[z] robotLastPosition[x] robotLastPosition[y] robotLastPosition[z]
     *              encoder[0] encoder[1] lastEncoder[0] lastEncoder[1] */
    FILE *data =  fopen("./matrix.txt", "w");
    if (data!=NULL)
    {
        for (int i=0; i<va->size()/num; i++) {
            for (int j=0; j<num; j++) {
                fprintf(data, "%d,",(*va)[i*num+j]);
            }
            fprintf(data, "\n");
        }
        fprintf(data, "\n");
        fflush(data);
        fclose(data);
      }
      else
        std::cout << "Unable to open file";
}
void printValarray (std::valarray<int>* va, int num)
{
    for (int i=0; i<va->size()/num; i++) {
        for (int j=0; j<num; j++) {
            std::cout << (*va)[i*num+j] << ' ';
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}
//indexes : {line,column}
void coordinatesToMatrixIndex(simxFloat* coord,simxInt* indexes,int step){
    int posX = static_cast<int>((coord[0]+7.5)*100);
    int posY = static_cast<int>((coord[1]+7.5)*100);
    posX = posX/step;
    posY = posY/step;
    indexes[0] = posX;
    indexes[1] = posY;
}
void updateMatrix(std::valarray<int>* ambiente,simxInt* posRobot,simxInt** indexesObstacles,int size ){

    //std::cout<< "xrobo "<<posRobot[0]<<std::endl;
    //std::cout<< "yrobo "<<posRobot[1]<<std::endl;
    //std::cout<< "numberLine "<<size<<std::endl;


    (*ambiente)[posRobot[0]*size+posRobot[1]] = 0;
    for(int i =0;i<8;i++){
        if(indexesObstacles[i][0] >0 && indexesObstacles[i][1]>0)
            (*ambiente)[indexesObstacles[i][0]*size+indexesObstacles[i][1]] = 1;
    }
}
/*
void showMatrixExample(){
    int numberLines = (SIZE_AMBIENTE/STEP);

    CImg<double> matrix(200,200, 1, 1, -1);
    //matrix.fill(255);
    for(int x=2; x<55; x++){
        for(int y=40; y<100; y++){
            matrix.atXY(x,y) = -1;
        }
    }

    for(int x=70; x<120; x++){
        for(int y=120; y<150; y++){
            matrix.atXY(x,y) = 1;
        }
    }

    for(int x=180; x<200; x++){
        for(int y=0; y<150; y++){
            matrix.atXY(x,y) = 0;
        }
    }

    matrix.display("My matrix");
    matrix.save("test.bmp");
}*/

int main(int argc, char *argv[])
{

//    MatrixXd m(2,2);
//    m(0,0) = 3;
//    m(1,0) = 2.5;
//    m(0,1) = -1;
//    m(1,1) = m(1,0) + m(0,1);
//    std::cout << "matrix :"<< std::endl << m << std::endl;

    //showMatrixExample();

    int numberLines = (SIZE_AMBIENTE/STEP);

    std::valarray<int> ambiente(-1,numberLines*numberLines);
    Robot *robot;
    Simulator *vrep = new Simulator("127.0.0.1", 25000);
    simxInt indexesRobot[2];
    simxFloat* positionAbsoluteObstacles[8];
    simxInt* indexesObstacles[8];
    for(int i=0;i<8;i++){
        positionAbsoluteObstacles[i] = new simxFloat[2];
        indexesObstacles[i] = new simxInt[2];
    }
    int id;
    if (id = vrep->connect() ==-1){
        std::cout << "Failed to Connect" << std::endl;
        return 0;
    }

    robot = new Robot(id, "Pioneer_p3dx");
    //std::cout<< ambiente.size()<<"gello"<<std::endl;
    for (int i=0; i<9000; ++i)
    {
//        std::cout<<"still running..."<<std::endl;
        //std::cout << "Here we go... " << i << std::endl;

//        std::cout<<"ok"<<std::endl;
        robot->updateInfo();

        //robot->update();
        coordinatesToMatrixIndex(robot->robotPosition,indexesRobot,STEP);
        robot->detectedPosition(positionAbsoluteObstacles);
//        robot->updatePosition();
        robot->wallFollow();
//        robot->avoidObstacles();
        for(int j=0;j<8;j++){
            coordinatesToMatrixIndex(positionAbsoluteObstacles[j],indexesObstacles[j],STEP);
           //std::cout<<"sensor"<<j<<" seen obstacle, x = "<<positionAbsoluteObstacles[j][0]<<" y = "<<positionAbsoluteObstacles[j][1]<<std::endl;
           //std::cout<<"sensor"<<j<<" index, x = "<<indexesObstacles[j][0]<<" y = "<<indexesObstacles[j][1]<<std::endl;
        }
        updateMatrix(&ambiente,indexesRobot,indexesObstacles,numberLines);

        robot->writeGT();
        robot->writeSonars();
        extApi_sleepMs(50);
    }

    printValarray(&ambiente,numberLines);

    writeMatrix(&ambiente,numberLines);
    vrep->disconnect();

    exit(0);
}
