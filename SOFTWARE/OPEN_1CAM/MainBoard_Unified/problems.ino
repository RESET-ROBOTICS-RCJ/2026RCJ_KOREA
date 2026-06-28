// find ball then kick
//  void doTest(){
//      updateSensors();
//      dribbler(40);
//      while(abs(ballAngle>10)){
//              updateSensors();

//         if(ballAngle>10){
//             move(90,30);
//         }
//         else if(ballAngle<10){
//             move(-90,30);
//         }
//     }
//     while(ballDistance>20){
//             updateSensors();

//         move(0,30);

//     }
//         while(abs(ballAngle>10)){
//             updateSensors();

//         if(ballAngle>10){
//             move(90,30);
//         }
//         else if(ballAngle<10){
//             move(-90,30);
//         }
//     }
//     moveForMs(0,30,2000);
//     while(abs(gYGOALAngle)>10|| gYGOALAngle== -1){
//         updateSensors();
//         if(gYGOALAngle>10){
//             move(90,30);
//         }
//         else if(gYGOALAngle<10){
//             move(-90,30);
//         }
//     }
//     kickBall();

// }

// two corner kick
//  void doTest(){
//      updateSensors();
//      dribbler(40);
//      moveForMs(0,30,1000,false,false,false,false);
//      moveForMs(-90,30,5000,false,false,true,false);

//     moveForMs(180,30,2000,false,false,false,false);
//     moveForMs(45,30,3000,false,false,false,false);
//     moveForMs(90,30,3000,false,false,false,true);

//     moveForMs(180,30,2000,false,false,false,false);

//     moveForMs(-45,30,3000,false,false,false,false);
//     dribbler(0);
//     moveForMs(0,30,1000);
//     kickBall();

// }

// circle
//  void doTest(){
//      int out = 0;
//      int nout = 0;
//      while(out == 0){
//          updateSensors();
//          if(abs(ballAngle)>30){
//              nout =1;
//          }
//          if(nout ==1 && abs(ballAngle)<30){
//              break;
//          }

//         move(norm180(ballAngle+90),30);
//     }
//     moveForMs(0,30,1000,false,false,false,false);
//     while (abs(ballAngle)>10){
//                 updateSensors();

//         if(ballAngle>10){
//             move(90,30);
//         }
//         else if(ballAngle<-10){
//             move(-90,30);
//         }
//     }
//         moveForMs(0,30,3000,false,false,false,false);

//     kickBall();

// }
