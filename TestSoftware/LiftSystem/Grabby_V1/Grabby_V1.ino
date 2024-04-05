void setup() {
  // put your setup code here, to run once:

}

enum ShelfFloor{FLOOR_0, FLOOR_1, FLOOR_2};

void loop() {

  //grab sequence
  platform.goTo(FLOOR_1); //add an enum type with 1st floor, 2nd floor ... and the enum have the value of the height of the platform
  platform.slider.front(): //add bool parmeter, if true the pump goes on some times to ensure the package is pulled
  platform.grabPackage(); //add bool param, if true we check if package is really grabbeb
  platform.slider.back(); 
  platform.camera.checkPackageLoaded();
  platform.goTo(FLOOR_0);


  //release sequence
  platform.goTo(FLOOR_1);
  platform.slider.front(): //add bool param that states if there is a package tu push
  platform.grabPackage(); //add bool param, if true we check if package is really grabbeb
  platform.slider.back(); //add bool param that states if there is a package tu push
  platform.camera.checkPackageLoaded();
  platform.goTo(FLOOR_0);


}


//---------------------------------------------------------------------MAIN STRUCTURE OF THE CODE IN THE ROBOT---------------------------------------------------------------
void MAIN(){
  // 0  ask the server for a task (if no task go to charge station)
  // 0.2  task (cf TASK method) given to the robot by the PC in communication with the robot per WIFI
  // 2  robot nows the warehouse, it accept the order and plans a route to first part of the task (plans the second part during the travel to the first (multi thread))
  // 3  picks up package
  // 4  goes to the second part of the task (route already planned)
  // 5  drops package
}

//----------------------------------------
void MAIN_0(){
  //communication with the server
  //reception of the task
}
void MAIN_0_2(){
  //processing the first path the robot has to take
  //gives the order to the rebot to move
  //start processing the second one
}
void MAIN_1(){
  //Path following algorithm and obstacle avoidance
}
void MAIN_2(){
  //retrieving procedure
  //IF BOX ON A SHELF
  //robot stop when at the right distance in the row (cf method LOCATION)
  //robot turn QR reader to the shelf and platform goes up to the right hight so it can read the QR code of the boxes on the shelf
  //when in front of the right box, robot turns and retrieve the box
  //platform goes down
  //ready to move again

  //IF BOX IN THE ROBOT TO HUMAN INTERFACE (HTRI)
  // TODO
}
void MAIN_3(){
  //same as MAIN_1
}
void MAIN_4(){
  //drop off procedure
  //IF DROP OFF ON SHELF
  //same as MAIN_2

  //IF DROP OFF IN HTRI
  // TODO: still need to figure out how it will look like (HTRI)

}
//----------------------------------------

void TASK(){
  //task will be something like see this box ? it is here and has to go there.
  //Task will contain: BOX ID, BOX current location, BOX destination (cf location method)
}

void LOCATION(){
  //this will be how we describe a place in the warehouse
  //Row number and QR code ID
  //floor number
  //distance in the row (QR code every meter so that the robot can recalibrate its position)

}
//----------------------------------------

