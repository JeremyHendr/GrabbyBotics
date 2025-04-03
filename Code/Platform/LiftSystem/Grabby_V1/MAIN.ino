//This will be the loop function
//the robot cant be in multiple of the steps at the same time
//it will execut the steps one by one in order


// 0  ask the server for a task (if no task go to charge station)
// 0.2  task (cf TASK method) given to the robot by the PC in communication with the robot per WIFI
// 2  robot nows the warehouse, it accept the order and plans a route to first part of the task (plans the second part during the travel to the first (multi thread))
// 3  picks up package
// 4  goes to the second part of the task (route already planned)
// 5  drops package

bool task_available=0;



void loop(){
  while(!task_available) {
    //ask for a task
    //if no task available, break loop and task_given=false
  }
  if (task_available){
    //execute the steps
    //at the end no more task available
    executeTask(task);
    task_available=false;
  }
  else{
    //goes to charging station and does nothing until asked by the server
    goToChargingStation();
    waitForServerRequest();
  }
}