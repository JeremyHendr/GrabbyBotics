//This will be the loop function
//the robot cant be in multiple of the steps at the same time
//to determine the step it is currently in we are gonna use an enum type

void MAIN(){
  // 0  ask the server for a task (if no task go to charge station)
  // 0.2  task (cf TASK method) given to the robot by the PC in communication with the robot per WIFI
  // 2  robot nows the warehouse, it accept the order and plans a route to first part of the task (plans the second part during the travel to the first (multi thread))
  // 3  picks up package
  // 4  goes to the second part of the task (route already planned)
  // 5  drops package
}