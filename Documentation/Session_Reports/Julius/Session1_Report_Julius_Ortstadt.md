# Session 1 Report - 14.11.2023 - Julius Ortstadt

During today's session, I gathered components for the robot and started testing and modelling them.
These included:
- 3 different types of IR Sensors for the line following.
- 1 ultrasonic sensor
- 1 set of screws

I also made some sketches and wrote down some ideas. These included sketches of how the robot will implement the motors, wheels and how to manage the space. Since there isn't much of the latter, it is important that we optimize space between the motors, the sensors, Arduino, Jetson and maybe a battery.\
![Sketch 1](/Documentation/Session_Reports/Julius/Pictures/Session_1/Sketch_1.png)

Another sketch also visualized the 1st base prototype to test the motors and the line following.
![Sketch 2](/Documentation/Session_Reports/Julius/Pictures/Session_1/Sketch_2.png)

Other elements done during this session included:

- Test of different IR-sensors (single sensor and board with 5 sensors) via Arduino in order to test their functionality and to see if the black tape is working with these sensors. Module chosen for the robot: TDRCT-5000
```
/*
 * Code to test IR board with 5 sensors
 */

const int IR1 = 3;

void setup() {
  Serial.begin(9600);
  pinMode(IR1, INPUT);
}

void loop() {
  int IR_Stat = digitalRead(IR1);
  if (IR_Stat == HIGH){
    Serial.println("HIGH");
  }
  if (IR_Stat == LOW){
    Serial.println("LOW");
  }
}
```

- 3D model creation of Infra_Red Sensor board TDRCT-5000.\
![TDRCT-5000](/Documentation/Session_Reports/Julius/Pictures/Session_1/CAD_Board.png)


- 3D model creation of Motor 4844.\
![Motor-4844](/Documentation/Session_Reports/Julius/Pictures/Session_1/Motor4844.png)

- 3D model creation of motor holding bracket.\
![Bracket](/Documentation/Session_Reports/Julius/Pictures/Session_1/Motor_Bracket.png)


- Start of 3D model creation of holding system for HC-SR04 ultrasonic sensor.





