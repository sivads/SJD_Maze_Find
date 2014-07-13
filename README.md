##SJD_Maze_Find

###Maze Navigation and Object Retrieval for ArcBotics Sparki Robot

Programming by S. Davis and J. Davis
Date: 04/2014
```
 ___           _      ___     _         _   _       
|SJ \ __ ___ _(c)___ | _ \___| |__  ___| |_(_)__ ___
| |) / _` \ V / (_-< |   / _ \ '_ \/ _ \  _| / _(_-<
|___/\__,_|\_/|_/__/ |_|_\___/_.__/\___/\__|_\__/__/
```
Copyright (c) 2014, SJ Davis Robotics (Steven E. Davis and Jonathan Davis)

This program aims to demonstrate the maze navigation and line following functionality of 
the ArcBotics Sparki robot platform using the Infrared Reflectance Sensors and the 
Ultrasonic Range Finder.

Sparki's goal is to navigate the maze and find the object a the END of the maze.  He will 
pick up the object and return it to the START of the maze.

1.  He uses all five of his Infrared Reflectance Sensors to follow the maze center black
line to ensur that he is always traveling in the center between the maze walls.
2.  He uses his Ultrasonic Range Finder to detect the maze walls and make navigation 
decisions.
3.  He uses his Infrared Reflectance Sensors to detect the START and END of the maze.
4.  He uses his front Gripper to pick up the object at the END of the maze and return it
to the START of the maze.
5.  He uses his buzzer (beeper) to indicate turns, walls, and dead ends.
6.  He uses his buzzer (beeper) to play a melody when he completes the maze.
7.  He uses his RGB LED to indicate on (green) and off (red) centerline conditions.
8.  He uses his RGB LED to indicate START and END marker detection (blue). 

Documentation, including the Maze Specifications, Program Flow, and Special Maneuvering 
algorithms is included here as PDF files.
               
This application was created as part of Science Fair project for my son's elementary
school.  The theme of the project was Robot Sensors.  We worked very hard and had lots of 
fun learning about Sparki and developing the code together!  If you like what we did or 
find this code useful for your own project, please consider donating to our education 
fund so that can continue to do projects like this and pursue higher educational goals 

E-mail/PayPal Donations: donations@sjdavisrobotics.com

BitCoin Donations: 1Hp3htmCAyiYNg52XXhoVtjB7VdkqaQDMq

**Thank you!**

Steve (dad) and Jonathan (son) Davis

 
