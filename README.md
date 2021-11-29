# MEngRefactored

This repository contained the code associated with my MEng Project: Fault Tolerant Coordination of Multiple Rovers For Planetary Exploration (@University of Glasgow). The project was carried out in the academic year 2020/2021 and is being refactored as of October 2021. All mathematical modelling and simulation for this project has been carried out using MATLAB.

## What's in the simulation?

### Rover Model 
A simple, 4-wheel symmetrical rover is described. This mathematical model of a rover was developed and validated at the University of Glasgow in [1]. Due to Covid-19 restrictions, no further validation could take place.The image below shows the rover used within the project, a Lynxmotion 4WD3 mobile robot. This rover has the following dimensions: height of 0.1270m, width of 0.2488m and length of 0.3500m. 

![image](https://user-images.githubusercontent.com/46682827/143875179-228c38fe-604a-473a-b634-858828c76ef4.png)


### Rover Group 
This project proposes the use of multiple rovers for a planetary exploration mission. The aim of using multiple rovers is to extend technical capabilities during the mission by increasing the overall sensor footprint, or by increasing the number of points of scientific interest which can be investigated during a limited mission lifespan. The group will be configured to either prioritise its sensor footprint or the acquisition of target points. This mission priority will affect the reconfiguration action taken in the presence of faults.  Within the rover group, there are multiple ‘child’ rovers and one ‘parent’ rover. 

The full rover system incorporates five Lynxmotion rover modelsto act as child rovers. Five child rovers have been selected in order to significantly extend the data collecting capabilities of a given mission. If one or more rovers are lost during operation, more data gathering would be carried out than during a single rover mission. A larger group of child rovers was not considered as financial and mass budgetary constraints severely limit the number of rovers which could realistically be transported to a mission location. 

The parent rover acts as a central planning unit and is designed to carry out coordination, health monitoring, and fault recovery for the rover group. It could also act as a central hub for the rover group, providing a communications link with satellite orbiters and rover mission control on Earth. 


### Simulated Environment

An environmental model was developed in MATLAB, representing the operational area of the rover group. The information provided by the environmental model enables the ability to plan paths for each rover in advance, as part of a path planning and coordination system. An area of 25m2 is used for testing, representing approximately 80X80 pixels of HiRISE terrain data. The operational area was designed to provide a sufficient path planning and traversal challenge to both individual rovers and the group. Individual rovers must follow a safe path which avoids high risk areas and hazards. The group must be able to sufficiently coordinate their paths such that rovers can pass through narrow areas without collisions. There are three layers of information within the environmental model: large obstacles/impassable areas, terrain classification, and small obstacles/hazards.

MATLAB ‘polyshapes’ are used to define large obstacles and impassable areas. These areas mimic terrain data collected by the HiRISE camera such as craters, cliffs and ditches. Path planning will not allow any traversal through these areas in order to minimise danger to the rover. 

A terrain classification map is defined using further MATLAB 'polyshapes' to identify the potential risk associated with traversing a particular segment of the map. Commonly occurring terrain classes have been selected from [2]; smooth regolith, rough regolith and steep slopes. Smooth regolith is defined as a firm and smooth surface, without any obstacles or hazards, where the rover should experience minimal slip. Similarly, rough regolith should not cause the rover to experience substantial slip. However, rough regolith terrain contains many small rocks which may require the rover to reduce its forward velocity. Rough regolith can be considered a ‘rock field’ and will be referred to as such for the remainder of this work. Steep slopes border the impassable areas, presenting a high risk for rover slip during operation. A risk assessment was carried out on the selected terrain classes based on required velocity reduction and potential for slip during path traversal (shown on the table below). Risks are rated from 1 to 5, with 5 representing the most severe potential performance degradation.


| Terrain Class   | Velocity Reduction | Slip Risk | Total Risk |
| -------------   |:------------------:| :--------:| :---------:|
| Smooth Regolith | 0                  | 1         | 1          |
| Rough Regolith  | 4                  | 1         | 5          |
| Steep Slope     | 4                  | 5         | 9          |


An array of small obstacles is defined within the MATLAB simulation. These mimic any obstacles that must be detected during operation by rovers as they were too small to be observed by the orbiter. As such, the location of the small obstacles is not available to the path planner and rovers must adjust their planned paths online. Each small obstacle is defined to have a radius of 2.5cm. 

The figure below shows the environmental model; where black regions are impassable, pink regions represent steep slopes, green regions represent a rough regolith rock field, magenta points represent small obstacles surrounded by a blue safety radius, and white areas represent smooth regolith.
 
![image](https://user-images.githubusercontent.com/46682827/143875155-9b760ca5-c55b-487b-958a-96936074e0a4.png)


## How To Use This Simulaiton
For information on how to use this simulation, please see the project [wiki](https://github.com/sarahswinton/MEngRefactored/wiki).


## Sources 
[1] Worrall K.J. Guidance and Search Algorithms For Mobile Robots: Application and Analysis Within The Context of Urban Search and Rescue [PhD Thesis]. Glasgow, Scotland. University of Glasgow. 2008

[2] Rothrock B, Kennedy R, Cunningham C, Papon J, Heverly M, and Ono M. SPOC: Deep Learning-based Terrain Classification for Mars Rover Missions. AIAA Space. 2016
