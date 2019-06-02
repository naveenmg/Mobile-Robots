# Mobile-Robots
### The aim of the project is to build a collection of wheeled robots that are capable of communicating to each other and mapping a maze. The advantage of the system is its capability to map the given area in the minimum amount of time by splitting the work between two or more bots. The design and implementation of multiple mapping robots is undertaken using Digital Magnetic Compass, Ultrasonic Sensor and Arduino. The designed robots uses a metric, world centric approach for mapping algorithm. Robots follow the wall while continuously sending its co-ordinates to the base station. Base station or map monitor has PC with NRF module link connected with mobile robots and map is plotted on a GUI. The proposed approach is a simple and low cost robotic application to solve SLAM problem.

## 1.	 INTRODUCTION 
     
      We all know programming just one mobile robot with artificial intelligence is hard.
      So adding more robots and having them exhibit a collective behavior can increase the difficultly level exponentially.
      This is what makes such an intelligence a hot topic in the world of robotics today. 
      The project can be dealt with under various stages each more difficult than the previous.
      Robust feature detection is one of the crucial problems of simultaneous localization and map building (SLAM) of mobile robot.
      Because the detected feature is used as a component to construct a feature-based map and also as a landmark to localize a mobile robot. 
      If failing to detect features or obtaining inaccurate information of feature position, the robot should excessively depend on the odometry data to estimate its pose. 
      However, unboundness property of the odometry data makes the estimation divergent. 
      Therefore, to perform the SLAM successfully, feature detection should give accurate feature information as much as possible.
      
      
      Role of sensing system is to detect the presence of objects and measure their positions. 
      The objects can be neighbouring robots, obstacles and target.
      For interaction of the multiple robots, communication between robots is important to carry out specific task where one robot delivers orders or updates to other robots [1], [2], [3].
      Multiple robots can be sent into an unknown building to produce a floor map [4]. 
      Research is going on in the area of different mapping techniques over the time. 
      Representing the geometry of the environment should have high accuracy [5]. 
      A metric approach of mapping is one that determines the geometric properties of the environment. 
      This representation is very useful, but is sensitive to noise. On the other hand topological approach is one that determines the relationships of locations of interest in the environment.
      World-centric mapping represents the map relative to some fixed coordinate system, while robot-centric mapping represents the map relative to the robot. [2]. 
      Simultaneous localization and mapping determines the location or pose of a robot and construct map of an unknown environment at the same time. 
      Robot path and map are both unknown. There are various related works on localization, mapping, or both (SLAM). 
      Most of this work focuses on exploring an information space of environment[6],[7],[8],[9]. 
      An autonomous mobile robot must recognize its position and find the path for itself. 
      Some of the SLAM mobile robot in the indoor environment uses digital magnetic compass and ultrasonic sensors. 
      Multiple sensor technique makes use of ultrasonic sensor, infrared sensor, laser scanner, stereo camera, and electronic compass to solve SLAM problem[10], [11],[12], [13].
      Robot need to plan a motion path through the environment to navigate without colliding with obstacles. Data obtained by ultrasonic range measurements is used to detect and avoid obstacles in environment [14], [15].  

## 2.	THE HARDWARE DESIGN
    The hardware assembly and specifications of mapping robot is explained in this section. Fig 1 shows the robot and its different modules. 
    Fig 2 shows basic architecture block diagram of the robot.
    The robot uses Arduino Mega board as central processor and other input and output devices along with communication module and power supply.
![1](https://user-images.githubusercontent.com/38221793/58768344-10832a00-859a-11e9-9c71-b6d60ee75650.png)

Fig. 1.  Mapping Robot: Actual Assembly

