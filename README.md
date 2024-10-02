Future Engineering Journal - ADAS X



1. Getting Started

Introduction of team members:

Virika Nanda: Hello! I am Virika, studying 11th grade at Ahmedabad International School. My interest in robotics and engineering sparked ever since I got my first Lego set. I’ve always been amazed by how we could innovate something great using our minds and hands. Over the years I’ve participated in various robotics competitions such as WRO Elementary and FLL, which have taught me how we can use tech to make the world a better place. This was also my motive for participating in WRO Future Engineering, as the challenge to make a self-driving car involves solving real-life problems that can have a huge impact.

Vyom Sheth: Hello everyone. I am Vyom Sheth and I am thrilled to be participating in the WRO Future Engineering Competition. I have developed an interest in robotics over the past 3 years and have participated in Techfest by IIT Bombay which I won as the first runner-up at a national level. Over the years, I have gained valuable skills in this field like programming, construction, problem-solving, and most important:- Teamwork. I believe that participating in the WRO Future Engineering will not only challenge me but also allow me to showcase my creativity and problem-solving abilities.


2. Vision of WRO Future Engineering and its importance

Our perception of the theme and goals:

The theme for the WRO 2024 Future Engineers category is self-driving cars. In this challenge, we need to design and build autonomous vehicles equipped with electromechanical components, such as sensors and cameras. Our goal is to program these cars to navigate a track and avoid obstacles without human intervention as quickly as possible. 
We think that this is a great opportunity for students all around the world to collaborate and build a project that could contribute to a more sustainable future. We aim to explore numerous hardware and software and choose the ones best suitable for our robot. Our team name, ADAS X (Advanced Driver Assistance System) also reflects the design of the robot.

Relevance for the future:
Self-driving cars can positively impact the world by reducing carbon emissions and promoting sustainability. They will improve fuel efficiency through optimized driving patterns, decrease congestion, and lower accident rates, leading to fewer emissions from idling and traffic jams. Autonomous electric vehicles (EVs) can also help transition away from fossil fuels by integrating into smart grids and renewable energy systems. By reducing the need for individual car ownership, they can foster shared mobility solutions, decreasing the overall number of vehicles and minimizing urban sprawl.



3. Getting Started
Knowing the game field - June

In our first two classes, we deepened our understanding of the game by exploring the game field and its obstacles through online videos and the detailed resources provided on the official competition website. Our focus was not only on visualizing the layout and challenges but also on familiarizing ourselves with the specific rules governing gameplay. We paid close attention to the scoring system, which allowed us to better grasp how to strategically approach the challenge. This comprehensive review helped us form a solid foundation, ensuring that we clearly understood the objectives and scoring mechanics of the competition.

Initial ideas 
After familiarizing ourselves with the rules and scoring system, we moved on to brainstorming ideas for our robot's size and structure. We started by thinking about how the robot would need to move around the game field and deal with obstacles. This helped us decide on an ideal size that would allow it to be both stable and easy to maneuver. We also considered what components, like motors and sensors, would give the robot better control and functionality. Working together, we came up with different design ideas that would fit the competition's challenges, helping us lay the groundwork for building our robot.



4. Sessions - July/ August / September/ October

Every 1-2 weeks for these two months we explored different hardware and software and constructed basic projects with them. These included Arduino, 3d printing, raspberry pi, python, etc.

3D Printing:
We used PTC Creo to learn CAD and practiced by designing real-life objects on it. This helped us gain an understanding of the tools and commands involved.


Arduino:
Using the Arduino uno board and IDE we made projects such as LED blinking, traffic light, temperature/ humidity, and ultrasonic sensing.  


Raspberry pi:
Each project fosters creativity and encourages experimentation, making Raspberry Pi an excellent tool for building a solid foundation in technology and preparing for more advanced topics in computing and electronics. 


Python:
Using Python for basic projects taught us fundamental programming concepts such as syntax, data structures, and control flow. It enhanced problem-solving skills through practical application and debugging. Projects like building a simple game introduced us to libraries and modules, fostering familiarity with coding practices and software design. We learned about variables, functions, libraries, and many more features on various IDEs such as Pycharm that helped us understand programming. A few practice projects included a ping pong game and a calculator. 


5. Our Robot
Initial sketches and designs (August)
After learning the basics of various hardware and software, we began brainstorming about the design of our robot. We knew that we wanted it to be small and compact so it could easily maneuver itself around the game field, so the dimensions were around 20x20x20cm. 
The initial plan was to use the Arduino uno board, however, we realized that it had limited processing power and computational abilities which is why it wouldn’t be able to handle complex algorithms. The memory constraints and insufficient sensor support were also a few reasons why we chose not to use Arduino in the self-driving car. 
Surfing the internet, we came across a better replacement for Arduino Uno, which was the ESP 32 driver board. It had a faster processor and more GPIO pins, along with 520 KB of SRAM (compared to just 2 KB on the Arduino Uno). This would allow it to handle more complex programs and store large data sets for sensor readings or calculations. So we decided to use this for the car.


Raspberry pi was the best fit for the camera detection in the obstacle challenge, and the VL53L1X ultrasonic sensor for measuring distances. We planned on using a DC motor and a servo motor to operate the robot.


1st Design (3d printed):


The first design of the car was completely made using CAD in ptc creo. The base was designed according to the desired measurements, and made in a minimalistic way as we wanted the design to be simple. The back tires would be attached by a dc motor, and the front tires would be attached using a servo motor using the front tyre mechanism. For this mechanism, we designed 2 connectors that had a T shape, where one end would be connected to the base using a screw, and the other one would be connected to the beam on either side of it. This was an efficient turning mechanism for the front tires. There were 4 slots in the center of the base where we intended to mount raspberry pi and esp 32 board.
However, once it was 3d printed we realized that it was not very sturdy and wouldn't be able to support the components on it as they weren't able to fit properly. It was also difficult for the sensors to get an appropriate value since they weren’t fixed at a proper height. So we realized we wouldn't be able to use this and decided to make a second design.


2nd Design:


For the second design, the base of an existing remote control car was used, along with its tires. The existing motor was too powerful for the project, so we replaced it with our ow, and added components such as theArduinoo uno board and sensors. However it was observed that the dimensions of the car were larger than needed, and the motor was still very powerful, which is why it was difficult to control. The torque and speed were higher than required and they couldn't be changed by programming, so the design had to be modified. 


Final Design:


The final and current design of the car includes a base constructed from lego ev3 parts such as beams, screws, axles and square beams. This is the most compact and reliable design till now, as we can add or subtract parts whenever needed. The sensor height can be changed as and when required which is beneficial for the challenge. A 3d printed plate is placed on top of the base to attach all the components such as boards andbatteriesy using zip ties. 
Front three mechanism: The front trees are controlled using a servo motor to steer them. 
Back tyre mechanism: A medium motor from the ev3 parts is used to move the back tyres which have a differential gear mechanism. They are connected using the standard ev3 cable which is cut off from one end to connect the individual wires inside it to the esp32 board. Cv cc
