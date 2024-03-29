# Flight-Controller#

   At the beginning of my senior year project 1, I wanted to design an entire drone, but with the lack of men power and access to lab equipment I had to significantly trim down my expectation, so I went to making an entire drone to just a FC for pluto x quadcopter and use the pluto x RC to control the FC. But an issue occur as the BLE of the pluto x was not compatible with Arduino nano 33 BLE sense so in Feb 2021 I decide instead to build my own phone RC using the platform called ”Appinventor”. Unfortunately, the nano 33 was not compatible with the libraries used by Appinvetor. Moreover, using the pluto x was not a good option because we will have to make a circuit board meaning we need to pay attention to the weight and need access to good equipment (not option due to covid). Which by March I have move to buying a quadcopter frame and a RC for testing my FC on it. But I realize that buying a RC was way over the purpose I set for myself, that’s why by May I went ahead and had my final cut I was to design a physical RC Prototype with an Arduino nano 33 IOT. What is the limitation of my designs?  Well, the limitations for my designs are somewhat complicate and tidies work: Hardware wise, I will need to put together the frame and other component needed, this include the RC. Expectation wise, my FC will get the quadcopter to take off and hold an altitude predefined when a button is push and then at the push of a second button land. And the second expectation will have the RC and the FC to communicate via BLE. How am I going to achieve my objective?  Well As an Engineer in training, I need to organize my work well and commit to the work. The Gantt Chart is the best tools. 

  ![Screen Shot 2021-07-16 at 10 01 49 AM](https://user-images.githubusercontent.com/29171502/125960104-e78052b7-854e-49ea-8232-512342bb1d5a.png)

   This Gantt chart is a little bit offset as I made the error not saving my progress Realtime. Nonetheless, the task is the ones I followed and did. Every milestone was mark on the progress tab by a 100%. After a Gantt Chart another concept needs to be observe. That concept is call design iterations.

![Screen Shot 2021-07-16 at 10 09 37 AM](https://user-images.githubusercontent.com/29171502/125961171-b7be5ce7-9d0f-4384-908a-44bf17f8bd31.png)

   Iterative design is a process of designing a product in which the product is tested and evaluated repeatedly at different stages of design to eliminate usability flaws before the product is designed and launched. In other words, iterative design is a process of improving and polishing the design over time. This is the concept I sat on all along my project. Making this project required material and equipment.
  
  
  ![Screen Shot 2021-07-16 at 10 11 16 AM](https://user-images.githubusercontent.com/29171502/125961344-385bb74d-dfeb-4c35-95d6-1dd761adf0d2.png)


   The BOM basically list all the material and equipment needed for this project. The total price of a self-made quadcopter is $173.35. Now that we have all the material we need to start working on our FC and RC. But one question arose what part am I going to work on? The FC first or the RC? Instinctively I pick to start with the part that make sense which is the FC.


  ![Screen Shot 2021-07-16 at 10 13 03 AM](https://user-images.githubusercontent.com/29171502/125961592-5bc3c395-215e-45a5-9d9b-08b2ae50bed9.png)


   Hardware wise, the FC will be made of a couple sensors: An Ultrasonic sensor which is an electronic device that measures the distance of a target object by emitting ultrasonic sound waves and converts the reflected sound into an electrical signal. Ultrasonic waves travel faster than the speed of audible sound. The second hardware on the list for my FC is the Arduino nano 33 BLE sense which is loaded with multiple sensors the one I will be using for my project is the IMU which stand for “Inertial Measurement Unit,” and we use it to describe a collection of measurement tools. When installed in a device, these tools can capture data about the device's movement. IMUs contain sensors such as accelerometers, gyroscopes, and magnetometers. But for full disclosure we will be only use both the gyroscopes and the accelerometers. The picture above show how everything is assemble, I have integrated a colored legend for wiring. 
                    
 ![Screen Shot 2021-07-16 at 10 14 32 AM](https://user-images.githubusercontent.com/29171502/125961752-b9a0a72e-6efe-41e2-9b8b-63657f8f66d8.png)

   The Schematic is self-explanatory. Let me ask you a question, what do a drone, or a plane need to fly perfectly?  The answer is stabilization or equilibrium (just like a us human before we walk, we need equilibrium). In a vehicle that travels flat on the ground like a car or truck, or on the surface of the water like a boat, you generally only travel in 2 dimensions – straight and level. On an aircraft, spacecraft or underwater submarine, you have the added 3rd dimension of depth to deal with as well. This is where things get a little more complex and axes of rotation come into play. An axis can be thought of as a real or imaginary line about which an object, such as an aircraft, can and will rotate around. Those angles are called Pitch, Roll and Yaw.

![Screen Shot 2021-07-16 at 10 17 08 AM](https://user-images.githubusercontent.com/29171502/125962052-ba694b02-e811-4a70-92ff-333aa7cc8282.png)
     
   For the purpose of my project, I will only use both Pitch and Roll. Since the objective of our Quadcopter is to take off and then hover at a preset altitude all autonomously, I needed a way to control the two angles (pitch and roll). All closed loop control systems are designed to keep a specific device, or system of devices, operating at a pre-determined setpoint. Whether it is the thermostat on the wall that controls the temperature of your home, the cruise control in your car that keeps it traveling down the highway at a specific speed, or the gyro sensors in your multirotor that keep it level when you let go of the controls, all closed loop systems rely on PID settings to keep the system in equilibrium.
                    
![Screen Shot 2021-07-16 at 10 18 56 AM](https://user-images.githubusercontent.com/29171502/125962404-b6f29174-93ff-4f6f-8957-a9d7a47b1443.png)

   So what are PID’s? Some people refer to them as “pid” settings, said like the word “kid”, while others will refer to them as 3 individual letters, calling them “P-I-D” settings. While either one works, the letters do mean something. The letter “P” stands for “Proportional Control”, the letter “I” refers to “Integral Control” and the letter “D” stands for “Derivative Control”. Any of the readers that remember taking a calculus class in high school or college are familiar with the terms Proportional, Integral and Derivative, and when talking about PID settings, these terms do have similar meanings. To understand PID settings, we need to discuss each of these terms in brief detail in order to see how each one affects the control of our multirotor aircraft. In every closed loop control system there are three terms that must be considered: A particular variable, a desired setpoint and an error value. When talking about multirotors, the variable could be the roll angle, with a desired setpoint of 0 degrees, or altitude with a desired setpoint of 50 feet above the ground. The error value only exists if the variable is NOT at the desired setpoint. To clarify this a little more, here is an example. If you are looking at the roll axis in a multirotor, and at one instant in time, the frame is rolled 4 degrees to the right, then the variable is Roll, the desired setpoint is 0 degrees, and the error value is 4 degrees. The purpose of the control loop is to sense the 4 degrees of error in roll, and then tell the system what to do in order to put the multirotor frame back to the desired set-point, which would be 0 degrees, or level, for a hover. Figure above shows a block diagram for the operation of a typical PID closed loop control system. Starting at the left side of the diagram, we have two values that are compared against one another, namely, the desired Setpoint value and the current Output value of the system. Any difference or “error” between these two values is then fed into the 3 sections of the PID control loop for processing. In any PID control loop, the “P” term is related to the present error, the “I” term is related to the accumulation of past errors, and the “D” term is related to prediction of future errors. The corrections that are generated by each of these three sections are then added together to form a total correction value, which is finally fed into the Process section of the control loop.
    For a multirotor controller, the process section creates each of the control pulse signals that get sent out to the speed controllers, which then drive the motors in the aircraft. Finally, the props that are attached to the motors provide the corrective forces needed to move the multirotor back to the desired setpoint. The output from the gyro sensors in the multirotor controller are looped back and compared once again to the setpoint value, and the entire process starts all over. This control loop runs continuously, hundreds of times per second, trying to keep the multirotor level when no control input is made, or moving the multirotor to a new attitude whenever a control input is given.
When all three sections of the PID control loop work together, and are properly tuned with respect to one another, you get a system that responds incredibly well to the constantly changing environment that is inherent in multirotor flight. The truly amazing thing is that there are multiple PID loops in a multirotor controller, all running together at the same time! Each axis of flight requires a separate control loop that is driven by the various sensors. Pitch, Roll, Yaw, Altitude, Heading, and Position are all continuously monitored and maintained at some specific set point as directed by the internal sensors, or to new set points as commanded by the pilot. When you stop and think about it, multirotor controllers do a lot of stuff that we simply take for granted!
Now that we have a basic understanding of what each of the P-I-D terms means, and how they relate to one another, let’s look at each one in a little more detail to get a better understanding of how they actually work. As we said earlier, the P part of the PID loop stands for Proportional Control. Like the name suggests, proportional control responds proportionally to the amount of error in the system. If there is a small error in the system, then the feedback loop will generate a small force to correct it back to the desired setpoint. If there is a large error detected, then a large force will be applied to correct the system back to the desired setpoint.

![Screen Shot 2021-07-16 at 10 24 13 AM](https://user-images.githubusercontent.com/29171502/125963003-44cef6fb-2f7e-47a4-abd7-6b2e45634a8f.png)
                           
   The output of a PID controller, which is equal to the control input to the plant, is calculated in the time domain from the feedback error as follows:
First, let's look at how the PID controller works in a closed-loop system using the formula shown above. The variable (e(t)) represents the tracking error, the difference between the desired output and the actual output. This error signal (e(t)) is fed to the PID controller, and the controller computes both the derivative and the integral of this error signal with respect to time. The control signal (u(t)) to the plant is equal to the proportional gain (Kp) times the magnitude of the error plus the integral gain (Ki) times the integral of the error plus the derivative gain (Kd) times the derivative of the error. This control signal (u(t)) is fed to the plant and the new output is obtained. The new output is then fed back and compared to the reference to find the new error signal (e(t)). The controller takes this new error signal and computes an update of the control input. This process continues while the controller is in effect. The transfer function of a PID controller is found by taking the Laplace transform of Equation u(t). Below is the representation of PID in code. 

![Screen Shot 2021-07-16 at 10 27 52 AM](https://user-images.githubusercontent.com/29171502/125963444-0ff9425d-127c-4b59-9456-1fceab4510c5.png)


![Screen Shot 2021-07-16 at 10 28 40 AM](https://user-images.githubusercontent.com/29171502/125963571-948b36c3-ec66-494d-acde-cebc38c8f915.png)

   Now that I have done covering the FC part let me cover the hardware and software parts of the RC. As you can see on the sketch above it’s only made of an Arduino nano 33 IOT, 2 pull up resistor and 2 buttons. As always, I included a colored legend for wiring.  

![Screen Shot 2021-07-16 at 10 29 23 AM](https://user-images.githubusercontent.com/29171502/125963676-d694be47-2f57-47b9-b6c4-3c722e5f2a01.png)
                
![Screen Shot 2021-07-16 at 10 30 17 AM](https://user-images.githubusercontent.com/29171502/125963848-eb3847ee-cf56-4ca4-ba24-d3c7dea11284.png)

   The important part of the RC code is to create characteristic for each button you are having on your RC. The characteristic is bunch of number linked to the BLE code. Here the first 8 ascii or character represent the characteristic and the last 24 character are BLE’s.           

![Screen Shot 2021-07-16 at 10 31 02 AM](https://user-images.githubusercontent.com/29171502/125963961-e8da443f-2bef-4780-88bf-9d021957be9e.png)


![Screen Shot 2021-07-16 at 10 32 35 AM](https://user-images.githubusercontent.com/29171502/125965506-e8188161-40bf-4bce-b69f-05c4997313e3.png)


The video of the working prototype and others:
   
https://msudenver.yuja.com/V/Video?v=3328722&node=11168075&a=1629715379&preload=false


https://msudenver.yuja.com/V/Video?v=3328778&node=11168175&a=1395076051&preload=false


https://msudenver.yuja.com/V/Video?v=3328731&node=11168092&a=1565057628&preload=false


https://msudenver.yuja.com/V/Video?v=3328727&node=11168080&a=1861341087&preload=false


https://msudenver.yuja.com/V/Video?v=3328722&node=11168075&a=1629715379&preload=false


https://msudenver.yuja.com/V/Video?v=3328717&node=11168064&a=1034560910&preload=false

   
All the codes can be found in repository above.

Conclusion:
    Even though I was not able to combine both, just the altitude PID control do an excellent job holding the project together. I mean both PID’s are working individually perfectly but the combination of both offset both. The picture showed above show the final prototype of both the FC and the RC. And the video that I am about to play you see how perfect my drone hover. At the push of the button my drone will take off even though it’s not completely stable at the beginning. The landing button has issue too because my landing code does not execute, and I figure out why, but I did not have anymore time left to correct it. I need to find a way to exit the take off loop so the landing loop can be activated at the push of the other button. It is important for me to clarify that I uncover several issues when I was doing this project. There is room for improvement for this project, future senior can work to ameliorate the design and add up stuff to it. Before I close this presentation, I need to say that I underestimate this project (because doing this project alone is impossible specially when you have other upper-level classes to think about. I have done more work in the past 2 months than the entire 8 months).

