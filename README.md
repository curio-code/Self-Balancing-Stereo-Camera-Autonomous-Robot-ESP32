
# Self-Balancing-Autonomous-Robot(SBAR)-ESP32

<p align="center">
  <img src="/Graphics/Robot_Model.gif" alt="animated" width="600" height="600" />
</p>

<h2> Theme </h2>
To provide a DIY and Budget alternative to expensive miniature autonomous vehicle platforms like AWS DeepRacer.

<h2> Methodology </h2>
<p style='text-align: justify;'> To reduce the cost, minimal 'on-robot' processing was encouraged. Heavy lifting tasks like Image Processing etc, were to be done on a remote PC. To achieve this, all the sensor data has to be wirelessly communicated to the PC, and the PC sends back the processed results to the robot. Hence ESP32 was an ideal platform, as it has inbuilt WiFi capability and can host small websites. ESP32-CAM modules with ov2460 cameras were chosen for Stereo Camera setup. And an EPS32 Development Board was selected for self-balancing functionality. To generate the odometry data, encoder discs with light-blocking sensors were used.</p>

<h2> Code Structure </h2>
1. Left_Camera <br>
&nbsp;&nbsp;&nbsp;&nbsp; -   Embedded Code for the Left Camera.<br>
2. Right_Camera <br>
&nbsp;&nbsp;&nbsp;&nbsp; -   Embedded Code for the Right Camera. <br>
3. Self_Balancing <br>
&nbsp;&nbsp;&nbsp;&nbsp; - Embedded Code for the Central ESP32 Board with Web interface for giving  commands. <br>
&nbsp;&nbsp;&nbsp;&nbsp; - Embedded Code for the Central ESP32 Board with Web interface for tuning PID parameters. <br>
4. Depth Map <br>
&nbsp;&nbsp;&nbsp;&nbsp; - Code to stream the images from both cameras and processing them to generate the disparity map. <br>
            
<h2> Hardware Implementation </h2>
<p align="center">
  <img src="/Graphics/Acutual_Photo.png" />
</p>

<h2> Result </h2>
<h3> Stereo Camera Calibration and Disparity Map Generation </h3>
  <img src="/Graphics/StereoDemo_hi.gif" alt="animated" width="1000" height="350"/>
