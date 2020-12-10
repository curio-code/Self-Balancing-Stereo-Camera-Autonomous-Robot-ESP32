
# Self-Balancing-Stereo-Camera-Autonomous-Robot-ESP32

<p align="center">
  <img src="/Graphics/Robot_Model.gif" alt="animated" width="600" height="600" />
</p>
<h2> Theme </h2>
To provide a DIY and Budget alternative to expensive miniature autonomous vehicle platforms like AWS DeepRacer.
<h2> Methodology </h2>
To reduce the cost, minimal 'on-robot' proceesing was encouraged. Heavy lifting tasks like Image Processing etc was to be done by a remote PC. To achieve this all the sensor data has to be wirelessly communicated to the PC and PC sends back the processed results to robot. Hence ESP32 was ideal platform, it has inbuilt WiFi capabilty and can host small websites. ESP32-CAM modules with ov2460 cameras were choosen for Stereo Camera setup. And an EPS32 Developement Board were chosen for self balancing functionality. To generate the odometry data encoder disc with light blocking sensors was used.
<h2> Code Structure </h2>
1. Left_Camera <br>
            -   Embedded Code for the Left Camera.<br>
2. Right_Camera <br>
            -   Embedded Code for the Right Camera. <br>
3. Self_Balancing <br>
            - Embedded Code for the Central ESP32 Board with Web interface for giving  commands. <br>
            - Embedded Code for the Central ESP32 Board with Web interface for tuning PID parameters. <br>
<h2> Hardware Implementation </h2>
<p align="center">
  <img src="/Graphics/Acutual_Photo.png" />
</p>
<h2> Result </h2>
<h3> Stereo Camera Calibration and Disparity Map Generation </h3>
  <img src="/Graphics/StereoDemo_hi.gif" alt="animated" width="1000" height="350"/>
