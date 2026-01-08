<h1>Self-Balancing Robot Using Arduino</h1>

<p>
This project demonstrates a <b>self-balancing two-wheeled robot</b> using an
<b>Arduino Uno</b> and a <b>PID control algorithm</b>.
The robot maintains balance by detecting its tilt angle using an
<b>MPU6050 Gyroscope/Accelerometer</b> and adjusting motor speed via an
<b>L298N Motor Driver</b>.
</p>

<hr>

<h2>Project Objective</h2>

<p>
To build a robot that autonomously maintains balance on two wheels by
continuously correcting its tilt using real-time sensor feedback.
</p>

<hr>

<h2>Hardware Components</h2>

<ul>
  <li><b>Arduino Uno</b> – Main microcontroller</li>
  <li><b>MPU6050</b> – Gyroscope & accelerometer (tilt sensing)</li>
  <li><b>L298N Motor Driver</b> – Motor speed and direction control</li>
  <li><b>2 × DC Geared Motors</b> – Wheel actuation</li>
  <li><b>7.4V Li-ion Battery</b> – Power supply</li>
  <li><b>Wheels & Chassis</b> – Mechanical structure</li>
</ul>

<hr>

<h2>Control System Overview</h2>

<h3>MPU6050 Sensor</h3>
<p>
The MPU6050 continuously measures the robot’s orientation and provides
<b>Yaw, Pitch, and Roll</b> data.  
The <b>pitch angle</b> is used as the primary input for balancing.
</p>

<h3>PID Controller</h3>
<p>
A <b>PID (Proportional–Integral–Derivative)</b> controller computes motor
corrections to maintain the upright position.
</p>

<ul>
  <li><b>Kp (21)</b> – Corrects current tilt</li>
  <li><b>Ki (140)</b> – Eliminates accumulated error</li>
  <li><b>Kd (0.8)</b> – Smooths rapid movements</li>
</ul>

<p>
<b>Setpoint:</b> 176° (upright position)
</p>

<hr>

<h2>Key Operations</h2>

<h3>Sensor Initialization</h3>
<pre><code>
mpu.initialize();
mpu.setXGyroOffset(220);
mpu.setYGyroOffset(76);
mpu.setZGyroOffset(-85);
mpu.setZAccelOffset(1688);
</code></pre>

<h3>Tilt Calculation</h3>
<pre><code>
mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
input = ypr[1] * 180 / M_PI + 180;
</code></pre>

<h3>PID Computation & Motor Control</h3>
<pre><code>
pid.Compute();

if (output > 0) Forward();
else Reverse();
</code></pre>

<hr>

<h2>How It Works</h2>

<ul>
  <li>The MPU6050 detects the robot’s tilt angle.</li>
  <li>The PID controller calculates the required correction.</li>
  <li>The motor driver adjusts motor speed and direction.</li>
  <li>This feedback loop runs continuously to maintain balance.</li>
</ul>

<hr>

<h2>Conclusion</h2>

<p>
This project demonstrates the practical use of <b>feedback control,
sensor fusion, and motor control</b> in robotics.
It serves as an excellent introduction to <b>PID control systems</b> and
real-time embedded programming.
</p>

<p>
Experiment with PID tuning and mechanical adjustments to improve stability
and performance.
</p>
