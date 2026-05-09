within FIRE_CP_Modelica_Update.Sensors.IMU;

model LowFidelity
  import FIRE_CP_Modelica_Update.Utilities.Math.rotateBodyToSensor;
  import FIRE_CP_Modelica_Update.Utilities.Math.wrapPi;

  parameter Real samplePeriod = 0.0025 "IMU sample period [s]";
  parameter Real mountRoll = 0 "Sensor roll mounting angle relative to body [rad]";
  parameter Real mountPitch = 0 "Sensor pitch mounting angle relative to body [rad]";
  parameter Real mountYaw = 0 "Sensor yaw mounting angle relative to body [rad]";
  parameter Real accelBias[3] = {0, 0, 0} "Accelerometer bias [m/s2]";
  parameter Real gyroBias[3] = {0, 0, 0} "Gyroscope bias [rad/s]";

  Interfaces.SensorBus sensor annotation(
    Placement(transformation(extent = {{90, -10}, {110, 10}})));

  input Real velocityBody[3];
  input Real accelerationBody[3];
  input Real euler[3];
  input Real rates[3];

  Modelica.Blocks.Interfaces.RealOutput u;
  Modelica.Blocks.Interfaces.RealOutput v;
  Modelica.Blocks.Interfaces.RealOutput w;
  Modelica.Blocks.Interfaces.RealOutput ax;
  Modelica.Blocks.Interfaces.RealOutput ay;
  Modelica.Blocks.Interfaces.RealOutput az;
  Modelica.Blocks.Interfaces.RealOutput phi;
  Modelica.Blocks.Interfaces.RealOutput theta;
  Modelica.Blocks.Interfaces.RealOutput psi;
  Modelica.Blocks.Interfaces.RealOutput p;
  Modelica.Blocks.Interfaces.RealOutput q;
  Modelica.Blocks.Interfaces.RealOutput r;

protected
  discrete Real velocityBodyBuffer[3](start = {0, 0, 0});
  discrete Real accelerationBuffer[3](start = {0, 0, 0});
  discrete Real eulerBuffer[3](start = {0, 0, 0});
  discrete Real ratesBuffer[3](start = {0, 0, 0});
  Real velocityMounted[3];
  Real accelerationMounted[3];
  Real ratesMounted[3];

equation
  connect(sensor.u, u);
  connect(sensor.v, v);
  connect(sensor.w, w);
  connect(sensor.ax, ax);
  connect(sensor.ay, ay);
  connect(sensor.az, az);
  connect(sensor.phi, phi);
  connect(sensor.theta, theta);
  connect(sensor.psi, psi);
  connect(sensor.p, p);
  connect(sensor.q, q);
  connect(sensor.r, r);

  velocityMounted = rotateBodyToSensor(velocityBody, mountRoll, mountPitch, mountYaw);
  accelerationMounted = rotateBodyToSensor(accelerationBody, mountRoll, mountPitch, mountYaw);
  ratesMounted = rotateBodyToSensor(rates, mountRoll, mountPitch, mountYaw);

  u = pre(velocityBodyBuffer[1]);
  v = pre(velocityBodyBuffer[2]);
  w = pre(velocityBodyBuffer[3]);
  ax = pre(accelerationBuffer[1]);
  ay = pre(accelerationBuffer[2]);
  az = pre(accelerationBuffer[3]);
  phi = pre(eulerBuffer[1]);
  theta = pre(eulerBuffer[2]);
  psi = wrapPi(pre(eulerBuffer[3]));
  p = pre(ratesBuffer[1]);
  q = pre(ratesBuffer[2]);
  r = pre(ratesBuffer[3]);

algorithm
  when sample(0, samplePeriod) then
    velocityBodyBuffer := velocityMounted;
    accelerationBuffer := accelerationMounted + accelBias;
    eulerBuffer := euler;
    ratesBuffer := ratesMounted + gyroBias;
  end when;
end LowFidelity;
