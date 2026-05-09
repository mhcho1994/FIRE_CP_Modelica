within FIRE_CP_Modelica_Update.Sensors.Magnetometer;

model LowFidelity
  import FIRE_CP_Modelica_Update.Utilities.Math.euler321ToRotationMatrix;
  import FIRE_CP_Modelica_Update.Utilities.Math.rotateBodyToSensor;

  parameter Real samplePeriod = 0.02 "Magnetometer sample period [s]";
  parameter Real mountRoll = 0 "Sensor roll mounting angle relative to body [rad]";
  parameter Real mountPitch = 0 "Sensor pitch mounting angle relative to body [rad]";
  parameter Real mountYaw = 0 "Sensor yaw mounting angle relative to body [rad]";
  parameter Real magBias[3] = {0, 0, 0} "Magnetometer bias [T]";

  Interfaces.WorldBus world annotation(
    Placement(transformation(extent = {{-110, 40}, {-90, 60}})));
  Interfaces.SensorBus sensor annotation(
    Placement(transformation(extent = {{90, -10}, {110, 10}})));

  input Real euler[3];

  Modelica.Blocks.Interfaces.RealOutput mx;
  Modelica.Blocks.Interfaces.RealOutput my;
  Modelica.Blocks.Interfaces.RealOutput mz;

protected
  discrete Real magBuffer[3](start = {0, 0, 0});
  Real Rwb[3, 3];
  Real magBody[3];
  Real magMounted[3];

equation
  connect(sensor.mx, mx);
  connect(sensor.my, my);
  connect(sensor.mz, mz);

  Rwb = euler321ToRotationMatrix(euler);
  magBody = {
    Rwb[1, 1] * world.magneticField[1] + Rwb[2, 1] * world.magneticField[2] + Rwb[3, 1] * world.magneticField[3],
    Rwb[1, 2] * world.magneticField[1] + Rwb[2, 2] * world.magneticField[2] + Rwb[3, 2] * world.magneticField[3],
    Rwb[1, 3] * world.magneticField[1] + Rwb[2, 3] * world.magneticField[2] + Rwb[3, 3] * world.magneticField[3]
  };
  magMounted = rotateBodyToSensor(magBody, mountRoll, mountPitch, mountYaw);

  mx = pre(magBuffer[1]);
  my = pre(magBuffer[2]);
  mz = pre(magBuffer[3]);

algorithm
  when sample(0, samplePeriod) then
    magBuffer := magMounted + magBias;
  end when;
end LowFidelity;
