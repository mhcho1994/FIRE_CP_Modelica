within FIRE_CP_Modelica_Update.Sensors.GNSS;

model LowFidelity
  parameter Real samplePeriod = 0.2 "GNSS sample period [s]";
  parameter Real positionBias[3] = {0, 0, 0} "Position bias [m]";
  parameter Real velocityBias[3] = {0, 0, 0} "Velocity bias [m/s]";

  Interfaces.SensorBus sensor annotation(
    Placement(transformation(extent = {{90, -10}, {110, 10}})));

  input Real position[3];
  input Real velocityWorld[3];

  Modelica.Blocks.Interfaces.RealOutput x;
  Modelica.Blocks.Interfaces.RealOutput y;
  Modelica.Blocks.Interfaces.RealOutput z;
  Modelica.Blocks.Interfaces.RealOutput vx;
  Modelica.Blocks.Interfaces.RealOutput vy;
  Modelica.Blocks.Interfaces.RealOutput vz;

protected
  discrete Real positionBuffer[3](start = {0, 0, 0});
  discrete Real velocityWorldBuffer[3](start = {0, 0, 0});

equation
  connect(sensor.x, x);
  connect(sensor.y, y);
  connect(sensor.z, z);
  connect(sensor.vx, vx);
  connect(sensor.vy, vy);
  connect(sensor.vz, vz);

  x = pre(positionBuffer[1]);
  y = pre(positionBuffer[2]);
  z = pre(positionBuffer[3]);
  vx = pre(velocityWorldBuffer[1]);
  vy = pre(velocityWorldBuffer[2]);
  vz = pre(velocityWorldBuffer[3]);

algorithm
  when sample(0, samplePeriod) then
    positionBuffer := position + positionBias;
    velocityWorldBuffer := velocityWorld + velocityBias;
  end when;
end LowFidelity;
