within FIRE_CP_Modelica_Update.Sensors.Barometer;

model LowFidelity
  parameter Real samplePeriod = 0.02 "Barometer sample period [s]";
  parameter Real pressureBias = 0 "Pressure bias [Pa]";
  parameter Real altitudeBias = 0 "Altitude bias [m]";

  Interfaces.WorldBus world annotation(
    Placement(transformation(extent = {{-110, 40}, {-90, 60}})));
  Interfaces.SensorBus sensor annotation(
    Placement(transformation(extent = {{90, -10}, {110, 10}})));

  input Real position[3];
  input Real velocityWorld[3];

  Modelica.Blocks.Interfaces.RealOutput pressure;
  Modelica.Blocks.Interfaces.RealOutput temperature;
  Modelica.Blocks.Interfaces.RealOutput baroAltitude;
  Modelica.Blocks.Interfaces.RealOutput baroClimbRate;

protected
  discrete Real pressureBuffer(start = 101325);
  discrete Real temperatureBuffer(start = 288.15);
  discrete Real altitudeBuffer(start = 0);
  discrete Real climbRateBuffer(start = 0);

equation
  connect(sensor.pressure, pressure);
  connect(sensor.temperature, temperature);
  connect(sensor.baroAltitude, baroAltitude);
  connect(sensor.baroClimbRate, baroClimbRate);

  pressure = pre(pressureBuffer);
  temperature = pre(temperatureBuffer);
  baroAltitude = pre(altitudeBuffer);
  baroClimbRate = pre(climbRateBuffer);

algorithm
  when sample(0, samplePeriod) then
    pressureBuffer := world.pressure + pressureBias;
    temperatureBuffer := world.temperature;
    altitudeBuffer := position[3] + altitudeBias;
    climbRateBuffer := velocityWorld[3];
  end when;
end LowFidelity;
