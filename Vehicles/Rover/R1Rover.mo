within FIRE_CP_Modelica_Update.Vehicles.Rover;

model R1Rover
  parameter Boolean skidSteering = false;
  parameter Boolean vectoredThrust = false;
  parameter Real actuatorSamplePeriod = 0.02;
  parameter Real sensorSamplePeriod = 0.02;
  parameter Real gnssSamplePeriod = 0.2;

  Interfaces.PwmBus pwm annotation(
    Placement(transformation(extent = {{-110, -10}, {-90, 10}})));
  Interfaces.WorldBus world annotation(
    Placement(transformation(extent = {{-10, 90}, {10, 110}})));
  Interfaces.SensorBus sensor annotation(
    Placement(transformation(extent = {{90, -10}, {110, 10}})));

  Actuators.PwmActuator pwmActuator(
    nChannels = 2,
    samplePeriod = actuatorSamplePeriod);
  RoverCommandMapper commandMapper(
    nChannels = 2,
    skidSteering = skidSteering,
    vectoredThrust = vectoredThrust);
  R1RoverSITLPlant plant(
    skidSteering = skidSteering,
    vectoredThrust = vectoredThrust);
  Sensors.Barometer.LowFidelity barometer(samplePeriod = sensorSamplePeriod);
  Sensors.GNSS.LowFidelity gnss(samplePeriod = gnssSamplePeriod);
  Sensors.IMU.LowFidelity imu(samplePeriod = sensorSamplePeriod);
  Sensors.Magnetometer.LowFidelity magnetometer(samplePeriod = sensorSamplePeriod);

equation
  connect(pwm.ch_0, pwmActuator.pwmCommand[1]);
  connect(pwm.ch_2, pwmActuator.pwmCommand[2]);
  connect(pwmActuator.normalizedCommand, commandMapper.actuatorCommand);
  connect(commandMapper.command, plant.command);
  connect(world, barometer.world);
  connect(world, magnetometer.world);
  connect(barometer.sensor, sensor);
  connect(gnss.sensor, sensor);
  connect(imu.sensor, sensor);
  connect(magnetometer.sensor, sensor);

  barometer.position = plant.position;
  barometer.velocityWorld = plant.velocityWorld;
  gnss.position = plant.position;
  gnss.velocityWorld = plant.velocityWorld;
  imu.velocityBody = plant.velocityBody;
  imu.accelerationBody = plant.accelerationBody;
  imu.euler = plant.euler;
  imu.rates = plant.rates;
  magnetometer.euler = plant.euler;
end R1Rover;
