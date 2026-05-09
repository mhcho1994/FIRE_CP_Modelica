within FIRE_CP_Modelica_Update.Vehicles.FixedWing;

model SkywalkerX8
  extends FixedWingBase;

  parameter Real actuatorSamplePeriod = 0.02;
  parameter Real imuSamplePeriod = 0.0025;
  parameter Real barometerSamplePeriod = 0.02;
  parameter Real gnssSamplePeriod = 0.2;
  parameter Real magnetometerSamplePeriod = 0.02;
  parameter Real imuMountRoll = 180 * FIRE_CP_Modelica_Update.Utilities.Constants.d2r
    "IMU roll mounting angle from SITL SDF [rad]";
  parameter Real imuMountPitch = 0 "IMU pitch mounting angle [rad]";
  parameter Real imuMountYaw = 0 "IMU yaw mounting angle [rad]";
  parameter Real magnetometerMountRoll = 0 "Magnetometer roll mounting angle [rad]";
  parameter Real magnetometerMountPitch = 0 "Magnetometer pitch mounting angle [rad]";
  parameter Real magnetometerMountYaw = 0 "Magnetometer yaw mounting angle [rad]";

  Interfaces.PwmBus pwm annotation(
    Placement(transformation(extent = {{-110, -10}, {-90, 10}})));
  Interfaces.WorldBus world annotation(
    Placement(transformation(extent = {{-10, 90}, {10, 110}})));
  Interfaces.SensorBus sensor annotation(
    Placement(transformation(extent = {{90, -10}, {110, 10}})));

  Actuators.PwmActuator pwmActuator(
    nChannels = 3,
    pwmMin = {1100, 1100, 1000},
    pwmTrim = {1500, 1500, 1000},
    pwmMax = {1900, 1900, 2000},
    samplePeriod = actuatorSamplePeriod);
  FixedWingCommandMapper commandMapper(nChannels = 3);
  SkywalkerX8SITLPlant plant;
  Sensors.Barometer.LowFidelity barometer(samplePeriod = barometerSamplePeriod);
  Sensors.GNSS.LowFidelity gnss(samplePeriod = gnssSamplePeriod);
  Sensors.IMU.LowFidelity imu(
    samplePeriod = imuSamplePeriod,
    mountRoll = imuMountRoll,
    mountPitch = imuMountPitch,
    mountYaw = imuMountYaw);
  Sensors.Magnetometer.LowFidelity magnetometer(
    samplePeriod = magnetometerSamplePeriod,
    mountRoll = magnetometerMountRoll,
    mountPitch = magnetometerMountPitch,
    mountYaw = magnetometerMountYaw);

equation
  connect(pwm.ch_0, pwmActuator.pwmCommand[1]);
  connect(pwm.ch_1, pwmActuator.pwmCommand[2]);
  connect(pwm.ch_2, pwmActuator.pwmCommand[3]);
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

  position = plant.position;
  velocityWorld = plant.velocityWorld;
  velocityBody = plant.velocityBody;
  accelerationBody = plant.accelerationBody;
  euler = plant.euler;
  rates = plant.rates;
end SkywalkerX8;
