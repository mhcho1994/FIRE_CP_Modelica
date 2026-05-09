within FIRE_CP_Modelica_Update.Vehicles.Rover;

model RoverCommandMapper
  import FIRE_CP_Modelica_Update.Utilities.Math.clip;

  parameter Boolean skidSteering = false
    "Interpret actuator channels as left/right motor commands";
  parameter Boolean vectoredThrust = false
    "Apply SITL vectored-thrust steering conversion";
  parameter Integer nChannels(min = 2) = 2
    "Number of normalized actuator channels";
  parameter Integer steeringChannel(min = 1) = 1
    "Normalized actuator channel used for steering";
  parameter Integer throttleChannel(min = 1) = 2
    "Normalized actuator channel used for throttle";
  parameter Real vectoredAngleMax = 90 * FIRE_CP_Modelica_Update.Utilities.Constants.d2r
    "Maximum vector angle [rad]";

  Modelica.Blocks.Interfaces.RealInput actuatorCommand[nChannels] annotation(
    Placement(transformation(extent = {{-110, -10}, {-90, 10}})));
  Interfaces.CommandBus command annotation(
    Placement(transformation(extent = {{90, -10}, {110, 10}})));

  Modelica.Blocks.Interfaces.RealOutput steering;
  Modelica.Blocks.Interfaces.RealOutput throttle;

protected
  Real vectorAngle;
  Real leftMotor;
  Real rightMotor;

equation
  connect(command.steering, steering);
  connect(command.throttle, throttle);

  assert(steeringChannel <= nChannels, "steeringChannel must be less than or equal to nChannels");
  assert(throttleChannel <= nChannels, "throttleChannel must be less than or equal to nChannels");

  vectorAngle = actuatorCommand[steeringChannel] * vectoredAngleMax;
  leftMotor = actuatorCommand[steeringChannel];
  rightMotor = actuatorCommand[throttleChannel];

  if skidSteering then
    steering = clip(leftMotor - rightMotor, -1, 1);
    throttle = clip(0.5 * (leftMotor + rightMotor), -1, 1);
  elseif vectoredThrust then
    steering = sin(vectorAngle) * actuatorCommand[throttleChannel];
    throttle = cos(vectorAngle) * actuatorCommand[throttleChannel];
  else
    steering = actuatorCommand[steeringChannel];
    throttle = actuatorCommand[throttleChannel];
  end if;
end RoverCommandMapper;
