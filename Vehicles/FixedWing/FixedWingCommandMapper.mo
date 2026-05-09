within FIRE_CP_Modelica_Update.Vehicles.FixedWing;

model FixedWingCommandMapper
  import FIRE_CP_Modelica_Update.Utilities.Math.clip;

  parameter Integer nChannels(min = 3) = 3
    "Number of normalized actuator channels";
  parameter Integer leftElevonChannel(min = 1) = 1
    "Normalized actuator channel used for left elevon";
  parameter Integer rightElevonChannel(min = 1) = 2
    "Normalized actuator channel used for right elevon";
  parameter Integer throttleChannel(min = 1) = 3
    "Normalized actuator channel used for throttle";
  parameter Real elevonDeflectionMax = 0.524
    "Maximum elevon joint deflection from SITL model [rad]";

  Modelica.Blocks.Interfaces.RealInput actuatorCommand[nChannels] annotation(
    Placement(transformation(extent = {{-110, -10}, {-90, 10}})));
  Interfaces.CommandBus command annotation(
    Placement(transformation(extent = {{90, -10}, {110, 10}})));

  Modelica.Blocks.Interfaces.RealOutput leftElevon;
  Modelica.Blocks.Interfaces.RealOutput rightElevon;
  Modelica.Blocks.Interfaces.RealOutput throttle;

equation
  connect(command.leftElevon, leftElevon);
  connect(command.rightElevon, rightElevon);
  connect(command.throttle, throttle);

  assert(leftElevonChannel <= nChannels, "leftElevonChannel must be less than or equal to nChannels");
  assert(rightElevonChannel <= nChannels, "rightElevonChannel must be less than or equal to nChannels");
  assert(throttleChannel <= nChannels, "throttleChannel must be less than or equal to nChannels");

  leftElevon = clip(actuatorCommand[leftElevonChannel], -1, 1);
  rightElevon = clip(actuatorCommand[rightElevonChannel], -1, 1);
  throttle = clip(actuatorCommand[throttleChannel], 0, 1);
end FixedWingCommandMapper;
