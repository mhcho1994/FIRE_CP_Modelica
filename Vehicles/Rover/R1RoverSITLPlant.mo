within FIRE_CP_Modelica_Update.Vehicles.Rover;

model R1RoverSITLPlant
  import FIRE_CP_Modelica_Update.Utilities.Math.clip;
  import FIRE_CP_Modelica_Update.Utilities.Math.wrapPi;

  parameter Boolean skidSteering = false "Use skid-steering yaw mapping";
  parameter Boolean vectoredThrust = false "Use vectored-thrust yaw mapping";
  parameter Real maxAccel = if skidSteering then 14.0 else 10.0
    "Maximum longitudinal acceleration [m/s2]";
  parameter Real maxSpeed = if skidSteering then 4.0 else 8.0
    "Maximum forward speed [m/s]";
  parameter Real turningCircle = 1.8 "Nominal turn-circle diameter [m]";
  parameter Real maxWheelTurn = 35 * FIRE_CP_Modelica_Update.Utilities.Constants.d2r
    "Maximum steering wheel angle [rad]";
  parameter Real skidTurnRate = 140 * FIRE_CP_Modelica_Update.Utilities.Constants.d2r
    "Skid-steer yaw rate at full steering [rad/s]";
  parameter Real vectoredTurnRateMax = 120 * FIRE_CP_Modelica_Update.Utilities.Constants.d2r
    "Vectored-thrust yaw rate at full steering [rad/s]";
  parameter Real x_start = 0;
  parameter Real y_start = 0;
  parameter Real psi_start = 0;

  Interfaces.CommandBus command annotation(
    Placement(transformation(extent = {{-110, -10}, {-90, 10}})));

  Modelica.Blocks.Interfaces.RealInput steering;
  Modelica.Blocks.Interfaces.RealInput throttle;

  output Real position[3] "World position [m]";
  output Real velocityWorld[3] "World velocity [m/s]";
  output Real velocityBody[3] "Body velocity [m/s]";
  output Real accelerationBody[3] "Body specific force [m/s2]";
  output Real euler[3] "Roll, pitch, yaw [rad]";
  output Real rates[3] "Body angular rates [rad/s]";
  output Real yawRate "Yaw rate [rad/s]";
  output Real targetSpeed "Throttle target speed [m/s]";

protected
  Real x(start = x_start, fixed = true);
  Real y(start = y_start, fixed = true);
  Real psi(start = psi_start, fixed = true);
  Real speed(start = 0, fixed = true);
  Real steeringLimited;
  Real throttleLimited;
  Real turnDiameter;
  Real longitudinalAccel;
  Real lateralAccel;

equation
  connect(command.steering, steering);
  connect(command.throttle, throttle);

  steeringLimited = clip(steering, -1, 1);
  throttleLimited = clip(throttle, -1, 1);
  targetSpeed = throttleLimited * maxSpeed;

  turnDiameter =
    if abs(steeringLimited) < FIRE_CP_Modelica_Update.Utilities.Constants.eps then 0
    else turningCircle * sin(maxWheelTurn) / sin(steeringLimited * maxWheelTurn);

  yawRate =
    if skidSteering then steeringLimited * skidTurnRate
    elseif vectoredThrust then steeringLimited * vectoredTurnRateMax
    elseif abs(steeringLimited) < FIRE_CP_Modelica_Update.Utilities.Constants.eps or abs(speed) < FIRE_CP_Modelica_Update.Utilities.Constants.eps then 0
    else 2 * FIRE_CP_Modelica_Update.Utilities.Constants.PI * speed / (FIRE_CP_Modelica_Update.Utilities.Constants.PI * turnDiameter);

  longitudinalAccel = maxAccel * (targetSpeed - speed) / max(maxSpeed, FIRE_CP_Modelica_Update.Utilities.Constants.eps);
  lateralAccel = yawRate * speed;

  der(psi) = yawRate;
  der(speed) = longitudinalAccel;
  der(x) = speed * cos(psi);
  der(y) = speed * sin(psi);

  position = {x, y, 0};
  velocityWorld = {speed * cos(psi), speed * sin(psi), 0};
  velocityBody = {speed, 0, 0};
  accelerationBody = {longitudinalAccel, lateralAccel, -FIRE_CP_Modelica_Update.Utilities.Constants.g};
  euler = {0, 0, wrapPi(psi)};
  rates = {0, 0, yawRate};
end R1RoverSITLPlant;
