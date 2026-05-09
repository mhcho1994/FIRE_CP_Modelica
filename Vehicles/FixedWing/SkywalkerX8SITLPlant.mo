within FIRE_CP_Modelica_Update.Vehicles.FixedWing;

model SkywalkerX8SITLPlant
  import FIRE_CP_Modelica_Update.Utilities.Math.clip;
  import FIRE_CP_Modelica_Update.Utilities.Math.wrapPi;

  extends FixedWingBase;

  parameter Real mass = 4.5 "Mass from SITL SDF base_link [kg]";
  parameter Real ixx = 0.45 "Roll inertia from SITL SDF [kg.m2]";
  parameter Real iyy = 0.325 "Pitch inertia from SITL SDF [kg.m2]";
  parameter Real izz = 0.75 "Yaw inertia from SITL SDF [kg.m2]";
  parameter Real elevonDeflectionMax = 0.524
    "Elevon deflection limit from SITL SDF [rad]";
  parameter Real minAirspeed = 10.0 "Minimum trimmed flight speed [m/s]";
  parameter Real maxAirspeed = 28.0 "Maximum trimmed flight speed [m/s]";
  parameter Real speedTimeConstant = 1.5 "Throttle response time constant [s]";
  parameter Real maxRollRate = 120 * FIRE_CP_Modelica_Update.Utilities.Constants.d2r
    "Roll rate at full differential elevon [rad/s]";
  parameter Real maxPitchRate = 60 * FIRE_CP_Modelica_Update.Utilities.Constants.d2r
    "Pitch rate at full symmetric elevon [rad/s]";
  parameter Real rollDamping = 1.6 "Roll damping coefficient [1/s]";
  parameter Real pitchDamping = 1.2 "Pitch damping coefficient [1/s]";
  parameter Real x_start = 0;
  parameter Real y_start = 0;
  parameter Real z_start = 50;
  parameter Real psi_start = 0;
  parameter Real speed_start = 16;

  Interfaces.CommandBus command annotation(
    Placement(transformation(extent = {{-110, -10}, {-90, 10}})));

  Modelica.Blocks.Interfaces.RealInput leftElevon;
  Modelica.Blocks.Interfaces.RealInput rightElevon;
  Modelica.Blocks.Interfaces.RealInput throttle;

  output Real rollCommand "Differential elevon roll command [-1, 1]";
  output Real pitchCommand "Symmetric elevon pitch command [-1, 1]";
  output Real targetAirspeed "Throttle target airspeed [m/s]";

protected
  Real x(start = x_start, fixed = true);
  Real y(start = y_start, fixed = true);
  Real z(start = z_start, fixed = true);
  Real phi(start = 0, fixed = true);
  Real theta(start = 0, fixed = true);
  Real psi(start = psi_start, fixed = true);
  Real speed(start = speed_start, fixed = true);
  Real leftElevonLimited;
  Real rightElevonLimited;
  Real throttleLimited;
  Real rollRate;
  Real pitchRate;
  Real yawRate;
  Real longitudinalAccel;

equation
  connect(command.leftElevon, leftElevon);
  connect(command.rightElevon, rightElevon);
  connect(command.throttle, throttle);

  leftElevonLimited = clip(leftElevon, -1, 1);
  rightElevonLimited = clip(rightElevon, -1, 1);
  throttleLimited = clip(throttle, 0, 1);

  rollCommand = clip(0.5 * (leftElevonLimited - rightElevonLimited), -1, 1);
  pitchCommand = clip(0.5 * (leftElevonLimited + rightElevonLimited), -1, 1);
  targetAirspeed = minAirspeed + throttleLimited * (maxAirspeed - minAirspeed);

  longitudinalAccel = (targetAirspeed - speed) / max(speedTimeConstant, FIRE_CP_Modelica_Update.Utilities.Constants.eps);
  rollRate = maxRollRate * rollCommand - rollDamping * phi;
  pitchRate = maxPitchRate * pitchCommand - pitchDamping * theta;
  yawRate = FIRE_CP_Modelica_Update.Utilities.Constants.g * tan(phi) / max(speed, 1.0);

  der(speed) = longitudinalAccel;
  der(phi) = rollRate;
  der(theta) = pitchRate;
  der(psi) = yawRate;
  der(x) = speed * cos(theta) * cos(psi);
  der(y) = speed * cos(theta) * sin(psi);
  der(z) = speed * sin(theta);

  position = {x, y, z};
  velocityWorld = {speed * cos(theta) * cos(psi), speed * cos(theta) * sin(psi), speed * sin(theta)};
  velocityBody = {speed, 0, 0};
  accelerationBody = {longitudinalAccel, 0, -FIRE_CP_Modelica_Update.Utilities.Constants.g};
  euler = {phi, theta, wrapPi(psi)};
  rates = {rollRate, pitchRate, yawRate};
end SkywalkerX8SITLPlant;
