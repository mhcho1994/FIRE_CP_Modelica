within FIRE_CP_Modelica_Update.Vehicles.Common;

partial model Vehicle6DOF
  output Real position[3] "World position [m]";
  output Real velocityWorld[3] "World velocity [m/s]";
  output Real velocityBody[3] "Body velocity [m/s]";
  output Real accelerationBody[3] "Body specific force [m/s2]";
  output Real euler[3] "Roll, pitch, yaw [rad]";
  output Real rates[3] "Body rates [rad/s]";
end Vehicle6DOF;
