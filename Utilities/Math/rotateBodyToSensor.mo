within FIRE_CP_Modelica_Update.Utilities.Math;

function rotateBodyToSensor
  input Real vBody[3] "Vector resolved in body frame";
  input Real roll "Sensor roll mounting angle relative to body [rad]";
  input Real pitch "Sensor pitch mounting angle relative to body [rad]";
  input Real yaw "Sensor yaw mounting angle relative to body [rad]";
  output Real vSensor[3] "Vector resolved in sensor frame";

protected
  Real Rbs[3, 3];

algorithm
  Rbs := euler321ToRotationMatrix({roll, pitch, yaw});
  vSensor[1] := Rbs[1, 1] * vBody[1] + Rbs[2, 1] * vBody[2] + Rbs[3, 1] * vBody[3];
  vSensor[2] := Rbs[1, 2] * vBody[1] + Rbs[2, 2] * vBody[2] + Rbs[3, 2] * vBody[3];
  vSensor[3] := Rbs[1, 3] * vBody[1] + Rbs[2, 3] * vBody[2] + Rbs[3, 3] * vBody[3];
end rotateBodyToSensor;
