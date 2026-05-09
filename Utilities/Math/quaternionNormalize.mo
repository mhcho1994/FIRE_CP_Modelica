within FIRE_CP_Modelica_Update.Utilities.Math;

function quaternionNormalize
  input Real q[4] "Quaternion {w, x, y, z}";
  output Real qUnit[4] "Normalized quaternion";

protected
  Real n;

algorithm
  n := sqrt(q[1] * q[1] + q[2] * q[2] + q[3] * q[3] + q[4] * q[4]);
  qUnit := q / max(n, FIRE_CP_Modelica_Update.Utilities.Constants.eps);
end quaternionNormalize;
