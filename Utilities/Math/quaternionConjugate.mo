within FIRE_CP_Modelica_Update.Utilities.Math;

function quaternionConjugate
  input Real q[4] "Quaternion {w, x, y, z}";
  output Real qConj[4] "Quaternion conjugate";

algorithm
  qConj := {q[1], -q[2], -q[3], -q[4]};
end quaternionConjugate;
