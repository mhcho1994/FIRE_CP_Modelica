within FIRE_CP_Modelica_Update.Utilities.Math;

function quaternionToRotationMatrix
  input Real q[4] "Quaternion {w, x, y, z}, body to world";
  output Real R[3, 3] "Rotation matrix from body frame to world frame";

protected
  Real qn[4];
  Real w;
  Real x;
  Real y;
  Real z;

algorithm
  qn := quaternionNormalize(q);
  w := qn[1];
  x := qn[2];
  y := qn[3];
  z := qn[4];

  R[1, 1] := 1 - 2 * (y * y + z * z);
  R[1, 2] := 2 * (x * y - z * w);
  R[1, 3] := 2 * (x * z + y * w);
  R[2, 1] := 2 * (x * y + z * w);
  R[2, 2] := 1 - 2 * (x * x + z * z);
  R[2, 3] := 2 * (y * z - x * w);
  R[3, 1] := 2 * (x * z - y * w);
  R[3, 2] := 2 * (y * z + x * w);
  R[3, 3] := 1 - 2 * (x * x + y * y);
end quaternionToRotationMatrix;
