within FIRE_CP_Modelica_Update.Utilities.Math;

function rotateVectorByQuaternion
  input Real v[3] "Vector in the source frame";
  input Real q[4] "Quaternion {w, x, y, z} from source to target";
  output Real vRotated[3] "Vector resolved in the target frame";

protected
  Real R[3, 3];

algorithm
  R := quaternionToRotationMatrix(q);
  vRotated[1] := R[1, 1] * v[1] + R[1, 2] * v[2] + R[1, 3] * v[3];
  vRotated[2] := R[2, 1] * v[1] + R[2, 2] * v[2] + R[2, 3] * v[3];
  vRotated[3] := R[3, 1] * v[1] + R[3, 2] * v[2] + R[3, 3] * v[3];
end rotateVectorByQuaternion;
