//convert all axis
final int minAngle = -180;
final int maxAngle = 180;

void convert() {

  /* Convert the kalman filter x-axis */
  if (stringV_meas != null) {
    stringV_meas = trim(stringV_meas); // Trim off any whitespace
    V_meas[V_meas.length - 1] = map(float(stringV_meas), minAngle, maxAngle, 0, height); // Convert to a float and map to the screen height, then save in buffer
  }
  
    /* Convert the kalman filter x-axis */
  if (stringOutput != null) {
    stringOutput = trim(stringOutput); // Trim off any whitespace
    Output[Output.length - 1] = map(float(stringOutput), minAngle, maxAngle, 0, height); // Convert to a float and map to the screen height, then save in buffer
  }

}
