void drawAxisX() {
  /* Draw yaw x-axis */
  noFill();
  stroke(255, 0, 0);// Red
  // Redraw everything
  beginShape();
  vertex(0, V_meas[0]);
  for (int i = 1; i < V_meas.length; i++) {
    if ((V_meas[i] < height/4 && V_meas[i - 1] > height/4*3) || (V_meas[i] > height/4*3 && V_meas[i - 1] < height/4)) {
      endShape();
      beginShape();
    }
    vertex(i, V_meas[i]);
  }
  endShape();

  // Put all data one array back
  for (int i = 1; i < V_meas.length; i++)
    V_meas[i-1] = V_meas[i];
}

void drawAxisY() {
  /* Draw kalman filter y-axis */
  noFill();
  stroke(0, 0, 0); // Black
  // Redraw everything
  beginShape();
  vertex(0, Output[0]);
  for (int i = 1; i < Output.length; i++) {
    if ((Output[i] < height/4 && Output[i - 1] > height/4*3) || (Output[i] > height/4*3 && Output[i - 1] < height/4)) {
      endShape();
      beginShape();
    }
    vertex(i, Output[i]);
  }
  endShape();

  // Put all data one array back
  for (int i = 1; i<Output.length;i++)
    Output[i-1] = Output[i];
}

