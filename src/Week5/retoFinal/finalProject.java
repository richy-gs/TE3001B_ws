import processing.opengl.*;

PMatrix3D matrix = new PMatrix3D();

float fingerPt1_length = 50;
float fingerPt2_length = 50;
float fingerPt3_length = 30;
float fingertipCylinder_height = 14;

boolean first_execution = true;

float[] sol1  = new float[3];
float[] sol2  = new float[3];
int[] validFingerAngles  = new int[3];
int[] validBoxPos  = new int[6];
float[] currentReach = new float[3];

int[] boxPosChange  = new int[6];
int[] boxAngleChange  = new int[6];

float[] angles_F1 = new float[3];
float[] angles_F2 = new float[3];
float[] angles_F3 = new float[3];

int traslationFlag = 0;
int rotationFlag = 0;
int slidingFlag = 0;
int sol1Clamped = 0;
int sol2Clamped = 0;

// prev valid values
float prevX = 0, prevY = 0, prevZ = 0;
float prevPitch = 0, prevYaw = 0, prevRoll = 0;
float[] prev_angles_F1 = new float[3];
float[] prev_angles_F2 = new float[3];
float[] prev_angles_F3 = new float[3];

float rotX = 0;
float rotY = 0;
float pi = PI;

float[] localPointF1, localPointF2, localPointF3;
float[][] sphereCenterF = new float[3][3];

float f1_j1_ang_uplim = PI/2, f1_j1_ang_botlim = -PI/2, f2_j1_ang_uplim = PI/2, f2_j1_ang_botlim = -PI/2, f3_j1_ang_uplim = PI/2, f3_j1_ang_botlim = -PI/2;
float f1_j2_ang_uplim = 1.8, f1_j2_ang_botlim = -1.8, f2_j2_ang_uplim = 1.8, f2_j2_ang_botlim = -1.8, f3_j2_ang_uplim = 1.8, f3_j2_ang_botlim = -1.8;
float f1_j3_ang_uplim = 2.2, f1_j3_ang_botlim = -2.2, f2_j3_ang_uplim = 2.2, f2_j3_ang_botlim = -2.2, f3_j3_ang_uplim = 2.2, f3_j3_ang_botlim = -2.2;

float theta_x = 0, theta_y = 0, theta_z = 0, sphere_radius = 7;

float targetX_F1 = 70, targetY_F1 = -70, targetZ_F1 = -50, currentX_F1 = 0, currentY_F1 = 0, currentZ_F1 = 0;
float targetX_F2 = 70, targetY_F2 = -70, targetZ_F2 = -50, currentX_F2 = 0, currentY_F2 = 0, currentZ_F2 = 0;
float targetX_F3 = 70, targetY_F3 = -70, targetZ_F3 = 150, currentX_F3 = 0, currentY_F3 = 0, currentZ_F3 = 0;

float finger1_j1_angle = 0, finger1_j2_angle = 0, finger1_j3_angle = 0;
float finger2_j1_angle = 0, finger2_j2_angle = 0, finger2_j3_angle = 0;
float finger3_j1_angle = 0, finger3_j2_angle = 0, finger3_j3_angle = 0;

float finger1_x_offset = -20, finger1_y_offset = 0, finger1_z_offset = 0;
float finger2_x_offset = 20, finger2_y_offset = 0, finger2_z_offset = 0;
float finger3_x_offset = 0, finger3_y_offset = 0, finger3_z_offset = 90;

float angle_F1 = 0;       // Will track our current angle for circular motion
float radius_F1 = 15;     // Circle radius
float angleSpeed_F1 = 0.05; // How fast we move around the circle

float angle_F2 = 0;
float radius_F2 = 5;
float angleSpeed_F2 = 0.05;

boolean drawAxes = true;

// BOX LIMITS
float maxReach = 104.5;
float xUpLim = 96, xBotLim = -96, yUpLim = -98, yBotLim = -144, zUpLim = 91, zBotLim = -1;
float xNearUpLim = 0, xNearBotLim = 0, yNearUpLim = 0, yNearBotLim = 0, zNearUpLim = 0, zNearBotLim = 0;
float pitchUpLim = 2.2, pitchBotLim = 1.1, rollUpLim = 0.4, rollBotLim = -0.4, yawUpLim = 0.27, yawBotLim = -0.27;
float pitchNearUpLim = 0, pitchNearBotLim = 0, rollNearUpLim = 0, rollNearBotLim = 0, yawNearUpLim = 0, yawNearBotLim = 0;

// Object (box) position in global coordinates
float xPos = 0, yPos = -100, zPos = 35;

// Global positions for points in box
float globalPosX_F1 = 0, globalPosY_F1 = 0, globalPosZ_F1 = 0;
float globalPosX_F2 = 0, globalPosY_F2 = 0, globalPosZ_F2 = 0;
float globalPosX_F3 = 0, globalPosY_F3 = 0, globalPosZ_F3 = 0;

// Finger offsets from box center of mass (local coordinates)
float localPosX_F1 = 30, localPosY_F1 = -12.5, localPosZ_F1 = 0;
float localPosX_F2 = -30, localPosY_F2 = -12.5, localPosZ_F2 = 0;
float localPosX_F3 = 0, localPosY_F3 = 12.5, localPosZ_F3 = 0;

// Box rotation variables (pitch, yaw, roll)
float boxPitch = PI/2, boxYaw = 0, boxRoll = 0;
// Box move and rotation speed
float moveSpeed = 2.0, rotateSpeed = 0.1;

// Movement flags for translation
boolean wPressed = false, aPressed = false, sPressed = false, dPressed = false, qPressed = false, ePressed = false;
// Movement flags for box rotation
boolean rPressed = false, tPressed = false, yPressed = false, fPressed = false, gPressed = false, hPressed = false;

PVector contactPointF1, contactPointF2, contactPointF3;
PVector spherePosF1, spherePosF2, spherePosF3;

// Extend the shortest distance line through the cube to the opposite side
PVector directionF1, directionF2, directionF3, extendedExitPointF1, extendedExitPointF2, extendedExitPointF3;

void setup() {
  size(800, 600, P3D);
  contactPointF1 = new PVector();
  contactPointF2 = new PVector();
  contactPointF3 = new PVector();
  spherePosF1 = new PVector();
  spherePosF2 = new PVector();
  spherePosF3 = new PVector();
  directionF1 = new PVector();
  extendedExitPointF1 = new PVector();
  directionF2 = new PVector();
  extendedExitPointF2 = new PVector();
  directionF3 = new PVector();
  extendedExitPointF3 = new PVector();
}

void draw() {
  background(0);
  lights();

  // --- Box movement & rotation from keys ---
  moveSpeed = 2.0;
  if (currentReach[0] < maxReach - 2*moveSpeed && currentReach[1] < maxReach - 2*moveSpeed && currentReach[2] < maxReach - 2*moveSpeed ) {
    if (wPressed && yPos > yBotLim) { yPos -= moveSpeed; boxPosChange[3] = 1; traslationFlag = 1;}
  }
  
  //println("currentReach F1 " + currentReach[0] + " F2 " + currentReach[1] + " F3 " + currentReach[2]);
    
  if (currentReach[0] >= (maxReach - 2*moveSpeed) || currentReach[1] >= (maxReach - 2*moveSpeed) || currentReach[2] >= (maxReach - 2*moveSpeed) ) {
    xNearUpLim = xUpLim - xPos;
    xNearBotLim = xBotLim - xPos;
    yNearUpLim = yUpLim - yPos;
    yNearBotLim = yBotLim - yPos;
    zNearUpLim = zUpLim - zPos;
    zNearBotLim = zBotLim - zPos;
    
    if (dPressed && xPos < xUpLim && abs(xNearUpLim) > abs(xNearBotLim)) { xPos += moveSpeed; boxPosChange[0] = 1; traslationFlag = 1;}
    if (aPressed && xPos > xBotLim && abs(xNearUpLim) < abs(xNearBotLim)) { xPos -= moveSpeed; boxPosChange[1] = 1; traslationFlag = 1;}
    if (qPressed && zPos < zUpLim && abs(zNearUpLim) > abs(zNearBotLim)) { zPos += moveSpeed; boxPosChange[4] = 1; traslationFlag = 1;}
    if (ePressed && zPos > zBotLim && abs(zNearUpLim) < abs(zNearBotLim)) { zPos -= moveSpeed; boxPosChange[5] = 1; traslationFlag = 1;}
    if (sPressed && yPos < yUpLim) {  yPos += moveSpeed; boxPosChange[2] = 1; traslationFlag = 1;}
  }
  else {
    if (dPressed && xPos < xUpLim) { xPos += moveSpeed; boxPosChange[0] = 1; traslationFlag = 1;}
    if (aPressed && xPos > xBotLim) { xPos -= moveSpeed; boxPosChange[1] = 1; traslationFlag = 1;}
    if (qPressed && zPos < zUpLim) { zPos += moveSpeed; boxPosChange[4] = 1; traslationFlag = 1;}
    if (ePressed && zPos > zBotLim) { zPos -= moveSpeed; boxPosChange[5] = 1; traslationFlag = 1;}
    if (sPressed && yPos < yUpLim) { yPos += moveSpeed; boxPosChange[2] = 1; traslationFlag = 1;}
  }

   
    rotateSpeed = 0.01;
  if (currentReach[0] >= (maxReach - 2*moveSpeed) || currentReach[1] >= (maxReach - 2*moveSpeed) || currentReach[2] >= (maxReach - 2*moveSpeed) ) {
    pitchNearUpLim  = pitchUpLim  - boxPitch;
    pitchNearBotLim = pitchBotLim - boxPitch;
    rollNearUpLim  = rollUpLim  - boxRoll;
    rollNearBotLim = rollBotLim - boxRoll;
    yawNearUpLim   = yawUpLim   - boxYaw;
    yawNearBotLim  = yawBotLim  - boxYaw;
    
    if (rPressed && boxPitch < pitchUpLim && abs(pitchNearUpLim) > abs(pitchNearBotLim)) { boxPitch += rotateSpeed; boxAngleChange[0] = 1; rotationFlag = 1;}
    if (fPressed && boxPitch > pitchBotLim && abs(pitchNearUpLim) < abs(pitchNearBotLim)) { boxPitch -= rotateSpeed; boxAngleChange[1] = 1; rotationFlag = 1;}
    if (tPressed && boxYaw < yawUpLim && abs(yawNearUpLim) > abs(yawNearBotLim)) { boxYaw += rotateSpeed; boxAngleChange[2] = 1; rotationFlag = 1;}
    if (gPressed && boxYaw > yawBotLim && abs(yawNearUpLim) < abs(yawNearBotLim)) { boxYaw   -= rotateSpeed; boxAngleChange[3] = 1; rotationFlag = 1;}
    if (yPressed && boxRoll < rollUpLim && abs(rollNearUpLim) > abs(rollNearBotLim)) { boxRoll  += rotateSpeed; boxAngleChange[4] = 1; rotationFlag = 1;}
    if (hPressed && boxRoll > rollBotLim && abs(rollNearUpLim) < abs(rollNearBotLim)) { boxRoll  -= rotateSpeed; boxAngleChange[5] = 1; rotationFlag = 1;}
    
  } else {
    if (rPressed && boxPitch < pitchUpLim) { boxPitch += rotateSpeed; boxAngleChange[0] = 1; rotationFlag = 1;}
    if (fPressed && boxPitch > pitchBotLim) { boxPitch -= rotateSpeed; boxAngleChange[1] = 1; rotationFlag = 1;}
    if (tPressed && boxYaw < yawUpLim) { boxYaw += rotateSpeed; boxAngleChange[2] = 1; rotationFlag = 1;}
    if (gPressed && boxYaw > yawBotLim) { boxYaw -= rotateSpeed; boxAngleChange[3] = 1; rotationFlag = 1;}
    if (yPressed && boxRoll < rollUpLim) { boxRoll += rotateSpeed; boxAngleChange[4] = 1; rotationFlag = 1;}
    if (hPressed && boxRoll > rollBotLim) { boxRoll -= rotateSpeed; boxAngleChange[5] = 1; rotationFlag = 1;}
  }

  targetX_F1 = globalPosX_F2;
  targetY_F1 = globalPosY_F2;
  targetZ_F1 = globalPosZ_F2;

  targetX_F2 = globalPosX_F1;
  targetY_F2 = globalPosY_F1;
  targetZ_F2 = globalPosZ_F1;

  targetX_F3 = globalPosX_F3;
  targetY_F3 = globalPosY_F3;
  targetZ_F3 = globalPosZ_F3;

  // --- Inverse kinematics for each finger ---
  if (!first_execution) {
    inverseKinematics(targetX_F1 - finger1_x_offset, targetY_F1 - finger1_y_offset, targetZ_F1 - finger1_z_offset, angles_F1, 1, currentReach);

    inverseKinematics(targetX_F2 - finger2_x_offset, targetY_F2 - finger2_y_offset, targetZ_F2 - finger2_z_offset, angles_F2, 2, currentReach);
  
    inverseKinematics(targetX_F3 - finger3_x_offset, targetY_F3 - finger3_y_offset, targetZ_F3 - finger3_z_offset, angles_F3, 3, currentReach);
  }
  
  if ((validFingerAngles[0] == 0 || validFingerAngles[1] == 0 || validFingerAngles[2] == 0) && !first_execution){
    // Revert box position change    
    angles_F1 = prev_angles_F1;
    angles_F2 = prev_angles_F2;
    angles_F3 = prev_angles_F3;
    xPos = prevX; yPos = prevY; zPos = prevZ;
    boxPitch = prevPitch; boxYaw = prevYaw; boxRoll = prevRoll; 
  }
  validFingerAngles = new int[]{1,1,1};
  boxPosChange = new int[]{0, 0, 0, 0, 0, 0};
  boxAngleChange = new int[]{0, 0, 0, 0, 0, 0};

  if (validFingerAngles[0] == 1) {
    finger1_j1_angle = angles_F1[0];
    finger1_j2_angle = angles_F1[1];
    finger1_j3_angle = angles_F1[2];
  }

  if (validFingerAngles[1] == 1) {
    finger2_j1_angle = angles_F2[0];
    finger2_j2_angle = angles_F2[1];
    finger2_j3_angle = angles_F2[2];
  }

  if (validFingerAngles[2] == 1) {
    finger3_j1_angle = angles_F3[0];
    finger3_j2_angle = angles_F3[1];
    finger3_j3_angle = angles_F3[2];
  }
  
  if (validFingerAngles[0] == 1 && validFingerAngles[1] == 1 && validFingerAngles[2] == 1) {
    prev_angles_F1 = angles_F1;
    prev_angles_F2 = angles_F2;
    prev_angles_F3 = angles_F3;
    prevX = xPos; prevY = yPos; prevZ = zPos;
    prevPitch = boxPitch; prevYaw = boxYaw; prevRoll = boxRoll;
  }

  /*println("F1 J1: " + finger1_j1_angle + " J2: " + finger1_j2_angle + " J3: " + finger1_j3_angle
   + "    F2 J1: " + finger2_j1_angle + " J1: " + finger2_j2_angle + " J3: " + finger2_j3_angle
   + "    F3 J1: " + finger3_j1_angle + " J2: " + finger3_j2_angle + " J3: " + finger3_j3_angle);*/

  // --- CAMERA / SCENE TRANSFORM ONCE ---
  pushMatrix();
  translate(width / 2, height / 2, 0);
  rotateX(rotX);
  rotateY(rotY);
  rotateX(-PI/2);

  // (Optional) scale(1.0);

  // Draw global axes for reference
  drawMainAxes();

  /*// 1) Draw Spheres at finger IK targets (yellow)
   pushMatrix();
   translate(targetX_F1, targetY_F1, targetZ_F1);
   fill(255, 255, 0);
   sphere(3);
   popMatrix();
   
   pushMatrix();
   translate(targetX_F2, targetY_F2, targetZ_F2);
   fill(255, 255, 0);
   sphere(3);
   popMatrix();
   
   pushMatrix();
   translate(targetX_F3, targetY_F3, targetZ_F3);
   fill(255, 255, 0);
   sphere(5);
   popMatrix();*/

  pushMatrix();
  rotateX(PI/2);
  translate(0,200,-35);
  drawMainBase();
  drawBasePt2();
  popMatrix();

  // Draw each finger in the same coordinate space
  pushMatrix();
  drawFinger(finger1_x_offset, finger1_y_offset, finger1_z_offset,
    finger1_j1_angle, finger1_j2_angle, finger1_j3_angle);
  forwardKinematicsFinger(finger1_x_offset, finger1_y_offset, finger1_z_offset,
    finger1_j1_angle, finger1_j2_angle, finger1_j3_angle, 1);
  //fill(255);
  //sphere(3);
  popMatrix();

  pushMatrix();
  drawFinger(finger2_x_offset, finger2_y_offset, finger2_z_offset,
    finger2_j1_angle, finger2_j2_angle, finger2_j3_angle);
  forwardKinematicsFinger(finger2_x_offset, finger2_y_offset, finger2_z_offset,
    finger2_j1_angle, finger2_j2_angle, finger2_j3_angle, 2);
  //fill(255, 0, 0);
  //sphere(3);
  popMatrix();

  pushMatrix();
  drawFinger(finger3_x_offset, finger3_y_offset, finger3_z_offset,
    finger3_j1_angle, finger3_j2_angle, finger3_j3_angle);
  forwardKinematicsFinger(finger3_x_offset, finger3_y_offset, finger3_z_offset,
    finger3_j1_angle, finger3_j2_angle, finger3_j3_angle, 3);
  //fill(255, 0, 0);
  //sphere(3);
  popMatrix();

  // Draw the box in the same coordinate frame
  //    so it appears in the same "world" as the fingers.
  pushMatrix();
  // Translate & rotate box

  translate(xPos, yPos, zPos);
  rotateX(boxPitch);
  rotateY(boxYaw);
  rotateZ(boxRoll);

  // Draw local axes for the box
  drawLocalAxes();
  fill(255, 255, 255);
  //noFill();
  // Draw the actual box
  box(100, 25, 35);

  //println(" Box Positions  X " + xPos + " Y " + yPos + " Z " + zPos + " roll " + boxRoll + " pitch " + boxPitch  + " yaw " + boxYaw);

  // --- Compute global coordinates of local points on the box ---
  // Because we are still inside the same pushMatrix that moved/rotated the box,
  // modelX/modelY/modelZ will give us global positions for the local offsets:
  globalPosX_F1 = modelX(localPosX_F1, localPosY_F1, localPosZ_F1);
  globalPosY_F1 = modelY(localPosX_F1, localPosY_F1, localPosZ_F1);
  globalPosZ_F1 = modelZ(localPosX_F1, localPosY_F1, localPosZ_F1);

  globalPosX_F2 = modelX(localPosX_F2, localPosY_F2, localPosZ_F2);
  globalPosY_F2 = modelY(localPosX_F2, localPosY_F2, localPosZ_F2);
  globalPosZ_F2 = modelZ(localPosX_F2, localPosY_F2, localPosZ_F2);

  globalPosX_F3 = modelX(localPosX_F3, localPosY_F3, localPosZ_F3);
  globalPosY_F3 = modelY(localPosX_F3, localPosY_F3, localPosZ_F3);
  globalPosZ_F3 = modelZ(localPosX_F3, localPosY_F3, localPosZ_F3);
  popMatrix();  // done with box transform

  // Print the global positions to console
  /*println(
   "Global Positions F1 : (" + nf(globalPosX_F1, 1, 2) + ", " + nf(globalPosY_F1, 1, 2) + ", " +  nf(globalPosZ_F1, 1, 2) + ")  " +
   "F2 : (" + nf(globalPosX_F2, 1, 2) + ", " + nf(globalPosY_F2, 1, 2) + ", " +  nf(globalPosZ_F2, 1, 2) + ")  " +
   "F3 : (" + nf(globalPosX_F3, 1, 2) + ", " +  nf(globalPosY_F3, 1, 2) + ", " +  nf(globalPosZ_F3, 1, 2) + ")"
   );*/
  
  // Extend the line completely through the cube to the opposite side
  strokeWeight(3);
  stroke(255, 165, 0); // Cyan line
  line(contactPointF1.x, contactPointF1.y, contactPointF1.z, extendedExitPointF1.x, extendedExitPointF1.y, extendedExitPointF1.z);
  noStroke();
  
  strokeWeight(3);
  stroke(255, 165, 0); // Cyan line
  line(contactPointF2.x, contactPointF2.y, contactPointF2.z, extendedExitPointF2.x, extendedExitPointF2.y, extendedExitPointF2.z);
  noStroke();
  
  strokeWeight(3);
  stroke(255, 165, 0); // Cyan line
  line(contactPointF3.x, contactPointF3.y, contactPointF3.z, extendedExitPointF3.x, extendedExitPointF3.y, extendedExitPointF3.z);
  noStroke();
  
  popMatrix(); // end of camera transform

  // 4) Draw red spheres at the computed global positions (on the box)
  /*pushMatrix();
   fill(255, 0, 0);
   translate(globalPosX_F1, globalPosY_F1, globalPosZ_F1);
   sphere(5);
   popMatrix();
   
   pushMatrix();
   fill(255, 0, 0);
   translate(globalPosX_F2, globalPosY_F2, globalPosZ_F2);
   sphere(5);
   popMatrix();
   
   pushMatrix();
   fill(255, 0, 0);
   translate(globalPosX_F3, globalPosY_F3, globalPosZ_F3);
   sphere(5);
   popMatrix();*/

  // Suppose you have a global point (for example, from model() without transforms)

  localPointF1  = convertGlobalToMainAxes(globalPosX_F2, globalPosY_F2, globalPosZ_F2);
  globalPosX_F2 = localPointF1[0];
  globalPosY_F2 = localPointF1[1];
  globalPosZ_F2 = localPointF1[2];

  localPointF2  = convertGlobalToMainAxes(globalPosX_F1, globalPosY_F1, globalPosZ_F1);
  globalPosX_F1 = localPointF2[0];
  globalPosY_F1 = localPointF2[1];
  globalPosZ_F1 = localPointF2[2];

  localPointF3  = convertGlobalToMainAxes(globalPosX_F3, globalPosY_F3, globalPosZ_F3);
  globalPosX_F3 = localPointF3[0];
  globalPosY_F3 = localPointF3[1];
  globalPosZ_F3 = localPointF3[2];
  
  contactPointF1.set(localPointF1[0],localPointF1[1],localPointF1[2]);
  contactPointF2.set(localPointF2[0],localPointF2[1],localPointF2[2]);
  contactPointF3.set(localPointF3[0],localPointF3[1],localPointF3[2]);
  
  spherePosF1.set(sphereCenterF[0][0],sphereCenterF[0][1],sphereCenterF[0][2]);
  spherePosF2.set(sphereCenterF[1][0],sphereCenterF[1][1],sphereCenterF[1][2]);
  spherePosF3.set(sphereCenterF[2][0],sphereCenterF[2][1],sphereCenterF[2][2]);
  
  //float distance = PVector.dist(contactPointF1, spherePosF1);
  
  // Extend the shortest distance line through the cube to the opposite side
  directionF1 = PVector.sub(contactPointF1, spherePosF1).normalize();
  directionF1.z *= -1;
  directionF1.y *= -1;
  extendedExitPointF1 = PVector.add(contactPointF1, PVector.mult(directionF1, 55));
  
  directionF2 = PVector.sub(contactPointF2, spherePosF2).normalize();
  directionF2.z *= -1;
  directionF2.y *= -1;
  extendedExitPointF2 = PVector.add(contactPointF2, PVector.mult(directionF2, 55));
  
  directionF3 = PVector.sub(contactPointF3, spherePosF3).normalize();
  directionF3.z *= -1;
  directionF3.y *= -1;
  extendedExitPointF3 = PVector.add(contactPointF3, PVector.mult(directionF3, 55));
    
  if (traslationFlag == 1) { println(" FIXED-POINT MODE: Traslation required "); traslationFlag = 0;}
  if (rotationFlag == 1) { println(" ROLLING MODE: Rotation along end effector surface required "); rotationFlag = 0;}
  if (slidingFlag == 1) { println(" SLIDING MODE: Joint limits reached "); slidingFlag = 0; }    
    
  first_execution = false;
}

float[] convertGlobalToMainAxes(float gx, float gy, float gz) {
  // Create the camera transform (the same one used to draw your main axes)
  PMatrix3D camTransform = new PMatrix3D();
  camTransform.translate(width/2, height/2, 0);
  camTransform.rotateX(rotX);
  camTransform.rotateY(rotY);
  camTransform.rotateX(-PI/2);

  // Invert the camera transform so we can convert from global coordinates
  // to the main axes coordinate system.
  PMatrix3D invCamTransform = camTransform.get();
  invCamTransform.invert();

  // Build the global point vector in homogeneous coordinates.
  float[] globalPoint = { gx, gy, gz, 1 };
  float[] localPoint  = new float[4];

  // Multiply the global point by the inverted transform.
  invCamTransform.mult(globalPoint, localPoint);

  // For debugging: print the converted coordinates.
  /*println("Local (main axes) coordinates: "
   + nf(localPoint[0], 1, 2) + ", "
   + nf(localPoint[1], 1, 2) + ", "
   + nf(localPoint[2], 1, 2));*/

  return localPoint;
}


// --- The rest of your code (functions) remains the same ---

void forwardKinematicsFinger(float finger_x_offset, float finger_y_offset, float finger_z_offset,
  float finger_j1_angle, float finger_j2_angle, float finger_j3_angle, int fingerNum) {
  translate(finger_x_offset, finger_y_offset, finger_z_offset);

  matrix.set(
    cos(finger_j1_angle), -sin(finger_j1_angle), 0, fingerPt1_length * sin(finger_j1_angle),
    sin(finger_j1_angle), cos(finger_j1_angle), 0, -fingerPt1_length * cos(finger_j1_angle),
    0, 0, 1, 0,
    0, 0, 0, 1
    );
  applyMatrix(matrix);
  //drawMainAxes();
  /*println("L1: ");
   printCurrentMatrix();
   println("Joint 1 Position: (" + matrix.m03 + ", " + matrix.m13 + ", " + matrix.m23 + ")");*/

  matrix.set(
    1, 0, 0, 0,
    0, cos(finger_j2_angle), -sin(finger_j2_angle), -fingerPt2_length * cos(finger_j2_angle),
    0, sin(finger_j2_angle), cos(finger_j2_angle), -fingerPt2_length * sin(finger_j2_angle),
    0, 0, 0, 1
    );
  applyMatrix(matrix);
  //drawMainAxes();
  /*println("L2: ");
   printCurrentMatrix();
   println("Joint 2 Position: (" + matrix.m03 + ", " + matrix.m13 + ", " + matrix.m23 + ")");*/

  matrix.set(
    1, 0, 0, 0,
    0, cos(finger_j3_angle), -sin(finger_j3_angle), -fingerPt3_length * cos(finger_j3_angle),
    0, sin(finger_j3_angle), cos(finger_j3_angle), -fingerPt3_length * sin(finger_j3_angle),
    0, 0, 0, 1
    );
  applyMatrix(matrix);
  //drawMainAxes();
  /*println("L3: ");
   printCurrentMatrix();
   println("Joint 3 Position: (" + matrix.m03 + ", " + matrix.m13 + ", " + matrix.m23 + ")");*/

  matrix.set(
    1, 0, 0, 0,
    0, 1, 0, -fingertipCylinder_height,
    0, 0, 1, 0,
    0, 0, 0, 1
    );
  applyMatrix(matrix);
  //drawMainAxes();
  //println("Fingertip: ");
  //printCurrentMatrix();
  
  float mainAxesFPoints[] = convertGlobalToMainAxes(modelX(matrix.m03, matrix.m13, matrix.m23),modelY(matrix.m03, matrix.m13, matrix.m23),modelZ(matrix.m03, matrix.m13, matrix.m23));
  
  sphereCenterF[fingerNum - 1][0] = mainAxesFPoints[0];
  sphereCenterF[fingerNum - 1][1] = mainAxesFPoints[1];
  sphereCenterF[fingerNum - 1][2] = mainAxesFPoints[2];
  
  //println("Fingertip Position: "+ fingerNum+" (" + sphereCenterF[fingerNum - 1][0] + ", " + sphereCenterF[fingerNum - 1][1] + ", " + sphereCenterF[fingerNum - 1][2] + ")");

  matrix.set(
    cos(theta_y) * cos(theta_z),
    cos(theta_z) * sin(theta_y) * sin(theta_x) - sin(theta_z) * cos(theta_x),
    cos(theta_z) * sin(theta_y) * cos(theta_x) + sin(theta_z) * sin(theta_x),
    (cos(theta_z) * sin(theta_y) * sin(theta_x) - sin(theta_z) * cos(theta_x)) * (-sphere_radius-1),

    cos(theta_y) * sin(theta_z),
    sin(theta_z) * sin(theta_y) * sin(theta_x) + cos(theta_z) * cos(theta_x),
    sin(theta_z) * sin(theta_y) * cos(theta_x) - cos(theta_z) * sin(theta_x),
    (sin(theta_z) * sin(theta_y) * sin(theta_x) + cos(theta_z) * cos(theta_x)) * (-sphere_radius-1),

    -sin(theta_y),
    cos(theta_y) * sin(theta_x),
    cos(theta_y) * cos(theta_x),
    (cos(theta_y) * sin(theta_x)) * (-sphere_radius-1),

    0, 0, 0, 1
    );
  applyMatrix(matrix);
  //drawMainAxes();
  //println("Fingertip Contact: ");
  //printCurrentMatrix();
  //println("Final Contact Position: (" + matrix.m03 + ", " + matrix.m13 + ", " + matrix.m23 + ")");
}

void drawMainBase() {
  pushMatrix();
  fill(50, 50, 150);
  box(120, 250, 35);
  //println("BASE");
  //printCurrentMatrix();
  //drawSphereAt(0, 0, 0, 25);
  popMatrix();
}

void drawBasePt2() {
  pushMatrix();
  translate(0, -150, 15);
  rotateX(1.35);
  fill(50, 50, 150);
  box(85, 20, 135);
  //println("BASE PT2: ");
  //printCurrentMatrix();
  //drawSphereAt(0, 0, 0, 25);
  popMatrix();
}

void drawFinger(float xOffset, float yOffset, float zOffset,
  float joint1_rotation, float joint2_rotation, float joint3_rotation) {
  pushMatrix();
  translate(xOffset, yOffset, zOffset);
  fill(0, 255, 100);
  applyRotation(joint1_rotation, joint2_rotation, joint3_rotation);
  popMatrix();
}

void applyRotation(float joint1_rotation, float joint2_rotation, float joint3_rotation) {
  // Rotating Joint 1
  rotateZ(joint1_rotation);
  rotateX(-pi/2);
  drawCylinder(15, 32);
  rotateX(pi/2);
  fill(0, 100, 250);
  translate(0, -fingerPt1_length / 2, 0);
  box(30, 55, 30);
  translate(0, -50, 0);
  fill(0, 255, 100);

  // Rotating Joint 2
  translate(0, fingerPt2_length/2, 0);
  rotateZ(-pi/2);
  rotateY(joint2_rotation);
  drawCylinder(12, 26);
  rotateZ(pi/2);
  translate(0, -fingerPt2_length / 2, 0);
  fill(0, 100, 250);
  box(24, 50, 24);
  translate(0, -40, 0);

  // Rotating Joint 3
  translate(0, fingerPt3_length/2, 0);
  fill(0, 255, 100);
  rotateZ(-pi/2);
  rotateY(joint3_rotation);
  drawCylinder(9, 19);
  rotateZ(pi/2);
  translate(0, -fingerPt3_length/2, 0);
  fill(0, 100, 250);
  box(18, 40, 18);
  fill(255, 100, 50);
  translate(0, (-fingerPt3_length -fingertipCylinder_height - 5)/2, 0);
  drawCylinder(7, 15);
  translate(0, -7.5, 0);
  sphere(7);
}

void drawCylinder(float r, float h) {
  int detail = 20;
  float angleStep = TWO_PI / detail;
  beginShape(QUAD_STRIP);
  for (int i = 0; i <= detail; i++) {
    float angle = i * angleStep;
    float x = cos(angle) * r;
    float z = sin(angle) * r;
    vertex(x, -h / 2, z);
    vertex(x, h / 2, z);
  }
  endShape();

  beginShape(TRIANGLE_FAN);
  vertex(0, h / 2, 0);
  for (int i = 0; i <= detail; i++) {
    float angle = i * angleStep;
    float x = cos(angle) * r;
    float z = sin(angle) * r;
    vertex(x, h / 2, z);
  }
  endShape();

  beginShape(TRIANGLE_FAN);
  vertex(0, -h / 2, 0);
  for (int i = 0; i <= detail; i++) {
    float angle = i * angleStep;
    float x = cos(angle) * r;
    float z = sin(angle) * r;
    vertex(x, -h / 2, z);
  }
  endShape();
}

void printCurrentMatrix() {
  PGraphicsOpenGL pgl = (PGraphicsOpenGL) g;
  float[] mat = new float[16]; // 4x4 Matrix
  pgl.modelview.get(mat); // Get the current modelview matrix

  println("Current Transformation Matrix:");
  for (int i = 0; i < 4; i++) {
    println(mat[i*4] + "  " + mat[i*4+1] + "  " + mat[i*4+2] + "  " + mat[i*4+3]);
  }
  println(); // Add space for readability
}

void drawMainAxes() {
  if (drawAxes) {
    // Y-axis (Green)
    stroke(0, 255, 0);
    line(0, 0, 0, 0, 100, 0);

    // X-axis (Red)
    stroke(255, 0, 0);
    line(0, 0, 0, 100, 0, 0);

    // Z-axis (Blue)
    stroke(0, 0, 255);
    line(0, 0, 0, 0, 0, 100);
  }
  noStroke();
}

void drawLocalAxes() {
  if (drawAxes) {
    strokeWeight(1);
    // Local x-axis (yellow)
    stroke(255, 255, 0);
    line(0, 0, 0, 100, 0, 0);

    // Local y-axis (cyan)
    stroke(0, 255, 255);
    line(0, 0, 0, 0, 100, 0);

    // Local z-axis (magenta)
    stroke(255, 0, 255);
    line(0, 0, 0, 0, 0, 100);

    strokeWeight(1);
  }
  noStroke();
}

void mouseDragged() {
  float sensitivity = 0.01;
  rotY += (mouseX - pmouseX) * sensitivity;
  rotX -= (mouseY - pmouseY) * sensitivity;
}

// Simple 3-link inverse kinematics with two solution branches.
// Returns the valid solution for the specified finger (1,2,or 3)
// If no solution is within limits, angles are set to -1.
void inverseKinematics(float targetX, float targetY, float targetZ, float angles[], int fingerNum, float reach[]) {
  // Coordinate transformation (as in your original code)
  float temp = -targetY;
  targetY = -targetX;
  targetX = temp;

  // Check joint limits for each solution based on fingerNum.
  boolean sol1Valid = true;
  boolean sol2Valid = true;
  validBoxPos[fingerNum - 1] = 1;
  validFingerAngles[fingerNum - 1] = 1;

  // Link lengths
  float L1 = 50;    // first link length
  float L2 = 50;    // second link length
  float L3 = 54.5;  // third link (end-effector)

  // Compute theta1
  float theta1 = atan2(targetY, targetX);
  float cos1 = cos(theta1);
  float sin1 = sin(theta1);

  // Transform target to local coordinate system for the 2-link planar part
  float localX = cos1 * targetX + sin1 * targetY;
  // localY is computed but not used further in this IK formulation
  float localY = -sin1 * targetX + cos1 * targetY;

  localX -= L1;  // offset due to first link

  float d2 = localX * localX + targetZ * targetZ;  
  float d = sqrt(d2);
  reach[fingerNum - 1] = d;
  
  // 1) Check if the base angle is out of range (if you have a base-angle limit):
  float baseAngle = -theta1;  // your final sign convention
  // Suppose finger #1’s J1 has limits f1_j1_ang_botlim to f1_j1_ang_uplim:
  if (fingerNum == 1) {
    if (baseAngle < f1_j1_ang_botlim) {
      println("Finger 1 base angle too negative: " + baseAngle);
    } else if (baseAngle > f1_j1_ang_uplim) {
      println("Finger 1 base angle too positive: " + baseAngle);
    }
  }

  // 2) Check if localX is negative (if you don’t allow the arm to bend “behind” the pivot):
  if (localX < 0) {
    println("Target behind the first link pivot. localX = " + localX);
  }

  // 3) Check if distance d is larger than the max extension of links 2 & 3.
  float maxReach = L2 + L3;  // ignoring L1 if the pivot must start after L1
  if (d > maxReach) {
    println("Target too far for link2+link3: distance " + d + " > " + maxReach);
  }

  // 4) Check if distance d is smaller than the “minimum” extension of links 2 & 3 (if any).
  // For a typical 2-link chain, the minimum distance is |L2 - L3|.
  float minReach = abs(L2 - L3);
  if (d < minReach) {
    println("Target too close for link2+link3 to fold: distance " + d + " < " + minReach);
  }

  // Cosine of theta3 using the cosine law for the 2-link chain
  float D = (d2 - (L2 * L2 + L3 * L3)) / (2 * L2 * L3);
  if (D < -1 || D > 1) {
    println("Target out of reach");
    //angles[0] = 0;
    //angles[1] = 0;
    //angles[2] = 0;
    validBoxPos[fingerNum - 1] = 0;
    return;
  }

  // Compute both solutions for theta3
  float theta3_1 = acos(D);
  float theta3_2 = -acos(D);

  // Common intermediate angle phi for both solutions
  float phi = atan2(targetZ, localX);

  // First solution for theta2
  float alpha1 = atan2(L3 * sin(theta3_1), L2 + L3 * cos(theta3_1));
  float theta2_1 = phi - alpha1;

  // Second solution for theta2
  float alpha2 = atan2(L3 * sin(theta3_2), L2 + L3 * cos(theta3_2));
  float theta2_2 = phi - alpha2;

  // Convert angles to output convention (as in your original code)
  // Note: theta1 is the same for both solutions.
  sol1[0] = -theta1;
  sol1[1] = -theta2_1;
  sol1[2] = -theta3_1;

  sol2[0] = -theta1;
  sol2[1] = -theta2_2;
  sol2[2] = -theta3_2;

  checkJointLimits(sol1, sol2, fingerNum);

  // Choose which solution to use.
  if (sol1Valid && fingerNum != 3) {
    angles[0] = sol1[0];
    angles[1] = sol1[1];
    angles[2] = sol1[2];
    if (sol1Clamped == 1) slidingFlag = 1;
  } else if (sol2Valid  && fingerNum == 3) {
    angles[0] = sol2[0];
    angles[1] = sol2[1];
    angles[2] = sol2[2];
    if (sol2Clamped == 1) slidingFlag = 1;
  } else if (sol2Valid) {
    angles[0] = sol2[0];
    angles[1] = sol2[1];
    angles[2] = sol2[2];
    if (sol2Clamped == 1) slidingFlag = 1;
  } else {
    println("INVALID ANGLES F" +fingerNum +" J1: " + angles[0] + " J2: " + angles[1] + " J3: " + angles[2]);
    /*if (fingerNum == 3) {
      angles[0] = sol2[0];
      angles[1] = sol2[1];
      angles[2] = sol2[2];
    } else {
      angles[0] = sol1[0];
      angles[1] = sol1[1];
      angles[2] = sol1[2];
    }*/
    slidingFlag = 1;
    validFingerAngles[fingerNum - 1] = 0;
  }
  sol1Clamped = 0;
  sol2Clamped = 0;
}

// Key handling
void keyPressed() {
  if (key == 'w' || key == 'W') wPressed = true;
  if (key == 's' || key == 'S') sPressed = true;
  if (key == 'a' || key == 'A') aPressed = true;
  if (key == 'd' || key == 'D') dPressed = true;
  if (key == 'q' || key == 'Q') qPressed = true;
  if (key == 'e' || key == 'E') ePressed = true;

  if (key == 'r' || key == 'R') rPressed = true;
  if (key == 'f' || key == 'F') fPressed = true;
  if (key == 't' || key == 'T') tPressed = true;
  if (key == 'g' || key == 'G') gPressed = true;
  if (key == 'y' || key == 'Y') yPressed = true;
  if (key == 'h' || key == 'H') hPressed = true;
}

void keyReleased() {
  if (key == 'w' || key == 'W') wPressed = false;
  if (key == 's' || key == 'S') sPressed = false;
  if (key == 'a' || key == 'A') aPressed = false;  
  if (key == 'd' || key == 'D') dPressed = false;
  if (key == 'q' || key == 'Q') qPressed = false;
  if (key == 'e' || key == 'E') ePressed = false;

  if (key == 'r' || key == 'R') rPressed = false;
  if (key == 'f' || key == 'F') fPressed = false;
  if (key == 't' || key == 'T') tPressed = false;
  if (key == 'g' || key == 'G') gPressed = false;
  if (key == 'y' || key == 'Y') yPressed = false;
  if (key == 'h' || key == 'H') hPressed = false;
}


void checkJointLimits(float sol1[], float sol2[], int fingerNum) {
    
    if (fingerNum == 1) {
    // --- SOLUTION 1 checks ---
    if (sol1[0] > f1_j1_ang_uplim) {
      println(millis()/1000.0 + " Finger 1, Sol1, J1 above upper limit: " + sol1[0] + " clamped to " + f1_j1_ang_uplim);
      sol1[0] = f1_j1_ang_uplim; sol1Clamped = 1;
    }
    else if (sol1[0] < f1_j1_ang_botlim) {
      println(millis()/1000.0 + " Finger 1, Sol1, J1 below lower limit: " + sol1[0] + " clamped to " + f1_j1_ang_botlim);
      sol1[0] = f1_j1_ang_botlim; sol1Clamped = 1;
    }
    if (sol1[1] > f1_j2_ang_uplim) {
      println(millis()/1000.0 + " Finger 1, Sol1, J2 above upper limit: " + sol1[1] + " clamped to " + f1_j2_ang_uplim);
      sol1[1] = f1_j2_ang_uplim; sol1Clamped = 1;
    }
    else if (sol1[1] < f1_j2_ang_botlim) {
      println(millis()/1000.0 + " Finger 1, Sol1, J2 below lower limit: " + sol1[1] + " clamped to " + f1_j2_ang_botlim);
      sol1[1] = f1_j2_ang_botlim; sol1Clamped = 1;
    }
    if (sol1[2] > f1_j3_ang_uplim) {
      println(millis()/1000.0 + " Finger 1, Sol1, J3 above upper limit: " + sol1[2] + " clamped to " + f1_j3_ang_uplim);
      sol1[2] = f1_j3_ang_uplim; sol1Clamped = 1;
    }
    else if (sol1[2] < f1_j3_ang_botlim) {
      println(millis()/1000.0 + " Finger 1, Sol1, J3 below lower limit: " + sol1[2] + " clamped to " + f1_j3_ang_botlim);
      sol1[2] = f1_j3_ang_botlim; sol1Clamped = 1;
    }
  
    // --- SOLUTION 2 checks ---
    if (sol2[0] > f1_j1_ang_uplim) {
      println(millis()/1000.0 + " Finger 1, Sol2, J1 above upper limit: " + sol2[0] + " clamped to " + f1_j1_ang_uplim);
      sol2[0] = f1_j1_ang_uplim; sol2Clamped = 1;
    }
    else if (sol2[0] < f1_j1_ang_botlim) {
      println(millis()/1000.0 + " Finger 1, Sol2, J1 below lower limit: " + sol2[0] + " clamped to " + f1_j1_ang_botlim);
      sol2[0] = f1_j1_ang_botlim; sol2Clamped = 1;
    }
    if (sol2[1] > f1_j2_ang_uplim) {
      println(millis()/1000.0 + " Finger 1, Sol2, J2 above upper limit: " + sol2[1] + " clamped to " + f1_j2_ang_uplim);
      sol2[1] = f1_j2_ang_uplim; sol2Clamped = 1;
    }
    else if (sol2[1] < f1_j2_ang_botlim) {
      println(millis()/1000.0 + " Finger 1, Sol2, J2 below lower limit: " + sol2[1] + " clamped to " + f1_j2_ang_botlim);
      sol2[1] = f1_j2_ang_botlim; sol2Clamped = 1;
    }
    if (sol2[2] > f1_j3_ang_uplim) {
      println(millis()/1000.0 + " Finger 1, Sol2, J3 above upper limit: " + sol2[2] + " clamped to " + f1_j3_ang_uplim);
      sol2[2] = f1_j3_ang_uplim; sol2Clamped = 1;
    }
    else if (sol2[2] < f1_j3_ang_botlim) {
      println(millis()/1000.0 + " Finger 1, Sol2, J3 below lower limit: " + sol2[2] + " clamped to " + f1_j3_ang_botlim);
      sol2[2] = f1_j3_ang_botlim; sol2Clamped = 1;
    }
  }
  else if (fingerNum == 2) {
    // --- SOLUTION 1 checks ---
    if (sol1[0] > f2_j1_ang_uplim) {
      println(millis()/1000.0 + " Finger 2, Sol1, J1 above upper limit: " + sol1[0] + " clamped to " + f2_j1_ang_uplim);
      sol1[0] = f2_j1_ang_uplim; sol1Clamped = 1;
    }
    else if (sol1[0] < f2_j1_ang_botlim) {
      println(millis()/1000.0 + " Finger 2, Sol1, J1 below lower limit: " + sol1[0] + " clamped to " + f2_j1_ang_botlim);
      sol1[0] = f2_j1_ang_botlim; sol1Clamped = 1;
    }
    if (sol1[1] > f2_j2_ang_uplim) {
      println(millis()/1000.0 + " Finger 2, Sol1, J2 above upper limit: " + sol1[1] + " clamped to " + f2_j2_ang_uplim);
      sol1[1] = f2_j2_ang_uplim; sol1Clamped = 1;
    }
    else if (sol1[1] < f2_j2_ang_botlim) {
      println(millis()/1000.0 + " Finger 2, Sol1, J2 below lower limit: " + sol1[1] + " clamped to " + f2_j2_ang_botlim);
      sol1[1] = f2_j2_ang_botlim; sol1Clamped = 1;
    }
    if (sol1[2] > f2_j3_ang_uplim) {
      println(millis()/1000.0 + " Finger 2, Sol1, J3 above upper limit: " + sol1[2] + " clamped to " + f2_j3_ang_uplim);
      sol1[2] = f2_j3_ang_uplim; sol1Clamped = 1;
    }
    else if (sol1[2] < f2_j3_ang_botlim) {
      println(millis()/1000.0 + " Finger 2, Sol1, J3 below lower limit: " + sol1[2] + " clamped to " + f2_j3_ang_botlim);
      sol1[2] = f2_j3_ang_botlim; sol1Clamped = 1;
    }
  
    // --- SOLUTION 2 checks ---
    if (sol2[0] > f2_j1_ang_uplim) {
      println(millis()/1000.0 + " Finger 2, Sol2, J1 above upper limit: " + sol2[0] + " clamped to " + f2_j1_ang_uplim);
      sol2[0] = f2_j1_ang_uplim; sol2Clamped = 1;
    }
    else if (sol2[0] < f2_j1_ang_botlim) {
      println(millis()/1000.0 + " Finger 2, Sol2, J1 below lower limit: " + sol2[0] + " clamped to " + f2_j1_ang_botlim);
      sol2[0] = f2_j1_ang_botlim; sol2Clamped = 1;
    }
    if (sol2[1] > f2_j2_ang_uplim) {
      println(millis()/1000.0 + " Finger 2, Sol2, J2 above upper limit: " + sol2[1] + " clamped to " + f2_j2_ang_uplim);
      sol2[1] = f2_j2_ang_uplim; sol2Clamped = 1;
    }
    else if (sol2[1] < f2_j2_ang_botlim) {
      println(millis()/1000.0 + " Finger 2, Sol2, J2 below lower limit: " + sol2[1] + " clamped to " + f2_j2_ang_botlim);
      sol2[1] = f2_j2_ang_botlim; sol2Clamped = 1;
    }
    if (sol2[2] > f2_j3_ang_uplim) {
      println(millis()/1000.0 + " Finger 2, Sol2, J3 above upper limit: " + sol2[2] + " clamped to " + f2_j3_ang_uplim);
      sol2[2] = f2_j3_ang_uplim; sol2Clamped = 1;
    }
    else if (sol2[2] < f2_j3_ang_botlim) {
      println(millis()/1000.0 + " Finger 2, Sol2, J3 below lower limit: " + sol2[2] + " clamped to " + f2_j3_ang_botlim);
      sol2[2] = f2_j3_ang_botlim; sol2Clamped = 1;
    }
  }
  else if (fingerNum == 3) {
    // --- SOLUTION 1 checks ---
    if (sol1[0] > f3_j1_ang_uplim) {
      println(millis()/1000.0 + " Finger 3, Sol1, J1 above upper limit: " + sol1[0] + " clamped to " + f3_j1_ang_uplim);
      sol1[0] = f3_j1_ang_uplim; sol1Clamped = 1;
    }
    else if (sol1[0] < f3_j1_ang_botlim) {
      println(millis()/1000.0 + " Finger 3, Sol1, J1 below lower limit: " + sol1[0] + " clamped to " + f3_j1_ang_botlim);
      sol1[0] = f3_j1_ang_botlim; sol1Clamped = 1;
    }
    if (sol1[1] > f3_j2_ang_uplim) {
      println(millis()/1000.0 + " Finger 3, Sol1, J2 above upper limit: " + sol1[1] + " clamped to " + f3_j2_ang_uplim);
      sol1[1] = f3_j2_ang_uplim; sol1Clamped = 1;
    }
    else if (sol1[1] < f3_j2_ang_botlim) {
      println(millis()/1000.0 + " Finger 3, Sol1, J2 below lower limit: " + sol1[1] + " clamped to " + f3_j2_ang_botlim);
      sol1[1] = f3_j2_ang_botlim; sol1Clamped = 1;
    }
    if (sol1[2] > f3_j3_ang_uplim) {
      println(millis()/1000.0 + " Finger 3, Sol1, J3 above upper limit: " + sol1[2] + " clamped to " + f3_j3_ang_uplim);
      sol1[2] = f3_j3_ang_uplim; sol1Clamped = 1;
    }
    else if (sol1[2] < f3_j3_ang_botlim) {
      println(millis()/1000.0 + " Finger 3, Sol1, J3 below lower limit: " + sol1[2] + " clamped to " + f3_j3_ang_botlim);
      sol1[2] = f3_j3_ang_botlim; sol1Clamped = 1;
    }
  
    // --- SOLUTION 2 checks ---
    if (sol2[0] > f3_j1_ang_uplim) {
      println(millis()/1000.0 + " Finger 3, Sol2, J1 above upper limit: " + sol2[0] + " clamped to " + f3_j1_ang_uplim);
      sol2[0] = f3_j1_ang_uplim; sol2Clamped = 1;
    }
    else if (sol2[0] < f3_j1_ang_botlim) {
      println(millis()/1000.0 + " Finger 3, Sol2, J1 below lower limit: " + sol2[0] + " clamped to " + f3_j1_ang_botlim);
      sol2[0] = f3_j1_ang_botlim; sol2Clamped = 1;
    }
    if (sol2[1] > f3_j2_ang_uplim) {
      println(millis()/1000.0 + " Finger 3, Sol2, J2 above upper limit: " + sol2[1] + " clamped to " + f3_j2_ang_uplim);
      sol2[1] = f3_j2_ang_uplim; sol2Clamped = 1;
    }
    else if (sol2[1] < f3_j2_ang_botlim) {
      println(millis()/1000.0 + " Finger 3, Sol2, J2 below lower limit: " + sol2[1] + " clamped to " + f3_j2_ang_botlim);
      sol2[1] = f3_j2_ang_botlim; sol2Clamped = 1;
    }
    if (sol2[2] > f3_j3_ang_uplim) {
      println(millis()/1000.0 + " Finger 3, Sol2, J3 above upper limit: " + sol2[2] + " clamped to " + f3_j3_ang_uplim);
      sol2[2] = f3_j3_ang_uplim; sol2Clamped = 1;
    }
    else if (sol2[2] < f3_j3_ang_botlim) {
      println(millis()/1000.0 + " Finger 3, Sol2, J3 below lower limit: " + sol2[2] + " clamped to " + f3_j3_ang_botlim);
      sol2[2] = f3_j3_ang_botlim; sol2Clamped = 1;
    }
  }

} 



