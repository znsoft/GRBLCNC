
/*
  Mini CNC Plotter firmware, based in TinyCNC https://github.com/MakerBlock/TinyCNC-Sketches
  Send GCODE to this Sketch using gctrl.pde https://github.com/damellis/gctrl
  Convert SVG to GCODE with MakerBot Unicorn plugin for Inkscape available here https://github.com/martymcguire/inkscape-unicorn

  More information about the Mini CNC Plotter here (german, sorry): http://www.makerblog.at/2015/02/projekt-mini-cnc-plotter-aus-alten-cddvd-laufwerken/
*/

#include <Servo.h>
#include <AFMotor.h>

#define LINE_BUFFER_LENGTH 512

char STEP = MICROSTEP ;

// Servo position for Up and Down
const int penZUp = 120;
const int penZDown = 19;
int laserwait = 10;
// Servo on PWM pin 10
const int penServoPin = 10 ;
bool isPenDown = false;
// Should be right for DVD steppers, but is not too important here
const int stepsPerRevolution = 48;

// create servo object to control a servo
Servo penServo;

// Initialize steppers for X- and Y-axis using this Arduino pins for the L293D H-bridge
AF_Stepper myStepperY(stepsPerRevolution, 1);
AF_Stepper myStepperX(stepsPerRevolution, 2);

/* Structures, global variables    */
struct point {
  float x;
  float y;
  float z;
};

// Current position of plothead
struct point actuatorPos;

//  Drawing settings, should be OK
float StepInc = 1;
int StepDelayOff = 2;//for fine step
int StepDelayOn = 3;
int LineDelay = 0;
int penDelay = 0;

// Motor steps to go 1 millimeter.
// Use test sketch to go 100 steps. Measure the length of line.
// Calculate steps per mm. Enter here.
float StepsPerMillimeterX = 118.0;
float StepsPerMillimeterY = 118.0;

// Drawing robot limits, in mm
// OK to start with. Could go up to 50 mm if calibrated well.
float Xmin = 0;
float Xmax = 40;
float Ymin = 0;
float Ymax = 40;
float Zmin = 0;
float Zmax = 1;
float scale = 1;
float Xpos = Xmin;
float Ypos = Ymin;
float Zpos = Zmax;

// Set to true to get debug output.
boolean verbose = false;

//  Needs to interpret
//  G1 for moving
//  G4 P300 (wait 150ms)
//  M300 S30 (pen down)
//  M300 S50 (pen up)
//  Discard anything with a (
//  Discard any other command!

/**********************
   void setup() - Initialisations
 ***********************/
void setup() {
  //  Setup

  Serial.begin( 9600 );
  pinMode(13, OUTPUT);
  penServo.attach(penServoPin);
  //penServo.write(penZUp);
  delay(100);

  // Decrease if necessary
  myStepperX.setSpeed(600);

  myStepperY.setSpeed(600);


  //  Set & move to initial default position
  // TBD

  //  Notifications!!!
  Serial.println("Mini CNC Plotter alive and kicking!");
  Serial.print("X range is from ");
  Serial.print(Xmin);
  Serial.print(" to ");
  Serial.print(Xmax);
  Serial.println(" mm.");
  Serial.print("Y range is from ");
  Serial.print(Ymin);
  Serial.print(" to ");
  Serial.print(Ymax);
  Serial.println(" mm.");
}

/**********************
   void loop() - Main loop
 ***********************/
void loop()
{

  delay(100);
  char line[ LINE_BUFFER_LENGTH ];
  char c;
  int lineIndex;
  bool lineIsComment, lineSemiColon;

  lineIndex = 0;
  lineSemiColon = false;
  lineIsComment = false;

  while (1) {

    // Serial reception - Mostly from Grbl, added semicolon support
    while ( Serial.available() > 0 ) {
      c = Serial.read();
      if (( c == '\n') || (c == '\r') ) {             // End of line reached
        if ( lineIndex > 0 ) {                        // Line is complete. Then execute!
          line[ lineIndex ] = '\0';                   // Terminate string
          if (verbose) {
            Serial.print( "Received : ");
            Serial.println( line );
          }
          processIncomingLine( line, lineIndex );
          lineIndex = 0;
        }
        else {
          // Empty or comment line. Skip block.
        }
        lineIsComment = false;
        lineSemiColon = false;
        Serial.println("ok");
      }
      else {
        if ( (lineIsComment) || (lineSemiColon) ) {   // Throw away all comment characters
          if ( c == ')' )  lineIsComment = false;     // End of comment. Resume line.
        }
        else {
          //if ( c <= ' ' ) {                           // Throw away whitepace and control characters
          //}else
          if ( c == '/' ) {                    // Block delete not supported. Ignore character.
          }
          else if ( c == '(' ) {                    // Enable comments flag and ignore all characters until ')' or EOL.
            lineIsComment = true;
          }
          else if ( c == ';' ) {
            lineSemiColon = true;
          }
          else if ( lineIndex >= LINE_BUFFER_LENGTH - 1 ) {
            Serial.println( "ERROR - lineBuffer overflow" );
            lineIsComment = false;
            lineSemiColon = false;
          }
          else if ( c >= 'a' && c <= 'z' ) {        // Upcase lowercase
            line[ lineIndex++ ] = c - 'a' + 'A';
          }
          else {
            line[ lineIndex++ ] = c;
          }
        }
      }
    }
  }
}


float getFloatParam(char* line, char param, float defaultValue) {
  char* index = strchr( line, param );
  if (index <= 0)return defaultValue;

  char* indexSpace = strchr( index, ' ' );
  indexSpace = '\0';
  return atof( index + 1);


}

int getIntParam(char* line, char param, int defaultValue) {
  char* index = strchr( line, param );
  if (verbose)Serial.println(index);
  if (index <= 0)return defaultValue;

  char* indexSpace = strchr( index, ' ' );
  indexSpace = '\0';
  return atoi( index + 1);


}


void setZPos(float z) {
  int iZ = round(z);
  if (iZ < penZDown)iZ = penZDown;
  if (iZ > penZUp) iZ = penZUp;
  penServo.write(iZ);

}

void processArc( struct point newPos, char* line,  int currentIndex, bool isCCW) {
  newPos.x = getFloatParam(line + currentIndex, 'X', actuatorPos.x) * scale;
  newPos.y = getFloatParam(line + currentIndex, 'Y', actuatorPos.y) * scale;
  newPos.z = getFloatParam(line + currentIndex, 'Z', actuatorPos.z) * scale;
  float I = getFloatParam(line + currentIndex, 'I', 0.0) * scale;
  float J = getFloatParam(line + currentIndex, 'J', 0.0) * scale;
  float R = getFloatParam(line + currentIndex, 'R', 0.0) * scale;
  setZPos(newPos.z);
  drawarc(newPos.x, newPos.y, I, J, R, isCCW );
  //        Serial.println("ok");
  actuatorPos.x = newPos.x;
  actuatorPos.y = newPos.y;
  actuatorPos.z = newPos.z;

}

void processLinear( struct point newPos, char* line,  int currentIndex) {
  newPos.x = getFloatParam(line + currentIndex, 'X', actuatorPos.x) * scale;
  newPos.y = getFloatParam(line + currentIndex, 'Y', actuatorPos.y) * scale;
  newPos.z = getFloatParam(line + currentIndex, 'Z', actuatorPos.z) * scale;
  setZPos(newPos.z);
  drawLine(newPos.x, newPos.y );

  actuatorPos.x = newPos.x;
  actuatorPos.y = newPos.y;
  actuatorPos.z = newPos.z;
}

void processIncomingLine( char* line, int charNB ) {
  int currentIndex = 0, g;
  char* indexSpace;
  char buffer[ 64 ];                                 // Hope that 64 is enough for 1 parameter
  struct point newPos;

  newPos.x = 0.0;
  newPos.y = 0.0;

  //  Needs to interpret
  //  G1 for moving
  //  G4 P300 (wait 150ms)
  //  G1 X60 Y30
  //  G1 X30 Y50
  //  M300 S30 (pen down)
  //  M300 S50 (pen up)
  //  Discard anything with a (
  //  Discard any other command!

  while ( currentIndex < charNB ) {
    switch ( line[ currentIndex++ ] ) {              // Select command, if any
      case 'U':
        penUp();
        break;
      case 'D':
        if (verbose)
          Serial.println("D");
        penDown();
        break;
      case 'G':

        indexSpace = strchr( line + currentIndex, ' ' );
        indexSpace = '\0';
        g = atoi( line + currentIndex );

        if (verbose)Serial.println(g);
        switch (g  ) {                  // Select G command

          case 0:                                   // G00 & G01 - Movement or movement with laser
            if (verbose)
              Serial.println("G0");

            penUp();
            processLinear( newPos, line,   currentIndex);
            break;

          case 1:
            if (verbose)
              Serial.println("G1");
            penDown();
            processLinear( newPos, line,   currentIndex);
            penUp();
            break;

          case 2:
            if (verbose)
              Serial.println("G2");
            penDown();
            processArc( newPos, line,   currentIndex, true);
            penUp();
            break;

          case 3:
            if (verbose)
              Serial.println("G3");

            penDown();
            processArc( newPos, line,   currentIndex, false);
            penUp();
            break;

          case 4:
            delay(getIntParam(line + currentIndex, 'P', 0));
            break;

          case 23:
            scale = getFloatParam(line + currentIndex, 'S', scale);
            break;
        }
        break;
      case 'T':
        buffer[0] = line[ currentIndex++ ];        // /!\ Dirty - Only works with 3 digit commands
        buffer[1] = line[ currentIndex++ ];
        buffer[2] = line[ currentIndex++ ];
        buffer[3] = '\0';
        StepDelayOn = atoi( buffer );


        break;
      case 'M':
        buffer[0] = line[ currentIndex++ ];        // /!\ Dirty - Only works with 3 digit commands
        buffer[1] = line[ currentIndex++ ];
        buffer[2] = line[ currentIndex++ ];
        buffer[3] = '\0';
        switch ( atoi( buffer ) ) {
          case 300:
            {
              char* indexS = strchr( line + currentIndex, 'S' );
              float Spos = atof( indexS + 1);
              //         Serial.println("ok");
              if (Spos == 30) {
                penDown();
              }
              if (Spos == 50) {
                penUp();
              }
              break;
            }
          case 4:
          case 3:
            penDown();
            break;
          case 5:
            penUp();
            break;
          case 114:                                // M114 - Repport position
            Serial.print( "Absolute position : X = " );
            Serial.print( actuatorPos.x );
            Serial.print( "  -  Y = " );
            Serial.println( actuatorPos.y );
            break;
          default:
            Serial.print( "Command not recognized : M");
            Serial.println( buffer );
        }
    }
  }



}

void StepDalay() {
  delay(isPenDown ? StepDelayOn : StepDelayOff);

}

/*********************************
   Draw a line from (x0;y0) to (x1;y1).
   Bresenham algo from https://www.marginallyclever.com/blog/2013/08/how-to-build-an-2-axis-arduino-cnc-gcode-interpreter/
   int (x1;y1) : Starting coordinates
   int (x2;y2) : Ending coordinates
 **********************************/
void drawLine(float x1, float y1) {

  if (verbose)
  {
    Serial.print("fx1, fy1: ");
    Serial.print(x1);
    Serial.print(",");
    Serial.print(y1);
    Serial.println("");
  }

  //  Bring instructions within limits
  if (x1 >= Xmax) {
    x1 = Xmax;
  }
  if (x1 <= Xmin) {
    x1 = Xmin;
  }
  if (y1 >= Ymax) {
    y1 = Ymax;
  }
  if (y1 <= Ymin) {
    y1 = Ymin;
  }

  if (verbose)
  {
    Serial.print("Xpos, Ypos: ");
    Serial.print(Xpos);
    Serial.print(",");
    Serial.print(Ypos);
    Serial.println("");
  }

  if (verbose)
  {
    Serial.print("x1, y1: ");
    Serial.print(x1);
    Serial.print(",");
    Serial.print(y1);
    Serial.println("");
  }

  //  Convert coordinates to steps
  x1 = (int)(x1 * StepsPerMillimeterX);
  y1 = (int)(y1 * StepsPerMillimeterY);
  float x0 = Xpos;
  float y0 = Ypos;

  //  Let's find out the change for the coordinates
  long dx = abs(x1 - x0);
  long dy = abs(y1 - y0);
  int sx = x0 < x1 ? StepInc : -StepInc;
  int sy = y0 < y1 ? StepInc : -StepInc;

  long i;
  long over = 0;

  if (dx > dy) {
    for (i = 0; i < dx; ++i) {
      myStepperX.onestep(sx, STEP);
      over += dy;
      if (over >= dx) {
        over -= dx;
        myStepperY.onestep(sy, STEP);
      }
      StepDalay();
    }
  }
  else {
    for (i = 0; i < dy; ++i) {
      myStepperY.onestep(sy, STEP);
      over += dx;
      if (over >= dy) {
        over -= dy;
        myStepperX.onestep(sx, STEP);
      }
      StepDalay();
    }
  }

  if (verbose)
  {
    Serial.print("dx, dy:");
    Serial.print(dx);
    Serial.print(",");
    Serial.print(dy);
    Serial.println("");
  }

  if (verbose)
  {
    Serial.print("Going to (");
    Serial.print(x0);
    Serial.print(",");
    Serial.print(y0);
    Serial.println(")");
  }

  //  Delay before any next lines are submitted
  // delay(LineDelay);
  //  Update the positions
  Xpos = x1;
  Ypos = y1;
}

//  Raises pen
void penUp() {
  isPenDown = false;
  //penServo.write(penZUp);
  //delay(penDelay);
  //Zpos = Zmax;
  digitalWrite(15, LOW);
  digitalWrite(13, LOW);
  digitalWrite(16, HIGH);
  if (verbose) {
    Serial.println("Pen up!");
  }
}
//  Lowers pen
void penDown() {
  isPenDown = true;
  //penServo.write(penZDown);
  //delay(penDelay);
  //Zpos = Zmin;
  digitalWrite(15, HIGH);
  digitalWrite(13, HIGH);
  //delay(laserwait);
  digitalWrite(16, LOW);
  if (verbose) {
    Serial.println("Pen down.");
  }
}

#define ARC_ANGULAR_TRAVEL_EPSILON 5E-7
#define DEFAULT_ARC_TOLERANCE 0.002
#define N_ARC_CORRECTION 12
// Execute an arc in offset mode format. position == current xyz, target == target xyz,
// offset == offset from current xyz, axis_X defines circle plane in tool space, axis_linear is
// the direction of helical travel, radius == circle radius, isclockwise boolean. Used
// for vector transformation direction.
// The arc is approximated by generating a huge number of tiny, linear segments. The chordal tolerance
// of each segment is configured in settings.arc_tolerance, which is defined to be the maximum normal
// distance from segment to the circle when the end points both lie on the circle.
void drawarc( float xt, float yt, float xo, float yo, float radius, bool is_clockwise_arc)
{
  float center_axis0 = actuatorPos.x + xo;
  float center_axis1 = actuatorPos.y + yo;
  float r_axis0 = -xo;  // Radius vector from center to current location
  float r_axis1 = -yo;
  float rt_axis0 = xt - center_axis0;
  float rt_axis1 = yt - center_axis1;
  if (radius == 0.0) radius = sqrt(r_axis0 * r_axis0 + r_axis1 * r_axis1);
  // CCW angle between position and target from circle center. Only one atan2() trig computation required.
  float angular_travel = atan2(r_axis0 * rt_axis1 - r_axis1 * rt_axis0, r_axis0 * rt_axis0 + r_axis1 * rt_axis1);
  if (is_clockwise_arc) { // Correct atan2 output per direction
    if (angular_travel >= -ARC_ANGULAR_TRAVEL_EPSILON) {
      angular_travel -= 2 * M_PI;
    }
  } else {
    if (angular_travel <= ARC_ANGULAR_TRAVEL_EPSILON) {
      angular_travel += 2 * M_PI;
    }
  }

  // NOTE: Segment end points are on the arc, which can lead to the arc diameter being smaller by up to
  // (2x) settings.arc_tolerance. For 99% of users, this is just fine. If a different arc segment fit
  // is desired, i.e. least-squares, midpoint on arc, just change the mm_per_arc_segment calculation.
  // For the intended uses of Grbl, this value shouldn't exceed 2000 for the strictest of cases.
  uint16_t segments = floor(fabs(0.5 * angular_travel * radius) /
                            sqrt(DEFAULT_ARC_TOLERANCE * (2 * radius - DEFAULT_ARC_TOLERANCE)) );
  if (verbose)
  {
    Serial.print("segments: ");
    Serial.print(segments);
    Serial.print(" , angular_travel:");
    Serial.print(angular_travel);
    Serial.println("");
  }
  if (segments) {
    // Multiply inverse feed_rate to compensate for the fact that this movement is approximated
    // by a number of discrete segments. The inverse feed_rate should be correct for the sum of
    // all segments.

    float theta_per_segment = angular_travel / segments;
    float linear_per_segment = ( - Zpos) / segments;

    /* Vector rotation by transformation matrix: r is the original vector, r_T is the rotated vector,
       and phi is the angle of rotation. Solution approach by Jens Geisler.
           r_T = [cos(phi) -sin(phi);
                  sin(phi)  cos(phi] * r ;

       For arc generation, the center of the circle is the axis of rotation and the radius vector is
       defined from the circle center to the initial position. Each line segment is formed by successive
       vector rotations. Single precision values can accumulate error greater than tool precision in rare
       cases. So, exact arc path correction is implemented. This approach avoids the problem of too many very
       expensive trig operations [sin(),cos(),tan()] which can take 100-200 usec each to compute.

       Small angle approximation may be used to reduce computation overhead further. A third-order approximation
       (second order sin() has too much error) holds for most, if not, all CNC applications. Note that this
       approximation will begin to accumulate a numerical drift error when theta_per_segment is greater than
       ~0.25 rad(14 deg) AND the approximation is successively used without correction several dozen times. This
       scenario is extremely unlikely, since segment lengths and theta_per_segment are automatically generated
       and scaled by the arc tolerance setting. Only a very large arc tolerance setting, unrealistic for CNC
       applications, would cause this numerical drift error. However, it is best to set N_ARC_CORRECTION from a
       low of ~4 to a high of ~20 or so to avoid trig operations while keeping arc generation accurate.

       This approximation also allows mc_arc to immediately insert a line segment into the planner
       without the initial overhead of computing cos() or sin(). By the time the arc needs to be applied
       a correction, the planner should have caught up to the lag caused by the initial mc_arc overhead.
       This is important when there are successive arc motions.
    */
    // Computes: cos_T = 1 - theta_per_segment^2/2, sin_T = theta_per_segment - theta_per_segment^3/6) in ~52usec
    float cos_T = 2.0 - theta_per_segment * theta_per_segment;
    float sin_T = theta_per_segment * 0.16666667 * (cos_T + 4.0);
    cos_T *= 0.5;

    float sin_Ti;
    float cos_Ti;
    float r_axisi;
    uint16_t i;
    uint8_t count = 0;

    for (i = 1; i < segments; i++) { // Increment (segments-1).

      if (count < N_ARC_CORRECTION) {
        // Apply vector rotation matrix. ~40 usec
        r_axisi = r_axis0 * sin_T + r_axis1 * cos_T;
        r_axis0 = r_axis0 * cos_T - r_axis1 * sin_T;
        r_axis1 = r_axisi;
        count++;
      } else {
        // Arc correction to radius vector. Computed only every N_ARC_CORRECTION increments. ~375 usec
        // Compute exact location by applying transformation matrix from initial radius vector(=-offset).
        cos_Ti = cos(i * theta_per_segment);
        sin_Ti = sin(i * theta_per_segment);
        r_axis0 = -xo * cos_Ti + yo * sin_Ti;
        r_axis1 = -xo * sin_Ti - yo * cos_Ti;
        count = 0;
      }

      // Update arc_target location
      float x = center_axis0 + r_axis0;
      float y = center_axis1 + r_axis1;
      drawLine(x, y );
      // mc_line(position, feed_rate, invert_feed_rate);

      // Bail mid-circle on system abort. Runtime command check already performed by mc_line.
      // if (sys.abort) { return; }
    }
  }
  // Ensure last segment arrives at target location.
  drawLine(xt, yt );
}



