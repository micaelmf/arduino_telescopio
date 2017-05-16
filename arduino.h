///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ARDUINO CODE:
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**************************************************************************
 * MEqM1DriverControl Arduino File
 *
 * My German Equatorial Mount Controller with Arduino Due that implements
 * LX200 Protocol Resumed and control the stepper motor driver DRV8825.
 *
 * Created: 29 de julho de 2014
 *
 * Author: Luiz Antonio Gabriel
 *
 **************************************************************************/
#include "Wire.h"
#include <MEqM1DriverControl.h>
MEqM1DriverControl Driver;
void setup() {
  Wire.begin();
  SerialUSB.begin(9600);
  Driver.initializeDateTime();
}
void loop() {
  if (SerialUSB.available()) {
    Driver.answerSerialRequest();
  }
  Driver.shaftDecController();
  Driver.shaftRAController();
  Driver.slewController();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
C++ HEADER:
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**************************************************************************
 * MEqM1DriverControl Class Header File
 *
 * My German Equatorial Mount Controller with Arduino Due that implements
 * LX200 Protocol Resumed and control the stepper motor driver DRV8825.
 *
 * Created: 29 de julho de 2014
 *
 * Author: Luiz Antonio Gabriel
 *
 **************************************************************************/
#ifndef MEqM1DriverControl_h
#define MEqM1DriverControl_h
// Arduino Driver Pins Configuration.
#define DecDRV8825Dir   2
#define DecDRV8825Step  3
#define DecDRV8825M2    4
#define DecDRV8825M1    5
#define DecDRV8825M0    6
#define DecDRV8825En    7
#define RADRV8825Dir    8
#define RADRV8825Step   9
#define RADRV8825M2     10
#define RADRV8825M1     11
#define RADRV8825M0     12
#define RADRV8825En     13
#define SLEWALLOWED     31
#define JOYSTICKVER     A9
#define JOYSTICKHOR     A10
#define JOYSTICKSEL     A11
// Define minimum semi-period of stepper motors (maximum frequency).
#define GOTO_RATE       625
// The Earth spend 23.9344698888889 h to complete one rotation relative
// to a fixed star, this implies in a frequency of
// 2.37686020007898e-5 µS/µs that result in a half period of 21036 µs.
#define TRACKING_RATE   21036
#define SLEW_RATE       1775
// Define the PCF8563 address.
#define PCF8563ADDRESS  0x51
#define PCF8563TIMEADD  0x02
#define PCF8563DATEADD  0x05
// library interface description
class MEqM1DriverControl {
  public:
    // MEqM1DriverControl Constructor.
    MEqM1DriverControl();
    // Translates serial input in accordance of LX200 protocol and answer it.
    void answerSerialRequest(void);
    // Initializes date and time;
    void initializeDateTime(void);
    // Controls the Thumb Joystick and slew.
    void slewController(void);
    // Controls the DEC shafts.
    void shaftDecController(void);
    // Controls the RA shafts.
    void shaftRAController(void);
  private:
    /*******
     * Private Members Variables.
     *****************************/ 
   
    // Holds the state of button.
    bool buttonState;
    // The variable is TRUE if target is check and is OK otherwise is FALSE.
    bool gotoDecAllowed;
    // The variable is TRUE if target is check and is OK otherwise is FALSE.
    bool gotoRAAllowed;
    // Holds the time elapsed since initial time.
    unsigned long initialMicrosecond;
    // Holds the initial time.
    double initialTime;
    // Holds the high limit that is allowing to point the telescope.
    double highLimit;
    // Holds the initialJD0.
    double initialJD0;
    // Holds the last micros() to calculate the delay for step pulse.
    unsigned long lastDecMicro;
    // Holds the last micros() to calculate the delay for step pulse.
    unsigned long lastRAMicro;
    // Holds the last micros() to calculate the delay for step pulse.
    unsigned long lastSlewMicro;
    // Local latitude in decimal format.
    double latitude;
    // Local longitude in decimal format.
    double longitude;
    // Holds the microsteps for DEC DRV8825 driver.
    short microstepModeDec;
    // Holds the microsteps for RA DRV8825 driver.
    short microstepModeRA;
    // Holds the all response for serial communications.
    char *response;
    // Current position of Dec Shaft given in microsteps (mode 5 = 1/32).
    long shaftDecPosition;
    // Current position of RA Shaft given in microsteps (mode 5 = 1/32).
    long shaftRAPosition;
    // Holds the state of slew.
    bool slewEnabled;
    // Steps to go to reach the Dec target given in microsteps (mode 5 = 1/32).
    long microStepsTogoDec;
    // Steps to go to reach the RA target given in microsteps (mode 5 = 1/32).
    long microStepsTogoRA;
    // Target Dec Coordinate in decimal degrees.
    double targetDec;
    // Target RA Coordinate in decimal degrees.
    double targetRA;
    // Controls the Dec, RA and Slew clocks.
    bool tictacDec;
    bool tictacRA;
    // Turn on/off the syncronization of RA.
    bool tracking;
   
    /*******
     * Private Members Functions.
     *****************************/ 
   
    // Transforms bcd format to decimal.
    int BCDToDecimal(byte bcd);
    // Verify if the telescope is allowable to reach the target.
    short checkTarget(void);
    // Transforms decimal format to bcd.
    byte decimalToBCD(int decimal);
    // Translate decimal format into HMS.
    void decimalToHMS(double value, bool sign);
    // Disables the DEC Driver.
    void disableDecDriver(void);
    // Disables the RA Driver.
    void disableRADriver(void);
    // Enables the DEC Driver.
    void enableDecDriver(void);
    // Enables the RA Driver.
    void enableRADriver(void);
    // Enables tracking.
    void enableTracking(void);
    // Selects the best mode to reach the target.
    short getBestMode(long microStepsToGo);
    // Returns Dec current coordinate in decimal degrees.
    double getCurrentDec(void);
    // Returns RA current coordinate in decimal degrees.
    double getCurrentRA(void);
    // Returns the current date on the standard buffer (response).
    void getDate(void);
    // Returns HA in decimal degrees with range correction.
    double getHA(double RA);
    // Gets the Julian Date.
    double getJD0(void);
    // Returns the Local Sidereal Time in decimal degrees.
    double getLST(void);
    // Gets the microsteps to go.
    void getMicrostepsTogo(void);
    // Gets the precise time.
    double getPreciseTime(void);
    // Gets the coordinate of shaft position for RA/Dec.
    void getTargetShaftPosition(double RA, double Dec,
                long &tgShRA, long &tgShDec);
    // Gets the local time in decimal format.
    double getTime(void);
    // Transform the coordinates RA/Dec to Alt/Az.
    void RADecToAltAz(double RA, double Dec, double &Alt, double &Az);
    // Sets the Greenwich julian date from date passed in format (DD, MM, AAAA).
    bool setDate(int day, int mo, int yr);
    // Sets the high limit.
    bool setHighLimit(double highLimit);
    // Sets the local latitude.
    void setLatitude(double latitude);
    // Sets the local longitude.
    void setLongitude(double longitude);
    // Sets the two drivers to the mode of microstep chosed.
    void setMicrostepDecMode(short mode);
    // Sets the two drivers to the mode of microstep chosed.
    void setMicrostepRAMode(short mode);
    // Sets target Dec Coordinate in decimal degrees.
    void setTargetDec(int d, int m, int s);
    // Sets target RA Coordinate in decimal degrees.
    void setTargetRA(int h, int m, int s);
    // Sets the real time clock on PCF8563 and updated initial local time.
    void setTime(int h, int m, int s);
    // Return the sign of x.
    int sign(double x);
    // Translate HMS format into decimal.
    double translateHMSToDec(char *value);
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
C++ CODE:
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**************************************************************************
 * MEqM1DriverControl Class Implementation File
 *
 * My German Equatorial Mount Controller with Arduino Due that implements
 * LX200 Protocol Resumed and control the stepper motor driver DRV8825.
 *
 * Created: 29 de julho de 2014
 *
 * Author: Luiz Antonio Gabriel
 *
 **************************************************************************/
#include "Arduino.h"
#include "Wire.h"
#include "MEqM1DriverControl.h"
/*****************************************************************
 *
 *  MEqM1DriverControl Constructor
 *
 *****************************************************************/
MEqM1DriverControl::MEqM1DriverControl() {
  // Initially puts the state of button to HIGH (true).
  this->buttonState = true;
  // Initially make the goto function not allowed.
  this->gotoDecAllowed = false;
  this->gotoRAAllowed = false;
  // Initializes high limit as 90º.
  this->highLimit = 90.0;
  // Initializes with the latitude of Guarulhos by default.
  this->latitude = -23.4600000;
  // Initializes with the longitude of Guarulhos by default.
  this->longitude = -46.4900028;
  // Put the full step mode for both shafts.
  this->microstepModeDec = 0;
  this->microstepModeRA = 0;
  // Initialize microsteps to go with zero for both shafts.
  this->microStepsTogoDec = 0L;
  this->microStepsTogoRA = 0L;
  // Create the standard buffer to responses.
  this->response = new char[30];
  // Initially points to south (-90º,0h) or north (+90º,0h) celestial pole.
  this->shaftDecPosition = 0L;
  this->shaftRAPosition = 0L;
  // Initially sets the slew mode to off (false).
  this->slewEnabled = false;
  // Puts the target at the South sidereal pole.
  this->targetDec = -90.0;
  this->targetRA = 0.0;
  // Puts the initial tictac to HIGH for both shafts.
  this->tictacDec = true;
  this->tictacRA = true;
  // Turn off the tracking initially.
  this->tracking = false;
 
// setup the pins on the microcontroller:
  pinMode(DecDRV8825Dir,  OUTPUT);
  pinMode(DecDRV8825Step, OUTPUT);
  pinMode(DecDRV8825M2,   OUTPUT);
  pinMode(DecDRV8825M1,   OUTPUT);
  pinMode(DecDRV8825M0,   OUTPUT);
  pinMode(RADRV8825Dir,   OUTPUT);
  pinMode(RADRV8825Step,  OUTPUT);
  pinMode(RADRV8825M2,    OUTPUT);
  pinMode(RADRV8825M1,    OUTPUT);
  pinMode(RADRV8825M0,    OUTPUT);
  pinMode(SLEWALLOWED,    OUTPUT);
  pinMode(JOYSTICKSEL,    INPUT);
  pinMode(JOYSTICKHOR,    INPUT);
  pinMode(JOYSTICKVER,    INPUT);
}
/*****************************************************************
 *
 *  Public Function Members
 *
 *****************************************************************/
void MEqM1DriverControl::answerSerialRequest(void) {
  int delayRead = 60;
  char request = 0;
  int d, h, m, s, y;
  double Alt, Az;
  short checkResult;
 
  while (SerialUSB.available() > 0 && request != ':') {
    request=SerialUSB.read();
    delayMicroseconds(delayRead);
  }
  // Get the first character after beginning of command symbol ':'.
  request=SerialUSB.read();
  switch(request) {
    case 'G': // Get Telescope Information.
      // Get next character.
      request=SerialUSB.read();
      if (request == 'A') { // Telescope Altitude. Returns: sDD:MM:SS#
        RADecToAltAz(getCurrentRA(), getCurrentDec(),Alt, Az);
        decimalToHMS(Alt, false);
        SerialUSB.print(this->response);
      } else if (request == 'C') { // Current date. Returns: MM/DD/YY#
        getDate();
        SerialUSB.println(this->response);
      } else if (request == 'd') { // Currently Target DEC. Returns: sDD:MM:SS#
        decimalToHMS(this->targetDec, true);
        SerialUSB.print(this->response);
      } else if (request == 'D') { // Telescope DEC. Returns: sDD:MM:SS#
        decimalToHMS(getCurrentDec(), true);
        SerialUSB.print(this->response);
      } else if (request == 'g') { // Current Site Longitude. Returns: sDDD:MM#
        decimalToHMS(this->longitude, true);
        SerialUSB.print(this->response);
      } else if (request == 'G') { // UTC offset time. Returns: sHH# or sHH.H#
        sprintf(this->response, "%+02i#", int(this->longitude/15));
        SerialUSB.print(this->response);
      } else if (request == 'h') { // High Limit. Returns: sDD#
        SerialUSB.println(this->highLimit);
      } else if (request == 'H') { // Daylight Saving Time Setting.
                                 // Returns: 0# if daylight savings is disabled
                                 // Returns: 1# if daylight savings is enabled
      } else if (request == 'K') { // Microsteps to go Extended Command.
        getMicrostepsTogo();
        sprintf(this->response,"RA: %d, DEC: %d",
          this->microStepsTogoRA, this->microStepsTogoDec);
        SerialUSB.println(this->response);
      } else if (request == 'L') { // Local Time (24h). Returns: HH:MM:SS#
        decimalToHMS(getTime(), false);
        SerialUSB.println(this->response);
      } else if (request == 'o') { // Lower Limit. Returns: DD#
        SerialUSB.println("00#");
      } else if (request == 'r') { // Current Target RA. Returns: HH:MM:SS#
        decimalToHMS(this->targetRA, false);
        SerialUSB.print(this->response);
      } else if (request == 'R') { // Current Telescope RA. Returns: HH:MM:SS#
        decimalToHMS(getCurrentRA()/15.0, false);
        SerialUSB.print(this->response);
      } else if (request == 'S') { // Sidereal Time. Returns: HH:MM:SS#
        decimalToHMS(getLST()/15.0, false);
        SerialUSB.print(this->response);
      } else if (request == 't') { // Current Site Latitude. Returns: sDD:MM#
        decimalToHMS(this->latitude, true);
        SerialUSB.print(this->response);
      } else if (request == 'T') { // Tracking Rate. Returns: TT.T#
        SerialUSB.print("23.768602#"); // microsteps/s (mode 5).
      } else if (request == 'V') {
        // Get next character.
        request=SerialUSB.read();
        if (request == 'D') {        // EM1 Firmware Date. Returns: mmm dd yyyy#
          SerialUSB.print("jul 29 2014#");
        } else if (request == 'N') { // EM1 Firmware Version. Returns: dd.d#
          SerialUSB.print("1.0#");
        }         
      } else if (request == 'Z') { // Telescope Azimuth. Returns: DDD:MM:SS#
        RADecToAltAz(getCurrentRA(), getCurrentDec(), Alt, Az);
        decimalToHMS(Az, false);
        SerialUSB.print(this->response);
      }
      break;
    case 'M': // Telescope Movement Commands.
      // Get next character.
      request=SerialUSB.read();
      if (request == 'A') { // Slew Alt/Az. Format: MA#
        // Returns: 0 - no fault, 1 - fault.
      } else if (request == 'e') { // Move to East at current rate. Format: Me#
        // Returns: Nothing.
      } else if (request == 'g') { // Move to direction for milliseconds.
        // Get next character.
        request=SerialUSB.read();
        if (request == 'e') {        // East. Format: MgeDDDD#
          // Returns: Nothing.
        } else if (request == 'n') { // North. Format: MgnDDDD#
          // Returns: Nothing.
        } else if (request == 's') { // South. Format: MgsDDDD#
          // Returns: Nothing.
        } else if (request == 'w') { // West. Format: MgwDDDD#
          // Returns: Nothing.
        }
      } else if (request == 'n') { // Move to North at current rate. Format: Mn#
        // Returns: Nothing.
      } else if (request == 's') { // Move to South at current rate. Format: Ms#
        // Returns: Nothing.
      } else if (request == 'S') { // Slew to Target. Format: MS#
        // Returns: 0 - possible, 1<str># - below horizont w/ string,
        // 2<str># - Over higher w/ string.
        // Verify if target position is above the horizon.
        checkResult = checkTarget();
        if (checkResult == 0) {
          SerialUSB.print("0#"); // If OK send '0'.
        } else if (checkResult == 1) {
          // If it's below the horizon respond with '1'.
          SerialUSB.print("1Object below low limit.#");
        } else {
          // If it's over higher respond with '2'.
          SerialUSB.print("2Object over high limit.#");
        }
      }
      break;
    case 'Q': // Movement Commands.
      // Get next character.
      request=SerialUSB.read();
      if (request != '#') {
        if (request == 'e') {        // Halt Eastward slew. Format: Qe#
          // Returns: Nothing.
        } else if (request == 'n') { // Halt Northward slew. Format: Qn#
          // Returns: Nothing.
        } else if (request == 's') { // Halt Southward slew. Format: Qs#
          // Returns: Nothing.
        } else if (request == 'w') { // Halt Westward slew. Format: Qw#
          // Returns: Nothing.
        }
      } else { // Halt all current slewing. Format: Q#
        disableDecDriver();
        disableRADriver();
        // Returns: Nothing.
      }
      break;
    case 'S': // Telescope Set Commands.
      // Get next character.
      request=SerialUSB.read();
      if (request == 'C') { // Date. Format: SCMM/DD/YY.
        d = SerialUSB.parseInt();   // Initial day.
        m = SerialUSB.parseInt();   // Initial month.
        y = SerialUSB.parseInt();   // Initial year.
        // Returns: '0' if date is invalid. '1' if valid + "Date updated!#"
        if (setDate(d, m, y)) SerialUSB.println("1Date updated!#");
        else SerialUSB.println("0#");
      } else if (request == 'd') {  // Target DEC. Format: Sdsdd:mm:ss.
        d = SerialUSB.parseInt();   // Target DEC degrees.
        m = SerialUSB.parseInt();   // Target DEC minutes of arc.
        s = SerialUSB.parseInt();   // Target DEC seconds of arc.
        // Returns: 0 - invalid, 1 - valid.
        setTargetDec(d, m, s);
        SerialUSB.print("1#");
      } else if (request == 'g') {  // Current site's long. Format: SgDDD:MM#
        // Returns: 0 - invalid, 1 - valid.
      } else if (request == 'G') {  // Number of hours to UTC. Format: SGsHH.H#
        // Returns: 0 - invalid, 1 - valid.
      } else if (request == 'h') {  // Maximum elevation limit. Format: ShDD#
        // Returns: 0 - invalid, 1 - valid.
        h = SerialUSB.parseInt();   // Take the high limit.
        if (setHighLimit(h)) SerialUSB.print("1#");
        else SerialUSB.print("0#");
      } else if (request == 'H') {  // Daylight saving. Format: SH1# or SH0#
        // Returns: nothing.
      } else if (request == 'L') {  // Local time. Format: SLHH:MM:SS#
        h = SerialUSB.parseInt();   // Initial hour.
        m = SerialUSB.parseInt();   // Initial minute.
        s = SerialUSB.parseInt();   // Initial second.
        // Returns: 0 - invalid, 1 - valid.
        setTime(h, m, s);
        SerialUSB.print("1#");
      } else if (request == 'o') { // Lowest elevation. Format: SoDD#
        // Returns: 0 - invalid, 1 - valid.
      } else if (request == 'r') {  // Target RA. Format: SrHH:MM:SS#.
        h = SerialUSB.parseInt();   // Target RA hour of arc.
        m = SerialUSB.parseInt(); // Target RA minutes of arc.
        s = SerialUSB.parseInt(); // Target Ra seconds of arc.
        // Returns: 0 - invalid, 1 - valid.
        setTargetRA(h, m, s);
        SerialUSB.print("1#");
      } else if (request == 'S') { // LST. Format: SSHH:MM:SS#
        // Returns: 0 - invalid, 1 - valid.
      } else if (request == 't') { // Current site latitude. Format: StsDD:MM#
        // Returns: 0 - invalid, 1 - valid.
      } else if (request == 'T') { // Tracking rate. Format: STdd.ddddddd#
        // Tracking rate (microsteps/s). One shaft revolution = 2048000 µS.
        // Sidereal rate: 23.7686019 µS/s or one microstep every 42072 µs with
        // an error of 1 arc-sec every 2,5 h.
        // Returns: 0 - invalid, 1 - valid.
      } else if (request == 'w') { // Slew rate. Format: SwN#
        // Returns: 0 - invalid, 1 - valid.
      } else if (request == 'z') { // Target Azimuth. Format: SzDDD:MM#
        // Returns: 0 - invalid, 1 - valid.
      }
      break;
    case 'T': // Tracking Commands.
      // Get next character.
      request=SerialUSB.read();
      if (request == '+') {        // Increment Manual rate. Format: T+#
        // Return nothing.
      } else if (request == '-') { // Decrement Manual rate. Format: T-#
        // Return nothing.
      } else if (request == 'L') { // Lunar Tracking rate. Format: TL#
        // Return nothing.
      } else if (request == 'M') { // Custom tracking rate. Format: TM#
        // Return nothing.
      } else if (request == 'Q') { // Sidereal tracking rate. Format: TQ#
        // Return nothing.
      } else if (request == 'S') { // Solar tracking rate. Format: TS#
        // Return nothing.
      }
      break;
    default:
      break;
  }
}
void MEqM1DriverControl::initializeDateTime(void) {
  int day, dow, month, year;
  Wire.beginTransmission(PCF8563ADDRESS);
  Wire.write(PCF8563DATEADD);
  Wire.endTransmission();
  Wire.requestFrom(PCF8563ADDRESS, 4);
  day   = BCDToDecimal(Wire.read() & 0x3f);
  dow   = BCDToDecimal(Wire.read() & 0x07); 
  month = BCDToDecimal(Wire.read() & 0x1f);
  year  = 2000 + BCDToDecimal(Wire.read());
 
  // Calculate the JD0 (Julian Day without hours calculation) and return it.
  this->initialJD0 = double(367.0 * year -
    int(7 * (year + int((month + 9) / 12.0)) / 4.0) +
    int(275.0 * month / 9.0) + day + 1721013.5);
  // Gets initial time.
  this->initialTime = getTime();
  // Holds the microseconds of initial time.
  this->initialMicrosecond = micros();
}
void MEqM1DriverControl::slewController(void) {
  long timeElapsed = 0L;
  long microsNow = 0L;
  int button, horizontal, vertical;
  // Read the state of joystick.
  button  = analogRead(JOYSTICKSEL);
  // Verify if joystick button was pressed.
  if ((button == 0) && buttonState) { // If it is transition to down.
    buttonState = false;
    // Wait 0.1 s to stop bouncing.
    delay(100);
  } else if ((button != 0) && !buttonState) { // If it is transition to up.
    buttonState = true;
    // Wait 0.1 s to stop bouncing.
    delay(100);
    // Do not allow the RA shaft exceeds the limit of ± 90° (512000 microsteps)
    // and the DEC shaft exceeds the limit of ± 180° (1024000 microsteps).
    if ((abs(this->shaftDecPosition) < 1024000) &&
          (abs(this->shaftRAPosition) < 512000)) {
      // Change the state of slew mode.
      this->slewEnabled = !this->slewEnabled;
    }
    // Run the Once Time Routine.
    if (this->slewEnabled) {
      // Turn on the slew indicator led.
      digitalWrite(SLEWALLOWED, HIGH);
      // Disable all moviment of the telescope.
      disableDecDriver();
      disableRADriver();
      // Enables the movement of the drivers.
      digitalWrite(DecDRV8825En, LOW);
      digitalWrite(RADRV8825En, LOW);
      // Sets the microstep to mode 5 in both axis.
      setMicrostepDecMode(5);
      setMicrostepRAMode(5);
    } else {
      // Turn off the slew indicator led.
      digitalWrite(SLEWALLOWED, LOW);
      // Disable all moviment of DEC shaft.
      disableDecDriver();
      // Restart the tracking.
      enableTracking();
    }
  }
  if (this->slewEnabled) {
    vertical = analogRead(JOYSTICKVER);
    horizontal = analogRead(JOYSTICKHOR);
    if (!((horizontal >= 700) && (horizontal <= 800)) ||
        !((vertical >= 700) && (vertical <= 800))) {
      microsNow = micros();
      // Corrects, if necessary, timeElapsed for micros() overflow.
      if (microsNow < this->lastSlewMicro) {
        timeElapsed = (4294967295 - this->lastSlewMicro) + microsNow;
      } else {
        timeElapsed = microsNow - this->lastSlewMicro;
      }
      if (timeElapsed > SLEW_RATE) {
        // Update the clock holder.
        this->lastSlewMicro = microsNow;
        if (horizontal < 700) { // slew to the WEST.
          // Increment exactly one microstep by cycle.
          this->shaftRAPosition++;
          digitalWrite(RADRV8825Dir, LOW);
          digitalWrite(RADRV8825Step, LOW);
          delayMicroseconds(100);
          digitalWrite(RADRV8825Step, HIGH);
        } else if (horizontal > 800) { // slew to the EAST.
          // Decrement exactly one microstep by cycle.
          this->shaftRAPosition--;
          digitalWrite(RADRV8825Dir, HIGH);
          digitalWrite(RADRV8825Step, LOW);
          delayMicroseconds(100);
          digitalWrite(RADRV8825Step, HIGH);
        }
        if (vertical < 700) { // slew to the NORTH.
          // Increment exactly one microstep by cycle.
          this->shaftDecPosition++;
          digitalWrite(DecDRV8825Dir, LOW);
          digitalWrite(DecDRV8825Step, LOW);
          delayMicroseconds(100);
          digitalWrite(DecDRV8825Step, HIGH);
        } else if (vertical > 800) { // slew to the EAST.
          // Decrement exactly one microstep by cycle.
          this->shaftDecPosition--;
          digitalWrite(DecDRV8825Dir, HIGH);
          digitalWrite(DecDRV8825Step, LOW);
          delayMicroseconds(100);
          digitalWrite(DecDRV8825Step, HIGH);
        }
      }
    }
  }
}
void MEqM1DriverControl::shaftDecController(void) {
  long timeElapsed = 0L;
  long microsNow = 0L;
  int advance = 0;
  if (this->gotoDecAllowed) {
    microsNow = micros();
    // Corrects, if necessary, timeElapsed for micros() overflow.
    if (microsNow < this->lastDecMicro) {
      timeElapsed = (4294967295 - this->lastDecMicro) + microsNow;
    } else {
      timeElapsed = microsNow - this->lastDecMicro;
    }
    if (timeElapsed > GOTO_RATE){
      // Update the clock holder.
      this->lastDecMicro = microsNow;
      if (this->tictacDec) {
        this->tictacDec = false;
        // Get the best mode for Dec.
        setMicrostepDecMode(getBestMode(this->microStepsTogoDec));
        advance = sign(this->microStepsTogoDec) * pow(2,5-microstepModeDec);
        // Do not allow the DEC shaft exceeds the
        // limit of ± 180° (1024000 microsteps).
        if (abs(this->shaftDecPosition + advance) >= 1024000) {
          this->gotoDecAllowed = false;
        } else {
          this->shaftDecPosition += advance;
          // Update the stepsTogo in acordance to the how many microsteps the
          // correspondent shaft slewed.
          this->microStepsTogoDec -= advance;
          // Verify if the target was achieved.
          if (this->microStepsTogoDec == 0) {
            disableDecDriver();
          } else {
            // Change the output of wire to HIGH
            digitalWrite(DecDRV8825Step, HIGH);
          }
        }
      } else {
        this->tictacDec = true;
        // Change the output of wire to LOW.
        digitalWrite(DecDRV8825Step, LOW);
      }
    }
  }
}
void MEqM1DriverControl::shaftRAController(void) {
  long timeElapsed = 0L;
  long microsNow = 0L;
  long tgShRA  = 0L;
  long tgShDec = 0L;
  int advance = 0;
  if (this->gotoRAAllowed) {
    microsNow = micros();
    // Corrects, if necessary, timeElapsed for micros() overflow.
    if (microsNow < this->lastRAMicro) {
      timeElapsed = (4294967295 - this->lastRAMicro) + microsNow;
    } else {
      timeElapsed = microsNow - this->lastRAMicro;
    }
    if (timeElapsed > GOTO_RATE){
      // Update the clock holder.
      this->lastRAMicro = microsNow;
      if (this->tictacRA) {
        this->tictacRA = false;
        // Get the best mode for RA.
        setMicrostepRAMode(getBestMode(this->microStepsTogoRA));
        advance = sign(this->microStepsTogoRA) * pow(2,5-microstepModeRA);
        // Do not allow the RA shaft exceeds the
        // limit of ± 90° (512000 microsteps).
        if (abs(this->shaftRAPosition + advance)>= 512000) {
          disableRADriver();
        } else {
          this->shaftRAPosition += advance;
          // Gets the new position of target in microsteps.
          getTargetShaftPosition(targetRA, targetDec, tgShRA, tgShDec);
          // Gets the microsteps to go from current position
          // to the new target position.
          this->microStepsTogoRA  = tgShRA - this->shaftRAPosition;
          if (this->microStepsTogoRA == 0) {
            enableTracking();
          } else {
            // Change the output of wire to HIGH
            digitalWrite(RADRV8825Step, HIGH);
          }
        }
      } else {
        this->tictacRA = true;
        // Change the output of wire to LOW.
        digitalWrite(RADRV8825Step, LOW);
      }
    } // If tracking is enable and slew is disable.
  } else if (this->tracking) {
    microsNow = micros();
    // Corrects, if necessary, timeElapsed for micros() overflow.
    if (microsNow < this->lastRAMicro) {
      timeElapsed = (4294967295 - this->lastRAMicro) + microsNow;
    } else {
      timeElapsed = microsNow - this->lastRAMicro;
    }
    if (timeElapsed > TRACKING_RATE){
      // Update the clock holder.
      this->lastRAMicro = microsNow;
      if (this->tictacRA) {
        this->tictacRA = false;
        // Do not allow the RA shaft exceeds the
        // limit of ± 90° (512000 microsteps).
        if (abs(this->shaftRAPosition) >= 512000) {
          disableRADriver();
        } else {
          // Increment exactly one microstep by cycle.
          this->shaftRAPosition++;
          // Change the output of wire to HIGH
          digitalWrite(RADRV8825Step, HIGH);
        }
      } else {
        this->tictacRA = true;
        // Change the output of wire to LOW.
        digitalWrite(RADRV8825Step, LOW);
      }
    }
  }
}
/*****************************************************************
 *
 *  Private Function Members
 *
 *****************************************************************/
int MEqM1DriverControl::BCDToDecimal(byte bcd) {
  return ((bcd / 16) * 10 + bcd % 16);
}
short MEqM1DriverControl::checkTarget(void) {
  double Alt = 0.0;
  double Az = 0.0;
  short AltCheck = 1;
  RADecToAltAz(this->targetRA, this->targetDec, Alt, Az);
  // If future position is above the horizon then allow goes to the target.
  if (Alt > 0) {
    if (Alt > this->highLimit) {
      disableDecDriver();
      disableRADriver();
      AltCheck = 2;
    } else {
      AltCheck = 0;
      // Gets the microsteps to go to the target.
      getMicrostepsTogo();
      // Enables both drivers.
      enableDecDriver();
      enableRADriver();
      // Initialize lastMicro by the first time before goto start.
      this->lastRAMicro = this->lastDecMicro = micros();
    }
  } else {
    disableDecDriver();
    disableRADriver();
    AltCheck = 1;
  }
 
  return AltCheck;
}
byte MEqM1DriverControl::decimalToBCD(int decimal) {
  return byte(decimal / 10 * 16 + decimal % 10);
}
void MEqM1DriverControl::decimalToHMS(double value, bool sign) {
  int h = 0;
  int m = 0;
  int s = 0;
  h = int(value);
  m = int(abs(value - h) * 60);
  s = round((abs(value) - abs(h) - m/60.0) * 3600);
  // Put the symbol 'ß' instead ':' for numbers with sign
  // and increase the number of character to write.
  if (sign) {
    sprintf(this->response, "%+03d%c%02d:%02d#", h, char(223), m, s);
  } else {
    sprintf(this->response, "%02d:%02d:%02d#", h, m, s);
  }
}
void MEqM1DriverControl::disableDecDriver(void) {
  // Stop the motion in DEC shaft.
  this->gotoDecAllowed = false;
  // Change all currents LOW to save energy.
  setMicrostepDecMode(0);
  digitalWrite(DecDRV8825Step, LOW);
  digitalWrite(DecDRV8825Dir, LOW);
  // Disables the driver.
  digitalWrite(DecDRV8825En, HIGH);
}
void MEqM1DriverControl::disableRADriver(void) {
  this->gotoRAAllowed = false;
  this->tracking = false;
  // Change all currents LOW to save energy.
  setMicrostepRAMode(0);
  digitalWrite(RADRV8825Step, LOW);
  digitalWrite(RADRV8825Dir, LOW);
  // Disables the driver.
  digitalWrite(RADRV8825En, HIGH);
}
void MEqM1DriverControl::enableDecDriver(void) {
  // Go to is allowed.
  this->gotoDecAllowed = true;
  // Set the direction of rotation.
  if (this->microStepsTogoRA > 0) {
    digitalWrite(RADRV8825Dir, LOW);
  } else {
    digitalWrite(RADRV8825Dir, HIGH);
  }
  setMicrostepDecMode(0);
  // Enables the driver.
  digitalWrite(DecDRV8825En, LOW);
}
void MEqM1DriverControl::enableRADriver(void) {
  // Go to is allowed.
  this->gotoRAAllowed = true;
  // When enable the goto disable, initially, the tracking.
  this->tracking = false;
  // Set the direction of rotation.
  if (this->microStepsTogoDec > 0) {
    digitalWrite(DecDRV8825Dir, LOW);
  } else {
    digitalWrite(DecDRV8825Dir, HIGH);
  }
  // Enables the driver.
  digitalWrite(RADRV8825En, LOW);
}
void MEqM1DriverControl::enableTracking(void) {
  this->gotoRAAllowed = false;
  // Prepares to track.
  this->tictacRA = false;
  this->tracking = true;
  // Sets the microstep to mode 5.
  setMicrostepRAMode(5);
  // Sets the correct direction of rotation. Clockwise in South.
  if (this->latitude < 0) {
    digitalWrite(RADRV8825Dir, LOW);
  } else {
    digitalWrite(RADRV8825Dir, HIGH);
  }
}
short MEqM1DriverControl::getBestMode(long microStepsToGo) {
  short mode = 0;
  long limit = abs(microStepsToGo);
  // Control the distance of final position where the slew rate slowing down.
  if (limit > 25600) {
    mode = 0;
  } else if (limit > 12800) {
    mode = 1;
  } else if (limit > 6400) {
    mode = 2;
  } else if (limit > 3200) {
    mode = 3;
  } else if (limit > 1600) {
    mode = 4;
  } else {
    mode = 5;
  }
  return mode;
}
double MEqM1DriverControl::getCurrentDec(void) {
  double dec = 0.0;
  double shaftDec = 0.0;
  double shaftRA = 0.0;
  int signLAT = sign(this->latitude);
  // Transform from microsteps (1/32 of full step) into degrees.
  // 360° of shaft = 320*200*32 = 2.048.000 microsteps.
  shaftDec = this->shaftDecPosition * 360.0 / 2048000.0;
  shaftRA = this->shaftRAPosition * 360.0 / 2048000.0;
 
  // Calculation of RA/DEC from shaft position and its respectives signal.
  // HA Quadrant <- signals for South(shaftRA, shaftDec): 1:-+ 2:++ 3:-- 4:+-
  // HA Quadrant <- signals for North(shaftRA, shaftDec): 1:+- 2:-- 3:++ 4:-+
  // Range of HA(0,360). Range of Dec(-90,90).
 
  // (-+): 1st Quadrant in South. 4th Quadrant in North.
  if (shaftRA <= 0 && shaftDec >= 0) {
    dec = (90.0 - shaftDec) * signLAT;         // South and North: (0,180).
  // (++): 2nd in Quadrant South. 3rd in Quadrant North.
  } else if (shaftRA >= 0 && shaftDec >= 0) {
    dec = (90.0 - shaftDec) * signLAT;         // South and North: (0,180).
  // (--): 3rd Quadrant in South. 2nd Quadrant in North.
  } else if (shaftRA <= 0 && shaftDec <= 0) {
    dec = (shaftDec + 90) * signLAT;           // South and North: (0,180).
  // (+-): 4th Quadrant in South. 1st Quadrant in North.
  } else {
    dec = (shaftDec + 90) * signLAT;           // South and North: (0,180).
  }
  return dec;
}
double MEqM1DriverControl::getCurrentRA(void) {
  double ha = 0.0;
  double ra = 0.0;
  double shaftDec = 0.0;
  double shaftRA = 0.0;
  int signLAT = sign(this->latitude);
  // Transform from microsteps (1/32 of full step) into degrees.
  // 360° of shaft = 320*200*32 = 2.048.000 microsteps.
  shaftDec = this->shaftDecPosition * 360.0 / 2048000.0;
  shaftRA = this->shaftRAPosition * 360.0 / 2048000.0;
 
  // Calculation of RA/DEC from shaft position and its respectives signal.
  // HA Quadrant <- signals for South(shaftRA, shaftDec): 1:-+ 2:++ 3:-- 4:+-
  // HA Quadrant <- signals for North(shaftRA, shaftDec): 1:+- 2:-- 3:++ 4:-+
  // Range of HA(0,360). Range of Dec(-90,90).
 
  // (-+): 1st Quadrant in South. 4th Quadrant in North.
  if (shaftRA <= 0 && shaftDec >= 0) {
    if (signLAT < 0) {
      ha  = shaftRA + 90.0;      // South: HA(0,90).
    } else {
      ha  = 270.0 - shaftRA;     // North: HA(270,360).
      if (ha == 360.0) ha = 0.0;
    }
  // (++): 2nd Quadrant in South. 3rd Quadrant in North.
  } else if (shaftRA >= 0 && shaftDec >= 0) {
    if (signLAT < 0) {
      ha  = shaftRA + 90.0;      // South: HA(90,180).
    } else {
      ha  = 270.0 - shaftRA;     // North: HA(180,270).
    }
  // (--): 3rd Quadrant in South. 2nd Quadrant in North.
  } else if (shaftRA <= 0 && shaftDec <= 0) {
    if (signLAT < 0) {
      ha  = 270.0 + shaftRA;     // South: HA(180,270).
    } else {
      ha  = 180.0 + shaftRA;     // North: HA(90,180).
    }
  // (+-): 4th Quadrant in South. 1st Quadrant in North.
  } else {
    if (signLAT < 0) {
      ha  = 270.0 + shaftRA;     // South: HA(270,360).
      if (ha == 360.0) ha = 0.0;
    } else {
      ha  = 90.0 - shaftRA;      // North: HA(0,90).
    }
  }
  // HA = LST - RA
  ra = getLST() - ha;
  if (ra < 0) ra = 360 + ra;     // Correct the range of RA (from 0° to 360°);
  return ra;
}
void MEqM1DriverControl::getDate(void) {
  int day, dow, month, year;
  Wire.beginTransmission(PCF8563ADDRESS);
  Wire.write(PCF8563DATEADD);
  Wire.endTransmission();
  Wire.requestFrom(PCF8563ADDRESS, 4);
  day   = BCDToDecimal(Wire.read() & 0x3f);
  dow   = BCDToDecimal(Wire.read() & 0x07); 
  month = BCDToDecimal(Wire.read() & 0x1f);
  year  = BCDToDecimal(Wire.read());
  sprintf(this->response, "%02d/%02d/%02d#", day, month, year);
}
double MEqM1DriverControl::getHA(double RA) {
  double HA = 0.0;
  HA = getLST() - RA;
  if (HA < 0) HA = 360 + HA;     // Correct the range of HA (from 0° to 360°);
  return HA;
}
double MEqM1DriverControl::getLST(void) {
  double GST, GST0, JC, LST;
  JC = (this->initialJD0 - 2451545.0) / 36525.0;
  GST0 = 100.4606184 + 36000.77005361 * JC + 0.00038793 * JC * JC;
  GST = GST0 + 15.041068640247729 * (getPreciseTime() -
    int(this->longitude/15.0));
  // Puts the value into the range of 0º to 360º;
  GST = GST - int(GST / 360.0) * 360.0;
  LST = GST + this->longitude;
 
  return LST;
}
void MEqM1DriverControl::getMicrostepsTogo(void){
  long tgShRA  = 0L;
  long tgShDec = 0L;
 
  // Gets the position of target in microsteps.
  getTargetShaftPosition(targetRA, targetDec, tgShRA, tgShDec);
  // Gets the microsteps to go from current position to the target position.
  this->microStepsTogoRA  = tgShRA  - this->shaftRAPosition;
  this->microStepsTogoDec = tgShDec - this->shaftDecPosition; 
}
double MEqM1DriverControl::getPreciseTime(void) {
  unsigned long timeElapsed, microsNow;
 
    microsNow = micros();
    // Corrects, if necessary, for micros() overflow.
    if (microsNow < this->initialMicrosecond) {
      timeElapsed = (4294967295 - this->initialMicrosecond) + microsNow;
      this->initialTime += (this->initialMicrosecond/3.6e9);
      this->initialMicrosecond = microsNow;
    } else {
      timeElapsed = microsNow - this->initialMicrosecond;
    }
  return this->initialTime + timeElapsed/3.6e9;
}
void MEqM1DriverControl::getTargetShaftPosition(double RA, double Dec,
                long &tgShRA, long &tgShDec) {
  double HA = 0.0;
  double shaftDec = 0.0;
  double shaftRA = 0.0;
  int signLAT = sign(this->latitude);
  HA = getHA(RA);
  // Calculation of shaftRa and shaftDec from HA quadrant.
  // Quadrant signals for South(RA, Dec): 1:-+ 2:++ 3:-- 4:+-
  // Quadrant signals for North(RA, Dec): 1:+- 2:-- 3:++ 4:-+
  // Range of shaftRA(-90,90). Range of shaftDec(-180,180).
  if (HA >= 0 && HA < 90) {            // First Quadrant.
    shaftRA  = (90.0 - HA) * signLAT;   // Sinal - in South, + in North.
    shaftDec = Dec - 90 * signLAT;      // Sinal + in South, - in North.
  } else if (HA >= 90 && HA < 180) {   // Second Quadrant.
    shaftRA  = (90.0 - HA) * signLAT;   // Sinal + in South, - in North.
    shaftDec = Dec - 90 * signLAT;      // Sinal + in South, - in North.
  } else if (HA >= 180 && HA < 270) {  // Third Quadrant.
    shaftRA  = (270.0 - HA) * signLAT;  // Sinal - in South, + in North.
    shaftDec = -(Dec - 90 * signLAT);   // Sinal - in South, + in North.
  } else {                              // Fourth Quadrant.
    shaftRA  = (270.0 - HA) * signLAT;  // Sinal + in South, - in North.
    shaftDec = -(Dec - 90 * signLAT);   // Sinal - in South, + in North.
  }
  // Transform from degrees into microsteps (1/32 of full step).
  // 360° of shaft = 320*200*32 = 2.048.000 microsteps.
  tgShRA = long(shaftRA / 360.0 * 2048000);
  tgShDec = long(shaftDec / 360.0 * 2048000);
}
double MEqM1DriverControl::getTime(void) {
  byte h, m, s;
 
  Wire.beginTransmission(PCF8563ADDRESS);
  Wire.write(PCF8563TIMEADD);
  Wire.endTransmission();
  Wire.requestFrom(PCF8563ADDRESS, 3);
  s = BCDToDecimal(Wire.read() & 0x7f);
  m = BCDToDecimal(Wire.read() & 0x7f);
  h = BCDToDecimal(Wire.read() & 0x3f);
 
  return double(h + m/60.0 + s/3600.0);
}
void MEqM1DriverControl::RADecToAltAz(double RA, double Dec,
        double &Alt, double &Az) {
  double D2R = 3.1415926535897 / 180.0; // º -> radians transformation factor.
  double DE = 0.0;
  double HA = 0.0;
  double LAT = 0.0;
  // Transform degree in radians.
  HA = getHA(RA)*D2R;
  DE = Dec*D2R;
  LAT = this->latitude*D2R;
  // Coordinates transformation.
  Alt = asin(sin(DE)*sin(LAT) + cos(DE)*cos(LAT)*cos(HA));
  Az = acos((sin(DE) - sin(LAT)*sin(Alt))/(cos(Alt)*cos(LAT)));
  // Transform radians back to the degree.
  Alt /= D2R;
  Az /= D2R;
  // Correction for the range of the azimute (Az).
  if (HA > 0) Az = 360 - Az;
}
bool MEqM1DriverControl::setDate(int day, int mo, int yr) {
  bool validDate = false;
 
  // Make a basic verification of date. Because of use of J2000
  // and PCF8563 the year must be greater than 2000 -> 00.
  // Pay attention to the more common errors.
  if ((mo == 2) && (day > 28)) {
    // If is leap year allow the 29th day.
    if ((yr / 4.0 == 0) && (day == 29)) {
      validDate = true;
    } else {
      validDate = false;
    }
  } else if ((mo == 4) || (mo == 6) || (mo == 9) || (mo == 11) && (day > 30)) {
    validDate = false;     
  } else if ((day > 0) && (day <= 31) && (mo > 0) && (mo <=12)) {
    validDate = true;
    // Update the time on PCF8563.
    Wire.beginTransmission(PCF8563ADDRESS);
    Wire.write(PCF8563DATEADD);
    Wire.write(decimalToBCD(day));
    Wire.write(decimalToBCD(1)); // Not used. Don't worry about.
    Wire.write(decimalToBCD(mo));
    Wire.write(decimalToBCD(yr));
    Wire.endTransmission();
  }
  return validDate;
}
bool MEqM1DriverControl::setHighLimit(double highLimit) {
  if (highLimit >= 0.0 && highLimit <= 90.0) {
    this->highLimit = highLimit;
    return 1;
  } else {
    return 0;
  }
}
void MEqM1DriverControl::setLatitude(double latitude) {
  this->latitude = latitude;
}
void MEqM1DriverControl::setLongitude(double longitude) {
  this->longitude = longitude;
}
void MEqM1DriverControl::setMicrostepDecMode(short mode) {
  switch (mode) {
    case 0:
      digitalWrite(DecDRV8825M2, LOW);
      digitalWrite(DecDRV8825M1, LOW);
      digitalWrite(DecDRV8825M0, LOW);
      break;
    case 1:
      digitalWrite(DecDRV8825M2, LOW);
      digitalWrite(DecDRV8825M1, LOW);
      digitalWrite(DecDRV8825M0, HIGH);
      break;
    case 2:
      digitalWrite(DecDRV8825M2, LOW);
      digitalWrite(DecDRV8825M1, HIGH);
      digitalWrite(DecDRV8825M0, LOW);
      break;
    case 3:
      digitalWrite(DecDRV8825M2, LOW);
      digitalWrite(DecDRV8825M1, HIGH);
      digitalWrite(DecDRV8825M0, HIGH);
      break;
    case 4:
      digitalWrite(DecDRV8825M2, HIGH);
      digitalWrite(DecDRV8825M1, LOW);
      digitalWrite(DecDRV8825M0, LOW);
      break;
    case 5:
      digitalWrite(DecDRV8825M2, HIGH);
      digitalWrite(DecDRV8825M1, LOW);
      digitalWrite(DecDRV8825M0, HIGH);
      break;
    default:
      digitalWrite(DecDRV8825M2, LOW);
      digitalWrite(DecDRV8825M1, LOW);
      digitalWrite(DecDRV8825M0, LOW);
      break;
  }
  // Update the microstep mode for the Dec shaft.
  this->microstepModeDec = mode;
}
void MEqM1DriverControl::setMicrostepRAMode(short mode) {
  switch (mode) {
    case 0:
      digitalWrite(RADRV8825M2, LOW);
      digitalWrite(RADRV8825M1, LOW);
      digitalWrite(RADRV8825M0, LOW);
      break;
    case 1:
      digitalWrite(RADRV8825M2, LOW);
      digitalWrite(RADRV8825M1, LOW);
      digitalWrite(RADRV8825M0, HIGH);
      break;
    case 2:
      digitalWrite(RADRV8825M2, LOW);
      digitalWrite(RADRV8825M1, HIGH);
      digitalWrite(RADRV8825M0, LOW);
      break;
    case 3:
      digitalWrite(RADRV8825M2, LOW);
      digitalWrite(RADRV8825M1, HIGH);
      digitalWrite(RADRV8825M0, HIGH);
      break;
    case 4:
      digitalWrite(RADRV8825M2, HIGH);
      digitalWrite(RADRV8825M1, LOW);
      digitalWrite(RADRV8825M0, LOW);
      break;
    case 5:
      digitalWrite(RADRV8825M2, HIGH);
      digitalWrite(RADRV8825M1, LOW);
      digitalWrite(RADRV8825M0, HIGH);
      break;
    default:
      digitalWrite(RADRV8825M2, LOW);
      digitalWrite(RADRV8825M1, LOW);
      digitalWrite(RADRV8825M0, LOW);
      break;
  }
  // Update the microstep mode for the RA shaft.
  this->microstepModeRA = mode;
}
void MEqM1DriverControl::setTargetDec(int d, int m, int s) {
  this->targetDec = sign(d)*(abs(d) + float(m) / 60.0 + float(s) / 3600.0);
}
void MEqM1DriverControl::setTargetRA(int h, int m, int s) {
  this->targetRA = (h + float(m) / 60.0 + float(s) / 3600.0) * 15;
}
void MEqM1DriverControl::setTime(int h, int m, int s) {
  // Update the time on PCF8563.
  Wire.beginTransmission(PCF8563ADDRESS);
  Wire.write(PCF8563TIMEADD);
  Wire.write(decimalToBCD(s));
  Wire.write(decimalToBCD(m));
  Wire.write(decimalToBCD(h));
  Wire.endTransmission();
}
int MEqM1DriverControl::sign(double x) {
  int sign = 1;
  // Verify if x is zero because of the denominator in the expression below.
  if (x == 0) sign = 0;
  else sign = int (x / abs(x));
  return sign;
}
double MEqM1DriverControl::translateHMSToDec(char *value) {
  char buffer[5];
  int number[3];
  int j = 0;
  int k = 0;
  for (int i = 0; i < strlen(value); i++) {
    if (value[i] == ':' or value[i] == char(223)) {
      buffer[j] = 0;            // Close the string.
      number[k] = atoi(buffer); // Convert from string to integer.
      j = 0;                    // Points to the first place of the buffer.
      ++k;                      // Points to the next integer of array.
    } else {
      buffer[j] = value[i];     // Copy the value to buffer.
      ++j;
    }
  }
  buffer[j] = 0;                // Close the last string.
  number[2] = atoi(buffer);     // Gets the last number.
  return double(abs(number[0]) + number[1] / 60.0 +
          number[2] / 3600.0) * sign(number[0]);
}﻿