#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHRCS.h>
#include <FEHSD.h>
#include <FEHServo.h>

#define COUNTS_PER_INCH 40.489
#define TRACK_WIDTH_INCHES 7.25
#define MAX_VEL 24
#define PULSE_TIME 0.03
#define PULSE_POWER 35
#define RCS_WAIT_TIME_IN_SEC 0.1

DigitalEncoder right_encoder(FEHIO::Pin8); //black ground, red 3.3
DigitalEncoder left_encoder(FEHIO::Pin9);

FEHMotor right_motor(FEHMotor::Motor1,9.0);
FEHMotor left_motor(FEHMotor::Motor3,9.0);

AnalogInputPin cds(FEHIO::Pin0); //orange ground, blue 3.3, yellow signal
#define CDS_RED_LIGHT 0.22 //these values are good for red filter
#define CDS_BLUE_LIGHT 0.8
#define CDS_ERROR 0.15

AnalogInputPin leftOpto(FEHIO::Pin7); //more light = lower voltage
AnalogInputPin midOpto(FEHIO::Pin6);
AnalogInputPin rightOpto(FEHIO::Pin5);
#define LEFT_WHITE_VOLTAGE 2.54
#define LEFT_BLACK_VOLTAGE 4.36
#define MID_WHITE_VOLTAGE 2.51
#define MID_BLACK_VOLTAGE 4.24
#define RIGHT_WHITE_VOLTAGE 1.34
#define RIGHT_BLACK_VOLTAGE 3.92
#define OPTO_ERROR 0.03
#define NO_LINE_FOUND -1
#define ON_LINE 0
#define TO_LEFT 1
#define TO_RIGHT 2

//Pressed = false
DigitalInputPin FLbumper(FEHIO::Pin1);
DigitalInputPin FRbumper(FEHIO::Pin2);
DigitalInputPin BLbumper(FEHIO::Pin3);
DigitalInputPin BRbumper(FEHIO::Pin4);

FEHServo armServo(FEHServo::Servo0);
#define BUCKET_PICK_UP_DEGREE 110
#define BUCKET_CARRY_DEGREE 38
#define TABLE_DROP_DEGREE 80
#define LEVER_OVER_DEGREE 38
#define LEVER_FLICK_DOWN_DEGREE 95
#define LEVER_UNDER_DEGREE 105
#define LEVER_FLICK_UP_DEGREE 72

struct PIDController{
    float previousTime = 0;

    float previousCounts = 0;
    float errorSum = 0;
    float previousVelocityError = 0;

    float Kp = 0.75;
    float Ki = 0.07;
    float Kd = 0.25;

    DigitalEncoder encoder = right_encoder; //Default constructor should never be used
    FEHMotor motor = right_motor;

    PIDController(DigitalEncoder encoder, FEHMotor motor){
        this->encoder = encoder;
        this->motor = motor;
    }

    void reset(){

        encoder.ResetCounts();
        previousCounts = 0;
        errorSum = 0;
        previousVelocityError = 0;

        previousTime = TimeNow();

        Sleep(0.03);
    }

    float PIDAdjustment(float expectedVelocity){
        float deltaCount = encoder.Counts() - previousCounts;
        previousCounts = encoder.Counts();

        float deltaTime = TimeNow() - previousTime;
        previousTime = TimeNow();

        float realVelocity = (deltaCount / COUNTS_PER_INCH) / deltaTime;
        float velocityError = expectedVelocity - realVelocity;

        errorSum += velocityError;

        float pTerm = velocityError * Kp;
        float iTerm = errorSum * Ki;
        float dTerm = (velocityError - previousVelocityError) * Kd;
        previousVelocityError = velocityError;

        return pTerm + iTerm + dTerm;
    }

    void setMotor(float expectedVelocity, float expectedPower, int reverse){
        float adjustment = this->PIDAdjustment(expectedVelocity);

        motor.SetPercent(reverse * (expectedPower + adjustment));
    }

};

PIDController rightPIDController(right_encoder, right_motor);
PIDController leftPIDController(left_encoder, left_motor);

//negative velocity is backwards
void drive(float expectedVelocity, float inches)
{

    float maxCounts = inches * COUNTS_PER_INCH;
    float expectedPower = (expectedVelocity / MAX_VEL) * 100;

    rightPIDController.reset();
    leftPIDController.reset();

    if(expectedVelocity > 0){
        while((left_encoder.Counts() + right_encoder.Counts()) / 2 < maxCounts){

            rightPIDController.setMotor(expectedVelocity, expectedPower, 1);
            leftPIDController.setMotor(expectedVelocity, expectedPower, 1);

            Sleep(0.02);
        }
    }else{
        expectedPower *= -1;
        expectedVelocity *= -1;
        while((left_encoder.Counts() + right_encoder.Counts()) / 2 < maxCounts){

            rightPIDController.setMotor(expectedVelocity, expectedPower, -1);
            leftPIDController.setMotor(expectedVelocity, expectedPower, -1);

            Sleep(0.02);
        }
    }

    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}

//drives forward INCHES until it finds a line, if a line is found will drive on that line for INCHES inches
void driveOnLine(float expectedVelocity, float inches){
    float maxCounts = inches * COUNTS_PER_INCH;
    float expectedPower = (expectedVelocity / MAX_VEL) * 100;

    rightPIDController.reset();
    leftPIDController.reset();

    int state = NO_LINE_FOUND;

    int onLineOffset = 0;

    float leftVal;
    float midVal;
    float rightVal;

    boolean midSeesWhite;
    boolean leftSeesWhite;
    boolean rightSeesWhite;

    boolean midSeesBlack;
    boolean leftSeesBlack;
    boolean rightSeesBlack;

    while((left_encoder.Counts() + right_encoder.Counts() - onLineOffset) / 2 < maxCounts){

        switch (state)
        {
        case NO_LINE_FOUND:
            rightPIDController.setMotor(expectedVelocity, expectedPower, 1);
            leftPIDController.setMotor(expectedVelocity, expectedPower, 1);

            leftVal = leftOpto.Value();
            midVal = midOpto.Value();
            rightVal = rightOpto.Value();

            midSeesWhite = abs(midVal - MID_WHITE_VOLTAGE) < OPTO_ERROR;
            leftSeesWhite = abs(leftVal - LEFT_WHITE_VOLTAGE) < OPTO_ERROR;
            rightSeesWhite = abs(rightVal - RIGHT_WHITE_VOLTAGE) < OPTO_ERROR;

            midSeesBlack = abs(midVal - MID_BLACK_VOLTAGE) < OPTO_ERROR;
            leftSeesBlack = abs(leftVal - LEFT_BLACK_VOLTAGE) < OPTO_ERROR;
            rightSeesBlack = abs(rightVal - RIGHT_BLACK_VOLTAGE) < OPTO_ERROR;

            
            if(midSeesWhite && leftSeesWhite && rightSeesWhite){
                //Intentionally empty
            }else if(midSeesBlack){
                state = ON_LINE;
                onLineOffset = left_encoder.Counts() + right_encoder.Counts();
            }else if(leftSeesBlack || leftSeesWhite){
                state = TO_RIGHT;
                onLineOffset = left_encoder.Counts() + right_encoder.Counts();
            }else if(rightSeesBlack || rightSeesWhite){
                state = TO_LEFT;
                onLineOffset = left_encoder.Counts() + right_encoder.Counts();
            }

            break;
        case ON_LINE:
            rightPIDController.setMotor(expectedVelocity, expectedPower, 1);
            leftPIDController.setMotor(expectedVelocity, expectedPower, 1);

            leftVal = leftOpto.Value();
            rightVal = rightOpto.Value();

            leftSeesBlack = abs(leftVal - LEFT_BLACK_VOLTAGE) < OPTO_ERROR;
            rightSeesBlack = abs(rightVal - RIGHT_BLACK_VOLTAGE) < OPTO_ERROR;

            if(leftSeesBlack && !rightSeesBlack){
                state = TO_RIGHT;
            }else if(rightSeesBlack && !leftSeesBlack){
                state = TO_LEFT;
            }

            break;
        case TO_LEFT:
            rightPIDController.setMotor(expectedVelocity/2, expectedPower/2, -1);
            leftPIDController.setMotor(expectedVelocity/2, expectedPower/2, 1);

            midVal = midOpto.Value();
            leftVal = leftOpto.Value();

            midSeesBlack = abs(midVal - MID_BLACK_VOLTAGE) < OPTO_ERROR;
            leftSeesBlack = abs(leftVal - LEFT_BLACK_VOLTAGE) < OPTO_ERROR;
            rightSeesBlack = abs(rightVal - RIGHT_BLACK_VOLTAGE) < OPTO_ERROR;

            if(midSeesBlack){
                state = ON_LINE;
            }else if(leftSeesBlack && !rightSeesBlack){
                state = TO_RIGHT;
            }

            break;
        case TO_RIGHT:
            rightPIDController.setMotor(expectedVelocity/2, expectedPower/2, 1);
            leftPIDController.setMotor(expectedVelocity/2, expectedPower/2, -1);

            midVal = midOpto.Value();

            midSeesBlack = abs(midVal - MID_BLACK_VOLTAGE) < OPTO_ERROR;
            leftSeesBlack = abs(leftVal - LEFT_BLACK_VOLTAGE) < OPTO_ERROR;
            rightSeesBlack = abs(rightVal - RIGHT_BLACK_VOLTAGE) < OPTO_ERROR;

            if(midSeesBlack){
                state = ON_LINE;
            }else if(rightSeesBlack && !leftSeesBlack){
                state = TO_LEFT;
            }

            break;
        default:
            break;
        }

        Sleep(0.02);
    }

    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}

void driveTilCollision(float expectedVelocity, float timeout, boolean strict){

    float expectedPower = (expectedVelocity / MAX_VEL) * 100;
    float startTime = TimeNow();

    rightPIDController.reset();
    leftPIDController.reset();

    if(expectedVelocity > 0){
        while(TimeNow() - startTime < timeout){

            boolean FLPressed = !FLbumper.Value();
            boolean FRPressed = !FRbumper.Value();

            if(strict){
                if((FLPressed && FRPressed)){
                    break;
                }
            }else{
                if((FLPressed || FRPressed)){
                    break;
                }
            }

            rightPIDController.setMotor(expectedVelocity, expectedPower, 1);
            leftPIDController.setMotor(expectedVelocity, expectedPower, 1);

            Sleep(0.02);
        }
    }else{
        expectedPower *= -1;
        expectedVelocity *= -1;
        while(TimeNow() - startTime < timeout){

            boolean BLPressed = !BLbumper.Value();
            boolean BRPressed = !BRbumper.Value();

            if(strict){
                if((BLPressed && BRPressed)){
                    break;
                }
            }else{
                if((BLPressed || BRPressed)){
                    break;
                }
            }

            rightPIDController.setMotor(expectedVelocity, expectedPower, -1);
            leftPIDController.setMotor(expectedVelocity, expectedPower, -1);

            Sleep(0.02);
        }
    }

    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}

//turn in the center of the wheels, positive = left
void centerAxisTurn(float expectedVelocity, float degrees){

    float turn_ratio = degrees / 360;
    float full_circumference = TRACK_WIDTH_INCHES * PI;
    float drive_inches = full_circumference * turn_ratio;
    float max_counts = drive_inches * COUNTS_PER_INCH * 1.05;

    float expectedPower = (expectedVelocity / MAX_VEL) * 100;

    rightPIDController.reset();
    leftPIDController.reset();

    if(degrees > 0){ //Turn counterclockwise (left)

        while((left_encoder.Counts() + right_encoder.Counts()) / 2 < max_counts){

            rightPIDController.setMotor(expectedVelocity, expectedPower, 1);
            leftPIDController.setMotor(expectedVelocity, expectedPower, -1);

            Sleep(0.02);
        }
    }else{ //Turn clockwise (right)
        max_counts = max_counts * -1;

        while((left_encoder.Counts() + right_encoder.Counts()) / 2 < max_counts){

            rightPIDController.setMotor(expectedVelocity, expectedPower, -1);
            leftPIDController.setMotor(expectedVelocity, expectedPower, 1);

            Sleep(0.02);
        }
    }

    left_motor.Stop();
    right_motor.Stop();

}

//turn on the left wheel, positive = left
void leftAxisTurn(float expectedVelocity, float degrees){

    float turn_ratio = degrees / 360;
    float full_circumference = TRACK_WIDTH_INCHES * PI;
    float drive_inches = full_circumference * turn_ratio * 2;
    float max_counts = drive_inches * COUNTS_PER_INCH * 1.16;

    float expectedPower = (expectedVelocity / MAX_VEL) * 100;

    rightPIDController.reset();

    if(degrees > 0){ //Turn counterclockwise (left)

        while((right_encoder.Counts()) < max_counts){

            rightPIDController.setMotor(expectedVelocity, expectedPower, 1);

            Sleep(0.02);
        }
    }else{ //Turn clockwise (right)
        max_counts = max_counts * -1;

        while((right_encoder.Counts())< max_counts){

            rightPIDController.setMotor(expectedVelocity, expectedPower, -1);

            Sleep(0.02);
        }
    }

    right_motor.Stop();

}

//turn on the right wheel, positive degree = left
void rightAxisTurn(float expectedVelocity, float degrees){

    float turn_ratio = degrees / 360;
    float full_circumference = TRACK_WIDTH_INCHES * PI;
    float drive_inches = full_circumference * turn_ratio * 2;
    float max_counts = drive_inches * COUNTS_PER_INCH * 1.16;

    float expectedPower = (expectedVelocity / MAX_VEL) * 100;

    leftPIDController.reset();

    if(degrees > 0){ //Turn counterclockwise (left)

        while((left_encoder.Counts()) < max_counts){

            leftPIDController.setMotor(expectedVelocity, expectedPower, 1);

            Sleep(0.02);
        }
    }else{ //Turn clockwise (right)
        max_counts = max_counts * -1;

        while((left_encoder.Counts())< max_counts){

            leftPIDController.setMotor(expectedVelocity, expectedPower, -1);

            Sleep(0.02);
        }
    }

    left_motor.Stop();

}

void getMaxVelocities(){
    float previousTime = TimeNow();

    left_encoder.ResetCounts();
    right_encoder.ResetCounts();

    float previousLeftCounts = 0;
    float previousRightCounts = 0;

    right_motor.SetPercent(-100);
    left_motor.SetPercent(100);

    Sleep(0.1);

    while((left_encoder.Counts() + right_encoder.Counts()) / 2 < 24 * COUNTS_PER_INCH){

        int deltaRightCount = right_encoder.Counts() - previousRightCounts;
        int deltaLeftCount = left_encoder.Counts() - previousLeftCounts;

        float deltaTime = TimeNow() - previousTime;

        float realRightVelocity = (deltaRightCount / COUNTS_PER_INCH) / deltaTime;
        float realLeftVelocity = (deltaLeftCount / COUNTS_PER_INCH) / deltaTime;

        LCD.Write("Right Velocity:");
        LCD.WriteLine(realRightVelocity);
        LCD.Write("Left Velocity:");
        LCD.WriteLine(realLeftVelocity);

        previousRightCounts = right_encoder.Counts();
        previousLeftCounts = left_encoder.Counts();
        previousTime = TimeNow();

        Sleep(0.1);
    }

    right_motor.SetPercent(0);
    left_motor.SetPercent(0);
}

void pulse_forward(int percent, float seconds) 
{
    // Set both motors to desired percent
    right_motor.SetPercent(percent);
    left_motor.SetPercent(percent);

    // Wait for the correct number of seconds
    Sleep(seconds);

    // Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}

void pulse_counterclockwise(int percent, float seconds) 
{
    // Set both motors to desired percent
    right_motor.SetPercent(percent);
    left_motor.SetPercent(-percent);

    // Wait for the correct number of seconds
    Sleep(seconds);

    // Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}

//Use RCS to move to the desired x_coordinate based on the orientation of the AruCo code
void fix_x(float x_coordinate)
{
    RCSPose* pose = RCS.RequestPosition();

    float error = abs(pose->x - x_coordinate);

    int headingFix = 1;

    if(pose->heading > 90 && pose->heading < 270){ //facing x = 0 direction (left)
        headingFix = -1;
    }

    int fixCounts = 0;

    while(error > 1 && fixCounts < 10)
    {
        if(pose->x > 0){
            if(pose->x > x_coordinate)
            {
                pulse_forward(-PULSE_POWER * headingFix, PULSE_TIME);
            }
            else if(pose->x < x_coordinate)
            {
                pulse_forward(PULSE_POWER * headingFix, PULSE_TIME);
            }
            fixCounts++;
        }
        Sleep(RCS_WAIT_TIME_IN_SEC);
        pose = RCS.RequestPosition();
        error = abs(pose->x - x_coordinate);
    }

}

//Use RCS to move to the desired y_coordinate based on the orientation of the QR code
void fix_y(float y_coordinate)
{

    RCSPose* pose = RCS.RequestPosition();

    float error = abs(pose->y - y_coordinate);

    int headingFix = -1;

    if(pose->heading < 180){ //facing up
        headingFix = 1;
    }

    int fixCounts = 0;

    while(error > 1 && fixCounts < 10)
    {
        if(pose->y > 0){
            if(pose->y > y_coordinate)
            {
                pulse_forward(-PULSE_POWER * headingFix, PULSE_TIME);
            }
            else if(pose->y < y_coordinate)
            {
                pulse_forward(PULSE_POWER * headingFix, PULSE_TIME);
            }
            fixCounts++;
        }
        Sleep(RCS_WAIT_TIME_IN_SEC);
        pose = RCS.RequestPosition();
        error = abs(pose->y - y_coordinate);
    }
}

//Use RCS to move to the desired heading
void fix_heading(float heading)
{

    RCSPose* pose = RCS.RequestPosition();
    float curHeading = pose->heading;
    float error = curHeading - heading;

    while(abs(curHeading - heading) > 5){
        if(curHeading > 0){
            if(error < 0){
                pulse_counterclockwise(PULSE_POWER, PULSE_TIME);
            }else{
                pulse_counterclockwise(-PULSE_POWER, PULSE_TIME);
            }
            
        }
        Sleep(RCS_WAIT_TIME_IN_SEC);
        pose = RCS.RequestPosition();
        curHeading = pose->heading;
        error = curHeading - heading;
        
    }

}

void waitForStart(float timeout){
    float startTime = TimeNow();
    while(abs(cds.Value() - CDS_RED_LIGHT) > CDS_ERROR && (TimeNow() - startTime) < timeout){
        Sleep(0.01);
    }
}

void printCDSValues(){
    while(true){
        LCD.Write(cds.Value());
        Sleep(0.1);
        LCD.Clear();
    }
}

void printOptoValues(){
    while(true){
        LCD.Write(leftOpto.Value());
        LCD.Write(" ");
        LCD.Write(midOpto.Value());
        LCD.Write(" ");
        LCD.Write(rightOpto.Value());
        Sleep(0.1);
        LCD.Clear();
    }
}

void printRCSValues(){

    while(true){
        RCSPose* pose = RCS.RequestPosition();

        LCD.Write("X: ");
        LCD.WriteLine(pose->x);
        LCD.Write("Y: ");
        LCD.WriteLine(pose->y);
        LCD.Write("Heading: ");
        LCD.WriteLine(pose->heading);

        Sleep(0.1);

        LCD.Clear();
    }
}

void printBumperValues(){
    while(true){
        LCD.Write(FRbumper.Value());
        LCD.Write(" ");
        LCD.Write(FLbumper.Value());
        Sleep(0.1);
        LCD.Clear();
    }
}

void servoDebug(){
    LCD.Clear();
    int degree = 0;
    int lastDegree = -1;
    int height = 40;
    int width = 150;
    int centerPosition = 319/2 - width/2;

    LCD.DrawRectangle(centerPosition, 10, width, height);
    LCD.WriteAt(degree, centerPosition + width/2, 10 + height/2);
    LCD.DrawRectangle(centerPosition, 70, width, height);
    LCD.WriteAt("Degree Up", centerPosition, 70 + height/2);
    LCD.DrawRectangle(centerPosition, 130, width, height);
    LCD.WriteAt("Degree Down", centerPosition, 130 + height/2);

    int x,y;

    while(true){
        if(degree != lastDegree){
            lastDegree = degree;
            armServo.SetDegree(degree);
            LCD.SetFontColor(BLACK);
            LCD.FillRectangle(centerPosition + 5, 15, width - 10, height - 10);
            LCD.SetFontColor(WHITE);
            LCD.WriteAt(degree, centerPosition + width/2, 10 + height/2);
        }
        if(LCD.Touch(&x, &y)){

            if(x > centerPosition && x < centerPosition + width){

                if(y > 70 && y < 70 + height && degree < 180){ //Up
                    degree = degree + 1;
                }
                if(y > 130 && y < 130 + height && degree > 0){ //down
                    degree = degree - 1;
                }

            }

        }
    }
}

//Returns 1 for red, 2 for blue, -1 if timed out
int waitForHumidifierLight(float timeout){
    float startTime = TimeNow();
    while((TimeNow() - startTime) < timeout){

        if(abs(cds.Value() - CDS_RED_LIGHT) < CDS_ERROR){
            LCD.SetFontColor(RED);
            LCD.FillRectangle(0,0,1000,1000);
            return 1;
        }

        if(abs(cds.Value() - CDS_BLUE_LIGHT) < CDS_ERROR){
            LCD.SetFontColor(BLUE);
            LCD.FillRectangle(0,0,1000,1000);
            return 2;
        }

        Sleep(0.01);
    }

    LCD.SetFontColor(BLUE);
    LCD.FillRectangle(0,0,1000,1000);

    return -1;
}

void ERCMain()
{
    armServo.SetMin(500);
    armServo.SetMax(2500);
    // RCS.DisableRateLimit();
    // RCS.InitializeTouchMenu("Z1TESTING");

    //FEHFile *fptr = SD.FOpen("test.txt", "w");

    int touch_x, touch_y;

    LCD.WriteLine("Press to start");
    while (!LCD.Touch(&touch_x, &touch_y));
    while (LCD.Touch(&touch_x, &touch_y));
    LCD.Clear();

    armServo.SetDegree(BUCKET_PICK_UP_DEGREE);
    drive(8, 8);

    driveOnLine(8, 12);

    armServo.SetDegree(BUCKET_CARRY_DEGREE);

    Sleep(1.0);

    driveTilCollision(-12, 5, true);

    drive(-8, 1.0);

    drive(8, 3);

    centerAxisTurn(6, -90);

    driveTilCollision(12, 5, false);

    armServo.SetDegree(TABLE_DROP_DEGREE);

    drive(-8, 3);

    centerAxisTurn(6, 90);

}