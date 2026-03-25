#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHRCS.h>
#include <FEHSD.h>

#define COUNTS_PER_INCH 40.489
#define TRACK_WIDTH_INCHES 7.375
#define MAX_VEL 24
#define PULSE_TIME 0.03
#define PULSE_POWER 35
#define RCS_WAIT_TIME_IN_SEC 0.1

DigitalEncoder right_encoder(FEHIO::Pin8); //black ground, red 3.3
DigitalEncoder left_encoder(FEHIO::Pin9);

FEHMotor right_motor(FEHMotor::Motor1,9.0);
FEHMotor left_motor(FEHMotor::Motor3,9.0);

AnalogInputPin cds(FEHIO::Pin0); //orange ground, blue 3.3, yellow signal
#define CDS_RED_LIGHT 0.22 //these values are good for no filter
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
#define OPTO_ERROR 0.05
#define NO_LINE_FOUND -1
#define ON_LINE 0
#define TO_LEFT 1
#define TO_RIGHT 2

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

            Sleep(0.05);
        }
    }else{
        expectedPower *= -1;
        expectedVelocity *= -1;
        while((left_encoder.Counts() + right_encoder.Counts()) / 2 < maxCounts){

            rightPIDController.setMotor(expectedVelocity, expectedPower, -1);
            leftPIDController.setMotor(expectedVelocity, expectedPower, -1);

            Sleep(0.05);
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

    while((left_encoder.Counts() + right_encoder.Counts() - onLineOffset) / 2 < maxCounts){

        switch (state)
        {
        case NO_LINE_FOUND:
            rightPIDController.setMotor(expectedVelocity, expectedPower, 1);
            leftPIDController.setMotor(expectedVelocity, expectedPower, 1);

            leftVal = leftOpto.Value();
            midVal = midOpto.Value();
            rightVal = rightOpto.Value();

            if(abs(midVal - MID_BLACK_VOLTAGE) < OPTO_ERROR){
                state = ON_LINE;
                onLineOffset = left_encoder.Counts() + right_encoder.Counts();
            }else if(abs(leftVal - LEFT_BLACK_VOLTAGE) < OPTO_ERROR || abs(leftVal - LEFT_WHITE_VOLTAGE) < OPTO_ERROR){
                state = TO_RIGHT;
                onLineOffset = left_encoder.Counts() + right_encoder.Counts();
            }else if(abs(rightVal - RIGHT_BLACK_VOLTAGE) < OPTO_ERROR || abs(rightVal - RIGHT_WHITE_VOLTAGE) < OPTO_ERROR){
                state = TO_LEFT;
                onLineOffset = left_encoder.Counts() + right_encoder.Counts();
            }

            break;
        case ON_LINE:
            rightPIDController.setMotor(expectedVelocity, expectedPower, 1);
            leftPIDController.setMotor(expectedVelocity, expectedPower, 1);

            leftVal = leftOpto.Value();
            rightVal = rightOpto.Value();

            if(abs(leftVal - LEFT_BLACK_VOLTAGE) < OPTO_ERROR && !(abs(rightVal - RIGHT_BLACK_VOLTAGE) < OPTO_ERROR)){
                state = TO_RIGHT;
            }else if(abs(rightVal - RIGHT_BLACK_VOLTAGE) < OPTO_ERROR && !(abs(leftVal - LEFT_BLACK_VOLTAGE) < OPTO_ERROR)){
                state = TO_LEFT;
            }

            break;
        case TO_LEFT:
            rightPIDController.setMotor(expectedVelocity/2, expectedPower/2, -1);
            leftPIDController.setMotor(expectedVelocity/2, expectedPower/2, 1);

            midVal = midOpto.Value();

            if(abs(midVal - MID_BLACK_VOLTAGE) < OPTO_ERROR){
                state = ON_LINE;
            }

            break;
        case TO_RIGHT:
            rightPIDController.setMotor(expectedVelocity/2, expectedPower/2, 1);
            leftPIDController.setMotor(expectedVelocity/2, expectedPower/2, -1);

            midVal = midOpto.Value();

            if(abs(midVal - MID_BLACK_VOLTAGE) < OPTO_ERROR){
                state = ON_LINE;
            }

            break;
        default:
            break;
        }

        Sleep(0.05);
    }

    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}

//Turning circle is defined by the track width times pi. to turn 90 degrees, drive the motors 1/4 of the turning circle
void turn(float expectedVelocity, float degrees){

    float turn_ratio = degrees / 360;
    float full_circumference = TRACK_WIDTH_INCHES * PI;
    float drive_inches = full_circumference * turn_ratio;
    float max_counts = drive_inches * COUNTS_PER_INCH * 1.17;

    float expectedPower = (expectedVelocity / MAX_VEL) * 100;

    rightPIDController.reset();
    leftPIDController.reset();

    if(degrees > 0){ //Turn counterclockwise (left)

        while((left_encoder.Counts() + right_encoder.Counts()) / 2 < max_counts){

            rightPIDController.setMotor(expectedVelocity, expectedPower, 1);
            leftPIDController.setMotor(expectedVelocity, expectedPower, -1);

            Sleep(0.05);
        }
    }else{ //Turn clockwise (right)
        max_counts = max_counts * -1;

        while((left_encoder.Counts() + right_encoder.Counts()) / 2 < max_counts){

            rightPIDController.setMotor(expectedVelocity, expectedPower, -1);
            leftPIDController.setMotor(expectedVelocity, expectedPower, 1);

            Sleep(0.05);
        }
    }

    left_motor.Stop();
    right_motor.Stop();

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

    // RCS.DisableRateLimit();
    // RCS.InitializeTouchMenu("Z1TESTING");

    //FEHFile *fptr = SD.FOpen("test.txt", "w");

    int touch_x, touch_y;

    LCD.WriteLine("Press to start");
    while (!LCD.Touch(&touch_x, &touch_y));
    while (LCD.Touch(&touch_x, &touch_y));
    LCD.Clear();

    driveOnLine(6, 24);
}