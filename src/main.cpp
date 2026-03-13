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
#define CDS_RED_LIGHT 0.27 //these values are good for no filter
#define CDS_BLUE_LIGHT 1.22

DigitalEncoder right_encoder(FEHIO::Pin8); //black ground, red 3.3
DigitalEncoder left_encoder(FEHIO::Pin9);

//Right motor has flipped polarity (- percent is forward)
FEHMotor right_motor(FEHMotor::Motor0,9.0);
FEHMotor left_motor(FEHMotor::Motor3,9.0);

AnalogInputPin cds(FEHIO::Pin0); //orange ground, blue 3.3, yellow signal

AnalogInputPin leftOpto(FEHIO::Pin5);
AnalogInputPin midOpto(FEHIO::Pin6);
AnalogInputPin rightOpto(FEHIO::Pin7);

struct PIDController{
    float previousTime = 0;

    float previousCounts = 0;
    float errorSum = 0;
    float previousVelocityError = 0;

    float Kp = 0.75;
    float Ki = 0.07;
    float Kd = 0.25;

    DigitalEncoder encoder = right_encoder; //Default constructor should never be used

    PIDController(DigitalEncoder encoder){
        this->encoder = encoder;
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

};

PIDController rightPIDController(right_encoder);
PIDController leftPIDController(left_encoder);

void drive(float expectedVelocity, float inches)
{

    float maxCounts = inches * COUNTS_PER_INCH;
    float expectedPower = (expectedVelocity / MAX_VEL) * 100;

    rightPIDController.reset();
    leftPIDController.reset();

    if(expectedVelocity > 0){
        while((left_encoder.Counts() + right_encoder.Counts()) / 2 < maxCounts){

            float leftAdjustment = leftPIDController.PIDAdjustment(expectedVelocity);
            float rightAdjustment = rightPIDController.PIDAdjustment(expectedVelocity);

            right_motor.SetPercent(-1 * (expectedPower + rightAdjustment));
            left_motor.SetPercent(expectedPower + leftAdjustment);

            Sleep(0.05);
        }
    }else{
        expectedPower *= -1;
        expectedVelocity *= -1;
        while((left_encoder.Counts() + right_encoder.Counts()) / 2 < maxCounts){

            float leftAdjustment = leftPIDController.PIDAdjustment(expectedVelocity);
            float rightAdjustment = rightPIDController.PIDAdjustment(expectedVelocity);

            right_motor.SetPercent(expectedPower + rightAdjustment);
            left_motor.SetPercent(-1 * (expectedPower + leftAdjustment));

            Sleep(0.05);
        }
    }

    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}

//Turning circle is defined by the track width times pi. to turn 90 degrees, drive the motors 1/4 of the turning circle
void turn(float power, float degrees){

    float turn_ratio = degrees / 360;
    float full_circumference = TRACK_WIDTH_INCHES * PI;
    float drive_inches = full_circumference * turn_ratio;
    float max_counts = drive_inches * COUNTS_PER_INCH * 1.19;
    float expectedVelocity = MAX_VEL * (power / 100);

    float leftAdjustment = 0;
    float rightAdjustment = 0;

    rightPIDController.reset();
    leftPIDController.reset();

    if(degrees > 0){ //Turn counterclockwise (left)

        while((left_encoder.Counts() + right_encoder.Counts()) / 2 < max_counts){
            right_motor.SetPercent(-1 * (power + rightAdjustment));
            left_motor.SetPercent(-1 * (power + leftAdjustment));

            leftAdjustment = leftPIDController.PIDAdjustment(expectedVelocity);
            rightAdjustment = rightPIDController.PIDAdjustment(expectedVelocity);

            Sleep(0.05);
        }
    }else{ //Turn clockwise (right)
        max_counts = max_counts * -1;

        while((left_encoder.Counts() + right_encoder.Counts()) / 2 < max_counts){
            right_motor.SetPercent(power + rightAdjustment);
            left_motor.SetPercent(power + leftAdjustment);

            leftAdjustment = leftPIDController.PIDAdjustment(expectedVelocity);
            rightAdjustment = rightPIDController.PIDAdjustment(expectedVelocity);

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
    while(abs(cds.Value() - CDS_RED_LIGHT) > 0.05 && (TimeNow() - startTime) < timeout){
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

//Returns 1 for red, 2 for blue, -1 if timed out
int waitForHumidifierLight(float timeout){
    float startTime = TimeNow();
    while((TimeNow() - startTime) < timeout){

        if(abs(cds.Value() - CDS_RED_LIGHT) < 0.05){
            LCD.SetFontColor(RED);
            LCD.FillRectangle(0,0,1000,1000);
            return 1;
        }

        if(abs(cds.Value() - CDS_BLUE_LIGHT) < 0.05){
            LCD.SetFontColor(BLUE);
            LCD.FillRectangle(0,0,1000,1000);
            return 2;
        }

        Sleep(0.01);
    }

    return -1;
}

void ERCMain()
{

    // RCS.DisableRateLimit();
    // RCS.InitializeTouchMenu("Z1TESTING");

    //FEHFile *fptr = SD.FOpen("test.txt", "w");

    int touch_x, touch_y;

    LCD.WriteLine("Press to start #1 and #2");
    while (!LCD.Touch(&touch_x, &touch_y));
    while (LCD.Touch(&touch_x, &touch_y));
    LCD.Clear();

    waitForStart(10);

    drive(-12, 1);

    drive(12, 1);

    turn(25, -45);

    LCD.WriteLine("Press to start #3");
    while (!LCD.Touch(&touch_x, &touch_y));
    while (LCD.Touch(&touch_x, &touch_y));
    LCD.Clear();

    drive(12, 36);

    LCD.WriteLine("Press to start #4, #5, and #6");
    while (!LCD.Touch(&touch_x, &touch_y));
    while (LCD.Touch(&touch_x, &touch_y));
    LCD.Clear();

    int light = waitForHumidifierLight(10);
    drive(12, 4);

    if(light == 1){
        turn(50, 90);
    }
    if(light == 2){
        turn(50, -90);
    }
}