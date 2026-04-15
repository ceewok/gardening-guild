#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHRCS.h>
#include <FEHSD.h>
#include <FEHServo.h>
#include "FEHBuzzer.h"

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
#define BUCKET_CARRY_DEGREE 0
#define TABLE_DROP_DEGREE 80
#define BASKET_DROP_DEGREE 125
#define LEVER_OVER_DEGREE 38
#define LEVER_FLICK_DOWN_DEGREE 95
#define LEVER_UNDER_DEGREE 105
#define LEVER_FLICK_UP_DEGREE 72

FEHMotor compostMotor(FEHMotor::Motor2, 5.0);

FEHServo windowServo(FEHServo::Servo7);

bool fixPosition(float expectedX, float expectedY, float expectedHeading);

struct coordinate{
    float x = 0;
    float y = 0;
    float h = 0;

    coordinate(float x, float y, float h){
        this->x = x;
        this->y = y;
        this->h = h;
    }

    void set(float x, float y, float h){
        this->x = x;
        this->y = y;
        this->h = h;
    }

    bool go(){
        return fixPosition(x, y, h);
    }
};

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

        Sleep(0.01);
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

    float maxCounts = inches * COUNTS_PER_INCH * 1.07;
    float expectedPower = (expectedVelocity / MAX_VEL) * 100;

    rightPIDController.reset();
    leftPIDController.reset();

    if(expectedVelocity > 0){
        while((left_encoder.Counts() + right_encoder.Counts()) / 2 < maxCounts){

            rightPIDController.setMotor(expectedVelocity, expectedPower, 1);
            leftPIDController.setMotor(expectedVelocity, expectedPower, 1);

            Sleep(0.01);
        }
    }else{
        expectedPower *= -1;
        expectedVelocity *= -1;
        while((left_encoder.Counts() + right_encoder.Counts()) / 2 < maxCounts){

            rightPIDController.setMotor(expectedVelocity, expectedPower, -1);
            leftPIDController.setMotor(expectedVelocity, expectedPower, -1);

            Sleep(0.01);
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

        Sleep(0.01);
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

            Sleep(0.01);
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

            Sleep(0.01);
        }
    }

    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}

//turn in the center of the wheels, positive degree = left
void centerAxisTurn(float expectedVelocity, float degrees){

    float turn_ratio = degrees / 360;
    float full_circumference = TRACK_WIDTH_INCHES * PI;
    float drive_inches = full_circumference * turn_ratio;
    float max_counts = drive_inches * COUNTS_PER_INCH * 1.07;

    float expectedPower = (expectedVelocity / MAX_VEL) * 100;

    rightPIDController.reset();
    leftPIDController.reset();

    if(degrees > 0){ //Turn counterclockwise (left)

        while((left_encoder.Counts() + right_encoder.Counts()) / 2 < max_counts){

            rightPIDController.setMotor(expectedVelocity, expectedPower, 1);
            leftPIDController.setMotor(expectedVelocity, expectedPower, -1);

            Sleep(0.01);
        }
    }else{ //Turn clockwise (right)
        max_counts = max_counts * -1;

        while((left_encoder.Counts() + right_encoder.Counts()) / 2 < max_counts){

            rightPIDController.setMotor(expectedVelocity, expectedPower, -1);
            leftPIDController.setMotor(expectedVelocity, expectedPower, 1);

            Sleep(0.01);
        }
    }

    left_motor.Stop();
    right_motor.Stop();

}

//turn on the left wheel, positive degree = left
void leftAxisTurn(float expectedVelocity, float degrees){

    float turn_ratio = degrees / 360;
    float full_circumference = TRACK_WIDTH_INCHES * PI;
    float drive_inches = full_circumference * turn_ratio * 2;
    float max_counts = drive_inches * COUNTS_PER_INCH * 1.07;

    float expectedPower = (expectedVelocity / MAX_VEL) * 100;

    rightPIDController.reset();

    if(degrees > 0){ //Turn counterclockwise (left)

        while((right_encoder.Counts()) < max_counts){

            rightPIDController.setMotor(expectedVelocity, expectedPower, 1);

            Sleep(0.01);
        }
    }else{ //Turn clockwise (right)
        max_counts = max_counts * -1;

        while((right_encoder.Counts())< max_counts){

            rightPIDController.setMotor(expectedVelocity, expectedPower, -1);

            Sleep(0.01);
        }
    }

    right_motor.Stop();

}

//turn on the right wheel, positive degree = right
void rightAxisTurn(float expectedVelocity, float degrees){

    float turn_ratio = degrees / 360;
    float full_circumference = TRACK_WIDTH_INCHES * PI;
    float drive_inches = full_circumference * turn_ratio * 2;
    float max_counts = drive_inches * COUNTS_PER_INCH * 1.07;

    float expectedPower = (expectedVelocity / MAX_VEL) * 100;

    leftPIDController.reset();

    if(degrees > 0){ //Turn clockwise (right)

        while((left_encoder.Counts()) < max_counts){

            leftPIDController.setMotor(expectedVelocity, expectedPower, 1);

            Sleep(0.01);
        }
    }else{ //Turn counterclockwise (left)
        max_counts = max_counts * -1;

        while((left_encoder.Counts())< max_counts){

            leftPIDController.setMotor(expectedVelocity, expectedPower, -1);

            Sleep(0.01);
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

    return -1;
}

void slowSetDegree(int targetDegree, int currentDegree, float stepTime){
    int deg = currentDegree;
    while(true){
        if(targetDegree < currentDegree){
            if(abs(deg - targetDegree) < 10){
                armServo.SetDegree(targetDegree);
                break;
            }else{
                deg -= 3;
                armServo.SetDegree(deg);
            }
        }else{
            if(abs(deg - targetDegree) < 10){
                armServo.SetDegree(targetDegree);
                break;
            }else{
                deg += 3;
                armServo.SetDegree(deg);
            }
        }
        Sleep(stepTime);
    }
}

bool getPosition(float *x, float *y, float *heading, int retries){
    int tries = 0;
    while(tries < retries){
        tries++;
        RCSPose* pose = RCS.RequestPosition();
        if(pose != nullptr){
            *x = pose->x;
            *y = pose->y;
            *heading = pose->heading;
            return true;
        }
    }
    return false;
}

float wrapAngle(float angle) {
    while (angle > 180) angle -= 360;
    while (angle < -180) angle += 360;
    return angle;
}

bool fixPosition(float expectedX, float expectedY, float expectedHeading){

    float x,y,heading;
    if (!getPosition(&x,&y,&heading,2)) return false;
    if (x < 0 || y < 0) return false;

    LCD.Write(x);
    LCD.Write(" ");
    LCD.Write(y);
    LCD.Write(" ");
    LCD.Write(heading);
    LCD.WriteLine(" ");
    LCD.Write(expectedX);
    LCD.Write(" ");
    LCD.Write(expectedY);
    LCD.Write(" ");
    LCD.Write(expectedHeading);
    LCD.WriteLine(" ");

    float dx = expectedX - x;
    float dy = expectedY - y;
    float dist = hypot(dx, dy);
    float mathAngle = atan2(dy, dx) * (180.0/PI);
    float targetAngle = wrapAngle(mathAngle - 90);

    LCD.Write(dist);
    LCD.Write(" ");
    LCD.Write(targetAngle);
    LCD.Write(" ");

    float forwardTurn = wrapAngle(targetAngle - heading);
    float backwardTurn = wrapAngle((targetAngle + 180) - heading);

    LCD.Write(forwardTurn);
    LCD.Write(" ");
    LCD.Write(backwardTurn);
    LCD.WriteLine(" ");

    if(abs(forwardTurn) < abs(backwardTurn)){
        centerAxisTurn(6, forwardTurn);
        heading = targetAngle;
        drive(6, dist);
    }else{
        centerAxisTurn(6, backwardTurn);
        heading = wrapAngle(targetAngle + 180);
        drive(-6, dist);
    }

    float finalTurn = wrapAngle(expectedHeading - heading);
    centerAxisTurn(6, finalTurn);

    LCD.Write(finalTurn);
    return true;
}

coordinate appleBucket(0,0,0);
void initAppleBucketCoordinate(){
    int touch_x,touch_y;
    float x,y,heading;

    armServo.SetDegree(BUCKET_PICK_UP_DEGREE);
    LCD.WriteLine("Apple bucket");
    while (!LCD.Touch(&touch_x, &touch_y));
    while (LCD.Touch(&touch_x, &touch_y));
    while(x < 0){
        getPosition(&x,&y,&heading,1);
    }
    appleBucket.set(x, y, heading);
    Buzzer.Beep();
    LCD.Clear();
}

coordinate table(0,0,0);
void initTableCoordinate(){
    int touch_x,touch_y;
    float x = -1,y = -1,heading = -1;

    armServo.SetDegree(TABLE_DROP_DEGREE);
    LCD.WriteLine("Table");
    while (!LCD.Touch(&touch_x, &touch_y));
    while (LCD.Touch(&touch_x, &touch_y));
    while(x <= 0 && y <= 0 && (heading < 0 || heading > 360)){
        getPosition(&x,&y,&heading,1);
    }
    table.set(x, y, heading);
    Buzzer.Beep();
    Sleep(1.5);
    armServo.SetDegree(0);
    LCD.Clear();
}

coordinate windowOpen(0,0,0);
void initWindowOpenCoordinate(){
    int touch_x,touch_y;
    float x,y,heading;

    windowServo.SetDegree(90);
    LCD.WriteLine("Window Open");
    while (!LCD.Touch(&touch_x, &touch_y));
    while (LCD.Touch(&touch_x, &touch_y));
    while(x < 0){
        getPosition(&x,&y,&heading,1);
    }
    windowOpen.set(x, y, heading);
    windowServo.SetDegree(0);
    Buzzer.Beep();
    LCD.Clear();
}

coordinate windowClosed(0,0,0);
void initWindowClosedCoordinate(){
    int touch_x,touch_y;
    float x,y,heading;

    windowServo.SetDegree(0);
    LCD.WriteLine("Window Closed");
    while (!LCD.Touch(&touch_x, &touch_y));
    while (LCD.Touch(&touch_x, &touch_y));
    while(x < 0){
        getPosition(&x,&y,&heading,1);
    }
    windowClosed.set(x, y, heading);
    Buzzer.Beep();
    LCD.Clear();
}

coordinate humidifier(0,0,0);
void initHumidifierCoordinate(){
    int touch_x,touch_y;
    float x,y,heading;

    LCD.WriteLine("Humidifier");
    while (!LCD.Touch(&touch_x, &touch_y));
    while (LCD.Touch(&touch_x, &touch_y));
    while(x < 0){
        getPosition(&x,&y,&heading,1);
    }
    humidifier.set(x, y, heading);
    Buzzer.Beep();
    LCD.Clear();
}

void initRCSCoordinates(){
    int touch_x,touch_y;
    float x,y,heading;

    initAppleBucketCoordinate();
    Sleep(1.0);

    initTableCoordinate();
    Sleep(1.0);

    initWindowClosedCoordinate();
    Sleep(1.0);

    initWindowOpenCoordinate();
    Sleep(1.0);

    initHumidifierCoordinate();
    LCD.Clear();
}

void task_ComposterFromStart(int driveSpeed, int collisionDriveSpeed, int turnSpeed){
    drive(-driveSpeed, 1); //8

    drive(driveSpeed, 15);

    centerAxisTurn(turnSpeed, -45); //6

    driveTilCollision(-collisionDriveSpeed, 3, true); //12

    rightAxisTurn(turnSpeed, 90);

    leftAxisTurn(turnSpeed, 3);

    drive(-6, 15);

    compostMotor.SetPercent(-80);

    Sleep(1.5);

    compostMotor.SetPercent(80);

    Sleep(1.5);

    compostMotor.SetPercent(-20);
    Sleep(0.5);

    drive(driveSpeed, 4);
    compostMotor.Stop();

    centerAxisTurn(turnSpeed, 90);

    drive(driveSpeed, 16);

    centerAxisTurn(turnSpeed, -90);

    driveTilCollision(collisionDriveSpeed, 2, true);

    drive(-driveSpeed, 12);

    centerAxisTurn(turnSpeed, -90);

    driveTilCollision(-collisionDriveSpeed, 2, true);

    drive(driveSpeed, 11.25);

    centerAxisTurn(turnSpeed, 90);
}

void task_appleBucket(int driveSpeed, int collisionDriveSpeed, int turnSpeed){
    armServo.SetDegree(BUCKET_PICK_UP_DEGREE);

    drive(driveSpeed, 2.5);

    slowSetDegree(BUCKET_CARRY_DEGREE, BUCKET_PICK_UP_DEGREE, 0.05);

    drive(-driveSpeed, 4);

    centerAxisTurn(turnSpeed, -45);

    drive(-driveSpeed, 6);

    centerAxisTurn(turnSpeed, 45);

    driveTilCollision(-collisionDriveSpeed, 3, true);

    rightAxisTurn(turnSpeed, 90);

    drive(driveSpeed, 23);

    centerAxisTurn(turnSpeed, 90);

    driveTilCollision(-collisionDriveSpeed, 2, true);

    drive(driveSpeed, 10);

    centerAxisTurn(turnSpeed, -90);

    driveTilCollision(collisionDriveSpeed, 3, false);

    drive(-driveSpeed, 3.5);

    slowSetDegree(BASKET_DROP_DEGREE, BUCKET_CARRY_DEGREE, 0.05);

    drive(-driveSpeed, 6);

    armServo.SetDegree(0);

    centerAxisTurn(turnSpeed, 90);

    driveTilCollision(-collisionDriveSpeed, 3, true);

    drive(driveSpeed, 17.5);
}

void task_humidifier(int driveSpeed, int turnSpeed){
    int light = waitForHumidifierLight(0.5);
    do{
        if(light == -1){
            drive(driveSpeed, 0.5);
            light = waitForHumidifierLight(0.25);
            if(light > 0) break;
            drive(-driveSpeed, 1);
            light = waitForHumidifierLight(0.25);
            if(light > 0) break;
            drive(-driveSpeed, 0.5);
            light = waitForHumidifierLight(0.25);
            if(light > 0) break;
        }
    }while(false);

    if(light == 1){
        centerAxisTurn(turnSpeed, -17);
        drive(driveSpeed, 8);
    }else{
        centerAxisTurn(turnSpeed, 17);
        drive(driveSpeed, 8);
    }

}

void task_leversFromHumidifier(int driveSpeed, int collisionDriveSpeed, int turnSpeed){
    driveTilCollision(collisionDriveSpeed, 2, false);
    drive(driveSpeed, 2);
    int lever = RCS.GetLever();

    if(lever == 0){ //left
        drive(-driveSpeed, 6); //MEASURE
        centerAxisTurn(turnSpeed, -45);
        armServo.SetDegree(LEVER_OVER_DEGREE);
        drive(driveSpeed, 8); //MEASURE
        armServo.SetDegree(LEVER_FLICK_DOWN_DEGREE);
        Sleep(0.1);
        float timeFlickedDown = TimeNow();
        drive(-driveSpeed, 4);
        armServo.SetDegree(LEVER_UNDER_DEGREE);
        while(TimeNow() - timeFlickedDown < 5) Sleep(0.1);
        drive(driveSpeed, 4);
    }else if(lever == 1){ //middle
        drive(-driveSpeed, 12); //MEASURE
        centerAxisTurn(turnSpeed, -45);
        armServo.SetDegree(LEVER_OVER_DEGREE);
        drive(driveSpeed, 11.5); //MEASURE
        armServo.SetDegree(LEVER_FLICK_DOWN_DEGREE);
        Sleep(0.1);
        float timeFlickedDown = TimeNow();
        drive(-driveSpeed, 4);
        armServo.SetDegree(LEVER_UNDER_DEGREE);
        while(TimeNow() - timeFlickedDown < 5) Sleep(0.1);
        drive(driveSpeed, 4);
    }else{ //right
        drive(-driveSpeed, 12); //MEASURE
        centerAxisTurn(turnSpeed, -57);
        armServo.SetDegree(LEVER_OVER_DEGREE);
        drive(driveSpeed, 11.5); //MEASURE
        armServo.SetDegree(LEVER_FLICK_DOWN_DEGREE);
        Sleep(0.1);
        float timeFlickedDown = TimeNow();
        drive(-driveSpeed, 4);
        armServo.SetDegree(LEVER_UNDER_DEGREE);
        while(TimeNow() - timeFlickedDown < 5) Sleep(0.1);
        drive(driveSpeed, 4);
        armServo.SetDegree(LEVER_FLICK_UP_DEGREE);
    }
    drive(driveSpeed, -4);
    armServo.SetDegree(0);
}

void task_window(int driveSpeed){
    windowServo.SetDegree(0);
    drive(driveSpeed, -1); //MEASURE
    windowServo.SetDegree(90);
    Sleep(0.2);
    drive(-driveSpeed, -1); //MEASURE
}

void ERCMain()
{
    //FEHFile *fptr = SD.FOpen("test.txt", "w");
    windowServo.SetMin(600);
    windowServo.SetMax(2150);
    windowServo.SetDegree(0);
    armServo.SetMin(500);
    armServo.SetMax(2500);
    armServo.SetDegree(0);
    //RCS.InitializeTouchMenu("0910B9LKI");

    WaitForFinalAction();

    task_ComposterFromStart(8, 12, 6);

    task_appleBucket(8, 12, 6);

    task_humidifier(8, 6);

}