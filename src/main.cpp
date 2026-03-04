#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>

#define COUNTS_PER_INCH 40.489
#define TRACK_WIDTH_INCHES 7.375
#define MAX_VEL 42000


DigitalEncoder right_encoder(FEHIO::Pin8);
DigitalEncoder left_encoder(FEHIO::Pin9);

//Right motor has flipped polarity (- percent is forward)
FEHMotor right_motor(FEHMotor::Motor0,9.0);
FEHMotor left_motor(FEHMotor::Motor3,9.0);

struct PIDController{
    float previousTime = TimeNow();

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

        Sleep(0.01);
    }

    float PIDAdjustment(float expectedVelocity){
        int deltaCount = encoder.Counts() - previousCounts;
        previousCounts = encoder.Counts();

        int deltaTime = TimeNow() - previousTime;
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

void drive_forward(int power, float inches)
{

    float maxCounts = inches * COUNTS_PER_INCH;
    float expectedVelocity = power * MAX_VEL;

    rightPIDController.reset();
    leftPIDController.reset();

    while((left_encoder.Counts() + right_encoder.Counts()) / 2 < maxCounts){
        float leftAdjustment = leftPIDController.PIDAdjustment(expectedVelocity);
        float rightAdjustment = rightPIDController.PIDAdjustment(expectedVelocity);
        right_motor.SetPercent(power + rightAdjustment);
        left_motor.SetPercent(power + leftAdjustment);

        Sleep(0.1);
    }

    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}

//Turning circle is defined by the track width times pi. to turn 90 degrees, drive the motors 1/4 of the turning circle

void turn(int percent, float degrees){

    float turn_ratio = degrees / 360;
    float full_circumference = TRACK_WIDTH_INCHES * PI;
    float drive_inches = full_circumference * turn_ratio;
    float max_counts = drive_inches * COUNTS_PER_INCH;

    right_encoder.ResetCounts();
    left_encoder.ResetCounts();

    if(degrees > 0){ //Turn counterclockwise
        right_motor.SetPercent(-percent);
        left_motor.SetPercent(-percent);

        while((left_encoder.Counts() + right_encoder.Counts()) / 2 < max_counts);
    }else{ //Turn clockwise
        right_motor.SetPercent(percent);
        left_motor.SetPercent(percent);

        drive_inches = drive_inches * -1;

        while((left_encoder.Counts() + right_encoder.Counts()) / 2 < max_counts);
    }

    left_motor.Stop();
    right_motor.Stop();

}

void getMaxVelocities(){
    float previousTime = TimeNow();
    float startTime = TimeNow();

    left_encoder.ResetCounts();
    right_encoder.ResetCounts();

    int previousLeftCounts = left_encoder.Counts();
    int previousRightCounts = right_encoder.Counts();

    float leftVelocityTotal = 0;
    float rightVelocityTotal = 0;

    right_motor.SetPercent(-100);
    left_motor.SetPercent(100);

    Sleep(0.1);

    while((left_encoder.Counts() + right_encoder.Counts()) / 2 < 120 * COUNTS_PER_INCH){

        int deltaRightCount = right_encoder.Counts() - previousRightCounts;
        int deltaLeftCount = left_encoder.Counts() - previousLeftCounts;

        float deltaTime = TimeNow() - previousTime;

        float realRightVelocity = (deltaRightCount / COUNTS_PER_INCH) / deltaTime;
        float realLeftVelocity = (deltaLeftCount / COUNTS_PER_INCH) / deltaTime;

        leftVelocityTotal += realLeftVelocity;
        rightVelocityTotal += realRightVelocity;

        previousRightCounts = right_encoder.Counts();
        previousLeftCounts = left_encoder.Counts();
        previousTime = TimeNow();

        Sleep(0.1);
    }

    float rightVelocity = rightVelocityTotal / (TimeNow() - startTime);
    float leftVelocity = leftVelocityTotal / (TimeNow() - startTime);

    LCD.Write("Right Velocity:");
    LCD.WriteLine(rightVelocity);
    LCD.Write("Left Velocity:");
    LCD.WriteLine(leftVelocity);

    right_motor.SetPercent(0);
    left_motor.SetPercent(0);
}

void ERCMain()
{
    //int motor_percent = 50;

    int x, y; //for touch screen

    //Initialize the screen
    LCD.Clear(BLACK);
    LCD.SetFontColor(WHITE);

    LCD.WriteLine("Touch the screen");
    while(!LCD.Touch(&x,&y)); //Wait for screen to be pressed
    while(LCD.Touch(&x,&y)); //Wait for screen to be unpressed
    LCD.Clear();

    getMaxVelocities();

}