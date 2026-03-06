#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>

#define COUNTS_PER_INCH 40.489
#define TRACK_WIDTH_INCHES 7.375
#define MAX_VEL 24

DigitalEncoder right_encoder(FEHIO::Pin8);
DigitalEncoder left_encoder(FEHIO::Pin9);

//Right motor has flipped polarity (- percent is forward)
FEHMotor right_motor(FEHMotor::Motor0,9.0);
FEHMotor left_motor(FEHMotor::Motor3,9.0);

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

void drive(float power, float inches)
{

    float maxCounts = inches * COUNTS_PER_INCH;
    float expectedVelocity = MAX_VEL * (power / 100);

    rightPIDController.reset();
    leftPIDController.reset();

    if(power > 0){
        while((left_encoder.Counts() + right_encoder.Counts()) / 2 < maxCounts){

            float leftAdjustment = leftPIDController.PIDAdjustment(expectedVelocity);
            float rightAdjustment = rightPIDController.PIDAdjustment(expectedVelocity);

            right_motor.SetPercent(-1 * (power + rightAdjustment));
            left_motor.SetPercent(power + leftAdjustment);

            Sleep(0.05);
        }
    }else{
        power *= -1;
        expectedVelocity *= -1;
        while((left_encoder.Counts() + right_encoder.Counts()) / 2 < maxCounts){

            float leftAdjustment = leftPIDController.PIDAdjustment(expectedVelocity);
            float rightAdjustment = rightPIDController.PIDAdjustment(expectedVelocity);

            right_motor.SetPercent(power + rightAdjustment);
            left_motor.SetPercent(-1 * (power + leftAdjustment));

            Sleep(0.05);
        }
    }

    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}

void newDrive(float expectedVelocity, float inches)
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

    if(degrees > 0){ //Turn counterclockwise

        while((left_encoder.Counts() + right_encoder.Counts()) / 2 < max_counts){
            right_motor.SetPercent(-1 * (power + rightAdjustment));
            left_motor.SetPercent(-1 * (power + leftAdjustment));

            leftAdjustment = leftPIDController.PIDAdjustment(expectedVelocity);
            rightAdjustment = rightPIDController.PIDAdjustment(expectedVelocity);

            Sleep(0.05);
        }
    }else{ //Turn clockwise
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

void ERCMain()
{

    int x, y;

    LCD.Clear(BLACK);
    LCD.SetFontColor(WHITE);

    LCD.WriteLine("Touch the screen");
    while(!LCD.Touch(&x,&y)); //Wait for screen to be pressed
    while(LCD.Touch(&x,&y)); //Wait for screen to be unpressed
    LCD.Clear();

    newDrive(12, 30);

    Sleep(0.25);

    newDrive(-12, 30);

}