#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>

#define COUNTS_PER_INCH 40.489

//Declarations for encoders & motors
DigitalEncoder right_encoder(FEHIO::Pin10);
DigitalEncoder left_encoder(FEHIO::Pin9);
FEHMotor right_motor(FEHMotor::Motor0,9.0);
FEHMotor left_motor(FEHMotor::Motor1,9.0);

void move_forward(int percent, float inches) //using encoders
{

    float counts = inches * COUNTS_PER_INCH;

    //Reset encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();

    //Set both motors to desired percent
    right_motor.SetPercent(percent);
    left_motor.SetPercent(percent);

    //While the average of the left and right encoder is less than counts,
    //keep running motors
    while((left_encoder.Counts())< counts);

    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}

//Turning circle is defined by the track width times pi. to turn 90 degrees, drive the motors 1/4 of the turning circle

void turn_motors(int percent, float degrees){

    float turn_ratio = degrees / 360;
    float full_circumference = 7 * PI; //7 is track width
    float drive_inches = full_circumference * turn_ratio;

    right_encoder.ResetCounts();
    left_encoder.ResetCounts();

    if(degrees > 0){ //Turn counterclockwise
        right_motor.SetPercent(percent);
        left_motor.SetPercent(-percent);

        while((left_encoder.Counts()) < COUNTS_PER_INCH * drive_inches);
    }else{ //Turn clockwise
        right_motor.SetPercent(-percent);
        left_motor.SetPercent(percent);

        drive_inches = drive_inches * -1;

        while((left_encoder.Counts()) < COUNTS_PER_INCH * drive_inches);
    }

    left_motor.Stop();
    right_motor.Stop();

}

void ERCMain()
{
    int motor_percent = 25; //Input power level here
    int requested_inches = 6;

    int x, y; //for touch screen

    //Initialize the screen
    LCD.Clear(BLACK);
    LCD.SetFontColor(WHITE);

    LCD.WriteLine("Touch the screen");
    while(!LCD.Touch(&x,&y)); //Wait for screen to be pressed
    while(LCD.Touch(&x,&y)); //Wait for screen to be unpressed
    LCD.Clear();

    move_forward(motor_percent, 14); //see function

    Sleep(0.25);

    turn_motors(motor_percent, 90);

    Sleep(0.25);

    move_forward(motor_percent, 10);

    Sleep(0.25);

    turn_motors(motor_percent, -90);

    Sleep(0.25);

    move_forward(motor_percent, 4);


}