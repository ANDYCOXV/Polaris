/*
Author: Andy Cox V 
Date: 5/12/2018
Programming Language: Java (OnBotJava)
Description:
This is the Red Autonomous for the robot for FTC robotics 2017.
This Autonomous is programmed for the X-Drive.
This Autonomous removes the blue jewel and moves the robot into the safe zone.
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.concurrent.TimeUnit;

@Autonomous(name="RedAuto", group="Autonomous")

public class RedAuto extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private final double MOTOR = 0.4; //Motor speed.
    private final int GET_BALL_TIME = 175; //Time to nock off jewel.
    private final int SAFE_ZONE_TIME = 1500; //Time to get into safe zone.
    private final int ROTATE_TIME = 125; //Time to rotate the robot.
    private final boolean RIGHT = false; //Rotate robot right.
    private final boolean LEFT = true; //Rotate robot left.
    
    /* Declare motor and servo variables. */
    private DcMotor motor1 = null;
    private DcMotor motor2 = null;
    private DcMotor motor3 = null;
    private DcMotor motor4 = null;
    private DcMotor armmotor = null;
    private Servo servoL = null;
    private Servo servoR = null;
    private Servo servoArm = null;
    private ColorSensor color = null;
    
    //Used for move function.
    private final int BACKWARDS = 2;
    private final int FORWARDS = 1;
    private final int BREAK = 0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        //Initilize Hardware.
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor4 = hardwareMap.get(DcMotor.class, "motor4");
        armmotor = hardwareMap.get(DcMotor.class, "armmotor");
        servoL = hardwareMap.get(Servo.class, "servoL");
        servoR = hardwareMap.get(Servo.class, "servoR");
        servoArm = hardwareMap.get(Servo.class, "servoArm");
        color = hardwareMap.get(ColorSensor.class, "colorsensor");
        
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        waitForStart();
        runtime.reset();

        //Grab the block and lift.
        //Setup the servos.
        servoL.setPosition(-1);
        servoR.setPosition(1);
        armmotor.setPower(0.5);
        delay(2000);
        armmotor.setPower(0);

        servoArm.setPosition(1); //Drop robot arm.
        delay(2000);
        
        //Use for finding the color of the jewel.
        if(color.red() > color.blue()) //Red ball is front.
        {
            rotate(LEFT);
            delay(ROTATE_TIME);
            rotate(RIGHT);
            delay(ROTATE_TIME);
            move(BACKWARDS);
        }
        
        //Seperate into two diffrent statments to prevent complicaitons.
        if(color.red() < color.blue()) //Blue ball is front.
        {
            rotate(RIGHT);
            delay(ROTATE_TIME);
            rotate(LEFT);
            delay(ROTATE_TIME);
            move(FORWARDS);
        }

            delay(GET_BALL_TIME); //Move the robot enough to nock off the jewel.
            move(BREAK);
            
            servoArm.setPosition(0); //Raise the arm.
            delay(1500); //Delay to raise the arm.
            
            move(FORWARDS); //Reposition robot to move to safe zone.
            delay(SAFE_ZONE_TIME);
            move(BREAK);
    }
    
    //Used for timed delays in milliseconds.
    public void delay(int time)
    {
        try
        {
            TimeUnit.MILLISECONDS.sleep(time);
        }
        catch(Exception e)
        {
            telemetry.addData("Status", "ERROR");
            telemetry.update();
        }
    }
    
    //Function to move the robot forwards = 1, backwards = 2, or break = 0.
    public void move(int type)
    {
        double a = 0;
        double b = 0;
        double c = 0;
        double d = 0;
        
        switch(type)
        {
            case 0: //Break.
                a = 0;
                b = 0;
                c = 0;
                d = 0;
                break;
            case 1: //Forwards.
                a = MOTOR;
                b = -MOTOR;
                c = MOTOR;
                d = -MOTOR;
                break;
            case 2: //Backwards.
                a = -MOTOR;
                b = MOTOR;
                c = -MOTOR;
                d = MOTOR;
                break;
        }
        
        //Set the motors power.
        motor1.setPower(a);
        motor2.setPower(b);
        motor3.setPower(c);
        motor4.setPower(d);
    }
    
    //Function to move the robot right = false, left = true.
    public void rotate(boolean type)
    {
        double a = MOTOR;
        double b = MOTOR;
        double c = MOTOR;
        double d = MOTOR;
        
        if(type) //Move left.
        {
            a *= -1;
            b *= -1;
            c *= -1;
            d *= -1;
        }
        
        //Set the motors power.
        motor1.setPower(a);
        motor2.setPower(b);
        motor3.setPower(c);
        motor4.setPower(d);
    }
}
