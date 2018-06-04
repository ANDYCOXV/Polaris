/*
Author: Andy Cox V 
Date: 5/12/2018
Programming Language: Java (OnBotJava)
Description:
This is the manual control for a robot that uses X-Drive for omni-directional
movement. A claw for grabbing two glyps and a rack an pinion for vertical
movement of the blocks.
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import java.util.Map;
import java.util.Set;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.SimpleDateFormat;
import java.util.Date;

@TeleOp(name="XDrive", group="Iterative Opmode")

public class XDrive extends OpMode
{
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    
    /* Declare motor and servo variables. */
    private DcMotor motor1 = null;
    private DcMotor motor2 = null;
    private DcMotor motor3 = null;
    private DcMotor motor4 = null;
    private DcMotor armmotor = null;
    private Servo servoL = null;
    private Servo servoR = null;

    //Declare Constants
    private static final double SERVO_STEPS = 0.02; //Speed for the claw.
    //TURBO
    private static final double TURBO_SENSITIVITY = 0.75;
    private static final double TURBO_ROTATION_SPEED = 0.5;
    //TURTLE
    private static final double TURTLE_SENSITIVITY = 0.375;
    private static final double TURTLE_ROTATION_SPEED = 0.25;
    
    //Declare global variables.
    private double GLOBAL_sensitivity = TURTLE_SENSITIVITY;
    private double GLOBAL_rotationSpeed = TURTLE_ROTATION_SPEED;

    /* Code to run ONCE when the driver hits INIT */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        
        //Initilize motors and servos -- Keep here for exception handling!
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor4 = hardwareMap.get(DcMotor.class, "motor4");
        armmotor = hardwareMap.get(DcMotor.class, "armmotor");
        servoL = hardwareMap.get(Servo.class, "servoL");
        servoR = hardwareMap.get(Servo.class, "servoR");
        
        //Setup the motor directions.
        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.FORWARD);
        motor3.setDirection(DcMotor.Direction.FORWARD);
        motor4.setDirection(DcMotor.Direction.FORWARD);
        armmotor.setDirection(DcMotor.Direction.FORWARD);
        
        //Setup the motor zero power behaviors.
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        armmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        //Setup the servos.
        servoL.scaleRange(0, 0.7);
        servoR.scaleRange(0.2, 1);
    }

    /* Code to run REPEATEDLY after INIT, but before PLAY */
    @Override //Not needed.
    public void init_loop() {
    }

    /* Code to run ONCE when the driver hits PLAY */
    @Override
    public void start() {
        runtime.reset();
    }

    /* Code to run REPEATEDLY after PLAY but before STOP */
    @Override
    public void loop() {
        
        double yPos = gamepad1.left_stick_y * GLOBAL_sensitivity; //y-position.
        double xPos = gamepad1.left_stick_x * GLOBAL_sensitivity; //x-position.
        
        //Armmotor controll.
        if(gamepad1.right_stick_y != 0)
        {
            if(gamepad1.right_stick_y > 0)
                armmotor.setPower(-1);
            else
                armmotor.setPower(1);
        }
        else
            armmotor.setPower(0);
        
        //Horizontal and vertical steering.
        if(Math.abs(gamepad1.left_stick_y) > Math.abs(gamepad1.left_stick_x))
        {
            SetMotorBehavor(-yPos, yPos, -yPos, yPos);
        }
        else if(Math.abs(gamepad1.left_stick_y) < Math.abs(gamepad1.left_stick_x))
        {
            motor2.setDirection(DcMotor.Direction.REVERSE);
            SetMotorBehavor(xPos, xPos, -xPos, -xPos);
            motor2.setDirection(DcMotor.Direction.FORWARD);
        }
        else
        {
            SetMotorBehavor(0,0,0,0);
        }

        if(gamepad1.left_bumper) //Turning Left.
            SetMotorBehavor(GLOBAL_rotationSpeed, GLOBAL_rotationSpeed, 
            GLOBAL_rotationSpeed, GLOBAL_rotationSpeed);
        
        if(gamepad1.right_bumper) //Turning Right.
            SetMotorBehavor(-GLOBAL_rotationSpeed, -GLOBAL_rotationSpeed, 
            -GLOBAL_rotationSpeed, -GLOBAL_rotationSpeed);

        if(gamepad1.dpad_up) //Set the speed for turbo.
        {
            GLOBAL_sensitivity = TURBO_SENSITIVITY;
            GLOBAL_rotationSpeed = TURBO_ROTATION_SPEED;
        }
        
        if(gamepad1.dpad_down) //Set the speed for turtle.
        {
            GLOBAL_sensitivity = TURTLE_SENSITIVITY;
            GLOBAL_rotationSpeed = TURTLE_ROTATION_SPEED;
        }

        if(gamepad1.a) //Open the claw.
        {
            servoL.setPosition(servoL.getPosition() - SERVO_STEPS);
            servoR.setPosition(servoR.getPosition() + SERVO_STEPS);
        }

        if(gamepad1.b) //Close the claw.
        {
            servoL.setPosition(servoL.getPosition() + SERVO_STEPS);
            servoR.setPosition(servoR.getPosition() - SERVO_STEPS);
        }
        telemetry.addData("Status", "Running: " + runtime.toString());
    }

    /* Code to run ONCE after the driver hits STOP */
    @Override
    public void stop() {
    }
    
    /* Simplifies code, sets the power for all four motors. */
    public void SetMotorBehavor(double motora, double motorb, double motorc, double motord)
    {
        motor1.setPower(motora);
        motor2.setPower(motord);
        motor3.setPower(motorc);
        motor4.setPower(motord);
    }
}
