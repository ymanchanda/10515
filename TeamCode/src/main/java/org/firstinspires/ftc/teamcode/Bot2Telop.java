package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.annotations.ServoType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/* Created by Aashray on 10/29/18. */
@TeleOp(name = "Bot 2 Teleop ", group = "Team10515")
public class Bot2Telop extends OpMode
{
    /* Declare OpMode members. */
    BOT2HMap robot = new BOT2HMap();

    final double MOTORMAX = 1.00;
    final double MOTORMIN = -1.00;
    final double LIFT_UP = 1.00;
    final double LIFT_DOWN = -1.00;
    final double SLIDE_IN = -1.0;
    final double SLIDE_OUT = 1.0;
    final double POWERMULTIPLIER = 0.25;
    final double SERVOPOWER = 1.0;
    boolean FWD = false;
    boolean directionDrive = false;
    double  clawOffset  = 0.0 ;                  // Servo mid position

    ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    /* Run code on INIT */
    @Override
    public void init() {
        /* Initialize the hardware map*/
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Init", "Hello Rover");    //
        updateTelemetry(telemetry);
    }

    /* Loop after INIT, NOT used. We can add telemetry for testing */
    @Override
    public void init_loop() {
    }

    /* Code to run ONCE when the driver hits PLAY */
    @Override
    public void start() {
        telemetry.addData("Started", "Ready for Exploration");
        telemetry.update();
        runtime.reset();
    }

    /* Run this in a loop, process drive station input, till STOP */
    @Override
    public void loop() {

        //double dist = robot.rangeSensor.getDistance(DistanceUnit.INCH);
        //telemetry.addData("Distance", String.format("%.2f", dist));
        //double odist = robot.rangeSensor.rawUltrasonic();
        //telemetry.addData("Raw Ultra", String.format("%.2f", odist));
        //telemetry.update();
        if (gamepad1.left_bumper)
        {
            directionDrive = true;
            robot.FleftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.FrightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.BLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.BRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        if (gamepad1.right_bumper)
        {
            directionDrive = false;
            robot.FleftMotor.setDirection(DcMotor.Direction.FORWARD);
            robot.FrightMotor.setDirection(DcMotor.Direction.REVERSE);
            robot.BLeftMotor.setDirection(DcMotor.Direction.FORWARD);
            robot.BRightMotor.setDirection(DcMotor.Direction.REVERSE);
        }
        if (gamepad1.left_stick_x !=0 || gamepad1.left_stick_y !=0 || gamepad1.right_stick_x !=0)
        {

                double leftFront = -gamepad1.left_stick_x + gamepad1.left_stick_y - gamepad1.right_stick_x;
                double rightFront = gamepad1.left_stick_x + gamepad1.left_stick_y + gamepad1.right_stick_x;
                double leftRear = -gamepad1.left_stick_x + gamepad1.left_stick_y + gamepad1.right_stick_x;
                double rightRear = gamepad1.left_stick_x + gamepad1.left_stick_y - gamepad1.right_stick_x;

                double frontMax = Math.max(Math.abs(leftFront), Math.abs(rightFront));
                double rearMax = Math.max(Math.abs(leftRear), Math.abs(rightRear));
                double maxDrive = Math.max(frontMax, rearMax);
                maxDrive = (maxDrive > MOTORMAX) ? maxDrive : MOTORMAX;

                leftFront = leftFront / maxDrive;
                leftFront = Range.clip(leftFront, MOTORMIN, MOTORMAX);
                rightFront = rightFront / maxDrive;
                rightFront = Range.clip(rightFront, MOTORMIN, MOTORMAX);

                leftRear = leftRear / maxDrive;
                leftRear = Range.clip(leftRear, MOTORMIN, MOTORMAX);
                rightRear = rightRear / maxDrive;
                rightRear = Range.clip(rightRear, MOTORMIN, MOTORMAX);

                robot.FleftMotor.setPower(leftFront * POWERMULTIPLIER);
                robot.FrightMotor.setPower(rightFront * POWERMULTIPLIER);
                robot.BLeftMotor.setPower(leftRear * POWERMULTIPLIER);
                robot.BRightMotor.setPower(rightRear * POWERMULTIPLIER);


        }
        else
            stopMoving();

    if (gamepad1.y) {
        //if (dist > 3.5)
            robot.liftMotor.setPower(LIFT_UP);
    }
    else if (gamepad1.a) {
            //if (dist < 9.5)
        robot.liftMotor.setPower(LIFT_DOWN);
    }
    else
        robot.liftMotor.setPower(0);

    if (gamepad1.b){
      if (runtime.time() > 20.0) {
            clawOffset = clawOffset + 0.05;
            clawOffset = Range.clip(clawOffset, 0, 0.9);
            robot.depositor.setPosition(1 - clawOffset);
            runtime.reset();
         }
    } else if (gamepad1.x) {
        if (runtime.time() > 20.0) {
            clawOffset = clawOffset - 0.05;
            clawOffset = Range.clip(clawOffset, 0, 1);
            robot.depositor.setPosition(1 - clawOffset);
            runtime.reset();
        }
    }

    if (gamepad2.left_bumper && runtime.time() > 200)
        if (!FWD) {
            robot.spinner.setDirection(CRServo.Direction.REVERSE);
            robot.spinner.setPower(SERVOPOWER);
            FWD = true;
            runtime.reset();
        }
        else {
            robot.spinner.setDirection(CRServo.Direction.FORWARD);
            robot.spinner.setPower(SERVOPOWER);
            FWD = false;
            runtime.reset();
        }

        if (gamepad2.right_bumper && runtime.time() > 200)
            robot.spinner.setPower(0);

        if(gamepad2.b)
            robot.armMotor.setPower(LIFT_DOWN);
        else if (gamepad2.x)
            robot.armMotor.setPower(LIFT_UP);
        else if (gamepad2.a)
            robot.SliderMotor.setPower(SLIDE_IN);
        else if (gamepad2.y)
            robot.SliderMotor.setPower(SLIDE_OUT);
        else {
            robot.SliderMotor.setPower(0);
            robot.armMotor.setPower(0);
        }
    }

    /* Code to run ONCE after the driver hits STOP  */
    //@Override
    public void stop()
    {
        stopRobot();
    }

    private void stopRobot() {
        robot.FleftMotor.setPower(0);
        robot.FrightMotor.setPower(0);
        robot.BLeftMotor.setPower(0);
        robot.BRightMotor.setPower(0);
        robot.SliderMotor.setPower(0);
        robot.liftMotor.setPower(0);
        robot.armMotor.setPower(0);
        //robot.depositor.setPower(0);
        robot.claw.setPosition(0);
        robot.spinner.setPower(0);
    }
    private void stopMoving() {
        robot.FleftMotor.setPower(0);
        robot.FrightMotor.setPower(0);
        robot.BLeftMotor.setPower(0);
        robot.BRightMotor.setPower(0);

    }
}