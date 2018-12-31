package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/* Created by Aashray on 12/22/18. */
@TeleOp(name = "D-Bot Drive", group = "XtremeV")
public class DBotTelop extends OpMode
{
    /* Declare OpMode members. */
    DBotTeleOpHWMap robot = new DBotTeleOpHWMap();

    final double MOTORMAX = 1.00;
    final double MOTORMIN = -1.00;
    final double LIFT_UP = 0.30;
    final double LIFT_DOWN = -0.30;
    final double SLIDE_IN = -0.50;
    final double SLIDE_OUT = 0.50;
    final double SERVOPOWER = 1.0;
    boolean FWD = false;
    double speedModifier = 0.30;
    double depositorOffset = 0.0;
    double markerOffset = 0.0;

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
        //mecanum wheels using left & right sticks
        if (gamepad1.left_stick_x !=0 || gamepad1.left_stick_y !=0 || gamepad1.right_stick_x !=0)
        {
            double leftFront = -gamepad1.left_stick_x + gamepad1.left_stick_y - gamepad1.right_stick_x;
            double rightFront = gamepad1.left_stick_x + gamepad1.left_stick_y + gamepad1.right_stick_x;
            double leftRear = -gamepad1.left_stick_x + gamepad1.left_stick_y + gamepad1.right_stick_x;
            double rightRear = gamepad1.left_stick_x + gamepad1.left_stick_y - gamepad1.right_stick_x;

            double frontMax = Math.max(Math.abs(leftFront), Math.abs(rightFront));
            double rearMax = Math.max(Math.abs(leftRear), Math.abs(rightRear));
            double maxDrive = Math.max(frontMax, rearMax);
            maxDrive = (maxDrive < MOTORMAX) ? maxDrive : MOTORMAX;

            leftFront = leftFront / maxDrive;
            leftFront = Range.clip(leftFront, MOTORMIN, MOTORMAX);
            rightFront = rightFront / maxDrive;
            rightFront = Range.clip(rightFront, MOTORMIN, MOTORMAX);
            leftRear = leftRear / maxDrive;
            leftRear = Range.clip(leftRear, MOTORMIN, MOTORMAX);
            rightRear = rightRear / maxDrive;
            rightRear = Range.clip(rightRear, MOTORMIN, MOTORMAX);

            robot.FL.setPower(leftFront * speedModifier);
            robot.FR.setPower(rightFront * speedModifier);
            robot.RL.setPower(leftRear * speedModifier);
            robot.RR.setPower(rightRear * speedModifier);
        }
        else
            stopMoving();

        //Depositor Lift controls
        if (gamepad1.y) {
            robot.DL.setPower(LIFT_UP);
        }
        else if (gamepad1.a) {
            robot.DL.setPower(LIFT_DOWN);
        }
        else
            robot.DL.setPower(0);

        //Depositor Servo controls
        if (gamepad1.b)
            if (runtime.time() > 20.0) {
                depositorOffset = depositorOffset + 0.04;
                depositorOffset = Range.clip(depositorOffset, 0, 0.9);
                robot.depositor.setPosition(1 - depositorOffset);
                runtime.reset();
            }
        else if (gamepad1.x)
            if (runtime.time() > 20.0) {
                depositorOffset = depositorOffset - 0.04;
                depositorOffset = Range.clip(depositorOffset, 0, 1);
                robot.depositor.setPosition(1 - depositorOffset);
                runtime.reset();
            }

        if (gamepad1.left_bumper)
            if (runtime.time() > 20.0) {
                markerOffset = markerOffset + 0.02;
                markerOffset = Range.clip(markerOffset, 0, 0.9);
                robot.marker.setPosition(1 - markerOffset);
                runtime.reset();
            }
        else if (gamepad1.right_bumper)
            if (runtime.time() > 20.0) {
                markerOffset = markerOffset - 0.02;
                markerOffset = Range.clip(markerOffset, 0, 1);
                robot.marker.setPosition(1 - markerOffset);
                runtime.reset();
            }

        //Latch Lift controls
        if (gamepad2.dpad_up) {
            robot.LL.setPower(LIFT_UP);
        }
        else if (gamepad2.dpad_down) {
            robot.LL.setPower(LIFT_DOWN);
        }
        else
            robot.LL.setPower(0);

        //Sweeper Controls
        if (gamepad2.left_bumper && runtime.time() > 200)
            if (!FWD) {
                robot.sweeper.setDirection(CRServo.Direction.REVERSE);
                robot.sweeper.setPower(SERVOPOWER);
                FWD = true;
                runtime.reset();
            }
            else {
                robot.sweeper.setDirection(CRServo.Direction.FORWARD);
                robot.sweeper.setPower(SERVOPOWER);
                FWD = false;
                runtime.reset();
            }

        if (gamepad2.right_bumper && runtime.time() > 200)
            robot.sweeper.setPower(0);

        //Access Arm Controls
        if(gamepad2.x)
            robot.AA.setPower(LIFT_DOWN);
        else if (gamepad2.b)
            robot.AA.setPower(LIFT_UP);
        else
            robot.AA.setPower(0);

        //Arm Extension
        if (gamepad2.a)
            robot.AE.setPower(SLIDE_IN);
        else if (gamepad2.y)
            robot.AE.setPower(SLIDE_OUT);
        else
            robot.AE.setPower(0);
    }

    private void stopRobot() {
        robot.FL.setPower(0);
        robot.FR.setPower(0);
        robot.RL.setPower(0);
        robot.RR.setPower(0);
        robot.AE.setPower(0);
        robot.LL.setPower(0);
        robot.AA.setPower(0);
        robot.DL.setPower(0);
    }
    private void stopMoving() {
        robot.FL.setPower(0);
        robot.FR.setPower(0);
        robot.RL.setPower(0);
        robot.RR.setPower(0);
    }

    private String getAEColor()
    {
        if (robot.colorAE.red() > robot.colorAE.blue() + 3) {
            return "Red";
        } else if (robot.colorAE.blue() > robot.colorAE.red() + 3) {
            return "Blue";
        }
        else {
            return "Unknown";
        }
    }
}