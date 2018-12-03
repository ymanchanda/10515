package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Aashray on 10/29/18.
 */
@TeleOp(name = "Rover Ruckus Teleop ", group = "Team10515")
public class RoverRuckusTeleOp extends OpMode
{
    /* Declare OpMode members. */
    RR10515HardwareMap robot = new RR10515HardwareMap(); // use the class created to define a Pushbot's hardware
    final private double MOTORMAX = 1.00;
    final private double MOTORMIN = -1.00;
    final private double LIFT_UP = -0.75;
    final private double LIFT_DOWN = 0.75;
    final private double POWERMULTIPLIER = 0.75;

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
        telemetry.addData("Started", "Ready for Exploration");    //
        updateTelemetry(telemetry);
        runtime.reset();
    }

    /* Run this in a loop, process drive station input, till STOP */
    @Override
    public void loop() {
        if (gamepad1.left_stick_x !=0 || gamepad1.left_stick_y !=0 || gamepad1.right_stick_x !=0)
        {
            double leftFront = -gamepad1.left_stick_x + gamepad1.left_stick_y + gamepad1.right_stick_x;
            double rightFront = gamepad1.left_stick_x + gamepad1.left_stick_y - gamepad1.right_stick_x;
            double leftRear = -gamepad1.left_stick_x + gamepad1.left_stick_y - gamepad1.right_stick_x;
            double rightRear = gamepad1.left_stick_x + gamepad1.left_stick_y + gamepad1.right_stick_x;

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

            robot.FleftMotor.setPower(leftFront*POWERMULTIPLIER);
            robot.FrightMotor.setPower(rightFront*POWERMULTIPLIER);
            robot.BLeftMotor.setPower(leftRear*POWERMULTIPLIER);
            robot.BRightMotor.setPower(rightRear*POWERMULTIPLIER);
        }
        else
            stopMoving();

        if (gamepad1.a)
            robot.latchSlideMotor.setPower(LIFT_DOWN);
        else if (gamepad1.y)
            robot.latchSlideMotor.setPower(LIFT_UP);
        else
            robot.latchSlideMotor.setPower(0);
   }

     private void stopRobot() {
        robot.FleftMotor.setPower(0.0);
        robot.FrightMotor.setPower(0.0);
        robot.BLeftMotor.setPower(0.0);
        robot.BRightMotor.setPower(0.0);
        robot.latchSlideMotor.setPower(0.0);
    }

    private void stopMoving() {
        robot.FleftMotor.setPower(0.0);
        robot.FrightMotor.setPower(0.0);
        robot.BLeftMotor.setPower(0.0);
        robot.BRightMotor.setPower(0.0);
    }

}