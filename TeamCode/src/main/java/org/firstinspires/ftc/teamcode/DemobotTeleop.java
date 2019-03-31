package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by umang on 4/1/19.
 */

@TeleOp(name = "DemoOP ", group = "Team10515")
public class DemobotTeleop extends OpMode
{

    /* Declare OpMode members. */
    DemobotHardwareMap robot = new DemobotHardwareMap(); // use the class created to define the bots hardware
    final double MOTORMAX = 1.00;
    final double MOTORMIN = -1.00;

    ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        updateTelemetry(telemetry);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        telemetry.addData("Say", "Ready to Play");    //
        updateTelemetry(telemetry);
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop()
    {
        if (gamepad1.left_stick_x !=0 || gamepad1.left_stick_y !=0 || gamepad1.right_stick_x !=0)
        {

            double leftRear = -gamepad1.left_stick_x + gamepad1.left_stick_y + gamepad1.right_stick_x;
            double rightRear = gamepad1.left_stick_x + gamepad1.left_stick_y - gamepad1.right_stick_x;

            //double frontMax = Math.max(Math.abs(leftFront), Math.abs(rightFront));
            double rearMax = Math.max(Math.abs(leftRear), Math.abs(rightRear));
            //double maxDrive = Math.max(frontMax, rearMax);
            //maxDrive = (maxDrive > MOTORMAX) ? maxDrive : MOTORMAX;

            //robot.LF.setPower(leftFront * 0.25);
            //robot.RF.setPower(rightFront * 0.25);
            robot.BLeftMotor.setPower(leftRear * 0.25);
            robot.BRightMotor.setPower(rightRear * 0.25);
        }
        if (gamepad1.a)
        {
            robot.arm.setPosition(0.5);
        }
        if(gamepad1.y)
        {
            robot.arm.setPosition(0);
        }
        if(gamepad1.x)
        {
            robot.claw.setPosition(0.5);

        }
        if(gamepad1.b)
        {
            robot.claw.setPosition(0);
        }
        if(gamepad1.dpad_left)
        {
            robot.hand.setPosition(0.2);
        }
        if(gamepad1.dpad_right)
        {
            robot.hand.setPosition(0.8);
        }

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    //@Override
    public void stop()
    {
        stopRobot();
    }


    public void stopRobot()
    {
        robot.BLeftMotor.setPower(0);
        robot.BRightMotor.setPower(0);
    }
}



