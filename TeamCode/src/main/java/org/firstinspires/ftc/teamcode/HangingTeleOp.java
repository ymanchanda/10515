package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by umang on 9/14/18.
 */
@TeleOp(name = "XtremeV Hanging Teleop ", group = "Team10515")
public class HangingTeleOp extends OpMode
{

    /* Declare OpMode members. */
    HH10515HardwareMap robot = new HH10515HardwareMap(); // use the class created to define the bots hardware
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
        if(gamepad1.y)
        {
            robot.HangMotor.setPower(robot.HANG_POWER);
            telemetry.addData("Going ","Up");
            updateTelemetry(telemetry);

        }
        else if (gamepad1.a)
        {
            robot.HangMotor.setPower(-robot.HANG_POWER);
            telemetry.addData("Going ", "Down");
            updateTelemetry(telemetry);
        }
        else {
            robot.HangMotor.setPower(0);
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


    public void stopRobot() {
        robot.HangMotor.setPower(0.0);
    }
}



