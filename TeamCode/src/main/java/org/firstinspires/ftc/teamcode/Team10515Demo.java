package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/* Created by Aashray on 10/29/18. */
@TeleOp(name = "Demo", group = "Team10515")
public class Team10515Demo extends OpMode
{
    /* Declare OpMode members. */
    Team10515HWDemo robot = new Team10515HWDemo();

    final double MOTORMAX = 1.00;
    final double MOTORMIN = -1.00;

    ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    double clawoffset;
    double tiltoffset;

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
            robot.LR.setPower(leftRear * 0.25);
            robot.RR.setPower(rightRear * 0.25);
        }
        else
            stopMoving();


    if (gamepad1.b){
      if (runtime.time() > 20.0) {
            clawoffset = clawoffset + 0.02;
          clawoffset = Range.clip(clawoffset, 0, 0.9);
            robot.claw.setPosition(1 - clawoffset);
            runtime.reset();
         }
    } else if (gamepad1.x) {
        if (runtime.time() > 20.0) {
            clawoffset = clawoffset - 0.02;
            clawoffset = Range.clip(clawoffset, 0, 1);
            robot.claw.setPosition(1 - clawoffset);
            runtime.reset();
        }
    }

    if (gamepad1.a){
        if (runtime.time() > 20.0) {
            tiltoffset = tiltoffset + 0.02;
            tiltoffset = Range.clip(tiltoffset, 0, 0.9);
            robot.tilt.setPosition(1 - tiltoffset);
            runtime.reset();
        }
    } else if (gamepad1.y) {
        if (runtime.time() > 20.0) {
            tiltoffset = tiltoffset - 0.02;
            tiltoffset = Range.clip(tiltoffset, 0, 1);
            robot.tilt.setPosition(1 - tiltoffset);
            runtime.reset();
        }
    }

    /*
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

        if(gamepad2.x)
            robot.AA.setPower(LIFT_DOWN);
        else if (gamepad2.b)
            robot.AA.setPower(LIFT_UP);
        else if (gamepad2.a)
            robot.AE.setPower(SLIDE_IN);
        else if (gamepad2.y)
            robot.AE.setPower(SLIDE_OUT);
        else {
            robot.AE.setPower(0);
            robot.AA.setPower(0);
        }
        */
    }

    private void stopRobot() {
        //robot.LF.setPower(0);
        robot.LR.setPower(0);
        //robot.RF.setPower(0);
        robot.RR.setPower(0);
        //robot.AE.setPower(0);
        //robot.LL.setPower(0);
        //robot.AA.setPower(0);
        //robot.depositor.setPower(0);
        //robot.marker.setPosition(0);
        //robot.sweeper.setPower(0);
    }
    private void stopMoving() {
        //robot.LF.setPower(0);
        //robot.RF.setPower(0);
        robot.LR.setPower(0);
        robot.RR.setPower(0);

    }
}