package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/* Created by Aashray on 12/22/18. */
@TeleOp(name = "Test Lift", group = "XtremeV")
public class TestTeleop extends OpMode {
    /* Declare OpMode members. */
    TestBotHWMap robot = new TestBotHWMap();

    final double MOTORMAX = 1.00;
    final double MOTORMIN = -1.00;
    final double LIFT_UP = 0.75;
    final double LIFT_DOWN = -0.75;
    final double ARM_UP = 1.00;
    final double ARM_DOWN = -1.00;
    final double SLIDE_IN = -0.50;
    final double SLIDE_OUT = 0.50;
    final double SERVOPOWER = 1.0;
    boolean FWD = false;
    double speedModifier = 0.55;
    double depositorOffset = 0.0;
    double markerOffset = 0;
    boolean ledOn = true;

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

//        depositorOffset = robot.depositor.getPosition();
//        markerOffset = robot.marker.getPosition();
//        robot.colorAE.enableLed(true);
    }

    /* Run this in a loop, process drive station input, till STOP */
    @Override
    public void loop() {
        try {
            //telemetry.addData("depositor", depositorOffset);
            //telemetry.addData("marker", markerOffset);
            //telemetry.addData("runtime", runtime.time());
            //telemetry.addData("Color", getAEColor());
            //telemetry.update();

            //mecanum wheels using left & right sticks
            if (gamepad1.y) {
                robot.DL.setPower(.5);
            } else if (gamepad2.a) {
                robot.DL.setPower(0);
            }

        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}