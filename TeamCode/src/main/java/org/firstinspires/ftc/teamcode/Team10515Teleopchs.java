package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="XtremeV Teleopchs", group="Team10515")
public class Team10515Teleopchs extends OpMode{

    /* Declare OpMode members. */
    Team10515HWDemo robot       = new Team10515HWDemo(); // use the class created to define a Pushbot's hardware
                                                         // could also use HardwarePushbotMatrix class.
    double          clawOffset  = 0.0 ;                  // Servo mid position
    double          relicholdOffset  = 0.0 ;                  // Servo mid position
    double          relicarmOffset  = 0.0 ;                  // Servo mid position
    final double    CLAW_SPEED  = 0.025 ;                 // sets rate to move servo
    final double    RELICHOLD_SPEED  = 0.025 ;                 // sets rate to move servo
    final double    RELICARM_SPEED  = 0.025 ;
    boolean FWD = false;// sets rate to move servo


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
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double left;
        double right;
        double drive;
        double turn;
        double max;
        double hWheel;
        //double Arm;

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        //right = -gamepad1.left_stick_y;
        //left = -gamepad1.right_stick_y;


        drive = -gamepad1.left_stick_y;
        turn = gamepad1.right_stick_x;
        //hWheel = gamepad1.right_stick_x;
        //Arm = gamepad1.right_stick_y;

        // Combine drive and turn for blended motion.
        left = (drive - turn);
        right = (drive + turn);

        // Normalize the values so neither exceed +/- 1.0
        max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0) {
            left /= max;
            right /= max;
        }
        if (FWD) {
            drive = -gamepad2.left_stick_y;
            turn = gamepad2.right_stick_x;

        } else {
            robot.rightFrontMotor.setPower(right);
            robot.leftFrontMotor.setPower(left);
            robot.rightRearMotor.setPower(right);
            robot.leftRearMotor.setPower(left);
        }

/*
        if (gamepad1.dpad_left)
            robot.hWheel.setPower(-0.8);
        else if (gamepad1.dpad_right)
            robot.hWheel.setPower(0.8);
        else
            robot.hWheel.setPower(0.0);
*/
        // Use gamepad X & b buttons to open and close the claw
        if (gamepad1.b) {
            clawOffset = clawOffset + 0.010;
            if (clawOffset > 1) clawOffset = 1;
            robot.claw.setPosition(1 - clawOffset);
        } else if (gamepad1.x) {
            clawOffset = clawOffset - 0.010;
            if (clawOffset < 0) clawOffset = 0;
            robot.claw.setPosition(1 - clawOffset);
        }
    }

        // Use gamepad buttons to move the arm up (Y) and down (A)
     /*   if (gamepad1.a) {
            robot.liftMotor.setPower(robot.LIFT_DOWN_POWER);
        } else if (gamepad1.y) {
            robot.liftMotor.setPower(robot.LIFT_UP_POWER);
        } else {
            robot.liftMotor.setPower(0.0);
        }

    }*/
       /* if (gamepad2.a)
        {
            relicarmOffset = relicarmOffset + 0.005;
            if (relicarmOffset > 1) relicarmOffset = 1;
            robot.relicArm.setPosition(1 - relicarmOffset);
        } else if (gamepad2.y) {
            relicarmOffset = relicarmOffset - 0.005;
            if (relicarmOffset < 0) relicarmOffset = 0;
            robot.relicArm.setPosition(1 - relicarmOffset);

        }
        if (gamepad2.b)
        {
            robot.relicSlideMotor.setPower(robot.SLIDE_OUT_POWER);
        }
        else if (gamepad2.x)
        {
            robot.relicSlideMotor.setPower(robot.SLIDE_IN_POWER);
        }
        else
        {
            robot.relicSlideMotor.setPower(0);
        }
        if (gamepad2.left_bumper)
        {
            relicholdOffset = relicholdOffset + 0.010;
            if (relicholdOffset > 1) relicholdOffset = 1;
            robot.relicHold.setPosition(1 - relicholdOffset);
        } else if (gamepad2.right_bumper) {
            relicholdOffset = relicholdOffset - 0.010;
            if (relicholdOffset < 0) relicholdOffset = 0;
            robot.relicHold.setPosition(1 - relicholdOffset);
        }

        // Send telemetry message to signify robot running;
        //telemetry.addData("claw",  "Offset = %.2f", clawOffset);
        //telemetry.addData("left",  "%.2f", left);
        //telemetry.addData("right", "%.2f", right);
        updateTelemetry(telemetry);
    }
*/
    /*
     * Code to run ONCE after the driver hits STOP
     */
    //@Override
    public void stop()
    {

    }



}
