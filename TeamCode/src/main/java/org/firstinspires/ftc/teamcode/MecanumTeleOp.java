package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by umang on 9/14/18.
 */
@TeleOp(name = "XtremeV Mecanum Teleop ", group = "Team10515")
public class MecanumTeleOp extends OpMode
{



    /* Declare OpMode members. */
    RR10515HardwareMap robot = new RR10515HardwareMap(); // use the class created to define a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.
    double clawOffset = 0.0;                  // Servo mid position
    double relicholdOffset = 0.0;                  // Servo mid position
    double relicarmOffset = 0.0;                  // Servo mid position
    final double CLAW_SPEED = 0.025;                 // sets rate to move servo
    final double RELICHOLD_SPEED = 0.025;                 // sets rate to move servo
    final double RELICARM_SPEED = 0.025;
    final double MOTORMAX = 1.00;
    final double MOTORMIN = -1.00;
    boolean FWD = false;// sets rate to move servo
    int counter = 0;

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
       double maxDrive;
        double frontMax;
        double rearMax;

        double leftFront;
        double rightFront;
        double leftRear;
        double rightRear;

        leftFront = -gamepad1.left_stick_x + gamepad1.left_stick_y + gamepad1.right_stick_x;
        rightFront = gamepad1.left_stick_x + gamepad1.left_stick_y - gamepad1.right_stick_x;
        leftRear = -gamepad1.left_stick_x + gamepad1.left_stick_y - gamepad1.right_stick_x;
        rightRear = gamepad1.left_stick_x + gamepad1.left_stick_y + gamepad1.right_stick_x;
/*
        if(gamepad1.left_stick_x < 0)
        {
            leftFront = gamepad1.left_stick_x;
            leftRear = gamepad1.left_stick_x;
            rightFront = - gamepad1.left_stick_x;
            rightRear = - gamepad1.left_stick_x;

        }
        else if (gamepad1.left_stick_x > 0)
        {
            leftFront = -gamepad1.left_stick_x;
            leftRear = -gamepad1.left_stick_x;
            rightFront = gamepad1.left_stick_x;
            rightRear = gamepad1.left_stick_x;

        }
*/
        frontMax = Math.max(Math.abs(leftFront), Math.abs(rightFront));
        rearMax = Math.max(Math.abs(leftRear), Math.abs(rightRear));
        maxDrive = Math.max(frontMax, rearMax);
        maxDrive = (maxDrive > MOTORMAX) ? maxDrive : MOTORMAX;

        leftFront = leftFront/maxDrive;
        leftFront = Range.clip(leftFront, MOTORMIN, MOTORMAX);
        rightFront = rightFront/maxDrive;
        rightFront = Range.clip(rightFront, MOTORMIN, MOTORMAX);

        leftRear = leftRear/maxDrive;
        leftRear = Range.clip(leftRear, MOTORMIN, MOTORMAX);
        rightRear = rightRear/maxDrive;
        rightRear = Range.clip(rightRear, MOTORMIN, MOTORMAX);

        telemetry.addData("The position is" ,leftFront);
        telemetry.addData("The position is" ,rightFront);
        telemetry.addData("The position is" ,rightRear);
        telemetry.addData("The position is" ,leftRear);
        updateTelemetry(telemetry);
        //telemetry.update();

       robot.FleftMotor.setPower(leftFront);
        robot.FrightMotor.setPower(rightFront);
        robot.BLeftMotor.setPower(leftRear);
        robot.BRightMotor.setPower(rightRear);

 /*    double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
       double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
       double rightX = gamepad1.right_stick_x;
      final double v1 = r * Math.cos(robotAngle) + rightX;
      final double v2 = r * Math.sin(robotAngle) - rightX;
      final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        robot.FleftMotor.setPower(v1);
         robot.FrightMotor.setPower(v2);
        robot.BLeftMotor.setPower(v3);
      robot.BRightMotor.setPower(v4);
      */
          /*  double left;
            double right;
            double drive;
            double turn;
            double max;
            double lift;
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
                drive = -gamepad2.left_stick_y;     // when driver presses button the speed changes
                turn = gamepad2.right_stick_x;
                left = (drive + turn);
                right = (drive - turn);

                robot.rightMotor.setPower(left / 1.5);
                robot.leftMotor.setPower(right / 1.5);


            } else {
                robot.rightMotor.setPower(left);
                robot.leftMotor.setPower(right);
            }
            if (gamepad1.right_bumper) {
                FWD = true;
                robot.leftMotor.setDirection(DcMotor.Direction.FORWARD);
                robot.rightMotor.setDirection(DcMotor.Direction.REVERSE);
                // robot.leftMotor.se
            } else if (gamepad1.left_bumper) {
                FWD = false;
                robot.leftMotor.setDirection(DcMotor.Direction.REVERSE);
                robot.rightMotor.setDirection(DcMotor.Direction.FORWARD);
            }


        }
*/
        // Send telemetry message to signify robot running;
        //telemetry.addData("claw",  "Offset = %.2f", clawOffset);
        //telemetry.addData("left",  "%.2f", left);
        //telemetry.addData("right", "%.2f", right);
        updateTelemetry(telemetry);
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
        robot.FleftMotor.setPower(0.0);
        robot.FrightMotor.setPower(0.0);
        robot.BLeftMotor.setPower(0.0);
        robot.BRightMotor.setPower(0.0);

    }
}



