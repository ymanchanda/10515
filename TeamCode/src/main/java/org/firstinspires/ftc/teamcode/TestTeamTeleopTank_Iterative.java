package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name=" Test XtremeV Teleop", group="Team10515")
@Disabled
public class TestTeamTeleopTank_Iterative extends OpMode{

    /* Declare OpMode members. */
    TestTeamHardwarePushbot robot    = new TestTeamHardwarePushbot(); // use the class created to define a Pushbot's hardware
                                                         // could also use HardwarePushbotMatrix class.
   double          clawOffset  = 0.0 ;                  // Servo mid position
    final double    CLAW_SPEED  = 0.025 ;                 // sets rate to move servo


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
      //  right = -gamepad1.left_stick_y;
        //left = -gamepad1.right_stick_y;

        drive = -gamepad1.left_stick_y;
        turn  =  gamepad1.left_stick_x;
        hWheel = gamepad1.right_stick_x;
        //Arm = gamepad1.right_stick_y;

        // Combine drive and turn for blended motion.
        left  = (drive + turn)/2;
        right = (drive - turn)/2;

        // Normalize the values so neither exceed +/- 1.0
        max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0)
        {
            left /= max;
            right /= max;
        }

        robot.leftMotor.setPower(left);
        robot.rightMotor.setPower(right);
       // robot.rightRearMotor.setPower(right);
        //robot.leftRearMotor.setPower(left);
        robot.hWheelMotor.setPower(hWheel/2);
        //robot.upMotor.setPower(Arm);

        // Use gamepad X & b buttons to open and close the claw
     /*  if (gamepad1.a){
            clawOffset = clawOffset + 0.025;
            if (clawOffset > 1) clawOffset = 1;
            robot.claw.setPosition(1 - clawOffset);
            //robot.rightClaw.setPosition(clawOffset);
        } else if (gamepad1.y) {
            clawOffset = clawOffset - 0.025;
            if (clawOffset < 0) clawOffset = 0;
            robot.claw.setPosition(1 - clawOffset);
           // robot.rightClaw.setPosition(clawOffset);
        }
*/

        // Use gamepad buttons to move the arm up (Y) and down (A)
      /*  if (gamepad1.b) {
            robot.liftMotor.setPower(robot.ARM_UP_POWER);
        }else if (gamepad1.x) {
            robot.liftMotor.setPower(robot.ARM_DOWN_POWER);
        }else {
            robot.liftMotor.setPower(0.0);
        }
*/

        // Use to move the sweeper
      /* if (gamepad2.a) {
            robot.upMotor.setPower(robot.ARM_UP_POWER);
        }else if (gamepad2.y) {
            robot.upMotor.setPower(robot.ARM_DOWN_POWER);
        } else {
           // robot.upMotor.setPower(0.0);
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

    }



}
