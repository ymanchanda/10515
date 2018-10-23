package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * This program will be used for autonomous Straight mode
 * It uses 2 color sensors.
 *      Rev Color Sensor is used to detect the platform color
 *      Color sensor is used to detect the jewel color
 * Based on the platform color the bot will move the appropriate jewel
 *      If RED  then  move  the blue jewel
 *      If BLUE then  move  the red  jewel
 *
 * Based on the vuforia reading the bot  will place glyph accordingly
 *
 */

@Autonomous(name="Auto: Test", group="Team10515")
public class Team10515AutoTest extends Team10515Base {

    static final double     INIT_FORWARD_SPEED = 0.1;
    static final double     FORWARD_SPEED = 0.4;
    static final double     BACKWARD_SPEED = 0.4;
    static final double     TURN_SPEED    = 0.6;
    static final double     HWHEEL_SPEED = 0.5;

    static final double ARM_UP_POWER = 0.75;
    static final double ARM_DOWN_POWER  = -0.5;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        initialize();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();



       // turnLeft(0.5,0.2);

/*        repositionBot(-180);
        stopRobot();
        sleep(500);
        repositionBot(-90);
        stopRobot();

        sleep(2000);*/
//       repositionBot(-45);

  //      stopRobot();

       repositionBotAntiClock(90);
        stopRobot();

      /*  double currentDistance = getDistance();
        //telemetry.addData("raw ultrasonic", currentDistance);
        telemetry.addData("inch", "%.2f inch", currentDistance);
        telemetry.update();
        sleep(1000);

        while (opModeIsActive() && getDistance() > currentDistance - 10.0) {
            telemetry.addData("inch","%.2f inch", getDistance());
            //telemetry.update();
            hLeft(0.8,0.5);
            stopRobot();
            sleep(500);
        }

        stopRobot();
        currentDistance = getDistance();
        telemetry.addData("inch", "%.2f inch", currentDistance);
        telemetry.update();
        sleep(1000);*/

    }


}
