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

@Autonomous(name="Auto: Straight Slide sensor", group="Team10515")
public class Team10515AutoStraight_Slide_Sensor extends Team10515Base {

    static final double     INIT_FORWARD_SPEED = 0.1;
    static final double     FORWARD_SPEED = 0.4;
    static final double     BACKWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.6;
    static final double     HWHEEL_SPEED = 0.8;

    static final double ARM_UP_POWER = 0.3;
    static final double ARM_DOWN_POWER  = -0.5;



    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        initialize();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.claw.setPosition(1);
        String platformColor = colorSenseRev();
        sleep(100);
        String glyphPosition = vuforiaCapture();
        telemetry.addData("The position is" ,glyphPosition);
        telemetry.update();
        sleep(200);
        liftUp(ARM_UP_POWER,0.2);
        stopRobot();
        //handDown();
        sleep(100);
        //String jewelColor = colorSense();
        sleep(200);

        if(platformColor.equals("RED"))
        {
           // moveBlueJewel(jewelColor);
            sleep(200);

            goBack(BACKWARD_SPEED,0.5);
            stopRobot();
            sleep(200);

        //   repositionBotAntiClock(1);
            stopRobot();
            sleep(200);
        }
        else
        {
          //  moveRedJewel(jewelColor);
            sleep(200);

            hRight(0.4,2.0);
            stopRobot();
            sleep(200);

            goStraight(FORWARD_SPEED,1.5);
            stopRobot();
            sleep(200);


            repositionBotAntiClock(1.0);
            stopRobot();
            sleep(200);

        }

       // liftUp(ARM_UP_POWER,0.3);
       //glyphPlacement(glyphPosition,platformColor);
       //glyphPlacement(glyphPosition,platformColor);

    }

/*
    private void glyphPlacement(String glyphPosition,String platformColor)
    {
        if (platformColor.equals("BLUE")) {

            if (glyphPosition.equals("LEFT") || glyphPosition.equals("UNKNOWN")) {
                moveByRange(HWHEEL_SPEED,21.0);
            } else if (glyphPosition.equals("RIGHT")) {
                moveByRange(HWHEEL_SPEED,31.0);
            } else if (glyphPosition.equals("CENTER")) {
                moveByRange(HWHEEL_SPEED,26.5);
            }
        }else if (platformColor.equals("RED")){
            telemetry.addData("inside","red");
            telemetry.update();
            if (glyphPosition.equals("RIGHT") || glyphPosition.equals("UNKNOWN")) {
                moveByRange(HWHEEL_SPEED,16.5); //21
            } else if (glyphPosition.equals("LEFT")) {
                moveByRange(HWHEEL_SPEED,27.0); //34
            } else if (glyphPosition.equals("CENTER")) {
                moveByRange(HWHEEL_SPEED,19.0); //28
            }


            repositionBot(-180.0);
            stopRobot();
            sleep(200);

            goBack(BACKWARD_SPEED,0.1);
            stopRobot();

        }

        stopRobot();
        sleep(200);
        goStraight(FORWARD_SPEED,1.5 );
        stopRobot();
        sleep(200);
        robot.claw.setPosition(0.5);
        stopRobot();
        goBack(BACKWARD_SPEED,0.1);
        stopRobot();

    }
*/

    private void moveBlueJewel(String jewelColor) {

        if (jewelColor.equals("RED")) {
          //  turnRight(TURN_SPEED, 0.2);
            goStraight(FORWARD_SPEED,0.3);
            stopRobot();
           // handUp();
            goBack(BACKWARD_SPEED,0.2);
          //  turnLeft(TURN_SPEED, 0.2);
        } else if (jewelColor.equals("BLUE")) {
         //   turnLeft(TURN_SPEED, 0.2);
            goBack(BACKWARD_SPEED,0.3);
            stopRobot();
           // handUp();
            goStraight(FORWARD_SPEED,0.5);
           // turnRight(TURN_SPEED, 0.2);

        } else {
            telemetry.addData("Nothing can be done", jewelColor);
            telemetry.update();
          //  handUp();
        }

        stopRobot();
    }

    private void moveRedJewel(String jewelColor) {

        if (jewelColor.equals("BLUE")) {
            turnRight(FORWARD_SPEED, 0.2);
            stopRobot();
           // handUp();
            turnLeft(FORWARD_SPEED, 0.2);
            stopRobot();
        } else if (jewelColor.equals("RED")) {
            turnLeft(BACKWARD_SPEED, 0.2);
           // goBack(BACKWARD_SPEED,0.20);
            stopRobot();
            //handUp();
            //goStraight(FORWARD_SPEED,0.5);
            turnRight(BACKWARD_SPEED, 0.2);

        } else {
            telemetry.addData("Nothing can be done", jewelColor);
            telemetry.update();
           // handUp();
        }

        stopRobot();
    }
}
