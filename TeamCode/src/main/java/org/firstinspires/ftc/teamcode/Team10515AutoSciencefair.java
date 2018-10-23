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

@Autonomous(name="Auto: Science", group="Team10515")
public class Team10515AutoSciencefair extends Team10515Basescience {

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

       // String platformColor = colorSenseRev();
       // sleep(100);
        //String glyphPosition = vuforiaCapture();
        //telemetry.addData("The position is" ,glyphPosition);
        //telemetry.update();
        //sleep(200);
        liftUp(ARM_UP_POWER,.5);
        stopRobot();
      //  handDown();
        sleep(100);
        boolean check = true;
        while (check)
        {
           int value =  moveColorJewel();
            if (value == 1)
            {
                check = false;
            }
        }
        //
         //String jewelColor = colorSense();
        sleep(200);

      /*  if(platformColor.equals("RED"))
        {
            moveBlueJewel(jewelColor);
            sleep(200);

            goBack(BACKWARD_SPEED,0.9);
            stopRobot();
            sleep(200);

            //turnRight(TURN_SPEED,1.5);
            repositionBot(-180.0);
            stopRobot();
            sleep(200);

         /*   goStraight(FORWARD_SPEED,.3);
            stopRobot();
            sleep(1000);*/
    }
  /*      else
        {
            moveRedJewel(jewelColor);
            sleep(200);

            goStraight(FORWARD_SPEED,1.5);
            stopRobot();
            sleep(200);


            repositionBot(5.0);
            stopRobot();
            sleep(200);

            /*goStraight(FORWARD_SPEED,.3);
            stopRobot();
            sleep(1000);*/
        //}


    //glyphPlacement(glyphPosition,platformColor);




    private void glyphPlacement(String glyphPosition,String platformColor)
    {
        if (platformColor.equals("BLUE")) {
            if (glyphPosition.equals("LEFT") || glyphPosition.equals("UNKNOWN")) {
                hRight(HWHEEL_SPEED, .8);
                //hRight(0.2,1,getDistance());
            } else if (glyphPosition.equals("RIGHT")) {
                hRight(HWHEEL_SPEED, 2.5);
            } else if (glyphPosition.equals("CENTER")) {
                hRight(HWHEEL_SPEED, 2);
            }
        }else if (platformColor.equals("RED")){
            if (glyphPosition.equals("RIGHT") || glyphPosition.equals("UNKNOWN")) {
                hLeft(HWHEEL_SPEED, .6);
            } else if (glyphPosition.equals("LEFT")) {
                hLeft(HWHEEL_SPEED, 2.2);
            } else if (glyphPosition.equals("CENTER")) {
                hLeft(HWHEEL_SPEED, 1.5);
            }


        }
        goStraight(FORWARD_SPEED,.3 );
        stopRobot();
        goBack(BACKWARD_SPEED,0.2);
        stopRobot();
        robot.claw.setPosition(0);
    }



    private int moveColorJewel() {

        String color = colorSenseRev();

        if (color.equals("BLUE")) {
            telemetry.addData("I think I am BLUES CLUES",color);
           // turnRight(FORWARD_SPEED, 0.2);
            goStraight(FORWARD_SPEED,0.2);
            stopRobot();
            liftDown(ARM_DOWN_POWER,.5);
            clawClose();
            liftUp(ARM_UP_POWER,.5);
            //handUp();
            goBack(BACKWARD_SPEED,0.2);
            return 0;
            //turnLeft(FORWARD_SPEED, 0.2);
        } else if (color.equals("RED")) {
            telemetry.addData("Yay im ",color);
           // turnLeft(BACKWARD_SPEED, 0.2);
            goBack(BACKWARD_SPEED,0.5);
            stopRobot();
            repositionBot(-180.0);
            //turnRight();
           // handUp();
            goBack(FORWARD_SPEED,0.5);
            //turnRight(BACKWARD_SPEED, 0.2);
            return 0;

        } else {
            telemetry.addData("Oops",color);
            repositionBot(360);
            stopRobot();
            hLeft(HWHEEL_SPEED,.5);
            stopRobot();
            sleep(1000);
            return 1;


            //telemetry.addData("Nothing can be done");
           // telemetry.update();
            //handUp();
        }

        //stopRobot();
    }
}
