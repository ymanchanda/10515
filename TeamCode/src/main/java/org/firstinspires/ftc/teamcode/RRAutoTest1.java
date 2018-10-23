

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Locale;
import java.util.concurrent.TimeUnit;

    /**
     * This program will go straight  hit the capball  and park the bot at the center vertex
     */

    @Autonomous(name="Encoder Test", group="Team10515")

    public class RRAutoTest1 extends RR10515Base
    {

        static final double INIT_FORWARD_SPEED = 0.1;
        static final double FORWARD_SPEED = 0.4;
        static final double BACKWARD_SPEED = 0.4;
        static final double TURN_SPEED = 0.1;

        private static final double CIRCUMFERENCE = 12.35;
        private static final double SIDEWAYS = 15.25;


        boolean redColor = false;
        boolean blueColor = false;

        public static final String TAG = "Vuforia VuMark Sample";

        OpenGLMatrix lastLocation = null;

        /**
         * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
         * localization engine.
         */
        //VuforiaLocalizer vuforia;


        @Override
        public void runOpMode() {



        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
            robot.init(hardwareMap);
            //robot.colorSensor.enableLed(false);
            //calibrateGyro();
            Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            //String angle = formatAngle(angles.angleUnit, angles.firstAngle);

            //telemetry.addData("heading",angle);
            //telemetry.addData("firstAngle",angles.firstAngle);


            //sleep(2000);

            // Send telemetry message to signify robotrt waiting;
            telemetry.addData("Status", "Ready to run");    //
            telemetry.update();

            // Wait for the game to start (driver presses PLAY)
            waitForStart();

           // mechmove(180,0.5,5);
            //stopRobot();
            //delay(100);
           moveStraightEncoder(2240,0.5);
           stopRobot();
            moveRSideEncoder(1120,0.5);
            stopRobot();
            moveStraightEncoder(1120,0.5);
            stopRobot();
            repositionBot(60.0);
            stopRobot();
            moveStraightEncoder(6720,0.5);
            stopRobot();
        }
    }
