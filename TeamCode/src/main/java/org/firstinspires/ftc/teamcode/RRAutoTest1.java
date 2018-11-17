

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
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.Locale;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;


    /**
     * This program will go straight  hit the capball  and park the bot at the center vertex
     */

    @Autonomous(name="Blue Auto", group="Team10515")

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
        private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
        private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
        private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

        // State used for updating telemetry
        Orientation angles;
        Acceleration gravity;

        private static final String VUFORIA_KEY = "AVVrDYb/////AAABmT0TlZXDYE3gpf/zMjQrOgACsYT0LcTPCkhjAmq0XO3HT0RdGx2eP+Lwumhftz4e/g28CBGg1HmaFfy5kW9ioO4UGDeokDyxRfqWjNQwKG3BanmjCXxMxACaJ7iom5J3o4ylWNmuiyxsK8n1fFf2dVsTUsvUI7aRxqTahnIqqRJRsGmxld18eHy/ZhHfIjOyifi4svZUQiput21/jAloTx0sTnnrpR1Y/xGOz+68sGuXIgLZHpAQSoZnXiczGKdahGXOg3n6dXlQPIiASE1kHp253CTwO40l1HHN083m4wYjP4FCl/9TH3tb0Wj/Ccmlhfz2omhnZQKOBe7RsIxRk+PuEGkIe5hCs/lV9+yf9iBm";
        private VuforiaLocalizer vuforia;
        private TFObjectDetector tfod;

        @Override
        public void runOpMode() {

            /*BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled      = true;
            parameters.loggingTag          = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
            double myTurn = 90;*/

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
            robot.init(hardwareMap);
            //robot.colorSensor.enableLed(false);
            //calibrateGyro();
            //Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
       //     imu = hardwareMap.get(BNO055IMU.class, "imu");
         //   imu.initialize(parameters);
           // String angle = formatAngle(angles.angleUnit, angles.firstAngle);
            //telemetry.addData("heading",angle);
            //telemetry.addData("firstAngle",angles.firstAngle);


            //sleep(2000);

            // Send telemetry message to signify robotrt waiting;
            telemetry.addData("Status", "Ready to run");    //
            telemetry.update();

        //    composeTelemetry();
            // Wait for the game to start (driver presses PLAY)
            waitForStart();



            // Start the logging of measured acceleration
           // robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

            //goStraight(0.3,0.2);
            //stopRobot();
            //sleep(1000 );
            Orientation angles1 = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            String angle = formatAngle(angles1.angleUnit, angles1.firstAngle);
            telemetry.addData("headingP",angle);


             telemetry.update();
            sleep(1000);
            //turnRight(0.4, 0.2);
            //stopRobot();
            //sleep(1000);
            angles1 = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            angle = formatAngle(angles1.angleUnit, angles1.firstAngle);
            telemetry.addData("headingP",angle);

            telemetry.update();
            sleep(1000);

            //turnLeft(0.4,0.2);
            //stopRobot();
            //sleep(1000);
            angles1 = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            angle = formatAngle(angles1.angleUnit, angles1.firstAngle);
            telemetry.addData("headingP",angle);

            telemetry.update();
            sleep(1000);

            String goldPos = getGoldPosition();
            //if (goldPos == "Left") RemoveGoldLeft()
            //else if (goldPos = "Right") RemoveGoldRight()
            //else if (goldPos = "Center") RemoveGoldCenter()
            //else don't know which one is Gold move on with rest of Auto

            //repositionBot(-65.0);
            //repositionBotAntiClock(90.0);

            //sleep(5000);

         //moveStraightEncoder(-746,0.3); //746 will move 12.35 inches on Rev Motor
          //  moveRSideEncoder(2238,.03);
            //moveRSideEncoder(100,.2);
           //boolean  iscaptured = moveVuforia();
            //telemetry.addData("is captured",iscaptured);
            //telemetry.update();
            //repositionBot(90.0);
         //   imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
           // double x = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
//            while((x>myTurn +5) || (x<myTurn-5))
//            {
//                turnLeft(.2,.2);
//                telemetry.addData("calib",x);
//                telemetry.update();
//            }
            //vuforiaCapture();
           stopRobot();
//            moveRSideEncoder(1120,0.5);
//            stopRobot();
//            moveStraightEncoder(1120,0.5);
//            stopRobot();
//            repositionBot(60.0);
//            stopRobot();

//            moveStraightEncoder(6720,0.5);
//            stopRobot();
        }

        public String getGoldPosition() {
            String pos = "Error";

            try {

                initVuforia();

                if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
                    initTfod();
                } else {
                    telemetry.addData("Sorry!", "This device is not compatible with TFOD");
                }

                if (tfod != null) {
                    tfod.activate();
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 3) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                            }
                            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                    pos = "Left";
                                    telemetry.addData("Gold Mineral Position", "Left");
                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    pos = "Right";
                                    telemetry.addData("Gold Mineral Position", "Right");
                                } else {
                                    pos = "Center";
                                    telemetry.addData("Gold Mineral Position", "Center");
                                }
                            }
                        }
                    }
                }

                if (tfod != null) {
                    tfod.shutdown();
                }
            }
            catch (Exception ex){
                pos = "Error";
                telemetry.addData("Don't know Gold Mineral Position", "Error");

                if (tfod != null) {
                    tfod.shutdown();
                }
            }

            telemetry.update();
            return pos;
        }

        /**
         * Initialize the Vuforia localization engine.
         */
        private void initVuforia() {
            /*
             * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
             */
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

            parameters.vuforiaLicenseKey = VUFORIA_KEY;
            parameters.cameraDirection = CameraDirection.FRONT;

            //  Instantiate the Vuforia engine
            vuforia = ClassFactory.getInstance().createVuforia(parameters);

            // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
        }

        /**
         * Initialize the Tensor Flow Object Detection engine.
         */
        private void initTfod() {
            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
        }

        void composeTelemetry() {

            // At the beginning of each telemetry update, grab a bunch of data
            // from the IMU that we will then display in separate lines.
            telemetry.addAction(new Runnable() { @Override public void run()
            {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity  = robot.imu.getGravity();
            }
            });

            telemetry.addLine()
                    .addData("status", new Func<String>() {
                        @Override public String value() {
                            return robot.imu.getSystemStatus().toShortString();
                        }
                    })
                    .addData("calib", new Func<String>() {
                        @Override public String value() {
                            return robot.imu.getCalibrationStatus().toString();
                        }
                    });

            telemetry.addLine()
                    .addData("heading", new Func<String>() {
                        @Override public String value() {
                            return formatAngle(angles.angleUnit, angles.firstAngle);
                        }
                    })
                    .addData("roll", new Func<String>() {
                        @Override public String value() {
                            return formatAngle(angles.angleUnit, angles.secondAngle);
                        }
                    })
                    .addData("pitch", new Func<String>() {
                        @Override public String value() {
                            return formatAngle(angles.angleUnit, angles.thirdAngle);
                        }
                    });

            telemetry.addLine()
                    .addData("grvty", new Func<String>() {
                        @Override public String value() {
                            return gravity.toString();
                        }
                    })
                    .addData("mag", new Func<String>() {
                        @Override public String value() {
                            return String.format(Locale.getDefault(), "%.3f",
                                    Math.sqrt(gravity.xAccel*gravity.xAccel
                                            + gravity.yAccel*gravity.yAccel
                                            + gravity.zAccel*gravity.zAccel));
                        }
                    });
        }


        String formatAngle(AngleUnit angleUnit, double angle) {
            return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
        }
        String formatDegrees(double degrees){
            return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
        }
    }
