

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
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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


/* This program will go straight  hit the capball  and park the bot at the center vertex  */
@Disabled
@Autonomous(name="Depot Facing 1", group="Ttyteam10515")
public class D16Auto1 extends RR10515Base
{
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final int ENCODERVALUE = -495;

    // State used for updating telemetry
    private Orientation angles;
    private Acceleration gravity;

    //Vuforia key
    private static final String VUFORIA_KEY = "AVVrDYb/////AAABmT0TlZXDYE3gpf/zMjQrOgACsYT0LcTPCkhjAmq0XO3HT0RdGx2eP+Lwumhftz4e/g28CBGg1HmaFfy5kW9ioO4UGDeokDyxRfqWjNQwKG3BanmjCXxMxACaJ7iom5J3o4ylWNmuiyxsK8n1fFf2dVsTUsvUI7aRxqTahnIqqRJRsGmxld18eHy/ZhHfIjOyifi4svZUQiput21/jAloTx0sTnnrpR1Y/xGOz+68sGuXIgLZHpAQSoZnXiczGKdahGXOg3n6dXlQPIiASE1kHp253CTwO40l1HHN083m4wYjP4FCl/9TH3tb0Wj/Ccmlhfz2omhnZQKOBe7RsIxRk+PuEGkIe5hCs/lV9+yf9iBm";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        // Send telemetry message to signify robotrt waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        unLatch(1.0,5.5);
        stopRobot();
        sleep(500);
        //moveStraightEncoder(-60,0.2);
        //sleep(2000);

        moveSideEncoder(-255,.2);
        String position = getGoldPosition();
        sleep(200);
        moveStraightEncoder(-495,.2);
        if (position.equals("CENTER")) {
            removeCenter();
        }
        else if(position.equals("LEFT"))
        {
            removeLeft();
        }
        else if(position.equals("RIGHT"))
        {
            removeRight();
        }
        else {
          removeCenter();
        }
        // Start the logging of measured acceleration
        stopRobot();
    }

    public void removeCenter()
    {
        moveSideEncoder(275,.2);
        moveStraightEncoder(-1800,0.2);
        repositionBotAntiClock(45);
        moveStraightEncoder(-200,0.2);
        markerDrop();
        moveStraightEncoder(600,0.2);
        moveSideEncoder(300,0.2);
        goToCrater();
        //sweep mineral in
    }
    public void removeLeft()
    {
        moveSideEncoder(-495,0.2);
        //repositionBot(20.0);
        //sleep(2000);
        moveStraightEncoder(-1450,0.2);
        repositionBotAntiClock(45.0);
        moveSideEncoder(800,0.2);
        moveStraightEncoder(-100,0.2);
        markerDrop();
        moveStraightEncoder(800,0.2);
        moveSideEncoder(300,0.2);
        goToCrater();
        //sweep mineral in and then get ready to go in to depot

    }
    public void removeRight()
    {
        moveSideEncoder(949,0.2);
        moveStraightEncoder(-1049,0.2);
        sleep(500);
        repositionBotAntiClock(45.0);
        moveStraightEncoder(-1100,.2);
        markerDrop();
        moveStraightEncoder(600,0.2);
        moveSideEncoder(200,.2);
        goToCrater();
        //sweep mineral in and then get ready to go in to depot

    }
  private void goToCrater()
    {
        moveStraightEncoder(1700,.2);
        moveSideEncoder(230,.2);
        moveStraightEncoder(800,.2);
        repositionBotAntiClock(65);
        parkCrater(0,.5);
    }


    public String getGoldPosition() {
        String pos = "Error";

        try {

            initVuforia();

            telemetry.addData("Info!", "Init Vuforia");
            telemetry.update();

            if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
                initTfod();
                telemetry.addData("Info!", "Init TFOD");
                telemetry.update();

            } else {
                telemetry.addData("Sorry!", "This device is not compatible with TFOD");
                telemetry.update();
            }

            sleep(500);
            if (tfod != null) {
                tfod.activate();
                telemetry.addData("Info!", "TFOD Active");
                telemetry.update();

                sleep(1000);

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
                                pos = "LEFT";
                                telemetry.addData("Gold Mineral Position", "Left");
                            } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                pos = "RIGHT";
                                telemetry.addData("Gold Mineral Position", "Right");
                            } else {
                                pos = "CENTER";
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
        /* Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine. */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.FRONT;
        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /* Initialize the Tensor Flow Object Detection engine. */
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
