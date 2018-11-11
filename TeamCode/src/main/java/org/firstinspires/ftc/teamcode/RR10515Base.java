package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.List;


public abstract class RR10515Base extends LinearOpMode {

    /* Declare OpMode members. */
    RR10515HardwareMap robot = new RR10515HardwareMap();   // Use our Team 10515 hardware
    ElapsedTime runtime = new ElapsedTime();

    //public static final String TAG = "Vuforia VuMark Sample";
    //public static final double ARM_DOWN_POWER  = 0.3;
    //public static final double ARM_UP_POWER  = -0.3;


    OpenGLMatrix lastLocation = null;

    VuforiaLocalizer vuforia;

    public void goStraight(double speed, double period)
    {

        robot.BLeftMotor.setPower(-speed);
        robot.BRightMotor.setPower(-speed);
        robot.FrightMotor.setPower(-speed);
        robot.FleftMotor.setPower(-speed);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < period)) {
          //  telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            //  telemetry.update();
        }
    }
    public void goStraight(double speed)
    {

        robot.BLeftMotor.setPower(speed);
        robot.BRightMotor.setPower(speed);
        robot.FleftMotor.setPower(speed);
        robot.FrightMotor.setPower(speed);

        runtime.reset();

    }
   /* public void driveStraightDistance(int tenthsOfIn, int masterPower)
    {
        int tickGoal = (42 * tenthsOfIn) / 10;

        //This will count up the total encoder ticks despite the fact that the encoders are constantly reset.
        int totalTicks = 0;

        //Initialise slavePower as masterPower - 5 so we don't get huge error for the first few iterations. The
        //-5 value is based off a rough guess of how much the motors are different, which prevents the robot from
        //veering off course at the start of the function.
        int slavePower = masterPower - 5;

        int error = 0;

        int kp = 5;

        SensorValue[leftEncoder] = 0;
        SensorValue[rightEncoder] = 0;

        //Monitor 'totalTicks', instead of the values of the encoders which are constantly reset.
        while(abs(totalTicks) < tickGoal)
        {
            //Proportional algorithm to keep the robot going straight.
            motor[leftServo] = masterPower;
            motor[rightServo] = slavePower;

            error = SensorValue[leftEncoder] - SensorValue[rightEncoder];

            slavePower += error / kp;

            SensorValue[leftEncoder] = 0;
            SensorValue[rightEncoder] = 0;

            wait1Msec(100);

            //Add this iteration's encoder values to totalTicks.
            totalTicks+= SensorValue[leftEncoder];
        }
        motor[leftServo] = 0; // Stop the loop once the encoders have counted up the correct number of encoder ticks.
        motor[rightServo] = 0;
    }
    */

   public void moveStraightEncoder(int distance,double power)
   {
       robot.FrightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       robot.FleftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       robot.BRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       robot.BLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

       robot.FrightMotor.setTargetPosition(distance);
       robot.FleftMotor.setTargetPosition(distance);
       robot.BRightMotor.setTargetPosition(distance);
       robot.BLeftMotor.setTargetPosition(distance);


       robot.FrightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       robot.FleftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       robot.BRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       robot.BLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

       goStraight(power);

       while(robot.FrightMotor.isBusy()&&robot.FleftMotor.isBusy()&&robot.BRightMotor.isBusy()&& robot.BLeftMotor.isBusy())
       {

       }
       stopRobot();
       robot.FleftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       robot.FrightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       robot.BLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       robot.BRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

   /* int in1 = robot.FrightMotor.getCurrentPosition();
       int in2 = robot.FleftMotor.getCurrentPosition();
       int in4 = robot.BRightMotor.getCurrentPosition();
       int in3 = robot.BLeftMotor.getCurrentPosition();
    robot.FrightMotor.setTargetPosition(in1+rotations);
    robot.FleftMotor.setTargetPosition(in2+rotations);
    robot.BLeftMotor.setTargetPosition(in3+rotations);
    robot.BRightMotor.setTargetPosition(in4+rotations);
*/
   }

    public void goBack(double speed, double period) {

        robot.BLeftMotor.setPower(-speed);
        robot.BRightMotor.setPower(-speed);
        robot.FleftMotor.setPower(-speed);
        robot.FrightMotor.setPower(-speed);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < period)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            //   telemetry.update();
        }
    }

    public void turnRight(double speed, double period) {

        //  Spin right x seconds
        robot.BLeftMotor.setPower(-speed);
        robot.BRightMotor.setPower(speed);
        robot.FleftMotor.setPower(-speed);
        robot.FrightMotor.setPower(speed);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < period)) {
            //  telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            //   telemetry.update();
        }
    }

    public void turnLeft(double speed, double period) {

        //  Spin Left for x seconds
        robot.BLeftMotor.setPower(speed);
        robot.BRightMotor.setPower(-speed);
        robot.FleftMotor.setPower(speed);
        robot.FrightMotor.setPower(-speed);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < period)) {
            //  telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            //   telemetry.update();
        }

    }
    public void moveRSideEncoder(int distance,double power)
    {
        robot.FrightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FleftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.FrightMotor.setTargetPosition(distance);
        robot.FleftMotor.setTargetPosition(-distance);
        robot.BRightMotor.setTargetPosition(-distance);
        robot.BLeftMotor.setTargetPosition(distance);


        robot.FrightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.FleftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.BRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.BLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        goRightSide(power);

        while(robot.FrightMotor.isBusy()&&robot.FleftMotor.isBusy()&&robot.BRightMotor.isBusy()&& robot.BLeftMotor.isBusy())
        {

        }
        stopRobot();
        robot.FleftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FrightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

   /* int in1 = robot.FrightMotor.getCurrentPosition();
       int in2 = robot.FleftMotor.getCurrentPosition();
       int in4 = robot.BRightMotor.getCurrentPosition();
       int in3 = robot.BLeftMotor.getCurrentPosition();
    robot.FrightMotor.setTargetPosition(in1+rotations);
    robot.FleftMotor.setTargetPosition(in2+rotations);
    robot.BLeftMotor.setTargetPosition(in3+rotations);
    robot.BRightMotor.setTargetPosition(in4+rotations);
*/
    }
        public void goRightSide(double speed)
        {
            robot.BLeftMotor.setPower(speed);
            robot.BRightMotor.setPower(-speed);
            robot.FleftMotor.setPower(-speed);
            robot.FrightMotor.setPower(speed);


        }
    public void stopRobot() {
        robot.BLeftMotor.setPower(0.0);
        robot.BRightMotor.setPower(0.0);
        robot.FrightMotor.setPower(0.0);
        robot.FleftMotor.setPower(0.0);
        // robot.liftMotor.setPower(0.0);
        //robot.hWheel.setPower(0.0);
    }


   /*public void moveByRange(double speed,double distanceToWall) {

        while (opModeIsActive() && getDistance() > distanceToWall ) {
            //hLeft(0.8, 0.3);
            //stopRobot();
           // sleep(500);
            robot.hWheel.setPower(-speed);
        }

        while (opModeIsActive() && getDistance() < distanceToWall - 0.5) {
            //hRight(0.8, 0.3);
            //stopRobot();
            // sleep(500);
            robot.hWheel.setPower(speed);
        }

        stopRobot();
        sleep(200);
    }
*/

    public void latchUp(double speed, double time) {
        robot.latchSlideMotor.setPower(speed);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <= time)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            //  telemetry.update();
        }
    }


    public void latchDown(double speed, double time) {
        robot.latchSlideMotor.setPower(-speed);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <= time)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            //   telemetry.update();
        }
    }

    public String vuforiaCapture() {
        int cameraMonitorViewId = robot.hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", robot.hwMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AeYAHIn/////AAAAGfVr1aFjUEHlh1uCvvWMJFtG8Y1D0YvNXpfCJTXkpgrNedm+jaqR+2trR9dGNzyeuHUMqo42P7DuJIp1IPDBDF5oepx6kw121V3vAc3sR5F43oix5brWapqdLcvFYcdFmWqg3AvIy436p1bkMhhJgcVEzXzIususTncxlVaHDDohnS9zN38qFcbFeKWH8cLG8lbt+2sNqoGJgOQ1/Oq6wEf3ceIS1x2BsguyUtkPLG0OQALkjbktRMdfLHe34ldDuCddP1ekNgkvwauoxOJqYKJKZX15h3VZfRtnp4mArn6Bxx8vWITXm690wfsdAio1LrRGm+NBovMapDxs9IKJuiH53nEoYrvat8IGG9IhMp67";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets that for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");


        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsRoverRuckus);

        targetsRoverRuckus.activate();

        boolean targetVisible = false;
        int i = 0;

        while (!targetVisible && i < 5) {
            sleep(1000);
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    telemetry.update();
                    targetVisible = true;
                    return trackable.getName();
                }
            }
            i++;
        }

        if (!targetVisible)
            telemetry.addData("visible target", "not visible");

        telemetry.update();
    return "test";
    }
    public boolean moveVuforia()
    {
        boolean isCaptured= false;
        while(!isCaptured)
        {
            moveRSideEncoder(200, .2);
            String x = vuforiaCapture();
            if (x.equals("Red-Footprint"))
            {
                isCaptured = true;
            }
            sleep(500);
        }
        stopRobot();
        return isCaptured;
    }

    public void initialize() {
        robot.init(hardwareMap);
        // calibrateGyro();
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        telemetry.addData("heading", angles.firstAngle);
        telemetry.addData("firstAngle", angles.firstAngle);

        //robot.claw.setPosition(.5);
        // robot.liftMotor.setPower(ARM_UP_POWER);

        sleep(2000);

        // Send telemetry message to signify robotrt waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

    }

    public String colorSenseRev() {

        String color = "NOT READ";
        ElapsedTime time = new ElapsedTime();
        time.reset();

        while (time.time() < 5) {
            if (robot.colorSensorRev.red() > robot.colorSensorRev.blue() + 10) {
                telemetry.addData("color rev", "RED");
                telemetry.update();
                color = "RED";
                break;
            } else if (robot.colorSensorRev.blue() > robot.colorSensorRev.red() + 3) {
                telemetry.addData("color rev", "BLUE");
                telemetry.update();
                color = "BLUE";
                break;
            } else {
                telemetry.addData("color rev", "UNKNOWN");
                telemetry.update();
                color = "UNKNOWN";
            }
        }
        return color;
    }

     /* public String colorSense() {
          robot.colorSensor.enableLed(true);

          String color = "NOT READ";
          ElapsedTime time = new ElapsedTime();
          time.reset();

          while (time.time() < 5) {
              if (robot.colorSensor.red() > robot.colorSensor.blue() + 3) {
                  telemetry.addData("color", "RED");
                  telemetry.update();
                  color = "RED";
                  break;
              } else if (robot.colorSensor.blue() > robot.colorSensor.red() + 3) {
                  telemetry.addData("color", "BLUE");
                  telemetry.update();
                  color = "BLUE";
                  break;
              } else {
                  telemetry.addData("color", "UNKNOWN");
                  telemetry.update();
                  color = "UNKNOWN";
              }
          }

          robot.colorSensor.enableLed(false);
          return color;
      }
  */
    public void repositionBot(double angleDegrees) {

        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        telemetry.addData("heading", angles.firstAngle);
        telemetry.addData("firstAngle", angles.firstAngle);
        telemetry.update();
         sleep(2000);

        while (angles.firstAngle > angleDegrees || angles.firstAngle < -angleDegrees) {
            if (angles.firstAngle > angleDegrees) {
                turnRight(0.2, 0.1);

            } else if (angles.firstAngle < -angleDegrees) {
                turnLeft(0.2, 0.1);
            }

            stopRobot();
            sleep(200);
            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            telemetry.addData("firstAngle", angles.firstAngle);
            telemetry.update();
            sleep(1000);
            if (angles.firstAngle > angleDegrees - 5 && angles.firstAngle < angleDegrees + 5) {
                telemetry.addData("firstAngle", angles.firstAngle);
                telemetry.update();
                sleep(1000);
                if(angles.firstAngle > angleDegrees -8 && angles.firstAngle < angleDegrees +8){
                    telemetry.addData("firstAngle", angles.firstAngle);
                    telemetry.update();
                    //   sleep(1000);
                    break;
                }

                telemetry.addData("heading", angles.firstAngle);
                telemetry.addData("firstAngle", angles.firstAngle);
                telemetry.update();
                sleep(1000);
            }
        }}


    public void repositionBotAntiClock(double angleDegrees) {

        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        telemetry.addData("heading", angles.firstAngle);
        telemetry.addData("firstAngle", angles.firstAngle);
        telemetry.update();
        //sleep(2000);

        while (angles.firstAngle < angleDegrees || angles.firstAngle > -angleDegrees) {
            if (angles.firstAngle < angleDegrees) {
                turnLeft(0.2, 0.1);

            } else if (angles.firstAngle > -angleDegrees) {
                turnRight(0.2, 0.1);
            }

            stopRobot();
            sleep(200);
            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            telemetry.addData("firstAngle", angles.firstAngle);
            telemetry.update();
            sleep(1000);
            if (angles.firstAngle > angleDegrees - 5 && angles.firstAngle < angleDegrees + 5) {
                if(angles.firstAngle > angleDegrees -5 && angles.firstAngle < angleDegrees +5){
                    telemetry.addData("firstAngle", angles.firstAngle);
                    telemetry.update();
                    //  sleep(1000);
                    break;
                }

                telemetry.addData("heading", angles.firstAngle);
                telemetry.addData("firstAngle", angles.firstAngle);
                telemetry.update();
                  sleep(1000);
            }
        }

    }}


   /* public double getDistance(){

        double distance = robot.rangeSensor.getDistance(DistanceUnit.INCH);
        telemetry.addData("raw ultrasonic", robot.rangeSensor.rawUltrasonic());
        telemetry.addData("raw optical", robot.rangeSensor.rawOptical());
        // telemetry.addData("cm optical", "%.2f cm", rangeSensor.cmOptical());
        telemetry.addData("inch", "%.2f inch", distance);
        telemetry.update();
      //  sleep(500);
        return distance;
    }
}
*/