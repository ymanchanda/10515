package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class DemobotHardwareMap
{


    public DcMotor  BRightMotor = null;
    public DcMotor  BLeftMotor    = null;
    public DcMotor  latchSlideMotor    = null;

    public Servo   arm  = null;
    public Servo claw = null;
    public  Servo hand = null;


    public BNO055IMU imu = null;

    static final String  BRIGHT_MOTOR = "BackRight";
    static final String  BLEFT_MOTOR = "BackLeft";

    static final String  CLAW = "Claw";
    static final String  ARM = "ARM";
    static  final String HAND = "HAND";

    // static final String  COLOR_SENSOR = "Color";
    //   static final String  COLOR_SENSORREV = "RevColor";
    static final String  IMU_SENSOR = "Imu";

    public static final double LIFT_UP_POWER    =  .5 ;
    public static final double LIFT_DOWN_POWER  = -0.6;
    public static final double SLIDE_OUT_POWER    =  .5 ;
    public static final double SLIDE_IN_POWER  = -.3;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    //public RR10515HardwareMap()
    //{

    //}

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors

        BLeftMotor    = hwMap.dcMotor.get (BLEFT_MOTOR);
        BRightMotor   = hwMap.dcMotor.get(BRIGHT_MOTOR);
        //latchSlideMotor = hwMap.dcMotor.get(LATCH_SLIDE_MOTOR);

        claw   = hwMap.servo.get(CLAW);
        arm  = hwMap.servo.get(ARM);
        hand = hwMap.servo.get(HAND);




        //colorSensor = hwMap.colorSensor.get(COLOR_SENSOR);
        //     colorSensorRev = hwMap.get(ColorSensor.class, COLOR_SENSORREV);

        BLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        BRightMotor.setDirection((DcMotor.Direction.REVERSE));
        latchSlideMotor.setDirection(DcMotor.Direction.REVERSE);


        //colorSensor.enableLed(false);
        //   colorSensorRev.enableLed(false);


        claw.setDirection(Servo.Direction.REVERSE);
        arm.setDirection(Servo.Direction.REVERSE);
        hand.setDirection(Servo.Direction.REVERSE);
        //relicArm.setDirection(Servo.Direction.REVERSE);

        // Set all motors to zero power

        BRightMotor.setPower(0);
        BLeftMotor.setPower(0);
        latchSlideMotor.setPower(0);


        claw.setPosition(0);
        hand.setPosition(0);
        arm.setPosition(0);
        //relicArm.setPosition (0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.

        BLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        latchSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, IMU_SENSOR);
        imu.initialize(parameters);

    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}
