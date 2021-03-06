package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
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
public class BOT2HMap
{

    public DcMotor  FrightMotor = null;
    public DcMotor  FleftMotor = null;
    public DcMotor  BRightMotor = null;
    public DcMotor  BLeftMotor    = null;
    public DcMotor  SliderMotor    = null;
    public DcMotor  liftMotor = null;
    public DcMotor  armMotor = null;

    public CRServo   spinner   = null;
    public Servo claw = null;
    public Servo    depositor = null;
    // public Servo    relicArm = null;

    //public ColorSensor colorSensor     = null;
    //public ColorSensor colorSensorRev = null;
    public BNO055IMU imu = null;
    public Rev2mDistanceSensor rangeSensor = null;

    static final String FRIGHT_MOTOR   = "FrontRight";
    static final String  FLEFT_MOTOR   = "FrontLeft";
    static final String  BRIGHT_MOTOR  = "BackRight";
    static final String  BLEFT_MOTOR   = "BackLeft";
    static final String  SLIDER_MOTOR = "Slider";
    static final String  CLAW         = "Claw";
    static final String  SPINNER      = "Spinner";
    static final String   DEPOSITOR   = "Depositor";
    static final String  LIFT_MOTOR   = "Lift";
    static final String  ARM_MOTOR    = "Arm";

    // static final String  COLOR_SENSOR = "Color";
    //   static final String  COLOR_SENSORREV = "RevColor";
    static final String  IMU_SENSOR = "imu";
    static final String  RANGE_SENSOR = "Range";

    public static final double LIFT_UP_POWER    =  .5 ;
    public static final double LIFT_DOWN_POWER  = -0.6;
    public static final double SLIDE_OUT_POWER  =  .5;
    public static final double SLIDE_IN_POWER  = -.3;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public BOT2HMap()
    {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        FleftMotor   = hwMap.dcMotor.get(FLEFT_MOTOR);
        FrightMotor  = hwMap.dcMotor.get(FRIGHT_MOTOR);
        BLeftMotor    = hwMap.dcMotor.get (BLEFT_MOTOR);
        BRightMotor   = hwMap.dcMotor.get(BRIGHT_MOTOR);
        SliderMotor = hwMap.dcMotor.get(SLIDER_MOTOR);
        liftMotor = hwMap.dcMotor.get(LIFT_MOTOR);
        armMotor = hwMap.dcMotor.get(ARM_MOTOR);

        claw   = hwMap.servo.get(CLAW);
        spinner   = hwMap.crservo.get(SPINNER);
        depositor = hwMap.servo.get(DEPOSITOR);

        //colorSensor = hwMap.colorSensor.get(COLOR_SENSOR);
        //colorSensorRev = hwMap.get(ColorSensor.class, COLOR_SENSORREV);
        rangeSensor = hwMap.get(Rev2mDistanceSensor.class, RANGE_SENSOR);

        FleftMotor.setDirection(DcMotor.Direction.FORWARD);
        FrightMotor.setDirection(DcMotor.Direction.REVERSE);
        BLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        BRightMotor.setDirection(DcMotor.Direction.REVERSE);
        SliderMotor.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        //rangeSensor.

        claw.setDirection(Servo.Direction.REVERSE);
        spinner.setDirection(CRServo.Direction.REVERSE);
        depositor.setDirection(Servo.Direction.REVERSE);
        //relicArm.setDirection(Servo.Direction.REVERSE);

        // Set all motors to zero power
        FrightMotor.setPower(0);
        FleftMotor.setPower(0);
        BRightMotor.setPower(0);
        BLeftMotor.setPower(0);
        SliderMotor.setPower(0);
        liftMotor.setPower(0);
        armMotor.setPower(0);

        //claw.setPosition(0);
        spinner.setPower(0);
        //depositor.setPosition(0.5);
        //relicArm.setPosition (0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        FleftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SliderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
