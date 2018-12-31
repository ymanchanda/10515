package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DBotAutoHWMap
{

    public DcMotor  FR = null;  //FrontRight
    public DcMotor  FL = null;  //FrontLeft
    public DcMotor  RR = null;  //RearRight
    public DcMotor  RL = null;  //RearLeft
    public DcMotor  LL = null;  //LatchLift
    public DcMotor  AE = null;  //ArmExtension
    public DcMotor  DL = null;  //DepositorLift
    public DcMotor  AA = null;  //AccessArm

    public Servo    marker      = null;     //marker
    public Servo    depositor   = null;     //depositor

    public BNO055IMU    imu                 = null;  //gyro
    public Rev2mDistanceSensor rightSensor  = null;  //right distance sensor

    static private final String  FRONTRIGHT = "FR";
    static private final String  FRONTLEFT  = "FL";
    static private final String  REARRIGHT  = "RR";
    static private final String  REARLEFT   = "RL";
    static private final String  LATCHLIFT  = "LL";
    static private final String  ARMEXT     = "AE";
    static private final String  DEPOSITLIFT= "DL";
    static private final String  ACCESSARM  = "AA";
    static private final String  MARKER     = "M";
    static private final String  DEPOSITOR  = "D";
    static private final String  SWEEPER    = "S";

    static private final String  COLORAE    = "CAE";
    static private final String  IMU        = "imu";
    static private final String  RIGHTSENSOR = "R";

     /* local OpMode members. */
    HardwareMap hwMap           =  null;

    /* Constructor */
    public DBotAutoHWMap(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        FL      = hwMap.dcMotor.get(FRONTLEFT);
        FR      = hwMap.dcMotor.get(FRONTRIGHT);
        RL      = hwMap.dcMotor.get(REARLEFT);
        RR      = hwMap.dcMotor.get(REARRIGHT);
        LL      = hwMap.dcMotor.get(LATCHLIFT);
        AE      = hwMap.dcMotor.get(ARMEXT);
        DL      = hwMap.dcMotor.get(DEPOSITLIFT);
        AA      = hwMap.dcMotor.get(ACCESSARM);

        marker  = hwMap.servo.get(MARKER);
        depositor = hwMap.servo.get(DEPOSITOR);
        rightSensor = hwMap.get(Rev2mDistanceSensor.class, RIGHTSENSOR);

        FL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);
        RL.setDirection(DcMotor.Direction.REVERSE);
        RR.setDirection(DcMotor.Direction.FORWARD);
        AE.setDirection(DcMotor.Direction.REVERSE);
        LL.setDirection(DcMotor.Direction.REVERSE);
        AA.setDirection(DcMotor.Direction.REVERSE);

        //set motor power behaviour
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        AE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        AA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FL.setPower(0);
        FR.setPower(0);
        RL.setPower(0);
        RR.setPower(0);
        AE.setPower(0);
        LL.setPower(0);
        AA.setPower(0);

        rightSensor.initialize();

        // May want to use RUN_USING_ENCODERS if encoders are installed.
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        AE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        AA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        calibrateGyro();
    }

    public void calibrateGyro()
    {
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
        imu = hwMap.get(BNO055IMU.class, IMU);
        imu.initialize(parameters);
    }
}
