package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DBotTeleOpHWMap
{

    public DcMotor  FR = null;  //FrontRight
    public DcMotor  FL = null;  //FrontLeft
    public DcMotor  RR = null;  //RearRight
    public DcMotor  RL = null;  //RearLeft
    public DcMotor  LL = null;  //LatchLift
    public DcMotor  AE = null;  //ArmExtension
    public DcMotor  DL = null;  //DepositorLift
    public DcMotor  AA = null;  //AccessArm

    public Servo        marker      = null; //marker
    public Servo        depositor   = null; //depositor
    public CRServo      sweeper     = null; //sweeper
    public ColorSensor  colorAE     = null; //color Sensor

    static private final String  FRONTRIGHT     = "FR";
    static private final String  FRONTLEFT      = "FL";
    static private final String  REARRIGHT      = "RR";
    static private final String  REARLEFT       = "RL";
    static private final String  LATCHLIFT      = "LL";
    static private final String  ARMEXT         = "AE";
    static private final String  DEPOSITLIFT    = "DL";
    static private final String  ACCESSARM      = "AA";
    static private final String  MARKER         = "M";
    static private final String  DEPOSITOR      = "D";
    static private final String  SWEEPER        = "S";
    static private final String  COLORAE        = "CAE";

    /* local OpMode members. */
    HardwareMap hwMap           =  null;

    /* Constructor */
    public DBotTeleOpHWMap() {
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
        sweeper = hwMap.crservo.get(SWEEPER);
        depositor = hwMap.servo.get(DEPOSITOR);

        colorAE = hwMap.get(ColorSensor.class, COLORAE);;
        colorAE.enableLed(false);

        FL.setDirection(DcMotor.Direction.FORWARD);
        FR.setDirection(DcMotor.Direction.REVERSE);
        RL.setDirection(DcMotor.Direction.FORWARD);
        RR.setDirection(DcMotor.Direction.REVERSE);
        AE.setDirection(DcMotor.Direction.REVERSE);
        LL.setDirection(DcMotor.Direction.REVERSE);
        AA.setDirection(DcMotor.Direction.REVERSE);

        sweeper.setDirection(CRServo.Direction.REVERSE);

        //set motor power behaviour
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        AE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        AA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // May want to use RUN_USING_ENCODERS if encoders are installed.
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        AE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        AA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
