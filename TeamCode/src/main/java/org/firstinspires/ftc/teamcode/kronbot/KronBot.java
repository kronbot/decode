package org.firstinspires.ftc.teamcode.kronbot;



import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.kronbot.utils.drivers.MotorDriver;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.ControlHubGyroscope;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Servo;

public class KronBot {
    public MotorDriver motors;
    public ControlHubGyroscope gyroscope;

    public DcMotorEx leftOuttake;
    public DcMotorEx rightOuttake;

    public Servo loaderServo;

    public void initMotors(HardwareMap hardwareMap) {
        DcMotorEx leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        leftOuttake = hardwareMap.get(DcMotorEx.class, "leftOuttake");
        rightOuttake = hardwareMap.get(DcMotorEx.class, "rightOuttake");


        motors = new MotorDriver();
        motors.init(leftRear, leftFront, rightRear, rightFront);
    }

    public void initServos(HardwareMap hardwareMap) {
        loaderServo = new Servo(hardwareMap);
        loaderServo.init("loaderServo", true, false, 0, 0, 0);
        loaderServo.runContinuous(false, false);
    }

    public void initIMU(HardwareMap hardwareMap) {
        BHI260IMU imu = hardwareMap.get(BHI260IMU.class, "imu");
        gyroscope = new ControlHubGyroscope(hardwareMap);
        gyroscope.init(imu);
    }

    public void initAutonomy(HardwareMap hardwareMap) {
        initMotors(hardwareMap);
        initIMU(hardwareMap);
        initServos(hardwareMap);
    }

    public void initTeleop(HardwareMap hardwareMap) {
        initMotors(hardwareMap);
        initIMU(hardwareMap);
        initServos(hardwareMap);
    }

    public void initSimpleDriving(HardwareMap hardwareMap) {
        initIMU(hardwareMap);
        initMotors(hardwareMap);
    }
}