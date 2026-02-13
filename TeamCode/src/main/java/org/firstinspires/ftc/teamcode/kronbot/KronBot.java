package org.firstinspires.ftc.teamcode.kronbot;


import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.FLAP_CLOSED;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.FLAP_OPEN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.OUT_MOTOR_KD;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.OUT_MOTOR_KF;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.OUT_MOTOR_KI;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.OUT_MOTOR_KP;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.kronbot.utils.drivers.MotorDriver;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.ControlHubGyroscope;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;


public class KronBot {
    public MotorDriver motors;
    public ControlHubGyroscope gyroscope;
    public ModernRoboticsI2cRangeSensor rangeSensor;

    public DcMotorEx intakeMotor, leftOuttake, rightOuttake;
    public Servo loaderServo;

    public Servo turretServo, angleServo, flapsServo;
    public ColorSensor outtakeColor;



    public void initDrivetrain(HardwareMap hardwareMap) {
        DcMotorEx leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        motors = new MotorDriver();
        motors.init(leftRear, leftFront, rightRear, rightFront);
    }
    public void initMotors(HardwareMap hardwareMap) {

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        leftOuttake = hardwareMap.get(DcMotorEx.class, "shooter0");
        leftOuttake.setDirection(DcMotorSimple.Direction.REVERSE);
        leftOuttake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftOuttake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftOuttake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftOuttake.setVelocityPIDFCoefficients(
                OUT_MOTOR_KP,   // P - main stabilizer
                OUT_MOTOR_KI,   // I - usually 0
                OUT_MOTOR_KD,   // D - reduces overshoot
                OUT_MOTOR_KF    // F - feedforward (VERY important)
        );
        rightOuttake = hardwareMap.get(DcMotorEx.class, "shooter1");
        rightOuttake.setDirection(DcMotorSimple.Direction.REVERSE);
        rightOuttake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOuttake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightOuttake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightOuttake.setVelocityPIDFCoefficients(
                OUT_MOTOR_KP,   // P - main stabilizer
                OUT_MOTOR_KI,   // I - usually 0
                OUT_MOTOR_KD,   // D - reduces overshoot
                OUT_MOTOR_KF    // F - feedforward (VERY important)
        );

    }


    public void initServos(HardwareMap hardwareMap) {
        loaderServo = new Servo(hardwareMap);
        loaderServo.init("loader", true, false, 0, 0, 0);
        loaderServo.runContinuous(false, false);
        turretServo = new Servo(hardwareMap);
        turretServo.init("turretPivot", false, false, 0, 1, 0.5);
        flapsServo = new Servo(hardwareMap);
        flapsServo.init("flapsServo", false, false, 0, 1, FLAP_CLOSED);
        angleServo = new Servo(hardwareMap);
        angleServo.init("anglePivot", false, false, 0, 1, 0);
    }

    public void initSensors(HardwareMap hardwareMap) {
        //outtakeColor = hardwareMap.get(ColorSensor.class, "outtakeColor");
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensor");
    }




    public void initAutonomy(HardwareMap hardwareMap) {
        initMotors(hardwareMap);
        initServos(hardwareMap);
        initSensors(hardwareMap);
    }

    public void initTeleop(HardwareMap hardwareMap) {
        initMotors(hardwareMap);
        initDrivetrain(hardwareMap);

        //initIMU2(hardwareMap);
        initServos(hardwareMap);
        initSensors(hardwareMap);
    }

    public void initSimpleDriving(HardwareMap hardwareMap) {
        //initIMU2(hardwareMap);
        initMotors(hardwareMap);
    }

    public void init(HardwareMap hardwareMap){
        initMotors(hardwareMap);
        //initIMU2(hardwareMap);
        initServos(hardwareMap);
        initSensors(hardwareMap);
    }
}