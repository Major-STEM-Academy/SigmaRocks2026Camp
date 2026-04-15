package org.firstinspires.ftc.teamcode.hardware;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;


public class robotHardware {
    HardwareMap hwMap =  null;

    public IMU imu;
    public DcMotor motorfl = null;
    public DcMotor motorfr = null;
    public DcMotor motorbr = null;
    public DcMotor motorbl = null;
    public DcMotor motorintake = null;
    public DcMotorEx motorturret = null;
    // The gate servo is to block artifacts before shoot motor is reaching a target rpm.
    public Servo gate = null;
    public Servo flapper2 = null;
    // The flapper servo is to push the flapper inward so that artifacts can touch 3rd stage intake for shooting artifacts.
    public Servo flapper3 = null;
    //public CRServo pushServo = null;
    public DcMotor elevator = null;

    public GoBildaPinpointDriver pinpoint = null;
    public Limelight3A limelight;

    // Use DcMotorEx for shooter so we can control velocity (ticks/sec)
    public DcMotorEx motorshoot = null;

    public DistanceSensor distanceF = null;
    public DistanceSensor distanceR = null;
    public DistanceSensor distanceL = null;


    // Initial robot orientation
    public YawPitchRollAngles orientation0;
    public AngularVelocity angularVelocity0;
    public double yaw0;

    final double STOP_SPEED = 0.0;
    static final double TICKS_PER_REVOLUTION = 28.0;

    final double SHOOTER_TARGET_INIT_RPM = 2500;    // RPM: Rotations Per Minute
    final double SHOOTER_TARGET_RANGE = 100;


    static final double MAX_TICKS_PER_SEC = 2800.0;

    private double shooter_target_rpm = SHOOTER_TARGET_INIT_RPM;
    private double shooter_target_ticks = shooter_target_rpm * TICKS_PER_REVOLUTION / 60;
    private double shooter_target_ticks_low = (shooter_target_rpm - SHOOTER_TARGET_RANGE) * TICKS_PER_REVOLUTION / 60;



    // For motot encoders
    static final double TICKS_DRIVE_PER_REVOLUTION = 384.5;    // GoBilda 435 rpm Yellow Jacket Motor
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double TICKS_PER_INCH = (TICKS_DRIVE_PER_REVOLUTION * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    public static final double GATE_CLOSE = 0.2; //0.7
    public static final double GATE_OPEN = 0.5; //0.2
    // Flapper for 2nd stage intake
    public static final double FLAPPER_2_OPEN = 0.1;
    public static final double FLAPPER_2_CLOSE = 0.7;
    // Flapper is open to hold an artifact without touching the 3rd stage intake.
    public static final double FLAPPER_3_OPEN = 0.514;
    public static final double FLAPPER_3_CLOSE = 0.499;

    public static final double INTAKE_POWER_INTAKE = -1.0;
    public static final double INTAKE_POWER_OUTTAKE = 1.0;
    public static final double INTAKE_POWER_STOP = 0.0;

    private final double turretTicksPerDegree = 14.8;


    boolean bShootRequested = false;
    int countShots = 0;

    ElapsedTime triggerShootTimer = new ElapsedTime();
    ElapsedTime triggerReadyTimer = new ElapsedTime();
    ElapsedTime shootTimer = new ElapsedTime();

    public enum LAUNCH_STATES {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,  // trigger SHOOT state
        LAUNCHED,   // trigger back to READY
    }

    private LAUNCH_STATES launchState;

    public double ticksPerInch = 31.3;
    public double ticksPerDegree = 12;
    static final double TRIGGER_SHOOT_TIME = 0.5;
    static final double TRIGGER_READY_TIME = 5;
    static final double LAST_TRIGGER_READY_TIME = 2;
    static final double TOTAL_SHOOT_TIME = 15;


    public robotHardware() {}

    public void init(HardwareMap ahwMap)    {
        hwMap = ahwMap;

        motorfl = hwMap.get(DcMotor.class, "fl");
        motorfr = hwMap.get(DcMotor.class, "fr");
        motorbl = hwMap.get(DcMotor.class, "bl");
        motorbr = hwMap.get(DcMotor.class, "br");
        //front right motor no encoder


        motorfr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorfl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorbr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorbl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // motorfl.setDirection(DcMotor.Direction.REVERSE);
        motorfr.setDirection(DcMotor.Direction.REVERSE);
        motorbr.setDirection(DcMotor.Direction.REVERSE);


        setDrivetrainMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu = hwMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

    }

    public void setAutoDriveMotorMode() {
        motorfr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorfl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorbr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorbl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorfr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorfl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorbr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorbl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorfr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorfl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorbr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorbl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public double getTurretTicks(double tx){
        double turretDist = 1.0; // inches





        return ticksPerDegree;
    }



    public double getCurrentYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void setDrivetrainMode(DcMotor.RunMode mode) {
        motorfl.setMode(mode);
        motorfr.setMode(mode);
        motorbl.setMode(mode);
        motorbr.setMode(mode);
    }

    public void setDrivePower(double fl, double fr, double bl, double br) {
        if (fl > 1.0)
            fl = 1.0;
        else if (fl < -1.0)
            fl = -1.0;

        if (fr > 1.0)
            fr = 1.0;
        else if (fr < -1.0)
            fr = -1.0;

        if (bl > 1.0)
            bl = 1.0;
        else if (bl < -1.0)
            bl = -1.0;

        if (br > 1.0)
            br = 1.0;
        else if (br < -1.0)
            br = -1.0;

        motorfl.setPower(fl);
        motorfr.setPower(fr);
        motorbl.setPower(bl);
        motorbr.setPower(br);
    }


    // Shooter helpers using DcMotorEx velocity control (ticks/sec)
    public void startShooterAtRPM(double rpm) {
        if (motorshoot == null) return;
        // convert RPM to ticks/sec
        double ticksPerRev = 28; //FIX  2786
        double ticksPerSec = ticksPerRev * rpm / 60.0;
        motorshoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorshoot.setVelocity(ticksPerSec);
    }

    public double getShooterRPM() {
        if (motorshoot == null) return 0.0;
        double ticksPerSec = motorshoot.getVelocity();
        double ticksPerRev = motorshoot.getMotorType().getTicksPerRev();
        return ticksPerSec * 60.0 / ticksPerRev;
    }

    public void stopShooter() {
        if (motorshoot == null) return;
        motorshoot.setPower(0.0);
        motorshoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void autoShoot(double rpm){
        startShooterAtRPM(rpm + 100);
        if(motorshoot.getVelocity() > ((rpm/2) - 100)){
            motorintake.setPower(1.0);
            // sleep(400);
        }
        motorintake.setPower(0.0);

    }
    public void setShooterTargetRpm(double target_rpm) {
        shooter_target_rpm = target_rpm;
        shooter_target_ticks = Math.max(0, Math.min(MAX_TICKS_PER_SEC,
                shooter_target_rpm * TICKS_PER_REVOLUTION / 60));
        shooter_target_ticks_low = Math.max(0, Math.min(MAX_TICKS_PER_SEC,
                (shooter_target_rpm - SHOOTER_TARGET_RANGE) * TICKS_PER_REVOLUTION / 60));
    }

    public void driveMotors(int flTarget, int blTarget, int frTarget, int brTarget,
                            double power,
                            boolean bKeepYaw, double targetYaw) {
        double currentYaw, diffYaw;
        double powerDeltaPct, powerL, powerR;
        int direction;

        motorfl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorbl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorfr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorbr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorfl.setTargetPosition(flTarget);
        motorbl.setTargetPosition(blTarget);
        motorfr.setTargetPosition(frTarget);
        motorbr.setTargetPosition(brTarget);

        motorfl.setPower(power);
        motorbl.setPower(power);
        motorfr.setPower(power);
        motorbr.setPower(power);

        motorfl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorbl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorfr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorbr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Defensive programming.
        // Use bKeepYaw only when all targets are the same, meaning moving in a straight line
        if (!((flTarget == blTarget)
                && (flTarget == frTarget)
                && (flTarget == brTarget)))
            bKeepYaw = false;
        direction = (flTarget > 0) ? 1 : -1;
        while ((motorfl.isBusy() &&
                        motorbl.isBusy() &&
                        motorfr.isBusy() &&
                        motorbr.isBusy())) {
            if (bKeepYaw) {

                currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                if (Math.abs(currentYaw - targetYaw) > 2.0)
                    powerDeltaPct = 0.25;
                else
                    powerDeltaPct = Math.abs(currentYaw - targetYaw) / 2.0 * 0.25;
                if (currentYaw < targetYaw) {
                    powerL = power * (1 - direction * powerDeltaPct);
                    powerR = power * (1 + direction * powerDeltaPct);
                } else {
                    powerL = power * (1 + direction * powerDeltaPct);
                    powerR = power * (1 - direction * powerDeltaPct);
                }
                if (powerL > 1.0)
                    powerL = 1.0;
                if (powerR > 1.0)
                    powerR = 1.0;
                motorfl.setPower(powerL);
                motorbl.setPower(powerL);
                motorfr.setPower(powerR);
                motorbr.setPower(powerR);
            }
        }

        motorfl.setPower(0);
        motorbl.setPower(0);
        motorfr.setPower(0);
        motorbr.setPower(0);
    }


    public void turnOnIntakeSubsystem() {
        motorintake.setPower(-1.0);
        //servoL.setPower(-1.0);
        //servoR.setPower(1.0);
    }
    public void turnOffIntakeSubsystem() {
        motorintake.setPower(0);
        //servoL.setPower(0);
        //servoR.setPower(0);
    }

    public void launch(int numShots) {
        switch (launchState) {
            case IDLE:
                if (bShootRequested) {
                    launchState = LAUNCH_STATES.SPIN_UP;
                }
                break;
            case SPIN_UP:
                if (bShootRequested) {
                    if (motorshoot.getVelocity() > shooter_target_ticks_low) {
                        launchState = LAUNCH_STATES.LAUNCH;
                    }
                }
                break;
            case LAUNCH:
                launchState = LAUNCH_STATES.LAUNCHING;
                turnOnIntakeSubsystem();
                shootTimer.reset();
                /*
                trigger.setPosition(TRIGGER_SHOOT);
                triggerShootTimer.reset();*/
                break;
            case LAUNCHING:
                if (shootTimer.seconds() > TOTAL_SHOOT_TIME) {
                    launchState = LAUNCH_STATES.LAUNCHED;
                    /*
                    trigger.setPosition(TRIGGER_READY);
                    triggerReadyTimer.reset();*/
                }
                break;
            case LAUNCHED:
                turnOffIntakeSubsystem();
                bShootRequested = false;
                launchState = LAUNCH_STATES.IDLE;
                /*

                if (triggerReadyTimer.seconds() > TRIGGER_READY_TIME
                        || ((countShots == (numShots - 1))
                        && triggerReadyTimer.seconds() > LAST_TRIGGER_READY_TIME)) {
                    countShots ++;
                    triggerReadyTimer.reset();
                    if (countShots >= numShots) {
                        // Finished all shots. Stop shooting...
                        bShootRequested = false;
                        launchState = LAUNCH_STATES.IDLE;
                    } else {
                        launchState = LAUNCH_STATES.LAUNCH;
                    }
                }
                 */
                break;
        }
        if (bShootRequested) {
            motorshoot.setVelocity(shooter_target_ticks);
            motorshoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        else {
            motorshoot.setVelocity(STOP_SPEED);
            motorshoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            launchState = LAUNCH_STATES.IDLE;
        }
        telemetry.addData("State", launchState);
        telemetry.addData("shooter Velocity Target (RPM)", shooter_target_rpm);
        telemetry.addData("shooter Velocity Actual (RPM)",
                motorshoot.getVelocity() / TICKS_PER_REVOLUTION * 60);
        telemetry.addData("Shot", countShots + " of " + numShots);
        telemetry.addData("TriggerShootTimer", triggerShootTimer.seconds());
        telemetry.addData("TriggerReadyTimer", triggerReadyTimer.seconds());
        telemetry.update();
    }

    public double getTicksPerInch() {
        return TICKS_PER_INCH;
    }

    public void prepareShoot() {
        bShootRequested = true;
        countShots = 0;
    }
    public boolean isShootRequested() {
        return bShootRequested;
    }


}
