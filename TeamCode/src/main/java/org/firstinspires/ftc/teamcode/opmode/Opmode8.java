package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.hardware.robotHardware.FLAPPER_2_CLOSE;
import static org.firstinspires.ftc.teamcode.hardware.robotHardware.FLAPPER_2_OPEN;
import static org.firstinspires.ftc.teamcode.hardware.robotHardware.FLAPPER_3_CLOSE;
import static org.firstinspires.ftc.teamcode.hardware.robotHardware.FLAPPER_3_OPEN;
import static org.firstinspires.ftc.teamcode.hardware.robotHardware.GATE_CLOSE;
import static org.firstinspires.ftc.teamcode.hardware.robotHardware.GATE_OPEN;
import static org.firstinspires.ftc.teamcode.hardware.robotHardware.INTAKE_POWER_INTAKE;
import static org.firstinspires.ftc.teamcode.hardware.robotHardware.INTAKE_POWER_OUTTAKE;
import static org.firstinspires.ftc.teamcode.hardware.robotHardware.INTAKE_POWER_STOP;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.robotHardware;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.hardware.robotHardware;

import java.util.function.Supplier;

@TeleOp(name = "Opmode8")
public class Opmode8 extends LinearOpMode {
    public robotHardware robot = new robotHardware();
    private double controller1Speed = 1.0;
    int ShootSpeed=2000;
    ElapsedTime TimeFromSpinUp = new ElapsedTime();
    boolean bLaunchRequested = false;
    boolean last_right_bumper=false;
    boolean IsPusherOpen=true;
    boolean need_back_spin=false;
    ElapsedTime backSpinTimer = new ElapsedTime();
    public enum LAUNCH_STATES {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHED,   // trigger back to READY
    }
    public LAUNCH_STATES LaunchState;
    @Override
    public void runOpMode() {
        waitForStart();
        robot.init(hardwareMap);
        TimeFromSpinUp.reset();
        robot.pusher.setPosition(1);

        while (opModeIsActive()) {
//DRIVING
            double x_dir = gamepad1.left_stick_x * controller1Speed;
            double y_dir = gamepad1.left_stick_y * controller1Speed;
            double turn = gamepad1.right_stick_x * controller1Speed;

            double flPower = -x_dir + y_dir - turn;
            double blPower =  x_dir + y_dir - turn;
            double frPower =  x_dir + y_dir + turn;
            double brPower = -x_dir + y_dir + turn;

            // The following code it ensure power of all motors are scaling down properly
            // so that the power is within the range [-1.0, 1.0]
            // Direct limit check could cause imbalanced power levels...
            double scaling = Math.max(1.0,
                    Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)),
                            Math.max(Math.abs(blPower), Math.abs(brPower))));
            flPower = flPower / scaling;
            frPower = frPower / scaling;

            blPower = blPower / scaling;
            brPower = brPower / scaling;

            robot.setDrivePower(flPower, frPower, blPower, brPower);

            if(gamepad2.left_trigger>=0.2){
                need_back_spin=true;
                backSpinTimer.reset();
            }

            if(gamepad2.left_bumper){
                robot.pusher.setPosition(0);
            } else if(IsPusherOpen) robot.pusher.setPosition(0.7);
            else robot.pusher.setPosition(1);
            if(gamepad2.right_bumper&&!last_right_bumper){
                last_right_bumper=true;
                IsPusherOpen=!IsPusherOpen;
            }else if(!gamepad2.right_bumper){
                last_right_bumper=false;
            }
            if(gamepad2.right_trigger>=0.2) {
                bLaunchRequested = true;
                TimeFromSpinUp.reset();
                LaunchState = LAUNCH_STATES.IDLE;
            }
            backSpin();
            launch();
            idle();

            //pusher=1开
            //pusher=0.5合
            //pusher-0推
        }
    }
    void backSpin(){
        if(!need_back_spin) return;
        robot.motorshoot.setVelocity(-1000);
        if(backSpinTimer.seconds()>0.2){
            need_back_spin=false;
            robot.motorshoot.setVelocity(0);
        }
    }
     void launch(){

        if (!bLaunchRequested)
            return;
        switch(LaunchState){
            case IDLE:
                TimeFromSpinUp.reset();
                robot.pusher.setPosition(0.7);
                robot.motorshoot.setVelocity(ShootSpeed);
                LaunchState=LAUNCH_STATES.SPIN_UP;
                telemetry.addLine("LaunchState: IDLE");
                break;
            case SPIN_UP:
                if(TimeFromSpinUp.seconds()>3)
                    LaunchState=LAUNCH_STATES.LAUNCH;

                telemetry.addData("motor speed", robot.motorshoot.getVelocity());
                break;
            case LAUNCH:
                robot.pusher.setPosition(0);
                if(TimeFromSpinUp.seconds()<3.5) break;
                robot.pusher.setPosition(1);
                LaunchState=LAUNCH_STATES.LAUNCHED;
                robot.motorshoot.setVelocity(0);
                telemetry.addLine("LaunchState: Launch");
                break;
            case LAUNCHED:
                robot.motorshoot.setVelocity(0);
                robot.pusher.setPosition(1);

                LaunchState = LAUNCH_STATES.IDLE;
                bLaunchRequested = false;
                telemetry.addLine("Done Launching");
                break;
        }
    }
}

