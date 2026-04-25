package org.firstinspires.ftc.teamcode.auto;


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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.robotHardware;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.hardware.robotHardware;
import org.firstinspires.ftc.teamcode.opmode.Opmode8;
import org.firstinspires.ftc.teamcode.opmode.Opmode8.LAUNCH_STATES;

import java.util.function.Supplier;
@Autonomous(name = "auto")
public class auto extends LinearOpMode {
    private boolean bLaunchReqeusted = true;
    ElapsedTime TimeFromSpinUp = new ElapsedTime();
    public LAUNCH_STATES LaunchState= LAUNCH_STATES.IDLE;
    public enum LAUNCH_STATES {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHED,   // trigger back to READY
    }
    public robotHardware robot = new robotHardware();
    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        waitForStart();

        robot.setDrivePower(0.3, 0.3, 0.3, 0.3);
        sleep(3200);
        robot.setDrivePower(0, 0, 0, 0);
        TimeFromSpinUp.reset();
        while(bLaunchReqeusted)
            launch();
        robot.setDrivePower(-0.3,0.3,0.3,-0.3);
        //0.3,-0.3,-0.3,0.3 for blue side
        //-0.3,0.3,0.3,-0.3 for red side
        sleep(1670);
        robot.setDrivePower(0, 0, 0, 0);
    }
    void launch(){

        if (!bLaunchReqeusted)
            return;
        switch(LaunchState){
            case IDLE:
                TimeFromSpinUp.reset();
                robot.pusher.setPosition(0.7);
                robot.motorshoot.setVelocity(1000);
                LaunchState= LAUNCH_STATES.SPIN_UP;
                telemetry.addData("case","IDLE");
                break;
            case SPIN_UP:
                if(TimeFromSpinUp.seconds()>2.5)
                    LaunchState= LAUNCH_STATES.LAUNCH;

                telemetry.addData("motor speed", robot.motorshoot.getVelocity());
                break;
            case LAUNCH:
                robot.pusher.setPosition(0);
                if(TimeFromSpinUp.seconds()<3) break;
                robot.pusher.setPosition(1);
                robot.motorshoot.setVelocity(0);
                telemetry.addData("case","LAUNCH");
                LaunchState= LAUNCH_STATES.LAUNCHED;
                break;
            case LAUNCHED:
                robot.motorshoot.setVelocity(0);
                robot.pusher.setPosition(1);

                LaunchState = LAUNCH_STATES.IDLE;
                bLaunchReqeusted = false;
                break;
        }
    }
    
}