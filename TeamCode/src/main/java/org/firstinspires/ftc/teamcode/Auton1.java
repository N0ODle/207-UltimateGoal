package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Hardware.MovementEnum;
//import org.firstinspires.ftc.teamcode.TensorFlowStuff.TensorFlow;
import org.firstinspires.ftc.teamcode.Hardware.Bot;
import java.lang.Thread;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
//import com.vuforia.CameraDevice;

@Autonomous(name="auton test", group="Autonomous")
public class Auton1 extends OpMode{
    private static final double TICKS_PER_REV = 403.9 * (80.0/72.0);
    private static final double TICKS_PER_INCH = TICKS_PER_REV/(4.0 * Math.PI);
    private ElapsedTime runtime = new ElapsedTime();
    //    private DigitalChannel DigChannel;
    Bot robot = new Bot();
    int auto = 0;

    public void init() {
        robot.init(hardwareMap, telemetry, false);
//        tensorFlow.init(hardwareMap, telemetry);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot.wobbleHook.setPosition(1.0);



//        robot.hook.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop(){
        try{
            switch (auto) {
                case 0:
                    robot.changeRunModeAuton(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    telemetry.addData(">", robot.FL.getCurrentPosition());
                    telemetry.addData(">", auto);
                    telemetry.update();


                    auto++;
                    break;

                case 1:
                    robot.wobbleGrabber.setPosition(0.0);
                    robot.wobbleHook.setPosition(1.0);
                    Thread.sleep(1000);
                    auto++;
                    break;

                case 2:
                    int en = robot.autonDrive(MovementEnum.FORWARD, (int)(TICKS_PER_INCH * 70));
                    robot.changeRunModeAuton(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.drivePower(0.5);
                    telemetry.addData("Cas1, en: ", en);
                    telemetry.addData("FL: ", robot.FL.getCurrentPosition());
                    telemetry.addData("FR: ", robot.FR.getCurrentPosition());
                    telemetry.addData("BL: ", robot.BL.getCurrentPosition());
                    telemetry.addData("BR: ", robot.BR.getCurrentPosition());

                    telemetry.update();

                    if(en >= (int)(TICKS_PER_INCH * 70)){
                        robot.autonDrive(MovementEnum.STOP, 0);
                        robot.changeRunModeAuton(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.changeRunModeAuton(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.strafePower(0.0);
                        auto++;
                    }
                    break;

                case 3:
                    robot.shooter.setPower(-0.75);
                    runtime.reset();
                    auto++;
                    break;

                case 4:
                    for(int i = 0; i < 3; i++){
                        if(runtime.seconds() >= 1.5){
                            robot.discPlacer.setPosition(0.1);
                        }

                        robot.discPlacer.setPosition(1.0);
                        runtime.reset();
                    }

                    auto = -1;
                    break;

                case 5:
                    en = robot.autonDrive(MovementEnum.BACKWARD, 2900);
                    robot.changeRunModeAuton(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.drivePower(1.0);
                    telemetry.addData("Cas1, en: ", en);
                    telemetry.addData("FL: ", robot.FL.getCurrentPosition());
                    telemetry.addData("FR: ", robot.FR.getCurrentPosition());
                    telemetry.addData("BL: ", robot.BL.getCurrentPosition());
                    telemetry.addData("BR: ", robot.BR.getCurrentPosition());


                    telemetry.update();


                    if(en >= 2900){
                        robot.autonDrive(MovementEnum.STOP, 0);
                        robot.changeRunModeAuton(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.changeRunModeAuton(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.strafePower(0.0);
                        auto++;
                    }
                    break;

                case 6:
                    en = robot.autonDrive(MovementEnum.LEFTTURN, 1000);
                    robot.changeRunModeAuton(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.drivePower(0.5);
//                robot.drivePower(1.0);
                    telemetry.addData("Cas1, en: ", en);
                    telemetry.addData("FL: ", robot.FL.getCurrentPosition());
                    telemetry.addData("FR: ", robot.FR.getCurrentPosition());
                    telemetry.addData("BL: ", robot.BL.getCurrentPosition());
                    telemetry.addData("BR: ", robot.BR.getCurrentPosition());


                    telemetry.update();


                    if(en >= 1000){
                        robot.autonDrive(MovementEnum.STOP, 0);
                        robot.changeRunModeAuton(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.changeRunModeAuton(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.strafePower(0.0);
                        robot.wobbleGrabber.setPosition(0.9);
                        robot.wobbleHook.setPosition(1.0);
                        Thread.sleep(1000);
                        auto++;
                    }
                    break;

                case 7:
                    en = robot.autonDrive(MovementEnum.RIGHTTURN, 1000);
                    robot.changeRunModeAuton(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.drivePower(0.5);
                    telemetry.addData("Cas1, en: ", en);
                    telemetry.addData("FL: ", robot.FL.getCurrentPosition());
                    telemetry.addData("FR: ", robot.FR.getCurrentPosition());
                    telemetry.addData("BL: ", robot.BL.getCurrentPosition());
                    telemetry.addData("BR: ", robot.BR.getCurrentPosition());


                    telemetry.update();


                    if(en >= 1000){
                        robot.autonDrive(MovementEnum.STOP, 0);
                        robot.changeRunModeAuton(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.changeRunModeAuton(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.strafePower(0.0);
                        auto++;
                    }
                    break;


                case 8:
                    en = robot.autonDrive(MovementEnum.RIGHTSTRAFE, 200);
                    robot.changeRunModeAuton(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.drivePower(0.5);
                    telemetry.addData("Cas1, en: ", en);
                    telemetry.addData("FL: ", robot.FL.getCurrentPosition());
                    telemetry.addData("FR: ", robot.FR.getCurrentPosition());
                    telemetry.addData("BL: ", robot.BL.getCurrentPosition());
                    telemetry.addData("BR: ", robot.BR.getCurrentPosition());


                    telemetry.update();


                    if(en >= 200){
                        robot.autonDrive(MovementEnum.STOP, 0);
                        robot.changeRunModeAuton(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.changeRunModeAuton(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.strafePower(0.0);
                        robot.intake.setPower(0.9);
                        Thread.sleep(1000);
                        auto++;
                    }
                    break;

                case 9:
                    en = robot.autonDrive(MovementEnum.FORWARD, 1800);
                    robot.changeRunModeAuton(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.drivePower(1.0);
                    telemetry.addData("Cas1, en: ", en);
                    telemetry.addData("FL: ", robot.FL.getCurrentPosition());
                    telemetry.addData("FR: ", robot.FR.getCurrentPosition());
                    telemetry.addData("BL: ", robot.BL.getCurrentPosition());
                    telemetry.addData("BR: ", robot.BR.getCurrentPosition());


                    telemetry.update();


                    if(en >= 1800){
                        robot.autonDrive(MovementEnum.STOP, 0);
                        robot.changeRunModeAuton(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.changeRunModeAuton(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.strafePower(0.0);
                        robot.shooter.setPower(0.7);
                        Thread.sleep(1000);
                        auto++;
                    }
                    break;


                case 10:
                    en = robot.autonDrive(MovementEnum.FORWARD, 600);
                    robot.changeRunModeAuton(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.drivePower(1.0);
                    telemetry.addData("Cas1, en: ", en);
                    telemetry.addData("FL: ", robot.FL.getCurrentPosition());
                    telemetry.addData("FR: ", robot.FR.getCurrentPosition());
                    telemetry.addData("BL: ", robot.BL.getCurrentPosition());
                    telemetry.addData("BR: ", robot.BR.getCurrentPosition());


                    telemetry.update();


                    if(en >= 600){
                        robot.autonDrive(MovementEnum.STOP, 0);
                        robot.changeRunModeAuton(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.changeRunModeAuton(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.strafePower(0.0);
                        robot.wobbleHook.setPosition(0.0);
                        robot.wobbleGrabber.setPosition(0.1);
                        Thread.sleep(1000);
                        auto++;
                    }
                    break;

                    
            }
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();  // set interrupt flag
            System.out.println("Failed to compute sum");
        }
        telemetry.update();
    }
}