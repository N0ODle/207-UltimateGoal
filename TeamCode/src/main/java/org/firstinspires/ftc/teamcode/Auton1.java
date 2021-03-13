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

@Autonomous(name="Auton1", group="Autonomous")
public class Auton1 extends OpMode{
    private static final double TICKS_PER_REV = 403.9 * (80.0/72.0);
    private static final double TICKS_PER_INCH = TICKS_PER_REV/(4.0 * Math.PI);
    private ElapsedTime runtime = new ElapsedTime();
    //    private DigitalChannel DigChannel;
    Bot robot = new Bot();
    int auto = 0;

    // count the number of ring shots taken
    int shotCount = 1;

    // variable for the wobble hook (hook starts out as closed)
    boolean hookOpen = false;

    // variable for the wobble arm (arm starts out as out)
    boolean armIn = false;

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
                    // set wobble hook pos to grabbing position and wobble grabber to an in position
                    robot.wobbleGrabber.setPosition(0.0);
                    robot.wobbleHook.setPosition(1.0);
                    Thread.sleep(1000);
                    auto++;
                    break;

                case 2:
                    // drive forward 70 inches (to the line) and stop
                    int en = robot.autonDrive(MovementEnum.FORWARD, (int)(TICKS_PER_INCH * 70));
                    robot.changeRunModeAuton(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.drivePower(0.5);
                    telemetry.addData("Cas1, en: ", en);
                    telemetry.addData("FL: ", robot.FL.getCurrentPosition());
                    telemetry.addData("FR: ", robot.FR.getCurrentPosition());
                    telemetry.addData("BL: ", robot.BL.getCurrentPosition());
                    telemetry.addData("BR: ", robot.BR.getCurrentPosition());

                    telemetry.update();

                    // stop when at 70 inches
                    if(en >= (int)(TICKS_PER_INCH * 70)){
                        robot.autonDrive(MovementEnum.STOP, 0);
                        robot.changeRunModeAuton(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.changeRunModeAuton(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.strafePower(0.0);
                        auto++;
                    }
                    break;

                case 3:
                    // Run the shooter to the correct power and start the timer
                    robot.shooter.setPower(-0.75);
                    runtime.reset();
                    auto++;
                    break;

                case 4:
                    // if the timer is less than 1.5 sec from last reset, do nothing
                    double delay = 1.0;
                    if(runtime.seconds() < 1.5){ }

                    // if it is 2 sec from last reset, shoot disc
                    else if(runtime.seconds() < delay * 2){
                        robot.discPlacer.setPosition(0.1);
                    }

                    // one sec after shooting dic, reset arm
                    else if(runtime.seconds() < delay * 3){
                        robot.discPlacer.setPosition(1.0);
                    }

                    else{
                        // If the amount of shots shot is 3, move onto next case
                        shotCount++;
                        if(shotCount > 3){
                            auto++;
                        }

                        // if not, reset the timer and go through the case again
                        else{
                            runtime.reset();
                        }
                    }

                    break;

                case 5:
                    // Set shooter power to 0 and move on
                    robot.shooter.setPower(0.0);
                    auto = -1;
                    break;

                case 6:
                    // drive forward another 8 inches
                    en = robot.autonDrive(MovementEnum.FORWARD, (int)(TICKS_PER_INCH * 8));
                    robot.changeRunModeAuton(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.drivePower(0.5);
                    telemetry.addData("Cas1, en: ", en);
                    telemetry.addData("FL: ", robot.FL.getCurrentPosition());
                    telemetry.addData("FR: ", robot.FR.getCurrentPosition());
                    telemetry.addData("BL: ", robot.BL.getCurrentPosition());
                    telemetry.addData("BR: ", robot.BR.getCurrentPosition());

                    telemetry.update();

                    // once at 8 inches, stop and put the wobble goal down
                    if(en >= (int)(TICKS_PER_INCH * 8)){
                        robot.autonDrive(MovementEnum.STOP, 0);
                        robot.changeRunModeAuton(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.changeRunModeAuton(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.strafePower(0.0);
                        robot.wobbleGrabber.setPosition(1.0);
                        runtime.reset();
                        auto++;
                    }

                    break;

                case 7:
                    if(runtime.seconds() < 1.5){ }

                    else if(runtime.seconds() < 2){
                        hookOpen = true;
                        robot.wobbleHook.setPosition(0.1);
                    }

                    else{
                        if(hookOpen == true){
                            runtime.reset();
                            auto++;
                        }
                    }
                    break;


                case 8:
                    if(runtime.seconds() < 1.5){ }

                    else if(runtime.seconds() < 2){
                        armIn = true;
                        robot.wobbleGrabber.setPosition(0.1);
                    }

                    else{
                        if(armIn == true){
                            auto++;
                        }
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