
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotor;

//import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.teamcode.Hardware.Bot;
import org.firstinspires.ftc.teamcode.Hardware.*;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Tele", group="Teleop")

//@Disabled
public class Tele extends OpMode {

    Bot bot = new Bot();
    int TankDrive = -1;
    double factor = 0.75;
    TankDrive drive;

    // Buffer area for controller triggers
    private double buffer = 0.15;

    // Position for the disc pusher arm (-1 is out and 1 is pushing) and debouncing variable
    //     which prevents continuous update so the position does not try to change with every loop
    //     pass
    private int armPos = -1;
    private boolean debounceArm = false;

    // Position for the wobble hook (-1 is in and 1 is out) and debouncing variable
    private int hookPos = -1;
    private boolean debounceHook = false;

    // Position for the wobble arm (-1 is back and 1 is forward) and debouncing variable
    private int wobbleArmPos = -1;
    private boolean debounceWobble = false;
    /*
    fr = 1
    fl = 2
    bl = 3
    br = 4

     */

    @Override
    public void init() {
        bot.init(hardwareMap, telemetry, false);
        drive = new TankDrive(bot);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        // Tank/Game drive initialization moved to init so it is not re-initialized every time
        double leftStickY = (double) -gamepad1.left_stick_y;
        double rightTrigger = (double) gamepad1.right_trigger;
        double leftTrigger = (double) gamepad1.left_trigger;
        double rightStickX = (double) gamepad1.right_stick_x;
        double rightStickY = (double) -gamepad1.right_stick_y;

 //       double leftTriggerIntake = (double) gamepad2.left_trigger;
 //       double rightTriggerIntake = (double) -gamepad2.right_trigger;
  //      double leftStickServo = (double) gamepad2.left_stick_x;
 //       double rightStickServo = (double) gamepad2.right_stick_x;

        drive.driveBot(leftStickY, rightStickX, rightTrigger, leftTrigger, rightStickY, 0.0, factor);

        bot.shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bot.intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Update all gamepad2 mechanisms
        updateShooter();
        updateIntake();
        updateDiscPlacer();
        updateWobble();
    }


    // Update's shooter power
    private void updateShooter(){
        double minPower = 0.0;
        double goalPower = -0.45;
        double pegPower = -0.45;

        // If right bumper is pressed, add enough power to shoot into the top goal
        if(gamepad2.right_bumper){
            bot.shooter.setPower(goalPower);
        }
        // If left bumper not pressed, do not do anything
        else if(gamepad2.left_bumper){
            bot.shooter.setPower(pegPower);
        }
        // if right bumper is pressed down, add enough power to shoot the peg
        else {
            bot.shooter.setPower(minPower);
        }
    }

    // Updates intake power
    private void updateIntake(){
        double minIntakePower = 0.0;

        // Set intake power whatever percent the left trigger is pushed down (if it is)
        if(gamepad2.left_trigger > buffer) {
            bot.intake.setPower(-gamepad2.left_trigger);
        }
        // Set outtake power to whatever percent the right trigger is pushed down(if it is)
        else if(gamepad2.right_trigger > buffer){
            bot.intake.setPower(gamepad2.right_trigger);
        }
        // If neither trigger is pushed down, set the intake power to 0
        else{
            bot.intake.setPower(minIntakePower);
        }
    }

    // Updates disc pusher
    private void updateDiscPlacer(){
        // if a is pressed, the arm can only switch position if the debounce variable if false
        //     if it is, then it will become true after the  position switches, allowing the
        //     player to hold down the button without it trying to switch positions
        if(gamepad2.a){
            // Arm goes to pushing position if it is at rest
            if(!debounceArm && armPos == -1){
                bot.discPlacer.setPosition(0.9);
                telemetry.addData(">", "0.9");
                telemetry.update();
                armPos *= -1;
                debounceArm = true;
            }

            // Arm goes to rest position if a is pressed an it is pushing
            else if(!debounceArm && armPos == 1){
                bot.discPlacer.setPosition(0.1);
                telemetry.addData(">", "0.1");
                telemetry.update();
                armPos *= -1;
                debounceArm = true;
            }
        }

        // If a is not pressed, make sure the debounce variable is false
        else{
            debounceArm = false;
        }

//        // If b is pressed, disc pusher is pushing
//        if(gamepad2.b) {
//            bot.discPlacer.setPosition(0.9);
//            telemetry.addData(">", "0.9");
//            telemetry.update();
//        }
    }

    // Updates wobble mechanism
    private void updateWobble(){
        // Same debouncing logic
        // x-button on gamepad2 sets wobble hook position to in/out
        if(gamepad2.x){
            if(!debounceHook && hookPos == -1) {
                bot.wobbleGrabber.setPosition(0);
                telemetry.addData(">", "0");
                telemetry.update();
                hookPos *= -1;
                debounceHook = true;
            }
            else if(!debounceHook && hookPos == 1) {
                bot.wobbleGrabber.setPosition(0.9);
                telemetry.addData(">", "0.9");
                telemetry.update();
                hookPos *= -1;
                debounceHook = true;
            }
        }

        else{
            debounceHook = false;
        }

        // Same debouncing logic
        if(gamepad2.y){
            // y-button on gamepad2 sets wobble arm position to in/out
            if(!debounceWobble && wobbleArmPos == -1) {
                bot.wobbleHook.setPosition(0);
                telemetry.addData(">", "0");
                telemetry.update();
                wobbleArmPos *= -1;
                debounceWobble = true;
            }
            else if(!debounceWobble && wobbleArmPos == 1) {
                bot.wobbleHook.setPosition(1);
                telemetry.addData(">", "1");
                telemetry.update();
                wobbleArmPos *= -1;
                debounceWobble = true;
            }
        }

        else{
            debounceWobble = false;
        }
    }

    @Override
    public void stop() {
        bot.stop();

    }
}