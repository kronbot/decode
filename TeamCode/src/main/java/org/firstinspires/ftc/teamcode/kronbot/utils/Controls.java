package org.firstinspires.ftc.teamcode.kronbot.utils;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Button;

public class Controls {
    private Gamepad gamepad;
    public class Joystick{
        public ButtonWrapper button;
        public float x,y;
        private boolean left;

        public Joystick(boolean left) {
            this.left=left;
            if(left)
                this.button = new ButtonWrapper(()->gamepad.left_stick_button);
            else
                this.button = new ButtonWrapper(()->gamepad.right_stick_button);
        }

        public void update(){
            this.button.update();
            if(left){
                this.x = gamepad.left_stick_x;
                this.y = gamepad.left_stick_y;
            } else {
                this.x = gamepad.right_stick_x;
                this.y = gamepad.right_stick_y;
            }
        }
    }

    // Button wrappers
    public final ButtonWrapper cross;
    public final ButtonWrapper circle;
    public final ButtonWrapper square;
    public final ButtonWrapper triangle;
    public final ButtonWrapper leftBumper;
    public final ButtonWrapper rightBumper;
    public final ButtonWrapper dpadUp;
    public final ButtonWrapper dpadDown;
    public final ButtonWrapper dpadLeft;
    public final ButtonWrapper dpadRight;

    //Variable buttons
    public volatile float leftTrigger;
    public volatile float rightTrigger;

    //Joysticks
    public Joystick leftStick;
    public Joystick rightStick;


    public Controls(Gamepad gamepad) {
        this.gamepad = gamepad;
        cross = new ButtonWrapper(() -> gamepad.cross);
        circle = new ButtonWrapper(() -> gamepad.circle);
        square = new ButtonWrapper(() -> gamepad.square);
        triangle = new ButtonWrapper(() -> gamepad.triangle);
        leftBumper = new ButtonWrapper(() -> gamepad.left_bumper);
        rightBumper = new ButtonWrapper(() -> gamepad.right_bumper);
        dpadUp = new ButtonWrapper(() -> gamepad.dpad_up);
        dpadDown = new ButtonWrapper(() -> gamepad.dpad_down);
        dpadLeft = new ButtonWrapper(() -> gamepad.dpad_left);
        dpadRight = new ButtonWrapper(() -> gamepad.dpad_right);

        leftStick = new Joystick(true);
        rightStick = new Joystick(false);
    }

    // Update all buttons (call once per loop)
    public void update() {
        cross.update();
        circle.update();
        square.update();
        triangle.update();
        leftBumper.update();
        rightBumper.update();
        dpadUp.update();
        dpadDown.update();
        dpadLeft.update();
        dpadRight.update();

        leftTrigger = gamepad.left_trigger;
        rightTrigger = gamepad.right_trigger;

        leftStick.update();
        rightStick.update();
    }

    // Display all controller inputs in telemetry
    public void telemetry(Telemetry telemetry) {
        telemetry.addLine("=== CONTROLLER INPUTS ===");
        
        // Face buttons
        telemetry.addData("Cross (X)", cross.pressed());
        telemetry.addData("Circle (O)", circle.pressed());
        telemetry.addData("Square (□)", square.pressed());
        telemetry.addData("Triangle (△)", triangle.pressed());
        
        // Bumpers
        telemetry.addData("Left Bumper", leftBumper.pressed());
        telemetry.addData("Right Bumper", rightBumper.pressed());
        
        // Triggers
        telemetry.addData("Left Trigger", "%.2f", leftTrigger);
        telemetry.addData("Right Trigger", "%.2f", rightTrigger);
        
        // D-pad
        telemetry.addData("D-pad Up", dpadUp.pressed());
        telemetry.addData("D-pad Down", dpadDown.pressed());
        telemetry.addData("D-pad Left", dpadLeft.pressed());
        telemetry.addData("D-pad Right", dpadRight.pressed());
        
        // Left stick
        telemetry.addLine("--- Left Stick ---");
        telemetry.addData("Left Stick X", "%.2f", leftStick.x);
        telemetry.addData("Left Stick Y", "%.2f", leftStick.y);
        telemetry.addData("Left Stick Button", leftStick.button.pressed());
        
        // Right stick
        telemetry.addLine("--- Right Stick ---");
        telemetry.addData("Right Stick X", "%.2f", rightStick.x);
        telemetry.addData("Right Stick Y", "%.2f", rightStick.y);
        telemetry.addData("Right Stick Button", rightStick.button.pressed());
    }

    // Wrapper that simplifies button usage
    public static class ButtonWrapper {
        private final Button button = new Button();
        private final BooleanSupplier supplier;

        public ButtonWrapper(BooleanSupplier supplier) {
            this.supplier = supplier;
        }

        void update() {
            button.updateButton(supplier.get());
        }

        public boolean pressed() {
            return button.press();
        }

        public boolean shortPressed() {
            button.shortPress();
            return button.getShortToggle();
        }

        public boolean longPressed() {
            button.longPress();
            return button.getLongToggle();
        }

        public boolean justPressed() {
            return button.toggle();
        }

        public void resetToggles(){
            button.resetToggles();
        }
    }

    @FunctionalInterface
    interface BooleanSupplier {
        boolean get();
    }
}