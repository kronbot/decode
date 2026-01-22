package org.firstinspires.ftc.teamcode.kronbot.utils;

import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.CONTROLLER_DEADZONE;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.POWER_EXPONENT;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ROBOT_SPEED;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Button;

public class Controls {
    private Gamepad gamepad;

    /** Wrapper for a controller's joystick.<br>
     * The joystick axes have dead-zone correction.
     */
    public class Joystick{
        public ButtonWrapper button;
        public double x,y;
        private boolean left;

        public Joystick(boolean left) {
            this.left=left;
            if(left)
                this.button = new ButtonWrapper(()->gamepad.left_stick_button);
            else
                this.button = new ButtonWrapper(()->gamepad.right_stick_button);
        }

        /** Automatically called by update() in Controls */
        public void update(){
            this.button.update();
            if(left){
                this.x = gamepad.left_stick_x;
                this.y = gamepad.left_stick_y;
            } else {
                this.x = gamepad.right_stick_x;
                this.y = gamepad.right_stick_y;
            }

            if (Math.abs(this.x) < CONTROLLER_DEADZONE) this.x = 0;
            else {
                double normalizedValue = (Math.abs(this.x) - CONTROLLER_DEADZONE) / (1 - CONTROLLER_DEADZONE);
                this.x = Math.signum(this.x) * normalizedValue;
            }

            if (Math.abs(this.y) < CONTROLLER_DEADZONE) this.y = 0;
            else {
                double normalizedValue = (Math.abs(this.y) - CONTROLLER_DEADZONE) / (1 - CONTROLLER_DEADZONE);
                this.y = Math.signum(this.y) * normalizedValue;
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

    /** Update all buttons (call once per loop) */
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

    /** Wrapper that simplifies button usage */
    public static class ButtonWrapper {
        private final Button button = new Button();
        private final BooleanSupplier supplier;

        public ButtonWrapper(BooleanSupplier supplier) {
            this.supplier = supplier;
        }

        /**
         * Calls update on the underlying Button class. Used to update timings for toggles (mainly).
         * Automatically called by update() in Controls
         */
        void update() {
            button.updateButton(supplier.get());
        }

        /** True if button pressed. */
        public boolean pressed() {
            return button.press();
        }

        /** Toggle on short button presses (taps). Value flips on button release. */
        public boolean shortToggle() {
            button.shortPress();
            return button.getShortToggle();
        }

        /** Toggle on long button presses (holding the button for a bit more). Value flips on button release. */
        public boolean longToggle() {
            button.longPress();
            return button.getLongToggle();
        }

        /** Returns true only the first time it is called after the button was pressed. */
        public boolean justPressed() {
            return button.toggle();
        }

        /** Returns true only the first time it is called after the button was released. */
        public boolean justReleased() { return button.releaseToggle(); }
    }

    @FunctionalInterface
    interface BooleanSupplier {
        boolean get();
    }
}