package frc.robot.controllers;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.GenericHID;

public class GuitarHeroController extends GenericHID {
    /**
     * Construct an instance of a controller.
     *
     * @param port The port index on the Driver Station that the controller is
     *             plugged into.
     */
    public GuitarHeroController(final int port) {
        super(port);

        HAL.report(tResourceType.kResourceType_XboxController, port + 1);
    }

    /** Represents a digital button on an XboxController. */
    public enum Button {
        /** Left bumper. */
        kGreen(8),
        /** Right bumper. */
        kRed(6),
        /** Left stick. */
        kYelloq(9),
        /** Right stick. */
        kBlue(10),
        /** A. */
        kOrange(1),
        /** B. */
        kSelect(2),
        /** Y. */
        kY(3),
        /** Back. */
        kBack(6),
        /** Start. */
        kStart(7);

        /** Button value. */
        public final int value;

        Button(int value) {
            this.value = value;
        }

        /**
         * Get the human-friendly name of the button, matching the relevant methods.
         * This is done by
         * stripping the leading `k`, and if not a Bumper button append `Button`.
         *
         * <p>
         * Primarily used for automated unit tests.
         *
         * @return the human-friendly name of the button.
         */
        @Override
        public String toString() {
            var name = this.name().substring(1); // Remove leading `k`
            if (name.endsWith("Bumper")) {
                return name;
            }
            return name + "Button";
        }
    }

    /** Represents an axis on an GuitarHeroController. */
    public enum Axis {
        /** Left X. */
        kWammyBar(0);

        public final int value;

        Axis(int value) {
            this.value = value;
        }

        /**
         * Get the human-friendly name of the axis, matching the relevant methods. This
         * is done by
         * stripping the leading `k`, and if a trigger axis append `Axis`.
         *
         * <p>
         * Primarily used for automated unit tests.
         *
         * @return the human-friendly name of the axis.
         */
        @Override
        public String toString() {
            var name = this.name().substring(1); // Remove leading `k`
            if (name.endsWith("Trigger")) {
                return name + "Axis";
            }
            return name;
        }
    }

    public boolean getGreen() {
        System.out.println("Cameron was Here");
        return getRawButton(Button.kGreen.value);
    }

}
