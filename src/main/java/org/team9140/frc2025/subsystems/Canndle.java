package org.team9140.frc2025.subsystems;

import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Canndle extends SubsystemBase {
    private static Canndle instance;

    public static final Color8Bit RED = new Color8Bit(255, 0, 0);
    public static final Color8Bit GREEN = new Color8Bit(0, 255, 0);
    public static final Color8Bit BLUE = new Color8Bit(0, 0, 255);
    public static final Color8Bit PINK = new Color8Bit(245, 110, 229);
    public static final Color8Bit ORANGE = new Color8Bit(255, 157, 0);
    public static final Color8Bit OFF = new Color8Bit(0, 0, 0);

    private CANdle candle;

    private Canndle() {
        this.candle = new CANdle(0, "moe");
    }

    public static Canndle getInstance() {
        return (instance == null) ? instance = new Canndle() : instance;
    }

    public Command setColor(Color8Bit color) {
        return this.runOnce(() -> this.candle.setLEDs(color.red, color.green, color.blue));
    }

    public Command solidAllianceColor() {
        return new ConditionalCommand(this.setColor(RED), this.setColor(BLUE),
                () -> DriverStation.getAlliance().isPresent()
                        && DriverStation.Alliance.Red.equals(DriverStation.getAlliance().get()));
    }

    public Color8Bit getAllianceColor() {
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)) {
            return BLUE;
        } else {
            return RED;
        }
    }

    public Command turnOff() {
        return this.setColor(new Color8Bit(0, 0, 0));
    }

    public Command blinkColorEndsAlliance(Color8Bit color, double wait, double timeout) {
        return this.changeColors(color, OFF, wait, timeout, getAllianceColor());
    }

    public Command blinkColorEndsOff(Color8Bit color, double wait, double timeout) {
        return this.changeColors(color, OFF, wait, timeout, OFF);
    }

    public Command blinkColorEndsColor(Color8Bit color, double wait, double timeout) {
        return this.changeColors(color, OFF, wait, timeout, color);
    }

    public Command switchTwoColorsEndsAlliance(Color8Bit firstColor, Color8Bit secondColor, double wait, double timeout) {
        return this.changeColors(firstColor, secondColor, wait, timeout, getAllianceColor());
    }

    public Command switchWithAllianceEndsAlliance(Color8Bit firstColor, Color8Bit secondColor, double wait, double timeout) {
        return this.changeColors(firstColor, getAllianceColor(), wait, timeout, getAllianceColor());
    }

    public Command switchWithAllianceEndsFirst(Color8Bit firstColor, Color8Bit secondColor, double wait, double timeout) {
        return this.changeColors(firstColor, getAllianceColor(), wait, timeout, firstColor);
    }

    public Command switchWithAllianceEndsOff(Color8Bit firstColor, Color8Bit secondColor, double wait, double timeout) {
        return this.changeColors(firstColor, getAllianceColor(), wait, timeout, OFF);
    }

    public Command switchTwoColorsEndsFirst(Color8Bit firstColor, Color8Bit secondColor, double wait, double timeout) {
        return this.changeColors(firstColor, secondColor, wait, timeout, firstColor);
    }

    public Command switchTwoColorsEndsSecond(Color8Bit firstColor, Color8Bit secondColor, double wait, double timeout) {
        return this.changeColors(firstColor, secondColor, wait, timeout, secondColor);
    }

    public Command switchTwoColorsEndsOff(Color8Bit firstColor, Color8Bit secondColor, double wait, double timeout) {
        return this.changeColors(firstColor, secondColor, wait, timeout, OFF);
    }

    public Command changeColors(Color8Bit firstColor, Color8Bit secondColor, double wait, double timeout, Color8Bit finalColor) {
        return this.setColor(firstColor).andThen(Commands.waitSeconds(wait)).andThen(setColor(secondColor)).andThen(Commands.waitSeconds(wait)).repeatedly().withTimeout(timeout).andThen(setColor(finalColor));

    }

    public Command flashColor(Color8Bit color, double holdTime) {
        return this.setColor(color).andThen(Commands.waitSeconds(holdTime)).andThen(this.turnOff());
    }
}



