package org.team9140.frc2025.helpers;

import edu.wpi.first.math.geometry.Pose2d;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.team9140.frc2025.Constants;

import java.util.TreeMap;

public class AutoAimingTest {
    @Test
    @DisplayName("Generate reef stick poses for Choreo")
    public void GenerateChoreoPoses() {
        AutoAiming.ReefFaces[] faces = AutoAiming.ReefFaces.values();
        TreeMap<String, Pose2d> variables = new TreeMap<>();
        for (int i = 0; i < faces.length / 2; i++) {
            AutoAiming.ReefFaces face = faces[i];
            Constants.ElevatorSetbacks[] setbacks = {
                    Constants.ElevatorSetbacks.L3,
                    Constants.ElevatorSetbacks.L4
            };
            for (Constants.ElevatorSetbacks setback : setbacks) {
                variables.put(face.toString().charAt(1) + "_" + setback, face.getCenter(setback).plus(Constants.AutoAlign.HORIZONTAL_BRANCH_DISTANCE_FROM_CENTER.inverse()));
                variables.put(face.toString().charAt(0) + "_" + setback, face.getCenter(setback).plus(Constants.AutoAlign.HORIZONTAL_BRANCH_DISTANCE_FROM_CENTER));
            }
        }

        StringBuilder json = new StringBuilder();
        variables.forEach((name, pose) -> {
            json.append(String.format("\"%s\":{\n" +
                    "      \"x\":{\n" +
                    "       \"exp\":\"%f m\",\n" +
                    "       \"val\":%f\n" +
                    "      },\n" +
                    "      \"y\":{\n" +
                    "       \"exp\":\"%f m\",\n" +
                    "       \"val\":%f\n" +
                    "      },\n" +
                    "      \"heading\":{\n" +
                    "       \"exp\":\"%f rad\",\n" +
                    "       \"val\":%f\n" +
                    "      }\n" +
                    "},\n", name, pose.getX(), pose.getX(), pose.getY(), pose.getY(), pose.getRotation().getRadians(), pose.getRotation().getRadians()));
        });

        System.out.println(json);
    }
}
