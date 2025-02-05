package org.team9140.frc2025;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

import org.team9140.frc2025.limelight.LimelightConstants;
import org.team9140.frc2025.limelight.LimelightConstantsFactory;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;

import static edu.wpi.first.units.Units.*;

public class Constants {
    public static class Drive {
        public static final LinearVelocity MIN_TRANSLATE_MPS = MetersPerSecond.of(0.01);
        public static final AngularVelocity MIN_ROTATE_RPS = DegreesPerSecond.of(1);

    }

    public static class Camera {
        public static final double kResolutionWidth = 1280;
        public static final double kResolutionHeight = 800;

        public static final String kPracticeBotMACAddress = ""; //limelight MAC address I think
        public static final boolean kPracticeBot = hasMacAddress(kPracticeBotMACAddress);

        public static final String kPracticeLLId = "back2024";
        public static final String kCompLLId = "B";

        public static final LimelightConstants kLimelightConstants = LimelightConstantsFactory.getConstantsForId(kPracticeLLId);
        public static final double kLimelightTransmissionTimeLatency = 0.0 / 1000.0; // seconds
        public static final double kImageCaptureLatency = 11.0; // milliseconds
        public static final double kLensHeight = 1.0;

        public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    }

    /**
     * Check if this system has a certain mac address in any network device.
     * @param mac_address Mac address to check.
     * @return true if some device with this mac address exists on this system.
     */
    public static boolean hasMacAddress(final String mac_address) {
        try {
            Enumeration<NetworkInterface> nwInterface = NetworkInterface.getNetworkInterfaces();
            while (nwInterface.hasMoreElements()) {
                NetworkInterface nis = nwInterface.nextElement();
                if (nis == null) {
                    continue;
                }
                StringBuilder device_mac_sb = new StringBuilder();
                System.out.println("hasMacAddress: NIS: " + nis.getDisplayName());
                byte[] mac = nis.getHardwareAddress();
                if (mac != null) {
                    for (int i = 0; i < mac.length; i++) {
                        device_mac_sb.append(String.format("%02X%s", mac[i], (i < mac.length - 1) ? ":" : ""));
                    }
                    String device_mac = device_mac_sb.toString();
                    System.out.println("hasMacAddress: NIS " + nis.getDisplayName() + " device_mac: " + device_mac);
                    if (mac_address.equals(device_mac)) {
                        System.out.println("hasMacAddress: ** Mac address match! " + device_mac);
                        return true;
                    }
                } else {
                    System.out.println("hasMacAddress: Address doesn't exist or is not accessible");
                }
            }

        } catch (SocketException e) {
            e.printStackTrace();
        }
        return false;
    }
}
