package org.firstinspires.ftc.teamcode.Test;

import android.app.Activity;
import android.content.pm.PackageManager;
import android.os.Build;
import android.util.Log;
import android.view.ViewGroup;
import android.widget.RelativeLayout;
import android.widget.ScrollView;
import android.widget.TextView;

import com.revAmped.components.Button;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.Version;

import org.firstinspires.ftc.teamcode.R;

import java.io.BufferedReader;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.Set;

/**
 * Created by Parthiv Nair on 2/13/2018.
 */
@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "OverchargedDiag", group = "Individual")
public class OverchargedDiag extends OpMode {
    @Override
    public void init() {
        this.gamepad1.reset();
        this.gamepad1.setJoystickDeadzone(0.15f);
        this.gamepad2.reset();
        this.gamepad2.setJoystickDeadzone(0.15f);
        SelectNonLinear selectNonLinear = new SelectNonLinear(this);
        String lookForLineType = null;
        try {
            lookForLineType = selectNonLinear.selectLogTypeString();
        } catch (InterruptedException e) {
            e.printStackTrace();
            lookForLineType = " E Overcharged";
        }

        String log =  ReadLog("robotControllerLog.txt", lookForLineType);
        /**
         String log = "Log:\n";
         try {
            Process logcat = Runtime.getRuntime().exec("logcat -d");
            Thread.sleep(1000);
            BufferedReader reader = new BufferedReader(new InputStreamReader(logcat.getInputStream()));

            CharBuffer chars = CharBuffer.allocate(8192);
            reader.read(chars);
            log += chars.toString();
        } catch (IOException e) {
            Log.e(TAG_D, "Cannot start logcat monitor", e);
            log += e.getLocalizedMessage();
        } catch (InterruptedException e) {
            return;
        }

        if (log.equals("Log:\n")) {
            FileInputStream logcatFile = null;
            try {
                File file = new File("/sdcard/com.qualcomm.ftcrobotcontroller.logcat");
                logcatFile = new FileInputStream(file);
                InputStreamReader reader = new InputStreamReader(logcatFile);

                char[] chars = new char[(int) file.length()];
                reader.read(chars);
                log += new String(chars);
            } catch (IOException e) {
                Log.e(TAG_D, e.getLocalizedMessage(), e);
            }
        }
         **/
        String board = "Board: " + Build.BOARD +'\n';
        String brand = "Brand: " + Build.BRAND +'\n';
        String model = "Model No: " + Build.MODEL +'\n';
        String serial = "RC Serial: " + Build.SERIAL +'\n';
        String version = "Android Version: " + Build.VERSION.RELEASE +'\n';

        String rcVersion;
        try {
            rcVersion = "RC Version: " + hardwareMap.appContext.getPackageManager().
                    getPackageInfo(this.getClass().getPackage().getName(), 0).versionName +'\n';
        } catch (PackageManager.NameNotFoundException e) {
            e.printStackTrace();
            rcVersion = e.getLocalizedMessage() + '\n';
        }

        String rcLib = "RC Library: " + Version.getLibraryVersion() +'\n';

        StringBuilder deviceLogBuilder = new StringBuilder("Devices: \n");
        deviceLogBuilder.append((new DeviceMap<>(hardwareMap.dcMotorController)).details());
        deviceLogBuilder.append((new DeviceMap<>(hardwareMap.dcMotor)).details());
        deviceLogBuilder.append((new DeviceMap<>(hardwareMap.servoController)).details( ));
        deviceLogBuilder.append((new DeviceMap<>(hardwareMap.servo)).details());
        deviceLogBuilder.append((new DeviceMap<>(hardwareMap.legacyModule)).details());
        deviceLogBuilder.append((new DeviceMap<>(hardwareMap.deviceInterfaceModule)).details());
        deviceLogBuilder.append((new DeviceMap<>(hardwareMap.analogInput)).details());
        deviceLogBuilder.append((new DeviceMap<>(hardwareMap.digitalChannel)).details() );
        deviceLogBuilder.append((new DeviceMap<>(hardwareMap.pwmOutput)).details());
        deviceLogBuilder.append((new DeviceMap<>(hardwareMap.i2cDevice)).details());
        deviceLogBuilder.append((new DeviceMap<>(hardwareMap.analogOutput)).details());
        deviceLogBuilder.append((new DeviceMap<>(hardwareMap.led)).details());

        StringBuilder result = new StringBuilder(this.getClass().getSimpleName() + '\n');
        result.append(board).append(brand).append(model).append(serial).append(version)
                .append(rcVersion).append(rcLib).append(deviceLogBuilder).append(log).trimToSize();

        String msg = result.toString();
        Log.i(this.getClass().getSimpleName(), msg);
        RobotLog.setGlobalErrorMsg(msg.substring(0, msg.length() < 256 ? msg.length() : 255));

        final RelativeLayout layout = (RelativeLayout)
                ((Activity) hardwareMap.appContext).findViewById(R.id.RelativeLayout);
        final String message = msg;
        layout.post(new Runnable() {
            @Override
            public void run() {
                TextView child = new TextView(hardwareMap.appContext);
                child.setText(message);
                RelativeLayout.LayoutParams layoutParams = new RelativeLayout.LayoutParams(
                        ViewGroup.LayoutParams.WRAP_CONTENT, ViewGroup.LayoutParams.WRAP_CONTENT);
                layoutParams.addRule(RelativeLayout.BELOW, R.id.textErrorMessage);
                ScrollView scrollView = new ScrollView(hardwareMap.appContext);
                scrollView.addView(child);
                layout.addView(scrollView, layoutParams);
            }
        });
    }

    private String ReadLog(String fileName, String lookForLineType)
    {
        String lineType;
        int strn = lookForLineType.length();
        // Open the file
        FileInputStream fstream = null;
        try {
            fstream = new FileInputStream(fileName);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
            return "Could not open file " + fileName + " Exception: " + e.getMessage();
        }

        /**
         *
        try (BufferedReader br = new BufferedReader(new FileReader(file))) {
            String line;
            while ((line = br.readLine()) != null) {
                // process the line.
            }
        }
         */
        StringBuilder sb = new StringBuilder();
        BufferedReader br = new BufferedReader(new InputStreamReader(fstream));
        String strLine;

        //Read File Line By Line
        try {
            while ((strLine = br.readLine()) != null)   {
                lineType = strLine.substring(30, 30+strn);
                if (Objects.equals(lineType, lookForLineType)) {
                    sb.append(strLine);
                    sb.append("\n");
                }
            }
            br.close();
        } catch (IOException e) {
            e.printStackTrace();
            sb.append("IOException reading file ");
            sb.append(fileName);
            sb.append("\n");
            sb.append(e.getMessage());
            sb.append("\n");
        }
        return sb.toString();
    }
    @Override
    public void loop() {

    }
}

class DeviceMap<K, T extends HardwareDevice> extends HashMap<String, T> {
    public DeviceMap(HardwareMap.DeviceMapping<T> deviceMapping) {
        super(deviceMapping.size());
        buildFromDeviceMapping(deviceMapping);
    }

    public DeviceMap<K, T> buildFromDeviceMapping(HardwareMap.DeviceMapping<T > deviceMapping) {
        Set<Entry<String, T>> entries = deviceMapping.entrySet();
        for (Entry<String, T> device : entries) {
            super.put(device.getKey(), device.getValue());
        }
        return this;
    }

    public String details() {
        StringBuilder stringBuilder = new StringBuilder();

        for (Map.Entry<String, T> entry : entrySet()) {
            T value = entry.getValue();
            stringBuilder.append(value.getClass().getSimpleName()).append(" ")
                    .append(entry.getKey()).append(" ")
                    .append(value.toString()).append('\n');
        }
        return stringBuilder.toString();
    }
}

/**
 * Make selection within the OpMode
 * Selects the type of log information to display
 */
class SelectNonLinear
{
    private OpMode op;

    public SelectNonLinear(OpMode op) {
        this.op = op;
    }
    /**
     * Enumeration for type of log
     * The log statements are of type Error, Warning, Information and Verbose
     */
    public enum LogType {
        ALL_ERROR,
        ALL_WARNING,
        ALL_INFORMATION,
        ALL_VERBOSE,
        OVERCHARGED_ERROR,
        OVERCHARGED_WARNING,
        OVERCHARGED_INFORMATION,
        OVERCHARGED_VERBOSE;
    }

    private String[] getLogTypeNames() {
        return Arrays.toString(LogType.values()).replaceAll("^.|.$", "").split(", ");
    }

    /**
     * Select the log display type string (E, W, I, V ...)
     * @return true for red, false for blue
     * @throws InterruptedException
     */
    public String selectLogTypeString()
            throws InterruptedException {
        LogType logType = selectLogType();
        String lookForLineType;
        switch (logType) {
            case ALL_WARNING:
                lookForLineType = new String(" W ");
                break;
            case ALL_INFORMATION:
                lookForLineType = new String(" I ");
                break;
            case ALL_VERBOSE:
                lookForLineType = new String(" V ");
                break;
            case OVERCHARGED_ERROR:
                lookForLineType = new String(" E Overcharged");
                break;
            case OVERCHARGED_WARNING:
                lookForLineType = new String(" W Overcharged");
                break;
            case OVERCHARGED_INFORMATION:
                lookForLineType = new String(" I Overcharged");
                break;
            case OVERCHARGED_VERBOSE:
                lookForLineType = new String(" V Overcharged");
                break;
            default:
                lookForLineType = new String(" E ");
                break;
        }
        return lookForLineType;
    }

    /**
     * Select the log display type
     * @return true for red, false for blue
     * @throws InterruptedException
     */
    private LogType selectLogType ()
            throws InterruptedException {

        String[] types = getLogTypeNames();
        int index = select(types, "Log Type");
        return LogType.values()[index];
    }

    /**
     * Select from the given array
     * @return index from the given array starting 0
     * @throws InterruptedException
     */
    public int select (Object[] list,
                       String title)
            throws InterruptedException {
        int index = 0;

        op.gamepad1.reset();
        while (!Thread.currentThread().isInterrupted()) {
            long timeStamp = System.currentTimeMillis();
            if(op.gamepad1.b && Button.BTN_NEXT.canPress(timeStamp)) {
                index++;
                if(index >= list.length){
                    index = 0;
                }
            } else if(op.gamepad1.x && Button.BTN_PREV.canPress(timeStamp)) {
                index--;
                if(index < 0){
                    index = list.length - 1;
                }
            }
            else if (op.gamepad1.start && Button.BTN_START.canPress(timeStamp)) {
                break;
            }

            op.telemetry.addData(title, list[index]);
            op.telemetry.addData("Select", "Next: B  Prev: X");
            op.telemetry.addData("Confirm", "Start");
            op.telemetry.update();

            Thread.yield();
        }

        RobotLog.i("Select: " + title + "=" + list[index]);
        Thread.yield();
        return index;
    }

    /**
     * Confirm selection
     * @param title item to be confirmed
     * @param def default value
     * @return true to confirm
     */
    public boolean confirm (String title,
                            boolean def)
            throws InterruptedException {

        String[] options;
        if (def) {
            options = new String[]{"Yes", "No"};
        } else {
            options = new String[]{"No", "Yes"};
        }

        int index = select(options,
                title);
        return index == (def ? 0 : 1);
    }
}
