package com.revAmped.util;

import com.qualcomm.robotcore.util.RobotLog;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.ObjectInputStream;

/**
 * Created by zwang on 7/3/2016.
 */
public class ByteLoader {
    private ObjectInputStream oin = null;
    private String fileName;

    public ByteLoader(String fileName)
        throws IOException {
        this.fileName = fileName;

        File directoryPath = new File("/sdcard/FIRST/DataLogger");
        File filePath = new File(directoryPath, fileName + ".ser");

        directoryPath.mkdir();        // Make sure that the directory exists

        try {
            FileInputStream fin = new FileInputStream(filePath);
            oin = new ObjectInputStream(fin);
        } catch (IOException e) {
            RobotLog.e("Failed to open file " + filePath, e);
            throw e;
        }
    }

    public void close() {
        if (this.oin != null) {
            try {
                this.oin.close();
            } catch (IOException e) {
                // no opt
            }
        }
    }

    public void load(byte[] bytes) throws IOException {
        if (bytes == null || this.oin == null) {
            return;
        }
        try {
            this.oin.readFully(bytes);
        } catch (IOException e) {
            RobotLog.e("Failed to read file " + fileName, e);
            throw e;
        }
    }
}
