package org.exponential.utility;

import android.os.Environment;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Reader;
import java.util.ArrayList;

public class FileReader {
    private static final String BASE_FOLDER_NAME = "FIRST";

    public static ArrayList<String> getLinesFromCSVFile(String filename) {
        ArrayList<String> lines = new ArrayList<String>();
        String directoryPath = Environment.getExternalStorageDirectory().getPath()+"/"+BASE_FOLDER_NAME;
        File directory = new File(directoryPath);
        //noinspection ResultOfMethodCallIgnored
        directory.mkdir();
        try(BufferedReader br = new BufferedReader(new java.io.FileReader(directoryPath+"/"+filename+".csv"))) {
            for(String line; (line = br.readLine()) != null; ) {
                lines.add(line);
            }
            // line is not visible here.
        }
        catch (Exception e) {
            e.printStackTrace();
        }
        return lines;
    }

}
