package com.mecatronica.arduinoadk;

import android.util.Log;

import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;

/**
 * Class for basic byte-based communication through USB
 */
public class SerialUsbCommunication implements UsbCommunicationInterface {
        // Debug flag
        public static final boolean D = true;
        // TAG is used to debug in Android console
        private static final String TAG = "ArduinoADK";
        private static final String ACTION_USB_PERMISSION = "com.mecatronica.arduinoadk.USB_PERMISSION";

        private byte[] buffer = new byte[1024];
        private ByteBuffer buf;
        private boolean running;

        private FileInputStream mInputStream;
        private FileOutputStream mOutputStream;

        SerialUsbCommunication(FileInputStream input, FileOutputStream output)
        {
            this.running = true;
            this.buf = ByteBuffer.allocate(1024);

            this.mInputStream = input;
            this.mOutputStream = output;
        }

        public void run()
        {
            while (running)
            {
                try
                {
                    // Read data
                    int bytes = mInputStream.read(buffer);
                    Log.d(TAG, "Device read " + bytes);
                    // Sleep for 20 ms
                    Thread.sleep(20);
                }
                catch (Exception ignore)
                {
                    Log.e(TAG, "Error reading accesory");
                }

            }
        }

        public byte get()
        {
            return buffer[0];
        }

        public void send(byte data) throws IOException
        {
                mOutputStream.write(data);
        }


        public void cancel()
        {
            running = false;
        }

}
