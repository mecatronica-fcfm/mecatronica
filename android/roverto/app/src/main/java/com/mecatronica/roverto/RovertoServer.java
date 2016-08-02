package com.mecatronica.roverto;

import android.util.Log;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.IOException;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.net.UnknownHostException;

/**
 * Created by rodrigo on 29-07-16.
 */
public class RovertoServer {
    // TAG is used to debug in Android console
    private static final String TAG = "roverto";

    private Socket server_;
    private BufferedInputStream serverI_;
    private BufferedOutputStream serverO_;
    // Server data
    private static final int SERVER_PORT = 3500;
    private static final String SERVER_NAME = "intermedio.ddns.net";

    boolean connected;

    // Max value
    public static final int MIN_VALUE = 0;
    public static final int MAX_VALUE = 255;

    public boolean isConnected()
    {
        return connected;
    }

    public void connect()
    {
        try {
            server_ = new Socket();
            server_.connect(new InetSocketAddress(SERVER_NAME, SERVER_PORT));
            serverI_ = new BufferedInputStream(server_.getInputStream());
            serverO_ = new BufferedOutputStream(server_.getOutputStream());
            connected = true;
        }
        catch (UnknownHostException e)
        {
            Log.e(TAG, "Fail to find server");
            e.printStackTrace();
        }
        catch (IOException e)
        {
            Log.d(TAG, "Fail init socket");
            e.printStackTrace();
        }
        finally{
            server_ = null;
            connected = false;
        }
    }

    public void write(int data)
    {
        // Saturation
        data = data<MIN_VALUE ? MIN_VALUE : (data>MAX_VALUE ? MAX_VALUE : data);
        try {
            serverO_.write(data);
        }
        catch (IOException e)
        {
            Log.d(TAG, "Error on write request");
        }
    }

    public void write(RovertoStatus st){
        try {
            serverO_.write(255);
            serverO_.write(255);
            int sum = 0;
            int data;
            for (int i = 0; i < RovertoStatus.MESSAGE_SIZE; i++) {
                data = st.data[i];
                sum += data;
                serverO_.write(data);
            }
            serverO_.write(sum%255);
        }
        catch (IOException e)
        {
            Log.d(TAG, "Error on write request");
        }
    }

    public int read()
    {
        try
        {
            if (serverI_.available()==0) return -1;
            return serverI_.read();
        }
        catch (IOException e)
        {
            Log.d(TAG, "Error on read request");
        }
        return -1;
    }

    public int available()
    {
        try {
            return serverI_.available();
        } catch (IOException e) {
            e.printStackTrace();
        }
        return -1;
    }

    public void flush()
    {
        try {
            serverO_.flush();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
