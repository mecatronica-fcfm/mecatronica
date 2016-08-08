package com.mecatronica.roverto;

import android.app.Service;
import android.content.Intent;
import android.os.Binder;
import android.os.IBinder;
import android.util.Log;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;

/**
 * Created by rodrigo on 01-08-16.
 */
public class RovertoSocketService extends Service {
    // Interface for clients that bind
    private final IBinder mBinder = new LocalBinder();
    // Indicates how to behave if the service is killed
    int mStartMode;

    // TAG is used to debug in Android console
    private static final String TAG = "roverto";

    // Socket
    private Socket mSocket;
    private InputStream mInputStream;
    private OutputStream mOutputStream;
    // Socket thread management
    SocketCommunication mSocketComm;
    // Communication Queues
    BlockingQueue<Integer> mReceivedQueue = new LinkedBlockingQueue<Integer>();
    BlockingQueue<Integer> mToSendQueue = new LinkedBlockingQueue<Integer>();
    // Min and max values
    public static final int MIN_VALUE = 0;
    public static final int MAX_VALUE = 255;
    // Server data
    private static final int SERVER_PORT = 3500;
    private static final String SERVER_NAME = "intermedio.ddns.net";

    @Override
    public int onStartCommand(Intent intent,int flags, int startId){
        super.onStartCommand(intent, flags, startId);
        Log.d(TAG, "Starting service");
        mStartMode = START_STICKY;
        return mStartMode;
    }

    @Override
    public void onCreate() {
        // The service is being created
        super.onCreate();
        Log.d(TAG, "Creating service");
        mSocketComm = new SocketCommunication();
        mSocketComm.start();
    }

    class SocketCommunication extends Thread {

        // Running flag
        boolean running;

        SocketCommunication(){
            running = false;
        }

        @Override
        public void run() {
            try {
                // Socket connection
                Log.e(TAG, "Connecting to socket.");
                mSocket = new Socket();
                mSocket.connect(new InetSocketAddress(SERVER_NAME, SERVER_PORT));

                try {
                    // Get streams
                    mInputStream = mSocket.getInputStream();
                    mOutputStream = mSocket.getOutputStream();
                    // Set running flag
                    running = true;
                    // Main loop
                    while (running) {
                        try {
                            // Read data from stream
                            if (mInputStream.available() > 0){
                                int raw_data = mInputStream.read();
                                if (raw_data > 0) {
                                    Integer data = new Integer(raw_data);
                                    mReceivedQueue.put(data);
                                    Log.d(TAG, "Data from socket: " + data.toString());
                                }
                            }
                            // Write data
                            Integer write_data = null;
                            try {
                                // Get data from queue
                                write_data = mToSendQueue.poll(1, TimeUnit.MILLISECONDS);
                                // Write de data in the stream
                                if (write_data != null) {
                                    //Log.d(TAG, "Write data to socket: " + write_data.toString());
                                    mOutputStream.write(write_data.intValue());
                                    mOutputStream.flush();
                                }
                            } catch (InterruptedException e) {
                                Log.d(TAG, "InterruptedException writing data");
                            }
                        } catch (Exception ignore) {
                        }
                    }

                }
                catch (IOException e) {
                    Log.e(TAG, "Error opening streams");
                }
            } catch (IOException e) {
                Log.e(TAG, "Error on socket connection");
                e.printStackTrace();
            }
        }

        public void cancel() {
            running = false;
        }
    }

    /*
     * Communication methods
     */
    public int read() {
        Integer data = null;
        try {
            data = mReceivedQueue.poll(1, TimeUnit.MILLISECONDS);
        } catch (InterruptedException e) {
            Log.d(TAG, "InterruptedException getting data");
        }
        return data.intValue();
    }

    public void write(int data) {
        // Saturation using min and max values
        data = data<MIN_VALUE ? MIN_VALUE : (data>MAX_VALUE ? MAX_VALUE : data);
        mToSendQueue.add(new Integer(data));
    }

    public void write(RovertoStatus st){
        this.write(255);
        this.write(255);
        int sum = 0;
        int data;
        for (int i = 0; i < RovertoStatus.MESSAGE_SIZE; i++) {
            data = st.data[i];
            sum += data;
            this.write(data);
        }
        this.write(sum%255);
    }

    public int available() {
        return mReceivedQueue.size();
    }



    @Override
    public void onDestroy() {
        super.onDestroy();
        // Stop thread
        mSocketComm.cancel();
        // Close input stream
        if (mInputStream != null){
            try {
                mInputStream.close();
            } catch (IOException e) {
                Log.e(TAG, "Error on input stream close");
                e.printStackTrace();
            }
        }
        // Close output stream
        if (mOutputStream != null){
            try {
                mOutputStream.close();
            } catch (IOException e) {
                Log.e(TAG, "Error on output stream close");
                e.printStackTrace();
            }
        }
        // Close mSocket
        if (mSocket != null){
            try {
                mSocket.close();
            } catch (IOException e) {
                Log.e(TAG, "Error on socket close");
                e.printStackTrace();
            }
        }
    }



    /**
     * Class used for the client Binder.
     */
    public class LocalBinder extends Binder {
        RovertoSocketService getService() {
            Log.d(TAG, "Call getService");
            // Return this instance
            return RovertoSocketService.this;
        }
    }

    @Override
    public IBinder onBind(Intent intent) {
        return mBinder;
    }


}