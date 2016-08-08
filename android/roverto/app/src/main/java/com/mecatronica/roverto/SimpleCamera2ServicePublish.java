package com.mecatronica.roverto;

import android.app.Service;
import android.content.Intent;
import android.graphics.Bitmap;
import android.graphics.ImageFormat;
import android.graphics.PixelFormat;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CaptureRequest;
import android.media.Image;
import android.media.ImageReader;
import android.os.Binder;
import android.os.IBinder;
import android.support.annotation.NonNull;
import android.util.Log;

import java.nio.ByteBuffer;
import java.util.Arrays;

/*
Reference: http://stackoverflow.com/questions/28003186/capture-picture-without-preview-using-camera2-api

Problem
   1.  BufferQueue has been abandoned  from ImageCapture
 */
public class SimpleCamera2ServicePublish extends Service {
    // Interface for clients that bind
    private final IBinder mBinder = new LocalBinder();
    // Indicates how to behave if the service is killed
    int mStartMode;

    protected static final String TAG = "roverto";
    protected static final int CAMERACHOICE = CameraCharacteristics.LENS_FACING_BACK;
    protected CameraDevice cameraDevice;
    protected CameraCaptureSession session;
    protected ImageReader imageReader;

    private final static int HEIGHT = 144;
    private final static int WIDTH = 176;
    private int[] mImage = new int[HEIGHT*WIDTH];

    protected CameraDevice.StateCallback cameraStateCallback = new CameraDevice.StateCallback() {
        @Override
        public void onOpened(@NonNull CameraDevice camera) {
            Log.i(TAG, "CameraDevice.StateCallback onOpened");
            cameraDevice = camera;
            actOnReadyCameraDevice();
        }

        @Override
        public void onDisconnected(@NonNull CameraDevice camera) {
            Log.w(TAG, "CameraDevice.StateCallback onDisconnected");
        }

        @Override
        public void onError(@NonNull CameraDevice camera, int error) {
            Log.e(TAG, "CameraDevice.StateCallback onError " + error);
        }
    };

    protected CameraCaptureSession.StateCallback sessionStateCallback = new CameraCaptureSession.StateCallback() {
        @Override
        public void onConfigured(@NonNull CameraCaptureSession session) {
            Log.i(TAG, "CameraCaptureSession.StateCallback onConfigured");
            SimpleCamera2ServicePublish.this.session = session;
            try {
                session.setRepeatingRequest(createCaptureRequest(), null, null);
            } catch (CameraAccessException e){
                Log.e(TAG, e.getMessage());
            }
        }

        @Override
        public void onConfigureFailed(@NonNull CameraCaptureSession session) {}
    };

    protected ImageReader.OnImageAvailableListener onImageAvailableListener = new ImageReader.OnImageAvailableListener() {
        @Override
        public void onImageAvailable(ImageReader reader) {
            Image img = reader.acquireLatestImage();
            if (img != null) {
                Image.Plane[] planes = img.getPlanes();
                if (planes[0].getBuffer() == null) {
                    return;
                }
                int width = img.getWidth();
                int height = img.getHeight();
                int pixelStride = planes[0].getPixelStride();
                int rowStride = planes[0].getRowStride();
                int rowPadding = rowStride - pixelStride * width;

                int offset = 0;
                ByteBuffer buffer = planes[0].getBuffer();
                synchronized (mImage){
                    for (int i = 0; i < height; ++i) {
                        for (int j = 0; j < width; ++j) {
                            int pixel = 0;
                            pixel += (buffer.get(offset) & 0xff);     // R
                            pixel += (buffer.get(offset + 1) & 0xff); // G
                            pixel += (buffer.get(offset + 2) & 0xff); // B
                            pixel /= 3; // Monochrome data
                            mImage[i*height + j]= pixel;
                            offset += pixelStride;
                        }
                        offset += rowPadding;
                    }
                }
                img.close();
            }
        }
    };

    public void readyCamera()
    {
        CameraManager manager = (CameraManager) getSystemService(CAMERA_SERVICE);
        try {
            String pickedCamera = getCamera(manager);
            manager.openCamera(pickedCamera, cameraStateCallback, null);
            imageReader = ImageReader.newInstance(176, 144, PixelFormat.RGBA_8888, 2 /* images buffered */);
            imageReader.setOnImageAvailableListener(onImageAvailableListener, null);
            Log.i(TAG, "imageReader created");
        } catch (CameraAccessException e){
            Log.e(TAG, e.getMessage());
        } catch (SecurityException e){
            Log.e(TAG, e.getMessage());
        }
    }


    /**
     *  Return the Camera Id which matches the field CAMERACHOICE.
     */
    public String getCamera(CameraManager manager){
        try {
            for (String cameraId : manager.getCameraIdList()) {
                CameraCharacteristics characteristics = manager.getCameraCharacteristics(cameraId);
                int cOrientation = characteristics.get(CameraCharacteristics.LENS_FACING);
                if (cOrientation == CAMERACHOICE) {
                    return cameraId;
                }
            }
        } catch (CameraAccessException e){
            e.printStackTrace();
        }
        return null;
    }

    @Override
    public int onStartCommand(Intent intent, int flags, int startId) {
        Log.i(TAG, "onStartCommand flags " + flags + " startId " + startId);

        readyCamera();

        mStartMode = START_STICKY;
        return mStartMode;
    }

    public void actOnReadyCameraDevice()
    {
        try {
            cameraDevice.createCaptureSession(Arrays.asList(imageReader.getSurface()), sessionStateCallback, null);
        } catch (CameraAccessException e){
            Log.e(TAG, e.getMessage());
        }
    }

    @Override
    public void onDestroy() {
        try {
            session.abortCaptures();
        } catch (CameraAccessException e){
            Log.e(TAG, e.getMessage());
        }
        session.close();
        cameraDevice.close();
        imageReader.close();
    }

    /**
     *  Process image data as desired.
     */
    public int[] getImage(){
        int[] data = new int[WIDTH*HEIGHT];
        synchronized (mImage)
        {
            System.arraycopy(mImage,0,data,0, mImage.length);
        }
        return data;
    }

    protected CaptureRequest createCaptureRequest() {
        try {
            CaptureRequest.Builder builder = cameraDevice.createCaptureRequest(CameraDevice.TEMPLATE_RECORD);
            builder.addTarget(imageReader.getSurface());
            return builder.build();
        } catch (CameraAccessException e) {
            Log.e(TAG, e.getMessage());
            return null;
        }
    }

    /**
     * Class used for the client Binder.
     */
    public class LocalBinder extends Binder {
        SimpleCamera2ServicePublish getService() {
            Log.d(TAG, "Call getService");
            // Return this instance
            return SimpleCamera2ServicePublish.this;
        }
    }

    @Override
    public IBinder onBind(Intent intent) {
        return mBinder;
    }
}
