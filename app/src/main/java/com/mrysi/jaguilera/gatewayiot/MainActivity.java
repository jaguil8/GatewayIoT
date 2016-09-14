package com.mrysi.jaguilera.gatewayiot;

import android.Manifest;
import android.app.Activity;
import android.content.pm.PackageManager;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;
import com.orbotix.ConvenienceRobot;
import com.orbotix.Ollie;
import com.orbotix.async.DeviceSensorAsyncMessage;
import com.orbotix.common.DiscoveryException;
import com.orbotix.common.ResponseListener;
import com.orbotix.common.Robot;
import com.orbotix.common.internal.AsyncMessage;
import com.orbotix.common.internal.DeviceResponse;
import com.orbotix.common.sensor.AccelerometerData;
import com.orbotix.common.sensor.AttitudeSensor;
import com.orbotix.common.sensor.BackEMFSensor;
import com.orbotix.common.sensor.DeviceSensorsData;
import com.orbotix.common.sensor.GyroData;
import com.orbotix.common.sensor.QuaternionSensor;
import com.orbotix.common.sensor.SensorFlag;
import com.orbotix.DualStackDiscoveryAgent;
import com.orbotix.le.RobotLE;
import com.orbotix.common.RobotChangedStateListener;
import com.orbotix.subsystem.SensorControl;
import com.pubnub.api.*;


import org.json.JSONObject;

import java.util.ArrayList;
import java.util.List;

public class MainActivity extends Activity implements View.OnClickListener, RobotChangedStateListener,
        ResponseListener {



    private ConvenienceRobot mRobot;

    private static final int REQUEST_CODE_LOCATION_PERMISSION = 42;
    private static final float ROBOT_VELOCITY = 0.6f;

    private DualStackDiscoveryAgent mDiscoveryAgent;

    private Button mBtn0;
    private Button mBtn90;
    private Button mBtn180;
    private Button mBtn270;
    private Button mBtnStop;
    private TextView mAccelX;
    private TextView mAccelY;
    private TextView mAccelZ;
    private TextView mYawValue;
    private TextView mRollValue;
    private TextView mPitchValue;
    private TextView mQ0Value;
    private TextView mQ1Value;
    private TextView mQ2Value;
    private TextView mQ3Value;
    private TextView mGyroX;
    private TextView mGyroY;
    private TextView mGyroZ;
    private TextView mLeftMotor;
    private TextView mRightMotor;

    private String subscribeKey = "demo";
    private String publishKey = "demo";
    Pubnub pubnub = new Pubnub(publishKey,subscribeKey);

    @Override
    protected void onCreate( Bundle savedInstanceState ) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        initViews();

        mDiscoveryAgent = new DualStackDiscoveryAgent();
        mDiscoveryAgent.addRobotStateListener(this);


        if( Build.VERSION.SDK_INT >= Build.VERSION_CODES.M ) {
            int hasLocationPermission = checkSelfPermission( Manifest.permission.ACCESS_COARSE_LOCATION );
            if( hasLocationPermission != PackageManager.PERMISSION_GRANTED ) {
                Log.e( "Sphero", "Permiso de Ubicación no ha sido autorizado." );
                List<String> permissions = new ArrayList<String>();
                permissions.add( Manifest.permission.ACCESS_COARSE_LOCATION);
                requestPermissions(permissions.toArray(new String[permissions.size()] ), REQUEST_CODE_LOCATION_PERMISSION );
            } else {
                Log.d( "Sphero", "Permiso de Ubicación ha sido autorizado" );
            }
        }

        try {
            pubnub.subscribe("mrysi-jrap", new Callback() {
                @Override
                public void connectCallback(String channel, Object message) {
                    System.out.println("SUBSCRIBE : CONECTADO al canal:" + channel
                            + " : " + message.getClass() + " : "
                            + message.toString());
                }

                @Override
                public void disconnectCallback(String channel, Object message) {
                    System.out.println("SUBSCRIBE : DESCONECTADO del canal:" + channel
                            + " : " + message.getClass() + " : "
                            + message.toString());
                }

                @Override
                public void reconnectCallback(String channel, Object message) {
                    System.out.println("SUBSCRIBE : RECONECTADO al canal:" + channel
                            + " : " + message.getClass() + " : "
                            + message.toString());
                }

                @Override
                public void successCallback(String channel, Object message) {
                    System.out.println("SUBSCRIBE : " + channel
                            + " : " + message.getClass() + " : "
                            + message.toString());
                }

                @Override
                public void errorCallback(String channel, PubnubError error) {
                    System.out.println("SUBSCRIBE : ERROR en el canal:" + channel
                            + " : " + error.toString());
                }
            });
        } catch (PubnubException e) {
            e.printStackTrace();
        }
    }

    private void initViews() {
        mBtn0 = (Button) findViewById( R.id.btn_0 );
        mBtn90 = (Button) findViewById( R.id.btn_90 );
        mBtn180 = (Button) findViewById( R.id.btn_180 );
        mBtn270 = (Button) findViewById( R.id.btn_270 );
        mBtnStop = (Button) findViewById( R.id.btn_alto );

        mBtn0.setOnClickListener( this );
        mBtn90.setOnClickListener( this );
        mBtn180.setOnClickListener( this );
        mBtn270.setOnClickListener( this );
        mBtnStop.setOnClickListener( this );

        mAccelX = (TextView) findViewById( R.id.accel_x );
        mAccelY = (TextView) findViewById( R.id.accel_y );
        mAccelZ = (TextView) findViewById( R.id.accel_z );

        mRollValue = (TextView) findViewById( R.id.value_roll );
        mPitchValue = (TextView) findViewById( R.id.value_pitch );
        mYawValue = (TextView) findViewById( R.id.value_yaw );

        mQ0Value = (TextView) findViewById( R.id.value_q0 );
        mQ1Value = (TextView) findViewById( R.id.value_q1 );
        mQ2Value = (TextView) findViewById( R.id.value_q2 );
        mQ3Value = (TextView) findViewById( R.id.value_q3 );

        mGyroX = (TextView) findViewById( R.id.gyroscope_x );
        mGyroY = (TextView) findViewById( R.id.gyroscope_y );
        mGyroZ = (TextView) findViewById( R.id.gyroscope_z );

        mLeftMotor = (TextView) findViewById( R.id.left_motor );
        mRightMotor = (TextView) findViewById( R.id.right_motor );
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, String[] permissions, int[] grantResults) {
        switch ( requestCode ) {
            case REQUEST_CODE_LOCATION_PERMISSION: {
                for( int i = 0; i < permissions.length; i++ ) {
                    if( grantResults[i] == PackageManager.PERMISSION_GRANTED ) {
                        startDiscovery();
                        Log.d( "Permisos", "Permisos Autorizado: " + permissions[i] );
                    } else if( grantResults[i] == PackageManager.PERMISSION_DENIED ) {
                        Log.d( "Permisos", "Permiso Denegado: " + permissions[i] );
                    }
                }
            }
            break;
            default: {
                super.onRequestPermissionsResult(requestCode, permissions, grantResults);
            }
        }
    }

    //Enciende o apaga el LED del Robot cada dos segundos.
    private void parpadeo( final boolean ledEncendido ) {
        //Si no hay conexión con el robot se interrumpe la ejecución del método.
        if( mRobot == null )
            return;

        //Si el led está encendido se apaga.
        if( ledEncendido ) {
            mRobot.setLed( 0.0f, 0.0f, 0.0f );
        } else {
        //En caso contrario el led se enciende en color azul.
            mRobot.setLed( 0.0f, 0.0f, 1.0f );
        }

        //Se agrega un retardo entre cada cambio de estado del led.
        final Handler handler = new Handler();
        handler.postDelayed(new Runnable() {
            public void run() {
                parpadeo(!ledEncendido);
            }
        }, 2000);
    }

    @Override
    protected void onStart() {
        super.onStart();

        //Se verifica que se tengan los permisos para manejar la conexión Bluetooth.
        if( Build.VERSION.SDK_INT < Build.VERSION_CODES.M
                || checkSelfPermission( Manifest.permission.ACCESS_COARSE_LOCATION ) == PackageManager.PERMISSION_GRANTED ) {
            startDiscovery();
        }
    }

    private void startDiscovery() {
        //Si el agente de descubrimiento no se encuentra ya buscando robots, se inicia la busqueda.
        if( !mDiscoveryAgent.isDiscovering() ) {
            try {
                mDiscoveryAgent.startDiscovery(getApplicationContext());
            } catch (DiscoveryException e) {
                Log.e("Sphero", "Excepción de Descubrimiento: " + e.getMessage());
            }
        }
    }

    @Override
    protected void onStop() {
        //Si el Agente de Descubrimiento está en modo de busqueda, se detiene.
        if( mDiscoveryAgent.isDiscovering() ) {
            mDiscoveryAgent.stopDiscovery();
        }

        //Si un robot está conectado, se desconecta.
        if( mRobot != null ) {
            mRobot.sleep();
            mRobot = null;
        }

        super.onStop();
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        mDiscoveryAgent.addRobotStateListener(null);
    }

    @Override
    public void onClick(View v) {
        //Si el objeto mRobot es nulo entonces no hay robot conectado y no se tiene que realizar nada.
        if( mRobot == null ) {
            return;
        }

        /*
            Cuando un botón de movimiento es presionado, se manda el comando al robot para moverse
            en dicha dirección. Todas las direcciones están basadas en la posición del robot al
            momento de conectarse.
         */
        switch( v.getId() ) {
            case R.id.btn_0: {
                //Adelante
                mRobot.drive( 0.0f, ROBOT_VELOCITY );
                break;
            }
            case R.id.btn_90: {
                //A la derecha.
                mRobot.drive( 90.0f, ROBOT_VELOCITY );
                break;
            }
            case R.id.btn_180: {
                //Reversa
                mRobot.drive( 180.0f, ROBOT_VELOCITY );
                break;
            }
            case R.id.btn_270: {
                //A la izquierda
                mRobot.drive( 270.0f, ROBOT_VELOCITY );
                break;
            }
            case R.id.btn_alto: {
                //Detiene el robot.
                mRobot.stop();
                break;
            }
        }
    }

    @Override
    public void handleRobotChangedState( Robot robot, RobotChangedStateNotificationType type ) {
        switch( type ) {
            case Online: {

                //Se notifica al robot para indicar en cuales sensores estamos interesados.
                long sensorFlag = SensorFlag.QUATERNION.longValue()
                        | SensorFlag.ACCELEROMETER_NORMALIZED.longValue()
                        | SensorFlag.GYRO_NORMALIZED.longValue()
                        | SensorFlag.MOTOR_BACKEMF_NORMALIZED.longValue()
                        | SensorFlag.ATTITUDE.longValue();


                // Bluetooth LE (Ollie)
                if (robot instanceof RobotLE) {
                    ((RobotLE) robot).setDeveloperMode(true);
                }

                //Se graba el robot como un nuevo Robot Ollie para tener acceso a métodos específicos
                // del dispositivo.
                mRobot = new Ollie( robot );

                //Se deshabilita la estabilización del robot para permitir mayor amplitud de movimientos.
                mRobot.enableStabilization(false);

                //Se habilitan sensores basado en la notificación definida arriba y se establece que el envío de los datos 10
                // veces por segundo.
                mRobot.enableSensors( sensorFlag, SensorControl.StreamingRate.STREAMING_RATE10 );

                //Se agrega un Listener(escucha) para las respuestas del robot.
                mRobot.addResponseListener(this);

                //El led del robot enciende y se apaga mientras la conexión se encuentra activa.
                parpadeo( false );

                break;
            }
        }
    }

    @Override
    public void handleResponse(DeviceResponse response, Robot robot) {

    }

    @Override
    public void handleStringResponse(String stringResponse, Robot robot) {

    }

    @Override
    public void handleAsyncMessage(AsyncMessage asyncMessage, Robot robot) {
        if( asyncMessage == null )
            return;

        //Verifica el tipo de asyncMessage para ver si es de tipo DeviceSensor.
        if( asyncMessage instanceof DeviceSensorAsyncMessage) {
            DeviceSensorAsyncMessage message = (DeviceSensorAsyncMessage) asyncMessage;

            System.out.println("Este es el mensaje del robot:" + message);
            

            if( message.getAsyncData() == null
                    || message.getAsyncData().isEmpty()
                    || message.getAsyncData().get( 0 ) == null )
                return;

            JSONObject mensajeJson = new JSONObject();

            //Extrae el DeviceSensorData del mensaje asíncrono.
            DeviceSensorsData data = message.getAsyncData().get( 0 );

            //Extrae los datos del acelerómetro del objeto data.
            displayAccelerometer(data.getAccelerometerData());

            //Extrae los datos de attitude (Dirección, alabeo y elevación) del objeto data.
            displayAttitude(data.getAttitudeData());

            //Extrae los datos de los cuaterniones del objeto data.
            displayQuaterions( data.getQuaternion() );

            //Extrae los datos de Fuerza Electromotriz Inversa del objeto data.
            displayBackEMF( data.getBackEMFData().getEMFFiltered() );

            //Extrae los datos del giroscopio del objeto data.
            displayGyroscope( data.getGyroData() );


        }
    }

    private void displayBackEMF( BackEMFSensor sensor ) {
        if( sensor == null )
            return;

        mLeftMotor.setText( String.valueOf( sensor.leftMotorValue ) );
        mRightMotor.setText( String.valueOf( sensor.rightMotorValue ) );
    }

    private void displayGyroscope( GyroData data ) {
        mGyroX.setText( String.valueOf( data.getRotationRateFiltered().x ) );
        mGyroY.setText( String.valueOf( data.getRotationRateFiltered().y ) );
        mGyroZ.setText( String.valueOf( data.getRotationRateFiltered().z ) );
    }

    private void displayAccelerometer( AccelerometerData accelerometer ) {
        if( accelerometer == null || accelerometer.getFilteredAcceleration() == null ) {
            return;
        }

        //Despliega las lecturas de los componentes X, Y y Z del acelerómetro
        mAccelX.setText( String.format( "%.4f", accelerometer.getFilteredAcceleration().x ) );
        mAccelY.setText( String.format( "%.4f", accelerometer.getFilteredAcceleration().y ) );
        mAccelZ.setText( String.format( "%.4f", accelerometer.getFilteredAcceleration().z ) );
    }

    private void displayAttitude( AttitudeSensor attitude ) {
        if( attitude == null )
            return;

        //Despliega, la elevación, dirección y alabeo de attitude.
        mRollValue.setText( String.format( "%3d", attitude.roll ) + "°" );
        mPitchValue.setText( String.format( "%3d", attitude.pitch ) + "°" );
        mYawValue.setText( String.format( "%3d", attitude.yaw) + "°" );
    }

    private void displayQuaterions( QuaternionSensor quaternion ) {
        if( quaternion == null )
            return;

        //Despliega los datos de los 4 cuaterniones.
        mQ0Value.setText( String.format( "%.5f", quaternion.getQ0() ) );
        mQ1Value.setText( String.format( "%.5f", quaternion.getQ1()) );
        mQ2Value.setText( String.format( "%.5f", quaternion.getQ2()) );
        mQ3Value.setText( String.format( "%.5f", quaternion.getQ3()) );

    }
}
