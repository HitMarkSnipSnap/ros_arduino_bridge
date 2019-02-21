#ifdef USE_IMU

  #ifdef ADAFRUIT_9DOF_L3GD20H_LSM303
  /* This is the OLDER Adafruit 9-DOF IMU Breakout - L3GD20H + LSM303 that is now DISCONTINUED */
  
    #include <Wire.h>
    #include <Adafruit_Sensor.h>
    #include <Adafruit_LSM303_U.h>
    #include <Adafruit_L3GD20_U.h>
    #include <Adafruit_9DOF.h>
    
    /* Assign a unique ID to the sensors */
    Adafruit_9DOF                 dof   = Adafruit_9DOF();
    Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
    Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
    Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

    
    /* Update this with the correct SLP for accurate altitude measurements */
    float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
    
    void initIMU()
    {
      accel.begin();
      mag.begin();
      gyro.begin();
    }
  
    imuData readIMU() {
      imuData_s data;
    
      sensors_event_t accel_event;
      sensors_event_t mag_event;
      sensors_vec_t   orientation;
      sensors_event_t event;
      
      /* Calculate pitch and roll from the raw accelerometer data */
      accel.getEvent(&accel_event);
      data.ax = accel_event.acceleration.x;
      data.ay = accel_event.acceleration.y;
      data.az = accel_event.acceleration.z;

      gyro.getEvent(&event);
      data.gx = event.gyro.x;
      data.gy = event.gyro.y;
      data.gz = event.gyro.z;
/*
      if (dof.accelGetOrientation(&accel_event, &orientation))
      {
        data.roll = orientation.roll;
        data.pitch = orientation.pitch;
      }
*/

      /* Calculate the heading using the magnetometer */
      mag.getEvent(&mag_event);

/*
      if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation))
      {
        data.mz = orientation.heading;
      }
*/

      if (dof.fusionGetOrientation(&accel_event, &mag_event, &orientation))
      {
        data.roll = orientation.roll;
        data.pitch = orientation.pitch;
        data.ch = orientation.heading;
      }
      
      return data;
    }

  #elif defined ADAFRUIT_9DOF_FXOS8700_FXAS21002

  /* This is the NEWER Adafruit 9-DOF IMU Breakout  */
  /* https://learn.adafruit.com/nxp-precision-9dof-breakout/overview */
  
    #include <Wire.h>
    #include <Adafruit_Sensor.h>
    #include <Adafruit_FXAS21002C.h>
    #include <Adafruit_FXOS8700.h>
    #include <Mahony.h>
    #include <Madgwick.h>
   
    /* Assign a unique ID to the sensors */
    Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
    Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);
    
    /* Update this with the correct SLP for accurate altitude measurements */
    float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
    // Mag calibration values are calculated via ahrs_calibration.
    // These values must be determined for each board/environment.
    // See the image in this sketch folder for the values used
    // below.

    // Offsets applied to raw x/y/z mag values
    float mag_offsets[3]            = { 0.93F, -7.47F, -35.23F };

    // Soft iron error compensation matrix
    float mag_softiron_matrix[3][3] = { {  0.943,  0.011,  0.020 },
                                        {  0.022,  0.918, -0.008 },
                                        {  0.020, -0.008,  1.156 } };

    float mag_field_strength        = 50.23F;

    // Offsets applied to compensate for gyro zero-drift error for x/y/z
    float gyro_zero_offsets[3]      = { 0.0F, 0.0F, 0.0F };

    // Mahony is lighter weight as a filter and should be used
    // on slower systems
    Mahony filter;
    //Madgwick filter;
  
    void initIMU()
    {
      #ifdef DEBUG
         Serial.println("HELLO, start of initIMU() ");
      #endif
      accelmag.begin(ACCEL_RANGE_4G);
      #ifdef DEBUG
         Serial.println("HELLO, start of gyro() ");
      #endif
      gyro.begin();
      #ifdef DEBUG
         Serial.println("HELLO, end of initIMU() ");
      #endif
    }
  
    imuData readIMU() {
      imuData_s data;
    
      sensors_event_t accel_event;
      sensors_event_t mag_event;
      sensors_vec_t   orientation;
      sensors_event_t gyro_event;
      
      /* Calculate pitch and roll from the raw accelerometer data */
      accelmag.getEvent(&accel_event, &mag_event);
      gyro.getEvent(&gyro_event);

      // Apply mag offset compensation (base values in uTesla)
      float x = mag_event.magnetic.x - mag_offsets[0];
      float y = mag_event.magnetic.y - mag_offsets[1];
      float z = mag_event.magnetic.z - mag_offsets[2];

      // Apply mag soft iron error compensation
      float mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
      float my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
      float mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];

      // Apply gyro zero-rate error compensation
      float gx = gyro_event.gyro.x + gyro_zero_offsets[0];
      float gy = gyro_event.gyro.y + gyro_zero_offsets[1];
      float gz = gyro_event.gyro.z + gyro_zero_offsets[2];

      // The filter library expects gyro data in degrees/s, but adafruit sensor
      // uses rad/s so we need to convert them first (or adapt the filter lib
      // where they are being converted)
      float dgx = gx * 57.2958F;
      float dgy = gy * 57.2958F;
      float dgz = gz * 57.2958F;

      // Update the filter
      filter.update(dgx, dgy, dgz,
                    accel_event.acceleration.x, 
                    accel_event.acceleration.y, 
                    accel_event.acceleration.z,
                    mx, my, mz);

      data.ax = accel_event.acceleration.x;
      data.ay = accel_event.acceleration.y;
      data.az = accel_event.acceleration.z;

      data.gx = gx;
      data.gy = gy;
      data.gz = gz;

      data.roll = filter.getRoll();
      data.pitch = filter.getPitch();
      data.ch = filter.getYaw();
      
      return data;
    }


  #else
    #error A encoder driver must be selected!
  #endif
 
#endif
