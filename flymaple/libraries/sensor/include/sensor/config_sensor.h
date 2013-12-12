#ifndef CONFIG_SENSOR_H
#define CONFIG_SENSOR_H

#define ACCEL_X_MIN ((float) -260)
#define ACCEL_X_MAX ((float) 285)
#define ACCEL_Y_MIN ((float) -282)
#define ACCEL_Y_MAX ((float) 264)
#define ACCEL_Z_MIN ((float) -345)
#define ACCEL_Z_MAX ((float) 235)

#define ACCEL_X_OFFSET ((ACCEL_X_MIN + ACCEL_X_MAX) / 2.0f)
#define ACCEL_Y_OFFSET ((ACCEL_Y_MIN + ACCEL_Y_MAX) / 2.0f)
#define ACCEL_Z_OFFSET ((ACCEL_Z_MIN + ACCEL_Z_MAX) / 2.0f)
#define ACCEL_X_SCALE (GRAVITY / (ACCEL_X_MAX - ACCEL_X_OFFSET))
#define ACCEL_Y_SCALE (GRAVITY / (ACCEL_Y_MAX - ACCEL_Y_OFFSET))
#define ACCEL_Z_SCALE (GRAVITY / (ACCEL_Z_MAX - ACCEL_Z_OFFSET))

#define ROLL    0
#define PITCH   1
#define YAW     2

#define GRAVITY 256.0f 
#define GYRO_GAIN 0.06957 // Same gain on all axes
#define G_SCALE 0.038281f

#define GYRO_SCALED_RAD(x) (x * TO_RAD(GYRO_GAIN))
#define ACC_SCALED_G(x) (x * G_SCALE)

#define ACCEL_X_MIN ((float) -260)
#define ACCEL_X_MAX ((float) 285)
#define ACCEL_Y_MIN ((float) -282)
#define ACCEL_Y_MAX ((float) 264)
#define ACCEL_Z_MIN ((float) -345)
#define ACCEL_Z_MAX ((float) 235)

#define ACCEL_X_OFFSET ((ACCEL_X_MIN + ACCEL_X_MAX) / 2.0f)
#define ACCEL_Y_OFFSET ((ACCEL_Y_MIN + ACCEL_Y_MAX) / 2.0f)
#define ACCEL_Z_OFFSET ((ACCEL_Z_MIN + ACCEL_Z_MAX) / 2.0f)
#define ACCEL_X_SCALE (GRAVITY / (ACCEL_X_MAX - ACCEL_X_OFFSET))
#define ACCEL_Y_SCALE (GRAVITY / (ACCEL_Y_MAX - ACCEL_Y_OFFSET))
#define ACCEL_Z_SCALE (GRAVITY / (ACCEL_Z_MAX - ACCEL_Z_OFFSET))

//mag x,y,z (min/max) = -628.00/309.00  -584.00/373.00  -354.00/549.00
#define MAGN_X_MIN ((float) -628)
#define MAGN_X_MAX ((float) 309)
#define MAGN_Y_MIN ((float) -584)
#define MAGN_Y_MAX ((float) 373)
#define MAGN_Z_MIN ((float) -354)
#define MAGN_Z_MAX ((float) 549)

#define MAGN_X_OFFSET ((MAGN_X_MIN + MAGN_X_MAX) / 2.0f)
#define MAGN_Y_OFFSET ((MAGN_Y_MIN + MAGN_Y_MAX) / 2.0f)
#define MAGN_Z_OFFSET ((MAGN_Z_MIN + MAGN_Z_MAX) / 2.0f)
#define MAGN_X_SCALE (100.0f / (MAGN_X_MAX - MAGN_X_OFFSET))
#define MAGN_Y_SCALE (100.0f / (MAGN_Y_MAX - MAGN_Y_OFFSET))
#define MAGN_Z_SCALE (100.0f / (MAGN_Z_MAX - MAGN_Z_OFFSET))


#endif
