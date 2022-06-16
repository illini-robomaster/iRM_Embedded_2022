% Run the IMU under static environment (i.e. no motion)

acce = squeeze(out.acce);
gyro = squeeze(out.gyro);
mag = squeeze(out.mag);

acce_var = var(acce, 0, 2);
gyro_var = var(gyro, 0, 2);
mag_var = var(mag, 0, 2);