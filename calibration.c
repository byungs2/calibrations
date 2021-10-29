#include <stdio.h>
#include <stdlib.h>
#include <k4a/k4a.h>
#include <math.h>

#define PI 3.1415926

int main(int argc, char *argv[]) {
  k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
  config.camera_fps = K4A_FRAMES_PER_SECOND_30;
  config.color_format = K4A_IMAGE_FORMAT_COLOR_MJPG;
  config.color_resolution = K4A_COLOR_RESOLUTION_720P;

  k4a_device_t device_handler;
  k4a_calibration_t calibration;

  k4a_device_open(0, &device_handler);
  size_t serial_size = 0;
  k4a_device_get_serialnum(device_handler, NULL, &serial_size);
  char *serial = (char*)(malloc(serial_size));
  k4a_device_get_serialnum(device_handler, serial, &serial_size);
  printf("Serial num :: %s\n", serial);
  free(serial);

  k4a_result_t result;
  result = k4a_device_get_calibration(device_handler, K4A_DEPTH_MODE_NFOV_UNBINNED, K4A_COLOR_RESOLUTION_720P, &calibration);
  printf(result == K4A_RESULT_SUCCEEDED ? "Success\n" : "Failure\n");
  float cx = calibration.color_camera_calibration.intrinsics.parameters.param.cx;
  float cy = calibration.color_camera_calibration.intrinsics.parameters.param.cy;
  float fx = calibration.color_camera_calibration.intrinsics.parameters.param.fx;
  float fy = calibration.color_camera_calibration.intrinsics.parameters.param.fy;
  float k1 = calibration.color_camera_calibration.intrinsics.parameters.param.k1;
  float k2 = calibration.color_camera_calibration.intrinsics.parameters.param.k2;
  float k3 = calibration.color_camera_calibration.intrinsics.parameters.param.k3;
  float k4 = calibration.color_camera_calibration.intrinsics.parameters.param.k4;
  float k5 = calibration.color_camera_calibration.intrinsics.parameters.param.k5;
  float k6 = calibration.color_camera_calibration.intrinsics.parameters.param.k6;
  float codx = calibration.color_camera_calibration.intrinsics.parameters.param.codx;
  float cody = calibration.color_camera_calibration.intrinsics.parameters.param.cody;
  float p2 = calibration.color_camera_calibration.intrinsics.parameters.param.p2;
  float p1 = calibration.color_camera_calibration.intrinsics.parameters.param.p1;
  float matric_radius = calibration.color_camera_calibration.intrinsics.parameters.param.metric_radius;
  printf("cx :: %f\ncy :: %f\nfx :: %f\nfy :: %f\n", cx, cy, fx, fy);
  printf("k1 :: %f\nk2 :: %f\nk3 :: %f\nk4 :: %f\nk5 ::%f\nk6 :: %f\n", k1, k2, k3, k4, k5, k6);
  printf("codx :: %f\ncody :: %f\np2 :: %f\np1 :: %f\n", codx, cody, p2, p1);
  printf("matric_radius :: %f\n", matric_radius);

  k4a_device_start_cameras(device_handler, &config);
  k4a_device_start_imu(device_handler);

  k4a_imu_sample_t imu_sample;

  // Capture a imu sample
  k4a_device_get_imu_sample(device_handler, &imu_sample, 1000000);

  // Access the accelerometer readings
  printf(" | Accelerometer temperature:%.2f x:%.4f y:%.4f z: %.4f\n",
        imu_sample.temperature,
        imu_sample.acc_sample.xyz.x,
        imu_sample.acc_sample.xyz.y,
        imu_sample.acc_sample.xyz.z);
  double x;
  double y;
  double z;
  x = imu_sample.acc_sample.xyz.x;
  y = imu_sample.acc_sample.xyz.y;
  z = imu_sample.acc_sample.xyz.z;
  double g;
  double roll;
  double pitch;
  g = sqrt(x*x + y*y + z*z);
  printf("Gravity :: %.2f\n", g);
  const double pi = (double)PI;
  roll = atan2(y, z);
  pitch = atan2(-x, y*sin(roll) + z * cos(roll));
  printf("Roll :: %.2f\nPitch :: %.2f\n", roll, pitch);
  roll = roll * 180 / pi;
  pitch = pitch * 180 / pi;
  printf("Roll :: %.2f\nPitch :: %.2f\n", roll, pitch);

  return 0;
}

