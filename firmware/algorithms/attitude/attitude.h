#ifndef UAV_ATTITUDE_ALGORITHM_H
#define UAV_ATTITUDE_ALGORITHM_H
typedef struct {
    float q[4],integral[3];
    float roll_deg,pitch_deg,yaw_deg;
} uav_attitude_t;
/* Instance-owned Mahony fusion, SI gyro rad/s, dt seconds, direction-only
 * acceleration and magnetic vectors. Zero vectors are ignored. Returns 0
 * for success, -1 for nonfinite inputs or dt outside (0,0.1]. Nonblocking. */
void uav_attitude_init(uav_attitude_t *state);
int uav_attitude_step(uav_attitude_t *state,const float acc[3],const float gyro[3],
                      const float mag[3],float dt_s);
#endif
