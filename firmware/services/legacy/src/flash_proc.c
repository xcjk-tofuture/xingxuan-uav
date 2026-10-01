#include "flash_proc.h"
#include "queue.h"
#include "semphr.h"
#include <math.h>
static SemaphoreHandle_t storage_mutex;
static QueueHandle_t writes;
typedef struct {
    uint8_t kind;
    union {
        _imuData_all imu;
        _sbus_ch_struct remote;
        _uav_control_data motor;
    } data;
} storage_request_t;
osThreadId FlashTaskHandle;
int uav_storage_init(void) {
    storage_mutex = xSemaphoreCreateMutex();
    writes = xQueueCreate(2, sizeof(storage_request_t));
    return storage_mutex && writes ? 0 : -1;
}
static void storage_lock(void) {
    if (!storage_mutex || xSemaphoreTake(storage_mutex, pdMS_TO_TICKS(1000)) != pdTRUE)
        Error_Handler();
}
static void storage_unlock(void) { xSemaphoreGive(storage_mutex); }

#define EXTERN_FLASH 1

#define FLASH_IMU_ADDR 0X000
#define FLASH_REMOTE_ADDR 0X200
#define FLASH_MOTOR_ADDR 0X400

// extern _uav_control_data uav_control_data;
//_uav_control_data uav_control_test_data;

// 读飞控IMU数据 主要占据0x000 - 0x199
void UAV_Read_Param_IMU(_imuData_all *imu_data) {
    storage_lock();

    u8 temp_imu_read[18 * 4] = {0};
    float temp_imu_read_value[18] = {0};
    W25QXX_Read(temp_imu_read, FLASH_IMU_ADDR, sizeof(temp_imu_read));
    memcpy(temp_imu_read_value, temp_imu_read, sizeof(temp_imu_read));

    //	imu_data->accoffsetbias.x = temp_imu_read_value[0];
    //	imu_data->accoffsetbias.y = temp_imu_read_value[1];
    //	imu_data->accoffsetbias.z = temp_imu_read_value[2];
    //	imu_data->accscalebias.x = temp_imu_read_value[3];
    //	imu_data->accscalebias.y = temp_imu_read_value[4];
    //	imu_data->accscalebias.z = temp_imu_read_value[5];
    //
    //
    //  imu_data->gyrooffsetbias.x = temp_imu_read_value[6];
    //	imu_data->gyrooffsetbias.y = temp_imu_read_value[7];
    //	imu_data->gyrooffsetbias.z = temp_imu_read_value[8];
    //	imu_data->gyroscalebias.x = temp_imu_read_value[9];
    //	imu_data->gyroscalebias.y = temp_imu_read_value[10];
    //	imu_data->gyroscalebias.z = temp_imu_read_value[11];

    for (unsigned i = 12; i < 18; i++) {
        if (!isfinite(temp_imu_read_value[i]))
            temp_imu_read_value[i] = i < 15 ? 0.0f : 1.0f;
    }
    imu_data->magoffsetbias.x = temp_imu_read_value[12];
    imu_data->magoffsetbias.y = temp_imu_read_value[13];
    imu_data->magoffsetbias.z = temp_imu_read_value[14];
    imu_data->magscalebias.x = temp_imu_read_value[15];
    imu_data->magscalebias.y = temp_imu_read_value[16];
    imu_data->magscalebias.z = temp_imu_read_value[17];

    storage_unlock();
}

// 读飞控遥控器数据 主要占据0x200 - 0x399
void UAV_Read_Param_Remote(_sbus_ch_struct *channel_data) {
    storage_lock();

    u8 temp_remote_read[8 * 2 * 2] = {0};
    u16 temp_remote_read_value[8 * 2] = {0};
    W25QXX_Read(temp_remote_read, FLASH_REMOTE_ADDR, sizeof(temp_remote_read));
    memcpy(temp_remote_read_value, temp_remote_read, sizeof(temp_remote_read));

    channel_data->CH1_MAX = temp_remote_read_value[0];
    channel_data->CH1_MIN = temp_remote_read_value[1];

    channel_data->CH2_MAX = temp_remote_read_value[2];
    channel_data->CH2_MIN = temp_remote_read_value[3];

    channel_data->CH3_MAX = temp_remote_read_value[4];
    channel_data->CH3_MIN = temp_remote_read_value[5];

    channel_data->CH4_MAX = temp_remote_read_value[6];
    channel_data->CH4_MIN = temp_remote_read_value[7];

    channel_data->CH5_MAX = temp_remote_read_value[8];
    channel_data->CH5_MIN = temp_remote_read_value[9];

    channel_data->CH6_MAX = temp_remote_read_value[10];
    channel_data->CH6_MIN = temp_remote_read_value[11];

    channel_data->CH7_MAX = temp_remote_read_value[12];
    channel_data->CH7_MIN = temp_remote_read_value[13];

    channel_data->CH8_MAX = temp_remote_read_value[14];
    channel_data->CH8_MIN = temp_remote_read_value[15];
    storage_unlock();
}

// 读飞控电机数据  主要占据0x400 - 0x599
void UAV_Read_Param_Motor(_uav_control_data *motor_data) {
    storage_lock();

    u8 temp_motor_read[5 * 3 * 4] = {0};
    float temp_motor_read_value[15] = {0};
    W25QXX_Read(temp_motor_read, FLASH_MOTOR_ADDR, sizeof(temp_motor_read));
    memcpy(temp_motor_read_value, temp_motor_read, sizeof(temp_motor_read));

    motor_data->rollData.Kp = temp_motor_read_value[0];
    motor_data->rollData.Ki = temp_motor_read_value[1];
    motor_data->rollData.Kd = temp_motor_read_value[2];

    motor_data->pitchData.Kp = temp_motor_read_value[3];
    motor_data->pitchData.Ki = temp_motor_read_value[4];
    motor_data->pitchData.Kd = temp_motor_read_value[5];

    motor_data->yawData.Kp = temp_motor_read_value[6];
    motor_data->yawData.Ki = temp_motor_read_value[7];
    motor_data->yawData.Kd = temp_motor_read_value[8];

    motor_data->rollSpeedData.Kp = temp_motor_read_value[9];
    motor_data->rollSpeedData.Ki = temp_motor_read_value[10];
    motor_data->rollSpeedData.Kd = temp_motor_read_value[11];

    motor_data->pitchSpeedData.Kp = temp_motor_read_value[12];
    motor_data->pitchSpeedData.Ki = temp_motor_read_value[13];
    motor_data->pitchSpeedData.Kd = temp_motor_read_value[14];

    storage_unlock();
}

// 写飞控IMU数据 主要占据0x000 - 0x199
static void storage_write_IMU(_imuData_all imu_data) {
    float temp_imu_write[18] = {0};
    temp_imu_write[0] = imu_data.accoffsetbias.x;
    temp_imu_write[1] = imu_data.accoffsetbias.y;
    temp_imu_write[2] = imu_data.accoffsetbias.z;
    temp_imu_write[3] = imu_data.accscalebias.x;
    temp_imu_write[4] = imu_data.accscalebias.y;
    temp_imu_write[5] = imu_data.accscalebias.z;

    temp_imu_write[6] = imu_data.gyrooffsetbias.x;
    temp_imu_write[7] = imu_data.gyrooffsetbias.y;
    temp_imu_write[8] = imu_data.gyrooffsetbias.z;
    temp_imu_write[9] = imu_data.gyroscalebias.x;
    temp_imu_write[10] = imu_data.gyroscalebias.y;
    temp_imu_write[11] = imu_data.gyroscalebias.z;

    temp_imu_write[12] = imu_data.magoffsetbias.x;
    temp_imu_write[13] = imu_data.magoffsetbias.y;
    temp_imu_write[14] = imu_data.magoffsetbias.z;
    temp_imu_write[15] = imu_data.magscalebias.x;
    temp_imu_write[16] = imu_data.magscalebias.y;
    temp_imu_write[17] = imu_data.magscalebias.z;

    W25QXX_Write((u8 *)temp_imu_write, FLASH_IMU_ADDR, sizeof(temp_imu_write));
}

// 写飞控遥控器数据 主要占据0x200 - 0x399
static void storage_write_Remote(_sbus_ch_struct channe_data) {
    uint16_t temp_remote_write[16] = {0};
    temp_remote_write[0] = channe_data.CH1_MAX;
    temp_remote_write[1] = channe_data.CH1_MIN;

    temp_remote_write[2] = channe_data.CH2_MAX;
    temp_remote_write[3] = channe_data.CH2_MIN;

    temp_remote_write[4] = channe_data.CH3_MAX;
    temp_remote_write[5] = channe_data.CH3_MIN;

    temp_remote_write[6] = channe_data.CH4_MAX;
    temp_remote_write[7] = channe_data.CH4_MIN;

    temp_remote_write[8] = channe_data.CH5_MAX;
    temp_remote_write[9] = channe_data.CH5_MIN;

    temp_remote_write[10] = channe_data.CH6_MAX;
    temp_remote_write[11] = channe_data.CH6_MIN;

    temp_remote_write[12] = channe_data.CH7_MAX;
    temp_remote_write[13] = channe_data.CH7_MIN;

    temp_remote_write[14] = channe_data.CH8_MAX;
    temp_remote_write[15] = channe_data.CH8_MIN;

    W25QXX_Write((u8 *)temp_remote_write, FLASH_REMOTE_ADDR, sizeof(temp_remote_write));
}

// 写飞控电机数据  主要占据0x400 - 0x599
static void storage_write_Motor(_uav_control_data motor_data) {
    float temp_motor_write[15] = {0};

    temp_motor_write[0] = motor_data.rollData.Kp;
    temp_motor_write[1] = motor_data.rollData.Ki;
    temp_motor_write[2] = motor_data.rollData.Kd;

    temp_motor_write[3] = motor_data.pitchData.Kp;
    temp_motor_write[4] = motor_data.pitchData.Ki;
    temp_motor_write[5] = motor_data.pitchData.Kd;

    temp_motor_write[6] = motor_data.yawData.Kp;
    temp_motor_write[7] = motor_data.yawData.Ki;
    temp_motor_write[8] = motor_data.yawData.Kd;

    temp_motor_write[9] = motor_data.rollSpeedData.Kp;
    temp_motor_write[10] = motor_data.rollSpeedData.Ki;
    temp_motor_write[11] = motor_data.rollSpeedData.Kd;

    temp_motor_write[12] = motor_data.pitchSpeedData.Kp;
    temp_motor_write[13] = motor_data.pitchSpeedData.Ki;
    temp_motor_write[14] = motor_data.pitchSpeedData.Kd;

    W25QXX_Write((u8 *)temp_motor_write, FLASH_MOTOR_ADDR, sizeof(temp_motor_write));
}

// SPI读写一个字节
// TxData:要写入的字节
// 返回值:读取到的字节
void UAV_Write_Param_IMU(_imuData_all data) {
    storage_request_t request = {.kind = 0};
    request.data.imu = data;
    if (!writes || xQueueSend(writes, &request, 0) != pdPASS)
        Error_Handler();
}
void UAV_Write_Param_Remote(_sbus_ch_struct data) {
    storage_request_t request = {.kind = 1};
    request.data.remote = data;
    if (!writes || xQueueSend(writes, &request, 0) != pdPASS)
        Error_Handler();
}
void UAV_Write_Param_Motor(_uav_control_data data) {
    storage_request_t request = {.kind = 2};
    request.data.motor = data;
    if (!writes || xQueueSend(writes, &request, 0) != pdPASS)
        Error_Handler();
}
void Flash_Task_Proc(void const *argument) {
    storage_request_t request;
    (void)argument;
    for (;;) {
        if (xQueueReceive(writes, &request, portMAX_DELAY) != pdPASS)
            continue;
        storage_lock();
        if (request.kind == 0)
            storage_write_IMU(request.data.imu);
        else if (request.kind == 1)
            storage_write_Remote(request.data.remote);
        else if (request.kind == 2)
            storage_write_Motor(request.data.motor);
        storage_unlock();
    }
}
