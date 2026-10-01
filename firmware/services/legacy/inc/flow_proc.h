#ifndef UAV_FLOW_SERVICE_H
#define UAV_FLOW_SERVICE_H
#include <stdint.h>
typedef struct {
    int16_t xNowOffset,xLastOffset,yNowOffset,yLastOffset,zNowHeight,zLastHeight;
    uint16_t delatTime;
    float xFlowVel,yFlowVel,zFlowVel;
    uint8_t flowFlag,flowConf;
} _flow_data;
/* Init before task creation; ISR copies at most one 14-byte frame, never waits.
 * Queue-full drops a sample; only Flow task writes state. Snapshot is task-only. */
int flow_transport_init(void);
void Flow_Task_Proc(void const *argument);
void Flow_Data_Proc(uint16_t size);
void flow_snapshot(_flow_data *out);
#endif
