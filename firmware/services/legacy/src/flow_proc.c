#include "flow_proc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "cmsis_os.h"
#include <string.h>
extern uint8_t uart2RX[200];
static QueueHandle_t frames;
static _flow_data state;
osThreadId FlowTaskHandle;
int flow_transport_init(void) { frames=xQueueCreate(4,14);return frames?0:-1; }
void Flow_Data_Proc(uint16_t size) {
    if(size!=14 || !frames || uart2RX[0]!=0xfe || uart2RX[1]!=0x0a)return;
    BaseType_t wake=pdFALSE;xQueueSendFromISR(frames,uart2RX,&wake);portYIELD_FROM_ISR(wake);
}
void flow_snapshot(_flow_data *out) {if(!out)return;taskENTER_CRITICAL();*out=state;taskEXIT_CRITICAL();}
static uint16_t le16(const uint8_t *p) {return (uint16_t)(p[0]|((uint16_t)p[1]<<8));}
void Flow_Task_Proc(void const *argument) {
    uint8_t frame[14];_flow_data next={0};uint8_t previous=0;(void)argument;
    for(;;) {
        if(xQueueReceive(frames,frame,pdMS_TO_TICKS(100))!=pdPASS) {
            next.flowFlag=0;next.xFlowVel=next.yFlowVel=next.zFlowVel=0;previous=0;
        } else {
            uint16_t dt=le16(frame+6);
            next.zNowHeight=(int16_t)le16(frame+8);
            next.xNowOffset=(int16_t)(le16(frame+2)*next.zNowHeight/10000.0f);
            next.yNowOffset=(int16_t)(le16(frame+4)*next.zNowHeight/10000.0f);
            next.delatTime=dt;next.flowFlag=frame[10];next.flowConf=frame[11];
            if(dt && previous) {
                float seconds=dt/1000000.0f;
                next.xFlowVel=(next.xNowOffset-next.xLastOffset)/seconds;
                next.yFlowVel=(next.yNowOffset-next.yLastOffset)/seconds;
                next.zFlowVel=(next.zNowHeight-next.zLastHeight)/seconds;
            } else {next.xFlowVel=next.yFlowVel=next.zFlowVel=0;}
            previous=dt!=0;next.xLastOffset=next.xNowOffset;
            next.yLastOffset=next.yNowOffset;next.zLastHeight=next.zNowHeight;
        }
        taskENTER_CRITICAL();state=next;taskEXIT_CRITICAL();
    }
}
