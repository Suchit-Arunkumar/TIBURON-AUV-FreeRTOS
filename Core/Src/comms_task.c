#include "comms_task.h"
#include "packet.h"

TaskHandle_t commsTaskHandle = NULL;
QueueHandle_t commandQueue = NULL;


void comms_task(void *argument)
{
    (void)argument;

    while (1)
    {
        // TODO 1:
        // Block here until USART1 ISR notifies this task.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);


        // TODO 2:
        // Create a local CommandPayload variable.
        // This will hold the parsed command.
        CommandPayload cmd;


        // TODO 3:
        // Try to parse a command from the ring buffer.
        // packet_parse_cmd() returns:
        //   1 -> valid packet parsed
        //   0 -> no valid packet available
        if (packet_parse_cmd(&cmd))
          {
              // valid command
        	xQueueSend(commandQueue ,&cmd , pdMS_TO_TICKS(1));
          }
    }





}
