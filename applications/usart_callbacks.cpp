#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"
#include "referee.hpp"
#include "usart.h"

extern sp::DBus remote;

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t size)
{
  if (huart == &huart3) {
    remote.update(size, osKernelSysTick());
    remote.request();
  }
  else if (huart == pm02.huart) {
    pm02.update(size);
    pm02.request();
  }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart == &huart3) {
    remote.request();
  }
  else if (huart == pm02.huart) {
    pm02.request();
  }
}
