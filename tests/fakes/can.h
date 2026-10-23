#pragma once
#include <cstddef>
#include <cstdint>

struct CAN_HandleTypeDef {};
struct CAN_FilterTypeDef {};
struct CAN_RxHeaderTypeDef { uint32_t StdId, IDE, RTR, DLC; };
constexpr uint32_t CAN_RX_FIFO0 = 0U, CAN_ID_STD = 0U, CAN_RTR_DATA = 0U;
constexpr int HAL_OK = 0, HAL_ERROR = 1;
extern "C" {
extern CAN_HandleTypeDef hcan1;
uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef *, uint32_t);
int HAL_CAN_GetRxMessage(CAN_HandleTypeDef *, uint32_t, CAN_RxHeaderTypeDef *, uint8_t *);
}
