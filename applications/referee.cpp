#include "referee.hpp"

sp::PM02 pm02(&huart6);

void referee_start_receive()
{
  if (pm02.huart != nullptr) {
    pm02.request();
  }
}
