#include "Touch.h"



#include <engine/orocaboy.h>
extern Gamebuino gb;

namespace Gamebuino_Meta {





uint8_t Touch::isDetected(void)
{
  return tsIsDetected();
}

uint16_t Touch::getX(uint8_t detect_num)
{
  return tsGetXAxis(detect_num) * gb.display.width() / DISPLAY_LCD_WIDTH;
}

uint16_t Touch::getY(uint8_t detect_num)
{
  return tsGetYAxis(detect_num) * gb.display.height() / DISPLAY_LCD_HEIGHT;
}

uint8_t Touch::getWeight(uint8_t detect_num)
{
  return tsGetWeight(detect_num);
}


} // namespace Gamebuino_Meta
