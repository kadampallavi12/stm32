#include <gui/screen2_screen/Screen2View.hpp>

uint16_t counter=0;

Screen2View::Screen2View()
{

}

void Screen2View::setupScreen()
{
    Screen2ViewBase::setupScreen();
}

void Screen2View::tearDownScreen()
{
    Screen2ViewBase::tearDownScreen();

}

void Screen2View::function1()
{
    counter++;
	Unicode::snprintf(textArea1Buffer, TEXTAREA1_SIZE, "%u", counter);

}

void Screen2View::function2()
{
    counter--;
  Unicode::snprintf(textArea1Buffer, TEXTAREA1_SIZE, "%u", counter);

}