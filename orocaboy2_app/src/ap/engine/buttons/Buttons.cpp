/*
This file is part of the Gamebuino-Meta library,
Copyright (c) Aadalie 2014, 2016-2017

This is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License (LGPL)
as published by the Free Software Foundation, either version 3 of
the License, or (at your option) any later version.

This is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License (LGPL) for more details.

You should have received a copy of the GNU Lesser General Public
License (LGPL) along with the library.
If not, see <http://www.gnu.org/licenses/>.

Authors:
 - Aurelien Rodot
 - Sorunome
*/

#include "Buttons.h"



namespace Gamebuino_Meta {


void Buttons::begin() {
}

/*
 * reads each button states and store it
 */
void Buttons::update() {
	
	byte buttonsData = 0;



	if (adcRead12(_HW_DEF_ADC_Y_AXIS) > 3000) buttonsData |= (1<<(int)Button::down);
	if (adcRead12(_HW_DEF_ADC_X_AXIS) > 3000) buttonsData |= (1<<(int)Button::left);
	if (adcRead12(_HW_DEF_ADC_X_AXIS) < 1000) buttonsData |= (1<<(int)Button::right);
	if (adcRead12(_HW_DEF_ADC_Y_AXIS) < 1000) buttonsData |= (1<<(int)Button::up);

	if (buttonGetPressed(_HW_DEF_BUTTON_A)) buttonsData |= (1<<(int)Button::a);
	if (buttonGetPressed(_HW_DEF_BUTTON_B)) buttonsData |= (1<<(int)Button::b);
	if (buttonGetPressed(_HW_DEF_BUTTON_MENU)) buttonsData |= (1<<(int)Button::menu);
	if (buttonGetPressed(_HW_DEF_BUTTON_HOME)) buttonsData |= (1<<(int)Button::home);


  
	for (uint8_t thisButton = 0; thisButton < NUM_BTN; thisButton++) {
		//extract the corresponding bit corresponding to the current button
		bool pressed = (buttonsData & (1 << thisButton)) == (1 << thisButton);
		
		if (pressed) { //if button pressed
			if (states[thisButton] < 0xFFFD) { // we want 0xFFFE to be max value for the counter
				states[thisButton]++; //increase button hold time
			} else if (states[thisButton] == 0xFFFF) { // if we release / hold again too fast
				states[thisButton] = 1;
			}
		} else {
			if (states[thisButton] == 0) {//button idle
				continue;
			}
			if (states[thisButton] == 0xFFFF) {//if previously released
				states[thisButton] = 0; //set to idle
			} else {
				states[thisButton] = 0xFFFF; //button just released
			}
		}
	}
}

/*
 * Returns true when 'button' is pressed.
 * The button has to be released for it to be triggered again.
 */
bool Buttons::pressed(Button button) {
	return states[(uint8_t)button] == 1;
}

/*
 * return true if 'button' is released
 */
bool Buttons::released(Button button) {
	return states[(uint8_t)button] == 0xFFFF;
}

/**
 * returns true ONCE when 'button' is held for 'time' frames
 * @param button The button's ID
 * @param time How much frames button must be held, between 1 and 254.
 * @return true when 'button' is held for 'time' frames
 */
bool Buttons::held(Button button, uint16_t time){
	return states[(uint8_t)button] == (time+1);
}

/**
 * returns true every 'period' frames when 'button' is held
 * @param button The button's ID
 * @param period How much frames button must be held, between 1 and 254.
 * @return true if the button is held for the given time
 */
bool Buttons::repeat(Button button, uint16_t period) {
	if (period <= 1) {
		if ((states[(uint8_t)button] != 0xFFFF) && (states[(uint8_t)button])) {
			return true;
		}
	} else {
		if ((states[(uint8_t)button] != 0xFFFF) && ((states[(uint8_t)button] % period) == 1)) {
			return true;
		}
	}
	return false;
}

/**
 * 
 * @param button The button's ID
 * @return The number of frames during which the button has been held.
 */
uint16_t Buttons::timeHeld(Button button){
	if(states[(uint8_t)button] != 0xFFFF) {
		return states[(uint8_t)button];
	} else {
		return 0;
	}
}

} // namespace Gamebuino_Meta
