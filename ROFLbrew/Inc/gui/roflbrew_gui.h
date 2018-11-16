/*
 * roflbrew_gui.h
 *
 *  Created on: 05.11.2016
 *      Author: washed
 */

#ifndef GUI_ROFLBREW_GUI_H_
#define GUI_ROFLBREW_GUI_H_

#include "ugui.h"

#define PADDING_X 10

extern char is_temperature_string[];
extern UG_GUI gui;

void gui_init();
void gui_init_mainwindow();
void gui_update();

#endif /* GUI_ROFLBREW_GUI_H_ */
