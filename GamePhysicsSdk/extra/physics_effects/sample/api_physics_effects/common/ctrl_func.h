/*
Physics Effects Copyright(C) 2012 Sony Computer Entertainment Inc.
All rights reserved.

Physics Effects is open software; you can redistribute it and/or
modify it under the terms of the BSD License.

Physics Effects is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the BSD License for more details.

A copy of the BSD License is distributed with
Physics Effects under the filename: physics_effects_license.txt
*/

#ifndef __CTRL_FUNC_H__
#define __CTRL_FUNC_H__

//E Serialize and character controller share same buttons
//J シリアライズとキャラクター操作はボタンを共有します。

enum ButtonID {
	BTN_SCENE_RESET=0,
	BTN_SCENE_NEXT,
	BTN_SIMULATION,
	BTN_STEP,
	BTN_UP,
	BTN_DOWN,
	BTN_LEFT,
	BTN_RIGHT,
	BTN_ZOOM_IN,
	BTN_ZOOM_OUT,
	BTN_PICK,
	BTN_SERIALIZE_IN, 
	BTN_SERIALIZE_OUT,
	BTN_JUMP,
	BTN_CROUCH,
	BTN_CAMERA_TOP,
	BTN_NUM
};

enum ButtonStatus {
	BTN_STAT_NONE = 0,
	BTN_STAT_DOWN,
	BTN_STAT_UP,
	BTN_STAT_KEEP
};

void ctrlInit();
void ctrlRelease();
void ctrlUpdate();

void ctrlSetScreenSize(int w,int h);
void ctrlGetCursorPosition(int &cursorX,int &cursorY);
bool ctrlGetCursorEnable();

ButtonStatus ctrlButtonPressed(ButtonID btnId);

#endif /* __CTRL_FUNC_H__ */
