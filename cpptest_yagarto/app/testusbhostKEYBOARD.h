/*
 * testusbhostKEYBOARD.h
 *
 *  Created on: 2013Äê8ÔÂ21ÈÕ
 *      Author: Hz
 */

#ifndef TESTUSBHOSTKEYBOARD_H_
#define TESTUSBHOSTKEYBOARD_H_

class KbdRptParser : public KeyboardReportParser
{
	void PrintKey(uint8_t mod, uint8_t key);

protected:
	virtual void OnControlKeysChanged(uint8_t before, uint8_t after);

	virtual void OnKeyDown	(uint8_t mod, uint8_t key);
	virtual void OnKeyUp	(uint8_t mod, uint8_t key);
	virtual void OnKeyPressed(uint8_t key);
};

#endif /* TESTUSBHOSTKEYBOARD_H_ */
