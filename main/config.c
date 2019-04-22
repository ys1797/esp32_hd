/*
esp32_hd http interface
Copyright (c) 2018 ys1797 (yuri@rus.net)

License (MIT license):
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:
  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "esp_log.h"
#include "config.h"

vaiable_list DEFL_PARAMS[] =
{
	{"maxPower",		VARIABLE_INT,	0,	15000,	"2000"},
	{"ustPowerReg",		VARIABLE_INT,	0,	15000,	"900"},
	{"tempEndRectRazgon",	VARIABLE_FLOAT, 0,	120,	"83.0"},
	{"powerRect",		VARIABLE_INT,	0,	15000,	"1000"},
	{"tEndRectOtbGlv",	VARIABLE_FLOAT, 0,	120,	"85.4"},
	{"timeChimRectOtbGlv",	VARIABLE_INT,	0,	10000,	"20"},
	{"procChimOtbGlv",	VARIABLE_INT,	0,	101,	"5"},
	{"minProcChimOtbSR",	VARIABLE_INT,	0,	101,	"20"},
	{"beginProcChimOtbSR",	VARIABLE_INT,	0,	100,	"40"},
	{"timeChimRectOtbSR",	VARIABLE_INT,	0,	1500,	"10"},
	{"tempDeltaRect",	VARIABLE_FLOAT, 0,	120,	"0.3"},
	{"tempEndRectOtbSR",	VARIABLE_FLOAT, 0,	120,	"96.5"},
	{"tempEndRect",		VARIABLE_FLOAT, 0,	120,	"99.5"},
	{"p_MPX5010",		VARIABLE_INT,	0,	100,	"0"},
	{"timeStabKolonna",	VARIABLE_INT,	0,	3500,	"900"},
	{"timeRestabKolonna",	VARIABLE_INT,	0,	3500,	"1800"},
	{"pShim", 		VARIABLE_INT,	0,	101,	"90"},
	{"klpSilentNode", 	VARIABLE_CHECKBOX, 0,	1,	"1"},
	{"urovenProvodimostSR", VARIABLE_INT,	0,	1000,	"0"},
	{"cntCHIM", 		VARIABLE_INT,	-100,	100,	"-4"},
	{"decrementCHIM", 	VARIABLE_INT,	0,	100,	"10"},
	{"incrementCHIM", 	VARIABLE_INT,	0,	100,	"5"},
	{"timeAutoIncCHIM", 	VARIABLE_INT,	0,	1000,	"600"},
	{"alarmMPX5010", 	VARIABLE_INT,	0,	100,	"0"},
	{"beepChangeState", 	VARIABLE_CHECKBOX, 0,	1,	"1"},

	{"powerDistil",		VARIABLE_INT,	0,	15000,	"1000"},
	{"tempEndDistil",	VARIABLE_FLOAT,	0,	120,	"99.5"},

	{NULL}
};
