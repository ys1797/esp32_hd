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

#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <dirent.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <cJSON.h>

#include "esp_log.h"
#include "config.h"

vaiable_list DEFL_PARAMS[] =
{
	{"maxPower",		VARIABLE_INT,	0,	15000,	"2000",	NULL},
	{"ustPowerReg",		VARIABLE_INT,	0,	15000,	"900",	NULL},
	{"tempEndRectRazgon",	VARIABLE_FLOAT, 0,	120,	"83.0",	NULL},
	{"powerRect",		VARIABLE_INT,	0,	15000,	"1000",	NULL},
	{"tEndRectOtbGlv",	VARIABLE_FLOAT, 0,	120,	"85.4",	NULL},
	{"timeChimRectOtbGlv",	VARIABLE_INT,	0,	10000,	"20",	 NULL},
	{"procChimOtbGlv",	VARIABLE_INT,	0,	101,	"5",	NULL},
	{"minProcChimOtbSR",	VARIABLE_INT,	0,	101,	"20",	NULL},
	{"beginProcChimOtbSR",	VARIABLE_INT,	0,	100,	"40",	NULL},
	{"timeChimRectOtbSR",	VARIABLE_INT,	0,	1500,	"10",	NULL},
	{"tempDeltaRect",	VARIABLE_FLOAT, 0,	120,	"0.3",	NULL},
	{"tempEndRectOtbSR",	VARIABLE_FLOAT, 0,	120,	"96.5", NULL},
	{"tempEndRect",		VARIABLE_FLOAT, 0,	120,	"99.5", NULL},
	{"p_MPX5010",		VARIABLE_INT,	0,	100,	"0",	NULL},
	{"timeStabKolonna",	VARIABLE_INT,	0,	3500,	"900",	NULL},
	{"timeRestabKolonna",	VARIABLE_INT,	0,	3500,	"1800", NULL},
	{"pShim", 		VARIABLE_INT,	0,	101,	"90",	NULL},
	{"klpSilentNode", 	VARIABLE_CHECKBOX, 0,	1,	"1",	NULL},
	{"urovenProvodimostSR", VARIABLE_INT,	0,	1000,	"0",	NULL},
	{"cntCHIM", 		VARIABLE_INT,	-100,	100,	"-4",	NULL},
	{"decrementCHIM", 	VARIABLE_INT,	0,	100,	"10",	NULL},
	{"incrementCHIM", 	VARIABLE_INT,	0,	100,	"5",	NULL},
	{"timeAutoIncCHIM", 	VARIABLE_INT,	0,	1000,	"600",	NULL},
	{"alarmMPX5010", 	VARIABLE_INT,	0,	100,	"0",	NULL},
	{"beepChangeState", 	VARIABLE_CHECKBOX, 0,	1,	"1",	NULL},

	{"powerDistil",		VARIABLE_INT,	0,	15000,	"1000", NULL},
	{"tempEndDistil",	VARIABLE_FLOAT,	0,	120,	"99.5", NULL},

	{NULL}
};

/* Сброс параметров в значение по умолчанию */
int param_default(void)
{
	FILE *f = fopen(RECT_CONFIGURATION, "w");
	vaiable_list *v;
	int  i;
	double d;

	if (f) {
		cJSON *ja = cJSON_CreateObject();
		for (v = DEFL_PARAMS; v && v->name; v++) {
			v->val = strdup(v->default_val);
			switch (v->type) {
			case VARIABLE_CHECKBOX:
				i = atoi(v->default_val);
				cJSON_AddItemToObject(ja, v->name, cJSON_CreateBool(i));
				break;
			case VARIABLE_STRING:
				cJSON_AddItemToObject(ja, v->name, cJSON_CreateString(v->default_val));
				break;
			case VARIABLE_INT:
				i = atoi(v->default_val);
				cJSON_AddItemToObject(ja, v->name, cJSON_CreateNumber(i));
				break;
			case VARIABLE_FLOAT:
				d = atof(v->default_val);
				cJSON_AddItemToObject(ja, v->name, cJSON_CreateNumber(d));
				break;
			}
		}
		char *r=cJSON_Print(ja);
		fprintf(f, "%s", r);
		if (r) free(r);
		cJSON_Delete(ja);
		fclose(f);
	}
	return 0;
}

/* Загрузка и установка параметров работы */
int param_load(void)
{
	struct stat st;
	char *buff = NULL;
	size_t size;
	FILE *f;
	vaiable_list *v;
	cJSON *cj;
	int  i;
	double d;

       	if (stat(RECT_CONFIGURATION, &st) != 0) {
		// Файл не найден - заполняем значениями по умолчанию
		return param_default();
	}
	f = fopen(RECT_CONFIGURATION, "r");
	buff = malloc(st.st_size+2);
	if (!buff) {
		fclose(f);
		return -2;
	}
	size = fread(buff, 1, st.st_size, f);
	buff[size]=0;
	fclose(f);

	cJSON *root = cJSON_Parse(buff);
	free(buff);
	if (!root) return -3;


	for (v = DEFL_PARAMS; v && v->name; v++) {
		cj = cJSON_GetObjectItem(root, v->name);
		if (!cj) continue; // TODO - need default value
		v->val = strdup(cj->valuestring);

		switch (v->type) {
		case VARIABLE_CHECKBOX:
			i = atoi(v->val);
			break;
		case VARIABLE_INT:
			i = atoi(v->val);
			break;
		case VARIABLE_FLOAT:
			d = atof(v->val);
			break;
		default:
			break;
		}
	}

	return 0;
}


/* Получение текстовой переменной */
char *getStringParam(char *name)
{
	vaiable_list *v;
	for (v = DEFL_PARAMS; v && v->name; v++) {
		if (VARIABLE_STRING == v->type && !strcasecmp(name, v->name)) {
			if (NULL == v->val) return v->default_val;
			return v->val;
		}
	}
	return "";
}

/* Получение переменной типа int */
int  getIntParam(char *name)
{
	vaiable_list *v;
	for (v = DEFL_PARAMS; v && v->name; v++) {
		if (VARIABLE_INT == v->type && !strcasecmp(name, v->name)) {
			if (NULL == v->val) return atoi(v->default_val);
			return atoi(v->val);
		}
	}

	return 0;
}

/* Получение переменной типа float*/
float getFloatParam(char *name)
{
	vaiable_list *v;
	for (v = DEFL_PARAMS; v && v->name; v++) {
		if (VARIABLE_FLOAT == v->type && !strcasecmp(name, v->name)) {
			if (NULL == v->val) return atof(v->default_val);
			return atof(v->val);
		}
	}

	return 0;
}
