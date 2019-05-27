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

/* Сетевые параметры */
vaiable_list NET_PARAMS[] =
{
	{"host",		VARIABLE_STRING, 0,	0,	DEFAULT_HOST,	NULL},
	{"user",		VARIABLE_STRING, 0,	0,	DEFAULT_USERNAME,NULL},
	{"pass",		VARIABLE_STRING, 0,	0,	DEFAULT_PASSWORD,NULL},
	{"secure",		VARIABLE_INT,	 0,	2,	"0",		NULL},
	{"smscUser",		VARIABLE_STRING, 0,	0,	"",		NULL},
	{"smscHash",		VARIABLE_STRING, 0,	0,	"",		NULL},
	{"smscPhones",		VARIABLE_STRING, 0,	0,	"",		NULL},
	{"useSmsc", 		VARIABLE_INT,	 0,	2,      "0",		NULL},
	{"wsPeriod", 		VARIABLE_INT,	 0, 	60,	"5",		NULL},
	{NULL}
};


/* Параметры устройства */
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

	{"processGpio",		VARIABLE_INT,	0,	40,	"0",	NULL},
	{"klp1_isPWM", 		VARIABLE_CHECKBOX, 0,	1,	"0",	NULL},

	{NULL}
};

/* Сброс параметров в значение по умолчанию */
int param_default(vaiable_list list[], const char *finename)
{
	FILE *f = fopen(finename, "w");
	vaiable_list *v;
	int  i;
	double d;

	if (f) {
		cJSON *ja = cJSON_CreateObject();
		for (v = list; v && v->name; v++) {
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
int param_load(vaiable_list list[], const char *finename)
{
	struct stat st;
	char *buff = NULL;
	size_t size;
	FILE *f;
	vaiable_list *v;
	cJSON *cj;
	int  i;
	double d;
	int save_param = 0;	// flag - save parameters

       	if (stat(finename, &st) != 0) {
		// Файл не найден - заполняем значениями по умолчанию
		return param_default(list, finename);
	}
	f = fopen(finename, "r");
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


	for (v = list; v && v->name; v++) {
		cj = cJSON_GetObjectItem(root, v->name);
		if (!cj) {
			// New parameter - create default and save
			v->val = strdup(v->default_val);
			save_param++;
			continue;
		}
		switch(cj->type) {
		case cJSON_True:
			v->val = strdup("1");
			break;
		case cJSON_False:		
			v->val = strdup("0");
			break;
		case cJSON_Number:
			{
			char b[32];
			snprintf(b, sizeof(b)-1, "%f", cj->valuedouble);
			v->val = strdup(b);
			}
			break;
		case cJSON_String:	
			v->val = strdup(cj->valuestring);
			break;
		default:
			continue;
		}
		switch (v->type) {
		case VARIABLE_CHECKBOX:
			i = atoi(v->val);
			break;
		case VARIABLE_INT:
			i = atoi(v->val);
			if (i < v->min || i > v->max) {
				free(v->val);
				v->val = strdup(v->default_val);
				save_param++;
			}
			break;
		case VARIABLE_FLOAT:
			d = atof(v->val);
			if (d < (float)(v->min) || d > (float)(v->max)) {
				free(v->val);
				v->val = strdup(v->default_val);
				save_param++;
			}
			break;
		default:
			break;
		}
	}
	if (save_param) param_save(list, finename);
	return 0;
}

/* Сохранение параметров работы */
int param_save(vaiable_list list[], const char *finename)
{
	vaiable_list *v;
	cJSON *ja;
	FILE *f = fopen(finename, "w");

	if (!f) {
		ESP_LOGI(TAG, "Save configuration failed. Can't open file: %s", finename);
		return -1;
	}

	ja = cJSON_CreateObject();
	for (v = list; v && v->name; v++) {
		cJSON_AddItemToObject(ja, v->name, cJSON_CreateString(v->val));
	}

	char *r=cJSON_Print(ja);
	fprintf(f, "%s", r);
	if (r) free(r);
	cJSON_Delete(ja);
	fclose(f);
	return 0;
}

/* Проверка существования параметра */
int checkParam(vaiable_list list[], char *name)
{
	vaiable_list *v;
	for (v = list; v && v->name; v++) {
		if (!strcasecmp(name, v->name)) return 1;
	}
	return 0;
}

/* Установка  переменной */
int setParam(vaiable_list list[], char *name, char *value)
{
	vaiable_list *v;
	for (v = list; v && v->name; v++) {
		if (!strcasecmp(name, v->name)) {
			switch (v->type) {
			case VARIABLE_CHECKBOX:
			case VARIABLE_STRING:
				if (v->val) free(v->val);
				v->val = strdup(value);
				break;
			case VARIABLE_INT:
				if (v->val) free(v->val);
				int i = atoi(v->default_val);
				if (i < v->min || i > v->max) {
					v->val = strdup(v->default_val);
				} else {
					v->val = strdup(value);
				}
				break;
			case VARIABLE_FLOAT:
				if (v->val) free(v->val);
				double d = atof(v->default_val);
				if (d < (float)(v->min) || d > (float)(v->max)) {
					v->val = strdup(v->default_val);
				} else {
					v->val = strdup(value);
				}
				break;
			}
			return 1;
		}
	}
	return 0;
}


/* Получение текстовой переменной */
char *getStringParam(vaiable_list list[], char *name)
{
	vaiable_list *v;
	for (v = list; v && v->name; v++) {
		if (!strcasecmp(name, v->name)) {
			if (NULL == v->val) return v->default_val;
			return v->val;
		}
	}
	return "";
}

/* Получение переменной типа int */
int  getIntParam(vaiable_list list[], char *name)
{
	vaiable_list *v;
	for (v = list; v && v->name; v++) {
		if (!strcasecmp(name, v->name)) {
			if (NULL == v->val) return atoi(v->default_val);
			return atoi(v->val);
		}
	}
	return 0;
}

/* Получение переменной типа float*/
float getFloatParam(vaiable_list list[], char *name)
{
	vaiable_list *v;
	for (v = list; v && v->name; v++) {
		if (!strcasecmp(name, v->name)) {
			if (NULL == v->val) return atof(v->default_val);
			return atof(v->val);
		}
	}
	return 0;
}
