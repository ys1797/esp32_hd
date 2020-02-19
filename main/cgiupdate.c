#include "esp_platform.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "cgiupdate.h"
#include "httpd-platform.h"
#include "esp_request.h"
#include "config.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#define PAGELEN 4096

static const char updateIndexHtml[] = R"END(
<!doctype html>
<html><head>
<meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=yes" />
<meta http-equiv='Content-Type' content='text/html; charset=utf-8' />
<title>HD32 Update page</title>
<style>.fs{display:none}body,html{font-family: Arial;line-height: 1.2em; font-size: 14px; background-color:#FFFFFF;}
input {background-color: #dfdfdf; padding: 5px;}</style>
<script src='http://hd.rus.net/jq.js'></script>
</head>
<body>
<div style="text-align: center;">
<H2>hd_ESP32 - Обновление системы</H2>
<div class="bold">Текущая версия: <span id='f_v'>0.0</span></div>
<div id=f_i class=fs>Получение информации..</div>
<div id=f_e class=fs>Оимбка получения информации. Проверьте интернет соединение.</div>
<div id=f_l class=fs>Последняя версия. Нет необходимости в обновлении.</div>
<div id=f_a class=fs>Последняя версия: <span id=newversion></span></div>
<div>Url для обновления: <input type="text" name="url" id="url" value="http://hd.rus.net" /></div>
<div><form name="upload">
<input type="file" name="myfile">
<input type="submit" value="Загрузить">
</form></div> 
<input type=button onclick='start_update();' value='Начать обновление' />
<input type=button onclick='window.location.href="/index.html";' value='Вернуться на главную' />
</div>
<div id=f_p class=fs>Начало обнавления..</div>
<hr />
</div>
<script>
var url, fw, files, cnt;
$.ajaxSetup({async: false});
$(document).ready(function() {
  url = $('#url').val();
  $('#f_i').show();
  $.get('/version', {}, function(d) {
    $('#f_v').html(d.fw); $('#f_v').show(); $('#f_i').hide();
    fw=d.fw;
  }, 'json');
  $.get(url+'/ver.php', {}, function(d) {
    $('#newversion').html(d.version);
    $('#f_a').show();
  }, 'json');
  
});

wait_dl = function () {
  $.get('/dl_status', {}, function(d) {
    var f = files[cnt];
    if (!eval(d.finished)) setTimeout(wait_dl, 100);
    else {
      if (!eval(d.error)) {
        $('#f_p').append('<br> Скачивание '+f+' успешно завершено');
        cnt++;
        if (cnt<files.length) setTimeout(start_dl, 10, files[cnt]);
        else $('#f_p').append('<br> Загрузка завершена');
      } else {
        $('#f_p').append('<br> Ошибка скачивания '+f+' '+d.msg);
      }
    }
  }, 'json');
}

start_dl = function (f) {
  $.get('/dl', {'a':'add', 'f':f, 'u':url+'/'+f}, function(d) {
    $('#f_p').append('<br> Скачивание '+ f);
    setTimeout(wait_dl, 1000);
  }, 'html');
}

start_update = function () {
  url = $('#url').val();
  $.get(url+'/ver.php', {}, function(d) {
    files = d.files; cnt = 0;
    $('#f_p').show();
    if (!files.length) {$('#f_p').append('<br> нечего обновлять'); return false;}
    start_dl(files[cnt]);
  }, 'json');
}

function upload(blobOrFile,act) {
  var xhr = new XMLHttpRequest();
    xhr.onload = xhr.onerror = function() {

    if (this.status == 200) {
               var j=JSON.parse(this.response);
               var log=document.getElementById('f_p');
      log.innerHTML=log.innerHTML+'success '+ j.received+'/'+j.written;
    } else {
      log.innerHTML=log.innerHTML+'<br>error ' + this.status;
    }
  };
  xhr.open('POST', '/'+act+'?filename='+blobOrFile.name, true);
  xhr.send(blobOrFile);
  return false;
}

document.forms.upload.onsubmit = function() {
 var input = this.elements.myfile;
 var file = input.files[0];
 var log=document.getElementById('f_p');
 log.style.display='block';
 log.innerHTML=log.innerHTML+'<br>upload file: '+file.name;
 var act ='fu';
 if (file.name=='esp32_hd.bin')act ='fw';
 if (file) {
   upload(file,act);
 }
 return false;
}


</script></body></html>
)END";

static const char *json_ok = "{\"err\":\"0\",\"msg\":\"OK\"}";

UpdaterState update_state;  // Статус процесса обновлений
char *update_url;
char *update_filename;
int update_finished;
int update_error;
char *update_errorMsg;
FILE *opened_file = NULL;

typedef struct {
	const char *stringPos;
} sendState;

static HttpdPlatTimerHandle updateTimer;

int download_callback(request_t *req, char *data, int len)
{
	if (opened_file) {
		fwrite(data, 1, len, opened_file);
	}
	return 0;
}

static void download_task(void *arg) {
	request_t *req;
	int ret;
	ESP_LOGI(TAG, ">> Start download");
	unlink(update_filename);
	opened_file = fopen(update_filename, "w");
	if (!opened_file) {
		ESP_LOGI(TAG, "fopen failed: %s %d %s", update_filename, errno, strerror(errno));
		update_error = -1;
		update_errorMsg = "file open failed";
		update_finished = 1;
		update_state = U_Idle;
		return;
	}
//	fseek(opened_file, 0, SEEK_SET);

	req = req_new(update_url); 
	req_setopt(req, REQ_SET_METHOD, "GET");
	req_setopt(req, REQ_FUNC_DOWNLOAD_CB, download_callback);
	ret = req_perform(req);
	if (ret/100 > 2) {
		ESP_LOGI(TAG, "%s download failed, error code: %d", update_url, ret);
	}
	req_clean(req);
	update_finished = 1;
	if (opened_file) {
		fclose(opened_file);
		opened_file = NULL;
	}
	if (update_url) {
		free(update_url);
		update_url = NULL;
	}
	if (update_filename) {
		free(update_filename);
		update_filename = NULL;
	}
	update_state = U_Idle;
	ESP_LOGI(TAG, "<< Done download");
}

int cgiSendVersion(HttpdConnData *connData)
{
	char v_str[128];
	// construct customer parameter
	sprintf(v_str, "{\"fw\":\"%s\"}", ESP32_VERSION);

	httpdStartResponse(connData, 200);
	httpdHeader(connData, "Cache-Control", "no-cache, no-store, must-revalidate");
	httpdHeader(connData, "Pragma", "no-cache");
	httpdHeader(connData, "Expires", "-1");
	httpdHeader(connData, "Content-Type", "application/json");
	httpdEndHeaders(connData);
	httpdSend(connData, v_str, strlen(v_str));
	return HTTPD_CGI_DONE;
}


// Download file from url
int cgiDownload(HttpdConnData *connData)
{
	char action[8];
	char url[256];
	char filename[64], file[32];
	if (update_state != U_Idle) {
        	// Busy
		httpdStartResponse(connData, 403);
		httpdEndHeaders(connData);
		return HTTPD_CGI_DONE;
	}
	if (-1 == httpdFindArg(connData->getArgs, "a", action, sizeof(action))) {
		httpdStartResponse(connData, 403);
		httpdEndHeaders(connData);
		return HTTPD_CGI_DONE;
	}
	if (-1 == httpdFindArg(connData->getArgs, "f", file, sizeof(file)-1)) {
		httpdStartResponse(connData, 403);
		httpdEndHeaders(connData);
		return HTTPD_CGI_DONE;
	}
	if (file[0] != '/') sprintf(filename, "/s/%s", file);
	else sprintf(filename, "/s%s", file);

	if (!strcasecmp(action,"add")) {
		if (-1 == httpdFindArg(connData->getArgs, "u", url, sizeof(url))) {
			httpdStartResponse(connData, 403);
			httpdEndHeaders(connData);
			return HTTPD_CGI_DONE;
		}
		update_url = strdup(url);
		update_filename = strdup(filename);
		update_error = 0;
		update_finished = 0;
		update_errorMsg = "";
		update_state = U_FileDownloadPending;
		httpdStartResponse(connData, 200);
		httpdHeader(connData, "Content-Type", "application/json");
		httpdHeader(connData, "Cache-Control", "no-cache, no-store, must-revalidate");
		httpdHeader(connData, "Pragma", "no-cache");
		httpdHeader(connData, "Expires", "-1");
		httpdEndHeaders(connData);
		httpdSend(connData, json_ok, strlen(json_ok));
		updateTimer=httpdPlatTimerCreate("download_task", 200, 0, download_task, NULL);
		httpdPlatTimerStart(updateTimer);

//		xTaskCreate(&download_task, "download_task", 4096, NULL, 5, NULL);
		return HTTPD_CGI_DONE;
	}
	httpdStartResponse(connData, 403);
	httpdEndHeaders(connData);
	return HTTPD_CGI_DONE;
}

// Получение статуса закачки файла с удаленного сервера
int cgiDownloadStatus(HttpdConnData *connData)
{
	char msg[128];
	if (update_state == U_Idle){
        	// Завершено
		httpdStartResponse(connData, 200);
		httpdHeader(connData, "Content-Type", "application/json");
		httpdEndHeaders(connData);
	        if (update_finished) {
			strcpy(msg, "{\"finished\":1,\"error\":0}");
		} else {
			sprintf(msg, "{\"finished\":1,\"error\":%d,\"msg\":\"%s\"}",
				update_error, update_errorMsg? update_errorMsg:"");
		}
		httpdSend(connData, msg, strlen(msg));
	} else if (update_state == U_FileDownloadPending || update_state == U_FileDownloading) {
		httpdStartResponse(connData, 200);
		httpdHeader(connData, "Content-Type", "application/json");
		httpdEndHeaders(connData);
		strcpy(msg, "{\"finished\":0,\"error\":0}");
		httpdSend(connData, msg, strlen(msg));
	}
	return HTTPD_CGI_DONE;
}

int cgiUpdate(HttpdConnData *connData) {
	sendState *state = connData->cgiData;
	int len;

	if (connData->conn==NULL) {
		//Connection aborted. Clean up.
		if (state!=NULL) free(state);
		return HTTPD_CGI_DONE;
	}

	if (state==NULL) {
		state=malloc(sizeof(sendState));
		connData->cgiData=state;
		state->stringPos=updateIndexHtml;
		httpdStartResponse(connData, 200);
		httpdHeader(connData, "Content-Type", "text/html");
		httpdHeader(connData, "Cache-Control", "no-cache, no-store, must-revalidate");
		httpdHeader(connData, "Pragma", "no-cache");
		httpdHeader(connData, "Expires", "-1");
		httpdEndHeaders(connData);
	}

	len=strlen(state->stringPos); //Get remaining length
	if (len>128) len=128; //Never send more than 128 bytes
	httpdSend(connData, state->stringPos, len);
	state->stringPos+=len;
	if (strlen(state->stringPos)!=0) {
		return HTTPD_CGI_MORE;
	}
	free(state);
	return HTTPD_CGI_DONE;
}


