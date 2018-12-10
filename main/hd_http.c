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

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs_flash.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <dirent.h>
#include <sys/unistd.h>
#include <ctype.h>
#include "esp_platform.h"
#include "esp_log.h"
#include "cgiflash.h"
#include "esp_log.h"
#include <cJSON.h>
#include "cgiflash.h"
#include "esp_spiffs.h"
#include "md5.h"
#include "httpd-platform.h"
#include "httpd.h"
#include "httpdespfs.h"
#include "cgiupdate.h"
#include "cgiwebsocket.h"
#include "config.h"
#include "hd_wifi.h"
#include "hd_main.h"
#include "ds.h"


char ha1[34];
typedef struct {
	char *text;
} wwwPage;

typedef struct {
	wwwPage *page;
	int part;
	void *tplArg;
	char token[64];
	int tokenPos;
} TplData;

/* Definition for Digest authorization */
struct http_digest {
        char username[80];
        char nonce[80];
        char uri[80];
        char realm[80];
        char domain[80];
        char response[80];
        char cnonce[80];
        char opaque[80];
        char nc[80];
        int qop;                /* Flag set to 1, if we send/recv qop="quth" */
};


static const char *json_ok = "{\"err\":\"0\",\"msg\":\"OK\"}";
static const char *json_err = "{\"err\":\"1\",\"msg\":\"Error\"}";
static const char *auth_html ="<!DOCTYPE HTML>\n"
"<html>\n<head>\n<title>401 Unauthorized</title>\n</head><body>\n"
"<h1>401 Unauthorized</h1>\n"
"<hr />\n"
"<address>ESP device</address>\n"
"</body></html>\n";

wwwPage WIFI_PAGE[] = {
	{"<!DOCTYPE html><html><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1, user-scalable=no\"/><title>wifi first setup</title>"},
	{"<style>.c{text-align: center;} div,input{padding:5px;font-size:1em;} input{width:95%%;} body{text-align: center;font-family:verdana;} button{border:0;border-radius:0.3rem;background-color:#1fa3ec;color:#fff;line-height:2.4rem;font-size:1.2rem;width:100%%;} .ssid{width: 128px;text-align: left;} .q{width: 64px;text-align: right;} .l{background: url(\"data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAACAAAAAgCAMAAABEpIrGAAAALVBMVEX///8EBwfBwsLw8PAzNjaCg4NTVVUjJiZDRUUUFxdiZGSho6OSk5Pg4eFydHTCjaf3AAAAZElEQVQ4je2NSw7AIAhEBamKn97/uMXEGBvozkWb9C2Zx4xzWykBhFAeYp9gkLyZE0zIMno9n4g19hmdY39scwqVkOXaxph0ZCXQcqxSpgQpONa59wkRDOL93eAXvimwlbPbwwVAegLS1HGfZAAAAABJRU5ErkJggg==\") no-repeat left center;background-size: 1em;}</style>"},
	{"<script>function c(l){document.getElementById('ssid').value=l.innerText||l.textContent;document.getElementById('pass').focus();}</script>"},
	{"</head><body><div style='text-align:left;display:inline-block;min-width:260px;'>"},
	{"<form action=\"/wifi0\" method=\"get\"><button>Rescan Wifi</button></form>"},
	{"<table><thead><tr><th>SSID</th><th>Sig.</th><th>Auth</th></tr></thead><tbody>\%scanwifi%</tbody></table>"},

	{"<form method='POST' action='/addwifi'><input id='ssid' name='ssid' length='32' placeholder='SSID' /><br /><input id='pass' name='pass' length='64' type='password' placeholder='password' /><br/>"},
	{"<input type='hidden' name='redirect' value='/wifiok' /><br/><button type='submit'>save</button></form>"},
	{"<form action=\"/r\" method=\"post\"><button>Reset</button></form>"},
	{"</div></body></html>"},
	{NULL}
};

vaiable_list DEFL_PARAMS[] =
{
	{"maxPower", VARIABLE_INT, 0, 15000},
	{"ustPowerReg", VARIABLE_INT, 0, 15000},
	{"tempEndRectRazgon", VARIABLE_FLOAT, 0, 120},
	{"powerRect", VARIABLE_INT, 0, 15000},
	{"tEndRectOtbGlv", VARIABLE_FLOAT, 0, 120},
	{"timeChimRectOtbGlv", VARIABLE_INT, 0, 10000},
	{"procChimOtbGlv", VARIABLE_INT, 0, 101},
	{"minProcChimOtbSR", VARIABLE_INT, 0, 101},
	{"tempStabSR", VARIABLE_FLOAT, 0, 120},
	{"beginProcChimOtbSR", VARIABLE_INT, 0, 100},
	{"timeChimRectOtbSR", VARIABLE_INT, 0, 1500},
	{"tempDeltaRect", VARIABLE_FLOAT, 0, 120},
	{"tempEndRectOtbSR", VARIABLE_FLOAT, 0, 120},
	{"tempEndRect", VARIABLE_FLOAT, 0, 120},
	{"p_MPX5010", VARIABLE_INT, 0, 100},
	{"timeStabKolonna", VARIABLE_INT, 0, 3500},
	{"timeRestabKolonna", VARIABLE_INT, 0, 3500},
	{"pShim", VARIABLE_INT, 0, 101},
	{"klpSilentNode", VARIABLE_CHECKBOX, 0, 1},
	{"urovenProvodimostSR", VARIABLE_INT, 0, 1000},
	{"cntCHIM", VARIABLE_INT, -100, 100},
	{"decrementCHIM", VARIABLE_INT, 0, 100},
	{"incrementCHIM", VARIABLE_INT, 0, 100},
	{"timeAutoIncCHIM", VARIABLE_INT, 0, 1000},
	{"alarmMPX5010", VARIABLE_INT, 0, 100},
	{"beepChangeState", VARIABLE_CHECKBOX, 0, 1},

	{"powerDistil", VARIABLE_INT, 0, 15000},
	{"tempEndDistil", VARIABLE_FLOAT, 0, 120},

	{NULL}
};


static const char *get_http_method(char method)
{
	if (method==HTTPD_METHOD_GET) return "GET";
	else if (method==HTTPD_METHOD_POST) return "POST";
	return "";
}
static inline char* skip_blanks(const char *str)
{
        while (*str && ((unsigned char) *str) < 33) str++;
        return (char *) str;
}

static inline char *skip_nonblanks(const char *str)
{
        while (*str && ((unsigned char) *str) > 32) str++;
        return (char *) str;
}


/*!
 * Parse digest authorization header.
 * Returns -1 if we have no auth or something wrong with digest.
 * This function may be used for Digest request and responce header.
 * request arg is set to nonzero, if we parse Digest Request.
 * pedantic arg can be set to nonzero if we need to do addition Digest check.
 */
int parse_digest(const char *digest, struct http_digest *d, int request, int pedantic)
{
	int i;
	char *c, key[40], val[80];

	if (!digest || !strlen(digest) || !d) return -1;

	c = skip_blanks(digest);
	if (strncasecmp(c, "Digest ", strlen("Digest "))) {
                ESP_LOGW(TAG, "Missing Digest");
                return -1;
        }
	c += strlen("Digest ");
	/* lookup for keys/value pair */
	while (*c && *(c = skip_blanks(c))) {
                /* find key */
                i = 0;
                while (*c && *c != '=' && *c != ',' && !isspace((int)*c)) key[i++] = *c++;
                key[i] = '\0';
                c = skip_blanks(c);
		if (*c == '=') {
			c = skip_blanks(++c);
			i = 0;
			if (*c == '\"') {
                                /* in quotes. Skip first and look for last */
                                c++;
                                while (*c && *c != '\"') {
                                        if (*c == '\\' && c[1] != '\0') { /* unescape chars */
                                                c++;
                                        }
                                        val[i++] = *c++;
                                }
                        } else {
                                /* token */
                                while (*c && *c != ',' && !isspace((int)*c)) {
                                        val[i++] = *c++;
                                }
                        }
                        val[i] = '\0';
		}
                while (*c && *c != ',') c++;
		if (*c) c++;
		if (!strlen(key) || !strlen(val) || strlen(val) > 79) {
                        return -1;
                }
		if (!strcasecmp(key, "username")) {
                        strcpy(d->username, val);
                } else if (!strcasecmp(key, "realm")) {
                        strcpy(d->realm, val);
                } else if (!strcasecmp(key, "nonce")) {
                        strcpy(d->nonce, val);
                } else if (!strcasecmp(key, "uri")) {
                        strcpy(d->uri, val);
                } else if (!strcasecmp(key, "domain")) {
                        strcpy(d->domain, val);
                } else if (!strcasecmp(key, "response")) {
                        strcpy(d->response, val);
                } else if (!strcasecmp(key, "algorithm")) {
			if (strcasecmp(val, "MD5")) {
                                ESP_LOGW(TAG, "http: Digest algorithm: \"%s\" not supported.", val);
                                return -1;
                        }
                } else if (!strcasecmp(key, "cnonce")) {
                        strcpy(d->cnonce, val);
                } else if (!strcasecmp(key, "opaque")) {
                        strcpy(d->opaque, val);
                } else if (!strcasecmp(key, "qop") && !strcasecmp(val, "auth")) {
                        d->qop = 1;
		} else if (!strcasecmp(key, "nc")) {
                        unsigned long u;
                        if (sscanf(val, "%30lx", &u) != 1) {
                                ESP_LOGW(TAG, "http: Incorrect Digest nc value: \"%s\".", val);
                                return -1;
                        }
                        strcpy(d->nc, val);
                }
	}

	/* "realm" and "nonce" MUST be always exist */
	if (!strlen(d->realm) || !strlen(d->nonce)) return -1;

	/* Additional check for Digest response */
	if (!strlen(d->username) || !strlen(d->uri) || !strlen(d->response)) return -1;
	if (d->qop && (!strlen(d->cnonce) || !d->nc)) return -1;
        return 0;
}

void md5_hash(char *output, const char *input)
{
	md5_context ctx;
        unsigned char digest[16];
        char *ptr;
        int x;

	md5_starts(&ctx);
        md5_update(&ctx, (const unsigned char *) input, strlen(input));
        md5_finish(&ctx, digest);
        ptr = output;
        for (x = 0; x < 16; x++) {
		ptr += sprintf(ptr, "%2.2x", digest[x]);
        }
}

/*
 * Check the user's password, return 1 if OK
 */
static int authorize(const struct http_digest *digest, char method)
{
        char ha2[34];
        char resp[256];
        char resp_hash[34]="";


	if (strcmp(digest->username, httpUser) && strcmp(digest->realm, CONFIG_REALM)) {
		return -1;
	}
	/* XXX  Due to a bug in MSIE, we do not compare the URI  */
        /* Also, we do not check for authentication timeout */
	if (/*strcmp(dig->uri, c->ouri) != 0 || */
            strlen(digest->response) != 32 /*||
            now - strtoul(dig->nonce, NULL, 10) > 3600 */)
                return (0);

        snprintf(resp, sizeof(resp), "%s:%s", get_http_method(method), digest->uri);
        md5_hash(ha2, resp);
	if (digest->qop) {
                /* RFC 2617 */
                snprintf(resp, sizeof(resp), "%s:%s:%s:%s:auth:%s", ha1, digest->nonce,
			digest->nc, digest->cnonce, ha2);
        }  else {
                /* RFC 2069 */
                snprintf(resp, sizeof(resp), "%s:%s:%s", ha1, digest->nonce, ha2);
        }
        md5_hash(resp_hash, resp);
        return (!memcmp(resp_hash, digest->response, 32));
}





/* Send http "401 Unauthorized" responce and close socket*/
static void send_http_auth(HttpdConnData *connData, const char *realm,
        const unsigned long nonce, const unsigned long opaque, int stale)
{
	char b[128];
	httpdStartResponse(connData, 401);
	snprintf(b, 128, "Digest algorithm=MD5, realm=\"%s\", nonce=\"%08lx\", qop=\"auth\", opaque=\"%08lx\"%s",
	realm, nonce, opaque, stale ? ", stale=true" : "");

	httpdHeader(connData, "WWW-authenticate", b);
	httpdHeader(connData, "Content-Type", "text/html");
	httpdEndHeaders(connData);
	httpdSend(connData, auth_html, strlen(auth_html));
}

/* send json responce headers */
static void send_json_headers(HttpdConnData *connData)
{
	httpdStartResponse(connData, 200);
	httpdHeader(connData, "Content-Type", "application/json");
	httpdHeader(connData, "Cache-Control", "no-cache, no-store, must-revalidate");
	httpdHeader(connData, "Pragma", "no-cache");
	httpdHeader(connData, "Expires", "-1");
	httpdEndHeaders(connData);
}


int checkAuth(HttpdConnData *connData)
{
	int r;
	struct http_digest d;
	int stale = 0;
        unsigned long nonce = 0, nc;

	char hdr[256];

	if (!httpSecure) return 0;

	r=httpdGetHeader(connData, "Authorization", hdr, sizeof(hdr));
	if (r && strncmp(hdr, "Digest", 6)) {
		ESP_LOGW(TAG, "Missing Digest field");
		nonce = 0;
		goto out_401;
	}
	memset(&d, 0, sizeof(struct http_digest));
	if (parse_digest(hdr, &d, 0, 1)) {
		/* Error in Digest - send new one */
		ESP_LOGW(TAG, "Error in Digest");
		nonce = 0;
		goto out_401;
	}
	if (sscanf(d.nc, "%30lx", &nc) != 1) {
		ESP_LOGW(TAG, "Received incorrect nonce in Digest");
		nonce = 0;
		goto out_401;
	}
	if (!authorize(&d, connData->requestType)) {
		ESP_LOGW(TAG, "Auth fail");
		nonce = 0;
		goto out_401;
	}

	return 0;
out_401:
	if (!nonce) nonce = esp_random();
	send_http_auth(connData, CONFIG_REALM, nonce, nonce, stale);
	return -1;
}

int handle204(HttpdConnData *connData)
{
	if (connData->conn==NULL) return HTTPD_CGI_DONE;
	httpdStartResponseEx(connData, 204, "No Response");
	httpdHeader(connData, "Cache-Control", "no-cache, no-store, must-revalidate");
	httpdHeader(connData, "Pragma", "no-cache");
	httpdHeader(connData, "Expires", "-1");
	httpdHeader(connData, "Content-Type", "text/plain");
	httpdEndHeaders(connData);
	httpdSend(connData, "", -1);
	return HTTPD_CGI_DONE;

}


int httpIndex(HttpdConnData *connData)
{
	struct stat st;
	const char *loc;	
	wifi_mode_t mode;

	if (connData->conn==NULL) return HTTPD_CGI_DONE;

	esp_wifi_get_mode(&mode);

	if (stat("/s/index.html", &st) == 0) loc = "/index.html";
	else if (WIFI_MODE_STA == mode) loc = "/update";
	else loc = "/wifi0";

	httpdStartResponseEx(connData, 302, "Redirect");
	httpdHeader(connData, "Location", loc);
	httpdEndHeaders(connData);
	httpdSend(connData, "Moved to ", -1);
	httpdSend(connData, loc, -1);
	return HTTPD_CGI_DONE;
}

int httpNetSetup(HttpdConnData *connData)
{
	if (connData->conn==NULL) return HTTPD_CGI_DONE;
	if (checkAuth(connData)) return HTTPD_CGI_DONE;

	send_json_headers(connData);

	if (connData->requestType == HTTPD_METHOD_GET) {
		cJSON *ja = cJSON_CreateObject();
		cJSON_AddItemToObject(ja, "host", cJSON_CreateString(Hostname?Hostname:""));
		cJSON_AddItemToObject(ja, "user", cJSON_CreateString(httpUser?httpUser:""));
		cJSON_AddItemToObject(ja, "pass", cJSON_CreateString(httpPassword?httpPassword:""));
		cJSON_AddItemToObject(ja, "secure", cJSON_CreateNumber(httpSecure));
		cJSON_AddItemToObject(ja, "smscUser", cJSON_CreateString(smscUser?smscUser:""));
		cJSON_AddItemToObject(ja, "smscHash", cJSON_CreateString(smscHash?smscHash:""));
		cJSON_AddItemToObject(ja, "smscPhones", cJSON_CreateString(smscPhones?smscPhones:""));
		cJSON_AddItemToObject(ja, "useSmsc", cJSON_CreateNumber(useSmsc));
		char *r=cJSON_Print(ja);
		httpdSend(connData, r, strlen(r));
		if (r) free(r);
		cJSON_Delete(ja);
        } else if (connData->requestType == HTTPD_METHOD_POST) {
		char user[33], pass[33], host[82], su[33]="", sh[82]="", sp[82]="";
		int secure=0, usesmsc=0;
		if (httpdFindArg(connData->post->buff, "secure", user, sizeof(user)) >0) {
			secure = atoi(user);
		}		
		if (httpdFindArg(connData->post->buff, "useSmsc", user, sizeof(user)) >0) {
			usesmsc = atoi(user);
		}		

		httpdFindArg(connData->post->buff, "host", host, sizeof(host));
		httpdFindArg(connData->post->buff, "user", user, sizeof(user));
		httpdFindArg(connData->post->buff, "pass", pass, sizeof(pass));
		httpdFindArg(connData->post->buff, "smscUser", su, sizeof(su));
		httpdFindArg(connData->post->buff, "smscHash", sh, sizeof(sh));
		httpdFindArg(connData->post->buff, "smscPhones", sp, sizeof(sp));

		if (!set_network_config(host,user,pass,secure,su,sh,sp,usesmsc)) {
			// recalculate h1
			char b[128];
			snprintf(b, sizeof(b), "%s:%s:%s", httpUser, CONFIG_REALM, httpPassword);
			md5_hash(ha1, b);
			httpdSend(connData, json_ok, strlen(json_ok));
		} else {
			httpdSend(connData, json_err, strlen(json_err));
		}
	}
	return HTTPD_CGI_DONE;
}


/* —брос параметров в значение по умолчанию */
int httpParamDefault(HttpdConnData *connData)
{
	if (connData->conn==NULL) return HTTPD_CGI_DONE;
	if (checkAuth(connData)) return HTTPD_CGI_DONE;

	send_json_headers(connData);
	param_default();
	httpdSend(connData, json_ok, strlen(json_ok));
	return HTTPD_CGI_DONE;
}


int httpParamSetup(HttpdConnData *connData)
{
	if (connData->conn==NULL) return HTTPD_CGI_DONE;
	if (checkAuth(connData)) return HTTPD_CGI_DONE;

	send_json_headers(connData);

	if (connData->requestType == HTTPD_METHOD_POST) {
		FILE *f = fopen(RECT_CONFIGURATION, "w");
		if (f) {
			int  i;
			double d;
			char param[82];
			vaiable_list *v;
			cJSON *ja = cJSON_CreateObject();
			for (v = DEFL_PARAMS; v && v->name; v++) {
				if (httpdFindArg(connData->post->buff, v->name, param, sizeof(param))>=0) {
					switch (v->type) {
					case VARIABLE_CHECKBOX:
						i = atoi(param);
						cJSON_AddItemToObject(ja, v->name, cJSON_CreateBool(i));
						break;
					case VARIABLE_STRING:
						cJSON_AddItemToObject(ja, v->name, cJSON_CreateString(param));
						break;
					case VARIABLE_INT:
						i = atoi(param);
						if (i >= v->min && i < v->max) {
							cJSON_AddItemToObject(ja, v->name, cJSON_CreateNumber(i));
						}
						break;
					case VARIABLE_FLOAT:
						d = atof(param);
						if (d >= v->min && d < v->max) {
							cJSON_AddItemToObject(ja, v->name, cJSON_CreateNumber(d));
						}
						break;

					}
				}	
			}
			char *r=cJSON_Print(ja);
			fprintf(f, "%s", r);
			if (r) free(r);
			cJSON_Delete(ja);
			fclose(f);
		}
		httpdSend(connData, json_ok, strlen(json_ok));
	}
	return HTTPD_CGI_DONE;
}

int httpSetPower(HttpdConnData *connData)
{
	if (connData->conn==NULL) return HTTPD_CGI_DONE;
	if (checkAuth(connData)) return HTTPD_CGI_DONE;

	send_json_headers(connData);

	if (connData->requestType == HTTPD_METHOD_POST) {
		char ps[10];

		httpdFindArg(connData->post->buff, "power", ps, sizeof(ps));
		setPower(atoi(ps));
	}
	httpdSend(connData, json_ok, strlen(json_ok));
	return HTTPD_CGI_DONE;
}

int httpSetMainMode(HttpdConnData *connData)
{
	if (connData->conn==NULL) return HTTPD_CGI_DONE;
	if (checkAuth(connData)) return HTTPD_CGI_DONE;

	send_json_headers(connData);
	if (connData->requestType == HTTPD_METHOD_POST) {
		char nm[10]="0";
		httpdFindArg(connData->post->buff, "new_mode", nm, sizeof(nm));
		setMainMode(atoi(nm));
	}
	httpdSend(connData, json_ok, strlen(json_ok));
	return HTTPD_CGI_DONE;
}

int httpSetStatus(HttpdConnData *connData)
{
	if (connData->conn==NULL) return HTTPD_CGI_DONE;
	if (checkAuth(connData)) return HTTPD_CGI_DONE;

	send_json_headers(connData);
	if (connData->requestType == HTTPD_METHOD_POST) {
		char nm[10]="0";
		httpdFindArg(connData->post->buff, "new", nm, sizeof(nm));
		setStatus(atoi(nm));
	}
	httpdSend(connData, json_ok, strlen(json_ok));
	return HTTPD_CGI_DONE;
}


int httpStartProcess(HttpdConnData *connData)
{
	if (connData->conn==NULL) return HTTPD_CGI_DONE;
	if (checkAuth(connData)) return HTTPD_CGI_DONE;
	send_json_headers(connData);
	MainStatus = PROC_START;
	httpdSend(connData, json_ok, strlen(json_ok));
	return HTTPD_CGI_DONE;

}

int httpEndProcess(HttpdConnData *connData)
{
	if (connData->conn==NULL) return HTTPD_CGI_DONE;
	if (checkAuth(connData)) return HTTPD_CGI_DONE;
	send_json_headers(connData);
	MainStatus = PROC_END;
	httpdSend(connData, json_ok, strlen(json_ok));
	return HTTPD_CGI_DONE;
}

int httpKlpStatus(HttpdConnData *connData)
{
	cJSON *ja;
	char id[10];
	int i;

	if (connData->conn==NULL) return HTTPD_CGI_DONE;
	if (checkAuth(connData)) return HTTPD_CGI_DONE;
	send_json_headers(connData);
	ja = cJSON_CreateObject();

	httpdFindArg(connData->getArgs, "id", id, sizeof(id));
	i = atoi(id);
	if (i < MAX_KLP) {
		int pwm = Klp[i].open_time+Klp[i].close_time;
		float pwm_percent = 0;
		if (Klp[i].open_time>0) {
			float p = pwm/Klp[i].open_time;
			if (p) pwm_percent = 100/p;
		}
		cJSON_AddItemToObject(ja, "id", cJSON_CreateNumber(i));
		cJSON_AddItemToObject(ja, "is_pwm", cJSON_CreateNumber(Klp[i].is_pwm));
		cJSON_AddItemToObject(ja, "is_open", cJSON_CreateNumber(Klp[i].is_open));
		cJSON_AddItemToObject(ja, "pwm_time", cJSON_CreateNumber(pwm));
		cJSON_AddItemToObject(ja, "pwm_percent", cJSON_CreateNumber(pwm_percent));
		cJSON_AddItemToObject(ja, "err", cJSON_CreateNumber(0));
		cJSON_AddItemToObject(ja, "msg", cJSON_CreateString("OK"));
	}
	char *r=cJSON_Print(ja);
	httpdSend(connData, r, strlen(r));
	if (r) free(r);
	cJSON_Delete(ja);

	return HTTPD_CGI_DONE;
}

int httpKlpOn(HttpdConnData *connData)
{
	if (connData->conn==NULL) return HTTPD_CGI_DONE;
	if (checkAuth(connData)) return HTTPD_CGI_DONE;
	send_json_headers(connData);
	if (connData->requestType == HTTPD_METHOD_POST) {
		char id[10];
		httpdFindArg(connData->post->buff, "id", id, sizeof(id));
		openKlp(atoi(id));
	}
	httpdSend(connData, json_ok, strlen(json_ok));
	return HTTPD_CGI_DONE;

}

int httpKlpOff(HttpdConnData *connData)
{
	if (connData->conn==NULL) return HTTPD_CGI_DONE;
	if (checkAuth(connData)) return HTTPD_CGI_DONE;
	send_json_headers(connData);
	if (connData->requestType == HTTPD_METHOD_POST) {
		char id[10];
		httpdFindArg(connData->post->buff, "id", id, sizeof(id));
		closeKlp(atoi(id));
	}
	httpdSend(connData, json_ok, strlen(json_ok));
	return HTTPD_CGI_DONE;
}

int httpKlpShimOn(HttpdConnData *connData)
{
	if (connData->conn==NULL) return HTTPD_CGI_DONE;
	if (checkAuth(connData)) return HTTPD_CGI_DONE;
	send_json_headers(connData);
	if (connData->requestType == HTTPD_METHOD_POST) {
		float topen = 0, period, percent;
		char id[10], p[10]="0", k[10]="0";
		httpdFindArg(connData->post->buff, "id", id, sizeof(id));
		httpdFindArg(connData->post->buff, "period", p, sizeof(p));
		httpdFindArg(connData->post->buff, "klp_on", k, sizeof(k));
		period = atoi(p);
		percent = atoi(k);
		if (percent>100) percent = 100;
		if(period && percent) {
			topen = period/100*percent;
			if (topen > 0) startKlpPwm(atoi(id), topen, period-topen);
		}
	}

	httpdSend(connData, json_ok, strlen(json_ok));
	return HTTPD_CGI_DONE;
}

int httpListSensor(HttpdConnData *connData)
{
	cJSON *ja, *j;
	DS18 *d;

	if (connData->conn==NULL) return HTTPD_CGI_DONE;
	if (checkAuth(connData)) return HTTPD_CGI_DONE;

	send_json_headers(connData);

	ja = cJSON_CreateArray();
	for (int i=0; i<MAX_DS; i++) {
		d = &ds[i];
		if (!d->is_connected) continue;
		j = cJSON_CreateObject();
		cJSON_AddItemToArray(ja, j);
		cJSON_AddItemToObject(j, "id", cJSON_CreateNumber(i));
		cJSON_AddItemToObject(j, "rom", cJSON_CreateString(d->adressStr));
		cJSON_AddItemToObject(j, "descr", cJSON_CreateString(d->description?d->description:""));
		cJSON_AddItemToObject(j, "type_str", cJSON_CreateString(getDsTypeStr(d->type)));
		cJSON_AddItemToObject(j, "type", cJSON_CreateNumber(d->type));
		cJSON_AddItemToObject(j, "corr", cJSON_CreateNumber(d->corr));
		cJSON_AddItemToObject(j, "talert", cJSON_CreateNumber(d->talert));
		cJSON_AddItemToObject(j, "temp", cJSON_CreateNumber(d->Ce));

	}

	char *r=cJSON_Print(ja);
	httpdSend(connData, r, strlen(r));
	if (r) free(r);
	cJSON_Delete(ja);
	return HTTPD_CGI_DONE;
}

int httpUpdateSensor(HttpdConnData *connData)
{
	DS18 *d;
	if (connData->conn==NULL) return HTTPD_CGI_DONE;
	if (checkAuth(connData)) return HTTPD_CGI_DONE;

	send_json_headers(connData);

	if (connData->requestType == HTTPD_METHOD_POST) {
		struct stat st;
		uint8_t id;
		FILE *f;
		char descr[81];
		cJSON *n = NULL, *root = NULL;

		httpdFindArg(connData->post->buff, "id", descr, sizeof(descr));
		id = atoi(descr);

		if(id>=MAX_DS) {
			httpdSend(connData, json_err, -1);
			return HTTPD_CGI_DONE;
		}
		d = &ds[id];

		httpdFindArg(connData->post->buff, "type", descr, sizeof(descr));
		if (strlen(descr)) d->type = (DsType) atoi(descr);
		httpdFindArg(connData->post->buff, "corr", descr, sizeof(descr));
		if (strlen(descr)) d->corr = atof(descr);
		httpdFindArg(connData->post->buff, "talert", descr, sizeof(descr));
		if (strlen(descr)) d->talert = atof(descr);
		httpdFindArg(connData->post->buff, "descr", descr, sizeof(descr));
		if (strlen(descr)) {
			if (d->description) free(d->description);
			d->description = strdup(descr);
		}
			

		n=cJSON_CreateObject();	
		cJSON_AddItemToObject(n, "rom", cJSON_CreateString(d->adressStr));
		cJSON_AddItemToObject(n, "type", cJSON_CreateNumber(d->type));
		cJSON_AddItemToObject(n, "corr", cJSON_CreateNumber(d->corr));
		cJSON_AddItemToObject(n, "talert", cJSON_CreateNumber(d->talert));
		cJSON_AddItemToObject(n, "descr", cJSON_CreateString(d->description?d->description:""));

		if (stat(SENS_CONFIGURATION, &st) == 0) {
			char *buff = NULL;
			f = fopen(SENS_CONFIGURATION, "r");
			if (f) {
				int size;
				buff = malloc(st.st_size+2);
				size = fread(buff, 1, st.st_size, f);
				buff[size]=0;
				fclose(f);
			}

			if (buff) {
				root = cJSON_Parse(buff);
				free(buff);
        		}
			if (root) {
				int size=cJSON_GetArraySize(root);
				// Check for dublication
				bool dupe = false;
				for (int i=0;i<size;i++) {
					cJSON *sens=cJSON_GetArrayItem(root,i);
					if (sens) {
						cJSON *s = cJSON_GetObjectItem(sens, "rom");
						if (s && s->valuestring && !strcmp(s->valuestring, d->adressStr)) {
							dupe = true;
							cJSON_ReplaceItemInArray(root, i, n);
							break;
						}       
					}
				}
				if (!dupe) {
					cJSON_AddItemToArray(root, n);
				}
			}
		} else {
			// New file
			root = cJSON_CreateArray();
			cJSON_AddItemToArray(root, n);
			
		}
		if (root) {
			char *r=cJSON_Print(root);
			f = fopen(SENS_CONFIGURATION, "w");
			if (r) {
				fprintf(f, "%s", r);
				free(r);
			}			
			fclose(f);
			cJSON_Delete(root);
		}
	}
	httpdSend(connData, json_ok, -1);
	return HTTPD_CGI_DONE;
}

int httpRescanSensor(HttpdConnData *connData)
{
	if (connData->conn==NULL) return HTTPD_CGI_DONE;
	if (checkAuth(connData)) return HTTPD_CGI_DONE;

	send_json_headers(connData);
	ds_init(0, NULL);
	httpdSend(connData, json_ok, -1);
	return HTTPD_CGI_DONE;
}

int httpMainInfo(HttpdConnData *connData)
{
        /* SPIFFS data */
	if (connData->conn==NULL) return HTTPD_CGI_DONE;
	if (checkAuth(connData)) return HTTPD_CGI_DONE;

	send_json_headers(connData);
	cJSON *ja = getInformation();
	char *r=cJSON_Print(ja);
	httpdSend(connData, r, strlen(r));
	cJSON_Delete(ja);
	if (r) free(r);
	return HTTPD_CGI_DONE;
}

int httpSysinfo(HttpdConnData *connData)
{
        /* SPIFFS data */
	size_t SPIFFS_total = 0, SPIFFS_used = 0;

	if (connData->conn==NULL) return HTTPD_CGI_DONE;
	if (checkAuth(connData)) return HTTPD_CGI_DONE;

	esp_spiffs_info(NULL, &SPIFFS_total, &SPIFFS_used);

	send_json_headers(connData);

	cJSON *ja = cJSON_CreateObject();
	cJSON_AddItemToObject(ja, "version", cJSON_CreateString(ESP32_VERSION));
	cJSON_AddItemToObject(ja, "totalBytes", cJSON_CreateNumber(SPIFFS_total));
	cJSON_AddItemToObject(ja, "usedBytes", cJSON_CreateNumber(SPIFFS_used));
	cJSON_AddItemToObject(ja, "heap", cJSON_CreateNumber(esp_get_free_heap_size()));
	char *r=cJSON_Print(ja);
	httpdSend(connData, r, strlen(r));
	cJSON_Delete(ja);
	if (r) free(r);
	return HTTPD_CGI_DONE;
}

int httpDirList(HttpdConnData *connData)
{
	cJSON *ja, *j;
	DIR *dir;
	struct dirent *dent;
	struct stat st;

	if (connData->conn==NULL) return HTTPD_CGI_DONE;
	if (checkAuth(connData)) return HTTPD_CGI_DONE;

	send_json_headers(connData);

	ja = cJSON_CreateArray();
	dir = opendir("/s");
	if (dir) {

		while ((dent=readdir(dir))) {
			char f[80];
			snprintf(f, sizeof(f), "/s/%s", dent->d_name);
			j = cJSON_CreateObject();
			cJSON_AddItemToArray(ja, j);
			cJSON_AddItemToObject(j, "name", cJSON_CreateString(dent->d_name));
			if (stat(f, &st) == 0) {
				cJSON_AddItemToObject(j, "size", cJSON_CreateNumber(st.st_size));
			}
		}	
		closedir(dir);
	}

	char *r=cJSON_Print(ja);
	httpdSend(connData, r, strlen(r));
	cJSON_Delete(ja);
	if (r) free(r);
	return HTTPD_CGI_DONE;
}

int httpRmFile(HttpdConnData *connData)
{
	struct stat st;

	if (connData->requestType == HTTPD_METHOD_POST) {
		char f[85];
		f[0] = '/'; f[1]='s'; f[2] = '/';
		httpdFindArg(connData->post->buff, "file", &f[3], sizeof(f)-3);
		if (strlen(f)>0 && stat(f, &st) == 0) {
			// Delete it if it exists
			unlink(f);
		}
	}
	send_json_headers(connData);
	httpdSend(connData, json_ok, strlen(json_ok));
	return HTTPD_CGI_DONE;
}


int httpScanWiFi(HttpdConnData *connData)
{
	cJSON *ja, *j;
	wifi_ap_record_t *w;

	if (connData->conn==NULL) return HTTPD_CGI_DONE;
	if (checkAuth(connData)) return HTTPD_CGI_DONE;

	send_json_headers(connData);
	ja = cJSON_CreateArray();

	if (scanWifiNetworks(false, true, false, 300) >=0) {	
		for (int i = 0; i < WIFI_scanCount && i < 16; ++i) {
			w = &WIFI_scanResult[i];
			j = cJSON_CreateObject();
			cJSON_AddItemToArray(ja, j);
			cJSON_AddItemToObject(j, "ssid", cJSON_CreateString((const char*)w->ssid));
			cJSON_AddNumberToObject(j, "rssi", w->rssi);
			cJSON_AddNumberToObject(j, "primary", w->primary);
			cJSON_AddItemToObject(j, "authmode", cJSON_CreateString(wifi_auth_string(w->authmode)));
		}
	}
	char *r=cJSON_Print(ja);
	httpdSend(connData, r, strlen(r));
	cJSON_Delete(ja);
	if (r) free(r);
	return HTTPD_CGI_DONE;
}

int httpGetWiFiCfg(HttpdConnData *connData)
{
	FILE *file=connData->cgiData;
	int len;
	char buff[128];

	if (connData->conn==NULL) {
		if(file) fclose(file);
		return HTTPD_CGI_DONE;
	}
	if (checkAuth(connData)) return HTTPD_CGI_DONE;

	if (file==NULL) {
		file = fopen(WIFI_CONFIGURATION, "r");
		if (!file) {
			send_json_headers(connData);
			httpdSend(connData, "{}", 2);
			return HTTPD_CGI_DONE;
		}
		connData->cgiData=file;
		send_json_headers(connData);
	}

	len=fread(buff, 1, 128, file);
	if (len>0) httpdSend(connData, buff, len);
	if (len!=128) {
		//We're done.
		fclose(file);
		return HTTPD_CGI_DONE;
	}
	return HTTPD_CGI_MORE;
}


int httpAddWiFi(HttpdConnData *connData)
{
	char redirect[33];

	if (connData->conn==NULL) return HTTPD_CGI_DONE;
	if (checkAuth(connData)) return HTTPD_CGI_DONE;

	httpdStartResponse(connData, 200);

	if (connData->requestType == HTTPD_METHOD_POST) {
		bool dupe=false;
		struct stat st;
		FILE *f;
		char ssid[33], pass[33];
		cJSON *newssid = NULL, *root = NULL;

		httpdFindArg(connData->post->buff, "ssid", ssid, sizeof(ssid));
		httpdFindArg(connData->post->buff, "pass", pass, sizeof(pass));
		if (strlen(ssid)>0 && strlen(pass)>0) {
			newssid=cJSON_CreateObject();	
			cJSON_AddItemToObject(newssid, "ssid", cJSON_CreateString(ssid));
			cJSON_AddItemToObject(newssid, "pass", cJSON_CreateString(pass));
		}

		if (newssid && stat(WIFI_CONFIGURATION, &st) == 0) {
			char *buff = NULL;
			f = fopen(WIFI_CONFIGURATION, "r");
			if (f) {
				int size;
				buff = malloc(st.st_size+2);
				size = fread(buff, 1, st.st_size, f);
				buff[size]=0;
				fclose(f);
			}

			if (buff) {
				root = cJSON_Parse(buff);
				free(buff);
        		}
			if (root) {
				int size=cJSON_GetArraySize(root);
				// Check for dublication
				for (int i=0;i<size;i++) {
					cJSON *ap=cJSON_GetArrayItem(root,i);
					if (ap) {
						cJSON *s = cJSON_GetObjectItem(ap, "ssid");
						if (s && s->valuestring) {
							if (!strcasecmp(s->valuestring, ssid)) {
								cJSON_ReplaceItemInArray(root, i, newssid);
								dupe = true;
								break;
							}
						}       
					}
				}
				if (!dupe) {
					if (size>=10) cJSON_ReplaceItemInArray(root, 9, newssid);
					else cJSON_AddItemToArray(root, newssid);
				}
			}
		} else if (newssid) {
			// New file
			root = cJSON_CreateArray();
			cJSON_AddItemToArray(root, newssid);			
			
		}
		if (root) {
			char *r=cJSON_Print(root);
			f = fopen(WIFI_CONFIGURATION, "w");
			if (r) {
				fprintf(f, "%s", r);
				free(r);
			}			
			fclose(f);
			cJSON_Delete(root);
		}

		if (httpdFindArg(connData->post->buff, "redirect", redirect, sizeof(redirect)) > 0) {
			httpdHeader(connData, "Content-Type", "text/html");
			httpdEndHeaders(connData);
			httpdSend(connData, "<html><head></head><body><div>Credentials Saved<br />Trying to connect ESP to network after reset.<br />If it fails reconnect to AP to try again.</div></body></html>", -1);
			return HTTPD_CGI_DONE;
		}
	}

	send_json_headers(connData);
	httpdSend(connData, json_ok, strlen(json_ok));
	return HTTPD_CGI_DONE;
}


int httpDeleteWiFi(HttpdConnData *connData)
{
	if (connData->conn==NULL) return HTTPD_CGI_DONE;
	if (checkAuth(connData)) return HTTPD_CGI_DONE;

	send_json_headers(connData);

	if (connData->requestType == HTTPD_METHOD_POST) {
		FILE *f;
		struct stat st;
		char *buff = NULL;
		cJSON *root = NULL;
		char ssid[33];

		httpdFindArg(connData->post->buff, "ssid", ssid, sizeof(ssid));
		if (strlen(ssid)>0 && stat(WIFI_CONFIGURATION, &st) == 0) {
			f = fopen(WIFI_CONFIGURATION, "r");
			if (f) {
				int size;
				buff = malloc(st.st_size+2);
				size = fread(buff, 1, st.st_size, f);
				buff[size]=0;
				fclose(f);
			}

			if (buff) {
				root = cJSON_Parse(buff);
				free(buff);
	       		}
			if (root) {
				int size=cJSON_GetArraySize(root);
				// Check for dublication
				for (int i=0;i<size;i++) {
					cJSON *ap=cJSON_GetArrayItem(root,i);
					if (ap) {
						cJSON *s = cJSON_GetObjectItem(ap, "ssid");
						if (s && s->valuestring) {
							if (!strcasecmp(s->valuestring, ssid)) {
								cJSON_DetachItemFromArray(root, i);
								break;
							}
						}       
					}
				}
			}
		}

		if (root) {
			char *r=cJSON_Print(root);
			f = fopen(WIFI_CONFIGURATION, "w");
			if (r) {
				fprintf(f, "%s", r);
				free(r);
			}			
			fclose(f);
			cJSON_Delete(root);
		}
	}
	httpdSend(connData, json_ok, strlen(json_ok));
	return HTTPD_CGI_DONE;
}


//This is a catch-all cgi function. It takes the url passed to it, looks up the corresponding
//path in the filesystem and if it exists, passes the file through. This simulates what a normal
//webserver would do with static files.
int httpDefault(HttpdConnData *connData) {
	FILE *file=connData->cgiData;
	int len;
	char buff[512];
	
	if (connData->conn==NULL) {
		//Connection aborted. Clean up.
		if(file) fclose(file);
		return HTTPD_CGI_DONE;
	}

	//First call to this cgi.
	if (file==NULL) {
		char f[128];
		if (checkAuth(connData)) return HTTPD_CGI_DONE;
		//Open the file so we can read it.
		snprintf(f, sizeof(f), "/s%s", connData->url);
		file = fopen(f, "r");
		if (!file) {
			if (strstr(connData->url, "index")) {
				httpdStartResponse(connData, 302);
				httpdHeader(connData, "Location", "/");
				httpdEndHeaders(connData);
				httpdSend(connData, "Moved to ", -1);
				httpdSend(connData, "/", -1);
				return HTTPD_CGI_DONE;
			}		
			return HTTPD_CGI_NOTFOUND;
		}

		// The gzip checking code is intentionally without #ifdefs because checking
		// for FLAG_GZIP (which indicates gzip compressed file) is very easy, doesn't
		// mean additional overhead and is actually safer to be on at all times.
		// If there are no gzipped files in the image, the code bellow will not cause any harm.

		connData->cgiData=file;
		httpdStartResponse(connData, 200);
		httpdHeader(connData, "Cache-Control", "no-cache, no-store, must-revalidate");
		httpdHeader(connData, "Pragma", "no-cache");
		httpdHeader(connData, "Expires", "-1");
        	httpdHeader(connData, "Content-Type", httpdGetMimetype(connData->url));
//		if (isGzip) {
//			httpdHeader(connData, "Content-Encoding", "gzip");
//		}
		httpdEndHeaders(connData);
		return HTTPD_CGI_MORE;
	}

	len=fread(buff, 1, sizeof(buff), file);
	if (len>0) httpdSend(connData, buff, len);
	if (len != sizeof(buff)) {
		fclose(file);
		return HTTPD_CGI_DONE;
	} else {
		//Ok, till next time.
		return HTTPD_CGI_MORE;
	}
}


void TplCallback(HttpdConnData *connData, char *token, void **arg)
{
	if (connData->conn==NULL || token==NULL) return;
	if (!strcmp(token,"scanwifi")) {
		if (scanWifiNetworks(false, true, false, 300) >=0) {	
			for (int i = 0; i < WIFI_scanCount && i < 16; ++i) {
				wifi_ap_record_t *w = &WIFI_scanResult[i];
				char buff[150];
				snprintf(buff, sizeof(buff)-1, "<tr><td><a href='#p' onclick='c(this); return false;'>%s</a></td><td>%dDb</td><td class='q'>%s</td></tr>",
					w->ssid, w->rssi, wifi_auth_string(w->authmode));
					httpdSend(connData, buff, strlen(buff));
			}
		}
	}
}

int httpTemplate(HttpdConnData *connData)
{
	TplData *tpd=connData->cgiData;
	int len;
	int x, sp=0;
	char *e=NULL, *buff;
	if (connData->conn==NULL) {
		//Connection aborted. Clean up.
		TplCallback(connData, NULL, &tpd->tplArg);
		free(tpd);
		return HTTPD_CGI_DONE;
	}

	if (!tpd) {
		//First call to this cgi. Open the file so we can read it.
		tpd=(TplData *)malloc(sizeof(TplData));
		if (tpd==NULL) return HTTPD_CGI_NOTFOUND;
		tpd->page = (wwwPage*) connData->cgiArg;
		tpd->part = 0;
		tpd->tplArg = NULL;
		tpd->tokenPos = -1;
		connData->cgiData=tpd;
		httpdStartResponse(connData, 200);
		httpdHeader(connData, "Cache-Control", "no-cache, no-store, must-revalidate");
		httpdHeader(connData, "Pragma", "no-cache");
		httpdHeader(connData, "Expires", "-1");
        	httpdHeader(connData, "Content-Type", httpdGetMimetype(connData->url));
		httpdEndHeaders(connData);
		return HTTPD_CGI_MORE;
	}

	buff = tpd->page[tpd->part].text;
	if (!buff) {
		//We're done.
		TplCallback(connData, NULL, &tpd->tplArg);
		free(tpd);
		return HTTPD_CGI_DONE;
	}

	len = strlen(buff);
	e = buff;
	for (x=0; x<len; x++) {
		if (tpd->tokenPos==-1) {
			//Inside ordinary text.
			if (buff[x]=='%') {
				//Send raw data up to now
				if (sp!=0) httpdSend(connData, e, sp);
				sp=0;
				//Go collect token chars.
				tpd->tokenPos=0;
			} else {
				sp++;
			}
		} else {
			if (buff[x]=='%') {
				if (tpd->tokenPos==0) {
					//This is the second % of a %% escape string.
					//Send a single % and resume with the normal program flow.
					httpdSend(connData, "%", 1);
				} else {
					//This is an actual token.

					tpd->token[tpd->tokenPos++]=0; //zero-terminate token
					TplCallback(connData, tpd->token, &tpd->tplArg);
				}
				//Go collect normal chars again.
				e=&buff[x+1];
				tpd->tokenPos=-1;
			} else {
				if (tpd->tokenPos<(sizeof(tpd->token)-1)) tpd->token[tpd->tokenPos++]=buff[x];
			}
		}
	}
        if (sp!=0) httpdSend(connData, e, sp);
	tpd->part++;
	return HTTPD_CGI_MORE;
}


static HttpdPlatTimerHandle resetTimer;

static void resetTimerCb(void *arg)
{
	ESP_LOGI(TAG, "Restarting");
	esp_restart();
}


// Handle request to reboot into the new firmware
int httpReset(HttpdConnData *connData)
{

	if (connData->conn==NULL) return HTTPD_CGI_DONE;

	//Do reboot in a timer callback so we still have time to send the response.
	resetTimer=httpdPlatTimerCreate("flashreset", 200, 0, resetTimerCb, NULL);
	httpdPlatTimerStart(resetTimer);

	httpdStartResponse(connData, 200);
	httpdHeader(connData, "Content-Type", "text/plain");
	httpdEndHeaders(connData);
	httpdSend(connData, "Rebooting...", -1);
	return HTTPD_CGI_DONE;
}


int httpSms(HttpdConnData *connData)
{

	if (connData->conn==NULL) return HTTPD_CGI_DONE;
	send_json_headers(connData);
	sendSMS("test sms");
	httpdSend(connData, json_ok, strlen(json_ok));
	return HTTPD_CGI_DONE;
}



void WebsocketConnected(Websock *ws)
{

}

	// Start http server
HttpdBuiltInUrl builtInUrls[]={
	{"/generate_204", handle204, NULL},
	{"/connecttest.txt", cgiRedirect, "/"},
	{"/success.txt", cgiRedirect, "/"},
	{"/", httpIndex, NULL},
	{"/update", cgiUpdate, NULL},
	{"/version", cgiSendVersion, NULL},
	{"/dl", cgiDownload, NULL},
	{"/dl_status", cgiDownloadStatus, NULL},
	{"/netcfg", httpNetSetup, NULL},

	{"/defparam", httpParamDefault, NULL},
	{"/saveparam", httpParamSetup, NULL},
	{"/setpower", httpSetPower, NULL},
	{"/setmainmode", httpSetMainMode, NULL},
	{"/set_status", httpSetStatus, NULL},
	{"/startprocess", httpStartProcess, NULL},
	{"/endprocess", httpEndProcess, NULL},
	{"/klp_status", httpKlpStatus, NULL},
	{"/klp_on", httpKlpOn, NULL},
	{"/klp_off", httpKlpOff, NULL},
	{"/klp_shim_on", httpKlpShimOn, NULL},

	{"/senslist", httpListSensor, NULL},
	{"/updatesens", httpUpdateSensor, NULL},
	{"/rescansens", httpRescanSensor, NULL},

	{"/maininfo", httpMainInfo, NULL},
	{"/sysinfo", httpSysinfo, NULL},

	{"/list", httpDirList, NULL},
	{"/rm", httpRmFile, NULL},
	{"/wifi", httpScanWiFi, NULL},
	{"/wificfg", httpGetWiFiCfg, NULL},
	{"/addwifi", httpAddWiFi, NULL},
	{"/delwifi", httpDeleteWiFi, NULL},

	{"/wifi0", httpTemplate, WIFI_PAGE},
	{"/r", httpReset, NULL},
	{"/sms", httpSms, NULL},

	{"/ws", cgiWebsocket, WebsocketConnected},
	{"*", httpDefault, NULL},
	{NULL, NULL, NULL}
};

int hd_httpd_init(void)
{
	char b[128];
	snprintf(b, sizeof(b), "%s:%s:%s", httpUser, CONFIG_REALM, httpPassword);
	md5_hash(ha1, b);
	httpdInit(builtInUrls, 80);
	ESP_LOGI(TAG, "Httpd server start");
	return 0;
}