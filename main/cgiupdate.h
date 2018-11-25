#ifndef CGIUPDATE_H
#define CGIUPDATE_H

#include "httpd.h"
typedef enum UpdaterState {
	U_Idle,
	U_FirmwareUpdatePending,
	U_FirmwareUpdating,
	U_FileDownloadPending,
	U_FileDownloading,
	U_RestartInitiated,
	U_RestartPending,
	U_FormatPending,
	U_Formating
} UpdaterState;


int cgiSendVersion(HttpdConnData *connData);
int cgiUpdate(HttpdConnData *connData);
int cgiDownload(HttpdConnData *connData);
int cgiDownloadStatus(HttpdConnData *connData);

#endif
