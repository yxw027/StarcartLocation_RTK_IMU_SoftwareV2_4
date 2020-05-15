/*
	By     : shilei
	Date   : 2019-09-25
	From   : Starcart Ltd.
*/

#ifndef _KFAPP_H
#define _KFAPP_H

#include "PSINS.h"


class CKFApp:public CSINSTDKF
{
public:
	double tmeas;
	CVect3 measGPSVn, measGPSPos;

	CKFApp(void);
	void Init16(const CSINS &sins0);
	virtual void SetMeas(void);
	void SetMeas(CVect3 *vnm, CVect3 *posm, double tm);
};

#endif

