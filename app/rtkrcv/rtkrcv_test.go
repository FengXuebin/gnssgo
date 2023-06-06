/*------------------------------------------------------------------------------
* rtkrcv.c : rtk-gps/gnss receiver console ap
*
*          Copyright (C) 2009-2015 by T.TAKASU, All rights reserved.
*
* notes   :
*     current version does not support win32 without pthread library
*
* version : $Revision:$ $Date:$
* history : 2009/12/13 1.0  new
*           2010/07/18 1.1  add option -m
*           2010/08/12 1.2  fix bug on ftp/http
*           2011/01/22 1.3  add option misc-proxyaddr,misc-fswapmargin
*           2011/08/19 1.4  fix bug on size of arg solopt arg for rtksvrstart()
*           2012/11/03 1.5  fix bug on setting output format
*           2013/06/30 1.6  add "nvs" option for inpstr*-format
*           2014/02/10 1.7  fix bug on printing obs data
*                           add print of status, glonass nav data
*                           ignore SIGHUP
*           2014/04/27 1.8  add "binex" option for inpstr*-format
*           2014/08/10 1.9  fix cpu overload with abnormal telnet shutdown
*           2014/08/26 1.10 support input format "rt17"
*                           change file paths of solution status and debug trace
*           2015/01/10 1.11 add line editting and command history
*                           separate codes for virtual console to vt.c
*           2015/05/22 1.12 fix bug on sp3 id in inpstr*-format options
*           2015/07/31 1.13 accept 4:stat for outstr1-format or outstr2-format
*                           add reading satellite dcb
*           2015/12/14 1.14 add option -sta for station name (#339)
*           2015/12/25 1.15 fix bug on -sta option (#339)
*           2015/01/26 1.16 support septentrio
*           2016/07/01 1.17 support CMR/CMR+
*           2016/08/20 1.18 add output of patch level with version
*           2016/09/05 1.19 support ntrip caster for output stream
*           2016/09/19 1.20 support multiple remote console connections
*                           add option -w
*           2017/09/01 1.21 add command ssr
*-----------------------------------------------------------------------------*/

package main

import "testing"

func Test_prstatus(t *testing.T) {
	type args struct {
		vt *VTerm
	}
	tests := []struct {
		name string
		args args
	}{
		// TODO: Add test cases.
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			prstatus(tt.args.vt)
		})
	}
}
