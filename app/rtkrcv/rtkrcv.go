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

import (
	"bufio"
	"flag"
	"fmt"
	"gnssgo"
	"log"
	"os"
	"os/signal"
	"strings"
	"syscall"
	"time"

	// "encoding/json"

	// "github.com/google/uuid"
	// db "github.com/influxdata/influxdb-client-go/v2"
	// es "github.com/olivere/elastic/v7"
	// "go.mongodb.org/mongo-driver/mongo"
	// "go.mongodb.org/mongo-driver/mongo/options"

	_ "github.com/ClickHouse/clickhouse-go"
	"github.com/jmoiron/sqlx"
)

var PRGNAME string = "rtkrcv"                    /* program name */
var CMDPROMPT string = "rtkrcv> "                /* command prompt */
var MAXCON int = 32                              /* max number of consoles */
var MAXARG int = 10                              /* max number of args in a command */
var MAXCMD int = 256                             /* max length of a command */
var MAXSTR int = 1024                            /* max length of a stream */
var OPTSDIR string = "."                         /* default config directory */
var OPTSFILE string = "rtkrcv.conf"              /* default config file */
var NAVIFILE string = "rtkrcv.nav"               /* navigation save file */
var STATFILE string = "rtkrcv_%Y%m%d%h%M.stat"   /* solution status file */
var TRACEFILE string = "rtkrcv_%Y%m%d%h%M.trace" /* debug trace file */
var INTKEEPALIVE int = 1000                      /* keep alive interval (ms) */

var ESC_CLEAR string = "\033[H\033[2J" /* ansi/vt100 escape: erase screen */
var ESC_RESET string = "\033[0m"       /* ansi/vt100: reset attribute */
var ESC_BOLD string = "\033[1m"        /* ansi/vt100: bold */
var D2R = gnssgo.D2R
var R2D = gnssgo.R2D

/* function prototypes -------------------------------------------------------*/
// extern FILE *popen(const char *, const char *);
// extern int pclose(FILE *);

/* global variables ----------------------------------------------------------*/
var (
	svr  gnssgo.RtkSvr /* rtk server struct */
	moni gnssgo.Stream /* monitor stream */

	intflg = 0 /* interrupt flag (2:shtdown) */

	passwd   = "admin" /* login password */
	timetype = 0       /* time format (0:gpst,1:utc,2:jst,3:tow) */
	soltype  = 0       /* sol format (0:dms,1:deg,2:xyz,3:enu,4:pyl) */
	solflag  = 2       /* sol flag (1:std+2:age/ratio/ns) */
	strtype  = []int{  /* stream types */
		gnssgo.STR_SERIAL, gnssgo.STR_NONE, gnssgo.STR_NONE,
		gnssgo.STR_NONE, gnssgo.STR_NONE, gnssgo.STR_NONE,
		gnssgo.STR_NONE, gnssgo.STR_NONE}
	strpath = [8]string{"", "", "", "", "", "", "", ""} /* stream paths */
	strfmt  = []int{                                    /* stream formats */
		gnssgo.STRFMT_UBX, gnssgo.STRFMT_RTCM3, gnssgo.STRFMT_SP3, gnssgo.SOLF_LLH, gnssgo.SOLF_NMEA}
	svrcycle  = 10                 /* server cycle (ms) */
	timeout   = 10000              /* timeout time (ms) */
	reconnect = 10000              /* reconnect interval (ms) */
	nmeacycle = 5000               /* nmea request cycle (ms) */
	buffsize  = 32768              /* input buffer size (bytes) */
	navmsgsel = 0                  /* navigation mesaage select */
	proxyaddr = ""                 /* http/ntrip proxy */
	nmeareq   = 0                  /* nmea request type (0:off,1:lat/lon,2:single) */
	nmeapos   = []float64{0, 0, 0} /* nmea position (lat/lon/height) (deg,m) */
	rcvcmds   [3]string            /* receiver commands files */
	startcmd  = ""                 /* start command */
	stopcmd   = ""                 /* stop command */
	// modflgr     [256]int             /* modified flags of receiver options */
	// modflgs     [256]int             /* modified flags of system options */
	moniport = 0 /* monitor port */
	// keepalive   = 0                  /* keep alive flag */
	fswapmargin = 30 /* file swap margin (s) */
	sta_name    = "" /* station name */

	prcopt gnssgo.PrcOpt    /* processing options */
	solopt [2]gnssgo.SolOpt /* solution options */
	filopt gnssgo.FilOpt /* file options */)

/* help text -----------------------------------------------------------------*/
var usage []string = []string{
	"usage: rtkrcv [-s][-p port][-d dev][-o file][-w pwd][-r level][-t level][-sta sta]",
	"options",
	"  -s         start RTK server on program startup",
	"  -p port    port number for telnet console",
	"  -m port    port number for monitor stream",
	"  -d dev     terminal device for console",
	"  -o file    processing options file",
	"  -w pwd     login password for remote console (\"\": no password)",
	"  -r level   output solution status file (0:off,1:states,2:residuals)",
	"  -t level   debug trace level (0:off,1-5:on)",
	"  -sta sta   station name for receiver dcb"}
var helptxt []string = []string{
	"start                 : start rtk server",
	"stop                  : stop rtk server",
	"restart               : restart rtk sever",
	"solution [cycle]      : show solution",
	"status [cycle]        : show rtk status",
	"satellite [-n] [cycle]: show satellite status",
	"observ [-n] [cycle]   : show observation data",
	"navidata [cycle]      : show navigation data",
	"stream [cycle]        : show stream status",
	"ssr [cycle]           : show ssr corrections",
	"error                 : show error/warning messages",
	"option [opt]          : show option(s)",
	"set opt [val]         : set option",
	"load [file]           : load options from file",
	"save [file]           : save options to file",
	"log [file|off]        : start/stop log to file",
	"help|? [path]         : print help",
	"exit|ctr-D            : logout console (only for telnet)",
	"shutdown              : shutdown rtk server",
	"!command [arg...]     : execute command in shell",
					""}
var pathopts []string = []string{ /* path options help */
	"stream path formats",
	"serial   : port[:bit_rate[:byte[:parity(n|o|e)[:stopb[:fctr(off|on)]]]]]",
	"file     : path[::T[::+offset][::xspeed]]",
	"tcpsvr   : :port",
	"tcpcli   : addr:port",
	"ntripsvr : user:passwd@addr:port/mntpnt[:str]",
	"ntripcli : user:passwd@addr:port/mntpnt",
	"ntripc_s : :passwd@:port",
	"ntripc_c : user:passwd@:port",
	"ftp      : user:passwd@addr/path[::T=poff,tint,off,rint]",
	"http     : addr/path[::T=poff,tint,off,rint]",
	""}

/* receiver options table ----------------------------------------------------*/
var TIMOPT string = "0:gpst,1:utc,2:jst,3:tow"
var CONOPT string = "0:dms,1:deg,2:xyz,3:enu,4:pyl"
var FLGOPT string = "0:off,1:std+2:age/ratio/ns"
var ISTOPT string = "0:off,1:serial,2:file,3:tcpsvr,4:tcpcli,5:ntripsvr,6:ntripcli,7:ftp,8:http"
var OSTOPT string = "0:off,1:serial,2:file,3:tcpsvr,4:tcpcli,6:ntripsvr,11:ntripc_c"
var FMTOPT string = "0:rtcm2,1:rtcm3,2:oem4,3:oem3,4:ubx,5:ss2,6:hemis,7:skytraq,8:gw10,9:javad,10:nvs,11:binex,12:rt17,13:sbf,14:cmr,15:tersus,18:sp3"
var NMEOPT string = "0:off,1:latlon,2:single"
var SOLOPT string = "0:llh,1:xyz,2:enu,3:nmea,4:stat"
var MSGOPT string = "0:all,1:rover,2:base,3:corr"

var rcvopts map[string]*gnssgo.Opt = map[string]*gnssgo.Opt{
	"console-passwd":   {Name: "console-passwd", Format: 2, VarInt: nil, VarFloat: nil, VarString: &passwd, Comment: ""},
	"console-timetype": {Name: "console-timetype", Format: 3, VarInt: &timetype, VarFloat: nil, VarString: nil, Comment: TIMOPT},
	"console-soltype":  {Name: "console-soltype", Format: 3, VarInt: &soltype, VarFloat: nil, VarString: nil, Comment: CONOPT},
	"console-solflag":  {Name: "console-solflag", Format: 0, VarInt: &solflag, VarFloat: nil, VarString: nil, Comment: FLGOPT},
	"inpstr1-type":     {Name: "inpstr1-type", Format: 3, VarInt: &strtype[0], VarFloat: nil, VarString: nil, Comment: ISTOPT},
	"inpstr2-type":     {Name: "inpstr2-type", Format: 3, VarInt: &strtype[1], VarFloat: nil, VarString: nil, Comment: ISTOPT},
	"inpstr3-type":     {Name: "inpstr3-type", Format: 3, VarInt: &strtype[2], VarFloat: nil, VarString: nil, Comment: ISTOPT},
	"inpstr1-path":     {Name: "inpstr1-path", Format: 2, VarInt: nil, VarFloat: nil, VarString: &strpath[0], Comment: ""},
	"inpstr2-path":     {Name: "inpstr2-path", Format: 2, VarInt: nil, VarFloat: nil, VarString: &strpath[1], Comment: ""},
	"inpstr3-path":     {Name: "inpstr3-path", Format: 2, VarInt: nil, VarFloat: nil, VarString: &strpath[2], Comment: ""},
	"inpstr1-format":   {Name: "inpstr1-format", Format: 3, VarInt: &strfmt[0], VarFloat: nil, VarString: nil, Comment: FMTOPT},
	"inpstr2-format":   {Name: "inpstr2-format", Format: 3, VarInt: &strfmt[1], VarFloat: nil, VarString: nil, Comment: FMTOPT},
	"inpstr3-format":   {Name: "inpstr3-format", Format: 3, VarInt: &strfmt[2], VarFloat: nil, VarString: nil, Comment: FMTOPT},
	"inpstr2-nmeareq":  {Name: "inpstr2-nmeareq", Format: 3, VarInt: &nmeareq, VarFloat: nil, VarString: nil, Comment: NMEOPT},
	"inpstr2-nmealat":  {Name: "inpstr2-nmealat", Format: 1, VarInt: nil, VarFloat: &nmeapos[0], VarString: nil, Comment: "deg"},
	"inpstr2-nmealon":  {Name: "inpstr2-nmealon", Format: 1, VarInt: nil, VarFloat: &nmeapos[1], VarString: nil, Comment: "deg"},
	"inpstr2-nmeahgt":  {Name: "inpstr2-nmeahgt", Format: 1, VarInt: nil, VarFloat: &nmeapos[2], VarString: nil, Comment: "m"},
	"outstr1-type":     {Name: "outstr1-type", Format: 3, VarInt: &strtype[3], VarFloat: nil, VarString: nil, Comment: OSTOPT},
	"outstr2-type":     {Name: "outstr2-type", Format: 3, VarInt: &strtype[4], VarFloat: nil, VarString: nil, Comment: OSTOPT},
	"outstr1-path":     {Name: "outstr1-path", Format: 2, VarInt: nil, VarFloat: nil, VarString: &strpath[3], Comment: ""},
	"outstr2-path":     {Name: "outstr2-path", Format: 2, VarInt: nil, VarFloat: nil, VarString: &strpath[4], Comment: ""},
	"outstr1-format":   {Name: "outstr1-format", Format: 3, VarInt: &strfmt[3], VarFloat: nil, VarString: nil, Comment: SOLOPT},
	"outstr2-format":   {Name: "outstr2-format", Format: 3, VarInt: &strfmt[4], VarFloat: nil, VarString: nil, Comment: SOLOPT},
	"logstr1-type":     {Name: "logstr1-type", Format: 3, VarInt: &strtype[5], VarFloat: nil, VarString: nil, Comment: OSTOPT},
	"logstr2-type":     {Name: "logstr2-type", Format: 3, VarInt: &strtype[6], VarFloat: nil, VarString: nil, Comment: OSTOPT},
	"logstr3-type":     {Name: "logstr3-type", Format: 3, VarInt: &strtype[7], VarFloat: nil, VarString: nil, Comment: OSTOPT},
	"logstr1-path":     {Name: "logstr1-path", Format: 2, VarInt: nil, VarFloat: nil, VarString: &strpath[5], Comment: ""},
	"logstr2-path":     {Name: "logstr2-path", Format: 2, VarInt: nil, VarFloat: nil, VarString: &strpath[6], Comment: ""},
	"logstr3-path":     {Name: "logstr3-path", Format: 2, VarInt: nil, VarFloat: nil, VarString: &strpath[7], Comment: ""},
	"misc-svrcycle":    {Name: "misc-svrcycle", Format: 0, VarInt: &svrcycle, VarFloat: nil, VarString: nil, Comment: "ms"},
	"misc-timeout":     {Name: "misc-timeout", Format: 0, VarInt: &timeout, VarFloat: nil, VarString: nil, Comment: "ms"},
	"misc-reconnect":   {Name: "misc-reconnect", Format: 0, VarInt: &reconnect, VarFloat: nil, VarString: nil, Comment: "ms"},
	"misc-nmeacycle":   {Name: "misc-nmeacycle", Format: 0, VarInt: &nmeacycle, VarFloat: nil, VarString: nil, Comment: "ms"},
	"misc-buffsize":    {Name: "misc-buffsize", Format: 0, VarInt: &buffsize, VarFloat: nil, VarString: nil, Comment: "bytes"},
	"misc-navmsgsel":   {Name: "misc-navmsgsel", Format: 3, VarInt: &navmsgsel, VarFloat: nil, VarString: nil, Comment: MSGOPT},
	"misc-proxyaddr":   {Name: "misc-proxyaddr", Format: 2, VarInt: nil, VarFloat: nil, VarString: &proxyaddr, Comment: ""},
	"misc-fswapmargin": {Name: "misc-fswapmargin", Format: 0, VarInt: &fswapmargin, VarFloat: nil, VarString: nil, Comment: "s"},

	"misc-startcmd": {Name: "misc-startcmd", Format: 2, VarInt: nil, VarFloat: nil, VarString: &startcmd, Comment: ""},
	"misc-stopcmd":  {Name: "misc-stopcmd", Format: 2, VarInt: nil, VarFloat: nil, VarString: &stopcmd, Comment: ""},

	"file-cmdfile1": {Name: "file-cmdfile1", Format: 2, VarInt: nil, VarFloat: nil, VarString: &rcvcmds[0], Comment: ""},
	"file-cmdfile2": {Name: "file-cmdfile2", Format: 2, VarInt: nil, VarFloat: nil, VarString: &rcvcmds[1], Comment: ""},
	"file-cmdfile3": {Name: "file-cmdfile3", Format: 2, VarInt: nil, VarFloat: nil, VarString: &rcvcmds[2], Comment: ""}}

func printusage() {
	for _, v := range usage {
		fmt.Fprintf(os.Stderr, v+"\n")
	}
	os.Exit(0)
}
func searchHelp(key string) string {
	for _, v := range helptxt {
		if strings.Contains(v, key) {
			return v
		}
	}
	return "no surported augument"
}

/* show message --------------------------------------------------------------*/
func showmsg(format string, v ...interface{}) int {
	fmt.Fprintf(os.Stderr, format, v...)
	if len(format) > 0 {
		fmt.Fprintf(os.Stderr, "\r")
	} else {
		fmt.Fprintf(os.Stderr, "\n")
	}
	return 0
}

/* read receiver commands ----------------------------------------------------*/
func readcmd(file string, cmd *string, ctype int) int {
	var (
		fp   *os.File
		buff string
		i    int = 0
		err  error
	)

	log.Printf("readcmd: file=%s\n", file)

	if fp, _ = os.OpenFile(file, os.O_RDONLY, 0666); fp == nil {
		return 0
	}
	rd := bufio.NewReader(fp)
	for {
		buff, err = rd.ReadString('\n')
		if err != nil {
			break
		}
		if buff[0] == '@' {
			i++
		} else if i == ctype && len(*cmd)+len(buff)+1 < gnssgo.MAXRCVCMD {
			*cmd += buff
		}
	}
	fp.Close()
	return 1
}

/* read antenna file ---------------------------------------------------------*/
func readant(opt *gnssgo.PrcOpt, nav *gnssgo.Nav) {
	var (
		pcv0       gnssgo.Pcv
		pcvr, pcvs gnssgo.Pcvs
		pcv        *gnssgo.Pcv
		time       gnssgo.Gtime = gnssgo.TimeGet()
		i          int
	)

	log.Printf("readant:\n")

	opt.Pcvr[0], opt.Pcvr[1] = pcv0, pcv0
	if len(filopt.RcvAntPara) == 0 {
		return
	}

	if gnssgo.ReadPcv(filopt.RcvAntPara, &pcvr) != 0 {
		for i = 0; i < 2; i++ {
			if len(opt.AntType[i]) == 0 {
				continue
			}
			if pcv = gnssgo.SearchPcv(0, opt.AntType[i], time, &pcvr); pcv == nil {
				continue
			}
			opt.Pcvr[i] = *pcv
		}
	}

	if gnssgo.ReadPcv(filopt.SatAntPara, &pcvs) > 0 {
		for i = 0; i < gnssgo.MAXSAT; i++ {
			if pcv = gnssgo.SearchPcv(i+1, "", time, &pcvs); pcv == nil {
				continue
			}
			nav.Pcvs[i] = *pcv
		}
	}

	pcvr.Pcv = nil
	pcvs.Pcv = nil
}

/* start rtk server ----------------------------------------------------------*/
func startsvr() int {
	var (
		sta           [gnssgo.MAXRCV]gnssgo.Sta
		pos, npos     [3]float64
		s1            [3]string = [3]string{"", "", ""}
		cmds          [3]string
		s2            [3]string = [3]string{"", "", ""}
		cmds_periodic [3]string
		ropts         []string = []string{"", "", ""}
		paths         []string = []string{
			strpath[0], strpath[1], strpath[2], strpath[3], strpath[4], strpath[5],
			strpath[6], strpath[7]}
		errmsg string = ""
		i      int
		stropt [8]int
	)

	log.Printf("startsvr:\n")
	/* read start commads from command files */
	for i = 0; i < 3; i++ {
		if len(rcvcmds[i]) == 0 {
			continue
		}
		if readcmd(rcvcmds[i], &s1[i], 0) == 0 {
		} else {
			cmds[i] = s1[i]
		}
		if readcmd(rcvcmds[i], &s2[i], 2) == 0 {
		} else {
			cmds_periodic[i] = s2[i]
		}
	}

	/* confirm overwrite */
	// for i = 3; i < 8; i++ {
	// 	if strtype[i] == int(gnssgo.STR_FILE) && confwrite(vt, strpath[i]) == 0 {
	// 		return 0
	// 	}
	// }
	if prcopt.RefPos == 4 { /* rtcm */
		for i = 0; i < 3; i++ {
			prcopt.Rb[i] = 0.0
		}
	}
	pos[0] = nmeapos[0] * D2R
	pos[1] = nmeapos[1] * D2R
	pos[2] = nmeapos[2]
	gnssgo.Pos2Ecef(pos[:], npos[:])

	/* read antenna file */
	readant(&prcopt, &svr.NavData)

	/* read dcb file */
	if len(filopt.Dcb) > 0 {
		sta[0].Name = sta_name
		svr.NavData.ReadDcb(filopt.Dcb, sta[:])
	}
	/* open geoid data file */
	if solopt[0].Geoid > 0 && gnssgo.OpenGeoid(solopt[0].Geoid, filopt.Geoid) == 0 {
		log.Printf("geoid data open error: %s\n", filopt.Geoid)
	}
	// for  i=0; len(rcvopts[i].Name)>0 ;i++ { modflgr[i]=0;}
	// for  i=0; len(gnssgo.SysOpts[i].Name)>0;i++ { modflgs[i]=0;}

	/* set stream options */
	stropt[0] = timeout
	stropt[1] = reconnect
	stropt[2] = 1000
	stropt[3] = buffsize
	stropt[4] = fswapmargin
	gnssgo.StreamSetOpt(stropt[:])

	if strfmt[2] == 8 {
		strfmt[2] = int(gnssgo.STRFMT_SP3)
	}

	/* set ftp/http directory and proxy */
	gnssgo.StreamSetDir(filopt.TempDir)
	gnssgo.StreamSetProxy(proxyaddr)

	/* execute start command */
	if len(startcmd) > 0 && gnssgo.ExecCmd(startcmd) < 0 {
		log.Printf("command exec error: %s \n", startcmd)
	}
	solopt[0].Posf = strfmt[3]
	solopt[1].Posf = strfmt[4]

	/* start rtk server */
	if svr.RtkSvrStart(svrcycle, buffsize, strtype, paths, strfmt, navmsgsel,
		cmds[:], cmds_periodic[:], ropts, nmeacycle, nmeareq, npos[:], &prcopt,
		solopt[:], &moni, &errmsg) == 0 {
		log.Printf("rtk server start error (%s)\n", errmsg)
		return 0
	}
	//	go writeObs(gnssgo.ObsChannel)
	//	go writeObs2MongoDB(gnssgo.ObsChannel)
	// go writeObs2ElasticSearch(gnssgo.ObsChannel)
	// go writeRbSol(gnssgo.RbSolChannel)
	go writeObs2ClickHouse(gnssgo.ObsChannel)
	return 1
}

/* stop rtk server -----------------------------------------------------------*/
func stopsvr() {
	var (
		s    [3]string = [3]string{"", "", ""}
		cmds [3]string
		i    int
	)

	log.Printf("stopsvr:\n")

	if svr.State == 0 {
		return
	}

	/* read stop commads from command files */
	for i = 0; i < 3; i++ {
		if len(rcvcmds[i]) == 0 {
			continue
		}
		if readcmd(rcvcmds[i], &s[i], 1) == 0 {
		} else {
			cmds[i] = s[i]
		}
	}
	/* stop rtk server */
	svr.RtkSvrStop(cmds[:])

	/* execute stop command */
	if len(stopcmd) > 0 && gnssgo.ExecCmd(stopcmd) < 0 {
		log.Printf("command exec error: %s \n", stopcmd)
	}
	if solopt[0].Geoid > 0 {
		gnssgo.CloseGeoid()
	}

}

/* write obs data to influxDB */
func writeObs(ch chan gnssgo.ObsD) {
	// client := db.NewClient("http://localhost:8086", "qdhQU9SHk2xlWZGwA9UxxQDdSjFfeGJOfVLWjACQ1isHgWCZye7bziwPF00AAqb9jd2QuszxksmedJ98CZ21Sw==")
	// writeAPI := client.WriteAPI("idtsz", "gnss")

	// for intflg == 0 {
	// 	data, ok := <-ch
	// 	if !ok {
	// 		continue
	// 	}
	// 	// t := time.Unix(int64(data.Time.Time), int64(data.Time.Sec))

	// 	p := db.NewPointWithMeasurement("obs2").
	// 		AddField("sat", data.Sat).
	// 		AddField("rcv", data.Rcv).
	// 		AddField("SNR1", data.SNR[0]).
	// 		AddField("SNR2", data.SNR[1]).
	// 		AddField("SNR3", data.SNR[2]).
	// 		AddField("LLI1", data.LLI[0]).
	// 		AddField("LLI2", data.LLI[1]).
	// 		AddField("LLI3", data.LLI[2]).
	// 		AddField("Code1", data.Code[0]).
	// 		AddField("Code2", data.Code[1]).
	// 		AddField("Code3", data.Code[2]).
	// 		AddField("L1", data.L[0]).
	// 		AddField("L2", data.L[1]).
	// 		AddField("L3", data.L[2]).
	// 		AddField("P1", data.P[0]).
	// 		AddField("P2", data.P[1]).
	// 		AddField("P3", data.P[2]).
	// 		AddField("D1", data.D[0]).
	// 		AddField("D2", data.D[1]).
	// 		AddField("D3", data.D[2]).
	// 		SetTime(time.Now()) // Flush writes

	// 		//		fmt.Printf("sat=%d", data.Sat)
	// 	writeAPI.WritePoint(p)
	// 	writeAPI.Flush()
	// }
}

func writeRbSol(ch chan gnssgo.RBSol) {
	// client := db.NewClient("http://localhost:8086", "qdhQU9SHk2xlWZGwA9UxxQDdSjFfeGJOfVLWjACQ1isHgWCZye7bziwPF00AAqb9jd2QuszxksmedJ98CZ21Sw==")
	// writeAPI := client.WriteAPI("idtsz", "gnss")
	// var (
	// 	pos, rr, enu [3]float64
	// 	P, Q         [9]float64
	// )
	// for intflg == 0 {
	// 	data, ok := <-ch
	// 	if !ok {
	// 		continue
	// 	}
	// 	t := time.Unix(int64(data.Sol.Time.Time), 0)
	// 	for i := 0; i < 3; i++ {
	// 		rr[i] = data.Sol.Rr[i] - data.Rb[i]
	// 	}
	// 	gnssgo.Ecef2Pos(data.Rb[:], pos[:])
	// 	data.Sol.Sol2Cov(P[:])
	// 	gnssgo.Cov2Enu(pos[:], P[:], Q[:])
	// 	gnssgo.Ecef2Enu(pos[:], rr[:], enu[:])
	// 	// enu[0], sep, enu[1], sep, enu[2], sep, sol.Stat, sep, sol.Ns, sep,
	// 	// SQRT(Q[0]), sep, SQRT(Q[4]), sep, SQRT(Q[8]), sep, sqvar(Q[1]),
	// 	// sep, sqvar(Q[5]), sep, sqvar(Q[2]), sep, sol.Age, sep, sol.Ratio)
	// 	p := db.NewPointWithMeasurement("rtkpos").
	// 		AddField("enu0", enu[0]).
	// 		AddField("enu1", enu[1]).
	// 		AddField("enu2", enu[2]).
	// 		AddField("Q0", Q[0]).
	// 		AddField("Q4", Q[4]).
	// 		AddField("Q8", Q[8]).
	// 		AddField("Q1", Q[1]).
	// 		AddField("Q5", Q[5]).
	// 		AddField("Q5", Q[2]).
	// 		AddField("Ns", data.Sol.Ns).
	// 		AddField("Stat", data.Sol.Stat).
	// 		AddField("Age", data.Sol.Age).
	// 		AddField("Ratio", data.Sol.Ratio).
	// 		SetTime(t) // Flush writes

	// 	writeAPI.WritePoint(p)
	// }
	// writeAPI.Flush()
}

func writeObs2MongoDB(ch chan gnssgo.ObsD) {
	// clientOptions := options.Client().ApplyURI("mongodb://localhost:27017")

	// // 连接到MongoDB
	// client, err := mongo.Connect(context.TODO(), clientOptions)

	// if err != nil {
	// 	fmt.Printf("failed to connect mongodb\n")
	// 	return
	// }
	// collection := client.Database("gnss").Collection("gnsstest")
	// for intflg == 0 {
	// 	data, ok := <-ch
	// 	if !ok {
	// 		continue
	// 	}

	// 	//插入某一条数据
	// 	if _, err = collection.InsertOne(context.TODO(), data); err != nil {
	// 		fmt.Print(err)
	// 		return
	// 	}
	// }
}

type obsitem struct {
	// Time gnssgo.Gtime `json:"time"`
	Sat  int      `json:"sat"`
	Rcv  int      `json:"rcv"`
	SNR  []uint16 `json:"snr"`
	LLI  []uint8  `json:"lli"`
	Code []uint8  `json:"code"`
	// L    []float64 `json:"l"`
	// P    []float64 `json:"p"`
	// D    []float64 `json:"d"`
}

func writeObs2ElasticSearch(ch chan gnssgo.ObsD) {
	// client, err := es.NewClient(
	// 	es.SetSniff(false),                  // SetSniff启用或禁用嗅探器（默认情况下启用）。
	// 	es.SetURL("http://127.0.0.1:9200/"), // URL地址
	// 	es.SetBasicAuth("", ""),             // 账号密码
	// )

	// if err != nil {
	// 	panic(err)
	// }

	// for intflg == 0 {
	// 	data, ok := <-ch
	// 	if !ok {
	// 		continue
	// 	}
	// 	jdata, _ := json.Marshal(data)
	// 	fmt.Print(string(jdata))
	// 	item := obsitem{
	// 		// Time: data.Time,
	// 		Sat:  data.Sat,
	// 		Rcv:  data.Rcv,
	// 		SNR:  data.SNR[:],
	// 		LLI:  data.LLI[:],
	// 		Code: data.Code[:],
	// 		// 	L:    data.L[:],
	// 		// 	P:    data.P[:],
	// 		// 	D:    data.D[:],
	// 	}
	// 	_, err := client.Index().
	// 		Index("obs").         // 索引名称
	// 		Id(uuid.NewString()). // 指定文档id
	// 		BodyJson(item).       // 可序列化JSON
	// 		Do(context.Background())
	// 	if err != nil {
	// 		panic(err)
	// 	}
	// }
}

func writeObs2ClickHouse(ch chan gnssgo.ObsD) {
	host1 := "192.168.1.181:8123"
	otherHost := "192.168.1.181:9000"
	user := "admin"
	password := "admin"
	database := "gnss"
	tcpInfo := "http://%s/%s?username=%s&password=%s&database=%s&read_timeout=5&write_timeout=5&debug=true&compress=true&alt_hosts=%s"
	tcpInfo = fmt.Sprintf(tcpInfo, host1, database, user, password, database, otherHost)
	client, err := sqlx.Open("clickhouse", tcpInfo)
	if err != nil {
		panic(err.Error())
	}
	if err != nil {
		panic("failed to connect database")
	}
	client.SetMaxOpenConns(50)
	client.SetMaxIdleConns(50)

	for intflg == 0 {
		data, ok := <-ch
		if !ok {
			continue
		}
		tx, err := client.Begin()
		if err != nil {
			log.Println("xxx", err)
			panic("failed to invoke client.Begin()")
		}
		log.Println("start Prepare")
		stmt, err := tx.Prepare("insert into Obs (`Time`,Sat, Rcv, SNR, Code, LLI, L, P, D)")
		if err != nil {
			log.Println("xxx2", err)
			panic("failed to invoke tx.Prepare(\"insert into obs (`Time`,Sat, Rcv, SNR, Code, LLI, L, P, D)\")")
		}
		t := time.Unix(int64(data.Time.Time), 0)
		if _, err := stmt.Exec(
			t,
			data.Sat,
			data.Rcv,
			data.SNR[:],
			data.Code[:],
			data.LLI[:],
			data.L[:],
			data.P[:],
			data.D[:],
		); err != nil {
			log.Fatal("2", err)
		}

		if err := tx.Commit(); err != nil {
			log.Fatal("3", err)
		}
	}
}

/* external stop signal ------------------------------------------------------*/
func sigshut(sig int) {
	log.Printf("sigshut: sig=%d\n", sig)

	intflg = 1
	os.Exit(0)
}

func main() {
	var (
		port, outstat, trace int
		dev, file            string
		bstart               bool = false
		start                int  = 0
		ss                        = []string{"E", "-", "W", "C", "C"}
	)

	gnssgo.ShowMsg_Ptr = showmsg

	flag.BoolVar(&bstart, "s", false, searchHelp("-s"))
	flag.IntVar(&port, "p", port, searchHelp("-p"))
	flag.IntVar(&moniport, "m", moniport, searchHelp("-m"))
	flag.StringVar(&dev, "d", dev, searchHelp("-d"))
	flag.StringVar(&file, "o", file, searchHelp("-o"))
	flag.StringVar(&passwd, "w", passwd, searchHelp("-w"))
	flag.IntVar(&outstat, "r", outstat, searchHelp("-r"))
	flag.IntVar(&trace, "t", trace, searchHelp("-t"))
	flag.StringVar(&sta_name, "sta", sta_name, searchHelp("-sta"))

	flag.Parse()

	if flag.NFlag() < 1 {
		// if there is not any arguments, exit
		printusage()
	}

	if bstart {
		start = 1
	}
	if trace > 0 {
		gnssgo.TraceOpen(TRACEFILE)
		gnssgo.TraceLevel(trace)
	}

	svr.InitRtkSvr()
	moni.InitStream()

	/* load options file */
	if len(file) == 0 {
		file = fmt.Sprintf("%s/%s", OPTSDIR, OPTSFILE)
	}
	gnssgo.ResetSysOpts()
	if gnssgo.LoadOpts(file, &rcvopts) == 0 || gnssgo.LoadOpts(file, &gnssgo.SysOpts) == 0 {
		fmt.Fprintf(os.Stderr, "no options file: %s. defaults used\n", file)
		return
	}
	gnssgo.GetSysOpts(&prcopt, &solopt[0], &filopt)
	flag.Parse()

	/* read navigation data */
	if svr.NavData.ReadNav(NAVIFILE) == 0 {
		fmt.Fprintf(os.Stderr, "no navigation data: %s\n", NAVIFILE)
	}
	if outstat > 0 {
		gnssgo.RtkOpenStat(STATFILE, outstat)
	}

	c := make(chan os.Signal)
	signal.Notify(c, syscall.SIGINT, syscall.SIGTERM, //syscall.SIGUSR2, //no defined under windows
		syscall.SIGHUP, syscall.SIGPIPE)
	go func() {
		for s := range c {
			switch s {
			case syscall.SIGHUP, syscall.SIGINT, syscall.SIGTERM:
				sigshut(0)
			case syscall.SIGPIPE:
				fmt.Println("usr1 signal", s)
				//    case syscall.SIGUSR2:
				// 		   fmt.Println("usr2 signal", s)
			default:
				fmt.Println("other signal", s)
			}
		}
	}()

	/* start rtk server */
	if start > 0 {
		startsvr()
	}
	for intflg == 0 {
		/* accept remote console connection */
		var sstat [gnssgo.MAXSTRRTK]int
		var strmsg, buff string
		/* get stream server status */
		svr.RtkSvrStreamStat(sstat[:], &strmsg)

		/* show stream server status */
		for i := 0; i < gnssgo.MAXSTRRTK; i++ {
			buff += ss[sstat[i]+1]
		}

		fmt.Fprintf(os.Stderr, "%s [%s] %s\n",
			gnssgo.TimeStr(gnssgo.Utc2GpsT(gnssgo.TimeGet()), 0), buff, strmsg)

		gnssgo.Sleepms(100)
	}
	/* stop rtk server */
	stopsvr()

	if outstat > 0 {
		gnssgo.RtkCloseStat()
	}

	/* save navigation data */
	if svr.NavData.SaveNav(NAVIFILE) == 0 {
		fmt.Fprintf(os.Stderr, "navigation data save error: %s\n", NAVIFILE)
	}

}
