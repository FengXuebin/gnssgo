/*------------------------------------------------------------------------------
* str2str.c : console version of stream server
*
*          Copyright (C) 2007-2020 by T.TAKASU, All rights reserved.
*
* version : $Revision: 1.1 $ $Date: 2008/07/17 21:54:53 $
* history : 2009/06/17  1.0  new
*           2011/05/29  1.1  add -f, -l and -x option
*           2011/11/29  1.2  fix bug on recognize ntrips:// (rtklib_2.4.1_p4)
*           2012/12/25  1.3  add format conversion functions
*                            add -msg, -opt and -sta options
*                            modify -p option
*           2013/01/25  1.4  fix bug on showing message
*           2014/02/21  1.5  ignore SIG_HUP
*           2014/08/10  1.5  fix bug on showing message
*           2014/08/26  1.6  support input format gw10, binex and rt17
*           2014/10/14  1.7  use stdin or stdout if option -in or -out omitted
*           2014/11/08  1.8  add option -a, -i and -o
*           2015/03/23  1.9  fix bug on parsing of command line options
*           2016/01/23  1.10 enable septentrio
*           2016/01/26  1.11 fix bug on station position by -p option (#126)
*                            add option -px
*           2016/07/01  1.12 support CMR/CMR+
*           2016/07/23  1.13 add option -c1 -c2 -c3 -c4
*           2016/09/03  1.14 support ntrip caster
*                            add option -ft,-fl
*           2016/09/06  1.15 add reload soure table by USR2 signal
*           2016/09/17  1.16 add option -b
*           2017/05/26  1.17 add input format tersus
*           2020/11/30  1.18 support api change strsvrstart(),strsvrstat()
*-----------------------------------------------------------------------------*/
package main

import (
	"bufio"
	"flag"
	"fmt"
	"os"
	"os/signal"
	"strconv"
	"strings"
	"syscall"

	"gnssgo"
)

var PRGNAME string = "str2str"      /* program name */
const MAXSTR int = 5                /* max number of streams */
var TRFILE string = "str2str.trace" /* trace file */

/* global variables ----------------------------------------------------------*/
var strsvr gnssgo.StreamSvr /* stream server */
var intrflg int = 0         /* interrupt flag */

/* help text -----------------------------------------------------------------*/
var help []string = []string{
	"",
	" usage: str2str [-in stream] [-out stream [-out stream...]] [options]",
	"",
	" Input data from a stream and divide and output them to multiple streams",
	" The input stream can be serial, tcp client, tcp server, ntrip client, or",
	" file. The output stream can be serial, tcp client, tcp server, ntrip server,",
	" or file. str2str is a resident type application. To stop it, type ctr-c in",
	" console if run foreground or send signal SIGINT for background process.",
	" if run foreground or send signal SIGINT for background process.",
	" if both of the input stream and the output stream follow #format, the",
	" format of input messages are converted to output. To specify the output",
	" messages, use -msg option. If the option -in or -out omitted, stdin for",
	" input or stdout for output is used. If the stream in the option -in or -out",
	" is null, stdin or stdout is used as well.",
	" Command options are as follows.",
	"",
	" -in  stream[#format] input  stream path and format",
	" -out stream[#format] output stream path and format",
	"",
	"  stream path",
	"    serial       : serial://port[:brate[:bsize[:parity[:stopb[:fctr]]]]]",
	"    tcp server   : tcpsvr://:port",
	"    tcp client   : tcpcli://addr[:port]",
	"    ntrip client : ntrip://[user[:passwd]@]addr[:port][/mntpnt]",
	"    ntrip server : ntrips://[:passwd@]addr[:port]/mntpnt[:str] (only out)",
	"    ntrip caster : ntripc://[user:passwd@][:port]/mntpnt[:srctbl] (only out)",
	"    file         : [file://]path[::T][::+start][::xseppd][::S=swap]",
	"",
	"  format",
	"    rtcm2        : RTCM 2 (only in)",
	"    rtcm3        : RTCM 3",
	"    nov          : NovAtel OEMV/4/6,OEMStar (only in)",
	"    oem3         : NovAtel OEM3 (only in)",
	"    ubx          : ublox LEA-4T/5T/6T (only in)",
	"    ss2          : NovAtel Superstar II (only in)",
	"    hemis        : Hemisphere Eclipse/Crescent (only in)",
	"    stq          : SkyTraq S1315F (only in)",
	"    javad        : Javad (only in)",
	"    nvs          : NVS BINR (only in)",
	"    binex        : BINEX (only in)",
	"    rt17         : Trimble RT17 (only in)",
	"    sbf          : Septentrio SBF (only in)",
	"",
	" -msg \"type[(tint)][,type[(tint)]...]\"",
	"                   rtcm message types and output intervals (s)",
	" -sta sta          station id",
	" -opt opt          receiver dependent options",
	" -s  msec          timeout time (ms) [10000]",
	" -r  msec          reconnect interval (ms) [10000]",
	" -n  msec          nmea request cycle (m) [0]",
	" -f  sec           file swap margin (s) [30]",
	" -c  file          input commands file [no]",
	" -c1 file          output 1 commands file [no]",
	" -c2 file          output 2 commands file [no]",
	" -c3 file          output 3 commands file [no]",
	" -c4 file          output 4 commands file [no]",
	" -p  lat lon hgt   station position (latitude/longitude/height) (deg,m)",
	" -px x y z         station position (x/y/z-ecef) (m)",
	" -a  antinfo       antenna info (separated by ,)",
	" -i  rcvinfo       receiver info (separated by ,)",
	" -o  e n u         antenna offset (e,n,u) (m)",
	" -l  local_dir     ftp/http local directory []",
	" -x  proxy_addr    http/ntrip proxy address [no]",
	" -b  str_no        relay back messages from output str to input str [no]",
	" -t  level         trace level [0]",
	" -fl file          log file [str2str.trace]",
	" -h                print help"}

func searchHelp(key string) string {
	for _, v := range help {
		if strings.Index(v, key) >= 0 {
			return v
		}
	}
	return "no surported augument"
}

/* print help ----------------------------------------------------------------*/
func printhelp() {
	for i, _ := range help {
		fmt.Fprintf(os.Stderr, "%s\n", help[i])
	}
	os.Exit(0)
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

/* signal handler ------------------------------------------------------------*/
func sigfunc(sig int) {
	intrflg = 1
}

/* decode format -------------------------------------------------------------*/
func decodefmt(path *string, fmt *int) {
	buff := *path
	*fmt = -1

	if idx := strings.LastIndex(buff, "#"); idx >= 0 {
		switch buff[idx:] {
		case "#rtcm2":
			*fmt = gnssgo.STRFMT_RTCM2
		case "#rtcm3":
			*fmt = gnssgo.STRFMT_RTCM3
		case "#nov":
			*fmt = gnssgo.STRFMT_OEM4
		case "#oem3":
			*fmt = gnssgo.STRFMT_OEM3
		case "#ubx":
			*fmt = gnssgo.STRFMT_UBX
		case "#ss2":
			*fmt = gnssgo.STRFMT_SS2
		case "#hemis":
			*fmt = gnssgo.STRFMT_CRES
		case "#stq":
			*fmt = gnssgo.STRFMT_STQ
		case "#javad":
			*fmt = gnssgo.STRFMT_JAVAD
		case "#nvs":
			*fmt = gnssgo.STRFMT_NVS
		case "#binex":
			*fmt = gnssgo.STRFMT_BINEX
		case "#rt17":
			*fmt = gnssgo.STRFMT_RT17
		case "#sbf":
			*fmt = gnssgo.STRFMT_SEPT
		default:
			return
		}

		*path = buff[:idx]
	}
}

/* decode stream path --------------------------------------------------------*/
func decodepath(path string, ctype *int, strpath *string, f *int) int {
	var buff string
	var idx int
	buff = path

	/* decode format */
	decodefmt(&buff, f)

	/* decode type */
	if idx = strings.Index(buff, "://"); idx < 0 {
		*strpath = buff
		*ctype = gnssgo.STR_FILE
		return 1
	}
	switch {
	case path[:6] == "serial":
		*ctype = gnssgo.STR_SERIAL
	case path[:6] == "tcpsvr":
		*ctype = gnssgo.STR_TCPSVR
	case path[:6] == "tcpcli":
		*ctype = gnssgo.STR_TCPCLI
	case path[:6] == "ntripc":
		*ctype = gnssgo.STR_NTRIPCAS
	case path[:6] == "ntrips":
		*ctype = gnssgo.STR_NTRIPSVR
	case path[:5] == "ntrip":
		*ctype = gnssgo.STR_NTRIPCLI
	case path[:4] == "file":
		*ctype = gnssgo.STR_FILE
	default:
		fmt.Fprintf(os.Stderr, "stream path error: %s\n", buff)
		return 0
	}
	//*strpath = buff
	*strpath = buff[idx+3:]
	return 1
}

/* read receiver commands ----------------------------------------------------*/
func readcmd(file string, cmd *string, ctype int) {
	var fp *os.File
	var buff string
	var i int = 0
	var err error

	if cmd == nil {
		return
	}
	*cmd = ""

	if fp, err = os.OpenFile(file, os.O_RDONLY, 0666); err != nil {
		return
	}
	defer fp.Close()
	rd := bufio.NewReader(fp)
	for {
		buff, err = rd.ReadString('\n')
		if err != nil {
			break
		}
		if buff[0] == '@' {
			i++
		} else if i == ctype {
			*cmd += buff
		}
	}
}

type posFlag struct {
	pos        []float64
	configured bool
}

func (f *posFlag) Set(s string) error {
	values := strings.Split(s, ",")
	if len(values) < 3 {
		return fmt.Errorf("too few arguments")
	}
	for i, v := range values {
		posFlag(*f).pos[i], _ = strconv.ParseFloat(v, 64)
	}
	f.configured = true
	return nil
}
func (f *posFlag) String() string {
	return "0,0,0"
}
func newFloatSlice(value []float64, p *[]float64) *posFlag {
	pf := posFlag{value, false}
	*p = pf.pos
	return &pf
}

/* str2str ----------------------------------------------------*/
func main() {
	var (
		cmd_strs, cmdfile, cmds, cmds_periodic     [MAXSTR]string
		cmd_periodic_strs                          [MAXSTR]string
		ss                                         []string = []string{"E", "-", "W", "C", "C"}
		conv                                       [MAXSTR]*gnssgo.StrConv
		pos, stapos, stadel                        [3]float64
		s1, s2, paths, logs                        [MAXSTR]string
		local, proxy, msg, opt, buff, strmsg       string
		ant                                        []string = []string{"", "", ""}
		rcv                                        []string = []string{"", "", ""}
		i, n, trlevel, sta                         int
		dispint                                    int         = 5000
		opts                                       []int       = []int{10000, 10000, 2000, 32768, 10, 0, 30, 0}
		types                                      [MAXSTR]int = [MAXSTR]int{gnssgo.STR_FILE, gnssgo.STR_FILE}
		stat, log_stat, fmts                       [MAXSTR]int
		bytes, bps                                 [MAXSTR]int
		infile, outfile, antinfo, rcvinfo, logfile string
	)
	msg = "1004,1019"
	for i = 0; i < MAXSTR; i++ {
		paths[i] = s1[i]
		logs[i] = s2[i]
		cmds[i] = cmd_strs[i]
		cmds_periodic[i] = cmd_periodic_strs[i]
	}
	gnssgo.ShowMsg_Ptr = showmsg

	flag.StringVar(&infile, "in", infile, searchHelp("-in"))
	flag.StringVar(&outfile, "out", outfile, searchHelp("-out"))
	rp := pos[:]
	rpFlag := newFloatSlice([]float64{}, &rp)
	flag.Var(rpFlag, "p", searchHelp("-p"))

	rsp := stapos[:]
	rspFlag := newFloatSlice([]float64{}, &rsp)
	flag.Var(rspFlag, "px", searchHelp("-px"))

	rsd := stadel[:]
	rsdFlag := newFloatSlice([]float64{}, &rsd)
	flag.Var(rsdFlag, "o", searchHelp("-p"))

	flag.StringVar(&msg, "msg", msg, searchHelp("-msg"))
	flag.StringVar(&opt, "opt", opt, searchHelp("-opt"))
	flag.IntVar(&sta, "sta", sta, searchHelp("-sta"))
	flag.IntVar(&dispint, "d", dispint, searchHelp("-d"))
	flag.IntVar(&opts[0], "s", opts[0], searchHelp("-s"))
	flag.IntVar(&opts[1], "r", opts[1], searchHelp("-r"))
	flag.IntVar(&opts[5], "n", opts[5], searchHelp("-n"))
	flag.IntVar(&opts[6], "f", opts[6], searchHelp("-f"))
	flag.IntVar(&opts[7], "b", opts[7], searchHelp("-f"))
	flag.StringVar(&cmdfile[0], "c", cmdfile[0], searchHelp("-c"))
	flag.StringVar(&cmdfile[1], "c1", cmdfile[1], searchHelp("-c1"))
	flag.StringVar(&cmdfile[2], "c2", cmdfile[2], searchHelp("-c2"))
	flag.StringVar(&cmdfile[3], "c3", cmdfile[3], searchHelp("-c3"))
	flag.StringVar(&cmdfile[4], "c4", cmdfile[4], searchHelp("-c4"))
	flag.StringVar(&antinfo, "a", antinfo, searchHelp("-a"))
	flag.StringVar(&rcvinfo, "i", rcvinfo, searchHelp("-i"))
	flag.StringVar(&local, "l", local, searchHelp("-l"))
	flag.StringVar(&proxy, "x", proxy, searchHelp("-x"))
	flag.StringVar(&logfile, "fl", logfile, searchHelp("-fl"))
	flag.IntVar(&trlevel, "t", trlevel, searchHelp("-t"))
	flag.Parse()
	if flag.NFlag() < 1 {
		// if there is not any arguments, exit
		printhelp()
		return
	}
	if rpFlag.configured {
		pos[0] = pos[0] * gnssgo.D2R
		pos[1] = pos[1] * gnssgo.D2R
		gnssgo.Pos2Ecef(pos[:], stapos[:])
	}

	if len(infile) > 0 {
		if decodepath(infile, &types[0], &paths[0], &fmts[0]) == 0 {
			os.Exit(-1)
		}
	}

	if len(outfile) > 0 {
		n = 0
		ofs := strings.Split(outfile, ",")
		for _, v := range ofs {
			if decodepath(v, &types[n+1], &paths[n+1], &fmts[n+1]) == 0 {
				os.Exit(-1)
			}
			n++
		}
	}

	if n < 0 {
		n = 1
	}

	for i = 0; i < n; i++ {
		if fmts[i+1] <= 0 {
			continue
		}
		if fmts[i+1] != gnssgo.STRFMT_RTCM3 {
			fmt.Fprintf(os.Stderr, "unsupported output format\n")
			os.Exit(-1)
		}
		if fmts[0] < 0 {
			fmt.Fprintf(os.Stderr, "specify input format\n")
			os.Exit(-1)
		}
		ista := 0
		if sta != 0 {
			ista = 1
		}

		if conv[i] = gnssgo.NewStreamConv(fmts[0], fmts[i+1], msg, sta, ista, opt); conv[i] == nil {
			fmt.Fprintf(os.Stderr, "stream conversion error\n")
			os.Exit(-1)
		}
		buff = antinfo
		if len(buff) > 0 {
			ant = strings.Split(buff, ",")
			conv[i].RtcmOutput.StaPara.AntDes = ant[0]
			conv[i].RtcmOutput.StaPara.AntSno = ant[1]
			conv[i].RtcmOutput.StaPara.AntSetup, _ = strconv.Atoi(ant[2])
		}

		buff = rcvinfo
		if len(buff) > 0 {
			rcv = strings.Split(buff, ",")
			conv[i].RtcmOutput.StaPara.Type = rcv[0]
			conv[i].RtcmOutput.StaPara.RecVer = rcv[1]
			conv[i].RtcmOutput.StaPara.RecSN = rcv[2]
		}
		gnssgo.MatCpy(conv[i].RtcmOutput.StaPara.Pos[:], stapos[:], 3, 1)
		gnssgo.MatCpy(conv[i].RtcmOutput.StaPara.Del[:], stadel[:], 3, 1)
	}
	c := make(chan os.Signal)
	signal.Notify(c, syscall.SIGINT, syscall.SIGTERM, //syscall.SIGUSR2, //no defined under windows
		syscall.SIGHUP, syscall.SIGPIPE)
	go func() {
		for s := range c {
			switch s {
			case syscall.SIGINT, syscall.SIGTERM:

				sigfunc(2)
			case syscall.SIGHUP, syscall.SIGPIPE:
				fmt.Println("usr1 signal", s)
				//    case syscall.SIGUSR2:
				// 		   fmt.Println("usr2 signal", s)
			default:
				fmt.Println("other signal", s)
			}
		}
	}()

	strsvr.InitStreamSvr(n + 1)

	if trlevel > 0 {
		if len(logfile) > 0 {
			gnssgo.TraceOpen(logfile)
		} else {
			gnssgo.TraceOpen(TRFILE)
		}
		gnssgo.TraceLevel(trlevel)
	}
	fmt.Fprintf(os.Stderr, "stream server start\n")

	gnssgo.StreamSetDir(local)
	gnssgo.StreamSetProxy(proxy)

	for i = 0; i < MAXSTR; i++ {
		if len(cmdfile[i]) > 0 {
			readcmd(cmdfile[i], &cmds[i], 0)
		}
		if len(cmdfile[i]) > 0 {
			readcmd(cmdfile[i], &cmds_periodic[i], 2)
		}
	}
	/* start stream server */
	if strsvr.StreamSvrStart(opts, types[:], paths[:], logs[:], conv[:], cmds[:], cmds_periodic[:], stapos[:]) == 0 {
		fmt.Fprintf(os.Stderr, "stream server start error\n")
		os.Exit(-1)
	}
	for intrflg = 0; intrflg == 0; {
		buff = ""
		strmsg = ""
		/* get stream server status */
		strsvr.StreamSvrStat(stat[:], log_stat[:], bytes[:], bps[:], &strmsg)

		/* show stream server status */
		for i = 0; i < MAXSTR; i++ {
			buff += ss[stat[i]+1]
		}

		fmt.Fprintf(os.Stderr, "%s [%s] %10d B %7d bps %s\n",
			gnssgo.TimeStr(gnssgo.Utc2GpsT(gnssgo.TimeGet()), 0), buff, bytes[0], bps[0], strmsg)

		gnssgo.Sleepms(dispint)
	}
	for i = 0; i < MAXSTR; i++ {
		if len(cmdfile[i]) > 0 {
			readcmd(cmdfile[i], &cmds[i], 1)
		}
	}
	/* stop stream server */
	strsvr.StreamSvrStop(cmds[:])

	for i = 0; i < n; i++ {
		conv[i].FreeStreamConv()
	}
	if trlevel > 0 {
		gnssgo.TraceClose()
	}
	fmt.Fprintf(os.Stderr, "stream server stop\n")
	// return 0;
}
