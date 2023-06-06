/*------------------------------------------------------------------------------
* convbin.c : convert receiver binary log file to rinex obs/nav, sbas messages
*
*          Copyright (C) 2007-2020 by T.TAKASU, All rights reserved.
*
* options : -DWIN32 use windows file path separator
*
* version : $Revision: 1.1 $ $Date: 2008/07/17 22:13:04 $
* history : 2008/06/22 1.0 new
*           2009/06/17 1.1 support glonass
*           2009/12/19 1.2 fix bug on disable of glonass
*                          fix bug on inproper header for rtcm2 and rtcm3
*           2010/07/18 1.3 add option -v, -t, -h, -x
*           2011/01/15 1.4 add option -ro, -hc, -hm, -hn, -ht, -ho, -hr, -ha,
*                            -hp, -hd, -y, -c, -q
*                          support gw10 and javad receiver, galileo, qzss
*                          support rinex file name convention
*           2012/10/22 1.5 add option -scan, -oi, -ot, -ol
*                          change default rinex version to 2.11
*                          fix bug on default output directory (/ . .)
*                          support galileo nav (LNAV) output
*                          support compass
*           2012/11/19 1.6 fix bug on setting code mask in rinex options
*           2013/02/18 1.7 support binex
*           2013/05/19 1.8 support auto format for file path with wild-card
*           2014/02/08 1.9 add option -span -trace -mask
*           2014/08/26 1.10 add Trimble RT17 support
*           2014/12/26 1.11 add option -nomask
*           2016/01/23 1.12 enable septentrio
*           2016/05/25 1.13 fix bug on initializing output file paths in
*                           convbin()
*           2016/06/09 1.14 fix bug on output file with -v 3.02
*           2016/07/01 1.15 support log format CMR/CMR+
*           2016/07/31 1.16 add option -halfc
*           2017/05/26 1.17 add input format tersus
*           2017/06/06 1.18 fix bug on output beidou and irnss nav files
*                           add option -tt
*           2018/10/10 1.19 default options are changed.
*                             scan input file: off - on
*                             number of freq: 2 . 3
*                           add option -noscan
*           2020/11/30 1.20 include NavIC in default systems
*                           force option -scan
*                           delete option -noscan
*                           surppress warnings
*-----------------------------------------------------------------------------*/

package main

import (
	"flag"
	"fmt"
	"os"
	"strings"

	"gnssgo"
	"strconv"
)

const (
	PRGNAME   = "CONVBIN"
	TRACEFILE = "convbin.trace"
	NOUTFILE  = 9 /* number of output files */
)

/* help text -----------------------------------------------------------------*/
var help []string = []string{
	"",
	" Synopsys",
	"",
	" convbin [option ...] file",
	"",
	" Description",
	"",
	" Convert RTCM, receiver raw data log and RINEX file to RINEX and SBAS/LEX",
	" message file. SBAS message file complies with RTKLIB SBAS/LEX message",
	" format. It supports the following messages or files.",
	"",
	" RTCM 2                : Type 1, 3, 9, 14, 16, 17, 18, 19, 22",
	" RTCM 3                : Type 1002, 1004, 1005, 1006, 1010, 1012, 1019, 1020",
	"                         Type 1071-1127 (MSM except for compact msg)",
	" NovAtel OEMV/4,OEMStar: RANGECMPB, RANGEB, RAWEPHEMB, IONUTCB, RAWWASSFRAMEB",
	" NovAtel OEM3          : RGEB, REGD, REPB, FRMB, IONB, UTCB",
	" u-blox LEA-4T/5T/6T   : RXM-RAW, RXM-SFRB",
	" NovAtel Superstar II  : ID#20, ID#21, ID#22, ID#23, ID#67",
	" Hemisphere            : BIN76, BIN80, BIN94, BIN95, BIN96",
	" SkyTraq S1315F        : msg0xDD, msg0xE0, msg0xDC",
	" GW10                  : msg0x08, msg0x03, msg0x27, msg0x20",
	" Javad                 : [R*],[r*],[*R],[*r],[P*],[p*],[*P],[*p],[D*],[*d],",
	"                         [E*],[*E],[F*],[TC],[GE],[NE],[EN],[QE],[UO],[IO],",
	"                         [WD]",
	" NVS                   : BINR",
	" BINEX                 : big-endian, regular CRC, forward record (0xE2)",
	"                         0x01-01,0x01-02,0x01-03,0x01-04,0x01-06,0x7f-05",
	" Trimble               : RT17",
	" Septentrio            : SBF",
	" RINEX                 : OBS, NAV, GNAV, HNAV, LNAV, QNAV",
	"",
	" Options [default]",
	"",
	"     file         input receiver binary log file",
	"     -ts y/m/d,h:m:s  start time [all]",
	"     -te y/m/d,h:m:s  end time [all]",
	"     -tr y/m/d,h:m:s  approximated time for RTCM",
	"     -ti tint     observation data interval (s) [all]",
	"     -tt ttol     observation data epoch tolerance (s) [0.005]",
	"     -span span   time span (h) [all]",
	"     -r format    log format type",
	"                  rtcm2= RTCM 2",
	"                  rtcm3= RTCM 3",
	"                  nov  = NovAtel OEM/4/V/6/7,OEMStar",
	"                  oem3 = NovAtel OEM3",
	"                  ubx  = ublox LEA-4T/5T/6T/7T/M8T/F9",
	"                  ss2  = NovAtel Superstar II",
	"                  hemis= Hemisphere Eclipse/Crescent",
	"                  stq  = SkyTraq S1315F",
	"                  javad= Javad GREIS",
	"                  nvs  = NVS NV08C BINR",
	"                  binex= BINEX",
	"                  rt17 = Trimble RT17",
	"                  sbf  = Septentrio SBF",
	"                  rinex= RINEX",
	"     -ro opt      receiver options",
	"     -f freq      number of frequencies [5]",
	"     -hc comment  rinex header: comment line",
	"     -hm marker   rinex header: marker name",
	"     -hn markno   rinex header: marker number",
	"     -ht marktype rinex header: marker type",
	"     -ho observ   rinex header: oberver name and agency separated by /",
	"     -hr rec      rinex header: receiver number, type and version separated by /",
	"     -ha ant      rinex header: antenna number and type separated by /",
	"     -hp pos      rinex header: approx position x/y/z separated by /",
	"     -hd delta    rinex header: antenna delta h/e/n separated by /",
	"     -v ver       rinex version [3.04]",
	"     -od          include doppler frequency in rinex obs [on]",
	"     -os          include snr in rinex obs [on]",
	"     -oi          include iono correction in rinex nav header [off]",
	"     -ot          include time correction in rinex nav header [off]",
	"     -ol          include leap seconds in rinex nav header [off]",
	"     -halfc       half-cycle ambiguity correction [off]",
	"     -mask   [sig[,...]] signal mask(s) (sig={G|R|E|J|S|C|I}L{1C|1P|1W|...})",
	"     -nomask [sig[,...]] signal no mask (same as above)",
	"     -x sat       exclude satellite",
	"     -y sys       exclude systems (G:GPS,R:GLO,E:GAL,J:QZS,S:SBS,C:BDS,I:IRN)",
	"     -d dir       output directory [same as input file]",
	"     -c staid     use RINEX file name convention with staid [off]",
	"     -o ofile     output RINEX OBS file",
	"     -n nfile     output RINEX NAV file",
	"     -g gfile     output RINEX GNAV file",
	"     -h hfile     output RINEX HNAV file",
	"     -q qfile     output RINEX QNAV file",
	"     -l lfile     output RINEX LNAV file",
	"     -b cfile     output RINEX CNAV file",
	"     -i ifile     output RINEX INAV file",
	"     -s sfile     output SBAS message file",
	"     -trace level output trace level [off]",
	"",
	" If any output file specified, default output files (<file>.obs,",
	" <file>.nav, <file>.gnav, <file>.hnav, <file>.qnav, <file>.lnav,",
	" <file>.cnav, <file>.inav and <file>.sbs) are used. To obtain week number info",
	" for RTCM file, use -tr option to specify the approximated log start time.",
	" Without -tr option, the program obtains the week number from the time-tag file (if it exists) or the last modified time of the log file instead.",
	"",
	" If receiver type is not specified, type is recognized by the input",
	" file extension as follows.",
	"     *.rtcm2       RTCM 2",
	"     *.rtcm3       RTCM 3",
	"     *.gps         NovAtel OEM4/V/6/7,OEMStar",
	"     *.ubx         u-blox LEA-4T/5T/6T/7T/M8T/F9",
	"     *.log         NovAtel Superstar II",
	"     *.bin         Hemisphere Eclipse/Crescent",
	"     *.stq         SkyTraq S1315F",
	"     *.jps         Javad GREIS",
	"     *.bnx,*binex  BINEX",
	"     *.rt17        Trimble RT17",
	"     *.sbf         Septentrio SBF",
	"     *.obs,*.*o    RINEX OBS",
	"     *.rnx         RINEX OBS",
	"     *.nav,*.*n    RINEX NAV"}

/* print help ----------------------------------------------------------------*/
func printhelp() {
	for i := range help {
		fmt.Fprintf(os.Stderr, "%s\n", help[i])
	}
	os.Exit(0)
}

/* convert main --------------------------------------------------------------*/
func convbin(format int, opt *gnssgo.RnxOpt, ifile string, file []string, dir string) int {
	var (
		i, def         int
		work, ifile_   string
		ofile          [NOUTFILE]string = [NOUTFILE]string{"", "", "", "", "", "", "", "", ""}
		extnav, extlog string           = "P", "sbs"
	)
	if opt.RnxVer <= 299 || opt.NavSys == gnssgo.SYS_GPS {
		extnav = "N"
	}

	/* replace wild-card (*) in input file by 0 */
	ifile_ = ifile
	ifile_ = strings.Replace(ifile_, "*", "0", -1)

	for i = range file {
		if len(file[i]) == 0 {
			def++
		}
	}
	if def >= 8 && i >= 8 {
		def = 1
	} else {
		def = 0
	}

	switch {
	case len(file[0]) > 0:
		ofile[0] = file[0]
	case len(opt.Staid) > 0:
		ofile[0] = "%r%n0.%yO"
	case def > 0:
		ofile[0] = ifile_
		if idx := strings.LastIndex(ofile[0], "."); idx >= 0 {
			ofile[0] = ofile[0][:idx] + ".obs"
		} else {
			ofile[0] += ".obs"
		}
	}

	switch {
	case len(file[1]) > 0:
		ofile[1] = file[1]
	case len(opt.Staid) > 0:
		ofile[1] = "%r%n0.%y"
		ofile[1] += extnav
	case def > 0:
		ofile[1] = ifile_
		if idx := strings.LastIndex(ofile[1], "."); idx >= 0 {
			ofile[1] = ofile[1][:idx] + ".nav"
		} else {
			ofile[1] += ".nav"
		}
	}

	switch {
	case len(file[2]) > 0:
		ofile[2] = file[2]
	case opt.RnxVer <= 299 && len(opt.Staid) > 0:
		ofile[2] = "%r%n0.%yG"
	case opt.RnxVer <= 299 && def > 0:
		ofile[2] = ifile_
		if idx := strings.LastIndex(ofile[2], "."); idx >= 0 {
			ofile[2] = ofile[2][:idx] + ".gnav"
		} else {
			ofile[2] += ".gnav"
		}
	}

	switch {
	case len(file[3]) > 0:
		ofile[3] = file[3]
	case opt.RnxVer <= 299 && len(opt.Staid) > 0:
		ofile[3] = "%r%n0.%yH"
	case opt.RnxVer <= 299 && def > 0:
		ofile[3] = ifile_
		if idx := strings.LastIndex(ofile[3], "."); idx >= 0 {
			ofile[3] = ofile[3][:idx] + ".hnav"
		} else {
			ofile[3] += ".hnav"
		}
	}

	switch {
	case len(file[4]) > 0:
		ofile[4] = file[4]
	case opt.RnxVer <= 299 && len(opt.Staid) > 0:
		ofile[4] = "%r%n0.%yQ"
	case opt.RnxVer <= 299 && def > 0:
		ofile[4] = ifile_
		if idx := strings.LastIndex(ofile[4], "."); idx >= 0 {
			ofile[4] = ofile[4][:idx] + ".qnav"
		} else {
			ofile[4] += ".qnav"
		}
	}

	switch {
	case len(file[5]) > 0:
		ofile[5] = file[5]
	case opt.RnxVer <= 299 && len(opt.Staid) > 0:
		ofile[5] = "%r%n0.%yL"
	case opt.RnxVer <= 299 && def > 0:
		ofile[5] = ifile_
		if idx := strings.LastIndex(ofile[5], "."); idx >= 0 {
			ofile[5] = ofile[5][:idx] + ".lnav"
		} else {
			ofile[5] += ".lnav"
		}
	}

	switch {
	case len(file[6]) > 0:
		ofile[6] = file[6]
	case opt.RnxVer <= 299 && len(opt.Staid) > 0:
		ofile[6] = "%r%n0.%yC"
	case opt.RnxVer <= 299 && def > 0:
		ofile[6] = ifile_
		if idx := strings.LastIndex(ofile[6], "."); idx >= 0 {
			ofile[6] = ofile[6][:idx] + ".cnav"
		} else {
			ofile[6] += ".cnav"
		}
	}

	switch {
	case len(file[7]) > 0:
		ofile[7] = file[7]
	case opt.RnxVer <= 299 && len(opt.Staid) > 0:
		ofile[7] = "%r%n0.%yI"
	case opt.RnxVer <= 299 && def > 0:
		ofile[7] = ifile_
		if idx := strings.LastIndex(ofile[7], "."); idx >= 0 {
			ofile[7] = ofile[7][:idx] + ".inav"
		} else {
			ofile[7] += ".inav"
		}
	}

	switch {
	case len(file[8]) > 0:
		ofile[8] = file[8]
	case len(opt.Staid) > 0:
		ofile[8] = "%r%n0_%y."
		ofile[8] += extlog
	case def > 0:
		ofile[8] = ifile_
		if idx := strings.LastIndex(ofile[8], "."); idx >= 0 {
			ofile[8] = ofile[8][:idx] + "."
		} else {
			ofile[8] += "."
		}
		ofile[8] += extlog
	}

	for i = 0; i < NOUTFILE; i++ {
		if len(dir) == 0 || len(ofile[i]) == 0 {
			continue
		}
		if idx := strings.LastIndex(ofile[i], gnssgo.FILEPATHSEP); idx >= 0 {
			work = ofile[i][idx+1:]
		} else {
			work = ofile[i]
		}
		ofile[i] = fmt.Sprintf("%s%s%s", dir, gnssgo.FILEPATHSEP, work)
	}

	fmt.Fprintf(os.Stderr, "input file  : %s (%s)\n", ifile, gnssgo.FormatStrs[format])

	if len(ofile[0]) > 0 {
		fmt.Fprintf(os.Stderr, ".rinex obs : %s\n", ofile[0])
	}
	if len(ofile[1]) > 0 {
		fmt.Fprintf(os.Stderr, ".rinex nav : %s\n", ofile[1])
	}
	if len(ofile[2]) > 0 {
		fmt.Fprintf(os.Stderr, ".rinex gnav: %s\n", ofile[2])
	}
	if len(ofile[3]) > 0 {
		fmt.Fprintf(os.Stderr, ".rinex hnav: %s\n", ofile[3])
	}
	if len(ofile[4]) > 0 {
		fmt.Fprintf(os.Stderr, ".rinex qnav: %s\n", ofile[4])
	}
	if len(ofile[5]) > 0 {
		fmt.Fprintf(os.Stderr, ".rinex lnav: %s\n", ofile[5])
	}
	if len(ofile[6]) > 0 {
		fmt.Fprintf(os.Stderr, ".rinex cnav: %s\n", ofile[6])
	}
	if len(ofile[7]) > 0 {
		fmt.Fprintf(os.Stderr, ".rinex inav: %s\n", ofile[7])
	}
	if len(ofile[8]) > 0 {
		fmt.Fprintf(os.Stderr, ".sbas log  : %s\n", ofile[8])
	}

	if gnssgo.ConvRnx(format, opt, ifile, ofile[:]) == 0 {
		fmt.Fprintf(os.Stderr, "\n")
		return -1
	}
	fmt.Fprintf(os.Stderr, "\n")
	return 0
}

/* set signal mask -----------------------------------------------------------*/
func setmask(argv string, opt *gnssgo.RnxOpt, mask int) {
	var i, code int

	args := strings.Split(argv, ",")
	for _, v := range args {
		if len(v) < 4 || v[1] != 'L' {
			continue
		}
		switch v[0] {
		case 'G':
			i = 0
		case 'R':
			i = 1
		case 'E':
			i = 2
		case 'J':
			i = 3
		case 'S':
			i = 4
		case 'C':
			i = 5
		case 'I':
			i = 6
		default:
			continue
		}
		if code = int(gnssgo.Obs2Code(v[2:])); code > 0 {
			opt.Mask[i][code-1] = '0'
			if mask > 0 {
				opt.Mask[i][code-1] = '1'
			}
		}
	}
}
func BytesToUint32(b []byte) uint32 {
	_ = b[3] // bounds check hint to compiler; see golang.org/issue/14808
	return uint32(b[0]) | uint32(b[1])<<8 | uint32(b[2])<<16 | uint32(b[3])<<24
}

/* get start time of input file -----------------------------------------------*/
func get_filetime(file string, time *gnssgo.Gtime) int {
	var (
		fp             *os.File
		buff           [64]byte
		ep             [6]float64
		path, path_tag string
		paths          [1]string
	)

	if gnssgo.ExPath(file, paths[:], 1) == 0 {
		return 0
	}
	path = paths[0]

	/* get start time of time-tag file */
	path_tag = fmt.Sprintf("%.1019s.tag", path)
	if fp, _ = os.OpenFile(path_tag, os.O_RDONLY, 0666); fp != nil {
		defer fp.Close()
		if n, _ := fp.Read(buff[:1]); n == 1 {
			if strings.Compare(string(buff[:7]), "TIMETAG") == 0 {
				if n, _ = fp.Read(buff[:4]); n == 4 {
					time.Time = uint64(BytesToUint32(buff[:4]))
					time.Sec = 0.0
					return 1
				}
			}
		}
	}

	/* get modified time of input file */
	if fp2, _ := os.Open(path); fp2 != nil {
		defer fp2.Close()
		if stat, err := fp2.Stat(); err == nil {
			tm := stat.ModTime()
			ep[0] = float64(tm.Year())
			ep[1] = float64(tm.Month())
			ep[2] = float64(tm.Day())
			ep[3] = float64(tm.Hour())
			ep[4] = float64(tm.Minute())
			ep[5] = float64(tm.Second())
			*time = gnssgo.Utc2GpsT(gnssgo.Epoch2Time(ep[:]))
			return 1
		}
	}
	return 0
}

func searchHelp(key string) string {
	for _, v := range help {
		if strings.Contains(v, key) {
			return v
		}
	}
	return "no surported augument"
}

type timeFlag struct {
	time       *gnssgo.Gtime
	configured bool
}

func (f *timeFlag) Set(s string) error {
	var es []float64 = []float64{2000, 1, 1, 0, 0, 0}
	n, _ := fmt.Sscanf(s, "%f/%f/%f,%f:%f:%f", &es[0], &es[1], &es[2], &es[3], &es[4], &es[5])
	if n < 6 {
		return fmt.Errorf("too few argument")
	}
	*(f.time) = gnssgo.Epoch2Time(es)
	f.configured = true
	return nil
}
func (f *timeFlag) String() string {
	return "2000/1/1,0:0:0"
}
func newGtime(p *gnssgo.Gtime) *timeFlag {
	tf := timeFlag{p, false}
	return &tf
}

type posFlag struct {
	pos        []float64
	configured bool
}

func (f *posFlag) Set(s string) error {
	values := strings.Split(s, "/")
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

var nc int = 2

type arrayFlags []string

// Value ...
func (i *arrayFlags) String() string {
	return ""
}

// Set 方法是flag.Value接口, 设置flag Value的方法.
// 通过多个flag指定的值， 所以我们追加到最终的数组上.
func (i *arrayFlags) Set(value string) error {
	*i = append(*i, value)
	return nil
}

/* parse command line options ------------------------------------------------*/
func cmdopts(opt *gnssgo.RnxOpt, ifile *string, ofile []string, dir *string, trace *int) int {
	var (
		span, ver                                         float64
		i, j, k, sat                                      int
		nf, format                                        int = 5, -1
		fmts, sys, names, recs, ants, satid, mask, nomask string
		bod, bos, boi, bot, bol, bscan, bhalfc            bool
	)
	opt.RnxVer = 304
	opt.ObsType = gnssgo.OBSTYPE_PR | gnssgo.OBSTYPE_CP
	opt.NavSys = gnssgo.SYS_GPS | gnssgo.SYS_GLO | gnssgo.SYS_GAL | gnssgo.SYS_QZS | gnssgo.SYS_SBS | gnssgo.SYS_CMP | gnssgo.SYS_IRN

	for i = 0; i < 6; i++ {
		for j = 0; j < 64; j++ {
			opt.Mask[i][j] = '1'
		}
	}
	flag.Var(newGtime(&opt.TS), "ts", searchHelp("-ts"))
	flag.Var(newGtime(&opt.TE), "te", searchHelp("-te"))
	flag.Var(newGtime(&opt.TRtcm), "tr", searchHelp("-tr"))
	flag.Float64Var(&opt.TInt, "ti", opt.TInt, searchHelp("-ti"))
	flag.Float64Var(&opt.TTol, "tt", opt.TTol, searchHelp("-tt"))
	flag.Float64Var(&span, "span", span, searchHelp("-span"))
	flag.StringVar(&fmts, "r", fmts, searchHelp("-r"))
	flag.StringVar(&opt.RcvOpt, "ro", opt.RcvOpt, searchHelp("-ro"))
	flag.IntVar(&nf, "f", nf, searchHelp("-f"))
	var comments arrayFlags
	flag.Var(&comments, "hc", searchHelp("-hc"))
	flag.StringVar(&opt.Marker, "hm", opt.Marker, searchHelp("-hm"))
	flag.StringVar(&opt.MarkerNo, "hn", opt.MarkerNo, searchHelp("-hn"))
	flag.StringVar(&opt.MarkerType, "ht", opt.MarkerType, searchHelp("-ht"))

	flag.StringVar(&names, "ho", names, searchHelp("-ho"))
	flag.StringVar(&recs, "hr", recs, searchHelp("-hr"))
	flag.StringVar(&ants, "ha", ants, searchHelp("-ha"))

	rp := opt.AppPos[:]
	rpFlag := newFloatSlice([]float64{}, &rp)
	flag.Var(rpFlag, "hp", searchHelp("-hp"))

	rd := opt.AntDel[:]
	rdFlag := newFloatSlice([]float64{}, &rd)
	flag.Var(rdFlag, "hd", searchHelp("-hd"))

	flag.Float64Var(&ver, "v", ver, searchHelp("-v"))

	flag.BoolVar(&bod, "od", bod, searchHelp("-od"))
	flag.BoolVar(&bos, "os", bos, searchHelp("-os"))
	flag.BoolVar(&boi, "oi", boi, searchHelp("-oi"))
	flag.BoolVar(&bot, "ot", bot, searchHelp("-ot"))
	flag.BoolVar(&bol, "ol", bol, searchHelp("-ol"))
	flag.BoolVar(&bscan, "scan", bscan, searchHelp("-scan"))
	flag.BoolVar(&bhalfc, "halfc", bod, searchHelp("-halfc"))
	flag.StringVar(&nomask, "nomask", nomask, searchHelp("-nomask"))
	flag.StringVar(&mask, "mask", mask, searchHelp("-mask"))

	flag.StringVar(&satid, "x", satid, searchHelp("-x"))
	flag.StringVar(&sys, "y", sys, searchHelp("-y"))
	flag.StringVar(dir, "d", *dir, searchHelp("-d"))
	flag.StringVar(&opt.Staid, "c", opt.Staid, searchHelp("-c"))

	flag.StringVar(&ofile[0], "o", ofile[0], searchHelp("-o"))
	flag.StringVar(&ofile[1], "n", ofile[1], searchHelp("-n"))
	flag.StringVar(&ofile[2], "g", ofile[2], searchHelp("-g"))
	flag.StringVar(&ofile[3], "h", ofile[3], searchHelp("-h"))
	flag.StringVar(&ofile[4], "q", ofile[4], searchHelp("-q"))
	flag.StringVar(&ofile[5], "l", ofile[5], searchHelp("-l"))
	flag.StringVar(&ofile[6], "b", ofile[6], searchHelp("-b"))
	flag.StringVar(&ofile[7], "i", ofile[7], searchHelp("-i"))
	flag.StringVar(&ofile[8], "s", ofile[8], searchHelp("-s"))

	flag.IntVar(trace, "trace", *trace, searchHelp("-trace"))

	flag.Parse()

	if len(comments) > 0 {
		for _, v := range comments {
			if nc < gnssgo.MAXCOMMENT {
				opt.Comment[nc] = v
				nc++
			}
		}
	}

	if len(names) > 0 {
		p := strings.Split(names, "/")
		for j = range p {
			opt.Name[j] = p[j]
			if j > 1 {
				break
			}
		}
	}

	if len(recs) > 0 {
		p := strings.Split(recs, "/")
		for j = range p {
			opt.Rec[j] = p[j]
			if j > 2 {
				break
			}
		}
	}

	if len(ants) > 0 {
		p := strings.Split(ants, "/")
		for j = range p {
			opt.Ant[j] = p[j]
			if j > 2 {
				break
			}
		}
	}

	if ver > 0 {
		opt.RnxVer = int(ver * 100.0)
	}

	if bod {
		opt.ObsType |= gnssgo.OBSTYPE_DOP
	}

	if bos {
		opt.ObsType |= gnssgo.OBSTYPE_SNR
	}
	if boi {
		opt.Outiono = 1
	}

	if bot {
		opt.OutputTime = 1
	}

	if bol {
		opt.Outleaps = 1
	}

	// if bscan {

	// }

	if bhalfc {
		opt.Halfcyc = 1
	}

	if len(mask) > 0 {
		for j = 0; j < 6; j++ {
			for k = 0; k < 64; k++ {
				opt.Mask[j][k] = '0'
			}
		}
		setmask(mask, opt, 1)
	}

	if len(nomask) > 0 {
		setmask(nomask, opt, 0)
	}

	if len(satid) > 0 {
		if sat = gnssgo.SatId2No(satid); sat > 0 {
			opt.ExSats[sat-1] = 1
		}
	}

	if len(sys) > 0 {
		switch sys[0] {
		case 'G':
			opt.NavSys &= ^gnssgo.SYS_GPS
		case 'R':
			opt.NavSys &= ^gnssgo.SYS_GLO
		case 'E':
			opt.NavSys &= ^gnssgo.SYS_GAL
		case 'J':
			opt.NavSys &= ^gnssgo.SYS_QZS
		case 'S':
			opt.NavSys &= ^gnssgo.SYS_SBS
		case 'C':
			opt.NavSys &= ^gnssgo.SYS_CMP
		case 'I':
			opt.NavSys &= ^gnssgo.SYS_IRN
		}
	}

	if flag.NFlag() < 1 {
		// if there is not any arguments, exit
		printhelp()
		os.Exit(0)
	}
	args := flag.CommandLine.Args()
	if len(args) > 0 {
		*ifile = args[0]
	}

	if span > 0.0 && opt.TStart.Time > 0 {
		opt.TEnd = gnssgo.TimeAdd(opt.TStart, span*3600.0-1e-3)
	}
	if nf >= 1 {
		opt.FreqType |= gnssgo.FREQTYPE_L1
	}
	if nf >= 2 {
		opt.FreqType |= gnssgo.FREQTYPE_L2
	}
	if nf >= 3 {
		opt.FreqType |= gnssgo.FREQTYPE_L3
	}
	if nf >= 4 {
		opt.FreqType |= gnssgo.FREQTYPE_L4
	}
	if nf >= 5 {
		opt.FreqType |= gnssgo.FREQTYPE_L5
	}

	if opt.TRtcm.Time == 0 {
		get_filetime(*ifile, &opt.TRtcm)
	}

	if len(fmts) > 0 {
		switch fmts {
		case "rtcm2":
			format = gnssgo.STRFMT_RTCM2
		case "rtcm3":
			format = gnssgo.STRFMT_RTCM3
		case "nov":
			format = gnssgo.STRFMT_OEM4
		case "oem3":
			format = gnssgo.STRFMT_OEM3
		case "ubx":
			format = gnssgo.STRFMT_UBX
		case "ss2":
			format = gnssgo.STRFMT_SS2
		case "hemis":
			format = gnssgo.STRFMT_CRES
		case "stq":
			format = gnssgo.STRFMT_STQ
		case "javad":
			format = gnssgo.STRFMT_JAVAD
		case "nvs":
			format = gnssgo.STRFMT_NVS
		case "binex":
			format = gnssgo.STRFMT_BINEX
		case "rt17":
			format = gnssgo.STRFMT_RT17
		case "sbf":
			format = gnssgo.STRFMT_SEPT
		case "rinex":
			format = gnssgo.STRFMT_RINEX
		}
	} else {
		var paths [1]string
		var path string
		var idx int
		if gnssgo.ExPath(*ifile, paths[:], 1) > 0 {
			path = paths[0]
			if idx = strings.LastIndex(path, "."); idx < 0 {
				return -1
			}
		} else {
			return -1
		}

		switch {
		case path[idx:] == ".rtcm2":
			format = gnssgo.STRFMT_RTCM2
		case path[idx:] == ".rtcm3":
			format = gnssgo.STRFMT_RTCM3
		case path[idx:] == ".gps":
			format = gnssgo.STRFMT_OEM4
		case path[idx:] == ".ubx":
			format = gnssgo.STRFMT_UBX
		case path[idx:] == ".log":
			format = gnssgo.STRFMT_SS2
		case path[idx:] == ".bin":
			format = gnssgo.STRFMT_CRES
		case path[idx:] == ".stq":
			format = gnssgo.STRFMT_STQ
		case path[idx:] == ".jps":
			format = gnssgo.STRFMT_JAVAD
		case path[idx:] == ".bnx":
			format = gnssgo.STRFMT_BINEX
		case path[idx:] == ".binex":
			format = gnssgo.STRFMT_BINEX
		case path[idx:] == ".rt17":
			format = gnssgo.STRFMT_RT17
		case path[idx:] == ".sbf":
			format = gnssgo.STRFMT_SEPT
		case path[idx:] == ".obs":
			format = gnssgo.STRFMT_RINEX
		case path[idx+3:] == "o":
			format = gnssgo.STRFMT_RINEX
		case path[idx+3:] == "O":
			format = gnssgo.STRFMT_RINEX
		case path[idx:] == ".rnx":
			format = gnssgo.STRFMT_RINEX
		case path[idx:] == ".nav":
			format = gnssgo.STRFMT_RINEX
		case path[idx+3:] == "n":
			format = gnssgo.STRFMT_RINEX
		case path[idx+3:] == "N":
			format = gnssgo.STRFMT_RINEX
		}
	}
	return format
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

/* main ----------------------------------------------------------------------*/
func main() {
	var (
		opt                 gnssgo.RnxOpt
		format, trace, stat int
		ifile, dir          string
		ofile               [NOUTFILE]string
	)
	/* parse command line options */
	format = cmdopts(&opt, &ifile, ofile[:], &dir, &trace)

	if len(ifile) == 0 {
		fmt.Fprintf(os.Stderr, "no input file\n")
		os.Exit(-1)
	}
	if format < 0 {
		fmt.Fprintf(os.Stderr, "input format can not be recognized\n")
		os.Exit(-1)
	}
	opt.Prog = fmt.Sprintf("%s %s", PRGNAME, gnssgo.VER_GNSSGO)

	if trace > 0 {
		gnssgo.TraceOpen(TRACEFILE)
		gnssgo.TraceLevel(trace)
	}

	gnssgo.ShowMsg_Ptr = showmsg

	stat = convbin(format, &opt, ifile, ofile[:], dir)

	gnssgo.TraceClose()

	os.Exit(stat)
}
