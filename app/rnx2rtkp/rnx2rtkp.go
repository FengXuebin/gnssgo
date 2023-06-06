/*------------------------------------------------------------------------------
* rnx2rtkp.c : read rinex obs/nav files and compute receiver positions
*
*          Copyright (C) 2007-2016 by T.TAKASU, All rights reserved.
*
* version : $Revision: 1.1 $ $Date: 2008/07/17 21:55:16 $
* history : 2007/01/16  1.0 new
*           2007/03/15  1.1 add library mode
*           2007/05/08  1.2 separate from postpos.c
*           2009/01/20  1.3 support rtklib 2.2.0 api
*           2009/12/12  1.4 support glonass
*                           add option -h, -a, -l, -x
*           2010/01/28  1.5 add option -k
*           2010/08/12  1.6 add option -y implementation (2.4.0_p1)
*           2014/01/27  1.7 fix bug on default output time format
*           2015/05/15  1.8 -r or -l options for fixed or ppp-fixed mode
*           2015/06/12  1.9 output patch level in header
*           2016/09/07  1.10 add option -sys
*-----------------------------------------------------------------------------*/

package main

import (
	"flag"
	"fmt"
	"os"
	"strconv"
	"strings"

	"gnssgo"
)

var PROGNAME string = "rnx2rtkp"

/* help text -----------------------------------------------------------------*/
var help []string = []string{
	"",
	" usage: rnx2rtkp [option]... file file [...]",
	"",
	" Read RINEX OBS/NAV/GNAV/HNAV/CLK, SP3, SBAS message log files and ccompute ",
	" receiver (rover) positions and output position solutions.",
	" The first RINEX OBS file shall contain receiver (rover) observations. For the",
	" relative mode, the second RINEX OBS file shall contain reference",
	" (base station) receiver observations. At least one RINEX NAV/GNAV/HNAV",
	" file shall be included in input files. To use SP3 precise ephemeris, specify",
	" the path in the files. The extension of the SP3 file shall be .sp3 or .eph.",
	" All of the input file paths can include wild-cards (*). To avoid command",
	" line deployment of wild-cards, use \"...\" for paths with wild-cards.",
	" Command line options are as follows ([]:default). With -k option, the",
	" processing options are input from the configuration file. In this case,",
	" command line options precede options in the configuration file.",
	"",
	" -?        print help",
	" -k file   input options from configuration file [off]",
	" -o file   set output file [stdout]",
	" -ts ds,ts start day/time (ds=y/m/d ts=h:m:s) [obs start time]",
	" -te de,te end day/time   (de=y/m/d te=h:m:s) [obs end time]",
	" -ti tint  time interval (sec) [all]",
	" -p mode   mode (0:single,1:dgps,2:kinematic,3:static,4:moving-base,",
	"                 5:fixed,6:ppp-kinematic,7:ppp-static) [2]",
	" -m mask   elevation mask angle (deg) [15]",
	" -sys s[,s...] nav system(s) (s=G:GPS,R:GLO,E:GAL,J:QZS,C:BDS,I:IRN) [G|R]",
	" -f freq   number of frequencies for relative mode (1:L1,2:L1+L2,3:L1+L2+L5) [2]",
	" -v thres  validation threshold for integer ambiguity (0.0:no AR) [3.0]",
	" -b        backward solutions [off]",
	" -c        forward/backward combined solutions [off]",
	" -i        instantaneous integer ambiguity resolution [off]",
	" -h        fix and hold for integer ambiguity resolution [off]",
	" -e        output x/y/z-ecef position [latitude/longitude/height]",
	" -a        output e/n/u-baseline [latitude/longitude/height]",
	" -n        output NMEA-0183 GGA sentence [off]",
	" -g        output latitude/longitude in the form of ddd mm ss.ss' [ddd.ddd]",
	" -t        output time in the form of yyyy/mm/dd hh:mm:ss.ss [sssss.ss]",
	" -u        output time in utc [gpst]",
	" -d col    number of decimals in time [3]",
	" -s sep    field separator [' ']",
	" -r x,y,z  reference (base) receiver ecef pos (m) [average of single pos]",
	"           rover receiver ecef pos (m) for fixed or ppp-fixed mode",
	" -l lat,lon,hgt reference (base) receiver latitude/longitude/height (deg/m)",
	"           rover latitude/longitude/height for fixed or ppp-fixed mode",
	" -y level  output soltion status (0:off,1:states,2:residuals) [0]",
	" -x level  debug trace level (0:off) [0]"}

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

type navsysFlag struct {
	sys *int
}

func (f *navsysFlag) Set(s string) error {
	sys := strings.Split(s, ",")
	if len(sys) == 0 {
		return fmt.Errorf("null argument")
	}
	var r int
	for _, v := range sys {
		switch v[0] {
		case 'G':
			r |= gnssgo.SYS_GPS
		case 'R':
			r |= gnssgo.SYS_GLO
		case 'E':
			r |= gnssgo.SYS_GAL
		case 'J':
			r |= gnssgo.SYS_QZS
		case 'C':
			r |= gnssgo.SYS_CMP
		case 'I':
			r |= gnssgo.SYS_IRN
		}
	}
	*f.sys = r
	return nil
}
func (f *navsysFlag) String() string {
	return "G,R"
}
func newNavSys(value int, p *int) *navsysFlag {
	nf := navsysFlag{p}
	return &nf
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

/* rnx2rtkp main -------------------------------------------------------------*/
func main() {
	var (
		filopt                                             gnssgo.FilOpt
		ts, te                                             gnssgo.Gtime
		tint                                               float64 = 0.0
		pos                                                [3]float64
		j, n                                               int
		infiles                                            []string
		outfile                                            string = ""
		config                                             string = ""
		soltype1, soltype2, modear2, modear3, timef, times bool
		posfxyz, posfenu, posfnmea, degf                   bool
	)
	prcopt := gnssgo.DefaultProcOpt()
	solopt := gnssgo.DefaultSolOpt()

	prcopt.Mode = gnssgo.PMODE_KINEMA
	prcopt.NavSys = 0
	prcopt.RefPos = 1
	prcopt.GloModeAr = 1
	solopt.TimeF = 0
	solopt.Prog = fmt.Sprintf("%s ver.%s %s", PROGNAME, gnssgo.VER_GNSSGO, gnssgo.PATCH_LEVEL)
	filopt.Trace = fmt.Sprintf("%s.trace", PROGNAME)

	gnssgo.ShowMsg_Ptr = showmsg

	flag.StringVar(&outfile, "o", outfile, searchHelp("-o"))
	flag.Var(newGtime(&ts), "ts", searchHelp("-ts"))
	flag.Var(newGtime(&te), "te", searchHelp("-te"))
	flag.Float64Var(&tint, "ti", tint, searchHelp("-ti"))
	flag.IntVar(&prcopt.Mode, "p", prcopt.Mode, searchHelp("-p"))
	flag.StringVar(&config, "k", config, searchHelp("-k"))
	flag.IntVar(&prcopt.Nf, "f", prcopt.Nf, searchHelp("-f"))
	nsFlag := newNavSys(prcopt.NavSys, &prcopt.NavSys)
	flag.Var(nsFlag, "sys", searchHelp("-sys"))
	flag.Float64Var(&prcopt.Elmin, "m", prcopt.Elmin, searchHelp("-m"))
	flag.Float64Var(&prcopt.ThresAr[0], "v", prcopt.ThresAr[0], searchHelp("-v"))
	flag.StringVar(&solopt.Sep, "s", solopt.Sep, searchHelp("-s"))
	flag.IntVar(&solopt.TimeU, "d", solopt.TimeU, searchHelp("-d"))
	flag.BoolVar(&soltype1, "b", false, searchHelp("-b"))
	flag.BoolVar(&soltype2, "c", false, searchHelp("-c"))
	flag.BoolVar(&modear2, "i", false, searchHelp("-i"))
	flag.BoolVar(&modear3, "h", false, searchHelp("-h"))
	flag.BoolVar(&timef, "t", false, searchHelp("-t"))
	flag.BoolVar(&times, "u", false, searchHelp("-u"))
	flag.BoolVar(&posfxyz, "e", false, searchHelp("-e"))
	flag.BoolVar(&posfenu, "a", false, searchHelp("-a"))
	flag.BoolVar(&posfnmea, "n", false, searchHelp("-n"))
	flag.BoolVar(&degf, "g", false, searchHelp("-g"))
	rb := prcopt.Rb[:]
	rbFlag := newFloatSlice([]float64{}, &rb)
	rp := pos[:]
	rpFlag := newFloatSlice([]float64{}, &rp)
	flag.Var(rbFlag, "r", searchHelp("-r"))
	flag.Var(rpFlag, "l", searchHelp("-l"))
	flag.IntVar(&solopt.SStat, "y", solopt.SStat, searchHelp("-y"))
	flag.IntVar(&solopt.Trace, "x", solopt.Trace, searchHelp("-x"))

	flag.Parse()

	if flag.NFlag() < 1 {
		// if there is not any arguments, exit
		for _, h := range help {
			fmt.Printf("%s\n", h)
		}
		return
	}

	if len(config) > 0 {
		gnssgo.ResetSysOpts()
		if gnssgo.LoadOpts(config, &gnssgo.SysOpts) == 0 {
			fmt.Fprintf(os.Stderr, "no options file: %s. defaults used\n", config)
			return
		}
		gnssgo.GetSysOpts(&prcopt, &solopt, &filopt)
	}

	flag.Parse()

	if soltype1 {
		prcopt.SolType = 1
	}
	if soltype2 {
		prcopt.SolType = 2
	}
	if modear2 {
		prcopt.ModeAr = 2
	}
	if modear3 {
		prcopt.ModeAr = 3
	}
	if timef {
		solopt.TimeF = 1
	}
	if times {
		solopt.TimeS = gnssgo.TIMES_UTC
	}
	if posfxyz {
		solopt.Posf = gnssgo.SOLF_XYZ
	}
	if posfenu {
		solopt.Posf = gnssgo.SOLF_ENU
	}
	if posfnmea {
		solopt.Posf = gnssgo.SOLF_NMEA
	}
	if degf {
		solopt.DegF = 1
	}
	if rbFlag.configured {
		prcopt.RefPos = 0
		prcopt.RovPos = 0
		gnssgo.MatCpy(prcopt.Ru[:], prcopt.Rb[:], 3, 1)
	}
	if rpFlag.configured {
		for j = 0; j < 2; j++ {
			pos[j] *= gnssgo.D2R
		}
		gnssgo.Pos2Ecef(pos[:], prcopt.Rb[:])
		gnssgo.MatCpy(prcopt.Ru[:], prcopt.Rb[:], 3, 1)
	}
	infiles = flag.CommandLine.Args()
	if n = len(infiles); n < 1 {
		return
	}
	if prcopt.NavSys == 0 {
		prcopt.NavSys = gnssgo.SYS_ALL
	}

	ret := gnssgo.PostPos(ts, te, tint, 0.0, &prcopt, &solopt, &filopt, infiles[:], n, &outfile, "", "")
	if ret == 0 {
		fmt.Printf("%40s\r", "")
	}
}
