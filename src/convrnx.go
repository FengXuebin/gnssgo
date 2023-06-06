/*------------------------------------------------------------------------------
* convrnx.c : rinex translator for rtcm and receiver raw data log
*
*          Copyright (C) 2009-2020 by T.TAKASU, All rights reserved.
*
* version : $Revision: 1.2 $ $Date: 2008/07/17 21:48:06 $
* history : 2009/04/10 1.0  new
*           2009/06/02 1.1  support glonass
*           2009/12/18 1.2  add check return of init_rtcm()/init_raw()
*           2010/07/15 1.3  support wildcard expansion of input file
*                           support rinex 3.00
*                           support rinex as input format
*                           support output of geo navigation message
*                           support rtcm antenna and receiver info
*                           changed api:
*                               convrnx()
*           2011/05/27 1.4  support GW10, Javad, LEX receiver
*                           support lex message conversion
*                           change api convrnx()
*           2012/10/18 1.5  support multiple codes in a frequency
*           2012/10/29 1.6  fix bug on scanning obs types
*                           support output of compass navigation data
*                           add supported obs types for rinex input
*           2013/03/11 1.7  support binex and rinex 3.02
*                           add approx position in rinex obs header if blank
*           2014/05/24 1.8  support beidou B1
*           2014/08/26 1.9  support input format rt17
*           2015/05/24 1.10 fix bug on setting antenna delta in rtcm2opt()
*           2016/07/04 1.11 support IRNSS
*           2016/10/10 1.12 support event output by staid change in rtcm
*                           support separated navigation files for ver.3
*           2017/06/06 1.13 fix bug on array overflow in set_obstype() and
*                           scan_obstype()
*           2018/10/10 1.14 add trace of half-cycle ambiguity status
*                           fix bug on missing navigation data
*           2020/11/30 1.15 force scanning receiver log for obs-types (2-pass)
*                           delete scanobs in RINEX options (rnxopt_t)
*                           add phase shift option (phshift) in rnxopt_t
*                           sort obs-types by freq-index and code priority
*                           add test obs-types supportted by RINEX versions
*                           support receiver/antenna info in raw data
*                           fix bug on writing BDS/IRN nav header in closefile()
*                           fix bug on screening time in screent_ttol()
*                           fix bug on screening QZS L1S messages as SBAS
*                           use integer types in stdint.h
*		    2022/05/31 1.0  rewrite convrnx.c with golang by fxb
*-----------------------------------------------------------------------------*/
package gnssgo

import (
	"bufio"
	"fmt"
	"math"
	"os"
	"strings"
)

const (
	NOUTFILE        = 9    /* number of output files */
	NSATSYS         = 7    /* number of satellite systems */
	TSTARTMARGIN    = 60.0 /* time margin for file name replacement */
	EVENT_STARTMOVE = 2    /* rinex event start moving antenna */
	EVENT_NEWSITE   = 3    /* rinex event new site occupation */
	EVENT_HEADER    = 4    /* rinex event header info follows */
	EVENT_EXTERNAL  = 5    /* rinex event external event */
)

/* type definitions ----------------------------------------------------------*/

type Stas struct { /* station list type */
	staid  int   /* station IS */
	ts, te Gtime /* first and last observation time */
	sta    Sta   /* station parameters */
	next   *Stas /* next list */
}

type Halfc struct { /* half-cycle ambiguity list type */
	ts, te Gtime  /* first and last observation time */
	stat   uint8  /* half-cycle ambiguity status */
	next   *Halfc /* next list */
}

type StreamFile struct { /* stream file type */
	format         int                            /* stream format (STRFMT_???) */
	staid          int                            /* station ID */
	ephsat, ephset int                            /* satelite and set of input ephemeris */
	time           Gtime                          /* current time */
	tstart         Gtime                          /* start time */
	obs            *Obs                           /* pointer to input observation data */
	nav            *Nav                           /* pointer to input navigation data */
	sta            *Sta                           /* pointer to input station parameters */
	rtcm           Rtcm                           /* input RTCM data */
	raw            Raw                            /* input receiver raw data */
	rnx            RnxCtr                         /* input RINEX control data */
	stas           *Stas                          /* station list */
	slips          [MAXSAT][NFREQ + NEXOBS]uint8  /* cycle slip flag cache */
	halfc          [MAXSAT][NFREQ + NEXOBS]*Halfc /* half-cycle ambiguity list */
	fp             *os.File                       /* output file pointer */
}

// Support B1C B2a Signal by cjb 2021-12-24
// RTCM MSM数据中北斗的signal mask从0~31，B2a data的mask为21，B2a pilot的mask为22，B2a data+pilot的mask为23，B1C data的mask为29，B1C pilot的mask为30，B1C data+pilot的mask为31
// 5D 5P 5X 1D 1P 1X
// static const char vercode[][MAXCODE]={ /* supported obs-type by RINEX version */
//
//	 /* 0........1.........2.........3.........4.........5.........6........          */
//	 /* 11111111111112222222222555777666666688822663331155599991555677788444     CODE */
//	 /* CPWYMNSLEABXZCDSLXPWYMNIQXIQXABCXZSLIQXIQIQIQXIQABCABCXDDPZEDPZDPABX          */
//	   "00000000...0.0000000000000..........................................", /* GPS */
//	   "00...........0....0..........44.4..........222...................444", /* GLO */
//	   "0........0000..........0000000000...000.............................", /* GAL */
//	   "2.....22...22..222.....222......2422....................4444........", /* QZS */
//	   "0......................000..........................................", /* SBS */
//	   ".4...4...4.4.....1.......41114..1.....41111............444..44444...", /* BDS */
//	   ".........................3......................3333333............."  /* IRN */
//	};
var vercode []string = []string{ /* supported obs-type by RINEX version */
	/* 0........1.........2.........3.........4.........5.........6........          */
	/* 11111111111112222222222555777666666688822663331155599991555677788444     CODE */
	/* CPWYMNSLEABXZCDSLXPWYMNIQXIQXABCXZSLIQXIQIQIQXIQABCABCXDDPZEDPZDPABX          */
	"00000000...0.0000000000000..........................................", /* GPS */
	"00...........0....0..........44.4..........222...................444", /* GLO */
	"0........0000..........0000000000...000.............................", /* GAL */
	"2.....22...22..222.....222......2422....................4444........", /* QZS */
	"0......................000..........................................", /* SBS */
	".4...4...4.4.....1......441114..1.....41111...........4444..44444...", /* BDS */
	".........................3......................3333333............." /* IRN */}

/* convert RINEX obs-type ver.3 => ver.2 -------------------------------------*/
func ConvRinexCode3_2(rnxver, sys int, t *string) {
	var str string = *t
	s := str[1:3]
	switch {
	case rnxver >= 212 &&
		(sys == SYS_GPS || sys == SYS_QZS || sys == SYS_SBS) && s == "1C": /* L1C/A */
		str = str[:1] + "A"
	case rnxver >= 212 && (sys == SYS_GPS || sys == SYS_QZS) && (s == "1S" || s == "1L" || s == "1X"): /* L1C */
		str = str[:1] + "B"
	case rnxver >= 212 && (sys == SYS_GPS || sys == SYS_QZS) && (s == "2S" || s == "2L" || s == "2X"): /* L2C */
		str = str[:1] + "C"
	case rnxver >= 212 && sys == SYS_GLO && s == "1C": /* L1C/A */
		str = str[:1] + "A"
	case rnxver >= 212 && sys == SYS_GLO && s == "2C": /* L2C/A */
		str = str[:1] + "D"
	case sys == SYS_CMP && (s == "2I" || s == "2Q" || s == "2X"): /* B1_2 */
		str = str[:1] + "2"
	case s == "C1P" || s == "C1W" || s == "C1Y" || s == "C1N": /* L1P,P(Y) */
		str = "P1"
	case s == "C2P" || s == "C2W" || s == "C2Y" || s == "C2N" || s == "C2D": /* L2P,P(Y) */
		str = "P2"
	default:
		str = str[:2]

	}
	*t = str
}

/* generate stream file ------------------------------------------------------*/
func GenStreamfile(format int, opt string) *StreamFile {
	var (
		str   StreamFile
		time0 Gtime
		i, j  int
	)

	Trace(4, "init_strfile:\n")

	str.format = format
	str.staid = -1
	str.ephsat, str.ephset = 0, 0
	str.time, str.tstart = time0, time0
	switch {
	case format == STRFMT_RTCM2 || format == STRFMT_RTCM3:
		if str.rtcm.InitRtcm() == 0 {
			ShowMsg_Ptr("init rtcm error")
			return nil
		}
		str.rtcm.Time = time0
		str.obs = &str.rtcm.ObsData
		str.nav = &str.rtcm.NavData
		str.sta = &str.rtcm.StaPara
		str.rtcm.Opt = opt
	case format <= MAXRCVFMT:
		if str.raw.InitRaw(format) == 0 {
			ShowMsg_Ptr("init raw error")
			return nil
		}
		str.raw.Time = time0
		str.obs = &str.raw.ObsData
		str.nav = &str.raw.NavData
		str.sta = &str.raw.StaData
		str.raw.Opt = opt
	case format == STRFMT_RINEX:
		if InitRnxCtr(&str.rnx) == 0 {
			ShowMsg_Ptr("init rnx error")
			return nil
		}
		str.rnx.time = time0
		str.obs = &str.rnx.obs
		str.nav = &str.rnx.nav
		str.sta = &str.rnx.sta
		str.rnx.opt = opt
	default:
		return nil
	}

	str.stas = nil
	for i = 0; i < MAXSAT; i++ {
		for j = 0; j < NFREQ+NEXOBS; j++ {
			str.slips[i][j] = 0
			str.halfc[i][j] = nil
		}
	}
	str.fp = nil
	return &str
}

/* free stream file ----------------------------------------------------------*/
func (str *StreamFile) FreeStrfile() {

	Trace(4, "free_strfile:\n")

	str.stas = nil
}

/* input stream file ---------------------------------------------------------*/
func (str *StreamFile) InputStreamFile(rd *bufio.Reader) int {
	var t int = 0

	Trace(4, "input_strfile:\n")

	switch {
	case str.format == STRFMT_RTCM2:
		if t = str.rtcm.InputRtcm2f(str.fp); t >= 1 {
			str.time = str.rtcm.Time
			str.ephsat = str.rtcm.EphSat
			str.ephset = str.rtcm.EphSet
			str.staid = str.rtcm.StaId
		}
	case str.format == STRFMT_RTCM3:
		if t = str.rtcm.InputRtcm3f(str.fp); t >= 1 {
			str.time = str.rtcm.Time
			str.ephsat = str.rtcm.EphSat
			str.ephset = str.rtcm.EphSet
			str.staid = str.rtcm.StaId
		}
	case str.format <= MAXRCVFMT:
		if t = str.raw.InputRawF(str.format, str.fp); t >= 1 {
			str.time = str.raw.Time
			str.ephsat = str.raw.EphSat
			str.ephset = str.raw.EphSet
			str.staid = 0
		}
	case str.format == STRFMT_RINEX:
		if t = str.rnx.InputRnxCtr(rd); t >= 1 {
			str.time = str.rnx.time
			str.ephsat = str.rnx.ephsat
			str.ephset = str.rnx.ephset
			str.staid = 0
		}
	}
	if str.tstart.Time == 0 && str.time.Time == 0 {
		str.tstart = str.time
	}
	Trace(5, "input_strfile: time=%s type=%d\n", TimeStr(str.time, 3), t)
	return t
}

/* open stream file ----------------------------------------------------------*/
func (str *StreamFile) OpenStreamFile(file string) (int, *bufio.Reader) {
	var err error
	Trace(3, "open_strfile: file=%s\n", file)
	if str.format == STRFMT_RTCM2 || str.format == STRFMT_RTCM3 {

		if str.fp, err = os.OpenFile(file, os.O_RDONLY, 0666); err != nil {
			ShowMsg_Ptr("rtcm open error: %s", file)
			return 0, nil
		}
		str.rtcm.Time = str.time
	} else if str.format <= MAXRCVFMT {
		if str.fp, err = os.OpenFile(file, os.O_RDONLY, 0666); err != nil {
			ShowMsg_Ptr("log open error: %s", file)
			return 0, nil
		}
		str.raw.Time = str.time
	} else if str.format == STRFMT_RINEX {
		if str.fp, err = os.OpenFile(file, os.O_RDONLY, 0666); err != nil {
			ShowMsg_Ptr("rinex open error: %s", file)
			return 0, nil
		}
		rd := bufio.NewReader(str.fp)
		/* open rinex control */
		if str.rnx.OpenRnxCtr(rd) == 0 {
			ShowMsg_Ptr("no rinex file: %s", file)
			str.fp.Close()
			return 0, nil
		}
		str.rnx.time = str.time
		return 1, rd
	}
	return 1, nil
}

/* close stream file ---------------------------------------------------------*/
func (str *StreamFile) CloseStreamFile() {
	Trace(4, "close_strfile:\n")

	if str.format == STRFMT_RTCM2 || str.format == STRFMT_RTCM3 ||
		str.format <= MAXRCVFMT || str.format == STRFMT_RINEX {
		if str.fp != nil {
			str.fp.Close()
		}
	}
}

/* set format and files in RINEX options comments ----------------------------*/
func SetOptFile(format int, paths []string, n int, mask []int, opt *RnxOpt) {
	var i, j int
	for i = 0; i < MAXCOMMENT; i++ {
		if len(opt.Comment[i]) == 0 {
			break
		}
	}
	if i < MAXCOMMENT {
		opt.Comment[i] = fmt.Sprintf("format: %.55s", FormatStrs[format])
		i++
	}
	for j = 0; j < n && i < MAXCOMMENT; j++ {
		if mask[j] == 0 {
			continue
		}
		opt.Comment[i] = fmt.Sprintf("log: %.58s", paths[j])
		i++
	}
}

/* unset RINEX options comments ----------------------------------------------*/
func UnsetOptFile(opt *RnxOpt) {
	var brk int = 0

	for i := MAXCOMMENT - 1; i >= 0 && brk == 0; i-- {
		if len(opt.Comment[i]) == 0 {
			continue
		}
		if strings.Compare(opt.Comment[i][:8], "format: ") == 0 {
			brk = 1
		}
		opt.Comment[i] = ""
	}
}

/* sort obs-types ------------------------------------------------------------*/
func sort_obstype(codes, types []uint8, n, sys int) {
	var i, j, idx1, idx2, pri1, pri2 int

	for i = 0; i < n-1; i++ {
		for j = i + 1; j < n; j++ {
			idx1 = Code2Idx(navsys[sys], codes[i])
			idx2 = Code2Idx(navsys[sys], codes[j])
			pri1 = GetCodePri(navsys[sys], codes[i], "")
			pri2 = GetCodePri(navsys[sys], codes[j], "")
			if idx1 < idx2 || (idx1 == idx2 && pri1 >= pri2) {
				continue
			}
			codes[i], codes[j] = codes[j], codes[i]
			types[i], types[j] = types[j], types[i]
		}
	}
}

/* set obs-types in RINEX options --------------------------------------------*/
func SetOptObsType(codes []uint8, types []uint8, sys int, opt *RnxOpt) {
	var (
		type_str     string = "CLDS"
		t, id        string
		ver          rune
		i, j, k, idx int
	)

	Trace(4, "setopt_obstype: sys=%d\n", sys)

	opt.NObs[sys] = 0

	if navsys[sys]&opt.NavSys == 0 {
		return
	}

	for i = 0; codes[i] > 0; i++ {
		id = Code2Obs(codes[i])
		idx = Code2Idx(navsys[sys], codes[i])
		if len(id) == 0 || idx < 0 {
			continue
		}
		if opt.FreqType&(1<<idx) == 0 || opt.Mask[sys][codes[i]-1] == '0' {
			continue
		}
		if opt.RnxVer >= 300 {
			ver = rune(vercode[sys][codes[i]-1])
			if ver < '0' || ver > rune('0')+rune(opt.RnxVer-300) {
				Trace(2, "unsupported obs type: rnxver=%.2f sys=%d code=%s\n", float64(opt.RnxVer)/100.0, sys, Code2Obs(codes[i]))
				continue
			}
		}
		for j = 0; j < 4; j++ {
			if opt.ObsType&(1<<j) == 0 {
				continue
			}
			if types != nil && types[i]&(1<<j) == 0 {
				continue
			}

			/* obs-types in ver.3 */
			t = fmt.Sprintf("%c%s", type_str[j], id)
			if t[0] == 'C' && t[2] == 'N' {
				continue
			} /* codeless */

			if opt.RnxVer <= 299 { /* ver.2 */

				/* ver.3 . ver.2 */
				ConvRinexCode3_2(opt.RnxVer, navsys[sys], &t)

				/* check duplicated obs-type */
				for k = 0; k < opt.NObs[0]; k++ {
					if strings.Compare(opt.TObs[0][k], t) == 0 {
						break
					}
				}
				if k >= opt.NObs[0] && opt.NObs[0] < MAXOBSTYPE {
					opt.TObs[0][opt.NObs[0]] = t
					opt.NObs[0]++
				}
			} else if opt.NObs[sys] < MAXOBSTYPE { /* ver.3 */
				opt.TObs[sys][opt.NObs[sys]] = t
				opt.NObs[sys]++
			}
		}
	}
}

/* set phase shift in RINEX options (RINEX 3.04 A23) -------------------------*/
func SetOptPhShift(opt *RnxOpt) {
	var (
		code uint8
		i, j int
	)

	for i = 0; i < NSATSYS; i++ {
		for j = 0; j < opt.NObs[i]; j++ {
			if opt.TObs[i][j][0] != 'L' {
				continue
			}
			code = Obs2Code(opt.TObs[i][j][1:])

			switch navsys[i] {
			case SYS_GPS:
				if code == CODE_L1S || code == CODE_L1L || code == CODE_L1X || code == CODE_L1P ||
					code == CODE_L1W || code == CODE_L1N {
					opt.Shift[i][j] = 0.25 /* +1/4 cyc */
				} else if code == CODE_L2C || code == CODE_L2S || code == CODE_L2L ||
					code == CODE_L2X || code == CODE_L5Q {
					opt.Shift[i][j] = -0.25 /* -1/4 cyc */
				}
			case SYS_GLO:
				if code == CODE_L1P || code == CODE_L2P || code == CODE_L3Q {
					opt.Shift[i][j] = 0.25 /* +1/4 cyc */
				}
			case SYS_GAL:
				if code == CODE_L1C {
					opt.Shift[i][j] = 0.5 /* +1/2 cyc */
				} else if code == CODE_L5Q || code == CODE_L7Q || code == CODE_L8Q {
					opt.Shift[i][j] = -0.25 /* -1/4 cyc */
				} else if code == CODE_L6C {
					opt.Shift[i][j] = -0.5 /* -1/2 cyc */
				}
			case SYS_QZS:
				if code == CODE_L1S || code == CODE_L1L || code == CODE_L1X {
					opt.Shift[i][j] = 0.25 /* +1/4 cyc */
				} else if code == CODE_L5Q || code == CODE_L5P {
					opt.Shift[i][j] = -0.25 /* -1/4 cyc */
				}
			case SYS_CMP:
				if code == CODE_L2P || code == CODE_L7Q || code == CODE_L6Q {
					opt.Shift[i][j] = -0.25 /* -1/4 cyc */
				} else if code == CODE_L1P || code == CODE_L5P || code == CODE_L7P {
					opt.Shift[i][j] = 0.25 /* +1/4 cyc */
				}
			}
		}
	}
}

/* set station ID list to RINEX options comments -----------------------------*/
func SetOptStaList(str *StreamFile, opt *RnxOpt) {
	var (
		p      *Stas
		s1, s2 string
		i, n   int = 0, 0
	)

	for p = str.stas; p != nil; p = p.next {
		n++
	}

	if n <= 1 {
		return
	}

	for i = 0; i < MAXCOMMENT; i++ {
		if len(opt.Comment[i]) == 0 {
			break
		}
	}
	opt.Comment[i] = fmt.Sprintf("%5s  %22s  %22s", "STAID", "TIME OF FIRST OBS",
		"TIME OF LAST OBS")
	i++
	n--
	for p = str.stas; p != nil && n >= 0; p = p.next {

		if i+n >= MAXCOMMENT {
			continue
		}

		Time2Str(p.ts, &s1, 2)
		Time2Str(p.te, &s2, 2)
		opt.Comment[i+n] = fmt.Sprintf(" %04d  %s  %s", p.staid, s1, s2)
		n--
	}
}

/* set station info in RINEX options -----------------------------------------*/
func SetOptSta(str *StreamFile, opt *RnxOpt) {
	var (
		p        *Stas
		sta      *Sta
		pos, enu [3]float64
	)

	Trace(4, "setopt_sta:\n")

	/* search first station in station list */
	for p = str.stas; p != nil; p = p.next {
		if p.next == nil {
			break
		}
		if TimeDiff(p.next.te, opt.TS) < 0.0 {
			break
		}
	}
	if p != nil {
		sta = &p.sta
		SetOptStaList(str, opt)
	} else {
		sta = str.sta
	}
	/* marker name and number */
	if len(opt.Marker) == 0 && len(opt.MarkerNo) == 0 {
		opt.Marker = sta.Name
		opt.MarkerNo = sta.Marker
	}
	/* receiver and antenna info */
	if len(opt.Rec[0]) == 0 && len(opt.Rec[1]) == 0 && len(opt.Rec[2]) == 0 {
		opt.Rec[0] = sta.RecSN
		opt.Rec[1] = sta.Type
		opt.Rec[2] = sta.RecVer
	}
	opt.Ant[0] = sta.AntSno[:]
	opt.Ant[1] = sta.AntDes[:]
	if sta.AntSetup > 0 {
		opt.Ant[2] = fmt.Sprintf("%d", sta.AntSetup)
	} else {
		opt.Ant[2] = ""
	}

	/* antenna approx position */
	if opt.AutoPos == 0 && Norm(sta.Pos[:], 3) > 0.0 {
		MatCpy(opt.AppPos[:], sta.Pos[:], 3, 1)
	}
	/* antenna delta */
	if Norm(opt.AntDel[:], 3) > 0.0 {

	} else if Norm(sta.Del[:], 3) > 0.0 {
		if sta.DelType == 0 && Norm(sta.Del[:], 3) > 0.0 { /* enu */
			opt.AntDel[0] = sta.Del[2] /* h */
			opt.AntDel[1] = sta.Del[0] /* e */
			opt.AntDel[2] = sta.Del[1] /* n */
		} else if Norm(sta.Pos[:], 3) > 0.0 { /* xyz */
			Ecef2Pos(sta.Pos[:], pos[:])
			Ecef2Enu(pos[:], sta.Del[:], enu[:])
			opt.AntDel[0] = enu[2] /* h */
			opt.AntDel[1] = enu[0] /* e */
			opt.AntDel[2] = enu[1] /* n */
		}
	} else {
		opt.AntDel[0] = sta.Hgt
		opt.AntDel[1] = 0.0
		opt.AntDel[2] = 0.0
	}
}

/* update station list -------------------------------------------------------*/
func (str *StreamFile) UpdateStas() {
	if str.stas == nil || str.stas.staid != str.staid { /* station ID changed */
		var p = new(Stas)
		p.staid = str.staid
		p.ts, p.te = str.time, str.time
		p.next = nil
		str.stas = p
	} else {
		str.stas.te = str.time

	}
}

/* update station info in station list ---------------------------------------*/
func (str *StreamFile) UpdateStaInf() {
	if str.stas != nil && str.stas.staid == str.staid {
		str.stas.sta = *str.sta
	}
}

/* dump station list ---------------------------------------------------------*/
func dump_stas(str *StreamFile) {
	/* for debug */
	var (
		p      *Stas
		pos    [3]float64
		s1, s2 string
	)

	Trace(2, "# STATION LIST\n")
	Trace(2, "# %17s %19s %5s %6s %16s %16s %12s %13s %9s %2s %6s %6s\n",
		"TIME", "STAID", "MARKER", "ANTENNA", "RECEIVER", "LATITUDE", "LONGITUDE",
		"HIGHT", "DT", "DEL1", "DEL2", "DEL3")

	for p = str.stas; p != nil; p = p.next {
		Time2Str(p.ts, &s1, 0)
		Time2Str(p.te, &s2, 0)
		Ecef2Pos(p.sta.Pos[:], pos[:])
		Trace(2, "%s %s  %04d %-6.6s %-16.16s %-16.16s %12.8f %13.8f %9.3f %2d %6.3f %6.3f %6.3f\n", s1, s2, p.staid, p.sta.Name, p.sta.AntDes,
			p.sta.Type, pos[0]*R2D, pos[1]*R2D, pos[2], p.sta.DelType,
			p.sta.Del[0], p.sta.Del[1], p.sta.Del[2])
	}

}

/* add half-cycle ambiguity list ---------------------------------------------*/
func (str *StreamFile) AddHalfc(sat, idx int, time Gtime) int {
	var p *Halfc = new(Halfc)
	p.ts, p.te = time, time
	p.stat = 0
	p.next = str.halfc[sat-1][idx]
	str.halfc[sat-1][idx] = p
	return 1
}

/* update half-cycle ambiguity -----------------------------------------------*/
func (str *StreamFile) UpdateHalfc(obs *ObsD) {
	var sat int = int(obs.Sat)

	for i := 0; i < NFREQ+NEXOBS; i++ {
		if obs.L[i] == 0.0 {
			continue
		}

		if str.halfc[sat-1][i] == nil {
			if str.AddHalfc(sat, i, obs.Time) == 0 {
				continue
			}
		}
		if obs.LLI[i]&LLI_SLIP > 0 {
			str.halfc[sat-1][i].stat = 0
		}
		if obs.LLI[i]&LLI_HALFC > 0 { /* halfcyc unknown */
			if str.halfc[sat-1][i].stat == 0 {
				str.halfc[sat-1][i].ts = obs.Time
			}
			str.halfc[sat-1][i].te = obs.Time
			str.halfc[sat-1][i].stat = 1 /* unresolved */
		} else if str.halfc[sat-1][i].stat == 1 { /* halfcyc unknown . known */
			if obs.LLI[i]&LLI_HALFA > 0 {
				str.halfc[sat-1][i].stat = 2 /* resolved with added */
			} else if obs.LLI[i]&LLI_HALFS > 0 {
				str.halfc[sat-1][i].stat = 3 /* resolved with subtracted */
			} else {
				str.halfc[sat-1][i].stat = 4 /* resolved with none */
			}
			if str.AddHalfc(sat, i, obs.Time) == 0 {
				continue
			}
		}
	}
}

/* dump half-cycle ambiguity list --------------------------------------------*/
func dump_halfc(str *StreamFile) {
	// #if 0 /* for debug */
	//     halfc_t *p;
	//     char s0[32],s1[32],s2[32],*stats[]={"ADD","SUB","NON"};
	//     int i,j;

	//     trace(2,"# HALF-CYCLE AMBIGUITY CORRECTIONS\n");
	//     trace(2,"# %20s %22s %4s %3s %3s\n","START","END","SAT","FRQ","COR");

	//     for (i=0;i<MAXSAT;i++) for (j=0;j<NFREQ+NEXOBS;j++) {
	//         for (p=str.halfc[i][j];p;p=p.next) {
	//             if (p.stat<=1) continue;
	//             satno2id(i+1,s0);
	//             time2str(p.ts,s1,2);
	//             time2str(p.te,s2,2);
	//             trace(2,"%s %s %4s %3d %3s\n",s1,s2,s0,j+1,stats[p.stat-2]);
	//         }
	//     }
	// #endif
}

/* resolve half-cycle ambiguity ----------------------------------------------*/
func (str *StreamFile) ResolveHalfc(data []ObsD, n int) {
	var (
		p         *Halfc
		i, j, sat int
	)

	for i = 0; i < n; i++ {
		for j = 0; j < NFREQ+NEXOBS; j++ {
			sat = int(data[i].Sat)

			for p = str.halfc[sat-1][j]; p != nil; p = p.next {
				if p.stat <= 1 {
					continue
				}
				if TimeDiff(data[i].Time, p.ts) < float64(-DTTOL) ||
					TimeDiff(data[i].Time, p.te) > float64(DTTOL) {
					continue
				}

				if p.stat == 3 {
					data[i].L[j] += 0.5
				} else if p.stat == 4 {
					data[i].L[j] -= 0.5
				}
				data[i].LLI[j] &= ^uint8(LLI_HALFC)
			}
			data[i].LLI[j] &= ^uint8(LLI_HALFA | LLI_HALFS)
		}
	}
}

/* scan input files ----------------------------------------------------------*/
func ScanRenixFile(files []string, nf int, opt *RnxOpt, str *StreamFile, mask []int) int {
	var (
		eph0                                 Eph  = Eph{Sat: 0, Iode: -1, Iodc: -1}
		geph0                                GEph = GEph{Sat: 0, Iode: -1}
		seph0                                SEph = SEph{Sat: 0}
		codes                                [NSATSYS][33]uint8
		types                                [NSATSYS][33]uint8
		msg                                  string
		i, j, k, l, m, c, t, sys, prn, abort int
		n                                    [NSATSYS]int
	)

	Trace(4, "scan_file: nf=%d\n", nf)

	for m = 0; m < nf && abort == 0; m++ {
		var rd *bufio.Reader
		var ret int
		if ret, rd = str.OpenStreamFile(files[m]); ret == 0 {
			continue
		}
		for {
			if t = str.InputStreamFile(rd); t < -1 {
				break
			}
			if opt.TS.Time > 0 && TimeDiff(str.time, opt.TS) < -opt.TTol {
				continue
			}
			if opt.TE.Time > 0 && TimeDiff(str.time, opt.TE) > -opt.TTol {
				break
			}
			mask[m] = 1 /* update file mask */

			if t == 1 { /* observation data */
				for i = 0; i < str.obs.N(); i++ {
					sys = SatSys(int(str.obs.Data[i].Sat), nil)
					if sys&opt.NavSys == 0 {
						continue
					}
					for l = 0; navsys[l] > 0; l++ {
						if navsys[l] == sys {
							break
						}
					}
					if navsys[l] == 0 {
						continue
					}

					/* update obs-types */
					for j = 0; j < NFREQ+NEXOBS; j++ {
						if str.obs.Data[i].Code[j] == 0 {
							continue
						}

						for k = 0; k < n[l]; k++ {
							if codes[l][k] == str.obs.Data[i].Code[j] {
								break
							}
						}
						if k >= n[l] && n[l] < 32 {
							codes[l][n[l]] = str.obs.Data[i].Code[j]
							n[l]++
						}
						if k < n[l] {
							if str.obs.Data[i].P[j] != 0.0 {
								types[l][k] |= 1
							}
							if str.obs.Data[i].L[j] != 0.0 {
								types[l][k] |= 2
							}
							if str.obs.Data[i].D[j] != 0.0 {
								types[l][k] |= 4
							}
							if str.obs.Data[i].SNR[j] != 0 {
								types[l][k] |= 8
							}
						}
					}
					/* update half-cycle ambiguity list */
					if opt.Halfcyc > 0 {
						str.UpdateHalfc(&str.obs.Data[i])
					}
				}
				/* update station list */
				str.UpdateStas()
			} else if t == 5 { /* station info */
				/* update station info */
				str.UpdateStaInf()
			}
			c++
			if c%11 > 0 {
				continue
			}
			var n0, n1, n2, n3, n4, n5, n6 string
			if n[0] > 0 {
				n0 = "G"
			}
			if n[1] > 0 {
				n1 = "R"
			}
			if n[2] > 0 {
				n2 = "E"
			}
			if n[3] > 0 {
				n3 = "J"
			}
			if n[4] > 0 {
				n4 = "S"
			}
			if n[5] > 0 {
				n5 = "C"
			}
			if n[6] > 0 {
				n6 = "I"
			}
			msg = fmt.Sprintf("scanning: %s %s%s%s%s%s%s%s", TimeStr(str.time, 0), n0, n1, n2, n3, n4, n5, n6)
			if abort = ShowMsg_Ptr(msg); abort > 0 {
				break
			}
		}
		str.CloseStreamFile()
	}
	ShowMsg_Ptr("")

	if abort > 0 {
		Trace(2, "aborted in scan\n")
		return 0
	}
	for i = 0; i < NSATSYS; i++ {
		for j = 0; j < n[i]; j++ {
			Trace(2, "scan_file: sys=%d code=%s type=%d\n", i, Code2Obs(codes[i][j]), types[i][j])
		}
	}
	/* sort and set obs-types in RINEX options */
	for i = 0; i < NSATSYS; i++ {
		sort_obstype(codes[i][:], types[i][:], n[i], i)
		SetOptObsType(codes[i][:], types[i][:], i, opt)

		for j = 0; j < n[i]; j++ {
			Trace(2, "scan_file: sys=%d code=%s\n", i, Code2Obs(codes[i][j]))
		}
	}
	/* set station info in RINEX options */
	SetOptSta(str, opt)

	/* set phase shifts in RINEX options */
	if opt.PhShift > 0 {
		SetOptPhShift(opt)
	}
	/* set GLONASS FCN and clear ephemeris */
	for i = 0; i < str.nav.N(); i++ {
		str.nav.Ephs[i] = eph0
	}
	for i = 0; i < str.nav.Ng(); i++ {
		if SatSys(str.nav.Geph[i].Sat, &prn) != SYS_GLO {
			continue
		}
		str.nav.Glo_fcn[prn-1] = str.nav.Geph[i].Frq + 8
		str.nav.Geph[i] = geph0
	}
	for i = 0; i < str.nav.Ns(); i++ {
		str.nav.Seph[i] = seph0
	}
	dump_stas(str)
	dump_halfc(str)
	return 1
}

/* write RINEX header --------------------------------------------------------*/
func WriteRinexHeader(ofp []*os.File, idx int, opt *RnxOpt, nav *Nav) {
	switch idx {
	case 0:
		OutRnxObsHeader(ofp[0], opt, nav)
	case 1:
		OutRnxNavHeader(ofp[1], opt, nav)
	case 2:
		OutRnxGnavHeader(ofp[2], opt, nav)
	case 3:
		OutRnxHnavHeader(ofp[3], opt, nav)
	case 4:
		OutRnxQnavHeader(ofp[4], opt, nav)
	case 5:
		OutRnxLnavHeader(ofp[5], opt, nav)
	case 6:
		OutRnxCnavHeader(ofp[6], opt, nav)
	case 7:
		OutRnxInavHeader(ofp[7], opt, nav)
	}
}

/* open output files ---------------------------------------------------------*/
func OpenRenixFile(ofp []*os.File, files []string, file string, opt *RnxOpt, nav *Nav) int {
	var (
		path string
		i    int
		err  error
	)

	Trace(4, "openfile:\n")

	for i = 0; i < len(files); i++ {

		if len(files[i]) == 0 {
			continue
		}

		path = files[i]

		/* check overwrite input file and modify output file */
		if strings.Compare(path, file) == 0 {
			path += "_"
		}

		/* create directory if not exist */
		CreateDir(path)
		ofp[i], err = os.OpenFile(path, os.O_CREATE|os.O_WRONLY|os.O_TRUNC, os.ModeAppend|os.ModePerm)
		if err != nil {
			ShowMsg_Ptr("file open error: %s", path)
			for i--; i >= 0; i-- {
				if ofp[i] != nil {
					ofp[i].Close()
				}
			}
			return 0
		}

		/* write RINEX header */
		WriteRinexHeader(ofp, i, opt, nav)
	}
	return 1
}

/* close output files --------------------------------------------------------*/
func CloseRenixFile(ofp []*os.File, opt *RnxOpt, nav *Nav) {
	Trace(2, "closefile:\n")

	for i := 0; i < len(ofp); i++ {

		/* rewrite RINEX header */
		ofp[i].Seek(0, 0)
		WriteRinexHeader(ofp, i, opt, nav)

		ofp[i].Close()
	}
}

/* output RINEX event --------------------------------------------------------*/
func OutRnxEvent(fp *os.File, opt *RnxOpt, time Gtime, event int, stas *Stas, staid int) {
	var (
		p, q          *Stas = nil, nil
		ep            [6]float64
		pos, enu, del [3]float64
	)
	Trace(4, "outrnxevent: event=%d\n", event)
	ver := 28
	sver := ""
	if opt.RnxVer >= 300 {
		ver = 31
		sver = ">"
	}
	if event == EVENT_STARTMOVE {

		fp.WriteString(fmt.Sprintf("%*s%d%3d\n", ver, "", event, 2))
		fp.WriteString(fmt.Sprintf("%-60s%-20s\n", "EVENT: START MOVING ANTENNA", "COMMENT"))
		fp.WriteString(fmt.Sprintf("%-60s%-20s\n", opt.Marker, "MARKER NAME"))
	} else if event == EVENT_NEWSITE {
		for q = stas; q != nil; q = q.next {
			if q.staid == staid && TimeDiff(time, q.te) <= 0.0 {
				p = q
			}
		}
		fp.WriteString(fmt.Sprintf("%*s%d%3d\n", ver, "", event, 6))
		fp.WriteString(fmt.Sprintf("%-60s%-20s\n", "EVENT: NEW SITE OCCUPATION", "COMMENT"))
		if q == nil {
			fp.WriteString(fmt.Sprintf("%04d%56s%-20s\n", staid, "", "MARKER NAME"))
			return
		}
		fp.WriteString(fmt.Sprintf("%-60s%-20s\n", p.sta.Name, "MARKER NAME"))
		fp.WriteString(fmt.Sprintf("%-20.20s%-20.20s%-20.20s%-20s\n", p.sta.RecSN,
			p.sta.Type, p.sta.RecVer, "REC # / TYPE / VERS"))
		fp.WriteString(fmt.Sprintf("%-20.20s%-20.20s%-20.20s%-20s\n", p.sta.AntSno,
			p.sta.AntDes, "", "ANT # / TYPE"))
		fp.WriteString(fmt.Sprintf("%14.4f%14.4f%14.4f%-18s%-20s\n", p.sta.Pos[0],
			p.sta.Pos[1], p.sta.Pos[2], "", "APPROX POSITION XYZ"))

		/* antenna delta */
		if Norm(p.sta.Del[:], 3) > 0.0 {
			if p.sta.DelType == 0 && Norm(p.sta.Del[:], 3) > 0.0 { /* enu */
				del[0] = p.sta.Del[2] /* h */
				del[1] = p.sta.Del[0] /* e */
				del[2] = p.sta.Del[1] /* n */
			} else if Norm(p.sta.Pos[:], 3) > 0.0 { /* xyz */
				Ecef2Pos(p.sta.Pos[:], pos[:])
				Ecef2Enu(pos[:], p.sta.Del[:], enu[:])
				del[0] = enu[2] /* h */
				del[1] = enu[0] /* e */
				del[2] = enu[1] /* n */
			}
		} else {
			del[0] = p.sta.Hgt
			del[1], del[2] = 0.0, 0.0
		}
		fp.WriteString(fmt.Sprintf("%14.4f%14.4f%14.4f%-18s%-20s\n", del[0], del[1], del[2], "",
			"ANTENNA: DELTA H/E/N"))
	} else if event == EVENT_EXTERNAL {
		Time2Epoch(time, ep[:])
		ver = int(math.Mod(ep[0], 100))
		if opt.RnxVer >= 300 {
			ver = int(ep[0])
		}
		fp.WriteString(fmt.Sprintf("%s %02d %02.0f %02.0f %02.0f %02.0f %010.7f  %d%3d\n",
			sver,
			ver,
			ep[1], ep[2], ep[3], ep[4], ep[5], event, 1))
		fp.WriteString(fmt.Sprintf("%-60s%-20s\n", "EXTERNAL EVENT", "COMMENT"))
	}
}

/* save cycle slips ----------------------------------------------------------*/
func save_slips(str *StreamFile, data []ObsD, n int) {
	for i := 0; i < n; i++ {
		for j := 0; j < NFREQ+NEXOBS; j++ {
			if data[i].LLI[j]&LLI_SLIP > 0 {
				str.slips[data[i].Sat-1][j] = 1
			}
		}
	}
}

/* restore cycle slips -------------------------------------------------------*/
func rest_slips(str *StreamFile, data []ObsD, n int) {

	for i := 0; i < n; i++ {
		for j := 0; j < NFREQ+NEXOBS; j++ {
			if data[i].L[j] != 0.0 && str.slips[data[i].Sat-1][j] != 0 {
				data[i].LLI[j] |= LLI_SLIP
				str.slips[data[i].Sat-1][j] = 0
			}
		}
	}
}

/* screen time with time tolerance -------------------------------------------*/
func screent_ttol(time, ts, te Gtime, tint, ttol float64) int {
	if ttol <= 0.0 {
		ttol = float64(DTTOL)
	}

	if tint <= 0.0 || math.Mod(Time2GpsT(time, nil)+ttol, tint) <= ttol*2.0 {
		if ts.Time == 0 || TimeDiff(time, ts) >= -ttol {
			if te.Time == 0 || TimeDiff(time, te) < ttol {
				return 1
			}
		}
	}
	return 0
}

/* convert observation data --------------------------------------------------*/
func ConvObs(ofp []*os.File, opt *RnxOpt, str *StreamFile, n []int, tend *Gtime, staid *int) {
	var time Gtime
	var i, j int

	Trace(4, "convobs :\n")

	if ofp[0] == nil || str.obs.N() <= 0 {
		return
	}

	time = str.obs.Data[0].Time

	/* avoid duplicated data by multiple files handover */
	if tend.Time > 0 && TimeDiff(time, *tend) < opt.TTol {
		return
	}
	*tend = time

	/* save cycle slips */
	save_slips(str, str.obs.Data, str.obs.N())

	if screent_ttol(time, opt.TS, opt.TE, opt.TInt, opt.TTol) == 0 {
		return
	}

	/* restore cycle slips */
	rest_slips(str, str.obs.Data[:], str.obs.N())

	if str.staid != *staid { /* station ID changed */

		if *staid >= 0 { /* output RINEX event */
			OutRnxEvent(ofp[0], opt, str.time, EVENT_NEWSITE, str.stas, str.staid)
		}
		*staid = str.staid

		/* set cycle slips */
		for i = 0; i < str.obs.N(); i++ {
			for j = 0; j < NFREQ+NEXOBS; j++ {
				if str.obs.Data[i].L[j] != 0.0 {
					str.obs.Data[i].LLI[j] |= LLI_SLIP
				}
			}
		}
	}
	/* resolve half-cycle ambiguity */
	if opt.Halfcyc > 0 {
		str.ResolveHalfc(str.obs.Data, str.obs.N())
	}
	/* output RINEX observation data */
	OutRnxObsBody(ofp[0], opt, str.obs.Data, str.obs.N(), 0)

	if opt.TStart.Time == 0 {
		opt.TStart = time
	}
	opt.TEnd = time

	n[0]++
}

/* convert navigattion data --------------------------------------------------*/
func ConvNav(ofp []*os.File, opt *RnxOpt, str *StreamFile, n []int) {
	var (
		ts                          Gtime
		dtoe                        float64
		sat, set, sys, prn, sep_nav int
	)

	if opt.RnxVer <= 299 || opt.Sep_Nav > 0 {
		sep_nav = 1
	}

	Trace(4, "convnav :\n")

	sat = str.ephsat
	set = str.ephset
	sys = SatSys(sat, &prn)
	if (sys&opt.NavSys) == 0 || opt.ExSats[sat-1] > 0 {
		return
	}

	switch sys {
	case SYS_GLO:
		dtoe = float64(MAXDTOE_GLO)
	case SYS_GAL:
		dtoe = float64(MAXDTOE_GAL)
	case SYS_QZS:
		dtoe = float64(MAXDTOE_QZS)
	case SYS_CMP:
		dtoe = float64(MAXDTOE_CMP)
	case SYS_IRN:
		dtoe = float64(MAXDTOE_IRN)
	case SYS_SBS:
		dtoe = float64(MAXDTOE_SBS)
	default:
		dtoe = float64(MAXDTOE)
	}
	ts = opt.TS
	if ts.Time != 0 {
		ts = TimeAdd(ts, -dtoe)
	}
	if ScreenTime(str.time, ts, opt.TE, 0.0) == 0 {
		return
	}

	switch sys {
	case SYS_GPS:
		if ofp[1] != nil {
			OutRnxNavBody(ofp[1], opt, &str.nav.Ephs[sat-1+MAXSAT*set])
			n[1]++
		}
	case SYS_GLO:
		if ofp[1] != nil && sep_nav == 0 {
			OutRnxGnavBody(ofp[1], opt, &str.nav.Geph[prn-1])
			n[1]++
		} else if ofp[2] != nil && sep_nav > 0 {
			OutRnxGnavBody(ofp[2], opt, &str.nav.Geph[prn-1])
			n[2]++
		}
	case SYS_SBS:
		if ofp[1] != nil && sep_nav == 0 {
			OutRnxHnavBody(ofp[1], opt, &str.nav.Seph[prn-MINPRNSBS])
			n[1]++
		} else if ofp[3] != nil && sep_nav > 0 {
			OutRnxHnavBody(ofp[3], opt, &str.nav.Seph[prn-MINPRNSBS])
			n[3]++
		}
	case SYS_QZS:
		if ofp[1] != nil && sep_nav == 0 {
			OutRnxNavBody(ofp[1], opt, &str.nav.Ephs[sat-1+MAXSAT*set])
			n[1]++
		} else if ofp[4] != nil && sep_nav > 0 {
			OutRnxNavBody(ofp[4], opt, &str.nav.Ephs[sat-1+MAXSAT*set])
			n[4]++
		}
	case SYS_GAL:
		if ofp[1] != nil && sep_nav == 0 {
			OutRnxNavBody(ofp[1], opt, &str.nav.Ephs[sat-1+MAXSAT*set])
			n[1]++
		} else if ofp[5] != nil && sep_nav > 0 {
			OutRnxNavBody(ofp[5], opt, &str.nav.Ephs[sat-1+MAXSAT*set])
			n[5]++
		}
	case SYS_CMP:
		if ofp[1] != nil && sep_nav == 0 {
			OutRnxNavBody(ofp[1], opt, &str.nav.Ephs[sat-1+MAXSAT*set])
			n[1]++
		} else if ofp[6] != nil && sep_nav > 0 {
			OutRnxNavBody(ofp[6], opt, &str.nav.Ephs[sat-1+MAXSAT*set])
			n[6]++
		}
	case SYS_IRN:
		if ofp[1] != nil && sep_nav == 0 {
			OutRnxNavBody(ofp[1], opt, &str.nav.Ephs[sat-1+MAXSAT*set])
			n[1]++
		} else if ofp[7] != nil && sep_nav > 0 {
			OutRnxNavBody(ofp[7], opt, &str.nav.Ephs[sat-1+MAXSAT*set])
			n[7]++
		}
	}
}

/* convert SBAS message ------------------------------------------------------*/
func ConvSbs(ofp []*os.File, opt *RnxOpt, str *StreamFile, n []int, tend *Gtime) {
	var (
		time                   Gtime
		prn, sat, sys, sep_nav int
	)

	if opt.RnxVer <= 299 || opt.Sep_Nav > 0 {
		sep_nav = 1
	}

	Trace(4, "convsbs :\n")

	time = GpsT2Time(str.raw.Sbsmsg.Week, float64(str.raw.Sbsmsg.Tow))

	if ScreenTime(time, opt.TS, opt.TE, 0.0) == 0 {
		return
	}

	/* avoid duplicated data by multiple files handover */
	tt := TimeDiff(time, *tend)
	if tend.Time > 0 && tt < opt.TTol && tt > 0 {
		return
	}
	*tend = time

	prn = int(str.raw.Sbsmsg.Prn)
	if MINPRNSBS <= prn && prn <= MAXPRNSBS {
		sys = SYS_SBS
	} else if MINPRNQZS_S <= prn && prn <= MAXPRNQZS_S {
		sys = SYS_QZS
		prn += 10
	} else {
		Trace(2, "sbas message satellite error: prn=%d\n", prn)
		return
	}
	sat = SatNo(sys, prn)
	if sat == 0 || opt.ExSats[sat-1] == 1 {
		return
	}

	/* output SBAS message log */
	if ofp[NOUTFILE-1] != nil {
		SbsOutMsg(ofp[NOUTFILE-1], &str.raw.Sbsmsg)
		n[NOUTFILE-1]++
	}
	/* output SBAS ephemeris */
	if (opt.NavSys&SYS_SBS) > 0 && SbsUpdateCorr(&str.raw.Sbsmsg, str.nav) == 9 {

		if ofp[1] != nil && sep_nav == 0 {
			OutRnxHnavBody(ofp[1], opt, &str.nav.Seph[prn-MINPRNSBS])
			n[1]++
		} else if ofp[3] != nil && sep_nav > 0 {
			OutRnxHnavBody(ofp[3], opt, &str.nav.Seph[prn-MINPRNSBS])
			n[3]++
		}
	}
}

/* set approx position in RINEX options --------------------------------------*/
func SetOptAppOos(str *StreamFile, opt *RnxOpt) {
	var (
		prcopt PrcOpt = DefaultProcOpt()
		sol    Sol
		msg    string
	)

	prcopt.NavSys = opt.NavSys

	/* point positioning with last obs data */
	if PntPos(str.obs.Data, str.obs.N(), str.nav, &prcopt, &sol, nil, nil, &msg) == 0 {
		Trace(2, "point position error (%s)\n", msg)
		return
	}
	MatCpy(opt.AppPos[:], sol.Rr[:], 3, 1)
}

/* show conversion status ----------------------------------------------------*/
func showstat(sess int, ts, te Gtime, n []int) int {
	var (
		t   string = "ONGHQLCISE"
		msg string = ""
		s   string
	)

	if sess > 0 {
		msg += fmt.Sprintf("(%d) ", sess)
	}
	if ts.Time != 0 {
		Time2Str(ts, &s, 0)
		msg += s
	}
	if te.Time != 0 && TimeDiff(te, ts) > 0.9 {
		Time2Str(te, &s, 0)
		msg += fmt.Sprintf("-%s", s[5:])
	}
	msg += ": "

	for i := 0; i < NOUTFILE+1; i++ {
		if n[i] == 0 {
			continue
		}
		c := ""
		if i < NOUTFILE {
			c = " "
		}

		msg += fmt.Sprintf("%c=%d%s", t[i], n[i], c)
	}
	return ShowMsg_Ptr("showstat:%s", msg)
}

/* RINEX converter for single-session ----------------------------------------*/
func convrnx_s(sess, format int, opt *RnxOpt, file string, ofile []string) int {
	var (
		ofp          [NOUTFILE]*os.File
		str          *StreamFile
		tend         [3]Gtime
		i, j, nf, t  int
		n            [NOUTFILE + 1]int
		mask         [MAXEXFILE]int
		staid, abort int = -1, 0
		path         string
		paths        [NOUTFILE]string
		epath        [MAXEXFILE]string
		staname      string
	)

	if len(opt.Staid) > 0 {
		staname = opt.Staid
	} else {
		staname = "0000"
	}

	Trace(4, "convrnx_s: sess=%d format=%d file=%s ofile=%s %s %s %s %s %s %s %s %s\n",
		sess, format, file, ofile[0], ofile[1], ofile[2], ofile[3],
		ofile[4], ofile[5], ofile[6], ofile[7], ofile[8])

	/* replace keywords in input file */
	if RepPath(file, &path, opt.TS, staname, "") < 0 {
		return ShowMsg_Ptr("no time for input file: %s", file)
	}
	/* expand wild-cards in input file */

	if nf = ExPath(path, epath[:], MAXEXFILE); nf <= 0 {
		ShowMsg_Ptr("no input file: %s", path)
		return 0
	}
	if str = GenStreamfile(format, opt.RcvOpt); str == nil {
		return 0
	}
	if format == STRFMT_RTCM2 || format == STRFMT_RTCM3 || format == STRFMT_RT17 {
		str.time = opt.TRtcm
	} else if opt.TS.Time > 0 {
		str.time = TimeAdd(opt.TS, -1.0)
	}
	/* set GLONASS FCN in RINEX options */
	for i = 0; i < MAXPRNGLO; i++ {
		str.nav.Glo_fcn[i] = opt.GloFcn[i] /* FCN+8 */
	}
	/* scan input files */
	if ScanRenixFile(epath[:], nf, opt, str, mask[:]) == 0 {
		return 0
	}
	/* set format and file in RINEX options comments */
	SetOptFile(format, epath[:], nf, mask[:], opt)

	/* replace keywords in output file */
	for i = 0; i < NOUTFILE; i++ {
		//paths[i]=s[i];
		time := str.tstart
		if opt.TS.Time > 0 {
			time = opt.TS
		}
		if RepPath(ofile[i], &paths[i], time,
			staname, "") < 0 {
			Trace(2, "no time for output path: %s", ofile[i])
			str.FreeStrfile()
			return 0
		}
	}
	/* open output files */
	if OpenRenixFile(ofp[:], paths[:], path, opt, str.nav) == 0 {

		str.FreeStrfile()
		return 0
	}
	str.time = str.tstart

	for i = 0; i < nf && abort == 0; i++ {
		if mask[i] == 0 {
			continue
		}

		var rd *bufio.Reader
		var ret int
		/* open stream file */
		if ret, rd = str.OpenStreamFile(epath[i]); ret == 0 {
			continue
		}

		/* input message */
		for j = 0; ; j++ {
			if t = str.InputStreamFile(rd); t < -1 {
				break
			}

			if abort = showstat(sess, str.time, str.time, n[:]); j%11 == 0 && abort > 0 {
				break
			}

			if opt.TS.Time > 0 && TimeDiff(str.time, opt.TS) < -opt.TTol {
				continue
			}
			if opt.TE.Time > 0 && TimeDiff(str.time, opt.TE) > -opt.TTol {
				break
			}

			/* convert message */
			switch t {
			case 1:
				ConvObs(ofp[:], opt, str, n[:], &tend[0], &staid)
			case 2:
				ConvNav(ofp[:], opt, str, n[:])
			case 3:
				ConvSbs(ofp[:], opt, str, n[:], &tend[1])
			case -1:
				n[NOUTFILE]++ /* error */
			}
			/* set approx position in rinex option */
			if t == 1 && opt.AutoPos == 0 && Norm(opt.AppPos[:], 3) <= 0.0 {
				SetOptAppOos(str, opt)
			}
		}
		/* close stream file */
		str.CloseStreamFile()
	}
	/* close output files */
	CloseRenixFile(ofp[:], opt, str.nav)

	/* remove empty output files */
	for i = 0; i < NOUTFILE; i++ {
		if ofp[i] != nil && n[i] <= 0 {
			os.Remove(ofile[i])
		}
	}
	showstat(sess, opt.TStart, opt.TEnd, n[:])

	/* unset RINEX options comments */
	UnsetOptFile(opt)

	str.FreeStrfile()
	if abort > 0 {
		return -1
	}
	return 1
}

/* RINEX converter -------------------------------------------------------------
* convert receiver log file to RINEX obs/nav, SBAS log files
* args   : int    format I      receiver raw format (STRFMT_???)
*          rnxopt_t *opt IO     RINEX options (see below)
*          char   *file  I      RTCM, receiver raw or RINEX file
*                               (wild-cards (*) are expanded)
*          char   **ofile IO    output files
*                               ofile[0] RINEX OBS file   ("": no output)
*                               ofile[1] RINEX NAV file   ("": no output)
*                               ofile[2] RINEX GNAV file  ("": no output)
*                               ofile[3] RINEX HNAV file  ("": no output)
*                               ofile[4] RINEX QNAV file  ("": no output)
*                               ofile[5] RINEX LNAV file  ("": no output)
*                               ofile[6] RINEX CNAV file  ("": no output)
*                               ofile[7] RINEX INAV file  ("": no output)
*                               ofile[8] SBAS log file    ("": no output)
* return : status (1:ok,0:error,-1:abort)
* notes  : the following members of opt are replaced by information in last
*          converted RINEX: opt.tstart, opt.tend, opt.obstype, opt.nobs
*          keywords in ofile[] are replaced by first observation date/time and
*          station ID (%r)
*          the order of wild-card expanded files must be in-order by time
*-----------------------------------------------------------------------------*/
func ConvRnx(format int, opt *RnxOpt, file string, ofile []string) int {

	var (
		t0            Gtime
		tu, ts        float64
		i, week, stat int
		opt_          RnxOpt = *opt
		sys_GRS              = SYS_GPS | SYS_GLO | SYS_SBS
	)
	stat = 1

	Trace(4, "convrnx: format=%d file=%s ofile=%s %s %s %s %s %s %s %s %s\n",
		format, file, ofile[0], ofile[1], ofile[2], ofile[3], ofile[4], ofile[5],
		ofile[6], ofile[7], ofile[8])

	ShowMsg_Ptr("")

	/* disable systems according to RINEX version */
	switch {
	case opt.RnxVer <= 210:
		opt_.NavSys &= sys_GRS
	case opt.RnxVer <= 211:
		opt_.NavSys &= sys_GRS | SYS_GAL
	case opt.RnxVer <= 212:
		opt_.NavSys &= sys_GRS | SYS_GAL | SYS_CMP
	case opt.RnxVer <= 300:
		opt_.NavSys &= sys_GRS | SYS_GAL
	case opt.RnxVer <= 301:
		opt_.NavSys &= sys_GRS | SYS_GAL | SYS_CMP
	case opt.RnxVer <= 302:
		opt_.NavSys &= sys_GRS | SYS_GAL | SYS_CMP | SYS_QZS
	}

	/* disable frequency according to RINEX version */
	if opt.RnxVer <= 210 {
		opt_.FreqType &= 0x3
	}

	if opt.TS.Time == 0 || opt.TE.Time == 0 || opt.TUnit <= 0.0 {

		/* single session */
		opt_.TStart = t0
		opt_.TEnd = t0
		stat = convrnx_s(0, format, &opt_, file, ofile)
	} else if (TimeDiff(opt.TS, opt.TE)) < 0.0 {

		/* multiple session */
		tu = 86400.0
		if opt.TUnit < 86400.0 {
			tu = opt.TUnit
		}

		ts = tu * math.Floor(Time2GpsT(opt.TS, &week)/tu)

		for i = 0; ; i++ { /* for each session */
			opt_.TS = GpsT2Time(week, ts+float64(i)*tu)
			opt_.TE = TimeAdd(opt.TS, tu)
			if opt.TRtcm.Time > 0 {
				opt_.TRtcm = TimeAdd(opt.TRtcm, TimeDiff(opt_.TS, opt.TS))
			}
			if TimeDiff(opt_.TS, opt.TE) > -opt.TTol {
				break
			}

			if TimeDiff(opt_.TS, opt.TS) < 0.0 {
				opt_.TS = opt.TS
			}
			if TimeDiff(opt_.TE, opt.TE) > 0.0 {
				opt_.TE = opt.TE
			}
			opt_.TStart, opt_.TEnd = t0, t0
			if stat = convrnx_s(i+1, format, &opt_, file, ofile); stat < 0 {
				break
			}
		}
	} else {
		ShowMsg_Ptr("no period")
		return 0
	}
	/* output start and end time */
	opt.TStart = opt_.TStart
	opt.TEnd = opt_.TEnd

	return stat
}
