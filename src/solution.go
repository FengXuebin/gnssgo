/*------------------------------------------------------------------------------
* solution.c : solution functions
*
*          Copyright (C) 2007-2020 by T.TAKASU, All rights reserved.
*
* references :
*     [1] National Marine Electronic Association and International Marine
*         Electronics Association, NMEA 0183 version 4.10, August 1, 2012
*     [2] NMEA 0183 Talker Identifier Mnemonics, March 3, 2019
*         (https://www.nmea.org/content/STANDARDS/NMEA_0183_Standard)
*
* version : $Revision:$ $Date:$
* history : 2007/11/03  1.0 new
*           2009/01/05  1.1  add function outsols(), outsolheads(),
*                            setsolformat(), outsolexs, outsolex
*           2009/04/02  1.2  add dummy fields in NMEA mesassage
*                            fix bug to format lat/lon as deg-min-sec
*           2009/04/14  1.3  add age and ratio field to solution
*           2009/11/25  1.4  add function readsolstat()
*           2010/02/14  1.5  fix bug on output of gpstime at week boundary
*           2010/07/05  1.6  added api:
*                                initsolbuf(),freesolbuf(),addsol(),getsol(),
*                                inputsol(),outprcopts(),outprcopt()
*                            modified api:
*                                readsol(),readsolt(),readsolstat(),
*                                readsolstatt(),outsolheads(),outsols(),
*                                outsolexs(),outsolhead(),outsol(),outsolex(),
*                                outnmea_rmc(),outnmea_gga(),outnmea_gsa(),
*                                outnmea_gsv()
*                            deleted api:
*                                setsolopt(),setsolformat()
*           2010/08/14  1.7  fix bug on initialize solution buffer (2.4.0_p2)
*                            suppress enu-solution if base pos not available
*                            (2.4.0_p3)
*           2010/08/16  1.8  suppress null record if solution is not available
*                            (2.4.0_p4)
*           2011/01/23  1.9  fix bug on reading nmea solution data
*                            add api freesolstatbuf()
*           2012/02/05  1.10 fix bug on output nmea gpgsv
*           2013/02/18  1.11 support nmea GLGSA,GAGSA,GLCSV,GACSV sentence
*           2013/09/01  1.12 fix bug on presentation of nmea time tag
*           2015/02/11  1.13 fix bug on checksum of $GLGSA and $GAGSA
*                            fix bug on satellite id of $GAGSA
*           2016/01/17  1.14 support reading NMEA GxZDA
*                            ignore NMEA talker ID
*           2016/07/30  1.15 suppress output if std is over opt.maxsolstd
*           2017/06/13  1.16 support output/input of velocity solution
*           2018/10/10  1.17 support reading solution status file
*           2020/11/30  1.18 add NMEA talker ID GQ and GI (NMEA 0183 4.11)
*                            add NMEA GQ/GB/GI-GSA/GSV sentences
*                            change talker ID GP to GN for NMEA RMC/GGA
*                            change newline to "\r\n" in SOLF_LLH,XYZ,ENU
*                            add reading age information in NMEA GGA
*                            use integer types in stdint.h
*                            suppress warnings
*		    2022/05/31 1.0  rewrite solution.c with golang by fxb
*-----------------------------------------------------------------------------*/

package gnssgo

import (
	"bufio"
	"fmt"
	"io"
	"math"
	"os"
	"sort"
	"strconv"
	"strings"
	"unicode"
)

const (
	NMEA_TID = "GN" /* NMEA talker ID for RMC and GGA sentences */
	MAXFIELD = 64   /* max number of fields in a record */
	MAXNMEA  = 256  /* max length of nmea sentence */
	KNOT2M   = 0.514444444 /* m/knot */)

var (
	nmea_sys []int = []int{ /* NMEA systems */
		SYS_GPS | SYS_SBS, SYS_GLO, SYS_GAL, SYS_CMP, SYS_QZS, SYS_IRN, 0}
	nmea_tid []string = []string{ /* NMEA talker IDs [2] */
		"GP", "GL", "GA", "GB", "GQ", "GI", ""}
	nmea_sid []int = []int{ /* NMEA system IDs [1] table 21 */
		1, 2, 3, 4, 5, 6, 0}
	nmea_solq []int = []int{ /* NMEA GPS quality indicator [1] */
		/* 0=Fix not available or invalidi */
		/* 1=GPS SPS Mode, fix valid */
		/* 2=Differential GPS, SPS Mode, fix valid */
		/* 3=GPS PPS Mode, fix valid */
		/* 4=Real Time Kinematic. System used in RTK mode with fixed integers */
		/* 5=Float RTK. Satellite system used in RTK mode, floating integers */
		/* 6=Estimated (dead reckoning) Mode */
		/* 7=Manual Input Mode */
		/* 8=Simulation Mode */
		SOLQ_NONE, SOLQ_SINGLE, SOLQ_DGPS, SOLQ_PPP, SOLQ_FIX,
		SOLQ_FLOAT, SOLQ_DR, SOLQ_NONE, SOLQ_NONE, SOLQ_NONE}
)

/* solution option to field separator ----------------------------------------*/
func opt2sep(opt *SolOpt) string {
	if strings.Compare(opt.Sep, "\\t") == 0 {
		return "\t"
	}
	return opt.Sep
}

/* separate fields -----------------------------------------------------------*/
func tonum(buff, sep string, v []float64) int {
	var i int
	p := strings.Fields(buff)
	for i = range p {
		if i > len(v)-1 {
			break
		}
		v[i], _ = strconv.ParseFloat(p[i], 64)
	}

	return i + 1
}

/* sqrt of covariance --------------------------------------------------------*/
func sqvar(covar float64) float64 {
	if covar < 0.0 {
		return -math.Sqrt(-covar)
	}
	return math.Sqrt(covar)
}

/* convert ddmm.mm in nmea format to deg -------------------------------------*/
func dmm2deg(dmm float64) float64 {
	return math.Floor(dmm/100.0) + math.Mod(dmm, 100.0)/60.0
}

/* convert time in nmea format to time ---------------------------------------*/
func septime(t float64, t1, t2, t3 *float64) {
	*t1 = math.Floor(t / 10000.0)
	t -= *t1 * 10000.0
	*t2 = math.Floor(t / 100.0)
	*t3 = t - *t2*100.0
}

/* solution to covariance ----------------------------------------------------*/
func (sol *Sol) Sol2Cov(P []float64) {
	P[0] = float64(sol.Qr[0])                           /* xx or ee */
	P[4] = float64(sol.Qr[1])                           /* yy or nn */
	P[8] = float64(sol.Qr[2])                           /* zz or uu */
	P[1], P[3] = float64(sol.Qr[3]), float64(sol.Qr[3]) /* xy or en */
	P[5], P[7] = float64(sol.Qr[4]), float64(sol.Qr[4]) /* yz or nu */
	P[2], P[6] = float64(sol.Qr[5]), float64(sol.Qr[5]) /* zx or ue */
}

/* covariance to solution ----------------------------------------------------*/
func (sol *Sol) Cov2Sol(P []float64) {
	sol.Qr[0] = float32(P[0]) /* xx or ee */
	sol.Qr[1] = float32(P[4]) /* yy or nn */
	sol.Qr[2] = float32(P[8]) /* zz or uu */
	sol.Qr[3] = float32(P[1]) /* xy or en */
	sol.Qr[4] = float32(P[5]) /* yz or nu */
	sol.Qr[5] = float32(P[2]) /* zx or ue */
}

/* solution to velocity covariance -------------------------------------------*/
func (sol *Sol) Sol2CovarianceVel(P []float64) {
	P[0] = float64(sol.Qv[0])                           /* xx */
	P[4] = float64(sol.Qv[1])                           /* yy */
	P[8] = float64(sol.Qv[2])                           /* zz */
	P[1], P[3] = float64(sol.Qv[3]), float64(sol.Qv[3]) /* xy */
	P[5], P[7] = float64(sol.Qv[4]), float64(sol.Qv[4]) /* yz */
	P[2], P[6] = float64(sol.Qv[5]), float64(sol.Qv[5]) /* zx */
}

/* velocity covariance to solution -------------------------------------------*/
func (sol *Sol) CovarianceVel2Sol(P []float64) {
	sol.Qv[0] = float32(P[0]) /* xx */
	sol.Qv[1] = float32(P[4]) /* yy */
	sol.Qv[2] = float32(P[8]) /* zz */
	sol.Qv[3] = float32(P[1]) /* xy */
	sol.Qv[4] = float32(P[5]) /* yz */
	sol.Qv[5] = float32(P[2]) /* zx */
}

/* decode NMEA RMC (Recommended Minumum Specific GNSS Data) sentence ---------*/
func (sol *Sol) DecodeNmeaRmc(val []string, n int) int {
	var (
		tod, lat, lon, vel, dir, date, ang float64
		ep                                 [6]float64
		pos                                [3]float64
	)
	act := " "
	ns := "N"
	ew := "E"
	mew := "E"
	mode := "A"
	var i int

	Trace(4, "decode_nmearmc: n=%d\n", n)

	for i = 0; i < n; i++ {
		switch i {
		case 0:
			tod, _ = strconv.ParseFloat(val[i], 64) /* time in utc (hhmmss) */
		case 1:
			act = val[i] /* A=active,V=void */
		case 2:
			lat, _ = strconv.ParseFloat(val[i], 64) /* latitude (ddmm.mmm) */
		case 3:
			ns = val[i] /* N=north,S=south */
		case 4:
			lon, _ = strconv.ParseFloat(val[i], 64) /* longitude (dddmm.mmm) */
		case 5:
			ew = val[i] /* E=east,W=west */
		case 6:
			vel, _ = strconv.ParseFloat(val[i], 64) /* speed (knots) */
		case 7:
			dir, _ = strconv.ParseFloat(val[i], 64) /* track angle (deg) */
		case 8:
			date, _ = strconv.ParseFloat(val[i], 64) /* date (ddmmyy) */
		case 9:
			ang, _ = strconv.ParseFloat(val[i], 64) /* magnetic variation */
		case 10:
			mew = val[i] /* E=east,W=west */
		case 11:
			mode = val[i] /* mode indicator (>nmea 2) */
			/* A=autonomous,D=differential */
			/* E=estimated,N=not valid,S=simulator */
		}
	}
	if (act != "A" && act != "V") || (ns != "N" && ns != "S") || (ew != "E" && ew != "W") {
		Trace(2, "invalid nmea rmc format\n")
		return 0
	}
	var tmp float64 = 1.0
	if ns == "S" {
		tmp = -1.0
	}
	pos[0] = tmp * dmm2deg(lat) * D2R
	tmp = 1.0
	if ew == "W" {
		tmp = -1.0
	}
	pos[1] = tmp * dmm2deg(lon) * D2R
	septime(date, &ep[2], &ep[1], &ep[0])
	septime(tod, &ep[3], &ep[4], &ep[5])
	tmp = 1900.0
	if ep[0] < 80.0 {
		tmp = 2000.0
	}
	ep[0] += tmp
	sol.Time = Utc2GpsT(Epoch2Time(ep[:]))
	Pos2Ecef(pos[:], sol.Rr[:])
	var stat uint8 = uint8(SOLQ_SINGLE)
	if mode == "D" {
		stat = uint8(SOLQ_DGPS)
	}
	sol.Stat = stat
	sol.Ns = 0

	sol.Type = 0 /* postion type = xyz */

	Trace(5, "decode_nmearmc: %s rr=%.3f %.3f %.3f stat=%d ns=%d vel=%.2f dir=%.0f ang=%.0f mew=%s mode=%s\n",
		TimeStr(sol.Time, 0), sol.Rr[0], sol.Rr[1], sol.Rr[2], sol.Stat, sol.Ns,
		vel, dir, ang, mew, mode)

	return 2 /* update time */
}

/* decode NMEA ZDA (Time and Date) sentence ----------------------------------*/
func (sol *Sol) DecodeNmeaZda(val []string, n int) int {
	var (
		ep [6]float64
		i  int
	)
	tod := 0.0

	Trace(4, "decode_nmeazda: n=%d\n", n)

	for i = 0; i < n; i++ {
		switch i {
		case 0:
			tod, _ = strconv.ParseFloat(val[i], 64) /* time in utc (hhmmss) */
		case 1:
			ep[2], _ = strconv.ParseFloat(val[i], 64) /* day (0-31) */
		case 2:
			ep[1], _ = strconv.ParseFloat(val[i], 64) /* mon (1-12) */
		case 3:
			ep[0], _ = strconv.ParseFloat(val[i], 64) /* year */
		}
	}
	septime(tod, &ep[3], &ep[4], &ep[5])
	sol.Time = Utc2GpsT(Epoch2Time(ep[:]))
	sol.Ns = 0

	Trace(5, "decode_nmeazda: %s\n", TimeStr(sol.Time, 0))

	return 2 /* update time */
}

/* decode NMEA GGA (Global Positioning System Fix Data) sentence -------------*/
func (sol *Sol) DecodeNmeaGga(val []string, n int) int {
	var (
		time                                   Gtime
		tod, lat, lon, hdop, alt, msl, tt, age float64
		ep                                     [6]float64
		pos                                    [3]float64
		ns, ew, ua, um                         string = "N", "E", " ", " "
		i, solq, nrcv                          int
	)
	Trace(4, "decode_nmeagga: n=%d\n", n)
	var tmp float64
	for i = 0; i < n; i++ {
		switch i {
		case 0:
			tod, _ = strconv.ParseFloat(val[i], 64) /* UTC of position (hhmmss) */
		case 1:
			lat, _ = strconv.ParseFloat(val[i], 64) /* Latitude (ddmm.mmm) */
		case 2:
			ns = val[i] /* N=north,S=south */
		case 3:
			lon, _ = strconv.ParseFloat(val[i], 64) /* Longitude (dddmm.mmm) */
		case 4:
			ew = val[i] /* E=east,W=west */
		case 5:
			si, _ := strconv.ParseInt(val[i], 0, 0)
			solq = int(si) /* GPS quality indicator */
		case 6:
			si, _ := strconv.ParseInt(val[i], 0, 0)
			nrcv = int(si) /* # of satellites in use */
		case 7:
			hdop, _ = strconv.ParseFloat(val[i], 64) /* HDOP */
		case 8:
			alt, _ = strconv.ParseFloat(val[i], 64) /* Altitude MSL */
		case 9:
			ua = val[i] /* unit (M) */
		case 10:
			msl, _ = strconv.ParseFloat(val[i], 64) /* Geoid separation */
		case 11:
			um = val[i] /* unit (M) */
		case 12:
			age, _ = strconv.ParseFloat(val[i], 64) /* Age of differential */
		}
	}
	if (ns != "N" && ns != "S") || (ew != "E" && ew != "W") {
		Trace(2, "invalid nmea gga format\n")
		return 0
	}
	if sol.Time.Time == 0 {
		Trace(2, "no date info for nmea gga\n")
		return 0
	}
	tmp = -1.0
	if ns == "N" {
		tmp = 1.0
	}
	pos[0] = tmp * dmm2deg(lat) * D2R
	tmp = -1.0
	if ew == "E" {
		tmp = 1.0
	}
	pos[1] = tmp * dmm2deg(lon) * D2R
	pos[2] = alt + msl

	Time2Epoch(sol.Time, ep[:])
	septime(tod, &ep[3], &ep[4], &ep[5])
	time = Utc2GpsT(Epoch2Time(ep[:]))
	tt = TimeDiff(time, sol.Time)
	switch {
	case tt < -43200.0:
		sol.Time = TimeAdd(time, 86400.0)
	case tt > 43200.0:
		sol.Time = TimeAdd(time, -86400.0)
	default:
		sol.Time = time
	}
	Pos2Ecef(pos[:], sol.Rr[:])
	var tmpi uint8 = SOLQ_NONE
	if 0 <= solq && solq <= 8 {
		tmpi = uint8(nmea_solq[solq])
	}
	sol.Stat = tmpi
	sol.Ns = uint8(nrcv)
	sol.Age = float32(age)

	sol.Type = 0 /* postion type = xyz */

	Trace(5, "decode_nmeagga: %s rr=%.3f %.3f %.3f stat=%d ns=%d hdop=%.1f ua=%s um=%s\n",
		TimeStr(sol.Time, 0), sol.Rr[0], sol.Rr[1], sol.Rr[2], sol.Stat, sol.Ns,
		hdop, ua, um)

	return 1
}

/* test NMEA sentence header -------------------------------------------------*/
func TestNmea(buff []byte) int {
	if len(buff) < 6 || buff[0] != '$' {
		return 0
	}
	if string(buff[1:3]) == "GP" || string(buff[1:3]) == "GA" || /* NMEA 4.10 [1] */
		string(buff[1:3]) == "GL" || string(buff[1:3]) == "GN" ||
		string(buff[1:3]) == "GB" || string(buff[1:3]) == "GQ" || /* NMEA 4.11 [2] */
		string(buff[1:3]) == "GI" ||
		string(buff[1:3]) == "BD" || string(buff[1:3]) == "QZ" /* extension */ {
		return 1
	}
	return 0
}

/* test solution status message header ---------------------------------------*/
func TestSolStat(buff []byte) int {
	if len(buff) < 7 || buff[0] != '$' {
		return 0
	}
	if string(buff[1:4]) == "POS" || string(buff[1:7]) == "VELACC" ||
		string(buff[1:4]) == "CLK" || string(buff[1:4]) == "ION" ||
		string(buff[1:5]) == "TROP" || string(buff[1:7]) == "HWBIAS" ||
		string(buff[1:5]) == "TRPG" || string(buff[1:4]) == "AMB" ||
		string(buff[1:4]) == "SAT" {
		return 1
	}
	return 0
}

/* decode NMEA sentence ------------------------------------------------------*/
func (sol *Sol) DecodeNmea(buff []byte) int {
	//char *p,*q,*val[MAXFIELD];
	var n int = 0

	Trace(4, "decode_nmea: buff=%s\n", buff)

	val := strings.FieldsFunc(string(buff), func(r rune) bool { return r == ',' || r == '*' })

	if n = len(val); n < 1 {
		return 0
	}
	switch val[3] {
	case "RMC": /* $xxRMC */
		return sol.DecodeNmeaRmc(val[1:], n-1)
	case "ZDA": /* $xxZDA */
		return sol.DecodeNmeaZda(val[1:], n-1)
	case "GGA": /* $xxGGA */
		return sol.DecodeNmeaGga(val[1:], n-1)
	}
	return 0
}

/* decode solution time ------------------------------------------------------*/
func DecodeSolTime(buff string, opt *SolOpt, time *Gtime) string {
	var (
		v                   [MAXFIELD]float64
		i, n, length, index int
		s                   string
	)
	Trace(4, "decode_soltime:\n")

	if opt.Sep == "\\t" {
		s = "\t"
	} else if opt.Sep != "" {
		s = opt.Sep
	}
	length = len(s)

	if opt.Posf == SOLF_STAT {
		return buff
	}
	if opt.Posf == SOLF_GSIF {
		n, _ = fmt.Sscanf(buff, "%f %f %f %f:%f:%f", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5])
		if n < 6 {
			return ""
		}
		*time = TimeAdd(Epoch2Time(v[:]), -12.0*3600.0)

		p := strings.Split(buff, ":")
		if len(p) < 3 {
			return ""
		}

		index = strings.IndexFunc(p[2], func(r rune) bool {
			if unicode.IsDigit(r) || r == '.' {
				return false
			}
			return true
		})

		// reg := regexp.MustCompile("[0-9]|[/.]")
		// ids := reg.FindStringIndex(p[2])
		// if len(ids) > 0 {
		// 	index = ids[0]
		// } else {
		// 	return ""
		// }
		return p[2][index+length:]
	}
	/* yyyy/mm/dd hh:mm:ss or yyyy mm dd hh:mm:ss */
	if n, _ = fmt.Sscanf(buff, "%f/%f/%f %f:%f:%f", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]); n >= 6 {
		if v[0] < 100.0 {
			if v[0] < 80.0 {
				v[0] += 2000.0
			} else {
				v[0] += 1900.0
			}
		}
		*time = Epoch2Time(v[:])
		if opt.TimeS == TIMES_UTC {
			*time = Utc2GpsT(*time)
		} else if opt.TimeS == TIMES_JST {
			*time = Utc2GpsT(TimeAdd(*time, -9*3600.0))
		}

		p := strings.Split(buff, ":")
		if len(p) < 3 {
			return ""
		}
		index = strings.IndexFunc(p[2], func(r rune) bool {
			if unicode.IsDigit(r) || r == '.' {
				return false
			}
			return true
		})
		return p[2][index+length:]
	} else { /* wwww ssss */
		idx := 0
		for i = 0; i < 2; {
			index = strings.Index(buff, s)
			if n, _ = fmt.Sscanf(buff[idx:], "%f", &v[i]); n == 1 {
				i++
			}
			if index < 0 {
				break
			}
			idx += index + len(s)
		}

		if n >= 2 && 0.0 <= v[0] && v[0] <= 3000.0 && 0.0 <= v[1] && v[1] < 604800.0 {
			*time = GpsT2Time(int(v[0]), v[1])
			return buff[idx:]
		}
	}
	return ""
}

/* decode x/y/z-ecef ---------------------------------------------------------*/
func (sol *Sol) DecodeSolXyz(buff string, opt *SolOpt) int {
	var (
		val     [MAXFIELD]float64
		P       [9]float64
		i, j, n int
	)
	sep := opt2sep(opt)

	Trace(4, "decode_solxyz:\n")

	if n = tonum(buff, sep, val[:]); n < 3 {
		return 0
	}

	for j = 0; j < 3; j++ {
		sol.Rr[j] = val[i] /* xyz */
		i++
	}
	if i < n {
		sol.Stat = uint8(val[i])
		i++
	}
	if i < n {
		sol.Ns = uint8(val[i])
		i++
	}
	if i+3 <= n {
		P[0] = SQRS(val[i])
		i++ /* sdx */
		P[4] = SQRS(val[i])
		i++ /* sdy */
		P[8] = SQRS(val[i])
		i++ /* sdz */
		if i+3 <= n {
			P[1] = SQRS(val[i])
			P[3] = P[1]
			i++ /* sdxy */
			P[5] = SQRS(val[i])
			P[7] = P[5]
			i++ /* sdyz */
			P[2] = SQRS(val[i])
			P[6] = P[2]
			i++ /* sdzx */
		}
		sol.Cov2Sol(P[:])
	}
	if i < n {
		sol.Age = float32(val[i])
		i++
	}
	if i < n {
		sol.Ratio = float32(val[i])
		i++
	}

	if i+3 <= n { /* velocity */
		for j = 0; j < 3; j++ {
			sol.Rr[j+3] = val[i] /* xyz */
			i++
		}
	}
	if i+3 <= n {
		for j = 0; j < 9; j++ {
			P[j] = 0.0
		}
		P[0] = SQRS(val[i])
		i++ /* sdx */
		P[4] = SQRS(val[i])
		i++ /* sdy */
		P[8] = SQRS(val[i])
		i++ /* sdz */
		if i+3 < n {
			P[1] = SQRS(val[i])
			P[3] = P[1]
			i++ /* sdxy */
			P[5] = SQRS(val[i])
			P[7] = P[5]
			i++ /* sdyz */
			P[2] = SQRS(val[i])
			P[6] = P[2]
			i++ /* sdzx */
		}
		sol.CovarianceVel2Sol(P[:])
	}
	sol.Type = 0 /* postion type = xyz */

	if MAXSOLQ < sol.Stat {
		sol.Stat = SOLQ_NONE
	}
	return 1
}

/* decode lat/lon/height -----------------------------------------------------*/
func (sol *Sol) DecodeSolLatLonHeight(buff string, opt *SolOpt) int {
	var (
		val      [MAXFIELD]float64
		pos, vel [3]float64
		Q, P     [9]float64
		i, j, n  int
	)
	sep := opt2sep(opt)

	Trace(4, "decode_solllh:\n")

	n = tonum(buff, sep, val[:])

	if opt.DegF == 0 {
		if n < 3 {
			return 0
		}
		pos[0] = val[i] * D2R
		i++ /* lat/lon/hgt (ddd.ddd) */
		pos[1] = val[i] * D2R
		i++
		pos[2] = val[i]
		i++
	} else {
		if n < 7 {
			return 0
		}
		pos[0] = Dms2Deg(val[:]) * D2R /* lat/lon/hgt (ddd mm ss) */
		pos[1] = Dms2Deg(val[3:]) * D2R
		pos[2] = val[6]
		i += 7
	}
	Pos2Ecef(pos[:], sol.Rr[:])
	if i < n {
		sol.Stat = uint8(val[i])
		i++
	}
	if i < n {
		sol.Ns = uint8(val[i])
		i++
	}
	if i+3 <= n {
		Q[4] = SQRS(val[i])
		i++ /* sdn */
		Q[0] = SQRS(val[i])
		i++ /* sde */
		Q[8] = SQRS(val[i])
		i++ /* sdu */
		if i+3 < n {
			Q[1] = SQRS(val[i])
			Q[3] = Q[1]
			i++ /* sdne */
			Q[2] = SQRS(val[i])
			Q[6] = Q[2]
			i++ /* sdeu */
			Q[5] = SQRS(val[i])
			Q[7] = Q[5]
			i++ /* sdun */
		}
		Cov2Ecef(pos[:], Q[:], P[:])
		sol.Cov2Sol(P[:])
	}
	if i < n {
		sol.Age = float32(val[i])
		i++
	}
	if i < n {
		sol.Ratio = float32(val[i])
		i++
	}

	if i+3 <= n { /* velocity */
		vel[1] = val[i]
		i++ /* vel-n */
		vel[0] = val[i]
		i++ /* vel-e */
		vel[2] = val[i]
		i++ /* vel-u */
		Enu2Ecef(pos[:], vel[:], sol.Rr[3:])
	}
	if i+3 <= n {
		for j = 0; j < 9; j++ {
			Q[j] = 0.0
		}
		Q[4] = SQRS(val[i])
		i++ /* sdn */
		Q[0] = SQRS(val[i])
		i++ /* sde */
		Q[8] = SQRS(val[i])
		i++ /* sdu */
		if i+3 <= n {
			Q[1] = SQRS(val[i])
			Q[3] = Q[1]
			i++ /* sdne */
			Q[2] = SQRS(val[i])
			Q[6] = Q[2]
			i++ /* sdeu */
			Q[5] = SQRS(val[i])
			Q[7] = Q[5]
			i++ /* sdun */
		}
		Cov2Ecef(pos[:], Q[:], P[:])
		sol.CovarianceVel2Sol(P[:])
	}
	sol.Type = 0 /* postion type = xyz */

	if MAXSOLQ < sol.Stat {
		sol.Stat = SOLQ_NONE
	}
	return 1
}

/* decode e/n/u-baseline -----------------------------------------------------*/
func (sol *Sol) DecodeSolEnu(buff string, opt *SolOpt) int {
	var (
		val     [MAXFIELD]float64
		Q       [9]float64
		i, j, n int
	)
	sep := opt2sep(opt)

	Trace(4, "decode_solenu:\n")

	if n = tonum(buff, sep, val[:]); n < 3 {
		return 0
	}

	for j = 0; j < 3; j++ {
		sol.Rr[j] = val[i]
		i++ /* enu */
	}
	if i < n {
		sol.Stat = uint8(val[i])
		i++
	}
	if i < n {
		sol.Ns = uint8(val[i])
		i++
	}
	if i+3 <= n {
		Q[0] = SQRS(val[i])
		i++ /* sde */
		Q[4] = SQRS(val[i])
		i++ /* sdn */
		Q[8] = SQRS(val[i])
		i++ /* sdu */
		if i+3 <= n {
			Q[1] = SQRS(val[i])
			Q[3] = Q[1]
			i++ /* sden */
			Q[5] = SQRS(val[i])
			Q[7] = Q[5]
			i++ /* sdnu */
			Q[2] = SQRS(val[i])
			Q[6] = Q[2]
			i++ /* sdue */
		}
		sol.Cov2Sol(Q[:])
	}
	if i < n {
		sol.Age = float32(val[i])
		i++
	}
	if i < n {
		sol.Ratio = float32(val[i])
		i++
	}

	sol.Type = 1 /* postion type = enu */

	if MAXSOLQ < sol.Stat {
		sol.Stat = SOLQ_NONE
	}
	return 1
}

/* decode solution status ----------------------------------------------------*/
func (sol *Sol) DecodeSolSss(buff []byte) int {
	var (
		tow           float64
		pos, std      [3]float64
		i, week, solq int
	)

	Trace(4, "decode_solsss:\n")

	if n, _ := fmt.Sscanf(string(buff), "$POS,%d,%f,%d,%f,%f,%f,%f,%f,%f", &week, &tow, &solq,
		&pos[0], &pos[1], &pos[2], &std[0], &std[1], &std[2]); n < 6 {
		return 0
	}
	if week <= 0 || Norm(pos[:], 3) <= 0.0 || solq == SOLQ_NONE {
		return 0
	}
	sol.Time = GpsT2Time(week, tow)
	for i = 0; i < 6; i++ {
		if i < 3 {
			sol.Rr[i] = pos[i]
			sol.Qr[i] = float32(SQRS(std[i]))
		} else {
			sol.Rr[i] = 0.0
			sol.Qr[i] = 0.0
		}

		sol.Dtr[i] = 0.0
	}
	sol.Ns = 0
	sol.Age, sol.Ratio, sol.Thres = 0.0, 0.0, 0.0
	sol.Type = 0 /* position type = xyz */
	sol.Stat = uint8(solq)
	return 1
}

/* decode GSI F solution -----------------------------------------------------*/
func (sol *Sol) DecodeSolGsi(buff string, opt *SolOpt) int {
	var (
		val  [MAXFIELD]float64
		i, j int
	)

	Trace(4, "decode_solgsi:\n")

	if n := tonum(buff, " ", val[:]); n < 3 {
		return 0
	}

	for j = 0; j < 3; j++ {
		sol.Rr[j] = val[i] /* xyz */
		i++
	}
	sol.Stat = SOLQ_FIX
	return 1
}

/* decode solution position --------------------------------------------------*/
func (sol *Sol) DecodeSolPos(buff string, opt *SolOpt) int {
	var sol0 Sol

	Trace(4, "decode_solpos: buff=%s\n", buff)

	*sol = sol0

	/* decode solution time */
	if buff = DecodeSolTime(buff, opt, &sol.Time); len(buff) == 0 {
		return 0
	}
	/* decode solution position */
	switch byte(opt.Posf) {
	case SOLF_XYZ:
		return sol.DecodeSolXyz(buff, opt)
	case SOLF_LLH:
		return sol.DecodeSolLatLonHeight(buff, opt)
	case SOLF_ENU:
		return sol.DecodeSolEnu(buff, opt)
	case SOLF_GSIF:
		return sol.DecodeSolGsi(buff, opt)
	}
	return 0
}

/* decode reference position -------------------------------------------------*/
func DecodeRefPos(buff []byte, opt *SolOpt, rb []float64) {
	var (
		val [MAXFIELD]float64
		pos [3]float64
		n   int
	)
	sep := opt2sep(opt)

	Trace(4, "decode_refpos: buff=%s\n", buff)

	if n = tonum(string(buff), sep, val[:]); n < 3 {
		return
	}

	switch {
	case opt.Posf == SOLF_XYZ: /* xyz */
		for i := 0; i < 3; i++ {
			rb[i] = val[i]
		}
	case opt.DegF == 0: /* lat/lon/hgt (ddd.ddd) */
		pos[0] = val[0] * D2R
		pos[1] = val[1] * D2R
		pos[2] = val[2]
		Pos2Ecef(pos[:], rb)
	case opt.DegF == 1 && n >= 7: /* lat/lon/hgt (ddd mm ss) */
		pos[0] = Dms2Deg(val[:]) * D2R
		pos[1] = Dms2Deg(val[3:]) * D2R
		pos[2] = val[6]
		Pos2Ecef(pos[:], rb)
	}
}

/* decode solution -----------------------------------------------------------*/
func (sol *Sol) DecodeSol(buff []byte, opt *SolOpt, rb []float64) int {
	var index int
	Trace(4, "decode_sol: buff=%s\n", buff)

	if TestNmea(buff) > 0 { /* decode nmea */
		return sol.DecodeNmea(buff)
	} else if TestSolStat(buff) > 0 { /* decode solution status */
		return sol.DecodeSolSss(buff)
	}
	if len(buff) > 0 && string(buff[0]) == COMMENTH {
		if !strings.Contains(string(buff), "ref pos") && !strings.Contains(string(buff), "slave pos") {
			return 0
		}
		if index = strings.Index(string(buff), ":"); index < 0 {
			return 0
		}
		DecodeRefPos(buff[index+1:], opt, rb)
		return 0
	}
	/* decode position record */
	return sol.DecodeSolPos(string(buff[:]), opt)
}

/* decode solution options ---------------------------------------------------*/
func DecodeSolOpt(buff string, opt *SolOpt) {
	Trace(4, "decode_solhead: buff=%s\n", buff)
	var index int
	if string(buff[0]) != COMMENTH && string(buff) != "+" {
		return
	}

	switch {
	case strings.Contains(buff, "GPST"):
		opt.TimeS = TIMES_GPST
	case strings.Contains(buff, "UTC"):
		opt.TimeS = TIMES_UTC
	case strings.Contains(buff, "JST"):
		opt.TimeS = TIMES_JST
	}

	if index = strings.Index(buff, "x-ecef(m)"); index >= 0 {
		opt.Posf = SOLF_XYZ
		opt.DegF = 0
		opt.Sep = string(buff[9+index])

	} else if index = strings.Index(buff, "latitude(d'\")"); index >= 0 {
		opt.Posf = SOLF_LLH
		opt.DegF = 1
		opt.Sep = string(buff[index+14])

	} else if index = strings.Index(buff, "latitude(deg)"); index >= 0 {
		opt.Posf = SOLF_LLH
		opt.DegF = 0
		opt.Sep = string(buff[index+13])
	} else if index = strings.Index(buff, "e-baseline(m)"); index >= 0 {
		opt.Posf = SOLF_ENU
		opt.DegF = 0
		opt.Sep = string(buff[index+13])
	} else if index = strings.Index(buff, "+SITE/INF"); index >= 0 { /* gsi f2/f3 solution */
		opt.TimeS = TIMES_GPST
		opt.Posf = SOLF_GSIF
		opt.DegF = 0
		opt.Sep = " "
	}
}

/* read solution option ------------------------------------------------------*/
func ReadSolOpt(fp *os.File, opt *SolOpt) {
	var i int

	Trace(4, "readsolopt:\n")
	reader := bufio.NewReader(fp)

	for i = 0; i < 100; i++ {
		buff, _, err := reader.ReadLine()
		if err == io.EOF {
			break
		}
		DecodeSolOpt(string(buff), opt)
	}
}

/* input solution data from stream ---------------------------------------------
* input solution data from stream
* args   : uint8_t data     I stream data
*          gtime_t ts       I  start time (ts.time==0: from start)
*          gtime_t te       I  end time   (te.time==0: to end)
*          double tint      I  time interval (0: all)
*          int    qflag     I  quality flag  (0: all)
*          solbuf_t *solbuf IO solution buffer
* return : status (1:solution received,0:no solution,-1:disconnect received)
*-----------------------------------------------------------------------------*/
func InputSol(data uint8, ts, te Gtime, tint float64,
	qflag int, opt *SolOpt, solbuf *SolBuf) int {
	var (
		sol  Sol
		stat int
	)

	Trace(4, "inputsol: data=0x%02x\n", data)

	sol.Time = solbuf.Time

	if data == '$' || (!strconv.IsPrint(rune(data)) && data != '\r' && data != '\n') { /* sync header */
		solbuf.buff = nil
		//	solbuf.nb = 0
	}
	if data != '\r' && data != '\n' {
		solbuf.buff = append(solbuf.buff, data)
		// solbuf.buff[solbuf.nb] = data
		// solbuf.nb++
	}
	if data != '\n' && len(solbuf.buff) < MAXSOLMSG {
		return 0
	} /* sync trailer */

	//	solbuf.buff[solbuf.nb] = 0
	//	solbuf.nb = 0
	idx := len(solbuf.buff)
	defer func() {
		solbuf.buff = nil
	}()
	/* check disconnect message */
	if len(solbuf.buff) > len(MSG_DISCONN)-2 && strings.Compare(string(solbuf.buff[:len(MSG_DISCONN)-2]), MSG_DISCONN) == 0 {
		Trace(2, "disconnect received\n")
		return -1
	}
	/* decode solution */
	sol.Time = solbuf.Time
	if stat = sol.DecodeSol(solbuf.buff[:idx], opt, solbuf.Rb[:]); stat > 0 {
		if stat > 0 {
			solbuf.Time = sol.Time
		} /* update current time */
		if stat != 1 {
			return 0
		}
	}
	if stat != 1 || ScreenTime(sol.Time, ts, te, tint) == 0 || (qflag > 0 && int(sol.Stat) != qflag) {
		return 0
	}
	/* add solution to solution buffer */
	return sol.AddSol(solbuf)
}

/* read solution data --------------------------------------------------------*/
func ReadSolData(fp *os.File, ts, te Gtime, tint float64, qflag int, opt *SolOpt, solbuf *SolBuf) int {

	var (
		c   rune
		err error
	)
	Trace(4, "readsoldata:\n")
	reader := bufio.NewReader(fp)
	for {
		if c, _, err = reader.ReadRune(); err == io.EOF {
			break
		}
		InputSol(uint8(c), ts, te, tint, qflag, opt, solbuf)
	}
	if solbuf.N > 0 {
		return 1
	}
	return 0
}

/* sort solution data --------------------------------------------------------*/
func SortSolBuf(solbuf *SolBuf) int {
	Trace(4, "sort_solbuf: n=%d\n", solbuf.N)

	if solbuf.N <= 0 {
		return 0
	}
	sort.Slice(solbuf.Data[:solbuf.N], func(i, j int) bool {
		tt := TimeDiff(solbuf.Data[i].Time, solbuf.Data[j].Time)
		return tt < 0.0
	})
	solbuf.Nmax = solbuf.N
	solbuf.Start = 0
	solbuf.End = solbuf.N - 1
	return 1
}

/* read solutions data from solution files -------------------------------------
* read solution data from soluiton files
* args   : char   *files[]  I  solution files
*          int    nfile     I  number of files
*         (gtime_t ts)      I  start time (ts.time==0: from start)
*         (gtime_t te)      I  end time   (te.time==0: to end)
*         (double tint)     I  time interval (0: all)
*         (int    qflag)    I  quality flag  (0: all)
*          solbuf_t *solbuf O  solution buffer
* return : status (1:ok,0:no data or error)
*-----------------------------------------------------------------------------*/
func ReadSolt(files []string, nfile int, ts, te Gtime,
	tint float64, qflag int, solbuf *SolBuf) int {
	var (
		fp  *os.File
		opt SolOpt = DefaultSolOpt()
		err error
	)

	Trace(4, "readsolt: nfile=%d\n", nfile)

	solbuf.InitSolBuf(0, 0)

	for i := 0; i < nfile; i++ {
		if fp, err = os.OpenFile(files[i], os.O_RDONLY, 0666); err == io.EOF {
			Trace(2, "readsolt: file open error %s\n", files[i])
			continue
		}
		/* read solution options in header */
		ReadSolOpt(fp, &opt)
		fp.Seek(0, 0)

		/* read solution data */
		if ReadSolData(fp, ts, te, tint, qflag, &opt, solbuf) == 0 {
			Trace(2, "readsolt: no solution in %s\n", files[i])
		}
		fp.Close()
	}
	return SortSolBuf(solbuf)
}
func ReadSol(files []string, nfile int, sol *SolBuf) int {
	var time Gtime

	Trace(4, "readsol: nfile=%d\n", nfile)

	return ReadSolt(files, nfile, time, time, 0.0, 0, sol)
}

/* add solution data to solution buffer ----------------------------------------
* add solution data to solution buffer
* args   : solbuf_t *solbuf IO solution buffer
*          sol_t  *sol      I  solution data
* return : status (1:ok,0:error)
*-----------------------------------------------------------------------------*/
func (sol *Sol) AddSol(solbuf *SolBuf) int {

	Trace(4, "addsol:\n")

	if solbuf.Cyclic > 0 { /* ring buffer */
		if solbuf.Nmax <= 1 {
			return 0
		}
		solbuf.Data = append(solbuf.Data, *sol)

		if solbuf.End++; solbuf.End >= solbuf.Nmax {
			solbuf.End = 0
		}
		if solbuf.Start == solbuf.End {
			if solbuf.Start++; solbuf.Start >= solbuf.Nmax {
				solbuf.Start = 0
			}
		} else {
			solbuf.N++
		}

		return 1
	}
	if solbuf.N >= solbuf.Nmax {
		if solbuf.Nmax == 0 {
			solbuf.Nmax = 8192
		} else {
			solbuf.Nmax = solbuf.Nmax * 2
		}
		solbuf_data := make([]Sol, solbuf.Nmax)
		copy(solbuf_data, solbuf.Data)
		solbuf.Data = solbuf_data
	}
	solbuf.Data[solbuf.N] = *sol
	solbuf.N++
	return 1
}

/* get solution data from solution buffer --------------------------------------
* get solution data by index from solution buffer
* args   : solbuf_t *solbuf I  solution buffer
*          int    index     I  index of solution (0...)
* return : solution data pointer (NULL: no solution, out of range)
*-----------------------------------------------------------------------------*/
func GetSol(solbuf *SolBuf, index int) []Sol {
	Trace(4, "getsol: index=%d\n", index)

	if index < 0 || solbuf.N <= index {
		return nil
	}
	if index = solbuf.Start + index; index >= solbuf.Nmax {
		index -= solbuf.Nmax
	}
	return solbuf.Data[index:]
}

/* initialize solution buffer --------------------------------------------------
* initialize position solutions
* args   : solbuf_t *solbuf I  solution buffer
*          int    cyclic    I  solution data buffer type (0:linear,1:cyclic)
*          int    nmax      I  initial number of solution data
* return : status (1:ok,0:error)
*-----------------------------------------------------------------------------*/
func (solbuf *SolBuf) InitSolBuf(cyclic, nmax int) {

	Trace(4, "initsolbuf: cyclic=%d nmax=%d\n", cyclic, nmax)

	solbuf.N, solbuf.Nmax, solbuf.Start, solbuf.End = 0, 0, 0, 0
	solbuf.Cyclic = cyclic

	solbuf.Data = nil
	for i := 0; i < 3; i++ {
		solbuf.Rb[i] = 0.0
	}
	if cyclic > 0 {
		if nmax <= 2 {
			nmax = 2
		}
		if solbuf.Data = make([]Sol, nmax); solbuf.Data == nil {
			Trace(2, "initsolbuf: memory allocation error\n")
			return
		}
		solbuf.Nmax = nmax
	}
}

/* free solution ---------------------------------------------------------------
* free memory for solution buffer
* args   : solbuf_t *solbuf I  solution buffer
* return : none
*-----------------------------------------------------------------------------*/
func (solbuf *SolBuf) FreeSolBuf() {
	Trace(4, "freesolbuf: n=%d\n", solbuf.N)

	solbuf.Data = nil
	solbuf.N, solbuf.Nmax, solbuf.Start, solbuf.End = 0, 0, 0, 0
	for i := 0; i < 3; i++ {
		solbuf.Rb[i] = 0.0
	}
}
func (solstatbuf *SolStatBuf) FreeSolStatBuf() {
	Trace(4, "freesolstatbuf: n=%d\n", solstatbuf.n)

	solstatbuf.n, solstatbuf.nmax = 0, 0
	solstatbuf.data = nil
}

/* sort solution data --------------------------------------------------------*/
func (statbuf *SolStatBuf) SortSolStat() int {
	Trace(4, "sort_solstat: n=%d\n", statbuf.n)

	if statbuf.n <= 0 {
		return 0
	}
	sort.Slice(statbuf.data[:statbuf.n], func(i, j int) bool {
		tt := TimeDiff(statbuf.data[i].Time, statbuf.data[j].Time)
		return tt < 0.0
	})

	statbuf.nmax = statbuf.n
	return 1
}

/* decode solution status ----------------------------------------------------*/
func (stat *SolStat) DecodeSolStat(buff string) int {
	var (
		stat0                                                       SolStat
		tow, az, el, resp, resc, snr                                float64
		n, week, sat, frq, vsat, fix, slip, lock, outc, slipc, rejc int
		id                                                          string = ""
	)

	Trace(4, "decode_solstat: buff=%s\n", buff)

	if strings.Index(buff, "$SAT") != 0 {
		return 0
	}
	buff = strings.ReplaceAll(buff, ",", " ")
	n, _ = fmt.Sscanf(buff, "$SAT%d%f%s%d%f%f%f%f%d%f%d%d%d%d%d%d",
		&week, &tow, id, &frq, &az, &el, &resp, &resc, &vsat, &snr, &fix, &slip,
		&lock, &outc, &slipc, &rejc)

	if n < 15 {
		Trace(2, "invalid format of solution status: %s\n", buff)
		return 0
	}
	if sat = SatId2No(id); sat <= 0 {
		Trace(2, "invalid satellite in solution status: %s\n", id)
		return 0
	}
	*stat = stat0
	stat.Time = GpsT2Time(week, tow)
	stat.Sat = uint8(sat)
	stat.Frq = uint8(frq)
	stat.Az = float32(az * D2R)
	stat.El = float32(el * D2R)
	stat.Resp = float32(resp)
	stat.Resc = float32(resc)
	stat.Flag = uint8((vsat << 5) + (slip << 3) + fix)
	stat.Snr = uint16(snr/float64(SNR_UNIT) + 0.5)
	stat.lock = uint16(lock)
	stat.outc = uint16(outc)
	stat.slipc = uint16(slipc)
	stat.rejc = uint16(rejc)
	return 1
}

/* add solution status data --------------------------------------------------*/
func (statbuf *SolStatBuf) AddSolStat(stat *SolStat) {
	Trace(4, "addsolstat:\n")

	if statbuf.n >= statbuf.nmax {
		if statbuf.nmax == 0 {
			statbuf.nmax = 8192
		} else {
			statbuf.nmax = statbuf.nmax * 2

		}
		statbuf_data := make([]SolStat, statbuf.nmax)
		copy(statbuf_data, statbuf.data)
		statbuf.data = statbuf_data
	}
	statbuf.data[statbuf.n] = *stat
	statbuf.n++
}

/* read solution status data -------------------------------------------------*/
func (statbuf *SolStatBuf) ReadSolStatData(fp *os.File, ts, te Gtime, tint float64) int {
	var stat SolStat

	var err error
	Trace(4, "readsolstatdata:\n")

	fileinfo, _ := fp.Stat()
	buff := make([]byte, fileinfo.Size())

	for {
		_, err = fp.Read(buff)
		if err == io.EOF {
			break
		}
		/* decode solution status */
		if stat.DecodeSolStat(string(buff)) == 0 {
			continue
		}

		/* add solution to solution buffer */
		if ScreenTime(stat.Time, ts, te, tint) > 0 {
			statbuf.AddSolStat(&stat)
		}
	}

	if statbuf.n > 0 {
		return 1
	}
	return 0
}

/* read solution status --------------------------------------------------------
* read solution status from solution status files
* args   : char   *files[]  I  solution status files
*          int    nfile     I  number of files
*         (gtime_t ts)      I  start time (ts.time==0: from start)
*         (gtime_t te)      I  end time   (te.time==0: to end)
*         (double tint)     I  time interval (0: all)
*          solstatbuf_t *statbuf O  solution status buffer
* return : status (1:ok,0:no data or error)
*-----------------------------------------------------------------------------*/
func (statbuf *SolStatBuf) ReadSolStatt(files []string, nfile int, ts, te Gtime,
	tint float64) int {
	var (
		fp    *os.File
		path  string
		index int
		err   error
	)
	Trace(4, "readsolstatt: nfile=%d\n", nfile)

	statbuf.n, statbuf.nmax = 0, 0
	statbuf.data = nil

	for i := 0; i < nfile; i++ {
		index = strings.LastIndex(files[i], ".")
		if index >= 0 && strings.EqualFold(files[i][index:], ".stat") {
			path = files[i]
		} else {
			path = fmt.Sprintf("%s.stat", files[i])
		}
		if fp, err = os.OpenFile(path, os.O_RDONLY, 0666); err != nil {
			Trace(2, "readsolstatt: file open error %s\n", path)
			continue
		}
		defer fp.Close()
		/* read solution status data */
		if statbuf.ReadSolStatData(fp, ts, te, tint) == 0 {
			Trace(2, "readsolstatt: no solution in %s\n", path)
		}
	}
	return statbuf.SortSolStat()
}
func (statbuf *SolStatBuf) ReadSolStat(files []string, nfile int) int {
	var time Gtime

	Trace(4, "readsolstat: nfile=%d\n", nfile)

	return statbuf.ReadSolStatt(files, nfile, time, time, 0.0)
}

/* output solution as the form of x/y/z-ecef ---------------------------------*/
func OutEcef(buff *string, s string, sol *Sol, opt *SolOpt) int {
	sep := opt2sep(opt)

	Trace(4, "outecef:\n")
	p := *buff
	p += fmt.Sprintf("%s%s%14.4f%s%14.4f%s%14.4f%s%3d%s%3d%s%8.4f%s%8.4f%s%8.4f%s%8.4f%s%8.4f%s%8.4f%s%6.2f%s%6.1f",
		s, sep, sol.Rr[0], sep, sol.Rr[1], sep, sol.Rr[2], sep, sol.Stat, sep,
		sol.Ns, sep, SQRT32(sol.Qr[0]), sep, SQRT32(sol.Qr[1]), sep,
		SQRT32(sol.Qr[2]), sep, sqvar(float64(sol.Qr[3])), sep, sqvar(float64(sol.Qr[4])), sep,
		sqvar(float64(sol.Qr[5])), sep, sol.Age, sep, sol.Ratio)

	if opt.OutVel > 0 { /* output velocity */
		p += fmt.Sprintf("%s%10.5f%s%10.5f%s%10.5f%s%9.5f%s%8.5f%s%8.5f%s%8.5f%s%8.5f%s%8.5f",
			sep, sol.Rr[3], sep, sol.Rr[4], sep, sol.Rr[5], sep,
			SQRT32(sol.Qv[0]), sep, SQRT32(sol.Qv[1]), sep, SQRT32(sol.Qv[2]),
			sep, sqvar(float64(sol.Qv[3])), sep, sqvar(float64(sol.Qv[4])), sep,
			sqvar(float64(sol.Qv[5])))
	}
	p += "\r\n"

	n := len(p) - len(*buff)
	*buff = p
	return n
}

/* output solution as the form of lat/lon/height -----------------------------*/
func (sol *Sol) OutSolPos(buff *string, s string, opt *SolOpt) int {
	var (
		pos, vel, dms1, dms2 [3]float64
		P, Q                 [9]float64
	)
	sep := opt2sep(opt)
	p := *buff

	Trace(4, "outpos  :\n")

	Ecef2Pos(sol.Rr[:], pos[:])
	sol.Sol2Cov(P[:])
	Cov2Enu(pos[:], P[:], Q[:])
	if opt.Height == 1 { /* geodetic height */
		pos[2] -= GeoidH(pos[:])
	}
	if opt.DegF > 0 {
		Deg2Dms(pos[0]*R2D, dms1[:], 5)
		Deg2Dms(pos[1]*R2D, dms2[:], 5)
		p += fmt.Sprintf("%s%s%4.0f%s%02.0f%s%08.5f%s%4.0f%s%02.0f%s%08.5f", s, sep,
			dms1[0], sep, dms1[1], sep, dms1[2], sep, dms2[0], sep, dms2[1], sep,
			dms2[2])
	} else {
		p += fmt.Sprintf("%s%s%14.9f%s%14.9f", s, sep, pos[0]*R2D, sep, pos[1]*R2D)
	}
	p += fmt.Sprintf("%s%10.4f%s%3d%s%3d%s%8.4f%s%8.4f%s%8.4f%s%8.4f%s%8.4f%s%8.4f%s%6.2f%s%6.1f",
		sep, pos[2], sep, sol.Stat, sep, sol.Ns, sep, SQRT(Q[4]), sep,
		SQRT(Q[0]), sep, SQRT(Q[8]), sep, sqvar(Q[1]), sep, sqvar(Q[2]),
		sep, sqvar(Q[5]), sep, sol.Age, sep, sol.Ratio)

	if opt.OutVel > 0 { /* output velocity */
		sol.Sol2CovarianceVel(P[:])
		Ecef2Enu(pos[:], sol.Rr[3:], vel[:])
		Cov2Enu(pos[:], P[:], Q[:])
		p += fmt.Sprintf("%s%10.5f%s%10.5f%s%10.5f%s%9.5f%s%8.5f%s%8.5f%s%8.5f%s%8.5f%s%8.5f",
			sep, vel[1], sep, vel[0], sep, vel[2], sep, SQRT(Q[4]), sep,
			SQRT(Q[0]), sep, SQRT(Q[8]), sep, sqvar(Q[1]), sep, sqvar(Q[2]),
			sep, sqvar(Q[5]))
	}
	p += "\r\n"
	n := len(p) - len(*buff)
	*buff = p
	return n
}

/* output solution as the form of e/n/u-baseline -----------------------------*/
func (sol *Sol) OutSolEnu(buff *string, s string, rb []float64, opt *SolOpt) int {
	var (
		pos, rr, enu [3]float64
		P, Q         [9]float64
		sep          = opt2sep(opt)
	)
	p := *buff

	Trace(4, "outenu  :\n")

	for i := 0; i < 3; i++ {
		rr[i] = sol.Rr[i] - rb[i]
	}
	Ecef2Pos(rb, pos[:])
	sol.Sol2Cov(P[:])
	Cov2Enu(pos[:], P[:], Q[:])
	Ecef2Enu(pos[:], rr[:], enu[:])
	p += fmt.Sprintf("%s%s%14.4f%s%14.4f%s%14.4f%s%3d%s%3d%s%8.4f%s%8.4f%s%8.4f%s%8.4f%s%8.4f%s%8.4f%s%6.2f%s%6.1f\r\n",
		s, sep, enu[0], sep, enu[1], sep, enu[2], sep, sol.Stat, sep, sol.Ns, sep,
		SQRT(Q[0]), sep, SQRT(Q[4]), sep, SQRT(Q[8]), sep, sqvar(Q[1]),
		sep, sqvar(Q[5]), sep, sqvar(Q[2]), sep, sol.Age, sep, sol.Ratio)
	n := len(p) - len(*buff)
	*buff = p
	return n
}

/* output solution in the form of NMEA RMC sentence --------------------------*/
func (sol *Sol) OutSolNmeaRmc(buff *string) int {
	dirp := 0.0
	var (
		time                  Gtime
		ep                    [6]float64
		pos, enuv, dms1, dms2 [3]float64
		vel, dir, amag        float64
		i                     int
		sum                   uint8
		emag, mode, status    string = "E", "A", "V"
	)
	p := *buff

	Trace(4, "outnmea_rmc:\n")

	if sol.Stat <= SOLQ_NONE {
		p += fmt.Sprintf("$%sRMC,,,,,,,,,,,,,", NMEA_TID)
		for i = 1; i < len(p); i++ {
			sum ^= p[i]
		}

		p += fmt.Sprintf("*%02X%c%c", sum, 0x0D, 0x0A)
		n := len(p) - len(*buff)
		*buff = p
		return n
	}
	time = GpsT2Utc(sol.Time)
	if time.Sec >= 0.995 {
		time.Time++
		time.Sec = 0.0
	}
	Time2Epoch(time, ep[:])
	Ecef2Pos(sol.Rr[:], pos[:])
	Ecef2Enu(pos[:], sol.Rr[3:], enuv[:])
	vel = Norm(enuv[:], 3)
	if vel >= 1.0 {
		dir = math.Atan2(enuv[0], enuv[1]) * R2D
		if dir < 0.0 {
			dir += 360.0
		}
		dirp = dir
	} else {
		dir = dirp
	}
	switch sol.Stat {
	case SOLQ_DGPS, SOLQ_SBAS:
		mode = "D"
	case SOLQ_FLOAT, SOLQ_FIX:
		mode = "R"
	case SOLQ_PPP:
		mode = "P"
	}
	Deg2Dms(math.Abs(pos[0])*R2D, dms1[:], 7)
	Deg2Dms(math.Abs(pos[1])*R2D, dms2[:], 7)
	var pos1, pos2 string
	if pos[0] >= 0 {
		pos1 = "N"
	} else {
		pos1 = "S"
	}
	if pos[1] >= 0 {
		pos2 = "E"
	} else {
		pos2 = "W"
	}
	p += fmt.Sprintf("$%sRMC,%02.0f%02.0f%05.2f,A,%02.0f%010.7f,%s,%03.0f%010.7f,%s,%4.2f,%4.2f,%02.0f%02.0f%02d,%.1f,%s,%s,%s",
		NMEA_TID, ep[3], ep[4], ep[5], dms1[0], dms1[1]+dms1[2]/60.0,
		pos1, dms2[0], dms2[1]+dms2[2]/60.0, pos2,
		vel/KNOT2M, dir, ep[2], ep[1], int(math.Mod(ep[0], 100.0)), amag, emag, mode, status)
	for i = 1; i < len(p); i++ {
		sum ^= p[i]
	}

	p += fmt.Sprintf("*%02X\r\n", sum)
	n := len(p) - len(*buff)
	*buff = p
	return n
}

/* output solution in the form of NMEA GGA sentence --------------------------*/
func (sol *Sol) OutSolNmeaGga(buff *string) int {
	var (
		time            Gtime
		h, dop          float64 = 0.0, 1.0
		ep              [6]float64
		pos, dms1, dms2 [3]float64
		solq, refid     int = 0, 0
		sum             uint8
		i               int
	)
	p := *buff

	Trace(4, "outnmea_gga:\n")

	if sol.Stat <= SOLQ_NONE {
		p += fmt.Sprintf("$%sGGA,,,,,,,,,,,,,,", NMEA_TID)
		for i = 1; i < len(p); i++ {
			sum ^= p[i]
		}
		p += fmt.Sprintf("*%02X%c%c", sum, 0x0D, 0x0A)
		n := len(p) - len(*buff)
		*buff = p
		return n
	}
	for solq = 0; solq < 8; solq++ {
		if nmea_solq[solq] == int(sol.Stat) {
			break
		}
	}
	if solq >= 8 {
		solq = 0
	}
	time = GpsT2Utc(sol.Time)
	if time.Sec >= 0.995 {
		time.Time++
		time.Sec = 0.0
	}
	Time2Epoch(time, ep[:])
	Ecef2Pos(sol.Rr[:], pos[:])
	h = GeoidH(pos[:])
	Deg2Dms(math.Abs(pos[0])*R2D, dms1[:], 7)
	Deg2Dms(math.Abs(pos[1])*R2D, dms2[:], 7)
	var pos1, pos2 string
	if pos[0] >= 0 {
		pos1 = "N"
	} else {
		pos1 = "S"
	}
	if pos[1] >= 0 {
		pos2 = "E"
	} else {
		pos2 = "W"
	}
	p += fmt.Sprintf("$%sGGA,%02.0f%02.0f%05.2f,%02.0f%010.7f,%s,%03.0f%010.7f,%s,%d,%02d,%.1f,%.3f,M,%.3f,M,%.1f,%04d",
		NMEA_TID, ep[3], ep[4], ep[5], dms1[0], dms1[1]+dms1[2]/60.0,
		pos1, dms2[0], dms2[1]+dms2[2]/60.0, pos2,
		solq, sol.Ns, dop, pos[2]-h, h, sol.Age, refid)
	for i = 1; i < len(p); i++ {
		sum ^= p[i]
	}
	p += fmt.Sprintf("*%02X\r\n", sum)
	n := len(p) - len(*buff)
	*buff = p
	return n
}

/* output solution in the form of NMEA GSA sentences -------------------------*/
func (sol *Sol) OutSolNmeaGsa(buff *string, ssat []SSat) int {
	var (
		azel                          []float64 = make([]float64, MAXSAT*2)
		dop                           [4]float64
		sum                           uint8
		j, sys, prn, nsat, mask, nsys int
		sats                          [MAXSAT]int
	)
	p := *buff

	Trace(4, "outnmea_gsa:\n")

	for i := 0; i < MAXSAT; i++ {
		if ssat[i].Vs == 0 {
			continue
		}
		sys = SatSys(i+1, nil)
		if sys&mask == 0 {
			nsys++
		} /* # of systems */
		mask |= sys
		azel[2*nsat] = float64(ssat[i].Azel[0])
		azel[2*nsat+1] = float64(ssat[i].Azel[1])
		sats[nsat] = i + 1
		nsat++
	}
	DOPs(nsat, azel, 0.0, dop[:])

	for i := 0; i < len(nmea_sys) && nmea_sys[i] > 0; i++ {
		for j, nsat = 0, 0; j < MAXSAT && nsat < 12; j++ {
			if SatSys(j+1, nil)&nmea_sys[i] == 0 {
				continue
			}
			if ssat[j].Vs > 0 {
				sats[nsat] = j + 1
				nsat++
			}
		}
		if nsat > 0 {

			var s1 string
			if nsys > 1 {
				s1 = "GN"
			} else {
				s1 = nmea_tid[i]
			}
			var d1 int
			if sol.Stat > 0 {
				d1 = 3
			} else {
				d1 = 1
			}
			p += fmt.Sprintf("$%sGSA,A,%d", s1, d1)
			for j = 0; j < 12; j++ {
				sys = SatSys(sats[j], &prn)
				switch sys {
				case SYS_SBS:
					prn -= 87 /* SBS: 33-64 */
				case SYS_GLO:
					prn += 64 /* GLO: 65-99 */
				case SYS_QZS:
					prn -= 192
				} /* QZS: 01-10 */
				if j < nsat {
					p += fmt.Sprintf(",%02d", prn)
				} else {
					p += ","
				}
			}
			p += fmt.Sprintf(",%3.1f,%3.1f,%3.1f,%d", dop[1], dop[2], dop[3],
				nmea_sid[i])
			for k := 1; k < len(p); k++ {
				sum ^= p[k]
			}
			p += fmt.Sprintf("*%02X\r\n", sum)
		}
	}
	n := len(p) - len(*buff)
	*buff = p
	return n
}

/* output solution in the form of NMEA GSV sentences -------------------------*/
func (sol *Sol) OutSolNmeaGsv(buff *string, ssat []SSat) int {
	var (
		az, el, snr                      float64
		i, j, k, n, nsat, nmsg, prn, sys int
		sats                             [MAXSAT]int
		sum                              uint8
	)
	p := *buff

	Trace(4, "outnmea_gsv:\n")

	for i = 0; i < len(nmea_sys) && nmea_sys[i] > 0; i++ {
		for j, nsat = 0, 0; j < MAXSAT && nsat < 36; j++ {
			if SatSys(j+1, nil)&nmea_sys[i] == 0 {
				continue
			}
			if ssat[j].Azel[1] > 0.0 {
				sats[nsat] = j + 1
				nsat++
			}
		}
		nmsg = (nsat + 3) / 4

		for j, n = 0, 0; j < nmsg; j++ {

			p += fmt.Sprintf("$%sGSV,%d,%d,%02d", nmea_tid[i], nmsg, j+1, nsat)
			for k = 0; k < 4; k++ {
				if n < nsat {
					sys = SatSys(sats[n], &prn)
					switch sys {
					case SYS_SBS:
						prn -= 87 /* SBS: 33-64 */
					case SYS_GLO:
						prn += 64 /* GLO: 65-99 */
					case SYS_QZS:
						prn -= 192
					} /* QZS: 01-10 */
					az = float64(ssat[sats[n]-1].Azel[0]) * R2D
					if az < 0.0 {
						az += 360.0
					}
					el = float64(ssat[sats[n]-1].Azel[1]) * R2D
					snr = float64(float32(ssat[sats[n]-1].Snr[0]) * SNR_UNIT)
					p += fmt.Sprintf(",%02d,%02.0f,%03.0f,%02.0f", prn, el, az, snr)
				} else {
					p += ",,,,"
				}
				n++
			}
			p += ",0" /* all signals */
			for k := 1; k < len(p); k++ {
				sum ^= p[k]
			}
			p += fmt.Sprintf("*%02X\r\n", sum)
		}
	}
	n = len(p) - len(*buff)
	*buff = p
	return n
}

/* output processing options ---------------------------------------------------
* output processing options to buffer
* args   : uint8_t *buff    IO  output buffer
*          prcopt_t *opt    I   processing options
* return : number of output bytes
*-----------------------------------------------------------------------------*/
func OutPrcOpts(buff *string, opt *PrcOpt) int {
	var (
		sys []int = []int{
			SYS_GPS, SYS_GLO, SYS_GAL, SYS_QZS, SYS_CMP, SYS_IRN, SYS_SBS, 0}
		s1 []string = []string{
			"Single", "DGPS", "Kinematic", "Static", "Moving-Base", "Fixed",
			"PPP Kinematic", "PPP Static", "PPP Fixed", "", "", ""}
		s2 []string = []string{
			"L1", "L1+2", "L1+2+3", "L1+2+3+4", "L1+2+3+4+5", "L1+2+3+4+5+6", "", "", ""}
		s3 []string = []string{
			"Forward", "Backward", "Combined", "", "", ""}
		s4 []string = []string{
			"OFF", "Broadcast", "SBAS", "Iono-Free LC", "Estimate TEC", "IONEX TEC",
			"QZSS Broadcast", "", "", "", ""}
		s5 []string = []string{
			"OFF", "Saastamoinen", "SBAS", "Estimate ZTD", "Estimate ZTD+Grad", "", "", ""}
		s6 []string = []string{
			"Broadcast", "Precise", "Broadcast+SBAS", "Broadcast+SSR APC",
			"Broadcast+SSR CoM", "", "", ""}
		s7 []string = []string{
			"GPS", "GLONASS", "Galileo", "QZSS", "BDS", "NavIC", "SBAS", "", "", ""}
		s8 []string = []string{
			"OFF", "Continuous", "Instantaneous", "Fix and Hold", "", "", ""}
		s9 []string = []string{
			"OFF", "ON", "", "", ""}
		i int
	)
	p := *buff

	Trace(4, "outprcopts:\n")

	p += fmt.Sprintf("%s pos mode  : %s\r\n", COMMENTH, s1[opt.Mode])

	if PMODE_DGPS <= opt.Mode && opt.Mode <= PMODE_FIXED {
		p += fmt.Sprintf("%s freqs     : %s\r\n", COMMENTH, s2[opt.Nf-1])
	}
	if opt.Mode > PMODE_SINGLE {
		p += fmt.Sprintf("%s solution  : %s\r\n", COMMENTH, s3[opt.SolType])
	}
	p += fmt.Sprintf("%s elev mask : %.1f deg\r\n", COMMENTH, opt.Elmin*R2D)
	if opt.Mode > PMODE_SINGLE {
		var s string
		if opt.Dynamics > 0 {
			s = "on"
		} else {
			s = "off"
		}
		p += fmt.Sprintf("%s dynamics  : %s\r\n", COMMENTH, s)
		if opt.TideCorr > 0 {
			s = "on"
		} else {
			s = "off"
		}
		p += fmt.Sprintf("%s tidecorr  : %s\r\n", COMMENTH, s)
	}
	if opt.Mode <= PMODE_FIXED {
		p += fmt.Sprintf("%s ionos opt : %s\r\n", COMMENTH, s4[opt.IonoOpt])
	}
	p += fmt.Sprintf("%s tropo opt : %s\r\n", COMMENTH, s5[opt.TropOpt])
	p += fmt.Sprintf("%s ephemeris : %s\r\n", COMMENTH, s6[opt.SatEph])
	p += fmt.Sprintf("%s navi sys  :", COMMENTH)
	for i = 0; sys[i] > 0; i++ {
		if opt.NavSys&sys[i] > 0 {
			p += fmt.Sprintf(" %s", s7[i])
		}
	}
	p += "\r\n"
	if PMODE_KINEMA <= opt.Mode && opt.Mode <= PMODE_FIXED {
		p += fmt.Sprintf("%s amb res   : %s\r\n", COMMENTH, s8[opt.ModeAr])
		if opt.NavSys&SYS_GLO > 0 {
			p += fmt.Sprintf("%s amb glo   : %s\r\n", COMMENTH, s9[opt.GloModeAr])
		}
		if opt.ThresAr[0] > 0.0 {
			p += fmt.Sprintf("%s val thres : %.1f\r\n", COMMENTH, opt.ThresAr[0])
		}
	}
	if opt.Mode == PMODE_MOVEB && opt.Baseline[0] > 0.0 {
		p += fmt.Sprintf("%s baseline  : %.4f %.4f m\r\n", COMMENTH,
			opt.Baseline[0], opt.Baseline[1])
	}
	for i = 0; i < 2; i++ {
		if opt.Mode == PMODE_SINGLE || (i >= 1 && opt.Mode > PMODE_FIXED) {
			continue
		}
		p += fmt.Sprintf("%s antenna%d  : %-21s (%7.4f %7.4f %7.4f)\r\n", COMMENTH,
			i+1, opt.AntType[i], opt.AntDel[i][0], opt.AntDel[i][1],
			opt.AntDel[i][2])
	}
	n := len(p) - len(*buff)
	*buff = p
	return n
}

/* output solution header ------------------------------------------------------
* output solution header to buffer
* args   : uint8_t *buff    IO  output buffer
*          solopt_t *opt    I   solution options
* return : number of output bytes
*-----------------------------------------------------------------------------*/
func OutSolHeader(buff *string, opt *SolOpt) int {
	var (
		s1    []string = []string{"WGS84", "Tokyo"}
		s2    []string = []string{"ellipsoidal", "geodetic"}
		s3    []string = []string{"GPST", "UTC ", "JST "}
		timeu int
		leg1  string = "Q=1:fix,2:float,3:sbas,4:dgps,5:single,6:ppp"
		leg2  string = "ns=# of satellites"
	)
	p := *buff
	sep := opt2sep(opt)
	if opt.TimeU < 0 {
		timeu = 0
	} else {
		if opt.TimeU > 20 {
			timeu = 20
		} else {
			timeu = opt.TimeU
		}
	}

	Trace(4, "outsolheads:\n")

	if opt.Posf == SOLF_NMEA || opt.Posf == SOLF_STAT || opt.Posf == SOLF_GSIF {
		return 0
	}
	if opt.OutHead > 0 {
		p += fmt.Sprintf("%s (", COMMENTH)
		switch opt.Posf {
		case SOLF_XYZ:
			p += "x/y/z-ecef=WGS84"
		case SOLF_ENU:
			p += "e/n/u-baseline=WGS84"
		default:
			p += fmt.Sprintf("lat/lon/height=%s/%s", s1[opt.Datum], s2[opt.Height])
		}
		p += fmt.Sprintf(",%s,%s)\r\n", leg1, leg2)
	}
	var tmf int = 8
	if opt.TimeF > 0 {
		tmf = 16
	}
	p += fmt.Sprintf("%s  %-*s%s", COMMENTH, tmf+timeu+1, s3[opt.TimeS], sep)

	switch opt.Posf {
	case SOLF_LLH: /* lat/lon/hgt */
		if opt.DegF > 0 {
			p += fmt.Sprintf("%16s%s%16s%s%10s%s%3s%s%3s%s%8s%s%8s%s%8s%s%8s%s%8s%s%8s%s%6s%s%6s",
				"latitude(d'\")", sep, "longitude(d'\")", sep, "height(m)",
				sep, "Q", sep, "ns", sep, "sdn(m)", sep, "sde(m)", sep, "sdu(m)",
				sep, "sdne(m)", sep, "sdeu(m)", sep, "sdue(m)", sep, "age(s)",
				sep, "ratio")
		} else {
			p += fmt.Sprintf("%14s%s%14s%s%10s%s%3s%s%3s%s%8s%s%8s%s%8s%s%8s%s%8s%s%8s%s%6s%s%6s",
				"latitude(deg)", sep, "longitude(deg)", sep, "height(m)", sep,
				"Q", sep, "ns", sep, "sdn(m)", sep, "sde(m)", sep, "sdu(m)", sep,
				"sdne(m)", sep, "sdeu(m)", sep, "sdun(m)", sep, "age(s)", sep,
				"ratio")
		}
		if opt.OutVel > 0 {
			p += fmt.Sprintf("%s%10s%s%10s%s%10s%s%9s%s%8s%s%8s%s%8s%s%8s%s%8s",
				sep, "vn(m/s)", sep, "ve(m/s)", sep, "vu(m/s)", sep, "sdvn", sep,
				"sdve", sep, "sdvu", sep, "sdvne", sep, "sdveu", sep, "sdvun")
		}
	case SOLF_XYZ: /* x/y/z-ecef */
		p += fmt.Sprintf("%14s%s%14s%s%14s%s%3s%s%3s%s%8s%s%8s%s%8s%s%8s%s%8s%s%8s%s%6s%s%6s",
			"x-ecef(m)", sep, "y-ecef(m)", sep, "z-ecef(m)", sep, "Q", sep, "ns",
			sep, "sdx(m)", sep, "sdy(m)", sep, "sdz(m)", sep, "sdxy(m)", sep,
			"sdyz(m)", sep, "sdzx(m)", sep, "age(s)", sep, "ratio")

		if opt.OutVel > 0 {
			p += fmt.Sprintf("%s%10s%s%10s%s%10s%s%9s%s%8s%s%8s%s%8s%s%8s%s%8s",
				sep, "vx(m/s)", sep, "vy(m/s)", sep, "vz(m/s)", sep, "sdvx", sep,
				"sdvy", sep, "sdvz", sep, "sdvxy", sep, "sdvyz", sep, "sdvzx")
		}
	case SOLF_ENU: /* e/n/u-baseline */
		p += fmt.Sprintf("%14s%s%14s%s%14s%s%3s%s%3s%s%8s%s%8s%s%8s%s%8s%s%8s%s%8s%s%6s%s%6s",
			"e-baseline(m)", sep, "n-baseline(m)", sep, "u-baseline(m)", sep,
			"Q", sep, "ns", sep, "sde(m)", sep, "sdn(m)", sep, "sdu(m)", sep,
			"sden(m)", sep, "sdnu(m)", sep, "sdue(m)", sep, "age(s)", sep,
			"ratio")
	}
	p += "\r\n"
	n := len(p) - len(*buff)
	*buff = p
	return n
}

/* std-dev of soltuion -------------------------------------------------------*/
func (sol *Sol) SolStd() float64 {
	/* approximate as max std-dev of 3-axis std-devs */
	if sol.Qr[0] > sol.Qr[1] && sol.Qr[0] > sol.Qr[2] {
		return float64(SQRT32(sol.Qr[0]))
	}
	if sol.Qr[1] > sol.Qr[2] {
		return float64(SQRT32(sol.Qr[1]))
	}
	return float64(SQRT32(sol.Qr[2]))
}

/* output solution body --------------------------------------------------------
* output solution body to buffer
* args   : uint8_t *buff    IO  output buffer
*          sol_t  *sol      I   solution
*          double *rb       I   base station position {x,y,z} (ecef) (m)
*          solopt_t *opt    I   solution options
* return : number of output bytes
*-----------------------------------------------------------------------------*/
func (sol *Sol) OutSols(buff *string, rb []float64, opt *SolOpt) int {
	var (
		time, ts    Gtime
		gpst        float64
		week, timeu int
		s, p        string
	)
	sep := opt2sep(opt)
	if buff != nil {
		p = *buff
	}

	Trace(4, "outsols :\n")

	/* suppress output if std is over opt.maxsolstd */
	if opt.MaxSolStd > 0.0 && sol.SolStd() > opt.MaxSolStd {
		return 0
	}
	if opt.Posf == SOLF_NMEA {
		if opt.NmeaIntv[0] < 0.0 {
			return 0
		}
		if ScreenTime(sol.Time, ts, ts, opt.NmeaIntv[0]) == 0 {
			return 0
		}
	}
	if sol.Stat <= SOLQ_NONE || (opt.Posf == SOLF_ENU && Norm(rb, 3) <= 0.0) {
		return 0
	}
	if opt.TimeU < 0 {
		timeu = 0
	} else {
		if opt.TimeU > 20 {
			timeu = 20
		} else {
			timeu = opt.TimeU
		}
	}

	time = sol.Time
	if opt.TimeS >= TIMES_UTC {
		time = GpsT2Utc(time)
	}
	if opt.TimeS == TIMES_JST {
		time = TimeAdd(time, 9*3600.0)
	}

	if opt.TimeF > 0 {
		Time2Str(time, &s, timeu)
	} else {
		gpst = Time2GpsT(time, &week)
		if 86400*7-gpst < 0.5/math.Pow(10.0, float64(timeu)) {
			week++
			gpst = 0.0
		}
		var tu int = timeu + 1
		if timeu <= 0 {
			tu = 0
		}
		s = fmt.Sprintf("%4d%.16s%*.*f", week, sep, 6+tu, timeu, gpst)
	}
	switch byte(opt.Posf) {
	case SOLF_LLH:
		sol.OutSolPos(&p, s, opt)

	case SOLF_XYZ:
		OutEcef(&p, s, sol, opt)

	case SOLF_ENU:
		sol.OutSolEnu(&p, s, rb, opt)

	case SOLF_NMEA:
		sol.OutSolNmeaRmc(&p)
		sol.OutSolNmeaGga(&p)

	}
	n := len(p) - len(*buff)
	*buff = p
	return n
}

/* output solution extended ----------------------------------------------------
* output solution exteneded infomation
* args   : uint8_t *buff    IO  output buffer
*          sol_t  *sol      I   solution
*          ssat_t *ssat     I   satellite status
*          solopt_t *opt    I   solution options
* return : number of output bytes
* notes  : only support nmea
*-----------------------------------------------------------------------------*/
func (sol *Sol) OutSolExs(buff *string, ssat []SSat, opt *SolOpt) int {
	var ts Gtime
	p := *buff

	Trace(4, "outsolexs:\n")

	/* suppress output if std is over opt.maxsolstd */
	if opt.MaxSolStd > 0.0 && sol.SolStd() > opt.MaxSolStd {
		return 0
	}
	if opt.Posf == SOLF_NMEA {
		if opt.NmeaIntv[1] < 0.0 {
			return 0
		}
		if ScreenTime(sol.Time, ts, ts, opt.NmeaIntv[1]) == 0 {
			return 0
		}
	}
	if opt.Posf == SOLF_NMEA {
		sol.OutSolNmeaGsa(&p, ssat)
		sol.OutSolNmeaGsv(&p, ssat)
	}
	n := len(p) - len(*buff)
	*buff = p
	return n
}

/* output processing option ----------------------------------------------------
* output processing option to file
* args   : FILE   *fp       I   output file pointer
*          prcopt_t *opt    I   processing options
* return : none
*-----------------------------------------------------------------------------*/
func OutPrcOpt(fp *os.File, opt *PrcOpt) {
	var buff string

	Trace(4, "outprcopt:\n")

	if n := OutPrcOpts(&buff, opt); n > 0 {
		fp.WriteString(buff)
	}
}

/* output solution header ------------------------------------------------------
* output solution heade to file
* args   : FILE   *fp       I   output file pointer
*          solopt_t *opt    I   solution options
* return : none
*-----------------------------------------------------------------------------*/
func OutSolHead(fp *os.File, opt *SolOpt) {
	var buff string

	Trace(4, "outsolhead:\n")

	if n := OutSolHeader(&buff, opt); n > 0 {
		fp.WriteString(buff)
	}
}

/* output solution body --------------------------------------------------------
* output solution body to file
* args   : FILE   *fp       I   output file pointer
*          sol_t  *sol      I   solution
*          double *rb       I   base station position {x,y,z} (ecef) (m)
*          solopt_t *opt    I   solution options
* return : none
*-----------------------------------------------------------------------------*/
func (sol *Sol) OutSol(fp *os.File, rb []float64, opt *SolOpt) {
	var buff string

	Trace(4, "outsol  :\n")

	if n := sol.OutSols(&buff, rb, opt); n > 0 {
		fp.WriteString(buff)
	}
}

/* output solution extended ----------------------------------------------------
* output solution exteneded infomation to file
* args   : FILE   *fp       I   output file pointer
*          sol_t  *sol      I   solution
*          ssat_t *ssat     I   satellite status
*          solopt_t *opt    I   solution options
* return : output size (bytes)
* notes  : only support nmea
*-----------------------------------------------------------------------------*/
func (sol *Sol) OutSolex(fp *os.File, ssat []SSat, opt *SolOpt) {
	var buff string

	Trace(4, "outsolex:\n")

	if n := sol.OutSolExs(&buff, ssat, opt); n > 0 {
		fp.WriteString(buff)
	}
}
