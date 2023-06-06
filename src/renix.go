/*------------------------------------------------------------------------------
* rinex.c : RINEX functions
*
*          Copyright (C) 2007-2020 by T.TAKASU, All rights reserved.
*
* reference :
*     [1] W.Gurtner and L.Estey, RINEX The Receiver Independent Exchange Format
*         Version 2.11, December 10, 2007
*     [2] W.Gurtner and L.Estey, RINEX The Receiver Independent Exchange Format
*         Version 3.00, November 28, 2007
*     [3] IS-GPS-200D, Navstar GPS Space Segment/Navigation User Interfaces,
*         7 March, 2006
*     [4] W.Gurtner and L.Estey, RINEX The Receiver Independent Exchange Format
*         Version 2.12, June 23, 2009
*     [5] W.Gurtner and L.Estey, RINEX The Receiver Independent Exchange Format
*         Version 3.01, June 22, 2009
*     [6] J.Ray and W.Gurtner, RINEX extentions to handle clock information
*         version 3.02, September 2, 2010
*     [7] RINEX The Receiver Independent Exchange Format Version 3.02,
*         International GNSS Service (IGS), RINEX Working Group and Radio
*         Technical Commission for Maritime Services Special Committee 104
*         (RTCM-SC104), December 10, 2012
*     [8] RINEX The Receiver Independent Exchange Format Version 3.03,
*         International GNSS Service (IGS), RINEX Working Group and Radio
*         Technical Commission for Maritime Services Special Committee 104
*         (RTCM-SC104), July 14, 2015
*     [9] RINEX The Receiver Independent Exchange Format Version 3.04,
*         International GNSS Service (IGS), RINEX Working Group and Radio
*         Technical Commission for Maritime Services Special Committee 104
*         (RTCM-SC104), November 23, 2018
*
* version : $Revision:$
* history : 2006/01/16 1.0  new
*           2007/03/14 1.1  read P1 if no obstype of C1
*           2007/04/27 1.2  add readrnxt() function
*           2007/05/25 1.3  add support of file path with wild-card (*)
*                           add support of compressed files
*           2007/11/02 1.4  support sbas/geo satellite
*                           support doppler observables
*                           support rinex bug of week handover
*                           add rinex obs/nav output functions
*           2008/06/16 1.5  export readrnxf(), add compress()
*                           separate sortobs(), uniqeph(), screent()
*           2008/10/28 1.6  fix bug on reading rinex obs header types of observ
*           2009/04/09 1.7  support rinex 2.11
*                           change api of outrnxobsh(),outrnxobsb(),outrnxnavb()
*           2009/06/02 1.8  add api outrnxgnavb()
*           2009/08/15 1.9  support glonass
*                           add slip save/restore functions
*           2010/03/03 1.10 fix bug of array access by disabled satellite
*           2010/07/21 1.11 support rinex ver.2.12, 3.00
*                           support rinex extension for qzss
*                           support geo navigation messages
*                           added api:
*                               setrnxcodepri(),outrnxhnavh(),outrnxhnavb(),
*                           changed api:
*                               readrnx(),readrnxt(),outrnxnavh(),outrnxgnavh()
*           2010/05/29 1.12 fix bug on skipping invalid satellite data
*                           fix bug on frequency number overflow
*                           output P1 instead of C1 if rnxopt.rcvopt=-L1P
*                           output C2 instead of P2 if rnxopt.rcvopt=-L2C
*                           change api:
*                               outrnxgnavh(),outrnxhnavh(),readrnx(),
*                               readrnxt()
*                           add api:
*                               outrnxlnavh(), outrnxqnav()
*                           move uniqeph(),uniqgeph,uniqseph()
*           2010/08/19 1.13 suppress warning
*           2012/03/01 1.14 add function to read cnes widelane fcb in rnxclk
*                           support compass rinex nav
*                           change api: setcodepri()
*           2012/10/17 1.15 support ver.2.12, ver.3.01
*                           add api init_rnxctr(),free_rnxctr(),open_rnxctr(),
*                           input_rnxctr()
*                           change api readrnxt(),readrnx()
*                           delete api setrnxcodepri()
*                           fix bug on message frame time in v.3 glonass nav
*           2013/02/09 1.16 add reading geph.iode derived from toe
*           2013/02/23 1.17 support rinex 3.02 (ref [7])
*                           change api outrnxobsh()
*                           add api outrnxcnavh()
*                           fix bug on output of fit interval
*           2013/05/08 1.18 fix bug on reading glo and geo nav in rinex 3
*           2013/09/01 1.19 fix bug on reading galileo "C1" in rinex 2.12
*           2013/12/16 1.20 reject C1 for 2.12
*           2014/05/26 1.21 fix bug on reading gps "C2" in rinex 2.11 or 2.12
*                           fix problem on type incompatibility
*                           support beidou
*           2014/08/29 1.22 fix bug on reading gps "C2" in rinex 2.11 or 2.12
*           2014/10/20 1.23 recognize "C2" in 2.12 as "C2W" instead of "C2D"
*           2014/12/07 1.24 add read rinex option -SYS=...
*           2016/07/01 1.25 support RINEX 3.03
*                           support IRNSS
*           2016/09/17 1.26 fix bug on fit interval in QZSS RINEX nav
*                           URA output value compliant to RINEX 3.03
*           2016/10/10 1.27 add api outrnxinavh()
*           2018/10/10 1.28 support galileo sisa value for rinex nav output
*                           fix bug on handling beidou B1 code in rinex 3.03
*           2019/08/19 1.29 support galileo sisa index for rinex nav input
*           2020/11/30 1.30 support RINEX 3.04 (ref [9])
*                           support phase shift in RINEX options rnxopt_t
*                           support high-resolution (16bit) C/N0 in obsd_t
*                           support dual sets of ephemerides in RINEX control
*                             (for Galileo I/NAV and F/NAV)
*                           no support RINEX 2 NAV extentions (QZS and BDS)
*                           no support CNES/GRG clock extension in comments
*                           fix bug on segfault to read NavIC/IRNSS OBS data
*                           fix bug on segfault with # obs data >= MAXOBS
*                           fix bug on reading/writing GLONASS slot/frq # lines
*                           fix bug on reading SBAS UTC parameters in RINEX nav
*                           fix bug on saving slip info in extended OBS slots
*                           add iono/utc param. in separated output RINEX NAV
*                           zero-padded satellite number (e.g. "G 1" . "G01")
*                           zero-padded month/date/hour/min/sec
*                           use exponent letter D instead of E for RINEX NAV
*                           use API code2idx() to get frequency index
*                           use intger types in stdint.h
*                           suppress warnings
*		    2022/05/31 1.0  rewrite renix.c with golang by fxb
*-----------------------------------------------------------------------------*/
package gnssgo

import (
	"bufio"
	"fmt"
	"io"
	"math"
	"os"
	"sort"
	"strings"
)

const (
	NAVEXP      = "D"                 /* exponent letter in RINEX NAV */
	NUMSYS      = 7                   /* number of systems */
	MAXRNXLEN   = (16*MAXOBSTYPE + 4) /* max RINEX record length */
	MAXPOSHEAD  = 1024                /* max head line position */
	MINFREQ_GLO = -7                  /* min frequency number GLONASS */
	MAXFREQ_GLO = 13                  /* max frequency number GLONASS */
	NINCOBS     = 262144              /* incremental number of obs data */
	syscodes    = "GREJSCI"           /* satellite system codes */
	obstype     = "CLDS"              /* observation type codes */
)

var ura_eph []float64 = []float64{ /* RAa values (ref [3] 20.3.3.3.1.1) */
	2.4, 3.4, 4.85, 6.85, 9.65, 13.65, 24.0, 48.0, 96.0, 192.0, 384.0, 768.0, 1536.0,
					3072.0, 6144.0, 0.0}
var ura_nominal []float64 = []float64{ /* URA nominal values */
	2.0, 2.8, 4.0, 5.7, 8.0, 11.3, 16.0, 32.0, 64.0, 128.0, 256.0, 512.0, 1024.0,
	2048.0, 4096.0, 8192.0}

/* type definition -----------------------------------------------------------*/
type Sigind struct { /* signal index type */
	n     int                 /* number of index */
	idx   [MAXOBSTYPE]int     /* signal freq-index */
	pos   [MAXOBSTYPE]int     /* signal index in obs data (-1:no) */
	pri   [MAXOBSTYPE]uint8   /* signal priority (15-0) */
	ctype [MAXOBSTYPE]uint8   /* ctype (0:C,1:L,2:D,3:S) */
	code  [MAXOBSTYPE]uint8   /* obs-code (CODE_L??) */
	shift [MAXOBSTYPE]float64 /* phase shift (cycle) */
}

/* set string without tail space ---------------------------------------------*/
func setstr(dst *string, src string, n int) {
	if len(src) < n {
		return
	}
	*dst = src[:n]
	*dst = strings.TrimRightFunc(*dst, func(r rune) bool {
		return r == ' '
	})
}

/* adjust time considering week handover -------------------------------------*/
func AdjWeek(t, t0 Gtime) Gtime {
	tt := TimeDiff(t, t0)
	if tt < -302400.0 {
		return TimeAdd(t, 604800.0)
	}
	if tt > 302400.0 {
		return TimeAdd(t, -604800.0)
	}
	return t
}

/* adjust time considering week handover -------------------------------------*/
func AdjDay(t, t0 Gtime) Gtime {
	tt := TimeDiff(t, t0)
	if tt < -43200.0 {
		return TimeAdd(t, 86400.0)
	}
	if tt > 43200.0 {
		return TimeAdd(t, -86400.0)
	}
	return t
}

/* time string for ver.3 (yyyymmdd hhmmss UTC) -------------------------------*/
func TimeStrRnx(str *string) {
	var ep [6]float64
	time := TimeGet()
	time.Sec = 0.0
	Time2Epoch(time, ep[:])
	*str = fmt.Sprintf("%04.0f%02.0f%02.0f %02.0f%02.0f%02.0f UTC", ep[0], ep[1], ep[2],
		ep[3], ep[4], ep[5])
}

/* satellite to satellite code -----------------------------------------------*/
func Sat2Code(sat int, code *string) int {
	var prn int
	switch SatSys(sat, &prn) {
	case SYS_GPS:
		*code = fmt.Sprintf("G%02d", prn-MINPRNGPS+1)
	case SYS_GLO:
		*code = fmt.Sprintf("R%02d", prn-MINPRNGLO+1)
	case SYS_GAL:
		*code = fmt.Sprintf("E%02d", prn-MINPRNGAL+1)
	case SYS_SBS:
		*code = fmt.Sprintf("S%02d", prn-100)
	case SYS_QZS:
		*code = fmt.Sprintf("J%02d", prn-MINPRNQZS+1)
	case SYS_CMP:
		*code = fmt.Sprintf("C%02d", prn-MINPRNCMP+1)
	case SYS_IRN:
		*code = fmt.Sprintf("I%02d", prn-MINPRNIRN+1)
	default:
		return 0
	}
	return 1
}

/* URA index to URA nominal value (m) ----------------------------------------*/
func UraValue(sva int) float64 {
	if 0 <= sva && sva < 15 {
		return ura_nominal[sva]
	} else {
		return 8192.0
	}
}

/* URA value (m) to URA index ------------------------------------------------*/
func UraIndex(value float64) int {
	var i int
	for i = 0; i < 15; i++ {
		if ura_eph[i] >= value {
			break
		}
	}
	return i
}

/* Galileo SISA index to SISA nominal value (m) ------------------------------*/
func SisaValue(sisa int) float64 {
	if sisa <= 49 {
		return float64(sisa) * 0.01
	}
	if sisa <= 74 {
		return 0.5 + float64(sisa-50)*0.02
	}
	if sisa <= 99 {
		return 1.0 + float64(sisa-75)*0.04
	}
	if sisa <= 125 {
		return 2.0 + float64(sisa-100)*0.16
	}
	return -1.0 /* unknown or NAPA */
}

/* Galileo SISA value (m) to SISA index --------------------------------------*/
func SisaIndex(value float64) int {
	switch {
	case value < 0.0 || value > 6.0:
		return 255 /* unknown or NAPA */
	case value <= 0.5:
		return int(value / 0.01)
	case value <= 1.0:
		return int((value-0.5)/0.02) + 50
	case value <= 2.0:
		return int((value-1.0)/0.04) + 75
	}
	return int((value-2.0)/0.16) + 100
}

/* initialize station parameter ----------------------------------------------*/
func (sta *Sta) InitSta() {
	if sta == nil {
		Trace(2, "sta is nil")
		return
	}
	sta.Name = ""
	sta.Marker = ""
	sta.AntDes = ""
	sta.AntSno = ""
	sta.Type = ""
	sta.RecVer = ""
	sta.RecSN = ""
	sta.AntSetup, sta.Itrf, sta.DelType = 0, 0, 0
	for i := 0; i < 3; i++ {
		sta.Pos[i] = 0.0
	}
	for i := 0; i < 3; i++ {
		sta.Del[i] = 0.0
	}
	sta.Hgt = 0.0
}

/*------------------------------------------------------------------------------
* input RINEX functions
*-----------------------------------------------------------------------------*/

/* convert RINEX obs-type ver.2 . ver.3 -------------------------------------*/
func ConvRinexCode2_3(ver float64, sys int, str string, ctype *string) {
	*ctype = "   "

	switch {
	case str == "P1": /* ver.2.11 GPS L1PY,GLO L2P */
		switch sys {
		case SYS_GPS:
			*ctype = fmt.Sprintf("%c1W", 'C')
		case SYS_GLO:
			*ctype = fmt.Sprintf("%c1P", 'C')
		}
	case str == "P2": /* ver.2.11 GPS L2PY,GLO L2P */
		switch sys {
		case SYS_GPS:
			*ctype = fmt.Sprintf("%c2W", 'C')
		case SYS_GLO:
			*ctype = fmt.Sprintf("%c2P", 'C')
		}
	case str == "C1": /* ver.2.11 GPS L1C,GLO L1C/A */
		switch {
		case ver >= 2.12: /* reject C1 for 2.12 */
		case sys == SYS_GPS:
			*ctype = fmt.Sprintf("%c1C", 'C')
		case sys == SYS_GLO:
			*ctype = fmt.Sprintf("%c1C", 'C')
		case sys == SYS_GAL:
			*ctype = fmt.Sprintf("%c1X", 'C') /* ver.2.12 */
		case sys == SYS_QZS:
			*ctype = fmt.Sprintf("%c1C", 'C')
		case sys == SYS_SBS:
			*ctype = fmt.Sprintf("%c1C", 'C')
		}
	case str == "C2":
		switch sys {
		case SYS_GPS:
			if ver >= 2.12 {
				*ctype = fmt.Sprintf("%c2W", 'C') /* L2P(Y) */
			} else {
				*ctype = fmt.Sprintf("%c2X", 'C') /* L2C */
			}
		case SYS_GLO:
			*ctype = fmt.Sprintf("%c2C", 'C')
		case SYS_QZS:
			*ctype = fmt.Sprintf("%c2X", 'C')
		case SYS_CMP:
			*ctype = fmt.Sprintf("%c2X", 'C') /* ver.2.12 B1_2 */
		}
	case ver >= 2.12 && str[1] == 'A': /* ver.2.12 L1C/A */
		switch sys {
		case SYS_GPS:
			*ctype = fmt.Sprintf("%c1C", str[0])
		case SYS_GLO:
			*ctype = fmt.Sprintf("%c1C", str[0])
		case SYS_QZS:
			*ctype = fmt.Sprintf("%c1C", str[0])
		case SYS_SBS:
			*ctype = fmt.Sprintf("%c1C", str[0])
		}
	case ver >= 2.12 && str[1] == 'B': /* ver.2.12 GPS L1C */
		switch sys {
		case SYS_GPS:
			*ctype = fmt.Sprintf("%c1X", str[0])
		case SYS_QZS:
			*ctype = fmt.Sprintf("%c1X", str[0])
		}
	case ver >= 2.12 && str[1] == 'C': /* ver.2.12 GPS L2C */
		switch sys {
		case SYS_GPS:
			*ctype = fmt.Sprintf("%c2X", str[0])
		case SYS_QZS:
			*ctype = fmt.Sprintf("%c2X", str[0])
		}
	case ver >= 2.12 && str[1] == 'D': /* ver.2.12 GLO L2C/A */
		if sys == SYS_GLO {
			*ctype = fmt.Sprintf("%c2C", str[0])
		}
	case ver >= 2.12 && str[1] == '1': /* ver.2.12 GPS L1PY,GLO L1P */
		switch sys {
		case SYS_GPS:
			*ctype = fmt.Sprintf("%c1W", str[0])
		case SYS_GLO:
			*ctype = fmt.Sprintf("%c1P", str[0])
		case SYS_GAL:
			*ctype = fmt.Sprintf("%c1X", str[0]) /* tentative */
		case SYS_CMP:
			*ctype = fmt.Sprintf("%c2X", str[0]) /* extension */
		}
	case ver < 2.12 && str[1] == '1':
		switch sys {
		case SYS_GPS:
			*ctype = fmt.Sprintf("%c1C", str[0])
		case SYS_GLO:
			*ctype = fmt.Sprintf("%c1C", str[0])
		case SYS_GAL:
			*ctype = fmt.Sprintf("%c1X", str[0]) /* tentative */
		case SYS_QZS:
			*ctype = fmt.Sprintf("%c1C", str[0])
		case SYS_SBS:
			*ctype = fmt.Sprintf("%c1C", str[0])
		}
	case str[1] == '2':
		switch sys {
		case SYS_GPS:
			*ctype = fmt.Sprintf("%c2W", str[0])
		case SYS_GLO:
			*ctype = fmt.Sprintf("%c2P", str[0])
		case SYS_QZS:
			*ctype = fmt.Sprintf("%c2X", str[0])
		case SYS_CMP:
			*ctype = fmt.Sprintf("%c2X", str[0]) /* ver.2.12 B1_2 */
		}
	case str[1] == '5':
		switch sys {
		case SYS_GPS:
			*ctype = fmt.Sprintf("%c5X", str[0])
		case SYS_GAL:
			*ctype = fmt.Sprintf("%c5X", str[0])
		case SYS_QZS:
			*ctype = fmt.Sprintf("%c5X", str[0])
		case SYS_SBS:
			*ctype = fmt.Sprintf("%c5X", str[0])
		}
	case str[1] == '6':
		switch sys {
		case SYS_GAL:
			*ctype = fmt.Sprintf("%c6X", str[0])
		case SYS_QZS:
			*ctype = fmt.Sprintf("%c6X", str[0])
		case SYS_CMP:
			*ctype = fmt.Sprintf("%c6X", str[0]) /* ver.2.12 B3 */
		}
	case str[1] == '7':
		switch sys {
		case SYS_GAL:
			*ctype = fmt.Sprintf("%c7X", str[0])
		case SYS_CMP:
			*ctype = fmt.Sprintf("%c7X", str[0]) /* ver.2.12 B2b */
		}
	case str[1] == '8':
		if sys == SYS_GAL {
			*ctype = fmt.Sprintf("%c8X", str[0])
		}
	}
	Trace(5, "convcode: ver=%.2f sys=%2d type= %s -> %s\n", ver, sys, str, *ctype)
}

/* decode RINEX observation data file header ---------------------------------*/
func DecodeObsHeader(rd *bufio.Reader, buff string, ver float64, tsys *int,
	tobs *TOBS, nav *Nav, sta *Sta) {
	/* default codes for unknown code */
	var (
		frqcodes string   = "1256789"
		defcodes []string = []string{
			"CWX    ", /* GPS: L125____ */
			"CCXX X ", /* GLO: L1234_6_ */
			"C XXXX ", /* GAL: L1_5678_ */
			"CXXX   ", /* QZS: L1256___ */
			"C X    ", /* SBS: L1_5____ */
			"XIXIIX ", /* BDS: L125678_ */
			"  A   A" /* IRN: L__5___9 */}
		del                             [3]float64
		i, j, k, n, nt, prn, fcn, index int
		str                             string
	)
	label := buff[60:]

	Trace(4, "decode_obsh: ver=%.2f\n", ver)

	switch {
	case strings.Contains(label, "MARKER NAME"):
		if sta != nil {
			setstr(&sta.Name, buff, 60)
		}
	case strings.Contains(label, "MARKER NUMBER"): /* opt */
		if sta != nil {
			setstr(&sta.Marker, buff, 20)
		}
	case strings.Contains(label, "MARKER TYPE"): /* ver.3 */
	case strings.Contains(label, "OBSERVER / AGENCY"):
	case strings.Contains(label, "REC # / TYPE / VERS"):
		if sta != nil {
			setstr(&sta.RecSN, buff, 20)
			setstr(&sta.Type, buff[20:], 20)
			setstr(&sta.RecVer, buff[40:], 20)
		}
	case strings.Contains(label, "ANT # / TYPE"):
		if sta != nil {
			setstr(&sta.AntSno, buff, 20)
			setstr(&sta.AntDes, buff[20:], 20)
		}
	case strings.Contains(label, "APPROX POSITION XYZ"):
		if sta != nil {
			for i, j = 0, 0; i < 3; i, j = i+1, j+14 {
				sta.Pos[i] = Str2Num(buff, j, 14)
			}
		}
	case strings.Contains(label, "ANTENNA: DELTA H/E/N"):
		if sta != nil {
			for i, j = 0, 0; i < 3; i, j = i+1, j+14 {
				del[i] = Str2Num(buff, j, 14)
			}
			sta.Del[2] = del[0] /* h */
			sta.Del[0] = del[1] /* e */
			sta.Del[1] = del[2] /* n */
		}
	case strings.Contains(label, "ANTENNA: DELTA X/Y/Z"): /* opt ver.3 */
	case strings.Contains(label, "ANTENNA: PHASECENTER"): /* opt ver.3 */
	case strings.Contains(label, "ANTENNA: B.SIGHT XYZ"): /* opt ver.3 */
	case strings.Contains(label, "ANTENNA: ZERODIR AZI"): /* opt ver.3 */
	case strings.Contains(label, "ANTENNA: ZERODIR XYZ"): /* opt ver.3 */
	case strings.Contains(label, "CENTER OF MASS: XYZ"): /* opt ver.3 */
	case strings.Contains(label, "SYS / # / OBS TYPES"): /* ver.3 */

		if index = strings.IndexRune(syscodes, rune(buff[0])); index < 0 {
			Trace(2, "invalid system code: sys=%c\n", buff[0])
			return
		}
		i = index
		n = int(Str2Num(buff, 3, 3))
		for j, nt, k = 0, 0, 7; j < n; j, k = j+1, k+4 {
			if k > 58 {
				buff, _ = rd.ReadString('\n')
				if len(buff) == 0 {
					break
				}
				k = 7
			}
			if nt < MAXOBSTYPE-1 {
				setstr(&tobs[i][nt], buff[k:], 3)
				nt++
			}
		}
		tobs[i][nt] = ""

		/* change BDS B1 code: 3.02 */
		if i == 5 && math.Abs(ver-3.02) < 1e-3 {
			for j = 0; j < nt; j++ {
				q := []rune(tobs[i][j])

				if q[1] == '1' {
					q[1] = '2'
				}
				tobs[i][j] = string(q)
			}
		}
		/* if unknown code in ver.3, set default code */
		for j = 0; j < nt; j++ {
			q := []rune(tobs[i][j])
			if len(q) > 1 {
				continue
			}
			if index = strings.IndexRune(frqcodes, q[1]); index < 0 {
				continue
			}
			q[2] = rune(defcodes[i][index])
			tobs[i][j] = string(q)
			Trace(2, "set default for unknown code: sys=%c code=%s\n", buff[0], tobs[i][j])
		}
	case strings.Contains(label, "WAVELENGTH FACT L1/2"): /* opt ver.2 */
	case strings.Contains(label, "# / TYPES OF OBSERV"): /* ver.2 */
		n = int(Str2Num(buff, 0, 6))
		for i, nt, j = 0, 0, 10; i < n; i, j = i+1, j+6 {
			if j > 58 {

				buff, _ = rd.ReadString('\n')
				if len(buff) == 0 {
					break
				}

				j = 10
			}
			if nt >= MAXOBSTYPE-1 {
				continue
			}
			if ver <= 2.99 {
				setstr(&str, buff[j:], 2)
				ConvRinexCode2_3(ver, SYS_GPS, str, &tobs[0][nt])
				ConvRinexCode2_3(ver, SYS_GLO, str, &tobs[1][nt])
				ConvRinexCode2_3(ver, SYS_GAL, str, &tobs[2][nt])
				ConvRinexCode2_3(ver, SYS_QZS, str, &tobs[3][nt])
				ConvRinexCode2_3(ver, SYS_SBS, str, &tobs[4][nt])
				ConvRinexCode2_3(ver, SYS_CMP, str, &tobs[5][nt])
			}
			nt++
		}
		tobs[0][nt] = ""
	case strings.Contains(label, "SIGNAL STRENGTH UNIT"): /* opt ver.3 */
	case strings.Contains(label, "INTERVAL"): /* opt */
	case strings.Contains(label, "TIME OF FIRST OBS"):
		switch {
		case strings.Contains(buff[48:51], "GPS"):
			*tsys = TSYS_GPS
		case strings.Contains(buff[48:51], "GLO"):
			*tsys = TSYS_UTC
		case strings.Contains(buff[48:51], "GAL"):
			*tsys = TSYS_GAL
		case strings.Contains(buff[48:51], "QZS"):
			*tsys = TSYS_QZS /* ver.3.02 */
		case strings.Contains(buff[48:51], "BDT"):
			*tsys = TSYS_CMP /* ver.3.02 */
		case strings.Contains(buff[48:51], "IRN"):
			*tsys = TSYS_IRN /* ver.3.03 */

		}
	case strings.Contains(label, "TIME OF LAST OBS"): /* opt */
	case strings.Contains(label, "RCV CLOCK OFFS APPL"): /* opt */
	case strings.Contains(label, "SYS / DCBS APPLIED"): /* opt ver.3 */
	case strings.Contains(label, "SYS / PCVS APPLIED"): /* opt ver.3 */
	case strings.Contains(label, "SYS / SCALE FACTOR"): /* opt ver.3 */
	case strings.Contains(label, "SYS / PHASE SHIFTS"): /* ver.3.01 */
	case strings.Contains(label, "GLONASS SLOT / FRQ #"): /* ver.3.02 */
		for i = 0; i < 8; i++ {
			if buff[4+i*7] != 'R' {
				continue
			}
			prn = int(Str2Num(buff, 5+i*7, 2))
			fcn = int(Str2Num(buff, 8+i*7, 2))
			if prn < 1 || prn > MAXPRNGLO || fcn < -7 || fcn > 6 {
				continue
			}
			if nav != nil {
				nav.Glo_fcn[prn-1] = fcn + 8
			}
		}
	case strings.Contains(label, "GLONASS COD/PHS/BIS"): /* ver.3.02 */
		if sta != nil {
			sta.glo_cp_bias[0] = Str2Num(buff, 5, 8)
			sta.glo_cp_bias[1] = Str2Num(buff, 18, 8)
			sta.glo_cp_bias[2] = Str2Num(buff, 31, 8)
			sta.glo_cp_bias[3] = Str2Num(buff, 44, 8)
		}
	case strings.Contains(label, "LEAP SECONDS"): /* opt */
		if nav != nil {
			nav.Utc_gps[4] = Str2Num(buff, 0, 6)
			nav.Utc_gps[7] = Str2Num(buff, 6, 6)
			nav.Utc_gps[5] = Str2Num(buff, 12, 6)
			nav.Utc_gps[6] = Str2Num(buff, 18, 6)
		}
	case strings.Contains(label, "# OF SALTELLITES"): /* opt */
		/* skip */
	case strings.Contains(label, "PRN / # OF OBS"): /* opt */
		/* skip */
	}
}

/* decode RINEX NAV header ---------------------------------------------------*/
func (nav *Nav) DecodeNavHeader(buff string) {
	var (
		i, j  int
		label string = buff[60:]
	)

	Trace(4, "decode_navh:\n")

	switch {
	case strings.Contains(label, "ION ALPHA"): /* opt ver.2 */
		if nav != nil {
			for i, j = 0, 2; i < 4; i, j = i+1, j+12 {
				nav.Ion_gps[i] = Str2Num(buff, j, 12)
			}
		}
	case strings.Contains(label, "ION BETA"): /* opt ver.2 */
		if nav != nil {
			for i, j = 0, 2; i < 4; i, j = i+1, j+12 {
				nav.Ion_gps[i+4] = Str2Num(buff, j, 12)
			}
		}
	case strings.Contains(label, "DELTA-UTC: A0,A1,T,W"): /* opt ver.2 */
		if nav != nil {
			for i, j = 0, 3; i < 2; i, j = i+1, j+19 {
				nav.Utc_gps[i] = Str2Num(buff, j, 19)
			}
			for ; i < 4; i, j = i+1, j+9 {
				nav.Utc_gps[i] = Str2Num(buff, j, 9)
			}
		}
	case strings.Contains(label, "IONOSPHERIC CORR"): /* opt ver.3 */
		if nav != nil {
			switch {
			case strings.Compare(buff[:4], "GPSA") == 0:
				for i, j = 0, 5; i < 4; i, j = i+1, j+12 {
					nav.Ion_gps[i] = Str2Num(buff, j, 12)
				}
			case strings.Compare(buff[:4], "GPSB") == 0:
				for i, j = 0, 5; i < 4; i, j = i+1, j+12 {
					nav.Ion_gps[i+4] = Str2Num(buff, j, 12)
				}
			case strings.Compare(buff[:3], "GAL") == 0:
				for i, j = 0, 5; i < 4; i, j = i+1, j+12 {
					nav.Ion_gal[i] = Str2Num(buff, j, 12)
				}
			case strings.Compare(buff[:4], "QZSA") == 0: /* v.3.02 */
				for i, j = 0, 5; i < 4; i, j = i+1, j+12 {
					nav.Ion_qzs[i] = Str2Num(buff, j, 12)
				}
			case strings.Compare(buff[:4], "QZSB") == 0: /* v.3.02 */
				for i, j = 0, 5; i < 4; i, j = i+1, j+12 {
					nav.Ion_qzs[i+4] = Str2Num(buff, j, 12)
				}
			case strings.Compare(buff[:4], "BDSA") == 0: /* v.3.02 */
				for i, j = 0, 5; i < 4; i, j = i+1, j+12 {
					nav.Ion_cmp[i] = Str2Num(buff, j, 12)
				}
			case strings.Compare(buff[:4], "BDSB") == 0: /* v.3.02 */
				for i, j = 0, 5; i < 4; i, j = i+1, j+12 {
					nav.Ion_cmp[i+4] = Str2Num(buff, j, 12)
				}
			case strings.Compare(buff[:4], "IRNA") == 0: /* v.3.03 */
				for i, j = 0, 5; i < 4; i, j = i+1, j+12 {
					nav.Ion_irn[i] = Str2Num(buff, j, 12)
				}
			case strings.Compare(buff[:4], "IRNB") == 0: /* v.3.03 */
				for i, j = 0, 5; i < 4; i, j = i+1, j+12 {
					nav.Ion_irn[i+4] = Str2Num(buff, j, 12)
				}
			}
		}
	case strings.Contains(label, "TIME SYSTEM CORR"): /* opt ver.3 */
		if nav != nil {
			switch {
			case strings.Compare(buff[:4], "GPUT") == 0:
				nav.Utc_gps[0] = Str2Num(buff, 5, 17)
				nav.Utc_gps[1] = Str2Num(buff, 22, 16)
				nav.Utc_gps[2] = Str2Num(buff, 38, 7)
				nav.Utc_gps[3] = Str2Num(buff, 45, 5)
			case strings.Compare(buff[:4], "GLUT") == 0:
				nav.Utc_glo[0] = -Str2Num(buff, 5, 17) /* tau_C */
			case strings.Compare(buff[:4], "GLGP") == 0:
				nav.Utc_glo[1] = Str2Num(buff, 5, 17) /* tau_GPS */
			case strings.Compare(buff[:4], "GAUT") == 0: /* v.3.02 */
				nav.Utc_gal[0] = Str2Num(buff, 5, 17)
				nav.Utc_gal[1] = Str2Num(buff, 22, 16)
				nav.Utc_gal[2] = Str2Num(buff, 38, 7)
				nav.Utc_gal[3] = Str2Num(buff, 45, 5)
			case strings.Compare(buff[:4], "QZUT") == 0: /* v.3.02 */
				nav.Utc_qzs[0] = Str2Num(buff, 5, 17)
				nav.Utc_qzs[1] = Str2Num(buff, 22, 16)
				nav.Utc_qzs[2] = Str2Num(buff, 38, 7)
				nav.Utc_qzs[3] = Str2Num(buff, 45, 5)
			case strings.Compare(buff[:4], "BDUT") == 0: /* v.3.02 */
				nav.Utc_cmp[0] = Str2Num(buff, 5, 17)
				nav.Utc_cmp[1] = Str2Num(buff, 22, 16)
				nav.Utc_cmp[2] = Str2Num(buff, 38, 7)
				nav.Utc_cmp[3] = Str2Num(buff, 45, 5)
			case strings.Compare(buff[:4], "SBUT") == 0: /* v.3.02 */
				nav.Utc_sbs[0] = Str2Num(buff, 5, 17)
				nav.Utc_sbs[1] = Str2Num(buff, 22, 16)
				nav.Utc_sbs[2] = Str2Num(buff, 38, 7)
				nav.Utc_sbs[3] = Str2Num(buff, 45, 5)
			case strings.Compare(buff[:4], "IRUT") == 0: /* v.3.03 */
				nav.Utc_irn[0] = Str2Num(buff, 5, 17)
				nav.Utc_irn[1] = Str2Num(buff, 22, 16)
				nav.Utc_irn[2] = Str2Num(buff, 38, 7)
				nav.Utc_irn[3] = Str2Num(buff, 45, 5)
				nav.Utc_irn[8] = 0.0 /* A2 */
			}
		}
	case strings.Contains(label, "LEAP SECONDS"): /* opt */
		if nav != nil {
			nav.Utc_gps[4] = Str2Num(buff, 0, 6)
			nav.Utc_gps[7] = Str2Num(buff, 6, 6)
			nav.Utc_gps[5] = Str2Num(buff, 12, 6)
			nav.Utc_gps[6] = Str2Num(buff, 18, 6)
		}
	}
}

/* decode GNAV header --------------------------------------------------------*/
func (nav *Nav) DecodeGNavHeader(buff string) {
	label := buff[60:]

	Trace(4, "decode_gnavh:\n")

	if strings.Contains(label, "CORR TO SYTEM TIME") { /* opt */
	}
	if strings.Contains(label, "LEAP SECONDS") {
	} /* opt */
}

/* decode GEO NAV header -----------------------------------------------------*/
func (nav *Nav) DecodeHNavHeader(buff string) {
	label := buff[60:]

	Trace(4, "decode_hnavh:\n")

	switch {
	case strings.Contains(label, "CORR TO SYTEM TIME"): /* opt */
	case strings.Contains(label, "D-UTC A0,A1,T,W,S,U"): /* opt */
	case strings.Contains(label, "LEAP SECONDS"):
	} /* opt */
}

/* read RINEX file header ----------------------------------------------------*/
func ReadRnxHeader(rd *bufio.Reader, ver *float64, ctype *byte, sys *int, tsys *int,
	tobs *TOBS, nav *Nav, sta *Sta) int {
	var (
		buff string //,*label=buff+60;
		i    int    = 0
		err  error
	)

	Trace(4, "readrnxh:\n")

	*ver = 2.10
	*ctype = ' '
	*sys = SYS_GPS

	for {
		buff, err = rd.ReadString('\n')
		if err != nil {
			break
		}

		if len(buff) <= 60 {
			continue
		}
		label := buff[60:]

		switch {
		case strings.Contains(label, "RINEX VERSION / TYPE"):
			*ver = Str2Num(buff, 0, 9)
			*ctype = buff[20]

			/* satellite system */
			switch buff[40] {
			case ' ', 'G':
				*sys = SYS_GPS
				*tsys = TSYS_GPS
			case 'R':
				*sys = SYS_GLO
				*tsys = TSYS_UTC
			case 'E':
				*sys = SYS_GAL
				*tsys = TSYS_GAL
				/* v.2.12 */
			case 'S':
				*sys = SYS_SBS
				*tsys = TSYS_GPS
			case 'J':
				*sys = SYS_QZS
				*tsys = TSYS_QZS
				/* v.3.02 */
			case 'C':
				*sys = SYS_CMP
				*tsys = TSYS_CMP
				/* v.2.12 */
			case 'I':
				*sys = SYS_IRN
				*tsys = TSYS_IRN
				/* v.3.03 */
			case 'M':
				*sys = SYS_NONE
				*tsys = TSYS_GPS
				/* mixed */
			default:
				Trace(2, "not supported satellite system: %s\n", buff[40:])

			}
			continue
		case strings.Contains(label, "PGM / RUN BY / DATE"):
			continue
		case strings.Contains(label, "COMMENT"):
			continue
		}
		//	vtype := *ctype
		switch *ctype { /* file type */
		case 'O':
			DecodeObsHeader(rd, buff, *ver, tsys, tobs, nav, sta)
		case 'N':
			nav.DecodeNavHeader(buff)
		case 'G':
			nav.DecodeGNavHeader(buff)
		case 'H':
			nav.DecodeHNavHeader(buff)
		case 'J':
			nav.DecodeNavHeader(buff)
			/* extension */
		case 'L':
			nav.DecodeNavHeader(buff)
			/* extension */
		}
		if strings.Contains(label, "END OF HEADER") {
			return 1
		}
		i++
		if i >= MAXPOSHEAD && *ctype == ' ' {
			break /* no RINEX file */
		}
	}
	return 0
}

/* decode observation epoch --------------------------------------------------*/
func Decode_ObsEpoch(rd *bufio.Reader, buff string, ver float64, time *Gtime, flag *int, sats []int) int {
	var (
		i, j, n int
		satid   string = ""
	)

	Trace(4, "decode_obsepoch: ver=%.2f\n", ver)

	if ver <= 2.99 { /* ver.2 */
		if n = int(Str2Num(buff, 29, 3)); n <= 0 {
			return 0
		}

		/* epoch flag: 3:new site,4:header info,5:external event */
		*flag = int(Str2Num(buff, 28, 1))

		if 3 <= *flag && *flag <= 5 {
			return n
		}

		if Str2Time(buff, 0, 26, time) > 0 {
			Trace(2, "rinex obs invalid epoch: epoch=%26.26s\n", buff)
			return 0
		}
		for i, j = 0, 32; i < n; i, j = i+1, j+3 {
			if j >= 68 {
				buff, _ = rd.ReadString('\n')
				if len(buff) == 0 {
					break
				}
				j = 32
			}
			satid = buff[j : j+3]

			sats[i] = int(SatId2No(satid))

		}
	} else { /* ver.3 */
		if n = int(Str2Num(buff, 32, 3)); n <= 0 {
			return 0
		}

		*flag = int(Str2Num(buff, 31, 1))

		if 3 <= *flag && *flag <= 5 {
			return n
		}

		if buff[0] != '>' || Str2Time(buff, 1, 28, time) > 0 {
			Trace(3, "rinex obs invalid epoch: epoch=%29.29s\n", buff)
			return 0
		}
	}
	Trace(4, "decode_obsepoch: time=%s flag=%d\n", TimeStr(*time, 3), *flag)
	return n
}

/* decode observation data ---------------------------------------------------*/
func (obs *ObsD) DecodeObsData(rd *bufio.Reader, buff string, ver float64, mask int, index []Sigind) int {
	//sigind_t *ind;
	var (
		ind        *Sigind
		val        [MAXOBSTYPE]float64
		lli        [MAXOBSTYPE]uint8
		satid      string = ""
		i, j, n, m int
		stat       int = 1
		p          [MAXOBSTYPE]int
		k, l       [16]int
	)

	Trace(4, "decode_obsdata: ver=%.2f\n", ver)

	if ver > 2.99 { /* ver.3 */
		satid = fmt.Sprintf("%.3s", buff)
		obs.Sat = SatId2No(satid)
	}
	if obs.Sat == 0 {
		Trace(2, "decode_obsdata: unsupported sat sat=%s\n", satid)
		stat = 0
	} else if (SatSys(obs.Sat, nil) & mask) == 0 {
		stat = 0
	}
	/* read observation data fields */
	switch SatSys(obs.Sat, nil) {
	case SYS_GLO:
		ind = &index[1]
	case SYS_GAL:
		ind = &index[2]
	case SYS_QZS:
		ind = &index[3]
	case SYS_SBS:
		ind = &index[4]
	case SYS_CMP:
		ind = &index[5]
	case SYS_IRN:
		ind = &index[6]
	default:
		ind = &index[0]
	}
	j = 3
	if ver <= 2.99 {
		j = 0
	}
	for i = 0; i < ind.n; i, j = i+1, j+16 {

		if ver <= 2.99 && j >= 80 { /* ver.2 */
			buff, _ = rd.ReadString('\n')
			if len(buff) == 0 {
				break
			}
			j = 0
		}
		if stat > 0 {
			val[i] = Str2Num(buff, j, 14) + ind.shift[i]
			lli[i] = uint8(Str2Num(buff, j+14, 1)) & 3
		}
	}
	if stat == 0 {
		return 0
	}

	for i = 0; i < NFREQ+NEXOBS; i++ {
		obs.P[i], obs.L[i] = 0.0, 0.0
		obs.D[i] = 0.0
		obs.SNR[i], obs.LLI[i], obs.Code[i] = 0, 0, 0
	}
	/* assign position in observation data */
	for i, n, m = 0, 0, 0; i < ind.n; i++ {

		p[i] = ind.pos[i]
		if ver <= 2.11 {
			p[i] = ind.idx[i]
		}
		if ind.ctype[i] == 0 && p[i] == 0 {
			k[n] = i
			n++ /* C1? index */
		}
		if ind.ctype[i] == 0 && p[i] == 1 {
			l[m] = i
			m++ /* C2? index */
		}
	}
	if ver <= 2.11 {

		/* if multiple codes (C1/P1,C2/P2), select higher priority */

		if n >= 2 {
			switch {
			case val[k[0]] == 0.0 && val[k[1]] == 0.0:
				p[k[0]] = -1
				p[k[1]] = -1
			case val[k[0]] != 0.0 && val[k[1]] == 0.0:
				p[k[0]] = 0
				p[k[1]] = -1
			case val[k[0]] == 0.0 && val[k[1]] != 0.0:
				p[k[0]] = -1
				p[k[1]] = 0
			case ind.pri[k[1]] > ind.pri[k[0]]:
				p[k[1]] = 0
				p[k[0]] = NFREQ
				if NEXOBS < 1 {
					p[k[0]] = -1
				}
			default:
				p[k[0]] = 0
				p[k[1]] = -1
				if NEXOBS >= 1 {
					p[k[1]] = NFREQ
				}
			}
		}
		if m >= 2 {
			switch {
			case val[l[0]] == 0.0 && val[l[1]] == 0.0:
				p[l[0]] = -1
				p[l[1]] = -1
			case val[l[0]] != 0.0 && val[l[1]] == 0.0:
				p[l[0]] = 1
				p[l[1]] = -1
			case val[l[0]] == 0.0 && val[l[1]] != 0.0:
				p[l[0]] = -1
				p[l[1]] = 1
			case ind.pri[l[1]] > ind.pri[l[0]]:
				p[l[1]] = 1
				p[l[0]] = -1
				if NEXOBS >= 2 {
					p[l[0]] = NFREQ + 1
				}
			default:
				p[l[0]] = 1
				p[l[1]] = -1
				if NEXOBS >= 2 {
					p[l[1]] = NFREQ + 1
				}
			}
		}
	}
	/* save observation data */
	for i = 0; i < ind.n; i++ {
		if p[i] < 0 || val[i] == 0.0 {
			continue
		}
		switch ind.ctype[i] {
		case 0:
			obs.P[p[i]] = val[i]
			obs.Code[p[i]] = ind.code[i]
		case 1:
			obs.L[p[i]] = val[i]
			obs.LLI[p[i]] = lli[i]
		case 2:
			obs.D[p[i]] = val[i]
		case 3:
			obs.SNR[p[i]] = uint16(val[i]/float64(SNR_UNIT) + 0.5)
		}
	}
	Trace(5, "decode_obsdata: time=%s sat=%2d\n", TimeStr(obs.Time, 0), obs.Sat)
	return 1
}

/* save cycle slips ----------------------------------------------------------*/
func (data *ObsD) SaveSlips(slips [][NFREQ + NEXOBS]uint8) {
	for i := 0; i < NFREQ+NEXOBS; i++ {
		if data.LLI[i]&1 != 0 {
			slips[data.Sat-1][i] |= LLI_SLIP
		}
	}
}

/* restore cycle slips -------------------------------------------------------*/
func (data *ObsD) RestoreSlips(slips [][NFREQ + NEXOBS]uint8) {
	for i := 0; i < NFREQ+NEXOBS; i++ {
		if slips[data.Sat-1][i]&1 != 0 {
			data.LLI[i] |= LLI_SLIP
		}
		slips[data.Sat-1][i] = 0
	}
}

/* add observation data ------------------------------------------------------*/
func (obs *Obs) AddObsData(data *ObsD) int {
	obs.Data = append(obs.Data, *data)
	return 1
}

/* set system mask -----------------------------------------------------------*/
func SetSysMask(opt string) int {

	mask := SYS_NONE
	index := strings.Index(opt, "-SYS=")
	if index < 0 {
		return SYS_ALL
	}
	p := []rune(opt)
	for i := 5; p[i] != ' '; i++ {
		switch p[i] {
		case 'G':
			mask |= SYS_GPS
		case 'R':
			mask |= SYS_GLO
		case 'E':
			mask |= SYS_GAL
		case 'J':
			mask |= SYS_QZS
		case 'C':
			mask |= SYS_CMP
		case 'I':
			mask |= SYS_IRN
		case 'S':
			mask |= SYS_SBS
		}
	}
	return mask
}

/* set signal index ----------------------------------------------------------*/
func SetIndex(ver float64, sys int, opt string, tobs []string, ind *Sigind) {
	var (
		str, optstr          string
		shift                float64
		i, j, k, m, n, index int
	)
	for i, n = 0, 0; len(tobs[i]) > 0; i, n = i+1, n+1 {
		ind.code[i] = Obs2Code(tobs[i][1:])
		if index = strings.IndexRune(obstype, rune(tobs[i][0])); index >= 0 {
			ind.ctype[i] = uint8(index)
		} else {
			ind.ctype[i] = 0
		}
		ind.idx[i] = Code2Idx(sys, ind.code[i])
		ind.pri[i] = uint8(GetCodePri(sys, ind.code[i], opt))
		ind.pos[i] = -1
	}
	/* parse phase shift options */
	switch sys {
	case SYS_GPS:
		optstr = "-GL%2s=%f"
	case SYS_GLO:
		optstr = "-RL%2s=%f"
	case SYS_GAL:
		optstr = "-EL%2s=%f"
	case SYS_QZS:
		optstr = "-JL%2s=%f"
	case SYS_SBS:
		optstr = "-SL%2s=%f"
	case SYS_CMP:
		optstr = "-CL%2s=%f"
	case SYS_IRN:
		optstr = "-IL%2s=%f"
	}
	for i = 0; i < len(opt); i++ {
		if index = strings.IndexRune(opt[i:], '-'); index < 0 {
			break
		}

		// }
		// for (p=opt;p&&(p=strchr(p,'-'));p++) {
		m, _ = fmt.Sscanf(opt[i:], optstr, str, &shift)
		if m < 2 {
			continue
		}
		for j = 0; j < n; j++ {
			if index = strings.Index(Code2Obs(ind.code[j]), str); index == 0 {
				continue
			}
			ind.shift[j] = shift
			Trace(2, "phase shift: sys=%2d tobs=%s shift=%.3f\n", sys, tobs[j], shift)
		}
	}
	/* assign index for highest priority code */
	for i = 0; i < NFREQ; i++ {
		for j, k = 0, -1; j < n; j++ {
			if k < 0 {
				if ind.idx[j] == i && ind.pri[j] > 0 {
					k = j
				}
			} else if ind.idx[j] == i && ind.pri[j] > 0 && ind.pri[j] > ind.pri[k] {
				k = j
			}
		}
		if k < 0 {
			continue
		}

		for j = 0; j < n; j++ {
			if ind.code[j] == ind.code[k] {
				ind.pos[j] = i
			}
		}
	}
	/* assign index of extended observation data */
	for i = 0; i < NEXOBS; i++ {
		for j = 0; j < n; j++ {
			if ind.code[j] > 0 && ind.pri[j] > 0 && ind.pos[j] < 0 {
				break
			}
		}
		if j >= n {
			break
		}

		for k = 0; k < n; k++ {
			if ind.code[k] == ind.code[j] {
				ind.pos[k] = NFREQ + i
			}
		}
	}
	for i = 0; i < n; i++ {
		if ind.code[i] == 0 || ind.pri[i] == 0 || ind.pos[i] >= 0 {
			continue
		}
		Trace(4, "reject obs type: sys=%2d, obs=%s\n", sys, tobs[i])
	}
	ind.n = n

}

/* read RINEX observation data body ------------------------------------------*/
func ReadRnxObsBody(rd *bufio.Reader, opt string, ver float64, tsys *int,
	tobs *TOBS, flag *int, data []ObsD, sta *Sta) int {
	var (
		time             Gtime
		index            [NUMSYS]Sigind
		buff             string
		i, n, nsat, mask int
		nsys             int = NUMSYS
		sats             [MAXOBS]int
	)

	/* set system mask */
	mask = SetSysMask(opt)

	/* set signal index */
	if nsys >= 1 {
		SetIndex(ver, SYS_GPS, opt, tobs[0][:], &index[0])
	}
	if nsys >= 2 {
		SetIndex(ver, SYS_GLO, opt, tobs[1][:], &index[1])
	}
	if nsys >= 3 {
		SetIndex(ver, SYS_GAL, opt, tobs[2][:], &index[2])
	}
	if nsys >= 4 {
		SetIndex(ver, SYS_QZS, opt, tobs[3][:], &index[3])
	}
	if nsys >= 5 {
		SetIndex(ver, SYS_SBS, opt, tobs[4][:], &index[4])
	}
	if nsys >= 6 {
		SetIndex(ver, SYS_CMP, opt, tobs[5][:], &index[5])
	}
	if nsys >= 7 {
		SetIndex(ver, SYS_IRN, opt, tobs[6][:], &index[6])
	}

	/* read record */
	for i = 0; ; {
		buff, _ = rd.ReadString('\n')

		if len(buff) == 0 {
			break
		}
		/* decode observation epoch */
		switch {
		case i == 0:
			if nsat = Decode_ObsEpoch(rd, buff, ver, &time, flag, sats[:]); nsat <= 0 {
				continue
			}
		case (*flag <= 2 || *flag == 6) && n < MAXOBS:
			data[n].Time = time
			data[n].Sat = sats[i-1]

			/* decode RINEX observation data */
			if data[n].DecodeObsData(rd, buff, ver, mask, index[:]) > 0 {
				n++
			}
		case *flag == 3 || *flag == 4: /* new site or header info follows */

			/* decode RINEX observation data file header */
			DecodeObsHeader(rd, buff, ver, tsys, tobs, nil, sta)
		}
		if i++; i > nsat {
			return n
		}
	}
	return -1
}

/* read RINEX observation data -----------------------------------------------*/
func (obs *Obs) ReadRnxObs(rd *bufio.Reader, ts, te Gtime, tint float64, opt string, rcv int, ver float64, tsys *int,
	tobs *TOBS, sta *Sta) int {
	var (
		slips            [MAXSAT][NFREQ + NEXOBS]uint8
		i, n, flag, stat int
	)

	Trace(4, "readrnxobs: rcv=%d ver=%.2f tsys=%d\n", rcv, ver, *tsys)

	if obs == nil || rcv > MAXRCV {
		return 0
	}

	data := make([]ObsD, MAXOBS)

	/* read RINEX observation data body */
	for {
		n = ReadRnxObsBody(rd, opt, ver, tsys, tobs, &flag, data, sta)
		if n < 0 || stat < 0 {
			break
		}

		for i = 0; i < n; i++ {

			/* UTC . GPST */
			if *tsys == TSYS_UTC {
				data[i].Time = Utc2GpsT(data[i].Time)
			}

			/* save cycle slip */
			data[i].SaveSlips(slips[:])
		}
		/* screen data by time */
		if n > 0 && ScreenTime(data[0].Time, ts, te, tint) == 0 {
			continue
		}

		for i = 0; i < n; i++ {

			/* restore cycle slip */
			data[i].RestoreSlips(slips[:])

			data[i].Rcv = rcv

			/* save obs data */
			if stat = obs.AddObsData(&data[i]); stat < 0 {
				break
			}
		}
	}
	Trace(5, "readrnxobs: nobs=%d stat=%d\n", obs.N(), stat)

	return stat
}

/* decode ephemeris ----------------------------------------------------------*/
func (eph *Eph) DecodeEph(ver float64, sat int, toc Gtime, data []float64) int {
	var (
		eph0 Eph
		sys  int
	)

	Trace(4, "decode_eph: ver=%.2f sat=%2d\n", ver, sat)

	sys = SatSys(sat, nil)

	if sys&(SYS_GPS|SYS_GAL|SYS_QZS|SYS_CMP|SYS_IRN) == 0 {
		Trace(2, "ephemeris error: invalid satellite sat=%2d\n", sat)
		return 0
	}
	*eph = eph0

	eph.Sat = sat
	eph.Toc = toc

	eph.F0 = data[0]
	eph.F1 = data[1]
	eph.F2 = data[2]

	eph.A = SQR(data[10])
	eph.E = data[8]
	eph.I0 = data[15]
	eph.OMG0 = data[13]
	eph.Omg = data[17]
	eph.M0 = data[6]
	eph.Deln = data[5]
	eph.OMGd = data[18]
	eph.Idot = data[19]
	eph.Crc = data[16]
	eph.Crs = data[4]
	eph.Cuc = data[7]
	eph.Cus = data[9]
	eph.Cic = data[12]
	eph.Cis = data[14]

	switch sys {
	case SYS_GPS, SYS_QZS:
		eph.Iode = int(data[3])  /* IODE */
		eph.Iodc = int(data[26]) /* IODC */
		eph.Toes = data[11]      /* Toe (s) in GPS week */
		eph.Week = int(data[21]) /* GPS week */
		eph.Toe = AdjWeek(GpsT2Time(eph.Week, data[11]), toc)
		eph.Ttr = AdjWeek(GpsT2Time(eph.Week, data[27]), toc)

		eph.Code = int(data[20])     /* GPS: codes on L2 ch */
		eph.Svh = int(data[24])      /* SV health */
		eph.Sva = UraIndex(data[23]) /* URA index (m.index) */
		eph.Flag = int(data[22])     /* GPS: L2 P data flag */

		eph.Tgd[0] = data[25] /* TGD */
		if sys == SYS_GPS {
			eph.Fit = data[28] /* fit interval (h) */
		} else {
			eph.Fit = 2.0
			if data[28] == 0.0 {
				eph.Fit = 1.0 /* fit interval (0:1h,1:>2h) */
			}
		}
	case SYS_GAL: /* GAL ver.3 */
		eph.Iode = int(data[3])  /* IODnav */
		eph.Toes = data[11]      /* Toe (s) in Galileo week */
		eph.Week = int(data[21]) /* Galileo week = GPS week */
		eph.Toe = AdjWeek(GpsT2Time(eph.Week, data[11]), toc)
		eph.Ttr = AdjWeek(GpsT2Time(eph.Week, data[27]), toc)

		eph.Code = int(data[20]) /* data sources */
		/* bit 0 set: I/NAV E1-B */
		/* bit 1 set: F/NAV E5a-I */
		/* bit 2 set: F/NAV E5b-I */
		/* bit 8 set: af0-af2 toc are for E5a.E1 */
		/* bit 9 set: af0-af2 toc are for E5b.E1 */
		eph.Svh = int(data[24]) /* sv health */
		/* bit     0: E1B DVS */
		/* bit   1-2: E1B HS */
		/* bit     3: E5a DVS */
		/* bit   4-5: E5a HS */
		/* bit     6: E5b DVS */
		/* bit   7-8: E5b HS */
		eph.Sva = SisaIndex(data[23]) /* sisa (m.index) */

		eph.Tgd[0] = data[25] /* BGD E5a/E1 */
		eph.Tgd[1] = data[26] /* BGD E5b/E1 */
	case SYS_CMP: /* BeiDou v.3.02 */
		eph.Toc = BDT2GpsT(eph.Toc)                      /* bdt . gpst */
		eph.Iode = int(data[3])                          /* AODE */
		eph.Iodc = int(data[28])                         /* AODC */
		eph.Toes = data[11]                              /* Toe (s) in BDT week */
		eph.Week = int(data[21])                         /* bdt week */
		eph.Toe = BDT2GpsT(BDT2Time(eph.Week, data[11])) /* BDT . GPST */
		eph.Ttr = BDT2GpsT(BDT2Time(eph.Week, data[27])) /* BDT . GPST */
		eph.Toe = AdjWeek(eph.Toe, toc)
		eph.Ttr = AdjWeek(eph.Ttr, toc)

		eph.Svh = int(data[24])      /* satH1 */
		eph.Sva = UraIndex(data[23]) /* URA index (m.index) */

		eph.Tgd[0] = data[25] /* TGD1 B1/B3 */
		eph.Tgd[1] = data[26] /* TGD2 B2/B3 */
	case SYS_IRN: /* IRNSS v.3.03 */
		eph.Iode = int(data[3])  /* IODEC */
		eph.Toes = data[11]      /* Toe (s) in IRNSS week */
		eph.Week = int(data[21]) /* IRNSS week */
		eph.Toe = AdjWeek(GpsT2Time(eph.Week, data[11]), toc)
		eph.Ttr = AdjWeek(GpsT2Time(eph.Week, data[27]), toc)
		eph.Svh = int(data[24])      /* SV health */
		eph.Sva = UraIndex(data[23]) /* URA index (m.index) */
		eph.Tgd[0] = data[25]        /* TGD */
	}
	if eph.Iode < 0 || 1023 < eph.Iode {
		Trace(2, "rinex nav invalid: sat=%2d iode=%d\n", sat, eph.Iode)
	}
	if eph.Iodc < 0 || 1023 < eph.Iodc {
		Trace(2, "rinex nav invalid: sat=%2d iodc=%d\n", sat, eph.Iodc)
	}
	return 1
}

/* decode GLONASS ephemeris --------------------------------------------------*/
func (geph *GEph) DecodeGEph(ver float64, sat int, toc Gtime, data []float64) int {
	var (
		geph0     GEph
		tof       Gtime
		tow, tod  float64
		week, dow int
	)

	Trace(4, "decode_geph: ver=%.2f sat=%2d\n", ver, sat)

	if SatSys(sat, nil) != SYS_GLO {
		Trace(2, "glonass ephemeris error: invalid satellite sat=%2d\n", sat)
		return 0
	}
	*geph = geph0

	geph.Sat = sat

	/* Toc rounded by 15 min in utc */
	tow = Time2GpsT(toc, &week)
	toc = GpsT2Time(week, math.Floor((tow+450.0)/900.0)*900)
	dow = int(math.Floor(tow / 86400.0))

	/* time of frame in UTC */
	tod = math.Mod(data[2], 86400.0)
	if ver <= 2.99 {
		tod = data[2] /* Tod (v.2), Tow (v.3) in UTC */
	}
	tof = GpsT2Time(week, tod+float64(dow)*86400.0)
	tof = AdjDay(tof, toc)

	geph.Toe = Utc2GpsT(toc) /* Toc (GPST) */
	geph.Tof = Utc2GpsT(tof) /* Tof (GPST) */

	/* IODE = Tb (7bit), Tb =index of UTC+3H within current day */
	geph.Iode = int(math.Mod(tow+10800.0, 86400.0)/900.0 + 0.5)

	geph.Taun = -data[0] /* -taun */
	geph.Gamn = data[1]  /* +gamman */

	geph.Pos[0] = data[3] * 1e3
	geph.Pos[1] = data[7] * 1e3
	geph.Pos[2] = data[11] * 1e3
	geph.Vel[0] = data[4] * 1e3
	geph.Vel[1] = data[8] * 1e3
	geph.Vel[2] = data[12] * 1e3
	geph.Acc[0] = data[5] * 1e3
	geph.Acc[1] = data[9] * 1e3
	geph.Acc[2] = data[13] * 1e3

	geph.Svh = int(data[6])
	geph.Frq = int(data[10])

	geph.Age = int(data[14])

	/* some receiver output >128 for minus frequency number */
	if geph.Frq > 128 {
		geph.Frq -= 256
	}

	if geph.Frq < MINFREQ_GLO || MAXFREQ_GLO < geph.Frq {
		Trace(2, "rinex gnav invalid freq: sat=%2d fn=%d\n", sat, geph.Frq)
	}
	return 1
}

/* decode GEO ephemeris ------------------------------------------------------*/
func (seph *SEph) DecodeSEph(ver float64, sat int, toc Gtime, data []float64) int {
	var (
		seph0 SEph
		week  int
	)

	Trace(4, "decode_seph: ver=%.2f sat=%2d\n", ver, sat)

	if SatSys(sat, nil) != SYS_SBS {
		Trace(2, "geo ephemeris error: invalid satellite sat=%2d\n", sat)
		return 0
	}
	*seph = seph0

	seph.Sat = sat
	seph.T0 = toc

	Time2GpsT(toc, &week)
	seph.Tof = AdjWeek(GpsT2Time(week, data[2]), toc)

	seph.Af0 = data[0]
	seph.Af1 = data[1]

	seph.Pos[0] = data[3] * 1e3
	seph.Pos[1] = data[7] * 1e3
	seph.Pos[2] = data[11] * 1e3
	seph.Vel[0] = data[4] * 1e3
	seph.Vel[1] = data[8] * 1e3
	seph.Vel[2] = data[12] * 1e3
	seph.Acc[0] = data[5] * 1e3
	seph.Acc[1] = data[9] * 1e3
	seph.Acc[2] = data[13] * 1e3

	seph.Svh = int(data[6])
	seph.Sva = UraIndex(data[10])

	return 1
}

/* read RINEX navigation data body -------------------------------------------*/
func ReadRnxNavBody(rd *bufio.Reader, opt string, ver float64, sys int,
	ctype *int, eph *Eph, geph *GEph, seph *SEph) int {
	var (
		toc                         Gtime
		data                        [64]float64
		i, j, prn, sat, mask, index int
		sp                          int = 3
		buff, id                    string
	)

	Trace(4, "readrnxnavb: ver=%.2f sys=%d\n", ver, sys)

	/* set system mask */
	mask = SetSysMask(opt)

	for i = 0; ; {
		buff, _ = rd.ReadString('\n')
		if len(buff) == 0 {
			break
		}

		if i == 0 {

			/* decode satellite field */
			if ver >= 3.0 || sys == SYS_GAL || sys == SYS_QZS { /* ver.3 or GAL/QZS */
				id = fmt.Sprintf("%.3s", buff)
				sat = SatId2No(id)
				sp = 4
				if ver >= 3.0 {
					sys = SatSys(sat, nil)
					if sys == 0 {
						switch id[0] {
						case 'S':
							sys = SYS_SBS
						case 'R':
							sys = SYS_GLO
						default:
							sys = SYS_GPS
						}
					}
				}
			} else {
				prn = int(Str2Num(buff, 0, 2))

				switch {
				case sys == SYS_SBS:
					sat = SatNo(SYS_SBS, prn+100)
				case sys == SYS_GLO:
					sat = SatNo(SYS_GLO, prn)
				case 93 <= prn && prn <= 97: /* extension */
					sat = SatNo(SYS_QZS, prn+100)
				default:
					sat = SatNo(SYS_GPS, prn)
				}
			}
			/* decode Toc field */
			if Str2Time(buff[sp:], 0, 19, &toc) > 0 {
				Trace(2, "rinex nav toc error: %23.23s\n", buff)
				return 0
			}
			/* decode data fields */
			for j, index = 0, sp+19; j < 3; i, j, index = i+1, j+1, index+19 {
				data[i] = Str2Num(buff[index:], 0, 19)
			}
		} else {
			/* decode data fields */
			for j, index = 0, sp; j < 4; i, j, index = i+1, j+1, index+19 {
				//				data[i] = Str2Num(buff[index:], 0, 19)
				data[i] = Str2Num(buff[:], index, 19)
			}
			/* decode ephemeris */
			switch {
			case sys == SYS_GLO && i >= 15:
				if mask&sys == 0 {
					return 0
				}
				*ctype = 1
				return geph.DecodeGEph(ver, sat, toc, data[:])
			case sys == SYS_SBS && i >= 15:
				if mask&sys == 0 {
					return 0
				}
				*ctype = 2
				return seph.DecodeSEph(ver, sat, toc, data[:])
			case i >= 31:
				if mask&sys == 0 {
					return 0
				}
				*ctype = 0
				return eph.DecodeEph(ver, sat, toc, data[:])
			}
		}
	}
	return -1
}

/* add ephemeris to navigation data ------------------------------------------*/
func (nav *Nav) AddEph(eph *Eph) int {
	nav.Ephs = append(nav.Ephs, *eph)
	return 1
}
func (nav *Nav) AddGEph(geph *GEph) int {
	nav.Geph = append(nav.Geph, *geph)
	return 1
}
func (nav *Nav) AddSEph(seph *SEph) int {
	nav.Seph = append(nav.Seph, *seph)
	return 1
}

/* read RINEX navigation data ------------------------------------------------*/
func (nav *Nav) ReadRnxNav(rd *bufio.Reader, opt string, ver float64, sys int) int {
	var (
		eph         Eph
		geph        GEph
		seph        SEph
		stat, ctype int
	)

	Trace(4, "readrnxnav: ver=%.2f sys=%d\n", ver, sys)

	if nav == nil {
		return 0
	}

	/* read RINEX navigation data body */

	for {
		stat = ReadRnxNavBody(rd, opt, ver, sys, &ctype, &eph, &geph, &seph)
		if stat < 0 {
			break
		}
		/* add ephemeris to navigation data */
		switch ctype {
		case 1:
			stat = nav.AddGEph(&geph)
		case 2:
			stat = nav.AddSEph(&seph)
		default:
			stat = nav.AddEph(&eph)
		}
		if stat == 0 {
			return 0
		}
	}
	if nav.N() > 0 || nav.Ng() > 0 || nav.Ns() > 0 {
		return 1
	}
	return 0
}

/* read RINEX clock ----------------------------------------------------------*/
func (nav *Nav) ReadRnxClk(rd *bufio.Reader, opt string, index int) int {
	var (
		time            Gtime
		data            [2]float64
		i, j, sat, mask int
		buff, satid     string
	)

	Trace(4, "readrnxclk: index=%d\n", index)

	if nav == nil {
		return 0
	}

	/* set system mask */
	mask = SetSysMask(opt)

	for {
		buff, _ = rd.ReadString('\n')
		if len(buff) == 0 {
			break
		}

		if Str2Time(buff, 8, 26, &time) > 0 {
			Trace(2, "rinex clk invalid epoch: %34.34s\n", buff)
			continue
		}
		satid = buff[3:7]

		/* only read AS (satellite clock) record */
		sat = SatId2No(satid)
		if strings.Compare(buff[:2], "AS") != 0 || sat == 0 {
			continue
		}

		if (SatSys(sat, nil) & mask) == 0 {
			continue
		}

		for i, j = 0, 40; i < 2; i, j = i+1, j+20 {
			data[i] = Str2Num(buff, j, 19)
		}

		if nav.Nc() <= 0 || math.Abs(TimeDiff(time, nav.Pclk[nav.Nc()-1].Time)) > 1e-9 {
			var pclk PClk
			pclk.Time = time
			pclk.Index = index
			nav.Pclk = append(nav.Pclk, pclk)
		}
		nav.Pclk[nav.Nc()-1].Clk[sat-1][0] = data[0]
		nav.Pclk[nav.Nc()-1].Std[sat-1][0] = float32(data[1])
	}
	if nav.Nc() > 0 {
		return 1
	}
	return 0
}

/* read RINEX file -----------------------------------------------------------*/
func ReadRnxFp(rd *bufio.Reader, ts, te Gtime, tint float64,
	opt string, flag, index int, ctype *byte,
	obs *Obs, nav *Nav, sta *Sta) int {
	var (
		ver       float64
		sys, tsys int = 0, TSYS_GPS
		tobs      TOBS
	)

	Trace(4, "readrnxfp: flag=%d index=%d\n", flag, index)

	/* read RINEX file header */
	if ReadRnxHeader(rd, &ver, ctype, &sys, &tsys, &tobs, nav, sta) == 0 {
		return 0
	}

	/* flag=0:except for clock,1:clock */

	if (flag == 0 && *ctype == 'C') || (flag > 0 && *ctype != 'C') {
		return 0
	}

	/* read RINEX file body */
	switch *ctype {
	case 'O':
		return obs.ReadRnxObs(rd, ts, te, tint, opt, index, ver, &tsys, &tobs,
			sta)
	case 'N':
		return nav.ReadRnxNav(rd, opt, ver, sys)
	case 'G':
		return nav.ReadRnxNav(rd, opt, ver, SYS_GLO)
	case 'H':
		return nav.ReadRnxNav(rd, opt, ver, SYS_SBS)
	case 'J':
		return nav.ReadRnxNav(rd, opt, ver, SYS_QZS) /* extension */
	case 'L':
		return nav.ReadRnxNav(rd, opt, ver, SYS_GAL) /* extension */
	case 'C':
		return nav.ReadRnxClk(rd, opt, index)
	}
	Trace(5, "unsupported rinex type ver=%.2f type=%c\n", ver, *ctype)
	return 0
}

/* uncompress and read RINEX file --------------------------------------------*/
func ReadRnxFile(file string, ts, te Gtime, tint float64,
	opt string, flag, index int, ctype *byte,
	obs *Obs, nav *Nav, sta *Sta) int {
	var (
		fp          *os.File
		cstat, stat int
		tmpfile     string
		err         error
	)

	Trace(4, "readrnxfile: file=%s flag=%d index=%d\n", file, flag, index)

	if sta != nil {
		sta.InitSta()
	}

	/* uncompress file */
	if cstat = Rtk_Uncompress(file, &tmpfile); cstat < 0 {
		Trace(2, "rinex file uncompact error: %s\n", file)
		return 0
	}
	if cstat > 0 {
		fp, err = os.OpenFile(tmpfile, os.O_RDONLY, 0666)
	} else {
		fp, err = os.OpenFile(file, os.O_RDONLY, 0666)
	}
	if err != nil {
		if cstat == 0 {
			tmpfile = file
		}
		Trace(2, "rinex file open error: %s\n", tmpfile)
		return 0
	}
	defer fp.Close()
	/* read RINEX file */
	rd := bufio.NewReader(fp)
	stat = ReadRnxFp(rd, ts, te, tint, opt, flag, index, ctype, obs, nav, sta)

	/* delete temporary file */
	if cstat > 0 {
		os.Remove(tmpfile)
	}

	return stat
}

/* read RINEX OBS and NAV files ------------------------------------------------
* read RINEX OBS and NAV files
* args   : char *file    I      file (wild-card * expanded) ("": stdin)
*          int   rcv     I      receiver number for obs data
*         (gtime_t ts)   I      observation time start (ts.time==0: no limit)
*         (gtime_t te)   I      observation time end   (te.time==0: no limit)
*         (double tint)  I      observation time interval (s) (0:all)
*          char  *opt    I      RINEX options (see below,"": no option)
*          obs_t *obs    IO     observation data   (NULL: no input)
*          nav_t *nav    IO     navigation data    (NULL: no input)
*          sta_t *sta    IO     station parameters (NULL: no input)
* return : status (1:ok,0:no data,-1:error)
* notes  : read data are appended to obs and nav struct
*          before calling the function, obs and nav should be initialized.
*          observation data and navigation data are not sorted.
*          navigation data may be duplicated.
*          call sortobs() or uniqnav() to sort data or delete duplicated eph.
*
*          RINEX options (separated by spaces) :
*
*            -GLss[=shift]: select GPS signal ss (ss: RINEX 3 code, "1C","2W"...)
*            -RLss[=shift]: select GLO signal ss
*            -ELss[=shift]: select GAL signal ss
*            -JLss[=shift]: select QZS signal ss
*            -CLss[=shift]: select BDS signal ss
*            -ILss[=shift]: select IRN signal ss
*            -SLss[=shift]: select SBS signal ss
*
*                 shift: carrier phase shift to be added (cycle)
*
*            -SYS=sys[,sys...]: select navigation systems
*                               (sys=G:GPS,R:GLO,E:GAL,J:QZS,C:BDS,I:IRN,S:SBS)
*
*-----------------------------------------------------------------------------*/
func ReadRnxT(file string, rcv int, ts, te Gtime, tint float64, opt string, obs *Obs, nav *Nav, sta *Sta) int {
	var (
		i, n, stat, index int
		ctype             byte
		files             []string = make([]string, MAXEXFILE)
	)

	Trace(4, "readrnxt: file=%s rcv=%d\n", file, rcv)

	/* expand wild-card */
	if n = ExPath(file, files, MAXEXFILE); n <= 0 {
		return 0
	}
	/* read rinex files */
	for i = 0; i < n && stat >= 0; i++ {
		stat = ReadRnxFile(files[i], ts, te, tint, opt, 0, rcv, &ctype, obs, nav, sta)
	}
	/* if station name empty, set 4-char name from file head */
	if ctype == 'O' && sta != nil {
		if index = strings.LastIndex(file, "/"); index < 0 {
			index = -1
		}

		if len(sta.Name) == 0 {
			setstr(&sta.Name, file[index+1:], 4)
		}
	}

	return stat
}
func ReadRnx(file string, rcv int, opt string, obs *Obs, nav *Nav, sta *Sta) int {
	var t Gtime

	Trace(4, "readrnx : file=%s rcv=%d\n", file, rcv)

	return ReadRnxT(file, rcv, t, t, 0.0, opt, obs, nav, sta)
}

/* compare precise clock -----------------------------------------------------*/
func cmppclk(q1, q2 *PClk) int {
	tt := TimeDiff(q1.Time, q2.Time)
	switch {
	case tt < -1e-9:
		return -1
	case tt > 1e-9:
		return 1
	default:
		return q1.Index - q2.Index
	}
}

/* combine precise clock -----------------------------------------------------*/
func (nav *Nav) CombinePrecClk() {
	var i, j, k int

	Trace(4, "combpclk: nc=%d\n", nav.Nc)

	if nav.Nc() <= 0 {
		return
	}

	sort.Slice(nav.Pclk, func(i, j int) bool {
		return cmppclk(&nav.Pclk[i], &nav.Pclk[j]) < 0
	})

	for i, j = 0, 1; j < nav.Nc(); j++ {
		if math.Abs(TimeDiff(nav.Pclk[i].Time, nav.Pclk[j].Time)) < 1e-9 {
			for k = 0; k < len(nav.Pclk[j].Clk); k++ {
				if nav.Pclk[j].Clk[k][0] == 0.0 {
					continue
				}
				nav.Pclk[i].Clk[k][0] = nav.Pclk[j].Clk[k][0]
				nav.Pclk[i].Std[k][0] = nav.Pclk[j].Std[k][0]
			}
		} else if i++; i < j {
			nav.Pclk[i] = nav.Pclk[j]
		}
	}
	nav.Pclk = nav.Pclk[:i+1]

	Trace(5, "combpclk: nc=%d\n", nav.Nc())
}

/* read RINEX clock files ------------------------------------------------------
* read RINEX clock files
* args   : char *file    I      file (wild-card * expanded)
*          nav_t *nav    IO     navigation data    (NULL: no input)
* return : number of precise clock
*-----------------------------------------------------------------------------*/
func (nav *Nav) ReadRnxC(file string) int {
	var (
		t           Gtime
		i, n, index int
		stat        int = 1
		files       [MAXEXFILE]string
		ctype       byte
	)

	Trace(4, "readrnxc: file=%s\n", file)

	/* expand wild-card */
	n = ExPath(file, files[:], MAXEXFILE)

	/* read rinex clock files */
	for i = 0; i < n; i, index = i+1, index+1 {
		if ReadRnxFile(files[i], t, t, 0.0, "", 1, index, &ctype, nil, nav, nil) > 0 {
			continue
		}
		stat = 0
		break
	}

	if stat == 0 {
		return 0
	}

	/* unique and combine ephemeris and precise clock */
	nav.CombinePrecClk()

	return nav.Nc()
}

/* initialize RINEX control ----------------------------------------------------
* initialize RINEX control struct and reallocate memory for observation and
* ephemeris buffer in RINEX control struct
* args   : rnxctr_t *rnx IO     RINEX control struct
* return : status (1:ok,0:memory allocation error)
*-----------------------------------------------------------------------------*/
func InitRnxCtr(rnx *RnxCtr) int {
	var (
		time0 Gtime
		data0 ObsD
		eph0  Eph  = Eph{Sat: 0, Iode: -1, Iodc: -1}
		geph0 GEph = GEph{Sat: 0, Iode: -1}
		seph0 SEph
		i, j  int
	)

	Trace(4, "init_rnxctr:\n")
	rnx.obs.Data = nil
	rnx.nav.Ephs = nil
	rnx.nav.Geph = nil
	rnx.nav.Seph = nil

	rnx.obs.Data = make([]ObsD, MAXOBS)
	rnx.nav.Ephs = make([]Eph, MAXSAT*2)
	rnx.nav.Geph = make([]GEph, NSATGLO)
	rnx.nav.Seph = make([]SEph, NSATSBS*2)

	rnx.time = time0
	rnx.ver = 0.0
	rnx.sys, rnx.tsys = 0, 0
	for i = 0; i < 6; i++ {
		for j = 0; j < MAXOBSTYPE; j++ {
			rnx.tobs[i][j] = ""
		}
	}
	// rnx.obs.N = 0
	// rnx.nav.N = MAXSAT * 2
	// rnx.nav.Ng = NSATGLO
	// rnx.nav.Ns = NSATSBS * 2
	for i = 0; i < rnx.obs.N(); i++ {
		rnx.obs.Data[i] = data0
	}
	for i = 0; i < rnx.nav.Ne(); i++ {
		rnx.nav.Ephs[i] = eph0
	}
	for i = 0; i < rnx.nav.Ng(); i++ {
		rnx.nav.Geph[i] = geph0
	}
	for i = 0; i < rnx.nav.Ns(); i++ {
		rnx.nav.Seph[i] = seph0
	}
	rnx.ephsat, rnx.ephset = 0, 0
	rnx.opt = ""

	return 1
}

/* free RINEX control ----------------------------------------------------------
* free observation and ephemris buffer in RINEX control struct
* args   : rnxctr_t *rnx IO  RINEX control struct
* return : none
*-----------------------------------------------------------------------------*/
func (rnx *RnxCtr) FreeRnxCtr() {
	Trace(4, "free_rnxctr:\n")

	rnx.obs.Data = nil
	// rnx.obs.N = 0
	rnx.nav.Ephs = nil
	rnx.nav.Geph = nil
	rnx.nav.Seph = nil
}

/* open RINEX data -------------------------------------------------------------
* fetch next RINEX message and input a messsage from file
* args   : rnxctr_t *rnx IO  RINEX control struct
*          FILE  *fp    I    file pointer
* return : status (-2: end of file, 0: no message, 1: input observation data,
*                   2: input navigation data)
*-----------------------------------------------------------------------------*/
func (rnx *RnxCtr) OpenRnxCtr(rd *bufio.Reader) int {
	rnxtypes := "ONGLJHC"
	var (
		ver             float64
		ctype           byte
		tobs            TOBS
		i, j, sys, tsys int
	)

	Trace(4, "open_rnxctr:\n")

	/* read RINEX header from file */
	if ReadRnxHeader(rd, &ver, &ctype, &sys, &tsys, &tobs, &rnx.nav, &rnx.sta) == 0 {
		Trace(2, "open_rnxctr: rinex header read error\n")
		return 0
	}

	if !strings.Contains(rnxtypes, string(ctype)) {
		Trace(2, "open_rnxctr: not supported rinex type type=%c\n", ctype)
		return 0
	}
	rnx.ver = ver
	rnx.filetype = string(ctype)
	rnx.sys = sys
	rnx.tsys = tsys
	for i = 0; i < NUMSYS; i++ {
		for j = 0; j < MAXOBSTYPE && len(tobs[i][j]) > 0; j++ {
			rnx.tobs[i][j] = tobs[i][j]
		}
	}
	rnx.ephset, rnx.ephsat = 0, 0
	return 1
}

/* input RINEX control ---------------------------------------------------------
* fetch next RINEX message and input a messsage from file
* args   : rnxctr_t *rnx    IO  RINEX control struct
*          FILE  *fp        I   file pointer
* return : status (-2: end of file, 0: no message, 1: input observation data,
*                   2: input navigation data)
* notes  : if status=1, input obs data are set to rnx as follows:
*            rnx.time      : obs data epoch time
*            rnx.obs.n     : number of obs data
*            rnx.obs.data[]: obs data
*          if status=2, input nav data are set to rnx as follows:
*            rnx.time      : ephemeris frame time
*            rnx.ephsat    : sat-no of input ephemeris
*            rnx.ephset    : set-no of input ephemeris (0:set1,1:set2)
*            rnx.nav.geph[prn-1]        : GLOASS ephemeris (prn=slot-no)
*            rnx.nav.seph[prn-MINPRNSBS]: SBAS ephemeris   (prn=PRN-no)
*            rnx.nav.eph [sat-1]        : other ephemeris set1 (sat=sat-no)
*            rnx.nav.eph [sat-1+MAXSAT] : other ephemeris set2 (sat=sat-no)
*-----------------------------------------------------------------------------*/
func (rnx *RnxCtr) InputRnxCtr(rd *bufio.Reader) int {
	var (
		eph                                 Eph
		geph                                GEph
		seph                                SEph
		n, sys, stat, flag, prn, itype, set int
	)

	Trace(4, "input_rnxctr:\n")

	/* read RINEX OBS data */
	if rnx.filetype == "O" {
		if n = ReadRnxObsBody(rd, rnx.opt, rnx.ver, &rnx.tsys, &rnx.tobs, &flag,
			rnx.obs.Data, &rnx.sta); n <= 0 {
			rnx.obs.n = 0 // 到达文件尾部，只能让rnx.obs.n为0，不能让rnx.obs.Data为nil！！！
			if n < 0 {
				return -2
			} else {
				return 0
			}
		}
		rnx.time = rnx.obs.Data[0].Time
		rnx.obs.n = n
		return 1
	}
	/* read RINEX NAV data */
	switch rnx.filetype[0] {
	case 'N':
		sys = SYS_NONE
	case 'G':
		sys = SYS_GLO
	case 'H':
		sys = SYS_SBS
	case 'L':
		sys = SYS_GAL
		/* extension */
	case 'J':
		sys = SYS_QZS
		/* extension */
	default:
		return 0
	}
	if stat = ReadRnxNavBody(rd, rnx.opt, rnx.ver, sys, &itype, &eph, &geph, &seph); stat <= 0 {
		if stat < 0 {
			return -2
		} else {
			return 0
		}
	}
	switch itype {
	case 1: /* GLONASS ephemeris */
		// sys = SatSys(geph.Sat, &prn)
		rnx.nav.Geph[prn-1] = geph
		rnx.time = geph.Tof
		rnx.ephsat = geph.Sat
		rnx.ephset = 0
	case 2: /* SBAS ephemeris */
		// sys = SatSys(seph.Sat, &prn)
		rnx.nav.Seph[prn-MINPRNSBS] = seph
		rnx.time = seph.Tof
		rnx.ephsat = seph.Sat
		rnx.ephset = 0
	default: /* other ephemeris */
		sys = SatSys(eph.Sat, &prn)
		set = 0
		if sys == SYS_GAL && (eph.Code&(1<<9)) > 0 {
			set = 1 /* GAL 0:I/NAV,1:F/NAV */
		}
		rnx.nav.Ephs[eph.Sat-1+MAXSAT*set] = eph
		rnx.time = eph.Ttr
		rnx.ephsat = eph.Sat
		rnx.ephset = set
	}
	return 2
}

/*------------------------------------------------------------------------------
* output RINEX functions
*-----------------------------------------------------------------------------*/

/* output obs-types RINEX ver.2 ----------------------------------------------*/
func OutObsTypeVer2(fp *os.File, opt *RnxOpt) {
	var (
		label = "# / TYPES OF OBSERV"
		i     int
	)

	Trace(4, "outobstype_ver2:\n")

	fp.WriteString(fmt.Sprintf("%6d", opt.NObs[0]))

	for i = 0; i < opt.NObs[0]; i++ {
		if i > 0 && i%9 == 0 {
			fp.WriteString("      ")
		}

		fp.WriteString(fmt.Sprintf("%6s", opt.TObs[0][i]))

		if i%9 == 8 {
			fp.WriteString(fmt.Sprintf("%-20s\n", label))
		}
	}
	if opt.NObs[0] == 0 || i%9 > 0 {
		fp.WriteString(fmt.Sprintf("%*s%-20s\n", (9-i%9)*6, "", label))
	}
}

/* output obs-types RINEX ver.3 ----------------------------------------------*/
func OutObsTypeVer3(fp *os.File, opt *RnxOpt) {
	var (
		label = "SYS / # / OBS TYPES"
		i, j  int
		tobs  []rune
	)

	Trace(4, "outobstype_ver3:\n")

	for i = 0; navsys[i] > 0; i++ {
		if (navsys[i]&opt.NavSys) == 0 || opt.NObs[i] == 0 {
			continue
		}

		fp.WriteString(fmt.Sprintf("%c  %3d", syscodes[i], opt.NObs[i]))

		for j = 0; j < opt.NObs[i]; j++ {
			if j > 0 && j%13 == 0 {
				fp.WriteString("      ")
			}

			tobs = []rune(opt.TObs[i][j])

			/* BDS B2x . 1x (3.02), 2x (other) */
			if navsys[i] == SYS_CMP {
				if opt.RnxVer == 302 && tobs[1] == '2' {
					tobs[1] = '1'
				}
			}
			fp.WriteString(fmt.Sprintf(" %3s", string(tobs)))

			if j%13 == 12 {
				fp.WriteString(fmt.Sprintf("  %-20s\n", label))
			}
		}
		if j%13 > 0 {
			fp.WriteString(fmt.Sprintf("%*s  %-20s\n", (13-j%13)*4, "", label))
		}
	}
}

/* output RINEX phase shift --------------------------------------------------*/
func OutRnxPhaseShift(fp *os.File, opt *RnxOpt, nav *Nav) {
	var (
		ref_code [][10]uint8 = [][10]uint8{ /* reference signal [9] table A23 */
			{CODE_L1C, CODE_L2P, CODE_L5I, 0},                                         /* GPS */
			{CODE_L1C, CODE_L4A, CODE_L2C, CODE_L6A, CODE_L3I, 0},                     /* GLO */
			{CODE_L1B, CODE_L5I, CODE_L7I, CODE_L8I, CODE_L6B, 0},                     /* GAL */
			{CODE_L1C, CODE_L2S, CODE_L5I, CODE_L5D, CODE_L6S, 0},                     /* QZS */
			{CODE_L1C, CODE_L5I, 0},                                                   /* SBS */
			{CODE_L2I, CODE_L1D, CODE_L5D, CODE_L7I, CODE_L7D, CODE_L8D, CODE_L6I, 0}, /* BDS */
			{CODE_L5A, CODE_L9A, 0} /* IRN */}
		label   = "SYS / PHASE SHIFT"
		obs     []rune
		i, j, k int
	)

	for i = 0; navsys[i] > 0; i++ {
		if (navsys[i]&opt.NavSys) == 0 || opt.NObs[i] == 0 {
			continue
		}
		for j = 0; j < opt.NObs[i]; j++ {
			if opt.TObs[i][j][0] != 'L' {
				continue
			}
			obs = []rune(opt.TObs[i][j])
			for k = 0; ref_code[i][k] > 0; k++ {
				if Obs2Code(string(obs[1:])) == ref_code[i][k] {
					break
				}
			}
			if navsys[i] == SYS_CMP { /* BDS B2x . 1x (3.02), 2x (other) */
				if opt.RnxVer == 302 && obs[1] == '2' {
					obs[1] = '1'
				}
			}
			if ref_code[i][k] > 0 {
				fp.WriteString(fmt.Sprintf("%c %3s %54s%-20s\n", syscodes[i], string(obs), "", label))
			} else {
				fp.WriteString(fmt.Sprintf("%c %3s %8.5f%46s%-20s\n", syscodes[i], string(obs),
					opt.Shift[i][j], "", label))
			}
		}
	}
}

/* output RINEX GLONASS slot/freq # ------------------------------------------*/
func OutRnxGloFcn(fp *os.File, opt *RnxOpt, nav *Nav) {
	var (
		label           = "GLONASS SLOT / FRQ #"
		i, j, k, n, sat int
		prn, fcn        [MAXPRNGLO]int
	)

	if opt.NavSys&SYS_GLO > 0 {
		for i = 0; i < MAXPRNGLO; i, n = i+1, n+1 {
			sat = SatNo(SYS_GLO, i+1)
			switch {
			case nav.Geph[i].Sat == sat:
				prn[n] = i + 1
				fcn[n] = nav.Geph[i].Frq
			case nav.Glo_fcn[i] > 0:
				prn[n] = i + 1
				fcn[n] = nav.Glo_fcn[i] - 8
			}
		}
	}
	if n <= 0 {
		n = 1
	} else {
		n = (n-1)/8 + 1
	}
	for i, j = 0, 0; i < n; i++ {
		if i == 0 {
			fp.WriteString(fmt.Sprintf("%3d", n))
		} else {
			fp.WriteString("   ")
		}
		for k = 0; k < 8 && j < n; k, j = k+1, j+1 {
			fp.WriteString(fmt.Sprintf(" R%02d %2d", prn[j], fcn[j]))
		}
		fp.WriteString(fmt.Sprintf("%*s %-20s\n", (8-k)*7, "", label))
	}
}

/* output RINEX GLONASS code/phase/bias --------------------------------------*/
func OutRnxGloBias(fp *os.File, opt *RnxOpt) {
	var (
		label           = "GLONASS COD/PHS/BIS"
		tobs  [4]string = [4]string{"C1C", "C1P", "C2C", "C2P"}
	)

	if opt.NavSys&SYS_GLO > 0 {
		fp.WriteString(fmt.Sprintf(" %s %8.3f %s %8.3f %s %8.3f %s %8.3f%8s%-20s\n",
			tobs[0], opt.Glo_cp_bias[0], tobs[1], opt.Glo_cp_bias[1],
			tobs[2], opt.Glo_cp_bias[2], tobs[3], opt.Glo_cp_bias[3], "",
			label))
	} else {
		fp.WriteString(fmt.Sprintf("%*s%-20s\n", 60, "", label))
	}
}

/* output RINEX observation data file header -----------------------------------
* output RINEX observation data file header
* args   : FILE   *fp       I   output file pointer
*          rnxopt_t *opt    I   RINEX options
*          nav_t  *nav      I   navigation data
* return : status (1:ok, 0:output error)
*-----------------------------------------------------------------------------*/
func OutRnxObsHeader(fp *os.File, opt *RnxOpt, nav *Nav) int {
	var (
		ep        [6]float64
		pos, del  [3]float64
		date, sys string
		tsys      = "GPS"
		i         int
	)

	Trace(4, "outrnxobsh:\n")

	TimeStrRnx(&date)

	switch opt.NavSys {
	case SYS_GPS:
		sys = "G: GPS"
	case SYS_GLO:
		sys = "R: GLONASS"
	case SYS_GAL:
		sys = "E: Galielo"
	case SYS_QZS:
		sys = "J: QZSS" /* ver.3.02 */
	case SYS_CMP:
		sys = "C: BeiDou" /* ver.3.02 */
	case SYS_IRN:
		sys = "I: IRNSS" /* ver.3.03 */
	case SYS_SBS:
		sys = "S: SBAS Payload"
	default:
		sys = "M: Mixed"
	}

	fp.WriteString(fmt.Sprintf("%9.2f%-11s%-20s%-20s%-20s\n", float64(opt.RnxVer)/100.0, "",
		"OBSERVATION DATA", sys, "RINEX VERSION / TYPE"))
	fp.WriteString(fmt.Sprintf("%-20.20s%-20.20s%-20.20s%-20s\n", opt.Prog, opt.RunBy, date,
		"PGM / RUN BY / DATE"))

	for i = 0; i < MAXCOMMENT; i++ {
		if len(opt.Comment[i]) == 0 {
			continue
		}
		fp.WriteString(fmt.Sprintf("%-60.60s%-20s\n", opt.Comment[i], "COMMENT"))
	}
	fp.WriteString(fmt.Sprintf("%-60.60s%-20s\n", opt.Marker, "MARKER NAME"))
	fp.WriteString(fmt.Sprintf("%-20.20s%-40.40s%-20s\n", opt.MarkerNo, "", "MARKER NUMBER"))

	if opt.RnxVer >= 300 {
		fp.WriteString(fmt.Sprintf("%-20.20s%-40.40s%-20s\n", opt.MarkerType, "", "MARKER TYPE"))
	}
	fp.WriteString(fmt.Sprintf("%-20.20s%-40.40s%-20s\n", opt.Name[0], opt.Name[1],
		"OBSERVER / AGENCY"))
	fp.WriteString(fmt.Sprintf("%-20.20s%-20.20s%-20.20s%-20s\n", opt.Rec[0], opt.Rec[1],
		opt.Rec[2], "REC # / TYPE / VERS"))
	fp.WriteString(fmt.Sprintf("%-20.20s%-20.20s%-20.20s%-20s\n", opt.Ant[0], opt.Ant[1],
		"", "ANT # / TYPE"))

	for i = 0; i < 3; i++ {
		if math.Abs(opt.AppPos[i]) < 1e8 {
			pos[i] = opt.AppPos[i]
		}
	}
	for i = 0; i < 3; i++ {
		if math.Abs(opt.AntDel[i]) < 1e8 {
			del[i] = opt.AntDel[i]
		}
	}
	fp.WriteString(fmt.Sprintf("%14.4f%14.4f%14.4f%-18s%-20s\n", pos[0], pos[1], pos[2], "",
		"APPROX POSITION XYZ"))
	fp.WriteString(fmt.Sprintf("%14.4f%14.4f%14.4f%-18s%-20s\n", del[0], del[1], del[2], "",
		"ANTENNA: DELTA H/E/N"))

	if opt.RnxVer <= 299 { /* ver.2 */
		fp.WriteString(fmt.Sprintf("%6d%6d%-48s%-20s\n", 1, 1, "", "WAVELENGTH FACT L1/2"))
		OutObsTypeVer2(fp, opt)
	} else { /* ver.3 */
		OutObsTypeVer3(fp, opt)
	}
	if opt.TInt > 0.0 {
		fp.WriteString(fmt.Sprintf("%10.3f%50s%-20s\n", opt.TInt, "", "INTERVAL"))
	}
	Time2Epoch(opt.TStart, ep[:])
	fp.WriteString(fmt.Sprintf("  %04.0f    %02.0f    %02.0f    %02.0f    %02.0f   %010.7f     %-12s%-20s\n",
		ep[0], ep[1], ep[2], ep[3], ep[4], ep[5], tsys, "TIME OF FIRST OBS"))

	Time2Epoch(opt.TEnd, ep[:])
	fp.WriteString(fmt.Sprintf("  %04.0f    %02.0f    %02.0f    %02.0f    %02.0f   %010.7f     %-12s%-20s\n",
		ep[0], ep[1], ep[2], ep[3], ep[4], ep[5], tsys, "TIME OF LAST OBS"))

	if opt.RnxVer >= 301 {
		OutRnxPhaseShift(fp, opt, nav) /* SYS / PHASE SHIFT */
	}
	if opt.RnxVer >= 302 {
		OutRnxGloFcn(fp, opt, nav) /* GLONASS SLOT / FRQ # */
	}
	if opt.RnxVer >= 302 {
		OutRnxGloBias(fp, opt) /* GLONASS COD/PHS/BIS */
	}
	_, err := fp.WriteString(fmt.Sprintf("%-60.60s%-20s\n", "", "END OF HEADER"))
	if err != io.EOF {
		return 1
	}
	return 0
}

/* output observation data field ---------------------------------------------*/
func OutRnxObsf(fp *os.File, obs float64, lli int) {
	if obs == 0.0 || obs <= -1e9 || obs >= 1e9 {
		fp.WriteString("              ")
	} else {
		fp.WriteString(fmt.Sprintf("%14.3f", obs))
	}
	if lli < 0 || (lli&(LLI_SLIP|LLI_HALFC|LLI_BOCTRK) == 0) {
		fp.WriteString("  ")
	} else {
		fp.WriteString(fmt.Sprintf("%1.1d ", lli&(LLI_SLIP|LLI_HALFC|LLI_BOCTRK)))
	}
}

/* search obsservattion data index -------------------------------------------*/
func RnxObsIndex(rnxver, sys int, code []uint8, tobs, mask string) int {
	var id string
	for i := 0; i < NFREQ+NEXOBS; i++ {

		/* signal mask */
		if code[i] < 1 || mask[code[i]-1] == '0' {
			continue
		}

		if rnxver <= 299 { /* ver.2 */
			switch {
			case strings.Index(tobs, "C1") == 0 && (sys == SYS_GPS || sys == SYS_GLO || sys == SYS_QZS ||
				sys == SYS_SBS || sys == SYS_CMP):
				if code[i] == CODE_L1C {
					return i
				}
			case strings.Index(tobs, "P1") == 0:
				if code[i] == CODE_L1P || code[i] == CODE_L1W || code[i] == CODE_L1Y ||
					code[i] == CODE_L1N {
					return i
				}
			case strings.Index(tobs, "C2") == 0 && (sys == SYS_GPS || sys == SYS_QZS):
				if code[i] == CODE_L2S || code[i] == CODE_L2L || code[i] == CODE_L2X {
					return i
				}
			case strings.Index(tobs, "C2") == 0 && sys == SYS_GLO:
				if code[i] == CODE_L2C {
					return i
				}
			case strings.Index(tobs, "P2") == 0:
				if code[i] == CODE_L2P || code[i] == CODE_L2W || code[i] == CODE_L2Y ||
					code[i] == CODE_L2N || code[i] == CODE_L2D {
					return i
				}
			case rnxver >= 212 && tobs[1] == 'A': /* L1C/A */
				if code[i] == CODE_L1C {
					return i
				}
			case rnxver >= 212 && tobs[1] == 'B': /* L1C */
				if code[i] == CODE_L1S || code[i] == CODE_L1L || code[i] == CODE_L1X {
					return i
				}
			case rnxver >= 212 && tobs[1] == 'C': /* L2C */
				if code[i] == CODE_L2S || code[i] == CODE_L2L || code[i] == CODE_L2X {
					return i
				}
			case rnxver >= 212 && tobs[1] == 'D' && sys == SYS_GLO: /* GLO L2C/A */
				if code[i] == CODE_L2C {
					return i
				}
			case tobs[1] == '2' && sys == SYS_CMP: /* BDS B1 */
				if code[i] == CODE_L2I || code[i] == CODE_L2Q || code[i] == CODE_L2X {
					return i
				}
			default:
				id = Code2Obs(code[i])
				if id[0] == tobs[1] {
					return i
				}
			}
		} else { /* ver.3 */
			id = Code2Obs(code[i])
			if strings.Compare(id, tobs[1:]) == 0 {
				return i
			}
		}
	}
	return -1
}

/* output RINEX observation data body ------------------------------------------
* output RINEX observation data body
* args   : FILE   *fp       I   output file pointer
*          rnxopt_t *opt    I   RINEX options
*          obsd_t *obs      I   observation data
*          int    n         I   number of observation data
*          int    flag      I   epoch flag (0:ok,1:power failure,>1:event flag)
* return : status (1:ok, 0:output error)
*-----------------------------------------------------------------------------*/
func OutRnxObsBody(fp *os.File, opt *RnxOpt, obs []ObsD, n, flag int) int {
	var (
		mask                string
		ep                  [6]float64
		dL                  float64
		sats                [MAXOBS]string
		i, j, k, m, ns, sys int
		ind, s              [MAXOBS]int
	)

	Trace(4, "outrnxobsb: n=%d\n", n)

	Time2Epoch(obs[0].Time, ep[:])

	for i, ns = 0, 0; i < n && ns < MAXOBS; i++ {
		sys = SatSys(obs[i].Sat, nil)
		if (sys&opt.NavSys) == 0 || opt.ExSats[obs[i].Sat-1] > 0 {
			continue
		}
		if Sat2Code(obs[i].Sat, &sats[ns]) == 0 {
			continue
		}
		switch sys {
		case SYS_GPS:
			s[ns] = 0
		case SYS_GLO:
			s[ns] = 1
		case SYS_GAL:
			s[ns] = 2
		case SYS_QZS:
			s[ns] = 3
		case SYS_SBS:
			s[ns] = 4
		case SYS_CMP:
			s[ns] = 5
		case SYS_IRN:
			s[ns] = 6
		}
		v := s[ns]
		if opt.RnxVer <= 299 {
			v = 0
		}
		if opt.NObs[v] == 0 {
			continue
		}
		ind[ns] = i
		ns++
	}
	if ns <= 0 {
		return 1
	}

	if opt.RnxVer <= 299 { /* ver.2 */
		fp.WriteString(fmt.Sprintf(" %02d %02.0f %02.0f %02.0f %02.0f %010.7f  %d%3d",
			int(ep[0])%100, ep[1], ep[2], ep[3], ep[4], ep[5], flag, ns))
		for i = 0; i < ns; i++ {
			if i > 0 && i%12 == 0 {
				fp.WriteString(fmt.Sprintf("\n%32s", ""))
			}
			fp.WriteString(fmt.Sprintf("%-3s", sats[i]))
		}
	} else { /* ver.3 */
		fp.WriteString(fmt.Sprintf("> %04.0f %02.0f %02.0f %02.0f %02.0f %010.7f  %d%3d%21s\n",
			ep[0], ep[1], ep[2], ep[3], ep[4], ep[5], flag, ns, ""))
	}
	for i = 0; i < ns; i++ {
		sys = SatSys(obs[ind[i]].Sat, nil)

		if opt.RnxVer <= 299 { /* ver.2 */
			m = 0
			mask = string(opt.Mask[s[i]][:])
		} else { /* ver.3 */
			fp.WriteString(fmt.Sprintf("%-3s", sats[i]))
			m = s[i]
			mask = string(opt.Mask[s[i]][:])
		}
		for j = 0; j < opt.NObs[m]; j++ {

			if opt.RnxVer <= 299 { /* ver.2 */
				if j%5 == 0 {
					fp.WriteString("\n")
				}
			}
			/* search obs data index */
			if k = RnxObsIndex(opt.RnxVer, sys, obs[ind[i]].Code[:], opt.TObs[m][j],
				mask); k < 0 {
				OutRnxObsf(fp, 0.0, -1)
				continue
			}
			/* phase shift (cyc) */
			dL = 0.0
			if obs[ind[i]].L[k] != 0.0 {
				dL = opt.Shift[m][j]
			}

			/* output field */
			switch opt.TObs[m][j][0] {
			case 'C', 'P':
				OutRnxObsf(fp, obs[ind[i]].P[k], -1)
			case 'L':
				OutRnxObsf(fp, obs[ind[i]].L[k]+dL, int(obs[ind[i]].LLI[k]))
			case 'D':
				OutRnxObsf(fp, obs[ind[i]].D[k], -1)
			case 'S':
				OutRnxObsf(fp, float64(obs[ind[i]].SNR[k])*float64(SNR_UNIT), -1)
			}
		}
		_, err := fp.WriteString("\n")
		if opt.RnxVer >= 300 && err == io.EOF {
			return 0
		}
	}
	if opt.RnxVer >= 300 {
		return 1
	}
	_, err := fp.WriteString("\n")

	if err != io.EOF {
		return 1
	}
	return 0
}

/* output data field in RINEX navigation data --------------------------------*/
func OutNavf_n(fp *os.File, value float64, n int) {
	e := math.Floor(math.Log10(math.Abs(value)) + 1.0)
	if math.Abs(value) < 1e-99 {
		e = 0.0
	}
	v := " "
	if value < 0.0 {
		v = "-"
	}
	fp.WriteString(fmt.Sprintf(" %s.%0*.0f%s%+03.0f", v, n,
		math.Abs(value)/math.Pow(10.0, e-float64(n)), NAVEXP, e))
}
func OutNavf(fp *os.File, value float64) {
	OutNavf_n(fp, value, 12)
}

/* output iono correction for a system ---------------------------------------*/
func OutIonoSys(fp *os.File, sys string, ion []float64, n int) {
	var (
		label1  []string = []string{"ION ALPHA", "ION BETA"}
		label2           = "IONOSPHERIC CORR"
		str     string
		i, j, k int
	)

	if Norm(ion, n) <= 0.0 {
		return
	}

	for i = 0; i < (n+3)/4; i++ {
		s := 'A' + i
		if len(sys) == 0 || n < 4 {
			s = ' '
		}
		str = fmt.Sprintf("%s%c", sys, s)
		k = 4
		if len(sys) == 0 {
			k = 1
		}
		fp.WriteString(fmt.Sprintf("%-*s ", k, str))
		for j = 0; j < 4 && i*4+j < n; j++ {
			fp.WriteString(" ")
			OutNavf_n(fp, ion[i*4+j], 4)
		}
		k = 7 + 12*(4-j)
		str = label2
		if len(sys) == 0 {
			k = 10
			str = label1[i]
		}
		fp.WriteString(fmt.Sprintf("%*s%-20s\n", k, "", str))
	}
}

/* output iono corrections --------------------------------------------------*/
func OutIono(fp *os.File, opt *RnxOpt, sys int, nav *Nav) {
	if opt.Outiono == 0 {
		return
	}

	if (sys & opt.NavSys & SYS_GPS) > 0 {
		if opt.RnxVer <= 211 {
			OutIonoSys(fp, "", nav.Ion_gps[:], 8)
		} else {
			OutIonoSys(fp, "GPS", nav.Ion_gps[:], 8)
		}
	}
	if (sys&opt.NavSys&SYS_GAL) > 0 && opt.RnxVer >= 212 {
		OutIonoSys(fp, "GAL", nav.Ion_gal[:], 3)
	}
	if (sys&opt.NavSys&SYS_QZS) > 0 && opt.RnxVer >= 302 {
		OutIonoSys(fp, "QZS", nav.Ion_qzs[:], 8)
	}
	if (sys&opt.NavSys&SYS_CMP) > 0 && opt.RnxVer >= 302 {
		OutIonoSys(fp, "BDS", nav.Ion_cmp[:], 8)
	}
	if (sys&opt.NavSys&SYS_IRN) > 0 && opt.RnxVer >= 303 {
		OutIonoSys(fp, "IRN", nav.Ion_irn[:], 8)
	}
}

/* output time system correction for a system --------------------------------*/
func OutTimeSys(fp *os.File, sys string, utc []float64) {
	var (
		label1 = "TIME SYSTEM CORR"
		label2 = "DELTA-UTC: A0,A1,T,W"
	)

	if Norm(utc, 3) <= 0.0 {
		return
	}

	if len(sys) > 0 {
		fp.WriteString(fmt.Sprintf("%-4s ", sys))
		OutNavf_n(fp, utc[0], 10)
		OutNavf_n(fp, utc[1], 9)
		fp.WriteString(fmt.Sprintf("%7.0f%5.0f%10s%-20s\n", utc[2], utc[3], "", label1))
	} else {
		fp.WriteString("   ")
		OutNavf_n(fp, utc[0], 12)
		OutNavf_n(fp, utc[1], 12)
		fp.WriteString(fmt.Sprintf("%9.0f%9.0f %-20s\n", utc[2], utc[3], label2))
	}
}

/* output time system corrections --------------------------------------------*/
func OutTime(fp *os.File, opt *RnxOpt, sys int, nav *Nav) {
	var utc [8]float64

	if opt.OutputTime == 0 {
		return
	}

	if (sys & opt.NavSys & SYS_GPS) > 0 {
		if opt.RnxVer <= 211 {
			OutTimeSys(fp, "", nav.Utc_gps[:])
		} else {
			OutTimeSys(fp, "GPUT", nav.Utc_gps[:])
		}
	}
	if (sys&opt.NavSys&SYS_GLO) > 0 && opt.RnxVer >= 212 {
		/* RINEX 2.12-3.02: tau_C, 3.03- : -tau_C */
		utc[0] = -nav.Utc_glo[0]
		if opt.RnxVer <= 302 {
			utc[0] = nav.Utc_glo[0]
		}
		OutTimeSys(fp, "GLUT", utc[:])
	}
	if (sys&opt.NavSys&SYS_SBS) > 0 && opt.RnxVer >= 212 {
		OutTimeSys(fp, "SBUT", nav.Utc_sbs[:])
	}
	if (sys&opt.NavSys&SYS_GAL) > 0 && opt.RnxVer >= 212 {
		OutTimeSys(fp, "GAUT", nav.Utc_gal[:])
	}
	if (sys&opt.NavSys&SYS_QZS) > 0 && opt.RnxVer >= 302 {
		OutTimeSys(fp, "QZUT", nav.Utc_qzs[:])
	}
	if (sys&opt.NavSys&SYS_CMP) > 0 && opt.RnxVer >= 302 {
		OutTimeSys(fp, "BDUT", nav.Utc_cmp[:])
	}
	if (sys&opt.NavSys&SYS_IRN) > 0 && opt.RnxVer >= 303 {
		OutTimeSys(fp, "IRUT", nav.Utc_irn[:])
	}
}

/* output leap seconds -------------------------------------------------------*/
func OutLeaps(fp *os.File, opt *RnxOpt, sys int, nav *Nav) {
	var (
		label = "LEAP SECONDS"
		leaps []float64
	)

	if opt.Outleaps == 0 {
		return
	}

	switch sys {
	case SYS_GAL:
		leaps = nav.Utc_gal[4:]
	case SYS_QZS:
		leaps = nav.Utc_qzs[4:]
	case SYS_CMP:
		leaps = nav.Utc_cmp[4:]
	case SYS_IRN:
		leaps = nav.Utc_irn[4:]
	default:
		leaps = nav.Utc_gps[4:]
	}
	if leaps[0] == 0.0 {
		return
	}

	s := ""
	if sys == SYS_CMP {
		s = "BDS"
	}
	switch {
	case opt.RnxVer <= 300:
		if sys == SYS_GPS {
			fp.WriteString(fmt.Sprintf("%6.0f%54s%-20s\n", leaps[0], "", label))
		}
	case Norm(leaps[1:], 3) <= 0.0:
		fp.WriteString(fmt.Sprintf("%6.0f%18s%3s%33s%-20s\n", leaps[0], "",
			s, "", label))
	default:
		fp.WriteString(fmt.Sprintf("%6.0f%6.0f%6.0f%6.0f%3s%33s%-20s\n", leaps[0],
			leaps[3], leaps[1], leaps[2], s, "", label))
	}
}

/* output RINEX navigation data file header ------------------------------------
* output RINEX navigation data file header
* args   : FILE   *fp       I   output file pointer
*          rnxopt_t *opt    I   RINEX options
*          nav_t  nav       I   navigation data (NULL: no input)
* return : status (1:ok, 0:output error)
*-----------------------------------------------------------------------------*/
func OutRnxNavHeader(fp *os.File, opt *RnxOpt, nav *Nav) int {
	var date, sys string

	Trace(4, "outrnxnavh:\n")

	TimeStrRnx(&date)

	if opt.RnxVer <= 299 { /* ver.2 */
		fp.WriteString(fmt.Sprintf("%9.2f           %-20s%-20s%-20s\n", float64(opt.RnxVer)/100.0,
			"N: GPS NAV DATA", "", "RINEX VERSION / TYPE"))
	} else { /* ver.3 */
		switch opt.NavSys {
		case SYS_GPS:
			sys = "G: GPS"
		case SYS_GLO:
			sys = "R: GLONASS"
		case SYS_GAL:
			sys = "E: Galileo"
		case SYS_QZS:
			sys = "J: QZSS" /* v.3.02 */
		case SYS_CMP:
			sys = "C: BeiDou" /* v.3.02 */
		case SYS_IRN:
			sys = "I: IRNSS" /* v.3.03 */
		case SYS_SBS:
			sys = "S: SBAS Payload"
		default:
			if opt.Sep_Nav > 0 {
				sys = "G: GPS"
			} else {
				sys = "M: Mixed"
			}
		}

		fp.WriteString(fmt.Sprintf("%9.2f           %-20s%-20s%-20s\n", float64(opt.RnxVer)/100.0,
			"N: GNSS NAV DATA", sys, "RINEX VERSION / TYPE"))
	}
	fp.WriteString(fmt.Sprintf("%-20.20s%-20.20s%-20.20s%-20s\n", opt.Prog, opt.RunBy, date,
		"PGM / RUN BY / DATE"))

	for i := 0; i < MAXCOMMENT; i++ {
		if len(opt.Comment[i]) == 0 {
			continue
		}
		fp.WriteString(fmt.Sprintf("%-60.60s%-20s\n", opt.Comment[i], "COMMENT"))
	}
	v := SYS_ALL
	if opt.Sep_Nav > 0 {
		v = SYS_GPS
	}
	OutIono(fp, opt, v, nav)
	OutTime(fp, opt, v, nav)
	OutLeaps(fp, opt, SYS_GPS, nav)
	_, err := fp.WriteString(fmt.Sprintf("%60s%-20s\n", "", "END OF HEADER"))
	if err != io.EOF {
		return 1
	}
	return 0
}

/* output RINEX navigation data file body --------------------------------------
* output RINEX navigation data file body
* args   : FILE   *fp       I   output file pointer
*          rnxopt_t *opt    I   RINEX options
*          eph_t  *eph      I   ephemeris
* return : status (1:ok, 0:output error)
*-----------------------------------------------------------------------------*/
func OutRnxNavBody(fp *os.File, opt *RnxOpt, eph *Eph) int {
	var (
		ep             [6]float64
		ttr            float64
		week, sys, prn int
		code, sep      string
	)

	Trace(4, "outrnxnavb: sat=%2d\n", eph.Sat)
	sys = SatSys(eph.Sat, &prn)
	if sys == 0 || (sys&opt.NavSys) == 0 {
		return 0
	}

	if sys != SYS_CMP {
		Time2Epoch(eph.Toc, ep[:])
	} else {
		Time2Epoch(GpsT2BDT(eph.Toc), ep[:]) /* gpst . bdt */
	}
	switch {
	case (opt.RnxVer >= 300 && sys == SYS_GPS) || (opt.RnxVer >= 212 && sys == SYS_GAL) ||
		(opt.RnxVer >= 302 && sys == SYS_QZS) || (opt.RnxVer >= 302 && sys == SYS_CMP) ||
		(opt.RnxVer >= 303 && sys == SYS_IRN):
		if Sat2Code(eph.Sat, &code) == 0 {
			return 0
		}
		fp.WriteString(fmt.Sprintf("%-3s %04.0f %02.0f %02.0f %02.0f %02.0f %02.0f", code, ep[0],
			ep[1], ep[2], ep[3], ep[4], ep[5]))
		sep = "    "
	case opt.RnxVer <= 299 && sys == SYS_GPS:
		fp.WriteString(fmt.Sprintf("%2d %02d %02.0f %02.0f %02.0f %02.0f %04.1f", prn,
			int(ep[0])%100, ep[1], ep[2], ep[3], ep[4], ep[5]))
		sep = "   "
	default:
		return 0
	}
	OutNavf(fp, eph.F0)
	OutNavf(fp, eph.F1)
	OutNavf(fp, eph.F2)
	fp.WriteString(fmt.Sprintf("\n%s", sep))

	OutNavf(fp, float64(eph.Iode)) /* GPS/QZS: IODE, GAL: IODnav, BDS: AODE */
	OutNavf(fp, eph.Crs)
	OutNavf(fp, eph.Deln)
	OutNavf(fp, eph.M0)
	fp.WriteString(fmt.Sprintf("\n%s", sep))

	OutNavf(fp, eph.Cuc)
	OutNavf(fp, eph.E)
	OutNavf(fp, eph.Cus)
	OutNavf(fp, math.Sqrt(eph.A))
	fp.WriteString(fmt.Sprintf("\n%s", sep))

	OutNavf(fp, eph.Toes)
	OutNavf(fp, eph.Cic)
	OutNavf(fp, eph.OMG0)
	OutNavf(fp, eph.Cis)
	fp.WriteString(fmt.Sprintf("\n%s", sep))

	OutNavf(fp, eph.I0)
	OutNavf(fp, eph.Crc)
	OutNavf(fp, eph.Omg)
	OutNavf(fp, eph.OMGd)
	fp.WriteString(fmt.Sprintf("\n%s", sep))

	OutNavf(fp, eph.Idot)
	OutNavf(fp, float64(eph.Code))
	OutNavf(fp, float64(eph.Week)) /* GPS/QZS: GPS week, GAL: GAL week, BDS: BDT week */
	if sys == SYS_GPS || sys == SYS_QZS {
		OutNavf(fp, float64(eph.Flag))
	} else {
		OutNavf(fp, 0.0) /* spare */
	}
	fp.WriteString(fmt.Sprintf("\n%s", sep))

	if sys == SYS_GAL {
		OutNavf(fp, SisaValue(eph.Sva))
	} else {
		OutNavf(fp, UraValue(eph.Sva))
	}
	OutNavf(fp, float64(eph.Svh))
	OutNavf(fp, eph.Tgd[0]) /* GPS/QZS:TGD, GAL:BGD E5a/E1, BDS: TGD1 B1/B3 */
	switch sys {
	case SYS_GAL, SYS_CMP:
		OutNavf(fp, eph.Tgd[1]) /* GAL:BGD E5b/E1, BDS: TGD2 B2/B3 */
	case SYS_GPS, SYS_QZS:
		OutNavf(fp, float64(eph.Iodc)) /* GPS/QZS:IODC */
	default:
		OutNavf(fp, 0.0) /* spare */
	}
	fp.WriteString(fmt.Sprintf("\n%s", sep))

	if sys != SYS_CMP {
		ttr = Time2GpsT(eph.Ttr, &week)
	} else {
		ttr = Time2BDT(GpsT2BDT(eph.Ttr), &week) /* gpst . bdt */
	}
	OutNavf(fp, ttr+float64(week-eph.Week)*604800.0)

	switch sys {
	case SYS_GPS:
		OutNavf(fp, eph.Fit)
	case SYS_QZS:
		v := 0.0
		if eph.Fit > 2.0 {
			v = 1.0
		}
		OutNavf(fp, v)
	case SYS_CMP:
		OutNavf(fp, float64(eph.Iodc)) /* AODC */
	default:
		OutNavf(fp, 0.0) /* spare */
	}
	_, err := fp.WriteString("\n")
	if err != io.EOF {
		return 1
	}
	return 0
}

/* output RINEX GNAV file header -----------------------------------------------
* output RINEX GNAV (GLONASS navigation data) file header
* args   : FILE   *fp       I   output file pointer
*          rnxopt_t *opt    I   RINEX options
*          nav_t  nav       I   navigation data (NULL: no input)
* return : status (1:ok, 0:output error)
*-----------------------------------------------------------------------------*/
func OutRnxGnavHeader(fp *os.File, opt *RnxOpt, nav *Nav) int {
	var date string

	Trace(4, "outrnxgnavh:\n")

	TimeStrRnx(&date)

	if opt.RnxVer <= 299 { /* ver.2 */
		fp.WriteString(fmt.Sprintf("%9.2f           %-20s%-20s%-20s\n", float64(opt.RnxVer)/100.0,
			"GLONASS NAV DATA", "", "RINEX VERSION / TYPE"))
	} else { /* ver.3 */
		fp.WriteString(fmt.Sprintf("%9.2f           %-20s%-20s%-20s\n", float64(opt.RnxVer)/100.0,
			"N: GNSS NAV DATA", "R: GLONASS", "RINEX VERSION / TYPE"))
	}
	fp.WriteString(fmt.Sprintf("%-20.20s%-20.20s%-20.20s%-20s\n", opt.Prog, opt.RunBy, date,
		"PGM / RUN BY / DATE"))

	for i := 0; i < MAXCOMMENT; i++ {
		if len(opt.Comment[i]) == 0 {
			continue
		}
		fp.WriteString(fmt.Sprintf("%-60.60s%-20s\n", opt.Comment[i], "COMMENT"))
	}
	OutTime(fp, opt, SYS_GLO, nav)
	OutLeaps(fp, opt, SYS_GPS, nav)

	_, err := fp.WriteString(fmt.Sprintf("%60s%-20s\n", "", "END OF HEADER"))
	if err != io.EOF {
		return 1
	}
	return 0
}

/* output RINEX GNAV file body -------------------------------------------------
* output RINEX GNAV (GLONASS navigation data) file body
* args   : FILE   *fp       I   output file pointer
*          rnxopt_t *opt    I   rinex options
*          geph_t  *geph    I   glonass ephemeris
* return : status (1:ok, 0:output error)
*-----------------------------------------------------------------------------*/
func OutRnxGnavBody(fp *os.File, opt *RnxOpt, geph *GEph) int {
	var (
		toe       Gtime
		ep        [6]float64
		tof       float64
		prn       int
		code, sep string
	)

	Trace(4, "outrnxgnavb: sat=%2d\n", geph.Sat)

	if (SatSys(geph.Sat, &prn) & opt.NavSys) != SYS_GLO {
		return 0
	}

	tof = Time2GpsT(GpsT2Utc(geph.Tof), nil) /* v.3: tow in utc */
	if opt.RnxVer <= 299 {
		tof = math.Mod(tof, 86400.0) /* v.2: tod in utc */
	}

	toe = GpsT2Utc(geph.Toe) /* gpst . utc */
	Time2Epoch(toe, ep[:])

	if opt.RnxVer <= 299 { /* ver.2 */
		fp.WriteString(fmt.Sprintf("%2d %02d %02.0f %02.0f %02.0f %02.0f %04.1f", prn,
			int(ep[0])%100, ep[1], ep[2], ep[3], ep[4], ep[5]))
		sep = "   "
	} else { /* ver.3 */
		if Sat2Code(geph.Sat, &code) == 0 {
			return 0
		}
		fp.WriteString(fmt.Sprintf("%-3s %04.0f %02.0f %02.0f %02.0f %02.0f %02.0f", code, ep[0],
			ep[1], ep[2], ep[3], ep[4], ep[5]))
		sep = "    "
	}
	OutNavf(fp, -geph.Taun)
	OutNavf(fp, geph.Gamn)
	OutNavf(fp, tof)
	fp.WriteString(fmt.Sprintf("\n%s", sep))

	OutNavf(fp, geph.Pos[0]/1e3)
	OutNavf(fp, geph.Vel[0]/1e3)
	OutNavf(fp, geph.Acc[0]/1e3)
	OutNavf(fp, float64(geph.Svh))
	fp.WriteString(fmt.Sprintf("\n%s", sep))

	OutNavf(fp, geph.Pos[1]/1e3)
	OutNavf(fp, geph.Vel[1]/1e3)
	OutNavf(fp, geph.Acc[1]/1e3)
	OutNavf(fp, float64(geph.Frq))
	fp.WriteString(fmt.Sprintf("\n%s", sep))

	OutNavf(fp, geph.Pos[2]/1e3)
	OutNavf(fp, geph.Vel[2]/1e3)
	OutNavf(fp, geph.Acc[2]/1e3)

	OutNavf(fp, float64(geph.Age))
	_, err := fp.WriteString("\n")
	if err != io.EOF {
		return 1
	}
	return 0
}

/* output RINEX GEO navigation data file header --------------------------------
* output RINEX GEO navigation data file header
* args   : FILE   *fp       I   output file pointer
*          rnxopt_t *opt    I   RINEX options
*          nav_t  nav       I   navigation data (NULL: no input)
* return : status (1:ok, 0:output error)
*-----------------------------------------------------------------------------*/
func OutRnxHnavHeader(fp *os.File, opt *RnxOpt, nav *Nav) int {
	var date string

	Trace(4, "outrnxhnavh:\n")

	TimeStrRnx(&date)

	if opt.RnxVer <= 299 { /* ver.2 */
		fp.WriteString(fmt.Sprintf("%9.2f           %-20s%-20s%-20s\n", float64(opt.RnxVer)/100.0,
			"H: GEO NAV MSG DATA", "", "RINEX VERSION / TYPE"))
	} else { /* ver.3 */
		fp.WriteString(fmt.Sprintf("%9.2f           %-20s%-20s%-20s\n", float64(opt.RnxVer)/100.0,
			"N: GNSS NAV DATA", "S: SBAS Payload", "RINEX VERSION / TYPE"))
	}
	fp.WriteString(fmt.Sprintf("%-20.20s%-20.20s%-20.20s%-20s\n", opt.Prog, opt.RunBy, date,
		"PGM / RUN BY / DATE"))

	for i := 0; i < MAXCOMMENT; i++ {
		if len(opt.Comment[i]) == 0 {
			continue
		}
		fp.WriteString(fmt.Sprintf("%-60.60s%-20s\n", opt.Comment[i], "COMMENT"))
	}
	OutTime(fp, opt, SYS_SBS, nav)
	OutLeaps(fp, opt, SYS_GPS, nav)
	_, err := fp.WriteString(fmt.Sprintf("%60s%-20s\n", "", "END OF HEADER"))
	if err != io.EOF {
		return 1
	}
	return 0
}

/* output RINEX GEO navigation data file body ----------------------------------
* output RINEX GEO navigation data file body
* args   : FILE   *fp       I   output file pointer
*          rnxopt_t *opt    I   RINEX options
*          seph_t  *seph    I   SBAS ephemeris
* return : status (1:ok, 0:output error)
*-----------------------------------------------------------------------------*/
func OutRnxHnavBody(fp *os.File, opt *RnxOpt, seph *SEph) int {
	var (
		ep        [6]float64
		prn       int
		code, sep string
	)

	Trace(4, "outrnxhnavb: sat=%2d\n", seph.Sat)

	if (SatSys(seph.Sat, &prn) & opt.NavSys) != SYS_SBS {
		return 0
	}

	Time2Epoch(seph.T0, ep[:])

	if opt.RnxVer <= 299 { /* ver.2 */
		fp.WriteString(fmt.Sprintf("%2d %02d %2.0f %2.0f %2.0f %2.0f %4.1f", prn-100,
			int(ep[0])%100, ep[1], ep[2], ep[3], ep[4], ep[5]))
		sep = "   "
	} else { /* ver.3 */
		if Sat2Code(seph.Sat, &code) == 0 {
			return 0
		}
		fp.WriteString(fmt.Sprintf("%-3s %04.0f %2.0f %2.0f %2.0f %2.0f %2.0f", code, ep[0], ep[1],
			ep[2], ep[3], ep[4], ep[5]))
		sep = "    "
	}
	OutNavf(fp, seph.Af0)
	OutNavf(fp, seph.Af1)
	OutNavf(fp, Time2GpsT(seph.Tof, nil))
	fp.WriteString(fmt.Sprintf("\n%s", sep))

	OutNavf(fp, seph.Pos[0]/1e3)
	OutNavf(fp, seph.Vel[0]/1e3)
	OutNavf(fp, seph.Acc[0]/1e3)
	OutNavf(fp, float64(seph.Svh))
	fp.WriteString(fmt.Sprintf("\n%s", sep))

	OutNavf(fp, seph.Pos[1]/1e3)
	OutNavf(fp, seph.Vel[1]/1e3)
	OutNavf(fp, seph.Acc[1]/1e3)
	OutNavf(fp, UraValue(seph.Sva))
	fp.WriteString(fmt.Sprintf("\n%s", sep))

	OutNavf(fp, seph.Pos[2]/1e3)
	OutNavf(fp, seph.Vel[2]/1e3)
	OutNavf(fp, seph.Acc[2]/1e3)
	OutNavf(fp, 0)
	_, err := fp.WriteString("\n")
	if err != io.EOF {
		return 1
	}
	return 0
}

/* output RINEX Galileo NAV header ---------------------------------------------
* output RINEX Galileo NAV file header (2.12)
* args   : FILE   *fp       I   output file pointer
*          rnxopt_t *opt    I   RINEX options
*          nav_t  nav       I   navigation data (NULL: no input)
* return : status (1:ok, 0:output error)
*-----------------------------------------------------------------------------*/
func OutRnxLnavHeader(fp *os.File, opt *RnxOpt, nav *Nav) int {
	var date string

	Trace(4, "outrnxlnavh:\n")

	if opt.RnxVer < 212 {
		return 0
	}

	TimeStrRnx(&date)

	fp.WriteString(fmt.Sprintf("%9.2f           %-20s%-20s%-20s\n", float64(opt.RnxVer)/100.0,
		"N: GNSS NAV DATA", "E: Galileo", "RINEX VERSION / TYPE"))

	fp.WriteString(fmt.Sprintf("%-20.20s%-20.20s%-20.20s%-20s\n", opt.Prog, opt.RunBy, date,
		"PGM / RUN BY / DATE"))

	for i := 0; i < MAXCOMMENT; i++ {
		if len(opt.Comment[i]) == 0 {
			continue
		}
		fp.WriteString(fmt.Sprintf("%-60.60s%-20s\n", opt.Comment[i], "COMMENT"))
	}
	OutIono(fp, opt, SYS_GAL, nav)
	OutTime(fp, opt, SYS_GAL, nav)
	OutLeaps(fp, opt, SYS_GAL, nav)
	_, err := fp.WriteString(fmt.Sprintf("%60s%-20s\n", "", "END OF HEADER"))
	if err != io.EOF {
		return 1
	}
	return 0
}

/* output RINEX QZSS navigation data file header -------------------------------
* output RINEX QZSS navigation data file header
* args   : FILE   *fp       I   output file pointer
*          rnxopt_t *opt    I   RINEX options
*          nav_t  nav       I   navigation data (NULL: no input)
* return : status (1:ok, 0:output error)
*-----------------------------------------------------------------------------*/
func OutRnxQnavHeader(fp *os.File, opt *RnxOpt, nav *Nav) int {

	var date string

	Trace(4, "outrnxqnavh:\n")

	if opt.RnxVer < 302 {
		return 0
	}

	TimeStrRnx(&date)

	fp.WriteString(fmt.Sprintf("%9.2f           %-20s%-20s%-20s\n", float64(opt.RnxVer)/100.0,
		"N: GNSS NAV DATA", "J: QZSS", "RINEX VERSION / TYPE"))

	fp.WriteString(fmt.Sprintf("%-20.20s%-20.20s%-20.20s%-20s\n", opt.Prog, opt.RunBy, date,
		"PGM / RUN BY / DATE"))

	for i := 0; i < MAXCOMMENT; i++ {
		if len(opt.Comment[i]) == 0 {
			continue
		}
		fp.WriteString(fmt.Sprintf("%-60.60s%-20s\n", opt.Comment[i], "COMMENT"))
	}
	OutIono(fp, opt, SYS_QZS, nav)
	OutTime(fp, opt, SYS_QZS, nav)
	OutLeaps(fp, opt, SYS_QZS, nav)
	_, err := fp.WriteString(fmt.Sprintf("%60s%-20s\n", "", "END OF HEADER"))
	if err != io.EOF {
		return 1
	}
	return 0
}

/* output RINEX BDS navigation data file header --------------------------------
* output RINEX BDS navigation data file header
* args   : FILE   *fp       I   output file pointer
*          rnxopt_t *opt    I   RINEX options
*          nav_t  nav       I   navigation data (NULL: no input)
* return : status (1:ok, 0:output error)
*-----------------------------------------------------------------------------*/
func OutRnxCnavHeader(fp *os.File, opt *RnxOpt, nav *Nav) int {

	var date string

	Trace(4, "outrnxcnavh:\n")

	if opt.RnxVer < 302 {
		return 0
	}

	TimeStrRnx(&date)

	fp.WriteString(fmt.Sprintf("%9.2f           %-20s%-20s%-20s\n", float64(opt.RnxVer)/100.0,
		"N: GNSS NAV DATA", "C: BeiDou", "RINEX VERSION / TYPE"))

	fp.WriteString(fmt.Sprintf("%-20.20s%-20.20s%-20.20s%-20s\n", opt.Prog, opt.RunBy, date,
		"PGM / RUN BY / DATE"))

	for i := 0; i < MAXCOMMENT; i++ {
		if len(opt.Comment[i]) == 0 {
			continue
		}
		fp.WriteString(fmt.Sprintf("%-60.60s%-20s\n", opt.Comment[i], "COMMENT"))
	}
	OutIono(fp, opt, SYS_CMP, nav)
	OutTime(fp, opt, SYS_CMP, nav)
	OutLeaps(fp, opt, SYS_CMP, nav)
	_, err := fp.WriteString(fmt.Sprintf("%60s%-20s\n", "", "END OF HEADER"))
	if err != io.EOF {
		return 1
	}
	return 0
}

/* output RINEX NavIC/IRNSS navigation data file header ------------------------
* output RINEX NavIC/IRNSS navigation data file header
* args   : FILE   *fp       I   output file pointer
*          rnxopt_t *opt    I   RINEX options
*          nav_t  nav       I   navigation data (NULL: no input)
* return : status (1:ok, 0:output error)
*-----------------------------------------------------------------------------*/
func OutRnxInavHeader(fp *os.File, opt *RnxOpt, nav *Nav) int {
	var date string

	Trace(4, "outrnxinavh:\n")

	if opt.RnxVer < 303 {
		return 0
	}

	TimeStrRnx(&date)

	fp.WriteString(fmt.Sprintf("%9.2f           %-20s%-20s%-20s\n", float64(opt.RnxVer)/100.0,
		"N: GNSS NAV DATA", "I: IRNSS", "RINEX VERSION / TYPE"))

	fp.WriteString(fmt.Sprintf("%-20.20s%-20.20s%-20.20s%-20s\n", opt.Prog, opt.RunBy, date,
		"PGM / RUN BY / DATE"))

	for i := 0; i < MAXCOMMENT; i++ {
		if len(opt.Comment[i]) == 0 {
			continue
		}
		fp.WriteString(fmt.Sprintf("%-60.60s%-20s\n", opt.Comment[i], "COMMENT"))
	}
	OutIono(fp, opt, SYS_IRN, nav)
	OutTime(fp, opt, SYS_IRN, nav)
	OutLeaps(fp, opt, SYS_IRN, nav)
	_, err := fp.WriteString(fmt.Sprintf("%60s%-20s\n", "", "END OF HEADER"))
	if err != io.EOF {
		return 1
	}
	return 0
}
