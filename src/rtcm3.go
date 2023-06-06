/*------------------------------------------------------------------------------
* rtcm3.c : RTCM ver.3 message decorder functions
*
*          Copyright (C) 2009-2020 by T.TAKASU, All rights reserved.
*
* references :
*     see rtcm.c
*
* version : $Revision:$ $Date:$
* history : 2012/05/14 1.0  separated from rtcm.c
*           2012/12/12 1.1  support gal/qzs ephemeris, gal/qzs ssr, msm
*                           add station id consistency test for obs data
*           2012/12/25 1.2  change compass msm id table
*           2013/01/31 1.3  change signal id by the latest draft (ref [13])
*           2013/02/23 1.4  change reference for rtcm 3 message (ref [14])
*           2013/05/19 1.5  gpst . bdt of time-tag in beidou msm message
*           2014/05/02 1.6  fix bug on dropping last field of ssr message
*                           comply with rtcm 3.2 with amendment 1/2 (ref[15])
*                           delete MT 1046 according to ref [15]
*           2014/09/14 1.7  add receiver option -RT_INP
*           2014/12/06 1.8  support SBAS/BeiDou SSR messages (ref [16])
*           2015/03/22 1.9  add handling of iodcrc for beidou/sbas ssr messages
*           2015/04/27 1.10 support phase bias messages (MT2065-2070)
*           2015/09/07 1.11 add message count of MT 2000-2099
*           2015/10/21 1.12 add MT1046 support for IGS MGEX
*                           fix bug on decode of SSR 3/7 (code/phase bias)
*           2015/12/04 1.13 add MT63 beidou ephemeris (rtcm draft)
*                           fix bug on ssr 3 message decoding (#321)
*           2016/01/22 1.14 fix bug on L2C code in MT1004 (#131)
*           2016/08/20 1.15 fix bug on loss-of-lock detection in MSM 6/7 (#134)
*           2016/09/20 1.16 fix bug on MT1045 Galileo week rollover
*           2016/10/09 1.17 support MT1029 unicode text string
*           2017/04/11 1.18 fix bug on unchange-test of beidou ephemeris
*                           fix bug on week number in galileo ephemeris struct
*           2018/10/10 1.19 merge changes for 2.4.2 p13
*                           fix problem on eph.code for galileo ephemeris
*                           change mt for ssr 7 phase biases
*                           add rtcm option -GALINAV, -GALFNAV
*           2018/11/05 1.20 fix problem on invalid time in message monitor
*           2019/05/10 1.21 save galileo E5b data to obs index 2
*           2020/11/30 1.22 support MT1230 GLONASS code-phase biases
*                           support MT1131-1137,1041 (NavIC MSM and ephemeris)
*                           support MT4076 IGS SSR
*                           update MSM signal ID table (ref [17])
*                           update SSR signal and tracking mode ID table
*                           add week adjustment in MT1019,1044,1045,1046,1042
*                           use API code2idx() to get freq-index
*                           use API code2freq() to get carrier frequency
*                           use integer types in stdint.h
*		    2022/05/31 1.0  rewrite rtcm3.c with golang by fxb
*-----------------------------------------------------------------------------*/

package gnssgo

import (
	"fmt"
	"math"
	"strings"
)

const (
	PRUNIT_GPS = 299792.458 /* rtcm ver.3 unit of gps pseudorange (m) */
	PRUNIT_GLO = 599584.916 /* rtcm ver.3 unit of glonass pseudorange (m) */
	RANGE_MS   = (CLIGHT * 0.001) /* range in 1 ms */)

/* type definition -----------------------------------------------------------*/

type Msm_h struct { /* multi-signal-message header type */
	iod        uint8     /* issue of data station */
	time_s     uint8     /* cumulative session transmitting time */
	clk_str    uint8     /* clock steering indicator */
	clk_ext    uint8     /* external clock indicator */
	smooth     uint8     /* divergence free smoothing indicator */
	tint_s     uint8     /* soothing interval */
	nsat, nsig uint8     /* number of satellites/signals */
	sats       [64]uint8 /* satellites */
	sigs       [32]uint8 /* signals */
	cellmask   [64]uint8 /* cell mask */
}

/* MSM signal ID table -------------------------------------------------------*/
var (
	msm_sig_gps [32]string = [32]string{
		/* GPS: ref [17] table 3.5-91 */
		"", "1C", "1P", "1W", "", "", "", "2C", "2P", "2W", "", "", /*  1-12 */
		"", "", "2S", "2L", "2X", "", "", "", "", "5I", "5Q", "5X", /* 13-24 */
		"", "", "", "", "", "1S", "1L", "1X" /* 25-32 */}
	msm_sig_glo [32]string = [32]string{
		/* GLONASS: ref [17] table 3.5-96 */
		"", "1C", "1P", "", "", "", "", "2C", "2P", "", "", "",
		"", "", "", "", "", "", "", "", "", "", "", "",
		"", "", "", "", "", "", "", ""}
	msm_sig_gal [32]string = [32]string{
		/* Galileo: ref [17] table 3.5-99 */
		"", "1C", "1A", "1B", "1X", "1Z", "", "6C", "6A", "6B", "6X", "6Z",
		"", "7I", "7Q", "7X", "", "8I", "8Q", "8X", "", "5I", "5Q", "5X",
		"", "", "", "", "", "", "", ""}
	msm_sig_qzs [32]string = [32]string{
		/* QZSS: ref [17] table 3.5-105 */
		"", "1C", "", "", "", "", "", "", "6S", "6L", "6X", "",
		"", "", "2S", "2L", "2X", "", "", "", "", "5I", "5Q", "5X",
		"", "", "", "", "", "1S", "1L", "1X"}
	msm_sig_sbs [32]string = [32]string{
		/* SBAS: ref [17] table 3.5-102 */
		"", "1C", "", "", "", "", "", "", "", "", "", "",
		"", "", "", "", "", "", "", "", "", "5I", "5Q", "5X",
		"", "", "", "", "", "", "", ""}

	//Support B1C B2a Signal by cjb 2021-12-24
	//const char *msm_sig_cmp[32]={
	//	/* BeiDou: ref [17] table 3.5-108 */
	//	""  ,"2I","2Q","2X",""  ,""  ,""  ,"6I","6Q","6X",""  ,""  ,
	//	""  ,"7I","7Q","7X",""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,
	//	""  ,""  ,""  ,""  ,""  ,""  ,""  ,""
	//};5D 5P 5X 1D 1P 1X
	//RTCM MSM数据中北斗的signal mask从0~31，B2a data的mask为21，B2a pilot的mask为22，B2a data+pilot的mask为23，B1C data的mask为29，B1C pilot的mask为30，B1C data+pilot的mask为31
	msm_sig_cmp [32]string = [32]string{
		/* BeiDou: ref [17] table 3.5-108 */
		"", "2I", "2Q", "2X", "", "", "", "6I", "6Q", "6X", "", "",
		"", "7I", "7Q", "7X", "", "", "", "", "", "5D", "5P", "5X",
		"", "", "", "", "", "1D", "1P", "1X"}
	msm_sig_irn [32]string = [32]string{
		/* NavIC/IRNSS: ref [17] table 3.5-108.3 */
		"", "", "", "", "", "", "", "", "", "", "", "",
		"", "", "", "", "", "", "", "", "", "5A", "", "",
		"", "", "", "", "", "", "", ""}

	/* SSR signal and tracking mode IDs ------------------------------------------*/
	ssr_sig_gps [32]uint8 = [32]uint8{
		CODE_L1C, CODE_L1P, CODE_L1W, CODE_L1S, CODE_L1L, CODE_L2C, CODE_L2D, CODE_L2S,
		CODE_L2L, CODE_L2X, CODE_L2P, CODE_L2W, 0, 0, CODE_L5I, CODE_L5Q}
	ssr_sig_glo [32]uint8 = [32]uint8{
		CODE_L1C, CODE_L1P, CODE_L2C, CODE_L2P, CODE_L4A, CODE_L4B, CODE_L6A, CODE_L6B,
		CODE_L3I, CODE_L3Q}
	ssr_sig_gal [32]uint8 = [32]uint8{
		CODE_L1A, CODE_L1B, CODE_L1C, 0, 0, CODE_L5I, CODE_L5Q, 0,
		CODE_L7I, CODE_L7Q, 0, CODE_L8I, CODE_L8Q, 0, CODE_L6A, CODE_L6B,
		CODE_L6C}
	ssr_sig_qzs [32]uint8 = [32]uint8{
		CODE_L1C, CODE_L1S, CODE_L1L, CODE_L2S, CODE_L2L, 0, CODE_L5I, CODE_L5Q,
		0, CODE_L6S, CODE_L6L, 0, 0, 0, 0, 0,
		0, CODE_L6E}
	ssr_sig_cmp [32]uint8 = [32]uint8{
		CODE_L2I, CODE_L2Q, 0, CODE_L6I, CODE_L6Q, 0, CODE_L7I, CODE_L7Q,
		0, CODE_L1D, CODE_L1P, 0, CODE_L5D, CODE_L5P, 0, CODE_L1A,
		0, 0, CODE_L6A}
	ssr_sig_sbs [32]uint8 = [32]uint8{
		CODE_L1C, CODE_L5I, CODE_L5Q}

	/* SSR update intervals ------------------------------------------------------*/
	ssrudint [16]float64 = [16]float64{
		1, 2, 5, 10, 15, 30, 60, 120, 240, 300, 600, 900, 1800, 3600, 7200, 10800}
)

func selectsys(sys int) (int, int, int, int, []uint8, bool) {
	var (
		np, ni, nj, offp int
		sigs             []uint8
		miss             bool = false
	)
	switch sys {
	case SYS_GPS:
		np, ni, nj, offp = 6, 8, 0, 0
		sigs = ssr_sig_gps[:]
	case SYS_GLO:
		np, ni, nj, offp = 5, 8, 0, 0
		sigs = ssr_sig_glo[:]
	case SYS_GAL:
		np, ni, nj, offp = 6, 10, 0, 0
		sigs = ssr_sig_gal[:]
	case SYS_QZS:
		np, ni, nj, offp = 4, 8, 0, 192
		sigs = ssr_sig_qzs[:]
	case SYS_CMP:
		np, ni, nj, offp = 6, 10, 24, 1
		sigs = ssr_sig_cmp[:]
	case SYS_SBS:
		np, ni, nj, offp = 6, 9, 24, 120
		sigs = ssr_sig_sbs[:]
	default:
		miss = true
	}
	return np, ni, nj, offp, sigs, miss
}

/* adjust weekly rollover of GPS time ----------------------------------------*/
func (rtcm *Rtcm) AdjWeek(tow float64) {
	var (
		tow_p float64
		week  int
	)

	/* if no time, get cpu time */
	if rtcm.Time.Time == 0 {
		rtcm.Time = Utc2GpsT(TimeGet())
	}
	tow_p = Time2GpsT(rtcm.Time, &week)
	if tow < tow_p-302400.0 {
		tow += 604800.0
	} else if tow > tow_p+302400.0 {
		tow -= 604800.0
	}
	rtcm.Time = GpsT2Time(week, tow)
}

/* adjust weekly rollover of BDS time ----------------------------------------*/
func AdjBDTWeek(week int) int {
	var w int
	Time2BDT(GpsT2BDT(Utc2GpsT(TimeGet())), &w)
	if w < 1 {
		w = 1 /* use 2006/1/1 if time is earlier than 2006/1/1 */
	}
	return week + (w-week+512)/1024*1024
}

/* adjust daily rollover of GLONASS time -------------------------------------*/
func (rtcm *Rtcm) AdjDay_Glot(tod float64) {
	var (
		time       Gtime
		tow, tod_p float64
		week       int
	)

	if rtcm.Time.Time == 0 {
		rtcm.Time = Utc2GpsT(TimeGet())
	}
	time = TimeAdd(GpsT2Utc(rtcm.Time), 10800.0) /* glonass time */
	tow = Time2GpsT(time, &week)
	tod_p = math.Mod(tow, 86400.0)
	tow -= tod_p
	if tod < tod_p-43200.0 {
		tod += 86400.0
	} else if tod > tod_p+43200.0 {
		tod -= 86400.0
	}
	time = GpsT2Time(week, tow+tod)
	rtcm.Time = Utc2GpsT(TimeAdd(time, -10800.0))
}

/* adjust carrier-phase rollover ---------------------------------------------*/
func (rtcm *Rtcm) AdjCP(sat, idx int, cp float64) float64 {
	switch {
	case rtcm.Cp[sat-1][idx] == 0.0:
	case cp < rtcm.Cp[sat-1][idx]-750.0:
		cp += 1500.0
	case cp > rtcm.Cp[sat-1][idx]+750.0:
		cp -= 1500.0
	}
	rtcm.Cp[sat-1][idx] = cp
	return cp
}

/* loss-of-lock indicator ----------------------------------------------------*/
func (rtcm *Rtcm) LossOfLock(sat, idx, lock int) int {
	var lli int = 0
	if (lock == 0 && rtcm.Lock[sat-1][idx] == 0) || lock < int(rtcm.Lock[sat-1][idx]) {
		lli = 1
	}
	rtcm.Lock[sat-1][idx] = uint16(lock)
	return lli
}

/* S/N ratio -----------------------------------------------------------------*/
func SnRatio(snr float64) uint16 {
	if snr <= 0.0 || 100.0 <= snr {
		return 0
	} else {
		return uint16(snr/float64(SNR_UNIT) + 0.5)
	}

}

/* test station ID consistency -----------------------------------------------*/
func (rtcm *Rtcm) test_staid(staid int) int {
	var ctype, id int

	/* test station id option */
	if index := strings.Index(rtcm.Opt, "-STA="); index >= 0 {
		n, _ := fmt.Sscanf(rtcm.Opt[index:], "-STA=%d", &id)
		if n == 1 && staid != id {
			return 0
		}
	}

	/* save station id */
	if rtcm.StaId == 0 || rtcm.ObsFlag > 0 {
		rtcm.StaId = staid
	} else if staid != rtcm.StaId {
		ctype = int(GetBitU(rtcm.Buff[:], 24, 12))
		Trace(2, "rtcm3 %d staid invalid id=%d %d\n", ctype, staid, rtcm.StaId)

		/* reset station id if station id error */
		rtcm.StaId = 0
		return 0
	}
	return 1
}

/* decode type 1001-1004 message header --------------------------------------*/
func (rtcm *Rtcm) decode_head1001(sync *int) int {
	var (
		tow                float64
		tstr               string
		i                  int = 24
		staid, nsat, ctype int
	)

	ctype = int(GetBitU(rtcm.Buff[:], i, 12))
	i += 12

	if i+52 <= rtcm.MsgLen*8 {
		staid = int(GetBitU(rtcm.Buff[:], i, 12))
		i += 12
		tow = float64(GetBitU(rtcm.Buff[:], i, 30)) * 0.001
		i += 30
		*sync = int(GetBitU(rtcm.Buff[:], i, 1))
		i += 1
		nsat = int(GetBitU(rtcm.Buff[:], i, 5))
	} else {
		Trace(2, "rtcm3 %d length error: len=%d\n", ctype, rtcm.MsgLen)
		return -1
	}
	/* test station ID */
	if rtcm.test_staid(staid) == 0 {
		return -1
	}

	rtcm.AdjWeek(tow)

	Time2Str(rtcm.Time, &tstr, 2)
	Trace(5, "decode_head1001: time=%s nsat=%d sync=%d\n", tstr, nsat, *sync)

	if rtcm.OutType > 0 {
		rtcm.MsgType += fmt.Sprintf(" staid=%4d %s nsat=%2d sync=%d", staid, tstr, nsat, *sync)
	}
	return nsat
}

/* decode type 1001: L1-only GPS RTK observation -----------------------------*/
func (rtcm *Rtcm) decode_type1001() int {
	var sync int
	if rtcm.decode_head1001(&sync) < 0 {
		return -1
	}
	return retsync(sync, &rtcm.ObsFlag)
}

/* decode type 1002: extended L1-only GPS RTK observables --------------------*/
func (rtcm *Rtcm) decode_type1002() int {
	var (
		pr1, cnr1, tt, cp1               float64
		i, j, index, nsat, sync, prn     int
		code, sat, ppr1, lock1, amb, sys int
	)
	freq := FREQ1
	i = 24 + 64
	if nsat = rtcm.decode_head1001(&sync); nsat < 0 {
		return -1
	}

	for j = 0; j < nsat && rtcm.ObsData.N() < MAXOBS+1 && i+74 <= rtcm.MsgLen*8; j++ {
		prn = int(GetBitU(rtcm.Buff[:], i, 6))
		i += 6
		code = int(GetBitU(rtcm.Buff[:], i, 1))
		i += 1
		pr1 = float64(GetBitU(rtcm.Buff[:], i, 24))
		i += 24
		ppr1 = int(GetBits(rtcm.Buff[:], i, 20))
		i += 20
		lock1 = int(GetBitU(rtcm.Buff[:], i, 7))
		i += 7
		amb = int(GetBitU(rtcm.Buff[:], i, 8))
		i += 8
		cnr1 = float64(GetBitU(rtcm.Buff[:], i, 8))
		i += 8
		if prn < 40 {
			sys = SYS_GPS
		} else {
			sys = SYS_SBS
			prn += 80
		}
		if sat = SatNo(sys, prn); sat == 0 {
			Trace(2, "rtcm3 1002 satellite number error: prn=%d\n", prn)
			continue
		}
		tt = TimeDiff(rtcm.ObsData.Data[0].Time, rtcm.Time)
		if rtcm.ObsFlag > 0 || math.Abs(tt) > 1e-9 {
			rtcm.ObsData.Data, rtcm.ObsFlag = nil, 0
		}
		if index = rtcm.ObsData.ObsIndex(rtcm.Time, sat); index < 0 {
			continue
		}
		pr1 = pr1*0.02 + float64(amb)*PRUNIT_GPS
		rtcm.ObsData.Data[index].P[0] = pr1

		if ppr1 != int(0xFFF80000) {
			cp1 = rtcm.AdjCP(sat, 0, float64(ppr1)*0.0005*freq/CLIGHT)
			rtcm.ObsData.Data[index].L[0] = pr1*freq/CLIGHT + cp1
		}
		rtcm.ObsData.Data[index].LLI[0] = uint8(rtcm.LossOfLock(sat, 0, lock1))
		rtcm.ObsData.Data[index].SNR[0] = SnRatio(cnr1 * 0.25)
		if code > 0 {
			rtcm.ObsData.Data[index].Code[0] = CODE_L1P
		} else {
			rtcm.ObsData.Data[index].Code[0] = CODE_L1C
		}
	}

	return retsync(sync, &i)
}

/* decode type 1003: L1&L2 gps rtk observables -------------------------------*/
func (rtcm *Rtcm) decode_type1003() int {
	var sync int
	if rtcm.decode_head1001(&sync) < 0 {
		return -1
	}
	return retsync(sync, &rtcm.ObsFlag)
}

/* decode type 1004: extended L1&L2 GPS RTK observables ----------------------*/
func (rtcm *Rtcm) decode_type1004() int {
	var (
		L2codes                           []byte = []byte{CODE_L2X, CODE_L2P, CODE_L2D, CODE_L2W}
		pr1, cnr1, cnr2, tt, cp1, cp2     float64
		freq                              [2]float64 = [2]float64{FREQ1, FREQ2}
		i, j, index, nsat, sync, prn, sat int
		code1, code2, pr21, ppr1, ppr2    int
		lock1, lock2, amb, sys            int
	)
	i = 24 + 64
	if nsat = rtcm.decode_head1001(&sync); nsat < 0 {
		return -1
	}

	for j = 0; j < nsat && rtcm.ObsData.N() < MAXOBS+1 && i+125 <= rtcm.MsgLen*8; j++ {
		prn = int(GetBitU(rtcm.Buff[:], i, 6))
		i += 6
		code1 = int(GetBitU(rtcm.Buff[:], i, 1))
		i += 1
		pr1 = float64(GetBitU(rtcm.Buff[:], i, 24))
		i += 24
		ppr1 = int(GetBits(rtcm.Buff[:], i, 20))
		i += 20
		lock1 = int(GetBitU(rtcm.Buff[:], i, 7))
		i += 7
		amb = int(GetBitU(rtcm.Buff[:], i, 8))
		i += 8
		cnr1 = float64(GetBitU(rtcm.Buff[:], i, 8))
		i += 8
		code2 = int(GetBitU(rtcm.Buff[:], i, 2))
		i += 2
		pr21 = int(GetBits(rtcm.Buff[:], i, 14))
		i += 14
		ppr2 = int(GetBits(rtcm.Buff[:], i, 20))
		i += 20
		lock2 = int(GetBitU(rtcm.Buff[:], i, 7))
		i += 7
		cnr2 = float64(GetBitU(rtcm.Buff[:], i, 8))
		i += 8
		if prn < 40 {
			sys = SYS_GPS
		} else {
			sys = SYS_SBS
			prn += 80
		}
		if sat = SatNo(sys, prn); sat == 0 {
			Trace(2, "rtcm3 1004 satellite number error: sys=%d prn=%d\n", sys, prn)
			continue
		}
		tt = TimeDiff(rtcm.ObsData.Data[0].Time, rtcm.Time)
		if rtcm.ObsFlag > 0 || math.Abs(tt) > 1e-9 {
			rtcm.ObsData.Data, rtcm.ObsFlag = nil, 0
		}
		if index = rtcm.ObsData.ObsIndex(rtcm.Time, sat); index < 0 {
			continue
		}
		pr1 = pr1*0.02 + float64(amb)*PRUNIT_GPS
		rtcm.ObsData.Data[index].P[0] = pr1

		if ppr1 != int(0xFFF80000) {
			cp1 = rtcm.AdjCP(sat, 0, float64(ppr1)*0.0005*freq[0]/CLIGHT)
			rtcm.ObsData.Data[index].L[0] = pr1*freq[0]/CLIGHT + cp1
		}
		rtcm.ObsData.Data[index].LLI[0] = uint8(rtcm.LossOfLock(sat, 0, lock1))
		rtcm.ObsData.Data[index].SNR[0] = SnRatio(cnr1 * 0.25)
		if code1 > 0 {
			rtcm.ObsData.Data[index].Code[0] = CODE_L1P
		} else {
			rtcm.ObsData.Data[index].Code[0] = CODE_L1C
		}

		if pr21 != int(0xFFFFE000) {
			rtcm.ObsData.Data[index].P[1] = pr1 + float64(pr21)*0.02
		}
		if ppr2 != int(0xFFF80000) {
			cp2 = rtcm.AdjCP(sat, 1, float64(ppr2)*0.0005*freq[1]/CLIGHT)
			rtcm.ObsData.Data[index].L[1] = pr1*freq[1]/CLIGHT + cp2
		}
		rtcm.ObsData.Data[index].LLI[1] = uint8(rtcm.LossOfLock(sat, 1, lock2))
		rtcm.ObsData.Data[index].SNR[1] = SnRatio(cnr2 * 0.25)
		rtcm.ObsData.Data[index].Code[1] = L2codes[code2]
	}
	return retsync(sync, &rtcm.ObsFlag)
}

/* get signed 38bit field ----------------------------------------------------*/
func getbits_38(buff []uint8, pos int) float64 {
	return float64(GetBits(buff, pos, 32))*64.0 + float64(GetBitU(buff, pos+32, 6))
}

/* decode type 1005: stationary RTK reference station ARP --------------------*/
func (rtcm *Rtcm) decode_type1005() int {
	var (
		rr, re, pos    [3]float64
		i              int = 24 + 12
		j, staid, itrf int
	)

	if i+140 == rtcm.MsgLen*8 {
		staid = int(GetBitU(rtcm.Buff[:], i, 12))
		i += 12
		itrf = int(GetBitU(rtcm.Buff[:], i, 6))
		i += 6 + 4
		rr[0] = getbits_38(rtcm.Buff[:], i)
		i += 38 + 2
		rr[1] = getbits_38(rtcm.Buff[:], i)
		i += 38 + 2
		rr[2] = getbits_38(rtcm.Buff[:], i)
	} else {
		Trace(2, "rtcm3 1005 length error: len=%d\n", rtcm.MsgLen)
		return -1
	}
	if rtcm.OutType > 0 {
		// msg=rtcm.msgtype+strlen(rtcm.msgtype);
		for j = 0; j < 3; j++ {
			re[j] = rr[j] * 0.0001
		}
		Ecef2Pos(re[:], pos[:])
		rtcm.MsgType += fmt.Sprintf(" staid=%4d pos=%.8f %.8f %.3f", staid, pos[0]*R2D, pos[1]*R2D, pos[2])
	}
	/* test station id */
	if rtcm.test_staid(staid) == 0 {
		return -1
	}

	rtcm.StaPara.Name = fmt.Sprintf("%04d", staid)
	rtcm.StaPara.DelType = 0 /* xyz */
	for j = 0; j < 3; j++ {
		rtcm.StaPara.Pos[j] = rr[j] * 0.0001
		rtcm.StaPara.Del[j] = 0.0
	}
	rtcm.StaPara.Hgt = 0.0
	rtcm.StaPara.Itrf = itrf
	return 5
}

/* decode type 1006: stationary RTK reference station ARP with height --------*/
func (rtcm *Rtcm) decode_type1006() int {
	var (
		rr, re, pos    [3]float64
		anth           float64
		i              int = 24 + 12
		j, staid, itrf int
	)

	if i+156 <= rtcm.MsgLen*8 {
		staid = int(GetBitU(rtcm.Buff[:], i, 12))
		i += 12
		itrf = int(GetBitU(rtcm.Buff[:], i, 6))
		i += 6 + 4
		rr[0] = float64(getbits_38(rtcm.Buff[:], i))
		i += 38 + 2
		rr[1] = float64(getbits_38(rtcm.Buff[:], i))
		i += 38 + 2
		rr[2] = float64(getbits_38(rtcm.Buff[:], i))
		i += 38
		anth = float64(GetBitU(rtcm.Buff[:], i, 16))
	} else {
		Trace(2, "rtcm3 1006 length error: len=%d\n", rtcm.MsgLen)
		return -1
	}
	if rtcm.OutType > 0 {
		for j = 0; j < 3; j++ {
			re[j] = rr[j] * 0.0001
		}
		Ecef2Pos(re[:], pos[:])
		rtcm.MsgType += fmt.Sprintf(" staid=%4d pos=%.8f %.8f %.3f anth=%.3f", staid, pos[0]*R2D,
			pos[1]*R2D, pos[2], anth*0.0001)
	}
	/* test station id */
	if rtcm.test_staid(staid) == 0 {
		return -1
	}

	rtcm.StaPara.Name = fmt.Sprintf("%04d", staid)
	rtcm.StaPara.DelType = 1 /* xyz */
	for j = 0; j < 3; j++ {
		rtcm.StaPara.Pos[j] = rr[j] * 0.0001
		rtcm.StaPara.Del[j] = 0.0
	}
	rtcm.StaPara.Hgt = anth * 0.0001
	rtcm.StaPara.Itrf = itrf
	return 5
}

/* decode type 1007: antenna descriptor --------------------------------------*/
func (rtcm *Rtcm) decode_type1007() int {
	var (
		des                [32]byte
		i                  int = 24 + 12
		j, staid, n, setup int
	)

	n = int(GetBitU(rtcm.Buff[:], i+12, 8))

	if i+28+8*n <= rtcm.MsgLen*8 {
		staid = int(GetBitU(rtcm.Buff[:], i, 12))
		i += 12 + 8
		for j = 0; j < n && j < 31; j++ {
			des[j] = byte(GetBitU(rtcm.Buff[:], i, 8))
			i += 8
		}
		setup = int(GetBitU(rtcm.Buff[:], i, 8))
	} else {
		Trace(2, "rtcm3 1007 length error: len=%d\n", rtcm.MsgLen)
		return -1
	}
	if rtcm.OutType > 0 {
		rtcm.MsgType += fmt.Sprintf(" staid=%4d", staid)
	}
	/* test station ID */
	if rtcm.test_staid(staid) == 0 {
		return -1
	}

	rtcm.StaPara.Name = fmt.Sprintf("%04d", staid)
	rtcm.StaPara.AntDes = string(des[:n])
	rtcm.StaPara.AntSetup = setup
	rtcm.StaPara.AntSno = ""
	return 5
}

/* decode type 1008: antenna descriptor & serial number ----------------------*/
func (rtcm *Rtcm) decode_type1008() int {
	var (
		des, sno              [32]byte
		i                     int = 24 + 12
		j, staid, n, m, setup int
	)

	n = int(GetBitU(rtcm.Buff[:], i+12, 8))
	m = int(GetBitU(rtcm.Buff[:], i+28+8*n, 8))

	if i+36+8*(n+m) <= rtcm.MsgLen*8 {
		staid = int(GetBitU(rtcm.Buff[:], i, 12))
		i += 12 + 8
		for j = 0; j < n && j < 31; j++ {
			des[j] = byte(GetBitU(rtcm.Buff[:], i, 8))
			i += 8
		}
		setup = int(GetBitU(rtcm.Buff[:], i, 8))
		i += 8 + 8
		for j = 0; j < m && j < 31; j++ {
			sno[j] = byte(GetBitU(rtcm.Buff[:], i, 8))
			i += 8
		}
	} else {
		Trace(2, "rtcm3 1008 length error: len=%d\n", rtcm.MsgLen)
		return -1
	}
	if rtcm.OutType > 0 {
		rtcm.MsgType += fmt.Sprintf(" staid=%4d", staid)
	}
	/* test station ID */
	if rtcm.test_staid(staid) == 0 {
		return -1
	}

	rtcm.StaPara.Name = fmt.Sprintf("%04d", staid)
	rtcm.StaPara.AntDes = string(des[:n])
	rtcm.StaPara.AntSetup = setup
	rtcm.StaPara.AntSno = string(sno[:m])
	return 5
}

/* decode type 1009-1012 message header --------------------------------------*/
func (rtcm *Rtcm) decode_head1009(sync *int) int {
	var (
		tod                float64
		tstr               string
		i                  int = 24
		staid, nsat, ctype int
	)

	ctype = int(GetBitU(rtcm.Buff[:], i, 12))
	i += 12

	if i+49 <= rtcm.MsgLen*8 {
		staid = int(GetBitU(rtcm.Buff[:], i, 12))
		i += 12
		tod = float64(GetBitU(rtcm.Buff[:], i, 27)) * 0.001
		i += 27 /* sec in a day */
		*sync = int(GetBitU(rtcm.Buff[:], i, 1))
		i += 1
		nsat = int(GetBitU(rtcm.Buff[:], i, 5))
	} else {
		Trace(2, "rtcm3 %d length error: len=%d\n", ctype, rtcm.MsgLen)
		return -1
	}
	/* test station ID */
	if rtcm.test_staid(staid) == 0 {
		return -1
	}

	rtcm.AdjDay_Glot(tod)

	Time2Str(rtcm.Time, &tstr, 2)
	Trace(5, "decode_head1009: time=%s nsat=%d sync=%d\n", tstr, nsat, *sync)

	if rtcm.OutType > 0 {
		rtcm.MsgType += fmt.Sprintf(" staid=%4d %s nsat=%2d sync=%d", staid, tstr, nsat, *sync)
	}
	return nsat
}

/* decode type 1009: L1-only glonass rtk observables -------------------------*/
func (rtcm *Rtcm) decode_type1009() int {
	var sync int
	if rtcm.decode_head1009(&sync) < 0 {
		return -1
	}
	return retsync(sync, &rtcm.ObsFlag)
}

/* decode type 1010: extended L1-only glonass rtk observables ----------------*/
func (rtcm *Rtcm) decode_type1010() int {
	var (
		pr1, cnr1, tt, cp1, freq1            float64
		i                                    int = 24 + 61
		j, index, nsat, sync, prn, sat, code int
		fcn, ppr1, lock1, amb                int
		sys                                  int = SYS_GLO
	)

	if nsat = rtcm.decode_head1009(&sync); nsat < 0 {
		return -1
	}

	for j = 0; j < nsat && rtcm.ObsData.N() < MAXOBS+1 && i+79 <= rtcm.MsgLen*8; j++ {
		prn = int(GetBitU(rtcm.Buff[:], i, 6))
		i += 6
		code = int(GetBitU(rtcm.Buff[:], i, 1))
		i += 1
		fcn = int(GetBitU(rtcm.Buff[:], i, 5))
		i += 5 /* fcn+7 */
		pr1 = float64(GetBitU(rtcm.Buff[:], i, 25))
		i += 25
		ppr1 = int(GetBits(rtcm.Buff[:], i, 20))
		i += 20
		lock1 = int(GetBitU(rtcm.Buff[:], i, 7))
		i += 7
		amb = int(GetBitU(rtcm.Buff[:], i, 7))
		i += 7
		cnr1 = float64(GetBitU(rtcm.Buff[:], i, 8))
		i += 8
		if sat = SatNo(sys, prn); sat == 0 {
			Trace(2, "rtcm3 1010 satellite number error: prn=%d\n", prn)
			continue
		}
		if rtcm.NavData.Glo_fcn[prn-1] == 0 {
			rtcm.NavData.Glo_fcn[prn-1] = fcn - 7 + 8 /* fcn+8 */
		}
		tt = TimeDiff(rtcm.ObsData.Data[0].Time, rtcm.Time)
		if rtcm.ObsFlag > 0 || math.Abs(tt) > 1e-9 {
			rtcm.ObsData.Data, rtcm.ObsFlag = nil, 0
		}
		if index = rtcm.ObsData.ObsIndex(rtcm.Time, sat); index < 0 {
			continue
		}
		pr1 = pr1*0.02 + float64(amb)*PRUNIT_GLO
		rtcm.ObsData.Data[index].P[0] = pr1

		if ppr1 != 0xFFF80000 {
			freq1 = Code2Freq(SYS_GLO, CODE_L1C, fcn-7)
			cp1 = rtcm.AdjCP(sat, 0, float64(ppr1)*0.0005*freq1/CLIGHT)
			rtcm.ObsData.Data[index].L[0] = pr1*freq1/CLIGHT + cp1
		}
		rtcm.ObsData.Data[index].LLI[0] = uint8(rtcm.LossOfLock(sat, 0, lock1))
		rtcm.ObsData.Data[index].SNR[0] = SnRatio(cnr1 * 0.25)

		rtcm.ObsData.Data[index].Code[0] = CODE_L1C
		if code > 0 {
			rtcm.ObsData.Data[index].Code[0] = CODE_L1P
		}
	}
	return retsync(sync, &index)
}

/* decode type 1011: L1&L2 GLONASS RTK observables ---------------------------*/
func (rtcm *Rtcm) decode_type1011() int {
	var sync int
	if rtcm.decode_head1009(&sync) < 0 {
		return -1
	}
	return retsync(sync, &rtcm.ObsFlag)
}

/* decode type 1012: extended L1&L2 GLONASS RTK observables ------------------*/
func (rtcm *Rtcm) decode_type1012() int {
	var (
		pr1, cnr1, cnr2, tt, cp1            float64
		cp2, freq1, freq2                   float64
		i                                   int = 24 + 61
		j, index, nsat, sync, prn, sat, fcn int
		code1, code2, pr21, ppr1, ppr2      int
		lock1, lock2, amb                   int
	)
	sys := SYS_GLO

	if nsat = rtcm.decode_head1009(&sync); nsat < 0 {
		return -1
	}

	for j = 0; j < nsat && rtcm.ObsData.N() < MAXOBS+1 && i+130 <= rtcm.MsgLen*8; j++ {
		prn = int(GetBitU(rtcm.Buff[:], i, 6))
		i += 6
		code1 = int(GetBitU(rtcm.Buff[:], i, 1))
		i += 1
		fcn = int(GetBitU(rtcm.Buff[:], i, 5))
		i += 5 /* fcn+7 */
		pr1 = float64(GetBitU(rtcm.Buff[:], i, 25))
		i += 25
		ppr1 = int(GetBits(rtcm.Buff[:], i, 20))
		i += 20
		lock1 = int(GetBitU(rtcm.Buff[:], i, 7))
		i += 7
		amb = int(GetBitU(rtcm.Buff[:], i, 7))
		i += 7
		cnr1 = float64(GetBitU(rtcm.Buff[:], i, 8))
		i += 8
		code2 = int(GetBitU(rtcm.Buff[:], i, 2))
		i += 2
		pr21 = int(GetBits(rtcm.Buff[:], i, 14))
		i += 14
		ppr2 = int(GetBits(rtcm.Buff[:], i, 20))
		i += 20
		lock2 = int(GetBitU(rtcm.Buff[:], i, 7))
		i += 7
		cnr2 = float64(GetBitU(rtcm.Buff[:], i, 8))
		i += 8
		if sat = SatNo(sys, prn); sat == 0 {
			Trace(2, "rtcm3 1012 satellite number error: sys=%d prn=%d\n", sys, prn)
			continue
		}
		if rtcm.NavData.Glo_fcn[prn-1] == 0 {
			rtcm.NavData.Glo_fcn[prn-1] = fcn - 7 + 8 /* fcn+8 */
		}
		tt = TimeDiff(rtcm.ObsData.Data[0].Time, rtcm.Time)
		if rtcm.ObsFlag > 0 || math.Abs(tt) > 1e-9 {
			rtcm.ObsData.Data, rtcm.ObsFlag = nil, 0
		}
		if index = rtcm.ObsData.ObsIndex(rtcm.Time, sat); index < 0 {
			continue
		}
		pr1 = pr1*0.02 + float64(amb)*PRUNIT_GLO
		rtcm.ObsData.Data[index].P[0] = pr1

		if ppr1 != 0xFFF80000 {
			freq1 = Code2Freq(SYS_GLO, CODE_L1C, fcn-7)
			cp1 = rtcm.AdjCP(sat, 0, float64(ppr1)*0.0005*freq1/CLIGHT)
			rtcm.ObsData.Data[index].L[0] = pr1*freq1/CLIGHT + cp1
		}
		rtcm.ObsData.Data[index].LLI[0] = uint8(rtcm.LossOfLock(sat, 0, lock1))
		rtcm.ObsData.Data[index].SNR[0] = SnRatio(cnr1 * 0.25)
		rtcm.ObsData.Data[index].Code[0] = CODE_L1C
		if code1 > 0 {
			rtcm.ObsData.Data[index].Code[0] = CODE_L1P
		}

		if pr21 != 0xFFFFE000 {
			rtcm.ObsData.Data[index].P[1] = pr1 + float64(pr21)*0.02
		}
		if ppr2 != 0xFFF80000 {
			freq2 = Code2Freq(SYS_GLO, CODE_L2C, fcn-7)
			cp2 = rtcm.AdjCP(sat, 1, float64(ppr2)*0.0005*freq2/CLIGHT)
			rtcm.ObsData.Data[index].L[1] = pr1*freq2/CLIGHT + cp2
		}
		rtcm.ObsData.Data[index].LLI[1] = uint8(rtcm.LossOfLock(sat, 1, lock2))
		rtcm.ObsData.Data[index].SNR[1] = SnRatio(cnr2 * 0.25)
		rtcm.ObsData.Data[index].Code[1] = CODE_L2C
		if code2 > 0 {
			rtcm.ObsData.Data[index].Code[1] = CODE_L2P
		}
	}
	return retsync(sync, &rtcm.ObsFlag)
}

/* decode type 1013: system parameters ---------------------------------------*/
func (rtcm *Rtcm) decode_type1013() int {
	return 0
}

/* decode type 1019: GPS ephemerides -----------------------------------------*/
func (rtcm *Rtcm) decode_type1019() int {
	var (
		eph            Eph
		toc, sqrtA, tt float64
		i              int = 24 + 12
		prn, sat, week int
	)
	sys := SYS_GPS
	if i+476 <= rtcm.MsgLen*8 {
		prn = int(GetBitU(rtcm.Buff[:], i, 6))
		i += 6
		week = int(GetBitU(rtcm.Buff[:], i, 10))
		i += 10
		eph.Sva = int(GetBitU(rtcm.Buff[:], i, 4))
		i += 4
		eph.Code = int(GetBitU(rtcm.Buff[:], i, 2))
		i += 2
		eph.Idot = float64(GetBits(rtcm.Buff[:], i, 14)) * P2_43 * SC2RAD
		i += 14
		eph.Iode = int(GetBitU(rtcm.Buff[:], i, 8))
		i += 8
		toc = float64(GetBitU(rtcm.Buff[:], i, 16)) * 16.0
		i += 16
		eph.F2 = float64(GetBits(rtcm.Buff[:], i, 8)) * P2_55
		i += 8
		eph.F1 = float64(GetBits(rtcm.Buff[:], i, 16)) * P2_43
		i += 16
		eph.F0 = float64(GetBits(rtcm.Buff[:], i, 22)) * P2_31
		i += 22
		eph.Iodc = int(GetBitU(rtcm.Buff[:], i, 10))
		i += 10
		eph.Crs = float64(GetBits(rtcm.Buff[:], i, 16)) * P2_5
		i += 16
		eph.Deln = float64(GetBits(rtcm.Buff[:], i, 16)) * P2_43 * SC2RAD
		i += 16
		eph.M0 = float64(GetBits(rtcm.Buff[:], i, 32)) * P2_31 * SC2RAD
		i += 32
		eph.Cuc = float64(GetBits(rtcm.Buff[:], i, 16)) * P2_29
		i += 16
		eph.E = float64(GetBitU(rtcm.Buff[:], i, 32)) * P2_33
		i += 32
		eph.Cus = float64(GetBits(rtcm.Buff[:], i, 16)) * P2_29
		i += 16
		sqrtA = float64(GetBitU(rtcm.Buff[:], i, 32)) * P2_19
		i += 32
		eph.Toes = float64(GetBitU(rtcm.Buff[:], i, 16)) * 16.0
		i += 16
		eph.Cic = float64(GetBits(rtcm.Buff[:], i, 16)) * P2_29
		i += 16
		eph.OMG0 = float64(GetBits(rtcm.Buff[:], i, 32)) * P2_31 * SC2RAD
		i += 32
		eph.Cis = float64(GetBits(rtcm.Buff[:], i, 16)) * P2_29
		i += 16
		eph.I0 = float64(GetBits(rtcm.Buff[:], i, 32)) * P2_31 * SC2RAD
		i += 32
		eph.Crc = float64(GetBits(rtcm.Buff[:], i, 16)) * P2_5
		i += 16
		eph.Omg = float64(GetBits(rtcm.Buff[:], i, 32)) * P2_31 * SC2RAD
		i += 32
		eph.OMGd = float64(GetBits(rtcm.Buff[:], i, 24)) * P2_43 * SC2RAD
		i += 24
		eph.Tgd[0] = float64(GetBits(rtcm.Buff[:], i, 8)) * P2_31
		i += 8
		eph.Svh = int(GetBitU(rtcm.Buff[:], i, 6))
		i += 6
		eph.Flag = int(GetBitU(rtcm.Buff[:], i, 1))
		i += 1
		eph.Fit = 4.0 /* 0:4hr,1:>4hr */
		if GetBitU(rtcm.Buff[:], i, 1) > 0 {
			eph.Fit = 0.0
		}
	} else {
		Trace(2, "rtcm3 1019 length error: len=%d\n", rtcm.MsgLen)
		return -1
	}
	if prn >= 40 {
		sys = SYS_SBS
		prn += 80
	}
	Trace(4, "decode_type1019: prn=%d iode=%d toe=%.0f\n", prn, eph.Iode, eph.Toes)

	if rtcm.OutType > 0 {
		rtcm.MsgType += fmt.Sprintf(" prn=%2d iode=%3d iodc=%3d week=%d toe=%6.0f toc=%6.0f svh=%02X",
			prn, eph.Iode, eph.Iodc, week, eph.Toes, toc, eph.Svh)
	}
	if sat = SatNo(sys, prn); sat == 0 {
		Trace(2, "rtcm3 1019 satellite number error: prn=%d\n", prn)
		return -1
	}
	eph.Sat = sat
	eph.Week = AdjGpsWeek(week)
	if rtcm.Time.Time == 0 {
		rtcm.Time = Utc2GpsT(TimeGet())
	}
	tt = TimeDiff(GpsT2Time(eph.Week, eph.Toes), rtcm.Time)
	if tt < -302400.0 {
		eph.Week++
	} else if tt >= 302400.0 {
		eph.Week--
	}
	eph.Toe = GpsT2Time(eph.Week, eph.Toes)
	eph.Toc = GpsT2Time(eph.Week, toc)
	eph.Ttr = rtcm.Time
	eph.A = sqrtA * sqrtA
	if !strings.Contains(rtcm.Opt, "-EPHALL") {
		if eph.Iode == rtcm.NavData.Ephs[sat-1].Iode {
			return 0 /* unchanged */
		}
	}
	rtcm.NavData.Ephs[sat-1] = eph
	rtcm.EphSat = sat
	rtcm.EphSet = 0
	return 2
}

/* decode type 1020: GLONASS ephemerides -------------------------------------*/
func (rtcm *Rtcm) decode_type1020() int {
	var (
		geph                                 GEph
		tk_h, tk_m, tk_s, toe, tow, tod, tof float64
		i                                    int = 24 + 12
		prn, sat, week, tb, bn               int
	)
	sys := SYS_GLO

	if i+348 <= rtcm.MsgLen*8 {
		prn = int(GetBitU(rtcm.Buff[:], i, 6))
		i += 6
		geph.Frq = int(GetBitU(rtcm.Buff[:], i, 5)) - 7
		i += 5 + 2 + 2
		tk_h = float64(GetBitU(rtcm.Buff[:], i, 5))
		i += 5
		tk_m = float64(GetBitU(rtcm.Buff[:], i, 6))
		i += 6
		tk_s = float64(GetBitU(rtcm.Buff[:], i, 1)) * 30.0
		i += 1
		bn = int(GetBitU(rtcm.Buff[:], i, 1))
		i += 1 + 1
		tb = int(GetBitU(rtcm.Buff[:], i, 7))
		i += 7
		geph.Vel[0] = getbitg(rtcm.Buff[:], i, 24) * P2_20 * 1e3
		i += 24
		geph.Pos[0] = getbitg(rtcm.Buff[:], i, 27) * P2_11 * 1e3
		i += 27
		geph.Acc[0] = getbitg(rtcm.Buff[:], i, 5) * P2_30 * 1e3
		i += 5
		geph.Vel[1] = getbitg(rtcm.Buff[:], i, 24) * P2_20 * 1e3
		i += 24
		geph.Pos[1] = getbitg(rtcm.Buff[:], i, 27) * P2_11 * 1e3
		i += 27
		geph.Acc[1] = getbitg(rtcm.Buff[:], i, 5) * P2_30 * 1e3
		i += 5
		geph.Vel[2] = getbitg(rtcm.Buff[:], i, 24) * P2_20 * 1e3
		i += 24
		geph.Pos[2] = getbitg(rtcm.Buff[:], i, 27) * P2_11 * 1e3
		i += 27
		geph.Acc[2] = getbitg(rtcm.Buff[:], i, 5) * P2_30 * 1e3
		i += 5 + 1
		geph.Gamn = getbitg(rtcm.Buff[:], i, 11) * P2_40
		i += 11 + 3
		geph.Taun = getbitg(rtcm.Buff[:], i, 22) * P2_30
		i += 22
		geph.DTaun = getbitg(rtcm.Buff[:], i, 5) * P2_30
		i += 5
		geph.Age = int(GetBitU(rtcm.Buff[:], i, 5))
	} else {
		Trace(2, "rtcm3 1020 length error: len=%d\n", rtcm.MsgLen)
		return -1
	}
	if sat = SatNo(sys, prn); sat == 0 {
		Trace(2, "rtcm3 1020 satellite number error: prn=%d\n", prn)
		return -1
	}
	Trace(4, "decode_type1020: prn=%d tk=%02.0f:%02.0f:%02.0f\n", prn, tk_h, tk_m, tk_s)

	if rtcm.OutType > 0 {
		rtcm.MsgType += fmt.Sprintf(" prn=%2d tk=%02.0f:%02.0f:%02.0f frq=%2d bn=%d tb=%d",
			prn, tk_h, tk_m, tk_s, geph.Frq, bn, tb)
	}
	geph.Sat = sat
	geph.Svh = bn
	geph.Iode = tb & 0x7F
	if rtcm.Time.Time == 0 {
		rtcm.Time = Utc2GpsT(TimeGet())
	}
	tow = Time2GpsT(GpsT2Utc(rtcm.Time), &week)
	tod = math.Mod(tow, 86400.0)
	tow -= tod
	tof = tk_h*3600.0 + tk_m*60.0 + tk_s - 10800.0 /* lt.utc */
	if tof < tod-43200.0 {
		tof += 86400.0
	} else if tof > tod+43200.0 {
		tof -= 86400.0
	}
	geph.Tof = Utc2GpsT(GpsT2Time(week, tow+tof))
	toe = float64(tb)*900.0 - 10800.0 /* lt.utc */
	if toe < tod-43200.0 {
		toe += 86400.0
	} else if toe > tod+43200.0 {
		toe -= 86400.0
	}
	geph.Toe = Utc2GpsT(GpsT2Time(week, tow+toe)) /* utc.gpst */

	if !strings.Contains(rtcm.Opt, "-EPHALL") {
		if math.Abs(TimeDiff(geph.Toe, rtcm.NavData.Geph[prn-1].Toe)) < 1.0 &&
			geph.Svh == rtcm.NavData.Geph[prn-1].Svh {
			return 0 /* unchanged */
		}
	}
	rtcm.NavData.Geph[prn-1] = geph
	rtcm.EphSat = sat
	rtcm.EphSet = 0
	return 2
}

/* decode type 1021: helmert/abridged molodenski -----------------------------*/
func (rtcm *Rtcm) decode_type1021() int {
	Trace(4, "rtcm3 1021: not supported message\n")
	return 0
}

/* decode type 1022: Moledenski-Badekas transfromation -----------------------*/
func (rtcm *Rtcm) decode_type1022() int {
	Trace(4, "rtcm3 1022: not supported message\n")
	return 0
}

/* decode type 1023: residual, ellipsoidal grid representation ---------------*/
func (rtcm *Rtcm) decode_type1023() int {
	Trace(4, "rtcm3 1023: not supported message\n")
	return 0
}

/* decode type 1024: residual, plane grid representation ---------------------*/
func (rtcm *Rtcm) decode_type1024() int {
	Trace(4, "rtcm3 1024: not supported message\n")
	return 0
}

/* decode type 1025: projection (types except LCC2SP,OM) ---------------------*/
func (rtcm *Rtcm) decode_type1025() int {
	Trace(4, "rtcm3 1025: not supported message\n")
	return 0
}

/* decode type 1026: projection (LCC2SP - lambert conic conformal (2sp)) -----*/
func (rtcm *Rtcm) decode_type1026() int {
	Trace(4, "rtcm3 1026: not supported message\n")
	return 0
}

/* decode type 1027: projection (type OM - oblique mercator) -----------------*/
func (rtcm *Rtcm) decode_type1027() int {
	Trace(4, "rtcm3 1027: not supported message\n")
	return 0
}

/* decode type 1029: UNICODE text string -------------------------------------*/
func (rtcm *Rtcm) decode_type1029() int {
	var (
		msg                              [128]rune
		i                                int = 24 + 12
		j, staid, mjd, tod, nchar, cunit int
	)

	if i+60 <= rtcm.MsgLen*8 {
		staid = int(GetBitU(rtcm.Buff[:], i, 12))
		i += 12
		mjd = int(GetBitU(rtcm.Buff[:], i, 16))
		i += 16
		tod = int(GetBitU(rtcm.Buff[:], i, 17))
		i += 17
		nchar = int(GetBitU(rtcm.Buff[:], i, 7))
		i += 7
		cunit = int(GetBitU(rtcm.Buff[:], i, 8))
		i += 8
	} else {
		Trace(2, "rtcm3 1029 length error: len=%d\n", rtcm.MsgLen)
		return -1
	}
	if i+nchar*8 > rtcm.MsgLen*8 {
		Trace(2, "rtcm3 1029 length error: len=%d nchar=%d%d%d%d\n", rtcm.MsgLen, nchar, mjd, tod, cunit)
		return -1
	}
	for j = 0; j < nchar && j < 126; j++ {
		msg[j] = rune(GetBitU(rtcm.Buff[:], i, 8))
		i += 8
	}
	rtcm.Msg = string(msg[:])

	if rtcm.OutType > 0 {
		rtcm.MsgType += fmt.Sprintf(" staid=%4d text=%s", staid, rtcm.Msg)
	}
	return 0
}

/* decode type 1030: network RTK residual ------------------------------------*/
func (rtcm *Rtcm) decode_type1030() int {
	Trace(4, "rtcm3 1030: not supported message\n")
	return 0
}

/* decode type 1031: GLONASS network RTK residual ----------------------------*/
func (rtcm *Rtcm) decode_type1031() int {
	Trace(4, "rtcm3 1031: not supported message\n")
	return 0
}

/* decode type 1032: physical reference station position information ---------*/
func (rtcm *Rtcm) decode_type1032() int {
	Trace(4, "rtcm3 1032: not supported message\n")
	return 0
}

/* decode type 1033: receiver and antenna descriptor -------------------------*/
func (rtcm *Rtcm) decode_type1033() int {
	var (
		des, sno, rec, ver, rsn           [32]byte
		i                                 int = 24 + 12
		j, staid, n, m, n1, n2, n3, setup int
	)

	n = int(GetBitU(rtcm.Buff[:], i+12, 8))
	m = int(GetBitU(rtcm.Buff[:], i+28+8*n, 8))
	n1 = int(GetBitU(rtcm.Buff[:], i+36+8*(n+m), 8))
	n2 = int(GetBitU(rtcm.Buff[:], i+44+8*(n+m+n1), 8))
	n3 = int(GetBitU(rtcm.Buff[:], i+52+8*(n+m+n1+n2), 8))

	if i+60+8*(n+m+n1+n2+n3) <= rtcm.MsgLen*8 {
		staid = int(GetBitU(rtcm.Buff[:], i, 12))
		i += 12 + 8
		for j = 0; j < n && j < 31; j++ {
			des[j] = byte(GetBitU(rtcm.Buff[:], i, 8))
			i += 8
		}
		setup = int(GetBitU(rtcm.Buff[:], i, 8))
		i += 8 + 8
		for j = 0; j < m && j < 31; j++ {
			sno[j] = byte(GetBitU(rtcm.Buff[:], i, 8))
			i += 8
		}
		i += 8
		for j = 0; j < n1 && j < 31; j++ {
			rec[j] = byte(GetBitU(rtcm.Buff[:], i, 8))
			i += 8
		}
		i += 8
		for j = 0; j < n2 && j < 31; j++ {
			ver[j] = byte(GetBitU(rtcm.Buff[:], i, 8))
			i += 8
		}
		i += 8
		for j = 0; j < n3 && j < 31; j++ {
			rsn[j] = byte(GetBitU(rtcm.Buff[:], i, 8))
			i += 8
		}
	} else {
		Trace(2, "rtcm3 1033 length error: len=%d\n", rtcm.MsgLen)
		return -1
	}
	if rtcm.OutType > 0 {
		rtcm.MsgType += fmt.Sprintf(" staid=%4d", staid)
	}
	/* test station id */
	if rtcm.test_staid(staid) == 0 {
		return -1
	}

	rtcm.StaPara.Name = fmt.Sprintf("%04d", staid)
	rtcm.StaPara.AntDes = string(des[:n])
	rtcm.StaPara.AntSetup = setup
	rtcm.StaPara.AntSno = string(sno[:m])
	rtcm.StaPara.Type = string(rec[:n1])
	rtcm.StaPara.RecVer = string(ver[:n2])
	rtcm.StaPara.RecSN = string(rsn[:n3])

	Trace(5, "rtcm3 1033: ant=%s:%s rec=%s:%s:%s\n", string(des[:]), string(sno[:]), string(rec[:]), string(ver[:]), string(rsn[:]))
	return 5
}

/* decode type 1034: GPS network FKP gradient --------------------------------*/
func (rtcm *Rtcm) decode_type1034() int {
	Trace(4, "rtcm3 1034: not supported message\n")
	return 0
}

/* decode type 1035: GLONASS network FKP gradient ----------------------------*/
func (rtcm *Rtcm) decode_type1035() int {
	Trace(4, "rtcm3 1035: not supported message\n")
	return 0
}

/* decode type 1037: GLONASS network RTK ionospheric correction difference ---*/
func (rtcm *Rtcm) decode_type1037() int {
	Trace(4, "rtcm3 1037: not supported message\n")
	return 0
}

/* decode type 1038: GLONASS network RTK geometic correction difference ------*/
func (rtcm *Rtcm) decode_type1038() int {
	Trace(4, "rtcm3 1038: not supported message\n")
	return 0
}

/* decode type 1039: GLONASS network RTK combined correction difference ------*/
func (rtcm *Rtcm) decode_type1039() int {
	Trace(4, "rtcm3 1039: not supported message\n")
	return 0
}

/* decode type 1041: NavIC/IRNSS ephemerides ---------------------------------*/
func (rtcm *Rtcm) decode_type1041() int {
	var (
		eph            Eph
		toc, sqrtA, tt float64
		i              int = 24 + 12
		prn, sat, week int
	)
	sys := SYS_IRN

	if i+482-12 <= rtcm.MsgLen*8 {
		prn = int(GetBitU(rtcm.Buff[:], i, 6))
		i += 6
		week = int(GetBitU(rtcm.Buff[:], i, 10))
		i += 10
		eph.F0 = float64(GetBits(rtcm.Buff[:], i, 22)) * P2_31
		i += 22
		eph.F1 = float64(GetBits(rtcm.Buff[:], i, 16)) * P2_43
		i += 16
		eph.F2 = float64(GetBits(rtcm.Buff[:], i, 8)) * P2_55
		i += 8
		eph.Sva = int(GetBitU(rtcm.Buff[:], i, 4))
		i += 4
		toc = float64(GetBitU(rtcm.Buff[:], i, 16)) * 16.0
		i += 16
		eph.Tgd[0] = float64(GetBits(rtcm.Buff[:], i, 8)) * P2_31
		i += 8
		eph.Deln = float64(GetBits(rtcm.Buff[:], i, 22)) * P2_41 * SC2RAD
		i += 22
		eph.Iode = int(GetBitU(rtcm.Buff[:], i, 8))
		i += 8 + 10 /* IODEC */
		eph.Svh = int(GetBitU(rtcm.Buff[:], i, 2))
		i += 2 /* L5+Sflag */
		eph.Cuc = float64(GetBits(rtcm.Buff[:], i, 15)) * P2_28
		i += 15
		eph.Cus = float64(GetBits(rtcm.Buff[:], i, 15)) * P2_28
		i += 15
		eph.Cic = float64(GetBits(rtcm.Buff[:], i, 15)) * P2_28
		i += 15
		eph.Cis = float64(GetBits(rtcm.Buff[:], i, 15)) * P2_28
		i += 15
		eph.Crc = float64(GetBits(rtcm.Buff[:], i, 15)) * 0.0625
		i += 15
		eph.Crs = float64(GetBits(rtcm.Buff[:], i, 15)) * 0.0625
		i += 15
		eph.Idot = float64(GetBits(rtcm.Buff[:], i, 14)) * P2_43 * SC2RAD
		i += 14
		eph.M0 = float64(GetBits(rtcm.Buff[:], i, 32)) * P2_31 * SC2RAD
		i += 32
		eph.Toes = float64(GetBitU(rtcm.Buff[:], i, 16)) * 16.0
		i += 16
		eph.E = float64(GetBitU(rtcm.Buff[:], i, 32)) * P2_33
		i += 32
		sqrtA = float64(GetBitU(rtcm.Buff[:], i, 32)) * P2_19
		i += 32
		eph.OMG0 = float64(GetBits(rtcm.Buff[:], i, 32)) * P2_31 * SC2RAD
		i += 32
		eph.Omg = float64(GetBits(rtcm.Buff[:], i, 32)) * P2_31 * SC2RAD
		i += 32
		eph.OMGd = float64(GetBits(rtcm.Buff[:], i, 22)) * P2_41 * SC2RAD
		i += 22
		eph.I0 = float64(GetBits(rtcm.Buff[:], i, 32)) * P2_31 * SC2RAD
	} else {
		Trace(2, "rtcm3 1041 length error: len=%d\n", rtcm.MsgLen)
		return -1
	}
	Trace(4, "decode_type1041: prn=%d iode=%d toe=%.0f\n", prn, eph.Iode, eph.Toes)

	if rtcm.OutType > 0 {
		rtcm.MsgType += fmt.Sprintf(" prn=%2d iode=%3d week=%d toe=%6.0f toc=%6.0f svh=%02X",
			prn, eph.Iode, week, eph.Toes, toc, eph.Svh)
	}
	if sat = SatNo(sys, prn); sat == 0 {
		Trace(2, "rtcm3 1041 satellite number error: prn=%d\n", prn)
		return -1
	}
	eph.Sat = sat
	eph.Week = AdjGpsWeek(week)
	if rtcm.Time.Time == 0 {
		rtcm.Time = Utc2GpsT(TimeGet())
	}
	tt = TimeDiff(GpsT2Time(eph.Week, eph.Toes), rtcm.Time)
	if tt < -302400.0 {
		eph.Week++
	} else if tt >= 302400.0 {
		eph.Week--
	}
	eph.Toe = GpsT2Time(eph.Week, eph.Toes)
	eph.Toc = GpsT2Time(eph.Week, toc)
	eph.Ttr = rtcm.Time
	eph.A = sqrtA * sqrtA
	eph.Iodc = eph.Iode
	if !strings.Contains(rtcm.Opt, "-EPHALL") {
		if eph.Iode == rtcm.NavData.Ephs[sat-1].Iode {
			return 0 /* unchanged */
		}
	}
	rtcm.NavData.Ephs[sat-1] = eph
	rtcm.EphSat = sat
	rtcm.EphSet = 0
	return 2
}

/* decode type 1044: QZSS ephemerides ----------------------------------------*/
func (rtcm *Rtcm) decode_type1044() int {
	var (
		eph            Eph
		toc, sqrtA, tt float64
		i              int = 24 + 12
		prn, sat, week int
	)
	sys := SYS_QZS

	if i+473 <= rtcm.MsgLen*8 {
		prn = int(GetBitU(rtcm.Buff[:], i, 4)) + 192
		i += 4
		toc = float64(GetBitU(rtcm.Buff[:], i, 16)) * 16.0
		i += 16
		eph.F2 = float64(GetBits(rtcm.Buff[:], i, 8)) * P2_55
		i += 8
		eph.F1 = float64(GetBits(rtcm.Buff[:], i, 16)) * P2_43
		i += 16
		eph.F0 = float64(GetBits(rtcm.Buff[:], i, 22)) * P2_31
		i += 22
		eph.Iode = int(GetBitU(rtcm.Buff[:], i, 8))
		i += 8
		eph.Crs = float64(GetBits(rtcm.Buff[:], i, 16)) * P2_5
		i += 16
		eph.Deln = float64(GetBits(rtcm.Buff[:], i, 16)) * P2_43 * SC2RAD
		i += 16
		eph.M0 = float64(GetBits(rtcm.Buff[:], i, 32)) * P2_31 * SC2RAD
		i += 32
		eph.Cuc = float64(GetBits(rtcm.Buff[:], i, 16)) * P2_29
		i += 16
		eph.E = float64(GetBitU(rtcm.Buff[:], i, 32)) * P2_33
		i += 32
		eph.Cus = float64(GetBits(rtcm.Buff[:], i, 16)) * P2_29
		i += 16
		sqrtA = float64(GetBitU(rtcm.Buff[:], i, 32)) * P2_19
		i += 32
		eph.Toes = float64(GetBitU(rtcm.Buff[:], i, 16)) * 16.0
		i += 16
		eph.Cic = float64(GetBits(rtcm.Buff[:], i, 16)) * P2_29
		i += 16
		eph.OMG0 = float64(GetBits(rtcm.Buff[:], i, 32)) * P2_31 * SC2RAD
		i += 32
		eph.Cis = float64(GetBits(rtcm.Buff[:], i, 16)) * P2_29
		i += 16
		eph.I0 = float64(GetBits(rtcm.Buff[:], i, 32)) * P2_31 * SC2RAD
		i += 32
		eph.Crc = float64(GetBits(rtcm.Buff[:], i, 16)) * P2_5
		i += 16
		eph.Omg = float64(GetBits(rtcm.Buff[:], i, 32)) * P2_31 * SC2RAD
		i += 32
		eph.OMGd = float64(GetBits(rtcm.Buff[:], i, 24)) * P2_43 * SC2RAD
		i += 24
		eph.Idot = float64(GetBits(rtcm.Buff[:], i, 14)) * P2_43 * SC2RAD
		i += 14
		eph.Code = int(GetBitU(rtcm.Buff[:], i, 2))
		i += 2
		week = int(GetBitU(rtcm.Buff[:], i, 10))
		i += 10
		eph.Sva = int(GetBitU(rtcm.Buff[:], i, 4))
		i += 4
		eph.Svh = int(GetBitU(rtcm.Buff[:], i, 6))
		i += 6
		eph.Tgd[0] = float64(GetBits(rtcm.Buff[:], i, 8)) * P2_31
		i += 8
		eph.Iodc = int(GetBitU(rtcm.Buff[:], i, 10))
		i += 10
		eph.Fit = 2.0 /* 0:2hr,1:>2hr */
		if GetBitU(rtcm.Buff[:], i, 1) > 0 {
			eph.Fit = 0.0
		}
	} else {
		Trace(2, "rtcm3 1044 length error: len=%d\n", rtcm.MsgLen)
		return -1
	}
	Trace(4, "decode_type1044: prn=%d iode=%d toe=%.0f\n", prn, eph.Iode, eph.Toes)

	if rtcm.OutType > 0 {
		rtcm.MsgType += fmt.Sprintf(" prn=%3d iode=%3d iodc=%3d week=%d toe=%6.0f toc=%6.0f svh=%02X",
			prn, eph.Iode, eph.Iodc, week, eph.Toes, toc, eph.Svh)
	}
	if sat = SatNo(sys, prn); sat == 0 {
		Trace(2, "rtcm3 1044 satellite number error: prn=%d\n", prn)
		return -1
	}
	eph.Sat = sat
	eph.Week = AdjGpsWeek(week)
	if rtcm.Time.Time == 0 {
		rtcm.Time = Utc2GpsT(TimeGet())
	}
	tt = TimeDiff(GpsT2Time(eph.Week, eph.Toes), rtcm.Time)
	if tt < -302400.0 {
		eph.Week++
	} else if tt >= 302400.0 {
		eph.Week--
	}
	eph.Toe = GpsT2Time(eph.Week, eph.Toes)
	eph.Toc = GpsT2Time(eph.Week, toc)
	eph.Ttr = rtcm.Time
	eph.A = sqrtA * sqrtA
	eph.Flag = 1 /* fixed to 1 */
	if !strings.Contains(rtcm.Opt, "-EPHALL") {
		if eph.Iode == rtcm.NavData.Ephs[sat-1].Iode &&
			eph.Iodc == rtcm.NavData.Ephs[sat-1].Iodc {
			return 0 /* unchanged */
		}
	}
	rtcm.NavData.Ephs[sat-1] = eph
	rtcm.EphSat = sat
	rtcm.EphSet = 0
	return 2
}

/* decode type 1045: Galileo F/NAV satellite ephemerides ---------------------*/
func (rtcm *Rtcm) decode_type1045() int {
	var (
		eph                                  Eph
		toc, sqrtA, tt                       float64
		i                                    int = 24 + 12
		prn, sat, week, e5a_hs, e5a_dvs, rsv int
	)
	sys := SYS_GAL

	if strings.Contains(rtcm.Opt, "-GALINAV") {
		return 0
	}

	if i+484 <= rtcm.MsgLen*8 {
		prn = int(GetBitU(rtcm.Buff[:], i, 6))
		i += 6
		week = int(GetBitU(rtcm.Buff[:], i, 12))
		i += 12 /* gst-week */
		eph.Iode = int(GetBitU(rtcm.Buff[:], i, 10))
		i += 10
		eph.Sva = int(GetBitU(rtcm.Buff[:], i, 8))
		i += 8
		eph.Idot = float64(GetBits(rtcm.Buff[:], i, 14)) * P2_43 * SC2RAD
		i += 14
		toc = float64(GetBitU(rtcm.Buff[:], i, 14)) * 60.0
		i += 14
		eph.F2 = float64(GetBits(rtcm.Buff[:], i, 6)) * P2_59
		i += 6
		eph.F1 = float64(GetBits(rtcm.Buff[:], i, 21)) * P2_46
		i += 21
		eph.F0 = float64(GetBits(rtcm.Buff[:], i, 31)) * P2_34
		i += 31
		eph.Crs = float64(GetBits(rtcm.Buff[:], i, 16)) * P2_5
		i += 16
		eph.Deln = float64(GetBits(rtcm.Buff[:], i, 16)) * P2_43 * SC2RAD
		i += 16
		eph.M0 = float64(GetBits(rtcm.Buff[:], i, 32)) * P2_31 * SC2RAD
		i += 32
		eph.Cuc = float64(GetBits(rtcm.Buff[:], i, 16)) * P2_29
		i += 16
		eph.E = float64(GetBitU(rtcm.Buff[:], i, 32)) * P2_33
		i += 32
		eph.Cus = float64(GetBits(rtcm.Buff[:], i, 16)) * P2_29
		i += 16
		sqrtA = float64(GetBitU(rtcm.Buff[:], i, 32)) * P2_19
		i += 32
		eph.Toes = float64(GetBitU(rtcm.Buff[:], i, 14)) * 60.0
		i += 14
		eph.Cic = float64(GetBits(rtcm.Buff[:], i, 16)) * P2_29
		i += 16
		eph.OMG0 = float64(GetBits(rtcm.Buff[:], i, 32)) * P2_31 * SC2RAD
		i += 32
		eph.Cis = float64(GetBits(rtcm.Buff[:], i, 16)) * P2_29
		i += 16
		eph.I0 = float64(GetBits(rtcm.Buff[:], i, 32)) * P2_31 * SC2RAD
		i += 32
		eph.Crc = float64(GetBits(rtcm.Buff[:], i, 16)) * P2_5
		i += 16
		eph.Omg = float64(GetBits(rtcm.Buff[:], i, 32)) * P2_31 * SC2RAD
		i += 32
		eph.OMGd = float64(GetBits(rtcm.Buff[:], i, 24)) * P2_43 * SC2RAD
		i += 24
		eph.Tgd[0] = float64(GetBits(rtcm.Buff[:], i, 10)) * P2_32
		i += 10 /* E5a/E1 */
		e5a_hs = int(GetBitU(rtcm.Buff[:], i, 2))
		i += 2 /* OSHS */
		e5a_dvs = int(GetBitU(rtcm.Buff[:], i, 1))
		i += 1 /* OSDVS */
		// rsv = int(GetBitU(rtcm.buff[:], i, 7))
	} else {
		Trace(2, "rtcm3 1045 length error: len=%d,%d\n", rtcm.MsgLen, rsv)
		return -1
	}
	Trace(4, "decode_type1045: prn=%d iode=%d toe=%.0f\n", prn, eph.Iode, eph.Toes)

	if rtcm.OutType > 0 {
		rtcm.MsgType += fmt.Sprintf(" prn=%2d iode=%3d week=%d toe=%6.0f toc=%6.0f hs=%d dvs=%d",
			prn, eph.Iode, week, eph.Toes, toc, e5a_hs, e5a_dvs)
	}
	if sat = SatNo(sys, prn); sat == 0 {
		Trace(2, "rtcm3 1045 satellite number error: prn=%d\n", prn)
		return -1
	}
	if strings.Contains(rtcm.Opt, "-GALINAV") {
		return 0
	}
	eph.Sat = sat
	eph.Week = week + 1024 /* gal-week = gst-week + 1024 */
	if rtcm.Time.Time == 0 {
		rtcm.Time = Utc2GpsT(TimeGet())
	}
	tt = TimeDiff(GpsT2Time(eph.Week, eph.Toes), rtcm.Time)
	if tt < -302400.0 {
		eph.Week++
	} else if tt >= 302400.0 {
		eph.Week--
	}
	eph.Toe = GpsT2Time(eph.Week, eph.Toes)
	eph.Toc = GpsT2Time(eph.Week, toc)
	eph.Ttr = rtcm.Time
	eph.A = sqrtA * sqrtA
	eph.Svh = (e5a_hs << 4) + (e5a_dvs << 3)
	eph.Code = (1 << 1) + (1 << 8) /* data source = F/NAV+E5a */
	eph.Iodc = eph.Iode
	if !strings.Contains(rtcm.Opt, "-EPHALL") {
		if eph.Iode == rtcm.NavData.Ephs[sat-1+MAXSAT].Iode {
			return 0 /* unchanged */
		}
	}
	rtcm.NavData.Ephs[sat-1+MAXSAT] = eph
	rtcm.EphSat = sat
	rtcm.EphSet = 1 /* F/NAV */
	return 2
}

/* decode type 1046: Galileo I/NAV satellite ephemerides ---------------------*/
func (rtcm *Rtcm) decode_type1046() int {
	var (
		eph                    Eph
		toc, sqrtA, tt         float64
		i                      int = 24 + 12
		prn, sat, week, e5b_hs int
		e5b_dvs, e1_hs, e1_dvs int
	)
	sys := SYS_GAL

	if strings.Contains(rtcm.Opt, "-GALFNAV") {
		return 0
	}

	if i+492 <= rtcm.MsgLen*8 {
		prn = int(GetBitU(rtcm.Buff[:], i, 6))
		i += 6
		week = int(GetBitU(rtcm.Buff[:], i, 12))
		i += 12
		eph.Iode = int(GetBitU(rtcm.Buff[:], i, 10))
		i += 10
		eph.Sva = int(GetBitU(rtcm.Buff[:], i, 8))
		i += 8
		eph.Idot = float64(GetBits(rtcm.Buff[:], i, 14)) * P2_43 * SC2RAD
		i += 14
		toc = float64(GetBitU(rtcm.Buff[:], i, 14)) * 60.0
		i += 14
		eph.F2 = float64(GetBits(rtcm.Buff[:], i, 6)) * P2_59
		i += 6
		eph.F1 = float64(GetBits(rtcm.Buff[:], i, 21)) * P2_46
		i += 21
		eph.F0 = float64(GetBits(rtcm.Buff[:], i, 31)) * P2_34
		i += 31
		eph.Crs = float64(GetBits(rtcm.Buff[:], i, 16)) * P2_5
		i += 16
		eph.Deln = float64(GetBits(rtcm.Buff[:], i, 16)) * P2_43 * SC2RAD
		i += 16
		eph.M0 = float64(GetBits(rtcm.Buff[:], i, 32)) * P2_31 * SC2RAD
		i += 32
		eph.Cuc = float64(GetBits(rtcm.Buff[:], i, 16)) * P2_29
		i += 16
		eph.E = float64(GetBitU(rtcm.Buff[:], i, 32)) * P2_33
		i += 32
		eph.Cus = float64(GetBits(rtcm.Buff[:], i, 16)) * P2_29
		i += 16
		sqrtA = float64(GetBitU(rtcm.Buff[:], i, 32)) * P2_19
		i += 32
		eph.Toes = float64(GetBitU(rtcm.Buff[:], i, 14)) * 60.0
		i += 14
		eph.Cic = float64(GetBits(rtcm.Buff[:], i, 16)) * P2_29
		i += 16
		eph.OMG0 = float64(GetBits(rtcm.Buff[:], i, 32)) * P2_31 * SC2RAD
		i += 32
		eph.Cis = float64(GetBits(rtcm.Buff[:], i, 16)) * P2_29
		i += 16
		eph.I0 = float64(GetBits(rtcm.Buff[:], i, 32)) * P2_31 * SC2RAD
		i += 32
		eph.Crc = float64(GetBits(rtcm.Buff[:], i, 16)) * P2_5
		i += 16
		eph.Omg = float64(GetBits(rtcm.Buff[:], i, 32)) * P2_31 * SC2RAD
		i += 32
		eph.OMGd = float64(GetBits(rtcm.Buff[:], i, 24)) * P2_43 * SC2RAD
		i += 24
		eph.Tgd[0] = float64(GetBits(rtcm.Buff[:], i, 10)) * P2_32
		i += 10 /* E5a/E1 */
		eph.Tgd[1] = float64(GetBits(rtcm.Buff[:], i, 10)) * P2_32
		i += 10 /* E5b/E1 */
		e5b_hs = int(GetBitU(rtcm.Buff[:], i, 2))
		i += 2 /* E5b OSHS */
		e5b_dvs = int(GetBitU(rtcm.Buff[:], i, 1))
		i += 1 /* E5b OSDVS */
		e1_hs = int(GetBitU(rtcm.Buff[:], i, 2))
		i += 2 /* E1 OSHS */
		e1_dvs = int(GetBitU(rtcm.Buff[:], i, 1))
		i += 1 /* E1 OSDVS */
	} else {
		Trace(2, "rtcm3 1046 length error: len=%d\n", rtcm.MsgLen)
		return -1
	}
	Trace(4, "decode_type1046: prn=%d iode=%d toe=%.0f\n", prn, eph.Iode, eph.Toes)

	if rtcm.OutType > 0 {
		rtcm.MsgType += fmt.Sprintf(" prn=%2d iode=%3d week=%d toe=%6.0f toc=%6.0f hs=%d %d dvs=%d %d",
			prn, eph.Iode, week, eph.Toes, toc, e5b_hs, e1_hs, e5b_dvs, e1_dvs)
	}
	if sat = SatNo(sys, prn); sat == 0 {
		Trace(2, "rtcm3 1046 satellite number error: prn=%d\n", prn)
		return -1
	}
	if strings.Contains(rtcm.Opt, "-GALFNAV") {
		return 0
	}
	eph.Sat = sat
	eph.Week = week + 1024 /* gal-week = gst-week + 1024 */
	if rtcm.Time.Time == 0 {
		rtcm.Time = Utc2GpsT(TimeGet())
	}
	tt = TimeDiff(GpsT2Time(eph.Week, eph.Toes), rtcm.Time)
	if tt < -302400.0 {
		eph.Week++
	} else if tt >= 302400.0 {
		eph.Week--
	}
	eph.Toe = GpsT2Time(eph.Week, eph.Toes)
	eph.Toc = GpsT2Time(eph.Week, toc)
	eph.Ttr = rtcm.Time
	eph.A = sqrtA * sqrtA
	eph.Svh = (e5b_hs << 7) + (e5b_dvs << 6) + (e1_hs << 1) + (e1_dvs << 0)
	eph.Code = (1 << 0) + (1 << 2) + (1 << 9) /* data source = I/NAV+E1+E5b */
	eph.Iodc = eph.Iode
	if !strings.Contains(rtcm.Opt, "-EPHALL") {
		if eph.Iode == rtcm.NavData.Ephs[sat-1].Iode {
			return 0 /* unchanged */
		}
	}
	rtcm.NavData.Ephs[sat-1] = eph
	rtcm.EphSat = sat
	rtcm.EphSet = 0 /* I/NAV */
	return 2
}

/* decode type 1042/63: Beidou ephemerides -----------------------------------*/
func (rtcm *Rtcm) decode_type1042() int {
	var (
		eph            Eph
		toc, sqrtA, tt float64
		i              int = 24 + 12
		prn, sat, week int
	)
	sys := SYS_CMP

	if i+499 <= rtcm.MsgLen*8 {
		prn = int(GetBitU(rtcm.Buff[:], i, 6))
		i += 6
		week = int(GetBitU(rtcm.Buff[:], i, 13))
		i += 13
		eph.Sva = int(GetBitU(rtcm.Buff[:], i, 4))
		i += 4
		eph.Idot = float64(GetBits(rtcm.Buff[:], i, 14)) * P2_43 * SC2RAD
		i += 14
		eph.Iode = int(GetBitU(rtcm.Buff[:], i, 5))
		i += 5 /* AODE */
		toc = float64(GetBitU(rtcm.Buff[:], i, 17)) * 8.0
		i += 17
		eph.F2 = float64(GetBits(rtcm.Buff[:], i, 11)) * P2_66
		i += 11
		eph.F1 = float64(GetBits(rtcm.Buff[:], i, 22)) * P2_50
		i += 22
		eph.F0 = float64(GetBits(rtcm.Buff[:], i, 24)) * P2_33
		i += 24
		eph.Iodc = int(GetBitU(rtcm.Buff[:], i, 5))
		i += 5 /* AODC */
		eph.Crs = float64(GetBits(rtcm.Buff[:], i, 18)) * P2_6
		i += 18
		eph.Deln = float64(GetBits(rtcm.Buff[:], i, 16)) * P2_43 * SC2RAD
		i += 16
		eph.M0 = float64(GetBits(rtcm.Buff[:], i, 32)) * P2_31 * SC2RAD
		i += 32
		eph.Cuc = float64(GetBits(rtcm.Buff[:], i, 18)) * P2_31
		i += 18
		eph.E = float64(GetBitU(rtcm.Buff[:], i, 32)) * P2_33
		i += 32
		eph.Cus = float64(GetBits(rtcm.Buff[:], i, 18)) * P2_31
		i += 18
		sqrtA = float64(GetBitU(rtcm.Buff[:], i, 32)) * P2_19
		i += 32
		eph.Toes = float64(GetBitU(rtcm.Buff[:], i, 17)) * 8.0
		i += 17
		eph.Cic = float64(GetBits(rtcm.Buff[:], i, 18)) * P2_31
		i += 18
		eph.OMG0 = float64(GetBits(rtcm.Buff[:], i, 32)) * P2_31 * SC2RAD
		i += 32
		eph.Cis = float64(GetBits(rtcm.Buff[:], i, 18)) * P2_31
		i += 18
		eph.I0 = float64(GetBits(rtcm.Buff[:], i, 32)) * P2_31 * SC2RAD
		i += 32
		eph.Crc = float64(GetBits(rtcm.Buff[:], i, 18)) * P2_6
		i += 18
		eph.Omg = float64(GetBits(rtcm.Buff[:], i, 32)) * P2_31 * SC2RAD
		i += 32
		eph.OMGd = float64(GetBits(rtcm.Buff[:], i, 24)) * P2_43 * SC2RAD
		i += 24
		eph.Tgd[0] = float64(GetBits(rtcm.Buff[:], i, 10)) * (1e-10)
		i += 10
		eph.Tgd[1] = float64(GetBits(rtcm.Buff[:], i, 10)) * (1e-10)
		i += 10
		eph.Svh = int(GetBitU(rtcm.Buff[:], i, 1))
		i += 1
	} else {
		Trace(2, "rtcm3 1042 length error: len=%d\n", rtcm.MsgLen)
		return -1
	}
	Trace(4, "decode_type1042: prn=%d iode=%d toe=%.0f\n", prn, eph.Iode, eph.Toes)

	if rtcm.OutType > 0 {
		rtcm.MsgType += fmt.Sprintf(" prn=%2d iode=%3d iodc=%3d week=%d toe=%6.0f toc=%6.0f svh=%02X",
			prn, eph.Iode, eph.Iodc, week, eph.Toes, toc, eph.Svh)
	}
	if sat = SatNo(sys, prn); sat == 0 {
		Trace(2, "rtcm3 1042 satellite number error: prn=%d\n", prn)
		return -1
	}
	eph.Sat = sat
	eph.Week = AdjBDTWeek(week)
	if rtcm.Time.Time == 0 {
		rtcm.Time = Utc2GpsT(TimeGet())
	}
	tt = TimeDiff(BDT2GpsT(BDT2Time(eph.Week, eph.Toes)), rtcm.Time)
	if tt < -302400.0 {
		eph.Week++
	} else if tt >= 302400.0 {
		eph.Week--
	}
	eph.Toe = BDT2GpsT(BDT2Time(eph.Week, eph.Toes)) /* bdt . gpst */
	eph.Toc = BDT2GpsT(BDT2Time(eph.Week, toc))      /* bdt . gpst */
	eph.Ttr = rtcm.Time
	eph.A = sqrtA * sqrtA
	if !strings.Contains(rtcm.Opt, "-EPHALL") {
		if TimeDiff(eph.Toe, rtcm.NavData.Ephs[sat-1].Toe) == 0.0 &&
			eph.Iode == rtcm.NavData.Ephs[sat-1].Iode &&
			eph.Iodc == rtcm.NavData.Ephs[sat-1].Iodc {
			return 0 /* unchanged */
		}
	}
	rtcm.NavData.Ephs[sat-1] = eph
	rtcm.EphSet = 0
	rtcm.EphSat = sat
	return 2
}

/* decode SSR message epoch time ---------------------------------------------*/
func (rtcm *Rtcm) DecodeSsrEpoch(sys, subtype int) int {
	var tod, tow float64
	i := 24 + 12

	if subtype == 0 { /* RTCM SSR */

		if sys == SYS_GLO {
			tod = float64(GetBitU(rtcm.Buff[:], i, 17))
			i += 17
			rtcm.AdjDay_Glot(tod)
		} else {
			tow = float64(GetBitU(rtcm.Buff[:], i, 20))
			i += 20
			rtcm.AdjWeek(tow)
		}
	} else { /* IGS SSR */
		i += 3 + 8
		tow = float64(GetBitU(rtcm.Buff[:], i, 20))
		i += 20
		rtcm.AdjWeek(tow)
	}
	return i
}

/* decode SSR 1,4 message header ---------------------------------------------*/
func (rtcm *Rtcm) decode_ssr1_head(sys, subtype int, sync,
	iod *int, udint *float64, refd, hsize *int) int {
	var (
		tstr                         string
		nsat, udi, provid, solid, ns int
	)
	i := 24 + 12
	if subtype == 0 { /* RTCM SSR */
		ns = 6
		if sys == SYS_QZS {
			ns = 4
		}
		if sys == SYS_GLO {
			if i+53+ns > rtcm.MsgLen*8 {
				return -1
			}
		} else {
			if i+50+ns > rtcm.MsgLen*8 {
				return -1
			}
		}
	} else { /* IGS SSR */
		ns = 6
		if i+3+8+50+ns > rtcm.MsgLen*8 {
			return -1
		}
	}
	i = rtcm.DecodeSsrEpoch(sys, subtype)
	udi = int(GetBitU(rtcm.Buff[:], i, 4))
	i += 4
	*sync = int(GetBitU(rtcm.Buff[:], i, 1))
	i += 1
	if subtype == 0 { /* RTCM SSR */
		*refd = int(GetBitU(rtcm.Buff[:], i, 1))
		i += 1 /* satellite ref datum */
	}
	*iod = int(GetBitU(rtcm.Buff[:], i, 4))
	i += 4 /* IOD SSR */
	provid = int(GetBitU(rtcm.Buff[:], i, 16))
	i += 16 /* provider ID */
	solid = int(GetBitU(rtcm.Buff[:], i, 4))
	i += 4           /* solution ID */
	if subtype > 0 { /* IGS SSR */
		*refd = int(GetBitU(rtcm.Buff[:], i, 1))
		i += 1 /* global/regional CRS indicator */
	}
	nsat = int(GetBitU(rtcm.Buff[:], i, ns))
	i += ns
	*udint = ssrudint[udi]

	Time2Str(rtcm.Time, &tstr, 2)
	Trace(5, "decode_ssr1_head: time=%s sys=%d subtype=%d nsat=%d sync=%d iod=%d provid=%d solid=%d\n", tstr, sys, subtype, nsat, *sync, *iod, provid, solid)

	if rtcm.OutType > 0 {
		rtcm.MsgType += fmt.Sprintf(" %s nsat=%2d iod=%2d udi=%2d sync=%d", tstr, nsat, *iod, udi,
			*sync)
	}
	*hsize = i
	return nsat
}

/* decode SSR 2,3,5,6 message header -----------------------------------------*/
func (rtcm *Rtcm) decode_ssr2_head(sys, subtype int, sync,
	iod *int, udint *float64, hsize *int) int {
	var (
		tstr                         string
		nsat, udi, provid, solid, ns int
	)
	i := 24 + 12
	if subtype == 0 { /* RTCM SSR */
		ns = 6
		if sys == SYS_QZS {
			ns = 4
		}

		if sys == SYS_GLO {
			if i+52+ns > rtcm.MsgLen*8 {
				return -1
			}
		} else {
			if i+49+ns > rtcm.MsgLen*8 {
				return -1
			}
		}
	} else {
		ns = 6
		if i+3+8+49+ns > rtcm.MsgLen*8 {
			return -1
		}
	}
	i = rtcm.DecodeSsrEpoch(sys, subtype)
	udi = int(GetBitU(rtcm.Buff[:], i, 4))
	i += 4
	*sync = int(GetBitU(rtcm.Buff[:], i, 1))
	i += 1
	*iod = int(GetBitU(rtcm.Buff[:], i, 4))
	i += 4
	provid = int(GetBitU(rtcm.Buff[:], i, 16))
	i += 16 /* provider ID */
	solid = int(GetBitU(rtcm.Buff[:], i, 4))
	i += 4 /* solution ID */
	nsat = int(GetBitU(rtcm.Buff[:], i, ns))
	i += ns
	*udint = ssrudint[udi]

	Time2Str(rtcm.Time, &tstr, 2)
	Trace(5, "decode_ssr2_head: time=%s sys=%d subtype=%d nsat=%d sync=%d iod=%d provid=%d solid=%d\n", tstr, sys, subtype, nsat, *sync, *iod, provid, solid)

	if rtcm.OutType > 0 {
		rtcm.MsgType += fmt.Sprintf(" %s nsat=%2d iod=%2d udi=%2d sync=%d", tstr, nsat, *iod, udi,
			*sync)
	}
	*hsize = i
	return nsat
}

/* decode SSR 1: orbit corrections -------------------------------------------*/
func (rtcm *Rtcm) decode_ssr1(sys, subtype int) int {
	var (
		udint                                     float64
		deph, ddeph                               [3]float64
		i, j, k, ctype, sync, iod, nsat, prn, sat int
		iode, iodcrc, refd, np, ni, nj, offp      int
	)

	ctype = int(GetBitU(rtcm.Buff[:], 24, 12))

	if nsat = rtcm.decode_ssr1_head(sys, subtype, &sync, &iod, &udint, &refd, &i); nsat < 0 {
		Trace(2, "rtcm3 %d length error: len=%d\n", ctype, rtcm.MsgLen)
		return -1
	}
	var miss bool
	np, ni, nj, offp, _, miss = selectsys(sys)
	if miss {
		if sync > 0 {
			return 0
		} else {
			return 10
		}
	}
	if subtype > 0 { /* IGS SSR */
		np, ni, nj, offp = 6, 8, 0, 0
		switch sys {
		case SYS_CMP:
			offp = 0
		case SYS_SBS:
			offp = 119
		}
	}
	for j = 0; j < nsat && i+121+np+ni+nj <= rtcm.MsgLen*8; j++ {
		prn = int(GetBitU(rtcm.Buff[:], i, np)) + offp
		i += np
		iode = int(GetBitU(rtcm.Buff[:], i, ni))
		i += ni
		iodcrc = int(GetBitU(rtcm.Buff[:], i, nj))
		i += nj
		deph[0] = float64(GetBits(rtcm.Buff[:], i, 22)) * (1e-4)
		i += 22
		deph[1] = float64(GetBits(rtcm.Buff[:], i, 20)) * (4e-4)
		i += 20
		deph[2] = float64(GetBits(rtcm.Buff[:], i, 20)) * (4e-4)
		i += 20
		ddeph[0] = float64(GetBits(rtcm.Buff[:], i, 21)) * (1e-6)
		i += 21
		ddeph[1] = float64(GetBits(rtcm.Buff[:], i, 19)) * (4e-6)
		i += 19
		ddeph[2] = float64(GetBits(rtcm.Buff[:], i, 19)) * (4e-6)
		i += 19

		if sat = SatNo(sys, prn); sat == 0 {
			Trace(2, "rtcm3 %d satellite number error: prn=%d\n", ctype, prn)
			continue
		}
		rtcm.Ssr[sat-1].T0[0] = rtcm.Time
		rtcm.Ssr[sat-1].Udi[0] = udint
		rtcm.Ssr[sat-1].Iod[0] = iod
		rtcm.Ssr[sat-1].Iode = iode     /* SBAS/BDS: toe/t0 modulo */
		rtcm.Ssr[sat-1].IodCrc = iodcrc /* SBAS/BDS: IOD CRC */
		rtcm.Ssr[sat-1].Refd = refd

		for k = 0; k < 3; k++ {
			rtcm.Ssr[sat-1].Deph[k] = deph[k]
			rtcm.Ssr[sat-1].Ddeph[k] = ddeph[k]
		}
		rtcm.Ssr[sat-1].Update = 1
	}

	if sync > 0 {
		return 0
	} else {
		return 10
	}
}

/* decode SSR 2: clock corrections -------------------------------------------*/
func (rtcm *Rtcm) decode_ssr2(sys, subtype int) int {
	var (
		udint                     float64
		dclk                      [3]float64
		i, j, k, ctype, sync, iod int
		nsat, prn, sat            int
	)

	ctype = int(GetBitU(rtcm.Buff[:], 24, 12))

	if nsat = rtcm.decode_ssr2_head(sys, subtype, &sync, &iod, &udint, &i); nsat < 0 {
		Trace(2, "rtcm3 %d length error: len=%d\n", ctype, rtcm.MsgLen)
		return -1
	}
	np, _, _, offp, _, miss := selectsys(sys)
	if miss {
		if sync > 0 {
			return 0
		} else {
			return 10
		}
	}
	if subtype > 0 { /* IGS SSR */
		np = 6
		switch sys {
		case SYS_CMP:
			offp = 0
		case SYS_SBS:
			offp = 119
		}
	}
	for j = 0; j < nsat && i+70+np <= rtcm.MsgLen*8; j++ {
		prn = int(GetBitU(rtcm.Buff[:], i, np)) + offp
		i += np
		dclk[0] = float64(GetBits(rtcm.Buff[:], i, 22)) * 1e-4
		i += 22
		dclk[1] = float64(GetBits(rtcm.Buff[:], i, 21)) * 1e-6
		i += 21
		dclk[2] = float64(GetBits(rtcm.Buff[:], i, 27)) * 2e-8
		i += 27

		if sat = SatNo(sys, prn); sat == 0 {
			Trace(2, "rtcm3 %d satellite number error: prn=%d\n", ctype, prn)
			continue
		}
		rtcm.Ssr[sat-1].T0[1] = rtcm.Time
		rtcm.Ssr[sat-1].Udi[1] = udint
		rtcm.Ssr[sat-1].Iod[1] = iod

		for k = 0; k < 3; k++ {
			rtcm.Ssr[sat-1].Dclk[k] = dclk[k]
		}
		rtcm.Ssr[sat-1].Update = 1
	}
	if sync > 0 {
		return 0
	} else {
		return 10
	}
}

/* decode SSR 3: satellite code biases ---------------------------------------*/
func (rtcm *Rtcm) decode_ssr3(sys, subtype int) int {
	var (
		udint, bias                float64
		cbias                      [MAXCODE]float64
		i, j, k, ctype, mode, sync int
		iod, nsat, prn, sat, nbias int
	)

	ctype = int(GetBitU(rtcm.Buff[:], 24, 12))

	if nsat = rtcm.decode_ssr2_head(sys, subtype, &sync, &iod, &udint, &i); nsat < 0 {
		Trace(2, "rtcm3 %d length error: len=%d\n", ctype, rtcm.MsgLen)
		return -1
	}
	np, _, _, offp, sigs, miss := selectsys(sys)
	if miss {
		if sync > 0 {
			return 0
		} else {
			return 10
		}
	}
	if subtype > 0 { /* IGS SSR */
		np = 6
		switch sys {
		case SYS_CMP:
			offp = 0
		case SYS_SBS:
			offp = 119
		}
	}
	for j = 0; j < nsat && i+5+np <= rtcm.MsgLen*8; j++ {
		prn = int(GetBitU(rtcm.Buff[:], i, np)) + offp
		i += np
		nbias = int(GetBitU(rtcm.Buff[:], i, 5))
		i += 5

		for k = 0; k < int(MAXCODE); k++ {
			cbias[k] = 0.0
		}
		for k = 0; k < nbias && i+19 <= rtcm.MsgLen*8; k++ {
			mode = int(GetBitU(rtcm.Buff[:], i, 5))
			i += 5
			bias = float64(GetBits(rtcm.Buff[:], i, 14)) * 0.01
			i += 14
			if sigs[mode] > 0 {
				cbias[sigs[mode]-1] = float64(bias)
			} else {
				Trace(2, "rtcm3 %d not supported mode: mode=%d\n", ctype, mode)
			}
		}
		if sat = SatNo(sys, prn); sat == 0 {
			Trace(2, "rtcm3 %d satellite number error: prn=%d\n", ctype, prn)
			continue
		}
		rtcm.Ssr[sat-1].T0[4] = rtcm.Time
		rtcm.Ssr[sat-1].Udi[4] = udint
		rtcm.Ssr[sat-1].Iod[4] = iod

		for k = 0; k < int(MAXCODE); k++ {
			rtcm.Ssr[sat-1].Cbias[k] = float32(cbias[k])
		}
		rtcm.Ssr[sat-1].Update = 1
	}
	if sync > 0 {
		return 0
	} else {
		return 10
	}
}

/* decode SSR 4: combined orbit and clock corrections ------------------------*/
func (rtcm *Rtcm) decode_ssr4(sys, subtype int) int {
	var (
		udint                           float64
		deph, ddeph, dclk               [3]float64
		i, j, k, ctype, nsat, sync, iod int
		prn, sat, iode, iodcrc, refd    int
	)

	ctype = int(GetBitU(rtcm.Buff[:], 24, 12))

	if nsat = rtcm.decode_ssr1_head(sys, subtype, &sync, &iod, &udint, &refd, &i); nsat < 0 {
		Trace(2, "rtcm3 %d length error: len=%d\n", ctype, rtcm.MsgLen)
		return -1
	}
	np, ni, nj, offp, _, miss := selectsys(sys)
	if miss {
		if sync > 0 {
			return 0
		} else {
			return 10
		}
	}
	if subtype > 0 { /* IGS SSR */
		np, ni, nj, offp = 6, 8, 0, 0
		switch sys {
		case SYS_CMP:
			offp = 0
		case SYS_SBS:
			offp = 119
		}
	}
	for j = 0; j < nsat && i+191+np+ni+nj <= rtcm.MsgLen*8; j++ {
		prn = int(GetBitU(rtcm.Buff[:], i, np)) + offp
		i += np
		iode = int(GetBitU(rtcm.Buff[:], i, ni))
		i += ni
		iodcrc = int(GetBitU(rtcm.Buff[:], i, nj))
		i += nj
		deph[0] = float64(GetBits(rtcm.Buff[:], i, 22)) * 1e-4
		i += 22
		deph[1] = float64(GetBits(rtcm.Buff[:], i, 20)) * 4e-4
		i += 20
		deph[2] = float64(GetBits(rtcm.Buff[:], i, 20)) * 4e-4
		i += 20
		ddeph[0] = float64(GetBits(rtcm.Buff[:], i, 21)) * 1e-6
		i += 21
		ddeph[1] = float64(GetBits(rtcm.Buff[:], i, 19)) * 4e-6
		i += 19
		ddeph[2] = float64(GetBits(rtcm.Buff[:], i, 19)) * 4e-6
		i += 19

		dclk[0] = float64(GetBits(rtcm.Buff[:], i, 22)) * 1e-4
		i += 22
		dclk[1] = float64(GetBits(rtcm.Buff[:], i, 21)) * 1e-6
		i += 21
		dclk[2] = float64(GetBits(rtcm.Buff[:], i, 27)) * 2e-8
		i += 27

		if sat = SatNo(sys, prn); sat == 0 {
			Trace(2, "rtcm3 %d satellite number error: prn=%d\n", ctype, prn)
			continue
		}
		rtcm.Ssr[sat-1].T0[0], rtcm.Ssr[sat-1].T0[1] = rtcm.Time, rtcm.Time
		rtcm.Ssr[sat-1].Udi[0], rtcm.Ssr[sat-1].Udi[1] = udint, udint
		rtcm.Ssr[sat-1].Iod[0], rtcm.Ssr[sat-1].Iod[1] = iod, iod
		rtcm.Ssr[sat-1].Iode = iode
		rtcm.Ssr[sat-1].IodCrc = iodcrc
		rtcm.Ssr[sat-1].Refd = refd

		for k = 0; k < 3; k++ {
			rtcm.Ssr[sat-1].Deph[k] = deph[k]
			rtcm.Ssr[sat-1].Ddeph[k] = ddeph[k]
			rtcm.Ssr[sat-1].Dclk[k] = dclk[k]
		}
		rtcm.Ssr[sat-1].Update = 1
	}
	if sync > 0 {
		return 0
	} else {
		return 10
	}
}

/* decode SSR 5: URA ---------------------------------------------------------*/
func (rtcm *Rtcm) decode_ssr5(sys, subtype int) int {
	var (
		udint                   float64
		i, j, ctype, nsat, sync int
		iod, prn, sat, ura      int
	)

	ctype = int(GetBitU(rtcm.Buff[:], 24, 12))

	if nsat = rtcm.decode_ssr2_head(sys, subtype, &sync, &iod, &udint, &i); nsat < 0 {
		Trace(2, "rtcm3 %d length error: len=%d\n", ctype, rtcm.MsgLen)
		return -1
	}
	np, _, _, offp, _, miss := selectsys(sys)
	if miss {
		if sync > 0 {
			return 0
		} else {
			return 10
		}
	}
	if subtype > 0 { /* IGS SSR */
		np = 6
		switch sys {
		case SYS_CMP:
			offp = 0
		case SYS_SBS:
			offp = 119
		}
	}
	for j = 0; j < nsat && i+6+np <= rtcm.MsgLen*8; j++ {
		prn = int(GetBitU(rtcm.Buff[:], i, np)) + offp
		i += np
		ura = int(GetBitU(rtcm.Buff[:], i, 6))
		i += 6

		if sat = SatNo(sys, prn); nsat == 0 {
			Trace(2, "rtcm3 %d satellite number error: prn=%d\n", ctype, prn)
			continue
		}
		rtcm.Ssr[sat-1].T0[3] = rtcm.Time
		rtcm.Ssr[sat-1].Udi[3] = udint
		rtcm.Ssr[sat-1].Iod[3] = iod
		rtcm.Ssr[sat-1].Ura = ura
		rtcm.Ssr[sat-1].Update = 1
	}
	if sync > 0 {
		return 0
	} else {
		return 10
	}
}

/* decode SSR 6: high rate clock correction ----------------------------------*/
func (rtcm *Rtcm) decode_ssr6(sys, subtype int) int {
	var (
		udint, hrclk            float64
		i, j, ctype, nsat, sync int
		iod, prn, sat, np, offp int
	)

	ctype = int(GetBitU(rtcm.Buff[:], 24, 12))

	if nsat = rtcm.decode_ssr2_head(sys, subtype, &sync, &iod, &udint, &i); nsat < 0 {
		Trace(2, "rtcm3 %d length error: len=%d\n", ctype, rtcm.MsgLen)
		return -1
	}
	np, _, _, offp, _, miss := selectsys(sys)
	if miss {
		if sync > 0 {
			return 0
		} else {
			return 10
		}
	}
	if subtype > 0 { /* IGS SSR */
		np = 6
		switch sys {
		case SYS_CMP:
			offp = 0
		case SYS_SBS:
			offp = 119
		}
	}
	for j = 0; j < nsat && i+22+np <= rtcm.MsgLen*8; j++ {
		prn = int(GetBitU(rtcm.Buff[:], i, np)) + offp
		i += np
		hrclk = float64(GetBits(rtcm.Buff[:], i, 22)) * 1e-4
		i += 22

		if sat = SatNo(sys, prn); sat == 0 {
			Trace(2, "rtcm3 %d satellite number error: prn=%d\n", ctype, prn)
			continue
		}
		rtcm.Ssr[sat-1].T0[2] = rtcm.Time
		rtcm.Ssr[sat-1].Udi[2] = udint
		rtcm.Ssr[sat-1].Iod[2] = iod
		rtcm.Ssr[sat-1].Brclk = hrclk
		rtcm.Ssr[sat-1].Update = 1
	}
	if sync > 0 {
		return 0
	} else {
		return 10
	}
}

/* decode SSR 7 message header -----------------------------------------------*/
func (rtcm *Rtcm) decode_ssr7_head(sys, subtype int, sync,
	iod *int, udint *float64, dispe, mw, hsize *int) int {
	var (
		tstr                         string
		nsat, udi, provid, solid, ns int
	)
	i := 24 + 12
	if subtype == 0 { /* RTCM SSR */
		ns = 6
		if sys == SYS_QZS {
			ns = 4
		}
		isys := 51
		if sys == SYS_GLO {
			isys = 54
		}
		if (i + isys + ns) > rtcm.MsgLen*8 {
			return -1
		}
	} else { /* IGS SSR */
		ns = 6
		if i+3+8+51+ns > rtcm.MsgLen*8 {
			return -1
		}
	}
	i = rtcm.DecodeSsrEpoch(sys, subtype)
	udi = int(GetBitU(rtcm.Buff[:], i, 4))
	i += 4
	*sync = int(GetBitU(rtcm.Buff[:], i, 1))
	i += 1
	*iod = int(GetBitU(rtcm.Buff[:], i, 4))
	i += 4
	provid = int(GetBitU(rtcm.Buff[:], i, 16))
	i += 16 /* provider ID */
	solid = int(GetBitU(rtcm.Buff[:], i, 4))
	i += 4 /* solution ID */
	*dispe = int(GetBitU(rtcm.Buff[:], i, 1))
	i += 1 /* dispersive bias consistency ind */
	*mw = int(GetBitU(rtcm.Buff[:], i, 1))
	i += 1 /* MW consistency indicator */
	nsat = int(GetBitU(rtcm.Buff[:], i, ns))
	i += ns
	*udint = ssrudint[udi]

	Time2Str(rtcm.Time, &tstr, 2)
	Trace(5, "decode_ssr7_head: time=%s sys=%d subtype=%d nsat=%d sync=%d iod=%d provid=%d solid=%d\n", tstr, sys, subtype, nsat, *sync, *iod, provid, solid)

	if rtcm.OutType > 0 {

		rtcm.MsgType += fmt.Sprintf(" %s nsat=%2d iod=%2d udi=%2d sync=%d", tstr, nsat, *iod, udi,
			*sync)
	}
	*hsize = i
	return nsat
}

/* decode SSR 7: phase bias --------------------------------------------------*/
func (rtcm *Rtcm) decode_ssr7(sys, subtype int) int {
	var (
		udint, bias, std                           float64
		pbias, stdpb                               [MAXCODE]float64
		i, j, k, ctype, mode, sync, iod, nsat, prn int
		sat, nbias, mw, sii, swl                   int
		dispe, sdc, yaw_ang, yaw_rate              int
	)

	ctype = int(GetBitU(rtcm.Buff[:], 24, 12))

	if nsat = rtcm.decode_ssr7_head(sys, subtype, &sync, &iod, &udint, &dispe, &mw, &i); nsat < 0 {
		Trace(5, "rtcm3 %d length error: len=%d\n", ctype, rtcm.MsgLen)
		return -1
	}
	np, _, _, offp, sigs, miss := selectsys(sys)
	if miss {
		if sync > 0 {
			return 0
		} else {
			return 10
		}
	}
	if subtype > 0 { /* IGS SSR */
		np = 6
		switch sys {
		case SYS_CMP:
			offp = 0
		case SYS_SBS:
			offp = 119
		}
	}
	for j = 0; j < nsat && i+5+17+np <= rtcm.MsgLen*8; j++ {
		prn = int(GetBitU(rtcm.Buff[:], i, np)) + offp
		i += np
		nbias = int(GetBitU(rtcm.Buff[:], i, 5))
		i += 5
		yaw_ang = int(GetBitU(rtcm.Buff[:], i, 9))
		i += 9
		yaw_rate = int(GetBits(rtcm.Buff[:], i, 8))
		i += 8

		for k = 0; k < int(MAXCODE); k++ {
			pbias[k], stdpb[k] = 0.0, 0.0
		}
		isubtype := 32
		if subtype == 0 {
			isubtype = 49
		}
		for k = 0; k < nbias && i+isubtype <= rtcm.MsgLen*8; k++ {
			mode = int(GetBitU(rtcm.Buff[:], i, 5))
			i += 5
			sii = int(GetBitU(rtcm.Buff[:], i, 1))
			i += 1 /* integer-indicator */
			swl = int(GetBitU(rtcm.Buff[:], i, 2))
			i += 2 /* WL integer-indicator */
			sdc = int(GetBitU(rtcm.Buff[:], i, 4))
			i += 4 /* discontinuity counter */
			bias = float64(GetBits(rtcm.Buff[:], i, 20))
			i += 20 /* phase bias (m) */
			if subtype == 0 {
				std = float64(GetBitU(rtcm.Buff[:], i, 17))
				i += 17 /* phase bias std-dev (m) */
			}
			if sigs[mode] > 0 {
				pbias[sigs[mode]-1] = bias * 0.0001 /* (m) */
				stdpb[sigs[mode]-1] = std * 0.0001  /* (m) */
			} else {
				Trace(2, "rtcm3 %d not supported mode: mode=%d\n", ctype, mode)
			}
		}
		if sat = SatNo(sys, prn); sat == 0 {
			Trace(2, "rtcm3 %d satellite number error: prn=%d%d%d%d\n", ctype, prn, sii, swl, sdc)
			continue
		}
		rtcm.Ssr[sat-1].T0[5] = rtcm.Time
		rtcm.Ssr[sat-1].Udi[5] = udint
		rtcm.Ssr[sat-1].Iod[5] = iod
		rtcm.Ssr[sat-1].Yaw_ang = float64(yaw_ang) / 256.0 * 180.0    /* (deg) */
		rtcm.Ssr[sat-1].Yaw_rate = float64(yaw_rate) / 8192.0 * 180.0 /* (deg/s) */

		for k = 0; k < MAXCODE; k++ {
			rtcm.Ssr[sat-1].Pbias[k] = pbias[k]
			rtcm.Ssr[sat-1].Stdpb[k] = float32(stdpb[k])
		}
	}
	return 20
}

/* get signal index ----------------------------------------------------------*/
func SigIndex(sys int, code []uint8, n int, opt string, idx []int) {
	var (
		i, nex, pri  int
		pri_h, index [8]int
		ex           [32]int
	)

	/* test code priority */
	for i = 0; i < n; i++ {
		if code[i] == 0 {
			continue
		}

		if idx[i] >= NFREQ { /* save as extended signal if idx >= NFREQ */
			ex[i] = 1
			continue
		}
		/* code priority */
		pri = GetCodePri(sys, code[i], opt)

		/* select highest priority signal */
		if pri > pri_h[idx[i]] {
			if index[idx[i]] > 0 {
				ex[index[idx[i]]-1] = 1
			}
			pri_h[idx[i]] = pri
			index[idx[i]] = i + 1
		} else {
			ex[i] = 1
		}
	}
	/* signal index in obs data */
	for i, nex = 0, 0; i < n; i++ {
		if ex[i] == 0 {
		} else if nex < NEXOBS {
			idx[i] = NFREQ + nex
			nex++
		} else { /* no space in obs data */
			Trace(2, "rtcm msm: no space in obs data sys=%d code=%d\n", sys, code[i])
			idx[i] = -1
		}
	}
}

/* save obs data in MSM message ----------------------------------------------*/
func (rtcm *Rtcm) SaveMsmObs(sys int, h *Msm_h, r, pr, cp, rr, rrf, cnr []float64, lock, ex, half []int) {
	var (
		sig                 [32][]rune
		tt, freq            float64
		code                [32]uint8
		msm_type            []rune = nil
		i, j, k, ctype, prn int
		sat, fcn, index     int
		idx                 [32]int
	)

	ctype = int(GetBitU(rtcm.Buff[:], 24, 12))

	switch sys {
	case SYS_GPS:
		msm_type = []rune(rtcm.MsmType[0])
	case SYS_GLO:
		msm_type = []rune(rtcm.MsmType[1])
	case SYS_GAL:
		msm_type = []rune(rtcm.MsmType[2])
	case SYS_QZS:
		msm_type = []rune(rtcm.MsmType[3])
	case SYS_SBS:
		msm_type = []rune(rtcm.MsmType[4])
	case SYS_CMP:
		msm_type = []rune(rtcm.MsmType[5])
	case SYS_IRN:
		msm_type = []rune(rtcm.MsmType[6])
	}
	/* id to signal */
	for i = 0; i < int(h.nsig); i++ {
		switch sys {
		case SYS_GPS:
			sig[i] = []rune(msm_sig_gps[h.sigs[i]-1])
		case SYS_GLO:
			sig[i] = []rune(msm_sig_glo[h.sigs[i]-1])
		case SYS_GAL:
			sig[i] = []rune(msm_sig_gal[h.sigs[i]-1])
		case SYS_QZS:
			sig[i] = []rune(msm_sig_qzs[h.sigs[i]-1])
		case SYS_SBS:
			sig[i] = []rune(msm_sig_sbs[h.sigs[i]-1])
		case SYS_CMP:
			sig[i] = []rune(msm_sig_cmp[h.sigs[i]-1])
		case SYS_IRN:
			sig[i] = []rune(msm_sig_irn[h.sigs[i]-1])
		default:
			sig[i] = []rune("")
		}
		/* signal to rinex obs type */
		code[i] = Obs2Code(string(sig[i]))
		idx[i] = Code2Idx(sys, code[i])

		str := ""
		if i < int(h.nsig)-1 {
			str = ","
		}
		if code[i] != CODE_NONE {
			if msm_type != nil {
				switch sys {
				case SYS_GPS:
					rtcm.MsmType[0] += fmt.Sprintf("L%s%s", string(sig[i]), str)
				case SYS_GLO:
					rtcm.MsmType[1] += fmt.Sprintf("L%s%s", string(sig[i]), str)
				case SYS_GAL:
					rtcm.MsmType[2] += fmt.Sprintf("L%s%s", string(sig[i]), str)
				case SYS_QZS:
					rtcm.MsmType[3] += fmt.Sprintf("L%s%s", string(sig[i]), str)
				case SYS_SBS:
					rtcm.MsmType[4] += fmt.Sprintf("L%s%s", string(sig[i]), str)
				case SYS_CMP:
					rtcm.MsmType[5] += fmt.Sprintf("L%s%s", string(sig[i]), str)
				case SYS_IRN:
					rtcm.MsmType[6] += fmt.Sprintf("L%s%s", string(sig[i]), str)
				}
			}
		} else {
			if msm_type != nil {
				switch sys {
				case SYS_GPS:
					rtcm.MsmType[0] += fmt.Sprintf("(%d)%s", h.sigs[i], str)
				case SYS_GLO:
					rtcm.MsmType[1] += fmt.Sprintf("(%d)%s", h.sigs[i], str)
				case SYS_GAL:
					rtcm.MsmType[2] += fmt.Sprintf("(%d)%s", h.sigs[i], str)
				case SYS_QZS:
					rtcm.MsmType[3] += fmt.Sprintf("(%d)%s", h.sigs[i], str)
				case SYS_SBS:
					rtcm.MsmType[4] += fmt.Sprintf("(%d)%s", h.sigs[i], str)
				case SYS_CMP:
					rtcm.MsmType[5] += fmt.Sprintf("(%d)%s", h.sigs[i], str)
				case SYS_IRN:
					rtcm.MsmType[6] += fmt.Sprintf("(%d)%s", h.sigs[i], str)
				}
			}

			Trace(2, "rtcm3 %d: unknown signal id=%2d\n", ctype, h.sigs[i])
		}
	}
	Trace(4, "rtcm3 %d: signals=%s\n", ctype, string(msm_type))

	/* get signal index */
	SigIndex(sys, code[:], int(h.nsig), rtcm.Opt, idx[:])

	for i, j = 0, 0; i < int(h.nsat); i++ {

		prn = int(h.sats[i])
		switch sys {
		case SYS_QZS:
			prn += MINPRNQZS - 1
		case SYS_SBS:
			prn += MINPRNSBS - 1
		}

		if sat = SatNo(sys, prn); sat > 0 {
			tt = TimeDiff(rtcm.ObsData.Data[0].Time, rtcm.Time)
			if rtcm.ObsFlag > 0 || math.Abs(tt) > 1e-9 {
				rtcm.ObsData.Data, rtcm.ObsFlag = nil, 0
			}
			index = rtcm.ObsData.ObsIndex(rtcm.Time, sat)
		} else {
			Trace(2, "rtcm3 %d satellite error: prn=%d\n", ctype, prn)
		}
		fcn = 0
		if sys == SYS_GLO {
			fcn = -8 /* no glonass fcn info */
			switch {
			case ex != nil && ex[i] <= 13:
				fcn = ex[i] - 7
				if rtcm.NavData.Glo_fcn[prn-1] == 0 {
					rtcm.NavData.Glo_fcn[prn-1] = fcn + 8 /* fcn+8 */
				}
			case rtcm.NavData.Geph[prn-1].Sat == sat:
				fcn = rtcm.NavData.Geph[prn-1].Frq
			case rtcm.NavData.Glo_fcn[prn-1] > 0:
				fcn = rtcm.NavData.Glo_fcn[prn-1] - 8
			}
		}
		for k = 0; k < int(h.nsig); k++ {
			if h.cellmask[k+i*int(h.nsig)] == 0 {
				continue
			}

			if sat > 0 && index >= 0 && idx[k] >= 0 {
				freq = Code2Freq(sys, code[k], fcn)
				if fcn < -7 {
					freq = 0.0
				}

				/* pseudorange (m) */
				if r[i] != 0.0 && pr[j] > -1e12 {
					rtcm.ObsData.Data[index].P[idx[k]] = r[i] + pr[j]
				}
				/* carrier-phase (cycle) */
				if r[i] != 0.0 && cp[j] > -1e12 {
					rtcm.ObsData.Data[index].L[idx[k]] = (r[i] + cp[j]) * freq / CLIGHT
				}
				/* doppler (hz) */
				if rr != nil && rrf != nil && rrf[j] > -1e12 {
					rtcm.ObsData.Data[index].D[idx[k]] =
						(-(rr[i] + rrf[j]) * freq / CLIGHT)
				}
				ihalf := 0
				if half[j] > 0 {
					ihalf = 3
				}
				rtcm.ObsData.Data[index].LLI[idx[k]] = uint8(rtcm.LossOfLock(sat, idx[k], lock[j]) + ihalf)
				rtcm.ObsData.Data[index].SNR[idx[k]] = uint16(cnr[j]/float64(SNR_UNIT) + 0.5)
				rtcm.ObsData.Data[index].Code[idx[k]] = code[k]
			}
			j++
		}
	}
}

/* decode type MSM message header --------------------------------------------*/
func (rtcm *Rtcm) decode_msm_head(sys int, sync, iod *int, h *Msm_h, hsize *int) int {
	var (
		h0                                Msm_h
		tow, tod                          float64
		tstr                              string
		j, dow, mask, staid, ctype, ncell int
	)
	i := 24

	ctype = int(GetBitU(rtcm.Buff[:], i, 12))
	i += 12

	*h = h0
	if i+157 <= rtcm.MsgLen*8 {
		staid = int(GetBitU(rtcm.Buff[:], i, 12))
		i += 12

		switch sys {
		case SYS_GLO:
			dow = int(GetBitU(rtcm.Buff[:], i, 3))
			i += 3
			tod = float64(GetBitU(rtcm.Buff[:], i, 27)) * 0.001
			i += 27
			rtcm.AdjDay_Glot(tod)
		case SYS_CMP:
			tow = float64(GetBitU(rtcm.Buff[:], i, 30)) * 0.001
			i += 30
			tow += 14.0 /* BDT . GPST */
			rtcm.AdjWeek(tow)
		default:
			tow = float64(GetBitU(rtcm.Buff[:], i, 30)) * 0.001
			i += 30
			rtcm.AdjWeek(tow)
		}
		*sync = int(GetBitU(rtcm.Buff[:], i, 1))
		i += 1
		*iod = int(GetBitU(rtcm.Buff[:], i, 3))
		i += 3
		h.time_s = uint8(GetBitU(rtcm.Buff[:], i, 7))
		i += 7
		h.clk_str = uint8(GetBitU(rtcm.Buff[:], i, 2))
		i += 2
		h.clk_ext = uint8(GetBitU(rtcm.Buff[:], i, 2))
		i += 2
		h.smooth = uint8(GetBitU(rtcm.Buff[:], i, 1))
		i += 1
		h.tint_s = uint8(GetBitU(rtcm.Buff[:], i, 3))
		i += 3
		for j = 1; j <= 64; j++ {
			mask = int(GetBitU(rtcm.Buff[:], i, 1))
			i += 1
			if mask > 0 {
				h.sats[h.nsat] = uint8(j)
				h.nsat++
			}
		}
		for j = 1; j <= 32; j++ {
			mask = int(GetBitU(rtcm.Buff[:], i, 1))
			i += 1
			if mask > 0 {
				h.sigs[h.nsig] = uint8(j)
				h.nsig++
			}
		}
	} else {
		Trace(2, "rtcm3 %d length error: len=%d\n", ctype, rtcm.MsgLen)
		return -1
	}
	/* test station id */
	if rtcm.test_staid(staid) == 0 {
		return -1
	}

	if h.nsat*h.nsig > 64 {
		Trace(2, "rtcm3 %d number of sats and sigs error: nsat=%d nsig=%d%d\n",
			ctype, h.nsat, h.nsig, dow)
		return -1
	}
	if i+int(h.nsat*h.nsig) > rtcm.MsgLen*8 {
		Trace(2, "rtcm3 %d length error: len=%d nsat=%d nsig=%d\n", ctype,
			rtcm.MsgLen, h.nsat, h.nsig)
		return -1
	}
	for j = 0; j < int(h.nsat*h.nsig); j++ {
		h.cellmask[j] = uint8(GetBitU(rtcm.Buff[:], i, 1))
		i += 1
		if h.cellmask[j] > 0 {
			ncell++
		}
	}
	*hsize = i

	Time2Str(rtcm.Time, &tstr, 2)
	Trace(4, "decode_head_msm: time=%s sys=%d staid=%d nsat=%d nsig=%d sync=%d iod=%d ncell=%d\n",
		tstr, sys, staid, h.nsat, h.nsig, *sync, *iod, ncell)

	if rtcm.OutType > 0 {
		rtcm.MsgType += fmt.Sprintf(" staid=%4d %s nsat=%2d nsig=%2d iod=%2d ncell=%2d sync=%d",
			staid, tstr, h.nsat, h.nsig, *iod, ncell, *sync)
	}
	return ncell
}

/* decode unsupported MSM message --------------------------------------------*/
func (rtcm *Rtcm) decode_msm0(sys int) int {
	var (
		h            Msm_h
		i, sync, iod int
	)
	if rtcm.decode_msm_head(sys, &sync, &iod, &h, &i) < 0 {
		return -1
	}
	return retsync(sync, &rtcm.ObsFlag)
}

/* decode MSM 4: full pseudorange and phaserange plus CNR --------------------*/
func (rtcm *Rtcm) decode_msm4(sys int) int {
	var (
		h                                                   Msm_h
		r, pr, cp, cnr                                      [64]float64
		i, j, ctype, sync, iod, ncell, rng, rng_m, prv, cpv int
		lock, half                                          [64]int
	)

	ctype = int(GetBitU(rtcm.Buff[:], 24, 12))

	/* decode msm header */
	if ncell = rtcm.decode_msm_head(sys, &sync, &iod, &h, &i); ncell < 0 {
		return -1
	}

	if i+int(h.nsat)*18+ncell*48 > rtcm.MsgLen*8 {
		Trace(2, "rtcm3 %d length error: nsat=%d ncell=%d len=%d\n", ctype, h.nsat,
			ncell, rtcm.MsgLen)
		return -1
	}
	for j = 0; j < int(h.nsat); j++ {
		r[j] = 0.0
	}
	for j = 0; j < ncell; j++ {
		pr[j], cp[j] = -1e16, -1e16
	}

	/* decode satellite data */
	for j = 0; j < int(h.nsat); j++ { /* range */
		rng = int(GetBitU(rtcm.Buff[:], i, 8))
		i += 8
		if rng != 255 {
			r[j] = float64(rng) * RANGE_MS
		}
	}
	for j = 0; j < int(h.nsat); j++ {
		rng_m = int(GetBitU(rtcm.Buff[:], i, 10))
		i += 10
		if r[j] != 0.0 {
			r[j] += float64(rng_m) * P2_10 * RANGE_MS
		}
	}
	/* decode signal data */
	for j = 0; j < ncell; j++ { /* pseudorange */
		prv = int(GetBits(rtcm.Buff[:], i, 15))
		i += 15
		if prv != -16384 {
			pr[j] = float64(prv) * P2_24 * RANGE_MS
		}
	}
	for j = 0; j < ncell; j++ { /* phaserange */
		cpv = int(GetBits(rtcm.Buff[:], i, 22))
		i += 22
		if cpv != -2097152 {
			cp[j] = float64(cpv) * P2_29 * RANGE_MS
		}
	}
	for j = 0; j < ncell; j++ { /* lock time */
		lock[j] = int(GetBitU(rtcm.Buff[:], i, 4))
		i += 4
	}
	for j = 0; j < ncell; j++ { /* half-cycle ambiguity */
		half[j] = int(GetBitU(rtcm.Buff[:], i, 1))
		i += 1
	}
	for j = 0; j < ncell; j++ { /* cnr */
		cnr[j] = float64(GetBitU(rtcm.Buff[:], i, 6)) * 1.0
		i += 6
	}
	/* save obs data in msm message */
	rtcm.SaveMsmObs(sys, &h, r[:], pr[:], cp[:], nil, nil, cnr[:], lock[:], nil, half[:])

	return retsync(sync, &rtcm.ObsFlag)
}

/* decode MSM 5: full pseudorange, phaserange, phaserangerate and CNR --------*/
func (rtcm *Rtcm) decode_msm5(sys int) int {
	var (
		h                                                              Msm_h
		r, rr, pr, cp, rrf, cnr                                        [64]float64
		i, j, ctype, sync, iod, ncell, rng, rng_m, rate, prv, cpv, rrv int
		lock, ex, half                                                 [64]int
	)

	ctype = int(GetBitU(rtcm.Buff[:], 24, 12))

	/* decode msm header */
	if ncell = rtcm.decode_msm_head(sys, &sync, &iod, &h, &i); ncell < 0 {
		return -1
	}

	if i+int(h.nsat)*36+ncell*63 > rtcm.MsgLen*8 {
		Trace(2, "rtcm3 %d length error: nsat=%d ncell=%d len=%d\n", ctype, h.nsat,
			ncell, rtcm.MsgLen)
		return -1
	}
	for j = 0; j < int(h.nsat); j++ {
		r[j], rr[j] = 0.0, 0.0
		ex[j] = 15
	}
	for j = 0; j < ncell; j++ {
		pr[j], cp[j], rrf[j] = -1e16, -1e16, -1e16
	}

	/* decode satellite data */
	for j = 0; j < int(h.nsat); j++ { /* range */
		rng = int(GetBitU(rtcm.Buff[:], i, 8))
		i += 8
		if rng != 255 {
			r[j] = float64(rng) * RANGE_MS
		}
	}
	for j = 0; j < int(h.nsat); j++ { /* extended info */
		ex[j] = int(GetBitU(rtcm.Buff[:], i, 4))
		i += 4
	}
	for j = 0; j < int(h.nsat); j++ {
		rng_m = int(GetBitU(rtcm.Buff[:], i, 10))
		i += 10
		if r[j] != 0.0 {
			r[j] += float64(rng_m) * P2_10 * RANGE_MS
		}
	}
	for j = 0; j < int(h.nsat); j++ { /* phaserangerate */
		rate = int(GetBits(rtcm.Buff[:], i, 14))
		i += 14
		if rate != -8192 {
			rr[j] = float64(rate) * 1.0
		}
	}
	/* decode signal data */
	for j = 0; j < ncell; j++ { /* pseudorange */
		prv = int(GetBits(rtcm.Buff[:], i, 15))
		i += 15
		if prv != -16384 {
			pr[j] = float64(prv) * P2_24 * RANGE_MS
		}
	}
	for j = 0; j < ncell; j++ { /* phaserange */
		cpv = int(GetBits(rtcm.Buff[:], i, 22))
		i += 22
		if cpv != -2097152 {
			cp[j] = float64(cpv) * P2_29 * RANGE_MS
		}
	}
	for j = 0; j < ncell; j++ { /* lock time */
		lock[j] = int(GetBitU(rtcm.Buff[:], i, 4))
		i += 4
	}
	for j = 0; j < ncell; j++ { /* half-cycle ambiguity */
		half[j] = int(GetBitU(rtcm.Buff[:], i, 1))
		i += 1
	}
	for j = 0; j < ncell; j++ { /* cnr */
		cnr[j] = float64(GetBitU(rtcm.Buff[:], i, 6)) * 1.0
		i += 6
	}
	for j = 0; j < ncell; j++ { /* phaserangerate */
		rrv = int(GetBits(rtcm.Buff[:], i, 15))
		i += 15
		if rrv != -16384 {
			rrf[j] = float64(rrv) * 0.0001
		}
	}
	/* save obs data in msm message */
	rtcm.SaveMsmObs(sys, &h, r[:], pr[:], cp[:], rr[:], rrf[:], cnr[:], lock[:], ex[:], half[:])

	return retsync(sync, &rtcm.ObsFlag)
}

/* decode MSM 6: full pseudorange and phaserange plus CNR (high-res) ---------*/
func (rtcm *Rtcm) decode_msm6(sys int) int {
	var (
		h                                                   Msm_h
		r, pr, cp, cnr                                      [64]float64
		i, j, ctype, sync, iod, ncell, rng, rng_m, prv, cpv int
		lock, half                                          [64]int
	)

	ctype = int(GetBitU(rtcm.Buff[:], 24, 12))

	/* decode msm header */
	if ncell = rtcm.decode_msm_head(sys, &sync, &iod, &h, &i); ncell < 0 {
		return -1
	}

	if i+int(h.nsat)*18+ncell*65 > rtcm.MsgLen*8 {
		Trace(2, "rtcm3 %d length error: nsat=%d ncell=%d len=%d\n", ctype, h.nsat,
			ncell, rtcm.MsgLen)
		return -1
	}
	for j = 0; j < int(h.nsat); j++ {
		r[j] = 0.0
	}
	for j = 0; j < ncell; j++ {
		pr[j], cp[j] = -1e16, -1e16
	}

	/* decode satellite data */
	for j = 0; j < int(h.nsat); j++ { /* range */
		rng = int(GetBitU(rtcm.Buff[:], i, 8))
		i += 8
		if rng != 255 {
			r[j] = float64(rng) * RANGE_MS
		}
	}
	for j = 0; j < int(h.nsat); j++ {
		rng_m = int(GetBitU(rtcm.Buff[:], i, 10))
		i += 10
		if r[j] != 0.0 {
			r[j] += float64(rng_m) * P2_10 * RANGE_MS
		}
	}
	/* decode signal data */
	for j = 0; j < ncell; j++ { /* pseudorange */
		prv = int(GetBits(rtcm.Buff[:], i, 20))
		i += 20
		if prv != -524288 {
			pr[j] = float64(prv) * P2_29 * RANGE_MS
		}
	}
	for j = 0; j < ncell; j++ { /* phaserange */
		cpv = int(GetBits(rtcm.Buff[:], i, 24))
		i += 24
		if cpv != -8388608 {
			cp[j] = float64(cpv) * P2_31 * RANGE_MS
		}
	}
	for j = 0; j < ncell; j++ { /* lock time */
		lock[j] = int(GetBitU(rtcm.Buff[:], i, 10))
		i += 10
	}
	for j = 0; j < ncell; j++ { /* half-cycle ambiguity */
		half[j] = int(GetBitU(rtcm.Buff[:], i, 1))
		i += 1
	}
	for j = 0; j < ncell; j++ { /* cnr */
		cnr[j] = float64(GetBitU(rtcm.Buff[:], i, 10)) * 0.0625
		i += 10
	}
	/* save obs data in msm message */
	rtcm.SaveMsmObs(sys, &h, r[:], pr[:], cp[:], nil, nil, cnr[:], lock[:], nil, half[:])

	return retsync(sync, &rtcm.ObsFlag)
}

/* decode MSM 7: full pseudorange, phaserange, phaserangerate and CNR (h-res) */
func (rtcm *Rtcm) decode_msm7(sys int) int {
	var (
		h                                                              Msm_h
		r, rr, pr, cp, rrf, cnr                                        [64]float64
		i, j, ctype, sync, iod, ncell, rng, rng_m, rate, prv, cpv, rrv int
		lock, ex, half                                                 [64]int
	)

	ctype = int(GetBitU(rtcm.Buff[:], 24, 12))

	/* decode msm header */
	if ncell = rtcm.decode_msm_head(sys, &sync, &iod, &h, &i); ncell < 0 {
		return -1
	}

	if i+int(h.nsat)*36+ncell*80 > rtcm.MsgLen*8 {
		Trace(2, "rtcm3 %d length error: nsat=%d ncell=%d len=%d\n", ctype, h.nsat,
			ncell, rtcm.MsgLen)
		return -1
	}
	for j = 0; j < int(h.nsat); j++ {
		r[j], rr[j] = 0.0, 0.0
		ex[j] = 15
	}
	for j = 0; j < ncell; j++ {
		pr[j], cp[j], rrf[j] = -1e16, -1e16, -1e16
	}

	/* decode satellite data */
	for j = 0; j < int(h.nsat); j++ { /* range */
		rng = int(GetBitU(rtcm.Buff[:], i, 8))
		i += 8
		if rng != 255 {
			r[j] = float64(rng) * RANGE_MS
		}
	}
	for j = 0; j < int(h.nsat); j++ { /* extended info */
		ex[j] = int(GetBitU(rtcm.Buff[:], i, 4))
		i += 4
	}
	for j = 0; j < int(h.nsat); j++ {
		rng_m = int(GetBitU(rtcm.Buff[:], i, 10))
		i += 10
		if r[j] != 0.0 {
			r[j] += float64(rng_m) * P2_10 * RANGE_MS
		}
	}
	for j = 0; j < int(h.nsat); j++ { /* phaserangerate */
		rate = int(GetBits(rtcm.Buff[:], i, 14))
		i += 14
		if rate != -8192 {
			rr[j] = float64(rate) * 1.0
		}
	}
	/* decode signal data */
	for j = 0; j < ncell; j++ { /* pseudorange */
		prv = int(GetBits(rtcm.Buff[:], i, 20))
		i += 20
		if prv != -524288 {
			pr[j] = float64(prv) * P2_29 * RANGE_MS
		}
	}
	for j = 0; j < ncell; j++ { /* phaserange */
		cpv = int(GetBits(rtcm.Buff[:], i, 24))
		i += 24
		if cpv != -8388608 {
			cp[j] = float64(cpv) * P2_31 * RANGE_MS
		}
	}
	for j = 0; j < ncell; j++ { /* lock time */
		lock[j] = int(GetBitU(rtcm.Buff[:], i, 10))
		i += 10
	}
	for j = 0; j < ncell; j++ { /* half-cycle amiguity */
		half[j] = int(GetBitU(rtcm.Buff[:], i, 1))
		i += 1
	}
	for j = 0; j < ncell; j++ { /* cnr */
		cnr[j] = float64(GetBitU(rtcm.Buff[:], i, 10)) * 0.0625
		i += 10
	}
	for j = 0; j < ncell; j++ { /* phaserangerate */
		rrv = int(GetBits(rtcm.Buff[:], i, 15))
		i += 15
		if rrv != -16384 {
			rrf[j] = float64(rrv) * 0.0001
		}
	}
	/* save obs data in msm message */
	rtcm.SaveMsmObs(sys, &h, r[:], pr[:], cp[:], rr[:], rrf[:], cnr[:], lock[:], ex[:], half[:])

	return retsync(sync, &rtcm.ObsFlag)
}

/* decode type 1230: GLONASS L1 and L2 code-phase biases ---------------------*/
func (rtcm *Rtcm) decode_type1230() int {
	var j, staid, align, mask, bias int
	i := 24 + 12
	if i+20 >= rtcm.MsgLen*8 {
		Trace(2, "rtcm3 1230: length error len=%d\n", rtcm.MsgLen)
		return -1
	}
	staid = int(GetBitU(rtcm.Buff[:], i, 12))
	i += 12
	align = int(GetBitU(rtcm.Buff[:], i, 1))
	i += 1 + 3
	mask = int(GetBitU(rtcm.Buff[:], i, 4))
	i += 4

	if rtcm.OutType > 0 {
		rtcm.MsgType += fmt.Sprintf(" staid=%4d align=%d mask=0x%X", staid, align, mask)
	}
	/* test station ID */
	if rtcm.test_staid(staid) == 0 {
		return -1
	}

	rtcm.StaPara.glo_cp_align = align
	for j = 0; j < 4; j++ {
		rtcm.StaPara.glo_cp_bias[j] = 0.0
	}
	for j = 0; j < 4 && i+16 <= rtcm.MsgLen*8; j++ {
		if mask&(1<<(3-j)) == 0 {
			continue
		}
		bias = int(GetBits(rtcm.Buff[:], i, 16))
		i += 16
		if bias != -32768 {
			rtcm.StaPara.glo_cp_bias[j] = float64(bias) * 0.02
		}
	}
	return 5
}

/* decode type 4073: proprietary message Mitsubishi Electric -----------------*/
func (rtcm *Rtcm) decode_type4073() int {
	i := 24 + 12
	subtype := int(GetBitU(rtcm.Buff[:], i, 4))
	i += 4

	if rtcm.OutType > 0 {
		rtcm.MsgType += fmt.Sprintf(" subtype=%d", subtype)
	}
	Trace(5, "rtcm3 4073: unsupported message subtype=%d\n", subtype)
	return 0
}

/* decode type 4076: proprietary message IGS ---------------------------------*/
func (rtcm *Rtcm) decode_type4076() int {
	var ver, subtype int
	i := 24 + 12
	if i+3+8 >= rtcm.MsgLen*8 {
		Trace(2, "rtcm3 4076: length error len=%d\n", rtcm.MsgLen)
		return -1
	}
	ver = int(GetBitU(rtcm.Buff[:], i, 3))
	i += 3
	subtype = int(GetBitU(rtcm.Buff[:], i, 8))
	i += 8

	if rtcm.OutType > 0 {
		rtcm.MsgType += fmt.Sprintf(" ver=%d subtype=%3d", ver, subtype)
	}
	switch subtype {
	case 21:
		return rtcm.decode_ssr1(SYS_GPS, subtype)
	case 22:
		return rtcm.decode_ssr2(SYS_GPS, subtype)
	case 23:
		return rtcm.decode_ssr4(SYS_GPS, subtype)
	case 24:
		return rtcm.decode_ssr6(SYS_GPS, subtype)
	case 25:
		return rtcm.decode_ssr3(SYS_GPS, subtype)
	case 26:
		return rtcm.decode_ssr7(SYS_GPS, subtype)
	case 27:
		return rtcm.decode_ssr5(SYS_GPS, subtype)
	case 41:
		return rtcm.decode_ssr1(SYS_GLO, subtype)
	case 42:
		return rtcm.decode_ssr2(SYS_GLO, subtype)
	case 43:
		return rtcm.decode_ssr4(SYS_GLO, subtype)
	case 44:
		return rtcm.decode_ssr6(SYS_GLO, subtype)
	case 45:
		return rtcm.decode_ssr3(SYS_GLO, subtype)
	case 46:
		return rtcm.decode_ssr7(SYS_GLO, subtype)
	case 47:
		return rtcm.decode_ssr5(SYS_GLO, subtype)
	case 61:
		return rtcm.decode_ssr1(SYS_GAL, subtype)
	case 62:
		return rtcm.decode_ssr2(SYS_GAL, subtype)
	case 63:
		return rtcm.decode_ssr4(SYS_GAL, subtype)
	case 64:
		return rtcm.decode_ssr6(SYS_GAL, subtype)
	case 65:
		return rtcm.decode_ssr3(SYS_GAL, subtype)
	case 66:
		return rtcm.decode_ssr7(SYS_GAL, subtype)
	case 67:
		return rtcm.decode_ssr5(SYS_GAL, subtype)
	case 81:
		return rtcm.decode_ssr1(SYS_QZS, subtype)
	case 82:
		return rtcm.decode_ssr2(SYS_QZS, subtype)
	case 83:
		return rtcm.decode_ssr4(SYS_QZS, subtype)
	case 84:
		return rtcm.decode_ssr6(SYS_QZS, subtype)
	case 85:
		return rtcm.decode_ssr3(SYS_QZS, subtype)
	case 86:
		return rtcm.decode_ssr7(SYS_QZS, subtype)
	case 87:
		return rtcm.decode_ssr5(SYS_QZS, subtype)
	case 101:
		return rtcm.decode_ssr1(SYS_CMP, subtype)
	case 102:
		return rtcm.decode_ssr2(SYS_CMP, subtype)
	case 103:
		return rtcm.decode_ssr4(SYS_CMP, subtype)
	case 104:
		return rtcm.decode_ssr6(SYS_CMP, subtype)
	case 105:
		return rtcm.decode_ssr3(SYS_CMP, subtype)
	case 106:
		return rtcm.decode_ssr7(SYS_CMP, subtype)
	case 107:
		return rtcm.decode_ssr5(SYS_CMP, subtype)
	case 121:
		return rtcm.decode_ssr1(SYS_SBS, subtype)
	case 122:
		return rtcm.decode_ssr2(SYS_SBS, subtype)
	case 123:
		return rtcm.decode_ssr4(SYS_SBS, subtype)
	case 124:
		return rtcm.decode_ssr6(SYS_SBS, subtype)
	case 125:
		return rtcm.decode_ssr3(SYS_SBS, subtype)
	case 126:
		return rtcm.decode_ssr7(SYS_SBS, subtype)
	case 127:
		return rtcm.decode_ssr5(SYS_SBS, subtype)
	}
	Trace(3, "rtcm3 4076: unsupported message subtype=%d\n", subtype)
	return 0
}

/* decode RTCM ver.3 message -------------------------------------------------*/
func (rtcm *Rtcm) DecodeRtcm3() int {
	var tow float64
	var ret, ctype, week int
	ctype = int(GetBitU(rtcm.Buff[:], 24, 12))
	Trace(4, "decode_rtcm3: len=%3d type=%d\n", rtcm.MsgLen, ctype)

	if rtcm.OutType > 0 {
		rtcm.MsgType += fmt.Sprintf("RTCM %4d (%4d):", ctype, rtcm.MsgLen)
	}
	/* real-time input option */
	if strings.Contains(rtcm.Opt, "-RT_INP") {
		tow = Time2GpsT(Utc2GpsT(TimeGet()), &week)
		rtcm.Time = GpsT2Time(week, math.Floor(tow))
	}
	switch ctype {
	case 1001:
		ret = rtcm.decode_type1001()
		/* not supported */
	case 1002:
		ret = rtcm.decode_type1002()
	case 1003:
		ret = rtcm.decode_type1003()
		/* not supported */
	case 1004:
		ret = rtcm.decode_type1004()
	case 1005:
		ret = rtcm.decode_type1005()
	case 1006:
		ret = rtcm.decode_type1006()
	case 1007:
		ret = rtcm.decode_type1007()
	case 1008:
		ret = rtcm.decode_type1008()
	case 1009:
		ret = rtcm.decode_type1009()
		/* not supported */
	case 1010:
		ret = rtcm.decode_type1010()
	case 1011:
		ret = rtcm.decode_type1011()
		/* not supported */
	case 1012:
		ret = rtcm.decode_type1012()
	case 1013:
		ret = rtcm.decode_type1013()
		/* not supported */
	case 1019:
		ret = rtcm.decode_type1019()
	case 1020:
		ret = rtcm.decode_type1020()
	case 1021:
		ret = rtcm.decode_type1021()
		/* not supported */
	case 1022:
		ret = rtcm.decode_type1022()
		/* not supported */
	case 1023:
		ret = rtcm.decode_type1023()
		/* not supported */
	case 1024:
		ret = rtcm.decode_type1024()
		/* not supported */
	case 1025:
		ret = rtcm.decode_type1025()
		/* not supported */
	case 1026:
		ret = rtcm.decode_type1026()
		/* not supported */
	case 1027:
		ret = rtcm.decode_type1027()
		/* not supported */
	case 1029:
		ret = rtcm.decode_type1029()
	case 1030:
		ret = rtcm.decode_type1030()
		/* not supported */
	case 1031:
		ret = rtcm.decode_type1031()
		/* not supported */
	case 1032:
		ret = rtcm.decode_type1032()
		/* not supported */
	case 1033:
		ret = rtcm.decode_type1033()
	case 1034:
		ret = rtcm.decode_type1034()
		/* not supported */
	case 1035:
		ret = rtcm.decode_type1035()
		/* not supported */
	case 1037:
		ret = rtcm.decode_type1037()
		/* not supported */
	case 1038:
		ret = rtcm.decode_type1038()
		/* not supported */
	case 1039:
		ret = rtcm.decode_type1039()
		/* not supported */
	case 1041:
		ret = rtcm.decode_type1041()
	case 1044:
		ret = rtcm.decode_type1044()
	case 1045:
		ret = rtcm.decode_type1045()
	case 1046:
		ret = rtcm.decode_type1046()
	case 63:
		ret = rtcm.decode_type1042()
		/* RTCM draft */
	case 1042:
		ret = rtcm.decode_type1042()
	case 1057:
		ret = rtcm.decode_ssr1(SYS_GPS, 0)
	case 1058:
		ret = rtcm.decode_ssr2(SYS_GPS, 0)
	case 1059:
		ret = rtcm.decode_ssr3(SYS_GPS, 0)
	case 1060:
		ret = rtcm.decode_ssr4(SYS_GPS, 0)
	case 1061:
		ret = rtcm.decode_ssr5(SYS_GPS, 0)
	case 1062:
		ret = rtcm.decode_ssr6(SYS_GPS, 0)
	case 1063:
		ret = rtcm.decode_ssr1(SYS_GLO, 0)
	case 1064:
		ret = rtcm.decode_ssr2(SYS_GLO, 0)
	case 1065:
		ret = rtcm.decode_ssr3(SYS_GLO, 0)
	case 1066:
		ret = rtcm.decode_ssr4(SYS_GLO, 0)
	case 1067:
		ret = rtcm.decode_ssr5(SYS_GLO, 0)
	case 1068:
		ret = rtcm.decode_ssr6(SYS_GLO, 0)
	case 1071:
		ret = rtcm.decode_msm0(SYS_GPS)
		/* not supported */
	case 1072:
		ret = rtcm.decode_msm0(SYS_GPS)
		/* not supported */
	case 1073:
		ret = rtcm.decode_msm0(SYS_GPS)
		/* not supported */
	case 1074:
		ret = rtcm.decode_msm4(SYS_GPS)
	case 1075:
		ret = rtcm.decode_msm5(SYS_GPS)
	case 1076:
		ret = rtcm.decode_msm6(SYS_GPS)
	case 1077:
		ret = rtcm.decode_msm7(SYS_GPS)
	case 1081:
		ret = rtcm.decode_msm0(SYS_GLO)
		/* not supported */
	case 1082:
		ret = rtcm.decode_msm0(SYS_GLO)
		/* not supported */
	case 1083:
		ret = rtcm.decode_msm0(SYS_GLO)
		/* not supported */
	case 1084:
		ret = rtcm.decode_msm4(SYS_GLO)
	case 1085:
		ret = rtcm.decode_msm5(SYS_GLO)
	case 1086:
		ret = rtcm.decode_msm6(SYS_GLO)
	case 1087:
		ret = rtcm.decode_msm7(SYS_GLO)
	case 1091:
		ret = rtcm.decode_msm0(SYS_GAL)
		/* not supported */
	case 1092:
		ret = rtcm.decode_msm0(SYS_GAL)
		/* not supported */
	case 1093:
		ret = rtcm.decode_msm0(SYS_GAL)
		/* not supported */
	case 1094:
		ret = rtcm.decode_msm4(SYS_GAL)
	case 1095:
		ret = rtcm.decode_msm5(SYS_GAL)
	case 1096:
		ret = rtcm.decode_msm6(SYS_GAL)
	case 1097:
		ret = rtcm.decode_msm7(SYS_GAL)
	case 1101:
		ret = rtcm.decode_msm0(SYS_SBS)
		/* not supported */
	case 1102:
		ret = rtcm.decode_msm0(SYS_SBS)
		/* not supported */
	case 1103:
		ret = rtcm.decode_msm0(SYS_SBS)
		/* not supported */
	case 1104:
		ret = rtcm.decode_msm4(SYS_SBS)
	case 1105:
		ret = rtcm.decode_msm5(SYS_SBS)
	case 1106:
		ret = rtcm.decode_msm6(SYS_SBS)
	case 1107:
		ret = rtcm.decode_msm7(SYS_SBS)
	case 1111:
		ret = rtcm.decode_msm0(SYS_QZS)
		/* not supported */
	case 1112:
		ret = rtcm.decode_msm0(SYS_QZS)
		/* not supported */
	case 1113:
		ret = rtcm.decode_msm0(SYS_QZS)
		/* not supported */
	case 1114:
		ret = rtcm.decode_msm4(SYS_QZS)
	case 1115:
		ret = rtcm.decode_msm5(SYS_QZS)
	case 1116:
		ret = rtcm.decode_msm6(SYS_QZS)
	case 1117:
		ret = rtcm.decode_msm7(SYS_QZS)
	case 1121:
		ret = rtcm.decode_msm0(SYS_CMP)
		/* not supported */
	case 1122:
		ret = rtcm.decode_msm0(SYS_CMP)
		/* not supported */
	case 1123:
		ret = rtcm.decode_msm0(SYS_CMP)
		/* not supported */
	case 1124:
		ret = rtcm.decode_msm4(SYS_CMP)
	case 1125:
		ret = rtcm.decode_msm5(SYS_CMP)
	case 1126:
		ret = rtcm.decode_msm6(SYS_CMP)
	case 1127:
		ret = rtcm.decode_msm7(SYS_CMP)
	case 1131:
		ret = rtcm.decode_msm0(SYS_IRN)
		/* not supported */
	case 1132:
		ret = rtcm.decode_msm0(SYS_IRN)
		/* not supported */
	case 1133:
		ret = rtcm.decode_msm0(SYS_IRN)
		/* not supported */
	case 1134:
		ret = rtcm.decode_msm4(SYS_IRN)

	case 1135:
		ret = rtcm.decode_msm5(SYS_IRN)
	case 1136:
		ret = rtcm.decode_msm6(SYS_IRN)
	case 1137:
		ret = rtcm.decode_msm7(SYS_IRN)
	case 1230:
		ret = rtcm.decode_type1230()
	case 1240:
		ret = rtcm.decode_ssr1(SYS_GAL, 0)
		/* draft */
	case 1241:
		ret = rtcm.decode_ssr2(SYS_GAL, 0)
		/* draft */
	case 1242:
		ret = rtcm.decode_ssr3(SYS_GAL, 0)
		/* draft */
	case 1243:
		ret = rtcm.decode_ssr4(SYS_GAL, 0)
		/* draft */
	case 1244:
		ret = rtcm.decode_ssr5(SYS_GAL, 0)
		/* draft */
	case 1245:
		ret = rtcm.decode_ssr6(SYS_GAL, 0)
		/* draft */
	case 1246:
		ret = rtcm.decode_ssr1(SYS_QZS, 0)
		/* draft */
	case 1247:
		ret = rtcm.decode_ssr2(SYS_QZS, 0)
		/* draft */
	case 1248:
		ret = rtcm.decode_ssr3(SYS_QZS, 0)
		/* draft */
	case 1249:
		ret = rtcm.decode_ssr4(SYS_QZS, 0)
		/* draft */
	case 1250:
		ret = rtcm.decode_ssr5(SYS_QZS, 0)
		/* draft */
	case 1251:
		ret = rtcm.decode_ssr6(SYS_QZS, 0)
		/* draft */
	case 1252:
		ret = rtcm.decode_ssr1(SYS_SBS, 0)
		/* draft */
	case 1253:
		ret = rtcm.decode_ssr2(SYS_SBS, 0)
		/* draft */
	case 1254:
		ret = rtcm.decode_ssr3(SYS_SBS, 0)
		/* draft */
	case 1255:
		ret = rtcm.decode_ssr4(SYS_SBS, 0)
		/* draft */
	case 1256:
		ret = rtcm.decode_ssr5(SYS_SBS, 0)
		/* draft */
	case 1257:
		ret = rtcm.decode_ssr6(SYS_SBS, 0)
		/* draft */
	case 1258:
		ret = rtcm.decode_ssr1(SYS_CMP, 0)
		/* draft */
	case 1259:
		ret = rtcm.decode_ssr2(SYS_CMP, 0)
		/* draft */
	case 1260:
		ret = rtcm.decode_ssr3(SYS_CMP, 0)
		/* draft */
	case 1261:
		ret = rtcm.decode_ssr4(SYS_CMP, 0)
		/* draft */
	case 1262:
		ret = rtcm.decode_ssr5(SYS_CMP, 0)
		/* draft */
	case 1263:
		ret = rtcm.decode_ssr6(SYS_CMP, 0)
		/* draft */
	case 11:
		ret = rtcm.decode_ssr7(SYS_GPS, 0)
		/* tentative */
	case 12:
		ret = rtcm.decode_ssr7(SYS_GAL, 0)
		/* tentative */
	case 13:
		ret = rtcm.decode_ssr7(SYS_QZS, 0)
		/* tentative */
	case 14:
		ret = rtcm.decode_ssr7(SYS_CMP, 0)
		/* tentative */
	case 4073:
		ret = rtcm.decode_type4073()

	case 4076:
		ret = rtcm.decode_type4076()

	}
	if ret >= 0 {
		if 1001 <= ctype && ctype <= 1299 {
			rtcm.Nmsg3[ctype-1000]++ /*   1-299 */
		} else if 4070 <= ctype && ctype <= 4099 {
			rtcm.Nmsg3[ctype-3770]++ /* 300-329 */
		} else {
			rtcm.Nmsg3[0]++ /* other */
		}
	}
	return ret
}
