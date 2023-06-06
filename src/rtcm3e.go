/*------------------------------------------------------------------------------
* rtcm3e.c : rtcm ver.3 message encoder functions
*
*          Copyright (C) 2012-2020 by T.TAKASU, All rights reserved.
*
* references :
*     see rtcm.c
*
* version : $Revision:$ $Date:$
* history : 2012/12/05 1.0  new
*           2012/12/16 1.1  fix bug on ssr high rate clock correction
*           2012/12/24 1.2  fix bug on msm carrier-phase offset correction
*                           fix bug on SBAS sat id in 1001-1004
*                           fix bug on carrier-phase in 1001-1004,1009-1012
*           2012/12/28 1.3  fix bug on compass carrier wave length
*           2013/01/18 1.4  fix bug on ssr message generation
*           2013/05/11 1.5  change type of arg value of setbig()
*           2013/05/19 1.5  gpst . bdt of time-tag in beidou msm message
*           2013/04/27 1.7  comply with rtcm 3.2 with amendment 1/2 (ref[15])
*                           delete MT 1046 according to ref [15]
*           2014/05/15 1.8  set NT field in MT 1020 glonass ephemeris
*           2014/12/06 1.9  support SBAS/BeiDou SSR messages (ref [16])
*                           fix bug on invalid staid in qzss ssr messages
*           2015/03/22 1.9  add handling of iodcrc for beidou/sbas ssr messages
*           2015/08/03 1.10 fix bug on wrong udint and iod in ssr 7.
*                           support rtcm ssr fcb message mt 2065-2069.
*           2015/09/07 1.11 add message count of MT 2000-2099
*           2015/10/21 1.12 add MT1046 support for IGS MGEX
*           2015/12/04 1.13 add MT63 beidou ephemeris (rtcm draft)
*                           fix bug on msm message generation of beidou
*                           fix bug on ssr 3 message generation (#321)
*           2016/06/12 1.14 fix bug on segmentation fault by generating msm1
*           2016/09/20 1.15 fix bug on MT1045 Galileo week rollover
*           2017/04/11 1.16 fix bug on gst-week in MT1045/1046
*           2018/10/10 1.17 merge changes for 2.4.2 p13
*                           change mt for ssr 7 phase biases
*           2019/05/10 1.21 save galileo E5b data to obs index 2
*           2020/11/30 1.22 support MT1230 GLONASS code-phase biases
*                           support MT1131-1137,1041 (NavIC MSM and ephemeris)
*                           support MT4076 IGS SSR
*                           fixed invalid delta clock C2 value for SSR 2 and 4
*                           delete SSR signal and tracking mode ID table
*                           use API code2idx() to get freq-index
*                           use API code2freq() to get carrier frequency
*                           use integer types in stdint.h
*		    2022/05/31 1.0  rewrite rtcm3e.c with golang by fxb
*-----------------------------------------------------------------------------*/
package gnssgo

import (
	"math"
	"strings"
)

/* constants and macros ------------------------------------------------------*/

func ROUND_I(x float64) int    { return int(math.Floor((x) + 0.5)) }
func ROUND_U(x float64) uint32 { return uint32(math.Floor(x + 0.5)) }

/* set sign-magnitude bits ---------------------------------------------------*/
func setbitg(buff []uint8, pos, len int, value int32) {
	if value < 0 {
		SetBitU(buff, pos, 1, 1)
		SetBitU(buff, pos+1, len-1, uint32(-value))
	} else {
		SetBitU(buff, pos, 1, 0)
		SetBitU(buff, pos+1, len-1, uint32(value))
	}
}

/* set signed 38 bit field ---------------------------------------------------*/
func set38bits(buff []uint8, pos int, value float64) {
	word_h := int(math.Floor(value / 64.0))
	word_l := uint32(value - float64(word_h)*64.0)
	SetBits(buff, pos, 32, int32(word_h))
	SetBitU(buff, pos+32, 6, word_l)
}

/* lock time -----------------------------------------------------------------*/
func locktime(time Gtime, lltime *Gtime, LLI uint8) int {
	if lltime.Time == 0 || (LLI&1) == 1 {
		*lltime = time
	}
	return int(TimeDiff(time, *lltime))
}

/* lock time in double -------------------------------------------------------*/
func locktime_d(time Gtime, lltime *Gtime, LLI uint8) float64 {
	if lltime.Time == 0 || LLI&1 == 1 {
		*lltime = time
	}
	return TimeDiff(time, *lltime)
}

/* GLONASS frequency channel number in RTCM (FCN+7,-1:error) -----------------*/
func (rtcm *Rtcm) FcnGlo(sat int) int {
	var prn int

	if SatSys(sat, &prn) != SYS_GLO {
		return -1
	}
	if rtcm.NavData.Geph[prn-1].Sat == sat {
		return rtcm.NavData.Geph[prn-1].Frq + 7
	}
	if rtcm.NavData.Glo_fcn[prn-1] > 0 { /* fcn+8 (0: no data) */
		return rtcm.NavData.Glo_fcn[prn-1] - 8 + 7
	}
	return -1
}

/* lock time indicator (ref [17] table 3.4-2) --------------------------------*/
func to_lock(lock int) int {
	if lock < 0 {
		return 0
	}
	if lock < 24 {
		return lock
	}
	if lock < 72 {
		return (lock + 24) / 2
	}
	if lock < 168 {
		return (lock + 120) / 4
	}
	if lock < 360 {
		return (lock + 408) / 8
	}
	if lock < 744 {
		return (lock + 1176) / 16
	}
	if lock < 937 {
		return (lock + 3096) / 32
	}
	return 127
}

/* MSM lock time indicator (ref [17] table 3.5-74) ---------------------------*/
func to_msm_lock(lock float64) int {
	if lock < 0.032 {
		return 0
	}
	if lock < 0.064 {
		return 1
	}
	if lock < 0.128 {
		return 2
	}
	if lock < 0.256 {
		return 3
	}
	if lock < 0.512 {
		return 4
	}
	if lock < 1.024 {
		return 5
	}
	if lock < 2.048 {
		return 6
	}
	if lock < 4.096 {
		return 7
	}
	if lock < 8.192 {
		return 8
	}
	if lock < 16.384 {
		return 9
	}
	if lock < 32.768 {
		return 10
	}
	if lock < 65.536 {
		return 11
	}
	if lock < 131.072 {
		return 12
	}
	if lock < 262.144 {
		return 13
	}
	if lock < 524.288 {
		return 14
	}
	return 15
}

/* MSM lock time indicator with extended-resolution (ref [17] table 3.5-76) --*/
func to_msm_lock_ex(lock float64) int {
	lock_ms := int(lock * 1000.0)

	if lock < 0.0 {
		return 0
	}
	if lock < 0.064 {
		return lock_ms
	}
	if lock < 0.128 {
		return (lock_ms + 64) / 2
	}
	if lock < 0.256 {
		return (lock_ms + 256) / 4
	}
	if lock < 0.512 {
		return (lock_ms + 768) / 8
	}
	if lock < 1.024 {
		return (lock_ms + 2048) / 16
	}
	if lock < 2.048 {
		return (lock_ms + 5120) / 32
	}
	if lock < 4.096 {
		return (lock_ms + 12288) / 64
	}
	if lock < 8.192 {
		return (lock_ms + 28672) / 128
	}
	if lock < 16.384 {
		return (lock_ms + 65536) / 256
	}
	if lock < 32.768 {
		return (lock_ms + 147456) / 512
	}
	if lock < 65.536 {
		return (lock_ms + 327680) / 1024
	}
	if lock < 131.072 {
		return (lock_ms + 720896) / 2048
	}
	if lock < 262.144 {
		return (lock_ms + 1572864) / 4096
	}
	if lock < 524.288 {
		return (lock_ms + 3407872) / 8192
	}
	if lock < 1048.576 {
		return (lock_ms + 7340032) / 16384
	}
	if lock < 2097.152 {
		return (lock_ms + 15728640) / 32768
	}
	if lock < 4194.304 {
		return (lock_ms + 33554432) / 65536
	}
	if lock < 8388.608 {
		return (lock_ms + 71303168) / 131072
	}
	if lock < 16777.216 {
		return (lock_ms + 150994944) / 262144
	}
	if lock < 33554.432 {
		return (lock_ms + 318767104) / 524288
	}
	if lock < 67108.864 {
		return (lock_ms + 671088640) / 1048576
	}
	return 704
}

/* L1 code indicator GPS -----------------------------------------------------*/
func to_code1_gps(code uint8) int {
	switch code {
	case CODE_L1C:
		return 0 /* L1 C/A */
	case CODE_L1P, CODE_L1W, CODE_L1Y, CODE_L1N:
		return 1 /* L1 P(Y) direct */
	}
	return 0
}

/* L2 code indicator GPS -----------------------------------------------------*/
func to_code2_gps(code uint8) int {
	switch code {
	case CODE_L2C, CODE_L2S, CODE_L2L, CODE_L2X:
		return 0 /* L2 C/A or L2C */
	case CODE_L2P, CODE_L2Y:
		return 1 /* L2 P(Y) direct */
	case CODE_L2D:
		return 2 /* L2 P(Y) cross-correlated */
	case CODE_L2W, CODE_L2N:
		return 3 /* L2 correlated P/Y */
	}
	return 0
}

/* L1 code indicator GLONASS -------------------------------------------------*/
func to_code1_glo(code uint8) int {
	switch code {
	case CODE_L1C:
		return 0 /* L1 C/A */
	case CODE_L1P:
		return 1 /* L1 P */
	}
	return 0
}

/* L2 code indicator GLONASS -------------------------------------------------*/
func to_code2_glo(code uint8) int {
	switch code {
	case CODE_L2C:
		return 0 /* L2 C/A */
	case CODE_L2P:
		return 1 /* L2 P */
	}
	return 0
}

/* carrier-phase - pseudorange in cycle --------------------------------------*/
func cp_pr(cp, pr_cyc float64) float64 {
	return math.Mod(cp-pr_cyc+750.0, 1500.0) - 750.0
}

/* generate obs field data GPS -----------------------------------------------*/
func (rtcm *Rtcm) GenObsGps(data *ObsD, code1, pr1, ppr1, lock1, amb, cnr1, code2, pr21, ppr2, lock2, cnr2 *int) {
	var (
		pr1c, ppr float64
		lt1, lt2  int
	)

	lam1 := CLIGHT / FREQ1
	lam2 := CLIGHT / FREQ2
	*pr1, *amb = 0, 0
	if ppr1 != nil {
		*ppr1 = 0xFFF80000 /* invalid values */
	}
	if pr21 != nil {
		*pr21 = 0xFFFFE000
	}
	if ppr2 != nil {
		*ppr2 = 0xFFF80000
	}

	/* L1 peudorange */
	if data.P[0] != 0.0 && data.Code[0] > 0 {
		*amb = int(math.Floor(data.P[0] / PRUNIT_GPS))
		*pr1 = ROUND_I((data.P[0] - float64(*amb)*PRUNIT_GPS) / 0.02)
		pr1c = float64(*pr1)*0.02 + float64(*amb)*PRUNIT_GPS
	}
	/* L1 phaserange - L1 pseudorange */
	if data.P[0] != 0.0 && data.L[0] != 0.0 && data.Code[0] > 0 {
		ppr = cp_pr(data.L[0], pr1c/lam1)
		if ppr1 != nil {
			*ppr1 = ROUND_I(ppr * lam1 / 0.0005)
		}
	}
	/* L2 -L1 pseudorange */
	if data.P[0] != 0.0 && data.P[1] != 0.0 && data.Code[0] > 0 && data.Code[1] > 0 &&
		math.Abs(data.P[1]-pr1c) <= 163.82 {
		if pr21 != nil {
			*pr21 = ROUND_I((data.P[1] - pr1c) / 0.02)
		}
	}
	/* L2 phaserange - L1 pseudorange */
	if data.P[0] != 0.0 && data.L[1] != 0.0 && data.Code[0] > 0 && data.Code[1] > 0 {
		ppr = cp_pr(data.L[1], pr1c/lam2)
		if ppr2 != nil {
			*ppr2 = ROUND_I(ppr * lam2 / 0.0005)
		}
	}
	lt1 = locktime(data.Time, &rtcm.Lltime[data.Sat-1][0], data.LLI[0])
	lt2 = locktime(data.Time, &rtcm.Lltime[data.Sat-1][1], data.LLI[1])

	if lock1 != nil {
		*lock1 = to_lock(lt1)
	}
	if lock2 != nil {
		*lock2 = to_lock(lt2)
	}
	if cnr1 != nil {
		*cnr1 = ROUND_I(float64(data.SNR[0]) * float64(SNR_UNIT) / 0.25)
	}
	if cnr2 != nil {
		*cnr2 = ROUND_I(float64(data.SNR[1]) * float64(SNR_UNIT) / 0.25)
	}
	if code1 != nil {
		*code1 = to_code1_gps(data.Code[0])
	}
	if code2 != nil {
		*code2 = to_code2_gps(data.Code[1])
	}
}

/* generate obs field data GLONASS -------------------------------------------*/
func (rtcm *Rtcm) GenObsGlo(data *ObsD, fcn int, code1, pr1, ppr1, lock1, amb, cnr1, code2, pr21, ppr2, lock2, cnr2 *int) {
	var (
		lam1, lam2, pr1c, ppr float64
		lt1, lt2              int
	)

	if fcn >= 0 { /* fcn+7 */
		lam1 = CLIGHT / (FREQ1_GLO + DFRQ1_GLO*float64(fcn-7))
		lam2 = CLIGHT / (FREQ2_GLO + DFRQ2_GLO*float64(fcn-7))
	}
	*pr1, *amb = 0, 0
	if ppr1 != nil {
		*ppr1 = 0xFFF80000 /* invalid values */
	}
	if pr21 != nil {
		*pr21 = 0xFFFFE000
	}
	if ppr2 != nil {
		*ppr2 = 0xFFF80000
	}

	/* L1 pseudorange */
	if data.P[0] != 0.0 {
		*amb = int(math.Floor(data.P[0] / PRUNIT_GLO))
		*pr1 = ROUND_I((data.P[0] - float64(*amb)*PRUNIT_GLO) / 0.02)
		pr1c = float64(*pr1)*0.02 + float64(*amb)*PRUNIT_GLO
	}
	/* L1 phaserange - L1 pseudorange */
	if data.P[0] != 0.0 && data.L[0] != 0.0 && data.Code[0] > 0 && lam1 > 0.0 {
		ppr = cp_pr(data.L[0], pr1c/lam1)
		if ppr1 != nil {
			*ppr1 = ROUND_I(ppr * lam1 / 0.0005)
		}
	}
	/* L2 -L1 pseudorange */
	if data.P[0] != 0.0 && data.P[1] != 0.0 && data.Code[0] > 0 && data.Code[1] > 0 &&
		math.Abs(data.P[1]-pr1c) <= 163.82 {
		if pr21 != nil {
			*pr21 = ROUND_I((data.P[1] - pr1c) / 0.02)
		}
	}
	/* L2 phaserange - L1 pseudorange */
	if data.P[0] != 0.0 && data.L[1] != 0.0 && data.Code[0] > 0 && data.Code[1] > 0 &&
		lam2 > 0.0 {
		ppr = cp_pr(data.L[1], pr1c/lam2)
		if ppr2 != nil {
			*ppr2 = ROUND_I(ppr * lam2 / 0.0005)
		}
	}
	lt1 = locktime(data.Time, &rtcm.Lltime[data.Sat-1][0], data.LLI[0])
	lt2 = locktime(data.Time, &rtcm.Lltime[data.Sat-1][1], data.LLI[1])

	if lock1 != nil {
		*lock1 = to_lock(lt1)
	}
	if lock2 != nil {
		*lock2 = to_lock(lt2)
	}
	if cnr1 != nil {
		*cnr1 = ROUND_I(float64(data.SNR[0]) * float64(SNR_UNIT) / 0.25)
	}
	if cnr2 != nil {
		*cnr2 = ROUND_I(float64(data.SNR[1]) * float64(SNR_UNIT) / 0.25)
	}
	if code1 != nil {
		*code1 = to_code1_glo(data.Code[0])
	}
	if code2 != nil {
		*code2 = to_code2_glo(data.Code[1])
	}
}

/* encode RTCM header --------------------------------------------------------*/
func (rtcm *Rtcm) encode_head(ctype, sys, sync, nsat int) int {
	var (
		tow         float64
		week, epoch int
	)
	i := 24
	Trace(4, "encode_head: type=%d sync=%d sys=%d nsat=%d\n", ctype, sync, sys, nsat)

	SetBitU(rtcm.Buff[:], i, 12, uint32(ctype))
	i += 12 /* message no */
	SetBitU(rtcm.Buff[:], i, 12, uint32(rtcm.StaId))
	i += 12 /* ref station id */

	if sys == SYS_GLO {
		tow = Time2GpsT(TimeAdd(GpsT2Utc(rtcm.Time), 10800.0), &week)
		epoch = ROUND_I(math.Mod(tow, 86400.0) / 0.001)
		SetBitU(rtcm.Buff[:], i, 27, uint32(epoch))
		i += 27 /* glonass epoch time */
	} else {
		tow = Time2GpsT(rtcm.Time, &week)
		epoch = ROUND_I(tow / 0.001)
		SetBitU(rtcm.Buff[:], i, 30, uint32(epoch))
		i += 30 /* gps epoch time */
	}
	SetBitU(rtcm.Buff[:], i, 1, uint32(sync))
	i += 1 /* synchronous gnss flag */
	SetBitU(rtcm.Buff[:], i, 5, uint32(nsat))
	i += 5 /* no of satellites */
	SetBitU(rtcm.Buff[:], i, 1, 0)
	i += 1 /* smoothing indicator */
	SetBitU(rtcm.Buff[:], i, 3, 0)
	i += 3 /* smoothing interval */
	return i
}

/* encode type 1001: basic L1-only GPS RTK observables -----------------------*/
func (rtcm *Rtcm) encode_type1001(sync int) int {
	var i, j, nsat, sys, prn, code1, pr1, ppr1, lock1, amb int

	Trace(3, "encode_type1001: sync=%d\n", sync)

	for j = 0; j < rtcm.ObsData.N() && nsat < MAXOBS; j++ {
		sys = SatSys(rtcm.ObsData.Data[j].Sat, &prn)
		if (sys & (SYS_GPS | SYS_SBS)) == 0 {
			continue
		}
		nsat++
	}
	/* encode header */
	i = rtcm.encode_head(1001, SYS_GPS, sync, nsat)

	for j = 0; j < rtcm.ObsData.N() && nsat < MAXOBS; j++ {
		sys = SatSys(rtcm.ObsData.Data[j].Sat, &prn)
		if (sys & (SYS_GPS | SYS_SBS)) == 0 {
			continue
		}

		if sys == SYS_SBS {
			prn -= 80 /* 40-58: sbas 120-138 */
		}

		/* generate obs field data gps */
		rtcm.GenObsGps(&rtcm.ObsData.Data[j], &code1, &pr1, &ppr1, &lock1, &amb, nil,
			nil, nil, nil, nil, nil)

		SetBitU(rtcm.Buff[:], i, 6, uint32(prn))
		i += 6
		SetBitU(rtcm.Buff[:], i, 1, uint32(code1))
		i += 1
		SetBitU(rtcm.Buff[:], i, 24, uint32(pr1))
		i += 24
		SetBits(rtcm.Buff[:], i, 20, int32(ppr1))
		i += 20
		SetBitU(rtcm.Buff[:], i, 7, uint32(lock1))
		i += 7
	}
	rtcm.Nbit = i
	return 1
}

/* encode type 1002: extended L1-only GPS RTK observables --------------------*/
func (rtcm *Rtcm) encode_type1002(sync int) int {
	var i, j, nsat, sys, prn, code1, pr1, ppr1, lock1, amb, cnr1 int

	Trace(3, "encode_type1002: sync=%d\n", sync)

	for j = 0; j < rtcm.ObsData.N() && nsat < MAXOBS; j++ {
		sys = SatSys(rtcm.ObsData.Data[j].Sat, &prn)
		if (sys & (SYS_GPS | SYS_SBS)) == 0 {
			continue
		}
		nsat++
	}
	/* encode header */
	i = rtcm.encode_head(1002, SYS_GPS, sync, nsat)

	for j = 0; j < rtcm.ObsData.N() && nsat < MAXOBS; j++ {
		sys = SatSys(rtcm.ObsData.Data[j].Sat, &prn)
		if (sys & (SYS_GPS | SYS_SBS)) == 0 {
			continue
		}

		if sys == SYS_SBS {
			prn -= 80 /* 40-58: sbas 120-138 */
		}

		/* generate obs field data gps */
		rtcm.GenObsGps(&rtcm.ObsData.Data[j], &code1, &pr1, &ppr1, &lock1, &amb, &cnr1,
			nil, nil, nil, nil, nil)

		SetBitU(rtcm.Buff[:], i, 6, uint32(prn))
		i += 6
		SetBitU(rtcm.Buff[:], i, 1, uint32(code1))
		i += 1
		SetBitU(rtcm.Buff[:], i, 24, uint32(pr1))
		i += 24
		SetBits(rtcm.Buff[:], i, 20, int32(ppr1))
		i += 20
		SetBitU(rtcm.Buff[:], i, 7, uint32(lock1))
		i += 7
		SetBitU(rtcm.Buff[:], i, 8, uint32(amb))
		i += 8
		SetBitU(rtcm.Buff[:], i, 8, uint32(cnr1))
		i += 8
	}
	rtcm.Nbit = i
	return 1
}

/* encode type 1003: basic L1&L2 GPS RTK observables -------------------------*/
func (rtcm *Rtcm) encode_type1003(sync int) int {
	var i, j, nsat, sys, prn, code1, pr1, ppr1, lock1, amb, code2, pr21, ppr2, lock2 int

	Trace(3, "encode_type1003: sync=%d\n", sync)

	for j = 0; j < rtcm.ObsData.N() && nsat < MAXOBS; j++ {
		sys = SatSys(rtcm.ObsData.Data[j].Sat, &prn)
		if (sys & (SYS_GPS | SYS_SBS)) == 0 {
			continue
		}
		nsat++
	}
	/* encode header */
	i = rtcm.encode_head(1003, SYS_GPS, sync, nsat)

	for j = 0; j < rtcm.ObsData.N() && nsat < MAXOBS; j++ {
		sys = SatSys(rtcm.ObsData.Data[j].Sat, &prn)
		if (sys & (SYS_GPS | SYS_SBS)) == 0 {
			continue
		}

		if sys == SYS_SBS {
			prn -= 80 /* 40-58: sbas 120-138 */
		}

		/* generate obs field data gps */
		rtcm.GenObsGps(&rtcm.ObsData.Data[j], &code1, &pr1, &ppr1, &lock1, &amb,
			nil, &code2, &pr21, &ppr2, &lock2, nil)

		SetBitU(rtcm.Buff[:], i, 6, uint32(prn))
		i += 6
		SetBitU(rtcm.Buff[:], i, 1, uint32(code1))
		i += 1
		SetBitU(rtcm.Buff[:], i, 24, uint32(pr1))
		i += 24
		SetBits(rtcm.Buff[:], i, 20, int32(ppr1))
		i += 20
		SetBitU(rtcm.Buff[:], i, 7, uint32(lock1))
		i += 7
		SetBitU(rtcm.Buff[:], i, 2, uint32(code2))
		i += 2
		SetBits(rtcm.Buff[:], i, 14, int32(pr21))
		i += 14
		SetBits(rtcm.Buff[:], i, 20, int32(ppr2))
		i += 20
		SetBitU(rtcm.Buff[:], i, 7, uint32(lock2))
		i += 7
	}
	rtcm.Nbit = i
	return 1
}

/* encode type 1004: extended L1&L2 GPS RTK observables ----------------------*/
func (rtcm *Rtcm) encode_type1004(sync int) int {
	var i, j, nsat, sys, prn, code1, pr1, ppr1, lock1, amb, cnr1, code2, pr21, ppr2, lock2, cnr2 int

	Trace(3, "encode_type1004: sync=%d\n", sync)

	for j = 0; j < rtcm.ObsData.N() && nsat < MAXOBS; j++ {
		sys = SatSys(rtcm.ObsData.Data[j].Sat, &prn)
		if (sys & (SYS_GPS | SYS_SBS)) == 0 {
			continue
		}
		nsat++
	}
	/* encode header */
	i = rtcm.encode_head(1004, SYS_GPS, sync, nsat)

	for j = 0; j < rtcm.ObsData.N() && nsat < MAXOBS; j++ {
		sys = SatSys(rtcm.ObsData.Data[j].Sat, &prn)
		if (sys & (SYS_GPS | SYS_SBS)) == 0 {
			continue
		}

		if sys == SYS_SBS {
			prn -= 80 /* 40-58: sbas 120-138 */
		}

		/* generate obs field data gps */
		rtcm.GenObsGps(&rtcm.ObsData.Data[j], &code1, &pr1, &ppr1, &lock1, &amb,
			&cnr1, &code2, &pr21, &ppr2, &lock2, &cnr2)

		SetBitU(rtcm.Buff[:], i, 6, uint32(prn))
		i += 6
		SetBitU(rtcm.Buff[:], i, 1, uint32(code1))
		i += 1
		SetBitU(rtcm.Buff[:], i, 24, uint32(pr1))
		i += 24
		SetBits(rtcm.Buff[:], i, 20, int32(ppr1))
		i += 20
		SetBitU(rtcm.Buff[:], i, 7, uint32(lock1))
		i += 7
		SetBitU(rtcm.Buff[:], i, 8, uint32(amb))
		i += 8
		SetBitU(rtcm.Buff[:], i, 8, uint32(cnr1))
		i += 8
		SetBitU(rtcm.Buff[:], i, 2, uint32(code2))
		i += 2
		SetBits(rtcm.Buff[:], i, 14, int32(pr21))
		i += 14
		SetBits(rtcm.Buff[:], i, 20, int32(ppr2))
		i += 20
		SetBitU(rtcm.Buff[:], i, 7, uint32(lock2))
		i += 7
		SetBitU(rtcm.Buff[:], i, 8, uint32(cnr2))
		i += 8
	}
	rtcm.Nbit = i
	return 1
}

/* encode type 1005: stationary RTK reference station ARP --------------------*/
func (rtcm *Rtcm) encode_type1005(sync int) int {
	p := rtcm.StaPara.Pos[:]
	i := 24

	Trace(3, "encode_type1005: sync=%d\n", sync)

	SetBitU(rtcm.Buff[:], i, 12, 1005)
	i += 12 /* message no */
	SetBitU(rtcm.Buff[:], i, 12, uint32(rtcm.StaId))
	i += 12 /* ref station id */
	SetBitU(rtcm.Buff[:], i, 6, 0)
	i += 6 /* itrf realization year */
	SetBitU(rtcm.Buff[:], i, 1, 1)
	i += 1 /* gps indicator */
	SetBitU(rtcm.Buff[:], i, 1, 1)
	i += 1 /* glonass indicator */
	SetBitU(rtcm.Buff[:], i, 1, 0)
	i += 1 /* galileo indicator */
	SetBitU(rtcm.Buff[:], i, 1, 0)
	i += 1 /* ref station indicator */
	set38bits(rtcm.Buff[:], i, p[0]/0.0001)
	i += 38 /* antenna ref point ecef-x */
	SetBitU(rtcm.Buff[:], i, 1, 1)
	i += 1 /* oscillator indicator */
	SetBitU(rtcm.Buff[:], i, 1, 0)
	i += 1 /* reserved */
	set38bits(rtcm.Buff[:], i, p[1]/0.0001)
	i += 38 /* antenna ref point ecef-y */
	SetBitU(rtcm.Buff[:], i, 2, 0)
	i += 2 /* quarter cycle indicator */
	set38bits(rtcm.Buff[:], i, p[2]/0.0001)
	i += 38 /* antenna ref point ecef-z */
	rtcm.Nbit = i
	return 1
}

/* encode type 1006: stationary RTK reference station ARP with height --------*/
func (rtcm *Rtcm) encode_type1006(sync int) int {
	p := rtcm.StaPara.Pos[:]
	i := 24
	var hgt uint32 = 0

	Trace(3, "encode_type1006: sync=%d\n", sync)

	if 0.0 <= rtcm.StaPara.Hgt && rtcm.StaPara.Hgt <= 6.5535 {
		hgt = uint32(ROUND_I(rtcm.StaPara.Hgt / 0.0001))
	} else {
		Trace(2, "antenna height error: h=%.4f\n", rtcm.StaPara.Hgt)
	}
	SetBitU(rtcm.Buff[:], i, 12, 1006)
	i += 12 /* message no */
	SetBitU(rtcm.Buff[:], i, 12, uint32(rtcm.StaId))
	i += 12 /* ref station id */
	SetBitU(rtcm.Buff[:], i, 6, 0)
	i += 6 /* itrf realization year */
	SetBitU(rtcm.Buff[:], i, 1, 1)
	i += 1 /* gps indicator */
	SetBitU(rtcm.Buff[:], i, 1, 1)
	i += 1 /* glonass indicator */
	SetBitU(rtcm.Buff[:], i, 1, 0)
	i += 1 /* galileo indicator */
	SetBitU(rtcm.Buff[:], i, 1, 0)
	i += 1 /* ref station indicator */
	set38bits(rtcm.Buff[:], i, p[0]/0.0001)
	i += 38 /* antenna ref point ecef-x */
	SetBitU(rtcm.Buff[:], i, 1, 1)
	i += 1 /* oscillator indicator */
	SetBitU(rtcm.Buff[:], i, 1, 0)
	i += 1 /* reserved */
	set38bits(rtcm.Buff[:], i, p[1]/0.0001)
	i += 38 /* antenna ref point ecef-y */
	SetBitU(rtcm.Buff[:], i, 2, 0)
	i += 2 /* quarter cycle indicator */
	set38bits(rtcm.Buff[:], i, p[2]/0.0001)
	i += 38 /* antenna ref point ecef-z */
	SetBitU(rtcm.Buff[:], i, 16, hgt)
	i += 16 /* antenna height */
	rtcm.Nbit = i
	return 1
}

/* encode type 1007: antenna descriptor --------------------------------------*/
func (rtcm *Rtcm) encode_type1007(sync int) int {
	var j int
	antsetup := rtcm.StaPara.AntSetup
	i := 24
	n := uint32(math.Min(float64(len(rtcm.StaPara.AntDes)), 31.0))

	Trace(3, "encode_type1007: sync=%d\n", sync)

	SetBitU(rtcm.Buff[:], i, 12, 1007)
	i += 12 /* message no */
	SetBitU(rtcm.Buff[:], i, 12, uint32(rtcm.StaId))
	i += 12 /* ref station id */

	/* antenna descriptor */
	SetBitU(rtcm.Buff[:], i, 8, n)
	i += 8
	des := rtcm.StaPara.AntDes[:]
	for j = 0; j < int(n); j++ {
		SetBitU(rtcm.Buff[:], i, 8, uint32(des[j]))
		i += 8
	}
	SetBitU(rtcm.Buff[:], i, 8, uint32(antsetup))
	i += 8 /* antetnna setup id */
	rtcm.Nbit = i
	return 1
}

/* encode type 1008: antenna descriptor & serial number ----------------------*/
func (rtcm *Rtcm) encode_type1008(sync int) int {
	i := 24
	j := 0
	antsetup := rtcm.StaPara.AntSetup
	n := uint32(math.Min(float64(len(rtcm.StaPara.AntDes)), 31.0))
	m := uint32(math.Min(float64(len(rtcm.StaPara.AntSno)), 31.0))

	Trace(3, "encode_type1008: sync=%d\n", sync)

	SetBitU(rtcm.Buff[:], i, 12, 1008)
	i += 12 /* message no */
	SetBitU(rtcm.Buff[:], i, 12, uint32(rtcm.StaId))
	i += 12 /* ref station id */

	/* antenna descriptor */
	SetBitU(rtcm.Buff[:], i, 8, n)
	i += 8
	des := rtcm.StaPara.AntDes[:]
	for j = 0; j < int(n); j++ {
		SetBitU(rtcm.Buff[:], i, 8, uint32(des[j]))
		i += 8
	}
	SetBitU(rtcm.Buff[:], i, 8, uint32(antsetup))
	i += 8 /* antenna setup id */

	/* antenna serial number */
	SetBitU(rtcm.Buff[:], i, 8, m)
	i += 8
	ant := rtcm.StaPara.AntSno[:]
	for j = 0; j < int(m); j++ {
		SetBitU(rtcm.Buff[:], i, 8, uint32(ant[j]))
		i += 8
	}
	rtcm.Nbit = i
	return 1
}

/* encode type 1009: basic L1-only GLONASS RTK observables -------------------*/
func (rtcm *Rtcm) encode_type1009(sync int) int {
	var i, j, nsat, sat, prn, fcn, code1, pr1, ppr1, lock1, amb int

	for j = 0; j < rtcm.ObsData.N() && nsat < MAXOBS; j++ {
		sat = rtcm.ObsData.Data[j].Sat
		if SatSys(sat, &prn) != SYS_GLO {
			continue
		}
		if fcn = rtcm.FcnGlo(sat); fcn < 0 {
			continue /* fcn+7 */
		}
		nsat++
	}
	/* encode header */
	i = rtcm.encode_head(1009, SYS_GLO, sync, nsat)

	for j = 0; j < rtcm.ObsData.N() && nsat < MAXOBS; j++ {
		sat = rtcm.ObsData.Data[j].Sat
		if SatSys(sat, &prn) != SYS_GLO {
			continue
		}
		if fcn = rtcm.FcnGlo(sat); fcn < 0 {
			continue /* fcn+7 */
		}

		/* generate obs field data glonass */
		rtcm.GenObsGlo(&rtcm.ObsData.Data[j], fcn, &code1, &pr1, &ppr1, &lock1, &amb,
			nil, nil, nil, nil, nil, nil)

		SetBitU(rtcm.Buff[:], i, 6, uint32(prn))
		i += 6
		SetBitU(rtcm.Buff[:], i, 1, uint32(code1))
		i += 1
		SetBitU(rtcm.Buff[:], i, 5, uint32(fcn))
		i += 5 /* fcn+7 */
		SetBitU(rtcm.Buff[:], i, 25, uint32(pr1))
		i += 25
		SetBits(rtcm.Buff[:], i, 20, int32(ppr1))
		i += 20
		SetBitU(rtcm.Buff[:], i, 7, uint32(lock1))
		i += 7
	}
	rtcm.Nbit = i
	return 1
}

/* encode type 1010: extended L1-only GLONASS RTK observables ----------------*/
func (rtcm *Rtcm) encode_type1010(sync int) int {
	var i, j, nsat, sat, prn, fcn, code1, pr1, ppr1, lock1, amb, cnr1 int

	Trace(3, "encode_type1010: sync=%d\n", sync)

	for j = 0; j < rtcm.ObsData.N() && nsat < MAXOBS; j++ {
		sat = rtcm.ObsData.Data[j].Sat
		if SatSys(sat, &prn) != SYS_GLO {
			continue
		}
		if fcn = rtcm.FcnGlo(sat); fcn < 0 {
			continue /* fcn+7 */
		}
		nsat++
	}
	/* encode header */
	i = rtcm.encode_head(1010, SYS_GLO, sync, nsat)

	for j = 0; j < rtcm.ObsData.N() && nsat < MAXOBS; j++ {
		sat = rtcm.ObsData.Data[j].Sat
		if SatSys(sat, &prn) != SYS_GLO {
			continue
		}
		if fcn = rtcm.FcnGlo(sat); fcn < 0 {
			continue /* fcn+7 */
		}
		/* generate obs field data glonass */
		rtcm.GenObsGlo(&rtcm.ObsData.Data[j], fcn, &code1, &pr1, &ppr1, &lock1, &amb,
			&cnr1, nil, nil, nil, nil, nil)

		SetBitU(rtcm.Buff[:], i, 6, uint32(prn))
		i += 6
		SetBitU(rtcm.Buff[:], i, 1, uint32(code1))
		i += 1
		SetBitU(rtcm.Buff[:], i, 5, uint32(fcn))
		i += 5 /* fcn+7 */
		SetBitU(rtcm.Buff[:], i, 25, uint32(pr1))
		i += 25
		SetBits(rtcm.Buff[:], i, 20, int32(ppr1))
		i += 20
		SetBitU(rtcm.Buff[:], i, 7, uint32(lock1))
		i += 7
		SetBitU(rtcm.Buff[:], i, 7, uint32(amb))
		i += 7
		SetBitU(rtcm.Buff[:], i, 8, uint32(cnr1))
		i += 8
	}
	rtcm.Nbit = i
	return 1
}

/* encode type 1011: basic  L1&L2 GLONASS RTK observables --------------------*/
func (rtcm *Rtcm) encode_type1011(sync int) int {
	var i, j, nsat, sat, prn, fcn, code1, pr1, ppr1, lock1, amb, code2, pr21, ppr2, lock2 int

	Trace(3, "encode_type1011: sync=%d\n", sync)

	for j = 0; j < rtcm.ObsData.N() && nsat < MAXOBS; j++ {
		sat = rtcm.ObsData.Data[j].Sat
		if SatSys(sat, &prn) != SYS_GLO {
			continue
		}
		if fcn = rtcm.FcnGlo(sat); fcn < 0 {
			continue /* fcn+7 */
		}
		nsat++
	}
	/* encode header */
	i = rtcm.encode_head(1011, SYS_GLO, sync, nsat)

	for j = 0; j < rtcm.ObsData.N() && nsat < MAXOBS; j++ {
		sat = rtcm.ObsData.Data[j].Sat
		if SatSys(sat, &prn) != SYS_GLO {
			continue
		}
		if fcn = rtcm.FcnGlo(sat); fcn < 0 {
			continue /* fcn+7 */
		}

		/* generate obs field data glonass */
		rtcm.GenObsGlo(&rtcm.ObsData.Data[j], fcn, &code1, &pr1, &ppr1, &lock1, &amb,
			nil, &code2, &pr21, &ppr2, &lock2, nil)

		SetBitU(rtcm.Buff[:], i, 6, uint32(prn))
		i += 6
		SetBitU(rtcm.Buff[:], i, 1, uint32(code1))
		i += 1
		SetBitU(rtcm.Buff[:], i, 5, uint32(fcn))
		i += 5 /* fcn+7 */
		SetBitU(rtcm.Buff[:], i, 25, uint32(pr1))
		i += 25
		SetBits(rtcm.Buff[:], i, 20, int32(ppr1))
		i += 20
		SetBitU(rtcm.Buff[:], i, 7, uint32(lock1))
		i += 7
		SetBitU(rtcm.Buff[:], i, 2, uint32(code2))
		i += 2
		SetBits(rtcm.Buff[:], i, 14, int32(pr21))
		i += 14
		SetBits(rtcm.Buff[:], i, 20, int32(ppr2))
		i += 20
		SetBitU(rtcm.Buff[:], i, 7, uint32(lock2))
		i += 7
	}
	rtcm.Nbit = i
	return 1
}

/* encode type 1012: extended L1&L2 GLONASS RTK observables ------------------*/
func (rtcm *Rtcm) encode_type1012(sync int) int {
	var i, j, nsat, sat, prn, fcn, code1, pr1, ppr1, lock1, amb, cnr1, code2, pr21, ppr2, lock2, cnr2 int

	Trace(3, "encode_type1012: sync=%d\n", sync)

	for j = 0; j < rtcm.ObsData.N() && nsat < MAXOBS; j++ {
		sat = rtcm.ObsData.Data[j].Sat
		if SatSys(sat, &prn) != SYS_GLO {
			continue
		}
		if fcn = rtcm.FcnGlo(sat); fcn < 0 {
			continue /* fcn+7 */
		}
		nsat++
	}
	/* encode header */
	i = rtcm.encode_head(1012, SYS_GLO, sync, nsat)

	for j = 0; j < rtcm.ObsData.N() && nsat < MAXOBS; j++ {
		sat = rtcm.ObsData.Data[j].Sat
		if SatSys(sat, &prn) != SYS_GLO {
			continue
		}
		if fcn = rtcm.FcnGlo(sat); fcn < 0 {
			continue /* fcn+7 */
		}

		/* generate obs field data glonass */
		rtcm.GenObsGlo(&rtcm.ObsData.Data[j], fcn, &code1, &pr1, &ppr1, &lock1, &amb,
			&cnr1, &code2, &pr21, &ppr2, &lock2, &cnr2)

		SetBitU(rtcm.Buff[:], i, 6, uint32(prn))
		i += 6
		SetBitU(rtcm.Buff[:], i, 1, uint32(code1))
		i += 1
		SetBitU(rtcm.Buff[:], i, 5, uint32(fcn))
		i += 5 /* fcn+7 */
		SetBitU(rtcm.Buff[:], i, 25, uint32(pr1))
		i += 25
		SetBits(rtcm.Buff[:], i, 20, int32(ppr1))
		i += 20
		SetBitU(rtcm.Buff[:], i, 7, uint32(lock1))
		i += 7
		SetBitU(rtcm.Buff[:], i, 7, uint32(amb))
		i += 7
		SetBitU(rtcm.Buff[:], i, 8, uint32(cnr1))
		i += 8
		SetBitU(rtcm.Buff[:], i, 2, uint32(code2))
		i += 2
		SetBits(rtcm.Buff[:], i, 14, int32(pr21))
		i += 14
		SetBits(rtcm.Buff[:], i, 20, int32(ppr2))
		i += 20
		SetBitU(rtcm.Buff[:], i, 7, uint32(lock2))
		i += 7
		SetBitU(rtcm.Buff[:], i, 8, uint32(cnr2))
		i += 8
	}
	rtcm.Nbit = i
	return 1
}

/* encode type 1019: GPS ephemerides -----------------------------------------*/
func (rtcm *Rtcm) encode_type1019(sync int) int {
	var (
		eph      *Eph
		sqrtA, e uint32
	)
	var i, prn, week, toe, toc, i0, OMG0, omg, M0, deln, idot, OMGd, crs, crc, cus, cuc, cis, cic, af0, af1, af2, tgd int

	i = 24
	Trace(3, "encode_type1019: sync=%d\n", sync)

	if SatSys(rtcm.EphSat, &prn) != SYS_GPS {
		return 0
	}
	eph = &rtcm.NavData.Ephs[rtcm.EphSat-1]
	if eph.Sat != rtcm.EphSat {
		return 0
	}
	week = eph.Week % 1024
	toe = ROUND_I(eph.Toes / 16.0)
	toc = ROUND_I(Time2GpsT(eph.Toc, nil) / 16.0)
	sqrtA = ROUND_U(math.Sqrt(eph.A) / P2_19)
	e = ROUND_U(eph.E / P2_33)
	i0 = ROUND_I(eph.I0 / P2_31 / SC2RAD)
	OMG0 = ROUND_I(eph.OMG0 / P2_31 / SC2RAD)
	omg = ROUND_I(eph.Omg / P2_31 / SC2RAD)
	M0 = ROUND_I(eph.M0 / P2_31 / SC2RAD)
	deln = ROUND_I(eph.Deln / P2_43 / SC2RAD)
	idot = ROUND_I(eph.Idot / P2_43 / SC2RAD)
	OMGd = ROUND_I(eph.OMGd / P2_43 / SC2RAD)
	crs = ROUND_I(eph.Crs / P2_5)
	crc = ROUND_I(eph.Crc / P2_5)
	cus = ROUND_I(eph.Cus / P2_29)
	cuc = ROUND_I(eph.Cuc / P2_29)
	cis = ROUND_I(eph.Cis / P2_29)
	cic = ROUND_I(eph.Cic / P2_29)
	af0 = ROUND_I(eph.F0 / P2_31)
	af1 = ROUND_I(eph.F1 / P2_43)
	af2 = ROUND_I(eph.F2 / P2_55)
	tgd = ROUND_I(eph.Tgd[0] / P2_31)

	SetBitU(rtcm.Buff[:], i, 12, 1019)
	i += 12
	SetBitU(rtcm.Buff[:], i, 6, uint32(prn))
	i += 6
	SetBitU(rtcm.Buff[:], i, 10, uint32(week))
	i += 10
	SetBitU(rtcm.Buff[:], i, 4, uint32(eph.Sva))
	i += 4
	SetBitU(rtcm.Buff[:], i, 2, uint32(eph.Code))
	i += 2
	SetBits(rtcm.Buff[:], i, 14, int32(idot))
	i += 14
	SetBitU(rtcm.Buff[:], i, 8, uint32(eph.Iode))
	i += 8
	SetBitU(rtcm.Buff[:], i, 16, uint32(toc))
	i += 16
	SetBits(rtcm.Buff[:], i, 8, int32(af2))
	i += 8
	SetBits(rtcm.Buff[:], i, 16, int32(af1))
	i += 16
	SetBits(rtcm.Buff[:], i, 22, int32(af0))
	i += 22
	SetBitU(rtcm.Buff[:], i, 10, uint32(eph.Iodc))
	i += 10
	SetBits(rtcm.Buff[:], i, 16, int32(crs))
	i += 16
	SetBits(rtcm.Buff[:], i, 16, int32(deln))
	i += 16
	SetBits(rtcm.Buff[:], i, 32, int32(M0))
	i += 32
	SetBits(rtcm.Buff[:], i, 16, int32(cuc))
	i += 16
	SetBitU(rtcm.Buff[:], i, 32, uint32(e))
	i += 32
	SetBits(rtcm.Buff[:], i, 16, int32(cus))
	i += 16
	SetBitU(rtcm.Buff[:], i, 32, uint32(sqrtA))
	i += 32
	SetBitU(rtcm.Buff[:], i, 16, uint32(toe))
	i += 16
	SetBits(rtcm.Buff[:], i, 16, int32(cic))
	i += 16
	SetBits(rtcm.Buff[:], i, 32, int32(OMG0))
	i += 32
	SetBits(rtcm.Buff[:], i, 16, int32(cis))
	i += 16
	SetBits(rtcm.Buff[:], i, 32, int32(i0))
	i += 32
	SetBits(rtcm.Buff[:], i, 16, int32(crc))
	i += 16
	SetBits(rtcm.Buff[:], i, 32, int32(omg))
	i += 32
	SetBits(rtcm.Buff[:], i, 24, int32(OMGd))
	i += 24
	SetBits(rtcm.Buff[:], i, 8, int32(tgd))
	i += 8
	SetBitU(rtcm.Buff[:], i, 6, uint32(eph.Svh))
	i += 6
	SetBitU(rtcm.Buff[:], i, 1, uint32(eph.Flag))
	i += 1
	if eph.Fit > 0.0 {
		SetBitU(rtcm.Buff[:], i, 1, 0)
	} else {
		SetBitU(rtcm.Buff[:], i, 1, 1)
	}

	i += 1
	rtcm.Nbit = i
	return 1
}

/* encode type 1020: GLONASS ephemerides -------------------------------------*/
func (rtcm *Rtcm) encode_type1020(sync int) int {
	var (
		geph                         *GEph
		time                         Gtime
		ep                           [6]float64
		j, prn, tk_h, tk_m, tk_s, tb int
		fcn, NT, gamn, taun, dtaun   int
		pos, vel, acc                [3]int
	)
	i := 24

	Trace(3, "encode_type1020: sync=%d\n", sync)

	if SatSys(rtcm.EphSat, &prn) != SYS_GLO {
		return 0
	}
	geph = &rtcm.NavData.Geph[prn-1]
	if geph.Sat != rtcm.EphSat {
		return 0
	}
	fcn = geph.Frq + 7

	/* time of frame within day (utc(su) + 3 hr) */
	time = TimeAdd(GpsT2Utc(geph.Tof), 10800.0)
	Time2Epoch(time, ep[:])
	tk_h = int(ep[3])
	tk_m = int(ep[4])
	tk_s = ROUND_I(ep[5] / 30.0)

	/* # of days since jan 1 in leap year */
	ep[0] = math.Floor(ep[0]/4.0) * 4.0
	ep[1], ep[2] = 1.0, 1.0
	ep[3], ep[4], ep[5] = 0.0, 0.0, 0.0
	NT = int(math.Floor(TimeDiff(time, Epoch2Time(ep[:]))/86400. + 1.0))

	/* index of time interval within day (utc(su) + 3 hr) */
	time = TimeAdd(GpsT2Utc(geph.Toe), 10800.0)
	Time2Epoch(time, ep[:])
	tb = ROUND_I((ep[3]*3600.0 + ep[4]*60.0 + ep[5]) / 900.0)

	for j = 0; j < 3; j++ {
		pos[j] = ROUND_I(geph.Pos[j] / P2_11 / 1e3)
		vel[j] = ROUND_I(geph.Vel[j] / P2_20 / 1e3)
		acc[j] = ROUND_I(geph.Acc[j] / P2_30 / 1e3)
	}
	gamn = ROUND_I(geph.Gamn / P2_40)
	taun = ROUND_I(geph.Taun / P2_30)
	dtaun = ROUND_I(geph.DTaun / P2_30)

	SetBitU(rtcm.Buff[:], i, 12, 1020)
	i += 12
	SetBitU(rtcm.Buff[:], i, 6, uint32(prn))
	i += 6
	SetBitU(rtcm.Buff[:], i, 5, uint32(fcn))
	i += 5
	SetBitU(rtcm.Buff[:], i, 4, 0)
	i += 4 /* almanac health,P1 */
	SetBitU(rtcm.Buff[:], i, 5, uint32(tk_h))
	i += 5
	SetBitU(rtcm.Buff[:], i, 6, uint32(tk_m))
	i += 6
	SetBitU(rtcm.Buff[:], i, 1, uint32(tk_s))
	i += 1
	SetBitU(rtcm.Buff[:], i, 1, uint32(geph.Svh))
	i += 1 /* Bn */
	SetBitU(rtcm.Buff[:], i, 1, 0)
	i += 1 /* P2 */
	SetBitU(rtcm.Buff[:], i, 7, uint32(tb))
	i += 7
	setbitg(rtcm.Buff[:], i, 24, int32(vel[0]))
	i += 24
	setbitg(rtcm.Buff[:], i, 27, int32(pos[0]))
	i += 27
	setbitg(rtcm.Buff[:], i, 5, int32(acc[0]))
	i += 5
	setbitg(rtcm.Buff[:], i, 24, int32(vel[1]))
	i += 24
	setbitg(rtcm.Buff[:], i, 27, int32(pos[1]))
	i += 27
	setbitg(rtcm.Buff[:], i, 5, int32(acc[1]))
	i += 5
	setbitg(rtcm.Buff[:], i, 24, int32(vel[2]))
	i += 24
	setbitg(rtcm.Buff[:], i, 27, int32(pos[2]))
	i += 27
	setbitg(rtcm.Buff[:], i, 5, int32(acc[2]))
	i += 5
	SetBitU(rtcm.Buff[:], i, 1, 0)
	i += 1 /* P3 */
	setbitg(rtcm.Buff[:], i, 11, int32(gamn))
	i += 11
	SetBitU(rtcm.Buff[:], i, 3, 0)
	i += 3 /* P,ln */
	setbitg(rtcm.Buff[:], i, 22, int32(taun))
	i += 22
	setbitg(rtcm.Buff[:], i, 5, int32(dtaun))
	i += 5
	SetBitU(rtcm.Buff[:], i, 5, uint32(geph.Age))
	i += 5 /* En */
	SetBitU(rtcm.Buff[:], i, 1, 0)
	i += 1 /* P4 */
	SetBitU(rtcm.Buff[:], i, 4, 0)
	i += 4 /* FT */
	SetBitU(rtcm.Buff[:], i, 11, uint32(NT))
	i += 11
	SetBitU(rtcm.Buff[:], i, 2, 0)
	i += 2 /* M */
	SetBitU(rtcm.Buff[:], i, 1, 0)
	i += 1 /* flag for additional data */
	SetBitU(rtcm.Buff[:], i, 11, 0)
	i += 11 /* NA */
	SetBitU(rtcm.Buff[:], i, 32, 0)
	i += 32 /* tauc */
	SetBitU(rtcm.Buff[:], i, 5, 0)
	i += 5 /* N4 */
	SetBitU(rtcm.Buff[:], i, 22, 0)
	i += 22 /* taugps */
	SetBitU(rtcm.Buff[:], i, 1, 0)
	i += 1 /* ln */
	SetBitU(rtcm.Buff[:], i, 7, 0)
	i += 7
	rtcm.Nbit = i
	return 1
}

/* encode type 1033: receiver and antenna descriptor -------------------------*/
func (rtcm *Rtcm) encode_type1033(sync int) int {
	var i, j, antsetup int = 24, 0, rtcm.StaPara.AntSetup
	n := uint32(math.Min(float64(len(rtcm.StaPara.AntDes)), 31.0))
	m := uint32(math.Min(float64(len(rtcm.StaPara.AntSno)), 31.0))
	I := uint32(math.Min(float64(len(rtcm.StaPara.Type)), 31.0))
	J := uint32(math.Min(float64(len(rtcm.StaPara.RecVer)), 31.0))
	K := uint32(math.Min(float64(len(rtcm.StaPara.RecSN)), 31.0))

	Trace(3, "encode_type1033: sync=%d\n", sync)

	SetBitU(rtcm.Buff[:], i, 12, 1033)
	i += 12
	SetBitU(rtcm.Buff[:], i, 12, uint32(rtcm.StaId))
	i += 12

	SetBitU(rtcm.Buff[:], i, 8, n)
	i += 8
	des := rtcm.StaPara.AntDes[:]
	for j = 0; j < int(n); j++ {
		SetBitU(rtcm.Buff[:], i, 8, uint32(des[j]))
		i += 8
	}
	SetBitU(rtcm.Buff[:], i, 8, uint32(antsetup))
	i += 8

	SetBitU(rtcm.Buff[:], i, 8, m)
	i += 8
	ant := rtcm.StaPara.AntSno[:]
	for j = 0; j < int(m); j++ {
		SetBitU(rtcm.Buff[:], i, 8, uint32(ant[j]))
		i += 8
	}
	SetBitU(rtcm.Buff[:], i, 8, I)
	i += 8
	rec := []rune(rtcm.StaPara.Type)
	for j = 0; j < int(I); j++ {
		SetBitU(rtcm.Buff[:], i, 8, uint32(rec[j]))
		i += 8
	}
	SetBitU(rtcm.Buff[:], i, 8, J)
	i += 8
	rec = []rune(rtcm.StaPara.RecVer)
	for j = 0; j < int(J); j++ {
		SetBitU(rtcm.Buff[:], i, 8, uint32(rec[j]))
		i += 8
	}
	SetBitU(rtcm.Buff[:], i, 8, K)
	i += 8
	rec = []rune(rtcm.StaPara.RecSN)
	for j = 0; j < int(K); j++ {
		SetBitU(rtcm.Buff[:], i, 8, uint32(rec[j]))
		i += 8
	}
	rtcm.Nbit = i
	return 1
}

/* encode type 1041: NavIC/IRNSS ephemerides ---------------------------------*/
func (rtcm *Rtcm) encode_type1041(sync int) int {
	var (
		eph                                       *Eph
		sqrtA, e                                  uint32
		prn, week, toe, toc, i0, OMG0, omg, M0    int
		deln, idot, OMGd, crs, crc, cus, cuc, cis int
		cic, af0, af1, af2, tgd                   int
	)
	i := 24

	Trace(3, "encode_type1041: sync=%d\n", sync)

	if SatSys(rtcm.EphSat, &prn) != SYS_IRN {
		return 0
	}
	eph = &rtcm.NavData.Ephs[rtcm.EphSat-1]
	if eph.Sat != rtcm.EphSat {
		return 0
	}
	week = eph.Week % 1024
	toe = ROUND_I(eph.Toes / 16.0)
	toc = ROUND_I(Time2GpsT(eph.Toc, nil) / 16.0)
	sqrtA = ROUND_U(math.Sqrt(eph.A) / P2_19)
	e = ROUND_U(eph.E / P2_33)
	i0 = ROUND_I(eph.I0 / P2_31 / SC2RAD)
	OMG0 = ROUND_I(eph.OMG0 / P2_31 / SC2RAD)
	omg = ROUND_I(eph.Omg / P2_31 / SC2RAD)
	M0 = ROUND_I(eph.M0 / P2_31 / SC2RAD)
	deln = ROUND_I(eph.Deln / P2_41 / SC2RAD)
	idot = ROUND_I(eph.Idot / P2_43 / SC2RAD)
	OMGd = ROUND_I(eph.OMGd / P2_41 / SC2RAD)
	crs = ROUND_I(eph.Crs / 0.0625)
	crc = ROUND_I(eph.Crc / 0.0625)
	cus = ROUND_I(eph.Cus / P2_28)
	cuc = ROUND_I(eph.Cuc / P2_28)
	cis = ROUND_I(eph.Cis / P2_28)
	cic = ROUND_I(eph.Cic / P2_28)
	af0 = ROUND_I(eph.F0 / P2_31)
	af1 = ROUND_I(eph.F1 / P2_43)
	af2 = ROUND_I(eph.F2 / P2_55)
	tgd = ROUND_I(eph.Tgd[0] / P2_31)

	SetBitU(rtcm.Buff[:], i, 12, 1041)
	i += 12
	SetBitU(rtcm.Buff[:], i, 6, uint32(prn))
	i += 6
	SetBitU(rtcm.Buff[:], i, 10, uint32(week))
	i += 10
	SetBits(rtcm.Buff[:], i, 22, int32(af0))
	i += 22
	SetBits(rtcm.Buff[:], i, 16, int32(af1))
	i += 16
	SetBits(rtcm.Buff[:], i, 8, int32(af2))
	i += 8
	SetBitU(rtcm.Buff[:], i, 4, uint32(eph.Sva))
	i += 4
	SetBitU(rtcm.Buff[:], i, 16, uint32(toc))
	i += 16
	SetBits(rtcm.Buff[:], i, 8, int32(tgd))
	i += 8
	SetBits(rtcm.Buff[:], i, 22, int32(deln))
	i += 22
	SetBitU(rtcm.Buff[:], i, 8, uint32(eph.Iode))
	i += 8 + 10 /* IODEC */
	SetBitU(rtcm.Buff[:], i, 2, uint32(eph.Svh))
	i += 2 /* L5+Sflag */
	SetBits(rtcm.Buff[:], i, 15, int32(cuc))
	i += 15
	SetBits(rtcm.Buff[:], i, 15, int32(cus))
	i += 15
	SetBits(rtcm.Buff[:], i, 15, int32(cic))
	i += 15
	SetBits(rtcm.Buff[:], i, 15, int32(cis))
	i += 15
	SetBits(rtcm.Buff[:], i, 15, int32(crc))
	i += 15
	SetBits(rtcm.Buff[:], i, 15, int32(crs))
	i += 15
	SetBits(rtcm.Buff[:], i, 14, int32(idot))
	i += 14
	SetBits(rtcm.Buff[:], i, 32, int32(M0))
	i += 32
	SetBitU(rtcm.Buff[:], i, 16, uint32(toe))
	i += 16
	SetBitU(rtcm.Buff[:], i, 32, e)
	i += 32
	SetBitU(rtcm.Buff[:], i, 32, sqrtA)
	i += 32
	SetBits(rtcm.Buff[:], i, 32, int32(OMG0))
	i += 32
	SetBits(rtcm.Buff[:], i, 32, int32(omg))
	i += 32
	SetBits(rtcm.Buff[:], i, 22, int32(OMGd))
	i += 22
	SetBits(rtcm.Buff[:], i, 32, int32(i0))
	i += 32 + 4
	rtcm.Nbit = i
	return 1
}

/* encode type 1044: QZSS ephemerides ----------------------------------------*/
func (rtcm *Rtcm) encode_type1044(sync int) int {
	var (
		eph                                      *Eph
		sqrtA, e                                 uint32
		prn, week, toe, toc, i0, OMG0, omg       int
		M0, deln, idot, OMGd, crs, crc, cus, cuc int
		cis, cic, af0, af1, af2, tgd             int
	)
	i := 24

	Trace(3, "encode_type1044: sync=%d\n", sync)

	if SatSys(rtcm.EphSat, &prn) != SYS_QZS {
		return 0
	}
	eph = &rtcm.NavData.Ephs[rtcm.EphSat-1]
	if eph.Sat != rtcm.EphSat {
		return 0
	}
	week = eph.Week % 1024
	toe = ROUND_I(eph.Toes / 16.0)
	toc = ROUND_I(Time2GpsT(eph.Toc, nil) / 16.0)
	sqrtA = ROUND_U(math.Sqrt(eph.A) / P2_19)
	e = ROUND_U(eph.E / P2_33)
	i0 = ROUND_I(eph.I0 / P2_31 / SC2RAD)
	OMG0 = ROUND_I(eph.OMG0 / P2_31 / SC2RAD)
	omg = ROUND_I(eph.Omg / P2_31 / SC2RAD)
	M0 = ROUND_I(eph.M0 / P2_31 / SC2RAD)
	deln = ROUND_I(eph.Deln / P2_43 / SC2RAD)
	idot = ROUND_I(eph.Idot / P2_43 / SC2RAD)
	OMGd = ROUND_I(eph.OMGd / P2_43 / SC2RAD)
	crs = ROUND_I(eph.Crs / P2_5)
	crc = ROUND_I(eph.Crc / P2_5)
	cus = ROUND_I(eph.Cus / P2_29)
	cuc = ROUND_I(eph.Cuc / P2_29)
	cis = ROUND_I(eph.Cis / P2_29)
	cic = ROUND_I(eph.Cic / P2_29)
	af0 = ROUND_I(eph.F0 / P2_31)
	af1 = ROUND_I(eph.F1 / P2_43)
	af2 = ROUND_I(eph.F2 / P2_55)
	tgd = ROUND_I(eph.Tgd[0] / P2_31)

	SetBitU(rtcm.Buff[:], i, 12, 1044)
	i += 12
	SetBitU(rtcm.Buff[:], i, 4, uint32(prn-192))
	i += 4
	SetBitU(rtcm.Buff[:], i, 16, uint32(toc))
	i += 16
	SetBits(rtcm.Buff[:], i, 8, int32(af2))
	i += 8
	SetBits(rtcm.Buff[:], i, 16, int32(af1))
	i += 16
	SetBits(rtcm.Buff[:], i, 22, int32(af0))
	i += 22
	SetBitU(rtcm.Buff[:], i, 8, uint32(eph.Iode))
	i += 8
	SetBits(rtcm.Buff[:], i, 16, int32(crs))
	i += 16
	SetBits(rtcm.Buff[:], i, 16, int32(deln))
	i += 16
	SetBits(rtcm.Buff[:], i, 32, int32(M0))
	i += 32
	SetBits(rtcm.Buff[:], i, 16, int32(cuc))
	i += 16
	SetBitU(rtcm.Buff[:], i, 32, uint32(e))
	i += 32
	SetBits(rtcm.Buff[:], i, 16, int32(cus))
	i += 16
	SetBitU(rtcm.Buff[:], i, 32, uint32(sqrtA))
	i += 32
	SetBitU(rtcm.Buff[:], i, 16, uint32(toe))
	i += 16
	SetBits(rtcm.Buff[:], i, 16, int32(cic))
	i += 16
	SetBits(rtcm.Buff[:], i, 32, int32(OMG0))
	i += 32
	SetBits(rtcm.Buff[:], i, 16, int32(cis))
	i += 16
	SetBits(rtcm.Buff[:], i, 32, int32(i0))
	i += 32
	SetBits(rtcm.Buff[:], i, 16, int32(crc))
	i += 16
	SetBits(rtcm.Buff[:], i, 32, int32(omg))
	i += 32
	SetBits(rtcm.Buff[:], i, 24, int32(OMGd))
	i += 24
	SetBits(rtcm.Buff[:], i, 14, int32(idot))
	i += 14
	SetBitU(rtcm.Buff[:], i, 2, uint32(eph.Code))
	i += 2
	SetBitU(rtcm.Buff[:], i, 10, uint32(week))
	i += 10
	SetBitU(rtcm.Buff[:], i, 4, uint32(eph.Sva))
	i += 4
	SetBitU(rtcm.Buff[:], i, 6, uint32(eph.Svh))
	i += 6
	SetBits(rtcm.Buff[:], i, 8, int32(tgd))
	i += 8
	SetBitU(rtcm.Buff[:], i, 10, uint32(eph.Iodc))
	i += 10
	if eph.Fit == 2.0 {
		SetBitU(rtcm.Buff[:], i, 1, 0)
	} else {
		SetBitU(rtcm.Buff[:], i, 1, 1)
	}
	i += 1
	rtcm.Nbit = i
	return 1
}

/* encode type 1045: Galileo F/NAV satellite ephemerides ---------------------*/
func (rtcm *Rtcm) encode_type1045(sync int) int {
	var (
		eph                                                  *Eph
		sqrtA, e                                             uint32
		prn, week, toe, toc, i0, OMG0                        int
		omg, M0, deln, idot, OMGd, crs, crc                  int
		cus, cuc, cis, cic, af0, af1, af2, bgd1, oshs, osdvs int
	)
	i := 24

	Trace(3, "encode_type1045: sync=%d\n", sync)

	if SatSys(rtcm.EphSat, &prn) != SYS_GAL {
		return 0
	}
	eph = &rtcm.NavData.Ephs[rtcm.EphSat-1+MAXSAT] /* F/NAV */
	if eph.Sat != rtcm.EphSat {
		return 0
	}
	week = (eph.Week - 1024) % 4096 /* gst-week = gal-week - 1024 */
	toe = ROUND_I(eph.Toes / 60.0)
	toc = ROUND_I(Time2GpsT(eph.Toc, nil) / 60.0)
	sqrtA = ROUND_U(math.Sqrt(eph.A) / P2_19)
	e = ROUND_U(eph.E / P2_33)
	i0 = ROUND_I(eph.I0 / P2_31 / SC2RAD)
	OMG0 = ROUND_I(eph.OMG0 / P2_31 / SC2RAD)
	omg = ROUND_I(eph.Omg / P2_31 / SC2RAD)
	M0 = ROUND_I(eph.M0 / P2_31 / SC2RAD)
	deln = ROUND_I(eph.Deln / P2_43 / SC2RAD)
	idot = ROUND_I(eph.Idot / P2_43 / SC2RAD)
	OMGd = ROUND_I(eph.OMGd / P2_43 / SC2RAD)
	crs = ROUND_I(eph.Crs / P2_5)
	crc = ROUND_I(eph.Crc / P2_5)
	cus = ROUND_I(eph.Cus / P2_29)
	cuc = ROUND_I(eph.Cuc / P2_29)
	cis = ROUND_I(eph.Cis / P2_29)
	cic = ROUND_I(eph.Cic / P2_29)
	af0 = ROUND_I(eph.F0 / P2_34)
	af1 = ROUND_I(eph.F1 / P2_46)
	af2 = ROUND_I(eph.F2 / P2_59)
	bgd1 = ROUND_I(eph.Tgd[0] / P2_32) /* E5a/E1 */
	//	bgd2 = ROUND(eph.tgd[1] / P2_32) /* E5b/E1 */
	oshs = (eph.Svh >> 4) & 3  /* E5a SVH */
	osdvs = (eph.Svh >> 3) & 1 /* E5a DVS */
	SetBitU(rtcm.Buff[:], i, 12, 1045)
	i += 12
	SetBitU(rtcm.Buff[:], i, 6, uint32(prn))
	i += 6
	SetBitU(rtcm.Buff[:], i, 12, uint32(week))
	i += 12
	SetBitU(rtcm.Buff[:], i, 10, uint32(eph.Iode))
	i += 10
	SetBitU(rtcm.Buff[:], i, 8, uint32(eph.Sva))
	i += 8
	SetBits(rtcm.Buff[:], i, 14, int32(idot))
	i += 14
	SetBitU(rtcm.Buff[:], i, 14, uint32(toc))
	i += 14
	SetBits(rtcm.Buff[:], i, 6, int32(af2))
	i += 6
	SetBits(rtcm.Buff[:], i, 21, int32(af1))
	i += 21
	SetBits(rtcm.Buff[:], i, 31, int32(af0))
	i += 31
	SetBits(rtcm.Buff[:], i, 16, int32(crs))
	i += 16
	SetBits(rtcm.Buff[:], i, 16, int32(deln))
	i += 16
	SetBits(rtcm.Buff[:], i, 32, int32(M0))
	i += 32
	SetBits(rtcm.Buff[:], i, 16, int32(cuc))
	i += 16
	SetBitU(rtcm.Buff[:], i, 32, e)
	i += 32
	SetBits(rtcm.Buff[:], i, 16, int32(cus))
	i += 16
	SetBitU(rtcm.Buff[:], i, 32, sqrtA)
	i += 32
	SetBitU(rtcm.Buff[:], i, 14, uint32(toe))
	i += 14
	SetBits(rtcm.Buff[:], i, 16, int32(cic))
	i += 16
	SetBits(rtcm.Buff[:], i, 32, int32(OMG0))
	i += 32
	SetBits(rtcm.Buff[:], i, 16, int32(cis))
	i += 16
	SetBits(rtcm.Buff[:], i, 32, int32(i0))
	i += 32
	SetBits(rtcm.Buff[:], i, 16, int32(crc))
	i += 16
	SetBits(rtcm.Buff[:], i, 32, int32(omg))
	i += 32
	SetBits(rtcm.Buff[:], i, 24, int32(OMGd))
	i += 24
	SetBits(rtcm.Buff[:], i, 10, int32(bgd1))
	i += 10
	SetBitU(rtcm.Buff[:], i, 2, uint32(oshs))
	i += 2 /* E5a SVH */
	SetBitU(rtcm.Buff[:], i, 1, uint32(osdvs))
	i += 1 /* E5a DVS */
	SetBitU(rtcm.Buff[:], i, 7, 0)
	i += 7 /* reserved */
	rtcm.Nbit = i
	return 1
}

/* encode type 1046: Galileo I/NAV satellite ephemerides ---------------------*/
func (rtcm *Rtcm) encode_type1046(sync int) int {
	var (
		eph                                        *Eph
		sqrtA, e                                   uint32
		prn, week, toe, toc, i0, OMG0, omg         int
		M0, deln, idot, OMGd, crs, crc, cus, cuc   int
		cis, cic, af0, af1, af2, bgd1, bgd2, oshs1 int
		osdvs1, oshs2, osdvs2                      int
	)
	i := 24

	Trace(3, "encode_type1046: sync=%d\n", sync)

	if SatSys(rtcm.EphSat, &prn) != SYS_GAL {
		return 0
	}
	eph = &rtcm.NavData.Ephs[rtcm.EphSat-1] /* I/NAV */
	if eph.Sat != rtcm.EphSat {
		return 0
	}
	week = (eph.Week - 1024) % 4096 /* gst-week = gal-week - 1024 */
	toe = ROUND_I(eph.Toes / 60.0)
	toc = ROUND_I(Time2GpsT(eph.Toc, nil) / 60.0)
	sqrtA = ROUND_U(math.Sqrt(eph.A) / P2_19)
	e = ROUND_U(eph.E / P2_33)
	i0 = ROUND_I(eph.I0 / P2_31 / SC2RAD)
	OMG0 = ROUND_I(eph.OMG0 / P2_31 / SC2RAD)
	omg = ROUND_I(eph.Omg / P2_31 / SC2RAD)
	M0 = ROUND_I(eph.M0 / P2_31 / SC2RAD)
	deln = ROUND_I(eph.Deln / P2_43 / SC2RAD)
	idot = ROUND_I(eph.Idot / P2_43 / SC2RAD)
	OMGd = ROUND_I(eph.OMGd / P2_43 / SC2RAD)
	crs = ROUND_I(eph.Crs / P2_5)
	crc = ROUND_I(eph.Crc / P2_5)
	cus = ROUND_I(eph.Cus / P2_29)
	cuc = ROUND_I(eph.Cuc / P2_29)
	cis = ROUND_I(eph.Cis / P2_29)
	cic = ROUND_I(eph.Cic / P2_29)
	af0 = ROUND_I(eph.F0 / P2_34)
	af1 = ROUND_I(eph.F1 / P2_46)
	af2 = ROUND_I(eph.F2 / P2_59)
	bgd1 = ROUND_I(eph.Tgd[0] / P2_32) /* E5a/E1 */
	bgd2 = ROUND_I(eph.Tgd[1] / P2_32) /* E5b/E1 */
	oshs1 = (eph.Svh >> 7) & 3         /* E5b SVH */
	osdvs1 = (eph.Svh >> 6) & 1        /* E5b DVS */
	oshs2 = (eph.Svh >> 1) & 3         /* E1 SVH */
	osdvs2 = (eph.Svh >> 0) & 1        /* E1 DVS */
	SetBitU(rtcm.Buff[:], i, 12, 1046)
	i += 12
	SetBitU(rtcm.Buff[:], i, 6, uint32(prn))
	i += 6
	SetBitU(rtcm.Buff[:], i, 12, uint32(week))
	i += 12
	SetBitU(rtcm.Buff[:], i, 10, uint32(eph.Iode))
	i += 10
	SetBitU(rtcm.Buff[:], i, 8, uint32(eph.Sva))
	i += 8
	SetBits(rtcm.Buff[:], i, 14, int32(idot))
	i += 14
	SetBitU(rtcm.Buff[:], i, 14, uint32(toc))
	i += 14
	SetBits(rtcm.Buff[:], i, 6, int32(af2))
	i += 6
	SetBits(rtcm.Buff[:], i, 21, int32(af1))
	i += 21
	SetBits(rtcm.Buff[:], i, 31, int32(af0))
	i += 31
	SetBits(rtcm.Buff[:], i, 16, int32(crs))
	i += 16
	SetBits(rtcm.Buff[:], i, 16, int32(deln))
	i += 16
	SetBits(rtcm.Buff[:], i, 32, int32(M0))
	i += 32
	SetBits(rtcm.Buff[:], i, 16, int32(cuc))
	i += 16
	SetBitU(rtcm.Buff[:], i, 32, e)
	i += 32
	SetBits(rtcm.Buff[:], i, 16, int32(cus))
	i += 16
	SetBitU(rtcm.Buff[:], i, 32, sqrtA)
	i += 32
	SetBitU(rtcm.Buff[:], i, 14, uint32(toe))
	i += 14
	SetBits(rtcm.Buff[:], i, 16, int32(cic))
	i += 16
	SetBits(rtcm.Buff[:], i, 32, int32(OMG0))
	i += 32
	SetBits(rtcm.Buff[:], i, 16, int32(cis))
	i += 16
	SetBits(rtcm.Buff[:], i, 32, int32(i0))
	i += 32
	SetBits(rtcm.Buff[:], i, 16, int32(crc))
	i += 16
	SetBits(rtcm.Buff[:], i, 32, int32(omg))
	i += 32
	SetBits(rtcm.Buff[:], i, 24, int32(OMGd))
	i += 24
	SetBits(rtcm.Buff[:], i, 10, int32(bgd1))
	i += 10
	SetBits(rtcm.Buff[:], i, 10, int32(bgd2))
	i += 10
	SetBitU(rtcm.Buff[:], i, 2, uint32(oshs1))
	i += 2 /* E5b SVH */
	SetBitU(rtcm.Buff[:], i, 1, uint32(osdvs1))
	i += 1 /* E5b DVS */
	SetBitU(rtcm.Buff[:], i, 2, uint32(oshs2))
	i += 2 /* E1 SVH */
	SetBitU(rtcm.Buff[:], i, 1, uint32(osdvs2))
	i += 1 /* E1 DVS */
	rtcm.Nbit = i
	return 1
}

/* encode type 1042: Beidou ephemerides --------------------------------------*/
func (rtcm *Rtcm) encode_type1042(sync int) int {
	var (
		eph                                                                 *Eph
		sqrtA, e                                                            uint32
		prn, week, toe, toc, i0, OMG0, omg, M0, deln                        int
		idot, OMGd, crs, crc, cus, cuc, cis, cic, af0, af1, af2, tgd1, tgd2 int
	)
	i := 24

	Trace(3, "encode_type1042: sync=%d\n", sync)

	if SatSys(rtcm.EphSat, &prn) != SYS_CMP {
		return 0
	}
	eph = &rtcm.NavData.Ephs[rtcm.EphSat-1]
	if eph.Sat != rtcm.EphSat {
		return 0
	}
	week = eph.Week % 8192
	toe = ROUND_I(eph.Toes / 8.0)
	toc = ROUND_I(Time2BDT(GpsT2BDT(eph.Toc), nil) / 8.0) /* gpst . bdt */
	sqrtA = ROUND_U(math.Sqrt(eph.A) / P2_19)
	e = ROUND_U(eph.E / P2_33)
	i0 = ROUND_I(eph.I0 / P2_31 / SC2RAD)
	OMG0 = ROUND_I(eph.OMG0 / P2_31 / SC2RAD)
	omg = ROUND_I(eph.Omg / P2_31 / SC2RAD)
	M0 = ROUND_I(eph.M0 / P2_31 / SC2RAD)
	deln = ROUND_I(eph.Deln / P2_43 / SC2RAD)
	idot = ROUND_I(eph.Idot / P2_43 / SC2RAD)
	OMGd = ROUND_I(eph.OMGd / P2_43 / SC2RAD)
	crs = ROUND_I(eph.Crs / P2_6)
	crc = ROUND_I(eph.Crc / P2_6)
	cus = ROUND_I(eph.Cus / P2_31)
	cuc = ROUND_I(eph.Cuc / P2_31)
	cis = ROUND_I(eph.Cis / P2_31)
	cic = ROUND_I(eph.Cic / P2_31)
	af0 = ROUND_I(eph.F0 / P2_33)
	af1 = ROUND_I(eph.F1 / P2_50)
	af2 = ROUND_I(eph.F2 / P2_66)
	tgd1 = ROUND_I(eph.Tgd[0] / 1e-10)
	tgd2 = ROUND_I(eph.Tgd[1] / 1e-10)

	SetBitU(rtcm.Buff[:], i, 12, 1042)
	i += 12
	SetBitU(rtcm.Buff[:], i, 6, uint32(prn))
	i += 6
	SetBitU(rtcm.Buff[:], i, 13, uint32(week))
	i += 13
	SetBitU(rtcm.Buff[:], i, 4, uint32(eph.Sva))
	i += 4
	SetBits(rtcm.Buff[:], i, 14, int32(idot))
	i += 14
	SetBitU(rtcm.Buff[:], i, 5, uint32(eph.Iode))
	i += 5
	SetBitU(rtcm.Buff[:], i, 17, uint32(toc))
	i += 17
	SetBits(rtcm.Buff[:], i, 11, int32(af2))
	i += 11
	SetBits(rtcm.Buff[:], i, 22, int32(af1))
	i += 22
	SetBits(rtcm.Buff[:], i, 24, int32(af0))
	i += 24
	SetBitU(rtcm.Buff[:], i, 5, uint32(eph.Iodc))
	i += 5
	SetBits(rtcm.Buff[:], i, 18, int32(crs))
	i += 18
	SetBits(rtcm.Buff[:], i, 16, int32(deln))
	i += 16
	SetBits(rtcm.Buff[:], i, 32, int32(M0))
	i += 32
	SetBits(rtcm.Buff[:], i, 18, int32(cuc))
	i += 18
	SetBitU(rtcm.Buff[:], i, 32, e)
	i += 32
	SetBits(rtcm.Buff[:], i, 18, int32(cus))
	i += 18
	SetBitU(rtcm.Buff[:], i, 32, sqrtA)
	i += 32
	SetBitU(rtcm.Buff[:], i, 17, uint32(toe))
	i += 17
	SetBits(rtcm.Buff[:], i, 18, int32(cic))
	i += 18
	SetBits(rtcm.Buff[:], i, 32, int32(OMG0))
	i += 32
	SetBits(rtcm.Buff[:], i, 18, int32(cis))
	i += 18
	SetBits(rtcm.Buff[:], i, 32, int32(i0))
	i += 32
	SetBits(rtcm.Buff[:], i, 18, int32(crc))
	i += 18
	SetBits(rtcm.Buff[:], i, 32, int32(omg))
	i += 32
	SetBits(rtcm.Buff[:], i, 24, int32(OMGd))
	i += 24
	SetBits(rtcm.Buff[:], i, 10, int32(tgd1))
	i += 10
	SetBits(rtcm.Buff[:], i, 10, int32(tgd2))
	i += 10
	SetBitU(rtcm.Buff[:], i, 1, uint32(eph.Svh))
	i += 1
	rtcm.Nbit = i
	return 1
}

/* encode type 63: Beidou ephemerides (RTCM draft) ---------------------------*/
func (rtcm *Rtcm) encode_type63(sync int) int {
	var (
		eph                                          *Eph
		sqrtA, e                                     uint32
		prn, week, toe, toc, i0, OMG0, omg, M0, deln int
		idot, OMGd, crs, crc, cus, cuc, cis          int
		cic, af0, af1, af2, tgd1, tgd2               int
	)
	i := 24

	Trace(3, "encode_type63: sync=%d\n", sync)

	if SatSys(rtcm.EphSat, &prn) != SYS_CMP {
		return 0
	}
	eph = &rtcm.NavData.Ephs[rtcm.EphSat-1]
	if eph.Sat != rtcm.EphSat {
		return 0
	}
	week = eph.Week % 8192
	toe = ROUND_I(eph.Toes / 8.0)
	toc = ROUND_I(Time2BDT(GpsT2BDT(eph.Toc), nil) / 8.0) /* gpst . bdt */
	sqrtA = ROUND_U(math.Sqrt(eph.A) / P2_19)
	e = ROUND_U(eph.E / P2_33)
	i0 = ROUND_I(eph.I0 / P2_31 / SC2RAD)
	OMG0 = ROUND_I(eph.OMG0 / P2_31 / SC2RAD)
	omg = ROUND_I(eph.Omg / P2_31 / SC2RAD)
	M0 = ROUND_I(eph.M0 / P2_31 / SC2RAD)
	deln = ROUND_I(eph.Deln / P2_43 / SC2RAD)
	idot = ROUND_I(eph.Idot / P2_43 / SC2RAD)
	OMGd = ROUND_I(eph.OMGd / P2_43 / SC2RAD)
	crs = ROUND_I(eph.Crs / P2_6)
	crc = ROUND_I(eph.Crc / P2_6)
	cus = ROUND_I(eph.Cus / P2_31)
	cuc = ROUND_I(eph.Cuc / P2_31)
	cis = ROUND_I(eph.Cis / P2_31)
	cic = ROUND_I(eph.Cic / P2_31)
	af0 = ROUND_I(eph.F0 / P2_33)
	af1 = ROUND_I(eph.F1 / P2_50)
	af2 = ROUND_I(eph.F2 / P2_66)
	tgd1 = ROUND_I(eph.Tgd[0] / 1e-10)
	tgd2 = ROUND_I(eph.Tgd[1] / 1e-10)

	SetBitU(rtcm.Buff[:], i, 12, 63)
	i += 12
	SetBitU(rtcm.Buff[:], i, 6, uint32(prn))
	i += 6
	SetBitU(rtcm.Buff[:], i, 13, uint32(week))
	i += 13
	SetBitU(rtcm.Buff[:], i, 4, uint32(eph.Sva))
	i += 4
	SetBits(rtcm.Buff[:], i, 14, int32(idot))
	i += 14
	SetBitU(rtcm.Buff[:], i, 5, uint32(eph.Iode))
	i += 5
	SetBitU(rtcm.Buff[:], i, 17, uint32(toc))
	i += 17
	SetBits(rtcm.Buff[:], i, 11, int32(af2))
	i += 11
	SetBits(rtcm.Buff[:], i, 22, int32(af1))
	i += 22
	SetBits(rtcm.Buff[:], i, 24, int32(af0))
	i += 24
	SetBitU(rtcm.Buff[:], i, 5, uint32(eph.Iodc))
	i += 5
	SetBits(rtcm.Buff[:], i, 18, int32(crs))
	i += 18
	SetBits(rtcm.Buff[:], i, 16, int32(deln))
	i += 16
	SetBits(rtcm.Buff[:], i, 32, int32(M0))
	i += 32
	SetBits(rtcm.Buff[:], i, 18, int32(cuc))
	i += 18
	SetBitU(rtcm.Buff[:], i, 32, e)
	i += 32
	SetBits(rtcm.Buff[:], i, 18, int32(cus))
	i += 18
	SetBitU(rtcm.Buff[:], i, 32, sqrtA)
	i += 32
	SetBitU(rtcm.Buff[:], i, 17, uint32(toe))
	i += 17
	SetBits(rtcm.Buff[:], i, 18, int32(cic))
	i += 18
	SetBits(rtcm.Buff[:], i, 32, int32(OMG0))
	i += 32
	SetBits(rtcm.Buff[:], i, 18, int32(cis))
	i += 18
	SetBits(rtcm.Buff[:], i, 32, int32(i0))
	i += 32
	SetBits(rtcm.Buff[:], i, 18, int32(crc))
	i += 18
	SetBits(rtcm.Buff[:], i, 32, int32(omg))
	i += 32
	SetBits(rtcm.Buff[:], i, 24, int32(OMGd))
	i += 24
	SetBits(rtcm.Buff[:], i, 10, int32(tgd1))
	i += 10
	SetBits(rtcm.Buff[:], i, 10, int32(tgd2))
	i += 10
	SetBitU(rtcm.Buff[:], i, 1, uint32(eph.Svh))
	i += 1
	rtcm.Nbit = i
	return 1
}

/* encode SSR header ---------------------------------------------------------*/
func (rtcm *Rtcm) encode_ssr_head(ctype, sys, subtype, nsat, sync, iod int, udint float64, refd, provid, solid int) int {
	var (
		tow                         float64
		msgno, epoch, week, udi, ns int
	)
	i := 24

	Trace(4, "encode_ssr_head: type=%d sys=%d subtype=%d nsat=%d sync=%d iod=%d udint=%.0f\n", ctype, sys, subtype, nsat, sync, iod, udint)

	if subtype == 0 { /* RTCM SSR */

		ns = 6
		if sys == SYS_QZS {
			ns = 4
		}
		if ctype == 7 {
			switch sys {
			case SYS_GPS:
				msgno = 11 + ctype
			case SYS_GLO:
				msgno = 0 + ctype
			case SYS_GAL:
				msgno = 12 + ctype
				/* draft */
			case SYS_QZS:
				msgno = 13 + ctype
				/* draft */
			case SYS_CMP:
				msgno = 14 + ctype
				/* draft */
			case SYS_SBS:
				msgno = 0 + ctype
				/* draft */
			default:
				return 0
			}

		} else {
			switch sys {
			case SYS_GPS:
				msgno = 1056 + ctype
			case SYS_GLO:
				msgno = 1062 + ctype
			case SYS_GAL:
				msgno = 1239 + ctype
				/* draft */
			case SYS_QZS:
				msgno = 1245 + ctype
				/* draft */
			case SYS_CMP:
				msgno = 1257 + ctype
				/* draft */
			case SYS_SBS:
				msgno = 1251 + ctype
				/* draft */
			default:
				return 0
			}
		}
		if msgno == 0 {
			return 0
		}
		SetBitU(rtcm.Buff[:], i, 12, uint32(msgno))
		i += 12 /* message type */

		if sys == SYS_GLO {
			tow = Time2GpsT(TimeAdd(GpsT2Utc(rtcm.Time), 10800.0), &week)
			epoch = ROUND_I(tow) % 86400
			SetBitU(rtcm.Buff[:], i, 17, uint32(epoch))
			i += 17 /* GLONASS epoch time */
		} else {
			tow = Time2GpsT(rtcm.Time, &week)
			epoch = ROUND_I(tow) % 604800
			SetBitU(rtcm.Buff[:], i, 20, uint32(epoch))
			i += 20 /* GPS epoch time */
		}
	} else { /* IGS SSR */
		ns = 6
		tow = Time2GpsT(rtcm.Time, &week)
		epoch = ROUND_I(tow) % 604800
		SetBitU(rtcm.Buff[:], i, 12, 4076)
		i += 12 /* message type */
		SetBitU(rtcm.Buff[:], i, 3, 1)
		i += 3 /* version */
		SetBitU(rtcm.Buff[:], i, 8, uint32(subtype))
		i += 8 /* subtype */
		SetBitU(rtcm.Buff[:], i, 20, uint32(epoch))
		i += 20 /* SSR epoch time */
	}
	for udi = 0; udi < 15; udi++ {
		if ssrudint[udi] >= udint {
			break
		}
	}
	SetBitU(rtcm.Buff[:], i, 4, uint32(udi))
	i += 4 /* update interval */
	SetBitU(rtcm.Buff[:], i, 1, uint32(sync))
	i += 1 /* multiple message indicator */
	if subtype == 0 && (ctype == 1 || ctype == 4) {
		SetBitU(rtcm.Buff[:], i, 1, uint32(refd))
		i += 1 /* satellite ref datum */
	}
	SetBitU(rtcm.Buff[:], i, 4, uint32(iod))
	i += 4 /* IOD SSR */
	SetBitU(rtcm.Buff[:], i, 16, uint32(provid))
	i += 16 /* provider ID */
	SetBitU(rtcm.Buff[:], i, 4, uint32(solid))
	i += 4 /* solution ID */
	if subtype > 0 && (ctype == 1 || ctype == 4) {
		SetBitU(rtcm.Buff[:], i, 1, uint32(refd))
		i += 1 /* global/regional CRS indicator */
	}
	if ctype == 7 {
		SetBitU(rtcm.Buff[:], i, 1, 0)
		i += 1 /* dispersive bias consistency ind */
		SetBitU(rtcm.Buff[:], i, 1, 0)
		i += 1 /* MW consistency indicator */
	}
	SetBitU(rtcm.Buff[:], i, ns, uint32(nsat))
	i += ns /* no of satellites */
	return i
}

/* SSR signal and tracking mode IDs ------------------------------------------*/
var codes_gps [32]int = [32]int{
	CODE_L1C,
	CODE_L1P,
	CODE_L1W,
	CODE_L1S,
	CODE_L1L,
	CODE_L2C,
	CODE_L2D,
	CODE_L2S,
	CODE_L2L,
	CODE_L2X,
	CODE_L2P,
	CODE_L2W, 0, 0,
	CODE_L5I,
	CODE_L5Q}
var codes_glo [32]int = [32]int{
	CODE_L1C,
	CODE_L1P,
	CODE_L2C,
	CODE_L2P,
	CODE_L4A,
	CODE_L4B,
	CODE_L6A,
	CODE_L6B,
	CODE_L3I,
	CODE_L3Q}
var codes_gal [32]int = [32]int{
	CODE_L1A,
	CODE_L1B,
	CODE_L1C, 0, 0,
	CODE_L5I,
	CODE_L5Q, 0,
	CODE_L7I,
	CODE_L7Q, 0,
	CODE_L8I,
	CODE_L8Q, 0,
	CODE_L6A,
	CODE_L6B,
	CODE_L6C}
var codes_qzs [32]int = [32]int{
	CODE_L1C,
	CODE_L1S,
	CODE_L1L,
	CODE_L2S,
	CODE_L2L, 0,
	CODE_L5I,
	CODE_L5Q, 0,
	CODE_L6S,
	CODE_L6L, 0, 0, 0, 0, 0, 0,
	CODE_L6E}
var codes_bds [32]int = [32]int{
	CODE_L2I,
	CODE_L2Q, 0,
	CODE_L6I,
	CODE_L6Q, 0,
	CODE_L7I,
	CODE_L7Q, 0,
	CODE_L1D,
	CODE_L1P, 0,
	CODE_L5D,
	CODE_L5P, 0,
	CODE_L1A, 0, 0,
	CODE_L6A}
var codes_sbs [32]int = [32]int{
	CODE_L1C,
	CODE_L5I,
	CODE_L5Q}

/* encode SSR 1: orbit corrections -------------------------------------------*/
func (rtcm *Rtcm) encode_ssr1(sys, subtype, sync int) int {
	var (
		udint                                    float64 = 0.0
		i, j, iod, nsat, prn, iode, iodcrc, refd int

		deph, ddeph [3]int
	)

	Trace(4, "encode_ssr1: sys=%d subtype=%d sync=%d\n", sys, subtype, sync)

	np, ni, nj, offp, _, _ := selectsys(sys)

	if subtype > 0 { /* IGS SSR */
		np = 6
		ni = 8
		nj = 0
		if sys == SYS_CMP {
			offp = 0
		} else if sys == SYS_SBS {
			offp = 119
		}
	}
	/* number of satellites */
	for j, nsat = 0, 0; j < MAXSAT; j++ {
		if SatSys(j+1, &prn) != sys || rtcm.Ssr[j].Update == 0 {
			continue
		}
		nsat++
		udint = rtcm.Ssr[j].Udi[0]
		iod = rtcm.Ssr[j].Iod[0]
		refd = rtcm.Ssr[j].Refd
	}
	/* encode SSR header */
	i = rtcm.encode_ssr_head(1, sys, subtype, nsat, sync, iod, udint, refd, 0, 0)

	for j = 0; j < MAXSAT; j++ {
		if SatSys(j+1, &prn) != sys || rtcm.Ssr[j].Update == 0 {
			continue
		}

		iode = rtcm.Ssr[j].Iode     /* SBAS/BDS: toe/t0 modulo */
		iodcrc = rtcm.Ssr[j].IodCrc /* SBAS/BDS: IOD CRC */

		if subtype > 0 { /* IGS SSR */
			iode &= 0xFF
		}
		deph[0] = ROUND_I(rtcm.Ssr[j].Deph[0] / 1e-4)
		deph[1] = ROUND_I(rtcm.Ssr[j].Deph[1] / 4e-4)
		deph[2] = ROUND_I(rtcm.Ssr[j].Deph[2] / 4e-4)
		ddeph[0] = ROUND_I(rtcm.Ssr[j].Ddeph[0] / 1e-6)
		ddeph[1] = ROUND_I(rtcm.Ssr[j].Ddeph[1] / 4e-6)
		ddeph[2] = ROUND_I(rtcm.Ssr[j].Ddeph[2] / 4e-6)

		SetBitU(rtcm.Buff[:], i, np, uint32(prn-offp))
		i += np /* satellite ID */
		SetBitU(rtcm.Buff[:], i, ni, uint32(iode))
		i += ni /* IODE */
		SetBitU(rtcm.Buff[:], i, nj, uint32(iodcrc))
		i += nj /* IODCRC */
		SetBits(rtcm.Buff[:], i, 22, int32(deph[0]))
		i += 22 /* delta radial */
		SetBits(rtcm.Buff[:], i, 20, int32(deph[1]))
		i += 20 /* delta along-track */
		SetBits(rtcm.Buff[:], i, 20, int32(deph[2]))
		i += 20 /* delta cross-track */
		SetBits(rtcm.Buff[:], i, 21, int32(ddeph[0]))
		i += 21 /* dot delta radial */
		SetBits(rtcm.Buff[:], i, 19, int32(ddeph[1]))
		i += 19 /* dot delta along-track */
		SetBits(rtcm.Buff[:], i, 19, int32(ddeph[2]))
		i += 19 /* dot delta cross-track */
	}
	rtcm.Nbit = i
	return 1
}

/* encode SSR 2: clock corrections -------------------------------------------*/
func (rtcm *Rtcm) encode_ssr2(sys, subtype, sync int) int {
	var (
		udint                          float64 = 0.0
		i, j, iod, nsat, prn, np, offp int
		dclk                           [3]int
	)

	Trace(4, "encode_ssr2: sys=%d subtype=%d sync=%d\n", sys, subtype, sync)

	np, _, _, offp, _, miss := selectsys(sys)
	if miss {
		return 0
	}
	if subtype > 0 { /* IGS SSR */
		np = 6
		if sys == SYS_CMP {
			offp = 0
		} else if sys == SYS_SBS {
			offp = 119
		}
	}
	/* number of satellites */
	for j, nsat = 0, 0; j < MAXSAT; j++ {
		if SatSys(j+1, &prn) != sys || rtcm.Ssr[j].Update == 0 {
			continue
		}
		nsat++
		udint = rtcm.Ssr[j].Udi[1]
		iod = rtcm.Ssr[j].Iod[1]
	}
	/* encode SSR header */
	i = rtcm.encode_ssr_head(2, sys, subtype, nsat, sync, iod, udint, 0, 0, 0)

	for j = 0; j < MAXSAT; j++ {
		if SatSys(j+1, &prn) != sys || rtcm.Ssr[j].Update == 0 {
			continue
		}

		dclk[0] = ROUND_I(rtcm.Ssr[j].Dclk[0] / 1e-4)
		dclk[1] = ROUND_I(rtcm.Ssr[j].Dclk[1] / 1e-6)
		dclk[2] = ROUND_I(rtcm.Ssr[j].Dclk[2] / 2e-8)

		SetBitU(rtcm.Buff[:], i, np, uint32(prn-offp))
		i += np /* satellite ID */
		SetBits(rtcm.Buff[:], i, 22, int32(dclk[0]))
		i += 22 /* delta clock C0 */
		SetBits(rtcm.Buff[:], i, 21, int32(dclk[1]))
		i += 21 /* delta clock C1 */
		SetBits(rtcm.Buff[:], i, 27, int32(dclk[2]))
		i += 27 /* delta clock C2 */
	}
	rtcm.Nbit = i
	return 1
}

/* encode SSR 3: satellite code biases ---------------------------------------*/
func (rtcm *Rtcm) encode_ssr3(sys, subtype, sync int) int {
	var (
		codes                                    []int
		udint                                    float64 = 0.0
		i, j, k, iod, nsat, prn, nbias, np, offp int
		code, bias                               [MAXCODE]int
	)

	Trace(4, "encode_ssr3: sys=%d subtype=%d sync=%d\n", sys, subtype, sync)

	switch sys {
	case SYS_GPS:
		np = 6
		offp = 0
		codes = codes_gps[:]

	case SYS_GLO:
		np = 5
		offp = 0
		codes = codes_glo[:]

	case SYS_GAL:
		np = 6
		offp = 0
		codes = codes_gal[:]

	case SYS_QZS:
		np = 4
		offp = 192
		codes = codes_qzs[:]

	case SYS_CMP:
		np = 6
		offp = 1
		codes = codes_bds[:]

	case SYS_SBS:
		np = 6
		offp = 120
		codes = codes_sbs[:]

	default:
		return 0
	}
	if subtype > 0 { /* IGS SSR */
		np = 6
		if sys == SYS_CMP {
			offp = 0
		} else if sys == SYS_SBS {
			offp = 119
		}
	}
	/* number of satellites */
	for j, nsat = 0, 0; j < MAXSAT; j++ {
		if SatSys(j+1, &prn) != sys || rtcm.Ssr[j].Update == 0 {
			continue
		}
		nsat++
		udint = rtcm.Ssr[j].Udi[4]
		iod = rtcm.Ssr[j].Iod[4]
	}
	/* encode SSR header */
	i = rtcm.encode_ssr_head(3, sys, subtype, nsat, sync, iod, udint, 0, 0, 0)

	for j, nsat = 0, 0; j < MAXSAT; j++ {
		if SatSys(j+1, &prn) != sys || rtcm.Ssr[j].Update == 0 {
			continue
		}

		for k, nbias = 0, 0; k < 32; k++ {
			if codes[k] == 0 || rtcm.Ssr[j].Cbias[codes[k]-1] == 0.0 {
				continue
			}
			code[nbias] = k
			bias[nbias] = ROUND_I(float64(rtcm.Ssr[j].Cbias[codes[k]-1]) / 0.01)
			nbias++
		}
		SetBitU(rtcm.Buff[:], i, np, uint32(prn-offp))
		i += np /* satellite ID */
		SetBitU(rtcm.Buff[:], i, 5, uint32(nbias))
		i += 5 /* number of code biases */

		for k = 0; k < nbias; k++ {
			SetBitU(rtcm.Buff[:], i, 5, uint32(code[k]))
			i += 5 /* signal indicator */
			SetBits(rtcm.Buff[:], i, 14, int32(bias[k]))
			i += 14 /* code bias */
		}
	}
	rtcm.Nbit = i
	return 1
}

/* encode SSR 4: combined orbit and clock corrections ------------------------*/
func (rtcm *Rtcm) encode_ssr4(sys, subtype, sync int) int {
	var (
		udint                                    float64 = 0.0
		i, j, iod, nsat, prn, iode, iodcrc, refd int
		deph, ddeph, dclk                        [3]int
	)

	Trace(4, "encode_ssr4: sys=%d subtype=%d sync=%d\n", sys, subtype, sync)

	np, ni, nj, offp, _, miss := selectsys(sys)
	if miss {
		return 0
	}
	if subtype > 0 { /* IGS SSR */
		np = 6
		ni = 8
		nj = 0
		if sys == SYS_CMP {
			offp = 0
		} else if sys == SYS_SBS {
			offp = 119
		}
	}
	/* number of satellites */
	for j, nsat = 0, 0; j < MAXSAT; j++ {
		if SatSys(j+1, &prn) != sys || rtcm.Ssr[j].Update == 0 {
			continue
		}
		nsat++
		udint = rtcm.Ssr[j].Udi[0]
		iod = rtcm.Ssr[j].Iod[0]
		refd = rtcm.Ssr[j].Refd
	}
	/* encode SSR header */
	i = rtcm.encode_ssr_head(4, sys, subtype, nsat, sync, iod, udint, refd, 0, 0)

	for j = 0; j < MAXSAT; j++ {
		if SatSys(j+1, &prn) != sys || rtcm.Ssr[j].Update == 0 {
			continue
		}

		iode = rtcm.Ssr[j].Iode
		iodcrc = rtcm.Ssr[j].IodCrc

		if subtype > 0 { /* IGS SSR */
			iode &= 0xFF
		}
		deph[0] = ROUND_I(rtcm.Ssr[j].Deph[0] / 1e-4)
		deph[1] = ROUND_I(rtcm.Ssr[j].Deph[1] / 4e-4)
		deph[2] = ROUND_I(rtcm.Ssr[j].Deph[2] / 4e-4)
		ddeph[0] = ROUND_I(rtcm.Ssr[j].Ddeph[0] / 1e-6)
		ddeph[1] = ROUND_I(rtcm.Ssr[j].Ddeph[1] / 4e-6)
		ddeph[2] = ROUND_I(rtcm.Ssr[j].Ddeph[2] / 4e-6)
		dclk[0] = ROUND_I(rtcm.Ssr[j].Dclk[0] / 1e-4)
		dclk[1] = ROUND_I(rtcm.Ssr[j].Dclk[1] / 1e-6)
		dclk[2] = ROUND_I(rtcm.Ssr[j].Dclk[2] / 2e-8)

		SetBitU(rtcm.Buff[:], i, np, uint32(prn-offp))
		i += np /* satellite ID */
		SetBitU(rtcm.Buff[:], i, ni, uint32(iode))
		i += ni /* IODE */
		SetBitU(rtcm.Buff[:], i, nj, uint32(iodcrc))
		i += nj /* IODCRC */
		SetBits(rtcm.Buff[:], i, 22, int32(deph[0]))
		i += 22 /* delta raidal */
		SetBits(rtcm.Buff[:], i, 20, int32(deph[1]))
		i += 20 /* delta along-track */
		SetBits(rtcm.Buff[:], i, 20, int32(deph[2]))
		i += 20 /* delta cross-track */
		SetBits(rtcm.Buff[:], i, 21, int32(ddeph[0]))
		i += 21 /* dot delta radial */
		SetBits(rtcm.Buff[:], i, 19, int32(ddeph[1]))
		i += 19 /* dot delta along-track */
		SetBits(rtcm.Buff[:], i, 19, int32(ddeph[2]))
		i += 19 /* dot delta cross-track */
		SetBits(rtcm.Buff[:], i, 22, int32(dclk[0]))
		i += 22 /* delta clock C0 */
		SetBits(rtcm.Buff[:], i, 21, int32(dclk[1]))
		i += 21 /* delta clock C1 */
		SetBits(rtcm.Buff[:], i, 27, int32(dclk[2]))
		i += 27 /* delta clock C2 */
	}
	rtcm.Nbit = i
	return 1
}

/* encode SSR 5: URA ---------------------------------------------------------*/
func (rtcm *Rtcm) encode_ssr5(sys, subtype, sync int) int {
	var (
		udint                     float64 = 0.0
		i, j, nsat, iod, prn, ura int
	)

	Trace(4, "encode_ssr5: sys=%d subtype=%d sync=%d\n", sys, subtype, sync)

	np, _, _, offp, _, miss := selectsys(sys)
	if miss {
		return 0
	}
	if subtype > 0 { /* IGS SSR */
		np = 6
		if sys == SYS_CMP {
			offp = 0
		} else if sys == SYS_SBS {
			offp = 119
		}
	}
	/* number of satellites */
	for j, nsat = 0, 0; j < MAXSAT; j++ {
		if SatSys(j+1, &prn) != sys || rtcm.Ssr[j].Update == 0 {
			continue
		}
		nsat++
		udint = rtcm.Ssr[j].Udi[3]
		iod = rtcm.Ssr[j].Iod[3]
	}
	/* encode ssr header */
	i = rtcm.encode_ssr_head(5, sys, subtype, nsat, sync, iod, udint, 0, 0, 0)

	for j = 0; j < MAXSAT; j++ {
		if SatSys(j+1, &prn) != sys || rtcm.Ssr[j].Update == 0 {
			continue
		}

		ura = rtcm.Ssr[j].Ura
		SetBitU(rtcm.Buff[:], i, np, uint32(prn-offp))
		i += np /* satellite id */
		SetBitU(rtcm.Buff[:], i, 6, uint32(ura))
		i += 6 /* ssr ura */
	}
	rtcm.Nbit = i
	return 1
}

/* encode SSR 6: high rate clock correction ----------------------------------*/
func (rtcm *Rtcm) encode_ssr6(sys, subtype, sync int) int {
	var (
		udint                       float64 = 0.0
		i, j, nsat, iod, prn, hrclk int
	)

	Trace(4, "encode_ssr6: sys=%d subtype=%d sync=%d\n", sys, subtype, sync)

	np, _, _, offp, _, miss := selectsys(sys)
	if miss {
		return 0
	}
	if subtype > 0 { /* IGS SSR */
		np = 6
		if sys == SYS_CMP {
			offp = 0
		} else if sys == SYS_SBS {
			offp = 119
		}
	}
	/* number of satellites */
	for j, nsat = 0, 0; j < MAXSAT; j++ {
		if SatSys(j+1, &prn) != sys || rtcm.Ssr[j].Update == 0 {
			continue
		}
		nsat++
		udint = rtcm.Ssr[j].Udi[2]
		iod = rtcm.Ssr[j].Iod[2]
	}
	/* encode SSR header */
	i = rtcm.encode_ssr_head(6, sys, subtype, nsat, sync, iod, udint, 0, 0, 0)

	for j = 0; j < MAXSAT; j++ {
		if SatSys(j+1, &prn) != sys || rtcm.Ssr[j].Update == 0 {
			continue
		}

		hrclk = ROUND_I(rtcm.Ssr[j].Brclk / 1e-4)

		SetBitU(rtcm.Buff[:], i, np, uint32(prn-offp))
		i += np /* satellite ID */
		SetBits(rtcm.Buff[:], i, 22, int32(hrclk))
		i += 22 /* high rate clock corr */
	}
	rtcm.Nbit = i
	return 1
}

/* encode SSR 7: satellite phase biases --------------------------------------*/
func (rtcm *Rtcm) encode_ssr7(sys, subtype, sync int) int {
	var (
		codes                                                       []int
		udint                                                       float64 = 0.0
		i, j, k, iod, nsat, prn, nbias, np, offp, yaw_ang, yaw_rate int
		code, pbias, stdpb                                          [MAXCODE]int
	)
	Trace(4, "encode_ssr7: sys=%d subtype=%d sync=%d\n", sys, subtype, sync)

	switch sys {
	case SYS_GPS:
		np = 6
		offp = 0
		codes = codes_gps[:]

	case SYS_GLO:
		np = 5
		offp = 0
		codes = codes_glo[:]

	case SYS_GAL:
		np = 6
		offp = 0
		codes = codes_gal[:]

	case SYS_QZS:
		np = 4
		offp = 192
		codes = codes_qzs[:]

	case SYS_CMP:
		np = 6
		offp = 1
		codes = codes_bds[:]

	case SYS_SBS:
		np = 6
		offp = 120
		codes = codes_sbs[:]

	default:
		return 0
	}
	if subtype > 0 { /* IGS SSR */
		np = 6
		if sys == SYS_CMP {
			offp = 0
		} else if sys == SYS_SBS {
			offp = 119
		}
	}
	/* number of satellites */
	for j, nsat = 0, 0; j < MAXSAT; j++ {
		if SatSys(j+1, &prn) != sys || rtcm.Ssr[j].Update == 0 {
			continue
		}
		nsat++
		udint = rtcm.Ssr[j].Udi[5]
		iod = rtcm.Ssr[j].Iod[5]
	}
	/* encode SSR header */
	i = rtcm.encode_ssr_head(7, sys, subtype, nsat, sync, iod, udint, 0, 0, 0)

	for j, nsat = 0, 0; j < MAXSAT; j++ {
		if SatSys(j+1, &prn) != sys || rtcm.Ssr[j].Update == 0 {
			continue
		}

		for k, nbias = 0, 0; k < 32; k++ {
			if codes[k] == 0 || rtcm.Ssr[j].Pbias[codes[k]-1] == 0.0 {
				continue
			}
			code[nbias] = k
			pbias[nbias] = ROUND_I(rtcm.Ssr[j].Pbias[codes[k]-1] / 0.0001)
			stdpb[nbias] = ROUND_I(float64(rtcm.Ssr[j].Stdpb[codes[k]-1]) / 0.0001)
			nbias++
		}
		yaw_ang = ROUND_I(rtcm.Ssr[j].Yaw_ang / 180.0 * 256.0)
		yaw_rate = ROUND_I(rtcm.Ssr[j].Yaw_rate / 180.0 * 8192.0)
		SetBitU(rtcm.Buff[:], i, np, uint32(prn-offp))
		i += np /* satellite ID */
		SetBitU(rtcm.Buff[:], i, 5, uint32(nbias))
		i += 5 /* number of code biases */
		SetBitU(rtcm.Buff[:], i, 9, uint32(yaw_ang))
		i += 9 /* yaw angle */
		SetBits(rtcm.Buff[:], i, 8, int32(yaw_rate))
		i += 8 /* yaw rate */

		for k = 0; k < nbias; k++ {
			SetBitU(rtcm.Buff[:], i, 5, uint32(code[k]))
			i += 5 /* signal indicator */
			SetBitU(rtcm.Buff[:], i, 1, 0)
			i += 1 /* integer-indicator */
			SetBitU(rtcm.Buff[:], i, 2, 0)
			i += 2 /* WL integer-indicator */
			SetBitU(rtcm.Buff[:], i, 4, 0)
			i += 4 /* discont counter */
			SetBits(rtcm.Buff[:], i, 20, int32(pbias[k]))
			i += 20 /* phase bias */
			if subtype == 0 {
				SetBits(rtcm.Buff[:], i, 17, int32(stdpb[k]))
				i += 17 /* std-dev ph-bias */
			}
		}
	}
	rtcm.Nbit = i
	return 1
}

/* satellite no to MSM satellite ID ------------------------------------------*/
func to_satid(sys, sat int) int {
	var prn int

	if SatSys(sat, &prn) != sys {
		return 0
	}

	if sys == SYS_QZS {
		prn -= MINPRNQZS - 1
	} else if sys == SYS_SBS {
		prn -= MINPRNSBS - 1
	}

	return prn
}

/* observation code to MSM signal ID -----------------------------------------*/
func to_sigid(sys int, code uint8) int {
	var msm_sig []string
	var sig string

	/* signal conversion for undefined signal by rtcm */
	if sys == SYS_GPS {
		switch code {
		case CODE_L1Y:
			code = CODE_L1P
		case CODE_L1M:
			code = CODE_L1P
		case CODE_L1N:
			code = CODE_L1P
		case CODE_L2D:
			code = CODE_L2P
		case CODE_L2Y:
			code = CODE_L2P
		case CODE_L2M:
			code = CODE_L2P
		case CODE_L2N:
			code = CODE_L2P
		}
	}
	sig = Code2Obs(code)

	switch sys {
	case SYS_GPS:
		msm_sig = msm_sig_gps[:]

	case SYS_GLO:
		msm_sig = msm_sig_glo[:]

	case SYS_GAL:
		msm_sig = msm_sig_gal[:]

	case SYS_QZS:
		msm_sig = msm_sig_qzs[:]

	case SYS_SBS:
		msm_sig = msm_sig_sbs[:]

	case SYS_CMP:
		msm_sig = msm_sig_cmp[:]

	case SYS_IRN:
		msm_sig = msm_sig_irn[:]

	default:
		return 0
	}
	for i := 0; i < 32; i++ {
		if strings.Compare(sig, msm_sig[i]) == 0 {
			return i + 1
		}
	}
	return 0
}

/* generate MSM satellite, signal and cell index -----------------------------*/
func (rtcm *Rtcm) gen_msm_index(sys int, nsat, nsig, ncell *int, sat_ind, sig_ind, cell_ind []uint8) {
	var i, j, sat, sig, cell int

	*nsat, *nsig, *ncell = 0, 0, 0

	/* generate satellite and signal index */
	for i = 0; i < rtcm.ObsData.N(); i++ {
		if sat = to_satid(sys, rtcm.ObsData.Data[i].Sat); sat == 0 {
			continue
		}

		for j = 0; j < NFREQ+NEXOBS; j++ {
			if sig = to_sigid(sys, rtcm.ObsData.Data[i].Code[j]); sig == 0 {
				continue
			}

			sat_ind[sat-1], sig_ind[sig-1] = 1, 1
		}
	}
	for i = 0; i < 64; i++ {
		if sat_ind[i] > 0 {
			(*nsat)++
			sat_ind[i] = uint8(*nsat)
		}
	}
	for i = 0; i < 32; i++ {
		if sig_ind[i] > 0 {
			(*nsig)++
			sig_ind[i] = uint8(*nsig)
		}
	}
	/* generate cell index */
	for i = 0; i < rtcm.ObsData.N(); i++ {
		if sat = to_satid(sys, rtcm.ObsData.Data[i].Sat); sat == 0 {
			continue
		}

		for j = 0; j < NFREQ+NEXOBS; j++ {
			if sig = to_sigid(sys, rtcm.ObsData.Data[i].Code[j]); sig == 0 {
				continue
			}

			cell = int(sig_ind[sig-1]) - 1 + int(sat_ind[sat-1]-1)*(*nsig)
			cell_ind[cell] = 1
		}
	}
	for i = 0; i < *nsat*(*nsig); i++ {
		if cell_ind[i] > 0 && (*ncell) < 64 {
			(*ncell)++
			cell_ind[i] = uint8(*ncell)
		}
	}
}

/* generate MSM satellite data fields ----------------------------------------*/
func (rtcm *Rtcm) gen_msm_sat(sys, nsat int, sat_ind []uint8, rrng, rrate []float64, info []uint8) {
	var (
		data                   *ObsD
		freq                   float64
		i, j, k, sat, sig, fcn int
	)

	for i = 0; i < 64; i++ {
		rrng[i], rrate[i] = 0.0, 0.0
	}

	for i = 0; i < rtcm.ObsData.N(); i++ {
		data = &rtcm.ObsData.Data[i]
		fcn = rtcm.FcnGlo(data.Sat) /* fcn+7 */

		if sat = to_satid(sys, data.Sat); sat == 0 {
			continue
		}

		for j = 0; j < NFREQ+NEXOBS; j++ {
			if sig = to_sigid(sys, data.Code[j]); sig == 0 {
				continue
			}
			k = int(sat_ind[sat-1]) - 1
			freq = Code2Freq(sys, data.Code[j], fcn-7)

			/* rough range (ms) and rough phase-range-rate (m/s) */
			if rrng[k] == 0.0 && data.P[j] != 0.0 {
				rrng[k] = float64(ROUND_I(data.P[j]/RANGE_MS/P2_10)) * RANGE_MS * P2_10
			}
			if rrate[k] == 0.0 && data.D[j] != 0.0 && freq > 0.0 {
				rrate[k] = float64(ROUND_I(-data.D[j]*CLIGHT/freq)) * 1.0
			}
			/* extended satellite info */
			if info != nil {
				info[k] = 0
				if sys == SYS_GLO {
					if fcn < 0 {
						info[k] = 15
					} else {
						info[k] = uint8(fcn)
					}
				}
			}
		}
	}
}

/* generate MSM signal data fields -------------------------------------------*/
func (rtcm *Rtcm) gen_msm_sig(sys, nsat, nsig, ncell int, sat_ind, sig_ind, cell_ind []uint8, rrng,
	rrate, psrng, phrng, rate, lock []float64, half []uint8, cnr []float32) {
	var (
		data                                       *ObsD
		freq, lambda, psrng_s, phrng_s, rate_s, lt float64
		i, j, k, sat, sig, fcn, cell, LLI          int
	)

	for i = 0; i < ncell; i++ {
		if psrng != nil {
			psrng[i] = 0.0
		}
		if phrng != nil {
			phrng[i] = 0.0
		}
		if rate != nil {
			rate[i] = 0.0
		}
	}
	for i = 0; i < rtcm.ObsData.N(); i++ {
		data = &rtcm.ObsData.Data[i]
		fcn = rtcm.FcnGlo(data.Sat) /* fcn+7 */

		if sat = to_satid(sys, data.Sat); sat == 0 {
			continue
		}

		for j = 0; j < NFREQ+NEXOBS; j++ {
			if sig = to_sigid(sys, data.Code[j]); sig == 0 {
				continue
			}

			k = int(sat_ind[sat-1]) - 1
			if cell = int(cell_ind[int(sig_ind[sig-1])-1+k*nsig]); cell >= 64 {
				continue
			}

			freq = Code2Freq(sys, data.Code[j], fcn-7)
			lambda = CLIGHT / freq
			if freq == 0.0 {
				lambda = 0.0
			}

			psrng_s = data.P[j] - rrng[k]
			if data.P[j] == 0.0 {
				psrng_s = 0.0
			}
			phrng_s = data.L[j]*lambda - rrng[k]
			if data.L[j] == 0.0 || lambda <= 0.0 {
				phrng_s = 0.0
			}
			rate_s = -data.D[j]*lambda - rrate[k]
			if data.D[j] == 0.0 || lambda <= 0.0 {
				rate_s = 0.0
			}

			/* subtract phase - psudorange integer cycle offset */
			LLI = int(data.LLI[j])
			if (LLI&1) > 0 || math.Abs(phrng_s-rtcm.Cp[data.Sat-1][j]) > 1171.0 {
				rtcm.Cp[data.Sat-1][j] = float64(ROUND_I(phrng_s/lambda)) * lambda
				LLI |= 1
			}
			phrng_s -= rtcm.Cp[data.Sat-1][j]

			lt = locktime_d(data.Time, &rtcm.Lltime[data.Sat-1][j], uint8(LLI))

			if psrng != nil && psrng_s != 0.0 {
				psrng[cell-1] = psrng_s
			}
			if phrng != nil && phrng_s != 0.0 {
				phrng[cell-1] = phrng_s
			}
			if rate != nil && rate_s != 0.0 {
				rate[cell-1] = rate_s
			}
			if lock != nil {
				lock[cell-1] = lt
			}
			if half != nil {
				half[cell-1] = 0
				if data.LLI[j]&2 > 0 {
					half[cell-1] = 1
				}
			}
			if cnr != nil {
				cnr[cell-1] = float32(data.SNR[j]) * SNR_UNIT
			}
		}
	}
}

/* encode MSM header ---------------------------------------------------------*/
func (rtcm *Rtcm) encode_msm_head(ctype, sys, sync int, nsat, ncell *int, rrng, rrate []float64,
	info []uint8, psrng, phrng, rate, lock []float64, half []uint8, cnr []float32) int {
	var (
		tow        float64
		sat_ind    [64]uint8
		sig_ind    [32]uint8
		cell_ind   [32 * 64]uint8
		dow, epoch uint32
		j, nsig    int
	)
	i := 24

	switch sys {
	case SYS_GPS:
		ctype += 1070

	case SYS_GLO:
		ctype += 1080

	case SYS_GAL:
		ctype += 1090

	case SYS_QZS:
		ctype += 1110

	case SYS_SBS:
		ctype += 1100

	case SYS_CMP:
		ctype += 1120

	case SYS_IRN:
		ctype += 1130

	default:
		return 0
	}
	/* generate msm satellite, signal and cell index */
	rtcm.gen_msm_index(sys, nsat, &nsig, ncell, sat_ind[:], sig_ind[:], cell_ind[:])

	if sys == SYS_GLO {
		/* GLONASS time (dow + tod-ms) */
		tow = Time2GpsT(TimeAdd(GpsT2Utc(rtcm.Time), 10800.0), nil)
		dow = uint32(tow / 86400.0)
		epoch = (dow << 27) + ROUND_U(math.Mod(tow, 86400.0)*1e3)
	} else if sys == SYS_CMP {
		/* BDS time (tow-ms) */
		epoch = ROUND_U(Time2GpsT(GpsT2BDT(rtcm.Time), nil) * 1e3)
	} else {
		/* GPS, QZSS, Galileo and IRNSS time (tow-ms) */
		epoch = ROUND_U(Time2GpsT(rtcm.Time, nil) * 1e3)
	}
	/* encode msm header (ref [15] table 3.5-78) */
	SetBitU(rtcm.Buff[:], i, 12, uint32(ctype))
	i += 12 /* message number */
	SetBitU(rtcm.Buff[:], i, 12, uint32(rtcm.StaId))
	i += 12 /* reference station id */
	SetBitU(rtcm.Buff[:], i, 30, uint32(epoch))
	i += 30 /* epoch time */
	SetBitU(rtcm.Buff[:], i, 1, uint32(sync))
	i += 1 /* multiple message bit */
	SetBitU(rtcm.Buff[:], i, 3, uint32(rtcm.SeqNo))
	i += 3 /* issue of data station */
	SetBitU(rtcm.Buff[:], i, 7, 0)
	i += 7 /* reserved */
	SetBitU(rtcm.Buff[:], i, 2, 0)
	i += 2 /* clock streering indicator */
	SetBitU(rtcm.Buff[:], i, 2, 0)
	i += 2 /* external clock indicator */
	SetBitU(rtcm.Buff[:], i, 1, 0)
	i += 1 /* smoothing indicator */
	SetBitU(rtcm.Buff[:], i, 3, 0)
	i += 3 /* smoothing interval */

	/* satellite mask */
	for j = 0; j < 64; j++ {
		SetBitU(rtcm.Buff[:], i, 1, 0)
		if sat_ind[j] > 0 {
			SetBitU(rtcm.Buff[:], i, 1, 1)
		}
		i += 1
	}
	/* signal mask */
	for j = 0; j < 32; j++ {
		SetBitU(rtcm.Buff[:], i, 1, 0)
		if sig_ind[j] > 0 {
			SetBitU(rtcm.Buff[:], i, 1, 1)
		}
		i += 1
	}
	/* cell mask */
	for j = 0; j < *nsat*nsig && j < 64; j++ {
		SetBitU(rtcm.Buff[:], i, 1, 0)
		if cell_ind[j] > 0 {
			SetBitU(rtcm.Buff[:], i, 1, 1)
		}
		i += 1
	}
	/* generate msm satellite data fields */
	rtcm.gen_msm_sat(sys, *nsat, sat_ind[:], rrng, rrate, info)

	/* generate msm signal data fields */
	rtcm.gen_msm_sig(sys, *nsat, nsig, *ncell, sat_ind[:], sig_ind[:], cell_ind[:], rrng, rrate,
		psrng, phrng, rate, lock, half, cnr)

	return i
}

/* encode rough range integer ms ---------------------------------------------*/
func (rtcm *Rtcm) encode_msm_int_rrng(i int, rrng []float64, nsat int) int {
	var int_ms uint32

	for j := 0; j < nsat; j++ {
		if rrng[j] == 0.0 {
			int_ms = 255
		} else if rrng[j] < 0.0 || rrng[j] > RANGE_MS*255.0 {
			Trace(2, "msm rough range overflow %s rrng=%.3f\n",
				TimeStr(rtcm.Time, 0), rrng[j])
			int_ms = 255
		} else {
			int_ms = ROUND_U(rrng[j]/RANGE_MS/P2_10) >> 10
		}
		SetBitU(rtcm.Buff[:], i, 8, int_ms)
		i += 8
	}
	return i
}

/* encode rough range modulo 1 ms --------------------------------------------*/
func (rtcm *Rtcm) encode_msm_mod_rrng(i int, rrng []float64, nsat int) int {
	var mod_ms uint32

	for j := 0; j < nsat; j++ {
		if rrng[j] <= 0.0 || rrng[j] > RANGE_MS*255.0 {
			mod_ms = 0
		} else {
			mod_ms = ROUND_U(rrng[j]/RANGE_MS/P2_10) & 0x3FF
		}
		SetBitU(rtcm.Buff[:], i, 10, mod_ms)
		i += 10
	}
	return i
}

/* encode extended satellite info --------------------------------------------*/
func (rtcm *Rtcm) encode_msm_info(i int, info []uint8, nsat int) int {
	for j := 0; j < nsat; j++ {
		SetBitU(rtcm.Buff[:], i, 4, uint32(info[j]))
		i += 4
	}
	return i
}

/* encode rough phase-range-rate ---------------------------------------------*/
func (rtcm *Rtcm) encode_msm_rrate(i int, rrate []float64, nsat int) int {
	var j, rrate_val int

	for j = 0; j < nsat; j++ {
		if math.Abs(rrate[j]) > 8191.0 {
			Trace(2, "msm rough phase-range-rate overflow %s rrate=%.4f\n",
				TimeStr(rtcm.Time, 0), rrate[j])
			rrate_val = -8192
		} else {
			rrate_val = ROUND_I(rrate[j] / 1.0)
		}
		SetBits(rtcm.Buff[:], i, 14, int32(rrate_val))
		i += 14
	}
	return i
}

/* encode fine pseudorange ---------------------------------------------------*/
func (rtcm *Rtcm) encode_msm_psrng(i int, psrng []float64, ncell int) int {
	var j, psrng_val int

	for j = 0; j < ncell; j++ {
		if psrng[j] == 0.0 {
			psrng_val = -16384
		} else if math.Abs(psrng[j]) > 292.7 {
			Trace(2, "msm fine pseudorange overflow %s psrng=%.3f\n",
				TimeStr(rtcm.Time, 0), psrng[j])
			psrng_val = -16384
		} else {
			psrng_val = ROUND_I(psrng[j] / RANGE_MS / P2_24)
		}
		SetBits(rtcm.Buff[:], i, 15, int32(psrng_val))
		i += 15
	}
	return i
}

/* encode fine pseudorange with extended resolution --------------------------*/
func (rtcm *Rtcm) encode_msm_psrng_ex(i int, psrng []float64, ncell int) int {
	var j, psrng_val int

	for j = 0; j < ncell; j++ {
		if psrng[j] == 0.0 {
			psrng_val = -524288
		} else if math.Abs(psrng[j]) > 292.7 {
			Trace(2, "msm fine pseudorange ext overflow %s psrng=%.3f\n",
				TimeStr(rtcm.Time, 0), psrng[j])
			psrng_val = -524288
		} else {
			psrng_val = ROUND_I(psrng[j] / RANGE_MS / P2_29)
		}
		SetBits(rtcm.Buff[:], i, 20, int32(psrng_val))
		i += 20
	}
	return i
}

/* encode fine phase-range ---------------------------------------------------*/
func (rtcm *Rtcm) encode_msm_phrng(i int, phrng []float64, ncell int) int {
	var j, phrng_val int

	for j = 0; j < ncell; j++ {
		if phrng[j] == 0.0 {
			phrng_val = -2097152
		} else if math.Abs(phrng[j]) > 1171.0 {
			Trace(2, "msm fine phase-range overflow %s phrng=%.3f\n",
				TimeStr(rtcm.Time, 0), phrng[j])
			phrng_val = -2097152
		} else {
			phrng_val = ROUND_I(phrng[j] / RANGE_MS / P2_29)
		}
		SetBits(rtcm.Buff[:], i, 22, int32(phrng_val))
		i += 22
	}
	return i
}

/* encode fine phase-range with extended resolution --------------------------*/
func (rtcm *Rtcm) encode_msm_phrng_ex(i int, phrng []float64, ncell int) int {
	var j, phrng_val int

	for j = 0; j < ncell; j++ {
		if phrng[j] == 0.0 {
			phrng_val = -8388608
		} else if math.Abs(phrng[j]) > 1171.0 {
			Trace(2, "msm fine phase-range ext overflow %s phrng=%.3f\n",
				TimeStr(rtcm.Time, 0), phrng[j])
			phrng_val = -8388608
		} else {
			phrng_val = ROUND_I(phrng[j] / RANGE_MS / P2_31)
		}
		SetBits(rtcm.Buff[:], i, 24, int32(phrng_val))
		i += 24
	}
	return i
}

/* encode lock-time indicator ------------------------------------------------*/
func (rtcm *Rtcm) encode_msm_lock(i int, lock []float64, ncell int) int {
	var j, lock_val int

	for j = 0; j < ncell; j++ {
		lock_val = to_msm_lock(lock[j])
		SetBitU(rtcm.Buff[:], i, 4, uint32(lock_val))
		i += 4
	}
	return i
}

/* encode lock-time indicator with extended range and resolution -------------*/
func (rtcm *Rtcm) encode_msm_lock_ex(i int, lock []float64, ncell int) int {
	var j, lock_val int

	for j = 0; j < ncell; j++ {
		lock_val = to_msm_lock_ex(lock[j])
		SetBitU(rtcm.Buff[:], i, 10, uint32(lock_val))
		i += 10
	}
	return i
}

/* encode half-cycle-ambiguity indicator -------------------------------------*/
func (rtcm *Rtcm) encode_msm_half_amb(i int, half []uint8, ncell int) int {
	for j := 0; j < ncell; j++ {
		SetBitU(rtcm.Buff[:], i, 1, uint32(half[j]))
		i += 1
	}
	return i
}

/* encode signal CNR ---------------------------------------------------------*/
func (rtcm *Rtcm) encode_msm_cnr(i int, cnr []float32, ncell int) int {
	var j, cnr_val int

	for j = 0; j < ncell; j++ {
		cnr_val = ROUND_I(float64(cnr[j]) / 1.0)
		SetBitU(rtcm.Buff[:], i, 6, uint32(cnr_val))
		i += 6
	}
	return i
}

/* encode signal CNR with extended resolution --------------------------------*/
func (rtcm *Rtcm) encode_msm_cnr_ex(i int, cnr []float32, ncell int) int {
	var j, cnr_val int

	for j = 0; j < ncell; j++ {
		cnr_val = ROUND_I(float64(cnr[j]) / 0.0625)
		SetBitU(rtcm.Buff[:], i, 10, uint32(cnr_val))
		i += 10
	}
	return i
}

/* encode fine phase-range-rate ----------------------------------------------*/
func (rtcm *Rtcm) encode_msm_rate(i int, rate []float64, ncell int) int {
	var j, rate_val int

	for j = 0; j < ncell; j++ {
		if rate[j] == 0.0 {
			rate_val = -16384
		} else if math.Abs(rate[j]) > 1.6384 {
			Trace(2, "msm fine phase-range-rate overflow %s rate=%.3f\n",
				TimeStr(rtcm.Time, 0), rate[j])
			rate_val = -16384
		} else {
			rate_val = ROUND_I(rate[j] / 0.0001)
		}
		SetBitU(rtcm.Buff[:], i, 15, uint32(rate_val))
		i += 15
	}
	return i
}

/* encode MSM 1: compact pseudorange -----------------------------------------*/
func (rtcm *Rtcm) encode_msm1(sys, sync int) int {
	var (
		rrng, rrate, psrng [64]float64
		i, nsat, ncell     int
	)

	Trace(3, "encode_msm1: sys=%d sync=%d\n", sys, sync)

	/* encode msm header */
	if i = rtcm.encode_msm_head(1, sys, sync, &nsat, &ncell, rrng[:], rrate[:], nil, psrng[:],
		nil, nil, nil, nil, nil); i == 0 {
		return 0
	}
	/* encode msm satellite data */
	i = rtcm.encode_msm_mod_rrng(i, rrng[:], nsat) /* rough range modulo 1 ms */

	/* encode msm signal data */
	i = rtcm.encode_msm_psrng(i, psrng[:], ncell) /* fine pseudorange */

	rtcm.Nbit = i
	return 1
}

/* encode MSM 2: compact phaserange ------------------------------------------*/
func (rtcm *Rtcm) encode_msm2(sys, sync int) int {
	var (
		rrng, rrate, phrng, lock [64]float64
		half                     [64]uint8
		i, nsat, ncell           int
	)

	Trace(4, "encode_msm2: sys=%d sync=%d\n", sys, sync)

	/* encode msm header */
	if i = rtcm.encode_msm_head(2, sys, sync, &nsat, &ncell, rrng[:], rrate[:], nil, nil,
		phrng[:], nil, lock[:], half[:], nil); i == 0 {
		return 0
	}
	/* encode msm satellite data */
	i = rtcm.encode_msm_mod_rrng(i, rrng[:], nsat) /* rough range modulo 1 ms */

	/* encode msm signal data */
	i = rtcm.encode_msm_phrng(i, phrng[:], ncell)   /* fine phase-range */
	i = rtcm.encode_msm_lock(i, lock[:], ncell)     /* lock-time indicator */
	i = rtcm.encode_msm_half_amb(i, half[:], ncell) /* half-cycle-amb indicator */

	rtcm.Nbit = i
	return 1
}

/* encode MSM 3: compact pseudorange and phaserange --------------------------*/
func (rtcm *Rtcm) encode_msm3(sys, sync int) int {
	var rrng, rrate, psrng, phrng, lock [64]float64
	var half [64]uint8
	var i, nsat, ncell int

	Trace(4, "encode_msm3: sys=%d sync=%d\n", sys, sync)

	/* encode msm header */
	if i = rtcm.encode_msm_head(3, sys, sync, &nsat, &ncell, rrng[:], rrate[:], nil, psrng[:],
		phrng[:], nil, lock[:], half[:], nil); i == 0 {
		return 0
	}
	/* encode msm satellite data */
	i = rtcm.encode_msm_mod_rrng(i, rrng[:], nsat) /* rough range modulo 1 ms */

	/* encode msm signal data */
	i = rtcm.encode_msm_psrng(i, psrng[:], ncell)   /* fine pseudorange */
	i = rtcm.encode_msm_phrng(i, phrng[:], ncell)   /* fine phase-range */
	i = rtcm.encode_msm_lock(i, lock[:], ncell)     /* lock-time indicator */
	i = rtcm.encode_msm_half_amb(i, half[:], ncell) /* half-cycle-amb indicator */

	rtcm.Nbit = i
	return 1
}

/* encode MSM 4: full pseudorange and phaserange plus CNR --------------------*/
func (rtcm *Rtcm) encode_msm4(sys, sync int) int {
	var (
		rrng, rrate, psrng, phrng, lock [64]float64
		cnr                             [64]float32
		half                            [64]uint8
		i, nsat, ncell                  int
	)

	Trace(4, "encode_msm4: sys=%d sync=%d\n", sys, sync)

	/* encode msm header */
	if i = rtcm.encode_msm_head(4, sys, sync, &nsat, &ncell, rrng[:], rrate[:], nil, psrng[:],
		phrng[:], nil, lock[:], half[:], cnr[:]); i == 0 {
		return 0
	}
	/* encode msm satellite data */
	i = rtcm.encode_msm_int_rrng(i, rrng[:], nsat) /* rough range integer ms */
	i = rtcm.encode_msm_mod_rrng(i, rrng[:], nsat) /* rough range modulo 1 ms */

	/* encode msm signal data */
	i = rtcm.encode_msm_psrng(i, psrng[:], ncell)   /* fine pseudorange */
	i = rtcm.encode_msm_phrng(i, phrng[:], ncell)   /* fine phase-range */
	i = rtcm.encode_msm_lock(i, lock[:], ncell)     /* lock-time indicator */
	i = rtcm.encode_msm_half_amb(i, half[:], ncell) /* half-cycle-amb indicator */
	i = rtcm.encode_msm_cnr(i, cnr[:], ncell)       /* signal cnr */
	rtcm.Nbit = i
	return 1
}

/* encode MSM 5: full pseudorange, phaserange, phaserangerate and CNR --------*/
func (rtcm *Rtcm) encode_msm5(sys, sync int) int {
	var (
		rrng, rrate, psrng, phrng, rate, lock [64]float64
		cnr                                   [64]float32
		info, half                            [64]uint8
		i, nsat, ncell                        int
	)

	Trace(4, "encode_msm5: sys=%d sync=%d\n", sys, sync)

	/* encode msm header */
	if i = rtcm.encode_msm_head(5, sys, sync, &nsat, &ncell, rrng[:], rrate[:], info[:], psrng[:],
		phrng[:], rate[:], lock[:], half[:], cnr[:]); i == 0 {
		return 0
	}
	/* encode msm satellite data */
	i = rtcm.encode_msm_int_rrng(i, rrng[:], nsat) /* rough range integer ms */
	i = rtcm.encode_msm_info(i, info[:], nsat)     /* extended satellite info */
	i = rtcm.encode_msm_mod_rrng(i, rrng[:], nsat) /* rough range modulo 1 ms */
	i = rtcm.encode_msm_rrate(i, rrate[:], nsat)   /* rough phase-range-rate */

	/* encode msm signal data */
	i = rtcm.encode_msm_psrng(i, psrng[:], ncell)   /* fine pseudorange */
	i = rtcm.encode_msm_phrng(i, phrng[:], ncell)   /* fine phase-range */
	i = rtcm.encode_msm_lock(i, lock[:], ncell)     /* lock-time indicator */
	i = rtcm.encode_msm_half_amb(i, half[:], ncell) /* half-cycle-amb indicator */
	i = rtcm.encode_msm_cnr(i, cnr[:], ncell)       /* signal cnr */
	i = rtcm.encode_msm_rate(i, rate[:], ncell)     /* fine phase-range-rate */
	rtcm.Nbit = i
	return 1
}

/* encode MSM 6: full pseudorange and phaserange plus CNR (high-res) ---------*/
func (rtcm *Rtcm) encode_msm6(sys, sync int) int {
	var (
		rrng, rrate, psrng, phrng, lock [64]float64
		cnr                             [64]float32
		half                            [64]uint8
		i, nsat, ncell                  int
	)

	Trace(4, "encode_msm6: sys=%d sync=%d\n", sys, sync)

	/* encode msm header */
	if i = rtcm.encode_msm_head(6, sys, sync, &nsat, &ncell, rrng[:], rrate[:], nil, psrng[:],
		phrng[:], nil, lock[:], half[:], cnr[:]); i == 0 {
		return 0
	}
	/* encode msm satellite data */
	i = rtcm.encode_msm_int_rrng(i, rrng[:], nsat) /* rough range integer ms */
	i = rtcm.encode_msm_mod_rrng(i, rrng[:], nsat) /* rough range modulo 1 ms */

	/* encode msm signal data */
	i = rtcm.encode_msm_psrng_ex(i, psrng[:], ncell) /* fine pseudorange ext */
	i = rtcm.encode_msm_phrng_ex(i, phrng[:], ncell) /* fine phase-range ext */
	i = rtcm.encode_msm_lock_ex(i, lock[:], ncell)   /* lock-time indicator ext */
	i = rtcm.encode_msm_half_amb(i, half[:], ncell)  /* half-cycle-amb indicator */
	i = rtcm.encode_msm_cnr_ex(i, cnr[:], ncell)     /* signal cnr ext */
	rtcm.Nbit = i
	return 1
}

/* encode MSM 7: full pseudorange, phaserange, phaserangerate and CNR (h-res) */
func (rtcm *Rtcm) encode_msm7(sys, sync int) int {
	var (
		rrng, rrate, psrng, phrng, rate, lock [64]float64
		cnr                                   [64]float32
		info, half                            [64]uint8
		i, nsat, ncell                        int
	)

	Trace(4, "encode_msm7: sys=%d sync=%d\n", sys, sync)

	/* encode msm header */
	if i = rtcm.encode_msm_head(7, sys, sync, &nsat, &ncell, rrng[:], rrate[:], info[:], psrng[:],
		phrng[:], rate[:], lock[:], half[:], cnr[:]); i == 0 {
		return 0
	}
	/* encode msm satellite data */
	i = rtcm.encode_msm_int_rrng(i, rrng[:], nsat) /* rough range integer ms */
	i = rtcm.encode_msm_info(i, info[:], nsat)     /* extended satellite info */
	i = rtcm.encode_msm_mod_rrng(i, rrng[:], nsat) /* rough range modulo 1 ms */
	i = rtcm.encode_msm_rrate(i, rrate[:], nsat)   /* rough phase-range-rate */

	/* encode msm signal data */
	i = rtcm.encode_msm_psrng_ex(i, psrng[:], ncell) /* fine pseudorange ext */
	i = rtcm.encode_msm_phrng_ex(i, phrng[:], ncell) /* fine phase-range ext */
	i = rtcm.encode_msm_lock_ex(i, lock[:], ncell)   /* lock-time indicator ext */
	i = rtcm.encode_msm_half_amb(i, half[:], ncell)  /* half-cycle-amb indicator */
	i = rtcm.encode_msm_cnr_ex(i, cnr[:], ncell)     /* signal cnr ext */
	i = rtcm.encode_msm_rate(i, rate[:], ncell)      /* fine phase-range-rate */
	rtcm.Nbit = i
	return 1
}

/* encode type 1230: GLONASS L1 and L2 code-phase biases ---------------------*/
func (rtcm *Rtcm) encode_type1230(sync int) int {
	var (
		j, align int
		bias     [4]int
	)
	i := 24
	mask := 15
	Trace(3, "encode_type1230: sync=%d\n", sync)

	align = rtcm.StaPara.glo_cp_align

	for j = 0; j < 4; j++ {
		bias[j] = ROUND_I(rtcm.StaPara.glo_cp_bias[j] / 0.02)
		if bias[j] <= -32768 || bias[j] > 32767 {
			bias[j] = -32768 /* invalid value */
		}
	}
	SetBitU(rtcm.Buff[:], i, 12, 1230)
	i += 12 /* message no */
	SetBitU(rtcm.Buff[:], i, 12, uint32(rtcm.StaId))
	i += 12 /* station ID */
	SetBitU(rtcm.Buff[:], i, 1, uint32(align))
	i += 1 /* GLO code-phase bias ind */
	SetBitU(rtcm.Buff[:], i, 3, 0)
	i += 3 /* reserved */
	SetBitU(rtcm.Buff[:], i, 4, uint32(mask))
	i += 4 /* GLO FDMA signals mask */
	SetBits(rtcm.Buff[:], i, 16, int32(bias[0]))
	i += 16 /* GLO C1 code-phase bias */
	SetBits(rtcm.Buff[:], i, 16, int32(bias[1]))
	i += 16 /* GLO P1 code-phase bias */
	SetBits(rtcm.Buff[:], i, 16, int32(bias[2]))
	i += 16 /* GLO C2 code-phase bias */
	SetBits(rtcm.Buff[:], i, 16, int32(bias[3]))
	i += 16 /* GLO P2 code-phase bias */
	rtcm.Nbit = i
	return 1
}

/* encode type 4073: proprietary message Mitsubishi Electric -----------------*/
func (rtcm *Rtcm) encode_type4073(subtype, sync int) int {
	Trace(4, "rtcm3 4073: unsupported message subtype=%d\n", subtype)
	return 0
}

/* encode type 4076: proprietary message IGS ---------------------------------*/
func (rtcm *Rtcm) encode_type4076(subtype, sync int) int {
	switch subtype {
	case 21:
		return rtcm.encode_ssr1(SYS_GPS, subtype, sync)
	case 22:
		return rtcm.encode_ssr2(SYS_GPS, subtype, sync)
	case 23:
		return rtcm.encode_ssr4(SYS_GPS, subtype, sync)
	case 24:
		return rtcm.encode_ssr6(SYS_GPS, subtype, sync)
	case 25:
		return rtcm.encode_ssr3(SYS_GPS, subtype, sync)
	case 26:
		return rtcm.encode_ssr7(SYS_GPS, subtype, sync)
	case 27:
		return rtcm.encode_ssr5(SYS_GPS, subtype, sync)
	case 41:
		return rtcm.encode_ssr1(SYS_GLO, subtype, sync)
	case 42:
		return rtcm.encode_ssr2(SYS_GLO, subtype, sync)
	case 43:
		return rtcm.encode_ssr4(SYS_GLO, subtype, sync)
	case 44:
		return rtcm.encode_ssr6(SYS_GLO, subtype, sync)
	case 45:
		return rtcm.encode_ssr3(SYS_GLO, subtype, sync)
	case 46:
		return rtcm.encode_ssr7(SYS_GLO, subtype, sync)
	case 47:
		return rtcm.encode_ssr5(SYS_GLO, subtype, sync)
	case 61:
		return rtcm.encode_ssr1(SYS_GAL, subtype, sync)
	case 62:
		return rtcm.encode_ssr2(SYS_GAL, subtype, sync)
	case 63:
		return rtcm.encode_ssr4(SYS_GAL, subtype, sync)
	case 64:
		return rtcm.encode_ssr6(SYS_GAL, subtype, sync)
	case 65:
		return rtcm.encode_ssr3(SYS_GAL, subtype, sync)
	case 66:
		return rtcm.encode_ssr7(SYS_GAL, subtype, sync)
	case 67:
		return rtcm.encode_ssr5(SYS_GAL, subtype, sync)
	case 81:
		return rtcm.encode_ssr1(SYS_QZS, subtype, sync)
	case 82:
		return rtcm.encode_ssr2(SYS_QZS, subtype, sync)
	case 83:
		return rtcm.encode_ssr4(SYS_QZS, subtype, sync)
	case 84:
		return rtcm.encode_ssr6(SYS_QZS, subtype, sync)
	case 85:
		return rtcm.encode_ssr3(SYS_QZS, subtype, sync)
	case 86:
		return rtcm.encode_ssr7(SYS_QZS, subtype, sync)
	case 87:
		return rtcm.encode_ssr5(SYS_QZS, subtype, sync)
	case 101:
		return rtcm.encode_ssr1(SYS_CMP, subtype, sync)
	case 102:
		return rtcm.encode_ssr2(SYS_CMP, subtype, sync)
	case 103:
		return rtcm.encode_ssr4(SYS_CMP, subtype, sync)
	case 104:
		return rtcm.encode_ssr6(SYS_CMP, subtype, sync)
	case 105:
		return rtcm.encode_ssr3(SYS_CMP, subtype, sync)
	case 106:
		return rtcm.encode_ssr7(SYS_CMP, subtype, sync)
	case 107:
		return rtcm.encode_ssr5(SYS_CMP, subtype, sync)
	case 121:
		return rtcm.encode_ssr1(SYS_SBS, subtype, sync)
	case 122:
		return rtcm.encode_ssr2(SYS_SBS, subtype, sync)
	case 123:
		return rtcm.encode_ssr4(SYS_SBS, subtype, sync)
	case 124:
		return rtcm.encode_ssr6(SYS_SBS, subtype, sync)
	case 125:
		return rtcm.encode_ssr3(SYS_SBS, subtype, sync)
	case 126:
		return rtcm.encode_ssr7(SYS_SBS, subtype, sync)
	case 127:
		return rtcm.encode_ssr5(SYS_SBS, subtype, sync)
	}
	Trace(5, "rtcm3 4076: unsupported message subtype=%d\n", subtype)
	return 0
}

/* encode RTCM ver.3 message -------------------------------------------------*/
func (rtcm *Rtcm) EncodeRtcm3(ctype, subtype, sync int) int {
	var ret int = 0

	Trace(4, "encode_rtcm3: type=%d subtype=%d sync=%d\n", ctype, subtype, sync)

	switch ctype {
	case 1001:
		ret = rtcm.encode_type1001(sync)

	case 1002:
		ret = rtcm.encode_type1002(sync)

	case 1003:
		ret = rtcm.encode_type1003(sync)

	case 1004:
		ret = rtcm.encode_type1004(sync)

	case 1005:
		ret = rtcm.encode_type1005(sync)

	case 1006:
		ret = rtcm.encode_type1006(sync)

	case 1007:
		ret = rtcm.encode_type1007(sync)

	case 1008:
		ret = rtcm.encode_type1008(sync)

	case 1009:
		ret = rtcm.encode_type1009(sync)

	case 1010:
		ret = rtcm.encode_type1010(sync)

	case 1011:
		ret = rtcm.encode_type1011(sync)

	case 1012:
		ret = rtcm.encode_type1012(sync)

	case 1019:
		ret = rtcm.encode_type1019(sync)

	case 1020:
		ret = rtcm.encode_type1020(sync)

	case 1033:
		ret = rtcm.encode_type1033(sync)

	case 1041:
		ret = rtcm.encode_type1041(sync)

	case 1042:
		ret = rtcm.encode_type1042(sync)

	case 1044:
		ret = rtcm.encode_type1044(sync)

	case 1045:
		ret = rtcm.encode_type1045(sync)

	case 1046:
		ret = rtcm.encode_type1046(sync)

	case 63:
		ret = rtcm.encode_type63(sync)
		/* draft */
	case 1057:
		ret = rtcm.encode_ssr1(SYS_GPS, 0, sync)

	case 1058:
		ret = rtcm.encode_ssr2(SYS_GPS, 0, sync)

	case 1059:
		ret = rtcm.encode_ssr3(SYS_GPS, 0, sync)

	case 1060:
		ret = rtcm.encode_ssr4(SYS_GPS, 0, sync)

	case 1061:
		ret = rtcm.encode_ssr5(SYS_GPS, 0, sync)

	case 1062:
		ret = rtcm.encode_ssr6(SYS_GPS, 0, sync)

	case 1063:
		ret = rtcm.encode_ssr1(SYS_GLO, 0, sync)

	case 1064:
		ret = rtcm.encode_ssr2(SYS_GLO, 0, sync)

	case 1065:
		ret = rtcm.encode_ssr3(SYS_GLO, 0, sync)

	case 1066:
		ret = rtcm.encode_ssr4(SYS_GLO, 0, sync)

	case 1067:
		ret = rtcm.encode_ssr5(SYS_GLO, 0, sync)

	case 1068:
		ret = rtcm.encode_ssr6(SYS_GLO, 0, sync)

	case 1071:
		ret = rtcm.encode_msm1(SYS_GPS, sync)

	case 1072:
		ret = rtcm.encode_msm2(SYS_GPS, sync)

	case 1073:
		ret = rtcm.encode_msm3(SYS_GPS, sync)

	case 1074:
		ret = rtcm.encode_msm4(SYS_GPS, sync)

	case 1075:
		ret = rtcm.encode_msm5(SYS_GPS, sync)

	case 1076:
		ret = rtcm.encode_msm6(SYS_GPS, sync)

	case 1077:
		ret = rtcm.encode_msm7(SYS_GPS, sync)

	case 1081:
		ret = rtcm.encode_msm1(SYS_GLO, sync)

	case 1082:
		ret = rtcm.encode_msm2(SYS_GLO, sync)

	case 1083:
		ret = rtcm.encode_msm3(SYS_GLO, sync)

	case 1084:
		ret = rtcm.encode_msm4(SYS_GLO, sync)

	case 1085:
		ret = rtcm.encode_msm5(SYS_GLO, sync)

	case 1086:
		ret = rtcm.encode_msm6(SYS_GLO, sync)

	case 1087:
		ret = rtcm.encode_msm7(SYS_GLO, sync)

	case 1091:
		ret = rtcm.encode_msm1(SYS_GAL, sync)

	case 1092:
		ret = rtcm.encode_msm2(SYS_GAL, sync)

	case 1093:
		ret = rtcm.encode_msm3(SYS_GAL, sync)

	case 1094:
		ret = rtcm.encode_msm4(SYS_GAL, sync)

	case 1095:
		ret = rtcm.encode_msm5(SYS_GAL, sync)

	case 1096:
		ret = rtcm.encode_msm6(SYS_GAL, sync)

	case 1097:
		ret = rtcm.encode_msm7(SYS_GAL, sync)

	case 1101:
		ret = rtcm.encode_msm1(SYS_SBS, sync)

	case 1102:
		ret = rtcm.encode_msm2(SYS_SBS, sync)

	case 1103:
		ret = rtcm.encode_msm3(SYS_SBS, sync)

	case 1104:
		ret = rtcm.encode_msm4(SYS_SBS, sync)

	case 1105:
		ret = rtcm.encode_msm5(SYS_SBS, sync)

	case 1106:
		ret = rtcm.encode_msm6(SYS_SBS, sync)

	case 1107:
		ret = rtcm.encode_msm7(SYS_SBS, sync)

	case 1111:
		ret = rtcm.encode_msm1(SYS_QZS, sync)

	case 1112:
		ret = rtcm.encode_msm2(SYS_QZS, sync)

	case 1113:
		ret = rtcm.encode_msm3(SYS_QZS, sync)

	case 1114:
		ret = rtcm.encode_msm4(SYS_QZS, sync)

	case 1115:
		ret = rtcm.encode_msm5(SYS_QZS, sync)

	case 1116:
		ret = rtcm.encode_msm6(SYS_QZS, sync)

	case 1117:
		ret = rtcm.encode_msm7(SYS_QZS, sync)

	case 1121:
		ret = rtcm.encode_msm1(SYS_CMP, sync)

	case 1122:
		ret = rtcm.encode_msm2(SYS_CMP, sync)

	case 1123:
		ret = rtcm.encode_msm3(SYS_CMP, sync)

	case 1124:
		ret = rtcm.encode_msm4(SYS_CMP, sync)

	case 1125:
		ret = rtcm.encode_msm5(SYS_CMP, sync)

	case 1126:
		ret = rtcm.encode_msm6(SYS_CMP, sync)

	case 1127:
		ret = rtcm.encode_msm7(SYS_CMP, sync)

	case 1131:
		ret = rtcm.encode_msm1(SYS_IRN, sync)

	case 1132:
		ret = rtcm.encode_msm2(SYS_IRN, sync)

	case 1133:
		ret = rtcm.encode_msm3(SYS_IRN, sync)

	case 1134:
		ret = rtcm.encode_msm4(SYS_IRN, sync)

	case 1135:
		ret = rtcm.encode_msm5(SYS_IRN, sync)

	case 1136:
		ret = rtcm.encode_msm6(SYS_IRN, sync)

	case 1137:
		ret = rtcm.encode_msm7(SYS_IRN, sync)

	case 1230:
		ret = rtcm.encode_type1230(sync)

	case 1240:
		ret = rtcm.encode_ssr1(SYS_GAL, 0, sync)
		/* draft */
	case 1241:
		ret = rtcm.encode_ssr2(SYS_GAL, 0, sync)
		/* draft */
	case 1242:
		ret = rtcm.encode_ssr3(SYS_GAL, 0, sync)
		/* draft */
	case 1243:
		ret = rtcm.encode_ssr4(SYS_GAL, 0, sync)
		/* draft */
	case 1244:
		ret = rtcm.encode_ssr5(SYS_GAL, 0, sync)
		/* draft */
	case 1245:
		ret = rtcm.encode_ssr6(SYS_GAL, 0, sync)
		/* draft */
	case 1246:
		ret = rtcm.encode_ssr1(SYS_QZS, 0, sync)
		/* draft */
	case 1247:
		ret = rtcm.encode_ssr2(SYS_QZS, 0, sync)
		/* draft */
	case 1248:
		ret = rtcm.encode_ssr3(SYS_QZS, 0, sync)
		/* draft */
	case 1249:
		ret = rtcm.encode_ssr4(SYS_QZS, 0, sync)
		/* draft */
	case 1250:
		ret = rtcm.encode_ssr5(SYS_QZS, 0, sync)
		/* draft */
	case 1251:
		ret = rtcm.encode_ssr6(SYS_QZS, 0, sync)
		/* draft */
	case 1252:
		ret = rtcm.encode_ssr1(SYS_SBS, 0, sync)
		/* draft */
	case 1253:
		ret = rtcm.encode_ssr2(SYS_SBS, 0, sync)
		/* draft */
	case 1254:
		ret = rtcm.encode_ssr3(SYS_SBS, 0, sync)
		/* draft */
	case 1255:
		ret = rtcm.encode_ssr4(SYS_SBS, 0, sync)
		/* draft */
	case 1256:
		ret = rtcm.encode_ssr5(SYS_SBS, 0, sync)
		/* draft */
	case 1257:
		ret = rtcm.encode_ssr6(SYS_SBS, 0, sync)
		/* draft */
	case 1258:
		ret = rtcm.encode_ssr1(SYS_CMP, 0, sync)
		/* draft */
	case 1259:
		ret = rtcm.encode_ssr2(SYS_CMP, 0, sync)
		/* draft */
	case 1260:
		ret = rtcm.encode_ssr3(SYS_CMP, 0, sync)
		/* draft */
	case 1261:
		ret = rtcm.encode_ssr4(SYS_CMP, 0, sync)
		/* draft */
	case 1262:
		ret = rtcm.encode_ssr5(SYS_CMP, 0, sync)
		/* draft */
	case 1263:
		ret = rtcm.encode_ssr6(SYS_CMP, 0, sync)
		/* draft */
	case 11:
		ret = rtcm.encode_ssr7(SYS_GPS, 0, sync)
		/* tentative */
	case 12:
		ret = rtcm.encode_ssr7(SYS_GAL, 0, sync)
		/* tentative */
	case 13:
		ret = rtcm.encode_ssr7(SYS_QZS, 0, sync)
		/* tentative */
	case 14:
		ret = rtcm.encode_ssr7(SYS_CMP, 0, sync)
		/* tentative */
	case 4073:
		ret = rtcm.encode_type4073(subtype, sync)

	case 4076:
		ret = rtcm.encode_type4076(subtype, sync)

	}
	if ret > 0 {
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
