/*------------------------------------------------------------------------------
* rtcm2.c : rtcm ver.2 message functions
*
*          Copyright (C) 2009-2014 by T.TAKASU, All rights reserved.
*
* references :
*     see rtcm.c
*
* version : $Revision:$ $Date:$
* history : 2011/11/28 1.0  separated from rtcm.c
*           2014/10/21 1.1  fix problem on week rollover in rtcm 2 type 14
*		    2022/05/31 1.0  rewrite rtcm2.c with golang by fxb
*-----------------------------------------------------------------------------*/

package gnssgo

import (
	"fmt"
	"math"
)

/* adjust hourly rollover of rtcm 2 time -------------------------------------*/
func (rtcm *Rtcm) AdjHour(zcnt float64) {
	var (
		tow, hour, sec float64
		week           int
	)

	/* if no time, get cpu time */
	if rtcm.Time.Time == 0 {
		rtcm.Time = Utc2GpsT(TimeGet())
	}
	tow = Time2GpsT(rtcm.Time, &week)
	hour = math.Floor(tow / 3600.0)
	sec = tow - hour*3600.0
	if zcnt < sec-1800.0 {
		zcnt += 3600.0
	} else if zcnt > sec+1800.0 {
		zcnt -= 3600.0
	}
	rtcm.Time = GpsT2Time(week, hour*3600+zcnt)
}

/* get observation data index ------------------------------------------------*/
func (obs *Obs) ObsIndex(time Gtime, sat int) int {
	var i, j int

	for i = 0; i < obs.N(); i++ {
		if obs.Data[i].Sat == sat {
			return i /* field already exists */
		}
	}
	if i >= MAXOBS {
		return -1 /* overflow */
	}

	/* add new field */
	var data ObsD

	data.Time = time
	data.Sat = sat
	for j = 0; j < NFREQ+NEXOBS; j++ {
		data.L[j], data.P[j] = 0.0, 0.0
		data.D[j] = 0.0
		data.SNR[j], data.LLI[j], data.Code[j] = 0, 0, 0
	}
	obs.AddObsData(&data)
	return i
}

/* decode type 1/9: differential gps correction/partial correction set -------*/
func (rtcm *Rtcm) decode_type1() int {
	var (
		i, fact, udre, prn, sat, iod int
		prc, rrc                     float64
	)

	Trace(4, "decode_type1: len=%d\n", rtcm.MsgLen)

	for i = 48; i+40 <= rtcm.MsgLen*8; {
		fact = int(GetBitU(rtcm.Buff[:], i, 1))
		i += 1
		udre = int(GetBitU(rtcm.Buff[:], i, 2))
		i += 2
		prn = int(GetBitU(rtcm.Buff[:], i, 5))
		i += 5
		prc = float64(GetBits(rtcm.Buff[:], i, 16))
		i += 16
		rrc = float64(GetBits(rtcm.Buff[:], i, 8))
		i += 8
		iod = int(GetBits(rtcm.Buff[:], i, 8))
		i += 8
		if prn == 0 {
			prn = 32
		}
		if prc == 0x80000000 || rrc == 0xFFFF8000 {
			Trace(2, "rtcm2 1 prc/rrc indicates satellite problem: prn=%d\n", prn)
			continue
		}

		sat = SatNo(SYS_GPS, prn)
		rtcm.Dgps[sat-1].t0 = rtcm.Time
		rtcm.Dgps[sat-1].prc = prc * 0.02
		rtcm.Dgps[sat-1].rrc = rrc * 0.002
		if fact > 0 {
			rtcm.Dgps[sat-1].prc = prc * 0.32
			rtcm.Dgps[sat-1].rrc = rrc * 0.032
		}
		rtcm.Dgps[sat-1].iod = iod
		rtcm.Dgps[sat-1].udre = float64(udre)

	}
	return 7
}

/* decode type 3: reference station parameter --------------------------------*/
func (rtcm *Rtcm) decode_type3() int {
	var i int = 48

	Trace(4, "decode_type3: len=%d\n", rtcm.MsgLen)

	if i+96 <= rtcm.MsgLen*8 {
		rtcm.StaPara.Pos[0] = float64(GetBits(rtcm.Buff[:], i, 32)) * 0.01
		i += 32
		rtcm.StaPara.Pos[1] = float64(GetBits(rtcm.Buff[:], i, 32)) * 0.01
		i += 32
		rtcm.StaPara.Pos[2] = float64(GetBits(rtcm.Buff[:], i, 32)) * 0.01
	} else {
		Trace(2, "rtcm2 3 length error: len=%d\n", rtcm.MsgLen)
		return -1
	}
	return 5
}

/* decode type 14: gps time of week ------------------------------------------*/
func (rtcm *Rtcm) decode_type14() int {
	var (
		zcnt              float64
		week, hour, leaps int
	)
	i := 48
	Trace(4, "decode_type14: len=%d\n", rtcm.MsgLen)

	zcnt = float64(GetBitU(rtcm.Buff[:], 24, 13))
	if i+24 <= rtcm.MsgLen*8 {
		week = int(GetBitU(rtcm.Buff[:], i, 10))
		i += 10
		hour = int(GetBitU(rtcm.Buff[:], i, 8))
		i += 8
		leaps = int(GetBitU(rtcm.Buff[:], i, 6))
	} else {
		Trace(2, "rtcm2 14 length error: len=%d\n", rtcm.MsgLen)
		return -1
	}
	week = AdjGpsWeek(week)
	rtcm.Time = GpsT2Time(week, float64(hour)*3600.0+zcnt*0.6)
	rtcm.NavData.Utc_gps[4] = float64(leaps)
	return 6
}

/* decode type 16: gps special message ---------------------------------------*/
func (rtcm *Rtcm) decode_type16() int {
	var (
		i, n int = 48, 0
		msg  [128]rune
	)
	Trace(4, "decode_type16: len=%d\n", rtcm.MsgLen)

	for ; i+8 <= rtcm.MsgLen*8 && n < 90; n, i = n+1, i+8 {
		msg[n] = rune(GetBitU(rtcm.Buff[:], i, 8))
	}
	rtcm.Msg = string(msg[:n])

	Trace(5, "rtcm2 16 message: %s\n", rtcm.Msg)
	return 9
}

/* decode type 17: gps ephemerides -------------------------------------------*/
func (rtcm *Rtcm) decode_type17() int {
	var (
		eph            Eph
		toc, sqrtA     float64
		week, prn, sat int
	)
	i := 48
	Trace(4, "decode_type17: len=%d\n", rtcm.MsgLen)

	if i+480 <= rtcm.MsgLen*8 {
		week = int(GetBitU(rtcm.Buff[:], i, 10))
		i += 10
		eph.Idot = float64(GetBits(rtcm.Buff[:], i, 14)) * P2_43 * SC2RAD
		i += 14
		eph.Iode = int(GetBitU(rtcm.Buff[:], i, 8))
		i += 8
		toc = float64(GetBitU(rtcm.Buff[:], i, 16)) * 16.0
		i += 16
		eph.F1 = float64(GetBits(rtcm.Buff[:], i, 16)) * P2_43
		i += 16
		eph.F2 = float64(GetBits(rtcm.Buff[:], i, 8)) * P2_55
		i += 8
		eph.Crs = float64(GetBits(rtcm.Buff[:], i, 16)) * P2_5
		i += 16
		eph.Deln = float64(GetBits(rtcm.Buff[:], i, 16)) * P2_43 * SC2RAD
		i += 16
		eph.Cuc = float64(GetBits(rtcm.Buff[:], i, 16)) * P2_29
		i += 16
		eph.E = float64(GetBitU(rtcm.Buff[:], i, 32)) * P2_33
		i += 32
		eph.Cus = float64(GetBits(rtcm.Buff[:], i, 16))
		i += 16
		sqrtA = float64(GetBitU(rtcm.Buff[:], i, 32)) * P2_19
		i += 32
		eph.Toes = float64(GetBitU(rtcm.Buff[:], i, 16))
		i += 16
		eph.OMG0 = float64(GetBits(rtcm.Buff[:], i, 32)) * P2_31 * SC2RAD
		i += 32
		eph.Cic = float64(GetBits(rtcm.Buff[:], i, 16)) * P2_29
		i += 16
		eph.I0 = float64(GetBits(rtcm.Buff[:], i, 32)) * P2_31 * SC2RAD
		i += 32
		eph.Cis = float64(GetBits(rtcm.Buff[:], i, 16)) * P2_29
		i += 16
		eph.Omg = float64(GetBits(rtcm.Buff[:], i, 32)) * P2_31 * SC2RAD
		i += 32
		eph.Crc = float64(GetBits(rtcm.Buff[:], i, 16)) * P2_5
		i += 16
		eph.OMGd = float64(GetBits(rtcm.Buff[:], i, 24)) * P2_43 * SC2RAD
		i += 24
		eph.M0 = float64(GetBits(rtcm.Buff[:], i, 32)) * P2_31 * SC2RAD
		i += 32
		eph.Iodc = int(GetBitU(rtcm.Buff[:], i, 10))
		i += 10
		eph.F0 = float64(GetBits(rtcm.Buff[:], i, 22)) * P2_31
		i += 22
		prn = int(GetBitU(rtcm.Buff[:], i, 5))
		i += 5 + 3
		eph.Tgd[0] = float64(GetBits(rtcm.Buff[:], i, 8)) * P2_31
		i += 8
		eph.Code = int(GetBitU(rtcm.Buff[:], i, 2))
		i += 2
		eph.Sva = int(GetBitU(rtcm.Buff[:], i, 4))
		i += 4
		eph.Svh = int(GetBitU(rtcm.Buff[:], i, 6))
		i += 6
		eph.Flag = int(GetBitU(rtcm.Buff[:], i, 1))
	} else {
		Trace(2, "rtcm2 17 length error: len=%d\n", rtcm.MsgLen)
		return -1
	}
	if prn == 0 {
		prn = 32
	}
	sat = SatNo(SYS_GPS, prn)
	eph.Sat = sat
	eph.Week = AdjGpsWeek(week)
	eph.Toe = GpsT2Time(eph.Week, eph.Toes)
	eph.Toc = GpsT2Time(eph.Week, toc)
	eph.Ttr = rtcm.Time
	eph.A = sqrtA * sqrtA
	rtcm.NavData.Ephs[sat-1] = eph
	rtcm.EphSet = 0
	rtcm.EphSat = sat
	return 2
}

/* decode type 18: rtk uncorrected carrier-phase -----------------------------*/
func (rtcm *Rtcm) decode_type18() int {
	var (
		time                                         Gtime
		usec, cp, tt                                 float64
		index, freq, sync, code, sys, prn, sat, loss int
	)
	i := 48
	Trace(4, "decode_type18: len=%d\n", rtcm.MsgLen)
	sync = 1
	if i+24 <= rtcm.MsgLen*8 {
		freq = int(GetBitU(rtcm.Buff[:], i, 2))
		i += 2 + 2
		usec = float64(GetBitU(rtcm.Buff[:], i, 20))
		i += 20
	} else {
		Trace(2, "rtcm2 18 length error: len=%d\n", rtcm.MsgLen)
		return -1
	}
	if freq&0x1 > 0 {
		Trace(2, "rtcm2 18 not supported frequency: freq=%d\n", freq)
		return -1
	}
	freq >>= 1

	for i+48 <= rtcm.MsgLen*8 && rtcm.ObsData.N() < MAXOBS+1 {
		sync = int(GetBitU(rtcm.Buff[:], i, 1))
		i += 1
		code = int(GetBitU(rtcm.Buff[:], i, 1))
		i += 1
		sys = int(GetBitU(rtcm.Buff[:], i, 1))
		i += 1
		prn = int(GetBitU(rtcm.Buff[:], i, 5))
		i += 5 + 3
		loss = int(GetBitU(rtcm.Buff[:], i, 5))
		i += 5
		cp = float64(GetBits(rtcm.Buff[:], i, 32))
		i += 32
		if prn == 0 {
			prn = 32
		}
		sysno := SYS_GPS
		if sys > 0 {
			sysno = SYS_GLO
		}
		if sat = SatNo(sysno, prn); sat == 0 {
			Trace(2, "rtcm2 18 satellite number error: sys=%d prn=%d\n", sys, prn)
			continue
		}
		time = TimeAdd(rtcm.Time, usec*1e-6)
		if sys > 0 {
			time = Utc2GpsT(time) /* convert glonass time to gpst */
		}

		tt = TimeDiff(rtcm.ObsData.Data[0].Time, time)
		if rtcm.ObsFlag > 0 || math.Abs(tt) > 1e-9 {
			rtcm.ObsData.Data, rtcm.ObsFlag = nil, 0
		}
		if index = rtcm.ObsData.ObsIndex(time, sat); index >= 0 {
			rtcm.ObsData.Data[index].L[freq] = -cp / 256.0
			if int(rtcm.Loss[sat-1][freq]) != loss {
				rtcm.ObsData.Data[index].LLI[freq] = 1
			} else {
				rtcm.ObsData.Data[index].LLI[freq] = 0
			}
			if freq == 0 {
				if code > 0 {
					rtcm.ObsData.Data[index].Code[freq] = CODE_L1P
				} else {
					rtcm.ObsData.Data[index].Code[freq] = CODE_L1C
				}
			} else {
				if code > 0 {
					rtcm.ObsData.Data[index].Code[freq] = CODE_L2P
				} else {
					rtcm.ObsData.Data[index].Code[freq] = CODE_L2C
				}
			}
			rtcm.Loss[sat-1][freq] = uint16(loss)
		}
	}

	return retsync(sync, &rtcm.ObsFlag)
}

/* decode type 19: rtk uncorrected pseudorange -------------------------------*/
func (rtcm *Rtcm) decode_type19() int {
	var (
		time                             Gtime
		usec, pr, tt                     float64
		index, freq, code, sys, prn, sat int
	)
	i := 48
	sync := 1
	Trace(4, "decode_type19: len=%d\n", rtcm.MsgLen)

	if i+24 <= rtcm.MsgLen*8 {
		freq = int(GetBitU(rtcm.Buff[:], i, 2))
		i += 2 + 2
		usec = float64(GetBitU(rtcm.Buff[:], i, 20))
		i += 20
	} else {
		Trace(2, "rtcm2 19 length error: len=%d\n", rtcm.MsgLen)
		return -1
	}
	if freq&0x1 > 0 {
		Trace(2, "rtcm2 19 not supported frequency: freq=%d\n", freq)
		return -1
	}
	freq >>= 1

	for i+48 <= rtcm.MsgLen*8 && rtcm.ObsData.N() < MAXOBS+1 {
		sync = int(GetBitU(rtcm.Buff[:], i, 1))
		i += 1
		code = int(GetBitU(rtcm.Buff[:], i, 1))
		i += 1
		sys = int(GetBitU(rtcm.Buff[:], i, 1))
		i += 1
		prn = int(GetBitU(rtcm.Buff[:], i, 5))
		i += 5 + 8
		pr = float64(GetBitU(rtcm.Buff[:], i, 32))
		i += 32
		if prn == 0 {
			prn = 32
		}
		sysno := SYS_GPS
		if sys > 0 {
			sysno = SYS_GLO
		}
		if sat = SatNo(sysno, prn); sat == 0 {
			Trace(2, "rtcm2 19 satellite number error: sys=%d prn=%d\n", sys, prn)
			continue
		}
		time = TimeAdd(rtcm.Time, usec*1e-6)
		if sys > 0 {
			time = Utc2GpsT(time) /* convert glonass time to gpst */
		}

		tt = TimeDiff(rtcm.ObsData.Data[0].Time, time)
		if rtcm.ObsFlag > 0 || math.Abs(tt) > 1e-9 {
			rtcm.ObsData.Data, rtcm.ObsFlag = nil, 0
		}
		if index = rtcm.ObsData.ObsIndex(time, sat); index >= 0 {
			rtcm.ObsData.Data[index].P[freq] = pr * 0.02
			if freq == 0 {
				if code > 0 {
					rtcm.ObsData.Data[index].Code[freq] = CODE_L1P
				} else {
					rtcm.ObsData.Data[index].Code[freq] = CODE_L1C
				}
			} else {
				if code > 0 {
					rtcm.ObsData.Data[index].Code[freq] = CODE_L2P
				} else {
					rtcm.ObsData.Data[index].Code[freq] = CODE_L2C
				}
			}
		}
	}
	return retsync(sync, &rtcm.ObsFlag)
}

/* decode type 22: extended reference station parameter ----------------------*/
func (rtcm *Rtcm) decode_type22() int {
	var (
		del    [2][3]float64
		hgt    float64 = 0.0
		j, noh int
	)
	i := 48
	Trace(4, "decode_type22: len=%d\n", rtcm.MsgLen)

	if i+24 <= rtcm.MsgLen*8 {
		del[0][0] = float64(GetBits(rtcm.Buff[:], i, 8)) / 25600.0
		i += 8
		del[0][1] = float64(GetBits(rtcm.Buff[:], i, 8)) / 25600.0
		i += 8
		del[0][2] = float64(GetBits(rtcm.Buff[:], i, 8)) / 25600.0
		i += 8
	} else {
		Trace(2, "rtcm2 22 length error: len=%d\n", rtcm.MsgLen)
		return -1
	}
	if i+24 <= rtcm.MsgLen*8 {
		i += 5
		noh = int(GetBits(rtcm.Buff[:], i, 1))
		i += 1
		if noh <= 0 {
			hgt = float64(GetBitU(rtcm.Buff[:], i, 18)) / 25600.0
		}

		i += 18
	}
	if i+24 <= rtcm.MsgLen*8 {
		del[1][0] = float64(GetBits(rtcm.Buff[:], i, 8)) / 1600.0
		i += 8
		del[1][1] = float64(GetBits(rtcm.Buff[:], i, 8)) / 1600.0
		i += 8
		del[1][2] = float64(GetBits(rtcm.Buff[:], i, 8)) / 1600.0
	}
	rtcm.StaPara.DelType = 1 /* xyz */
	for j = 0; j < 3; j++ {
		rtcm.StaPara.Del[j] = del[0][j]
	}
	rtcm.StaPara.Hgt = hgt
	return 5
}

/* decode type 23: antenna type definition record ----------------------------*/
func (rtcm *Rtcm) decode_type23() int {
	return 0
}

/* decode type 24: antenna reference point (arp) -----------------------------*/
func (rtcm *Rtcm) decode_type24() int {
	return 0
}

/* decode type 31: differential glonass correction ---------------------------*/
func (rtcm *Rtcm) decode_type31() int {
	return 0
}

/* decode type 32: differential glonass reference station parameters ---------*/
func (rtcm *Rtcm) decode_type32() int {
	return 0
}

/* decode type 34: glonass partial differential correction set ---------------*/
func (rtcm *Rtcm) decode_type34() int {
	return 0
}

/* decode type 36: glonass special message -----------------------------------*/
func (rtcm *Rtcm) decode_type36() int {
	return 0
}

/* decode type 37: gnss system time offset -----------------------------------*/
func (rtcm *Rtcm) decode_type37() int {
	return 0
}

/* decode type 59: proprietary message ---------------------------------------*/
func (rtcm *Rtcm) decode_type59() int {
	return 0
}

/* decode rtcm ver.2 message -------------------------------------------------*/
func (rtcm *Rtcm) DecodeRtcm2() int {
	var (
		zcnt                    float64
		staid, seqno, stah, ret int
	)
	ctype := int(GetBitU(rtcm.Buff[:], 8, 6))

	Trace(4, "decode_rtcm2: type=%2d len=%3d\n", ctype, rtcm.MsgLen)

	if zcnt = float64(GetBitU(rtcm.Buff[:], 24, 13)) * 0.6; zcnt >= 3600.0 {
		Trace(2, "rtcm2 modified z-count error: zcnt=%.1f\n", zcnt)
		return -1
	}
	rtcm.AdjHour(zcnt)
	staid = int(GetBitU(rtcm.Buff[:], 14, 10))
	seqno = int(GetBitU(rtcm.Buff[:], 37, 3))
	stah = int(GetBitU(rtcm.Buff[:], 45, 3))
	if seqno-rtcm.SeqNo != 1 && seqno-rtcm.SeqNo != -7 {
		Trace(2, "rtcm2 message outage: seqno=%d.%d\n", rtcm.SeqNo, seqno)
	}
	rtcm.SeqNo = seqno
	rtcm.StaHealth = stah

	if rtcm.OutType > 0 {
		rtcm.MsgType = fmt.Sprintf("RTCM %2d (%4d) zcnt=%7.1f staid=%3d seqno=%d",
			ctype, rtcm.MsgLen, zcnt, staid, seqno)
	}
	if ctype == 3 || ctype == 22 || ctype == 23 || ctype == 24 {
		if rtcm.StaId != 0 && staid != rtcm.StaId {
			Trace(2, "rtcm2 station id changed: %d.%d\n", rtcm.StaId, staid)
		}
		rtcm.StaId = staid
	}
	if rtcm.StaId != 0 && staid != rtcm.StaId {
		Trace(2, "rtcm2 station id invalid: %d %d\n", staid, rtcm.StaId)
		return -1
	}
	switch ctype {
	case 1:
		ret = rtcm.decode_type1()
	case 3:
		ret = rtcm.decode_type3()
	case 9:
		ret = rtcm.decode_type1()
	case 14:
		ret = rtcm.decode_type14()
	case 16:
		ret = rtcm.decode_type16()
	case 17:
		ret = rtcm.decode_type17()
	case 18:
		ret = rtcm.decode_type18()
	case 19:
		ret = rtcm.decode_type19()
	case 22:
		ret = rtcm.decode_type22()
	case 23:
		ret = rtcm.decode_type23()
		/* not supported */
	case 24:
		ret = rtcm.decode_type24()
		/* not supported */
	case 31:
		ret = rtcm.decode_type31()
		/* not supported */
	case 32:
		ret = rtcm.decode_type32()
		/* not supported */
	case 34:
		ret = rtcm.decode_type34()
		/* not supported */
	case 36:
		ret = rtcm.decode_type36()
		/* not supported */
	case 37:
		ret = rtcm.decode_type37()
		/* not supported */
	case 59:
		ret = rtcm.decode_type59()
		/* not supported */
	}
	if ret >= 0 {
		if 1 <= ctype && ctype <= 99 {
			rtcm.Nmsg2[ctype]++
		} else {
			rtcm.Nmsg2[0]++
		}
	}
	return ret
}
