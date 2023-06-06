/*------------------------------------------------------------------------------
* skytraq.c : skytraq receiver dependent functions
*
*          Copyright (C) 2009-2020 by T.TAKASU, All rights reserved.
*          Copyright (C) 2009-2022 by Feng Xuebin, All rights reserved.
*
* reference :
*     [1] Skytraq, Application Note AN0023 Binary Message of SkyTraq Venus 6
*         GPS Receiver, ver 1.4.8, August 21, 2008
*     [2] Skytraq, Application Note AN0024 Raw Measurement Binary Message
*         Extension of SkyTraq Venus 6 GPS Receiver, ver 0.5, October 9, 2009
*     [3] Skytraq, Application Note AN0024G2 Binary Message of SkyTraq Venus 7
*         GLONASS/GPS Receiver (Raw Measurement F/W), ver 1.4.26, April 26, 2012
*     [4] Skytraq, Application Note AN0030 Binary Message of Raw Measurement
*         Data Extension of SkyTraq Venus 8 GNSS Receiver, ver.1.4.29,
*         April 3, 2014
*     [5] Skytraq, Application Note AN0030 Binary Message of Raw Measurement
*         Data Extension of SkyTraq Venus 8 GNSS Receiver, ver.1.4.31,
*         August 12, 2014
*     [6] Skytraq, Application Note AN0030 Binary Message of Raw Measurement
*         Data Extension of SkyTraq Venus 8 GNSS Receiver, ver.1.4.32,
*         Sep 26, 2016
*
* notes   :
*     The byte order of S1315F raw message is big-endian inconsistent to [1].
*
* version : $Revision:$
* history : 2009/10/10 1.0 new
*           2009/11/08 1.1 flip carrier-phase polarity for F/W 1.8.23-20091106
*           2011/05/27 1.2 add almanac decoding
*                          fix problem with ARM compiler
*           2011/07/01 1.3 suppress warning
*           2013/03/10 1.5 change option -invcp to -INVCP
*           2014/11/09 1.6 support glonass, qzss and beidou
*           2016/10/09 1.7 support F/W version specified as ref [6]
*           2017/04/11 1.8 (char *) . (signed char *)
*           2017/05/08 1.9 fix bug on decoding extended raw meas v.1 (0xE5)
*                          fix bug on encoding CFG-BIN message (0x1E)
*                          add decode of ack/nack to request msg (0x83/0x84)
*           2020/10/30 1.10 add adjustment of gps week by cpu time
*                           CODE_L1I . CODE_L2I for BDS
*                           use integer type in stdint.h
*                           suppress warnings
*           2022/09/23 1.8 rewrite with golang
*-----------------------------------------------------------------------------*/
package gnssgo

import (
	"fmt"
	"io"
	"math"
	"os"
	"strconv"
	"strings"
)

const (
	STQSYNC1     = 0xA0 /* skytraq binary sync code 1 */
	STQSYNC2     = 0xA1 /* skytraq binary sync code 2 */
	ID_STQTIME   = 0xDC /* skytraq message id: measurement epoch */
	ID_STQRAW    = 0xDD /* skytraq message id: raw measurement */
	ID_STQSVCH   = 0xDE /* skytraq message id: SV and channel status */
	ID_STQSTAT   = 0xDF /* skytraq message id: navigation status */
	ID_STQGPS    = 0xE0 /* skytraq message id: gps/qzs subframe */
	ID_STQGLO    = 0xE1 /* skytraq message id: glonass string */
	ID_STQBDSD1  = 0xE2 /* skytraq message id: beidou d1 subframe */
	ID_STQBDSD2  = 0xE3 /* skytraq message id: beidou d2 subframe */
	ID_STQRAWX   = 0xE5 /* skytraq message id: extended raw meas v.1 */
	ID_STQGLOE   = 0x5C /* skytraq message id: glonass ephemeris */
	ID_STQACK    = 0x83 /* skytraq message id: ack to request msg */
	ID_STQNACK   = 0x84 /* skytraq message id: nack to request msg */
	ID_RESTART   = 0x01 /* skytraq message id: system restart */
	ID_CFGSERI   = 0x05 /* skytraq message id: configure serial port */
	ID_CFGFMT    = 0x09 /* skytraq message id: configure message format */
	ID_CFGRATE   = 0x12 /* skytraq message id: configure message rate */
	ID_CFGBIN    = 0x1E /* skytraq message id: configure binary message */
	ID_GETGLOEPH = 0x5B /* skytraq message id: get glonass ephemeris */
)

/* extract field (big-endian) ------------------------------------------------*/
// declared in binex.go
// #define U1(raw.Buff[idx:])       (*((uint8 *)(raw.Buff[idx:])))
// #define I1(raw.Buff[idx:])       (*((int8_t  *)(raw.Buff[idx:])))

// static uint16 U2(uint8 *raw.Buff[idx:])
// {
//     uint16 value;
//     uint8 *q=(uint8 *)&value+1;
//     int i;
//     for (i=0;i<2;i++) *q--=*raw.Buff[idx++;
//     return value;
// }
// static uint32 U4(uint8 *raw.Buff[idx:])
// {
//     uint32 value;
//     uint8 *q=(uint8 *)&value+3;
//     int i;
//     for (i=0;i<4;i++) *q--=*raw.Buff[idx++;
//     return value;
// }
// static float R4(uint8 *raw.Buff[idx:])
// {
//     float value;
//     uint8 *q=(uint8 *)&value+3;
//     int i;
//     for (i=0;i<4;i++) *q--=*raw.Buff[idx++;
//     return value;
// }
// static double R8(uint8 *raw.Buff[idx:])
// {
//     double value;
//     uint8 *q=(uint8 *)&value+7;
//     int i;
//     for (i=0;i<8;i++) *q--=*raw.Buff[idx++;
//     return value;
// }
/* checksum ------------------------------------------------------------------*/
func checksum_stq(buff []uint8, len int) uint8 {
	var cs uint8 = 0

	for i := 4; i < len-3; i++ {
		cs ^= buff[i]
	}
	return cs
}

/* 8-bit week . full week ---------------------------------------------------*/
// declared in javd.go commented by fxb
// static void adj_utcweek(gtime_t time, double *utc)
// {
//     int week;

//     if (utc[3]>=256.0) return;
//     time2gpst(time,&week);
//     utc[3]+=week/256*256;
//     if      (utc[3]<week-128) utc[3]+=256.0;
//     else if (utc[3]>week+128) utc[3]-=256.0;
// }
/* decode skytraq measurement epoch (0xDC) -----------------------------------*/
func decode_stqtime(raw *Raw) int {
	var (
		tow  float64
		week int
	)
	const idx = 4

	Trace(4, "decode_stqtime: len=%d\n", raw.Len)

	raw.Iod = int(U1(raw.Buff[idx+1:]))
	week = int(U2(raw.Buff[idx+2:]))
	week = AdjGpsWeek(week)
	tow = float64(U4(raw.Buff[idx+4:])) * 0.001
	raw.Time = GpsT2Time(week, tow)

	if raw.OutType > 0 {
		copy(raw.MsgType[len(string(raw.MsgType[:])):], []byte(fmt.Sprintf("SKYTRAQ EPOCH (%4d): iod=%d week=%d tow=%.3f",
			raw.Len, raw.Iod, week, tow)))
	}
	return 0
}

/* decode skytraq raw measurement (0xDD) -------------------------------------*/
func decode_stqraw(raw *Raw) int {
	var (
		idx                               = 4
		ind                               uint8
		pr1, cp1                          float64
		i, j, iod, prn, sys, sat, n, nsat int
	)

	Trace(4, "decode_stqraw: len=%d\n", raw.Len)

	if raw.OutType > 0 {
		copy(raw.MsgType[len(string(raw.MsgType[:])):], []byte(fmt.Sprintf("SKYTRAQ RAW   (%4d): nsat=%d", raw.Len, U1(raw.Buff[idx+2:]))))
	}
	iod = int(U1(raw.Buff[idx+1:]))
	if iod != raw.Iod { /* need preceding measurement epoch (0xDC) */
		Trace(2, "stq raw iod error: iod=%d %d\n", iod, raw.Iod)
		return -1
	}
	nsat = int(U1(raw.Buff[idx+2:]))
	if raw.Len < 8+23*nsat {
		Trace(2, "stq raw length error: len=%d nsat=%d\n", raw.Len, nsat)
		return -1
	}
	for i, idx = 0, idx+3; i < nsat && i < MAXOBS; i, idx = i+1, idx+23 {
		prn = int(U1(raw.Buff[:][idx:]))

		if MINPRNGPS <= prn && prn <= MAXPRNGPS {
			sys = SYS_GPS
		} else if MINPRNGLO <= prn-64 && prn-64 <= MAXPRNGLO {
			sys = SYS_GLO
			prn -= 64
		} else if MINPRNQZS <= prn && prn <= MAXPRNQZS {
			sys = SYS_QZS
		} else if MINPRNCMP <= prn-200 && prn-200 <= MAXPRNCMP {
			sys = SYS_CMP
			prn -= 200
		} else {
			Trace(2, "stq raw satellite number error: prn=%d\n", prn)
			continue
		}
		if sat = SatNo(sys, prn); sat == 0 {
			Trace(2, "stq raw satellite number error: sys=%d prn=%d\n", sys, prn)
			continue
		}
		ind = U1(raw.Buff[idx+22:])
		pr1 = 0.0
		if ind&1 > 0 {
			pr1 = R8(raw.Buff[idx+2:])
		}
		cp1 = 0.0
		if ind&4 > 0 {
			cp1 = R8(raw.Buff[idx+10:])
		}
		cp1 -= math.Floor((cp1+1e9)/2e9) * 2e9 /* -10^9 < cp1 < 10^9 */

		raw.ObsData.Data[n].P[0] = pr1
		raw.ObsData.Data[n].L[0] = cp1
		raw.ObsData.Data[n].D[0] = 0.0
		if ind&2 > 0 {
			raw.ObsData.Data[n].D[0] = float64(R4(raw.Buff[idx+18:]))
		}
		raw.ObsData.Data[n].SNR[0] = uint16(float64(U1(raw.Buff[idx+1:]))/SNR_UNIT + 0.5)
		raw.ObsData.Data[n].LLI[0] = 0
		if sys == SYS_CMP {
			raw.ObsData.Data[n].Code[0] = CODE_L2I
		} else {
			raw.ObsData.Data[n].Code[0] = CODE_L1C
		}
		raw.LockTime[sat-1][0] = 0
		if ind&8 > 0 {
			raw.LockTime[sat-1][0] = 1 /* cycle slip */
		}

		if raw.ObsData.Data[n].L[0] != 0.0 {
			raw.ObsData.Data[n].LLI[0] = uint8(raw.LockTime[sat-1][0])
			raw.LockTime[sat-1][0] = 0
		}
		/* receiver dependent options */
		if strings.Contains(raw.Opt, "-INVCP") {
			raw.ObsData.Data[n].L[0] *= -1.0
		}
		raw.ObsData.Data[n].Time = raw.Time
		raw.ObsData.Data[n].Sat = sat

		for j = 1; j < NFREQ+NEXOBS; j++ {
			raw.ObsData.Data[n].L[j], raw.ObsData.Data[n].P[j] = 0.0, 0.0
			raw.ObsData.Data[n].D[j] = 0.0
			raw.ObsData.Data[n].SNR[j], raw.ObsData.Data[n].LLI[j] = 0, 0
			raw.ObsData.Data[n].Code[j] = CODE_NONE
		}
		n++
	}
	raw.ObsData.n = n
	if n > 0 {
		return 1
	}
	return 0
}

/* decode skytraq extended raw measurement data v.1 (0xE5) -------------------*/
func decode_stqrawx(raw *Raw) int {
	var (
		idx                                     = 4
		ind                                     uint8
		tow, pr1, cp1                           float64
		i, j, week, nsat, sys, prn, sat, n, sig int
		gnss_type, signal_type                  int
	)

	Trace(4, "decode_stqraw: len=%d\n", raw.Len)

	if raw.OutType > 0 {
		copy(raw.MsgType[len(string(raw.MsgType[:])):], []byte(fmt.Sprintf("SKYTRAQ RAWX  (%4d): nsat=%2d", raw.Len, U1(raw.Buff[idx+13:]))))
	}
	_ = U1(raw.Buff[idx+1:]) /* ver */
	raw.Iod = int(U1(raw.Buff[idx+2:]))
	week = int(U2(raw.Buff[idx+3:]))
	week = AdjGpsWeek(week)
	tow = float64(U4(raw.Buff[idx+5:])) * 0.001
	raw.Time = GpsT2Time(week, tow)
	_ = float64(U2(raw.Buff[idx+9:])) * 0.001 /* peri */
	nsat = int(U1(raw.Buff[idx+13:]))
	if raw.Len < 19+31*nsat {
		Trace(2, "stq raw length error: len=%d nsat=%d\n", raw.Len, nsat)
		return -1
	}
	for i, idx = 0, idx+14; i < nsat && i < MAXOBS; i, idx = i+1, idx+31 {
		gnss_type = int(U1(raw.Buff[idx:]) & 0xF)
		signal_type = int(U1(raw.Buff[idx:])>>4) & 0xF
		switch gnss_type {
		case 0:
			{ /* GPS */
				sys = SYS_GPS
				switch signal_type {
				case 1:
					sig = CODE_L1X
				case 2:
					sig = CODE_L2X
				case 4:
					sig = CODE_L5X
				default:
					sig = CODE_L1C
				}
				prn = int(U1(raw.Buff[idx+1:]))
			}
		case 1:
			{ /* SBAS */
				sys = SYS_SBS
				sig = CODE_L1C
				prn = int(U1(raw.Buff[idx+1:]))
			}
		case 2:
			{ /* GLONASS */
				sys = SYS_GLO
				switch signal_type {
				case 2:
					sig = CODE_L2C
				case 4:
					sig = CODE_L3X
				default:
					sig = CODE_L1C
				}
				prn = int(U1(raw.Buff[idx+1:]))
			}
		case 3:
			{ /* Galileo */
				sys = SYS_GAL
				switch signal_type {
				case 4:
					sig = CODE_L5X
				case 5:
					sig = CODE_L7X
				case 6:
					sig = CODE_L6X
				default:
					sig = CODE_L1C
				}
				prn = int(U1(raw.Buff[idx+1:]))
			}
		case 4:
			{ /* QZSS */
				sys = SYS_QZS
				switch signal_type {
				case 1:
					sig = CODE_L1X
				case 2:
					sig = CODE_L2X
				case 4:
					sig = CODE_L5X
				case 6:
					sig = CODE_L6X
				default:
					sig = CODE_L1C
				}
				prn = int(U1(raw.Buff[idx+1:]))
			}
		case 5:
			{ /* BeiDou */
				sys = SYS_CMP
				switch signal_type {
				case 4:
					sig = CODE_L7I
				case 6:
					sig = CODE_L6I
				default:
					sig = CODE_L2I
				}
				prn = int(U1(raw.Buff[idx+1:]))
			}
		default:
			{
				Trace(2, "stq rawx gnss type error: type=%d\n", U1(raw.Buff[idx:]))
				continue
			}
		}
		if sat = SatNo(sys, prn); sat == 0 {
			Trace(2, "stq raw satellite number error: sys=%d prn=%d\n", sys, prn)
			continue
		}
		/* set glonass freq channel number */
		if gnss_type == 2 {
			raw.NavData.Geph[prn-1].Frq = (int)(U1(raw.Buff[idx+2:])&0xF) - 7
		}
		Trace(5, "stq raw sig: %d\n", sig)
		ind = uint8(U2(raw.Buff[idx+27:]))
		pr1 = 0.0
		if ind&1 > 0 {
			pr1 = R8(raw.Buff[idx+4:])
		}
		cp1 = 0.0
		if ind&4 > 0 {
			cp1 = R8(raw.Buff[idx+12:])
		}
		cp1 -= math.Floor((cp1+1e9)/2e9) * 2e9 /* -10^9 < cp1 < 10^9 */

		raw.ObsData.Data[n].P[0] = pr1
		raw.ObsData.Data[n].L[0] = cp1
		raw.ObsData.Data[n].D[0] = 0.0
		if ind&2 > 0 {
			raw.ObsData.Data[n].D[0] = float64(R4(raw.Buff[idx+20:]))
		}
		raw.ObsData.Data[n].SNR[0] = (uint16)(float64(U1(raw.Buff[idx+3:]))/SNR_UNIT + 0.5)
		raw.ObsData.Data[n].LLI[0] = 0
		if sys == SYS_CMP {
			raw.ObsData.Data[n].Code[0] = CODE_L2I
		} else {
			raw.ObsData.Data[n].Code[0] = CODE_L1C
		}
		raw.LockTime[sat-1][0] = 0
		if ind&8 > 0 {
			raw.LockTime[sat-1][0] = 1 /* cycle slip */
		}

		if raw.ObsData.Data[n].L[0] != 0.0 {
			raw.ObsData.Data[n].LLI[0] = uint8(raw.LockTime[sat-1][0])
			raw.LockTime[sat-1][0] = 0
		}
		/* receiver dependent options */
		if strings.Contains(raw.Opt, "-INVCP") {
			raw.ObsData.Data[n].L[0] *= -1.0
		}
		raw.ObsData.Data[n].Time = raw.Time
		raw.ObsData.Data[n].Sat = sat

		for j = 1; j < NFREQ+NEXOBS; j++ {
			raw.ObsData.Data[n].L[j], raw.ObsData.Data[n].P[j] = 0.0, 0.0
			raw.ObsData.Data[n].D[j] = 0.0
			raw.ObsData.Data[n].SNR[j], raw.ObsData.Data[n].LLI[j] = 0, 0
			raw.ObsData.Data[n].Code[j] = CODE_NONE
		}
		n++
	}
	raw.ObsData.n = n
	if n > 0 {
		return 1
	}
	return 0
}

/* save subframe -------------------------------------------------------------*/
func save_subfrm(sat int, raw *Raw) int {
	var (
		i, id int
	)
	const idx = 7

	Trace(4, "save_subfrm: sat=%2d\n", sat)

	/* check navigation subframe preamble */
	if raw.Buff[idx:][0] != 0x8B {
		Trace(2, "stq subframe preamble error: 0x%02X\n", raw.Buff[idx:][0])
		return 0
	}
	id = int(raw.Buff[idx:][5]>>2) & 0x7

	/* check subframe id */
	if id < 1 || 5 < id {
		Trace(2, "stq subframe id error: id=%d\n", id)
		return 0
	}
	q := raw.SubFrm[sat-1][(id-1)*30:]

	for i = 0; i < 30; i++ {
		q[i] = raw.Buff[idx:][i]
	}

	return id
}

/* decode ephemeris ----------------------------------------------------------*/
func decode_ephem(sat int, raw *Raw) int {
	var eph Eph

	Trace(4, "decode_ephem: sat=%2d\n", sat)

	if DecodeFrame(raw.SubFrm[sat-1][:], &eph, nil, nil, nil) == 0 {
		return 0
	}

	if !strings.Contains(raw.Opt, "-EPHALL") {
		if eph.Iode == raw.NavData.Ephs[sat-1].Iode &&
			eph.Iodc == raw.NavData.Ephs[sat-1].Iodc {
			return 0
		} /* unchanged */
	}
	eph.Sat = sat
	raw.NavData.Ephs[sat-1] = eph
	raw.EphSat = sat
	raw.EphSet = 0
	return 2
}

/* decode almanac and ion/utc ------------------------------------------------*/
func decode_alm1(sat int, raw *Raw) int {
	sys := SatSys(sat, nil)

	Trace(4, "decode_alm1 : sat=%2d\n", sat)

	if sys == SYS_GPS {
		DecodeFrame(raw.SubFrm[sat-1][:], nil, raw.NavData.Alm, raw.NavData.Ion_gps[:],
			raw.NavData.Utc_gps[:])
		adj_utcweek(raw.Time, raw.NavData.Utc_gps[:])
	} else if sys == SYS_QZS {
		DecodeFrame(raw.SubFrm[sat-1][:], nil, raw.NavData.Alm, raw.NavData.Ion_qzs[:],
			raw.NavData.Utc_qzs[:])
		adj_utcweek(raw.Time, raw.NavData.Utc_qzs[:])
	}
	return 9
}

/* decode almanac ------------------------------------------------------------*/
func decode_alm2(sat int, raw *Raw) int {
	sys := SatSys(sat, nil)

	Trace(4, "decode_alm2 : sat=%2d\n", sat)

	if sys == SYS_GPS {
		DecodeFrame(raw.SubFrm[sat-1][:], nil, raw.NavData.Alm, nil, nil)
	} else if sys == SYS_QZS {
		DecodeFrame(raw.SubFrm[sat-1][:], nil, raw.NavData.Alm, raw.NavData.Ion_qzs[:],
			raw.NavData.Utc_qzs[:])
		adj_utcweek(raw.Time, raw.NavData.Utc_qzs[:])
	}
	return 0
}

/* decode gps/qzss subframe (0xE0) -------------------------------------------*/
func decode_stqgps(raw *Raw) int {
	var prn, sat, id int
	const idx = 4

	Trace(4, "decode_stqgps: len=%d\n", raw.Len)

	if raw.Len < 40 {
		Trace(2, "stq gps/qzss subframe length error: len=%d\n", raw.Len)
		return -1
	}
	if raw.OutType > 0 {
		copy(raw.MsgType[len(string(raw.MsgType[:])):], []byte(fmt.Sprintf("SKYTRAQ GPSSF (%4d): prn=%2d id=%d", raw.Len,
			U1(raw.Buff[idx+1:]), (raw.Buff[idx:][8]>>2)&0x7)))
	}
	prn = int(U1(raw.Buff[idx+1:]))
	var isys = SYS_GPS
	if MINPRNQZS <= prn && prn <= MAXPRNQZS {
		isys = SYS_QZS
	}
	if sat = SatNo(isys, prn); sat == 0 {
		Trace(2, "stq gps/qzss subframe satellite number error: prn=%d\n", prn)
		return -1
	}
	id = save_subfrm(sat, raw)
	if id == 3 {
		return decode_ephem(sat, raw)
	}
	if id == 4 {
		return decode_alm1(sat, raw)
	}
	if id == 5 {
		return decode_alm2(sat, raw)
	}
	return 0
}

/* decode glonass string (0xE1) ----------------------------------------------*/
func decode_stqglo(raw *Raw) int {
	var (
		geph           GEph
		i, prn, sat, m int
	)
	const idx = 4

	Trace(4, "decode_stqglo: len=%d\n", raw.Len)

	if raw.Len < 19 {
		Trace(2, "stq glo string length error: len=%d\n", raw.Len)
		return -1
	}
	if raw.OutType > 0 {
		copy(raw.MsgType[len(string(raw.MsgType[:])):], []byte(fmt.Sprintf("SKYTRAQ GLSTR (%4d): prn=%2d no=%d", raw.Len,
			U1(raw.Buff[idx+1:])-64, U1(raw.Buff[idx+2:]))))
	}
	prn = int(U1(raw.Buff[idx+1:])) - 64
	if sat = SatNo(SYS_GLO, prn); sat == 0 {
		Trace(2, "stq glo string satellite number error: prn=%d\n", prn)
		return -1
	}
	m = int(U1(raw.Buff[idx+2:])) /* string number */
	if m < 1 || 4 < m {
		return 0 /* non-immediate info and almanac */
	}
	SetBitU(raw.SubFrm[sat-1][(m-1)*10:], 1, 4, uint32(m))
	for i = 0; i < 9; i++ {
		SetBitU(raw.SubFrm[sat-1][(m-1)*10:], 5+i*8, 8, uint32(raw.Buff[idx:][3+i]))
	}
	if m != 4 {
		return 0
	}

	/* decode glonass ephemeris strings */
	geph.Tof = raw.Time
	if Decode_Glostr(raw.SubFrm[sat-1][:], &geph, nil) == 0 || geph.Sat != sat {
		return 0
	}

	if !strings.Contains(raw.Opt, "-EPHALL") {
		if geph.Iode == raw.NavData.Geph[prn-1].Iode {
			return 0
		} /* unchanged */
	}
	/* keep freq channel number */
	geph.Frq = raw.NavData.Geph[prn-1].Frq
	raw.NavData.Geph[prn-1] = geph
	raw.EphSat = sat
	raw.EphSet = 0
	return 2
}

/* decode glonass string (requested) (0x5C) ----------------------------------*/
func decode_stqgloe(raw *Raw) int {
	var prn, sat int
	const idx = 4

	Trace(4, "decode_stqgloe: len=%d\n", raw.Len)

	if raw.Len < 50 {
		Trace(2, "stq glo string length error: len=%d\n", raw.Len)
		return -1
	}
	prn = int(U1(raw.Buff[idx+1:]))
	if sat = SatNo(SYS_GLO, prn); sat == 0 {
		Trace(2, "stq gloe string satellite number error: prn=%d\n", prn)
		return -1
	}
	/* set frequency channel number */
	raw.NavData.Geph[prn-1].Frq = int(I1(raw.Buff[idx+2:]))

	return 0
}

/* decode beidou subframe (0xE2,0xE3) ----------------------------------------*/
func decode_stqbds(raw *Raw) int {
	var (
		eph                     Eph
		word                    uint32
		i, j, id, pgn, prn, sat int
	)
	const idx = 4

	Trace(4, "decode_stqbds: len=%d\n", raw.Len)

	if raw.Len < 38 {
		Trace(2, "stq bds subframe length error: len=%d\n", raw.Len)
		return -1
	}
	if raw.OutType > 0 {
		copy(raw.MsgType[len(string(raw.MsgType[:])):], []byte(fmt.Sprintf("SKYTRAQ BDSSF (%4d): prn=%2d id=%d", raw.Len,
			U1(raw.Buff[idx+1:])-200, U1(raw.Buff[idx+2:]))))
	}
	prn = int(U1(raw.Buff[idx+1:]) - 200)
	if sat = SatNo(SYS_CMP, prn); sat == 0 {
		Trace(2, "stq bds subframe satellite number error: prn=%d\n", prn)
		return -1
	}
	id = int(U1(raw.Buff[idx+2:])) /* subframe id */
	if id < 1 || 5 < id {
		Trace(2, "stq bds subframe id error: prn=%2d\n", prn)
		return -1
	}
	if prn > 5 { /* IGSO/MEO */
		word = GetBitU(raw.Buff[idx+3:], j, 26) << 4
		j += 26
		SetBitU(raw.SubFrm[sat-1][(id-1)*38:], 0, 30, word)

		for i = 1; i < 10; i++ {
			word = GetBitU(raw.Buff[idx+3:], j, 22) << 8
			j += 22
			SetBitU(raw.SubFrm[sat-1][(id-1)*38:], i*30, 30, word)
		}
		if id != 3 {
			return 0
		}
		if DecodeBDSD1(raw.SubFrm[sat-1][:], &eph, nil, nil) == 0 {
			return 0
		}
	} else { /* GEO */
		if id != 1 {
			return 0
		}

		pgn = int(GetBitU(raw.Buff[idx+3:], 26+12, 4)) /* page number */
		if pgn < 1 || 10 < pgn {
			Trace(2, "stq bds subframe page number error: prn=%2d pgn=%d\n", prn, pgn)
			return -1
		}
		word = GetBitU(raw.Buff[idx+3:], j, 26) << 4
		j += 26
		SetBitU(raw.SubFrm[sat-1][(pgn-1)*38:], 0, 30, word)

		for i = 1; i < 10; i++ {
			word = GetBitU(raw.Buff[idx+3:], j, 22) << 8
			j += 22
			SetBitU(raw.SubFrm[sat-1][(pgn-1)*38:], i*30, 30, word)
		}
		if pgn != 10 {
			return 0
		}
		if DecodeBDSD2(raw.SubFrm[sat-1][:], &eph, nil) == 0 {
			return 0
		}
	}
	if !strings.Contains(raw.Opt, "-EPHALL") {
		if TimeDiff(eph.Toe, raw.NavData.Ephs[sat-1].Toe) == 0.0 {
			return 0
		} /* unchanged */
	}
	eph.Sat = sat
	raw.NavData.Ephs[sat-1] = eph
	raw.EphSat = sat
	raw.EphSet = 0
	return 2
}

/* decode ack to request msg (0x83) ------------------------------------------*/
func decode_stqack(raw *Raw) int {
	const idx = 4

	Trace(4, "decode_stqack: len=%d\n", raw.Len)

	if raw.Len < 9 {
		Trace(2, "stq ack length error: len=%d\n", raw.Len)
		return -1
	}
	if raw.OutType > 0 {
		copy(raw.MsgType[len(string(raw.MsgType[:])):], []byte(fmt.Sprintf("SKYTRAQ ACK   (%4d): msg=0x%02X", raw.Len,
			U1(raw.Buff[idx+1:]))))
	}
	return 0
}

/* decode nack to request msg (0x84) -----------------------------------------*/
func decode_stqnack(raw *Raw) int {
	const idx = 4

	Trace(4, "decode_stqnack: len=%d\n", raw.Len)

	if raw.Len < 9 {
		Trace(2, "stq nack length error: len=%d\n", raw.Len)
		return -1
	}
	if raw.OutType > 0 {
		copy(raw.MsgType[len(string(raw.MsgType[:])):], []byte(fmt.Sprintf("SKYTRAQ NACK  (%4d): msg=0x%02X", raw.Len,
			U1(raw.Buff[idx+1:]))))
	}
	return 0
}

/* decode skytraq message ----------------------------------------------------*/
func decode_stq(raw *Raw) int {
	var cs uint8
	idx := raw.Len - 3
	ctype := U1(raw.Buff[4:])

	Trace(3, "decode_stq: type=%02x len=%d\n", ctype, raw.Len)

	/* checksum */
	cs = checksum_stq(raw.Buff[:], raw.Len)

	if cs != raw.Buff[idx] || raw.Buff[idx+1] != 0x0D || raw.Buff[idx+2] != 0x0A {
		Trace(2, "stq checksum error: type=%02X cs=%02X tail=%02X%02X%02X\n",
			ctype, cs, raw.Buff[idx], raw.Buff[idx+1], raw.Buff[idx+2])
		return -1
	}
	if raw.OutType > 0 {
		copy(raw.MsgType[len(string(raw.MsgType[:])):], []byte(fmt.Sprintf("SKYTRAQ 0x%02X  (%4d):", ctype, raw.Len)))
	}

	switch ctype {
	case ID_STQTIME:
		return decode_stqtime(raw)
	case ID_STQRAW:
		return decode_stqraw(raw)
	case ID_STQRAWX:
		return decode_stqrawx(raw)
	case ID_STQGPS:
		return decode_stqgps(raw)
	case ID_STQGLO:
		return decode_stqglo(raw)
	case ID_STQGLOE:
		return decode_stqgloe(raw)
	case ID_STQBDSD1:
		return decode_stqbds(raw)
	case ID_STQBDSD2:
		return decode_stqbds(raw)
	case ID_STQACK:
		return decode_stqack(raw)
	case ID_STQNACK:
		return decode_stqnack(raw)
	}
	ctype = uint8(idx)
	return 0
}

/* sync code -----------------------------------------------------------------*/
func sync_stq(buff []uint8, data uint8) int {
	buff[0] = buff[1]
	buff[1] = data
	if buff[0] == STQSYNC1 && buff[1] == STQSYNC2 {
		return 1
	}
	return 0
}

/* input skytraq raw message from stream ---------------------------------------
* fetch next skytraq raw data and input a mesasge from stream
* args   : raw *Raw       IO  receiver raw data control struct
*          uint8 data     I   stream data (1 byte)
* return : status (-1: error message, 0: no message, 1: input observation data,
*                  2: input ephemeris, 3: input sbas message,
*                  9: input ion/utc parameter)
*
* notes  : to specify input options, set raw.Opt to the following option
*          strings separated by spaces.
*
*          -INVCP     : inverse polarity of carrier-phase
*
*-----------------------------------------------------------------------------*/
func Input_stq(raw *Raw, data uint8) int {
	Trace(5, "input_stq: data=%02x\n", data)

	/* synchronize frame */
	if raw.NumByte == 0 {
		if sync_stq(raw.Buff[:], data) == 0 {
			return 0
		}
		raw.NumByte = 2
		return 0
	}
	raw.Buff[raw.NumByte] = data
	raw.NumByte++

	if raw.NumByte == 4 {
		if raw.Len = int(U2(raw.Buff[2:])) + 7; raw.Len > MAXRAWLEN {
			Trace(2, "stq message length error: len=%d\n", raw.Len)
			raw.NumByte = 0
			return -1
		}
	}
	if raw.NumByte < 4 || raw.NumByte < raw.Len {
		return 0
	}
	raw.NumByte = 0

	/* decode skytraq raw message */
	return decode_stq(raw)
}

/* input skytraq raw message from file -----------------------------------------
* fetch next skytraq raw data and input a message from file
* args   : raw_t  *raw      IO  receiver raw data control struct
*          FILE   *fp       I   file pointer
* return : status(-2: end of file, -1...9: same as above)
*-----------------------------------------------------------------------------*/
func Input_stqf(raw *Raw, fp *os.File) int {
	Trace(4, "input_stqf:\n")
	var i int
	/* synchronize frame */
	if raw.NumByte == 0 {
		var c [1]byte
		/* synchronize frame */
		for i = 0; ; i++ {
			_, err := fp.Read(c[:])
			if err == io.EOF {
				return -2
			}
			if sync_stq(raw.Buff[:], c[0]) > 0 {
				break
			}
			if i >= 4096 {
				return 0
			}
		}
	}
	if n, _ := fp.Read(raw.Buff[2:4]); n < 2 {
		return -2
	}
	raw.NumByte = 4

	if raw.Len = int(U2(raw.Buff[2:])) + 7; raw.Len > MAXRAWLEN {
		Trace(2, "stq message length error: len=%d\n", raw.Len)
		raw.NumByte = 0
		return -1
	}
	if n, _ := fp.Read(raw.Buff[4:raw.Len]); n < raw.Len-4 {
		return -2
	}
	raw.NumByte = 0

	/* decode skytraq raw message */
	return decode_stq(raw)
}

/* generate skytraq binary message ---------------------------------------------
* generate skytraq binary message from message string
* args   : char  *msg       I   message string
*            "RESTART  [arg...]" system restart
*            "CFG-SERI [arg...]" configure serial port propperty
*            "CFG-FMT  [arg...]" configure output message format
*            "CFG-RATE [arg...]" configure binary measurement output rates
*            "CFG-BIN  [arg...]" configure general binary
*            "GET-GLOEPH [slot]" get glonass ephemeris for freq channel number
*          buff []uint8 O binary message
* return : length of binary message (0: error)
* note   : see reference [1][2][3][4] for details.
*-----------------------------------------------------------------------------*/
func gen_stq(msg string, buff []uint8) int {
	var (
		hz         []string = []string{"1Hz", "2Hz", "4Hz", "5Hz", "10Hz", "20Hz"}
		q                   = 0
		i, n, narg int
	)
	Trace(4, "gen_stq: msg=%s\n", msg)

	args := strings.Split(msg, " ")
	if len(args) < 1 {
		return 0
	}
	buff[q] = STQSYNC1
	q++
	buff[q] = STQSYNC2
	q++
	switch args[0] {
	case "RESTART":
		{
			buff[q] = 0
			q++
			buff[q] = 15
			q++
			buff[q] = ID_RESTART
			q++
			if len(args) > 2 {
				n, _ = strconv.Atoi(args[1])
				buff[q] = uint8(n)
			} else {
				buff[q] = 0
			}
			q++
			for i = 1; i < 15; i++ {
				buff[q] = 0
				q++
			} /* set all 0 */
		}
	case "CFG-SERI":
		{
			buff[q] = 0
			q++
			buff[q] = 4
			q++
			buff[q] = ID_CFGSERI
			q++
			for i = 1; i < 4; i++ {
				if len(args) > i+1 {
					n, _ = strconv.Atoi(args[i])
					buff[q] = uint8(n)
				} else {
					buff[q] = 0
				}
				q++
			}
		}
	case "CFG-FMT":
		{
			buff[q] = 0
			q++
			buff[q] = 3
			q++
			buff[q] = ID_CFGFMT
			q++
			for i = 1; i < 3; i++ {
				if len(args) > i+1 {
					n, _ = strconv.Atoi(args[i])
					buff[q] = uint8(n)
				} else {
					buff[q] = 0
				}
				q++
			}
		}
	case "CFG-RATE":
		{
			buff[q] = 0
			q++
			buff[q] = 8
			q++
			buff[q] = ID_CFGRATE
			q++
			if len(args) > 2 {
				for i = 0; i < len(hz); i++ {
					if args[1] == hz[i] {
						break
					}
				}
				if i < len(hz) {
					buff[q] = uint8(i)
				} else {
					n, _ = strconv.Atoi(args[1])
					buff[q] = uint8(n)
					q++
				}
			} else {
				buff[q] = 0
				q++
			}
			for i = 2; i < 8; i++ {
				if len(args) > i+1 {
					n, _ = strconv.Atoi(args[i])
					buff[q] = uint8(n)
				} else {
					buff[q] = 0
				}
				q++
			}
		}
	case "CFG-BIN":
		{
			buff[q] = 0
			q++
			buff[q] = 9
			q++ /* F/W 1.4.32 */
			buff[q] = ID_CFGBIN
			q++
			if narg > 2 {
				for i = 0; i < len(hz); i++ {
					if args[1] == hz[i] {
						break
					}
				}
				if i < len(hz) {
					buff[q] = uint8(i)
				} else {
					n, _ = strconv.Atoi(args[1])
					buff[q] = uint8(n)
					q++
				}
			} else {
				buff[q] = 0
				q++
			}
			for i = 2; i < 9; i++ {
				if len(args) > i+1 {
					n, _ = strconv.Atoi(args[i])
					buff[q] = uint8(n)
				} else {
					buff[q] = 0
				}
				q++
			}
		}
	case "GET-GLOEPH":
		{
			buff[q] = 0
			q++
			buff[q] = 2
			q++
			buff[q] = ID_GETGLOEPH
			q++
			if len(args) > 2 {
				n, _ = strconv.Atoi(args[1])
				buff[q] = uint8(n)
			} else {
				buff[q] = 0
			}
			q++
		}
	default:
		return 0
	}
	n = q
	buff[q] = checksum_stq(buff[:], n+3)
	q++
	buff[q] = 0x0D
	q++
	buff[q] = 0x0A

	Trace(4, "gen_stq: buff=\n")
	Traceb(4, buff, n+3)
	return n + 3
}
