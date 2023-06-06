/*------------------------------------------------------------------------------
* javad.c : javad receiver dependent functions
*
*          Copyright (C) 2011-2020 by T.TAKASU, All rights reserved.
*          Copyright (C) 2011-2022 by Feng Xuebin, All rights reserved.
*
* reference :
*     [1] Javad GNSS, GREIS GNSS Receiver External Interface Specification,
*         Reflects Firmware Version 3.2.0, July 22, 2010
*     [2] Javad navigation systemms, GPS Receiver Interface Language (GRIL)
*         Reference Guide Rev 2.2, Reflects Firmware Version 2.6.0
*     [3] Javad GNSS, User visible changes in the firmware vesion 3.4.0 since
*         version 3.3.x (NEWS_3_4_0.txt)
*     [4] Javad GNSS, GREIS GNSS Receiver External Interface Specification,
*         Reflects Firmware Version 3.4.6, October 9, 2012
*     [5] Javad GNSS, GREIS GNSS Receiver External Interface Specification,
*         Reflects Firmware Version 3.5.4, January 30, 2014
*     [6] Javad GNSS, GREIS GNSS Receiver External Interface Specification,
*         Reflects Firmware Version 3.6.7, August 25, 2016
*     [7] Javad GNSS, GREIS GNSS Receiver External Interface Specification,
*         Reflects Firmware Version 3.7.2, October 11, 2017
*
* version : $Revision:$ $Date:$
* history : 2011/05/27 1.0  new
*           2011/07/07 1.1  fix QZSS IODC-only-update problem
*           2012/07/17 1.2  change GALILEO scale factor for short pseudorange
*           2012/10/18 1.3  change receiver options and rinex obs code
*           2013/01/24 1.4  change compass factor for short pseudorange
*                           add raw option -NOET
*           2013/02/23 1.6  fix memory access violation problem on arm
*           2013/05/08 1.7  fix bug on week number of galileo ephemeris
*           2014/05/23 1.8  support beidou
*           2014/06/23 1.9  support [lD] for glonass raw navigation data
*           2014/08/26 1.10 fix bug on decoding iode in glonass ephemeris [NE]
*           2014/10/20 1.11 fix bug on receiver option -GL*,-RL*,-JL*
*           2016/01/26 1.12 fix problem on bus-error on ARM CPU (#129)
*           2017/04/11 1.13 support IRNSS
*                           fix bug on carrier frequency for beidou
*                           fix bug on unchange-test for beidou ephemeris
*                           update Asys coef for [r*] short pseudorange by [6]
*                           (char *) . (signed char *)
*           2018/10/10 1.14 update signal allocation by ref [7]
*                           update Ksys value for galileo by ref [7]
*                           fix problem to set eph.Code for beidou and galileo
*                           fix bug on saving galileo bgd to ephemeris
*                           add receiver option -GALINAV, -GALFNAV
*           2019/05/10 1.15 save galileo E5b data to obs index 2
*           2020/11/30 1.16 output L1C for GLONASS G1 as default
*                           change receiver option -RL1C . -RL1P
*                           CODE_L1I . CODE_L2I for BDS B1I (RINEX 3.04)
*                           output GAL I/NAV and F/NAV to separated ephem sets
*                           fix bug on decoding SVH in message [NE] for GLONASS
*                           use API code2idx() to get freq-index
*                           use integer types in stdint.h
*           2022/09/20      rewrite the file with golang
*-----------------------------------------------------------------------------*/
package gnssgo

import (
	"fmt"
	"io"
	"math"
	"os"
	"strings"
)

const PREAMB_CNAV = 0x8B

func ISTXT(c uint8) bool {
	return '0' <= (c) && (c) <= '~'
}
func ISHEX(c uint8) bool {
	return ('0' <= (c) && (c) <= '9') || ('A' <= (c) && (c) <= 'F')
}
func ROT_LEFT(val byte) byte {
	return val<<2 | val>>6
}

/* extract field (little-endian) ---------------------------------------------*/
// implemented in cresent.go

// #define U1(raw.Buff[idx:]) (*((uint8_t *)(raw.Buff[idx:])))
// #define I1(raw.Buff[idx:]) (*((int8_t  *)(raw.Buff[idx:])))
// static uint16_t U2L(uint8_t *p) {uint16_t u; memcpy(&u,p,2); return u;}
// static uint32_t U4L(uint8_t *p) {uint32_t u; memcpy(&u,p,4); return u;}
// static int16_t  I2L(uint8_t *p) {int16_t  i; memcpy(&i,p,2); return i;}
// static int32_t  I4L(uint8_t *p) {int32_t  i; memcpy(&i,p,4); return i;}

// static float R4L(uint8_t *p)
// {
//     float value;
//     uint8_t *q=(uint8_t *)&value;
//     int i;
//     if (U4L(raw.Buff[idx:])==0x7FC00000) return 0.0f; /* quiet nan */
//     for (i=0;i<4;i++) *q++=*p++;
//     return value;
// }
// static double R8L(uint8_t *p)
// {
//     double value;
//     uint8_t *q=(uint8_t *)&value;
//     int i;
//     if (U4L(p+4)==0x7FF80000&&U4L(raw.Buff[idx:])==0) return 0.0; /* quiet nan */
//     for (i=0;i<8;i++) *q++=*p++;
//     return value;
// }
/* decode message length -----------------------------------------------------*/
func decodelen(buff []uint8) int {
	var len int
	if !ISHEX(buff[0]) || !ISHEX(buff[1]) || !ISHEX(buff[2]) {
		return 0
	}
	if n, _ := fmt.Sscanf(string(buff), "%3X", &len); n == 1 {
		return len
	}
	return 0
}

/* test measurement data -----------------------------------------------------*/
func is_meas(sig rune) int {
	if sig == 'c' || sig == 'C' || sig == '1' || sig == '2' || sig == '3' || sig == '5' || sig == 'l' {
		return 1
	}
	return 0
}

/* convert signal to freq-index ----------------------------------------------*/
func sig2idx(sys int, sig rune, code *int) int {
	var codes [7][6]uint8 = [7][6]uint8{ /* ref [7] table 3-8 */
		/*  c/C       1        2        3        5        l  */
		/* (CA/L1    P/L1     P/L2    CA/L2      L5      L1C) */
		{CODE_L1C, CODE_L1W, CODE_L2W, CODE_L2X, CODE_L5X, CODE_L1X}, /* GPS */
		{CODE_L1C, CODE_L1Z, CODE_L6X, CODE_L2X, CODE_L5X, CODE_L1X}, /* QZS */
		{CODE_L1C, 0, 0, 0, CODE_L5X, 0},                             /* SBS */
		{CODE_L1X, CODE_L8X, CODE_L7X, CODE_L6X, CODE_L5X, 0},        /* GAL */
		{CODE_L1C, CODE_L1P, CODE_L2P, CODE_L2C, CODE_L3X, 0},        /* GLO */
		{CODE_L2I, 0, CODE_L7I, CODE_L6I, CODE_L5X, CODE_L1X},        /* BDS */
		{0, 0, 0, 0, CODE_L5X, 0},                                    /* IRN */
	}
	var i, j, idx int

	switch sig {
	case 'c':
	case 'C':
		i = 0
	case '1':
		i = 1
	case '2':
		i = 2
	case '3':
		i = 3
	case '5':
		i = 4
	case 'l':
		i = 5
	default:
		return -1
	}
	switch sys {
	case SYS_GPS:
		j = 0
	case SYS_QZS:
		j = 1
	case SYS_SBS:
		j = 2
	case SYS_GAL:
		j = 3
	case SYS_GLO:
		j = 4
	case SYS_CMP:
		j = 5
	case SYS_IRN:
		j = 6
	default:
		return -1
	}
	if *code = int(codes[j][i]); *code == 0 {
		return -1
	}
	idx = Code2Idx(sys, uint8(*code))
	if idx < NFREQ {
		return idx
	} else {
		return -1
	}
}

/* check code priority and return freq-index ---------------------------------*/
func checkpri(sys, code int, opt string, idx int) int {
	nex := NEXOBS /* number of extended obs data */

	switch sys {
	case SYS_GPS:
		{
			if strings.Contains(opt, "-GL1W") && idx == 0 {
				if code == CODE_L1W {
					return 0
				}
				return -1
			}
			if strings.Contains(opt, "-GL1X") && idx == 0 {
				if code == CODE_L1X {
					return 0
				}
				return -1
			}
			if strings.Contains(opt, "-GL2X") && idx == 1 {
				if code == CODE_L2X {
					return 1
				}
				return -1
			}
			if code == CODE_L1W {
				if nex < 1 {
					return -1
				}
				return NFREQ
			}
			if code == CODE_L2X {
				if nex < 2 {
					return -1
				}
				return NFREQ + 1
			}
			if code == CODE_L1X {
				if nex < 3 {
					return -1
				}
				return NFREQ + 2
			}
		}
	case SYS_GLO:
		{
			if strings.Contains(opt, "-RL1P") && idx == 0 {
				if code == CODE_L1P {
					return 0
				}
				return -1
			}
			if strings.Contains(opt, "-RL2C") && idx == 1 {
				if code == CODE_L2C {
					return 1
				}
				return -1
			}
			if code == CODE_L1P {
				if nex < 1 {
					return -1
				}
				return NFREQ
			}
			if code == CODE_L2C {
				if nex < 2 {
					return -1
				}
				return NFREQ + 1
			}
		}
	case SYS_QZS:
		{
			if strings.Contains(opt, "-JL1Z") && idx == 0 {
				if code == CODE_L1Z {
					return 0
				}
				return -1
			}
			if strings.Contains(opt, "-JL1X") && idx == 0 {
				if code == CODE_L1X {
					return 0
				}
				return -1
			}
			if code == CODE_L1Z {
				if nex < 1 {
					return -1
				}
				return NFREQ
			}

			if code == CODE_L1X {
				if nex < 2 {
					return -1
				}
				return NFREQ + 1
			}
		}
	}
	return idx
}

/* checksum ------------------------------------------------------------------*/
func checksum_javad(buff []uint8, len int) int {
	var cs uint8 = 0
	for i := 0; i < len-1; i++ {
		cs = ROT_LEFT(cs) ^ buff[i]
	}
	cs = ROT_LEFT(cs)
	if cs == buff[len-1] {
		return 1
	}
	return 0
}

/* adjust weekly rollover of GPS time ----------------------------------------*/
// func adjweek(Gtime time, double tow)Gtime {
//     double tow_p;
//     int week;
//     tow_p=Time2GpsT(time,&week);
//     if      (tow<tow_p-302400.0) tow+=604800.0;
//     else if (tow>tow_p+302400.0) tow-=604800.0;
//     return GpsT2Time(week,tow);
// }
/* adjust daily rollover of time ---------------------------------------------*/
// implemented in binex.go
// static Gtime adjday(Gtime time, double tod)
// {
//     double ep[6],tod_p;
//     time2epoch(time,ep);
//     tod_p=ep[3]*3600.0+ep[4]*60.0+ep[5];
//     if      (tod<tod_p-43200.0) tod+=86400.0;
//     else if (tod>tod_p+43200.0) tod-=86400.0;
//     ep[3]=ep[4]=ep[5]=0.0;
//     return TimeAdd(Epoch2Time(ep),tod);
// }
/* set time tag --------------------------------------------------------------*/
func settag(data *ObsD, time Gtime) int {
	var s1, s2 string

	if data.Time.Time != 0 && math.Abs(TimeDiff(data.Time, time)) > 5e-4 {
		Time2Str(data.Time, &s1, 4)
		Time2Str(time, &s2, 4)
		Trace(2, "time inconsistent: time=%s %s sat=%2d\n", s1, s2, data.Sat)
		return 0
	}
	data.Time = time
	return 1
}

/* flush observation data buffer ---------------------------------------------*/
func flushobuf(raw *Raw) int {
	var (
		time0   Gtime
		i, j, n int
	)

	Trace(3, "flushobuf: n=%d\n", raw.ObsBuf.N())

	/* copy observation data buffer */
	for i = 0; i < raw.ObsBuf.N() && i < MAXOBS; i++ {
		if SatSys(raw.ObsBuf.Data[i].Sat, nil) == 0 {
			continue
		}
		if raw.ObsBuf.Data[i].Time.Time == 0 {
			continue
		}
		raw.ObsData.Data[n] = raw.ObsBuf.Data[i]
		n++
	}
	raw.ObsData.n = n

	/* clear observation data buffer */
	for i = 0; i < MAXOBS; i++ {
		raw.ObsBuf.Data[i].Time = time0
		for j = 0; j < NFREQ+NEXOBS; j++ {
			raw.ObsBuf.Data[i].L[j], raw.ObsBuf.Data[i].P[j] = 0.0, 0.0
			raw.ObsBuf.Data[i].D[j] = 0.0
			raw.ObsBuf.Data[i].SNR[j], raw.ObsBuf.Data[i].LLI[j] = 0, 0
			raw.ObsBuf.Data[i].Code[j] = CODE_NONE
		}
	}
	for i = 0; i < MAXSAT; i++ {
		raw.PrCA[i], raw.DpCA[i] = 0.0, 0.0
	}
	if n > 0 {
		return 1
	}
	return 0
}

/* decode [~~] receiver time -------------------------------------------------*/
func decode_RT(raw *Raw) int {
	var (
		time Gtime
	)
	const idx int = 5
	if checksum_javad(raw.Buff[:], raw.Len) == 0 {
		Trace(2, "javad RT error: len=%d\n", raw.Len)
		return -1
	}
	if raw.Len < 10 {
		Trace(2, "javad RT length error: len=%d\n", raw.Len)
		return -1
	}
	raw.Tod = int(U4L(raw.Buff[idx:]))

	if raw.Time.Time == 0 {
		return 0
	}

	/* update receiver time */
	time = raw.Time
	if raw.Tbase >= 1 {
		time = GpsT2Utc(time)
	} /* GPST.UTC */
	time = adjday(time, float64(raw.Tod)*0.001)
	if raw.Tbase >= 1 {
		time = Utc2GpsT(time)
	} /* UTC.GPST */
	raw.Time = time

	Trace(3, "decode_RT: time=%s\n", TimeStr(time, 3))

	if raw.OutType > 0 {
		copy(raw.MsgType[len(string(raw.MsgType[:])):], []byte(fmt.Sprintf(" %s", TimeStr(time, 3))))
	}
	/* flush observation data buffer */
	return flushobuf(raw)
}

/* decode [::] epoch time ----------------------------------------------------*/
func decode_ET(raw *Raw) int {
	const idx int = 5

	if checksum_javad(raw.Buff[:], raw.Len) == 0 {
		Trace(2, "javad ET checksum error: len=%d\n", raw.Len)
		return -1
	}
	if raw.Len < 10 {
		Trace(2, "javad ET length error: len=%d\n", raw.Len)
		return -1
	}
	if raw.Tod != int(U4L(raw.Buff[idx:])) {
		Trace(2, "javad ET inconsistent tod: tod=%d %d\n", raw.Tod, U4L(raw.Buff[idx:]))
		return -1
	}
	raw.Tod = -1 /* end of epoch */

	/* flush observation data buffer */
	return flushobuf(raw)
}

/* decode [RD] receiver date -------------------------------------------------*/
func decode_RD(raw *Raw) int {
	var (
		ep  [6]float64
		idx int = 5
	)
	if checksum_javad(raw.Buff[:], raw.Len) == 9 {
		Trace(2, "javad RD checksum error: len=%d\n", raw.Len)
		return -1
	}
	if raw.Len < 11 {
		Trace(2, "javad RD length error: len=%d\n", raw.Len)
		return -1
	}
	ep[0] = float64(U2L(raw.Buff[idx:]))
	idx += 2
	ep[1] = float64(U1(raw.Buff[idx:]))
	idx += 1
	ep[2] = float64(U1(raw.Buff[idx:]))
	idx += 1
	raw.Tbase = int(U1(raw.Buff[idx:]))

	if raw.OutType > 0 {
		copy(raw.MsgType[len(string(raw.MsgType[:])):], []byte(fmt.Sprintf(" %04.0f/%02.0f/%02.0f base=%d", ep[0], ep[1], ep[2], raw.Tbase)))
	}
	if raw.Tod < 0 {
		Trace(2, "javad RD lack of preceding RT\n")
		return 0
	}
	raw.Time = TimeAdd(Epoch2Time(ep[:]), float64(raw.Tod)*0.001)
	if raw.Tbase >= 1 {
		raw.Time = Utc2GpsT(raw.Time)
	} /* UTC.GPST */

	Trace(3, "decode_RD: time=%s\n", TimeStr(raw.Time, 3))

	return 0
}

/* decode [SI] satellite indices ---------------------------------------------*/
func decode_SI(raw *Raw) int {
	var (
		i, usi, sat int
		idx         int = 5
	)

	if checksum_javad(raw.Buff[:], raw.Len) == 0 {
		Trace(2, "javad SI checksum error: len=%d\n", raw.Len)
		return -1
	}

	raw.ObsBuf.n = raw.Len - 6
	for i = 0; i < raw.ObsBuf.N() && i < MAXOBS; i++ {
		usi = int(U1(raw.Buff[idx:]))
		idx += 1

		switch {
		case usi <= 0:
			sat = 0 /* ref [5] table 3-6 */
		case usi <= 37:
			sat = SatNo(SYS_GPS, usi) /*   1- 37: GPS */
		case usi <= 70:
			sat = 255 /*  38- 70: GLONASS */
		case usi <= 119:
			sat = SatNo(SYS_GAL, usi-70) /*  71-119: GALILEO */
		case usi <= 142:
			sat = SatNo(SYS_SBS, usi) /* 120-142: SBAS */
		case usi <= 192:
			sat = 0
		case usi <= 197:
			sat = SatNo(SYS_QZS, usi) /* 193-197: QZSS */
		case usi <= 210:
			sat = 0
		case usi <= 240:
			sat = SatNo(SYS_CMP, usi-210) /* 211-240: BeiDou */
		case usi <= 247:
			sat = SatNo(SYS_IRN, usi-240) /* 241-247: IRNSS */
		default:
			sat = 0
		}
		raw.ObsBuf.Data[i].Time = raw.Time
		raw.ObsBuf.Data[i].Sat = sat

		/* glonass fcn (frequency channel number) */
		if sat == 255 {
			raw.FreqNum[i] = byte(usi - 45)
		}
	}
	Trace(4, "decode_SI: nsat=raw.ObsBuf_N\n")

	if raw.OutType > 0 {
		copy(raw.MsgType[len(string(raw.MsgType[:])):], []byte(fmt.Sprintf(" nsat=%2d", raw.ObsBuf.N())))
	}
	return 0
}

/* decode [NN] GLONASS satellite system numbers ------------------------------*/
func decode_NN(raw *Raw) int {
	var (
		idx                 int = 5
		i, n, ns, slot, sat int
		index               [MAXOBS]int
	)
	if checksum_javad(raw.Buff[:], raw.Len) == 0 {
		Trace(2, "javad NN checksum error: len=%d\n", raw.Len)
		return -1
	}
	for i, n = 0, 0; i < raw.ObsBuf.N() && i < MAXOBS; i++ {
		if raw.ObsBuf.Data[i].Sat == 255 {
			index[n] = i
			n++
		}
	}
	ns = raw.Len - 6

	for i = 0; i < ns && i < n; i++ {
		slot = int(U1(raw.Buff[idx:]))
		idx += 1
		sat = SatNo(SYS_GLO, slot)
		raw.ObsBuf.Data[index[i]].Sat = sat
	}
	if raw.OutType > 0 {
		copy(raw.MsgType[len(string(raw.MsgType[:])):], []byte(fmt.Sprintf(" nsat=%2d", ns)))
	}
	return 0
}

/* decode [GA] GPS almanac ---------------------------------------------------*/
func decode_GA(raw *Raw) int {
	Trace(3, "javad GA unsupported\n")

	return 0
}

/* decode [NA] GLONASS almanac -----------------------------------------------*/
func decode_NA(raw *Raw) int {
	Trace(3, "javad NA unsupported\n")

	return 0
}

/* decode [EA] Galileo almanac -----------------------------------------------*/
func decode_EA(raw *Raw) int {
	Trace(3, "javad EA unsupported\n")

	return 0
}

/* decode [WA] WAAS almanac --------------------------------------------------*/
func decode_WA(raw *Raw) int {
	Trace(3, "javad WA unsupported\n")

	return 0
}

/* decode [QA] QZSS almanac --------------------------------------------------*/
func decode_QA(raw *Raw) int {
	Trace(3, "javad QA unsupported\n")

	return 0
}

/* decode [CA] Beidou almanac ------------------------------------------------*/
func decode_CA(raw *Raw) int {
	Trace(3, "javad CA unsupported\n")

	return 0
}

/* decode [IA] IRNSS almanac -------------------------------------------------*/
func decode_IA(raw *Raw) int {
	Trace(3, "javad IA unsupported\n")

	return 0
}

/* decode GPS/Galileo/QZSS/Beidou ephemeris ----------------------------------*/
func decode_eph(raw *Raw, sys int) int {
	var (
		eph                                              Eph
		toc, sqrtA, tt                                   float64
		sat, prn, tow, flag, week, navtype, sigtype, set int
		eph_sel                                          int = 3 /* Galileo ephemeris selection */
		idx                                              int = 5
	)
	Trace(3, "decode_eph: sys=%2d prn=%3d\n", sys, U1(raw.Buff[idx:]))

	if strings.Contains(raw.Opt, "-GALINAV") {
		eph_sel = 1
	}
	if strings.Contains(raw.Opt, "-GALFNAV") {
		eph_sel = 2
	}

	prn = int(U1(raw.Buff[idx:]))
	idx += 1
	tow = int(U4L(raw.Buff[idx:]))
	idx += 4
	flag = int(U1(raw.Buff[idx:]))
	idx += 1
	eph.Iodc = int(I2L(raw.Buff[idx:]))
	idx += 2
	toc = float64(I4L(raw.Buff[idx:]))
	idx += 4
	eph.Sva = int(I1(raw.Buff[idx:]))
	idx += 1
	eph.Svh = int(U1(raw.Buff[idx:]))
	idx += 1
	week = int(I2L(raw.Buff[idx:]))
	idx += 2
	eph.Tgd[0] = float64(R4L(raw.Buff[idx:]))
	idx += 4
	eph.F2 = float64(R4L(raw.Buff[idx:]))
	idx += 4
	eph.F1 = float64(R4L(raw.Buff[idx:]))
	idx += 4
	eph.F0 = float64(R4L(raw.Buff[idx:]))
	idx += 4
	eph.Toes = float64(I4L(raw.Buff[idx:]))
	idx += 4
	eph.Iode = int(I2L(raw.Buff[idx:]))
	idx += 2
	sqrtA = R8L(raw.Buff[idx:])
	idx += 8
	eph.E = R8L(raw.Buff[idx:])
	idx += 8
	eph.M0 = R8L(raw.Buff[idx:]) * SC2RAD
	idx += 8
	eph.OMG0 = R8L(raw.Buff[idx:]) * SC2RAD
	idx += 8
	eph.I0 = R8L(raw.Buff[idx:]) * SC2RAD
	idx += 8
	eph.Omg = R8L(raw.Buff[idx:]) * SC2RAD
	idx += 8
	eph.Deln = float64(R4L(raw.Buff[idx:])) * SC2RAD
	idx += 4
	eph.OMGd = float64(R4L(raw.Buff[idx:])) * SC2RAD
	idx += 4
	eph.Idot = float64(R4L(raw.Buff[idx:])) * SC2RAD
	idx += 4
	eph.Crc = float64(R4L(raw.Buff[idx:]))
	idx += 4
	eph.Crs = float64(R4L(raw.Buff[idx:]))
	idx += 4
	eph.Cuc = float64(R4L(raw.Buff[idx:]))
	idx += 4
	eph.Cus = float64(R4L(raw.Buff[idx:]))
	idx += 4
	eph.Cic = float64(R4L(raw.Buff[idx:]))
	idx += 4
	eph.Cis = float64(R4L(raw.Buff[idx:]))
	idx += 4
	eph.A = sqrtA * sqrtA

	if raw.OutType > 0 {
		copy(raw.MsgType[len(string(raw.MsgType[:])):], []byte(fmt.Sprintf(" prn=%3d iode=%3d iodc=%3d toes=%6.0f", prn, eph.Iode,
			eph.Iodc, eph.Toes)))
	}
	if sys == SYS_GPS || sys == SYS_QZS || sys == SYS_IRN {
		if sat = SatNo(sys, prn); sat == 0 {
			Trace(2, "javad ephemeris satellite error: sys=%d prn=%d\n", sys, prn)
			return -1
		}
		eph.Flag = (flag >> 1) & 1
		eph.Code = (flag >> 2) & 3
		eph.Fit = float64(flag & 1)
		eph.Week = AdjGpsWeek(week)
		eph.Toe = GpsT2Time(eph.Week, eph.Toes)

		/* for week-handover problem */
		tt = TimeDiff(eph.Toe, raw.Time)
		if tt < -302400.0 {
			eph.Week++
		} else if tt > 302400.0 {
			eph.Week--
		}
		eph.Toe = GpsT2Time(eph.Week, eph.Toes)
		eph.Toc = GpsT2Time(eph.Week, toc)
		eph.Ttr = adjweek(eph.Toe, float64(tow))
	} else if sys == SYS_GAL {
		if sat = SatNo(sys, prn); sat == 0 {
			Trace(2, "javad ephemeris satellite error: sys=%d prn=%d\n", sys, prn)
			return -1
		}
		eph.Tgd[0] = float64(R4L(raw.Buff[idx:]))
		idx += 4 /* BGD: E1-E5A (s) */
		eph.Tgd[1] = float64(R4L(raw.Buff[idx:]))
		idx += 4 + 13                     /* BGD: E1-E5B (s) */
		navtype = int(U1(raw.Buff[idx:])) /* navtype: 0:E1B(INAV),1:E5A(FNAV) */
		/*          3:GIOVE E1B,4:GIOVE E5A */

		if navtype == 1 {
			set = 1
		} /* 0:I/NAV,1:F/NAV */
		if (eph_sel&1 == 0) && set == 0 {
			return 0
		}
		if (eph_sel&2 == 0) && set == 1 {
			return 0
		}
		if set > 0 {
			eph.Code = (1 << 1) + (1 << 8)

		} else {
			eph.Code = (1 << 0) + (1 << 2) + (1 << 9)
		}

		/* gal-week = gst-week + 1024 */
		eph.Week = week + 1024
		eph.Toe = GpsT2Time(eph.Week, eph.Toes)

		/* for week-handover problem */
		tt = TimeDiff(eph.Toe, raw.Time)
		if tt < -302400.0 {
			eph.Week++
		} else if tt > 302400.0 {
			eph.Week--
		}
		eph.Toe = GpsT2Time(eph.Week, eph.Toes)

		eph.Toc = GpsT2Time(eph.Week, toc)
		eph.Ttr = adjweek(eph.Toe, float64(tow))
	} else if sys == SYS_CMP {
		if sat = SatNo(sys, prn); sat == 0 {
			Trace(2, "javad ephemeris satellite error: sys=%d prn=%d\n", sys, prn)
			return -1
		}
		eph.Tgd[1] = float64(R4L(raw.Buff[idx:]))
		idx += 4                          /* TGD2 (s) */
		sigtype = int(U1(raw.Buff[idx:])) /* signal type: 0:B1,1:B2,2:B3 */
		switch sigtype {
		case 0:
			eph.Code = 1
		case 1:
			eph.Code = 3
		case 2:
			eph.Code = 5
		default:
			eph.Code = 0
		}

		eph.Week = week
		eph.Toe = BDT2Time(week, eph.Toes) /* BDT . GPST */
		eph.Toc = BDT2Time(week, toc)      /* BDT . GPST */
		eph.Ttr = adjweek(eph.Toe, float64(tow))
	} else {
		return 0
	}

	if !strings.Contains(raw.Opt, "-EPHALL") {
		if TimeDiff(raw.NavData.Ephs[sat-1+MAXSAT*set].Toe, eph.Toe) == 0.0 &&
			raw.NavData.Ephs[sat-1+MAXSAT*set].Iode == eph.Iode &&
			raw.NavData.Ephs[sat-1+MAXSAT*set].Iodc == eph.Iodc {
			return 0
		} /* unchanged */
	}
	eph.Sat = sat
	raw.NavData.Ephs[sat-1] = eph
	raw.EphSat = sat
	raw.EphSet = set
	return 2
}

/* decode [GE] GPS ephemeris -------------------------------------------------*/
func decode_GE(raw *Raw) int {
	if checksum_javad(raw.Buff[:], raw.Len) == 0 {
		Trace(2, "javad GE checksum error: len=%d\n", raw.Len)
		return -1
	}
	if raw.Len < 128 {
		Trace(2, "javad GE length error: len=%d\n", raw.Len)
		return -1
	}
	return decode_eph(raw, SYS_GPS)
}

/* decode [NE] GLONASS ephemeris ---------------------------------------------*/
func decode_NE(raw *Raw) int {
	var (
		geph        GEph
		tt          float64
		prn, tk, tb int
		idx         int = 5
	)
	if checksum_javad(raw.Buff[:], raw.Len) == 0 {
		Trace(2, "javad NE checksum error: len=%d\n", raw.Len)
		return -1
	}
	if raw.Len >= 85 { /* firmware v 2.6.0 [2] */
		prn = int(U1(raw.Buff[idx:]))
		idx += 1
		geph.Frq = int(I1(raw.Buff[idx:]))
		idx += 1 + 2
		tk = int(I4L(raw.Buff[idx:]))
		idx += 4
		tb = int(I4L(raw.Buff[idx:]))
		idx += 4
		geph.Svh = int(U1(raw.Buff[idx:])) & 0x1
		idx += 1 /* MSB of Bn */
		geph.Age = int(U1(raw.Buff[idx:]))
		idx += 1 + 1
		geph.Pos[0] = R8L(raw.Buff[idx:]) * 1e3
		idx += 8
		geph.Pos[1] = R8L(raw.Buff[idx:]) * 1e3
		idx += 8
		geph.Pos[2] = R8L(raw.Buff[idx:]) * 1e3
		idx += 8
		geph.Vel[0] = float64(R4L(raw.Buff[idx:])) * 1e3
		idx += 4
		geph.Vel[1] = float64(R4L(raw.Buff[idx:])) * 1e3
		idx += 4
		geph.Vel[2] = float64(R4L(raw.Buff[idx:])) * 1e3
		idx += 4
		geph.Acc[0] = float64(R4L(raw.Buff[idx:])) * 1e3
		idx += 4
		geph.Acc[1] = float64(R4L(raw.Buff[idx:])) * 1e3
		idx += 4
		geph.Acc[2] = float64(R4L(raw.Buff[idx:])) * 1e3
		idx += 4 + 8
		geph.Taun = float64(R4L(raw.Buff[idx:]))
		idx += 4
		geph.Gamn = float64(R4L(raw.Buff[idx:]))
		idx += 4
	} else {
		Trace(2, "javad NE length error: len=%d\n", raw.Len)
		return -1
	}
	if raw.Len >= 93 { /* firmware v 3.2.0 [1] */
		geph.DTaun = float64(R4L(raw.Buff[idx:]))
		idx += 4
		geph.Sva = int(U1(raw.Buff[idx:]))
	}
	if raw.OutType > 0 {
		copy(raw.MsgType[len(string(raw.MsgType[:])):], []byte(fmt.Sprintf(" prn=%2d frq=%2d tk=%6d tb=%4d", prn, geph.Frq, tk, tb)))
	}
	if geph.Sat = SatNo(SYS_GLO, prn); geph.Sat == 0 {
		Trace(2, "javad NE satellite error: prn=%d\n", prn)
		return 0
	}
	if raw.Time.Time == 0 {
		return 0
	}
	geph.Iode = (tb / 900) & 0x7F
	geph.Toe = Utc2GpsT(adjday(raw.Time, float64(tb)-10800.0))
	geph.Tof = Utc2GpsT(adjday(raw.Time, float64(tk)-10800.0))

	/* check illegal ephemeris by toe */
	tt = TimeDiff(raw.Time, geph.Toe)
	if math.Abs(tt) > 3600.0 {
		Trace(3, "javad NE illegal toe: prn=%2d tt=%6.0f\n", prn, tt)
		return 0
	}
	/* check illegal ephemeris by frequency number consistency */
	if raw.NavData.Geph[prn-1].Toe.Time > 0 && geph.Frq != raw.NavData.Geph[prn-1].Frq {
		Trace(2, "javad NE glonass fcn changed: prn=%2d fcn=%2d.%2d\n", prn,
			raw.NavData.Geph[prn-1].Frq, geph.Frq)
		return -1
	}
	if !strings.Contains(raw.Opt, "-EPHALL") {
		if math.Abs(TimeDiff(geph.Toe, raw.NavData.Geph[prn-1].Toe)) < 1.0 &&
			geph.Svh == raw.NavData.Geph[prn-1].Svh {
			return 0
		} /* unchanged */
	}
	raw.NavData.Geph[prn-1] = geph
	raw.EphSat = geph.Sat
	return 2
}

/* decode [EN] Galileo ephemeris ---------------------------------------------*/
func decode_EN(raw *Raw) int {
	if checksum_javad(raw.Buff[:], raw.Len) == 0 {
		Trace(2, "javad EN checksum error: len=%d\n", raw.Len)
		return -1
	}
	if raw.Len < 150 {
		Trace(2, "javad EN length error: len=%d\n", raw.Len)
		return -1
	}
	return decode_eph(raw, SYS_GAL)
}

/* decode [WE] SBAS ephemeris ------------------------------------------------*/
func decode_WE(raw *Raw) int {
	var (
		seph         SEph
		tod, tow     uint32
		i, prn, week int
		idx          int = 5
	)
	if checksum_javad(raw.Buff[:], raw.Len) == 0 {
		Trace(2, "javad WE checksum error: len=%d\n", raw.Len)
		return -1
	}
	if raw.Len < 44 {
		Trace(2, "javad WE length error: len=%d\n", raw.Len)
		return -1
	}
	prn = int(U1(raw.Buff[idx:]))
	idx += 1 + 1 + 1
	seph.Sva = int(U1(raw.Buff[idx:]))
	idx += 1
	tod = U4L(raw.Buff[idx:])
	idx += 4
	for i = 0; i < 3; i++ {
		seph.Pos[i] = R8L(raw.Buff[idx:])
		idx += 8
	}
	for i = 0; i < 3; i++ {
		seph.Vel[i] = float64(R4L(raw.Buff[idx:]))
		idx += 4
	}
	for i = 0; i < 3; i++ {
		seph.Acc[i] = float64(R4L(raw.Buff[idx:]))
		idx += 4
	}
	seph.Af0 = float64(R4L(raw.Buff[idx:]))
	idx += 4
	seph.Af1 = float64(R4L(raw.Buff[idx:]))
	idx += 4
	tow = U4L(raw.Buff[idx:])
	idx += 4
	week = int(U2L(raw.Buff[idx:]))

	if raw.OutType > 0 {
		copy(raw.MsgType[len(string(raw.MsgType[:])):], []byte(fmt.Sprintf(" prn=%3d tod=%6d", prn, tod)))
	}
	if seph.Sat = SatNo(SYS_SBS, prn); seph.Sat == 0 {
		Trace(2, "javad WE satellite error: prn=%d\n", prn)
		return -1
	}
	seph.Tof = GpsT2Time(AdjGpsWeek(week), float64(tow))
	seph.T0 = adjday(seph.Tof, float64(tod))

	if !strings.Contains(raw.Opt, "-EPHALL") {
		if math.Abs(TimeDiff(seph.T0, raw.NavData.Seph[prn-MINPRNSBS].T0)) < 1.0 &&
			seph.Sva == raw.NavData.Seph[prn-MINPRNSBS].Sva {
			return 0
		} /* unchanged */
	}
	raw.NavData.Seph[prn-MINPRNSBS] = seph
	raw.EphSat = seph.Sat
	return 2
}

/* decode [QE] QZSS ephemeris ------------------------------------------------*/
func decode_QE(raw *Raw) int {
	if checksum_javad(raw.Buff[:], raw.Len) == 0 {
		Trace(2, "javad QE checksum error: len=%d\n", raw.Len)
		return -1
	}
	if raw.Len < 128 {
		Trace(2, "javad QE length error: len=%d\n", raw.Len)
		return -1
	}
	return decode_eph(raw, SYS_QZS)
}

/* decode [CN] Beidou ephemeris ----------------------------------------------*/
func decode_CN(raw *Raw) int {
	if checksum_javad(raw.Buff[:], raw.Len) == 0 {
		Trace(2, "javad CN checksum error: len=%d\n", raw.Len)
		return -1
	}
	if raw.Len < 133 {
		Trace(2, "javad QE length error: len=%d\n", raw.Len)
		return -1
	}
	return decode_eph(raw, SYS_CMP)
}

/* decode [IE] IRNSS ephemeris -----------------------------------------------*/
func decode_IE(raw *Raw) int {
	if checksum_javad(raw.Buff[:], raw.Len) == 0 {
		Trace(2, "javad IE checksum error: len=%d\n", raw.Len)
		return -1
	}
	if raw.Len < 129 {
		Trace(2, "javad IE length error: len=%d\n", raw.Len)
		return -1
	}
	return decode_eph(raw, SYS_IRN)
}

/* decode [UO] GPS UTC time parameters ---------------------------------------*/
func decode_UO(raw *Raw) int {
	var idx int = 5

	if checksum_javad(raw.Buff[:], raw.Len) == 0 {
		Trace(2, "javad UO checksum error: len=%d\n", raw.Len)
		return -1
	}
	if raw.Len < 29 {
		Trace(2, "javad UO length error: len=%d\n", raw.Len)
		return -1
	}
	raw.NavData.Utc_gps[0] = R8L(raw.Buff[idx:])
	idx += 8
	raw.NavData.Utc_gps[1] = float64(R4L(raw.Buff[idx:]))
	idx += 4
	raw.NavData.Utc_gps[2] = float64(U4L(raw.Buff[idx:]))
	idx += 4
	raw.NavData.Utc_gps[3] = float64(AdjGpsWeek(int(U2L(raw.Buff[idx:]))))
	idx += 2
	raw.NavData.Utc_gps[4] = float64(I1(raw.Buff[idx:]))
	return 9
}

/* decode [NU] GLONASS UTC and GPS time parameters ---------------------------*/
func decode_NU(raw *Raw) int {
	Trace(3, "javad NU unsupported\n")

	return 0
}

/* decode [EU] Galileo UTC and GPS time parameters ---------------------------*/
func decode_EU(raw *Raw) int {
	Trace(3, "javad EU unsupported\n")

	return 0
}

/* decode [WU] WAAS UTC time parameters --------------------------------------*/
func decode_WU(raw *Raw) int {
	Trace(3, "javad WU unsupported\n")

	return 0
}

/* decode [QU] QZSS UTC and GPS time parameters ------------------------------*/
func decode_QU(raw *Raw) int {
	Trace(3, "javad QU unsupported\n")

	return 0
}

/* decode [IO] ionospheric parameters ----------------------------------------*/
func decode_IO(raw *Raw) int {
	var i int
	var idx = 5

	if checksum_javad(raw.Buff[:], raw.Len) == 0 {
		Trace(2, "javad IO checksum error: len=%d\n", raw.Len)
		return -1
	}
	if raw.Len < 44 {
		Trace(2, "javad IO length error: len=%d\n", raw.Len)
		return -1
	}
	idx += 4 + 2
	for i = 0; i < 8; i++ {
		raw.NavData.Ion_gps[i] = float64(R4L(raw.Buff[idx:]))
		idx += 4
	}
	return 9
}

/* decode L1 ephemeris -------------------------------------------------------*/
func decode_L1eph(sat int, raw *Raw) int {
	var eph Eph

	if DecodeFrame(raw.SubFrm[sat-1][:], &eph, nil, nil, nil) == 0 {
		return 0
	}

	if !strings.Contains(raw.Opt, "-EPHALL") {
		if eph.Iode == raw.NavData.Ephs[sat-1].Iode &&
			eph.Iodc == raw.NavData.Ephs[sat-1].Iodc {
			return 0
		}
	}
	eph.Sat = sat
	raw.NavData.Ephs[sat-1] = eph
	raw.EphSat = sat
	raw.EphSet = 0
	return 2
}

/* UTC 8-bit week . full week -----------------------------------------------*/
func adj_utcweek(time Gtime, utc []float64) {
	var week int

	Time2GpsT(time, &week)
	utc[3] += float64(week / 256 * 256)
	if utc[3] < float64(week-127) {
		utc[3] += 256.0
	} else if utc[3] > float64(week+127) {
		utc[3] -= 256.0
	}
	utc[5] += utc[3] / 256 * 256
	if utc[5] < utc[3]-127 {
		utc[5] += 256.0
	} else if utc[5] > utc[3]+127 {
		utc[5] -= 256.0
	}
}

/* decode L1 ION/UTC parameters ----------------------------------------------*/
func decode_L1ionutc(sat int, raw *Raw) int {
	var ion, utc [8]float64
	sys := SatSys(sat, nil)

	if DecodeFrame(raw.SubFrm[sat-1][:], nil, nil, ion[:], utc[:]) == 0 {
		return 0
	}

	adj_utcweek(raw.Time, utc[:])
	if sys == SYS_QZS {
		MatCpy(raw.NavData.Ion_qzs[:], ion[:], 8, 1)
		MatCpy(raw.NavData.Utc_qzs[:], utc[:], 8, 1)
	} else {
		MatCpy(raw.NavData.Ion_gps[:], ion[:], 8, 1)
		MatCpy(raw.NavData.Utc_gps[:], utc[:], 8, 1)
	}
	return 9
}

/* decode L1 NAV data --------------------------------------------------------*/
func decode_L1nav(buff []uint8, len, sat int, raw *Raw) int {
	var (
		subfrm     [30]uint8
		i, id, idx int
	)
	sys := SatSys(sat, nil)

	if sys != SYS_GPS && sys != SYS_QZS {
		Trace(2, "navigation subframe system error: sat=%d\n", sat)
		return -1
	}
	if len < 10 {
		Trace(2, "navigation subframe length error: len=%d\n", len)
		return -1
	}
	for i = 0; i < 10; i, idx = i+1, idx+4 {
		SetBitU(subfrm[:], 24*i, 24, U4L(raw.Buff[idx:])>>6)
	}
	id = int(GetBitU(subfrm[:], 43, 3))
	if id < 1 || id > 5 {
		Trace(2, "navigation subframe format error: sat=%d id=%d\n", sat, id)
		return -1
	}
	copy(raw.SubFrm[sat-1][(id-1)*30:], subfrm[:])

	if id == 3 {
		return decode_L1eph(sat, raw)
	} else if id == 4 || id == 5 {
		return decode_L1ionutc(sat, raw)
	}
	return 0
}

/* decode raw L2C CNAV data --------------------------------------------------*/
func decode_L2nav(buff []uint8, len, sat int, raw *Raw) int {
	var (
		msg                                  [1024]uint8
		i, j, preamb, prn, msgid, tow, alert int
	)
	Trace(3, "decode_L2nav len=%2d sat=%2d L5 CNAV\n", len, sat)

	for i = 0; i < len; i++ {
		for j = 0; j < 4; j++ {
			msg[3-j+i*4] = buff[j+i*4]
		}
	}
	i = 0
	preamb = int(GetBitU(msg[:], i, 8))
	i += 8
	prn = int(GetBitU(msg[:], i, 6))
	i += 6
	msgid = int(GetBitU(msg[:], i, 6))
	i += 6
	tow = int(GetBitU(msg[:], i, 17))
	i += 17
	alert = int(GetBitU(msg[:], i, 1))
	i += 1

	if preamb != PREAMB_CNAV {
		Trace(2, "javad *d sat=%2d L2 CNAV preamble error preamb=%02X\n", preamb)
		return -1
	}
	Trace(3, "L2CNAV: sat=%2d prn=%2d msgid=%2d tow=%6d alert=%d\n", sat, prn,
		msgid, tow, alert)

	return 0
}

/* decode raw L5 CNAV data ---------------------------------------------------*/
func decode_L5nav(buff []uint8, len, sat int, raw *Raw) int {
	var (
		msg                                  [1024]uint8
		i, j, preamb, prn, msgid, tow, alert int
	)
	Trace(3, "decode_L5nav len=%2d sat=%2d L5 CNAV\n", len, sat)

	for i = 0; i < len; i++ {
		for j = 0; j < 4; j++ {
			msg[3-j+i*4] = buff[j+i*4]
		}
	}
	i = 0
	preamb = int(GetBitU(msg[:], i, 8))
	i += 8
	prn = int(GetBitU(msg[:], i, 6))
	i += 6
	msgid = int(GetBitU(msg[:], i, 6))
	i += 6
	tow = int(GetBitU(msg[:], i, 17))
	i += 17
	alert = int(GetBitU(msg[:], i, 1))
	i += 1

	if preamb != PREAMB_CNAV {
		Trace(2, "javad *d sat=%2d L5 CNAV preamble error preamb=%02X\n", preamb)
		return -1
	}
	Trace(3, "L5CNAV: sat=%2d prn=%2d msgid=%2d tow=%6d alert=%d\n", sat, prn,
		msgid, tow, alert)

	return 0
}

/* decode raw L1C CNAV2 data -------------------------------------------------*/
func decode_L1Cnav(buff []uint8, len, sat int, raw *Raw) int {
	Trace(3, "javad *d len=%2d sat=%2d L1C CNAV2 unsupported\n", len, sat)

	return 0
}

/* decode [*D] raw navigation data -------------------------------------------*/
func decode_nD(raw *Raw, sys int) int {
	var i, n, siz, sat, prn, stat int
	var idx = 5

	if checksum_javad(raw.Buff[:], raw.Len) == 0 {
		Trace(2, "javad nD checksum error: sys=%d len=%d\n", sys, raw.Len)
		return -1
	}
	siz = int(U1(raw.Buff[idx:]))
	idx += 1
	n = (raw.Len - 7) / siz

	if n <= 0 {
		Trace(2, "javad nD length error: sys=%d len=%d\n", sys, raw.Len)
		return -1
	}
	for i = 0; i < n; i, idx = i+1, idx+siz {
		Trace(3, "decode_*D: sys=%2d prn=%3d\n", sys, U1(raw.Buff[idx:]))

		prn = int(U1(raw.Buff[idx:]))
		if sat = SatNo(sys, prn); sat == 0 {
			Trace(2, "javad nD satellite error: sys=%d prn=%d\n", sys, prn)
			continue
		}
		stat = decode_L1nav(raw.Buff[idx+2:], 0, sat, raw)
	}
	return stat
}

/* decode [*d] raw navigation data -------------------------------------------*/
func decode_nd(raw *Raw, sys int) int {
	var idx = 5
	var sat, prn, time, ctype, length int

	if checksum_javad(raw.Buff[:], raw.Len) == 0 {
		Trace(2, "javad nd checksum error: sys=%d len=%d\n", sys, raw.Len)
		return -1
	}
	Trace(3, "decode_*d: sys=%2d prn=%3d\n", sys, U1(raw.Buff[idx:]))

	prn = int(U1(raw.Buff[idx:]))
	idx += 1
	time = int(U4L(raw.Buff[idx:]))
	idx += 4
	ctype = int(U1(raw.Buff[idx:]))
	idx += 1
	length = int(U1(raw.Buff[idx:]))
	idx += 1
	if raw.Len != 13+length*4 {
		Trace(2, "javad nd length error: sys=%d len=%d\n", sys, raw.Len)
		return -1
	}
	if raw.OutType > 0 {

		copy(raw.MsgType[len(string(raw.MsgType[:])):], []byte(fmt.Sprintf(" prn=%3d time=%7d type=%d", prn, time, ctype)))
	}
	if sat = SatNo(sys, prn); sat == 0 {
		Trace(2, "javad nd satellite error: sys=%d prn=%d\n", sys, prn)
		return 0
	}
	Trace(4, "sat=%2d time=%7d type=%d len=%3d\n", sat, time, ctype, length)

	switch ctype {
	case 0:
		return decode_L1nav(raw.Buff[idx:], length, sat, raw) /* L1  NAV */
	case 1:
		return decode_L2nav(raw.Buff[idx:], length, sat, raw) /* L2C CNAV */
	case 2:
		return decode_L5nav(raw.Buff[idx:], length, sat, raw) /* L5  CNAV */
	case 3:
		return decode_L1Cnav(raw.Buff[idx:], length, sat, raw) /* L1C CNAV2 */
	}
	return 0
}

/* decode [LD] GLONASS raw navigation data -----------------------------------*/
func decode_LD(raw *Raw) int {
	Trace(3, "javad LD unsupported\n")

	return 0
}

/* decode [lD] GLONASS raw navigation data -----------------------------------*/
func decode_lD(raw *Raw) int {
	var (
		geph GEph
		idx  = 5

		i, sat, prn, frq, time, ctype, length, id int
	)

	if checksum_javad(raw.Buff[:], raw.Len) == 0 {
		Trace(2, "javad lD checksum error: len=%d\n", raw.Len)
		return -1
	}
	Trace(3, "decode_lD: prn=%3d\n", U1(raw.Buff[idx:]))

	prn = int(U1(raw.Buff[idx:]))
	idx += 1
	frq = int(I1(raw.Buff[idx:]))
	idx += 1
	time = int(U4L(raw.Buff[idx:]))
	idx += 4
	ctype = int(U1(raw.Buff[idx:]))
	idx += 1
	length = int(U1(raw.Buff[idx:]))
	idx += 1

	if raw.Len != 14+length*4 {
		Trace(2, "javad lD length error: len=%d\n", raw.Len)
		return -1
	}
	if raw.OutType > 0 {
		copy(raw.MsgType[len(string(raw.MsgType[:])):], []byte(fmt.Sprintf(" prn=%2d frq=%2d time=%7d type=%d", prn, frq, time, ctype)))
	}
	if sat = SatNo(SYS_GLO, prn); sat == 0 {
		Trace(2, "javad lD satellite error: prn=%d\n", prn)
		return 0
	}
	if ctype != 0 {
		Trace(3, "javad lD type unsupported: type=%d\n", ctype)
		return 0
	}
	if id = int(U4L(raw.Buff[idx:]) >> 20); id&0xF < 1 {
		return 0
	}

	/* get 77 bit (25x3+2) in frame without hamming and time mark */
	for i = 0; i < 4; i++ {
		if i < 3 {
			SetBitU(raw.SubFrm[sat-1][(id-1)*10:], i*25, 25,
				U4L(raw.Buff[idx+4*i:])>>0)
		} else {
			SetBitU(raw.SubFrm[sat-1][(id-1)*10:], i*25, 2,
				U4L(raw.Buff[idx+4*i:])>>23)
		}
	}
	if id != 4 {
		return 0
	}

	/* decode glonass ephemeris strings */
	geph.Tof = raw.Time
	if Decode_Glostr(raw.SubFrm[sat-1][:], &geph, nil) == 0 || geph.Sat != sat {
		return -1
	}
	geph.Frq = frq

	if !strings.Contains(raw.Opt, "-EPHALL") {
		if geph.Iode == raw.NavData.Geph[prn-1].Iode {
			return 0
		} /* unchanged */
	}
	raw.NavData.Geph[prn-1] = geph
	raw.EphSat = sat
	return 2
}

/* decode [ED] Galileo raw navigation data -----------------------------------*/
func decode_ED(raw *Raw) int {
	Trace(3, "javad ED unsupported\n")

	return 0
}

/* decode [cd] Beidou raw navigation data ------------------------------------*/
func decode_cd(raw *Raw) int {
	Trace(3, "javad cd unsupported\n")

	return 0
}

/* decode [id] IRNSS raw navigation data -------------------------------------*/
func decode_id(raw *Raw) int {
	Trace(3, "javad id unsupported\n")

	return 0
}

/* decode [WD] SBAS raw navigation data --------------------------------------*/
func decode_WD(raw *Raw) int {
	var i, prn, tow, tow_p, week int
	var p = 5

	if checksum_javad(raw.Buff[:], raw.Len) == 0 {
		Trace(2, "javad WD checksum error: len=%d\n", raw.Len)
		return -1
	}
	if raw.Len < 45 {
		Trace(2, "javad WD length error: len=%d\n", raw.Len)
		return -1
	}
	Trace(3, "decode_WD: prn=%3d\n", U1(raw.Buff[p:]))

	prn = int(U1(raw.Buff[p:]))
	p += 1
	tow = int(U4L(raw.Buff[p:]))
	p += 4 + 2

	if raw.OutType > 0 {

		copy(raw.MsgType[len(string(raw.MsgType[:])):], []byte(fmt.Sprintf(" prn=%3d tow=%6d", prn, tow)))
	}
	if (prn < MINPRNSBS || MAXPRNSBS < prn) && (prn < MINPRNQZS || MAXPRNQZS < prn) {
		Trace(2, "javad WD satellite error: prn=%d\n", prn)
		return 0
	}
	if prn >= MINPRNQZS && prn <= MAXPRNQZS {
		prn -= 10 /* QZSS L1S */
	}
	raw.Sbsmsg.Prn = uint8(prn)
	raw.Sbsmsg.Tow = tow

	if raw.Time.Time == 0 {
		raw.Sbsmsg.Week = 0
	} else {
		tow_p = int(Time2GpsT(raw.Time, &week))
		if tow < tow_p-302400.0 {
			week++
		} else if tow > tow_p+302400.0 {
			week--
		}
		raw.Sbsmsg.Week = week
	}
	for i = 0; i < 29; i++ {
		raw.Sbsmsg.Msg[i] = raw.Buff[p]
		p++
	}
	raw.Sbsmsg.Msg[28] &= 0xC0
	return 3
}

/* decode [R*] pseudoranges --------------------------------------------------*/
func decode_Rx(raw *Raw, sig rune) int {
	var (
		p                      = 5
		pr, prm                float64
		i, idx, code, sat, sys int
	)

	if is_meas(sig) == 0 || raw.Tod < 0 || raw.ObsBuf.N() == 0 {
		return 0
	}

	if checksum_javad(raw.Buff[:], raw.Len) == 0 {
		Trace(2, "javad R%c checksum error: len=%d\n", sig, raw.Len)
		return -1
	}
	if raw.Len != raw.ObsBuf.N()*8+6 {
		Trace(2, "javad R%c length error: n=%d len=%d\n", sig, raw.ObsBuf.N(), raw.Len)
		return -1
	}
	for i = 0; i < raw.ObsBuf.N() && i < MAXOBS; i++ {
		pr = R8L(raw.Buff[p:])
		p += 8
		if pr == 0.0 {
			continue
		}

		sat = raw.ObsBuf.Data[i].Sat
		if sys = SatSys(sat, nil); sys == 0 {
			continue
		}

		prm = pr * CLIGHT

		if sig == 'C' {
			raw.PrCA[sat-1] = prm
		}

		if idx = sig2idx(sys, sig, &code); p < 0 {
			continue
		}

		if idx = checkpri(sys, code, raw.Opt, idx); idx >= 0 {
			if settag(&raw.ObsBuf.Data[i], raw.Time) == 0 {
				continue
			}
			raw.ObsBuf.Data[i].P[idx] = prm
			raw.ObsBuf.Data[i].Code[idx] = uint8(code)
		}
	}
	return 0
}

/* decode [r*] short pseudoranges --------------------------------------------*/
func decode_rx(raw *Raw, sig rune) int {
	var (
		p                          = 5
		prm                        float64
		i, idx, code, pr, sat, sys int
	)
	if is_meas(sig) == 0 || raw.Tod < 0 || raw.ObsBuf.N() == 0 {
		return 0
	}

	if checksum_javad(raw.Buff[:], raw.Len) == 0 {
		Trace(2, "javad r%c checksum error: len=%d\n", sig, raw.Len)
		return -1
	}
	if raw.Len != raw.ObsBuf.N()*4+6 {
		Trace(2, "javad r%c length error: n=%d len=%d\n", sig, raw.ObsBuf.N(), raw.Len)
		return -1
	}
	for i = 0; i < raw.ObsBuf.N() && i < MAXOBS; i++ {
		pr = int(I4L(raw.Buff[p:]))
		p += 4
		sat = raw.ObsBuf.Data[i].Sat
		if sys = SatSys(sat, nil); sys == 0 {
			continue
		}

		if pr == 0x7FFFFFFF {
			Trace(3, "javad r%c value missing: sat=%2d\n", sig, sat)
			continue
		}
		/*                             Ksys  Asys */
		switch sys {
		case SYS_SBS:
			prm = (float64(pr)*1e-11 + 0.125) * CLIGHT /* [6] */
		case SYS_QZS:
			prm = (float64(pr)*2e-11 + 0.125) * CLIGHT /* [3] */
		case SYS_CMP:
			prm = (float64(pr)*2e-11 + 0.105) * CLIGHT /* [4] */
		case SYS_GAL:
			prm = (float64(pr)*2e-11 + 0.085) * CLIGHT /* [7] */
		case SYS_IRN:
			prm = (float64(pr)*2e-11 + 0.105) * CLIGHT /* [6] */
		default:
			prm = (float64(pr)*1e-11 + 0.075) * CLIGHT
		}
		if sig == 'c' {
			raw.PrCA[sat-1] = prm
		}

		if idx = sig2idx(sys, sig, &code); idx < 0 {
			continue
		}

		if p = checkpri(sys, code, raw.Opt, idx); idx >= 0 {
			if settag(&raw.ObsBuf.Data[i], raw.Time) == 0 {
				continue
			}
			raw.ObsBuf.Data[i].P[idx] = prm
			raw.ObsBuf.Data[i].Code[idx] = uint8(code)
		}
	}
	return 0
}

/* decode [*R] relative pseudoranges -----------------------------------------*/
func decode_xR(raw *Raw, sig rune) int {
	var (
		p                      = 5
		pr                     float64
		i, idx, code, sat, sys int
	)
	if is_meas(sig) == 0 || raw.Tod < 0 || raw.ObsBuf.N() == 0 {
		return 0
	}

	if checksum_javad(raw.Buff[:], raw.Len) == 0 {
		Trace(2, "javad %cR checksum error: len=%d\n", sig, raw.Len)
		return -1
	}
	if raw.Len != raw.ObsBuf.N()*4+6 {
		Trace(2, "javad %cR length error: n=%d len=%d\n", sig, raw.ObsBuf.N(), raw.Len)
		return -1
	}
	for i = 0; i < raw.ObsBuf.N() && i < MAXOBS; i++ {
		pr = float64(R4L(raw.Buff[p:]))
		p += 4
		if pr == 0.0 {
			continue
		}

		sat = raw.ObsBuf.Data[i].Sat
		if sys = SatSys(sat, nil); sys == 0 || raw.PrCA[sat-1] == 0.0 {
			continue
		}

		if idx = sig2idx(sys, sig, &code); idx < 0 {
			continue
		}

		if idx = checkpri(sys, code, raw.Opt, idx); idx >= 0 {
			if settag(&raw.ObsBuf.Data[i], raw.Time) == 0 {
				continue
			}
			raw.ObsBuf.Data[i].P[idx] = pr*CLIGHT + raw.PrCA[sat-1]
			raw.ObsBuf.Data[i].Code[idx] = uint8(code)
		}
	}
	return 0
}

/* decode [*r] short relative pseudoranges -----------------------------------*/
func decode_xr(raw *Raw, sig rune) int {
	var (
		p                      = 5
		prm                    float64
		pr                     int16
		i, idx, code, sat, sys int
	)
	if is_meas(sig) == 0 || raw.Tod < 0 || raw.ObsBuf.N() == 0 {
		return 0
	}

	if checksum_javad(raw.Buff[:], raw.Len) == 0 {
		Trace(2, "javad %cr checksum error: len=%d\n", sig, raw.Len)
		return -1
	}
	if raw.Len != raw.ObsBuf.N()*2+6 {
		Trace(2, "javad %cR length error: n=%d len=%d\n", sig, raw.ObsBuf.N(), raw.Len)
		return -1
	}
	for i = 0; i < raw.ObsBuf.N() && i < MAXOBS; i++ {
		pr = I2L(raw.Buff[p:])
		p += 2
		if pr == int16(0x7FFF) {
			continue
		}

		sat = raw.ObsBuf.Data[i].Sat
		if sys = SatSys(sat, nil); sys == 0 || raw.PrCA[sat-1] == 0.0 {
			continue
		}

		prm = (float64(pr)*1e-11+2e-7)*CLIGHT + raw.PrCA[sat-1]

		if idx = sig2idx(sys, sig, &code); idx < 0 {
			continue
		}

		if idx = checkpri(sys, code, raw.Opt, idx); idx >= 0 {
			if settag(&raw.ObsBuf.Data[i], raw.Time) == 0 {
				continue
			}
			raw.ObsBuf.Data[i].P[idx] = prm
			raw.ObsBuf.Data[i].Code[idx] = uint8(code)
		}
	}
	return 0
}

/* decode [P*] carrier phases ------------------------------------------------*/
func decode_Px(raw *Raw, sig rune) int {
	var (
		p                 = 5
		cp                float64
		i, idx, code, sys int
	)
	if is_meas(sig) == 0 || raw.Tod < 0 || raw.ObsBuf.N() == 0 {
		return 0
	}

	if checksum_javad(raw.Buff[:], raw.Len) == 0 {
		Trace(2, "javad P%c checksum error: len=%d\n", sig, raw.Len)
		return -1
	}
	if raw.Len != raw.ObsBuf.N()*8+6 {
		Trace(2, "javad P%c length error: n=%d len=%d\n", sig, raw.ObsBuf.N(), raw.Len)
		return -1
	}
	for i = 0; i < raw.ObsBuf.N() && i < MAXOBS; i++ {
		cp = R8L(raw.Buff[p:])
		p += 8
		if cp == 0.0 {
			continue
		}

		if sys = SatSys(raw.ObsBuf.Data[i].Sat, nil); sys == 0 {
			continue
		}

		if idx = sig2idx(sys, sig, &code); idx < 0 {
			continue
		}

		if idx = checkpri(sys, code, raw.Opt, idx); idx >= 0 {
			if settag(&raw.ObsBuf.Data[i], raw.Time) == 0 {
				continue
			}
			raw.ObsBuf.Data[i].L[idx] = cp
			raw.ObsBuf.Data[i].Code[idx] = uint8(code)
		}
	}
	return 0
}

/* decode [p*] short carrier phases ------------------------------------------*/
func decode_px(raw *Raw, sig rune) int {
	var (
		p                 = 5
		cp                uint32
		i, idx, code, sys int
	)
	if is_meas(sig) == 0 || raw.Tod < 0 || raw.ObsBuf.N() == 0 {
		return 0
	}

	if checksum_javad(raw.Buff[:], raw.Len) == 0 {
		Trace(2, "javad p%c checksum error: len=%d\n", sig, raw.Len)
		return -1
	}
	if raw.Len != raw.ObsBuf.N()*4+6 {
		Trace(2, "javad p%c length error: n=%d len=%d\n", sig, raw.ObsBuf.N(), raw.Len)
		return -1
	}
	for i = 0; i < raw.ObsBuf.N() && i < MAXOBS; i++ {
		cp = U4L(raw.Buff[p:])
		p += 4
		if cp == 0xFFFFFFFF {
			continue
		}

		if sys = SatSys(raw.ObsBuf.Data[i].Sat, nil); sys == 0 {
			continue
		}

		if idx = sig2idx(sys, sig, &code); idx < 0 {
			continue
		}

		if idx = checkpri(sys, code, raw.Opt, idx); idx >= 0 {
			if settag(&raw.ObsBuf.Data[i], raw.Time) == 0 {
				continue
			}
			raw.ObsBuf.Data[i].L[idx] = float64(cp) / 1024.0
			raw.ObsBuf.Data[i].Code[idx] = uint8(code)
		}
	}
	return 0
}

/* decode [*P] short relative carrier phases ---------------------------------*/
func decode_xP(raw *Raw, sig rune) int {
	var (
		p                      = 5
		cp, rcp, freq          float64
		i, idx, code, sat, sys int
	)
	if is_meas(sig) == 0 || raw.Tod < 0 || raw.ObsBuf.N() == 0 {
		return 0
	}

	if checksum_javad(raw.Buff[:], raw.Len) == 0 {
		Trace(2, "javad %cP checksum error: len=%d\n", sig, raw.Len)
		return -1
	}
	if raw.Len != raw.ObsBuf.N()*4+6 {
		Trace(2, "javad %cP length error: n=%d len=%d\n", sig, raw.ObsBuf.N(), raw.Len)
		return -1
	}
	for i = 0; i < raw.ObsBuf.N() && i < MAXOBS; i++ {
		rcp = float64(R4L(raw.Buff[p:]))
		p += 4
		if rcp == 0.0 {
			continue
		}

		sat = raw.ObsBuf.Data[i].Sat
		if sys = SatSys(sat, nil); sys == 0 || raw.PrCA[sat-1] == 0.0 {
			continue
		}

		if idx = sig2idx(sys, sig, &code); idx < 0 {
			continue
		}

		if idx = checkpri(sys, code, raw.Opt, idx); idx >= 0 {
			if settag(&raw.ObsBuf.Data[i], raw.Time) == 0 {
				continue
			}

			freq = Code2Freq(sys, uint8(code), int(raw.FreqNum[i]))
			cp = (rcp + raw.PrCA[sat-1]/CLIGHT) * freq

			raw.ObsBuf.Data[i].L[idx] = cp
			raw.ObsBuf.Data[i].Code[idx] = uint8(code)
		}
	}
	return 0
}

/* decode [*p] short relative carrier phases ---------------------------------*/
func decode_xp(raw *Raw, sig rune) int {
	var (
		p                           = 5
		cp, freq                    float64
		i, idx, code, rcp, sat, sys int
	)
	if is_meas(sig) == 0 || raw.Tod < 0 || raw.ObsBuf.N() == 0 {
		return 0
	}

	if checksum_javad(raw.Buff[:], raw.Len) == 0 {
		Trace(2, "javad %cp checksum error: len=%d\n", sig, raw.Len)
		return -1
	}
	if raw.Len != raw.ObsBuf.N()*4+6 {
		Trace(2, "javad %cp length error: n=%d len=%d\n", sig, raw.ObsBuf.N(), raw.Len)
		return -1
	}
	for i = 0; i < raw.ObsBuf.N() && i < MAXOBS; i++ {
		rcp = int(I4L(raw.Buff[p:]))
		p += 4
		if rcp == 0x7FFFFFFF {
			continue
		}

		sat = raw.ObsBuf.Data[i].Sat
		if sys = SatSys(sat, nil); sys == 0 || raw.PrCA[sat-1] == 0.0 {
			continue
		}

		if idx = sig2idx(sys, sig, &code); idx < 0 {
			continue
		}

		if idx = checkpri(sys, code, raw.Opt, idx); idx >= 0 {
			if settag(&raw.ObsBuf.Data[i], raw.Time) == 0 {
				continue
			}

			freq = Code2Freq(sys, uint8(code), int(raw.FreqNum[i]))
			cp = (float64(rcp)*P2_40 + raw.PrCA[sat-1]/CLIGHT) * freq

			raw.ObsBuf.Data[i].L[idx] = cp
			raw.ObsBuf.Data[i].Code[idx] = uint8(code)
		}
	}
	return 0
}

/* decode [D*] doppler -------------------------------------------------------*/
func decode_Dx(raw *Raw, sig rune) int {
	var (
		p                          = 5
		dop                        float64
		i, idx, code, dp, sat, sys int
	)
	if is_meas(sig) == 0 || raw.Tod < 0 || raw.ObsBuf.N() == 0 {
		return 0
	}

	if checksum_javad(raw.Buff[:], raw.Len) == 0 {
		Trace(2, "javad D%c checksum error: len=%d\n", sig, raw.Len)
		return -1
	}
	if raw.Len != raw.ObsBuf.N()*4+6 {
		Trace(2, "javad D%c length error: n=%d len=%d\n", sig, raw.ObsBuf.N(), raw.Len)
		return -1
	}
	for i = 0; i < raw.ObsBuf.N() && i < MAXOBS; i++ {
		dp = int(I4L(raw.Buff[p:]))
		p += 4
		if dp == 0x7FFFFFFF {
			continue
		}

		sat = raw.ObsBuf.Data[i].Sat
		if sys = SatSys(sat, nil); sys == 0 {
			continue
		}

		dop = -float64(dp) * 1e-4

		if sig == 'C' {
			raw.DpCA[sat-1] = dop
		}

		if idx = sig2idx(sys, sig, &code); idx < 0 {
			continue
		}

		if idx = checkpri(sys, code, raw.Opt, idx); idx >= 0 {
			if settag(&raw.ObsBuf.Data[i], raw.Time) == 0 {
				continue
			}
			raw.ObsBuf.Data[i].D[idx] = float64(dop)
		}
	}
	return 0
}

/* decode [*d] short relative doppler ----------------------------------------*/
func decode_xd(raw *Raw, sig rune) int {
	var (
		p                      = 5
		dop, f1, fn            float64
		rdp                    int16
		i, idx, code, sat, sys int
	)
	if is_meas(sig) == 0 || raw.Tod < 0 || raw.ObsBuf.N() == 0 {
		return 0
	}

	if checksum_javad(raw.Buff[:], raw.Len) == 0 {
		Trace(2, "javad %cd checksum error: len=%d\n", sig, raw.Len)
		return -1
	}
	if raw.Len != raw.ObsBuf.N()*2+6 {
		Trace(2, "javad %cd length error: n=%d len=%d\n", sig, raw.ObsBuf.N(), raw.Len)
		return -1
	}
	for i = 0; i < raw.ObsBuf.N() && i < MAXOBS; i++ {
		rdp = I2L(raw.Buff[p:])
		p += 2
		if rdp == int16(0x7FFF) {
			continue
		}

		sat = raw.ObsBuf.Data[i].Sat
		if sys = SatSys(sat, nil); sys == 0 || raw.DpCA[sat-1] == 0.0 {
			continue
		}

		if idx = sig2idx(sys, sig, &code); idx < 0 {
			continue
		}

		if idx = checkpri(sys, code, raw.Opt, idx); idx >= 0 {
			if settag(&raw.ObsBuf.Data[i], raw.Time) == 0 {
				continue
			}
			f1 = Code2Freq(sys, CODE_L1X, int(raw.FreqNum[i]))
			fn = Code2Freq(sys, uint8(code), int(raw.FreqNum[i]))
			dop = (-float64(rdp) + raw.DpCA[sat-1]*1e4) * fn / f1 * 1e-4

			raw.ObsBuf.Data[i].D[idx] = float64(dop)
		}
	}
	return 0
}

/* decode [E*] carrier to noise ratio ----------------------------------------*/
func decode_Ex(raw *Raw, sig rune) int {
	var (
		p                 = 5
		cnr               uint8
		i, idx, code, sys int
	)

	if is_meas(sig) == 0 || raw.Tod < 0 || raw.ObsBuf.N() == 0 {
		return 0
	}

	if checksum_javad(raw.Buff[:], raw.Len) == 0 {
		Trace(2, "javad E%c checksum error: len=%d\n", sig, raw.Len)
		return -1
	}
	if raw.Len != raw.ObsBuf.N()+6 {
		Trace(2, "javad E%c length error: n=%d len=%d\n", sig, raw.ObsBuf.N(), raw.Len)
		return -1
	}
	for i = 0; i < raw.ObsBuf.N() && i < MAXOBS; i++ {
		cnr = U1(raw.Buff[p:])
		p += 1
		if cnr == 255 {
			continue
		}

		if sys = SatSys(raw.ObsBuf.Data[i].Sat, nil); sys == 0 {
			continue
		}

		if idx = sig2idx(sys, sig, &code); idx < 0 {
			continue
		}

		if idx = checkpri(sys, code, raw.Opt, idx); idx >= 0 {
			if settag(&raw.ObsBuf.Data[i], raw.Time) == 0 {
				continue
			}
			raw.ObsBuf.Data[i].SNR[idx] = uint16(float64(cnr)/SNR_UNIT + 0.5)
		}
	}
	return 0
}

/* decode [*E] carrier to noise ratio x 4 ------------------------------------*/
func decode_xE(raw *Raw, sig rune) int {
	var (
		p                 = 5
		cnr               uint8
		i, idx, code, sys int
	)

	if is_meas(sig) == 0 || raw.Tod < 0 || raw.ObsBuf.N() == 0 {
		return 0
	}

	if checksum_javad(raw.Buff[:], raw.Len) == 0 {
		Trace(2, "javad %cE checksum error: len=%d\n", sig, raw.Len)
		return -1
	}
	if raw.Len != raw.ObsBuf.N()+6 {
		Trace(2, "javad %cE length error: n=%d len=%d\n", sig, raw.ObsBuf.N(), raw.Len)
		return -1
	}
	for i = 0; i < raw.ObsBuf.N() && i < MAXOBS; i++ {
		cnr = U1(raw.Buff[p:])
		p += 1
		if cnr == 255 {
			continue
		}

		if sys = SatSys(raw.ObsBuf.Data[i].Sat, nil); sys == 0 {
			continue
		}

		if idx = sig2idx(sys, sig, &code); idx < 0 {
			continue
		}

		if idx = checkpri(sys, code, raw.Opt, idx); idx >= 0 {
			if settag(&raw.ObsBuf.Data[i], raw.Time) == 0 {
				continue
			}
			raw.ObsBuf.Data[i].SNR[idx] = uint16(float64(cnr)*0.25/SNR_UNIT + 0.5)
		}
	}
	return 0
}

/* decode [F*] signal lock loop flags ----------------------------------------*/
func decode_Fx(raw *Raw, sig rune) int {
	var (
		p                      = 5
		flags                  uint16
		i, idx, code, sat, sys int
	)

	if is_meas(sig) == 0 || raw.Tod < 0 || raw.ObsBuf.N() == 0 {
		return 0
	}

	if checksum_javad(raw.Buff[:], raw.Len) == 0 {
		Trace(2, "javad F%c checksum error: len=%d\n", sig, raw.Len)
		return -1
	}
	if raw.Len != raw.ObsBuf.N()*2+6 {
		Trace(2, "javad F%c length error: n=%d len=%d\n", sig, raw.ObsBuf.N(), raw.Len)
		return -1
	}
	for i = 0; i < raw.ObsBuf.N() && i < MAXOBS; i++ {
		flags = U2L(raw.Buff[p:])
		p += 1
		if flags == 0xFFFF {
			continue
		}

		sat = raw.ObsBuf.Data[i].Sat
		if sys = SatSys(sat, nil); sys == 0 {
			continue
		}

		if idx = sig2idx(sys, sig, &code); idx < 0 {
			continue
		}

		if idx = checkpri(sys, code, raw.Opt, idx); idx >= 0 {
			if settag(&raw.ObsBuf.Data[i], raw.Time) == 0 {
				continue
			}
			// #if 0 /* disable to suppress overdetection of cycle-slips */
			//             if (flags&0x20) { /* loss-of-lock potential */
			//                 raw.ObsBuf.Data[i].LLI[idx]|=1;
			//             }
			//             if (!(flags&0x40)||!(flags&0x100)) { /* integral indicator */
			//                 raw.ObsBuf.Data[i].LLI[idx]|=2;
			//             }
			// #endif
		}
	}
	return 0
}

/* decode [TC] CA/L1 continuous tracking time --------------------------------*/
func decode_TC(raw *Raw) int {
	var (
		tt, tt_p uint16
		i, sat   int
		p        = 5
	)
	if raw.ObsBuf.N() == 0 {
		return 0
	}

	if checksum_javad(raw.Buff[:], raw.Len) == 0 {
		Trace(2, "javad TC checksum error: len=%d\n", raw.Len)
		return -1
	}
	if raw.Len != raw.ObsBuf.N()*2+6 {
		Trace(2, "javad TC length error: n=%d len=%d\n", raw.ObsBuf.N(), raw.Len)
		return -1
	}
	for i = 0; i < raw.ObsBuf.N() && i < MAXOBS; i++ {
		tt = U2L(raw.Buff[p:])
		p += 2
		if tt == 0xFFFF {
			continue
		}

		if settag(&raw.ObsBuf.Data[i], raw.Time) == 0 {
			continue
		}

		sat = raw.ObsBuf.Data[i].Sat
		tt_p = uint16(raw.LockTime[sat-1][0])

		Trace(4, "%s: sat=%2d tt=%6d.%6d\n", TimeStr(raw.Time, 3), sat, tt_p, tt)

		/* loss-of-lock detected by lock-time counter */
		if tt == 0 || tt < tt_p {
			Trace(3, "decode_TC: loss-of-lock detected: t=%s sat=%2d tt=%6d.%6d\n",
				TimeStr(raw.Time, 3), sat, tt_p, tt)
			raw.ObsBuf.Data[i].LLI[0] |= 1
		}
		raw.LockTime[sat-1][0] = float64(tt)
	}
	return 0
}

/* decode JAVAD raw message --------------------------------------------------*/
func decode_javad(raw *Raw) int {
	var idx int

	Trace(3, "decode_javad: type=%2.2s len=%3d\n", raw.Buff[idx:], raw.Len)

	if raw.OutType > 0 {
		copy(raw.MsgType[len(string(raw.MsgType[:])):], []byte(fmt.Sprintf("JAVAD %2.2s (%4d)", raw.Buff[idx:], raw.Len)))
	}
	if strings.Contains(string(raw.Buff[idx:2]), "~~") {
		return decode_RT(raw)
	} /* receiver time */

	if strings.Contains(raw.Opt, "-NOET") {
		if strings.Contains(string(raw.Buff[idx:2]), "::") {
			return decode_ET(raw)
		} /* epoch time */
	}
	if strings.Contains(string(raw.Buff[idx:2]), "RD") {
		return decode_RD(raw)
	} /* receiver date */
	if strings.Contains(string(raw.Buff[idx:2]), "SI") {
		return decode_SI(raw)
	} /* satellite indices */
	if strings.Contains(string(raw.Buff[idx:2]), "NN") {
		return decode_NN(raw)
	} /* GLONASS slot numbers */
	if strings.Contains(string(raw.Buff[idx:2]), "GA") {
		return decode_GA(raw)
	} /* GPS almanac */
	if strings.Contains(string(raw.Buff[idx:2]), "NA") {
		return decode_NA(raw)
	} /* GLONASS almanac */
	if strings.Contains(string(raw.Buff[idx:2]), "EA") {
		return decode_EA(raw)
	} /* Galileo almanac */
	if strings.Contains(string(raw.Buff[idx:2]), "WA") {
		return decode_WA(raw)
	} /* SBAS almanac */
	if strings.Contains(string(raw.Buff[idx:2]), "QA") {
		return decode_QA(raw)
	} /* QZSS almanac */
	if strings.Contains(string(raw.Buff[idx:2]), "CA") {
		return decode_CA(raw)
	} /* Beidou almanac */
	if strings.Contains(string(raw.Buff[idx:2]), "IA") {
		return decode_IA(raw)
	} /* IRNSS almanac */

	if strings.Contains(string(raw.Buff[idx:2]), "GE") {
		return decode_GE(raw)
	} /* GPS ephemeris */
	if strings.Contains(string(raw.Buff[idx:2]), "NE") {
		return decode_NE(raw)
	} /* GLONASS ephemeris */
	if strings.Contains(string(raw.Buff[idx:2]), "EN") {
		return decode_EN(raw)
	} /* Galileo ephemeris */
	if strings.Contains(string(raw.Buff[idx:2]), "WE") {
		return decode_WE(raw)
	} /* SBAS ephemeris */
	if strings.Contains(string(raw.Buff[idx:2]), "QE") {
		return decode_QE(raw)
	} /* QZSS ephemeris */
	if strings.Contains(string(raw.Buff[idx:2]), "CN") {
		return decode_CN(raw)
	} /* Beidou ephemeris */
	if strings.Contains(string(raw.Buff[idx:2]), "IE") {
		return decode_IE(raw)
	} /* IRNSS ephemeris */

	if strings.Contains(string(raw.Buff[idx:2]), "UO") {
		return decode_UO(raw)
	} /* GPS UTC time parameters */
	if strings.Contains(string(raw.Buff[idx:2]), "NU") {
		return decode_NU(raw)
	} /* GLONASS UTC and GPS time par */
	if strings.Contains(string(raw.Buff[idx:2]), "EU") {
		return decode_EU(raw)
	} /* Galileo UTC and GPS time par */
	if strings.Contains(string(raw.Buff[idx:2]), "WU") {
		return decode_WU(raw)
	} /* WAAS UTC time parameters */
	if strings.Contains(string(raw.Buff[idx:2]), "QU") {
		return decode_QU(raw)
	} /* QZSS UTC and GPS time par */
	if strings.Contains(string(raw.Buff[idx:2]), "IO") {
		return decode_IO(raw)
	} /* ionospheric parameters */

	if strings.Contains(string(raw.Buff[idx:2]), "GD") {
		return decode_nD(raw, SYS_GPS)
	} /* raw navigation data */
	if strings.Contains(string(raw.Buff[idx:2]), "QD") {
		return decode_nD(raw, SYS_QZS)
	} /* raw navigation data */
	if strings.Contains(string(raw.Buff[idx:2]), "gd") {
		return decode_nd(raw, SYS_GPS)
	} /* raw navigation data */
	if strings.Contains(string(raw.Buff[idx:2]), "qd") {
		return decode_nd(raw, SYS_QZS)
	} /* raw navigation data */
	if strings.Contains(string(raw.Buff[idx:2]), "ED") {
		return decode_ED(raw)
	} /* Galileo raw navigation data */

	if strings.Contains(string(raw.Buff[idx:2]), "cd") {
		return decode_cd(raw)
	} /* Beidou raw navigation data */
	if strings.Contains(string(raw.Buff[idx:2]), "id") {
		return decode_id(raw)
	} /* IRNSS raw navigation data */
	if strings.Contains(string(raw.Buff[idx:2]), "LD") {
		return decode_LD(raw)
	} /* GLONASS raw navigation data */
	if strings.Contains(string(raw.Buff[idx:2]), "lD") {
		return decode_lD(raw)
	} /* GLONASS raw navigation data */
	if strings.Contains(string(raw.Buff[idx:2]), "WD") {
		return decode_WD(raw)
	} /* SBAS raw navigation data */
	if strings.Contains(string(raw.Buff[idx:2]), "TC") {
		return decode_TC(raw)
	} /* CA/L1 continuous track time */

	if raw.Buff[0] == 'R' {
		return decode_Rx(raw, rune(raw.Buff[1]))
	} /* pseudoranges */
	if raw.Buff[0] == 'r' {
		return decode_rx(raw, rune(raw.Buff[1]))
	} /* short pseudoranges */
	if raw.Buff[1] == 'R' {
		return decode_xR(raw, rune(raw.Buff[0]))
	} /* relative pseudoranges */
	if raw.Buff[1] == 'r' {
		return decode_xr(raw, rune(raw.Buff[0]))
	} /* short relative pseudoranges */
	if raw.Buff[0] == 'P' {
		return decode_Px(raw, rune(raw.Buff[1]))
	} /* carrier phases */
	if raw.Buff[0] == 'p' {
		return decode_px(raw, rune(raw.Buff[1]))
	} /* short carrier phases */
	if raw.Buff[1] == 'P' {
		return decode_xP(raw, rune(raw.Buff[0]))
	} /* relative carrier phases */
	if raw.Buff[1] == 'p' {
		return decode_xp(raw, rune(raw.Buff[0]))
	} /* relative carrier phases */
	if raw.Buff[0] == 'D' {
		return decode_Dx(raw, rune(raw.Buff[1]))
	} /* doppler */
	if raw.Buff[1] == 'd' {
		return decode_xd(raw, rune(raw.Buff[0]))
	} /* short relative doppler */
	if raw.Buff[0] == 'E' {
		return decode_Ex(raw, rune(raw.Buff[1]))
	} /* carrier to noise ratio */
	if raw.Buff[1] == 'E' {
		return decode_xE(raw, rune(raw.Buff[0]))
	} /* carrier to noise ratio x 4 */
	if raw.Buff[0] == 'F' {
		return decode_Fx(raw, rune(raw.Buff[1]))
	} /* signal lock loop flags */

	return 0
}

/* sync JAVAD message --------------------------------------------------------*/
func sync_javad(buff []uint8, data uint8) int {
	p := buff[0]

	buff[0] = buff[1]
	buff[1] = buff[2]
	buff[2] = buff[3]
	buff[3] = buff[4]
	buff[4] = data

	/* sync message header {\r|\n}IIHHH (II:id,HHH: hex length) */
	if (p == '\r' || p == '\n') && ISTXT(buff[0]) && ISTXT(buff[1]) &&
		ISHEX(buff[2]) && ISHEX(buff[3]) && ISHEX(buff[4]) {
		return 1
	}
	return 0
}

/* clear buffer --------------------------------------------------------------*/
func clearbuff(raw *Raw) {

	for i := 0; i < 5; i++ {
		raw.Buff[i] = 0
	}
	raw.Len, raw.NumByte = 0, 0
}

/* input JAVAD raw message from stream -----------------------------------------
* fetch next JAVAD raw data and input a mesasge from stream
* args   : raw *Raw       IO  receiver raw data control struct
*          uint8_t data     I   stream data (1 byte)
* return : status (-1: error message, 0: no message, 1: input observation data,
*                  2: input ephemeris, 3: input sbas message,
*                  9: input ion/utc parameter)
*
* notes  : to specify input options, set raw.Opt to the following option
*          strings separated by spaces.
*
*          -EPHALL : input all ephemerides
*          -GL1W   : select 1W for GPS L1 (default 1C)
*          -GL1X   : select 1X for GPS L1 (default 1C)
*          -GL2X   : select 2X for GPS L2 (default 2W)
*          -RL1P   : select 1C for GLO G1 (default 1C)
*          -RL2C   : select 2C for GLO G2 (default 2P)
*          -JL1Z   : select 1Z for QZS L1 (default 1C)
*          -JL1X   : select 1X for QZS L1 (default 1C)
*          -NOET   : discard epoch time message ET (::)
*          -GALINAV: select F/NAV for Galileo ephemeris (default: all)
*          -GALFNAV: select F/NAV for Galileo ephemeris (default: all)
*-----------------------------------------------------------------------------*/
func Input_javad(raw *Raw, data uint8) int {
	var length, stat int

	Trace(5, "input_javad: data=%02x\n", data)

	/* synchronize message */
	if raw.NumByte == 0 {
		if sync_javad(raw.Buff[:], data) == 0 {
			return 0
		}
		if length = decodelen(raw.Buff[2:]); length == 0 || length > MAXRAWLEN-5 {
			Trace(2, "javad message length error: len=%d\n", length)
			clearbuff(raw)
			return -1
		}
		raw.Len = length + 5
		raw.NumByte = 5
		return 0
	}
	raw.Buff[raw.NumByte] = data
	raw.NumByte++

	if raw.NumByte < raw.Len {
		return 0
	}

	/* decode javad raw message */
	stat = decode_javad(raw)

	clearbuff(raw)
	return stat
}

/* start input file ----------------------------------------------------------*/
func startfile(raw *Raw) {
	raw.Tod = -1
	raw.ObsBuf.n = 0
	raw.Buff[4] = '\n'
}

/* end input file ------------------------------------------------------------*/
func endfile(raw *Raw) int {
	/* flush observation data buffer */
	if flushobuf(raw) == 0 {
		return -2
	}
	raw.ObsBuf.n = 0
	return 1
}

/* input JAVAD raw message from file -------------------------------------------
* fetch next JAVAD raw data and input a message from file
* args   : raw_t  *raw   IO     receiver raw data control struct
*          FILE   *fp    I      file pointer
* return : status(-2: end of file, -1...9: same as above)
*-----------------------------------------------------------------------------*/
func Input_javadf(raw *Raw, fp *os.File) int {
	var i, length, stat int

	Trace(4, "input_javadf:\n")

	/* start input file */
	if raw.Flag > 0 {
		startfile(raw)
		raw.Flag = 0
	}
	/* synchronize message */
	var c [1]byte
	if raw.NumByte == 0 {
		for i = 0; ; i++ {
			_, err := fp.Read(c[:])
			if err == io.EOF {
				return endfile(raw)
			}
			if sync_javad(raw.Buff[:], uint8(c[0])) > 0 {
				break
			}
			if i >= 4096 {
				return 0
			}
		}
	}
	if length = decodelen(raw.Buff[2:]); length == 0 || length > MAXRAWLEN-5 {
		Trace(2, "javad message length error: len=%3.3s\n", raw.Buff[2:])
		clearbuff(raw)
		return -1
	}
	raw.Len = length + 5
	raw.NumByte = 5

	n, _ := fp.Read(raw.Buff[5:raw.Len])
	if n < raw.Len-5 {
		return endfile(raw)
	}
	/* decode javad raw message */
	stat = decode_javad(raw)

	clearbuff(raw)
	return stat
}
