/*------------------------------------------------------------------------------
* nvs.c : NVS receiver dependent functions
*
*    Copyright (C) 2012-2016 by M.BAVARO and T.TAKASU, All rights reserved.
*    Copyright (C) 2014-2020 by T.TAKASU, All rights reserved.
*    Copyright (C) 2014-2022 by Feng Xuebin, All rights reserved.
*
*     [1] Description of BINR messages which is used by RC program for RINEX
*         files accumulation, NVS
*     [2] NAVIS Navis Standard Interface Protocol BINR, NVS
*
* version : $Revision:$ $Date:$
* history : 2012/01/30 1.0  first version by M.BAVARO
*           2012/11/08 1.1  modified by T.TAKASU
*           2013/02/23 1.2  fix memory access violation problem on arm
*           2013/04/24 1.3  fix bug on cycle-slip detection
*                           add range check of gps ephemeris week
*           2013/09/01 1.4  add check error of week, time jump, obs data range
*           2014/08/26 1.5  fix bug on iode in glonass ephemeris
*           2016/01/26 1.6  fix bug on unrecognized meas data (#130)
*           2017/04/11 1.7  (char *) . (signed char *)
*           2020/07/10 1.8  suppress warnings
*           2020/11/30 1.9  use integer type in stdint.h
*           2022/09/22 1.10 rewrite with golang
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
	NVSSYNC        = 0x10 /* nvs message sync code 1 */
	NVSENDMSG      = 0x03 /* nvs message sync code 1 */
	NVSCFG         = 0x06 /* nvs message cfg-??? */
	ID_XF5RAW      = 0xf5 /* nvs msg id: raw measurement data */
	ID_X4AIONO     = 0x4a /* nvs msg id: gps ionospheric data */
	ID_X4BTIME     = 0x4b /* nvs msg id: GPS/GLONASS/UTC timescale data */
	ID_XF7EPH      = 0xf7 /* nvs msg id: subframe buffer */
	ID_XE5BIT      = 0xe5 /* nvs msg id: bit information */
	ID_XD7ADVANCED = 0xd7 /* */
	ID_X02RATEPVT  = 0x02 /* */
	ID_XF4RATERAW  = 0xf4 /* */
	ID_XD7SMOOTH   = 0xd7 /* */
	ID_XD5BIT      = 0xd5 /* */
)

/* get fields (little-endian) ------------------------------------------------*/
// declared in crescent.go. comment by fxb
// #define U1(raw.Buff[idx:]) (*((uint8 *)(raw.Buff[idx:])))
// #define I1(raw.Buff[idx:]) (*((int8_t  *)(raw.Buff[idx:])))
// static uint16_t U2L(uint8 *raw.Buff[idx:]) {uint16_t u; memcpy(&u,raw.Buff[idx:],2); return u;}
// static uint32 U4L(uint8 *raw.Buff[idx:]) {uint32 u; memcpy(&u,raw.Buff[idx:],4); return u;}
// static int16_t  I2L(uint8 *raw.Buff[idx:]) {int16_t  i; memcpy(&i,raw.Buff[idx:],2); return i;}
// static int32_t  I4L(uint8 *raw.Buff[idx:]) {int32_t  i; memcpy(&i,raw.Buff[idx:],4); return i;}
// static float    R4L(uint8 *raw.Buff[idx:]) {float    r; memcpy(&r,raw.Buff[idx:],4); return r;}
// static float64   R8L(uint8 *raw.Buff[idx:]) {float64   r; memcpy(&r,raw.Buff[idx:],8); return r;}

/* ura values (ref [3] 20.3.3.3.1.1) -----------------------------------------*/
// declared in binex.go. comment by fxb
// static const float64 ura_eph[]={
//     2.4,3.4,4.85,6.85,9.65,13.65,24.0,48.0,96.0,192.0,384.0,768.0,1536.0,
//     3072.0,6144.0,0.0
// };
/* ura value (m) to ura index ------------------------------------------------*/
// declared in binex.go. comment by fxb
// static int uraindex(float64 value)
// {
//     int i;
//     for (i=0;i<15;i++) if (ura_eph[i]>=value) break;
//     return i;
// }
/* decode NVS xf5-raw: raw measurement data ----------------------------------*/
func decode_xf5raw(raw *Raw) int {
	var (
		time                                   Gtime
		tadj, toff, tn                         float64
		dTowInt                                int
		dTowUTC, dTowGPS, dTowFrac, L1, P1, D1 float64
		gpsutcTimescale                        float64
		sys, carrNo                            uint8
		i, j, prn, sat, n, nsat, week          int
		idx                                    = 2
		tstr                                   string
		flag                                   byte
	)
	Trace(4, "decode_xf5raw: len=%d\n", raw.Len)

	/* time tag adjustment option (-TADJ) */
	if q := strings.Index(raw.Opt, "-tadj"); q >= 0 {
		fmt.Sscanf(raw.Opt[q:], "-TADJ=%lf", &tadj)
	}
	dTowUTC = R8L(raw.Buff[idx:])
	week = int(U2L(raw.Buff[idx+8:]))
	gpsutcTimescale = R8L(raw.Buff[idx+10:])
	/* glonassutcTimescale = R8L(raw.Buff[idx:]+18); */
	_ = I1(raw.Buff[idx+26:]) /* rcvTimeScaleCorr */

	/* check gps week range */
	if week >= 4096 {
		Trace(2, "nvs xf5raw obs week error: week=%d\n", week)
		return -1
	}
	week = AdjGpsWeek(week)

	if (raw.Len-31)%30 > 0 {

		/* Message length is not correct: there could be an error in the stream */
		Trace(2, "nvs xf5raw len=%d seems not be correct\n", raw.Len)
		return -1
	}
	nsat = (raw.Len - 31) / 30

	dTowGPS = dTowUTC + gpsutcTimescale

	/* Tweak pseudoranges to allow Rinex to represent the NVS time of measure */
	dTowInt = int(10.0 * math.Floor((dTowGPS/10.0)+0.5))
	dTowFrac = dTowGPS - float64(dTowInt)
	time = GpsT2Time(week, float64(dTowInt)*0.001)

	/* time tag adjustment */
	if tadj > 0.0 {
		tn = Time2GpsT(time, &week) / tadj
		toff = (tn - math.Floor(tn+0.5)) * tadj
		time = TimeAdd(time, -toff)
	}
	/* check time tag jump and output warning */
	if raw.Time.Time > 0 && math.Abs(TimeDiff(time, raw.Time)) > 86400.0 {
		Time2Str(time, &tstr, 3)
		Trace(2, "nvs xf5raw time tag jump warning: time=%s\n", tstr)
	}
	if math.Abs(TimeDiff(time, raw.Time)) <= 1e-3 {
		Time2Str(time, &tstr, 3)
		Trace(2, "nvs xf5raw time tag duplicated: time=%s\n", tstr)
		return 0
	}
	for i, idx = 0, idx+27; (i < nsat) && (n < MAXOBS); i, idx = i+1, idx+30 {
		raw.ObsData.Data[n].Time = time
		switch U1(raw.Buff[idx:]) {
		case 1:
			sys = SYS_GLO
		case 2:
			sys = SYS_GPS
		case 4:
			sys = SYS_SBS
		default:
			sys = SYS_NONE
		}
		prn = int(U1(raw.Buff[idx+1:]))
		if sys == SYS_SBS {
			prn += 120
		} /* Correct this */
		if sat = SatNo(int(sys), prn); sat == 0 {
			Trace(2, "nvs xf5raw satellite number error: sys=%d prn=%d\n", sys, prn)
			continue
		}
		carrNo = uint8(I1(raw.Buff[idx+2:]))
		L1 = R8L(raw.Buff[idx+4:])
		P1 = R8L(raw.Buff[idx+12:])
		D1 = R8L(raw.Buff[idx+20:])

		/* check range error */
		if L1 < -1e10 || L1 > 1e10 || P1 < -1e10 || P1 > 1e10 || D1 < -1e5 || D1 > 1e5 {
			Trace(2, "nvs xf5raw obs range error: sat=%2d L1=%12.5e P1=%12.5e D1=%12.5e\n",
				sat, L1, P1, D1)
			continue
		}
		raw.ObsData.Data[n].SNR[0] = uint16(float64(I1(raw.Buff[idx+3:]))/SNR_UNIT + 0.5)
		if sys == SYS_GLO {
			raw.ObsData.Data[n].L[0] = L1 - toff*(FREQ1_GLO+DFRQ1_GLO*float64(carrNo))
		} else {
			raw.ObsData.Data[n].L[0] = L1 - toff*FREQ1
		}
		raw.ObsData.Data[n].P[0] = (P1-dTowFrac)*CLIGHT*0.001 - toff*CLIGHT /* in ms, needs to be converted */
		raw.ObsData.Data[n].D[0] = D1

		/* set LLI if meas flag 4 (carrier phase present) off . on */
		flag = U1(raw.Buff[idx+28:])
		raw.ObsData.Data[n].LLI[0] = 0
		if (flag&0x08) > 0 && (raw.Halfc[sat-1][0]&0x08) == 0 {
			raw.ObsData.Data[n].LLI[0] = 1
		}
		raw.Halfc[sat-1][0] = flag

		raw.ObsData.Data[n].Code[0] = CODE_L1C
		raw.ObsData.Data[n].Sat = sat

		for j = 1; j < NFREQ+NEXOBS; j++ {
			raw.ObsData.Data[n].L[j], raw.ObsData.Data[n].P[j] = 0.0, 0.0
			raw.ObsData.Data[n].D[j] = 0.0
			raw.ObsData.Data[n].SNR[j], raw.ObsData.Data[n].LLI[j] = 0, 0
			raw.ObsData.Data[n].Code[j] = CODE_NONE
		}
		n++
	}
	raw.Time = time
	raw.ObsData.n = n
	return 1
}

/* decode ephemeris ----------------------------------------------------------*/
func decode_gpsephem(sat int, raw *Raw) int {
	var (
		eph  Eph
		idx  = 2
		week uint16
		toc  float64
	)
	Trace(4, "decode_ephem: sat=%2d\n", sat)

	eph.Crs = float64(R4L(raw.Buff[idx+2:]))
	eph.Deln = float64(R4L(raw.Buff[idx+6:])) * 1e+3
	eph.M0 = R8L(raw.Buff[idx+10:])
	eph.Cuc = float64(R4L(raw.Buff[idx+18:]))
	eph.E = R8L(raw.Buff[idx+22:])
	eph.Cus = float64(R4L(raw.Buff[idx+30:]))
	eph.A = math.Pow(R8L(raw.Buff[idx+34:]), 2)
	eph.Toes = R8L(raw.Buff[idx+42:]) * 1e-3
	eph.Cic = float64(R4L(raw.Buff[idx+50:]))
	eph.OMG0 = R8L(raw.Buff[idx+54:])
	eph.Cis = float64(R4L(raw.Buff[idx+62:]))
	eph.I0 = R8L(raw.Buff[idx+66:])
	eph.Crc = float64(R4L(raw.Buff[idx+74:]))
	eph.Omg = R8L(raw.Buff[idx+78:])
	eph.OMGd = R8L(raw.Buff[idx+86:]) * 1e+3
	eph.Idot = R8L(raw.Buff[idx+94:]) * 1e+3
	eph.Tgd[0] = float64(R4L(raw.Buff[idx+102:])) * 1e-3
	toc = R8L(raw.Buff[idx+106:]) * 1e-3
	eph.F2 = float64(R4L(raw.Buff[idx+114:])) * 1e+3
	eph.F1 = float64(R4L(raw.Buff[idx+118:]))
	eph.F0 = float64(R4L(raw.Buff[idx+122:])) * 1e-3
	eph.Sva = uraindex(float64(I2L(raw.Buff[idx+126:])))
	eph.Iode = int(I2L(raw.Buff[idx+128:]))
	eph.Iodc = int(I2L(raw.Buff[idx+130:]))
	eph.Code = int(I2L(raw.Buff[idx+132:]))
	eph.Flag = int(I2L(raw.Buff[idx+134:]))
	week = uint16(I2L(raw.Buff[idx+136:]))
	eph.Fit = 0

	if week >= 4096 {
		Trace(2, "nvs gps ephemeris week error: sat=%2d week=%d\n", sat, week)
		return -1
	}
	eph.Week = AdjGpsWeek(int(week))
	eph.Toe = GpsT2Time(eph.Week, eph.Toes)
	eph.Toc = GpsT2Time(eph.Week, toc)
	eph.Ttr = raw.Time

	if !strings.Contains(raw.Opt, "-EPHALL") {
		if eph.Iode == raw.NavData.Ephs[sat-1].Iode {
			return 0
		} /* unchanged */
	}
	eph.Sat = sat
	raw.NavData.Ephs[sat-1] = eph
	raw.EphSat = sat
	raw.EphSet = 0
	return 2
}

/* adjust daily rollover of time ---------------------------------------------*/
// func  adjday(time Gtime,tod float64 )Gtime{
//     var  ep [6] float64
//     Time2Epoch(time,ep[:]);
//     tod_p:=ep[3]*3600.0+ep[4]*60.0+ep[5];
//     if      (tod<tod_p-43200.0) {tod+=86400.0;
//     }else if (tod>tod_p+43200.0) {tod-=86400.0;}
//     ep[3],ep[4],ep[5]=0.0, 0.0,0.0
//     return TimeAdd(Epoch2Time(ep[:]),tod);
// }
/* decode gloephem -----------------------------------------------------------*/
func decode_gloephem(sat int, raw *Raw) int {
	var (
		geph        GEph
		idx         = 2
		prn, tk, tb int
	)

	if raw.Len >= 93 {
		prn = int(I1(raw.Buff[idx+1:]))
		geph.Frq = int(I1(raw.Buff[idx+2:]))
		geph.Pos[0] = R8L(raw.Buff[idx+3:])
		geph.Pos[1] = R8L(raw.Buff[idx+11:])
		geph.Pos[2] = R8L(raw.Buff[idx+19:])
		geph.Vel[0] = R8L(raw.Buff[idx+27:]) * 1e+3
		geph.Vel[1] = R8L(raw.Buff[idx+35:]) * 1e+3
		geph.Vel[2] = R8L(raw.Buff[idx+43:]) * 1e+3
		geph.Acc[0] = R8L(raw.Buff[idx+51:]) * 1e+6
		geph.Acc[1] = R8L(raw.Buff[idx+59:]) * 1e+6
		geph.Acc[2] = R8L(raw.Buff[idx+67:]) * 1e+6
		tb = int(R8L(raw.Buff[idx+75:]) * 1e-3)
		tk = tb
		geph.Gamn = float64(R4L(raw.Buff[idx+83:]))
		geph.Taun = float64(R4L(raw.Buff[idx+87:])) * 1e-3
		geph.Age = int(I2L(raw.Buff[idx+91:]))
	} else {
		Trace(2, "nvs NE length error: len=%d\n", raw.Len)
		return -1
	}
	if geph.Sat = SatNo(SYS_GLO, prn); geph.Sat == 0 {
		Trace(2, "nvs NE satellite error: prn=%d\n", prn)
		return -1
	}
	if raw.Time.Time == 0 {
		return 0
	}

	geph.Iode = (tb / 900) & 0x7F
	geph.Toe = Utc2GpsT(adjday(raw.Time, float64(tb)-10800.0))
	geph.Tof = Utc2GpsT(adjday(raw.Time, float64(tk)-10800.0))
	// #if 0
	//     /* check illegal ephemeris by toe */
	//     tt=TimeDiff(raw.Time,geph.Toe);
	//     if (math.Abs(tt)>3600.0) {
	//         Trace(3,"nvs NE illegal toe: prn=%2d tt=%6.0f\n",prn,tt);
	//         return 0;
	//     }
	// #endif
	// #if 0
	//     /* check illegal ephemeris by frequency number consistency */
	//     if (raw.NavData.Geph[prn-MINPRNGLO].Toe.Time&&
	//         geph.Frq!=raw.NavData.Geph[prn-MINPRNGLO].Frq) {
	//         Trace(2,"nvs NE illegal freq change: prn=%2d frq=%2d.%2d\n",prn,
	//               raw.NavData.Geph[prn-MINPRNGLO].Frq,geph.Frq);
	//         return -1;
	//     }
	//     if (!strings.Contains(raw.Opt,"-EPHALL")) {
	//         if (math.Abs(TimeDiff(geph.Toe,raw.NavData.Geph[prn-MINPRNGLO].Toe))<1.0&&
	//             geph.svh==raw.NavData.Geph[prn-MINPRNGLO].svh) return 0;
	//     }
	// #endif
	raw.NavData.Geph[prn-1] = geph
	raw.EphSat = geph.Sat
	raw.EphSet = 0

	return 2
}

/* decode NVS ephemerides in clear -------------------------------------------*/
func decode_xf7eph(raw *Raw) int {
	var (
		prn, sat, sys int
		idx           = 0
	)

	Trace(4, "decode_xf7eph: len=%d\n", raw.Len)

	if (raw.Len) < 93 {
		Trace(2, "nvs xf7eph length error: len=%d\n", raw.Len)
		return -1
	}
	var isys = SYS_GLO
	switch U1(raw.Buff[idx+2:]) {
	case 1:
		sys = SYS_GPS
		isys = SYS_GPS
	case 2:
		sys = SYS_GLO
	default:
		sys = SYS_NONE
	}
	prn = int(U1(raw.Buff[idx+3:]))
	if sat = SatNo(isys, prn); sat == 0 {
		Trace(2, "nvs xf7eph satellite number error: prn=%d\n", prn)
		return -1
	}
	if sys == SYS_GPS {
		return decode_gpsephem(sat, raw)
	} else if sys == SYS_GLO {
		return decode_gloephem(sat, raw)
	}
	return 0
}

/* decode NVS rxm-sfrb: subframe buffer --------------------------------------*/
func decode_xe5bit(raw *Raw) int {
	var (
		iBlkStartIdx, iExpLen, iIdx, prn int
		words                            [10]uint32
		uiDataBlocks, uiDataType         uint8
		idx                              = 0
	)
	Trace(4, "decode_xe5bit: len=%d\n", raw.Len)

	idx += 2 /* Discard preamble and message identifier */
	uiDataBlocks = U1(raw.Buff[idx:])

	if uiDataBlocks >= 16 {
		Trace(2, "nvs xf5bit message error: data blocks %u\n", uiDataBlocks)
		return -1
	}
	iBlkStartIdx = 1
	for iIdx = 0; iIdx < int(uiDataBlocks); iIdx++ {
		iExpLen = (iBlkStartIdx + 10)
		if (raw.Len) < iExpLen {
			Trace(2, "nvs xf5bit message too short (expected at least %d)\n", iExpLen)
			return -1
		}
		uiDataType = U1(raw.Buff[idx+iBlkStartIdx+1:])

		switch uiDataType {
		case 1: /* Glonass */
			iBlkStartIdx += 19
		case 2: /* GPS */
			iBlkStartIdx += 47
		case 4: /* SBAS */
			prn = int(U1(raw.Buff[idx+iBlkStartIdx+2:])) + 120

			/* sat = SatNo(SYS_SBS, prn); */
			/* sys = satsys(sat,&prn); */
			for k := 0; k < 10; k++ {
				words[k] = 0
			}
			for iIdx, iBlkStartIdx = 0, iBlkStartIdx+7; iIdx < 10; iIdx, iBlkStartIdx = iIdx+1, iBlkStartIdx+4 {
				words[iIdx] = U4L(raw.Buff[idx+iBlkStartIdx:])
			}
			words[7] >>= 6
			if SbsDecodeMsg(raw.Time, prn, words[:], &raw.Sbsmsg) > 0 {
				return 3
			} else {
				return 0
			}
		default:
			Trace(2, "nvs xf5bit SNS type unknown (got %d)\n", uiDataType)
			return -1
		}
	}
	return 0
}

/* decode NVS x4aiono --------------------------------------------------------*/
func decode_x4aiono(raw *Raw) int {
	var idx = 2

	Trace(4, "decode_x4aiono: len=%d\n", raw.Len)

	raw.NavData.Ion_gps[0] = float64(R4L(raw.Buff[idx:]))
	raw.NavData.Ion_gps[1] = float64(R4L(raw.Buff[idx+4:]))
	raw.NavData.Ion_gps[2] = float64(R4L(raw.Buff[idx+8:]))
	raw.NavData.Ion_gps[3] = float64(R4L(raw.Buff[idx+12:]))
	raw.NavData.Ion_gps[4] = float64(R4L(raw.Buff[idx+16:]))
	raw.NavData.Ion_gps[5] = float64(R4L(raw.Buff[idx+20:]))
	raw.NavData.Ion_gps[6] = float64(R4L(raw.Buff[idx+24:]))
	raw.NavData.Ion_gps[7] = float64(R4L(raw.Buff[idx+28:]))

	return 9
}

/* decode NVS x4btime --------------------------------------------------------*/
func decode_x4btime(raw *Raw) int {
	var idx = 2

	Trace(4, "decode_x4btime: len=%d\n", raw.Len)

	raw.NavData.Utc_gps[1] = R8L(raw.Buff[idx:])
	raw.NavData.Utc_gps[0] = R8L(raw.Buff[idx+8:])
	raw.NavData.Utc_gps[2] = float64(I4L(raw.Buff[idx+16:]))
	raw.NavData.Utc_gps[3] = float64(I2L(raw.Buff[idx+20:]))
	raw.NavData.Utc_gps[4] = float64(I1(raw.Buff[idx+22:]))

	return 9
}

/* decode NVS raw message ----------------------------------------------------*/
func decode_nvs(raw *Raw) int {
	ctype := U1(raw.Buff[1:])

	Trace(3, "decode_nvs: type=%02x len=%d\n", ctype, raw.Len)

	copy(raw.MsgType[len(string(raw.MsgType[:])):], []byte(fmt.Sprintf("NVS: type=%2d len=%3d", ctype, raw.Len)))

	switch ctype {
	case ID_XF5RAW:
		return decode_xf5raw(raw)
	case ID_XF7EPH:
		return decode_xf7eph(raw)
	case ID_XE5BIT:
		return decode_xe5bit(raw)
	case ID_X4AIONO:
		return decode_x4aiono(raw)
	case ID_X4BTIME:
		return decode_x4btime(raw)
	default:
		break
	}
	return 0
}

/* input NVS raw message from stream -------------------------------------------
* fetch next NVS raw data and input a message from stream
* args   : raw *Raw   IO    receiver raw data control struct
*          uint8 data I     stream data (1 byte)
* return : status (-1: error message, 0: no message, 1: input observation data,
*                  2: input ephemeris, 3: input sbas message,
*                  9: input ion/utc parameter)
*
* notes  : to specify input options, set raw.Opt to the following option
*          strings separated by spaces.
*
*          -EPHALL    : input all ephemerides
*          -TADJ=tint : adjust time tags to multiples of tint (sec)
*
*-----------------------------------------------------------------------------*/
func Input_nvs(raw *Raw, data uint8) int {
	Trace(5, "input_nvs: data=%02x\n", data)

	/* synchronize frame */
	if (raw.NumByte == 0) && (data == NVSSYNC) {

		/* Search a 0x10 */
		raw.Buff[0] = data
		raw.NumByte = 1
		return 0
	}
	if (raw.NumByte == 1) && (data != NVSSYNC) && (data != NVSENDMSG) {

		/* Discard float64 0x10 and 0x10 0x03 at beginning of frame */
		raw.Buff[1] = data
		raw.NumByte = 2
		raw.Flag = 0
		return 0
	}
	/* This is all done to discard a float64 0x10 */
	if data == NVSSYNC {
		raw.Flag = (raw.Flag + 1) % 2
	}
	if (data != NVSSYNC) || (raw.Flag > 0) {

		/* Store the new byte */
		raw.Buff[(raw.NumByte)] = data
		raw.NumByte++
	}
	/* Detect ending sequence */
	if (data == NVSENDMSG) && (raw.Flag > 0) {
		raw.Len = raw.NumByte
		raw.NumByte = 0

		/* Decode NVS raw message */
		return decode_nvs(raw)
	}
	if raw.NumByte == MAXRAWLEN {
		Trace(2, "nvs message size error: len=%d\n", raw.NumByte)
		raw.NumByte = 0
		return -1
	}
	return 0
}

/* input NVS raw message from file ---------------------------------------------
* fetch next NVS raw data and input a message from file
* args   : raw_t  *raw  IO    receiver raw data control struct
*          FILE   *fp   I     file pointer
* return : status(-2: end of file, -1...9: same as above)
*-----------------------------------------------------------------------------*/
func Input_nvsf(raw *Raw, fp *os.File) int {
	var i, odd int

	Trace(4, "input_nvsf:\n")

	var c [1]byte
	/* synchronize frame */
	for i = 0; ; i++ {
		_, err := fp.Read(c[:])
		if err == io.EOF {
			return -2
		}

		/* Search a 0x10 */
		if c[0] == NVSSYNC {

			/* Store the frame begin */
			raw.Buff[0] = c[0]
			_, err := fp.Read(c[:])
			if err == io.EOF {
				return -2
			}

			/* Discard float64 0x10 and 0x10 0x03 */
			if (c[0] != NVSSYNC) && (c[0] != NVSENDMSG) {
				raw.Buff[1] = c[0]
				break
			}
		}
		if i >= 4096 {
			return 0
		}
	}
	raw.NumByte = 2
	for i = 0; ; i++ {
		_, err := fp.Read(c[:])
		if err == io.EOF {
			return -2
		}
		if c[0] == NVSSYNC {
			odd = (odd + 1) % 2
		}
		if (c[0] != NVSSYNC) || odd > 0 {

			/* Store the new byte */
			raw.Buff[(raw.NumByte)] = c[0]
			raw.NumByte++
		}
		/* Detect ending sequence */
		if (c[0] == NVSENDMSG) && odd > 0 {
			break
		}
		if i >= 4096 {
			return 0
		}
	}
	raw.Len = raw.NumByte
	if (raw.Len) > MAXRAWLEN {
		Trace(2, "nvs length error: len=%d\n", raw.Len)
		return -1
	}
	/* decode nvs raw message */
	return decode_nvs(raw)
}

/* generate NVS binary message -------------------------------------------------
* generate NVS binary message from message string
* args   : char  *msg   I      message string
*            "RESTART  [arg...]" system reset
*            "CFG-SERI [arg...]" configure serial port property
*            "CFG-FMT  [arg...]" configure output message format
*            "CFG-RATE [arg...]" configure binary measurement output rates
*          uint8 *buff O binary message
* return : length of binary message (0: error)
* note   : see reference [1][2] for details.
*-----------------------------------------------------------------------------*/
func gen_nvs(msg string, buff []uint8) int {
	var (
		b             uint32
		iRate, n, idx int
		ui100Ms       uint8
	)

	Trace(4, "gen_nvs: msg=%s\n", msg)

	args := strings.Split(msg, " ")
	if len(args) < 1 {
		return 0
	}
	buff[idx] = NVSSYNC /* DLE */
	idx++

	switch args[0] {
	case "CFG-PVTRATE":
		{
			buff[idx] = ID_XD7ADVANCED
			idx++
			buff[idx] = ID_X02RATEPVT
			idx++
			if len(args) > 1 {
				iRate, _ = strconv.Atoi(args[1])
				buff[idx] = uint8(iRate)
				idx++
			}
		}
	case "CFG-RAWRATE":
		{
			buff[idx] = ID_XF4RATERAW
			idx++
			if len(args) > 1 {
				iRate, _ = strconv.Atoi(args[1])
				switch iRate {
				case 2:
					ui100Ms = 5
				case 5:
					ui100Ms = 2
				case 10:
					ui100Ms = 1
				default:
					ui100Ms = 10
				}
				buff[idx] = ui100Ms
				idx++
			}
		}
	case "CFG-SMOOTH":
		{
			buff[idx] = ID_XD7SMOOTH
			idx++
			buff[idx] = 0x03
			idx++
			buff[idx] = 0x01
			idx++
			buff[idx] = 0x00
			idx++
		}
	case "CFG-BINR":
		{
			for n = 1; n < len(args); n++ {
				if bn, _ := fmt.Sscanf(args[n], "%2x", &b); bn > 0 {
					buff[idx] = uint8(b)
					idx++
				}
			}
		}
	default:
		return 0
	}
	n = idx

	buff[idx] = 0x10 /* ETX */
	idx++
	buff[idx] = 0x03 /* DLE */
	return n + 2
}
