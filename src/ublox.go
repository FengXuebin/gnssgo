/*------------------------------------------------------------------------------
* ublox.c : ublox receiver dependent functions
*
*          Copyright (C) 2007-2020 by T.TAKASU, All rights reserved.
*          Copyright (C) 2014 by T.SUZUKI, All rights reserved.
*          Copyright (C) 2022 by Feng Xuebin, All rights reserved.
*
* reference :
*     [1] ublox-AG, GPS.G3-X-03002-D, ANTARIS Positioning Engine NMEA and UBX
*         Protocol Specification, Version 5.00, 2003
*     [2] ublox-AG, UBX-13003221-R03, u-blox M8 Receiver Description including
*         Protocol Specification V5, Dec 20, 2013
*     [3] ublox-AG, UBX-13003221-R07, u-blox M8 Receiver Description including
*         Protocol Specification V15.00-17.00, Nov 3, 2014
*     [4] ublox-AG, UBX-13003221-R09, u-blox 8 /u-blox M8 Receiver Description
*         including Protocol Specification V15.00-18.00, January, 2016
*     [5] ublox-AG, UBX-18010854-R08, u-blox ZED-F9P Interface Description,
*         May, 2020
*
* version : $Revision: 1.2 $ $Date: 2008/07/14 00:05:05 $
* history : 2007/10/08 1.0  new
*           2008/06/16 1.1  separate common functions to rcvcmn.c
*           2009/04/01 1.2  add range check of prn number
*           2009/04/10 1.3  refactored
*           2009/09/25 1.4  add function gen_ubx()
*           2010/01/17 1.5  add time tag adjustment option -tadj sec
*           2010/10/31 1.6  fix bug on playback disabled for raw data (2.4.0_p9)
*           2011/05/27 1.7  add almanac decoding
*                           add -EPHALL option
*                           fix problem with ARM compiler
*           2013/02/23 1.8  fix memory access violation problem on arm
*                           change options -tadj to -TADJ, -invcp to -INVCP
*           2014/05/26 1.9  fix bug on message size of CFG-MSG
*                           fix bug on return code of decode_alm1()
*           2014/06/21 1.10 support message TRK-MEAS and TRK-SFRBX
*                           support message NAV-SOL and NAV-TIMEGPS to get time
*                           support message GFG-GNSS generation
*           2014/06/23 1.11 support message TRK-MEAS for beidou ephemeris
*           2014/08/11 1.12 fix bug on unable to read RXM-RAW
*                           fix problem on decoding glo ephemeris in TRK-SFRBX
*                           support message TRK-TRKD5
*           2014/08/31 1.13 suppress warning
*           2014/11/04 1.14 support message RXM-RAWX and RXM-SFRBX
*           2015/03/20 1.15 omit time adjustment for RXM-RAWX
*           2016/01/22 1.16 add time-tag in raw-message-type
*           2016/01/26 1.17 support galileo navigation data in RXM-SFRBX
*                           enable option -TADJ for RXM-RAWX
*           2016/05/25 1.18 fix bug on crc-buffer-overflow by decoding galileo
*                           navigation data
*           2016/07/04 1.19 add half-cycle vaild check for ubx-trk-meas
*           2016/07/29 1.20 support RXM-CFG-TMODE3 (0x06 0x71) for M8P
*                           crc24q() . Rtk_crc24q()
*                           check week number zero for ubx-rxm-raw and rawx
*           2016/08/20 1.21 add test of std-dev for carrier-phase valid
*           2016/08/26 1.22 add option -STD_SLIP to test slip by std-dev of cp
*                           fix on half-cyc valid for sbas in trkmeas
*           2017/04/11 1.23 (char *) . (signed char *)
*                           fix bug on week handover in decode_trkmeas/trkd5()
*                           fix bug on prn for geo in decode_cnav()
*           2017/06/10 1.24 output half-cycle-subtracted flag
*           2018/10/09 1.25 support ZED-F9P according to [5]
*                           beidou C17 is handled as GEO (navigation D2).
*           2018/11/05 1.26 fix problem on missing QZSS L2C signal
*                           save signal in obs data by signal index
*                           suppress warning for cnav in ubx-rxm-sfrbx
*           2019/05/10 1.27 disable half-cyc-subtract flag on LLI for RXM-RAWX
*                           save galileo E5b data to obs index 2
*                           handle C17 as no-GEO (MEO/IGSO)
*           2020/11/30 1.28 update reference [5]
*                           support UBX-CFG-VALDEL,VALGET,VALSET
*                           support hex field format for ubx binary message
*                           add quality test for receiver time in decode_trkd5()
*                           add half cycle shift correction for BDS GEO
*                           delete receiver option -GALFNAV
*                           use API code2idx() and Code2Freq()
*                           support QZSS L1S (CODE_L1Z)
*                           CODE_L1I . CODE_L2I for BDS B1I (RINEX 3.04)
*                           use integer types in stdint.h
*           2022/09/26 1.29 rewrite with golang
*-----------------------------------------------------------------------------*/
package gnssgo

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"io"
	"math"
	"os"
	"strconv"
	"strings"
)

const (
	UBXSYNC1 = 0xB5 /* ubx message sync code 1 */
	UBXSYNC2 = 0x62 /* ubx message sync code 2 */
	UBXCFG   = 0x06 /* ubx message cfg-??? */
	//	PREAMB_CNAV =0x8B        /* cnav preamble */
	ID_NAVSOL   = 0x0106 /* ubx message id: nav solution info */
	ID_NAVTIME  = 0x0120 /* ubx message id: nav time gps */
	ID_RXMRAW   = 0x0210 /* ubx message id: raw measurement data */
	ID_RXMSFRB  = 0x0211 /* ubx message id: subframe buffer */
	ID_RXMSFRBX = 0x0213 /* ubx message id: raw subframe data */
	ID_RXMRAWX  = 0x0215 /* ubx message id: multi-gnss raw meas data */
	ID_TRKD5    = 0x030A /* ubx message id: Trace mesurement data */
	ID_TRKMEAS  = 0x0310 /* ubx message id: Trace mesurement data */
	ID_TRKSFRBX = 0x030F /* ubx message id: Trace subframe buffer */
	FU1         = 1      /* ubx message field types */
	FU2         = 2
	FU4         = 3
	FI1         = 4
	FI2         = 5
	FI4         = 6
	FR4         = 7
	FR8         = 8
	FS32        = 9
	//	P2_10       =0.0009765625 /* 2^-10 */
	CPSTD_VALID = 5 /* std-dev threshold of carrier-phase valid */
)

// declared in rcm3e.go, renamed as ROUND_I
// #define ROUND(x)    (int)math.Floor((x)+0.5)

/* get fields (little-endian) ------------------------------------------------*/
// declared in cresent.go, rename it as xxL
// #define U1(p) (*((uint8 *)(p)))
// #define I1(p) (*((int8_t  *)(p)))
// static uint16 U2L(uint8 *p) {uint16 u; memcpy(&u,p,2); return u;}
// static uint32 U4L(uint8 *p) {uint32 u; memcpy(&u,p,4); return u;}
// static int32_t  I4L(uint8 *p) {int32_t  u; memcpy(&u,p,4); return u;}
// static float    R4L(uint8 *p) {float    r; memcpy(&r,p,4); return r;}
// static float64   R8L(uint8 *p) {float64   r; memcpy(&r,p,8); return r;}
func I8L(p []uint8) float64 { return float64(I4L(p[4:]))*4294967296.0 + float64(U4L(p[:])) }

/* set fields (little-endian) ------------------------------------------------*/
func setU1(p []uint8, u uint8) {
	p[0] = u
}
func setU2(p []uint8, u uint16) {
	binary.LittleEndian.PutUint16(p, u)
}
func setU4(p []uint8, u uint32) {
	binary.LittleEndian.PutUint32(p, u)
}
func setI1(p []uint8, u int8) {
	buff := bytes.NewBuffer(p)
	binary.Write(buff, binary.LittleEndian, u)
}
func setI2(p []uint8, u int16) {
	buff := bytes.NewBuffer(p)
	binary.Write(buff, binary.LittleEndian, u)

}
func setI4(p []uint8, u int32) {
	buff := bytes.NewBuffer(p)
	binary.Write(buff, binary.LittleEndian, u)

}
func setR4(p []uint8, u float32) {
	buff := bytes.NewBuffer(p)
	binary.Write(buff, binary.LittleEndian, u)

}
func setR8(p []uint8, u float64) {
	buff := bytes.NewBuffer(p)
	binary.Write(buff, binary.LittleEndian, u)

}

/* checksum ------------------------------------------------------------------*/
func checksum_ublox(buff []uint8, length int) int {
	var cka, ckb uint8

	for i := 2; i < length-2; i++ {
		cka += buff[i]
		ckb += cka
	}
	if cka == buff[length-2] && ckb == buff[length-1] {
		return 1
	}
	return 0
}
func setcs(buff []uint8, length int) {
	var cka, ckb uint8

	for i := 2; i < length-2; i++ {
		cka += buff[i]
		ckb += cka
	}
	buff[length-2] = cka
	buff[length-1] = ckb
}

/* UBX GNSSId to system (ref [2] 25) -----------------------------------------*/
func ubx_sys(gnssid int) int {
	switch gnssid {
	case 0:
		return SYS_GPS
	case 1:
		return SYS_SBS
	case 2:
		return SYS_GAL
	case 3:
		return SYS_CMP
	case 5:
		return SYS_QZS
	case 6:
		return SYS_GLO
	}
	return 0
}

/* UBX SigId to signal (ref [5] 1.5.4) ---------------------------------------*/
func ubx_sig(sys, sigid int) int {
	switch sys {
	case SYS_GPS:
		{
			switch sigid {
			case 0:
				return CODE_L1C /* L1C/A */
			case 3:
				return CODE_L2L /* L2CL */
			case 4:
				return CODE_L2S /* L2CM */
			}
		}
	case SYS_GLO:
		{
			switch sigid {
			case 0:
				return CODE_L1C /* G1C/A (GLO L1 OF: */
			case 2:
				return CODE_L2C /* G2C/A (GLO L2 OF) */
			}
		}
	case SYS_GAL:
		{
			switch sigid {
			case 0:
				return CODE_L1C /* E1C */
			case 1:
				return CODE_L1B /* E1B */
			case 5:
				return CODE_L7I /* E5bI */
			case 6:
				return CODE_L7Q /* E5bQ */
			}
		}
	case SYS_QZS:
		{
			switch sigid {
			case 0:
				return CODE_L1C /* L1C/A */
			case 1:
				return CODE_L1Z /* L1S */
			case 4:
				return CODE_L2S /* L2CM */
			case 5:
				return CODE_L2L /* L2CL */
			}
		}
	case SYS_CMP:
		{
			switch sigid {
			case 0:
				return CODE_L2I /* B1I D1 */
			case 1:
				return CODE_L2I /* B1I D2 */
			case 2:
				return CODE_L7I /* B2I D1 */
			case 3:
				return CODE_L7I /* B2I D2 */
			}
		}
	case SYS_SBS:
		{
			switch sigid {
			case 0:
				return CODE_L1C /* L1C/A */
			}
		}
	}
	return CODE_NONE
}

/* signal index in obs data --------------------------------------------------*/
func sig_idx(sys int, code uint8) int {
	idx := Code2Idx(sys, code)
	nex := NEXOBS

	switch sys {
	case SYS_GPS:
		{
			if code == CODE_L2S {
				if nex < 1 {
					return -1
				} else {
					return NFREQ
				}
			} /* L2CM */
		}
	case SYS_GAL:
		{
			if code == CODE_L1B {
				if nex < 1 {
					return -1
				} else {
					return NFREQ
				}
			} /* E1B */
			if code == CODE_L7I {
				if nex < 2 {
					return -1
				} else {
					return NFREQ + 1
				}
			} /* E5bI */
		}
	case SYS_QZS:
		{
			if code == CODE_L2S {
				if nex < 1 {
					return -1
				} else {
					return NFREQ
				}
			} /* L2CM */
			if code == CODE_L1Z {
				if nex < 2 {
					return -1
				} else {
					return NFREQ + 1
				}
			} /* L1S */
		}
	}
	if idx < NFREQ {
		return idx
	}
	return -1
}

/* decode UBX-RXM-RAW: raw measurement data ----------------------------------*/
func decode_rxmraw(raw *Raw) int {
	var (
		p                                = 6
		time                             Gtime
		tow, tt, tadj, toff, tn          float64
		i, j, prn, sat, n, nsat, week, q int
	)

	if raw.OutType > 0 {
		copy(raw.MsgType[:], []byte(fmt.Sprintf("UBX RXM-RAW   (%4d): nsat=%d", raw.Len, U1(raw.Buff[p+6:]))))
	}
	/* time tag adjustment option (-TADJ) */
	if q = strings.Index(raw.Opt, "-TADJ="); q >= 0 {
		fmt.Sscanf(raw.Opt[q:], "-TADJ=%lf", &tadj)
	}
	nsat = int(U1(raw.Buff[p+6:]))
	if raw.Len < 12+24*nsat {
		Trace(2, "ubx rxmraw length error: len=%d nsat=%d\n", raw.Len, nsat)
		return -1
	}
	tow = float64(U4L(raw.Buff[p:]))
	week = int(U2L(raw.Buff[p+4:]))
	time = GpsT2Time(week, tow*0.001)

	if week == 0 {
		Trace(3, "ubx rxmraw week=0 error: len=%d nsat=%d\n", raw.Len, nsat)
		return 0
	}
	/* time tag adjustment */
	if tadj > 0.0 {
		tn = Time2GpsT(time, &week) / tadj
		toff = (tn - math.Floor(tn+0.5)) * tadj
		time = TimeAdd(time, -toff)
	}
	tt = TimeDiff(time, raw.Time)

	for i, p = 0, p+8; i < nsat && i < MAXOBS; i, p = i+1, p+24 {
		raw.ObsData.Data[n].Time = time
		raw.ObsData.Data[n].L[0] = R8L(raw.Buff[p:]) - toff*FREQ1
		raw.ObsData.Data[n].P[0] = R8L(raw.Buff[p+8:]) - toff*CLIGHT
		raw.ObsData.Data[n].D[0] = float64(R4L(raw.Buff[p+16:]))
		prn = int(U1(raw.Buff[p+20:]))
		raw.ObsData.Data[n].SNR[0] = uint16(float64(I1(raw.Buff[p+22:]))*1.0/SNR_UNIT + 0.5)
		raw.ObsData.Data[n].LLI[0] = U1(raw.Buff[p+23:])
		raw.ObsData.Data[n].Code[0] = CODE_L1C

		/* phase polarity flip option (-INVCP) */
		if strings.Contains(raw.Opt, "-INVCP") {
			raw.ObsData.Data[n].L[0] = -raw.ObsData.Data[n].L[0]
		}
		sys := SYS_GPS
		if MINPRNSBS <= prn {
			sys = SYS_SBS
		}
		if sat = SatNo(sys, prn); sat == 0 {
			Trace(2, "ubx rxmraw sat number error: prn=%d\n", prn)
			continue
		}
		raw.ObsData.Data[n].Sat = sat

		if raw.ObsData.Data[n].LLI[0]&1 > 0 {
			raw.LockTime[sat-1][0] = 0.0
		} else if tt < 1.0 || 10.0 < tt {
			raw.LockTime[sat-1][0] = 0.0
		} else {
			raw.LockTime[sat-1][0] += tt
		}

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

/* decode UBX-RXM-RAWX: multi-GNSS raw measurement data (ref [3][4][5]) ------*/
func decode_rxmrawx(raw *Raw) int {
	var (
		p    = 6
		time Gtime
		//char *q,tstr[64];
		tow, P, L, D, tn, tadj, toff                                            float64
		i, j, k, idx, sys, prn, sat, code, slip, halfv, halfc, LLI, n, std_slip int
		week, nmeas, ver, gnss, svid, sigid, frqid, lockt, cn0, cpstd, tstat    int
		tstr                                                                    string
	)

	if raw.Len < 24 {
		Trace(2, "ubx rxmrawx length error: len=%d\n", raw.Len)
		return -1
	}
	tow = R8L(raw.Buff[p:])          /* rcvTow (s) */
	week = int(U2L(raw.Buff[p+8:]))  /* week */
	nmeas = int(U1(raw.Buff[p+11:])) /* numMeas */
	ver = int(U1(raw.Buff[p+13:]))   /* version ([5] 5.15.3.1) */

	if raw.Len < 24+32*nmeas {
		Trace(2, "ubx rxmrawx length error: len=%d nmeas=%d\n", raw.Len, nmeas)
		return -1
	}
	if week == 0 {
		Trace(3, "ubx rxmrawx week=0 error: len=%d nmeas=%d\n", raw.Len, nmeas)
		return 0
	}
	time = GpsT2Time(week, tow)

	if raw.OutType > 0 {
		Time2Str(time, &tstr, 2)
		copy(raw.MsgType[:], []byte(fmt.Sprintf("UBX RXM-RAWX  (%4d): time=%s nmeas=%d ver=%d",
			raw.Len, tstr, nmeas, ver)))
	}
	/* time tag adjustment option (-TADJ) */
	if q := strings.Index(raw.Opt, "-TADJ="); q >= 0 {
		fmt.Sscanf(raw.Opt[q:], "-TADJ=%lf", &tadj)
	}
	/* slip threshold of std-dev of carreir-phase (-STD_SLIP) */
	if q := strings.Index(raw.Opt, "-STD_SLIP="); q >= 0 {
		fmt.Sscanf(raw.Opt[q:], "-STD_SLIP=%d", &std_slip)
	}
	/* time tag adjustment */
	if tadj > 0.0 {
		tn = Time2GpsT(time, &week) / tadj
		toff = (tn - math.Floor(tn+0.5)) * tadj
		time = TimeAdd(time, -toff)
	}
	for i, p = 0, p+16; i < nmeas && n < MAXOBS; i, p = i+1, p+32 {
		P = R8L(raw.Buff[p:])                 /* prMes (m) */
		L = R8L(raw.Buff[p+8:])               /* cpMes (cyc) */
		D = float64(R4L(raw.Buff[p+16:]))     /* doMes (hz) */
		gnss = int(U1(raw.Buff[p+20:]))       /* gnssId */
		svid = int(U1(raw.Buff[p+21:]))       /* svId */
		sigid = int(U1(raw.Buff[p+22:]))      /* sigId ([5] 5.15.3.1) */
		frqid = int(U1(raw.Buff[p+23:]))      /* freqId (fcn + 7) */
		lockt = int(U2L(raw.Buff[p+24:]))     /* locktime (ms) */
		cn0 = int(U1(raw.Buff[p+26:]))        /* cn0 (dBHz) */
		cpstd = int(U1(raw.Buff[p+28:])) & 15 /* cpStdev (m) */
		tstat = int(U1(raw.Buff[p+30:]))      /* trkStat */
		if tstat&1 == 0 {
			P = 0.0
		}
		if (tstat&2) == 0 || L == -0.5 || cpstd > CPSTD_VALID {
			L = 0.0
		}

		if sys = ubx_sys(gnss); sys == 0 {
			Trace(2, "ubx rxmrawx: system error gnss=%d\n", gnss)
			continue
		}
		prn = svid
		if sys == SYS_QZS {
			prn = svid + 192
		}
		if sat = SatNo(sys, prn); sat == 0 {
			if sys == SYS_GLO && prn == 255 {
				continue /* suppress warning for unknown glo satellite */
			}
			Trace(2, "ubx rxmrawx sat number error: sys=%2d prn=%2d\n", sys, prn)
			continue
		}
		if sys == SYS_GLO && raw.NavData.Glo_fcn[prn-1] == 0 {
			raw.NavData.Glo_fcn[prn-1] = frqid - 7 + 8
		}
		if ver >= 1 {
			code = ubx_sig(sys, sigid)
		} else {
			switch sys {
			case SYS_CMP:
				code = CODE_L2I
			case SYS_GAL:
				code = CODE_L1X
			default:
				code = CODE_L1C
			}
		}
		/* signal index in obs data */
		if idx = sig_idx(sys, uint8(code)); idx < 0 {
			Trace(2, "ubx rxmrawx signal error: sat=%2d sigid=%d\n", sat, sigid)
			continue
		}
		/* offset by time tag adjustment */
		if toff != 0.0 {
			P -= toff * CLIGHT
			L -= toff * Code2Freq(sys, uint8(code), frqid-7)
		}
		/* half-cycle shift correction for BDS GEO */
		if sys == SYS_CMP && (prn <= 5 || prn >= 59) && L != 0.0 {
			L += 0.5
		}
		if tstat&4 > 0 {
			halfv = 1
		} else {
			halfv = 0
		} /* half cycle valid */
		if tstat&8 > 0 {
			halfc = 1
		} else {
			halfc = 0
		} /* half cycle subtracted from phase */
		if lockt == 0 || float64(lockt)*1e-3 < raw.LockTime[sat-1][idx] ||
			halfc != int(raw.Halfc[sat-1][idx]) || (std_slip > 0 && cpstd >= std_slip) {
			slip = 1
		} else {
			slip = 0
		}
		raw.LockTime[sat-1][idx] = float64(lockt) * 1e-3
		raw.Halfc[sat-1][idx] = uint8(halfc)
		if slip > 0 {
			LLI = LLI_SLIP
		} else {
			LLI = 0
		}
		if halfv == 0 {
			LLI |= LLI_HALFC
		}
		if halfc > 0 {
			LLI |= LLI_HALFS
		}

		for j = 0; j < n; j++ {
			if raw.ObsData.Data[j].Sat == sat {
				break
			}
		}
		if j >= n {
			raw.ObsData.Data[n].Time = time
			raw.ObsData.Data[n].Sat = sat
			raw.ObsData.Data[n].Rcv = 0
			for k = 0; k < NFREQ+NEXOBS; k++ {
				raw.ObsData.Data[n].L[k], raw.ObsData.Data[n].P[k] = 0.0, 0.0
				raw.ObsData.Data[n].D[k] = 0.0
				raw.ObsData.Data[n].SNR[k], raw.ObsData.Data[n].LLI[k] = 0, 0
				raw.ObsData.Data[n].Code[k] = CODE_NONE
			}
			n++
		}
		raw.ObsData.Data[j].L[idx] = L
		raw.ObsData.Data[j].P[idx] = P
		raw.ObsData.Data[j].D[idx] = D
		raw.ObsData.Data[j].SNR[idx] = uint16(float64(cn0)*1.0/SNR_UNIT + 0.5)
		raw.ObsData.Data[j].LLI[idx] = uint8(LLI)
		raw.ObsData.Data[j].Code[idx] = uint8(code)
	}
	raw.Time = time
	raw.ObsData.n = n
	return 1
}

/* decode UBX-NAV-SOL: navigation solution -----------------------------------*/
func decode_navsol(raw *Raw) int {
	var (
		p                = 6
		itow, ftow, week int
	)

	Trace(4, "decode_navsol: len=%d\n", raw.Len)

	if raw.OutType > 0 {
		copy(raw.MsgType[:], []byte(fmt.Sprintf("UBX NAV-SOL   (%4d):", raw.Len)))
	}
	itow = int(U4L(raw.Buff[p:]))
	ftow = int(I4L(raw.Buff[p+4:]))
	week = int(U2L(raw.Buff[p+8:]))
	if (U1(raw.Buff[p+11:]) & 0x0C) == 0x0C {
		raw.Time = GpsT2Time(week, float64(itow)*1e-3+float64(ftow)*1e-9)
	}
	return 0
}

/* decode UBX-NAV-TIMEGPS: GPS time solution ---------------------------------*/
func decode_navtime(raw *Raw) int {
	var (
		itow, ftow, week int
		p                = 6
	)

	Trace(4, "decode_navtime: len=%d\n", raw.Len)

	if raw.OutType > 0 {
		copy(raw.MsgType[:], []byte(fmt.Sprintf("UBX NAV-TIME  (%4d):", raw.Len)))
	}
	itow = int(U4L(raw.Buff[p:]))
	ftow = int(I4L(raw.Buff[p+4:]))
	week = int(U2L(raw.Buff[p+8:]))
	if (U1(raw.Buff[p+11:]) & 0x03) == 0x03 {
		raw.Time = GpsT2Time(week, float64(itow)*1e-3+float64(ftow)*1e-9)
	}
	return 0
}

/* decode UBX-TRK-MEAS: Trace measurement data (unofficial) ------------------*/
func decode_trkmeas(raw *Raw) int {
	var (
		adrs                                               [MAXSAT]float64
		p                                                  = 6
		time                                               Gtime
		ts, tr, t, tau, utc_gpst, snr, adr, dop            float64
		i, j, n, nch, sys, prn, sat, qi, flag, lock2, week int
	)
	tr = -1.0
	Trace(4, "decode_trkmeas: len=%d\n", raw.Len)

	if raw.OutType > 0 {
		copy(raw.MsgType[:], []byte(fmt.Sprintf("UBX TRK-MEAS  (%4d):", raw.Len)))
	}
	if raw.Time.Time == 0 {
		return 0
	}

	/* number of channels */
	nch = int(U1(raw.Buff[p+2:]))

	if raw.Len < 112+nch*56 {
		Trace(2, "decode_trkmeas: length error len=%d nch=%2d\n", raw.Len, nch)
		return -1
	}
	/* time-tag = max(transmission time + 0.08) rounded by 100 ms */
	for i, p = 0, p+110; i < nch; i, p = i+1, p+56 {
		if U1(raw.Buff[p+1:]) < 4 || ubx_sys(int(U1(raw.Buff[p+4:]))) != SYS_GPS {
			continue
		}
		if t = I8L(raw.Buff[p+24:]) * P2_32 / 1000.0; t > tr {
			tr = t
		}
	}
	if tr < 0.0 {
		return 0
	}

	tr = float64(ROUND_I((tr+0.08)/0.1)) * 0.1

	/* adjust week handover */
	t = Time2GpsT(raw.Time, &week)
	if tr < t-302400.0 {
		week++
	} else if tr > t+302400.0 {
		week--
	}
	time = GpsT2Time(week, tr)

	utc_gpst = TimeDiff(GpsT2Utc(time), time)

	for i, p = 0, p+110; i < nch; i, p = i+1, p+56 {

		/* quality indicator (0:idle,1:search,2:aquired,3:unusable, */
		/*                    4:code lock,5,6,7:code/carrier lock) */
		qi = int(U1(raw.Buff[p+1:]))
		if qi < 4 || 7 < qi {
			continue
		}

		/* system and satellite number */
		if sys = ubx_sys(int(U1(raw.Buff[p+4:]))); sys == 0 {
			Trace(2, "ubx trkmeas: system error\n")
			continue
		}
		if sys == SYS_QZS {
			prn = int(U1(raw.Buff[p+5:])) + 192
		} else {
			prn = int(U1(raw.Buff[p+5:]))
		}
		if sat = SatNo(sys, prn); sat == 0 {
			Trace(2, "ubx trkmeas sat number error: sys=%2d prn=%2d\n", sys, prn)
			continue
		}
		/* transmission time */
		ts = I8L(raw.Buff[p+24:]) * P2_32 / 1000.0
		if sys == SYS_CMP {
			ts += 14.0 /* bdt  . gpst */
		} else if sys == SYS_GLO {
			ts -= 10800.0 + utc_gpst /* glot . gpst */
		}

		/* signal travel time */
		tau = tr - ts
		if tau < -302400.0 {
			tau += 604800.0
		} else if tau > 302400.0 {
			tau -= 604800.0
		}

		_ = U1(raw.Buff[p+7:]) - 7       /* frequency */ /* frq */
		flag = int(U1(raw.Buff[p+8:]))   /* tracking status */
		_ = U1(raw.Buff[p+16:])          /* code lock count */ /* lock1 */
		lock2 = int(U1(raw.Buff[p+17:])) /* phase lock count */
		snr = float64(U2L(raw.Buff[p+20:])) / 256.0
		if flag&0x40 > 0 {
			adr = I8L(raw.Buff[p+32:])*P2_32 + 0.5
		} else {
			adr = I8L(raw.Buff[p+32:]) * P2_32
		}
		dop = float64(I4L(raw.Buff[p+40:])) * P2_10 * 10.0

		/* set slip flag */
		if lock2 == 0 || float64(lock2) < raw.LockTime[sat-1][0] {
			raw.LockTime[sat-1][1] = 1.0
		}
		raw.LockTime[sat-1][0] = float64(lock2)

		// #if 0 /* for debug */
		//         Trace(2,"[%2d] qi=%d sys=%d prn=%3d frq=%2d flag=%02X ?=%02X %02X "
		//               "%02X %02X %02X %02X %02X lock=%3d %3d ts=%10.3f snr=%4.1f "
		//               "dop=%9.3f adr=%13.3f %6.3f\n",U1(raw.Buff[p:]),qi,U1(raw.Buff[p:]+4),prn,frq,flag,
		//               U1(raw.Buff[p:]+9),U1(raw.Buff[p:]+10),U1(raw.Buff[p:]+11),U1(raw.Buff[p:]+12),U1(raw.Buff[p:]+13),U1(raw.Buff[p:]+14),U1(raw.Buff[p:]+15),
		//               lock1,lock2,ts,snr,dop,adr,
		//               adrs[sat-1]==0.0||dop==0.0?0.0:(adr-adrs[sat-1])-dop);
		// #endif
		adrs[sat-1] = adr

		/* check phase lock */
		if flag&0x20 == 0 {
			continue
		}

		raw.ObsData.Data[n].Time = time
		raw.ObsData.Data[n].Sat = sat
		raw.ObsData.Data[n].P[0] = tau * CLIGHT
		raw.ObsData.Data[n].L[0] = -adr
		raw.ObsData.Data[n].D[0] = dop
		raw.ObsData.Data[n].SNR[0] = (uint16)(snr/SNR_UNIT + 0.5)
		if sys == SYS_CMP {
			raw.ObsData.Data[n].Code[0] = CODE_L2I
		} else {
			raw.ObsData.Data[n].Code[0] = CODE_L1C
		}
		if raw.LockTime[sat-1][1] > 0.0 {
			raw.ObsData.Data[n].LLI[0] = 1
		} else {
			raw.ObsData.Data[n].LLI[0] = 0
		}
		if sys == SYS_SBS { /* half-cycle valid */
			if lock2 > 142 {
				raw.ObsData.Data[n].LLI[0] |= 0
			} else { /* half-cycle valid */
				raw.ObsData.Data[n].LLI[0] |= 2
			}
		} else {
			if flag&0x80 > 0 {
				raw.ObsData.Data[n].LLI[0] |= 0
			} else {
				raw.ObsData.Data[n].LLI[0] |= 2
			}
		}
		raw.LockTime[sat-1][1] = 0.0

		for j = 1; j < NFREQ+NEXOBS; j++ {
			raw.ObsData.Data[n].L[j], raw.ObsData.Data[n].P[j] = 0.0, 0.0
			raw.ObsData.Data[n].D[j] = 0.0
			raw.ObsData.Data[n].SNR[j], raw.ObsData.Data[n].LLI[j] = 0, 0
			raw.ObsData.Data[n].Code[j] = CODE_NONE
		}
		n++
	}
	if n <= 0 {
		return 0
	}
	raw.Time = time
	raw.ObsData.n = n
	return 1
}

/* decode UBX-TRKD5: Trace measurement data (unofficial) ---------------------*/
func decode_trkd5(raw *Raw) int {
	var (
		adrs                                                       [MAXSAT]float64
		time                                                       Gtime
		ts, tr, t, tau, adr, dop, snr, utc_gpst                    float64
		i, j, n, ctype, off, length, sys, prn, sat, qi, flag, week int
		p                                                          = 6
	)
	tr = -1.0
	Trace(4, "decode_trkd5: len=%d\n", raw.Len)

	if raw.OutType > 0 {
		copy(raw.MsgType[:], []byte(fmt.Sprintf("UBX TRK-D5    (%4d):", raw.Len)))
	}
	if raw.Time.Time == 0 {
		return 0
	}

	utc_gpst = TimeDiff(GpsT2Utc(raw.Time), raw.Time)

	switch ctype = int(U1(raw.Buff[p:])); ctype {
	case 3:
		off = 86
		length = 56
		break
	case 6:
		off = 86
		length = 64
		break /* u-blox 7 */
	default:
		off = 78
		length = 56
		break
	}
	for i, p = 0, off; p < raw.Len-2; i, p = i+1, p+length {
		qi = int(U1(raw.Buff[p+41:])) & 7
		if qi < 4 || 7 < qi {
			continue
		}
		t = I8L(raw.Buff[p:]) * P2_32 / 1000.0
		if ubx_sys(int(U1(raw.Buff[p+56:]))) == SYS_GLO {
			t -= 10800.0 + utc_gpst
		}
		if t > tr {
			tr = t
			break
		}
	}
	if tr < 0.0 {
		return 0
	}

	tr = float64(ROUND_I((tr+0.08)/0.1)) * 0.1

	/* adjust week handover */
	t = Time2GpsT(raw.Time, &week)
	if tr < t-302400.0 {
		week++
	} else if tr > t+302400.0 {
		week--
	}
	time = GpsT2Time(week, tr)

	Trace(4, "time=%s\n", TimeStr(time, 0))

	for i, p = 0, off; p < raw.Len-2; i, p = i+1, p+length {

		/* quality indicator */
		qi = int(U1(raw.Buff[p+41:])) & 7
		if qi < 4 || 7 < qi {
			continue
		}

		if ctype == 6 {
			if sys = ubx_sys(int(U1(raw.Buff[p+56:]))); sys == 0 {
				Trace(2, "ubx trkd5: system error\n")
				continue
			}
			if sys == SYS_QZS {
				prn = int(U1(raw.Buff[p+57:])) + 192
			} else {
				prn = int(U1(raw.Buff[p+57:]))
			}
			_ = U1(raw.Buff[p+59:]) - 7 /* frq */
		} else {
			prn = int(U1(raw.Buff[p+34:]))
			if prn < MINPRNSBS {
				sys = SYS_GPS
			} else {
				sys = SYS_SBS
			}
		}
		if sat = SatNo(sys, prn); sat == 0 {
			Trace(2, "ubx trkd5 sat number error: sys=%2d prn=%2d\n", sys, prn)
			continue
		}
		/* transmission time */
		ts = I8L(raw.Buff[p:]) * P2_32 / 1000.0
		if sys == SYS_GLO {
			ts -= 10800.0 + utc_gpst
		} /* glot . gpst */

		/* signal travel time */
		tau = tr - ts
		if tau < -302400.0 {
			tau += 604800.0
		} else if tau > 302400.0 {
			tau -= 604800.0
		}

		flag = int(U1(raw.Buff[p+54:])) /* tracking status */
		adr1 := 0.0
		if qi >= 6 {
			adr1 = I8L(raw.Buff[p+8:]) * P2_32
		}
		adr2 := 0.0
		if flag&0x01 <= 0 {
			adr2 = 0.5
		}
		adr = adr1 + adr2
		dop = float64(I4L(raw.Buff[p+16:])) * P2_10 / 4.0
		snr = float64(U2L(raw.Buff[p+32:])) / 256.0

		if snr <= 10.0 {
			raw.LockTime[sat-1][1] = 1.0
		}

		// #if 0 /* for debug */
		//         Trace(2,"[%2d] qi=%d sys=%d prn=%3d frq=%2d flag=%02X ts=%1.3f "
		//               "snr=%4.1f dop=%9.3f adr=%13.3f %6.3f\n",U1(raw.Buff[p:]+35),qi,U1(raw.Buff[p:]+56),
		//               prn,frq,flag,ts,snr,dop,adr,
		//               adrs[sat-1]==0.0||dop==0.0?0.0:(adr-adrs[sat-1])-dop);
		// #endif
		adrs[sat-1] = adr

		/* check phase lock */
		if flag&0x08 == 0 {
			continue
		}

		raw.ObsData.Data[n].Time = time
		raw.ObsData.Data[n].Sat = sat
		raw.ObsData.Data[n].P[0] = tau * CLIGHT
		raw.ObsData.Data[n].L[0] = -adr
		raw.ObsData.Data[n].D[0] = dop
		raw.ObsData.Data[n].SNR[0] = uint16(snr/SNR_UNIT + 0.5)
		if sys == SYS_CMP {
			raw.ObsData.Data[n].Code[0] = CODE_L2I
		} else {
			raw.ObsData.Data[n].Code[0] = CODE_L1C
		}
		if raw.LockTime[sat-1][1] > 0.0 {
			raw.ObsData.Data[n].LLI[0] = 1
		} else {
			raw.ObsData.Data[n].LLI[0] = 0
		}
		raw.LockTime[sat-1][1] = 0.0

		for j = 1; j < NFREQ+NEXOBS; j++ {
			raw.ObsData.Data[n].L[j], raw.ObsData.Data[n].P[j] = 0.0, 0.0
			raw.ObsData.Data[n].D[j] = 0.0
			raw.ObsData.Data[n].SNR[j], raw.ObsData.Data[n].LLI[j] = 0, 0
			raw.ObsData.Data[n].Code[j] = CODE_NONE
		}
		n++
	}
	if n <= 0 {
		return 0
	}
	raw.Time = time
	raw.ObsData.n = n
	return 1
}

/* UTC 8-bit week . full week -----------------------------------------------*/
// the func has implemented in javad.go named: adj_utcweek   comment by fxb
// static void adj_utcweek(time Gtime, float64 *utc)
// {
//     int week;

//     Time2GpsT(time,&week);
//     utc[3]+=week/256*256;
//     if      (utc[3]<week-127) utc[3]+=256.0;
//     else if (utc[3]>week+127) utc[3]-=256.0;
//     utc[5]+=utc[3]/256*256;
//     if      (utc[5]<utc[3]-127) utc[5]+=256.0;
//     else if (utc[5]>utc[3]+127) utc[5]-=256.0;
// }
/* decode GPS/QZSS ephemeris -------------------------------------------------*/
func decode_eph_ub(raw *Raw, sat int) int {
	var eph Eph

	if DecodeFrame(raw.SubFrm[sat-1][:], &eph, nil, nil, nil) == 0 {
		return 0
	}

	if !strings.Contains(raw.Opt, "-EPHALL") {
		if eph.Iode == raw.NavData.Ephs[sat-1].Iode &&
			eph.Iodc == raw.NavData.Ephs[sat-1].Iodc &&
			TimeDiff(eph.Toe, raw.NavData.Ephs[sat-1].Toe) == 0.0 &&
			TimeDiff(eph.Toc, raw.NavData.Ephs[sat-1].Toc) == 0.0 {
			return 0
		}
	}
	eph.Sat = sat
	raw.NavData.Ephs[sat-1] = eph
	raw.EphSat = sat
	raw.EphSet = 0
	return 2
}

/* decode GPS/QZSS ION/UTC parameters ----------------------------------------*/
func decode_ionutc(raw *Raw, sat int) int {
	var (
		ion, utc [8]float64
		sys      = SatSys(sat, nil)
	)

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

/* decode GPS/QZSS navigation data -------------------------------------------*/
func decode_nav(raw *Raw, sat, off int) int {
	var (
		p          = 6 + off
		buff       [30]uint8
		i, id, ret int
	)

	if raw.Len < 48+off {
		Trace(2, "ubx rxmsfrbx nav length error: sat=%d len=%d\n", sat, raw.Len)
		return -1
	}
	if (U4L(raw.Buff[p:]) >> 24) == PREAMB_CNAV {
		Trace(3, "ubx rxmsfrbx nav unsupported sat=%d len=%d\n", sat, raw.Len)
		return 0
	}
	for i = 0; i < 10; i, p = i+1, p+4 { /* 24 x 10 bits w/o parity */
		SetBitU(buff[:], 24*i, 24, U4L(raw.Buff[p:])>>6)
	}
	id = int(GetBitU(buff[:], 43, 3))
	if id < 1 || id > 5 {
		Trace(2, "ubx rxmsfrbx nav subframe id error: sat=%d id=%d\n", sat, id)
		return -1
	}
	copy(raw.SubFrm[sat-1][(id-1)*30:], buff[:30])

	if id == 3 {
		return decode_eph_ub(raw, sat)
	}
	if id == 4 || id == 5 {
		ret = decode_ionutc(raw, sat)
		// memset(raw.SubFrm[sat-1]+(id-1)*30,0,30);
		return ret
	}
	return 0
}

/* decode Galileo I/NAV navigation data --------------------------------------*/
func decode_enav(raw *Raw, sat, off int) int {
	var (
		eph                                     Eph
		ion                                     [4]float64
		utc                                     [8]float64
		p                                       = 6 + off
		buff                                    [32]uint8
		crc_buff                                [26]uint8
		i, j, part1, page1, part2, page2, ctype int
	)
	if raw.Len < 40+off {
		Trace(2, "ubx rxmsfrbx enav length error: sat=%d len=%d\n", sat, raw.Len)
		return -1
	}
	if raw.Len < 44+off {
		return 0 /* E5b I/NAV */
	}

	for i = 0; i < 8; i, p = i+1, p+4 {
		SetBitU(buff[:], 32*i, 32, U4L(raw.Buff[p:]))
	}
	part1 = int(GetBitU(buff[:], 0, 1))
	page1 = int(GetBitU(buff[:], 1, 1))
	part2 = int(GetBitU(buff[:], 128, 1))
	page2 = int(GetBitU(buff[:], 129, 1))

	if part1 != 0 || part2 != 1 {
		Trace(3, "ubx rxmsfrbx enav page even/odd error: sat=%d\n", sat)
		return -1
	}
	if page1 == 1 || page2 == 1 {
		return 0 /* alert page */
	}

	/* test crc (4(pad) + 114 + 82 bits) */
	for i, j = 0, 4; i < 15; i, j = i+1, j+8 {
		SetBitU(crc_buff[:], j, 8, GetBitU(buff[:], i*8, 8))
	}
	for i, j = 0, 118; i < 11; i, j = i+1, j+8 {
		SetBitU(crc_buff[:], j, 8, GetBitU(buff[:], i*8+128, 8))
	}
	if Rtk_CRC24q(crc_buff[:], 25) != GetBitU(buff[:], 128+82, 24) {
		Trace(2, "ubx rxmsfrbx enav crc error: sat=%d\n", sat)
		return -1
	}
	ctype = int(GetBitU(buff[:], 2, 6)) /* word type */

	if ctype > 6 {
		return 0
	}

	/* save 128 (112:even+16:odd) bits word */
	for i, j = 0, 2; i < 14; i, j = i+1, j+8 {
		raw.SubFrm[sat-1][ctype*16+i] = uint8(GetBitU(buff[:], j, 8))
	}
	for i, j = 14, 130; i < 16; i, j = i+1, j+8 {
		raw.SubFrm[sat-1][ctype*16+i] = uint8(GetBitU(buff[:], j, 8))
	}
	if ctype != 5 {
		return 0
	}
	if DecodeIrnNav(raw.SubFrm[sat-1][:], &eph, ion[:], utc[:]) == 0 {
		return 0
	}

	if eph.Sat != sat {
		Trace(2, "ubx rxmsfrbx enav satellite error: sat=%d %d\n", sat, eph.Sat)
		return -1
	}
	eph.Code |= (1 << 0) /* data source: E1 */

	adj_utcweek(raw.Time, utc[:])
	MatCpy(raw.NavData.Ion_gal[:], ion[:], 4, 1)
	MatCpy(raw.NavData.Utc_gal[:], utc[:], 8, 1)

	if !strings.Contains(raw.Opt, "-EPHALL") {
		if eph.Iode == raw.NavData.Ephs[sat-1].Iode &&
			TimeDiff(eph.Toe, raw.NavData.Ephs[sat-1].Toe) == 0.0 &&
			TimeDiff(eph.Toc, raw.NavData.Ephs[sat-1].Toc) == 0.0 {
			return 0
		}
	}
	raw.NavData.Ephs[sat-1] = eph
	raw.EphSat = sat
	raw.EphSet = 0 /* 0:I/NAV */
	return 2
}

/* decode BDS navigation data ------------------------------------------------*/
func decode_cnav(raw *Raw, sat, off int) int {
	var (
		eph             Eph
		ion, utc        [8]float64
		p               = 6 + off
		buff            [38]uint8
		i, id, pgn, prn int
	)
	if raw.Len < 48+off {
		Trace(2, "ubx rxmsfrbx cnav length error: sat=%d len=%d\n", sat, raw.Len)
		return -1
	}
	for i = 0; i < 10; i, p = i+1, p+4 {
		SetBitU(buff[:], 30*i, 30, U4L(raw.Buff[p:]))
	}
	id = int(GetBitU(buff[:], 15, 3)) /* subframe ID */
	if id < 1 || 5 < id {
		Trace(2, "ubx rxmsfrbx cnav subframe id error: sat=%2d\n", sat)
		return -1
	}
	SatSys(sat, &prn)

	if prn >= 6 && prn <= 58 { /* IGSO/MEO */
		copy(raw.SubFrm[sat-1][(id-1)*38:], buff[:38])

		if id == 3 {
			if DecodeBDSD1(raw.SubFrm[sat-1][:], &eph, nil, nil) == 0 {
				return 0
			}
		} else if id == 5 {
			if DecodeBDSD1(raw.SubFrm[sat-1][:], nil, ion[:], utc[:]) == 0 {
				return 0
			}
			MatCpy(raw.NavData.Ion_cmp[:], ion[:], 8, 1)
			MatCpy(raw.NavData.Utc_cmp[:], utc[:], 8, 1)
			return 9
		} else {
			return 0
		}
	} else { /* GEO */
		pgn = int(GetBitU(buff[:], 42, 4)) /* page numuber */

		if id == 1 && pgn >= 1 && pgn <= 10 {
			copy(raw.SubFrm[sat-1][(pgn-1)*38:], buff[:38])
			if pgn != 10 {
				return 0
			}
			if DecodeBDSD2(raw.SubFrm[sat-1][:], &eph, nil) == 0 {
				return 0
			}
		} else if id == 5 && pgn == 102 {
			copy(raw.SubFrm[sat-1][10*38:], buff[:38])
			if DecodeBDSD2(raw.SubFrm[sat-1][:], nil, utc[:]) == 0 {
				return 0
			}
			MatCpy(raw.NavData.Utc_cmp[:], utc[:], 8, 1)
			return 9
		} else {
			return 0
		}
	}
	if !strings.Contains(raw.Opt, "-EPHALL") {
		if TimeDiff(eph.Toe, raw.NavData.Ephs[sat-1].Toe) == 0.0 {
			return 0
		}
	}
	eph.Sat = sat
	raw.NavData.Ephs[sat-1] = eph
	raw.EphSat = sat
	raw.EphSet = 0
	return 2
}

/* decode GLONASS navigation data --------------------------------------------*/
func decode_gnav(raw *Raw, sat, off, frq int) int {
	var (
		geph            GEph
		utc_glo         [8]float64
		i, j, k, m, prn int
		p               = 6 + off
		buff            [64]uint8
		//,*fid;
	)
	SatSys(sat, &prn)

	if raw.Len < 24+off {
		Trace(2, "ubx rxmsfrbx gnav length error: len=%d\n", raw.Len)
		return -1
	}
	for i, k = 0, 0; i < 4; i, p = i+1, p+4 {
		for j = 0; j < 4; j++ {
			buff[k] = raw.Buff[p+3-j]
			k++
		}
	}
	/* test hamming of GLONASS string */
	if test_glostr(buff[:]) == 0 {
		Trace(2, "ubx rxmsfrbx gnav hamming error: sat=%2d\n", sat)
		return -1
	}
	m = int(GetBitU(buff[:], 1, 4))
	if m < 1 || 15 < m {
		Trace(2, "ubx rxmsfrbx gnav string no error: sat=%2d\n", sat)
		return -1
	}
	/* flush frame buffer if frame-ID changed */
	fid := raw.SubFrm[sat-1][150:]
	if fid[0] != buff[12] || fid[1] != buff[13] {
		for i = 0; i < 4; i++ {
			for k := 0; k < 10; k++ {
				raw.SubFrm[sat-1][i*10+k] = 0
			}
		}
		copy(fid[:], buff[12:14]) /* save frame-id */
	}
	copy(raw.SubFrm[sat-1][(m-1)*10:], buff[:10])

	if m == 4 {
		/* decode GLONASS ephemeris strings */
		geph.Tof = raw.Time
		if Decode_Glostr(raw.SubFrm[sat-1][:], &geph, nil) == 0 || geph.Sat != sat {
			return 0
		}
		geph.Frq = frq - 7

		if !strings.Contains(raw.Opt, "-EPHALL") {
			if geph.Iode == raw.NavData.Geph[prn-1].Iode {
				return 0
			}
		}
		raw.NavData.Geph[prn-1] = geph
		raw.EphSat = sat
		raw.EphSet = 0
		return 2
	} else if m == 5 {
		if Decode_Glostr(raw.SubFrm[sat-1][:], nil, utc_glo[:]) == 0 {
			return 0
		}
		MatCpy(raw.NavData.Utc_glo[:], utc_glo[:], 8, 1)
		return 9
	}
	return 0
}

/* decode SBAS navigation data -----------------------------------------------*/
func decode_snav(raw *Raw, prn, off int) int {
	var (
		i, tow, week int
		p            = 6 + off
		buff         [32]uint8
	)

	if raw.Len < 40+off {
		Trace(2, "ubx rxmsfrbx snav length error: len=%d\n", raw.Len)
		return -1
	}
	tow = int(Time2GpsT(TimeAdd(raw.Time, -1.0), &week))
	raw.Sbsmsg.Prn = uint8(prn)
	raw.Sbsmsg.Tow = tow
	raw.Sbsmsg.Week = week
	for i = 0; i < 8; i, p = i+1, p+4 {
		SetBitU(buff[:], 32*i, 32, U4L(raw.Buff[p:]))
	}
	copy(raw.Sbsmsg.Msg[:], buff[:29])
	raw.Sbsmsg.Msg[28] &= 0xC0
	return 3
}

/* decode UBX-RXM-SFRBX: raw subframe data (ref [3][4][5]) -------------------*/
func decode_rxmsfrbx(raw *Raw) int {
	var (
		p             = 6
		prn, sat, sys int
	)

	if raw.OutType > 0 {
		copy(raw.MsgType[:], []byte(fmt.Sprintf("UBX RXM-SFRBX (%4d): sys=%d prn=%3d", raw.Len,
			U1(raw.Buff[p:]), U1(raw.Buff[p+1:]))))
	}
	if sys = ubx_sys(int(U1(raw.Buff[p:]))); sys == 0 {
		Trace(2, "ubx rxmsfrbx sys id error: sys=%d\n", U1(raw.Buff[p:]))
		return -1
	}
	if sys == SYS_QZS {
		prn = int(U1(raw.Buff[p+1:])) + 192
	} else {
		prn = int(U1(raw.Buff[p+1:]))
	}
	if sat = SatNo(sys, prn); sat == 0 {
		if sys == SYS_GLO && prn == 255 {
			return 0 /* suppress error for unknown GLONASS satellite */
		}
		Trace(2, "ubx rxmsfrbx sat number error: sys=%d prn=%d\n", sys, prn)
		return -1
	}
	if sys == SYS_QZS && raw.Len == 52 { /* QZSS L1S */
		sys = SYS_SBS
		prn -= 10
	}
	switch sys {
	case SYS_GPS:
		return decode_nav(raw, sat, 8)
	case SYS_QZS:
		return decode_nav(raw, sat, 8)
	case SYS_GAL:
		return decode_enav(raw, sat, 8)
	case SYS_CMP:
		return decode_cnav(raw, sat, 8)
	case SYS_GLO:
		return decode_gnav(raw, sat, 8, int(U1(raw.Buff[p+3:])))
	case SYS_SBS:
		return decode_snav(raw, prn, 8)
	}
	return 0
}

/* decode UBX-TRK-SFRBX: subframe buffer extension (unoffitial) --------------*/
func decode_trksfrbx(raw *Raw) int {
	var (
		p             = 6
		prn, sat, sys int
	)

	if raw.OutType > 0 {
		copy(raw.MsgType[:], []byte(fmt.Sprintf("UBX TRK-SFRBX (%4d): sys=%d prn=%3d", raw.Len,
			U1(raw.Buff[p+1:]), U1(raw.Buff[p+2:]))))
	}
	if sys = ubx_sys(int(U1(raw.Buff[p+1:]))); sys == 0 {
		Trace(2, "ubx trksfrbx sys id error: sys=%d\n", U1(raw.Buff[p+1:]))
		return -1
	}
	if sys == SYS_QZS {
		prn = int(U1(raw.Buff[p+2:])) + 192
	} else {
		prn = int(U1(raw.Buff[p+2:]))
	}
	if sat = SatNo(sys, prn); sat == 0 {
		Trace(2, "ubx trksfrbx sat number error: sys=%d prn=%d\n", sys, prn)
		return -1
	}
	switch sys {
	case SYS_GPS:
		return decode_nav(raw, sat, 13)
	case SYS_QZS:
		return decode_nav(raw, sat, 13)
	case SYS_GAL:
		return decode_enav(raw, sat, 13)
	case SYS_CMP:
		return decode_cnav(raw, sat, 13)
	case SYS_GLO:
		return decode_gnav(raw, sat, 13, int(U1(raw.Buff[p+4:])))
	case SYS_SBS:
		return decode_snav(raw, sat, 13)
	}
	return 0
}

/* decode UBX-RXM-SFRB: subframe buffer (GPS/SBAS) ---------------------------*/
func decode_rxmsfrb(raw *Raw) int {
	var (
		words                [10]uint32
		p                    = 6
		buff                 [30]uint8
		i, sys, prn, sat, id int
	)

	if raw.OutType > 0 {
		copy(raw.MsgType[:], []byte(fmt.Sprintf("UBX RXM-SFRB  (%4d): prn=%2d", raw.Len, U1(raw.Buff[p+1:]))))
	}
	if raw.Len < 42 {
		Trace(2, "ubx rxmsfrb length error: len=%d\n", raw.Len)
		return -1
	}
	prn = int(U1(raw.Buff[p+1:]))
	if prn >= MINPRNSBS {
		sys = SYS_SBS
	} else {
		sys = SYS_GPS
	}
	if sat = SatNo(sys, prn); sat == 0 {
		Trace(2, "ubx rxmsfrb satellite error: prn=%d\n", prn)
		return -1
	}
	if sys == SYS_GPS {
		for i, p = 0, p+2; i < 10; i, p = i+1, p+4 {
			SetBitU(buff[:], 24*i, 24, U4L(raw.Buff[p:]))
		}
		id = int(GetBitU(buff[:], 43, 3))
		if id >= 1 && id <= 5 {
			copy(raw.SubFrm[sat-1][(id-1)*30:], buff[:30])
			if id == 3 {
				return decode_eph_ub(raw, sat)
			} else if id == 4 {
				return decode_ionutc(raw, sat)
			}
		}
	} else {
		for i, p = 0, p+2; i < 10; i, p = i+1, p+4 {
			words[i] = U4L(raw.Buff[p:])
		}
		if SbsDecodeMsg(raw.Time, prn, words[:], &raw.Sbsmsg) == 0 {
			return 0
		}
		return 3
	}
	return 0
}

/* decode ublox raw message --------------------------------------------------*/
func decode_ubx(raw *Raw) int {
	ctype := int(U1(raw.Buff[2:]))<<8 + int(U1(raw.Buff[3:]))

	Trace(3, "decode_ubx: type=%04x len=%d\n", ctype, raw.Len)

	/* checksum */
	if checksum_ublox(raw.Buff[:], raw.Len) == 0 {
		Trace(2, "ubx checksum error: type=%04x len=%d\n", ctype, raw.Len)
		return -1
	}
	switch ctype {
	case ID_RXMRAW:
		return decode_rxmraw(raw)
	case ID_RXMRAWX:
		return decode_rxmrawx(raw)
	case ID_RXMSFRB:
		return decode_rxmsfrb(raw)
	case ID_RXMSFRBX:
		return decode_rxmsfrbx(raw)
	case ID_NAVSOL:
		return decode_navsol(raw)
	case ID_NAVTIME:
		return decode_navtime(raw)
	case ID_TRKMEAS:
		return decode_trkmeas(raw)
	case ID_TRKD5:
		return decode_trkd5(raw)
	case ID_TRKSFRBX:
		return decode_trksfrbx(raw)
	}
	if raw.OutType > 0 {
		copy(raw.MsgType[:], []byte(fmt.Sprintf("UBX 0x%02X 0x%02X (%4d)", ctype>>8, ctype&0xF,
			raw.Len)))
	}
	return 0
}

/* sync code -----------------------------------------------------------------*/
func sync_ubx(buff []uint8, data uint8) int {
	buff[0] = buff[1]
	buff[1] = data
	if buff[0] == UBXSYNC1 && buff[1] == UBXSYNC2 {
		return 1
	}
	return 0
}

/* input ublox raw message from stream -----------------------------------------
* fetch next ublox raw data and input a mesasge from stream
* args   : raw *Raw       IO  receiver raw data control struct
*          uint8 data     I   stream data (1 byte)
* return : status (-1: error message, 0: no message, 1: input observation data,
*                  2: input ephemeris, 3: input sbas message,
*                  9: input ion/utc parameter)
*
* notes  : to specify input options, set raw.Opt to the following option
*          strings separated by spaces.
*
*          -EPHALL    : input all ephemerides
*          -INVCP     : invert polarity of carrier-phase
*          -TADJ=tint : adjust time tags to multiples of tint (sec)
*          -STD_SLIP=std: slip by std-dev of carrier phase under std
*
*          The supported messages are as follows.
*
*          UBX-RXM-RAW  : raw measurement data
*          UBX-RXM-RAWX : multi-gnss measurement data
*          UBX-RXM-SFRB : subframe buffer
*          UBX-RXM-SFRBX: subframe buffer extension
*
*          UBX-TRK-MEAS and UBX-TRK-SFRBX are based on NEO-M8N (F/W 2.01).
*          UBX-TRK-D5 is based on NEO-7N (F/W 1.00). They are not formally
*          documented and not supported by u-blox.
*          Users can use these messages by their own risk.
*-----------------------------------------------------------------------------*/
func input_ubx(raw *Raw, data uint8) int {
	Trace(5, "input_ubx: data=%02x\n", data)

	/* synchronize frame */
	if raw.NumByte == 0 {
		if sync_ubx(raw.Buff[:], data) == 0 {
			return 0
		}
		raw.NumByte = 2
		return 0
	}
	raw.Buff[raw.NumByte] = data
	raw.NumByte++

	if raw.NumByte == 6 {
		if raw.Len = int(U2L(raw.Buff[4:])) + 8; raw.Len > MAXRAWLEN {
			Trace(2, "ubx length error: len=%d\n", raw.Len)
			raw.NumByte = 0
			return -1
		}
	}
	if raw.NumByte < 6 || raw.NumByte < raw.Len {
		return 0
	}
	raw.NumByte = 0

	/* decode ublox raw message */
	return decode_ubx(raw)
}

/* input ublox raw message from file -------------------------------------------
* fetch next ublox raw data and input a message from file
* args   : raw_t  *raw      IO  receiver raw data control struct
*          FILE   *fp       I   file pointer
* return : status(-2: end of file, -1...9: same as above)
*-----------------------------------------------------------------------------*/
func input_ubxf(raw *Raw, fp *os.File) int {

	Trace(4, "input_ubxf:\n")

	/* synchronize frame */
	if raw.NumByte == 0 {
		var c [1]byte
		/* synchronize frame */
		for i := 0; ; i++ {
			_, err := fp.Read(c[:])
			if err == io.EOF {
				return -2
			}
			if sync_ubx(raw.Buff[:], uint8(c[0])) > 0 {
				break
			}
			if i >= 4096 {
				return 0
			}
		}
	}
	if n, _ := fp.Read(raw.Buff[2:6]); n < 4 {
		return -2
	}
	raw.NumByte = 6

	if raw.Len = int(U2L(raw.Buff[4:]) + 8); raw.Len > MAXRAWLEN {
		Trace(2, "ubx length error: len=%d\n", raw.Len)
		raw.NumByte = 0
		return -1
	}
	if n, _ := fp.Read(raw.Buff[6:raw.Len]); n < raw.Len-6 {
		return -2
	}
	raw.NumByte = 0

	/* decode ubx raw message */
	return decode_ubx(raw)
}

/* convert string to integer -------------------------------------------------*/
func stoi(s string) int {
	var n uint32
	if k, _ := fmt.Sscanf(s, "0x%X", &n); k == 1 {
		return int(n) /* hex (0xXXXX) */
	}
	k, _ := strconv.Atoi(s)
	return k
}

/* generate ublox binary message -----------------------------------------------
* generate ublox binary message from message string
* args   : char  *msg   IO     message string
*            "CFG-PRT   portid res0 res1 mode baudrate inmask outmask flags"
*            "CFG-USB   vendid prodid res1 res2 power flags vstr pstr serino"
*            "CFG-MSG   msgid rate0 rate1 rate2 rate3 rate4 rate5 rate6"
*            "CFG-NMEA  filter version numsv flags"
*            "CFG-RATE  meas nav time"
*            "CFG-CFG   clear_mask save_mask load_mask [dev_mask]"
*            "CFG-TP    interval length status time_ref res adelay rdelay udelay"
*            "CFG-NAV2  ..."
*            "CFG-DAT   maja flat dx dy dz rotx roty rotz scale"
*            "CFG-INF   protocolid res0 res1 res2 mask0 mask1 mask2 ... mask5"
*            "CFG-RST   navbbr reset res"
*            "CFG-RXM   gpsmode lpmode"
*            "CFG-ANT   flags pins"
*            "CFG-FXN   flags treacq tacq treacqoff tacqoff ton toff res basetow"
*            "CFG-SBAS  mode usage maxsbas res scanmode"
*            "CFG-LIC   key0 key1 key2 key3 key4 key5"
*            "CFG-TM    intid rate flags"
*            "CFG-TM2   ch res0 res1 rate flags"
*            "CFG-TMODE tmode posx posy posz posvar svinmindur svinvarlimit"
*            "CFG-EKF   ..."
*            "CFG-GNSS  ..."
*            "CFG-ITFM  conf conf2"
*            "CFG-LOGFILTER ver flag min_int time_thr speed_thr pos_thr"
*            "CFG-NAV5  ..."
*            "CFG-NAVX5 ..."
*            "CFG-ODO   ..."
*            "CFG-PM2   ..."
*            "CFG-PWR   ver rsv1 rsv2 rsv3 state"
*            "CFG-RINV  flag data ..."
*            "CFG-SMGR  ..."
*            "CFG-TMODE2 ..."
*            "CFG-TMODE3 ..."
*            "CFG-TPS   ..."
*            "CFG-TXSLOT ..."
*            "CFG-VALDEL ver layer res0 res1 key [key ...]"
*            "CFG-VALGET ver layer pos key [key ...]"
*            "CFG-VALSET ver layer res0 res1 key value [key value ...]"
*          buff []uint8 O binary message
* return : length of binary message (0: error)
* note   : see reference [1][3][5] for details.
*          the following messages are not supported:
*             CFG-DOSC,CFG-ESRC
*-----------------------------------------------------------------------------*/
func gen_ubx(msg string, buff []uint8) int {
	var cmd []string = []string{
		"PRT", "USB", "MSG", "NMEA", "RATE", "CFG", "TP", "NAV2", "DAT", "INF",
		"RST", "RXM", "ANT", "FXN", "SBAS", "LIC", "TM", "TM2", "TMODE", "EKF",
		"GNSS", "ITFM", "LOGFILTER", "NAV5", "NAVX5", "ODO", "PM2", "PWR", "RINV", "SMGR",
		"TMODE2", "TMODE3", "TPS", "TXSLOT",
		"VALDEL", "VALGET", "VALSET", "",
	}
	var id []uint8 = []uint8{
		0x00, 0x1B, 0x01, 0x17, 0x08, 0x09, 0x07, 0x1A, 0x06, 0x02,
		0x04, 0x11, 0x13, 0x0E, 0x16, 0x80, 0x10, 0x19, 0x1D, 0x12,
		0x3E, 0x39, 0x47, 0x24, 0x23, 0x1E, 0x3B, 0x57, 0x34, 0x62,
		0x36, 0x71, 0x31, 0x53,
		0x8c, 0x8b, 0x8a,
	}
	var prm [][32]int = [][32]int{
		{FU1, FU1, FU2, FU4, FU4, FU2, FU2, FU2, FU2},    /* PRT */
		{FU2, FU2, FU2, FU2, FU2, FU2, FS32, FS32, FS32}, /* USB */
		{FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1},         /* MSG */
		{FU1, FU1, FU1, FU1},                             /* NMEA */
		{FU2, FU2, FU2},                                  /* RATE */
		{FU4, FU4, FU4, FU1},                             /* CFG */
		{FU4, FU4, FI1, FU1, FU2, FI2, FI2, FI4},         /* TP */
		{FU1, FU1, FU2, FU1, FU1, FU1, FU1, FI4, FU1, FU1, FU1, FU1, FU1, FU1, FU2, FU2, FU2, FU2,
			FU2, FU1, FU1, FU2, FU4, FU4}, /* NAV2 */
		{FR8, FR8, FR4, FR4, FR4, FR4, FR4, FR4, FR4},      /* DAT */
		{FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1}, /* INF */
		{FU2, FU1, FU1},                                    /* RST */
		{FU1, FU1},                                         /* RXM */
		{FU2, FU2},                                         /* ANT */
		{FU4, FU4, FU4, FU4, FU4, FU4, FU4, FU4},           /* FXN */
		{FU1, FU1, FU1, FU1, FU4},                          /* SBAS */
		{FU2, FU2, FU2, FU2, FU2, FU2},                     /* LIC */
		{FU4, FU4, FU4},                                    /* TM */
		{FU1, FU1, FU2, FU4, FU4},                          /* TM2 */
		{FU4, FI4, FI4, FI4, FU4, FU4, FU4},                /* TMODE */
		{FU1, FU1, FU1, FU1, FU4, FU2, FU2, FU1, FU1, FU2}, /* EKF */
		{FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU4},      /* GNSS */
		{FU4, FU4},                                         /* ITFM */
		{FU1, FU1, FU2, FU2, FU2, FU4},                     /* LOGFILTER */
		{FU2, FU1, FU1, FI4, FU4, FI1, FU1, FU2, FU2, FU2, FU2, FU1, FU1, FU1, FU1, FU1, FU1, FU2,
			FU1, FU1, FU1, FU1, FU1, FU1}, /* NAV5 */
		{FU2, FU2, FU4, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU2, FU1, FU1, FU1, FU1,
			FU1, FU1, FU1, FU1, FU1, FU1, FU2}, /* NAVX5 */
		{FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1},                /* ODO */
		{FU1, FU1, FU1, FU1, FU4, FU4, FU4, FU4, FU2, FU2},           /* PM2 */
		{FU1, FU1, FU1, FU1, FU4},                                    /* PWR */
		{FU1, FU1},                                                   /* RINV */
		{FU1, FU1, FU2, FU2, FU1, FU1, FU2, FU2, FU2, FU2, FU4},      /* SMGR */
		{FU1, FU1, FU2, FI4, FI4, FI4, FU4, FU4, FU4},                /* TMODE2 */
		{FU1, FU1, FU2, FI4, FI4, FI4, FU4, FU4, FU4},                /* TMODE3 */
		{FU1, FU1, FU1, FU1, FI2, FI2, FU4, FU4, FU4, FU4, FI4, FU4}, /* TPS */
		{FU1, FU1, FU1, FU1, FU4, FU4, FU4, FU4, FU4},                /* TXSLOT */
		{FU1, FU1, FU1, FU1},                                         /* VALDEL */
		{FU1, FU1, FU2},                                              /* VALGET */
		{FU1, FU1, FU1, FU1},                                         /* VALSET */
	}
	var (
		q = 0
		// char mbuff[1024],*args[32],*p;
		i, j, n, narg int
	)
	Trace(4, "gen_ubxf: msg=%s\n", msg)

	args := strings.Split(msg, " ")
	if len(args) < 1 || !strings.EqualFold(args[0][:4], "CFG-") {
		return 0
	}

	for i = 0; i < len(cmd); i++ {
		if strings.EqualFold(args[0][4:], cmd[i]) {
			break
		}
	}
	if i >= len(cmd) {
		return 0
	}

	buff[q] = UBXSYNC1
	q++
	buff[q] = UBXSYNC2
	q++
	buff[q] = UBXCFG
	q++
	buff[q] = id[i]
	q++
	q += 2
	for j = 1; prm[i][j-1] > 0 || j < narg; j++ {
		switch prm[i][j-1] {
		case FU1:
			if j < len(args) {
				setU1(buff[q:], uint8(stoi(args[j])))
			} else {
				setU1(buff[q:], 0)
			}
			q += 1
		case FU2:
			if j < len(args) {
				setU2(buff[q:], uint16(stoi(args[j])))
			} else {
				setU2(buff[q:], 0)
			}
			q += 2
		case FU4:
			if j < len(args) {
				setU4(buff[q:], uint32(stoi(args[j])))
			} else {
				setU4(buff[q:], 0)
			}
			q += 4
		case FI1:
			if j < len(args) {
				setI1(buff[q:], int8(stoi(args[j])))
			} else {
				setI1(buff[q:], 0)
			}
			q += 1
		case FI2:
			if j < len(args) {
				setI2(buff[q:], int16(stoi(args[j])))
			} else {
				setI2(buff[q:], 0)
			}
			q += 2
		case FI4:
			if j < len(args) {
				setI4(buff[q:], int32(stoi(args[j])))
			} else {
				setI4(buff[q:], 0)
			}
			q += 4
		case FR4:
			if j < len(args) {
				v, _ := strconv.ParseFloat(args[j], 32)
				setR4(buff[q:], float32(v))
			} else {
				setR4(buff[q:], 0)
			}
			q += 4
		case FR8:
			if j < len(args) {
				v, _ := strconv.ParseFloat(args[j], 64)
				setR8(buff[q:], (v))
			} else {
				setR8(buff[q:], 0)
			}
			q += 8
		case FS32:
			if j < len(args) {
				copy(buff[q:], []byte(fmt.Sprintf("%-32.32s", args[j])))
			} else {
				copy(buff[q:], []byte(fmt.Sprintf("%-32.32s", "")))
			}
			q += 32
		default:
			if j < len(args) {
				setU1(buff[q:], uint8(stoi(args[j])))
			} else {
				setU1(buff[q:], 0)
			}
			q += 1
		}
	}
	n = q + 2
	setU2(buff[4:], uint16(n-8)) /* length */
	setcs(buff[:], n)

	Trace(5, "gen_ubx: buff=\n")
	Traceb(5, buff[:], n)
	return n
}
