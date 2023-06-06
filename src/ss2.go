/*------------------------------------------------------------------------------
* ss2.c : superstar II receiver dependent functions
*
*          Copyright (C) 2007-2020 by T.TAKASU, All rights reserved.
*          Copyright (C) 2007-2022 by Feng Xuebin, All rights reserved.
*
* reference:
*     [1] NovAtel, OM-20000086 Superstar II Firmware Reference Manual, 2005
*
* version : $Revision: 1.2 $ $Date: 2008/07/14 00:05:05 $
* history : 2008/05/18 1.0 new
*           2008/06/16 1.2 separate common functions to rcvcmn.c
*           2009/04/01 1.3 fix bug on decode #21 message
*           2010/08/20 1.4 fix problem with minus value of time slew in #23
*                          (2.4.0_p5)
*           2011/05/27 1.5 fix problem with ARM compiler
*           2013/02/23 1.6 fix memory access violation problem on arm
*           2020/11/30 1.7 use integer type in stdint.h
*           2022/09/27 1.8 rewrite with golang
*-----------------------------------------------------------------------------*/
package gnssgo

import (
	"fmt"
	"io"
	"math"
	"os"
	"strings"
)

const (
	SS2SOH     = 0x01 /* ss2 start of header */
	ID_SS2LLH  = 20   /* ss2 message ID#20 navigation data (user) */
	ID_SS2ECEF = 21   /* ss2 message ID#21 navigation data (ecef) */
	ID_SS2EPH  = 22   /* ss2 message ID#22 ephemeris data */
	ID_SS2RAW  = 23   /* ss2 message ID#23 measurement block */
	ID_SS2SBAS = 67   /* ss2 message ID#67 sbas data */
)

/* get/set fields (little-endian) --------------------------------------------*/
// declared in cresent.go, rename it as xxL
// #define U1(p) (*((uint8 *)(p)))
// static uint16 U2L(uint8 *p) {uint16 u; memcpy(&u,p,2); return u;}
// static uint32 U4L(uint8 *p) {uint32 u; memcpy(&u,p,4); return u;}
// static float64   R8L(uint8 *p) {float64   r; memcpy(&r,p,8); return r;}

/* checksum ------------------------------------------------------------------*/
func chksum_ss2(buff []uint8, length int) int {
	var sum uint16 = 0

	for i := 0; i < length-2; i++ {
		sum += uint16(buff[i])
	}
	if uint8(sum>>8) == buff[length-1] && uint8(sum&0xFF) == buff[length-2] {
		return 1
	}
	return 0
}

/* adjust week ---------------------------------------------------------------*/
func adjweek_ss2(raw *Raw, sec float64) int {
	var (
		tow  float64
		week int
	)

	if raw.Time.Time == 0 {
		return 0
	}
	tow = Time2GpsT(raw.Time, &week)
	if sec < tow-302400.0 {
		sec += 604800.0
	} else if sec > tow+302400.0 {
		sec -= 604800.0
	}
	raw.Time = GpsT2Time(week, sec)
	return 1
}

/* decode id#20 navigation data (user) ---------------------------------------*/
func decode_ss2llh(raw *Raw) int {
	var (
		ep [6]float64
		p  = 4
	)

	Trace(4, "decode_ss2llh: len=%d\n", raw.Len)

	if raw.Len != 77 {
		Trace(2, "ss2 id#20 length error: len=%d\n", raw.Len)
		return -1
	}
	ep[3] = float64(U1(raw.Buff[p:]))
	ep[4] = float64(U1(raw.Buff[p+1:]))
	ep[5] = R8L(raw.Buff[p+2:])
	ep[2] = float64(U1(raw.Buff[p+10:]))
	ep[1] = float64(U1(raw.Buff[p+11:]))
	ep[0] = float64(U2L(raw.Buff[p+12:]))
	raw.Time = Utc2GpsT(Epoch2Time(ep[:]))
	return 0
}

/* decode id#21 navigation data (ecef) ---------------------------------------*/
func decode_ss2ecef(raw *Raw) int {
	var p = 4

	Trace(4, "decode_ss2ecef: len=%d\n", raw.Len)

	if raw.Len != 85 {
		Trace(2, "ss2 id#21 length error: len=%d\n", raw.Len)
		return -1
	}
	raw.Time = GpsT2Time(int(U2L(raw.Buff[p+8:])), R8L(raw.Buff[p:]))
	return 0
}

/* decode id#23 measurement block --------------------------------------------*/
func decode_ss2meas(raw *Raw) int {
	const (
		freqif float64 = 1.405396825e6
		tslew          = 1.75e-7
	)
	var (
		tow, slew, code, icp, d float64
		i, j, n, prn, sat, nobs int
		p                       = 4
		sc                      uint32
	)

	Trace(4, "decode_ss2meas: len=%d\n", raw.Len)

	nobs = int(U1(raw.Buff[p+2:]))
	if 17+nobs*11 != raw.Len {
		Trace(2, "ss2 id#23 message length error: len=%d\n", raw.Len)
		return -1
	}
	tow = math.Floor(R8L(raw.Buff[p+3:])*1000.0+0.5) / 1000.0 /* rounded by 1ms */
	if adjweek_ss2(raw, tow) == 0 {
		Trace(2, "ss2 id#23 message time adjustment error\n")
		return -1
	}
	/* time slew defined as uchar (ref [1]) but minus value appears in some f/w */
	slew = float64(raw.Buff[p]) * tslew

	raw.Icpc += 4.5803 - freqif*slew - FREQ1*(slew-1e-6) /* phase correction */

	for i, n, p = 0, 0, p+11; i < nobs && n < MAXOBS; i, p = i+1, p+11 {
		prn = int(raw.Buff[p]&0x1F) + 1
		sys := SYS_GPS
		if raw.Buff[p]&0x20 > 0 {
			sys = SYS_SBS
		}
		if sat = SatNo(sys, prn); sat == 0 {
			Trace(2, "ss2 id#23 satellite number error: prn=%d\n", prn)
			continue
		}
		raw.ObsData.Data[n].Time = raw.Time
		raw.ObsData.Data[n].Sat = sat
		code = (tow - math.Floor(tow)) - float64(U4L(raw.Buff[p+2:]))/2095104000.0
		if code < 0.0 {
			raw.ObsData.Data[n].P[0] = CLIGHT * (code + 1.0)
		} else {
			raw.ObsData.Data[n].P[0] = CLIGHT * (code)
		}
		icp = float64(U4L(raw.Buff[p+6:])>>2)/1024.0 + raw.Off[sat-1] /* unwrap */
		if math.Abs(icp-raw.Icpp[sat-1]) > 524288.0 {
			if icp > raw.Icpp[sat-1] {
				d = -1048576.0
			} else {
				d = 1048576.0
			}
			raw.Off[sat-1] += d
			icp += d
		}
		raw.Icpp[sat-1] = icp
		raw.ObsData.Data[n].L[0] = icp + raw.Icpc
		raw.ObsData.Data[n].D[0] = 0.0
		raw.ObsData.Data[n].SNR[0] = uint16(float64(U1(raw.Buff[p+1:]))*0.25/SNR_UNIT + 0.5)
		sc = uint32(U1(raw.Buff[p+10:]))
		if uint8(sc)-uint8(raw.LockTime[sat-1][0]) > 0 {
			raw.ObsData.Data[n].LLI[0] = 1
		} else {
			raw.ObsData.Data[n].LLI[0] = 0
		}
		if U1(raw.Buff[p+6:])&1 > 0 {
			raw.ObsData.Data[n].LLI[0] |= 2
		} else {
			raw.ObsData.Data[n].LLI[0] |= 0
		}
		raw.ObsData.Data[n].Code[0] = CODE_L1C
		raw.LockTime[sat-1][0] = float64(sc)

		for j = 1; j < NFREQ; j++ {
			raw.ObsData.Data[n].L[j], raw.ObsData.Data[n].P[j] = 0.0, 0.0
			raw.ObsData.Data[n].D[j] = 0.0
			raw.ObsData.Data[n].SNR[j], raw.ObsData.Data[n].LLI[j] = 0, 0
			raw.ObsData.Data[n].Code[j] = CODE_NONE
		}
		n++
	}
	raw.ObsData.n = n
	return 1
}

/* decode id#22 ephemeris data ------------------------------------------------*/
func decode_ss2eph(raw *Raw) int {
	var (
		eph            Eph
		tow            uint32
		p              = 4
		buff           [90]uint8
		i, j, prn, sat int
	)

	Trace(4, "decode_ss2eph: len=%d\n", raw.Len)

	if raw.Len != 79 {
		Trace(2, "ss2 id#22 length error: len=%d\n", raw.Len)
		return -1
	}
	prn = int(U4L(raw.Buff[p:])&0x1F) + 1
	if sat = SatNo(SYS_GPS, prn); sat == 0 {
		Trace(2, "ss2 id#22 satellite number error: prn=%d\n", prn)
		return -1
	}
	if raw.Time.Time == 0 {
		Trace(2, "ss2 id#22 week number unknown error\n")
		return -1
	}
	tow = uint32(Time2GpsT(raw.Time, nil) / 6.0)
	for i = 0; i < 3; i++ {
		buff[30*i+3] = uint8(tow >> 9) /* add tow + subframe id */
		buff[30*i+4] = uint8(tow >> 1)
		buff[30*i+5] = uint8(((tow & 1) << 7) + uint32((i+1)<<2))
		for j = 0; j < 24; j++ {
			buff[30*i+6+j] = raw.Buff[p+1+24*i+j]
		}
	}
	if DecodeFrame(buff[:], &eph, nil, nil, nil) == 0 {
		Trace(2, "ss2 id#22 subframe error: prn=%d\n", prn)
		return -1
	}
	if !strings.Contains(raw.Opt, "-EPHALL") {
		if eph.Iode == raw.NavData.Ephs[sat-1].Iode {
			return 0 /* unchanged */
		}
	}
	eph.Sat = sat
	eph.Ttr = raw.Time
	raw.NavData.Ephs[sat-1] = eph
	raw.EphSat = sat
	raw.EphSet = 0
	return 2
}

/* decode id#67 sbas data ----------------------------------------------------*/
func decode_ss2sbas(raw *Raw) int {
	var (
		i, prn int
		p      = 4
	)

	Trace(4, "decode_ss2sbas: len=%d\n", raw.Len)

	if raw.Len != 54 {
		Trace(2, "ss2 id#67 length error: len=%d\n", raw.Len)
		return -1
	}
	prn = int(U4L(raw.Buff[p+12:]))
	if prn < MINPRNSBS || MAXPRNSBS < prn {
		Trace(3, "ss2 id#67 prn error: prn=%d\n", prn)
		return 0
	}
	raw.Sbsmsg.Week = int(U4L(raw.Buff[p:]))
	raw.Sbsmsg.Tow = int(R8L(raw.Buff[p+4:]))
	_ = GpsT2Time(raw.Sbsmsg.Week, float64(raw.Sbsmsg.Tow)) /* time */
	raw.Sbsmsg.Prn = uint8(prn)
	for i = 0; i < 29; i++ {
		raw.Sbsmsg.Msg[i] = raw.Buff[p+16+i]
	}
	return 3
}

/* decode superstar 2 raw message --------------------------------------------*/
func decode_ss2(raw *Raw) int {
	var (
		p         = 0
		ctype int = int(U1(raw.Buff[p+1:]))
	)

	Trace(3, "decode_ss2: type=%2d\n", ctype)

	if chksum_ss2(raw.Buff[:], raw.Len) == 0 {
		Trace(2, "ss2 message checksum error: type=%d len=%d\n", ctype, raw.Len)
		return -1
	}
	if raw.OutType > 0 {
		copy(raw.MsgType[:], []byte(fmt.Sprintf("SS2 %2d (%4d):", ctype, raw.Len)))
	}
	switch ctype {
	case ID_SS2LLH:
		return decode_ss2llh(raw)
	case ID_SS2ECEF:
		return decode_ss2ecef(raw)
	case ID_SS2RAW:
		return decode_ss2meas(raw)
	case ID_SS2EPH:
		return decode_ss2eph(raw)
	case ID_SS2SBAS:
		return decode_ss2sbas(raw)
	}
	return 0
}

/* sync code -----------------------------------------------------------------*/
func sync_ss2(buff []uint8, data uint8) int {
	buff[0] = buff[1]
	buff[1] = buff[2]
	buff[2] = data
	if buff[0] == SS2SOH && (buff[1]^buff[2]) == 0xFF {
		return 1
	}
	return 0
}

/* input superstar 2 raw message from stream -----------------------------------
* input next superstar 2 raw message from stream
* args   : raw *Raw   IO     receiver raw data control struct
*          uint8 data I      stream data (1 byte)
* return : status (-1: error message, 0: no message, 1: input observation data,
*                  2: input ephemeris, 3: input sbas message,
*                  9: input ion/utc parameter)
* notes  : needs #20 or #21 message to get proper week number of #23 raw
*          observation data
*-----------------------------------------------------------------------------*/
func input_ss2(raw *Raw, data uint8) int {
	Trace(5, "input_ss2: data=%02x\n", data)

	/* synchronize frame */
	if raw.NumByte == 0 {
		if sync_ss2(raw.Buff[:], data) == 0 {
			return 0
		}
		raw.NumByte = 3
		return 0
	}
	raw.Buff[raw.NumByte] = data
	raw.NumByte++

	if raw.NumByte == 4 {
		if raw.Len = int(U1(raw.Buff[3:])) + 6; raw.Len > MAXRAWLEN {
			Trace(2, "ss2 length error: len=%d\n", raw.Len)
			raw.NumByte = 0
			return -1
		}
	}
	if raw.NumByte < 4 || raw.NumByte < raw.Len {
		return 0
	}
	raw.NumByte = 0

	/* decode superstar 2 raw message */
	return decode_ss2(raw)
}

/* input superstar 2 raw message from file -------------------------------------
* input next superstar 2 raw message from file
* args   : raw_t  *raw   IO     receiver raw data control struct
*          FILE   *fp    I      file pointer
* return : status(-2: end of file, -1...9: same as above)
*-----------------------------------------------------------------------------*/
func input_ss2f(raw *Raw, fp *os.File) int {
	Trace(4, "input_ss2f:\n")

	/* synchronize frame */
	if raw.NumByte == 0 {
		var c [1]byte
		/* synchronize frame */
		for i := 0; ; i++ {
			_, err := fp.Read(c[:])
			if err == io.EOF {
				return -2
			}
			if sync_ss2(raw.Buff[:], uint8(c[0])) > 0 {
				break
			}
			if i >= 4096 {
				return 0
			}
		}
	}
	if n, _ := fp.Read(raw.Buff[3:4]); n < 1 {
		return -2
	}
	raw.NumByte = 4

	if raw.Len = int(U1(raw.Buff[3:]) + 6); raw.Len > MAXRAWLEN {
		Trace(2, "ss2 length error: len=%d\n", raw.Len)
		raw.NumByte = 0
		return -1
	}
	if n, _ := fp.Read(raw.Buff[4:raw.Len]); n < raw.Len-4 {
		return -2
	}
	raw.NumByte = 0

	/* decode superstar 2 raw message */
	return decode_ss2(raw)
}
