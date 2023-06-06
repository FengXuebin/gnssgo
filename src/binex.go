package gnssgo

/*------------------------------------------------------------------------------
* BINEX.c : BINEX dependent functions
*
*          Copyright (C) 2013-2020 by T.TAKASU, All rights reserved.
*          Copyright (C) 2013-2022 by Feng Xuebin, All rights reserved.
*
* reference :
*     [1] UNAVCO, BINEX: Binary exchange format (updated on July 13, 2018)
*         (http://BINEX.unavco.org/BINEX.html)
*
* version : $Revision:$ $Date:$
* history : 2013/02/20 1.0 new
*           2013/04/15 1.1 support 0x01-05 beidou-2/compass ephemeris
*           2013/05/18 1.2 fix bug on decoding obsflags in message 0x7f-05
*           2014/04/27 1.3 fix bug on decoding iode for message 0x01-02
*           2015/12/05 1.4 fix bug on decoding tgd for message 0x01-05
*           2016/07/29 1.5 crc16() . rtk_crc16()
*           2017/04/11 1.6 (char *) . (signed char *)
*                          fix bug on unchange-test of beidou ephemeris
*           2018/10/10 1.7 fix problem of sisa handling in galileo ephemeris
*                          add receiver option -GALINAV, -GALFNAV
*           2018/12/06 1.8 fix bug on decoding galileo ephemeirs iode (0x01-04)
*           2019/05/10 1.9 save galileo E5b data to obs index 2
*           2019/07/25 1.10 support upgraded galileo ephemeris (0x01-14)
*           2020/11/30 1.11 support NavIC/IRNSS raw obs data (0x7f-05)
*                           support BDS B2b in raw obs data (0x7f-05)
*                           support IRNSS decoded ephemeris (0x01-07)
*                           support station info in site metadata (0x00)
*                           handle I/NAV and F/NAV seperately for Galileo
*                           CODE_L1I/L1Q/L1X . CODE_L2I/L2Q/L2X for BDS B1I
*                           use API code2idx() to get frequency index
*                           use API code2idx() to get carrier frequency
*                           use integer types in stdint.h
*            2022/9/19      rewrite the file with golang
*-----------------------------------------------------------------------------*/

import (
	"encoding/binary"
	"fmt"
	"io"
	"math"
	"os"
	"strings"
)

const (
	BNXSYNC1  = 0xC2 /* BINEX sync (little-endian,regular-crc) */
	BNXSYNC2  = 0xE2 /* BINEX sync (big-endian   ,regular-crc) */
	BNXSYNC3  = 0xC8 /* BINEX sync (little-endian,enhanced-crc) */
	BNXSYNC4  = 0xE8 /* BINEX sync (big-endian   ,enhanced-crc) */
	BNXSYNC1R = 0xD2 /* BINEX sync (little-endian,regular-crc,rev) */
	BNXSYNC2R = 0xF2 /* BINEX sync (big-endian   ,regular-crc,rev) */
	BNXSYNC3R = 0xD8 /* BINEX sync (little-endian,enhanced-crc,rev) */
	BNXSYNC4R = 0xF8 /* BINEX sync (big-endian   ,enhanced-crc,rev) */
)

// #define MIN(x,y)    ((x)<(y)?(x):(y))
// #define SQR(x)      ((x)*(x))

/* URA table (URA index . URA value) ----------------------------------------*/
// declared in renix.go commented by fxb
// var ura_eph = []float64{
// 	2.4, 3.4, 4.85, 6.85, 9.65, 13.65, 24.0, 48.0, 96.0, 192.0, 384.0, 768.0, 1536.0,
// 	3072.0, 6144.0, 0.0,
// }

/* get fields (big-endian) ---------------------------------------------------*/

func U1(p []uint8) uint8 {
	return p[0]
}

func I1(p []uint8) int8 {
	return int8(p[0])
}
func U2(p []uint8) uint16 {
	return binary.BigEndian.Uint16(p)
}

func U4(p []uint8) uint32 {
	return binary.BigEndian.Uint32(p)
}
func I4(p []uint8) int32 {
	return int32(U4(p))
}
func R4(p []uint8) float32 {
	bits := binary.BigEndian.Uint32(p)
	return math.Float32frombits(bits)
}
func R8(p []uint8) float64 {
	bits := binary.BigEndian.Uint64(p)
	return math.Float64frombits(bits)
}

/* get BINEX 1-4 byte unsigned integer (big endian) --------------------------*/
func getbnxi(p []uint8, val *uint32) int {
	*val = binary.BigEndian.Uint32(p)
	return 4
}

/* checksum 8 parity ---------------------------------------------------------*/
func csum8(buff []uint8, length int) uint8 {
	var cs uint8 = 0

	for i := 0; i < length; i++ {
		cs ^= buff[i]
	}
	return cs
}

/* adjust weekly rollover of GPS time ----------------------------------------*/
func adjweek(time Gtime, tow float64) Gtime {
	var week int
	tow_p := Time2GpsT(time, &week)
	if tow < tow_p-302400.0 {
		tow += 604800.0
	} else if tow > tow_p+302400.0 {
		tow -= 604800.0
	}
	return GpsT2Time(week, tow)
}

/* adjust daily rollover of time ---------------------------------------------*/
func adjday(time Gtime, tod float64) Gtime {
	var ep [6]float64
	var tod_p float64
	Time2Epoch(time, ep[:])

	tod_p = ep[3]*3600.0 + ep[4]*60.0 + ep[5]
	if tod < tod_p-43200.0 {
		tod += 86400.0
	} else if tod > tod_p+43200.0 {
		tod -= 86400.0
	}
	ep[3], ep[4], ep[5] = 0.0, 0.0, 0.0
	return TimeAdd(Epoch2Time(ep[:]), tod)
}

/* URA value (m) to URA index ------------------------------------------------*/
func uraindex(value float64) int {
	var i int
	for i = 0; i < 15; i++ {
		if ura_eph[i] >= value {
			break
		}
	}
	return i
}

/* Galileo SISA value (m) to SISA index --------------------------------------*/
func sisaindex(value float64) int {
	switch {
	case (value < 0.5):
		return int((value) / 0.01)
	case value < 1.0:
		return int((value-0.5)/0.02) + 50
	case value < 2.0:
		return (int)((value-1.0)/0.04) + 75
	case value <= 6.0:
		return (int)((value-2.0)/0.16) + 100
	}
	return 255 /* NAPA */
}

/* decode BINEX mesaage 0x00: site metadata ----------------------------------*/
func decode_bnx_00(raw *Raw, buff []uint8, length int) int {
	var (
		gpst0                     []float64 = []float64{1980, 1, 6, 0, 0, 0}
		x                         [3]float64
		str                       string
		min, qsec, src, fid, flen uint32
		i, ret, idx               int
	)
	min = uint32(U4(buff[idx:]))
	idx += 4
	qsec = uint32(U1(buff[idx:]))
	idx += 1
	src = uint32(U1(buff[idx:]))
	idx += 1
	raw.Time = TimeAdd(Epoch2Time(gpst0), float64(min)*60.0+float64(qsec)*0.25)

	if raw.OutType > 0 {
		copy(raw.MsgType[len(string(raw.MsgType[:])):], []byte(fmt.Sprintf(" time=%s src=%d", TimeStr(raw.Time, 0), src)))
	}
	for idx < length {
		idx += getbnxi(buff[idx:], &fid)
		if idx >= length {
			break
		}

		if fid <= 0x0c || (fid >= 0x0f && fid <= 0x1c) || (fid >= 0x20 && fid <= 0x22) ||
			fid == 0x7f {
			idx += getbnxi(buff[idx:], &flen) /* field length*/
			str = fmt.Sprintf("%.*s", math.Min(float64(flen), float64(MAXANT-1)), buff[idx:])
			idx += int(flen)
			if raw.OutType > 0 {
				copy(raw.MsgType[len(string(raw.MsgType[:])):], []byte(fmt.Sprintf(" [%02x]%s", fid, str)))
			}
			switch fid {
			case 0x08:
				raw.StaData.Name = str
			case 0x09:
				raw.StaData.Marker = str
			case 0x17:
				raw.StaData.AntDes = str
			case 0x18:
				raw.StaData.AntSno = str
			case 0x19:
				raw.StaData.Type = str
			case 0x1a:
				raw.StaData.RecSN = str
			case 0x1b:
				raw.StaData.RecVer = str
			}
			ret = 5
		} else if fid == 0x1d || fid == 0x1e || fid == 0x1f {
			if fid == 0x1d || fid == 0x1e {
				idx += getbnxi(buff[idx:], &flen) /* subfield length */
				idx += int(flen)
			}
			for i = 0; i < 3; i++ {
				x[i] = R8(buff[idx:])
				idx += 8
			}
			if raw.OutType > 0 {
				copy(raw.MsgType[len(string(raw.MsgType[:])):], []byte(fmt.Sprintf(" [%02x]%.3f/%.3f/%.3f", fid, x[0], x[1], x[2])))
			}
			if fid == 0x1d { /* antenna ECEF X/Y/Z position */
				MatCpy(raw.StaData.Pos[:], x[:], 3, 1)
			} else if fid == 0x1e { /* antenna geographic position */
				x[0] *= D2R
				x[1] *= D2R
				Pos2Ecef(x[:], raw.StaData.Pos[:])
			} else if fid == 0x1f { /* antenna offset (H/E/N) */
				raw.StaData.DelType = 0 /* (E/N/U) */
				raw.StaData.Del[0] = x[1]
				raw.StaData.Del[1] = x[2]
				raw.StaData.Del[2] = x[0]
			}
			ret = 5
		} else {
			Trace(2, "BINEX 0x00: unsupported field fid=0x%02x\n", fid)
			break
		}

	}
	return ret
}

/* decode BINEX mesaage 0x01-00: coded (raw bytes) GNSS ephemeris ------------*/
func decode_bnx_01_00(raw *Raw, buff []uint8, length int) int {
	Trace(2, "BINEX 0x01-00: unsupported message\n")
	return 0
}

/* decode BINEX mesaage 0x01-01: decoded GPS ephmemeris ----------------------*/
func decode_bnx_01_01(raw *Raw, buff []uint8, length int) int {
	var (
		eph                 Eph
		tow, ura, sqrtA     float64
		prn, sat, flag, idx int
	)

	Trace(4, "BINEX 0x01-01: length=%d\n", length)

	if length >= 127 {
		prn = int(U1(buff[idx:]) + 1)
		idx += 1
		eph.Week = int(U2(buff[idx:]))
		idx += 2
		tow = float64(I4(buff[idx:]))
		idx += 4
		eph.Toes = float64(I4(buff[idx:]))
		idx += 4
		eph.Tgd[0] = float64(R4(buff[idx:]))
		idx += 4
		eph.Iodc = int(I4(buff[idx:]))
		idx += 4
		eph.F2 = float64(R4(buff[idx:]))
		idx += 4
		eph.F1 = float64(R4(buff[idx:]))
		idx += 4
		eph.F0 = float64(R4(buff[idx:]))
		idx += 4
		eph.Iode = int(I4(buff[idx:]))
		idx += 4
		eph.Deln = float64(R4(buff[idx:])) * SC2RAD
		idx += 4
		eph.M0 = R8(buff[idx:])
		idx += 8
		eph.E = R8(buff[idx:])
		idx += 8
		sqrtA = R8(buff[idx:])
		idx += 8
		eph.Cic = float64(R4(buff[idx:]))
		idx += 4
		eph.Crc = float64(R4(buff[idx:]))
		idx += 4
		eph.Cis = float64(R4(buff[idx:]))
		idx += 4
		eph.Crs = float64(R4(buff[idx:]))
		idx += 4
		eph.Cuc = float64(R4(buff[idx:]))
		idx += 4
		eph.Cus = float64(R4(buff[idx:]))
		idx += 4
		eph.OMG0 = R8(buff[idx:])
		idx += 8
		eph.Omg = R8(buff[idx:])
		idx += 8
		eph.I0 = R8(buff[idx:])
		idx += 8
		eph.OMGd = float64(R4(buff[idx:])) * SC2RAD
		idx += 4
		eph.Idot = float64(R4(buff[idx:])) * SC2RAD
		idx += 4
		ura = float64(R4(buff[idx:])) * 0.1
		idx += 4
		eph.Svh = int(U2(buff[idx:]))
		idx += 2
		flag = int(U2(buff[idx:]))
	} else {
		Trace(2, "BINEX 0x01-01: length error length=%d\n", length)
		return -1
	}
	if sat = SatNo(SYS_GPS, prn); sat == 0 {
		Trace(2, "BINEX 0x01-01: satellite error prn=%d\n", prn)
		return -1
	}
	eph.Sat = sat
	eph.A = sqrtA * sqrtA
	eph.Toe = GpsT2Time(eph.Week, eph.Toes)
	eph.Toc = GpsT2Time(eph.Week, eph.Toes)
	eph.Ttr = adjweek(eph.Toe, tow)
	eph.Fit = float64(flag & 0xFF)
	eph.Flag = (flag >> 8) & 0x01
	eph.Code = (flag >> 9) & 0x03
	eph.Sva = uraindex(ura)

	if !strings.Contains(raw.Opt, "-EPHALL") {
		if raw.NavData.Ephs[eph.Sat-1].Iode == eph.Iode &&
			raw.NavData.Ephs[eph.Sat-1].Iodc == eph.Iodc {
			return 0
		} /* unchanged */
	}
	raw.NavData.Ephs[sat-1] = eph
	raw.EphSat = sat
	raw.EphSet = 0
	return 2
}

/* decode BINEX mesaage 0x01-02: decoded GLONASS ephmemeris ------------------*/
func decode_bnx_01_02(raw *Raw, buff []uint8, length int) int {
	var (
		geph          GEph
		tod, tof      float64
		prn, sat, idx int
	)
	Trace(4, "BINEX 0x01-02: length=%d\n", length)

	if length >= 119 {
		prn = int(U1(buff[idx:])) + 1
		idx += 1
		_ = int(U2(buff[idx:])) //day
		idx += 2
		tod = float64(U4(buff[idx:]))
		idx += 4
		geph.Taun = -R8(buff[idx:])
		idx += 8
		geph.Gamn = R8(buff[idx:])
		idx += 8
		tof = float64(U4(buff[idx:]))
		idx += 4
		geph.Pos[0] = R8(buff[idx:]) * 1e3
		idx += 8
		geph.Vel[0] = R8(buff[idx:]) * 1e3
		idx += 8
		geph.Acc[0] = R8(buff[idx:]) * 1e3
		idx += 8
		geph.Pos[1] = R8(buff[idx:]) * 1e3
		idx += 8
		geph.Vel[1] = R8(buff[idx:]) * 1e3
		idx += 8
		geph.Acc[1] = R8(buff[idx:]) * 1e3
		idx += 8
		geph.Pos[2] = R8(buff[idx:]) * 1e3
		idx += 8
		geph.Vel[2] = R8(buff[idx:]) * 1e3
		idx += 8
		geph.Acc[2] = R8(buff[idx:]) * 1e3
		idx += 8
		geph.Svh = int(U1(buff[idx:]) & 0x1)
		idx += 1 /* MSB of Bn */
		geph.Frq = int(I1(buff[idx:]))
		idx += 1
		geph.Age = int(U1(buff[idx:]))
		idx += 1
		_ = int(U1(buff[idx:])) //leap
		idx += 1
		_ = R8(buff[idx:]) //tau_gps
		idx += 8
		geph.DTaun = R8(buff[idx:])
	} else {
		Trace(2, "BINEX 0x01-02: length error length=%d\n", length)
		return -1
	}
	if sat = SatNo(SYS_GLO, prn); sat == 0 {
		Trace(2, "BINEX 0x01-02: satellite error prn=%d\n", prn)
		return -1
	}
	if raw.Time.Time == 0 {
		return 0
	}
	geph.Sat = sat
	geph.Toe = Utc2GpsT(adjday(raw.Time, tod-10800.0))
	geph.Tof = Utc2GpsT(adjday(raw.Time, tof-10800.0))
	geph.Iode = int(math.Mod(tod, 86400.0)/900.0 + 0.5)

	if !strings.Contains(raw.Opt, "-EPHALL") {
		if math.Abs(TimeDiff(geph.Toe, raw.NavData.Geph[prn-MINPRNGLO].Toe)) < 1.0 &&
			geph.Svh == raw.NavData.Geph[prn-MINPRNGLO].Svh {
			return 0
		}
	}
	raw.NavData.Geph[prn-1] = geph
	raw.EphSat = sat
	raw.EphSet = 0
	return 2
}

/* decode BINEX mesaage 0x01-03: decoded SBAS ephmemeris ---------------------*/
func decode_bnx_01_03(raw *Raw, buff []uint8, length int) int {
	var (
		seph                SEph
		tow, tof            float64
		prn, sat, week, idx int
	)
	Trace(4, "BINEX 0x01-03: length=%d\n", length)

	if length >= 98 {
		prn = int(U1(buff[idx:]))
		idx += 1
		week = int(U2(buff[idx:]))
		idx += 2
		tow = float64(U4(buff[idx:]))
		idx += 4
		seph.Af0 = R8(buff[idx:])
		idx += 8
		_ = float64(R4(buff[idx:])) //tod
		idx += 4
		tof = float64(U4(buff[idx:]))
		idx += 4
		seph.Pos[0] = R8(buff[idx:]) * 1e3
		idx += 8
		seph.Vel[0] = R8(buff[idx:]) * 1e3
		idx += 8
		seph.Acc[0] = R8(buff[idx:]) * 1e3
		idx += 8
		seph.Pos[1] = R8(buff[idx:]) * 1e3
		idx += 8
		seph.Vel[1] = R8(buff[idx:]) * 1e3
		idx += 8
		seph.Acc[1] = R8(buff[idx:]) * 1e3
		idx += 8
		seph.Pos[2] = R8(buff[idx:]) * 1e3
		idx += 8
		seph.Vel[2] = R8(buff[idx:]) * 1e3
		idx += 8
		seph.Acc[2] = R8(buff[idx:]) * 1e3
		idx += 8
		seph.Svh = int(U1(buff[idx:]))
		idx += 1
		seph.Sva = int(U1(buff[idx:]))
		idx += 1
		_ = int(U1(buff[idx:])) //iodn
	} else {
		Trace(2, "BINEX 0x01-03 length error: length=%d\n", length)
		return -1
	}
	if sat = SatNo(SYS_SBS, prn); sat == 0 {
		Trace(2, "BINEX 0x01-03 satellite error: prn=%d\n", prn)
		return -1
	}
	seph.Sat = sat
	seph.T0 = GpsT2Time(week, tow)
	seph.Tof = adjweek(seph.T0, tof)

	if !strings.Contains(raw.Opt, "-EPHALL") {
		if math.Abs(TimeDiff(seph.T0, raw.NavData.Seph[prn-MINPRNSBS].T0)) < 1.0 &&
			seph.Sva == raw.NavData.Seph[prn-MINPRNSBS].Sva {
			return 0
		}
	}
	raw.NavData.Seph[prn-MINPRNSBS] = seph
	raw.EphSat = sat
	raw.EphSet = 0
	return 2
}

/* decode BINEX mesaage 0x01-04: decoded Galileo ephmemeris ------------------*/
func decode_bnx_01_04(raw *Raw, buff []uint8, length int) int {
	var (
		eph                Eph
		tow, ura, sqrtA    float64
		prn, sat, set, idx int
		eph_sel            int = 3 /* ephemeris selection (1:I/NAV+2:F/NAV) */
	)
	Trace(4, "BINEX 0x01-04: length=%d\n", length)

	if strings.Contains(raw.Opt, "-GALFNAV") {
		eph_sel = 1
	}
	if strings.Contains(raw.Opt, "-GALINAV") {
		eph_sel = 2
	}

	if length >= 127 {
		prn = int(U1(buff[idx:])) + 1
		idx += 1
		eph.Week = int(U2(buff[idx:]))
		idx += 2 /* gal-week = gps-week */
		tow = float64(I4(buff[idx:]))
		idx += 4
		eph.Toes = float64(I4(buff[idx:]))
		idx += 4
		eph.Tgd[0] = float64(R4(buff[idx:]))
		idx += 4 /* BGD E5a/E1 */
		eph.Tgd[1] = float64(R4(buff[idx:]))
		idx += 4 /* BGD E5b/E1 */
		eph.Iode = int(I4(buff[idx:]))
		idx += 4 /* IODnav */
		eph.F2 = float64(R4(buff[idx:]))
		idx += 4
		eph.F1 = float64(R4(buff[idx:]))
		idx += 4
		eph.F0 = float64(R4(buff[idx:]))
		idx += 4
		eph.Deln = float64(R4(buff[idx:])) * SC2RAD
		idx += 4
		eph.M0 = R8(buff[idx:])
		idx += 8
		eph.E = R8(buff[idx:])
		idx += 8
		sqrtA = R8(buff[idx:])
		idx += 8
		eph.Cic = float64(R4(buff[idx:]))
		idx += 4
		eph.Crc = float64(R4(buff[idx:]))
		idx += 4
		eph.Cis = float64(R4(buff[idx:]))
		idx += 4
		eph.Crs = float64(R4(buff[idx:]))
		idx += 4
		eph.Cuc = float64(R4(buff[idx:]))
		idx += 4
		eph.Cus = float64(R4(buff[idx:]))
		idx += 4
		eph.OMG0 = R8(buff[idx:])
		idx += 8
		eph.Omg = R8(buff[idx:])
		idx += 8
		eph.I0 = R8(buff[idx:])
		idx += 8
		eph.OMGd = float64(R4(buff[idx:])) * SC2RAD
		idx += 4
		eph.Idot = float64(R4(buff[idx:])) * SC2RAD
		idx += 4
		ura = float64(R4(buff[idx:]))
		idx += 4
		eph.Svh = int(U2(buff[idx:]))
		idx += 2
		eph.Code = int(U2(buff[idx:])) /* data source defined as RINEX 3.03 */
	} else {
		Trace(2, "BINEX 0x01-04: length error length=%d\n", length)
		return -1
	}
	if sat = SatNo(SYS_GAL, prn); sat == 0 {
		Trace(2, "BINEX 0x01-04: satellite error prn=%d\n", prn)
		return -1
	}
	set = 0
	if eph.Code&(1<<8) > 0 {
		set = 1
	} /* 0:I/NAV,1:F/NAV */
	if (eph_sel&1 == 0) && set == 0 {
		return 0
	}
	if (eph_sel&2 == 0) && set == 1 {
		return 0
	}

	eph.Sat = sat
	eph.A = sqrtA * sqrtA
	eph.Iodc = eph.Iode
	eph.Toe = GpsT2Time(eph.Week, eph.Toes)
	eph.Toc = GpsT2Time(eph.Week, eph.Toes)
	eph.Ttr = adjweek(eph.Toe, tow)
	eph.Sva = sisaindex(ura)
	if ura < 0.0 {
		eph.Sva = int(-ura) - 1
	} /* SISA index */
	if !strings.Contains(raw.Opt, "-EPHALL") {
		if raw.NavData.Ephs[sat-1+MAXSAT*set].Iode == eph.Iode &&
			math.Abs(TimeDiff(raw.NavData.Ephs[sat-1+MAXSAT*set].Toe, eph.Toe)) < 1.0 &&
			math.Abs(TimeDiff(raw.NavData.Ephs[sat-1+MAXSAT*set].Toc, eph.Toc)) < 1.0 {
			return 0
		}
	}
	raw.NavData.Ephs[sat-1+MAXSAT*set] = eph
	raw.EphSat = sat
	raw.EphSet = set
	return 2
}

/* BDS signed 10 bit Tgd . sec ----------------------------------------------*/
func bds_tgd(tgd int) float64 {
	tgd &= 0x3FF
	if (tgd & 0x200) > 0 {
		return (-1.0e-10) * float64((^tgd)&0x1FF)
	}
	return 1.0e-10 * float64(tgd&0x1FF)
}

/* decode BINEX mesaage 0x01-05: decoded Beidou-2/Compass ephmemeris ---------*/
func decode_bnx_01_05(raw *Raw, buff []uint8, length int) int {
	var (
		eph                         Eph
		tow, sqrtA                  float64
		prn, sat, flag1, flag2, idx int
	)

	Trace(4, "BINEX 0x01-05: length=%d\n", length)

	if length >= 117 {
		prn = int(U1(buff[idx:]))
		idx += 1
		eph.Week = int(U2(buff[idx:]))
		idx += 2
		tow = float64(I4(buff[idx:]))
		idx += 4
		_ = I4(buff[idx:]) //toc
		idx += 4
		eph.Toes = float64(I4(buff[idx:]))
		idx += 4
		eph.F2 = float64(R4(buff[idx:]))
		idx += 4
		eph.F1 = float64(R4(buff[idx:]))
		idx += 4
		eph.F0 = float64(R4(buff[idx:]))
		idx += 4
		eph.Deln = float64(R4(buff[idx:])) * SC2RAD
		idx += 4
		eph.M0 = R8(buff[idx:])
		idx += 8
		eph.E = R8(buff[idx:])
		idx += 8
		sqrtA = R8(buff[idx:])
		idx += 8
		eph.Cic = float64(R4(buff[idx:]))
		idx += 4
		eph.Crc = float64(R4(buff[idx:]))
		idx += 4
		eph.Cis = float64(R4(buff[idx:]))
		idx += 4
		eph.Crs = float64(R4(buff[idx:]))
		idx += 4
		eph.Cuc = float64(R4(buff[idx:]))
		idx += 4
		eph.Cus = float64(R4(buff[idx:]))
		idx += 4
		eph.OMG0 = R8(buff[idx:])
		idx += 8
		eph.Omg = R8(buff[idx:])
		idx += 8
		eph.I0 = R8(buff[idx:])
		idx += 8
		eph.OMGd = float64(R4(buff[idx:])) * SC2RAD
		idx += 4
		eph.Idot = float64(R4(buff[idx:])) * SC2RAD
		idx += 4
		flag1 = int(U2(buff[idx:]))
		idx += 2
		flag2 = int(U4(buff[idx:]))
	} else {
		Trace(2, "BINEX 0x01-05: length error length=%d\n", length)
		return -1
	}
	if sat = SatNo(SYS_CMP, prn); sat == 0 {
		Trace(2, "BINEX 0x01-05: satellite error prn=%d\n", prn)
		return 0
	}
	eph.Sat = sat
	eph.A = sqrtA * sqrtA
	eph.Toe = GpsT2Time(eph.Week+1356, eph.Toes+14.0) /* bdt . gpst */
	eph.Toc = GpsT2Time(eph.Week+1356, eph.Toes+14.0) /* bdt . gpst */
	eph.Ttr = adjweek(eph.Toe, tow+14.0)              /* bdt . gpst */
	eph.Iodc = (flag1 >> 1) & 0x1F
	eph.Iode = (flag1 >> 6) & 0x1F
	eph.Svh = flag1 & 0x01
	eph.Sva = flag2 & 0x0F            /* URA index */
	eph.Tgd[0] = bds_tgd(flag2 >> 4)  /* TGD1 (s) */
	eph.Tgd[1] = bds_tgd(flag2 >> 14) /* TGD2 (s) */
	eph.Flag = (flag1 >> 11) & 0x07   /* nav type (0:unknown,1:IGSO/MEO,2:GEO) */
	eph.Code = (flag2 >> 25) & 0x7F
	/* message source (0:unknown,1:B1I,2:B1Q,3:B2I,4:B2Q,5:B3I,6:B3Q)*/

	if !strings.Contains(raw.Opt, "-EPHALL") {
		if math.Abs(TimeDiff(raw.NavData.Ephs[sat-1].Toe, eph.Toe)) < 1.0 {
			return 0
		}
	}
	raw.NavData.Ephs[sat-1] = eph
	raw.EphSat = sat
	raw.EphSet = 0
	return 2
}

/* decode BINEX mesaage 0x01-06: decoded QZSS ephmemeris ---------------------*/
func decode_bnx_01_06(raw *Raw, buff []uint8, length int) int {
	var (
		eph                 Eph
		tow, ura, sqrtA     float64
		prn, sat, flag, idx int
	)

	Trace(4, "BINEX 0x01-06: length=%d\n", length)

	if length >= 127 {
		prn = int(U1(buff[idx:]))
		idx += 1
		eph.Week = int(U2(buff[idx:]))
		idx += 2
		tow = float64(I4(buff[idx:]))
		idx += 4
		eph.Toes = float64(I4(buff[idx:]))
		idx += 4
		eph.Tgd[0] = float64(R4(buff[idx:]))
		idx += 4
		eph.Iodc = int(I4(buff[idx:]))
		idx += 4
		eph.F2 = float64(R4(buff[idx:]))
		idx += 4
		eph.F1 = float64(R4(buff[idx:]))
		idx += 4
		eph.F0 = float64(R4(buff[idx:]))
		idx += 4
		eph.Iode = int(I4(buff[idx:]))
		idx += 4
		eph.Deln = float64(R4(buff[idx:])) * SC2RAD
		idx += 4
		eph.M0 = R8(buff[idx:])
		idx += 8
		eph.E = R8(buff[idx:])
		idx += 8
		sqrtA = R8(buff[idx:])
		idx += 8
		eph.Cic = float64(R4(buff[idx:]))
		idx += 4
		eph.Crc = float64(R4(buff[idx:]))
		idx += 4
		eph.Cis = float64(R4(buff[idx:]))
		idx += 4
		eph.Crs = float64(R4(buff[idx:]))
		idx += 4
		eph.Cuc = float64(R4(buff[idx:]))
		idx += 4
		eph.Cus = float64(R4(buff[idx:]))
		idx += 4
		eph.OMG0 = R8(buff[idx:])
		idx += 8
		eph.Omg = R8(buff[idx:])
		idx += 8
		eph.I0 = R8(buff[idx:])
		idx += 8
		eph.OMGd = float64(R4(buff[idx:])) * SC2RAD
		idx += 4
		eph.Idot = float64(R4(buff[idx:])) * SC2RAD
		idx += 4
		ura = float64(R4(buff[idx:])) * 0.1
		idx += 4
		eph.Svh = int(U2(buff[idx:]))
		idx += 2
		flag = int(U2(buff[idx:]))
	} else {
		Trace(2, "BINEX 0x01-06: length error length=%d\n", length)
		return -1
	}
	if sat = SatNo(SYS_QZS, prn); sat == 0 {
		Trace(2, "BINEX 0x01-06: satellite error prn=%d\n", prn)
		return 0
	}
	eph.Sat = sat
	eph.A = sqrtA * sqrtA
	eph.Toe = GpsT2Time(eph.Week, eph.Toes)
	eph.Toc = GpsT2Time(eph.Week, eph.Toes)
	eph.Ttr = adjweek(eph.Toe, tow)
	eph.Fit = 2.0
	if flag&0x01 > 0 {
		eph.Fit = 0.0
	} /* 0:2hr,1:>2hr */
	eph.Sva = uraindex(ura)
	eph.Code = 2 /* codes on L2 channel */

	if !strings.Contains(raw.Opt, "-EPHALL") {
		if raw.NavData.Ephs[sat-1].Iode == eph.Iode &&
			raw.NavData.Ephs[sat-1].Iodc == eph.Iodc {
			return 0
		}
	}
	raw.NavData.Ephs[sat-1] = eph
	raw.EphSat = sat
	raw.EphSet = 0
	return 2
}

/* decode BINEX mesaage 0x01-07: decoded IRNSS ephmemeris --------------------*/
func decode_bnx_01_07(raw *Raw, buff []uint8, length int) int {
	var (
		eph                                Eph
		tow, toc, sqrtA                    float64
		prn, sat, flag, iodec, idx, s1, s2 int
	)

	Trace(4, "BINEX 0x01-07: length=%d\n", length)

	if length >= 114 {
		prn = int(U1(buff[idx:]))
		idx += 1
		eph.Week = int(U2(buff[idx:])) + 1024
		idx += 2 /* IRNSS week . GPS week */
		tow = float64(I4(buff[idx:]))
		idx += 4
		toc = float64(I4(buff[idx:]))
		idx += 4
		eph.Toes = float64(I4(buff[idx:]))
		idx += 4
		eph.F2 = float64(R4(buff[idx:]))
		idx += 4
		eph.F1 = float64(R4(buff[idx:]))
		idx += 4
		eph.F0 = float64(R4(buff[idx:]))
		idx += 4
		eph.Deln = float64(R4(buff[idx:])) * SC2RAD
		idx += 4
		eph.M0 = R8(buff[idx:])
		idx += 8
		eph.E = R8(buff[idx:])
		idx += 8
		sqrtA = R8(buff[idx:])
		idx += 8
		eph.Cic = float64(R4(buff[idx:]))
		idx += 4
		eph.Crc = float64(R4(buff[idx:]))
		idx += 4
		eph.Cis = float64(R4(buff[idx:]))
		idx += 4
		eph.Crs = float64(R4(buff[idx:]))
		idx += 4
		eph.Cuc = float64(R4(buff[idx:]))
		idx += 4
		eph.Cus = float64(R4(buff[idx:]))
		idx += 4
		eph.OMG0 = R8(buff[idx:])
		idx += 8
		eph.Omg = R8(buff[idx:])
		idx += 8
		eph.I0 = R8(buff[idx:])
		idx += 8
		eph.OMGd = float64(R4(buff[idx:])) * SC2RAD
		idx += 4
		eph.Idot = float64(R4(buff[idx:])) * SC2RAD
		idx += 4
		flag = int(U1(buff[idx:]))
		idx += 1
		iodec = int(U2(buff[idx:]))
	} else {
		Trace(2, "BINEX 0x01-07: length error length=%d\n", length)
		return -1
	}
	if sat = SatNo(SYS_IRN, prn); sat == 0 {
		Trace(2, "BINEX 0x01-07: satellite error prn=%d\n", prn)
		return 0
	}
	eph.Sat = sat
	eph.A = sqrtA * sqrtA
	eph.Toe = GpsT2Time(eph.Week, eph.Toes)
	eph.Toc = adjweek(eph.Toe, toc)
	eph.Ttr = adjweek(eph.Toe, tow)
	eph.Sva = flag & 0x0F /* URA index (0-15) */
	eph.Svh = 0

	if flag&0x10 > 0 {
		s1 = 2
	}
	if flag&0x20 > 0 {
		s2 = 1
	}
	eph.Svh = s1 + s2
	eph.Iode, eph.Iodc = iodec&0xFF, iodec&0xFF
	eph.Tgd[0] = float64(iodec>>8) * P2_31

	if !strings.Contains(raw.Opt, "-EPHALL") {
		if raw.NavData.Ephs[sat-1].Iode == eph.Iode &&
			math.Abs(TimeDiff(raw.NavData.Ephs[sat-1].Toe, eph.Toe)) < 1.0 &&
			math.Abs(TimeDiff(raw.NavData.Ephs[sat-1].Toc, eph.Toc)) < 1.0 {
			return 0
		}
	}
	raw.NavData.Ephs[sat-1] = eph
	raw.EphSat = sat
	raw.EphSet = 0
	return 2
}

/* decode BINEX mesaage 0x01-14: upgraded decoded Galileo ephmemeris ---------*/
func decode_bnx_01_14(raw *Raw, buff []uint8, length int) int {
	var (
		eph                      Eph
		tow, ura, sqrtA          float64
		prn, sat, tocs, set, idx int
		eph_sel                  int = 3
	)

	Trace(4, "BINEX 0x01-14: length=%d\n", length)

	if strings.Contains(raw.Opt, "-GALFNAV") {
		eph_sel = 1
	}
	if strings.Contains(raw.Opt, "-GALINAV") {
		eph_sel = 2
	}

	if length >= 135 {
		prn = int(U1(buff[idx:])) + 1
		idx += 1
		eph.Week = int(U2(buff[idx:]))
		idx += 2 /* gal-week = gps-week */
		tow = float64(I4(buff[idx:]))
		idx += 4
		tocs = int(I4(buff[idx:]))
		idx += 4
		eph.Toes = float64(I4(buff[idx:]))
		idx += 4
		eph.Tgd[0] = float64(R4(buff[idx:]))
		idx += 4 /* BGD E5a/E1 */
		eph.Tgd[1] = float64(R4(buff[idx:]))
		idx += 4 /* BGD E5b/E1 */
		eph.Iode = int(I4(buff[idx:]))
		idx += 4 /* IODnav */
		eph.F2 = float64(R4(buff[idx:]))
		idx += 4
		eph.F1 = float64(R4(buff[idx:]))
		idx += 4
		eph.F0 = R8(buff[idx:])
		idx += 8
		eph.Deln = float64(R4(buff[idx:])) * SC2RAD
		idx += 4
		eph.M0 = R8(buff[idx:])
		idx += 8
		eph.E = R8(buff[idx:])
		idx += 8
		sqrtA = R8(buff[idx:])
		idx += 8
		eph.Cic = float64(R4(buff[idx:]))
		idx += 4
		eph.Crc = float64(R4(buff[idx:]))
		idx += 4
		eph.Cis = float64(R4(buff[idx:]))
		idx += 4
		eph.Crs = float64(R4(buff[idx:]))
		idx += 4
		eph.Cuc = float64(R4(buff[idx:]))
		idx += 4
		eph.Cus = float64(R4(buff[idx:]))
		idx += 4
		eph.OMG0 = R8(buff[idx:])
		idx += 8
		eph.Omg = R8(buff[idx:])
		idx += 8
		eph.I0 = R8(buff[idx:])
		idx += 8
		eph.OMGd = float64(R4(buff[idx:])) * SC2RAD
		idx += 4
		eph.Idot = float64(R4(buff[idx:])) * SC2RAD
		idx += 4
		ura = float64(R4(buff[idx:]))
		idx += 4
		eph.Svh = int(U2(buff[idx:]))
		idx += 2
		eph.Code = int(U2(buff[idx:])) /* data source defined as RINEX 3.03 */
	} else {
		Trace(2, "BINEX 0x01-14: length error length=%d\n", length)
		return -1
	}
	if sat = SatNo(SYS_GAL, prn); sat == 0 {
		Trace(2, "BINEX 0x01-14: satellite error prn=%d\n", prn)
		return -1
	}
	if eph.Code&(1<<8) > 0 {
		set = 1
	} /* 0:I/NAV,1:F/NAV */
	if (eph_sel&1) == 0 && set == 0 {
		return 0
	}
	if (eph_sel&2) == 0 && set == 1 {
		return 0
	}

	eph.Sat = sat
	eph.A = (sqrtA * sqrtA)
	eph.Iodc = eph.Iode
	eph.Toe = GpsT2Time(eph.Week, eph.Toes)
	eph.Toc = GpsT2Time(eph.Week, float64(tocs))
	eph.Ttr = adjweek(eph.Toe, tow)
	eph.Sva = sisaindex(ura)
	if ura < 0.0 {
		eph.Sva = int(-ura) - 1
	} /* SISA index */
	if !strings.Contains(raw.Opt, "-EPHALL") {
		if raw.NavData.Ephs[sat-1+MAXSAT*set].Iode == eph.Iode &&
			math.Abs(TimeDiff(raw.NavData.Ephs[sat-1+MAXSAT*set].Toe, eph.Toe)) < 1.0 &&
			math.Abs(TimeDiff(raw.NavData.Ephs[sat-1+MAXSAT*set].Toc, eph.Toc)) < 1.0 {
			return 0
		}
	}
	raw.NavData.Ephs[sat-1+MAXSAT*set] = eph
	raw.EphSat = sat
	raw.EphSet = set
	return 2
}

/* decode BINEX mesaage 0x01: GNSS navigation information --------------------*/
func decode_bnx_01(raw *Raw, buff []uint8, length int) int {
	srec := U1(buff[:])
	prn := U1(buff[1:])

	if raw.OutType > 0 {
		if srec == 0x01 || srec == 0x02 || srec == 0x04 {
			prn += 1
		} else if srec == 0x00 {
			prn = 0
		}
		copy(raw.MsgType[len(string(raw.MsgType[:])):], []byte(fmt.Sprintf(" subrec=%02X prn=%d", srec, prn)))
	}
	switch srec {
	case 0x00:
		return decode_bnx_01_00(raw, buff[1:], length-1)
	case 0x01:
		return decode_bnx_01_01(raw, buff[1:], length-1)
	case 0x02:
		return decode_bnx_01_02(raw, buff[1:], length-1)
	case 0x03:
		return decode_bnx_01_03(raw, buff[1:], length-1)
	case 0x04:
		return decode_bnx_01_04(raw, buff[1:], length-1)
	case 0x05:
		return decode_bnx_01_05(raw, buff[1:], length-1)
	case 0x06:
		return decode_bnx_01_06(raw, buff[1:], length-1)
	case 0x07:
		return decode_bnx_01_07(raw, buff[1:], length-1)
	case 0x14:
		return decode_bnx_01_14(raw, buff[1:], length-1)
	}
	return 0
}

/* decode BINEX mesaage 0x02: generalized GNSS data --------------------------*/
func decode_bnx_02(raw *Raw, buff []uint8, length int) int {
	Trace(2, "BINEX 0x02: unsupported message\n")
	return 0
}

/* decode BINEX mesaage 0x03: generalized ancillary site data ----------------*/
func decode_bnx_03(raw *Raw, buff []uint8, length int) int {
	Trace(2, "BINEX 0x03: unsupported message\n")
	return 0
}

/* decode BINEX mesaage 0x7d: receiver internal state prototyping ------------*/
func decode_bnx_7d(raw *Raw, buff []uint8, length int) int {
	Trace(2, "BINEX 0x7d: unsupported message\n")
	return 0
}

/* decode BINEX mesaage 0x7e: ancillary site data prototyping ----------------*/
func decode_bnx_7e(raw *Raw, buff []uint8, length int) int {
	Trace(2, "BINEX 0x7e: unsupported message\n")
	return 0
}

/* decode BINEX mesaage 0x7f-00: JPL fiducial site ---------------------------*/
func decode_bnx_7f_00(raw *Raw, buff []uint8, length int) int {
	Trace(2, "BINEX 0x7f-00: unsupported message\n")
	return 0
}

/* decode BINEX mesaage 0x7f-01: UCAR COSMIC ---------------------------------*/
func decode_bnx_7f_01(raw *Raw, buff []uint8, length int) int {
	Trace(2, "BINEX 0x7f-01: unsupported message\n")
	return 0
}

/* decode BINEX mesaage 0x7f-02: Trimble 4700 --------------------------------*/
func decode_bnx_7f_02(raw *Raw, buff []uint8, length int) int {
	Trace(2, "BINEX 0x7f-02: unsupported message\n")
	return 0
}

/* decode BINEX mesaage 0x7f-03: Trimble NetRS -------------------------------*/
func decode_bnx_7f_03(raw *Raw, buff []uint8, length int) int {
	Trace(2, "BINEX 0x7f-03: unsupported message\n")
	return 0
}

/* decode BINEX mesaage 0x7f-04: Trimble NetRS -------------------------------*/
func decode_bnx_7f_04(raw *Raw, buff []uint8, length int) int {
	Trace(2, "BINEX 0x7f-04: unsupported message\n")
	return 0
}

/* decode BINEX mesaage 0x7f-05: Trimble NetR8 obs data ----------------------*/
func decode_bnx_7f_05_obs(raw *Raw, buff []uint8, sat,
	nobs int, data *ObsD) int {
	var codes_gps [32]uint8 = [32]uint8{
		CODE_L1C, CODE_L1C, CODE_L1P, CODE_L1W, CODE_L1Y, CODE_L1M, /*  0- 5 */
		CODE_L1X, CODE_L1N, CODE_NONE, CODE_NONE, CODE_L2W, CODE_L2C, /*  6-11 */
		CODE_L2D, CODE_L2S, CODE_L2L, CODE_L2X, CODE_L2P, CODE_L2W, /* 12-17 */
		CODE_L2Y, CODE_L2M, CODE_L2N, CODE_NONE, CODE_NONE, CODE_L5X, /* 18-23 */
		CODE_L5I, CODE_L5Q, CODE_L5X, /* 24-26 */
	}
	var codes_glo [32]uint8 = [32]uint8{
		CODE_L1C, CODE_L1C, CODE_L1P, CODE_NONE, CODE_NONE, CODE_NONE, /*  0- 5 */
		CODE_NONE, CODE_NONE, CODE_NONE, CODE_NONE, CODE_L2C, CODE_L2C, /*  6-11 */
		CODE_L2P, CODE_L3X, CODE_L3I, CODE_L3Q, CODE_L3X, /* 12-16 */
	}
	var codes_gal [32]uint8 = [32]uint8{
		CODE_L1C, CODE_L1A, CODE_L1B, CODE_L1C, CODE_L1X, CODE_L1Z, /*  0- 5 */
		CODE_L5X, CODE_L5I, CODE_L5Q, CODE_L5X, CODE_L7X, CODE_L7I, /*  6-11 */
		CODE_L7Q, CODE_L7X, CODE_L8X, CODE_L8I, CODE_L8Q, CODE_L8X, /* 12-17 */
		CODE_L6X, CODE_L6A, CODE_L6B, CODE_L6C, CODE_L6X, CODE_L6Z, /* 18-23 */
	}
	var codes_sbs [32]uint8 = [32]uint8{
		CODE_L1C, CODE_L1C, CODE_NONE, CODE_NONE, CODE_NONE, CODE_NONE, /*  0- 5 */
		CODE_L5X, CODE_L5I, CODE_L5Q, CODE_L5X, /*  6- 9 */
	}
	var codes_cmp [32]uint8 = [32]uint8{
		CODE_L2X, CODE_L2I, CODE_L2Q, CODE_L2X, CODE_L7X, CODE_L7I, /*  0- 5 */
		CODE_L7Q, CODE_L7X, CODE_L6X, CODE_L6I, CODE_L6Q, CODE_L6X, /*  6-11 */
		CODE_L1X, CODE_L1D, CODE_L1P, CODE_L1X, CODE_L5X, CODE_L5D, /* 12-17 */
		CODE_L5P, CODE_L5X, CODE_L7Z, CODE_L7D, CODE_L7P, CODE_L7Z, /* 18-23 */
		/* 20-23: extension for BD990 F/W 5.48 */
	}
	var codes_qzs [32]uint8 = [32]uint8{
		CODE_L1C, CODE_L1C, CODE_L1S, CODE_L1L, CODE_L1X, CODE_NONE, /*  0- 5 */
		CODE_NONE, CODE_L2X, CODE_L2S, CODE_L2L, CODE_L2X, CODE_NONE, /*  6-11 */
		CODE_NONE, CODE_L5X, CODE_L5I, CODE_L5Q, CODE_L5X, CODE_NONE, /* 12-17 */
		CODE_NONE, CODE_L6X, CODE_L6S, CODE_L6L, CODE_L6X, CODE_NONE, /* 18-23 */
		CODE_NONE, CODE_NONE, CODE_NONE, CODE_NONE, CODE_NONE, CODE_NONE, /* 24-29 */
		CODE_L1Z, /* 30-30 */
	}
	var codes_irn [32]uint8 = [32]uint8{
		CODE_L5X, CODE_L5A, CODE_L5B, CODE_L5C, CODE_L5X, CODE_L9X, /*  0- 5 */
		CODE_L9A, CODE_L9B, CODE_L9C, CODE_L9X, /*  6- 9 */
	}
	var (
		codes                               []uint8
		frange, phase, cnr, dopp            [8]float64
		acc, freq                           float64
		flag                                uint8
		flags                               [4]uint8
		i, j, k, sys, prn, fcn, index       int
		code, slip, pri, idx, slipcnt, mask [8]int
	)
	fcn = -10
	Trace(5, "decode_bnx_7f_05_obs: sat=%2d nobs=%2d\n", sat, nobs)

	sys = SatSys(sat, &prn)

	switch sys {
	case SYS_GPS:
		codes = codes_gps[:]
	case SYS_GLO:
		codes = codes_glo[:]
	case SYS_GAL:
		codes = codes_gal[:]
	case SYS_QZS:
		codes = codes_qzs[:]
	case SYS_SBS:
		codes = codes_sbs[:]
	case SYS_CMP:
		codes = codes_cmp[:]
	case SYS_IRN:
		codes = codes_irn[:]
	default:
		return 0
	}
	for i = 0; i < nobs; i++ {
		flag = uint8(GetBitU(buff[:], 0, 1))
		slip[i] = int(GetBitU(buff[:], 2, 1))
		code[i] = int(GetBitU(buff[:], 3, 5))
		index++

		for j = 0; j < 4; j++ {
			flags[j] = 0
		}

		for j = 0; flag > 0 && j < 4; j++ {
			flag = U1(buff[index:])
			index++
			flags[flag&0x03] = flag & 0x7F
			flag &= 0x80
		}
		if flags[2] > 0 {
			fcn = int(GetBits(flags[2:], 2, 4))
			if sys == SYS_GLO && raw.NavData.Glo_fcn[prn-1] == 0 {
				raw.NavData.Glo_fcn[prn-1] = fcn + 8 /* fcn+8 */
			}
		}
		acc = 0.00002
		if flags[0]&0x20 > 0 {
			acc = 0.0001
		} /* phase accuracy */

		cnr[i] = float64(U1(buff[index:])) * 0.4
		index++

		if i == 0 {
			cnr[i] += float64(GetBits(buff[index:], 0, 2)) * 0.1
			frange[i] = float64(GetBitU(buff[index:], 2, 32))*0.064 + float64(GetBitU(buff[index:], 34, 6))*0.001
			index += 5
		} else if flags[0]&0x40 > 0 {
			cnr[i] += float64(GetBits(buff[index:], 0, 2)) * 0.1
			frange[i] = frange[0] + float64(GetBits(buff[index:], 4, 20))*0.001
			index += 3
		} else {
			frange[i] = frange[0] + float64(GetBits(buff[index:], 0, 16))*0.001
			index += 2
		}
		if flags[0]&0x40 > 0 {
			phase[i] = frange[i] + float64(GetBits(buff[index:], 0, 24))*acc
			index += 3
		} else {
			cnr[i] += float64(GetBits(buff[index:], 0, 2)) * 0.1
			phase[i] = frange[i] + float64(GetBits(buff[index:], 2, 22))*acc
			index += 3
		}
		if flags[0]&0x04 > 0 {
			dopp[i] = float64(GetBits(buff[index:], 0, 24)) / 256.0
			index += 3
		}
		if flags[0]&0x08 > 0 {
			if flags[0]&0x10 > 0 {
				slipcnt[i] = int(U2(buff[index:]))
				index += 2
			} else {
				slipcnt[i] = int(U1(buff[index:]))
				index += 1
			}
		}
		Trace(5, "(%d) CODE=%2d S=%d F=%02X %02X %02X %02X\n", i+1,
			code[i], slip[i], flags[0], flags[1], flags[2], flags[3])
		Trace(5, "(%d) P=%13.3f L=%13.3f D=%7.1f SNR=%4.1f SCNT=%2d\n",
			i+1, frange[i], phase[i], dopp[i], cnr[i], slipcnt[i])
	}
	if codes == nil {
		data.Sat = 0
		return index
	}
	data.Time = raw.Time
	data.Sat = sat

	/* get code priority */
	for i = 0; i < nobs; i++ {
		idx[i] = Code2Idx(sys, codes[code[i]])
		pri[i] = GetCodePri(sys, codes[code[i]], raw.Opt)
	}
	for i = 0; i < NFREQ; i++ {
		for j, k = 0, -1; j < nobs; j++ {
			if idx[j] == i && (k < 0 || pri[j] > pri[k]) {
				k = j
			}
		}
		if k < 0 {
			data.P[i], data.L[i] = 0.0, 0.0
			data.D[i] = 0.0
			data.SNR[i], data.LLI[i] = 0, 0
			data.Code[i] = CODE_NONE
		} else {
			freq = Code2Freq(sys, codes[code[k]], fcn)
			data.P[i] = frange[k]
			data.L[i] = phase[k] * freq / CLIGHT
			data.D[i] = dopp[k]
			data.SNR[i] = uint16(cnr[k]/SNR_UNIT + 0.5)
			data.Code[i] = codes[code[k]]
			data.LLI[i] = 0
			if slip[k] > 0 {
				data.LLI[i] = 1
			}
			mask[k] = 1
		}
	}
	for ; i < NFREQ+NEXOBS; i++ {
		for k = 0; k < nobs; k++ {
			if mask[k] == 0 {
				break
			}
		}
		if k >= nobs {
			data.P[i], data.L[i] = 0.0, 0.0
			data.D[i] = 0.0
			data.SNR[i], data.LLI[i] = 0, 0
			data.Code[i] = CODE_NONE
		} else {
			freq = Code2Freq(sys, codes[code[k]], fcn)
			data.P[i] = frange[k]
			data.L[i] = phase[k] * freq / CLIGHT
			data.D[i] = dopp[k]
			data.SNR[i] = uint16(cnr[k]/SNR_UNIT + 0.5)
			data.Code[i] = codes[code[k]]
			data.LLI[i] = 0
			if slip[k] > 0 {
				data.LLI[i] = 1
			}
			mask[k] = 1
		}
	}
	return index
}

/* decode BINEX mesaage 0x7f-05: Trimble NetR8 -------------------------------*/
func decode_bnx_7f_05(raw *Raw, buff []uint8, length int) int {
	var (
		data                                           ObsD
		toff                                           [16]float64
		flag                                           uint32
		i, nsat, nobs, prn, sys, sat, nsys, idx, index int
		tsys                                           [16]int
	)

	Trace(4, "decode_bnx_7f_05\n")

	raw.ObsData.n = 0
	flag = uint32(U1(buff[idx:]))
	idx++
	nsat = int(flag&0x3F) + 1

	if flag&0x80 > 0 { /* rxclkoff */
		_ = GetBits(buff[idx:], 0, 2)                    //clkrst
		_ = float64(GetBits(buff[idx:], 2, 22)) * (1e-9) //clkoff
		idx += 3
	}
	if flag&0x40 > 0 { /* systime */
		nsys = int(GetBitU(buff[idx:], 0, 4))
		_ = GetBitU(buff[idx:], 4, 4) //rsys
		idx++
		for i = 0; i < nsys; i++ {
			toff[i] = float64(GetBits(buff[idx:], 0, 24)) * 1e-9
			tsys[i] = int(GetBitU(buff[idx:], 28, 4))
			idx += 4
		}
	}
	for i = 0; i < nsat; i++ {
		prn = int(U1(buff[idx:]))
		idx++
		nobs = int(GetBitU(buff[idx:], 1, 3))
		sys = int(GetBitU(buff[idx:], 4, 4))
		idx++

		Trace(5, "BINEX 0x7F-05 PRN=%3d SYS=%d NOBS=%d\n", prn, sys, nobs)

		switch sys {
		case 0:
			sat = SatNo(SYS_GPS, prn)
		case 1:
			sat = SatNo(SYS_GLO, prn)
		case 2:
			sat = SatNo(SYS_SBS, prn)
		case 3:
			sat = SatNo(SYS_GAL, prn)
		case 4:
			sat = SatNo(SYS_CMP, prn)
		case 5:
			sat = SatNo(SYS_QZS, prn)
		case 6:
			sat = SatNo(SYS_IRN, prn)
		default:
			sat = 0
		}
		/* decode BINEX mesaage 0x7F-05 obs data */
		if index = decode_bnx_7f_05_obs(raw, buff[idx:], sat, nobs, &data); index == 0 {
			return -1
		}

		if index > length {
			Trace(2, "BINEX 0x7F-05 length error: nsat=%2d length=%d\n", nsat, length)
			return -1
		}
		/* save obs data to obs buffer */
		if data.Sat > 0 && raw.ObsData.n < MAXOBS {
			raw.ObsData.Data[raw.ObsData.n] = data
			raw.ObsData.n++
		}
	}
	if raw.OutType > 0 {
		copy(raw.MsgType[len(string(raw.MsgType[:])):], []byte(fmt.Sprintf(" nsat=%2d", nsat)))
	}
	if raw.ObsData.N() > 0 {
		return 1
	}
	return 0
}

/* decode BINEX mesaage 0x7f: GNSS data prototyping --------------------------*/
func decode_bnx_7f(raw *Raw, buff []uint8, length int) int {
	var gpst0 []float64 = []float64{1980, 1, 6, 0, 0, 0}
	var srec, min, msec, idx uint32

	srec = uint32(U1(buff[idx:]))
	idx += 1 /* subrecord ID */
	min = U4(buff[idx:])
	idx += 4
	msec = uint32(U2(buff[idx:]))
	idx += 2
	raw.Time = TimeAdd(Epoch2Time(gpst0), float64(min)*60.0+float64(msec)*0.001)

	if raw.OutType > 0 {
		copy(raw.MsgType[len(string(raw.MsgType[:])):], []byte(fmt.Sprintf(" subrec=%02X time%s", srec, TimeStr(raw.Time, 3))))
	}
	switch srec {
	case 0x00:
		return decode_bnx_7f_00(raw, buff[7:], length-7)
	case 0x01:
		return decode_bnx_7f_01(raw, buff[7:], length-7)
	case 0x02:
		return decode_bnx_7f_02(raw, buff[7:], length-7)
	case 0x03:
		return decode_bnx_7f_03(raw, buff[7:], length-7)
	case 0x04:
		return decode_bnx_7f_04(raw, buff[7:], length-7)
	case 0x05:
		return decode_bnx_7f_05(raw, buff[7:], length-7)
	}
	return 0
}

/* decode BINEX mesaage ------------------------------------------------------*/
func decode_bnx(raw *Raw) int {
	var (
		length, cs1, cs2 uint32
		rec, len_h       int
	)

	rec = int(raw.Buff[1]) /* record ID */

	/* record and header length */
	len_h = getbnxi(raw.Buff[2:], &length)

	Trace(5, "decode_bnx: rec=%02x length=%d\n", rec, length)

	/* check parity */
	if raw.Len-1 < 128 {
		cs1 = uint32(U1(raw.Buff[raw.Len:]))
		cs2 = uint32(csum8(raw.Buff[1:], raw.Len-1))
	} else {
		cs1 = uint32(U2(raw.Buff[raw.Len:]))
		cs2 = uint32(Rtk_CRC16(raw.Buff[1:], raw.Len-1))
	}
	if cs1 != cs2 {
		Trace(2, "BINEX 0x%02X parity error CS=%X %X\n", rec, cs1, cs2)
		return -1
	}
	if raw.OutType > 0 {
		copy(raw.MsgType[len(string(raw.MsgType[:])):], []byte(fmt.Sprintf("BINEX 0x%02X (%4d)", rec, raw.Len)))
	}
	/* decode BINEX message record */
	switch rec {
	case 0x00:
		return decode_bnx_00(raw, raw.Buff[2+len_h:], int(length))
	case 0x01:
		return decode_bnx_01(raw, raw.Buff[2+len_h:], int(length))
	case 0x02:
		return decode_bnx_02(raw, raw.Buff[2+len_h:], int(length))
	case 0x03:
		return decode_bnx_03(raw, raw.Buff[2+len_h:], int(length))
	case 0x7d:
		return decode_bnx_7d(raw, raw.Buff[2+len_h:], int(length))
	case 0x7e:
		return decode_bnx_7e(raw, raw.Buff[2+len_h:], int(length))
	case 0x7f:
		return decode_bnx_7f(raw, raw.Buff[2+len_h:], int(length))
	}
	return 0
}

/* synchronize BINEX message -------------------------------------------------*/
func sync_bnx(buff []uint8, data uint8) int {
	buff[0] = buff[1]
	buff[1] = data

	if int(buff[0]) == BNXSYNC2 &&
		(buff[1] == 0x00 || buff[1] == 0x01 || buff[1] == 0x02 || buff[1] == 0x03 ||
			buff[1] == 0x7D || buff[1] == 0x7E || buff[1] == 0x7F) {
		return 1
	}
	return 0

}

/* input BINEX message from stream ---------------------------------------------
* fetch next BINEX data and input a message from stream
* args   : raw_t *raw       IO  receiver raw data control struct
*          uint8_t data     I   stream data (1 byte)
* return : status (-1: error message, 0: no message, 1: input observation data,
*                  2: input ephemeris, 5: input station pos/ant parameters)
* notes  : support only the following message (ref [1])
*
*          - big-endian, regular CRC, forward record (sync=0xE2)
*          - record-subrecord:
*            0x00   : site metadata (monument,marker,ref point,setup)
*            0x01-01: decoded GPS ephemeris
*            0x01-02: decoded GLONASS ephemeris
*            0x01-03: decoded SBAS ephemeris
*            0x01-04: decoded Galileo ephemeris
*            0x01-05: decoded BDS-2/compass ephemeris
*            0x01-06: decoded QZSS ephemeris
*            0x01-07: decoded IRNSS ephemeris
*            0x01-14: decoded upgraded Galileo ephemeris
*            0x7f-05: GNSS data prototyping - Trimble NetR8
*
*          to specify input options, set rtcm.opt to the following option
*          strings separated by spaces.
*
*          -EPHALL  : input all ephemerides
*          -GLss    : select signal ss for GPS (ss=1C,1P,...)
*          -RLss    : select signal ss for GLO (ss=1C,1P,...)
*          -ELss    : select signal ss for GAL (ss=1C,1B,...)
*          -JLss    : select signal ss for QZS (ss=1C,2C,...)
*          -CLss    : select signal ss for BDS (ss=2I,2X,...)
*          -GALINAV : select I/NAV for Galileo ephemeris (default: all)
*          -GALFNAV : select F/NAV for Galileo ephemeris (default: all)
*-----------------------------------------------------------------------------*/
func Input_bnx(raw *Raw, data uint8) int {
	var (
		length       uint32
		len_h, len_c int
	)

	Trace(5, "input_bnx: data=%02x\n", data)

	/* synchronize BINEX message */
	if raw.NumByte == 0 {
		if sync_bnx(raw.Buff[:], data) == 0 {
			return 0
		}
		raw.NumByte = 2
		return 0
	}
	raw.Buff[raw.NumByte] = data
	raw.NumByte++
	if raw.NumByte < 4 {
		return 0
	}

	len_h = getbnxi(raw.Buff[2:], &length)

	raw.Len = int(length) + len_h + 2 /* length without CRC */

	if raw.Len-1 > 4096 {
		Trace(2, "BINEX length error: length=%d\n", raw.Len-1)
		raw.NumByte = 0
		return -1
	}
	len_c = 2
	if raw.Len-1 < 128 {
		len_c = 1
	}

	if raw.NumByte < int(raw.Len+len_c) {
		return 0
	}
	raw.NumByte = 0

	/* decode BINEX message */
	return decode_bnx(raw)
}

/* input BINEX message from file -----------------------------------------------
* fetch next BINEX data and input a message from file
* args   : raw_t  *raw      IO  receiver raw data control struct
*          FILE   *fp       I   file pointer
* return : status(-2: end of file, -1...9: same as above)
*-----------------------------------------------------------------------------*/
func Input_bnxf(raw *Raw, fp *os.File) int {
	var (
		length          uint32
		i, len_h, len_c int
	)

	Trace(4, "input_bnxf\n")
	var c [1]byte
	if raw.NumByte == 0 {
		for i = 0; ; i++ {
			_, err := fp.Read(c[:])
			if err == io.EOF {
				return -2
			}
			if sync_bnx(raw.Buff[:], uint8(c[0])) > 0 {
				break
			}
			if i >= 4096 {
				return 0
			}
		}
	}
	n, err := fp.Read(raw.Buff[2:6])
	if err == io.EOF || n < 4 {
		return -2
	}

	len_h = getbnxi(raw.Buff[2:], &length)

	raw.Len = int(length) + len_h + 2

	if raw.Len-1 > 4096 {
		Trace(2, "BINEX length error: length=%d\n", raw.Len-1)
		raw.NumByte = 0
		return -1
	}
	len_c = 2
	if raw.Len-1 < 128 {
		len_c = 1
	}
	n, err = fp.Read(raw.Buff[6 : raw.Len+len_c])
	if n < raw.Len+len_c-6 {
		return -2
	}
	raw.NumByte = 0

	/* decode BINEX message */
	return decode_bnx(raw)
}
