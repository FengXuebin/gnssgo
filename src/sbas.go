/*------------------------------------------------------------------------------
* sbas.c : sbas functions
*
*          Copyright (C) 2007-2020 by T.TAKASU, All rights reserved.
*
* option : -DRRCENA  enable rrc correction
*
* references :
*     [1] RTCA/DO-229C, Minimum operational performanc standards for global
*         positioning system/wide area augmentation system airborne equipment,
*         RTCA inc, November 28, 2001
*     [2] IS-QZSS v.1.1, Quasi-Zenith Satellite System Navigation Service
*         Interface Specification for QZSS, Japan Aerospace Exploration Agency,
*         July 31, 2009
*
* version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
* history : 2007/10/14 1.0  new
*           2009/01/24 1.1  modify sbspntpos() api
*                           improve fast/ion correction update
*           2009/04/08 1.2  move function crc24q() to rcvlog.c
*                           support glonass, galileo and qzss
*           2009/06/08 1.3  modify sbsupdatestat()
*                           delete sbssatpos()
*           2009/12/12 1.4  support glonass
*           2010/01/22 1.5  support ems (egnos message service) format
*           2010/06/10 1.6  added api:
*                               sbssatcorr(),sbstropcorr(),sbsioncorr(),
*                               sbsupdatecorr()
*                           changed api:
*                               sbsreadmsgt(),sbsreadmsg()
*                           deleted api:
*                               sbspntpos(),sbsupdatestat()
*           2010/08/16 1.7  not reject udre==14 or give==15 correction message
*                           (2.4.0_p4)
*           2011/01/15 1.8  use api ionppp()
*                           add prn mask of qzss for qzss L1SAIF
*           2016/07/29 1.9  crc24q() . rtk_crc24q()
*           2020/11/30 1.10 use integer types in stdint.h
*		    2022/05/31 1.0  rewrite sbas.c with golang by fxb
*-----------------------------------------------------------------------------*/
package gnssgo

import (
	"bufio"
	"fmt"
	"math"
	"os"
	"sort"
	"strings"
)

/* constants -----------------------------------------------------------------*/

const WEEKOFFSET = 1024 /* gps week offset for NovAtel OEM-3 */

/* sbas igp definition -------------------------------------------------------*/

var (
	x1 []int16 = []int16{-75, -65, -55, -50, -45, -40, -35, -30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20,
		25, 30, 35, 40, 45, 50, 55, 65, 75, 85}
	x2 []int16 = []int16{-55, -50, -45, -40, -35, -30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30,
		35, 40, 45, 50, 55}
	x3 []int16 = []int16{-75, -65, -55, -50, -45, -40, -35, -30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20,
		25, 30, 35, 40, 45, 50, 55, 65, 75}
	x4 []int16 = []int16{-85, -75, -65, -55, -50, -45, -40, -35, -30, -25, -20, -15, -10, -5, 0, 5, 10, 15,
		20, 25, 30, 35, 40, 45, 50, 55, 65, 75}
	x5 []int16 = []int16{-180, -175, -170, -165, -160, -155, -150, -145, -140, -135, -130, -125, -120, -115,
		-110, -105, -100, -95, -90, -85, -80, -75, -70, -65, -60, -55, -50, -45,
		-40, -35, -30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25,
		30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95,
		100, 105, 110, 115, 120, 125, 130, 135, 140, 145, 150, 155, 160, 165,
		170, 175}
	x6 []int16 = []int16{-180, -170, -160, -150, -140, -130, -120, -110, -100, -90, -80, -70, -60, -50,
		-40, -30, -20, -10, 0, 10, 20, 30, 40, 50, 60, 70, 80, 90,
		100, 110, 120, 130, 140, 150, 160, 170}
	x7 []int16 = []int16{-180, -150, -120, -90, -60, -30, 0, 30, 60, 90, 120, 150}
	x8 []int16 = []int16{-170, -140, -110, -80, -50, -20, 10, 40, 70, 100, 130, 160}

	igpband1 [9][8]SbsIgpBand = [9][8]SbsIgpBand{ /* band 0-8 */
		{{-180, x1, 1, 28}, {-175, x2, 29, 51}, {-170, x3, 52, 78}, {-165, x2, 79, 101},
			{-160, x3, 102, 128}, {-155, x2, 129, 151}, {-150, x3, 152, 178}, {-145, x2, 179, 201}},
		{{-140, x4, 1, 28}, {-135, x2, 29, 51}, {-130, x3, 52, 78}, {-125, x2, 79, 101},
			{-120, x3, 102, 128}, {-115, x2, 129, 151}, {-110, x3, 152, 178}, {-105, x2, 179, 201}},
		{{-100, x3, 1, 27}, {-95, x2, 28, 50}, {-90, x1, 51, 78}, {-85, x2, 79, 101},
			{-80, x3, 102, 128}, {-75, x2, 129, 151}, {-70, x3, 152, 178}, {-65, x2, 179, 201}},
		{{-60, x3, 1, 27}, {-55, x2, 28, 50}, {-50, x4, 51, 78}, {-45, x2, 79, 101},
			{-40, x3, 102, 128}, {-35, x2, 129, 151}, {-30, x3, 152, 178}, {-25, x2, 179, 201}},
		{{-20, x3, 1, 27}, {-15, x2, 28, 50}, {-10, x3, 51, 77}, {-5, x2, 78, 100},
			{0, x1, 101, 128}, {5, x2, 129, 151}, {10, x3, 152, 178}, {15, x2, 179, 201}},
		{{20, x3, 1, 27}, {25, x2, 28, 50}, {30, x3, 51, 77}, {35, x2, 78, 100},
			{40, x4, 101, 128}, {45, x2, 129, 151}, {50, x3, 152, 178}, {55, x2, 179, 201}},
		{{60, x3, 1, 27}, {65, x2, 28, 50}, {70, x3, 51, 77}, {75, x2, 78, 100},
			{80, x3, 101, 127}, {85, x2, 128, 150}, {90, x1, 151, 178}, {95, x2, 179, 201}},
		{{100, x3, 1, 27}, {105, x2, 28, 50}, {110, x3, 51, 77}, {115, x2, 78, 100},
			{120, x3, 101, 127}, {125, x2, 128, 150}, {130, x4, 151, 178}, {135, x2, 179, 201}},
		{{140, x3, 1, 27}, {145, x2, 28, 50}, {150, x3, 51, 77}, {155, x2, 78, 100},
			{160, x3, 101, 127}, {165, x2, 128, 150}, {170, x3, 151, 177}, {175, x2, 178, 200}}}
	igpband2 [2][5]SbsIgpBand = [2][5]SbsIgpBand{ /* band 9-10 */
		{{60, x5, 1, 72}, {65, x6, 73, 108}, {70, x6, 109, 144}, {75, x6, 145, 180},
			{85, x7, 181, 192}},
		{{-60, x5, 1, 72}, {-65, x6, 73, 108}, {-70, x6, 109, 144}, {-75, x6, 145, 180},
			{-85, x8, 181, 192}}}
)

/* extract field from line ---------------------------------------------------*/
func getfield(p string, pos int) int {
	var index int
	for pos--; pos > 0; pos, index = pos-1, index+1 {
		if index = strings.Index(p[index:], ","); index == 0 {
			return -1
		}
	}
	return index
}

/* variance of fast correction (udre=UDRE+1) ---------------------------------*/
func varfcorr(udre int) float64 {
	var fvar [14]float64 = [14]float64{
		0.052, 0.0924, 0.1444, 0.283, 0.4678, 0.8315, 1.2992, 1.8709, 2.5465, 3.326,
		5.1968, 20.7870, 230.9661, 2078.695}
	if 0 < udre && udre <= 14 {
		return fvar[udre-1]
	}
	return 0.0
}

/* variance of ionosphere correction (give=GIVEI+1) --------------------------*/
func varicorr(give int) float64 {
	var fvar [15]float64 = [15]float64{
		0.0084, 0.0333, 0.0749, 0.1331, 0.2079, 0.2994, 0.4075, 0.5322, 0.6735, 0.8315,
		1.1974, 1.8709, 3.326, 20.787, 187.0826}
	if 0 < give && give <= 15 {
		return fvar[give-1]
	}
	return 0.0
}

/* fast correction degradation -----------------------------------------------*/
func degfcorr(ai int) float64 {
	var degf [16]float64 = [16]float64{
		0.00000, 0.00005, 0.00009, 0.00012, 0.00015, 0.00020, 0.00030, 0.00045,
		0.00060, 0.00090, 0.00150, 0.00210, 0.00270, 0.00330, 0.00460, 0.00580}
	if 0 < ai && ai <= 15 {
		return degf[ai]
	}
	return 0.0058
}

/* decode type 1: prn masks --------------------------------------------------*/
func decode_sbstype1(msg *SbsMsg, sbssat *SbsSat) int {
	var i, n, sat int

	Trace(4, "decode_sbstype1:\n")

	for i, n = 1, 0; i <= 210 && n < MAXSAT; i++ {
		if GetBitU(msg.Msg[:], 13+i, 1) > 0 {
			switch {
			case i <= 37:
				sat = SatNo(SYS_GPS, i) /*   0- 37: gps */
			case i <= 61:
				sat = SatNo(SYS_GLO, i-37) /*  38- 61: glonass */
			case i <= 119:
				sat = 0 /*  62-119: future gnss */
			case i <= 138:
				sat = SatNo(SYS_SBS, i) /* 120-138: geo/waas */
			case i <= 182:
				sat = 0 /* 139-182: reserved */
			case i <= 192:
				sat = SatNo(SYS_SBS, i+10) /* 183-192: qzss ref [2] */
			case i <= 202:
				sat = SatNo(SYS_QZS, i) /* 193-202: qzss ref [2] */
			default:
				sat = 0 /* 203-   : reserved */
			}
			sbssat.sat[n].sat = sat
			n++
		}
	}
	sbssat.iodp = int(GetBitU(msg.Msg[:], 224, 2))
	sbssat.nsat = n

	Trace(5, "decode_sbstype1: nprn=%d iodp=%d\n", n, sbssat.iodp)
	return 1
}

/* decode type 2-5,0: fast corrections ---------------------------------------*/
func decode_sbstype2(msg *SbsMsg, sbssat *SbsSat) int {
	var (
		i, j, iodf, ctype, udre int
		prc, dt                 float64
		t0                      Gtime
	)

	Trace(4, "decode_sbstype2:\n")

	if sbssat.iodp != int(GetBitU(msg.Msg[:], 16, 2)) {
		return 0
	}

	ctype = int(GetBitU(msg.Msg[:], 8, 6))
	iodf = int(GetBitU(msg.Msg[:], 14, 2))

	for i = 0; i < 13; i++ {
		if ctype == 0 {
			j = i
		} else {
			j = 13*(ctype-2) + i
		}
		if j >= sbssat.nsat {
			break
		}
		udre = int(GetBitU(msg.Msg[:], 174+4*i, 4))
		t0 = sbssat.sat[j].fcorr.t0
		prc = sbssat.sat[j].fcorr.prc
		sbssat.sat[j].fcorr.t0 = GpsT2Time(msg.Week, float64(msg.Tow))
		sbssat.sat[j].fcorr.prc = float64(GetBits(msg.Msg[:], 18+i*12, 12)) * 0.125
		sbssat.sat[j].fcorr.udre = int16(udre) + 1
		dt = TimeDiff(sbssat.sat[j].fcorr.t0, t0)
		if t0.Time == 0 || dt <= 0.0 || 18.0 < dt || sbssat.sat[j].fcorr.ai == 0 {
			sbssat.sat[j].fcorr.rrc = 0.0
			sbssat.sat[j].fcorr.dt = 0.0
		} else {
			sbssat.sat[j].fcorr.rrc = (sbssat.sat[j].fcorr.prc - prc) / dt
			sbssat.sat[j].fcorr.dt = dt
		}
		sbssat.sat[j].fcorr.iodf = iodf
	}
	Trace(5, "decode_sbstype2: type=%d iodf=%d\n", ctype, iodf)
	return 1
}

/* decode type 6: integrity info ---------------------------------------------*/
func decode_sbstype6(msg *SbsMsg, sbssat *SbsSat) int {
	var (
		i, udre int
		iodf    [4]int
	)
	Trace(4, "decode_sbstype6:\n")

	for i = 0; i < 4; i++ {
		iodf[i] = int(GetBitU(msg.Msg[:], 14+i*2, 2))
	}
	for i = 0; i < sbssat.nsat && i < MAXSAT; i++ {
		if sbssat.sat[i].fcorr.iodf != iodf[int(i/13)] {
			continue
		}
		udre = int(GetBitU(msg.Msg[:], 22+i*4, 4))
		sbssat.sat[i].fcorr.udre = int16(udre) + 1
	}
	Trace(5, "decode_sbstype6: iodf=%d %d %d %d\n", iodf[0], iodf[1], iodf[2], iodf[3])
	return 1
}

/* decode type 7: fast correction degradation factor -------------------------*/
func decode_sbstype7(msg *SbsMsg, sbssat *SbsSat) int {
	Trace(4, "decode_sbstype7\n")

	if sbssat.iodp != int(GetBitU(msg.Msg[:], 18, 2)) {
		return 0
	}

	sbssat.tlat = int(GetBitU(msg.Msg[:], 14, 4))

	for i := 0; i < sbssat.nsat && i < MAXSAT; i++ {
		sbssat.sat[i].fcorr.ai = int16(GetBitU(msg.Msg[:], 22+i*4, 4))
	}
	return 1
}

/* decode type 9: geo navigation message -------------------------------------*/
func decode_sbstype9(msg *SbsMsg, nav *Nav) int {
	var (
		seph      SEph
		i, sat, t int
	)

	Trace(4, "decode_sbstype9:\n")

	if sat = SatNo(SYS_SBS, int(msg.Prn)); sat == 0 {
		Trace(2, "invalid prn in sbas type 9: prn=%3d\n", msg.Prn)
		return 0
	}
	t = int(GetBitU(msg.Msg[:], 22, 13))*16 - int(msg.Tow%86400)
	if t <= -43200 {
		t += 86400
	} else if t > 43200 {
		t -= 86400
	}
	seph.Sat = sat
	seph.T0 = GpsT2Time(msg.Week, float64(msg.Tow+t))
	seph.Tof = GpsT2Time(msg.Week, float64(msg.Tow))
	seph.Sva = int(GetBitU(msg.Msg[:], 35, 4))
	seph.Svh = 0 /* unhealthy if ura==15 */
	if seph.Sva == 15 {
		seph.Svh = 1
	}
	seph.Pos[0] = float64(GetBits(msg.Msg[:], 39, 30)) * 0.08
	seph.Pos[1] = float64(GetBits(msg.Msg[:], 69, 30)) * 0.08
	seph.Pos[2] = float64(GetBits(msg.Msg[:], 99, 25)) * 0.4
	seph.Vel[0] = float64(GetBits(msg.Msg[:], 124, 17)) * 0.000625
	seph.Vel[1] = float64(GetBits(msg.Msg[:], 141, 17)) * 0.000625
	seph.Vel[2] = float64(GetBits(msg.Msg[:], 158, 18)) * 0.004
	seph.Acc[0] = float64(GetBits(msg.Msg[:], 176, 10)) * 0.0000125
	seph.Acc[1] = float64(GetBits(msg.Msg[:], 186, 10)) * 0.0000125
	seph.Acc[2] = float64(GetBits(msg.Msg[:], 196, 10)) * 0.0000625

	seph.Af0 = float64(GetBits(msg.Msg[:], 206, 12)) * P2_31
	seph.Af1 = float64(GetBits(msg.Msg[:], 218, 8)) * P2_39 / 2.0

	i = int(msg.Prn) - MINPRNSBS
	if nav.Seph == nil || math.Abs(TimeDiff(nav.Seph[i].T0, seph.T0)) < 1e-3 { /* not change */
		return 0
	}
	nav.Seph[NSATSBS+i] = nav.Seph[i] /* previous */
	nav.Seph[i] = seph                /* current */

	Trace(5, "decode_sbstype9: prn=%d\n", msg.Prn)
	return 1
}

/* decode type 18: ionospheric grid point masks ------------------------------*/
func decode_sbstype18(msg *SbsMsg, sbsion []SbsIon) int {
	var (
		p          []SbsIgpBand
		i, j, n, m int
	)
	band := GetBitU(msg.Msg[:], 18, 4)

	Trace(4, "decode_sbstype18:\n")

	if band <= 8 {
		p = igpband1[band][:]
		m = 8
	} else if 9 <= band && band <= 10 {
		p = igpband2[band-9][:]
		m = 5
	} else {
		return 0
	}

	sbsion[band].iodi = int(GetBitU(msg.Msg[:], 22, 2))

	for i, n = 1, 0; i <= 201; i++ {
		if GetBitU(msg.Msg[:], 23+i, 1) == 0 {
			continue
		}
		for j = 0; j < m; j++ {
			if i < int(p[j].bits) || int(p[j].bite) < i {
				continue
			}
			sbsion[band].igp[n].lat = p[j].x
			sbsion[band].igp[n].lon = p[j].y[i-int(p[j].bits)]
			if band <= 8 {
				sbsion[band].igp[n].lat = p[j].y[i-int(p[j].bits)]
				sbsion[band].igp[n].lon = p[j].x
			}
			n++
			break
		}
	}
	sbsion[band].nigp = n

	Trace(5, "decode_sbstype18: band=%d nigp=%d\n", band, n)
	return 1
}

/* decode half long term correction (vel code=0) -----------------------------*/
func decode_longcorr0(msg *SbsMsg, p int, sbssat *SbsSat) int {
	var i int
	n := GetBitU(msg.Msg[:], p, 6)

	Trace(4, "decode_longcorr0:\n")

	if n == 0 || n > uint32(MAXSAT) {
		return 0
	}

	sbssat.sat[n-1].lcorr.iode = int(GetBitU(msg.Msg[:], p+6, 8))

	for i = 0; i < 3; i++ {
		sbssat.sat[n-1].lcorr.dpos[i] = float64(GetBits(msg.Msg[:], p+14+9*i, 9)) * 0.125
		sbssat.sat[n-1].lcorr.dvel[i] = 0.0
	}
	sbssat.sat[n-1].lcorr.daf0 = float64(GetBits(msg.Msg[:], p+41, 10)) * P2_31
	sbssat.sat[n-1].lcorr.daf1 = 0.0
	sbssat.sat[n-1].lcorr.t0 = GpsT2Time(msg.Week, float64(msg.Tow))

	Trace(5, "decode_longcorr0:sat=%2d\n", sbssat.sat[n-1].sat)
	return 1
}

/* decode half long term correction (vel code=1) -----------------------------*/
func decode_longcorr1(msg *SbsMsg, p int, sbssat *SbsSat) int {
	var i, t int
	n := GetBitU(msg.Msg[:], p, 6)

	Trace(4, "decode_longcorr1:\n")

	if n == 0 || n > uint32(MAXSAT) {
		return 0
	}

	sbssat.sat[n-1].lcorr.iode = int(GetBitU(msg.Msg[:], p+6, 8))

	for i = 0; i < 3; i++ {
		sbssat.sat[n-1].lcorr.dpos[i] = float64(GetBits(msg.Msg[:], p+14+i*11, 11)) * 0.125
		sbssat.sat[n-1].lcorr.dvel[i] = float64(GetBits(msg.Msg[:], p+58+i*8, 8)) * P2_11
	}
	sbssat.sat[n-1].lcorr.daf0 = float64(GetBits(msg.Msg[:], p+47, 11)) * P2_31
	sbssat.sat[n-1].lcorr.daf1 = float64(GetBits(msg.Msg[:], p+82, 8)) * P2_39
	t = int(GetBitU(msg.Msg[:], p+90, 13))*16 - int(msg.Tow%86400)
	if t <= -43200 {
		t += 86400
	} else if t > 43200 {
		t -= 86400
	}
	sbssat.sat[n-1].lcorr.t0 = GpsT2Time(msg.Week, float64(msg.Tow+t))

	Trace(5, "decode_longcorr1: sat=%2d\n", sbssat.sat[n-1].sat)
	return 1
}

/* decode half long term correction ------------------------------------------*/
func decode_longcorrh(msg *SbsMsg, p int, sbssat *SbsSat) int {
	Trace(4, "decode_longcorrh:\n")

	if GetBitU(msg.Msg[:], p, 1) == 0 { /* vel code=0 */
		if sbssat.iodp == int(GetBitU(msg.Msg[:], p+103, 2)) {
			if decode_longcorr0(msg, p+1, sbssat) > 0 &&
				decode_longcorr0(msg, p+52, sbssat) > 0 {
				return 1
			} else {
				return 0
			}

		}
	} else if sbssat.iodp == int(GetBitU(msg.Msg[:], p+104, 2)) {
		return decode_longcorr1(msg, p+1, sbssat)
	}
	return 0
}

/* decode type 24: mixed fast/long term correction ---------------------------*/
func decode_sbstype24(msg *SbsMsg, sbssat *SbsSat) int {
	var i, j, iodf, blk, udre int

	Trace(4, "decode_sbstype24:\n")

	if sbssat.iodp != int(GetBitU(msg.Msg[:], 110, 2)) {
		return 0 /* check IODP */
	}

	blk = int(GetBitU(msg.Msg[:], 112, 2))
	iodf = int(GetBitU(msg.Msg[:], 114, 2))

	for i = 0; i < 6; i++ {
		if j = 13*blk + i; j >= sbssat.nsat {
			break
		}
		udre = int(GetBitU(msg.Msg[:], 86+4*i, 4))

		sbssat.sat[j].fcorr.t0 = GpsT2Time(msg.Week, float64(msg.Tow))
		sbssat.sat[j].fcorr.prc = float64(GetBits(msg.Msg[:], 14+i*12, 12)) * 0.125
		sbssat.sat[j].fcorr.udre = int16(udre) + 1
		sbssat.sat[j].fcorr.iodf = iodf
	}
	return decode_longcorrh(msg, 120, sbssat)
}

/* decode type 25: long term satellite error correction ----------------------*/
func decode_sbstype25(msg *SbsMsg, sbssat *SbsSat) int {
	Trace(4, "decode_sbstype25:\n")

	if decode_longcorrh(msg, 14, sbssat) > 0 && decode_longcorrh(msg, 120, sbssat) > 0 {
		return 1
	}
	return 0
}

/* decode type 26: ionospheric deley corrections -----------------------------*/
func decode_sbstype26(msg *SbsMsg, sbsion []SbsIon) int {
	var i, j, block, delay, give int
	band := GetBitU(msg.Msg[:], 14, 4)

	Trace(4, "decode_sbstype26:\n")

	if band > uint32(MAXBAND) || sbsion[band].iodi != int(GetBitU(msg.Msg[:], 217, 2)) {
		return 0
	}

	block = int(GetBitU(msg.Msg[:], 18, 4))

	for i = 0; i < 15; i++ {
		if j = block*15 + i; j >= sbsion[band].nigp {
			continue
		}
		give = int(GetBitU(msg.Msg[:], 22+i*13+9, 4))

		delay = int(GetBitU(msg.Msg[:], 22+i*13, 9))
		sbsion[band].igp[j].t0 = GpsT2Time(msg.Week, float64(msg.Tow))
		sbsion[band].igp[j].delay = float32(delay) * 0.125
		if delay == 0x1FF {
			sbsion[band].igp[j].delay = 0.0
		}
		sbsion[band].igp[j].give = int16(give) + 1

		if sbsion[band].igp[j].give >= 16 {
			sbsion[band].igp[j].give = 0
		}
	}
	Trace(5, "decode_sbstype26: band=%d block=%d\n", band, block)
	return 1
}

/* update sbas corrections -----------------------------------------------------
* update sbas correction parameters in navigation data with a sbas message
* args   : sbsmg_t  *msg    I   sbas message
*          nav_t    *nav    IO  navigation data
* return : message type (-1: error or not supported type)
* notes  : nav.seph must point to seph[NSATSBS*2] (array of seph_t)
*               seph[prn-MINPRNSBS+1]          : sat prn current epehmeris
*               seph[prn-MINPRNSBS+1+MAXPRNSBS]: sat prn previous epehmeris
*-----------------------------------------------------------------------------*/
func SbsUpdateCorr(msg *SbsMsg, nav *Nav) int {
	ctype := GetBitU(msg.Msg[:], 8, 6)
	stat := -1

	Trace(3, "sbsupdatecorr: type=%d\n", ctype)

	if msg.Week == 0 {
		return -1
	}

	switch ctype {
	case 0:
		stat = decode_sbstype2(msg, &nav.SbasSat)

	case 1:
		stat = decode_sbstype1(msg, &nav.SbasSat)

	case 2, 3, 4, 5:
		stat = decode_sbstype2(msg, &nav.SbasSat)

	case 6:
		stat = decode_sbstype6(msg, &nav.SbasSat)

	case 7:
		stat = decode_sbstype7(msg, &nav.SbasSat)

	case 9:
		stat = decode_sbstype9(msg, nav)

	case 18:
		stat = decode_sbstype18(msg, nav.SbasIon[:])

	case 24:
		stat = decode_sbstype24(msg, &nav.SbasSat)

	case 25:
		stat = decode_sbstype25(msg, &nav.SbasSat)

	case 26:
		stat = decode_sbstype26(msg, nav.SbasIon[:])

	case 63:
		/* null message */

		/*default: trace(2,"unsupported sbas message: type=%d\n",type); break;*/
	}
	if stat > 0 {
		return int(ctype)
	}
	return -1
}

/* read sbas log file --------------------------------------------------------*/
func (sbs *Sbs) ReadMsgs(file string, sel int, ts, te Gtime) {
	var (
		i, week, prn, ch, msg int
		b                     uint32
		tow                   float64
		ep                    []float64 = make([]float64, 6)
		buff                  string
		time                  Gtime
		fp                    *os.File
		err                   error
	)

	Trace(3, "readmsgs: file=%s sel=%d\n", file, sel)

	fp, err = os.OpenFile(file, os.O_RDONLY, 0666)
	if err != nil {
		Trace(2, "sbas message file open error: %s\n", file)
		return
	}
	defer fp.Close()

	rd := bufio.NewReader(fp)
	for {
		buff, err = rd.ReadString('\n')
		if err != nil {
			break
		}
		n, _ := fmt.Sscanf(buff, "%d %f %d", &week, &tow, &prn)
		idx := strings.Index(buff, ": ")

		if n == 3 && idx >= 0 {
			buff = buff[idx+2:] /* rtklib form */
		} else if n, _ = fmt.Sscanf(buff, "%d %f %f %f %f %f %f %d",
			&prn, &ep[0], &ep[1], &ep[2], &ep[3], &ep[4], &ep[5], &msg); n == 8 {
			/* ems (EGNOS Message Service) form */
			if ep[0] < 70.0 {
				ep[0] += 2000.0
			} else {
				ep[0] += 1900.0
				tow = Time2GpsT(Epoch2Time(ep), &week)
				if msg >= 10 {
					buff = buff[25:]
				} else {
					buff = buff[24:]
				}
			}
		} else if strings.Compare(buff[:14], "#RAWWAASFRAMEA") == 0 { /* NovAtel OEM4/V */
			if idx = getfield(buff, 6); idx == -1 {
				continue
			} else {
				buff = buff[idx:]
			}

			if n, _ = fmt.Sscanf(buff, "%d,%f", &week, &tow); n < 2 {
				continue
			}
			if idx = strings.Index(buff[1:], ";"); idx < 0 {
				continue
			}
			//if (!(p=strchr(++p,';'))) continue;
			buff = buff[idx+1:]
			if n, _ = fmt.Sscanf(buff, "%d,%d", &ch, &prn); n < 2 {
				continue
			}

			if idx = getfield(buff, 4); idx == -1 {
				continue
			} else {
				buff = buff[idx:]
			}
		} else if strings.Compare(buff[:5], "$FRMA") == 0 { /* NovAtel OEM4/V */
			if idx = getfield(buff, 2); idx == -1 {
				continue
			} else {
				buff = buff[idx:]
			}

			if n, _ = fmt.Sscanf(buff, "%d,%f,%d", &week, &tow, &prn); n < 3 {
				continue
			}

			if idx = getfield(buff, 6); idx == -1 {
				continue
			} else {
				buff = buff[idx:]
			}
			if week < WEEKOFFSET {
				week += WEEKOFFSET
			}
		} else {
			continue
		}
		// }
		// while (fgets(buff,sizeof(buff),fp)) {
		//     if (sscanf(buff,"%d %f %d",&week,&tow,&prn)==3&&(p=strstr(buff,": "))) {
		//         p+=2; /* rtklib form */
		//     }
		//     else if (sscanf(buff,"%d %f %f %f %f %f %f %d",
		//                     &prn,ep,ep+1,ep+2,ep+3,ep+4,ep+5,&msg)==8) {
		//         /* ems (EGNOS Message Service) form */
		//         ep[0]+=ep[0]<70.0?2000.0:1900.0;
		//         tow=time2gpst(epoch2time(ep),&week);
		//         p=buff+(msg>=10?25:24);
		//     }
		//     else if (!strncmp(buff,"#RAWWAASFRAMEA",14)) { /* NovAtel OEM4/V */
		//         if (!(p=getfield(buff,6))) continue;
		//         if (sscanf(p,"%d,%f",&week,&tow)<2) continue;
		//         if (!(p=strchr(++p,';'))) continue;
		//         if (sscanf(++p,"%d,%d",&ch,&prn)<2) continue;
		//         if (!(p=getfield(p,4))) continue;
		//     }
		//     else if (!strncmp(buff,"$FRMA",5)) { /* NovAtel OEM3 */
		//         if (!(p=getfield(buff,2))) continue;
		//         if (sscanf(p,"%d,%f,%d",&week,&tow,&prn)<3) continue;
		//         if (!(p=getfield(p,6))) continue;
		//         if (week<WEEKOFFSET) week+=WEEKOFFSET;
		//     }
		//     else continue;

		if sel != 0 && sel != prn {
			continue
		}

		time = GpsT2Time(week, tow)

		if ScreenTime(time, ts, te, 0.0) == 0 {
			continue
		}

		var data SbsMsg

		data.Week = week
		data.Tow = int(tow + 0.5)
		data.Prn = uint8(prn)
		for i = 0; i < 29; i++ {
			data.Msg[i] = 0
		}
		idx = 0
		for i = 0; len(buff[idx:]) > 0 && i < 29; idx, i = idx+2, i+1 {
			if n, _ = fmt.Sscanf(buff[idx:], "%2X", &b); n == 1 {
				data.Msg[i] = uint8(b)
			}
		}
		data.Msg[28] &= 0xC0
		sbs.Msgs = append(sbs.Msgs, data)
	}
}

/* compare sbas messages -----------------------------------------------------*/
func cmpmsgs(q1, q2 *SbsMsg) int {
	switch {
	case q1.Week != q2.Week:
		return q1.Week - q2.Week
	case q1.Tow < q2.Tow:
		return -1
	case q1.Tow > q2.Tow:
		return 1
	}

	switch {
	case q1.Prn > q2.Prn:
		return 1
	case q1.Prn == q2.Prn:
		return 0
	default:
		return -1
	}
}

/* read sbas message file ------------------------------------------------------
* read sbas message file
* args   : char     *file   I   sbas message file (wind-card * is expanded)
*          int      sel     I   sbas satellite prn number selection (0:all)
*         (gtime_t  ts      I   start time)
*         (gtime_t  te      I   end time  )
*          sbs_t    *sbs    IO  sbas messages
* return : number of sbas messages
* notes  : sbas message are appended and sorted. before calling the funciton,
*          sbs.n, sbs.nmax and sbs.msgs must be set properly. (initially
*          sbs.n=sbs.nmax=0, sbs.msgs=NULL)
*          only the following file extentions after wild card expanded are valid
*          to read. others are skipped
*          .sbs, .SBS, .ems, .EMS
*-----------------------------------------------------------------------------*/
func (sbs *Sbs) SbsReadMsgt(file string, sel int, ts, te Gtime) int {
	var (
		efiles    []string = make([]string, MAXEXFILE)
		i, n, idx int
	)

	Trace(3, "sbsreadmsgt: file=%s sel=%d\n", file, sel)

	/* expand wild card in file path */
	n = ExPath(file, efiles, MAXEXFILE)

	for i = 0; i < n; i++ {
		if idx = strings.LastIndex(efiles[i], "."); idx < 0 {
			continue
		}
		// if (!(ext=strrchr(efiles[i],'.'))) continue;
		if !strings.EqualFold(efiles[i][idx:], ".sbs") && !strings.EqualFold(efiles[i][idx:], ".ems") {
			continue
		}
		// if (strcmp(ext,".sbs")&&strcmp(ext,".SBS")&&
		//     strcmp(ext,".ems")&&strcmp(ext,".EMS")) continue;

		sbs.ReadMsgs(efiles[i], sel, ts, te)
	}

	/* sort messages */
	if sbs.N() > 0 {
		sort.Slice(sbs.Msgs, func(i, j int) bool {
			return cmpmsgs(&sbs.Msgs[i], &sbs.Msgs[j]) < 0
		})
	}
	return sbs.N()
}
func sbsreadmsg(file string, sel int, sbs *Sbs) int {
	var ts, te Gtime

	Trace(3, "sbsreadmsg: file=%s sel=%d\n", file, sel)

	return sbs.SbsReadMsgt(file, sel, ts, te)
}

/* output sbas messages --------------------------------------------------------
* output sbas message record to output file in rtklib sbas log format
* args   : FILE   *fp       I   output file pointer
*          sbsmsg_t *sbsmsg I   sbas messages
* return : none
*-----------------------------------------------------------------------------*/
func SbsOutMsg(fp *os.File, sbsmsg *SbsMsg) {
	var i int
	ctype := sbsmsg.Msg[1] >> 2

	Trace(4, "sbsoutmsg:\n")

	fp.WriteString(fmt.Sprintf("%4d %6d %3d %2d : ", sbsmsg.Week, sbsmsg.Tow, sbsmsg.Prn, ctype))
	for i = 0; i < 29; i++ {
		fp.WriteString(fmt.Sprintf("%02X", sbsmsg.Msg[i]))
	}
	fp.WriteString("\n")
}

/* search igps ---------------------------------------------------------------*/
func SearchIgp(time Gtime, pos []float64, ion []SbsIon, igp []RefSbsIgp, x, y *float64) {
	var (
		i, j int
		latp [2]int
		lonp [4]int
	)
	lat := pos[0] * R2D
	lon := pos[1] * R2D
	//const sbsigp_t *p;

	Trace(4, "searchigp: pos=%.3f %.3f\n", pos[0]*R2D, pos[1]*R2D)

	if lon >= 180.0 {
		lon -= 360.0
	}
	if -55.0 <= lat && lat < 55.0 {
		latp[0] = int(math.Floor(lat/5.0) * 5)
		latp[1] = latp[0] + 5
		lonp[1] = int(math.Floor(lon/5.0) * 5)
		lonp[0] = lonp[1]
		lonp[2], lonp[3] = lonp[0]+5, lonp[0]+5
		*x = (lon - float64(lonp[0])) / 5.0
		*y = (lat - float64(latp[0])) / 5.0
	} else {
		latp[0] = int(math.Floor((lat-5.0)/10.0)*10 + 5)
		latp[1] = latp[0] + 10
		lonp[1] = int(math.Floor(lon/10.0) * 10)
		lonp[0] = lonp[1]
		lonp[2], lonp[3] = lonp[0]+10, lonp[0]+10
		*x = (lon - float64(lonp[0])) / 10.0
		*y = (lat - float64(latp[0])) / 10.0
		switch {
		case 75.0 <= lat && lat < 85.0:
			lonp[1] = int(math.Floor(lon/90.0) * 90)
			lonp[3] = lonp[1] + 90
		case -85.0 <= lat && lat < -75.0:
			lonp[0] = int(math.Floor((lon-50.0)/90.0)*90 + 40)
			lonp[2] = lonp[0] + 90
		case lat >= 85.0:
			for i = 0; i < 4; i++ {
				lonp[i] = int(math.Floor(lon/90.0) * 90)
			}
		case lat < -85.0:
			for i = 0; i < 4; i++ {
				lonp[i] = int(math.Floor((lon-50.0)/90.0)*90 + 40)
			}
		}
	}
	for i = 0; i < 4; i++ {
		if lonp[i] == 180 {
			lonp[i] = -180
		}
	}
	for i = 0; i <= MAXBAND; i++ {
		for j = 0; j < ion[i].nigp; j++ {
			if ion[i].igp[j].t0.Time == 0 {
				continue
			}
			switch {
			case ion[i].igp[j].lat == int16(latp[0]) && ion[i].igp[j].lon == int16(lonp[0]) && ion[i].igp[j].give > 0:
				*igp[0] = ion[i].igp[j]
			case ion[i].igp[j].lat == int16(latp[1]) && ion[i].igp[j].lon == int16(lonp[1]) && ion[i].igp[j].give > 0:
				*igp[1] = ion[i].igp[j]
			case ion[i].igp[j].lat == int16(latp[0]) && ion[i].igp[j].lon == int16(lonp[2]) && ion[i].igp[j].give > 0:
				*igp[2] = ion[i].igp[j]
			case ion[i].igp[j].lat == int16(latp[1]) && ion[i].igp[j].lon == int16(lonp[3]) && ion[i].igp[j].give > 0:
				*igp[3] = ion[i].igp[j]
			}
			if igp[0] != nil && igp[1] != nil && igp[2] != nil && igp[3] != nil {
				return
			}

		}
		// for  p:=ion[i].igp;p<ion[i].igp+ion[i].nigp;p++  {
		//     if (p.t0.time==0) continue;
		//     if      (p.lat==latp[0]&&p.lon==lonp[0]&&p.give>0) igp[0]=p;
		//     else if (p.lat==latp[1]&&p.lon==lonp[1]&&p.give>0) igp[1]=p;
		//     else if (p.lat==latp[0]&&p.lon==lonp[2]&&p.give>0) igp[2]=p;
		//     else if (p.lat==latp[1]&&p.lon==lonp[3]&&p.give>0) igp[3]=p;
		//     if (igp[0]&&igp[1]&&igp[2]&&igp[3]) return;
		// }
	}
}

/* sbas ionospheric delay correction -------------------------------------------
* compute sbas ionosphric delay correction
* args   : gtime_t  time    I   time
*          nav_t    *nav    I   navigation data
*          double   *pos    I   receiver position {lat,lon,height} (rad/m)
*          double   *azel   I   satellite azimuth/elavation angle (rad)
*          double   *delay  O   slant ionospheric delay (L1) (m)
*          double   *var    O   variance of ionospheric delay (m^2)
* return : status (1:ok, 0:no correction)
* notes  : before calling the function, sbas ionosphere correction parameters
*          in navigation data (nav.sbsion) must be set by callig
*          sbsupdatecorr()
*-----------------------------------------------------------------------------*/
func SbsIonCorr(time Gtime, nav *Nav, pos, azel []float64, delay, fvar *float64) int {
	var (
		i, err      int
		fp, x, y, t float64
		posp        [2]float64
		w           [4]float64
		igp         [4]RefSbsIgp /* {ws,wn,es,en} */)
	re := 6378.1363
	hion := 350.0

	Trace(4, "sbsioncorr: pos=%.3f %.3f azel=%.3f %.3f\n", pos[0]*R2D, pos[1]*R2D, azel[0]*R2D, azel[1]*R2D)

	*delay, *fvar = 0.0, 0.0
	if pos[2] < -100.0 || azel[1] <= 0 {
		return 1
	}

	/* ipp (ionospheric pierce point) position */
	fp = IonPPP(pos, azel, re, hion, posp[:])

	/* search igps around ipp */
	SearchIgp(time, posp[:], nav.SbasIon[:], igp[:], &x, &y)

	/* weight of igps */
	switch {
	case igp[0] != nil && igp[1] != nil && igp[2] != nil && igp[3] != nil:
		w[0] = (1.0 - x) * (1.0 - y)
		w[1] = (1.0 - x) * y
		w[2] = x * (1.0 - y)
		w[3] = x * y
	case igp[0] != nil && igp[1] != nil && igp[2] != nil:
		w[1] = y
		w[2] = x
		if w[0] = 1.0 - w[1] - w[2]; w[0] < 0.0 {
			err = 1
		}
	case igp[0] != nil && igp[2] != nil && igp[3] != nil:
		w[0] = 1.0 - x
		w[3] = y
		if w[2] = 1.0 - w[0] - w[3]; w[2] < 0.0 {
			err = 1
		}
	case igp[0] != nil && igp[1] != nil && igp[3] != nil:
		w[0] = 1.0 - y
		w[3] = x
		if w[1] = 1.0 - w[0] - w[3]; w[1] < 0.0 {
			err = 1
		}
	case igp[1] != nil && igp[2] != nil && igp[3] != nil:
		w[1] = 1.0 - x
		w[2] = 1.0 - y
		if w[3] = 1.0 - w[1] - w[2]; w[3] < 0.0 {
			err = 1
		}
	default:
		err = 1
	}

	if err > 0 {
		Trace(2, "no sbas iono correction: lat=%3.0f lon=%4.0f\n", posp[0]*R2D,
			posp[1]*R2D)
		return 0
	}
	for i = 0; i < 4; i++ {
		if igp[i] == nil {
			continue
		}
		t = TimeDiff(time, igp[i].t0)
		*delay += w[i] * float64(igp[i].delay)
		*fvar += w[i] * varicorr(int(igp[i].give)) * 9e-8 * math.Abs(t)
	}
	*delay *= fp
	*fvar *= fp * fp

	Trace(5, "sbsioncorr: dion=%7.2f sig=%7.2f\n", *delay, math.Sqrt(*fvar))
	return 1
}

/* get meterological parameters ----------------------------------------------*/
func getmet(lat float64, met []float64) {
	var (
		metprm [][10]float64 = [][10]float64{ /* lat=15,30,45,60,75 */
			{1013.25, 299.65, 26.31, 6.30e-3, 2.77, 0.00, 0.00, 0.00, 0.00e-3, 0.00},
			{1017.25, 294.15, 21.79, 6.05e-3, 3.15, -3.75, 7.00, 8.85, 0.25e-3, 0.33},
			{1015.75, 283.15, 11.66, 5.58e-3, 2.57, -2.25, 11.00, 7.24, 0.32e-3, 0.46},
			{1011.75, 272.15, 6.78, 5.39e-3, 1.81, -1.75, 15.00, 5.36, 0.81e-3, 0.74},
			{1013.00, 263.65, 4.11, 4.53e-3, 1.55, -0.50, 14.50, 3.39, 0.62e-3, 0.30}}
		i, j int
		a    float64
	)
	lat = math.Abs(lat)
	switch {
	case lat <= 15.0:
		for i = 0; i < 10; i++ {
			met[i] = metprm[0][i]
		}
	case lat >= 75.0:
		for i = 0; i < 10; i++ {
			met[i] = metprm[4][i]
		}
	default:
		j = int(lat / 15.0)
		a = (lat - float64(j)*15.0) / 15.0
		for i = 0; i < 10; i++ {
			met[i] = (1.0-a)*metprm[j-1][i] + a*metprm[j][i]
		}
	}
}

/* tropospheric delay correction -----------------------------------------------
* compute sbas tropospheric delay correction (mops model)
* args   : gtime_t time     I   time
*          double   *pos    I   receiver position {lat,lon,height} (rad/m)
*          double   *azel   I   satellite azimuth/elavation (rad)
*          double   *var    O   variance of troposphric error (m^2)
* return : slant tropospheric delay (m)
*-----------------------------------------------------------------------------*/
func SbsTropCorr(time Gtime, pos, azel []float64, fvar *float64) float64 {
	var (
		pos_         [3]float64
		c, zh, zw, m float64
		i            int
		met          [10]float64
	)
	k1 := 77.604
	k2 := 382000.0
	rd := 287.054
	gm := 9.784
	g := 9.80665
	sinel := math.Sin(azel[1])
	h := pos[2]

	Trace(4, "sbstropcorr: pos=%.3f %.3f azel=%.3f %.3f\n", pos[0]*R2D, pos[1]*R2D,
		azel[0]*R2D, azel[1]*R2D)

	if pos[2] < -100.0 || 10000.0 < pos[2] || azel[1] <= 0 {
		*fvar = 0.0
		return 0.0
	}
	if zh == 0.0 || math.Abs(pos[0]-pos_[0]) > 1e-7 || math.Abs(pos[1]-pos_[1]) > 1e-7 ||
		math.Abs(pos[2]-pos_[2]) > 1.0 {
		getmet(pos[0]*R2D, met[:])
		post := 211.0
		if pos[0] >= 0.0 {
			post = 28.0
		}
		c = math.Cos(2.0 * PI * (Time2DayOfYeay(time) - post) / 365.25)
		for i = 0; i < 5; i++ {
			met[i] -= met[i+5] * c
		}
		zh = 1e-6 * k1 * rd * met[0] / gm
		zw = 1e-6 * k2 * rd / (gm*(met[4]+1.0) - met[3]*rd) * met[2] / met[1]
		zh *= math.Pow(1.0-met[3]*h/met[1], g/(rd*met[3]))
		zw *= math.Pow(1.0-met[3]*h/met[1], (met[4]+1.0)*g/(rd*met[3])-1.0)
		for i = 0; i < 3; i++ {
			pos_[i] = pos[i]
		}
	}
	m = 1.001 / math.Sqrt(0.002001+sinel*sinel)
	*fvar = 0.12 * 0.12 * m * m
	return (zh + zw) * m
}

/* long term correction ------------------------------------------------------*/
func SbsLongCorr(time Gtime, sat int, sbssat *SbsSat, drs []float64, ddts *float64) int {
	var (
		t    float64
		i, j int
	)

	Trace(3, "sbslongcorr: sat=%2d\n", sat)

	for j = 0; j < sbssat.nsat; j++ {
		if sbssat.sat[j].sat != sat || sbssat.sat[j].lcorr.t0.Time == 0 {
			continue
		}
		t = TimeDiff(time, sbssat.sat[j].lcorr.t0)
		if math.Abs(t) > float64(MAXSBSAGEL) {
			Trace(2, "sbas long-term correction expired: %s sat=%2d t=%5.0f\n",
				TimeStr(time, 0), sat, t)
			return 0
		}
		for i = 0; i < 3; i++ {
			drs[i] = sbssat.sat[j].lcorr.dpos[i] + sbssat.sat[j].lcorr.dvel[i]*t
		}
		*ddts = sbssat.sat[j].lcorr.daf0 + sbssat.sat[j].lcorr.daf1*t

		Trace(5, "sbslongcorr: sat=%2d drs=%7.2f%7.2f%7.2f ddts=%7.2f\n",
			sat, drs[0], drs[1], drs[2], *ddts*CLIGHT)

		return 1
	}
	/* if sbas satellite without correction, no correction applied */
	if SatSys(sat, nil) == SYS_SBS {
		return 1
	}

	Trace(2, "no sbas long-term correction: %s sat=%2d\n", TimeStr(time, 0), sat)
	return 0
}

/* fast correction -----------------------------------------------------------*/
func SbsFastCorr(time Gtime, sat int, sbssat *SbsSat, prc, fvar *float64) int {
	var (
		t float64
		j int
	)
	Trace(3, "sbsfastcorr: sat=%2d\n", sat)

	for j = 0; j < sbssat.nsat; j++ {
		if sbssat.sat[j].sat != sat {
			continue
		}
		if sbssat.sat[j].fcorr.t0.Time == 0 {
			break
		}
		t = TimeDiff(time, sbssat.sat[j].fcorr.t0) + float64(sbssat.tlat)

		/* expire age of correction or UDRE==14 (not monitored) */
		if math.Abs(t) > float64(MAXSBSAGEF) || sbssat.sat[j].fcorr.udre >= 15 {
			continue
		}
		*prc = sbssat.sat[j].fcorr.prc
		if RRCENA == 1 {
			if sbssat.sat[j].fcorr.ai > 0 && math.Abs(t) <= 8.0*sbssat.sat[j].fcorr.dt {
				*prc += sbssat.sat[j].fcorr.rrc * t
			}
		}
		*fvar = varfcorr(int(sbssat.sat[j].fcorr.udre)) + degfcorr(int(sbssat.sat[j].fcorr.ai))*t*t/2.0

		Trace(5, "sbsfastcorr: sat=%3d prc=%7.2f sig=%7.2f t=%5.0f\n", sat,
			*prc, math.Sqrt(*fvar), t)
		return 1
	}
	Trace(2, "no sbas fast correction: %s sat=%2d\n", TimeStr(time, 0), sat)
	return 0
}

/* sbas satellite ephemeris and clock correction -------------------------------
* correct satellite position and clock bias with sbas satellite corrections
* args   : gtime_t time     I   reception time
*          int    sat       I   satellite
*          nav_t  *nav      I   navigation data
*          double *rs       IO  sat position and corrected {x,y,z} (ecef) (m)
*          double *dts      IO  sat clock bias and corrected (s)
*          double *var      O   sat position and clock variance (m^2)
* return : status (1:ok,0:no correction)
* notes  : before calling the function, sbas satellite correction parameters
*          in navigation data (nav.sbssat) must be set by callig
*          sbsupdatecorr().
*          satellite clock correction include long-term correction and fast
*          correction.
*          sbas clock correction is usually based on L1C/A code. TGD or DCB has
*          to be considered for other codes
*-----------------------------------------------------------------------------*/
func SbsSatCorr(time Gtime, sat int, nav *Nav, rs, dts []float64, fvar *float64) int {
	var (
		drs       [3]float64
		dclk, prc float64
		i         int
	)

	Trace(3, "sbssatcorr : sat=%2d\n", sat)

	/* sbas long term corrections */
	if SbsLongCorr(time, sat, &nav.SbasSat, drs[:], &dclk) == 0 {
		return 0
	}
	/* sbas fast corrections */
	if SbsFastCorr(time, sat, &nav.SbasSat, &prc, fvar) == 0 {
		return 0
	}
	for i = 0; i < 3; i++ {
		rs[i] += drs[i]
	}

	dts[0] += dclk + prc/CLIGHT

	Trace(5, "sbssatcorr: sat=%2d drs=%6.3f %6.3f %6.3f dclk=%.3f %.3f var=%.3f\n",
		sat, drs[0], drs[1], drs[2], dclk, prc/CLIGHT, *fvar)

	return 1
}

/* decode sbas message ---------------------------------------------------------
* decode sbas message frame words and check crc
* args   : gtime_t time     I   reception time
*          int    prn       I   sbas satellite prn number
*          uint32_t *word   I   message frame words (24bit x 10)
*          sbsmsg_t *sbsmsg O   sbas message
* return : status (1:ok,0:crc error)
*-----------------------------------------------------------------------------*/
func SbsDecodeMsg(time Gtime, prn int, words []uint32, sbsmsg *SbsMsg) int {
	var (
		i, j int
		f    [29]uint8
		tow  float64
	)

	Trace(5, "sbsdecodemsg: prn=%d\n", prn)

	if time.Time == 0 {
		return 0
	}
	tow = Time2GpsT(time, &sbsmsg.Week)
	sbsmsg.Tow = int(tow + float64(DTTOL))
	sbsmsg.Prn = uint8(prn)
	for i = 0; i < 7; i++ {
		for j = 0; j < 4; j++ {
			sbsmsg.Msg[i*4+j] = uint8(words[i] >> ((3 - j) * 8))
		}
	}
	sbsmsg.Msg[28] = uint8(words[7]>>18) & 0xC0
	for i = 28; i > 0; i-- {
		f[i] = (sbsmsg.Msg[i] >> 6) + (sbsmsg.Msg[i-1] << 2)
	}
	f[0] = sbsmsg.Msg[0] >> 6

	if Rtk_CRC24q(f[:], 29) == (words[7] & 0xFFFFFF) { /* check crc */
		return 1
	}
	return 0
}
