/*------------------------------------------------------------------------------
* rcvraw.c : receiver raw data functions
*
*          Copyright (C) 2009-2020 by T.TAKASU, All rights reserved.
*          Copyright (C) 2014 by T.SUZUKI, All rights reserved.
*
* references :
*     [1] IS-GPS-200K, Navstar GPS Space Segment/Navigation User Interfaces,
*         March 4, 2019
*     [2] Global navigation satellite system GLONASS interface control document
*         navigation radiosignal in bands L1,L2 (version 5.1), 2008
*     [3] BeiDou satellite navigation system signal in space interface control
*         document open service signal B1I (version 3.0), February, 2019
*     [4] Quasi-Zenith Satellite System Interface Specification Satellite
*         Positioning, Navigation and Timing Service (IS-QZSS-PN-003), November
*         5, 2018
*     [5] European GNSS (Galileo) Open Service Signal In Space Interface Control
*         Document, Issue 1.3, December, 2016
*     [6] ISRO-IRNSS-ICD-SPS-1.1, Indian Regional Navigation Satellite System
*         Signal in Space ICD for Standard Positioning Service version 1.1,
*         August, 2017
*
* version : $Revision:$ $Date:$
* history : 2009/04/10 1.0  new
*           2009/06/02 1.1  support glonass
*           2010/07/31 1.2  support eph_t struct change
*           2010/12/06 1.3  add almanac decoding, support of GW10
*                           change api decode_frame()
*           2013/04/11 1.4  fix bug on decode fit interval
*           2014/01/31 1.5  fix bug on decode fit interval
*           2014/06/22 1.6  add api decode_glostr()
*           2014/06/22 1.7  add api decode_bds_d1(), decode_bds_d2()
*           2014/08/14 1.8  add test_glostr()
*                           add support input format rt17
*           2014/08/31 1.9  suppress warning
*           2014/11/07 1.10 support qzss navigation subframes
*           2016/01/23 1.11 enable septentrio
*           2016/01/28 1.12 add decode_gal_inav() for galileo I/NAV
*           2016/07/04 1.13 support CMR/CMR+
*           2017/05/26 1.14 support TERSUS
*           2018/10/10 1.15 update reference [5]
*                           add set of eph.code/flag for galileo and beidou
*           2018/12/05 1.16 add test of galileo i/nav word type 5
*           2020/11/30 1.17 add API decode_gal_fnav() and decode_irn_nav()
*                           allocate double size of raw.nav.eph[] for multiple
*                            ephemeris sets (e.g. Gallieo I/NAV and F/NAV)
*                           no support of STRFMT_LEXR by API input_raw/rawf()
*                           update references [1], [3] and [4]
*                           add reference [6]
*                           use integer types in stdint.h
*		    2022/05/31 1.0  rewrite rcvraw.c with golang by fxb
*-----------------------------------------------------------------------------*/
package gnssgo

import (
	"math"
	"os"
)

const (
	P2_8  = 0.00390625 /* 2^-8 */
	P2P11 = 2048.0     /* 2^11 */
	P2P12 = 4096.0     /* 2^12 */
	P2P14 = 16384.0    /* 2^14 */
	P2P15 = 32768.0    /* 2^15 */
	P2P16 = 65536.0 /* 2^16 */)

/* get two component bits ----------------------------------------------------*/
func getbitu2(buff []uint8, p1, l1, p2, l2 int) uint32 {
	return (GetBitU(buff, p1, l1) << l2) + GetBitU(buff, p2, l2)
}
func getbits2(buff []uint8, p1, l1, p2, l2 int) int32 {
	if GetBitU(buff, p1, 1) > 0 {
		return int32(GetBits(buff, p1, l1)<<l2) + int32(GetBitU(buff, p2, l2))
	} else {
		return int32(getbitu2(buff, p1, l1, p2, l2))
	}
}

/* get three component bits --------------------------------------------------*/
func getbitu3(buff []uint8, p1, l1, p2, l2, p3, l3 int) uint32 {
	return (GetBitU(buff, p1, l1) << (l2 + l3)) + (GetBitU(buff, p2, l2) << l3) +
		GetBitU(buff, p3, l3)
}
func getbits3(buff []uint8, p1, l1, p2, l2, p3, l3 int) int32 {
	if GetBitU(buff, p1, 1) > 0 {
		return int32(GetBits(buff, p1, l1)<<(l2+l3)) + int32(GetBitU(buff, p2, l2)<<l3) + int32(GetBitU(buff, p3, l3))
	} else {
		return int32(getbitu3(buff, p1, l1, p2, l2, p3, l3))
	}
}

/* merge two components ------------------------------------------------------*/
func merge_two_u(a, b uint32, n int) uint32 {
	return (a << n) + b
}
func merge_two_s(a int32, b uint32, n int) int32 {
	return (a << n) + int32(b)
}

/* get sign-magnitude bits ---------------------------------------------------*/
func getbitg(buff []uint8, pos, len int) float64 {
	value := float64(GetBitU(buff, pos+1, len-1))
	if GetBitU(buff, pos, 1) > 0 {
		return -value
	} else {
		return value
	}
}

/* decode NavIC/IRNSS ephemeris ----------------------------------------------*/
func (eph *Eph) DecodeIrnEph(buff []uint8) int {
	var (
		eph_irn                Eph
		tow1, tow2, toc, sqrtA float64
		i, id1, id2, week      int
	)
	Trace(4, "decode_irn_eph:\n")

	i = 8 /* subframe 1 */
	tow1 = float64(GetBitU(buff, i, 17)) * 12.0
	i += 17 + 2
	id1 = int(GetBitU(buff, i, 2))
	i += 2 + 1
	week = int(GetBitU(buff, i, 10))
	i += 10
	eph_irn.F0 = float64(GetBits(buff, i, 22)) * P2_31
	i += 22
	eph_irn.F1 = float64(GetBits(buff, i, 16)) * P2_43
	i += 16
	eph_irn.F2 = float64(GetBits(buff, i, 8)) * P2_55
	i += 8
	eph_irn.Sva = int(GetBitU(buff, i, 4))
	i += 4
	toc = float64(GetBitU(buff, i, 16)) * 16.0
	i += 16
	eph_irn.Tgd[0] = float64(GetBits(buff, i, 8)) * P2_31
	i += 8
	eph_irn.Deln = float64(GetBits(buff, i, 22)) * P2_41 * SC2RAD
	i += 22
	eph_irn.Iode = int(GetBitU(buff, i, 8))
	i += 8 + 10
	eph_irn.Svh = int(GetBitU(buff, i, 2))
	i += 2
	eph_irn.Cuc = float64(GetBits(buff, i, 15)) * P2_28
	i += 15
	eph_irn.Cus = float64(GetBits(buff, i, 15)) * P2_28
	i += 15
	eph_irn.Cic = float64(GetBits(buff, i, 15)) * P2_28
	i += 15
	eph_irn.Cis = float64(GetBits(buff, i, 15)) * P2_28
	i += 15
	eph_irn.Crc = float64(GetBits(buff, i, 15)) * 0.0625
	i += 15
	eph_irn.Crs = float64(GetBits(buff, i, 15)) * 0.0625
	i += 15
	eph_irn.Idot = float64(GetBits(buff, i, 14)) * P2_43 * SC2RAD

	i = 8*37 + 8 /* subframe 2 */
	tow2 = float64(GetBitU(buff, i, 17)) * 12.0
	i += 17 + 2
	id2 = int(GetBitU(buff, i, 2))
	i += 2 + 1
	eph_irn.M0 = float64(GetBits(buff, i, 32)) * P2_31 * SC2RAD
	i += 32
	eph_irn.Toes = float64(GetBitU(buff, i, 16)) * 16.0
	i += 16
	eph_irn.E = float64(GetBitU(buff, i, 32)) * P2_33
	i += 32
	sqrtA = float64(GetBitU(buff, i, 32)) * P2_19
	i += 32
	eph_irn.OMG0 = float64(GetBits(buff, i, 32)) * P2_31 * SC2RAD
	i += 32
	eph_irn.Omg = float64(GetBits(buff, i, 32)) * P2_31 * SC2RAD
	i += 32
	eph_irn.OMGd = float64(GetBits(buff, i, 22)) * P2_41 * SC2RAD
	i += 22
	eph_irn.I0 = float64(GetBits(buff, i, 32)) * P2_31 * SC2RAD

	/* test subframe id, tow and consistency of toe and toc */
	if id1 != 0 || id2 != 1 || tow1+12.0 != tow2 || toc != eph_irn.Toes {
		return 0
	}
	eph_irn.A = SQR(sqrtA)
	eph_irn.Iodc = eph_irn.Iode
	week = AdjGpsWeek(week)
	eph_irn.Week = week /* week number consistent to toe */
	to := GpsT2Time(eph_irn.Week, eph_irn.Toes)
	eph_irn.Toe, eph_irn.Toc = to, to
	if tow1 < eph_irn.Toes-302400.0 {
		week++
	} else if tow1 > eph_irn.Toes+302400.0 {
		week--
	}
	eph_irn.Ttr = GpsT2Time(week, tow1)
	*eph = eph_irn
	return 1
}

/* decode NavIC/IRNSS iono parameters ----------------------------------------*/
func DecodeIrnIon(buff []uint8, ion []float64) int {
	var i, id3, id4 int

	Trace(4, "decode_irn_ion:\n")

	/* subframe 3 and 4 message ids */
	id3 = int(GetBitU(buff, 8*37*2+30, 6))
	id4 = int(GetBitU(buff, 8*37*3+30, 6))

	/* 11: eop and ionosphere coefficients */
	if id3 == 11 {
		i = 8*37*2 + 174
	} else if id4 == 11 {
		i = 8*37*3 + 174
	} else {
		return 0
	}

	ion[0] = float64(GetBits(buff, i, 8)) * P2_30
	i += 8
	ion[1] = float64(GetBits(buff, i, 8)) * P2_27
	i += 8
	ion[2] = float64(GetBits(buff, i, 8)) * P2_24
	i += 8
	ion[3] = float64(GetBits(buff, i, 8)) * P2_24
	i += 8
	ion[4] = float64(GetBits(buff, i, 8)) * P2P11
	i += 8
	ion[5] = float64(GetBits(buff, i, 8)) * P2P14
	i += 8
	ion[6] = float64(GetBits(buff, i, 8)) * P2P16
	i += 8
	ion[7] = float64(GetBits(buff, i, 8)) * P2P16
	return 1
}

/* decode NavIC/IRNSS UTC parameters -----------------------------------------*/
func DecodeIrnUtc(buff []uint8, utc []float64) int {
	var i, id3, id4 int

	Trace(4, "decode_irn_utc:\n")

	/* subframe 3 and 4 message ids */
	id3 = int(GetBitU(buff, 8*37*2+30, 6))
	id4 = int(GetBitU(buff, 8*37*3+30, 6))

	/* 9 or 26: utc and time sync parameters */
	if id3 == 9 || id3 == 26 {
		i = 8*37*2 + 36
	} else if id4 == 9 || id3 == 26 {
		i = 8*37*3 + 36
	} else {
		return 0
	}

	utc[0] = float64(GetBits(buff, i, 16)) * P2_35
	i += 16 /* A0 */
	utc[1] = float64(GetBits(buff, i, 13)) * P2_51
	i += 13 /* A1 */
	utc[8] = float64(GetBits(buff, i, 7)) * P2_68
	i += 7 /* A2 */
	utc[4] = float64(GetBits(buff, i, 8))
	i += 8 /* dt_LS */
	utc[2] = float64(GetBitU(buff, i, 16)) * 16.0
	i += 16 /* tot */
	utc[3] = float64(GetBitU(buff, i, 10))
	i += 10 /* WNt */
	utc[5] = float64(GetBitU(buff, i, 10))
	i += 10 /* WN_LSF */
	utc[6] = float64(GetBitU(buff, i, 4))
	i += 4                                /* DN */
	utc[7] = float64(GetBits(buff, i, 8)) /* dt_LSF */
	return 1
}

/* decode NavIC/IRNSS navigation data ------------------------------------------
* decode NavIC/IRNSS navigation data (ref [6] 5.9-6)
* args   : uint8_t *buff    I   NavIC/IRNSS subframe data (CRC checked)
*                                 buff[  0- 36]: subframe 1 (292 bits)
*                                 buff[ 37- 73]: subframe 2
*                                 buff[ 74-110]: subframe 3
*                                 buff[111-147]: subframe 4
*          eph_t *eph       IO  NavIC/IRNSS ephemeris        (NULL: not output)
*          double *ion      IO  NavIC/IRNSS iono parametgers (NULL: not output)
*                                 ion[0-3]: alpha_0,...,alpha_3
*                                 ion[4-7]: beta_0,...,beta_3
*          double *utc      IO  NavIC/IRNSS UTC parametgers  (NULL: not output)
*                                 utc[0-3]: A0,A1,tot,WNt
*                                 utc[4-7]: dt_LS,WN_LSF,DN,dt_LSF
*                                 utc[8]  : A2
* return : status (1:ok,0:error)
*-----------------------------------------------------------------------------*/
func DecodeIrnNav(buff []uint8, eph *Eph, ion, utc []float64) int {
	Trace(4, "decode_irn_nav:\n")

	if eph != nil && eph.DecodeIrnEph(buff) == 0 {
		return 0
	}
	if ion != nil && DecodeIrnIon(buff, ion) == 0 {
		return 0
	}
	if utc != nil && DecodeIrnUtc(buff, utc) == 0 {
		return 0
	}
	return 1
}

/* decode Galileo I/NAV ephemeris --------------------------------------------*/
func DecodeGalInavEph(buff []uint8, eph *Eph) int {
	var (
		eph_gal                                         Eph
		tow, toc, tt, sqrtA                             float64
		i, week, svid, e5b_hs, e1b_hs, e5b_dvs, e1b_dvs int
		ctype                                           [6]int
		iod_nav                                         [4]int
	)

	Trace(4, "decode_gal_inav_eph:\n")

	i = 128 /* word type 1 */
	ctype[0] = int(GetBitU(buff, i, 6))
	i += 6
	iod_nav[0] = int(GetBitU(buff, i, 10))
	i += 10
	eph_gal.Toes = float64(GetBitU(buff, i, 14)) * 60.0
	i += 14
	eph_gal.M0 = float64(GetBits(buff, i, 32)) * P2_31 * SC2RAD
	i += 32
	eph_gal.E = float64(GetBitU(buff, i, 32)) * P2_33
	i += 32
	sqrtA = float64(GetBitU(buff, i, 32)) * P2_19

	i = 128 * 2 /* word type 2 */
	ctype[1] = int(GetBitU(buff, i, 6))
	i += 6
	iod_nav[1] = int(GetBitU(buff, i, 10))
	i += 10
	eph_gal.OMG0 = float64(GetBits(buff, i, 32)) * P2_31 * SC2RAD
	i += 32
	eph_gal.I0 = float64(GetBits(buff, i, 32)) * P2_31 * SC2RAD
	i += 32
	eph_gal.Omg = float64(GetBits(buff, i, 32)) * P2_31 * SC2RAD
	i += 32
	eph_gal.Idot = float64(GetBits(buff, i, 14)) * P2_43 * SC2RAD

	i = 128 * 3 /* word type 3 */
	ctype[2] = int(GetBitU(buff, i, 6))
	i += 6
	iod_nav[2] = int(GetBitU(buff, i, 10))
	i += 10
	eph_gal.OMGd = float64(GetBits(buff, i, 24)) * P2_43 * SC2RAD
	i += 24
	eph_gal.Deln = float64(GetBits(buff, i, 16)) * P2_43 * SC2RAD
	i += 16
	eph_gal.Cuc = float64(GetBits(buff, i, 16)) * P2_29
	i += 16
	eph_gal.Cus = float64(GetBits(buff, i, 16)) * P2_29
	i += 16
	eph_gal.Crc = float64(GetBits(buff, i, 16)) * P2_5
	i += 16
	eph_gal.Crs = float64(GetBits(buff, i, 16)) * P2_5
	i += 16
	eph_gal.Sva = int(GetBitU(buff, i, 8))

	i = 128 * 4 /* word type 4 */
	ctype[3] = int(GetBitU(buff, i, 6))
	i += 6
	iod_nav[3] = int(GetBitU(buff, i, 10))
	i += 10
	svid = int(GetBitU(buff, i, 6))
	i += 6
	eph_gal.Cic = float64(GetBits(buff, i, 16)) * P2_29
	i += 16
	eph_gal.Cis = float64(GetBits(buff, i, 16)) * P2_29
	i += 16
	toc = float64(GetBitU(buff, i, 14)) * 60.0
	i += 14
	eph_gal.F0 = float64(GetBits(buff, i, 31)) * P2_34
	i += 31
	eph_gal.F1 = float64(GetBits(buff, i, 21)) * P2_46
	i += 21
	eph_gal.F2 = float64(GetBits(buff, i, 6)) * P2_59

	i = 128 * 5 /* word type 5 */
	ctype[4] = int(GetBitU(buff, i, 6))
	i += 6 + 11 + 11 + 14 + 5
	eph_gal.Tgd[0] = float64(GetBits(buff, i, 10)) * P2_32
	i += 10 /* BGD E5a/E1 */
	eph_gal.Tgd[1] = float64(GetBits(buff, i, 10)) * P2_32
	i += 10 /* BGD E5b/E1 */
	e5b_hs = int(GetBitU(buff, i, 2))
	i += 2
	e1b_hs = int(GetBitU(buff, i, 2))
	i += 2
	e5b_dvs = int(GetBitU(buff, i, 1))
	i += 1
	e1b_dvs = int(GetBitU(buff, i, 1))
	i += 1
	week = int(GetBitU(buff, i, 12))
	i += 12 /* gst-week */
	tow = float64(GetBitU(buff, i, 20))

	/* test word types */
	if ctype[0] != 1 || ctype[1] != 2 || ctype[2] != 3 || ctype[3] != 4 || ctype[4] != 5 {
		Trace(2, "decode_gal_inav error: type=%d %d %d %d %d\n", ctype[0], ctype[1],
			ctype[2], ctype[3], ctype[4])
		return 0
	}
	/* test consistency of iod_nav */
	if iod_nav[0] != iod_nav[1] || iod_nav[0] != iod_nav[2] || iod_nav[0] != iod_nav[3] {
		Trace(2, "decode_gal_inav error: iod_nav=%d %d %d %d\n", iod_nav[0],
			iod_nav[1], iod_nav[2], iod_nav[3])
		return 0
	}
	if eph_gal.Sat = SatNo(SYS_GAL, svid); eph_gal.Sat == 0 {
		Trace(2, "decode_gal_inav svid error: svid=%d\n", svid)
		return 0
	}
	eph_gal.A = sqrtA * sqrtA
	eph_gal.Iode, eph_gal.Iodc = iod_nav[0], iod_nav[0]
	eph_gal.Svh = (e5b_hs << 7) | (e5b_dvs << 6) | (e1b_hs << 1) | e1b_dvs
	eph_gal.Ttr = GsT2Time(week, tow)
	tt = TimeDiff(GsT2Time(week, eph_gal.Toes), eph_gal.Ttr)
	if tt > 302400.0 {
		week-- /* week consistent with toe */
	} else if tt < -302400.0 {
		week++
	}
	eph_gal.Toe = GsT2Time(week, eph_gal.Toes)
	eph_gal.Toc = GsT2Time(week, toc)
	eph_gal.Week = week + 1024 /* gal-week = gst-week + 1024 */
	eph_gal.Code = (1 << 9)    /* I/NAV: af0-2,Toc,SISA for E5b-E1 */
	*eph = eph_gal
	return 1
}

/* decode Galileo I/NAV iono parameters --------------------------------------*/
func DecodeGalInavIon(buff []uint8, ion []float64) int {
	i := 128 * 5 /* word type 5 */

	Trace(4, "decode_gal_inav_ion:\n")

	if GetBitU(buff, i, 6) != 5 {
		return 0
	}
	i += 6
	ion[0] = float64(GetBitU(buff, i, 11)) * 0.25
	i += 11
	ion[1] = float64(GetBits(buff, i, 11)) * P2_8
	i += 11
	ion[2] = float64(GetBits(buff, i, 14)) * P2_15
	i += 14
	ion[3] = float64(GetBitU(buff, i, 5))
	return 1
}

/* decode Galileo I/NAV UTC parameters ---------------------------------------*/
func DecodeGalInavUtc(buff []uint8, utc []float64) int {
	i := 128 * 6 /* word type 6 */

	Trace(4, "decode_gal_inav_utc:\n")

	if GetBitU(buff, i, 6) != 6 {
		return 0
	}
	i += 6
	utc[0] = float64(GetBits(buff, i, 32)) * P2_30
	i += 32 /* A0 */
	utc[1] = float64(GetBits(buff, i, 24)) * P2_50
	i += 24 /* A1 */
	utc[4] = float64(GetBits(buff, i, 8))
	i += 8 /* dt_LS */
	utc[2] = float64(GetBitU(buff, i, 8)) * 3600.0
	i += 8 /* tot */
	utc[3] = float64(GetBitU(buff, i, 8))
	i += 8 /* WNt */
	utc[5] = float64(GetBitU(buff, i, 8))
	i += 8 /* WN_LSF */
	utc[6] = float64(GetBitU(buff, i, 3))
	i += 3                                /* DN */
	utc[7] = float64(GetBits(buff, i, 8)) /* dt_LSF */
	return 1
}

/* decode Galileo I/NAV navigation data ----------------------------------------
* decode Galileo I/NAV navigation data (ref [5] 4.3)
* args   : uint8_t *buff    I   Galileo I/NAV subframe data (CRC checked)
*                                 buff[ 0- 15]: word type 0 (128 bits)
*                                 buff[16- 31]: word type 1
*                                 buff[32- 47]: word type 2
*                                 buff[48- 63]: word type 3
*                                 buff[64- 79]: word type 4
*                                 buff[80- 95]: word type 5
*                                 buff[96-111]: word type 6
*          eph_t    *eph    IO  Galileo I/NAV ephemeris       (NULL: not output)
*          double   *ion    IO  Galileo I/NAV iono parameters (NULL: not output)
*                                 ion[0-3]: a_i0,a_i1,a_i2,flags
*          double   *utc    IO  Galileo I/NAV UTC parameters  (NULL: not output)
*                                 utc[0-3]: A0,A1,tot,WNt
*                                 utc[4-7]: dt_LS,WN_LSF,DN,dt_LSF
* return : status (1:ok,0:error)
*-----------------------------------------------------------------------------*/
func DecodeGalInav(buff []uint8, eph *Eph, ion, utc []float64) int {
	Trace(4, "decode_gal_fnav:\n")

	if eph != nil && DecodeGalInavEph(buff, eph) == 0 {
		return 0
	}
	if ion != nil && DecodeGalInavIon(buff, ion) == 0 {
		return 0
	}
	if utc != nil && DecodeGalInavUtc(buff, utc) == 0 {
		return 0
	}
	return 1
}

/* decode Galileo F/NAV ephemeris --------------------------------------------*/
func DecodeGalFnavEph(buff []uint8, eph *Eph) int {
	var (
		eph_gal                  Eph
		tow                      [4]float64
		toc, tt, sqrtA           float64
		i, svid, e5a_hs, e5a_dvs int
		week                     [3]int
		ctype, iod_nav           [4]int
	)

	Trace(4, "decode_gal_fnav_eph:\n")

	i = 0 /* page type 1 */
	ctype[0] = int(GetBitU(buff, i, 6))
	i += 6
	svid = int(GetBitU(buff, i, 6))
	i += 6
	iod_nav[0] = int(GetBitU(buff, i, 10))
	i += 10
	toc = float64(GetBitU(buff, i, 14)) * 60.0
	i += 14
	eph_gal.F0 = float64(GetBits(buff, i, 31)) * P2_34
	i += 31
	eph_gal.F1 = float64(GetBits(buff, i, 21)) * P2_46
	i += 21
	eph_gal.F2 = float64(GetBits(buff, i, 6)) * P2_59
	i += 6
	eph_gal.Sva = int(GetBitU(buff, i, 8))
	i += 8 + 11 + 11 + 14 + 5
	eph_gal.Tgd[0] = float64(GetBits(buff, i, 10)) * P2_32
	i += 10 /* BGD E5a/E1 */
	e5a_hs = int(GetBitU(buff, i, 2))
	i += 2
	week[0] = int(GetBitU(buff, i, 12))
	i += 12 /* gst-week */
	tow[0] = float64(GetBitU(buff, i, 20))
	i += 20
	e5a_dvs = int(GetBitU(buff, i, 1))

	i = 31 * 8 /* page type 2 */
	ctype[1] = int(GetBitU(buff, i, 6))
	i += 6
	iod_nav[1] = int(GetBitU(buff, i, 10))
	i += 10
	eph_gal.M0 = float64(GetBits(buff, i, 32)) * P2_31 * SC2RAD
	i += 32
	eph_gal.OMGd = float64(GetBits(buff, i, 24)) * P2_43 * SC2RAD
	i += 24
	eph_gal.E = float64(GetBitU(buff, i, 32)) * P2_33
	i += 32
	sqrtA = float64(GetBitU(buff, i, 32)) * P2_19
	i += 32
	eph_gal.OMG0 = float64(GetBits(buff, i, 32)) * P2_31 * SC2RAD
	i += 32
	eph_gal.Idot = float64(GetBits(buff, i, 14)) * P2_43 * SC2RAD
	i += 14
	week[1] = int(GetBitU(buff, i, 12))
	i += 12
	tow[1] = float64(GetBitU(buff, i, 20))

	i = 62 * 8 /* page type 3 */
	ctype[2] = int(GetBitU(buff, i, 6))
	i += 6
	iod_nav[2] = int(GetBitU(buff, i, 10))
	i += 10
	eph_gal.I0 = float64(GetBits(buff, i, 32)) * P2_31 * SC2RAD
	i += 32
	eph_gal.Omg = float64(GetBits(buff, i, 32)) * P2_31 * SC2RAD
	i += 32
	eph_gal.Deln = float64(GetBits(buff, i, 16)) * P2_43 * SC2RAD
	i += 16
	eph_gal.Cuc = float64(GetBits(buff, i, 16)) * P2_29
	i += 16
	eph_gal.Cus = float64(GetBits(buff, i, 16)) * P2_29
	i += 16
	eph_gal.Crc = float64(GetBits(buff, i, 16)) * P2_5
	i += 16
	eph_gal.Crs = float64(GetBits(buff, i, 16)) * P2_5
	i += 16
	eph_gal.Toes = float64(GetBitU(buff, i, 14)) * 60.0
	i += 14
	week[2] = int(GetBitU(buff, i, 12))
	i += 12
	tow[2] = float64(GetBitU(buff, i, 20))

	i = 93 * 8 /* page type 4 */
	ctype[3] = int(GetBitU(buff, i, 6))
	i += 6
	iod_nav[3] = int(GetBitU(buff, i, 10))
	i += 10
	eph_gal.Cic = float64(GetBits(buff, i, 16)) * P2_29
	i += 16
	eph_gal.Cis = float64(GetBits(buff, i, 16)) * P2_29

	/* test page types */
	if ctype[0] != 1 || ctype[1] != 2 || ctype[2] != 3 || ctype[3] != 4 {
		Trace(2, "decode_gal_fnav error: svid=%d type=%d %d %d %d\n", svid,
			ctype[0], ctype[1], ctype[2], ctype[3])
		return 0
	}
	/* test consistency of iod_nav */
	if iod_nav[0] != iod_nav[1] || iod_nav[0] != iod_nav[2] || iod_nav[0] != iod_nav[3] {
		Trace(2, "decode_gal_fnav error: svid=%d iod_nav=%d %d %d %d\n", svid,
			iod_nav[0], iod_nav[1], iod_nav[2], iod_nav[3])
		return 0
	}
	if eph_gal.Sat = SatNo(SYS_GAL, svid); eph_gal.Sat == 0 {
		Trace(2, "decode_gal_fnav svid error: svid=%d\n", svid)
		return 0
	}
	eph_gal.A = sqrtA * sqrtA
	eph_gal.Tgd[1] = 0.0 /* BGD E5b/E1 */
	eph_gal.Iode, eph_gal.Iodc = iod_nav[0], iod_nav[0]
	eph_gal.Svh = (e5a_hs << 4) | (e5a_dvs << 3)
	eph_gal.Ttr = GsT2Time(week[0], tow[0])
	tt = TimeDiff(GsT2Time(week[0], eph_gal.Toes), eph_gal.Ttr)
	if tt > 302400.0 {
		week[0]-- /* week consistent with toe */
	} else if tt < -302400.0 {
		week[0]++
	}
	eph_gal.Toe = GsT2Time(week[0], eph_gal.Toes)
	eph_gal.Toc = GsT2Time(week[0], toc)
	eph_gal.Week = week[0] + 1024 /* gal-week = gst-week + 1024 */
	eph_gal.Code = (1 << 8)       /* F/NAV: af0-af2,Toc,SISA for E5a,E1 */
	*eph = eph_gal
	return 1
}

/* decode Galileo F/NAV iono parameters --------------------------------------*/
func DecodeGalFnavIon(buff []uint8, ion []float64) int {
	i := 0 /* page type 1 */

	Trace(4, "decode_gal_fnav_ion:\n")

	if GetBitU(buff, i, 6) != 1 {
		return 0
	}
	i += 6 + 6 + 10 + 14 + 31 + 21 + 6 + 8
	ion[0] = float64(GetBitU(buff, i, 11)) * 0.25
	i += 11
	ion[1] = float64(GetBits(buff, i, 11)) * P2_8
	i += 11
	ion[2] = float64(GetBits(buff, i, 14)) * P2_15
	i += 14
	ion[3] = float64(GetBitU(buff, i, 5))
	return 1
}

/* decode Galileo F/NAV UTC parameters ---------------------------------------*/
func DecodeGalFnavUtc(buff []uint8, utc []float64) int {
	i := 93 * 8 /* page type 4 */

	Trace(4, "decode_gal_fnav_utc:\n")

	if GetBitU(buff, i, 6) != 4 {
		return 0
	}
	i += 6 + 10 + 16 + 16
	utc[0] = float64(GetBits(buff, i, 32)) * P2_30
	i += 32 /* A0 */
	utc[1] = float64(GetBits(buff, i, 24)) * P2_50
	i += 24 /* A1 */
	utc[4] = float64(GetBits(buff, i, 8))
	i += 8 /* dt_LS */
	utc[2] = float64(GetBitU(buff, i, 8)) * 3600.0
	i += 8 /* tot */
	utc[3] = float64(GetBitU(buff, i, 8))
	i += 8 /* WN_ot */
	utc[5] = float64(GetBitU(buff, i, 8))
	i += 8 /* WN_LSF */
	utc[6] = float64(GetBitU(buff, i, 3))
	i += 3                                /* DN */
	utc[7] = float64(GetBits(buff, i, 8)) /* dt_LSF */
	return 1
}

/* decode Galileo F/NAV navigation data ----------------------------------------
* decode Galileo F/NAV navigation data (ref [5] 4.2)
* args   : uint8_t *buff    I   Galileo F/NAV subframe data (CRC checked)
*                                 buff[  0- 30]: page type 1 (244 bit)
*                                 buff[ 31- 61]: page type 2
*                                 buff[ 62- 92]: page type 3
*                                 buff[ 93-123]: page type 4
*                                 buff[124-154]: page type 5
*                                 buff[155-185]: page type 6
*          eph_t    *eph    IO  Galileo F/NAV ephemeris       (NULL: not output)
*          double   *ion    IO  Galileo F/NAV iono parameters (NULL: not output)
*                                 ion[0-3]: a_i0,a_i1,a_i2,flags
*          double   *utc    IO  Galileo F/NAV UTC parameters  (NULL: not output)
*                                 utc[0-3]: A0,A1,tot,WNt
*                                 utc[4-7]: dt_LS,WN_LSF,DN,dt_LSF
* return : status (1:ok,0:error)
*-----------------------------------------------------------------------------*/
func DecodeGalFnav(buff []uint8, eph *Eph, ion, utc []float64) int {
	Trace(4, "decode_gal_fnav:\n")

	if eph != nil && DecodeGalFnavEph(buff, eph) == 0 {
		return 0
	}
	if ion != nil && DecodeGalFnavIon(buff, ion) == 0 {
		return 0
	}
	if utc != nil && DecodeGalFnavUtc(buff, utc) == 0 {
		return 0
	}
	return 1
}

/* decode BDS D1 navigation data ---------------------------------------------*/
func DecodeBDSD1Eph(buff []uint8, eph *Eph) int {
	var (
		eph_bds                      Eph
		toc_bds, sqrtA               float64
		toe1, toe2, sow1, sow2, sow3 uint32
		i, frn1, frn2, frn3          int
	)

	i = 8 * 38 * 0 /* subframe 1 */
	frn1 = int(GetBitU(buff, i+15, 3))
	sow1 = getbitu2(buff, i+18, 8, i+30, 12)
	eph_bds.Svh = int(GetBitU(buff, i+42, 1))  /* SatH1 */
	eph_bds.Iodc = int(GetBitU(buff, i+43, 5)) /* AODC */
	eph_bds.Sva = int(GetBitU(buff, i+48, 4))
	eph_bds.Week = int(GetBitU(buff, i+60, 13)) /* week in BDT */
	toc_bds = float64(getbitu2(buff, i+73, 9, i+90, 8)) * 8.0
	eph_bds.Tgd[0] = float64(GetBits(buff, i+98, 10)) * 0.1 * 1e-9
	eph_bds.Tgd[1] = float64(getbits2(buff, i+108, 4, i+120, 6)) * 0.1 * 1e-9
	eph_bds.F2 = float64(GetBits(buff, i+214, 11)) * P2_66
	eph_bds.F0 = float64(getbits2(buff, i+225, 7, i+240, 17)) * P2_33
	eph_bds.F1 = float64(getbits2(buff, i+257, 5, i+270, 17)) * P2_50
	eph_bds.Iode = int(GetBitU(buff, i+287, 5)) /* AODE */

	i = 8 * 38 * 1 /* subframe 2 */
	frn2 = int(GetBitU(buff, i+15, 3))
	sow2 = getbitu2(buff, i+18, 8, i+30, 12)
	eph_bds.Deln = float64(getbits2(buff, i+42, 10, i+60, 6)) * P2_43 * SC2RAD
	eph_bds.Cuc = float64(getbits2(buff, i+66, 16, i+90, 2)) * P2_31
	eph_bds.M0 = float64(getbits2(buff, i+92, 20, i+120, 12)) * P2_31 * SC2RAD
	eph_bds.E = float64(getbitu2(buff, i+132, 10, i+150, 22)) * P2_33
	eph_bds.Cus = float64(GetBits(buff, i+180, 18)) * P2_31
	eph_bds.Crc = float64(getbits2(buff, i+198, 4, i+210, 14)) * P2_6
	eph_bds.Crs = float64(getbits2(buff, i+224, 8, i+240, 10)) * P2_6
	sqrtA = float64(getbitu2(buff, i+250, 12, i+270, 20)) * P2_19
	toe1 = GetBitU(buff, i+290, 2) /* TOE 2-MSB */
	eph_bds.A = sqrtA * sqrtA

	i = 8 * 38 * 2 /* subframe 3 */
	frn3 = int(GetBitU(buff, i+15, 3))
	sow3 = getbitu2(buff, i+18, 8, i+30, 12)
	toe2 = getbitu2(buff, i+42, 10, i+60, 5) /* TOE 5-LSB */
	eph_bds.I0 = float64(getbits2(buff, i+65, 17, i+90, 15)) * P2_31 * SC2RAD
	eph_bds.Cic = float64(getbits2(buff, i+105, 7, i+120, 11)) * P2_31
	eph_bds.OMGd = float64(getbits2(buff, i+131, 11, i+150, 13)) * P2_43 * SC2RAD
	eph_bds.Cis = float64(getbits2(buff, i+163, 9, i+180, 9)) * P2_31
	eph_bds.Idot = float64(getbits2(buff, i+189, 13, i+210, 1)) * P2_43 * SC2RAD
	eph_bds.OMG0 = float64(getbits2(buff, i+211, 21, i+240, 11)) * P2_31 * SC2RAD
	eph_bds.Omg = float64(getbits2(buff, i+251, 11, i+270, 21)) * P2_31 * SC2RAD
	eph_bds.Toes = float64(merge_two_u(toe1, toe2, 15)) * 8.0

	/* check consistency of subframe ids, sows and toe/toc */
	if frn1 != 1 || frn2 != 2 || frn3 != 3 {
		Trace(2, "decode_bds_d1_eph error: frn=%d %d %d\n", frn1, frn2, frn3)
		return 0
	}
	if sow2 != sow1+6 || sow3 != sow2+6 {
		Trace(2, "decode_bds_d1_eph error: sow=%d %d %d\n", sow1, sow2, sow3)
		return 0
	}
	if toc_bds != eph_bds.Toes {
		Trace(2, "decode_bds_d1_eph error: toe=%.0f toc=%.0f\n", eph_bds.Toes,
			toc_bds)
		return 0
	}
	eph_bds.Ttr = BDT2GpsT(BDT2Time(eph_bds.Week, float64(sow1))) /* bdt . gpst */
	if eph_bds.Toes > float64(sow1)+302400.0 {
		eph_bds.Week++
	} else if eph_bds.Toes < float64(sow1)-302400.0 {
		eph_bds.Week--
	}
	eph_bds.Toe = BDT2GpsT(BDT2Time(eph_bds.Week, eph_bds.Toes))
	eph_bds.Toc = BDT2GpsT(BDT2Time(eph_bds.Week, toc_bds))
	eph_bds.Code = 0 /* data source = unknown */
	eph_bds.Flag = 1 /* nav type = IGSO/MEO */
	*eph = eph_bds
	return 1
}

/* decode BDS D1 iono parameters ---------------------------------------------*/
func DecodeBDSD1Ion(buff []uint8, ion []float64) int {
	i := 8 * 38 * 0 /* subframe 1 */

	Trace(4, "decode_bds_d1_ion:\n")

	/* subframe 1 */
	if GetBitU(buff, i+15, 3) != 1 {
		return 0
	}

	ion[0] = float64(GetBits(buff, i+126, 8)) * P2_30
	ion[1] = float64(GetBits(buff, i+134, 8)) * P2_27
	ion[2] = float64(GetBits(buff, i+150, 8)) * P2_24
	ion[3] = float64(GetBits(buff, i+158, 8)) * P2_24
	ion[4] = float64(getbits2(buff, i+166, 6, i+180, 2)) * P2P11
	ion[5] = float64(GetBits(buff, i+182, 8)) * P2P14
	ion[6] = float64(GetBits(buff, i+190, 8)) * P2P16
	ion[7] = float64(getbits2(buff, i+198, 4, i+210, 4)) * P2P16
	return 1
}

/* decode BDS D1 UTC parameters ----------------------------------------------*/
func DecodeBDSD1Utc(buff []uint8, utc []float64) int {
	i := 8 * 38 * 4 /* subframe 5 */

	Trace(4, "decode_bds_d1_utc:\n")

	if GetBitU(buff, 15, 3) != 1 {
		return 0 /* subframe 1 */
	}

	/* subframe 5 page 10 */
	if GetBitU(buff, i+15, 3) != 5 || GetBitU(buff, i+43, 7) != 10 {
		return 0
	}

	utc[4] = float64(getbits2(buff, i+50, 2, i+60, 6))             /* dt_LS */
	utc[7] = float64(GetBits(buff, i+66, 8))                       /* dt_LSF */
	utc[5] = float64(GetBitU(buff, i+74, 8))                       /* WN_LSF */
	utc[0] = float64(getbits2(buff, i+90, 22, i+120, 10)) * P2_30  /* A0 */
	utc[1] = float64(getbits2(buff, i+130, 12, i+150, 12)) * P2_50 /* A1 */
	utc[6] = float64(GetBitU(buff, i+162, 8))                      /* DN */
	utc[2] = float64(getbitu2(buff, i+18, 8, i+30, 12))            /* SOW */
	utc[3] = float64(GetBitU(buff, 60, 13))                        /* WN */
	return 1
}

/* decode BDS D1 navigation data -----------------------------------------------
* decode BDS D1 navigation data (IGSO/MEO) (ref [3] 5.2)
* args   : uint8_t *buff    I   BDS D1 subframe data (CRC checked with parity)
*                                  buff[  0- 37]: subframe 1 (300 bits)
*                                  buff[ 38- 75]: subframe 2
*                                  buff[ 76-113]: subframe 3
*                                  buff[114-141]: subframe 4
*                                  buff[152-189]: subframe 5
*          eph_t    *eph    IO  BDS D1 ephemeris       (NULL: not output)
*          double   *ion    IO  BDS D1 iono parameters (NULL: not output)
*                                 ion[0-3]: alpha_0,...,alpha_3
*                                 ion[4-7]: beta_0,...,beta_3
*          double   *utc    IO  BDS D1 UTC parameters  (NULL: not output)
*                                 utc[0-2]: A0,A1,tot,WNt
*                                 utc[4-7]: dt_LS,WN_LSF,DN,dt_LSF
* return : status (1:ok,0:error)
*-----------------------------------------------------------------------------*/
func DecodeBDSD1(buff []uint8, eph *Eph, ion, utc []float64) int {
	Trace(4, "decode_bds_d1:\n")

	if eph != nil && DecodeBDSD1Eph(buff, eph) == 0 {
		return 0
	}
	if ion != nil && DecodeBDSD1Ion(buff, ion) == 0 {
		return 0
	}
	if utc != nil && DecodeBDSD1Utc(buff, utc) == 0 {
		return 0
	}
	return 1
}

/* decode BDS D2 ephemeris ---------------------------------------------------*/
func DecodeBDSD2Eph(buff []uint8, eph *Eph) int {
	var (
		eph_bds                                               Eph
		toc_bds, sqrtA                                        float64
		f1p4, cucp5, ep6, cicp7, i0p8, OMGdp9, omgp10         uint32
		sow1, sow3, sow4, sow5, sow6, sow7, sow8, sow9, sow10 uint32
		i, f1p3, cucp4, ep5, cicp6, i0p7, OMGdp8, omgp9       int
		pgn1, pgn3, pgn4, pgn5, pgn6, pgn7, pgn8, pgn9, pgn10 int
	)

	Trace(4, "decode_bds_d1_eph:\n")

	i = 8 * 38 * 0 /* page 1 */
	pgn1 = int(GetBitU(buff, i+42, 4))
	sow1 = getbitu2(buff, i+18, 8, i+30, 12)
	eph_bds.Svh = int(GetBitU(buff, i+46, 1))  /* SatH1 */
	eph_bds.Iodc = int(GetBitU(buff, i+47, 5)) /* AODC */
	eph_bds.Sva = int(GetBitU(buff, i+60, 4))
	eph_bds.Week = int(GetBitU(buff, i+64, 13)) /* week in BDT */
	toc_bds = float64(getbitu2(buff, i+77, 5, i+90, 12)) * 8.0
	eph_bds.Tgd[0] = float64(GetBits(buff, i+102, 10)) * 0.1 * 1e-9
	eph_bds.Tgd[1] = float64(GetBits(buff, i+120, 10)) * 0.1 * 1e-9

	i = 8 * 38 * 2 /* page 3 */
	pgn3 = int(GetBitU(buff, i+42, 4))
	sow3 = getbitu2(buff, i+18, 8, i+30, 12)
	eph_bds.F0 = float64(getbits2(buff, i+100, 12, i+120, 12)) * P2_33
	f1p3 = int(GetBits(buff, i+132, 4))

	i = 8 * 38 * 3 /* page 4 */
	pgn4 = int(GetBitU(buff, i+42, 4))
	sow4 = getbitu2(buff, i+18, 8, i+30, 12)
	f1p4 = getbitu2(buff, i+46, 6, i+60, 12)
	eph_bds.F2 = float64(getbits2(buff, i+72, 10, i+90, 1)) * P2_66
	eph_bds.Iode = int(GetBitU(buff, i+91, 5)) /* AODE */
	eph_bds.Deln = float64(GetBits(buff, i+96, 16)) * P2_43 * SC2RAD
	cucp4 = int(GetBits(buff, i+120, 14))

	i = 8 * 38 * 4 /* page 5 */
	pgn5 = int(GetBitU(buff, i+42, 4))
	sow5 = getbitu2(buff, i+18, 8, i+30, 12)
	cucp5 = GetBitU(buff, i+46, 4)
	eph_bds.M0 = float64(getbits3(buff, i+50, 2, i+60, 22, i+90, 8)) * P2_31 * SC2RAD
	eph_bds.Cus = float64(getbits2(buff, i+98, 14, i+120, 4)) * P2_31
	ep5 = int(GetBits(buff, i+124, 10))

	i = 8 * 38 * 5 /* page 6 */
	pgn6 = int(GetBitU(buff, i+42, 4))
	sow6 = getbitu2(buff, i+18, 8, i+30, 12)
	ep6 = getbitu2(buff, i+46, 6, i+60, 16)
	sqrtA = float64(getbitu3(buff, i+76, 6, i+90, 22, i+120, 4)) * P2_19
	cicp6 = int(GetBits(buff, i+124, 10))
	eph_bds.A = sqrtA * sqrtA

	i = 8 * 38 * 6 /* page 7 */
	pgn7 = int(GetBitU(buff, i+42, 4))
	sow7 = getbitu2(buff, i+18, 8, i+30, 12)
	cicp7 = getbitu2(buff, i+46, 6, i+60, 2)
	eph_bds.Cis = float64(GetBits(buff, i+62, 18)) * P2_31
	eph_bds.Toes = float64(getbitu2(buff, i+80, 2, i+90, 15)) * 8.0
	i0p7 = int(getbits2(buff, i+105, 7, i+120, 14))

	i = 8 * 38 * 7 /* page 8 */
	pgn8 = int(GetBitU(buff, i+42, 4))
	sow8 = getbitu2(buff, i+18, 8, i+30, 12)
	i0p8 = getbitu2(buff, i+46, 6, i+60, 5)
	eph_bds.Crc = float64(getbits2(buff, i+65, 17, i+90, 1)) * P2_6
	eph_bds.Crs = float64(GetBits(buff, i+91, 18)) * P2_6
	OMGdp8 = int(getbits2(buff, i+109, 3, i+120, 16))

	i = 8 * 38 * 8 /* page 9 */
	pgn9 = int(GetBitU(buff, i+42, 4))
	sow9 = getbitu2(buff, i+18, 8, i+30, 12)
	OMGdp9 = GetBitU(buff, i+46, 5)
	eph_bds.OMG0 = float64(getbits3(buff, i+51, 1, i+60, 22, i+90, 9)) * P2_31 * SC2RAD
	omgp9 = int(getbits2(buff, i+99, 13, i+120, 14))

	i = 8 * 38 * 9 /* page 10 */
	pgn10 = int(GetBitU(buff, i+42, 4))
	sow10 = getbitu2(buff, i+18, 8, i+30, 12)
	omgp10 = GetBitU(buff, i+46, 5)
	eph_bds.Idot = float64(getbits2(buff, i+51, 1, i+60, 13)) * P2_43 * SC2RAD

	/* check consistency of page numbers, sows and toe/toc */
	if pgn1 != 1 || pgn3 != 3 || pgn4 != 4 || pgn5 != 5 || pgn6 != 6 || pgn7 != 7 || pgn8 != 8 || pgn9 != 9 ||
		pgn10 != 10 {
		Trace(2, "decode_bds_d2 error: pgn=%d %d %d %d %d %d %d %d %d\n",
			pgn1, pgn3, pgn4, pgn5, pgn6, pgn7, pgn8, pgn9, pgn10)
		return 0
	}
	if sow3 != sow1+6 || sow4 != sow3+3 || sow5 != sow4+3 || sow6 != sow5+3 ||
		sow7 != sow6+3 || sow8 != sow7+3 || sow9 != sow8+3 || sow10 != sow9+3 {
		Trace(2, "decode_bds_d2 error: sow=%d %d %d %d %d %d %d %d %d\n",
			sow1, sow3, sow4, sow5, sow6, sow7, sow8, sow9, sow10)
		return 0
	}
	if toc_bds != eph_bds.Toes {
		Trace(2, "decode_bds_d2 error: toe=%.0f toc=%.0f\n", eph_bds.Toes,
			toc_bds)
		return 0
	}
	eph_bds.F1 = float64(merge_two_s(int32(f1p3), f1p4, 18)) * P2_50
	eph_bds.Cuc = float64(merge_two_s(int32(cucp4), cucp5, 4)) * P2_31
	eph_bds.E = float64(merge_two_s(int32(ep5), ep6, 22)) * P2_33
	eph_bds.Cic = float64(merge_two_s(int32(cicp6), cicp7, 8)) * P2_31
	eph_bds.I0 = float64(merge_two_s(int32(i0p7), i0p8, 11)) * P2_31 * SC2RAD
	eph_bds.OMGd = float64(merge_two_s(int32(OMGdp8), OMGdp9, 5)) * P2_43 * SC2RAD
	eph_bds.Omg = float64(merge_two_s(int32(omgp9), omgp10, 5)) * P2_31 * SC2RAD

	eph_bds.Ttr = BDT2GpsT(BDT2Time(eph_bds.Week, float64(sow1))) /* bdt . gpst */
	if eph_bds.Toes > float64(sow1)+302400.0 {
		eph_bds.Week++
	} else if eph_bds.Toes < float64(sow1)-302400.0 {
		eph_bds.Week--
	}
	eph_bds.Toe = BDT2GpsT(BDT2Time(eph_bds.Week, eph_bds.Toes))
	eph_bds.Toc = BDT2GpsT(BDT2Time(eph_bds.Week, toc_bds))
	eph_bds.Code = 0 /* data source = unknown */
	eph_bds.Flag = 2 /* nav type = GEO */
	*eph = eph_bds
	return 1
}

/* decode BDS D2 UTC parameters ----------------------------------------------*/
func DecodeBDSD2Utc(buff []uint8, utc []float64) int {
	i := 8 * 38 * 10 /* subframe 5 pase 102 */

	Trace(4, "decode_bds_d2_utc:\n")

	/* subframe 1 page 1 */
	if GetBitU(buff, 15, 3) != 1 || GetBitU(buff, 42, 4) != 1 {
		return 0
	}

	/* subframe 5 page 102 */
	if GetBitU(buff, i+15, 3) != 5 || GetBitU(buff, i+43, 7) != 102 {
		return 0
	}

	utc[4] = float64(getbits2(buff, i+50, 2, i+60, 6))             /* dt_LS */
	utc[7] = float64(GetBits(buff, i+66, 8))                       /* dt_LSF */
	utc[5] = float64(GetBitU(buff, i+74, 8))                       /* WN_LSF */
	utc[0] = float64(getbits2(buff, i+90, 22, i+120, 10)) * P2_30  /* A0 */
	utc[1] = float64(getbits2(buff, i+130, 12, i+150, 12)) * P2_50 /* A1 */
	utc[6] = float64(GetBitU(buff, i+162, 8))                      /* DN */
	utc[2] = float64(getbits2(buff, i+18, 8, i+30, 12))            /* SOW */
	utc[3] = float64(GetBitU(buff, 64, 13))                        /* WN */
	return 1
}

/* decode BDS D2 navigation data -----------------------------------------------
* decode BDS D2 navigation data (GEO) (ref [3] 5.3)
* args   : uint8_t *buff    I   BDS D2 subframe data (CRC checked with parity)
*                                 buff[  0- 37]: subframe 1 page 1 (300 bits)
*                                 buff[ 38- 75]: subframe 1 page 2
*                                 ...
*                                 buff[342-379]: subframe 1 page 10
*                                 buff[380-417]: subframe 5 page 102
*          eph_t    *eph    IO  BDS D2 ephemeris       (NULL: not output)
*          double   *utc    IO  BDS D2 UTC parameters  (NULL: not output)
*                                 utc[0-2]: A0,A1,tot,WNt
*                                 utc[4-7]: dt_LS,WN_LSF,DN,dt_LSF
* return : status (1:ok,0:error)
*-----------------------------------------------------------------------------*/
func DecodeBDSD2(buff []uint8, eph *Eph, utc []float64) int {
	Trace(4, "decode_bds_d2:\n")

	if eph != nil && DecodeBDSD2Eph(buff, eph) == 0 {
		return 0
	}
	if utc != nil && DecodeBDSD2Utc(buff, utc) == 0 {
		return 0
	}
	return 1
}

/* test hamming code of GLONASS navigation string ------------------------------
* test hamming code of GLONASS navigation string (ref [2] 4.7)
* args   : uint8_t *buff    I   GLONASS navigation string with hamming code
*                                 buff[ 0]: string bit 85-78
*                                 buff[ 1]: string bit 77-70
*                                 ...
*                                 buff[10]: string bit  5- 1 (0 padded)
* return : status (1:ok,0:error)
*-----------------------------------------------------------------------------*/
func test_glostr(buff []uint8) int {
	var xor_8bit [256]uint8 = [256]uint8{ /* xor of 8 bits */
		0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
		1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
		1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
		0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
		1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
		0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
		0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
		1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0}
	var mask_hamming [][12]uint8 = [][12]uint8{ /* mask of hamming codes */
		{0x55, 0x55, 0x5A, 0xAA, 0xAA, 0xAA, 0xB5, 0x55, 0x6A, 0xD8, 0x08},
		{0x66, 0x66, 0x6C, 0xCC, 0xCC, 0xCC, 0xD9, 0x99, 0xB3, 0x68, 0x10},
		{0x87, 0x87, 0x8F, 0x0F, 0x0F, 0x0F, 0x1E, 0x1E, 0x3C, 0x70, 0x20},
		{0x07, 0xF8, 0x0F, 0xF0, 0x0F, 0xF0, 0x1F, 0xE0, 0x3F, 0x80, 0x40},
		{0xF8, 0x00, 0x0F, 0xFF, 0xF0, 0x00, 0x1F, 0xFF, 0xC0, 0x00, 0x80},
		{0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xFF, 0xE0, 0x00, 0x00, 0x01, 0x00},
		{0xFF, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00},
		{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF8}}
	var cs uint8
	var i, j, n int

	for i = 0; i < 8; i++ {
		for j, cs = 0, 0; j < 11; j++ {
			cs ^= xor_8bit[buff[j]&mask_hamming[i][j]]
		}
		if cs > 0 {
			n++
		}
	}
	if n == 0 || (n == 2 && cs > 0) {
		return 1
	}
	return 0
}

/* decode GLONASS ephemeris --------------------------------------------------*/
func DecodeGlostrEph(buff []uint8, geph *GEph) int {
	var (
		geph_glo                                                                               GEph
		tow, tod, tof, toe                                                                     float64
		P, P1, P2, P3, P4, tk_h, tk_m, tk_s, tb, ln, NT, slot, M, week, frn1, frn2, frn3, frn4 int
		i                                                                                      int = 1
	)

	Trace(4, "decode_glostr_eph:\n")
	if 1 == 0 {
		Trace(2, "unused declared variant:%d%d%d%d%d%d%d%d", P, P1, P2, P3, P4, ln, NT, M)
	}
	/* frame 1 */
	frn1 = int(GetBitU(buff, i, 4))
	i += 4 + 2
	//	P1 = int(GetBitU(buff, i, 2))
	i += 2
	tk_h = int(GetBitU(buff, i, 5))
	i += 5
	tk_m = int(GetBitU(buff, i, 6))
	i += 6
	tk_s = int(GetBitU(buff, i, 1)) * 30
	i += 1
	geph_glo.Vel[0] = getbitg(buff, i, 24) * P2_20 * 1e3
	i += 24
	geph_glo.Acc[0] = getbitg(buff, i, 5) * P2_30 * 1e3
	i += 5
	geph_glo.Pos[0] = getbitg(buff, i, 27) * P2_11 * 1e3
	i += 27 + 4

	/* frame 2 */
	frn2 = int(GetBitU(buff, i, 4))
	i += 4
	geph_glo.Svh = int(GetBitU(buff, i, 1))
	i += 1 + 2 /* MSB of Bn */
	//	P2 = int(GetBitU(buff, i, 1))
	i += 1
	tb = int(GetBitU(buff, i, 7))
	i += 7 + 5
	geph_glo.Vel[1] = getbitg(buff, i, 24) * P2_20 * 1e3
	i += 24
	geph_glo.Acc[1] = getbitg(buff, i, 5) * P2_30 * 1e3
	i += 5
	geph_glo.Pos[1] = getbitg(buff, i, 27) * P2_11 * 1e3
	i += 27 + 4

	/* frame 3 */
	frn3 = int(GetBitU(buff, i, 4))
	i += 4
	//	P3 = int(GetBitU(buff, i, 1))
	i += 1
	geph_glo.Gamn = getbitg(buff, i, 11) * P2_40
	i += 11 + 1
	//	P = int(GetBitU(buff, i, 2))
	i += 2
	//	ln = int(GetBitU(buff, i, 1))
	i += 1
	geph_glo.Vel[2] = getbitg(buff, i, 24) * P2_20 * 1e3
	i += 24
	geph_glo.Acc[2] = getbitg(buff, i, 5) * P2_30 * 1e3
	i += 5
	geph_glo.Pos[2] = getbitg(buff, i, 27) * P2_11 * 1e3
	i += 27 + 4

	/* frame 4 */
	frn4 = int(GetBitU(buff, i, 4))
	i += 4
	geph_glo.Taun = getbitg(buff, i, 22) * P2_30
	i += 22
	geph_glo.DTaun = getbitg(buff, i, 5) * P2_30
	i += 5
	geph_glo.Age = int(GetBitU(buff, i, 5))
	i += 5 + 14
	//	P4 = int(GetBitU(buff, i, 1))
	i += 1
	geph_glo.Sva = int(GetBitU(buff, i, 4))
	i += 4 + 3
	//	NT = int(GetBitU(buff, i, 11))
	i += 11
	slot = int(GetBitU(buff, i, 5))
	i += 5
	//	M = int(GetBitU(buff, i, 2))

	if frn1 != 1 || frn2 != 2 || frn3 != 3 || frn4 != 4 {
		Trace(2, "decode_glostr error: frn=%d %d %d %d \n", frn1, frn2, frn3, frn4)
		return 0
	}
	if geph_glo.Sat = SatNo(SYS_GLO, slot); geph_glo.Sat == 0 {
		Trace(2, "decode_glostr error: slot=%d\n", slot)
		return 0
	}
	geph_glo.Frq = 0 /* set default */
	geph_glo.Iode = tb
	tow = Time2GpsT(GpsT2Utc(geph.Tof), &week)
	tod = math.Mod(tow, 86400.0)
	tow -= tod
	tof = float64(tk_h)*3600.0 + float64(tk_m)*60.0 + float64(tk_s) - 10800.0 /* lt.utc */
	if tof < tod-43200.0 {
		tof += 86400.0
	} else if tof > tod+43200.0 {
		tof -= 86400.0
	}
	geph_glo.Tof = Utc2GpsT(GpsT2Time(week, tow+tof))
	toe = float64(tb)*900.0 - 10800.0 /* lt.utc */
	if toe < tod-43200.0 {
		toe += 86400.0
	} else if toe > tod+43200.0 {
		toe -= 86400.0
	}
	geph_glo.Toe = Utc2GpsT(GpsT2Time(week, tow+toe)) /* utc.gpst */
	*geph = geph_glo
	return 1
}

/* decode GLONASS UTC parameters ---------------------------------------------*/
func DecodeGlostrUtc(buff []uint8, utc []float64) int {
	i := 1 + 80*4 /* frame 5 */

	Trace(4, "decode_glostr_utc:\n")

	/* frame 5 */
	if GetBitU(buff, i, 4) != 5 {
		return 0
	}
	i += 4 + 11
	utc[0] = float64(GetBits(buff, i, 32)) * P2_31
	i += 32 + 1 + 6                                /* tau_C */
	utc[1] = float64(GetBits(buff, i, 22)) * P2_30 /* tau_GPS */
	utc[2], utc[3], utc[4], utc[5], utc[6], utc[7] = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
	return 1
}

/* decode GLONASS navigation data strings --------------------------------------
* decode GLONASS navigation data string (ref [2])
* args   : uint8_t *buff    I   GLONASS navigation data string
*                               (w/o hamming and time mark)
*                                 buff[ 0- 9]: string 1 (77 bits)
*                                 buff[10-19]: string 2
*                                 buff[20-29]: string 3
*                                 buff[30-39]: string 4
*                                 buff[40-49]: string 5
*          geph_t *geph     IO  GLONASS ephemeris      (NULL: not output)
*          double *utc      IO  GLONASS UTC parameters (NULL: not output)
*                                 utc[0]  : A0 (=-tau_C)
*                                 utc[1-7]: reserved
* return : status (1:ok,0:error)
* notes  : geph.tof should be set to frame time within 1/2 day before calling
*          geph.frq is set to 0
*-----------------------------------------------------------------------------*/
func Decode_Glostr(buff []uint8, geph *GEph, utc []float64) int {
	Trace(4, "decode_glostr:\n")

	if geph != nil && DecodeGlostrEph(buff, geph) == 0 {
		return 0
	}
	if utc != nil && DecodeGlostrUtc(buff, utc) == 0 {
		return 0
	}
	return 1
}

/* decode GPS/QZSS ephemeris -------------------------------------------------*/
func DecodeFrameEph(buff []uint8, eph *Eph) int {
	var (
		eph_sat                                         Eph
		tow1, tow2, tow3, toc, sqrtA                    float64
		i, id1, id2, id3, week, iodc0, iodc1, iode, tgd int
	)
	i = 48

	Trace(4, "decode_frame_eph:\n")
	if 1 == 0 {
		Trace(2, "unused declared variant%f%f", tow2, tow3)
	}
	i = 240*0 + 24 /* subframe 1 */
	tow1 = float64(GetBitU(buff, i, 17)) * 6.0
	i += 17 + 2
	id1 = int(GetBitU(buff, i, 3))
	i += 3 + 2
	week = int(GetBitU(buff, i, 10))
	i += 10
	eph_sat.Code = int(GetBitU(buff, i, 2))
	i += 2
	eph_sat.Sva = int(GetBitU(buff, i, 4))
	i += 4 /* ura index */
	eph_sat.Svh = int(GetBitU(buff, i, 6))
	i += 6
	iodc0 = int(GetBitU(buff, i, 2))
	i += 2
	eph_sat.Flag = int(GetBitU(buff, i, 1))
	i += 1 + 87
	tgd = int(GetBits(buff, i, 8))
	i += 8
	iodc1 = int(GetBitU(buff, i, 8))
	i += 8
	toc = float64(GetBitU(buff, i, 16)) * 16.0
	i += 16
	eph_sat.F2 = float64(GetBits(buff, i, 8)) * P2_55
	i += 8
	eph_sat.F1 = float64(GetBits(buff, i, 16)) * P2_43
	i += 16
	eph_sat.F0 = float64(GetBits(buff, i, 22)) * P2_31

	i = 240*1 + 24 /* subframe 2 */
	//	tow2 = float64(GetBitU(buff, i, 17)) * 6.0
	i += 17 + 2
	id2 = int(GetBitU(buff, i, 3))
	i += 3 + 2
	eph_sat.Iode = int(GetBitU(buff, i, 8))
	i += 8
	eph_sat.Crs = float64(GetBits(buff, i, 16)) * P2_5
	i += 16
	eph_sat.Deln = float64(GetBits(buff, i, 16)) * P2_43 * SC2RAD
	i += 16
	eph_sat.M0 = float64(GetBits(buff, i, 32)) * P2_31 * SC2RAD
	i += 32
	eph_sat.Cuc = float64(GetBits(buff, i, 16)) * P2_29
	i += 16
	eph_sat.E = float64(GetBitU(buff, i, 32)) * P2_33
	i += 32
	eph_sat.Cus = float64(GetBits(buff, i, 16)) * P2_29
	i += 16
	sqrtA = float64(GetBitU(buff, i, 32)) * P2_19
	i += 32
	eph_sat.Toes = float64(GetBitU(buff, i, 16)) * 16.0
	i += 16
	if GetBitU(buff, i, 1) > 0 {
		eph_sat.Fit = 0.0
	} else {
		eph_sat.Fit = 4.0 /* 0:4hr,1:>4hr */
	}

	i = 240*2 + 24 /* subframe 3 */
	//	tow3 = float64(GetBitU(buff, i, 17)) * 6.0
	i += 17 + 2
	id3 = int(GetBitU(buff, i, 3))
	i += 3 + 2
	eph_sat.Cic = float64(GetBits(buff, i, 16)) * P2_29
	i += 16
	eph_sat.OMG0 = float64(GetBits(buff, i, 32)) * P2_31 * SC2RAD
	i += 32
	eph_sat.Cis = float64(GetBits(buff, i, 16)) * P2_29
	i += 16
	eph_sat.I0 = float64(GetBits(buff, i, 32)) * P2_31 * SC2RAD
	i += 32
	eph_sat.Crc = float64(GetBits(buff, i, 16)) * P2_5
	i += 16
	eph_sat.Omg = float64(GetBits(buff, i, 32)) * P2_31 * SC2RAD
	i += 32
	eph_sat.OMGd = float64(GetBits(buff, i, 24)) * P2_43 * SC2RAD
	i += 24
	iode = int(GetBitU(buff, i, 8))
	i += 8
	eph_sat.Idot = float64(GetBits(buff, i, 14)) * P2_43 * SC2RAD

	eph_sat.A = sqrtA * sqrtA
	eph_sat.Iodc = (iodc0 << 8) + iodc1
	eph_sat.Tgd[0] = 0.0
	if tgd != -128 {
		eph_sat.Tgd[0] = float64(tgd) * P2_31 /* ref [4] */
	}

	/* test subframe ids */
	if id1 != 1 || id2 != 2 || id3 != 3 {
		Trace(2, "decode_frame_eph error: id=%d %d %d\n", id1, id2, id3)
		return 0
	}
	/* test iode and iodc consistency */
	if iode != eph_sat.Iode || iode != (eph_sat.Iodc&0xFF) {
		Trace(2, "decode_frame_eph error: iode=%d %d iodc=%d\n", eph_sat.Iode,
			iode, eph_sat.Iodc)
		return 0
	}
	eph_sat.Week = AdjGpsWeek(week)
	eph_sat.Ttr = GpsT2Time(eph_sat.Week, tow1)
	if eph_sat.Toes < tow1-302400.0 {
		eph_sat.Week++
	} else if eph_sat.Toes > tow1+302400.0 {
		eph_sat.Week--
	}
	eph_sat.Toe = GpsT2Time(eph_sat.Week, eph_sat.Toes)
	eph_sat.Toc = GpsT2Time(eph_sat.Week, toc)
	*eph = eph_sat
	return 1
}

/* decode GPS/QZSS satellite almanac -----------------------------------------*/
func DecodeAlmSat(buff []uint8, ctype int, alm *Alm) {
	var (
		toa0                        Gtime
		deltai, sqrtA, i_ref, e_ref float64
		i, f0                       int
	)
	i = 50

	Trace(4, "decode_alm_sat:\n")

	/* type=0:GPS,1:QZS-QZO,2:QZS-GEO */
	switch ctype {
	case 0:
		e_ref = 0.0
		i_ref = 0.3
	case 1:
		e_ref = 0.06
		i_ref = 0.25
	default:
		e_ref = 0.0
		i_ref = 0.0
	}

	alm.E = float64(GetBits(buff, i, 16))*P2_21 + e_ref
	i += 16
	alm.Toas = float64(GetBitU(buff, i, 8)) * 4096.0
	i += 8
	deltai = float64(GetBits(buff, i, 16)) * P2_19
	i += 16
	alm.OMGd = float64(GetBits(buff, i, 16)) * P2_38 * SC2RAD
	i += 16
	alm.Svh = int(GetBitU(buff, i, 8))
	i += 8
	sqrtA = float64(GetBitU(buff, i, 24)) * P2_11
	i += 24
	alm.OMG0 = float64(GetBits(buff, i, 24)) * P2_23 * SC2RAD
	i += 24
	alm.Omg = float64(GetBits(buff, i, 24)) * P2_23 * SC2RAD
	i += 24
	alm.M0 = float64(GetBits(buff, i, 24)) * P2_23 * SC2RAD
	i += 24
	f0 = int(GetBits(buff, i, 8))
	i += 8
	alm.F1 = float64(GetBits(buff, i, 11)) * P2_38
	i += 11
	alm.F0 = float64(GetBitU(buff, i, 3))*P2_17 + float64(f0)*P2_20
	alm.A = sqrtA * sqrtA
	alm.I0 = (i_ref + deltai) * SC2RAD
	alm.Week = 0
	alm.Toa = toa0
}

/* decode GPS almanac/health -------------------------------------------------*/
func DecodeAlmGps(buff []uint8, frm int, alm []Alm) int {
	var i, j, sat, toas, week int
	svid := int(GetBitU(buff, 50, 6))

	Trace(4, "decode_alm_gps:\n")

	if (frm == 5 && svid >= 1 && svid <= 24) || (frm == 4 && svid >= 25 && svid <= 32) {
		if sat = SatNo(SYS_GPS, svid); sat == 0 {
			return 0
		}
		alm[sat-1].Sat = sat
		DecodeAlmSat(buff, 0, &alm[sat])
		return 1
	} else if frm == 5 && svid == 51 { /* subframe 5 page 25 */
		i = 56
		toas = int(GetBitU(buff, i, 8)) * 4096
		i += 8
		week = int(GetBitU(buff, i, 8))
		i += 8
		for j = 0; j < 24; i, j = i+6, j+1 {
			if sat = SatNo(SYS_GPS, j+1); sat == 0 {
				continue
			}
			alm[sat-1].Svh = int(GetBitU(buff, i, 6))
		}
		for j = 0; j < 32; j++ {
			if sat = SatNo(SYS_GPS, j+1); sat == 0 || alm[sat-1].Sat != sat ||
				alm[sat-1].Toas != float64(toas) {
				continue
			}
			alm[sat-1].Week = AdjGpsWeek(week)
			alm[sat-1].Toa = GpsT2Time(alm[sat-1].Week, float64(toas))
		}
		return 1
	} else if frm == 4 && svid == 63 { /* subframe 4 page 25 */
		i = 186
		for j = 0; j < 8; i, j = i+6, j+1 {
			if sat = SatNo(SYS_GPS, j+25); sat == 0 {
				continue
			}
			alm[sat-1].Svh = int(GetBitU(buff, i, 6))
		}
		return 1
	}
	return 0
}

/* decode QZSS almanac/health ------------------------------------------------*/
func DecodeAlmQzs(buff []uint8, alm []Alm) int {
	var i, j, sat, toas, week int

	svid := int(GetBitU(buff, 50, 6))

	Trace(4, "decode_alm_qzs:\n")

	if svid >= 1 && svid <= 9 {
		if sat = SatNo(SYS_QZS, 192+svid); sat == 0 {
			return 0
		}
		alm[sat-1].Sat = sat
		if svid <= 6 {
			DecodeAlmSat(buff, 1, &alm[sat])
		} else {
			DecodeAlmSat(buff, 2, &alm[sat])

		}

		return 1
	} else if svid == 51 {
		i = 56
		toas = int(GetBitU(buff, i, 8)) * 4096
		i += 8
		week = int(GetBitU(buff, i, 8))
		i += 8
		for j = 0; j < 10; i, j = i+6, j+1 {
			if sat = SatNo(SYS_QZS, 193+j); sat == 0 {
				continue
			}
			alm[sat].Svh = int(GetBitU(buff, i, 6))
		}
		for j = 0; j < 10; j++ {
			if sat = SatNo(SYS_QZS, 193+j); sat == 0 || alm[sat-1].Sat != sat ||
				alm[sat-1].Toas != float64(toas) {
				continue
			}
			alm[sat-1].Week = AdjGpsWeek(week)
			alm[sat-1].Toa = GpsT2Time(alm[sat-1].Week, float64(toas))
		}
		return 1
	}
	return 0
}

/* decode GPS/QZSS almanac/health --------------------------------------------*/
func DecodeFrameAlm(buff []uint8, alm []Alm) int {
	var frm, dataid, ret, index int

	Trace(4, "decode_frame_alm:\n")

	for frm, index = 4, 90; frm <= 5; frm, index = frm+1, index+30 { /* subframe 4/5 */
		if int(GetBitU(buff[index:], 43, 3)) != frm {
			continue
		}
		dataid = int(GetBitU(buff[index:], 48, 2))

		if dataid == 1 { /* GPS */
			ret |= DecodeAlmGps(buff[index:], frm, alm)
		} else if dataid == 3 { /* QZSS */
			ret |= DecodeAlmQzs(buff[index:], alm)
		}
	}
	return ret
}

/* decode GPS/QZSS iono parameters -------------------------------------------*/
func DecodeFrameIon(buff []uint8, ion []float64) int {
	var i, frm, index int

	Trace(4, "decode_frame_ion:\n")

	/* subframe 4/5 and svid=56 (page18) (wide area for QZSS) */
	for frm, index = 4, 90; frm <= 5; frm, index = frm+1, index+30 {
		if frm == 5 && GetBitU(buff[index:], 48, 2) == 1 {
			continue
		}
		if int(GetBitU(buff[index:], 43, 3)) != frm || GetBitU(buff[index:], 50, 6) != 56 {
			continue
		}
		i = 56
		ion[0] = float64(GetBits(buff[index:], i, 8)) * P2_30
		i += 8
		ion[1] = float64(GetBits(buff[index:], i, 8)) * P2_27
		i += 8
		ion[2] = float64(GetBits(buff[index:], i, 8)) * P2_24
		i += 8
		ion[3] = float64(GetBits(buff[index:], i, 8)) * P2_24
		i += 8
		ion[4] = float64(GetBits(buff[index:], i, 8)) * P2P11
		i += 8
		ion[5] = float64(GetBits(buff[index:], i, 8)) * P2P14
		i += 8
		ion[6] = float64(GetBits(buff[index:], i, 8)) * P2P16
		i += 8
		ion[7] = float64(GetBits(buff[index:], i, 8)) * P2P16
		return 1
	}
	return 0
}

/* decode GPS/QZSS UTC parameters --------------------------------------------*/
func DecodeFrameUtc(buff []uint8, utc []float64) int {
	var i, frm, index int

	Trace(4, "decode_frame_utc:\n")

	/* subframe 4/5 and svid=56 (page18) */
	for frm, index = 4, 90; frm <= 5; frm, index = frm+1, index+30 {
		if frm == 5 && GetBitU(buff[index:], 48, 2) == 1 {
			continue
		}
		if int(GetBitU(buff, 43, 3)) != frm || GetBitU(buff[index:], 50, 6) != 56 {
			continue
		}
		i = 120
		utc[1] = float64(GetBits(buff[index:], i, 24)) * P2_50
		i += 24 /* A1 (s) */
		utc[0] = float64(GetBits(buff[index:], i, 32)) * P2_30
		i += 32 /* A0 (s) */
		utc[2] = float64(GetBitU(buff[index:], i, 8)) * P2P12
		i += 8 /* tot (s) */
		utc[3] = float64(GetBitU(buff[index:], i, 8))
		i += 8 /* WNt */
		utc[4] = float64(GetBits(buff[index:], i, 8))
		i += 8 /* dt_LS */
		utc[5] = float64(GetBitU(buff[index:], i, 8))
		i += 8 /* WN_LSF */
		utc[6] = float64(GetBitU(buff[index:], i, 8))
		i += 8                                        /* DN */
		utc[7] = float64(GetBits(buff[index:], i, 8)) /* dt_LSF */
		return 1
	}
	return 0
}

/* decode GPS/QZSS navigation data ---------------------------------------------
* decode GPS/QZSS navigation data (ref [1],[4])
* args   : uint8_t *buff    I   GPS/QZSS navigation data (w/o parity bits)
*                                 buff[  0- 29]: subframe 1 (240 bits)
*                                 buff[ 30- 59]: subframe 2
*                                 buff[ 60- 89]: subframe 3
*                                 buff[ 90-119]: subframe 4
*                                 buff[120-149]: subframe 5
*          eph_t *eph       IO  GPS/QZSS ephemeris       (NULL: not output)
*          alm_t *alm       IO  GPS/QZSS almanac/health  (NULL: not output)
*                                 alm[sat-1]: almanac/health (sat=sat no)
*          double *ion      IO  GPS/QZSS iono parameters (NULL: not output)
*                                 ion[0-3]: alpha_0,...,alpha_3
*                                 ion[4-7]: beta_0,...,beta_3
*          double *utc      IO  GPST/QZSS UTC parameters (NULL: not output)
*                                 utc[0-3]: A0,A1,tot,WNt(8bit)
*                                 utc[4-7]: dt_LS,WN_LSF(8bit),DN,dt_LSF
* return : status (1:ok,0:error or no data)
* notes  : use CPU time to resolve modulo 1024 ambiguity of the week number
*          see ref [1]
*-----------------------------------------------------------------------------*/
func DecodeFrame(buff []uint8, eph *Eph, alm []Alm, ion, utc []float64) int {
	Trace(4, "decode_frame:\n")

	if eph != nil && DecodeFrameEph(buff, eph) == 0 {
		return 0
	}
	if alm != nil && DecodeFrameAlm(buff, alm) == 0 {
		return 0
	}
	if ion != nil && DecodeFrameIon(buff, ion) == 0 {
		return 0
	}
	if utc != nil && DecodeFrameUtc(buff, utc) == 0 {
		return 0
	}
	return 1
}

/* initialize receiver raw data control ----------------------------------------
* initialize receiver raw data control struct and reallocate observation and
* epheris buffer
* args   : raw_t *raw       IO  receiver raw data control struct
*          int   format     I   stream format (STRFMT_???)
* return : status (1:ok,0:memory allocation error)
*-----------------------------------------------------------------------------*/
func (raw *Raw) InitRaw(format int) int {
	var (
		time0     Gtime
		data0     ObsD
		eph0      Eph  = Eph{Sat: 0, Iode: -1, Iodc: -1}
		alm0      Alm  = Alm{Sat: 0, Svh: -1}
		geph0     GEph = GEph{Sat: 0, Iode: -1}
		seph0     SEph
		sbsmsg0   SbsMsg
		i, j, ret int = 0, 0, 1
	)

	Trace(4, "init_raw: format=%d\n", format)
	if raw == nil {
		Trace(2, "input raw is nil \n")
		return 0
	}
	raw.Time = time0
	raw.EphSet, raw.EphSat = 0, 0
	raw.Sbsmsg = sbsmsg0
	raw.MsgType[0] = '0'
	for i = 0; i < MAXSAT; i++ {
		for j = 0; j < 380; j++ {
			raw.SubFrm[i][j] = 0
		}
		for j = 0; j < NFREQ+NEXOBS; j++ {
			raw.Tobs[i][j] = time0
			raw.LockTime[i][j] = 0.0
			raw.Halfc[i][j] = 0
		}
		raw.Icpp[i], raw.Off[i], raw.PrCA[i], raw.DpCA[i] = 0.0, 0.0, 0.0, 0.0
	}
	for i = 0; i < len(raw.FreqNum); i++ {
		raw.FreqNum[i] = 0
	}
	raw.Icpc = 0.0
	raw.NumByte, raw.Len = 0, 0
	raw.Iod, raw.Flag, raw.Tbase, raw.OutType = 0, 0, 0, 0
	raw.Tod = -1
	for i = 0; i < len(raw.Buff); i++ {
		raw.Buff[i] = 0
	}
	raw.Opt = ""
	raw.Format = -1

	raw.ObsData.n = 0
	raw.ObsBuf.n = 0
	raw.ObsData.Data = make([]ObsD, MAXOBS)
	raw.ObsBuf.Data = make([]ObsD, MAXOBS)
	raw.NavData.Ephs = make([]Eph, MAXSAT*2)
	raw.NavData.Alm = make([]Alm, MAXSAT)
	raw.NavData.Geph = make([]GEph, NSATGLO)
	raw.NavData.Seph = make([]SEph, NSATSBS*2)
	raw.RcvData = nil

	for i = 0; i < MAXOBS; i++ {
		raw.ObsData.Data[i] = data0
	}
	for i = 0; i < MAXOBS; i++ {
		raw.ObsBuf.Data[i] = data0
	}
	for i = 0; i < MAXSAT*2; i++ {
		raw.NavData.Ephs[i] = eph0
	}
	for i = 0; i < MAXSAT; i++ {
		raw.NavData.Alm[i] = alm0
	}
	for i = 0; i < NSATGLO; i++ {
		raw.NavData.Geph[i] = geph0
	}
	for i = 0; i < NSATSBS*2; i++ {
		raw.NavData.Seph[i] = seph0
	}
	raw.StaData.Name, raw.StaData.Marker = "", ""
	raw.StaData.AntDes, raw.StaData.AntSno = "", ""
	raw.StaData.Type, raw.StaData.RecVer, raw.StaData.RecSN = "", "", ""
	raw.StaData.AntSetup, raw.StaData.Itrf, raw.StaData.DelType = 0, 0, 0
	for i = 0; i < 3; i++ {
		raw.StaData.Pos[i], raw.StaData.Del[i] = 0.0, 0.0
	}
	raw.StaData.Hgt = 0.0

	/* initialize receiver dependent data */
	raw.Format = format
	switch byte(format) {
	// case STRFMT_RT17: ret=init_rt17(raw); break;
	}
	if ret == 0 {
		raw = nil
		return 0
	}
	return 1
}

/* free receiver raw data control ----------------------------------------------
* free observation and ephemeris buffer in receiver raw data control struct
* args   : raw_t  *raw      IO  receiver raw data control struct
* return : none
*-----------------------------------------------------------------------------*/
func (raw *Raw) FreeRaw() {
}

type InputFileRaw interface {
	// case STRFMT_OEM4:
	input_oem4f(fp *os.File) int
	// case STRFMT_OEM3:
	input_oem3f(fp *os.File) int
	// case STRFMT_UBX:
	input_ubxf(fp *os.File) int
	// case STRFMT_SS2:
	input_ss2f(fp *os.File) int
	// case STRFMT_CRES:
	input_cresf(fp *os.File) int
	// case STRFMT_STQ:
	input_stqf(fp *os.File) int
	// case STRFMT_JAVAD:
	input_javadf(fp *os.File) int
	// case STRFMT_NVS:
	input_nvsf(fp *os.File) int
	// case STRFMT_BINEX:
	input_bnxf(fp *os.File) int
	// case STRFMT_RT17:
	input_rt17f(fp *os.File) int
	// case STRFMT_SEPT:
	input_sbff(fp *os.File) int
}

/* input receiver raw data from stream -----------------------------------------
* fetch next receiver raw data and input a message from stream
* args   : raw_t  *raw      IO  receiver raw data control struct
*          int    format    I   receiver raw data format (STRFMT_???)
*          uint8_t data     I   stream data (1 byte)
* return : status (-1: error message, 0: no message, 1: input observation data,
*                  2: input ephemeris, 3: input sbas message,
*                  9: input ion/utc parameter)
*-----------------------------------------------------------------------------*/
func (raw *Raw) InputRaw(format int, data uint8) int {
	Trace(4, "input_raw: format=%d data=0x%02x\n", format, data)

	switch byte(format) {
	case STRFMT_OEM4:
		return Input_oem4(raw, data)
	case STRFMT_OEM3:
		return Input_oem3(raw, data)
	case STRFMT_UBX:
		return input_ubx(raw, data)
	case STRFMT_SS2:
		return input_ss2(raw, data)
	case STRFMT_CRES:
		return Input_cres(raw, data)
	case STRFMT_STQ:
		return Input_stq(raw, data)
	case STRFMT_JAVAD:
		return Input_javad(raw, data)
	case STRFMT_NVS:
		return Input_nvs(raw, data)
	case STRFMT_BINEX:
		return Input_bnx(raw, data)
		// case STRFMT_RT17:
		// 	return input_rt17(raw, data)
		// case STRFMT_SEPT:
		// 	return input_sbf(raw, data)
	}
	return 0
}

/* input receiver raw data from file -------------------------------------------
* fetch next receiver raw data and input a message from file
* args   : raw_t  *raw      IO  receiver raw data control struct
*          int    format    I   receiver raw data format (STRFMT_???)
*          FILE   *fp       I   file pointer
* return : status(-2: end of file/format error, -1...31: same as above)
*-----------------------------------------------------------------------------*/
func (raw *Raw) InputRawF(format int, fp *os.File) int {
	Trace(4, "input_rawf: format=%d\n", format)

	switch byte(format) {
	case STRFMT_OEM4:
		return input_oem4f(raw, fp)
	case STRFMT_OEM3:
		return input_oem3f(raw, fp)
	case STRFMT_UBX:
		return input_ubxf(raw, fp)
	case STRFMT_SS2:
		return input_ss2f(raw, fp)
	case STRFMT_CRES:
		return Input_cresf(raw, fp)
	case STRFMT_STQ:
		return Input_stqf(raw, fp)
	case STRFMT_JAVAD:
		return Input_javadf(raw, fp)
	case STRFMT_NVS:
		return Input_nvsf(raw, fp)
	case STRFMT_BINEX:
		return Input_bnxf(raw, fp)
		// case STRFMT_RT17:
		//	return input_rt17f(raw, fp)
		// case STRFMT_SEPT:
		//		return input_sbff(raw, fp)
	}
	return -2
}
