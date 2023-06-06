package gnssgo

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"io"
	"math"
	"os"
	"strings"
)

/*------------------------------------------------------------------------------
* crescent.c : hemisphere crescent/eclipse receiver dependent functions
*
*          Copyright (C) 2007-2020 by T.TAKASU, All rights reserved.
*          Copyright (C) 2007-2022 by Feng Xuebin, All rights reserved.
*
* reference :
*     [1] Hemisphere GPS, Grescent Integrator's Manual, December, 2005
*     [2] Hemisphere GPS, GPS Technical Reference, Part No. 875-0175-000,
*         Rev.D1, 2008
*     [3] Hemisphere GPS, Hemisphere GPS Technical Reference Manual, v4.0,
*         June 30, 2020
*
* version : $Revision: 1.2 $ $Date: 2008/07/14 00:05:05 $
* history : 2008/05/21 1.0  new
*           2009/04/01 1.1  support sbas, set 0 to L2 observables
*                           fix bug on getting doppler observables
*           2009/10/19 1.2  support eclipse (message bin 76)
*           2009/10/24 1.3  ignore vaild phase flag
*           2011/05/27 1.4  add -EPHALL option
*                           fix problem with ARM compiler
*           2011/07/01 1.5  suppress warning
*           2013/02/23 1.6  fix memory access violation problem on arm
*           2014/05/13 1.7  support bin65 and bin66
*                           add receiver option -TTCORR
*           2014/06/21 1.8  move decode_glostr() to rcvraw.c
*           2017/04/11 1.9  (char *) . (signed char *)
*           2020/11/30 1.10 use integer type in stdint.h
*                           use Sat2Freq() instead of lam_carr()
*                           udpate reference [3]
*           2022/09/19      rewrite the file with golang
*-----------------------------------------------------------------------------*/

const (
	CRESSYNC = "$BIN" /* hemis bin sync code */

	ID_CRESPOS    = 1  /* hemis msg id: bin 1 position/velocity */
	ID_CRESGLOEPH = 65 /* hemis msg id: bin 65 glonass ephemeris */
	ID_CRESGLORAW = 66 /* hemis msg id: bin 66 glonass L1/L2 phase and code */
	ID_CRESRAW2   = 76 /* hemis msg id: bin 76 dual-freq raw */
	ID_CRESWAAS   = 80 /* hemis msg id: bin 80 waas messages */
	ID_CRESIONUTC = 94 /* hemis msg id: bin 94 ion/utc parameters */
	ID_CRESEPH    = 95 /* hemis msg id: bin 95 raw ephemeris */
	ID_CRESRAW    = 96 /* hemis msg id: bin 96 raw phase and code */

	SNR2CN0_L1 = 30.0 /* hemis snr to c/n0 offset (db) L1 */
	SNR2CN0_L2 = 30.0 /* hemis snr to c/n0 offset (db) L2 */
)

/* get fields (little-endian) ------------------------------------------------*/

func U2L(p []uint8) uint16 { return binary.LittleEndian.Uint16(p[:]) }
func U4L(p []uint8) uint32 { return binary.LittleEndian.Uint32(p[:]) }
func I2L(p []uint8) int16 {
	var r int16
	buf := bytes.NewReader(p)
	binary.Read(buf, binary.LittleEndian, &r)
	return r
}
func I4L(p []uint8) int32 {
	var r int32
	buf := bytes.NewReader(p)
	binary.Read(buf, binary.LittleEndian, &r)
	return r
}
func R4L(p []uint8) float32 {
	var r float32
	buf := bytes.NewReader(p)
	binary.Read(buf, binary.LittleEndian, &r)
	return r
}
func R8L(p []uint8) float64 {
	var r float64
	buf := bytes.NewReader(p)
	binary.Read(buf, binary.LittleEndian, &r)
	return r
}

/* checksum ------------------------------------------------------------------*/
func chksum(buff []uint8, length int) int {
	var sum uint16

	for i := 8; i < length-4; i++ {
		sum += uint16(buff[i])
	}
	Trace(4, "checksum=%02X%02X %02X%02X:%02X%02X\n",
		sum>>8, sum&0xFF, buff[length-3], buff[length-4], buff[length-2], buff[length-1])
	if (sum>>8) == uint16(buff[length-3]) && (sum&0xFF) == uint16(buff[length-4]) &&
		buff[length-2] == 0x0D && buff[length-1] == 0x0A {
		return 1
	}
	return 0
}

/* decode bin 1 postion/velocity ---------------------------------------------*/
func decode_crespos(raw *Raw) int {
	var (
		ns, week, mode int
		tow, std       float64
		pos, vel       [3]float64
		tstr           string
	)
	const idx int = 8
	Trace(4, "decode_crespos: length=%d\n", raw.Len)

	if raw.Len != 64 {
		Trace(2, "crescent bin 1 message length error: length=%d\n", raw.Len)
		return -1
	}
	ns = int(U1(raw.Buff[idx+1:]))
	week = int(U2L(raw.Buff[idx+2:]))
	tow = R8L(raw.Buff[idx+4:])
	pos[0] = R8L(raw.Buff[idx+12:])
	pos[1] = R8L(raw.Buff[idx+20:])
	pos[2] = float64(R4L(raw.Buff[idx+28:]))
	vel[0] = float64(R4L(raw.Buff[idx+32:]))
	vel[1] = float64(R4L(raw.Buff[idx+36:]))
	vel[2] = float64(R4L(raw.Buff[idx+40:]))
	std = float64(R4L(raw.Buff[idx+44:]))
	mode = int(U2L(raw.Buff[idx+48:]))
	Time2Str(GpsT2Time(week, tow), &tstr, 3)

	var m int
	switch {
	case mode == 6:
		m = 1
	case mode > 4:
		m = 2
	case mode > 1:
		m = 5
	default:
		m = 0
	}
	Trace(3, "$BIN1 %s %13.9f %14.9f %10.4f %4d %3d %.3f\n", tstr, pos[0], pos[1],
		pos[2], m, ns, std)
	return 0
}

/* decode bin 96 raw phase and code ------------------------------------------*/
func decode_cresraw(raw *Raw) int {
	var (
		time                                     Gtime
		tow, tows, toff, cp, pr, dop, snr        float64
		i, j, n, prn, sat, week, word2, lli, sys int
		word1, sn, sc                            uint32
		idx                                      int = 8
	)
	freq := FREQ1
	Trace(4, "decode_cresraw: length=%d\n", raw.Len)

	if raw.Len != 312 {
		Trace(2, "crescent bin 96 message length error: length=%d\n", raw.Len)
		return -1
	}
	week = int(U2L(raw.Buff[idx+2:]))
	tow = R8L(raw.Buff[idx+4:])
	tows = math.Floor(tow*1000.0+0.5) / 1000.0 /* round by 1ms */
	time = GpsT2Time(week, tows)

	/* time tag offset correction */
	if strings.Contains(raw.Opt, "-TTCORR") {
		toff = CLIGHT * (tows - tow)
	}

	for i, n, idx = 0, 0, idx+12; i < 12 && n < MAXOBS; i, idx = i+1, idx+24 {
		word1 = U4L(raw.Buff[idx:])
		word2 = int(I4(raw.Buff[idx+4:]))
		if prn = int(word1 & 0xFF); prn == 0 {
			continue
		} /* if 0, no data */
		if prn <= MAXPRNGPS {
			sys = SYS_GPS
		} else {
			sys = SYS_SBS
		}
		if sat = SatNo(sys, prn); sat == 0 {
			Trace(2, "creasent bin 96 satellite number error: prn=%d\n", prn)
			continue
		}
		pr = R8L(raw.Buff[idx+8:]) - toff
		cp = R8L(raw.Buff[idx+16:]) - toff
		if word2&1 == 0 {
			cp = 0.0
		} /* invalid phase */
		sn = (word1 >> 8) & 0xFF
		snr = 0.0
		if sn != 0 {
			snr = 10.0*math.Log10(0.8192*float64(sn)) + SNR2CN0_L1
		}

		sc = uint32(word1 >> 24)
		if raw.Time.Time != 0 {
			if (uint8(sc) - uint8(raw.LockTime[sat-1][0])) > 0 {
				lli = 1
			} else {
				lli = 0
			}

		}
		raw.LockTime[sat-1][0] = float64(sc)
		dop = float64(word2) / 16.0 / 4096.0

		var data ObsD
		data.Time = time
		data.Sat = sat
		data.P[0] = pr
		data.L[0] = cp * freq / CLIGHT
		data.D[0] = -float64(dop * freq / CLIGHT)
		data.SNR[0] = uint16(snr/SNR_UNIT + 0.5)
		data.LLI[0] = uint8(lli)
		data.Code[0] = CODE_L1C

		for j = 1; j < NFREQ; j++ {
			data.L[j], data.P[j] = 0.0, 0.0
			data.D[j] = 0.0
			data.SNR[j], data.LLI[j] = 0, 0
			data.Code[j] = CODE_NONE
		}
		raw.ObsData.Data[n] = data
		n++
	}
	raw.Time = time

	return 1
}

/* decode bin 76 dual-freq raw phase and code --------------------------------*/
func decode_cresraw2(raw *Raw) int {
	var (
		time                        Gtime
		pr1, tow, tows, toff        float64
		cp, pr, dop, snr            [2]float64
		freq                        [2]float64 = [2]float64{FREQ1, FREQ2}
		i, j, n, prn, sat, week     int
		lli                         [2]int
		word1, word2, word3, sc, sn uint32
		idx                         int = 8
	)

	Trace(4, "decode_cresraw2: length=%d\n", raw.Len)

	if raw.Len != 460 {
		Trace(2, "crescent bin 76 message length error: length=%d\n", raw.Len)
		return -1
	}
	tow = R8L(raw.Buff[idx:])
	week = int(U2L(raw.Buff[idx+8:]))
	tows = math.Floor(tow*1000.0+0.5) / 1000.0 /* round by 1ms */
	time = GpsT2Time(week, tows)

	/* time tag offset correction */
	if strings.Contains(raw.Opt, "-TTCORR") {
		toff = CLIGHT * (tows - tow)
	}
	for i, idx = 0, idx+16; i < 15 && n < MAXOBS; i++ {
		word1 = U4L(raw.Buff[idx+324+4*i:]) /* L1CACodeMSBsPRN */
		if prn = int(word1 & 0xFF); prn == 0 {
			continue
		} /* if 0, no data */
		sys := SYS_SBS
		if prn <= MAXPRNGPS {
			sys = SYS_GPS
		}
		if sat = SatNo(sys, prn); sat == 9 {
			Trace(2, "creasent bin 76 satellite number error: prn=%d\n", prn)
			continue
		}
		pr1 = float64(word1>>13) * 256.0 /* upper 19bit of L1CA pseudorange */

		word1 = U4L(raw.Buff[idx+144+12*i:]) /* L1CASatObs */
		word2 = U4L(raw.Buff[idx+148+12*i:])
		word3 = U4L(raw.Buff[idx+152+12*i:])
		if sn = word1 & 0xFFF; sn == 0 {
			snr[0] = 0.0
		} else {
			snr[0] = math.Log10(0.1024*float64(sn)) + SNR2CN0_L1
		}
		sc = uint32(word1 >> 24)
		if raw.Time.Time != 0 {
			if uint8(sc)-uint8(raw.LockTime[sat-1][0]) > 0 {
				lli[0] = 1

			} else {
				lli[0] = 0
			}
		} else {
			lli[0] = 0
		}
		if (word1>>12)&7 > 1 {
			lli[0] |= 2
		} else {
			lli[0] |= 0
		}
		raw.LockTime[sat-1][0] = float64(sc)
		dop[0] = float64((word2>>1)&0x7FFFFF) / 512.0
		if (word2>>24)&1 > 1 {
			dop[0] = -dop[0]
		}
		pr[0] = pr1 + float64(word3&0xFFFF)/256.0
		cp[0] = math.Floor(pr[0]*freq[0]/CLIGHT/8192.0) * 8192.0
		cp[0] += float64((word2&0xFE000000)+((word3&0xFFFF0000)>>7)) / 524288.0
		if cp[0]-pr[0]*freq[0]/CLIGHT < -4096.0 {
			cp[0] += 8192.0
		} else if cp[0]-pr[0]*freq[0]/CLIGHT > 4096.0 {
			cp[0] -= 8192.0
		}

		if i < 12 {
			word1 = U4L(raw.Buff[idx+12*i:]) /* L2PSatObs */
			word2 = U4L(raw.Buff[idx+4+12*i:])
			word3 = U4L(raw.Buff[idx+8+12*i:])
			if sn = word1 & 0xFFF; sn == 0 {
				snr[1] = 0.0
			} else {
				snr[1] = 10.0*math.Log10(0.1164*float64(sn)) + SNR2CN0_L2
			}
			sc = uint32(word1 >> 24)
			if raw.Time.Time == 0 {
				if (uint8(sc) - uint8(raw.LockTime[sat-1][1])) > 0 {
					lli[1] = 1
				} else {
					lli[1] = 0
				}
			} else {
				lli[1] = 0
			}
			if ((word1 >> 12) & 7) > 0 {
				lli[1] |= 2

			} else {
				lli[1] |= 0

			}
			raw.LockTime[sat-1][1] = float64(sc)
			dop[1] = float64((word2>>1)&0x7FFFFF) / 512.0
			if ((word2 >> 24) & 1) > 0 {
				dop[1] = -dop[1]
			}
			pr[1] = float64(word3&0xFFFF) / 256.0
			if pr[1] != 0.0 {
				pr[1] += pr1
				if pr[1]-pr[0] < -128.0 {
					pr[1] += 256.0
				} else if pr[1]-pr[0] > 128.0 {
					pr[1] -= 256.0
				}
				cp[1] = math.Floor(pr[1]*freq[1]/CLIGHT/8192.0) * 8192.0
				cp[1] += float64((word2&0xFE000000)+((word3&0xFFFF0000)>>7)) / 524288.0
				if cp[1]-pr[1]*freq[1]/CLIGHT < -4096.0 {
					cp[1] += 8192.0
				} else if cp[1]-pr[1]*freq[1]/CLIGHT > 4096.0 {
					cp[1] -= 8192.0
				}
			} else {
				cp[1] = 0.0
			}
		}
		var data ObsD
		data.Time = time
		data.Sat = sat
		for j = 0; j < NFREQ; j++ {
			if j == 0 || (j == 1 && i < 12) {
				if pr[j] == 0.0 {
					data.P[j] = 0.0

				} else {
					data.P[j] = pr[j] - toff
				}
				if cp[j] == 0.0 {
					data.L[j] = 0.0

				} else {
					data.L[j] = cp[j] - toff*freq[j]/CLIGHT

				}
				data.D[j] = -float64(dop[j])
				data.SNR[j] = uint16(snr[j]/SNR_UNIT + 0.5)
				data.LLI[j] = uint8(lli[j])
				if j == 0 {
					data.Code[j] = CODE_L1C

				} else {
					data.Code[j] = CODE_L2P
				}
			} else {
				data.L[j], data.P[j] = 0.0, 0.0
				data.D[j] = 0.0
				data.SNR[j], data.LLI[j] = 0, 0
				data.Code[j] = CODE_NONE
			}
		}
		raw.ObsData.Data[n] = data
		n++
	}
	raw.Time = time
	if strings.Contains(raw.Opt, "-ENAGLO") {
		return 0
	} /* glonass follows */
	return 1
}

/* decode bin 95 ephemeris ---------------------------------------------------*/
func decode_creseph(raw *Raw) int {
	var (
		eph               Eph
		word              uint32
		i, j, k, prn, sat int
		buff              [90]uint8
	)
	const idx = 8
	Trace(4, "decode_creseph: length=%d\n", raw.Len)

	if raw.Len != 140 {
		Trace(2, "crescent bin 95 message length error: length=%d\n", raw.Len)
		return -1
	}
	prn = int(U2L(raw.Buff[idx:]))
	if sat = SatNo(SYS_GPS, prn); sat == 0 {
		Trace(2, "crescent bin 95 satellite number error: prn=%d\n", prn)
		return -1
	}
	for i = 0; i < 3; i++ {
		for j = 0; j < 10; j++ {
			word = U4L(raw.Buff[idx+8+i*40+j*4:]) >> 6
			for k = 0; k < 3; k++ {
				buff[i*30+j*3+k] = uint8((word >> (8 * (2 - k))) & 0xFF)
			}
		}
	}
	if DecodeFrame(buff[:], &eph, nil, nil, nil) == 0 {
		Trace(2, "crescent bin 95 navigation frame error: prn=%d\n", prn)
		return -1
	}
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

/* decode bin 94 ion/utc parameters ------------------------------------------*/
func decode_cresionutc(raw *Raw) int {
	var (
		i int
	)
	const idx = 8
	Trace(4, "decode_cresionutc: length=%d\n", raw.Len)

	if raw.Len != 108 {
		Trace(2, "crescent bin 94 message length error: length=%d\n", raw.Len)
		return -1
	}
	for i = 0; i < 8; i++ {
		raw.NavData.Ion_gps[i] = R8L(raw.Buff[idx+i*8:])
	}
	raw.NavData.Utc_gps[0] = R8L(raw.Buff[idx+64:])
	raw.NavData.Utc_gps[1] = R8L(raw.Buff[idx+72:])
	raw.NavData.Utc_gps[2] = float64(U4L(raw.Buff[idx+80:]))
	raw.NavData.Utc_gps[3] = float64(U2L(raw.Buff[idx+84:]))
	raw.NavData.Utc_gps[4] = float64(I2L(raw.Buff[idx+90:]))
	return 9
}

/* decode bin 80 waas messages -----------------------------------------------*/
func decode_creswaas(raw *Raw) int {
	var (
		tow          float64
		word         uint32
		i, j, k, prn int
	)
	const idx = 8
	Trace(4, "decode_creswaas: length=%d\n", raw.Len)

	if raw.Len != 52 {
		Trace(2, "creasent bin 80 message length error: length=%d\n", raw.Len)
		return -1
	}
	prn = int(U2L(raw.Buff[idx:]))
	if prn < MINPRNSBS || MAXPRNSBS < prn {
		Trace(2, "creasent bin 80 satellite number error: prn=%d\n", prn)
		return -1
	}
	raw.Sbsmsg.Prn = uint8(prn)
	raw.Sbsmsg.Tow = int(U4L(raw.Buff[idx+4:]))
	tow = Time2GpsT(raw.Time, &raw.Sbsmsg.Week)
	if float64(raw.Sbsmsg.Tow) < tow-302400.0 {
		raw.Sbsmsg.Week++
	} else if float64(raw.Sbsmsg.Tow) > tow+302400.0 {
		raw.Sbsmsg.Week--
	}

	for i, k = 0, 0; i < 8 && k < 29; i++ {
		word = U4L(raw.Buff[idx+8+i*4:])
		for j = 0; j < 4 && k < 29; j++ {
			raw.Sbsmsg.Msg[k] = uint8(word >> (3 - j) * 8)
			k++
		}
	}
	raw.Sbsmsg.Msg[28] &= 0xC0
	return 3
}

/* decode bin 66 glonass L1/L2 code and carrier phase ------------------------*/
func decode_cresgloraw(raw *Raw) int {
	var (
		time                        Gtime
		tow, tows, toff, pr1        float64
		cp, pr, dop, snr, freq      [2]float64
		i, j, n, prn, sat, week     int
		lli                         [2]int
		idx                         int = 8
		word1, word2, word3, sc, sn uint32
	)
	Trace(4, "decode_cregloraw: length=%d\n", raw.Len)

	if !strings.Contains(raw.Opt, "-ENAGLO") {
		return 0
	}

	if raw.Len != 364 {
		Trace(2, "crescent bin 66 message length error: length=%d\n", raw.Len)
		return -1
	}
	tow = R8L(raw.Buff[idx:])
	week = int(U2L(raw.Buff[idx+8:]))
	tows = math.Floor(tow*1000.0+0.5) / 1000.0 /* round by 1ms */
	time = GpsT2Time(week, tows)

	/* time tag offset correction */
	if strings.Contains(raw.Opt, "-TTCORR") {
		toff = CLIGHT * (tows - tow)
	}
	for i, idx = 0, idx+16; i < 12 && n < MAXOBS; i++ {
		word1 = U4L(raw.Buff[idx+288+4*i:]) /* L1CACodeMSBsSlot */
		if prn = int(word1 & 0xFF); prn == 0 {
			continue
		} /* if 0, no data */
		if sat = SatNo(SYS_GLO, prn); sat == 0 {
			Trace(2, "creasent bin 66 satellite number error: prn=%d\n", prn)
			continue
		}
		pr1 = float64(word1>>13) * 256.0 /* upper 19bit of L1CA pseudorange */

		freq[0] = Sat2Freq(sat, CODE_L1C, &raw.NavData)
		freq[1] = Sat2Freq(sat, CODE_L2C, &raw.NavData)

		/* L1Obs */
		word1 = U4L(raw.Buff[idx+12*i:])
		word2 = U4L(raw.Buff[idx+4+12*i:])
		word3 = U4L(raw.Buff[idx+8+12*i:])
		sn = word1 & 0xFFF
		snr[0] = 0.0
		if sn != 0 {
			snr[0] = 10.0*math.Log10(0.1024*float64(sn)) + SNR2CN0_L1
		}
		sc = uint32(word1 >> 24)
		if raw.Time.Time != 0 {
			if int(uint8(sc)-uint8(raw.LockTime[sat-1][0])) > 0 {
				lli[0] = 1
			} else {
				lli[0] = 0
			}
		} else {
			lli[0] = 0
		}
		if ((word1 >> 12) & 7) > 1 {
			lli[0] |= 2
		} else {
			lli[0] |= 0
		}
		raw.LockTime[sat-1][0] = float64(sc)
		dop[0] = float64((word2>>1)&0x7FFFFF) / 512.0
		if (word2>>24)&1 > 0 {
			dop[0] = -dop[0]
		}
		pr[0] = pr1 + float64(word3&0xFFFF)/256.0
		cp[0] = math.Floor(pr[0]*freq[0]/CLIGHT/8192.0) * 8192.0
		cp[0] += float64((word2&0xFE000000)+((word3&0xFFFF0000)>>7)) / 524288.0
		if cp[0]-pr[0]*freq[0]/CLIGHT < -4096.0 {
			cp[0] += 8192.0
		} else if cp[0]-pr[0]*freq[0]/CLIGHT > 4096.0 {
			cp[0] -= 8192.0
		}

		/* L2Obs */
		word1 = U4L(raw.Buff[idx+144+12*i:])
		word2 = U4L(raw.Buff[idx+148+12*i:])
		word3 = U4L(raw.Buff[idx+152+12*i:])
		sn = word1 & 0xFFF
		snr[1] = 0.0
		if sn != 0 {
			snr[1] = 10.0*math.Log10(0.1164*float64(sn)) + SNR2CN0_L2
		}
		sc = uint32(word1 >> 24)
		if raw.Time.Time == 0 {
			if uint8(sc)-uint8(raw.LockTime[sat-1][1]) > 0 {
				lli[1] = 1
			} else {
				lli[1] = 0
			}
		} else {
			lli[1] = 0
		}
		if ((word1 >> 12) & 7) > 1 {
			lli[1] |= 2
		} else {
			lli[1] |= 0
		}
		raw.LockTime[sat-1][1] = float64(sc)
		dop[1] = float64((word2>>1)&0x7FFFFF) / 512.0
		if (word2>>24)&1 > 0 {
			dop[1] = -dop[1]
		}
		pr[1] = float64(word3&0xFFFF) / 256.0
		if pr[1] != 0.0 {
			pr[1] += pr1
			if pr[1]-pr[0] < -128.0 {
				pr[1] += 256.0
			} else if pr[1]-pr[0] > 128.0 {
				pr[1] -= 256.0
			}
			cp[1] = math.Floor(pr[1]*freq[1]/CLIGHT/8192.0) * 8192.0
			cp[1] += float64((word2&0xFE000000)+((word3&0xFFFF0000)>>7)) / 524288.0
			if cp[1]-pr[1]*freq[1]/CLIGHT < -4096.0 {
				cp[1] += 8192.0
			} else if cp[1]-pr[1]*freq[1]/CLIGHT > 4096.0 {
				cp[1] -= 8192.0
			}
		}
		var data ObsD
		data.Time = time
		data.Sat = sat
		for j = 0; j < NFREQ; j++ {
			if j == 0 || (j == 1 && i < 12) {
				if pr[j] == 0.0 {
					data.P[j] = 0.0
				} else {
					data.P[j] = pr[j] - toff

				}
				if cp[j] == 0.0 {
					data.L[j] = 0.0
				} else {
					data.L[j] = cp[j] - toff*freq[j]/CLIGHT

				}
				data.D[j] = -float64(dop[j])
				data.SNR[j] = uint16(snr[j]/SNR_UNIT + 0.5)
				data.LLI[j] = uint8(lli[j])
				if j == 0 {
					data.Code[j] = CODE_L1C
				} else {
					data.Code[j] = CODE_L2P

				}
			} else {
				data.L[j], data.P[j] = 0.0, 0.0
				data.D[j] = 0.0
				data.SNR[j], data.LLI[j] = 0, 0
				data.Code[j] = CODE_NONE
			}
		}
		raw.ObsData.Data[n] = data
		n++
	}
	raw.Time = time
	return 1
}

/* decode bin 65 glonass ephemeris -------------------------------------------*/
func decode_cresgloeph(raw *Raw) int {
	var (
		geph                       GEph
		str                        [12]byte
		i, j, k, sat, prn, frq, no int
	)
	var idx int = 8
	Trace(4, "decode_cregloeph: length=%d\n", raw.Len)

	if !strings.Contains(raw.Opt, "-ENAGLO") {
		return 0
	}

	prn = int(U1(raw.Buff[idx:]))
	idx += 1
	frq = int(U1(raw.Buff[idx:])) - 8
	idx += 1 + 2
	_ = U4L(raw.Buff[idx:])
	idx += 4

	if sat = SatNo(SYS_GLO, prn); sat == 0 {
		Trace(2, "creasent bin 65 satellite number error: prn=%d\n", prn)
		return -1
	}
	for i = 0; i < 5; i++ {
		for j = 0; j < 3; j++ {
			for k = 3; k >= 0; k-- {
				str[k+j*4] = U1(raw.Buff[idx:])
				idx++
			}
		}
		if no = int(GetBitU(str[:], 1, 4)); no != i+1 {
			Trace(2, "creasent bin 65 string no error: sat=%2d no=%d %d\n", sat,
				i+1, no)
			return -1
		}
		copy(raw.SubFrm[sat-1][10*i:], str[:10])
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
	raw.EphSet = 0
	return 2
}

/* decode crescent raw message -----------------------------------------------*/
func decode_cres(raw *Raw) int {
	ctype := U2L(raw.Buff[4:])

	Trace(3, "decode_cres: type=%2d length=%d\n", ctype, raw.Len)

	if chksum(raw.Buff[:], raw.Len) == 0 {
		Trace(2, "crescent checksum error: type=%2d length=%d\n", ctype, raw.Len)
		return -1
	}
	if raw.OutType > 0 {
		copy(raw.MsgType[:], []byte(fmt.Sprintf("HEMIS %2d (%4d):", ctype, raw.Len)))
	}
	switch ctype {
	case ID_CRESPOS:
		return decode_crespos(raw)
	case ID_CRESRAW:
		return decode_cresraw(raw)
	case ID_CRESRAW2:
		return decode_cresraw2(raw)
	case ID_CRESEPH:
		return decode_creseph(raw)
	case ID_CRESWAAS:
		return decode_creswaas(raw)
	case ID_CRESIONUTC:
		return decode_cresionutc(raw)
	case ID_CRESGLORAW:
		return decode_cresgloraw(raw)
	case ID_CRESGLOEPH:
		return decode_cresgloeph(raw)
	}
	return 0
}

/* sync code -----------------------------------------------------------------*/
func sync_cres(buff []uint8, data uint8) int {
	buff[0] = buff[1]
	buff[1] = buff[2]
	buff[2] = buff[3]
	buff[3] = data
	if buff[0] == CRESSYNC[0] && buff[1] == CRESSYNC[1] &&
		buff[2] == CRESSYNC[2] && buff[3] == CRESSYNC[3] {
		return 1
	}
	return 0
}

/* input cresent raw message ---------------------------------------------------
* input next crescent raw message from stream
* args   : raw_t *raw       IO  receiver raw data control struct
*          uint8_t data     I   stream data (1 byte)
* return : status (-1: error message, 0: no message, 1: input observation data,
*                  2: input ephemeris, 3: input sbas message,
*                  9: input ion/utc parameter)
*
* notes  : to specify input options, set raw.Opt to the following option
*          strings separated by spaces.
*
*          -EPHALL      : input all ephemerides
*          -TTCORR      : time-tag offset correction
*          -ENAGLO      : enable glonass messages
*
*-----------------------------------------------------------------------------*/
func Input_cres(raw *Raw, data uint8) int {
	Trace(5, "input_cres: data=%02x\n", data)

	/* synchronize frame */
	if raw.NumByte == 0 {
		if sync_cres(raw.Buff[:], data) == 0 {
			return 0
		}
		raw.NumByte = 4
		return 0
	}
	raw.Buff[raw.NumByte] = data
	raw.NumByte++

	if raw.NumByte == 8 {
		if raw.Len = int(U2L(raw.Buff[6:])) + 12; raw.Len > MAXRAWLEN {
			Trace(2, "cresent length error: length=%d\n", raw.Len)
			raw.NumByte = 0
			return -1
		}
	}
	if raw.NumByte < 8 || raw.NumByte < raw.Len {
		return 0
	}
	raw.NumByte = 0

	/* decode crescent raw message */
	return decode_cres(raw)
}

/* input crescent raw message from file ----------------------------------------
* input next crescent raw message from file
* args   : raw_t  *raw   IO     receiver raw data control struct
*          FILE   *fp    I      file pointer
* return : status(-2: end of file, -1...9: same as above)
*-----------------------------------------------------------------------------*/
func Input_cresf(raw *Raw, fp *os.File) int {
	var i int

	Trace(4, "input_cresf:\n")

	/* synchronize frame */
	var c [1]byte
	if raw.NumByte == 0 {
		for i = 0; ; i++ {
			_, err := fp.Read(c[:])
			if err == io.EOF {
				return -2
			}

			if sync_cres(raw.Buff[:], c[0]) > 0 {
				break
			}
			if i >= 4096 {
				return 0
			}
		}
	}
	n, err := fp.Read(raw.Buff[4:8])
	if err == io.EOF || n < 4 {
		return -2
	}
	raw.NumByte = 8

	if raw.Len = int(U2L(raw.Buff[6:])) + 12; raw.Len > MAXRAWLEN {
		Trace(2, "crescent length error: length=%d\n", raw.Len)
		raw.NumByte = 0
		return -1
	}
	n, err = fp.Read(raw.Buff[8:raw.Len])
	if n < raw.Len+-8 {
		return -2
	}
	raw.NumByte = 0

	/* decode crescent raw message */
	return decode_cres(raw)
}
