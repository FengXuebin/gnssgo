/*------------------------------------------------------------------------------
* notvatel.c : NovAtel OEM7/OEM6/OEM5/OEM4/OEM3 receiver functions
*
*          Copyright (C) 2007-2020 by T.TAKASU, All rights reserved.
*          Copyright (C) 2007-2022 by Feng Xuebin, All rights reserved.
*
* reference :
*     [1] NovAtel, OM-20000094 Rev6 OEMV Family Firmware Reference Manual, 2008
*     [2] NovAtel, OM-20000053 Rev2 MiLLennium GPSCard Software Versions 4.503
*         and 4.52 Command Descriptions Manual, 2001
*     [3] NovAtel, OM-20000129 Rev2 OEM6 Family Firmware Reference Manual, 2011
*     [4] NovAtel, OM-20000127 Rev1 OEMStar Firmware Reference Manual, 2009
*     [5] NovAtel, OM-20000129 Rev6 OEM6 Family Firmware Reference Manual, 2014
*     [6] NovAtel, OM-20000169 v15C OEM7 Commands and Logs Reference Manual,
*         June 2020
*
* version : $Revision: 1.2 $ $Date: 2008/07/14 00:05:05 $
* history : 2007/10/08 1.0 new
*           2008/05/09 1.1 fix bug lli flag outage
*           2008/06/16 1.2 separate common functions to rcvcmn.c
*           2009/04/01 1.3 add prn number check for raw obs data
*           2009/04/10 1.4 refactored
*                          add oem3, oem4 rangeb support
*           2009/06/06 1.5 fix bug on numerical exception with illegal snr
*                          support oem3 regd message
*           2009/12/09 1.6 support oem4 gloephemerisb message
*                          invalid if parity unknown in GLONASS range
*                          fix bug of dopper polarity inversion for oem3 regd
*           2010/04/29 1.7 add tod field in geph_t
*           2011/05/27 1.8 support RAWALM for oem4/v
*                          add almanac decoding
*                          add -EPHALL option
*                          fix problem on ARM compiler
*           2012/05/02 1.9 support OEM6,L5,QZSS
*           2012/10/18 1.10 change obs codes
*                           support Galileo
*                           support rawsbasframeb,galephemerisb,galalmanacb,
*                           galclockb,galionob
*           2012/11/08 1.11 support galfnavrawpageb, galinavrawword
*           2012/11/19 1.12 fix bug on decodeing rangeb
*           2013/02/23 1.13 fix memory access violation problem on arm
*           2013/03/28 1.14 fix invalid phase if glonass wavelen unavailable
*           2013/06/02 1.15 fix bug on reading galephemrisb,galalmanacb,
*                           galclockb,galionob
*                           fix bug on decoding rawwaasframeb for qzss-saif
*           2014/05/24 1.16 support beidou
*           2014/07/01 1.17 fix problem on decoding of bdsephemerisb
*                           fix bug on beidou tracking codes
*           2014/10/20 1.11 fix bug on receiver option -GL*,-RL*,-EL*
*           2016/01/28 1.12 precede I/NAV for galileo ephemeris
*                           add option -GALINAV and -GALFNAV
*           2016/07/31 1.13 add week number check to decode oem4 messages
*           2017/04/11 1.14 (char *) . (signed char *)
*                           improve unchange-test of beidou ephemeris
*           2017/06/15 1.15 add output half-cycle-ambiguity status to LLI
*                           improve slip-detection by lock-time rollback
*           2018/10/10 1.16 fix problem on data souce for galileo ephemeris
*                           output L2W instead of L2D for L2Pcodeless
*                           test toc difference to output beidou ephemeris
*           2019/05/10 1.17 save galileo E5b data to obs index 2
*           2020/11/30 1.18 support OEM7 receiver (ref [6])
*                           support NavIC/IRNSS
*                           support GPS/QZS L1C, GLO L3, GAL E6, QZS L6, BDS B3,
*                            B1C, B2a, B2b
*                           support message NAVICEPHEMERISB
*                           support QZS L1S in RANGEB and RANGECMPB
*                           no support message GALALMANACB
*                           add receiver option -GL1L,-GL2S,-GL2P,-EL6B,-JL1L,
*                            -JL1Z,-CL1P,-CL7D,-GLOBIAS=bias
*                           delete receiver option -GL1P,-GL2X,EL2C
*                           fix bug on reading SVH in GLOEPHEMERISB
*                           add reading of dtaun field in GLOEPHEMERISB
*                           output GAL I/NAV or F/NAV to seperated ephem sets
*                           use API Sat2Freq() to get carrier-frequency
*                           use API Code2Idx() to get freq-index
*                           use integer types in stdint.h
*           2022/09/21 1.19 rewrite the file with golang
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
	OEM4SYNC1 = 0xAA /* oem7/6/4 message start sync code 1 */
	OEM4SYNC2 = 0x44 /* oem7/6/4 message start sync code 2 */
	OEM4SYNC3 = 0x12 /* oem7/6/4 message start sync code 3 */
	OEM3SYNC1 = 0xAA /* oem3 message start sync code 1 */
	OEM3SYNC2 = 0x44 /* oem3 message start sync code 2 */
	OEM3SYNC3 = 0x11 /* oem3 message start sync code 3 */
	OEM4HLEN  = 28   /* oem7/6/4 message header length (bytes) */
	OEM3HLEN  = 12   /* oem3 message header length (bytes) */

	/* message IDs */
	ID_RANGECMP        = 140  /* oem7/6/4 range compressed */
	ID_RANGE           = 43   /* oem7/6/4 range measurement */
	ID_RAWEPHEM        = 41   /* oem7/6/4 raw ephemeris */
	ID_IONUTC          = 8    /* oem7/6/4 iono and utc data */
	ID_RAWWAASFRAME    = 287  /* oem7/6/4 raw waas frame */
	ID_RAWSBASFRAME    = 973  /* oem7/6 raw sbas frame */
	ID_GLOEPHEMERIS    = 723  /* oem7/6/4 glonass ephemeris */
	ID_GALEPHEMERIS    = 1122 /* oem7/6 decoded galileo ephemeris */
	ID_GALIONO         = 1127 /* oem7/6 decoded galileo iono corrections */
	ID_GALCLOCK        = 1121 /* oem7/6 galileo clock information */
	ID_QZSSRAWEPHEM    = 1331 /* oem7/6 qzss raw ephemeris */
	ID_QZSSRAWSUBFRAME = 1330 /* oem7/6 qzss raw subframe */
	ID_QZSSIONUTC      = 1347 /* oem7/6 qzss ion/utc parameters */
	ID_BDSEPHEMERIS    = 1696 /* oem7/6 decoded bds ephemeris */
	ID_NAVICEPHEMERIS  = 2123 /* oem7 decoded navic ephemeris */

	ID_ALMB = 18 /* oem3 decoded almanac */
	ID_IONB = 16 /* oem3 iono parameters */
	ID_UTCB = 17 /* oem3 utc parameters */
	ID_FRMB = 54 /* oem3 framed raw navigation data */
	ID_RALB = 15 /* oem3 raw almanac */
	ID_RASB = 66 /* oem3 raw almanac set */
	ID_REPB = 14 /* oem3 raw ephemeris */
	ID_RGEB = 32 /* oem3 range measurement */
	ID_RGED = 65 /* oem3 range compressed */

	WL1       = 0.1902936727984
	WL2       = 0.2442102134246
	MAXVAL    = 8388608.0
	OFF_FRQNO = -7 /* F/W ver.3.620 */
)

/* get fields (little-endian) ------------------------------------------------*/
// declared in crescent.go. comment by fxb
// #define U1(raw.Buff[idx:]) (*((uint8_t *)(raw.Buff[idx:])))
// #define I1(raw.Buff[idx:]) (*((int8_t  *)(raw.Buff[idx:])))
// static uint16_t U2L(uint8_t *raw.Buff[idx:]) {uint16_t u; memcpy(&u,raw.Buff[idx:],2); return u;}
// static uint32_t U4L(uint8_t *raw.Buff[idx:]) {uint32_t u; memcpy(&u,raw.Buff[idx:],4); return u;}
// static int32_t  I4L(uint8_t *raw.Buff[idx:]) {int32_t  i; memcpy(&i,raw.Buff[idx:],4); return i;}
// static float    R4L(uint8_t *raw.Buff[idx:]) {float    r; memcpy(&r,raw.Buff[idx:],4); return r;}
// static double   R8L(uint8_t *raw.Buff[idx:]) {double   r; memcpy(&r,raw.Buff[idx:],8); return r;}

/* extend sign ---------------------------------------------------------------*/
func exsign(v uint32, bits int) int32 {
	var d uint32 = 0
	if v&(1<<(bits-1)) > 0 {
		return int32(v | ((^d) << bits))
	}
	return int32(v)
}

// /* checksum ------------------------------------------------------------------*/
// the func has implemented in binex.go named: csum8   comment by fxb
// static uint8_t chksum(const uint8_t *buff, int len)
// {
//     uint8_t sum=0;
//     int i;
//     for (i=0;i<len;i++) sum^=buff[i];
//     return sum;
// }
// /* adjust weekly rollover of GPS time ----------------------------------------*/
// the func has implemented in binex.go named: adjweek   comment by fxb
// static gtime_t adjweek(time Gtime, double tow)
// {
//     double tow_p;
//     int week;
//     tow_p=Time2GpsT(time,&week);
//     if      (tow<tow_p-302400.0) tow+=604800.0;
//     else if (tow>tow_p+302400.0) tow-=604800.0;
//     return GpsT2Time(week,tow);
// }
/* UTC 8-bit week . full week -----------------------------------------------*/
// the func has implemented in javad.go named: adj_utcweek   comment by fxb
// func adj_utcweek(time Gtime,   utc[]float64){
//     var week int;

//     Time2GpsT(time,&week);
//     utc[3]+=week/256*256;
//     if      (utc[3]<week-127) {utc[3]+=256.0;
//     }else if (utc[3]>week+127){ utc[3]-=256.0;}
//     utc[5]+=utc[3]/256*256;
//     if      (utc[5]<utc[3]-127) {utc[5]+=256.0;
//     }else if (utc[5]>utc[3]+127) {utc[5]-=256.0;}
// }
/* get observation data index ------------------------------------------------*/
func obsindex(raw *Raw, time Gtime, sat int) int {
	var i, j int

	if raw.ObsData.n >= MAXOBS {
		return -1
	}
	for i = 0; i < raw.ObsData.n; i++ {
		if raw.ObsData.Data[i].Sat == sat {
			return i
		}
	}
	raw.ObsData.Data[i].Time = time
	raw.ObsData.Data[i].Sat = sat
	for j = 0; j < NFREQ+NEXOBS; j++ {
		raw.ObsData.Data[i].L[j], raw.ObsData.Data[i].P[j] = 0.0, 0.0
		raw.ObsData.Data[i].D[j] = 0.0
		raw.ObsData.Data[i].SNR[j], raw.ObsData.Data[i].LLI[j] = 0.0, 0.0
		raw.ObsData.Data[i].Code[j] = CODE_NONE
	}
	raw.ObsData.n++
	return i
}

/* URA value (m) to URA index ------------------------------------------------*/
// the func has implemented in binex.go named: uraindex   comment by fxb
// static int uraindex(double value)
// {
//     static const double ura_eph[]={
//         2.4,3.4,4.85,6.85,9.65,13.65,24.0,48.0,96.0,192.0,384.0,768.0,1536.0,
//         3072.0,6144.0,0.0
//     };
//     int i;
//     for (i=0;i<15;i++) if (ura_eph[i]>=value) break;
//     return i;
// }
/* signal type to obs code ---------------------------------------------------*/
func sig2code(sys, sigtype int) int {
	switch sys {
	case SYS_GPS:
		{
			switch sigtype {
			case 0:
				return CODE_L1C /* L1C/A */
			case 5:
				return CODE_L2P /* L2P    (OEM7) */
			case 9:
				return CODE_L2W /* L2P(Y),semi-codeless */
			case 14:
				return CODE_L5Q /* L5Q    (OEM6) */
			case 16:
				return CODE_L1L /* L1C(P) (OEM7) */
			case 17:
				return CODE_L2S /* L2C(M) (OEM7) */
			}
		}
	case SYS_GLO:
		{
			switch sigtype {
			case 0:
				return CODE_L1C /* L1C/A */
			case 1:
				return CODE_L2C /* L2C/A (OEM6) */
			case 5:
				return CODE_L2P /* L2P */
			case 6:
				return CODE_L3Q /* L3Q   (OEM7) */
			}
		}
	case SYS_GAL:
		{
			switch sigtype {
			case 2:
				return CODE_L1C /* E1C  (OEM6) */
			case 6:
				return CODE_L6B /* E6B  (OEM7) */
			case 7:
				return CODE_L6C /* E6C  (OEM7) */
			case 12:
				return CODE_L5Q /* E5aQ (OEM6) */
			case 17:
				return CODE_L7Q /* E5bQ (OEM6) */
			case 20:
				return CODE_L8Q /* AltBOCQ (OEM6) */
			}
		}
	case SYS_QZS:
		{
			switch sigtype {
			case 0:
				return CODE_L1C /* L1C/A */
			case 14:
				return CODE_L5Q /* L5Q    (OEM6) */
			case 16:
				return CODE_L1L /* L1C(P) (OEM7) */
			case 17:
				return CODE_L2S /* L2C(M) (OEM7) */
			case 27:
				return CODE_L6L /* L6P    (OEM7) */
			}
		}
	case SYS_CMP:
		{
			switch sigtype {
			case 0:
				return CODE_L2I /* B1I with D1 (OEM6) */
			case 1:
				return CODE_L7I /* B2I with D1 (OEM6) */
			case 2:
				return CODE_L6I /* B3I with D1 (OEM7) */
			case 4:
				return CODE_L2I /* B1I with D2 (OEM6) */
			case 5:
				return CODE_L7I /* B2I with D2 (OEM6) */
			case 6:
				return CODE_L6I /* B3I with D2 (OEM7) */
			case 7:
				return CODE_L1P /* B1C(P) (OEM7) */
			case 9:
				return CODE_L5P /* B2a(P) (OEM7) */
			case 11:
				return CODE_L7D /* B2b(I) (OEM7,F/W 7.08) */
			}
		}
	case SYS_IRN:
		{
			switch sigtype {
			case 0:
				return CODE_L5A /* L5 (OEM7) */
			}
		}
	case SYS_SBS:
		{
			switch sigtype {
			case 0:
				return CODE_L1C /* L1C/A */
			case 6:
				return CODE_L5I /* L5I (OEM6) */
			}
		}
	}
	return 0
}

/* decode receiver tracking status ---------------------------------------------
* decode receiver tracking status
* args   : uint32_t stat I  tracking status field
*          int    *sys   O      system (SYS_???)
*          int    *code  O      signal code (CODE_L??)
*          int    *track O      tracking state
*                         (OEM4/5)
*                         0=L1 idle                   8=L2 idle
*                         1=L1 sky search             9=L2 raw.Buff[idx:]-code align
*                         2=L1 wide freq pull-in     10=L2 search
*                         3=L1 narrow freq pull-in   11=L2 pll
*                         4=L1 pll                   12=L2 steering
*                         5=L1 reacq
*                         6=L1 steering
*                         7=L1 fll
*                         (OEM6/7)
*                         0=idle                      7=freq-lock loop
*                         2=wide freq band pull-in    9=channel alignment
*                         3=narrow freq band pull-in 10=code search
*                         4=phase lock loop          11=aided phase lock loop
*          int    *plock O      phase-lock flag   (0=not locked, 1=locked)
*          int    *clock O      code-lock flag    (0=not locked, 1=locked)
*          int    *parity O     parity known flag (0=not known,  1=known)
*          int    *halfc O      phase measurement (0=half-cycle not added,
*                                                  1=added)
* return : freq-index (-1:error)
* notes  : refer [1][3]
*-----------------------------------------------------------------------------*/
func decode_track_stat(stat uint32, sys, code, track,
	plock, clock, parity, halfc *int) int {
	var satsys, sigtype, idx int
	idx = -1

	*code = CODE_NONE
	*track = int(stat & 0x1F)
	*plock = int((stat >> 10) & 1)
	*parity = int((stat >> 11) & 1)
	*clock = int((stat >> 12) & 1)
	satsys = int((stat >> 16) & 7)
	*halfc = int((stat >> 28) & 1)
	sigtype = int((stat >> 21) & 0x1F)

	switch satsys {
	case 0:
		*sys = SYS_GPS
	case 1:
		*sys = SYS_GLO
	case 2:
		*sys = SYS_SBS
	case 3:
		*sys = SYS_GAL /* OEM6 */
	case 4:
		*sys = SYS_CMP /* OEM6 F/W 6.400 */
	case 5:
		*sys = SYS_QZS /* OEM6 */
	case 6:
		*sys = SYS_IRN /* OEM7 */
	default:
		Trace(2, "oem4 unknown system: sys=%d\n", satsys)
		return -1
	}
	*code = sig2code(*sys, sigtype)
	idx = Code2Idx(*sys, uint8(*code))
	if *code == 0 || idx < 0 {
		Trace(2, "oem4 signal type error: sys=%d sigtype=%d\n", *sys, sigtype)
		return -1
	}
	return idx
}

/* check code priority and return freq-index ---------------------------------*/
func checkpri_novatel(opt string, sys, code, idx int) int {
	nex := NEXOBS

	switch sys {
	case SYS_GPS:
		{
			if strings.Contains(opt, "-GL1L") && idx == 0 {
				if code == CODE_L1L {
					return 0
				}
				return -1
			}
			if strings.Contains(opt, "-GL2S") && idx == 1 {
				if code == CODE_L2X {
					return 1
				}
				return -1
			}
			if strings.Contains(opt, "-GL2P") && idx == 1 {
				if code == CODE_L2P {
					return 1
				}
				return -1
			}
			if code == CODE_L1L {
				if nex < 1 {
					return -1
				}
				return NFREQ
			}
			if code == CODE_L2S {
				if nex < 2 {
					return -1
				}
				return NFREQ + 1
			}
			if code == CODE_L2P {
				if nex < 3 {
					return -1
				}
				return NFREQ + 2
			}
		}
	case SYS_GLO:
		{
			if strings.Contains(opt, "-RL2C") && idx == 1 {
				if code == CODE_L2C {
					return 1
				}
				return -1
			}
			if code == CODE_L2C {
				if nex < 1 {
					return -1
				}
				return NFREQ
			}
		}
	case SYS_GAL:
		{
			if strings.Contains(opt, "-EL6B") && idx == 3 {
				if code == CODE_L6B {
					return 3
				}
				return -1
			}
			if code == CODE_L6B {
				if nex < 2 {
					return -1
				}
				return NFREQ
			}
		}
	case SYS_QZS:
		{
			if strings.Contains(opt, "-JL1L") && idx == 0 {
				if code == CODE_L1L {
					return 0
				}
				return -1
			}
			if strings.Contains(opt, "-JL1Z") && idx == 0 {
				if code == CODE_L1Z {
					return 0
				}
				return -1
			}
			if code == CODE_L1L {
				if nex < 1 {
					return -1
				}
				return NFREQ
			}
			if code == CODE_L1Z {
				if nex < 2 {
					return -1
				}
				return NFREQ + 1
			}
		}
	case SYS_CMP:
		{
			if strings.Contains(opt, "-CL1P") && idx == 0 {
				if code == CODE_L1P {
					return 0
				}
				return -1
			}
			if strings.Contains(opt, "-CL7D") && idx == 0 {
				if code == CODE_L7D {
					return 0
				}
				return -1
			}
			if code == CODE_L1P {
				if nex < 1 {
					return -1
				}
				return NFREQ
			}
			if code == CODE_L7D {
				if nex < 2 {
					return -1
				}
				return NFREQ + 1
			}
		}
	}
	if idx < NFREQ {
		return idx
	}
	return -1
}

/* decode RANGECMPB ----------------------------------------------------------*/
func decode_rangecmpb(raw *Raw) int {
	var (
		p = OEM4HLEN
		//   char *q;
		psr, adr, adr_rolls, lockt, tt, dop, snr, freq, glo_bias                          float64
		i, index, nobs, prn, sat, sys, code, idx, track, plock, clock, parity, halfc, lli int
	)
	if q := strings.Index(raw.Opt, "-GLOBIAS="); q >= 0 {
		fmt.Sscanf(raw.Opt, "-GLOBIAS=%lf", &glo_bias)
	}

	nobs = int(U4L(raw.Buff[p:]))
	if raw.Len < OEM4HLEN+4+nobs*24 {
		Trace(2, "oem4 rangecmpb length error: len=%d nobs=%d\n", raw.Len, nobs)
		return -1
	}
	if raw.OutType > 0 {
		copy(raw.MsgType[len(string(raw.MsgType[:])):], []byte(fmt.Sprintf(" nobs=%d", nobs)))
	}
	for i, p = 0, p+4; i < nobs; i, p = i+1, p+24 {
		if idx = decode_track_stat(U4L(raw.Buff[p:]), &sys, &code, &track, &plock, &clock,
			&parity, &halfc); idx < 0 {
			continue
		}
		prn = int(U1(raw.Buff[p+17:]))
		if sys == SYS_GLO {
			prn -= 37
		}
		if sys == SYS_SBS && prn >= MINPRNQZS_S && prn <= MAXPRNQZS_S && code == CODE_L1C {
			sys = SYS_QZS
			prn += 10
			code = CODE_L1Z /* QZS L1S */
		}
		if sat = SatNo(sys, prn); sat == 0 {
			Trace(3, "oem4 rangecmpb satellite number error: sys=%d,prn=%d\n", sys, prn)
			continue
		}
		if sys == SYS_GLO && parity == 0 {
			continue
		} /* invalid if GLO parity unknown */

		if idx = checkpri_novatel(raw.Opt, sys, code, idx); idx < 0 {
			continue
		}

		dop = float64(exsign(U4L(raw.Buff[p+4:])&0xFFFFFFF, 28)) / 256.0
		psr = float64(U4L(raw.Buff[p+7:])>>4)/128.0 + float64(U1(raw.Buff[p+11:]))*2097152.0

		if freq = Sat2Freq(sat, uint8(code), &raw.NavData); freq != 0.0 {
			adr = float64(I4L(raw.Buff[p+12:])) / 256.0
			adr_rolls = (psr*freq/CLIGHT + adr) / MAXVAL
			if adr_rolls <= 0 {
				adr = -adr + MAXVAL*math.Floor(adr_rolls+-0.5)
			} else {
				adr = -adr + MAXVAL*math.Floor(adr_rolls+0.5)
			}
			if sys == SYS_GLO {
				adr += glo_bias * freq / CLIGHT
			}
		} else {
			adr = 1e-9
		}
		lockt = float64(U4L(raw.Buff[p+18:])&0x1FFFFF) / 32.0 /* lock time */

		lli = 0
		if raw.Tobs[sat-1][idx].Time != 0 {
			tt = TimeDiff(raw.Time, raw.Tobs[sat-1][idx])
			if lockt < 65535.968 && lockt-raw.LockTime[sat-1][idx]+0.05 <= tt {
				lli = LLI_SLIP
			}
		}

		if parity == 0 {
			lli |= LLI_HALFC
		}
		if halfc > 0 {
			lli |= LLI_HALFA
		}
		raw.Tobs[sat-1][idx] = raw.Time
		raw.LockTime[sat-1][idx] = lockt
		raw.Halfc[sat-1][idx] = uint8(halfc)

		snr = float64((U2L(raw.Buff[p+20:])&0x3FF)>>5) + 20.0
		if clock == 0 {
			psr = 0.0
		} /* code unlock */
		if plock == 0 {
			adr, dop = 0.0, 0.0
		} /* phase unlock */

		if math.Abs(TimeDiff(raw.ObsData.Data[0].Time, raw.Time)) > 1e-9 {
			raw.ObsData.n = 0
		}
		if index = obsindex(raw, raw.Time, sat); index >= 0 {
			raw.ObsData.Data[index].L[idx] = adr
			raw.ObsData.Data[index].P[idx] = psr
			raw.ObsData.Data[index].D[idx] = dop
			raw.ObsData.Data[index].SNR[idx] = uint16(snr/SNR_UNIT + 0.5)
			raw.ObsData.Data[index].LLI[idx] = uint8(lli)
			raw.ObsData.Data[index].Code[idx] = uint8(code)
		}
	}
	return 1
}

/* decode RANGEB -------------------------------------------------------------*/
func decode_rangeb(raw *Raw) int {
	var (
		p                                                                                 = OEM4HLEN
		psr, adr, dop, snr, lockt, tt, freq, glo_bias                                     float64
		i, index, nobs, prn, sat, sys, code, idx, track, plock, clock, parity, halfc, lli int
		gfrq                                                                              int
	)
	if q := strings.Index(raw.Opt, "-GLOBIAS="); q >= 0 {
		fmt.Sscanf(raw.Opt[q:], "-GLOBIAS=%lf", &glo_bias)
	}

	nobs = int(U4L(raw.Buff[p:]))
	if raw.Len < OEM4HLEN+4+nobs*44 {
		Trace(2, "oem4 rangeb length error: len=%d nobs=%d\n", raw.Len, nobs)
		return -1
	}
	if raw.OutType > 0 {
		copy(raw.MsgType[len(string(raw.MsgType[:])):], []byte(fmt.Sprintf(" nobs=%d", nobs)))
	}
	for i, p = 0, p+4; i < nobs; i, p = i+1, p+44 {
		if idx = decode_track_stat(U4L(raw.Buff[p+40:]), &sys, &code, &track, &plock, &clock,
			&parity, &halfc); idx < 0 {
			continue
		}
		prn = int(U2L(raw.Buff[p:]))
		if sys == SYS_GLO {
			prn -= 37
		}
		if sys == SYS_SBS && prn >= MINPRNQZS_S && prn <= MAXPRNQZS_S && code == CODE_L1C {
			sys = SYS_QZS
			prn += 10
			code = CODE_L1Z /* QZS L1S */
		}
		if sat = SatNo(sys, prn); sat == 0 {
			Trace(3, "oem4 rangeb satellite number error: sys=%d,prn=%d\n", sys, prn)
			continue
		}
		if sys == SYS_GLO && parity == 0 {
			continue
		}

		if idx = checkpri_novatel(raw.Opt, sys, code, idx); idx < 0 {
			continue
		}

		gfrq = int(U2L(raw.Buff[p:+2])) /* GLONASS FCN+8 */
		psr = R8L(raw.Buff[p:+4])
		adr = R8L(raw.Buff[p:+16])
		dop = float64(R4L(raw.Buff[p:+28]))
		snr = float64(R4L(raw.Buff[p:+32]))
		lockt = float64(R4L(raw.Buff[p:+36]))

		if sys == SYS_GLO {
			freq = Sat2Freq(sat, uint8(code), &raw.NavData)
			adr -= glo_bias * freq / CLIGHT
			if raw.NavData.Glo_fcn[prn-1] == 0 {
				raw.NavData.Glo_fcn[prn-1] = gfrq /* fcn+8 */
			}
		}
		lli = 0
		if raw.Tobs[sat-1][idx].Time != 0 {
			tt = TimeDiff(raw.Time, raw.Tobs[sat-1][idx])
			if lockt-raw.LockTime[sat-1][idx]+0.05 <= tt {
				lli = LLI_SLIP
			}
		}
		if parity == 0 {
			lli |= LLI_HALFC
		}
		if halfc > 0 {
			lli |= LLI_HALFA
		}
		raw.Tobs[sat-1][idx] = raw.Time
		raw.LockTime[sat-1][idx] = lockt
		raw.Halfc[sat-1][idx] = uint8(halfc)

		if clock == 0 {
			psr = 0.0
		} /* code unlock */
		if plock == 0 {
			adr, dop = 0.0, 0.0
		} /* phase unlock */

		if math.Abs(TimeDiff(raw.ObsData.Data[0].Time, raw.Time)) > 1e-9 {
			raw.ObsData.n = 0
		}
		if index = obsindex(raw, raw.Time, sat); index >= 0 {
			raw.ObsData.Data[index].L[idx] = -adr
			raw.ObsData.Data[index].P[idx] = psr
			raw.ObsData.Data[index].D[idx] = dop
			raw.ObsData.Data[index].SNR[idx] = uint16(snr/SNR_UNIT + 0.5)
			raw.ObsData.Data[index].LLI[idx] = uint8(lli)
			raw.ObsData.Data[index].Code[idx] = uint8(code)
		}
	}
	return 1
}

/* decode RAWEPHEMB ----------------------------------------------------------*/
func decode_rawephemb(raw *Raw) int {
	var (
		eph      Eph
		idx      = OEM4HLEN
		subframe [30 * 5]uint8
		prn, sat int
	)

	if raw.Len < OEM4HLEN+102 {
		Trace(2, "oem4 rawephemb length error: len=%d\n", raw.Len)
		return -1
	}
	prn = int(U4L(raw.Buff[idx:]))
	if sat = SatNo(SYS_GPS, prn); sat == 0 {
		Trace(2, "oem4 rawephemb satellite number error: prn=%d\n", prn)
		return -1
	}
	if raw.OutType > 0 {
		copy(raw.MsgType[len(string(raw.MsgType[:])):], []byte(fmt.Sprintf(" prn=%d", prn)))
	}
	copy(subframe[:], raw.Buff[idx+12:idx+12+30*3]) /* subframe 1-3 */

	if DecodeFrame(subframe[:], &eph, nil, nil, nil) == 0 {
		Trace(2, "oem4 rawephemb subframe error: prn=%d\n", prn)
		return -1
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

/* decode IONUTCB ------------------------------------------------------------*/
func decode_ionutcb(raw *Raw) int {
	idx := OEM4HLEN

	if raw.Len < OEM4HLEN+108 {
		Trace(2, "oem4 ionutcb length error: len=%d\n", raw.Len)
		return -1
	}
	for i := 0; i < 8; i++ {
		raw.NavData.Ion_gps[i] = R8L(raw.Buff[idx+i*8:])
	}
	raw.NavData.Utc_gps[0] = R8L(raw.Buff[idx+72:])           /* A0 */
	raw.NavData.Utc_gps[1] = R8L(raw.Buff[idx+80:])           /* A1 */
	raw.NavData.Utc_gps[2] = float64(U4L(raw.Buff[idx+68:]))  /* tot */
	raw.NavData.Utc_gps[3] = float64(U4L(raw.Buff[idx+64:]))  /* WNt */
	raw.NavData.Utc_gps[4] = float64(I4L(raw.Buff[idx+96:]))  /* dt_LS */
	raw.NavData.Utc_gps[5] = float64(U4L(raw.Buff[idx+88:]))  /* WN_LSF */
	raw.NavData.Utc_gps[6] = float64(U4L(raw.Buff[idx+92:]))  /* DN */
	raw.NavData.Utc_gps[7] = float64(I4L(raw.Buff[idx+100:])) /* dt_LSF */
	return 9
}

/* decode RAWWAASFRAMEB ------------------------------------------------------*/
func decode_rawwaasframeb(raw *Raw) int {
	idx := OEM4HLEN

	if raw.Len < OEM4HLEN+48 {
		Trace(2, "oem4 rawwaasframeb length error: len=%d\n", raw.Len)
		return -1
	}
	prn := U4L(raw.Buff[idx+4:])
	if (prn < MINPRNSBS || prn > MAXPRNSBS) && (prn < MINPRNQZS_S || prn > MAXPRNQZS_S) {
		return 0
	}
	if raw.OutType > 0 {
		copy(raw.MsgType[len(string(raw.MsgType[:])):], []byte(fmt.Sprintf(" prn=%d", prn)))
	}
	raw.Sbsmsg.Tow = int(Time2GpsT(raw.Time, &raw.Sbsmsg.Week))
	raw.Sbsmsg.Prn = uint8(prn)
	copy(raw.Sbsmsg.Msg[:], raw.Buff[idx+12:idx+12+29])
	raw.Sbsmsg.Msg[28] &= 0xC0
	return 3
}

/* decode RAWSBASFRAMEB ------------------------------------------------------*/
func decode_rawsbasframeb(raw *Raw) int {
	return decode_rawwaasframeb(raw)
}

/* decode GLOEPHEMERISB ------------------------------------------------------*/
func decode_gloephemerisb(raw *Raw) int {
	var (
		idx            = OEM4HLEN
		geph           GEph
		tow, tof, toff float64
		sat, week      int
	)

	if raw.Len < OEM4HLEN+144 {
		Trace(2, "oem4 gloephemerisb length error: len=%d\n", raw.Len)
		return -1
	}
	prn := U2L(raw.Buff[idx:]) - 37

	if sat = SatNo(SYS_GLO, int(prn)); sat == 0 {
		Trace(2, "oem4 gloephemerisb prn error: prn=%d\n", prn)
		return -1
	}
	if raw.OutType > 0 {
		copy(raw.MsgType[len(string(raw.MsgType[:])):], []byte(fmt.Sprintf(" prn=%d", prn)))
	}
	geph.Frq = int(U2L(raw.Buff[idx+2:])) + OFF_FRQNO
	week = int(U2L(raw.Buff[idx+6:]))
	tow = math.Floor(float64(U4L(raw.Buff[idx+8:]))/1000.0 + 0.5) /* rounded to integer sec */
	toff = float64(U4L(raw.Buff[idx+12:]))
	geph.Iode = int(U4L(raw.Buff[idx+20:]) & 0x7F)
	geph.Svh = 1
	if U4L(raw.Buff[idx+24:]) < 4 {
		geph.Svh = 0 /* 0:healthy,1:unhealthy */
	}
	geph.Pos[0] = R8L(raw.Buff[idx+28:])
	geph.Pos[1] = R8L(raw.Buff[idx+36:])
	geph.Pos[2] = R8L(raw.Buff[idx+44:])
	geph.Vel[0] = R8L(raw.Buff[idx+52:])
	geph.Vel[1] = R8L(raw.Buff[idx+60:])
	geph.Vel[2] = R8L(raw.Buff[idx+68:])
	geph.Acc[0] = R8L(raw.Buff[idx+76:])
	geph.Acc[1] = R8L(raw.Buff[idx+84:])
	geph.Acc[2] = R8L(raw.Buff[idx+92:])
	geph.Taun = R8L(raw.Buff[idx+100:])
	geph.DTaun = R8L(raw.Buff[idx+108:])
	geph.Gamn = R8L(raw.Buff[idx+116:])
	tof = float64(U4L(raw.Buff[idx+124:])) - toff /* glonasst.gpst */
	geph.Age = int(U4L(raw.Buff[idx+136:]))
	geph.Toe = GpsT2Time(week, tow)
	tof += math.Floor(tow/86400.0) * 86400
	if tof < tow-43200.0 {
		tof += 86400.0
	} else if tof > tow+43200.0 {
		tof -= 86400.0
	}
	geph.Tof = GpsT2Time(week, tof)

	if !strings.Contains(raw.Opt, "-EPHALL") {
		if math.Abs(TimeDiff(geph.Toe, raw.NavData.Geph[prn-1].Toe)) < 1.0 &&
			geph.Svh == raw.NavData.Geph[prn-1].Svh {
			return 0
		} /* unchanged */
	}
	geph.Sat = sat
	raw.NavData.Geph[prn-1] = geph
	raw.EphSat = sat
	raw.EphSet = 0
	return 2
}

/* decode QZSSRAWEPHEMB ------------------------------------------------------*/
func decode_qzssrawephemb(raw *Raw) int {
	var (
		eph      Eph
		idx      = OEM4HLEN
		subfrm   [90]uint8
		prn, sat int
	)
	if raw.Len < OEM4HLEN+106 {
		Trace(2, "oem4 qzssrawephemb length error: len=%d\n", raw.Len)
		return -1
	}
	prn = int(U4L(raw.Buff[idx:]))
	if sat = SatNo(SYS_QZS, prn); sat == 0 {
		Trace(2, "oem4 qzssrawephemb satellite number error: prn=%d\n", prn)
		return -1
	}
	if raw.OutType > 0 {
		copy(raw.MsgType[len(string(raw.MsgType[:])):], []byte(fmt.Sprintf(" prn=%d", prn)))
	}
	copy(subfrm[:], raw.Buff[idx+12:idx+12+90])

	if DecodeFrame(subfrm[:], &eph, nil, nil, nil) == 0 {
		Trace(3, "oem4 qzssrawephemb ephemeris error: prn=%d\n", prn)
		return 0
	}
	if !strings.Contains(raw.Opt, "-EPHALL") {
		if eph.Iodc == raw.NavData.Ephs[sat-1].Iodc &&
			eph.Iode == raw.NavData.Ephs[sat-1].Iode {
			return 0
		} /* unchanged */
	}
	eph.Sat = sat
	raw.NavData.Ephs[sat-1] = eph
	raw.EphSat = sat
	raw.EphSet = 0
	return 2
}

/* decode QZSSRAWSUBFRAMEB ---------------------------------------------------*/
func decode_qzssrawsubframeb(raw *Raw) int {
	var (
		eph          Eph
		ion, utc     [8]float64
		idx          = OEM4HLEN
		prn, sat, id int
	)

	if raw.Len < OEM4HLEN+44 {
		Trace(2, "oem4 qzssrawsubframeb length error: len=%d\n", raw.Len)
		return -1
	}
	prn = int(U4L(raw.Buff[idx:]))
	id = int(U4L(raw.Buff[idx+4:]))
	if sat = SatNo(SYS_QZS, prn); sat == 0 {
		Trace(2, "oem4 qzssrawsubframeb satellite error: prn=%d\n", prn)
		return -1
	}
	if raw.OutType > 0 {
		copy(raw.MsgType[len(string(raw.MsgType[:])):], []byte(fmt.Sprintf(" prn=%d id=%d", prn, id)))
	}
	if id < 1 || id > 5 {
		Trace(2, "oem4 qzssrawsubframeb subfrm id error: prn=%d id=%d\n", prn, id)
		return -1
	}
	copy(raw.SubFrm[sat-1][30*(id-1):], raw.Buff[idx+8:idx+8+30])

	if id == 3 {
		if DecodeFrame(raw.SubFrm[sat-1][:], &eph, nil, nil, nil) == 0 {
			return 0
		}
		if !strings.Contains(raw.Opt, "-EPHALL") {
			if eph.Iodc == raw.NavData.Ephs[sat-1].Iodc &&
				eph.Iode == raw.NavData.Ephs[sat-1].Iode {
				return 0
			} /* unchanged */
		}
		eph.Sat = sat
		raw.NavData.Ephs[sat-1] = eph
		raw.EphSat = sat
		raw.EphSet = 0
		return 2
	} else if id == 4 || id == 5 {
		if DecodeFrame(raw.SubFrm[sat-1][:], nil, nil, ion[:], utc[:]) == 0 {
			return 0
		}
		adj_utcweek(raw.Time, utc[:])
		MatCpy(raw.NavData.Ion_qzs[:], ion[:], 8, 1)
		MatCpy(raw.NavData.Utc_qzs[:], utc[:], 8, 1)
		return 9
	}
	return 0
}

/* decode QZSSIONUTCB --------------------------------------------------------*/
func decode_qzssionutcb(raw *Raw) int {
	idx := OEM4HLEN

	if raw.Len < OEM4HLEN+108 {
		Trace(2, "oem4 qzssionutcb length error: len=%d\n", raw.Len)
		return -1
	}
	for i := 0; i < 8; i++ {
		raw.NavData.Ion_qzs[i] = R8L(raw.Buff[idx+i*8:])
	}
	raw.NavData.Utc_qzs[0] = R8L(raw.Buff[idx+72:])
	raw.NavData.Utc_qzs[1] = R8L(raw.Buff[idx+80:])
	raw.NavData.Utc_qzs[2] = float64(U4L(raw.Buff[idx+68:]))
	raw.NavData.Utc_qzs[3] = float64(U4L(raw.Buff[idx+64:]))
	raw.NavData.Utc_qzs[4] = float64(I4L(raw.Buff[idx+96:]))
	return 9
}

/* decode GALEPHEMERISB ------------------------------------------------------*/
func decode_galephemerisb(raw *Raw) int {
	var (
		eph                                                                   Eph
		idx                                                                   = OEM4HLEN
		sqrtA, af0_fnav, af1_fnav, af2_fnav, af0_inav, af1_inav, af2_inav, tt float64
		prn, sat, week, rcv_fnav, svh_e1b, svh_e5a, svh_e5b, dvs_e1b, dvs_e5a int
		dvs_e5b, toc_fnav, toc_inav, set                                      int /* 1:I/NAV+2:F/NAV */
		sel_eph                                                               = 3
	)

	if strings.Contains(raw.Opt, "-GALINAV") {
		sel_eph = 1
	}
	if strings.Contains(raw.Opt, "-GALFNAV") {
		sel_eph = 2
	}

	if raw.Len < OEM4HLEN+220 {
		Trace(2, "oem4 galephemrisb length error: len=%d\n", raw.Len)
		return -1
	}
	prn = int(U4L(raw.Buff[idx:]))
	idx += 4
	rcv_fnav = int(U4L(raw.Buff[idx:])) & 1
	idx += 4
	_ = int(U4L(raw.Buff[idx:])) & 1
	idx += 4 //rcv_inav
	svh_e1b = int(U1(raw.Buff[idx:])) & 3
	idx += 1
	svh_e5a = int(U1(raw.Buff[idx:])) & 3
	idx += 1
	svh_e5b = int(U1(raw.Buff[idx:])) & 3
	idx += 1
	dvs_e1b = int(U1(raw.Buff[idx:])) & 1
	idx += 1
	dvs_e5a = int(U1(raw.Buff[idx:])) & 1
	idx += 1
	dvs_e5b = int(U1(raw.Buff[idx:])) & 1
	idx += 1
	eph.Sva = int(U1(raw.Buff[idx:]))
	idx += 1 + 1 /* SISA index */
	eph.Iode = int(U4L(raw.Buff[idx:]))
	idx += 4 /* IODNav */
	eph.Toes = float64(U4L(raw.Buff[idx:]))
	idx += 4
	sqrtA = R8L(raw.Buff[idx:])
	idx += 8
	eph.Deln = R8L(raw.Buff[idx:])
	idx += 8
	eph.M0 = R8L(raw.Buff[idx:])
	idx += 8
	eph.E = R8L(raw.Buff[idx:])
	idx += 8
	eph.Omg = R8L(raw.Buff[idx:])
	idx += 8
	eph.Cuc = R8L(raw.Buff[idx:])
	idx += 8
	eph.Cus = R8L(raw.Buff[idx:])
	idx += 8
	eph.Crc = R8L(raw.Buff[idx:])
	idx += 8
	eph.Crs = R8L(raw.Buff[idx:])
	idx += 8
	eph.Cic = R8L(raw.Buff[idx:])
	idx += 8
	eph.Cis = R8L(raw.Buff[idx:])
	idx += 8
	eph.I0 = R8L(raw.Buff[idx:])
	idx += 8
	eph.Idot = R8L(raw.Buff[idx:])
	idx += 8
	eph.OMG0 = R8L(raw.Buff[idx:])
	idx += 8
	eph.OMGd = R8L(raw.Buff[idx:])
	idx += 8
	toc_fnav = int(U4L(raw.Buff[idx:]))
	idx += 4
	af0_fnav = R8L(raw.Buff[idx:])
	idx += 8
	af1_fnav = R8L(raw.Buff[idx:])
	idx += 8
	af2_fnav = R8L(raw.Buff[idx:])
	idx += 8
	toc_inav = int(U4L(raw.Buff[idx:]))
	idx += 4
	af0_inav = R8L(raw.Buff[idx:])
	idx += 8
	af1_inav = R8L(raw.Buff[idx:])
	idx += 8
	af2_inav = R8L(raw.Buff[idx:])
	idx += 8
	eph.Tgd[0] = R8L(raw.Buff[idx:])
	idx += 8                         /* BGD: E5A-E1 (s) */
	eph.Tgd[1] = R8L(raw.Buff[idx:]) /* BGD: E5B-E1 (s) */

	if sat = SatNo(SYS_GAL, prn); sat == 0 {
		Trace(2, "oemv galephemeris satellite error: prn=%d\n", prn)
		return -1
	}
	if raw.OutType > 0 {
		copy(raw.MsgType[len(string(raw.MsgType[:])):], []byte(fmt.Sprintf(" prn=%d", prn)))
	}
	if rcv_fnav > 0 { /* 0:I/NAV,1:F/NAV */
		set = 1
	}

	if (sel_eph&1) == 0 && set == 0 {
		return 0
	}
	if (sel_eph&2) == 0 && set == 1 {
		return 0
	}

	eph.Sat = sat
	eph.A = sqrtA * sqrtA
	if set > 0 {
		eph.F0 = af0_fnav
		eph.F1 = af1_fnav
		eph.F2 = af2_fnav
		eph.Code = ((1 << 1) + (1 << 8))
		eph.Toc = adjweek(raw.Time, float64(toc_fnav))

	} else {
		eph.F0 = af0_inav
		eph.F1 = af1_inav
		eph.F2 = af2_inav
		eph.Code = ((1 << 0) + (1 << 2) + (1 << 9))
		eph.Toc = adjweek(raw.Time, float64(toc_inav))
	}
	eph.Svh = ((svh_e5b << 7) | (dvs_e5b << 6) | (svh_e5a << 4) | (dvs_e5a << 3) |
		(svh_e1b << 1) | dvs_e1b)
	eph.Iodc = eph.Iode
	_ = Time2GpsT(raw.Time, &week) //tow
	eph.Week = week                /* gps-week = gal-week */
	eph.Toe = GpsT2Time(eph.Week, eph.Toes)

	tt = TimeDiff(eph.Toe, raw.Time)
	if tt < -302400.0 {
		eph.Week++
	} else if tt > 302400.0 {
		eph.Week--
	}
	eph.Toe = GpsT2Time(eph.Week, eph.Toes)
	eph.Ttr = raw.Time

	if !strings.Contains(raw.Opt, "-EPHALL") {
		if eph.Iode == raw.NavData.Ephs[sat-1+MAXSAT*set].Iode &&
			TimeDiff(eph.Toe, raw.NavData.Ephs[sat-1+MAXSAT*set].Toe) == 0.0 &&
			TimeDiff(eph.Toc, raw.NavData.Ephs[sat-1+MAXSAT*set].Toc) == 0.0 {
			return 0 /* unchanged */
		}
	}
	raw.NavData.Ephs[sat-1+MAXSAT*set] = eph
	raw.EphSat = sat
	raw.EphSet = set
	return 2
}

/* decode GALCLOCKB ----------------------------------------------------------*/
func decode_galclockb(raw *Raw) int {
	var (
		idx                              = OEM4HLEN
		a0, a1                           float64
		dtls, tot, wnt, wnlsf, dn, dtlsf int
	)

	if raw.Len < OEM4HLEN+64 {
		Trace(2, "oem4 galclockb length error: len=%d\n", raw.Len)
		return -1
	}
	a0 = R8L(raw.Buff[idx:])
	idx += 8
	a1 = R8L(raw.Buff[idx:])
	idx += 8
	dtls = int(I4L(raw.Buff[idx:]))
	idx += 4
	tot = int(U4L(raw.Buff[idx:]))
	idx += 4
	wnt = int(U4L(raw.Buff[idx:]))
	idx += 4
	wnlsf = int(U4L(raw.Buff[idx:]))
	idx += 4
	dn = int(U4L(raw.Buff[idx:]))
	idx += 4
	dtlsf = int(U4L(raw.Buff[idx:]))
	idx += 4
	_ = R8L(raw.Buff[idx:])
	idx += 8 //a0g
	_ = R8L(raw.Buff[idx:])
	idx += 8 //a1g
	_ = U4L(raw.Buff[idx:])
	idx += 4                //t0g
	_ = U4L(raw.Buff[idx:]) //wn0g
	raw.NavData.Utc_gal[0] = a0
	raw.NavData.Utc_gal[1] = a1
	raw.NavData.Utc_gal[2] = float64(tot)
	raw.NavData.Utc_gal[3] = float64(wnt)
	raw.NavData.Utc_gal[4] = float64(dtls)
	raw.NavData.Utc_gal[5] = float64(wnlsf)
	raw.NavData.Utc_gal[6] = float64(dn)
	raw.NavData.Utc_gal[7] = float64(dtlsf)
	return 9
}

/* decode GALIONOB -----------------------------------------------------------*/
func decode_galionob(raw *Raw) int {
	var (
		idx = OEM4HLEN
		ai  [3]float64
		sf  [5]int
	)

	if raw.Len < OEM4HLEN+29 {
		Trace(2, "oem4 galionob length error: len=%d\n", raw.Len)
		return -1
	}
	ai[0] = R8L(raw.Buff[idx:])
	idx += 8
	ai[1] = R8L(raw.Buff[idx:])
	idx += 8
	ai[2] = R8L(raw.Buff[idx:])
	idx += 8
	sf[0] = int(U1(raw.Buff[idx:]))
	idx += 1
	sf[1] = int(U1(raw.Buff[idx:]))
	idx += 1
	sf[2] = int(U1(raw.Buff[idx:]))
	idx += 1
	sf[3] = int(U1(raw.Buff[idx:]))
	idx += 1
	sf[4] = int(U1(raw.Buff[idx:]))

	for i := 0; i < 3; i++ {
		raw.NavData.Ion_gal[i] = ai[i]
	}
	return 9
}

/* decode BDSEPHEMERISB ------------------------------------------------------*/
func decode_bdsephemerisb(raw *Raw) int {
	var (
		eph           Eph
		idx           = OEM4HLEN
		ura, sqrtA    float64
		prn, sat, toc int
	)

	if raw.Len < OEM4HLEN+196 {
		Trace(2, "oem4 bdsephemrisb length error: len=%d\n", raw.Len)
		return -1
	}
	prn = int(U4L(raw.Buff[idx:]))
	idx += 4
	eph.Week = int(U4L(raw.Buff[idx:]))
	idx += 4
	ura = R8L(raw.Buff[idx:])
	idx += 8
	eph.Svh = int(U4L(raw.Buff[idx:]) & 1)
	idx += 4
	eph.Tgd[0] = R8L(raw.Buff[idx:])
	idx += 8 /* TGD1 for B1 (s) */
	eph.Tgd[1] = R8L(raw.Buff[idx:])
	idx += 8 /* TGD2 for B2 (s) */
	eph.Iodc = int(U4L(raw.Buff[idx:]))
	idx += 4 /* AODC */
	toc = int(U4L(raw.Buff[idx:]))
	idx += 4
	eph.F0 = R8L(raw.Buff[idx:])
	idx += 8
	eph.F1 = R8L(raw.Buff[idx:])
	idx += 8
	eph.F2 = R8L(raw.Buff[idx:])
	idx += 8
	eph.Iode = int(U4L(raw.Buff[idx:]))
	idx += 4 /* AODE */
	eph.Toes = float64(U4L(raw.Buff[idx:]))
	idx += 4
	sqrtA = R8L(raw.Buff[idx:])
	idx += 8
	eph.E = R8L(raw.Buff[idx:])
	idx += 8
	eph.Omg = R8L(raw.Buff[idx:])
	idx += 8
	eph.Deln = R8L(raw.Buff[idx:])
	idx += 8
	eph.M0 = R8L(raw.Buff[idx:])
	idx += 8
	eph.OMG0 = R8L(raw.Buff[idx:])
	idx += 8
	eph.OMGd = R8L(raw.Buff[idx:])
	idx += 8
	eph.I0 = R8L(raw.Buff[idx:])
	idx += 8
	eph.Idot = R8L(raw.Buff[idx:])
	idx += 8
	eph.Cuc = R8L(raw.Buff[idx:])
	idx += 8
	eph.Cus = R8L(raw.Buff[idx:])
	idx += 8
	eph.Crc = R8L(raw.Buff[idx:])
	idx += 8
	eph.Crs = R8L(raw.Buff[idx:])
	idx += 8
	eph.Cic = R8L(raw.Buff[idx:])
	idx += 8
	eph.Cis = R8L(raw.Buff[idx:])

	if sat = SatNo(SYS_CMP, prn); sat == 0 {
		Trace(2, "oemv bdsephemeris satellite error: prn=%d\n", prn)
		return -1
	}
	if raw.OutType > 0 {
		copy(raw.MsgType[len(string(raw.MsgType[:])):], []byte(fmt.Sprintf(" prn=%d", prn)))
	}
	eph.Sat = sat
	eph.A = sqrtA * sqrtA
	eph.Sva = uraindex(ura)
	eph.Toe = BDT2GpsT(BDT2Time(eph.Week, eph.Toes))     /* bdt . gpst */
	eph.Toc = BDT2GpsT(BDT2Time(eph.Week, float64(toc))) /* bdt . gpst */
	eph.Ttr = raw.Time

	if !strings.Contains(raw.Opt, "-EPHALL") {
		if TimeDiff(raw.NavData.Ephs[sat-1].Toe, eph.Toe) == 0.0 &&
			TimeDiff(raw.NavData.Ephs[sat-1].Toc, eph.Toc) == 0.0 {
			return 0
		}
	}
	raw.NavData.Ephs[sat-1] = eph
	raw.EphSat = sat
	raw.EphSet = 0
	return 2
}

/* decode NAVICEPHEMERISB ----------------------------------------------------*/
func decode_navicephemerisb(raw *Raw) int {
	var (
		eph                                Eph
		idx                                = OEM4HLEN
		sqrtA                              float64
		prn, sat, toc, l5_health, s_health int
	)

	if raw.Len < OEM4HLEN+204 {
		Trace(2, "oem4 navicephemrisb length error: len=%d\n", raw.Len)
		return -1
	}
	prn = int(U4L(raw.Buff[idx:]))
	idx += 4
	eph.Week = int(U4L(raw.Buff[idx:]))
	idx += 4
	eph.F0 = R8L(raw.Buff[idx:])
	idx += 8
	eph.F1 = R8L(raw.Buff[idx:])
	idx += 8
	eph.F2 = R8L(raw.Buff[idx:])
	idx += 8
	eph.Sva = int(U4L(raw.Buff[idx:]))
	idx += 4 /* URA index */
	toc = int(U4L(raw.Buff[idx:]))
	idx += 4
	eph.Tgd[0] = R8L(raw.Buff[idx:])
	idx += 8 /* TGD */
	eph.Deln = R8L(raw.Buff[idx:])
	idx += 8
	eph.Iode = int(U4L(raw.Buff[idx:]))
	idx += 4                /* IODEC */
	_ = U4L(raw.Buff[idx:]) /* rsv */
	idx += 4
	l5_health = int(U4L(raw.Buff[idx:]) & 1)
	idx += 4
	s_health = int(U4L(raw.Buff[idx:]) & 1)
	idx += 4
	eph.Cuc = R8L(raw.Buff[idx:])
	idx += 8
	eph.Cus = R8L(raw.Buff[idx:])
	idx += 8
	eph.Cic = R8L(raw.Buff[idx:])
	idx += 8
	eph.Cis = R8L(raw.Buff[idx:])
	idx += 8
	eph.Crc = R8L(raw.Buff[idx:])
	idx += 8
	eph.Crs = R8L(raw.Buff[idx:])
	idx += 8
	eph.Idot = R8L(raw.Buff[idx:])
	idx += 8
	_ = U4L(raw.Buff[idx:]) /* rsv */
	idx += 4
	eph.M0 = R8L(raw.Buff[idx:])
	idx += 8
	eph.Toes = float64(U4L(raw.Buff[idx:]))
	idx += 4
	eph.E = R8L(raw.Buff[idx:])
	idx += 8
	sqrtA = R8L(raw.Buff[idx:])
	idx += 8
	eph.OMG0 = R8L(raw.Buff[idx:])
	idx += 8
	eph.Omg = R8L(raw.Buff[idx:])
	idx += 8
	eph.OMGd = R8L(raw.Buff[idx:])
	idx += 8
	eph.I0 = R8L(raw.Buff[idx:])
	idx += 8
	_ = U4L(raw.Buff[idx:]) /* rsv */
	idx += 4
	_ = U4L(raw.Buff[idx:]) /* alert */
	idx += 4
	_ = U4L(raw.Buff[idx:]) /* autonav */

	if float64(toc) != eph.Toes { /* toe and toc should be matched */
		Trace(2, "oem4 navicephemrisb toe and toc unmatch prn=%d\n", prn)
		return -1
	}
	if sat = SatNo(SYS_IRN, prn); sat == 0 {
		Trace(2, "oemv navicephemeris satellite error: prn=%d\n", prn)
		return 0
	}
	if raw.OutType > 0 {
		copy(raw.MsgType[len(string(raw.MsgType[:])):], []byte(fmt.Sprintf(" prn=%d", prn)))
	}
	eph.Sat = sat
	eph.A = sqrtA * sqrtA
	eph.Svh = (l5_health << 1) | s_health
	eph.Iodc = eph.Iode
	eph.Week += 1024 /* irnss-week . gps-week */
	eph.Toe = GpsT2Time(eph.Week, eph.Toes)
	eph.Toc = GpsT2Time(eph.Week, float64(toc))
	eph.Ttr = raw.Time
	eph.Tgd[1] = 0.0

	if !strings.Contains(raw.Opt, "-EPHALL") {
		if TimeDiff(raw.NavData.Ephs[sat-1].Toe, eph.Toe) == 0.0 &&
			raw.NavData.Ephs[sat-1].Iode == eph.Iode {
			return 0
		} /* unchanged */
	}
	raw.NavData.Ephs[sat-1] = eph
	raw.EphSat = sat
	raw.EphSet = 0
	return 2
}

/* decode RGEB ---------------------------------------------------------------*/
func decode_rgeb(raw *Raw) int {
	var (
		idx                                                          = OEM3HLEN
		tow, psr, adr, tt, lockt, dop, snr                           float64
		i, week, nobs, prn, sat, stat, sys, parity, lli, index, freq int
	)

	week = AdjGpsWeek(int(U4L(raw.Buff[idx:])))
	tow = R8L(raw.Buff[idx+4:])
	nobs = int(U4L(raw.Buff[idx+12:]))
	raw.Time = GpsT2Time(week, tow)

	if raw.Len != OEM3HLEN+20+nobs*44 {
		Trace(2, "oem3 regb length error: len=%d nobs=%d\n", raw.Len, nobs)
		return -1
	}
	for i, idx = 0, idx+20; i < nobs; i, idx = i+1, idx+44 {
		prn = int(U4L(raw.Buff[idx:]))
		psr = R8L(raw.Buff[idx+4:])
		adr = R8L(raw.Buff[idx+16:])
		dop = float64(R4L(raw.Buff[idx+28:]))
		snr = float64(R4L(raw.Buff[idx+32:]))
		lockt = float64(R4L(raw.Buff[idx+36:])) /* lock time (s) */
		stat = int(I4L(raw.Buff[idx+40:]))      /* tracking status */
		freq = (stat >> 20) & 1                 /* L1:0,L2:1 */
		sys = (stat >> 15) & 7                  /* satellite sys (0:GPS,1:GLONASS,2:WAAS) */
		parity = (stat >> 10) & 1               /* parity known */
		var isys int
		switch sys {
		case 1:
			isys = SYS_GLO
		case 2:
			isys = SYS_SBS
		default:
			isys = SYS_GPS
		}
		if sat = SatNo(isys, prn); sat == 0 {
			Trace(2, "oem3 regb satellite number error: sys=%d prn=%d\n", sys, prn)
			continue
		}
		lli = 0
		if raw.Tobs[sat-1][freq].Time != 0 {
			tt = TimeDiff(raw.Time, raw.Tobs[sat-1][freq])
			if lockt-raw.LockTime[sat-1][freq]+0.05 < tt ||
				parity != int(raw.Halfc[sat-1][freq]) {
				lli = 1
			}
		}

		if parity == 0 {
			lli |= 2
		}
		raw.Tobs[sat-1][freq] = raw.Time
		raw.LockTime[sat-1][freq] = lockt
		raw.Halfc[sat-1][freq] = uint8(parity)

		if math.Abs(TimeDiff(raw.ObsData.Data[0].Time, raw.Time)) > 1e-9 {
			raw.ObsData.n = 0
		}
		if index = obsindex(raw, raw.Time, sat); index >= 0 {
			raw.ObsData.Data[index].L[freq] = -adr /* flip sign */
			raw.ObsData.Data[index].P[freq] = psr
			raw.ObsData.Data[index].D[freq] = dop
			if 0.0 <= snr && snr < 255.0 {
				raw.ObsData.Data[index].SNR[freq] =
					uint16(snr/SNR_UNIT + 0.5)
			} else {
				raw.ObsData.Data[index].SNR[freq] = 0
			}
			raw.ObsData.Data[index].LLI[freq] = uint8(lli)
			if freq == 0 {
				raw.ObsData.Data[index].Code[freq] = CODE_L1C
			} else {
				raw.ObsData.Data[index].Code[freq] = CODE_L2P
			}
		}
	}
	return 1
}

/* decode RGED ---------------------------------------------------------------*/
func decode_rged(raw *Raw) int {
	var (
		word                                                              uint32
		idx                                                               = OEM3HLEN
		tow, psrh, psrl, psr, adr, adr_rolls, tt, lockt, dop              float64
		i, week, nobs, prn, sat, stat, sys, parity, lli, index, freq, snr int
	)
	nobs = int(U2L(raw.Buff[idx:]))
	week = AdjGpsWeek(int(U2L(raw.Buff[idx+2:])))
	tow = float64(U4L(raw.Buff[idx+4:])) / 100.0
	raw.Time = GpsT2Time(week, tow)
	if raw.Len != OEM3HLEN+12+nobs*20 {
		Trace(2, "oem3 regd length error: len=%d nobs=%d\n", raw.Len, nobs)
		return -1
	}
	for i, idx = 0, idx+12; i < nobs; i, idx = i+1, idx+20 {
		word = U4L(raw.Buff[idx:])
		prn = int(word & 0x3F)
		snr = int((word>>6)&0x1F) + 20
		lockt = float64(word>>11) / 32.0
		adr = -float64(I4L(raw.Buff[idx+4:])) / 256.0
		word = U4L(raw.Buff[idx+8:])
		psrh = float64(word & 0xF)
		dop = float64(exsign(word>>4, 28)) / 256.0
		psrl = float64(U4L(raw.Buff[idx+12:]))
		stat = int(U4L(raw.Buff[idx+16:]) >> 8)
		freq = (stat >> 20) & 1   /* L1:0,L2:1 */
		sys = (stat >> 15) & 7    /* satellite sys (0:GPS,1:GLONASS,2:WAAS) */
		parity = (stat >> 10) & 1 /* parity known */
		var isys int
		switch sys {
		case 1:
			isys = SYS_GLO
		case 2:
			isys = SYS_SBS
		default:
			isys = SYS_GPS
		}
		if sat = SatNo(isys, prn); sat == 0 {
			Trace(2, "oem3 regd satellite number error: sys=%d prn=%d\n", sys, prn)
			continue
		}
		psr = (psrh*4294967296.0 + psrl) / 128.0
		if freq == 0 {
			adr_rolls = math.Floor((psr/(WL1)-adr)/MAXVAL + 0.5)
		} else {
			adr_rolls = math.Floor((psr/WL2-adr)/MAXVAL + 0.5)
		}
		adr = adr + MAXVAL*adr_rolls

		lli = 0
		if raw.Tobs[sat-1][freq].Time != 0 {
			tt = TimeDiff(raw.Time, raw.Tobs[sat-1][freq])
			if lockt-raw.LockTime[sat-1][freq]+0.05 < tt ||
				parity != int(raw.Halfc[sat-1][freq]) {
				lli = 1
			}
		}
		if parity == 0 {
			lli |= 2
		}
		raw.Tobs[sat-1][freq] = raw.Time
		raw.LockTime[sat-1][freq] = lockt
		raw.Halfc[sat-1][freq] = uint8(parity)

		if math.Abs(TimeDiff(raw.ObsData.Data[0].Time, raw.Time)) > 1e-9 {
			raw.ObsData.n = 0
		}
		if index = obsindex(raw, raw.Time, sat); index >= 0 {
			raw.ObsData.Data[index].L[freq] = adr
			raw.ObsData.Data[index].P[freq] = psr
			raw.ObsData.Data[index].D[freq] = dop
			raw.ObsData.Data[index].SNR[freq] = uint16(float64(snr)/SNR_UNIT + 0.5)
			raw.ObsData.Data[index].LLI[freq] = uint8(lli)
			if freq == 0 {
				raw.ObsData.Data[index].Code[freq] = CODE_L1C
			} else {
				raw.ObsData.Data[index].Code[freq] = CODE_L2P
			}
		}
	}
	return 1
}

/* decode REPB ---------------------------------------------------------------*/
func decode_repb(raw *Raw) int {
	var (
		idx      = OEM3HLEN
		eph      Eph
		prn, sat int
	)

	if raw.Len != OEM3HLEN+96 {
		Trace(2, "oem3 repb length error: len=%d\n", raw.Len)
		return -1
	}
	prn = int(U4L(raw.Buff[idx:]))
	if sat = SatNo(SYS_GPS, prn); sat == 0 {
		Trace(2, "oem3 repb satellite number error: prn=%d\n", prn)
		return -1
	}
	if DecodeFrame(raw.Buff[idx+4:], &eph, nil, nil, nil) == 0 {
		Trace(2, "oem3 repb subframe error: prn=%d\n", prn)
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

/* decode FRMB --------------------------------------------------------------*/
func decode_frmb(raw *Raw) int {
	var (
		idx                = OEM3HLEN
		tow                float64
		i, week, prn, nbit int
	)
	Trace(3, "decode_frmb: len=%d\n", raw.Len)

	week = AdjGpsWeek(int(U4L(raw.Buff[idx:])))
	tow = R8L(raw.Buff[idx+4:])
	prn = int(U4L(raw.Buff[idx+12:]))
	nbit = int(U4L(raw.Buff[idx+20:]))
	raw.Time = GpsT2Time(week, tow)
	if nbit != 250 {
		return 0
	}
	if prn < MINPRNSBS || MAXPRNSBS < prn {
		Trace(2, "oem3 frmb satellite number error: prn=%d\n", prn)
		return -1
	}
	raw.Sbsmsg.Week = week
	raw.Sbsmsg.Tow = int(tow)
	raw.Sbsmsg.Prn = uint8(prn)
	for i = 0; i < 29; i++ {
		raw.Sbsmsg.Msg[i] = raw.Buff[idx:][24+i]
	}
	return 3
}

/* decode IONB ---------------------------------------------------------------*/
func decode_ionb(raw *Raw) int {
	var idx = OEM3HLEN

	if raw.Len != 64+OEM3HLEN {
		Trace(2, "oem3 ionb length error: len=%d\n", raw.Len)
		return -1
	}
	for i := 0; i < 8; i++ {
		raw.NavData.Ion_gps[i] = R8L(raw.Buff[idx+i*8:])
	}
	return 9
}

/* decode UTCB ---------------------------------------------------------------*/
func decode_utcb(raw *Raw) int {
	idx := OEM3HLEN

	if raw.Len != 40+OEM3HLEN {
		Trace(2, "oem3 utcb length error: len=%d\n", raw.Len)
		return -1
	}
	raw.NavData.Utc_gps[0] = R8L(raw.Buff[idx:])
	raw.NavData.Utc_gps[1] = R8L(raw.Buff[idx+8:])
	raw.NavData.Utc_gps[2] = float64(U4L(raw.Buff[idx+16:]))
	raw.NavData.Utc_gps[3] = float64(AdjGpsWeek(int(U4L(raw.Buff[idx+20:]))))
	raw.NavData.Utc_gps[4] = float64(I4L(raw.Buff[idx+28:]))
	return 9
}

/* decode NovAtel OEM4/V/6/7 message -----------------------------------------*/
func decode_oem4(raw *Raw) int {
	var (
		tow             float64
		tstr            string
		msg, stat, week int
	)
	ctype := U2L(raw.Buff[4:])

	Trace(3, "decode_oem4: type=%3d len=%d\n", ctype, raw.Len)

	/* check crc32 */
	if Rtk_CRC32(raw.Buff[:], raw.Len) != U4L(raw.Buff[raw.Len:]) {
		Trace(2, "oem4 crc error: type=%3d len=%d\n", ctype, raw.Len)
		return -1
	}
	msg = int((U1(raw.Buff[6:]) >> 4) & 0x3) /* message type: 0=binary,1=ascii */
	stat = int(U1(raw.Buff[13:]))
	week = int(U2L(raw.Buff[14:]))

	if stat == 20 || week == 0 {
		Trace(3, "oem4 time error: type=%3d msg=%d stat=%d week=%d\n", ctype, msg,
			stat, week)
		return 0
	}
	week = AdjGpsWeek(week)
	tow = float64(U4L(raw.Buff[16:])) * 0.001
	raw.Time = GpsT2Time(week, tow)
	if msg != 0 {
		return 0
	}

	if raw.OutType > 0 {
		Time2Str(GpsT2Time(week, tow), &tstr, 2)
		copy(raw.MsgType[len(string(raw.MsgType[:])):], []byte(fmt.Sprintf("OEM4 %4d (%4d): %s", ctype, raw.Len, tstr)))
	}
	switch ctype {
	case ID_RANGECMP:
		return decode_rangecmpb(raw)
	case ID_RANGE:
		return decode_rangeb(raw)
	case ID_RAWEPHEM:
		return decode_rawephemb(raw)
	case ID_IONUTC:
		return decode_ionutcb(raw)
	case ID_RAWWAASFRAME:
		return decode_rawwaasframeb(raw)
	case ID_RAWSBASFRAME:
		return decode_rawsbasframeb(raw)
	case ID_GLOEPHEMERIS:
		return decode_gloephemerisb(raw)
	case ID_GALEPHEMERIS:
		return decode_galephemerisb(raw)
	case ID_GALIONO:
		return decode_galionob(raw)
	case ID_GALCLOCK:
		return decode_galclockb(raw)
	case ID_QZSSRAWEPHEM:
		return decode_qzssrawephemb(raw)
	case ID_QZSSRAWSUBFRAME:
		return decode_qzssrawsubframeb(raw)
	case ID_QZSSIONUTC:
		return decode_qzssionutcb(raw)
	case ID_BDSEPHEMERIS:
		return decode_bdsephemerisb(raw)
	case ID_NAVICEPHEMERIS:
		return decode_navicephemerisb(raw)
	}
	return 0
}

/* decode NovAtel OEM3 message -----------------------------------------------*/
func decode_oem3(raw *Raw) int {
	ctype := U4L(raw.Buff[4:])

	Trace(3, "decode_oem3: type=%3d len=%d\n", ctype, raw.Len)

	/* checksum */
	if csum8(raw.Buff[:], raw.Len) > 0 {
		Trace(2, "oem3 checksum error: type=%3d len=%d\n", ctype, raw.Len)
		return -1
	}
	if raw.OutType > 0 {
		copy(raw.MsgType[len(string(raw.MsgType[:])):], []byte(fmt.Sprintf("OEM3 %4d (%4d):", ctype, raw.Len)))
	}
	switch ctype {
	case ID_RGEB:
		return decode_rgeb(raw)
	case ID_RGED:
		return decode_rged(raw)
	case ID_REPB:
		return decode_repb(raw)
	case ID_FRMB:
		return decode_frmb(raw)
	case ID_IONB:
		return decode_ionb(raw)
	case ID_UTCB:
		return decode_utcb(raw)
	}
	return 0
}

/* sync header ---------------------------------------------------------------*/
func sync_oem4(buff []uint8, data uint8) int {
	buff[0] = buff[1]
	buff[1] = buff[2]
	buff[2] = data
	if buff[0] == OEM4SYNC1 && buff[1] == OEM4SYNC2 && buff[2] == OEM4SYNC3 {
		return 1
	}
	return 0
}
func sync_oem3(buff []uint8, data uint8) int {
	buff[0] = buff[1]
	buff[1] = buff[2]
	buff[2] = data
	if buff[0] == OEM3SYNC1 && buff[1] == OEM3SYNC2 && buff[2] == OEM3SYNC3 {
		return 1
	}
	return 0
}

/* input NovAtel OEM4/V/6/7 raw data from stream -------------------------------
* fetch next NovAtel OEM4/V/6/7 raw data and input a mesasge from stream
* args   : raw *Raw       IO  receiver raw data control struct
*          uint8_t data     I   stream data (1 byte)
* return : status (-1: error message, 0: no message, 1: input observation data,
*                  2: input ephemeris, 3: input sbas message,
*                  9: input ion/utc parameter)
*
* notes  : to specify input options for oem4, set raw.Opt to the following
*          option strings separated by spaces.
*
*          -EPHALL : input all ephemerides
*          -GL1L   : select 1L for GPS L1 (default 1C)
*          -GL2S   : select 2S for GPS L2 (default 2W)
*          -GL2P   : select 2P for GPS L2 (default 2W)
*          -RL2C   : select 2C for GLO G2 (default 2P)
*          -EL6B   : select 6B for GAL E6 (default 6C)
*          -JL1L   : select 1L for QZS L1 (default 1C)
*          -JL1Z   : select 1Z for QZS L1 (default 1C)
*          -CL1P   : select 1P for BDS B1 (default 2I)
*          -CL7D   : select 7D for BDS B2 (default 7I)
*          -GALINAV: select I/NAV for Galileo ephemeris (default: all)
*          -GALFNAV: select F/NAV for Galileo ephemeris (default: all)
*          -GLOBIAS=bias: GLONASS code-phase bias (m)
*-----------------------------------------------------------------------------*/
func Input_oem4(raw *Raw, data uint8) int {
	Trace(5, "input_oem4: data=%02x\n", data)

	/* synchronize frame */
	if raw.NumByte == 0 {
		if sync_oem4(raw.Buff[:], data) > 0 {
			raw.NumByte = 3
		}
		return 0
	}
	raw.Buff[raw.NumByte] = data
	raw.NumByte++
	raw.Len = int(U2L(raw.Buff[8:])) + OEM4HLEN
	if raw.NumByte == 10 && raw.Len > MAXRAWLEN-4 {
		Trace(2, "oem4 length error: len=%d\n", raw.Len)
		raw.NumByte = 0
		return -1
	}
	if raw.NumByte < 10 || raw.NumByte < raw.Len+4 {
		return 0
	}
	raw.NumByte = 0

	/* decode oem7/6/4 message */
	return decode_oem4(raw)
}

/* input NovAtel OEM3 raw data from stream -------------------------------------
* fetch next NovAtel OEM3 raw data and input a mesasge from stream
* args   : raw *Raw       IO  receiver raw data control struct
*          uint8_t data     I   stream data (1 byte)
* return : same as above
*-----------------------------------------------------------------------------*/
func Input_oem3(raw *Raw, data uint8) int {
	Trace(5, "input_oem3: data=%02x\n", data)

	/* synchronize frame */
	if raw.NumByte == 0 {
		if sync_oem3(raw.Buff[:], data) > 0 {
			raw.NumByte = 3
		}
		return 0
	}
	raw.Buff[raw.NumByte] = data
	raw.NumByte++
	raw.Len = int(U4L(raw.Buff[8:]))
	if raw.NumByte == 12 && raw.Len > MAXRAWLEN {
		Trace(2, "oem3 length error: len=%d\n", raw.Len)
		raw.NumByte = 0
		return -1
	}
	if raw.NumByte < 12 || raw.NumByte < raw.Len {
		return 0
	}
	raw.NumByte = 0

	/* decode oem3 message */
	return decode_oem3(raw)
}

/* input NovAtel OEM4/V/6/7 raw data from file ---------------------------------
* fetch next NovAtel OEM4/V/6/7 raw data and input a message from file
* args   : raw_t  *raw      IO  receiver raw data control struct
*          FILE   *fp       I   file pointer
* return : status(-2: end of file, -1...9: same as above)
*-----------------------------------------------------------------------------*/
func input_oem4f(raw *Raw, fp *os.File) int {

	Trace(4, "input_oem4f:\n")

	/* synchronize frame */
	var c [1]byte
	if raw.NumByte == 0 {
		for i := 0; ; i++ {
			_, err := fp.Read(c[:])
			if err == io.EOF {
				return -2
			}
			if sync_oem4(raw.Buff[:], uint8(c[0])) > 0 {
				break
			}
			if i >= 4096 {
				return 0
			}
		}
	}
	n, err := fp.Read(raw.Buff[3:10])
	if err == io.EOF || n < 7 {
		return -2
	}
	raw.NumByte = 10

	if raw.Len = int(U2L(raw.Buff[8:])) + OEM4HLEN; raw.Len > MAXRAWLEN-4 {
		Trace(2, "oem4 length error: len=%d\n", raw.Len)
		raw.NumByte = 0
		return -1
	}
	n, err = fp.Read(raw.Buff[10 : raw.Len+4])
	if n < raw.Len-6 {
		return -2
	}
	raw.NumByte = 0

	/* decode NovAtel OEM4/V/6/7 message */
	return decode_oem4(raw)
}

/* input NovAtel OEM3 raw data from file ---------------------------------------
* fetch next NovAtel OEM3 raw data and input a message from file
* args   : raw_t  *raw      IO  receiver raw data control struct
*          FILE   *fp       I   file pointer
* return : status(-2: end of file, -1...9: same as above)
*-----------------------------------------------------------------------------*/
func input_oem3f(raw *Raw, fp *os.File) int {

	Trace(4, "input_oem3f:\n")

	/* synchronize frame */
	var c [1]byte
	if raw.NumByte == 0 {
		for i := 0; ; i++ {
			_, err := fp.Read(c[:])
			if err == io.EOF {
				return -2
			}
			if sync_oem3(raw.Buff[:], uint8(c[0])) > 0 {
				break
			}
			if i >= 4096 {
				return 0
			}
		}
	}

	n, err := fp.Read(raw.Buff[3:12])
	if err == io.EOF || n < 9 {
		return -2
	}
	raw.NumByte = 12

	if raw.Len = int(U4L(raw.Buff[8:])); raw.Len > MAXRAWLEN {
		Trace(2, "oem3 length error: len=%d\n", raw.Len)
		raw.NumByte = 0
		return -1
	}
	n, err = fp.Read(raw.Buff[12:raw.Len])
	if n < raw.Len-12 {
		return -2
	}
	raw.NumByte = 0

	/* decode oem3 message */
	return decode_oem3(raw)
}
