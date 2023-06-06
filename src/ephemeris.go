/*------------------------------------------------------------------------------
* ephemeris.c : satellite ephemeris and clock functions
*
*          Copyright (C) 2010-2020 by T.TAKASU, All rights reserved.
*
* references :
*     [1] IS-GPS-200K, Navstar GPS Space Segment/Navigation User Interfaces,
*         May 6, 2019
*     [2] Global Navigation Satellite System GLONASS, Interface Control Document
*         Navigational radiosignal In bands L1, L2, (Version 5.1), 2008
*     [3] RTCA/DO-229C, Minimum operational performance standards for global
*         positioning system/wide area augmentation system airborne equipment,
*         RTCA inc, November 28, 2001
*     [4] RTCM Paper, April 12, 2010, Proposed SSR Messages for SV Orbit Clock,
*         Code Biases, URA
*     [5] RTCM Paper 012-2009-SC104-528, January 28, 2009 (previous ver of [4])
*     [6] RTCM Paper 012-2009-SC104-582, February 2, 2010 (previous ver of [4])
*     [7] European GNSS (Galileo) Open Service Signal In Space Interface Control
*         Document, Issue 1.3, December, 2016
*     [8] Quasi-Zenith Satellite System Interface Specification Satellite
*         Positioning, Navigation and Timing Service (IS-QZSS-PNT-003), Cabinet
*         Office, November 5, 2018
*     [9] BeiDou navigation satellite system signal in space interface control
*         document open service signal B1I (version 3.0), China Satellite
*         Navigation office, February, 2019
*     [10] RTCM Standard 10403.3, Differential GNSS (Global Navigation
*         Satellite Systems) Services - version 3, October 7, 2016
*
* version : $Revision:$ $Date:$
* history : 2010/07/28 1.1  moved from rtkcmn.c
*                           added api:
*                               eph2clk(),geph2clk(),seph2clk(),satantoff()
*                               satposs()
*                           changed api:
*                               eph2pos(),geph2pos(),satpos()
*                           deleted api:
*                               satposv(),satposiode()
*           2010/08/26 1.2  add ephemeris option EPHOPT_LEX
*           2010/09/09 1.3  fix problem when precise clock outage
*           2011/01/12 1.4  add api alm2pos()
*                           change api satpos(),satposs()
*                           enable valid unhealthy satellites and output status
*                           fix bug on exception by glonass ephem computation
*           2013/01/10 1.5  support beidou (compass)
*                           use newton's method to solve kepler eq.
*                           update ssr correction algorithm
*           2013/03/20 1.6  fix problem on ssr clock relativitic correction
*           2013/09/01 1.7  support negative pseudorange
*                           fix bug on variance in case of ura ssr = 63
*           2013/11/11 1.8  change constant MAXAGESSR 70.0 . 90.0
*           2014/10/24 1.9  fix bug on return of var_uraeph() if ura<0||15<ura
*           2014/12/07 1.10 modify MAXDTOE for qzss,gal and bds
*                           test max number of iteration for Kepler
*           2015/08/26 1.11 update RTOL_ELPLER 1E-14 . 1E-13
*                           set MAX_ITER_KEPLER for alm2pos()
*           2017/04/11 1.12 fix bug on max number of obs data in satposs()
*           2018/10/10 1.13 update reference [7]
*                           support ura value in var_uraeph() for galileo
*                           test eph.flag to recognize beidou geo
*                           add api satseleph() for ephemeris selection
*           2020/11/30 1.14 update references [1],[2],[8],[9] and [10]
*                           add API getseleph()
*                           rename API satseleph() as setseleph()
*                           support NavIC/IRNSS by API satpos() and satposs()
*                           support BDS C59-63 as GEO satellites in eph2pos()
*                           default selection of I/NAV for Galileo ephemeris
*                           no support EPHOPT_LEX by API satpos() and satposs()
*                           unselect Galileo ephemeris with AOD<=0 in seleph()
*                           fix bug on clock iteration in eph2clk(), geph2clk()
*                           fix bug on clock reference time in satpos_ssr()
*                           fix bug on wrong value with ura=15 in var_ura()
*                           use integer types in stdint.h
*		    2022/05/31 1.0  rewrite ephemeris.c with golang by fxb
*-----------------------------------------------------------------------------*/

package gnssgo

import (
	"math"
)

const (
	RE_GLO          = 6378136.0           /* radius of earth (m)            ref [2] */
	MU_GPS          = 3.9860050e14        /* gravitational constant         ref [1] */
	MU_GLO          = 3.9860044e14        /* gravitational constant         ref [2] */
	MU_GAL          = 3.986004418e14      /* earth gravitational constant   ref [7] */
	MU_CMP          = 3.986004418e14      /* earth gravitational constant   ref [9] */
	J2_GLO          = 1.0826257e-3        /* 2nd zonal harmonic of geopot   ref [2] */
	OMGE_GLO        = 7.292115e-5         /* earth angular velocity (rad/s) ref [2] */
	OMGE_GAL        = 7.2921151467e-5     /* earth angular velocity (rad/s) ref [7] */
	OMGE_CMP        = 7.292115e-5         /* earth angular velocity (rad/s) ref [9] */
	SIN_5           = -0.0871557427476582 /* sin(-5.0 deg) */
	COS_5           = 0.9961946980917456  /* cos(-5.0 deg) */
	Aref_MEO        = 27906100            /* support BDS-3 by cjb ref [2] */
	Aref_IGSO_GEO   = 42162200            /* support BDS-3 by cjb ref [2] */
	ERREPH_GLO      = 5.0                 /* error of glonass ephemeris (m) */
	TSTEP           = 60.0                /* integration step glonass ephemeris (s) */
	RTOL_KEPLER     = 1e-13               /* relative tolerance for Kepler equation */
	DEFURASSR       = 0.15                /* default accurary of ssr corr (m) */
	MAXECORSSR      = 10.0                /* max orbit correction of ssr (m) */
	MAXCCORSSR      = (1e-6 * CLIGHT)     /* max clock correction of ssr (m) */
	MAXAGESSR       = 90.0                /* max age of ssr orbit and clock (s) */
	MAXAGESSR_HRCLK = 10.0                /* max age of ssr high-rate clock (s) */
	STD_BRDCCLK     = 30.0                /* error of broadcast clock (m) */
	STD_GAL_NAPA    = 500.0               /* error of galileo ephemeris for NAPA (m) */
	MAX_ITER_KEPLER = 30 /* max number of iteration of Kelpler */)

/* ephemeris selections ------------------------------------------------------*/
var eph_sel [7]int = [7]int{ /* GPS,GLO,GAL,QZS,BDS,IRN,SBS */
	0, 0, 0, 0, 0, 0, 0}

/* variance by ura ephemeris -------------------------------------------------*/
func var_uraeph(sys, ura int) float64 {
	var ura_value []float64 = []float64{
		2.4, 3.4, 4.85, 6.85, 9.65, 13.65, 24.0, 48.0, 96.0, 192.0, 384.0, 768.0, 1536.0,
		3072.0, 6144.0}
	if sys == SYS_GAL { /* galileo sisa (ref [7] 5.1.11) */
		if ura <= 49 {
			return SQR(float64(ura) * 0.01)
		}
		if ura <= 74 {
			return SQR(0.5 + float64(ura-50)*0.02)
		}
		if ura <= 99 {
			return SQR(1.0 + float64(ura-75)*0.04)
		}
		if ura <= 125 {
			return SQR(2.0 + float64(ura-100)*0.16)
		}
		return SQR(STD_GAL_NAPA)
	} else { /* gps ura (ref [1] 20.3.3.3.1.1) */
		if ura < 0 || 14 < ura {
			return SQR(6144.0)
		}
		return SQR(ura_value[ura])
	}
}

/* variance by ura ssr (ref [10] table 3.3-1 DF389) --------------------------*/
func var_urassr(ura int) float64 {
	if ura <= 0 {
		return SQR(DEFURASSR)
	}
	if ura >= 63 {
		return SQR(5.4665)
	}
	std := (math.Pow(3.0, float64((ura>>3)&7))*(1.0+float64(ura&7)/4.0) - 1.0) * (1e-3)
	return SQR(std)
}

/* almanac to satellite position and clock bias --------------------------------
* compute satellite position and clock bias with almanac (gps, galileo, qzss)
* args   : gtime_t time     I   time (gpst)
*          alm_t *alm       I   almanac
*          double *rs       O   satellite position (ecef) {x,y,z} (m)
*          double *dts      O   satellite clock bias (s)
* return : none
* notes  : see ref [1],[7],[8]
*-----------------------------------------------------------------------------*/
func Alm2Pos(time Gtime, alm *Alm, rs []float64, dts *float64) {
	var (
		tk, M, E, Ek, sinE, cosE               float64
		u, r, i, O, x, y, sinO, cosO, cosi, mu float64
		n                                      int
	)

	Trace(4, "alm2pos : time=%s sat=%2d\n", TimeStr(time, 3), alm.Sat)

	tk = TimeDiff(time, alm.Toa)

	if alm.A <= 0.0 {
		rs[0], rs[1], rs[2], *dts = 0.0, 0.0, 0.0, 0.0
		return
	}
	if SatSys(alm.Sat, nil) == SYS_GAL {
		mu = MU_GAL
	} else {
		mu = MU_GPS
	}

	M = alm.M0 + math.Sqrt(mu/(alm.A*alm.A*alm.A))*tk
	E = M
	Ek = 0.0
	for n = 0; math.Abs(E-Ek) > RTOL_KEPLER && n < MAX_ITER_KEPLER; n++ {
		Ek = E
		E -= (E - alm.E*math.Sin(E) - M) / (1.0 - alm.E*math.Cos(E))
	}
	if n >= MAX_ITER_KEPLER {
		Trace(2, "alm2pos: kepler iteration overflow sat=%2d\n", alm.Sat)
		return
	}
	sinE = math.Sin(E)
	cosE = math.Cos(E)
	u = math.Atan2(math.Sqrt(1.0-alm.E*alm.E)*sinE, cosE-alm.E) + alm.Omg
	r = alm.A * (1.0 - alm.E*cosE)
	i = alm.I0
	O = alm.OMG0 + (alm.OMGd-OMGE)*tk - OMGE*alm.Toas
	x = r * math.Cos(u)
	y = r * math.Sin(u)
	sinO = math.Sin(O)
	cosO = math.Cos(O)
	cosi = math.Cos(i)
	rs[0] = x*cosO - y*cosi*sinO
	rs[1] = x*sinO + y*cosi*cosO
	rs[2] = y * math.Sin(i)
	*dts = alm.F0 + alm.F1*tk
}

/* broadcast ephemeris to satellite clock bias ---------------------------------
* compute satellite clock bias with broadcast ephemeris (gps, galileo, qzss)
* args   : gtime_t time     I   time by satellite clock (gpst)
*          eph_t *eph       I   broadcast ephemeris
* return : satellite clock bias (s) without relativeity correction
* notes  : see ref [1],[7],[8]
*          satellite clock does not include relativity correction and tdg
*-----------------------------------------------------------------------------*/
func Eph2Clk(time Gtime, eph *Eph) float64 {
	var t, ts float64

	Trace(4, "eph2clk : time=%s sat=%2d\n", TimeStr(time, 3), eph.Sat)

	t = TimeDiff(time, eph.Toc)
	ts = t

	for i := 0; i < 2; i++ {
		t = ts - (eph.F0 + eph.F1*t + eph.F2*t*t)
	}
	return eph.F0 + eph.F1*t + eph.F2*t*t
}

/* broadcast ephemeris to satellite position and clock bias --------------------
* compute satellite position and clock bias with broadcast ephemeris (gps,
* galileo, qzss)
* args   : gtime_t time     I   time (gpst)
*          eph_t *eph       I   broadcast ephemeris
*          double *rs       O   satellite position (ecef) {x,y,z} (m)
*          double *dts      O   satellite clock bias (s)
*          double *var      O   satellite position and clock variance (m^2)
* return : none
* notes  : see ref [1],[7],[8]
*          satellite clock includes relativity correction without code bias
*          (tgd or bgd)
*-----------------------------------------------------------------------------*/
func Eph2Pos(time Gtime, eph *Eph, rs []float64, dts, vari *float64) {
	var (
		tk, M, E, Ek, sinE, cosE, u, r, i, O           float64
		sin2u, cos2u, x, y, sinO, cosO, cosi, mu, omge float64
		xg, yg, zg, sino, coso                         float64
		n, sys, prn                                    int
		A, A0, deltNa, Na, N0                          float64
	)

	Trace(4, "eph2pos : time=%s sat=%2d\n", TimeStr(time, 3), eph.Sat)

	/* if (eph.A<=0.0) {
	   rs[0]=rs[1]=rs[2]=*dts=*var=0.0;
	   return;
	   }*/
	tk = TimeDiff(time, eph.Toe)

	switch sys = SatSys(eph.Sat, &prn); sys {
	case SYS_GAL:
		mu = MU_GAL
		omge = OMGE_GAL

	case SYS_CMP:
		mu = MU_CMP
		omge = OMGE_CMP

	default:
		mu = MU_GPS
		omge = OMGE

	}

	//M=eph.M0+(sqrt(mu/(eph.A*eph.A*eph.A))+eph.deln)*tk;

	if sys == SYS_CMP && (eph.Code == CODE_L1P || eph.Code == CODE_L8X) { //CODE_L1P B1C  CODE_L8X B2a support BDS-3 by cjb

		if eph.Flag == 1 { /*1:IGSO/MEO ????*/

			A0 = Aref_MEO + eph.A
		} else if eph.Flag == 2 { /*2:GEO*/

			A0 = Aref_IGSO_GEO + eph.A
		}
		A = math.Sqrt(A0 + eph.Adot*tk)

		N0 = math.Sqrt(mu / (A0 * A0 * A0))
		deltNa = eph.Deln + 1.0/2.0*eph.Ndot*tk
		Na = N0 + deltNa
		M = eph.M0 + Na*tk
	} else {
		A = eph.A
		M = eph.M0 + (math.Sqrt(mu/(eph.A*eph.A*eph.A))+eph.Deln)*tk
	}

	E = M
	Ek = 0.0
	for n = 0; math.Abs(E-Ek) > RTOL_KEPLER && n < MAX_ITER_KEPLER; n++ {
		Ek = E
		E -= (E - eph.E*math.Sin(E) - M) / (1.0 - eph.E*math.Cos(E))
	}
	if n >= MAX_ITER_KEPLER {
		Trace(2, "eph2pos: kepler iteration overflow sat=%2d\n", eph.Sat)
		return
	}
	sinE = math.Sin(E)
	cosE = math.Cos(E)

	Trace(5, "kepler: sat=%2d e=%8.5f n=%2d del=%10.3e\n", eph.Sat, eph.E, n, E-Ek)

	u = math.Atan2(math.Sqrt(1.0-eph.E*eph.E)*sinE, cosE-eph.E) + eph.Omg
	//r=eph.A*(1.0-eph.e*cosE);
	r = A * (1.0 - eph.E*cosE)
	i = eph.I0 + eph.Idot*tk
	sin2u = math.Sin(2.0 * u)
	cos2u = math.Cos(2.0 * u)
	u += eph.Cus*sin2u + eph.Cuc*cos2u
	r += eph.Crs*sin2u + eph.Crc*cos2u
	i += eph.Cis*sin2u + eph.Cic*cos2u
	x = r * math.Cos(u)
	y = r * math.Sin(u)
	cosi = math.Cos(i)

	/* beidou geo satellite */
	if sys == SYS_CMP && (prn <= 5 || prn >= 59) { /* ref [9] table 4-1 */
		O = eph.OMG0 + eph.OMGd*tk - omge*eph.Toes
		sinO = math.Sin(O)
		cosO = math.Cos(O)
		xg = x*cosO - y*cosi*sinO
		yg = x*sinO + y*cosi*cosO
		zg = y * math.Sin(i)
		sino = math.Sin(omge * tk)
		coso = math.Cos(omge * tk)
		rs[0] = xg*coso + yg*sino*COS_5 + zg*sino*SIN_5
		rs[1] = -xg*sino + yg*coso*COS_5 + zg*coso*SIN_5
		rs[2] = -yg*SIN_5 + zg*COS_5
	} else {
		O = eph.OMG0 + (eph.OMGd-omge)*tk - omge*eph.Toes
		sinO = math.Sin(O)
		cosO = math.Cos(O)
		rs[0] = x*cosO - y*cosi*sinO
		rs[1] = x*sinO + y*cosi*cosO
		rs[2] = y * math.Sin(i)
	}
	tk = TimeDiff(time, eph.Toc)
	*dts = eph.F0 + eph.F1*tk + eph.F2*tk*tk

	/* relativity correction */
	*dts -= 2.0 * math.Sqrt(mu*eph.A) * eph.E * sinE / SQR(CLIGHT)

	/* position and clock error variance */
	*vari = var_uraeph(sys, eph.Sva)
}

/* glonass orbit differential equations --------------------------------------*/
func Deq(x, xdot, acc []float64) {
	var a, b, c, r2, r3, omg2 float64
	r2 = Dot(x, x, 3)
	r3 = r2 * math.Sqrt(r2)
	omg2 = SQR(OMGE_GLO)

	if r2 <= 0.0 {
		xdot[0], xdot[1], xdot[2], xdot[3], xdot[4], xdot[5] = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
		return
	}
	/* ref [2] A.3.1.2 with bug fix for xdot[4],xdot[5] */
	a = 1.5 * J2_GLO * MU_GLO * SQR(RE_GLO) / r2 / r3 /* 3/2*J2*mu*Ae^2/r^5 */
	b = 5.0 * x[2] * x[2] / r2                        /* 5*z^2/r^2 */
	c = -MU_GLO/r3 - a*(1.0-b)                        /* -mu/r^3-a(1-b) */
	xdot[0] = x[3]
	xdot[1] = x[4]
	xdot[2] = x[5]
	xdot[3] = (c+omg2)*x[0] + 2.0*OMGE_GLO*x[4] + acc[0]
	xdot[4] = (c+omg2)*x[1] - 2.0*OMGE_GLO*x[3] + acc[1]
	xdot[5] = (c-2.0*a)*x[2] + acc[2]
}

/* glonass position and velocity by numerical integration --------------------*/
func Glorbit(t float64, x, acc []float64) {
	var (
		k1, k2, k3, k4, w [6]float64
		i                 int
	)
	Deq(x, k1[:], acc)
	for i = 0; i < 6; i++ {
		w[i] = x[i] + k1[i]*t/2.0
	}
	Deq(w[:], k2[:], acc)
	for i = 0; i < 6; i++ {
		w[i] = x[i] + k2[i]*t/2.0
	}
	Deq(w[:], k3[:], acc)
	for i = 0; i < 6; i++ {
		w[i] = x[i] + k3[i]*t
	}
	Deq(w[:], k4[:], acc)
	for i = 0; i < 6; i++ {
		x[i] += (k1[i] + 2.0*k2[i] + 2.0*k3[i] + k4[i]) * t / 6.0
	}
}

/* glonass ephemeris to satellite clock bias -----------------------------------
* compute satellite clock bias with glonass ephemeris
* args   : gtime_t time     I   time by satellite clock (gpst)
*          geph_t *geph     I   glonass ephemeris
* return : satellite clock bias (s)
* notes  : see ref [2]
*-----------------------------------------------------------------------------*/
func GEph2Clk(time Gtime, geph *GEph) float64 {
	var t, ts float64

	Trace(4, "geph2clk: time=%s sat=%2d\n", TimeStr(time, 3), geph.Sat)

	t = TimeDiff(time, geph.Toe)
	ts = t
	for i := 0; i < 2; i++ {
		t = ts - (-geph.Taun + geph.Gamn*t)
	}
	return -geph.Taun + geph.Gamn*t
}

/* glonass ephemeris to satellite position and clock bias ----------------------
* compute satellite position and clock bias with glonass ephemeris
* args   : gtime_t time     I   time (gpst)
*          geph_t *geph     I   glonass ephemeris
*          double *rs       O   satellite position {x,y,z} (ecef) (m)
*          double *dts      O   satellite clock bias (s)
*          double *var      O   satellite position and clock variance (m^2)
* return : none
* notes  : see ref [2]
*-----------------------------------------------------------------------------*/
func GEph2Pos(time Gtime, geph *GEph, rs []float64, dts, vari *float64) {
	var (
		t, tt float64
		x     [6]float64
	)

	Trace(4, "geph2pos: time=%s sat=%2d\n", TimeStr(time, 3), geph.Sat)

	t = TimeDiff(time, geph.Toe)

	*dts = -geph.Taun + geph.Gamn*t

	for i := 0; i < 3; i++ {
		x[i] = geph.Pos[i]
		x[i+3] = geph.Vel[i]
	}
	if t < 0.0 {
		tt = -TSTEP
	} else {
		tt = TSTEP
	}
	for ; math.Abs(t) > 1e-9; t -= tt {
		if math.Abs(t) < TSTEP {
			tt = t
		}
		Glorbit(tt, x[:], geph.Acc[:])
	}
	for i := 0; i < 3; i++ {
		rs[i] = x[i]
	}

	*vari = SQR(ERREPH_GLO)
}

/* sbas ephemeris to satellite clock bias --------------------------------------
* compute satellite clock bias with sbas ephemeris
* args   : gtime_t time     I   time by satellite clock (gpst)
*          seph_t *seph     I   sbas ephemeris
* return : satellite clock bias (s)
* notes  : see ref [3]
*-----------------------------------------------------------------------------*/
func SEph2Clk(time Gtime, seph *SEph) float64 {

	Trace(4, "seph2clk: time=%s sat=%2d\n", TimeStr(time, 3), seph.Sat)

	t := TimeDiff(time, seph.T0)

	for i := 0; i < 2; i++ {
		t -= seph.Af0 + seph.Af1*t
	}
	return seph.Af0 + seph.Af1*t
}

/* sbas ephemeris to satellite position and clock bias -------------------------
* compute satellite position and clock bias with sbas ephemeris
* args   : gtime_t time     I   time (gpst)
*          seph_t  *seph    I   sbas ephemeris
*          double  *rs      O   satellite position {x,y,z} (ecef) (m)
*          double  *dts     O   satellite clock bias (s)
*          double  *var     O   satellite position and clock variance (m^2)
* return : none
* notes  : see ref [3]
*-----------------------------------------------------------------------------*/
func SEph2Pos(time Gtime, seph *SEph, rs []float64, dts, vari *float64) {

	Trace(4, "seph2pos: time=%s sat=%2d\n", TimeStr(time, 3), seph.Sat)

	t := TimeDiff(time, seph.T0)

	for i := 0; i < 3; i++ {
		rs[i] = seph.Pos[i] + seph.Vel[i]*t + seph.Acc[i]*t*t/2.0
	}
	*dts = seph.Af0 + seph.Af1*t

	*vari = var_uraeph(SYS_SBS, seph.Sva)
}

/* select ephememeris --------------------------------------------------------*/
func (nav *Nav) SelEph(time Gtime, sat, iode int) *Eph {
	var (
		t, tmax, tmin  float64
		i, j, sys, sel int
		ret            *Eph = nil
	)
	j = -1

	Trace(4, "seleph  : time=%s sat=%2d iode=%d\n", TimeStr(time, 3), sat, iode)

	sys = SatSys(sat, nil)
	switch sys {
	case SYS_GPS:
		tmax = float64(MAXDTOE + 1.0)
		// sel = eph_sel[0]

	case SYS_GAL:
		tmax = float64(MAXDTOE_GAL)
		// sel = eph_sel[2]

	case SYS_QZS:
		tmax = float64(MAXDTOE_QZS + 1.0)
		// sel = eph_sel[3]

	case SYS_CMP:
		tmax = float64(MAXDTOE_CMP + 1.0)
		// sel = eph_sel[4]

	case SYS_IRN:
		tmax = float64(MAXDTOE_IRN + 1.0)
		// sel = eph_sel[5]

	default:
		tmax = float64(MAXDTOE + 1.0)

	}
	tmin = tmax + 1.0

	for i = 0; i < nav.N(); i++ {
		if nav.Ephs[i].Sat != sat {
			continue
		}
		if iode >= 0 && nav.Ephs[i].Iode != iode {
			continue
		}
		if sys == SYS_GAL {
			sel = GetSelEph(SYS_GAL)
			if sel == 0 && (nav.Ephs[i].Code&(1<<9)) == 0 {
				continue
			} /* I/NAV */
			if sel == 1 && (nav.Ephs[i].Code&(1<<8)) == 0 {
				continue
			} /* F/NAV */
			if TimeDiff(nav.Ephs[i].Toe, time) >= 0.0 {
				continue
			} /* AOD<=0 */
		}
		t = math.Abs(TimeDiff(nav.Ephs[i].Toe, time))
		if t > tmax {
			continue
		}
		if iode >= 0 {
			ret = &nav.Ephs[i]
			return ret
		}
		if t <= tmin {
			j = i
			tmin = t
		} /* toe closest to time */
	}
	if iode >= 0 || j < 0 {
		Trace(3, "no broadcast ephemeris: %s sat=%2d iode=%3d\n",
			TimeStr(time, 0), sat, iode)
		return ret
	}
	ret = &nav.Ephs[j]
	Trace(5, "found eph sat =%d j=%d\n", sat, j)
	return ret
}

/* select glonass ephememeris ------------------------------------------------*/
func (nav *Nav) SelGEph(time Gtime, sat, iode int) *GEph {
	var (
		t          float64
		tmax, tmin float64 = MAXDTOE_GLO, MAXDTOE_GLO + 1.0
		i, j       int     = 0, -1
		ret        *GEph   = nil
	)

	Trace(4, "selgeph : time=%s sat=%2d iode=%2d\n", TimeStr(time, 3), sat, iode)

	for i = 0; i < nav.Ng(); i++ {
		if nav.Geph[i].Sat != sat {
			continue
		}
		if iode >= 0 && nav.Geph[i].Iode != iode {
			continue
		}
		if t = math.Abs(TimeDiff(nav.Geph[i].Toe, time)); t > tmax {
			continue
		}
		if iode >= 0 {
			ret = &nav.Geph[i]
			return ret
		}
		if t <= tmin {
			j = i
			tmin = t
		} /* toe closest to time */
	}
	if iode >= 0 || j < 0 {
		Trace(3, "no glonass ephemeris  : %s sat=%2d iode=%2d\n", TimeStr(time, 0),
			sat, iode)
		return ret
	}
	ret = &nav.Geph[j]
	return ret
}

/* select sbas ephememeris ---------------------------------------------------*/
func (nav *Nav) SelSEph(time Gtime, sat int) *SEph {
	var (
		t          float64
		tmax, tmin float64 = MAXDTOE_SBS, MAXDTOE_SBS + 1.0
		i, j       int     = 0, -1
		ret        *SEph   = nil
	)
	Trace(4, "selseph : time=%s sat=%2d\n", TimeStr(time, 3), sat)

	for i = 0; i < nav.Ns(); i++ {
		if nav.Seph[i].Sat != sat {
			continue
		}
		if t = math.Abs(TimeDiff(nav.Seph[i].T0, time)); t > tmax {
			continue
		}
		if t <= tmin {
			j = i
			tmin = t
		} /* toe closest to time */
	}
	if j < 0 {
		Trace(3, "no sbas ephemeris     : %s sat=%2d\n", TimeStr(time, 0), sat)
		return nil
	}
	ret = &nav.Seph[j]
	return ret
}

/* satellite clock with broadcast ephemeris ----------------------------------*/
func (nav *Nav) EphClk(time, teph Gtime, sat int, dts *float64) int {
	var (
		eph  *Eph  = nil
		geph *GEph = nil
		seph *SEph = nil
		sys  int
	)

	Trace(4, "ephclk  : time=%s sat=%2d\n", TimeStr(time, 3), sat)

	sys = SatSys(sat, nil)

	switch sys {
	case SYS_GPS, SYS_GAL, SYS_QZS, SYS_CMP, SYS_IRN:
		if eph = nav.SelEph(teph, sat, -1); eph == nil {
			return 0
		}
		*dts = Eph2Clk(time, eph)
	case SYS_GLO:
		if geph = nav.SelGEph(teph, sat, -1); geph == nil {
			return 0
		}
		*dts = GEph2Clk(time, geph)
	case SYS_SBS:
		if seph = nav.SelSEph(teph, sat); seph == nil {
			return 0
		}
		*dts = SEph2Clk(time, seph)
	default:
		return 0
	}

	return 1
}

/* satellite position and clock by broadcast ephemeris -----------------------*/
func (nav *Nav) EphPos(time, teph Gtime, sat int,
	iode int, rs, dts []float64, vari *float64, svh *int) int {
	var (
		eph    *Eph  = nil
		geph   *GEph = nil
		seph   *SEph = nil
		rst    [3]float64
		dtst   [1]float64
		tt     = 1e-3
		i, sys int
	)
	Trace(4, "ephpos  : time=%s sat=%2d iode=%d\n", TimeStr(time, 3), sat, iode)

	sys = SatSys(sat, nil)

	*svh = -1

	switch sys {
	case SYS_GPS, SYS_GAL, SYS_QZS, SYS_CMP, SYS_IRN:
		if eph = nav.SelEph(teph, sat, iode); eph == nil {
			return 0
		}
		Eph2Pos(time, eph, rs, &dts[0], vari)
		time = TimeAdd(time, tt)
		Eph2Pos(time, eph, rst[:], &dtst[0], vari)
		*svh = eph.Svh
	case SYS_GLO:
		if geph = nav.SelGEph(teph, sat, iode); geph == nil {
			return 0
		}
		GEph2Pos(time, geph, rs, &dts[0], vari)
		time = TimeAdd(time, tt)
		GEph2Pos(time, geph, rst[:], &dtst[0], vari)
		*svh = geph.Svh
	case SYS_SBS:
		if seph = nav.SelSEph(teph, sat); seph == nil {
			return 0
		}
		SEph2Pos(time, seph, rs, &dts[0], vari)
		time = TimeAdd(time, tt)
		SEph2Pos(time, seph, rst[:], &dtst[0], vari)
		*svh = seph.Svh
	default:
		return 0
	}

	/* satellite velocity and clock drift by differential approx */
	for i = 0; i < 3; i++ {
		rs[i+3] = (rst[i] - rs[i]) / tt
	}
	dts[1] = (dtst[0] - dts[0]) / tt

	return 1
}

/* satellite position and clock with sbas correction -------------------------*/
func (nav *Nav) SatPosSbas(time, teph Gtime, sat int,
	rs, dts []float64, vari *float64, svh *int) int {
	var (
		sbs SbsSatP
		i   int
	)

	Trace(4, "satpos_sbas: time=%s sat=%2d\n", TimeStr(time, 3), sat)

	/* search sbas satellite correciton */
	for i = 0; i < nav.SbasSat.nsat; i++ {
		sbs = nav.SbasSat.sat[i]
		if sbs.sat == sat {
			break
		}
	}
	if i >= nav.SbasSat.nsat {
		Trace(2, "no sbas correction for orbit: %s sat=%2d\n", TimeStr(time, 0), sat)
		nav.EphPos(time, teph, sat, -1, rs, dts, vari, svh)
		*svh = -1
		return 0
	}
	/* satellite postion and clock by broadcast ephemeris */
	if nav.EphPos(time, teph, sat, sbs.lcorr.iode, rs, dts, vari, svh) == 0 {
		return 0
	}

	/* sbas satellite correction (long term and fast) */
	if SbsSatCorr(time, sat, nav, rs, dts, vari) > 0 {
		return 1
	}
	*svh = -1
	return 0
}

/* satellite position and clock with ssr correction --------------------------*/
func (nav *Nav) SatPosSsr(time, teph Gtime, sat int,
	opt int, rs, dts []float64, vari *float64, svh *int) int {
	var (
		ssr                        *SSR
		eph                        *Eph
		t1, t2, t3, dclk, tk       float64
		er, ea, ec, rc, deph, dant [3]float64
		i, sys                     int
	)

	Trace(4, "satpos_ssr: time=%s sat=%2d\n", TimeStr(time, 3), sat)

	ssr = &nav.Ssr[sat]

	if ssr.T0[0].Time == 0 {
		Trace(2, "no ssr orbit correction: %s sat=%2d\n", TimeStr(time, 0), sat)
		return 0
	}
	if ssr.T0[1].Time == 0 {
		Trace(2, "no ssr clock correction: %s sat=%2d\n", TimeStr(time, 0), sat)
		return 0
	}
	/* inconsistency between orbit and clock correction */
	if ssr.Iod[0] != ssr.Iod[1] {
		Trace(2, "inconsist ssr correction: %s sat=%2d iod=%d %d\n",
			TimeStr(time, 0), sat, ssr.Iod[0], ssr.Iod[1])
		*svh = -1
		return 0
	}
	t1 = TimeDiff(time, ssr.T0[0])
	t2 = TimeDiff(time, ssr.T0[1])
	t3 = TimeDiff(time, ssr.T0[2])

	/* ssr orbit and clock correction (ref [4]) */
	if math.Abs(t1) > MAXAGESSR || math.Abs(t2) > MAXAGESSR {
		Trace(2, "age of ssr error: %s sat=%2d t=%.0f %.0f\n", TimeStr(time, 0),
			sat, t1, t2)
		*svh = -1
		return 0
	}
	if ssr.Udi[0] >= 1.0 {
		t1 -= ssr.Udi[0] / 2.0
	}
	if ssr.Udi[1] >= 1.0 {
		t2 -= ssr.Udi[1] / 2.0
	}

	for i = 0; i < 3; i++ {
		deph[i] = ssr.Deph[i] + ssr.Ddeph[i]*t1
	}
	dclk = ssr.Dclk[0] + ssr.Dclk[1]*t2 + ssr.Dclk[2]*t2*t2

	/* ssr highrate clock correction (ref [4]) */
	if ssr.Iod[0] == ssr.Iod[2] && ssr.T0[2].Time > 0 && math.Abs(t3) < MAXAGESSR_HRCLK {
		dclk += ssr.Brclk
	}
	if Norm(deph[:], 3) > MAXECORSSR || math.Abs(dclk) > MAXCCORSSR {
		Trace(2, "invalid ssr correction: %s deph=%.1f dclk=%.1f\n",
			TimeStr(time, 0), Norm(deph[:], 3), dclk)
		*svh = -1
		return 0
	}
	/* satellite postion and clock by broadcast ephemeris */
	if nav.EphPos(time, teph, sat, ssr.Iode, rs, dts, vari, svh) == 0 {
		return 0
	}

	/* satellite clock for gps, galileo and qzss */
	sys = SatSys(sat, nil)
	if sys == SYS_GPS || sys == SYS_GAL || sys == SYS_QZS || sys == SYS_CMP {
		if eph = nav.SelEph(teph, sat, ssr.Iode); eph == nil {
			return 0
		}

		/* satellite clock by clock parameters */
		tk = TimeDiff(time, eph.Toc)
		dts[0] = eph.F0 + eph.F1*tk + eph.F2*tk*tk
		dts[1] = eph.F1 + 2.0*eph.F2*tk

		/* relativity correction */
		dts[0] -= 2.0 * Dot(rs[:], rs[3:], 3) / CLIGHT / CLIGHT
	}
	/* radial-along-cross directions in ecef */
	if NormV3(rs[3:], ea[:]) == 0 {
		return 0
	}
	Cross3(rs, rs[3:], rc[:])
	if NormV3(rc[:], ec[:]) == 0 {
		*svh = -1
		return 0
	}
	Cross3(ea[:], ec[:], er[:])

	/* satellite antenna offset correction */
	if opt > 0 {
		nav.SatAntOffset(time, rs, sat, dant[:])
	}
	for i = 0; i < 3; i++ {
		rs[i] += -(er[i]*deph[0] + ea[i]*deph[1] + ec[i]*deph[2]) + dant[i]
	}
	/* t_corr = t_sv - (dts(brdc) + dclk(ssr) / CLIGHT) (ref [10] eq.3.12-7) */
	dts[0] += dclk / CLIGHT

	/* variance by ssr ura */
	*vari = var_urassr(ssr.Ura)

	Trace(5, "satpos_ssr: %s sat=%2d deph=%6.3f %6.3f %6.3f er=%6.3f %6.3f %6.3f dclk=%6.3f var=%6.3f\n",
		TimeStr(time, 2), sat, deph[0], deph[1], deph[2], er[0], er[1], er[2], dclk, *vari)

	return 1
}

/* satellite position and clock ------------------------------------------------
* compute satellite position, velocity and clock
* args   : gtime_t time     I   time (gpst)
*          gtime_t teph     I   time to select ephemeris (gpst)
*          int    sat       I   satellite number
*          nav_t  *nav      I   navigation data
*          int    ephopt    I   ephemeris option (EPHOPT_???)
*          double *rs       O   sat position and velocity (ecef)
*                               {x,y,z,vx,vy,vz} (m|m/s)
*          double *dts      O   sat clock {bias,drift} (s|s/s)
*          double *var      O   sat position and clock error variance (m^2)
*          int    *svh      O   sat health flag (-1:correction not available)
* return : status (1:ok,0:error)
* notes  : satellite position is referenced to antenna phase center
*          satellite clock does not include code bias correction (tgd or bgd)
*-----------------------------------------------------------------------------*/
func (nav *Nav) SatPos(time, teph Gtime, sat, ephopt int,
	rs, dts []float64, vari *float64, svh *int) int {
	Trace(4, "satpos  : time=%s sat=%2d ephopt=%d\n", TimeStr(time, 3), sat, ephopt)

	*svh = 0

	switch byte(ephopt) {
	case EPHOPT_BRDC:
		return nav.EphPos(time, teph, sat, -1, rs, dts, vari, svh)
	case EPHOPT_SBAS:
		return nav.SatPosSbas(time, teph, sat, rs, dts, vari, svh)
	case EPHOPT_SSRAPC:
		return nav.SatPosSsr(time, teph, sat, 0, rs, dts, vari, svh)
	case EPHOPT_SSRCOM:
		return nav.SatPosSsr(time, teph, sat, 1, rs, dts, vari, svh)
	case EPHOPT_PREC:
		if nav.PEph2Pos(time, sat, 1, rs, dts, vari) == 0 {
			break
		} else {
			return 1
		}
	}
	*svh = -1
	return 0
}

/* satellite positions and clocks ----------------------------------------------
* compute satellite positions, velocities and clocks
* args   : gtime_t teph     I   time to select ephemeris (gpst)
*          obsd_t *obs      I   observation data
*          int    n         I   number of observation data
*          nav_t  *nav      I   navigation data
*          int    ephopt    I   ephemeris option (EPHOPT_???)
*          double *rs       O   satellite positions and velocities (ecef)
*          double *dts      O   satellite clocks
*          double *var      O   sat position and clock error variances (m^2)
*          int    *svh      O   sat health flag (-1:correction not available)
* return : none
* notes  : rs [(0:2)+i*6]= obs[i] sat position {x,y,z} (m)
*          rs [(3:5)+i*6]= obs[i] sat velocity {vx,vy,vz} (m/s)
*          dts[(0:1)+i*2]= obs[i] sat clock {bias,drift} (s|s/s)
*          var[i]        = obs[i] sat position and clock error variance (m^2)
*          svh[i]        = obs[i] sat health flag
*          if no navigation data, set 0 to rs[], dts[], var[] and svh[]
*          satellite position and clock are values at signal transmission time
*          satellite position is referenced to antenna phase center
*          satellite clock does not include code bias correction (tgd or bgd)
*          any pseudorange and broadcast ephemeris are always needed to get
*          signal transmission time
*-----------------------------------------------------------------------------*/
func (nav *Nav) SatPoss(teph Gtime, obs []ObsD, n int,
	ephopt int, rs, dts []float64, vari []float64, svh []int) {
	var (
		time   [2 * MAXOBS]Gtime
		dt, pr float64
		i, j   int
	)

	Trace(4, "satposs : teph=%s n=%d ephopt=%d\n", TimeStr(teph, 3), n, ephopt)

	for i = 0; i < n && i < 2*MAXOBS; i++ {
		for j = 0; j < 6; j++ {
			rs[j+i*6] = 0.0
		}
		for j = 0; j < 2; j++ {
			dts[j+i*2] = 0.0
		}
		vari[i] = 0.0
		svh[i] = 0

		/* search any pseudorange */
		pr = 0.0
		for j = 0; j < NFREQ; j++ {
			if pr = obs[i].P[j]; pr != 0.0 {
				break
			}
		}

		if j >= NFREQ {
			Trace(2, "no pseudorange %s sat=%2d\n", TimeStr(obs[i].Time, 3), obs[i].Sat)
			continue
		}
		/* transmission time by satellite clock */
		time[i] = TimeAdd(obs[i].Time, -pr/CLIGHT)

		/* satellite clock bias by broadcast ephemeris */
		if nav.EphClk(time[i], teph, int(obs[i].Sat), &dt) == 0 {
			Trace(2, "no broadcast clock %s sat=%2d\n", TimeStr(time[i], 3), obs[i].Sat)
			continue
		}
		time[i] = TimeAdd(time[i], -dt)

		/* satellite position and clock at transmission time */
		if nav.SatPos(time[i], teph, int(obs[i].Sat), ephopt, rs[i*6:], dts[i*2:], &vari[i], &svh[i]) == 0 {
			Trace(2, "no ephemeris %s sat=%2d\n", TimeStr(time[i], 3), obs[i].Sat)
			continue
		}
		/* if no precise clock available, use broadcast clock instead */
		if dts[i*2] == 0.0 {
			if nav.EphClk(time[i], teph, int(obs[i].Sat), &dts[i*2]) == 0 {
				continue
			}
			dts[1+i*2] = 0.0
			vari[i] = SQR(STD_BRDCCLK)
		}
	}
	for i = 0; i < n && i < 2*MAXOBS; i++ {
		Trace(5, "%s sat=%2d rs=%13.3f %13.3f %13.3f dts=%12.3f var=%7.3f svh=%02X\n",
			TimeStr(time[i], 6), obs[i].Sat, rs[i*6], rs[1+i*6], rs[2+i*6],
			dts[i*2]*1e9, vari[i], svh[i])
	}
}

/* set selected satellite ephemeris --------------------------------------------
* Set selected satellite ephemeris for multiple ones like LNAV - CNAV, I/NAV -
* F/NAV. Call it before calling satpos(),satposs() to use unselected one.
* args   : int    sys       I   satellite system (SYS_???)
*          int    sel       I   selection of ephemeris
*                                 GPS,QZS : 0:LNAV ,1:CNAV  (default: LNAV)
*                                 GAL     : 0:I/NAV,1:F/NAV (default: I/NAV)
*                                 others  : undefined
* return : none
* notes  : default ephemeris selection for galileo is any.
*-----------------------------------------------------------------------------*/
func SetSelEph(sys, sel int) {
	switch sys {
	case SYS_GPS:
		eph_sel[0] = sel

	case SYS_GLO:
		eph_sel[1] = sel

	case SYS_GAL:
		eph_sel[2] = sel

	case SYS_QZS:
		eph_sel[3] = sel

	case SYS_CMP:
		eph_sel[4] = sel

	case SYS_IRN:
		eph_sel[5] = sel

	case SYS_SBS:
		eph_sel[6] = sel

	}
}

/* get selected satellite ephemeris -------------------------------------------
* Get the selected satellite ephemeris.
* args   : int    sys       I   satellite system (SYS_???)
* return : selected ephemeris
*            refer setseleph()
*-----------------------------------------------------------------------------*/
func GetSelEph(sys int) int {
	switch sys {
	case SYS_GPS:
		return eph_sel[0]
	case SYS_GLO:
		return eph_sel[1]
	case SYS_GAL:
		return eph_sel[2]
	case SYS_QZS:
		return eph_sel[3]
	case SYS_CMP:
		return eph_sel[4]
	case SYS_IRN:
		return eph_sel[5]
	case SYS_SBS:
		return eph_sel[6]
	}
	return 0
}
