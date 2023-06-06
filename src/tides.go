/*------------------------------------------------------------------------------
* tides.c : tidal displacement corrections
*
*          Copyright (C) 2015-2017 by T.TAKASU, All rights reserved.
*
* options : -DIERS_MODEL use IERS tide model
*
* references :
*     [1] D.D.McCarthy, IERS Technical Note 21, IERS Conventions 1996, July 1996
*     [2] D.D.McCarthy and G.Petit, IERS Technical Note 32, IERS Conventions
*         2003, November 2003
*     [3] D.A.Vallado, Fundamentals of Astrodynamics and Applications 2nd ed,
*         Space Technology Library, 2004
*     [4] J.Kouba, A Guide to using International GNSS Service (IGS) products,
*         May 2009
*     [5] G.Petit and B.Luzum (eds), IERS Technical Note No. 36, IERS
*         Conventions (2010), 2010
*
* version : $Revision:$ $Date:$
* history : 2015/05/10 1.0  separated from ppp.c
*           2015/06/11 1.1  fix bug on computing days in tide_oload() (#128)
*           2017/04/11 1.2  fix bug on calling geterp() in timdedisp()
*		    2022/05/31 1.0  rewrite tides.c with golang by fxb
*-----------------------------------------------------------------------------*/
package gnssgo

import (
	"math"
)

const (
	GME = 3.986004415e+14 /* earth gravitational constant */
	GMS = 1.327124e+20    /* sun gravitational constant */
	GMM = 4.902801e+12    /* moon gravitational constant */
)

/* function prototypes -------------------------------------------------------*/
//#ifdef IERS_MODEL
func dehanttideinel_(xsta []float64, year, mon, day []int,
	fhr, xsun, xmon, dxtide []float64) int {
	return 0
}

//#endif

/* solar/lunar tides (ref [2] 7) ---------------------------------------------*/
//#ifndef IERS_MODEL
func Tide_pl(eu, rp []float64, GMp float64, pos, dr []float64) {
	var (
		H3, L3                                                        float64 = 0.292, 0.015
		r, latp, lonp, p, K2, K3, a, H2, L2, dp, du, cosp, sinl, cosl float64
		ep                                                            [3]float64
		i                                                             int
	)

	Trace(4, "tide_pl : pos=%.3f %.3f\n", pos[0]*R2D, pos[1]*R2D)

	if r = Norm(rp, 3); r <= 0.0 {
		return
	}

	for i = 0; i < 3; i++ {
		ep[i] = rp[i] / r
	}

	K2 = GMp / GME * SQR(RE_WGS84) * SQR(RE_WGS84) / (r * r * r)
	K3 = K2 * RE_WGS84 / r
	latp = math.Asin(ep[2])
	lonp = math.Atan2(ep[1], ep[0])
	cosp = math.Cos(latp)
	sinl = math.Sin(pos[0])
	cosl = math.Cos(pos[0])

	/* step1 in phase (degree 2) */
	p = (3.0*sinl*sinl - 1.0) / 2.0
	H2 = 0.6078 - 0.0006*p
	L2 = 0.0847 + 0.0002*p
	a = Dot(ep[:], eu, 3)
	dp = K2 * 3.0 * L2 * a
	du = K2 * (H2*(1.5*a*a-0.5) - 3.0*L2*a*a)

	/* step1 in phase (degree 3) */
	dp += K3 * L3 * (7.5*a*a - 1.5)
	du += K3 * (H3*(2.5*a*a*a-1.5*a) - L3*(7.5*a*a-1.5)*a)

	/* step1 out-of-phase (only radial) */
	du += 3.0 / 4.0 * 0.0025 * K2 * math.Sin(2.0*latp) * math.Sin(2.0*pos[0]) * math.Sin(pos[1]-lonp)
	du += 3.0 / 4.0 * 0.0022 * K2 * cosp * cosp * cosl * cosl * math.Sin(2.0*(pos[1]-lonp))

	dr[0] = dp*ep[0] + du*eu[0]
	dr[1] = dp*ep[1] + du*eu[1]
	dr[2] = dp*ep[2] + du*eu[2]

	Trace(5, "tide_pl : dr=%.3f %.3f %.3f\n", dr[0], dr[1], dr[2])
}

/* displacement by solid earth tide (ref [2] 7) ------------------------------*/
func Tide_solid(rsun, rmoon, pos, E []float64, gmst float64, opt int, dr []float64) {
	var (
		dr1, dr2, eu        [3]float64
		du, dn, sinl, sin2l float64
	)

	Trace(3, "tide_solid: pos=%.3f %.3f opt=%d\n", pos[0]*R2D, pos[1]*R2D, opt)

	/* step1: time domain */
	eu[0] = E[2]
	eu[1] = E[5]
	eu[2] = E[8]
	Tide_pl(eu[:], rsun, GMS, pos, dr1[:])
	Tide_pl(eu[:], rmoon, GMM, pos, dr2[:])

	/* step2: frequency domain, only K1 radial */
	sin2l = math.Sin(2.0 * pos[0])
	du = -0.012 * sin2l * math.Sin(gmst+pos[1])

	dr[0] = dr1[0] + dr2[0] + du*E[2]
	dr[1] = dr1[1] + dr2[1] + du*E[5]
	dr[2] = dr1[2] + dr2[2] + du*E[8]

	/* eliminate permanent deformation */
	if opt&8 > 0 {
		sinl = math.Sin(pos[0])
		du = 0.1196 * (1.5*sinl*sinl - 0.5)
		dn = 0.0247 * sin2l
		dr[0] += du*E[2] + dn*E[1]
		dr[1] += du*E[5] + dn*E[4]
		dr[2] += du*E[8] + dn*E[7]
	}
	Trace(5, "tide_solid: dr=%.3f %.3f %.3f\n", dr[0], dr[1], dr[2])
}

// #endif /* !IERS_MODEL */

/* displacement by ocean tide loading (ref [2] 7) ----------------------------*/
func Tide_oload(tut Gtime, odisp, denu []float64) {
	var (
		args [][5]float64 = [][5]float64{
			{1.40519e-4, 2.0, -2.0, 0.0, 0.00},  /* M2 */
			{1.45444e-4, 0.0, 0.0, 0.0, 0.00},   /* S2 */
			{1.37880e-4, 2.0, -3.0, 1.0, 0.00},  /* N2 */
			{1.45842e-4, 2.0, 0.0, 0.0, 0.00},   /* K2 */
			{0.72921e-4, 1.0, 0.0, 0.0, 0.25},   /* K1 */
			{0.67598e-4, 1.0, -2.0, 0.0, -0.25}, /* O1 */
			{0.72523e-4, -1.0, 0.0, 0.0, -0.25}, /* P1 */
			{0.64959e-4, 1.0, -3.0, 1.0, -0.25}, /* Q1 */
			{0.53234e-5, 0.0, 2.0, 0.0, 0.00},   /* Mf */
			{0.26392e-5, 0.0, 1.0, -1.0, 0.00},  /* Mm */
			{0.03982e-5, 2.0, 0.0, 0.0, 0.00} /* Ssa */}
		ep1975                     []float64 = []float64{1975, 1, 1, 0, 0, 0}
		fday, days, t, t2, t3, ang float64
		ep                         [6]float64
		a                          [5]float64
		dp                         [3]float64
		i, j                       int
	)

	Trace(3, "tide_oload:\n")

	/* angular argument: see subroutine arg.f for reference [1] */
	Time2Epoch(tut, ep[:])
	fday = ep[3]*3600.0 + ep[4]*60.0 + ep[5]
	ep[3], ep[4], ep[5] = 0.0, 0.0, 0.0
	days = TimeDiff(Epoch2Time(ep[:]), Epoch2Time(ep1975))/86400.0 + 1.0
	t = (27392.500528 + 1.000000035*days) / 36525.0
	t2 = t * t
	t3 = t2 * t

	a[0] = fday
	a[1] = (279.69668 + 36000.768930485*t + 3.03e-4*t2) * D2R               /* H0 */
	a[2] = (270.434358 + 481267.88314137*t - 0.001133*t2 + 1.9e-6*t3) * D2R /* S0 */
	a[3] = (334.329653 + 4069.0340329577*t - 0.010325*t2 - 1.2e-5*t3) * D2R /* P0 */
	a[4] = 2.0 * PI

	/* displacements by 11 constituents */
	for i = 0; i < 11; i++ {
		ang = 0.0
		for j = 0; j < 5; j++ {
			ang += a[j] * args[i][j]
		}
		for j = 0; j < 3; j++ {
			dp[j] += odisp[j+i*6] * math.Cos(ang-odisp[j+3+i*6]*D2R)
		}
	}
	denu[0] = -dp[1]
	denu[1] = -dp[2]
	denu[2] = dp[0]

	Trace(5, "tide_oload: denu=%.3f %.3f %.3f\n", denu[0], denu[1], denu[2])
}

/* iers mean pole (ref [7] eq.7.25) ------------------------------------------*/
func Iers_mean_pole(tut Gtime, xp_bar, yp_bar *float64) {
	var (
		ep2000    []float64 = []float64{2000, 1, 1, 0, 0, 0}
		y, y2, y3 float64
	)
	y = TimeDiff(tut, Epoch2Time(ep2000)) / 86400.0 / 365.25

	if y < 3653.0/365.25 { /* until 2010.0 */
		y2 = y * y
		y3 = y2 * y
		*xp_bar = 55.974 + 1.8243*y + 0.18413*y2 + 0.007024*y3 /* (mas) */
		*yp_bar = 346.346 + 1.7896*y - 0.10729*y2 - 0.000908*y3
	} else { /* after 2010.0 */
		*xp_bar = 23.513 + 7.6141*y /* (mas) */
		*yp_bar = 358.891 - 0.6287*y
	}
}

/* displacement by pole tide (ref [7] eq.7.26) --------------------------------*/
func Tide_pole(tut Gtime, pos, erpv, denu []float64) {
	var xp_bar, yp_bar, m1, m2, cosl, sinl float64

	Trace(3, "tide_pole: pos=%.3f %.3f\n", pos[0]*R2D, pos[1]*R2D)

	/* iers mean pole (mas) */
	Iers_mean_pole(tut, &xp_bar, &yp_bar)

	/* ref [7] eq.7.24 */
	m1 = erpv[0]/AS2R - xp_bar*1e-3 /* (as) */
	m2 = -erpv[1]/AS2R + yp_bar*1e-3

	/* sin(2*theta) = sin(2*phi), cos(2*theta)=-cos(2*phi) */
	cosl = math.Cos(pos[1])
	sinl = math.Sin(pos[1])
	denu[0] = 9e-3 * math.Sin(pos[0]) * (m1*sinl - m2*cosl)       /* de= Slambda (m) */
	denu[1] = -9e-3 * math.Cos(2.0*pos[0]) * (m1*cosl + m2*sinl)  /* dn=-Stheta  (m) */
	denu[2] = -33e-3 * math.Sin(2.0*pos[0]) * (m1*cosl + m2*sinl) /* du= Sr      (m) */

	Trace(5, "tide_pole : denu=%.3f %.3f %.3f\n", denu[0], denu[1], denu[2])
}

/* tidal displacement ----------------------------------------------------------
* displacements by earth tides
* args   : gtime_t tutc     I   time in utc
*          double *rr       I   site position (ecef) (m)
*          int    opt       I   options (or of the followings)
*                                 1: solid earth tide
*                                 2: ocean tide loading
*                                 4: pole tide
*                                 8: elimate permanent deformation
*          double *erp      I   earth rotation parameters (NULL: not used)
*          double *odisp    I   ocean loading parameters  (NULL: not used)
*                                 odisp[0+i*6]: consituent i amplitude radial(m)
*                                 odisp[1+i*6]: consituent i amplitude west  (m)
*                                 odisp[2+i*6]: consituent i amplitude south (m)
*                                 odisp[3+i*6]: consituent i phase radial  (deg)
*                                 odisp[4+i*6]: consituent i phase west    (deg)
*                                 odisp[5+i*6]: consituent i phase south   (deg)
*                                (i=0:M2,1:S2,2:N2,3:K2,4:K1,5:O1,6:P1,7:Q1,
*                                   8:Mf,9:Mm,10:Ssa)
*          double *dr       O   displacement by earth tides (ecef) (m)
* return : none
* notes  : see ref [1], [2] chap 7
*          see ref [4] 5.2.1, 5.2.2, 5.2.3
*          ver.2.4.0 does not use ocean loading and pole tide corrections
*-----------------------------------------------------------------------------*/
func TideDisp(tutc Gtime, rr []float64, opt int, erp *Erp, odisp, dr []float64) {
	var (
		tut               Gtime
		pos               [2]float64
		E                 [9]float64
		drt, denu, rs, rm [3]float64
		gmst              float64
		erpv              [5]float64
		i                 int
	)
	// #ifdef IERS_MODEL
	//     double ep[6],fhr;
	//     int year,mon,day;
	// #endif

	Trace(3, "tidedisp: tutc=%s\n", TimeStr(tutc, 0))

	if erp != nil {
		GetErp(erp, Utc2GpsT(tutc), erpv[:])
	}
	tut = TimeAdd(tutc, erpv[2])

	dr[0], dr[1], dr[2] = 0.0, 0.0, 0.0

	if Norm(rr, 3) <= 0.0 {
		return
	}

	pos[0] = math.Asin(rr[2] / Norm(rr, 3))
	pos[1] = math.Atan2(rr[1], rr[0])
	XYZ2Enu(pos[:], E[:])

	if opt&1 > 0 { /* solid earth tides */

		/* sun and moon position in ecef */
		SunMoonPos(tutc, erpv[:], rs[:], rm[:], &gmst)

		// #ifdef IERS_MODEL
		//         time2epoch(tutc,ep);
		//         year=(int)ep[0];
		//         mon =(int)ep[1];
		//         day =(int)ep[2];
		//         fhr =ep[3]+ep[4]/60.0+ep[5]/3600.0;

		//         /* call DEHANTTIDEINEL */
		//         dehanttideinel_((double *)rr,&year,&mon,&day,&fhr,rs,rm,drt);
		// #else
		Tide_solid(rs[:], rm[:], pos[:], E[:], gmst, opt, drt[:])
		//#endif
		for i = 0; i < 3; i++ {
			dr[i] += drt[i]
		}
	}
	if (opt&2 > 0) && odisp != nil { /* ocean tide loading */
		Tide_oload(tut, odisp, denu[:])
		MatMul("TN", 3, 1, 3, 1.0, E[:], denu[:], 0.0, drt[:])
		for i = 0; i < 3; i++ {
			dr[i] += drt[i]
		}
	}
	if (opt&4 > 0) && erp != nil { /* pole tide */
		Tide_pole(tut, pos[:], erpv[:], denu[:])
		MatMul("TN", 3, 1, 3, 1.0, E[:], denu[:], 0.0, drt[:])
		for i = 0; i < 3; i++ {
			dr[i] += drt[i]
		}
	}
	Trace(5, "tidedisp: dr=%.3f %.3f %.3f\n", dr[0], dr[1], dr[2])
}
