/*------------------------------------------------------------------------------
* ppp.c : precise point positioning
*
*          Copyright (C) 2010-2020 by T.TAKASU, All rights reserved.
*
* options : -DIERS_MODEL  use IERS tide model
*           -DOUTSTAT_AMB output ambiguity parameters to solution status
*
* references :
*    [1] D.D.McCarthy, IERS Technical Note 21, IERS Conventions 1996, July 1996
*    [2] D.D.McCarthy and G.Petit, IERS Technical Note 32, IERS Conventions
*        2003, November 2003
*    [3] D.A.Vallado, Fundamentals of Astrodynamics and Applications 2nd ed,
*        Space Technology Library, 2004
*    [4] J.Kouba, A Guide to using International GNSS Service (IGS) products,
*        May 2009
*    [5] RTCM Paper, April 12, 2010, Proposed SSR Messages for SV Orbit Clock,
*        Code Biases, URA
*    [6] MacMillan et al., Atmospheric gradients and the VLBI terrestrial and
*        celestial reference frames, Geophys. Res. Let., 1997
*    [7] G.Petit and B.Luzum (eds), IERS Technical Note No. 36, IERS
*         Conventions (2010), 2010
*    [8] J.Kouba, A simplified yaw-attitude model for eclipsing GPS satellites,
*        GPS Solutions, 13:1-12, 2009
*    [9] F.Dilssner, GPS IIF-1 satellite antenna phase center and attitude
*        modeling, InsideGNSS, September, 2010
*    [10] F.Dilssner, The GLONASS-M satellite yaw-attitude model, Advances in
*        Space Research, 2010
*    [11] IGS MGEX (http://igs.org/mgex)
*
* version : $Revision:$ $Date:$
* history : 2010/07/20 1.0  new
*                           added api:
*                               tidedisp()
*           2010/12/11 1.1  enable exclusion of eclipsing satellite
*           2012/02/01 1.2  add gps-glonass h/w bias correction
*                           move windupcorr() to rtkcmn.c
*           2013/03/11 1.3  add otl and pole tides corrections
*                           involve iers model with -DIERS_MODEL
*                           change initial variances
*                           suppress acos domain error
*           2013/09/01 1.4  pole tide model by iers 2010
*                           add mode of ionosphere model off
*           2014/05/23 1.5  add output of trop gradient in solution status
*           2014/10/13 1.6  fix bug on P0(a[3]) computation in tide_oload()
*                           fix bug on m2 computation in tide_pole()
*           2015/03/19 1.7  fix bug on ionosphere correction for GLO and BDS
*           2015/05/10 1.8  add function to detect slip by MW-LC jump
*                           fix ppp solutin problem with large clock variance
*           2015/06/08 1.9  add precise satellite yaw-models
*                           cope with day-boundary problem of satellite clock
*           2015/07/31 1.10 fix bug on nan-solution without glonass nav-data
*                           pppoutsolsat() . pppoutstat()
*           2015/11/13 1.11 add L5-receiver-dcb estimation
*                           merge post-residual validation by rnx2rtkp_test
*                           support support option opt.pppopt=-GAP_RESION=nnnn
*           2016/01/22 1.12 delete support for yaw-model bug
*                           add support for ura of ephemeris
*           2018/10/10 1.13 support api change of satexclude()
*           2020/11/30 1.14 use sat2freq() to get carrier frequency
*                           use E1-E5b for Galileo iono-free LC
*		    2022/05/31 1.0  rewrite ppp.c with golang by fxb
*-----------------------------------------------------------------------------*/
package gnssgo

import (
	"fmt"
	"math"
	"strings"
)

const (
	MAX_ITER     = 8    /* max number of iterations */
	MAX_STD_FIX  = 0.15 /* max std-dev (3d) to fix solution */
	MIN_NSAT_SOL = 4    /* min satellite number for solution */
	THRES_REJECT = 4.0  /* reject threshold of posfit-res (sigma) */

	THRES_MW_JUMP         = 10.0
	GAP_RESION    int     = 120           /* default gap to reset ionos parameters (ep) */
	EFACT_GPS_L5  float64 = 10.0          /* error factor of GPS/QZS L5 */
	MUDOT_GPS     float64 = 0.00836 * D2R /* average angular velocity GPS (rad/s) */
	MUDOT_GLO     float64 = 0.00888 * D2R /* average angular velocity GLO (rad/s) */
	EPS0_GPS      float64 = 13.5 * D2R    /* max shadow crossing angle GPS (rad) */
	EPS0_GLO      float64 = 14.2 * D2R    /* max shadow crossing angle GLO (rad) */
	T_POSTSHADOW  float64 = 1800.0        /* post-shadow recovery time (s) */
	QZS_EC_BETA   float64 = 20.0 /* max beta angle for qzss Ec (deg) */)

var (
	VAR_POS     float64 = SQR(60.0) /* init variance receiver position (m^2) */
	VAR_VEL     float64 = SQR(10.0) /* init variance of receiver vel ((m/s)^2) */
	VAR_ACC     float64 = SQR(10.0) /* init variance of receiver acc ((m/ss)^2) */
	VAR_CLK     float64 = SQR(60.0) /* init variance receiver clock (m^2) */
	VAR_ZTD     float64 = SQR(0.6)  /* init variance ztd (m^2) */
	VAR_GRA     float64 = SQR(0.01) /* init variance gradient (m^2) */
	VAR_DCB     float64 = SQR(30.0) /* init variance dcb (m^2) */
	VAR_BIAS    float64 = SQR(60.0) /* init variance phase-bias (m^2) */
	VAR_IONO    float64 = SQR(60.0) /* init variance iono-delay */
	VAR_GLO_IFB float64 = SQR(0.6) /* variance of glonass ifb */)

// const ERR_SAAS float64 = 0.3  /* saastamoinen model error std (m) */
// const ERR_BRDCI float64 = 0.5 /* broadcast iono model error factor */
// const ERR_CBIAS float64 = 0.3 /* code bias error std (m) */
// const REL_HUMI float64 = 0.7  /* relative humidity for saastamoinen model */

/* number and index of states */
func NF(opt *PrcOpt) int {
	if opt.IonoOpt == IONOOPT_IFLC {
		return 1
	} else {
		return opt.Nf
	}
}
func NP(opt *PrcOpt) int {
	if opt.Dynamics != 0 {
		return 9
	} else {
		return 3
	}
}
func NC(opt *PrcOpt) int { return NSYS }
func NT(opt *PrcOpt) int {
	if opt.TropOpt < TROPOPT_EST {
		return 0
	} else if opt.TropOpt == TROPOPT_EST {
		return 1
	} else {
		return 3
	}
}
func NI(opt *PrcOpt) int {
	if opt.IonoOpt == IONOOPT_EST {
		return MAXSAT
	} else {
		return 0
	}
}
func ND(opt *PrcOpt) int {
	if opt.Nf >= 3 {
		return 1
	} else {
		return 0
	}
}
func NR(opt *PrcOpt) int           { return NP(opt) + NC(opt) + NT(opt) + NI(opt) + ND(opt) }
func NB(opt *PrcOpt) int           { return NF(opt) * MAXSAT }
func NX(opt *PrcOpt) int           { return NR(opt) + NB(opt) }
func IC(s int, opt *PrcOpt) int    { return NP(opt) + (s) }
func IT(opt *PrcOpt) int           { return NP(opt) + NC(opt) }
func ITR(r int, opt *PrcOpt) int   { return NP(opt) + NI(opt) + NT(opt)/2*(r) } /* tropos (r:0=rov,1:ref) */
func II(s int, opt *PrcOpt) int    { return NP(opt) + NC(opt) + NT(opt) + (s) - 1 }
func ID(opt *PrcOpt) int           { return NP(opt) + NC(opt) + NT(opt) + NI(opt) }
func IB(s, f int, opt *PrcOpt) int { return NR(opt) + MAXSAT*(f) + (s) - 1 }
func IL(f int, opt *PrcOpt) int    { return NP(opt) + NI(opt) + NT(opt) + (f) } /* receiver h/w bias */

/* standard deviation of state -----------------------------------------------*/
func STD(rtk *Rtk, i int) float64 {
	if rtk.RtkSol.Stat == SOLQ_FIX {
		return SQRT(rtk.Pa[i+i*rtk.Nx])
	}
	return SQRT(rtk.P[i+i*rtk.Nx])
}

/* write solution status for PPP ---------------------------------------------*/
func OutPPPStat(rtk *Rtk, buff *string) int {
	var (
		ssat          *SSat
		tow           float64
		pos, vel, acc [3]float64
		x             []float64
		i, j, week    int
		p             string
	)

	if rtk.RtkSol.Stat == 0 {
		return 0
	}

	Trace(4, "pppoutstat:\n")

	tow = Time2GpsT(rtk.RtkSol.Time, &week)
	x = rtk.X
	if rtk.RtkSol.Stat == SOLQ_FIX {
		x = rtk.Xa
	}

	/* receiver position */
	p += fmt.Sprintf("$POS,%d,%.3f,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n", week, tow,
		rtk.RtkSol.Stat, x[0], x[1], x[2], STD(rtk, 0), STD(rtk, 1), STD(rtk, 2))

	/* receiver velocity and acceleration */
	if rtk.Opt.Dynamics > 0 {
		Ecef2Pos(rtk.RtkSol.Rr[:], pos[:])
		Ecef2Enu(pos[:], rtk.X[3:], vel[:])
		Ecef2Enu(pos[:], rtk.X[6:], acc[:])
		p += fmt.Sprintf("$VELACC,%d,%.3f,%d,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f\n", week, tow, rtk.RtkSol.Stat, vel[0], vel[1],
			vel[2], acc[0], acc[1], acc[2], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
	}
	/* receiver clocks */
	i = IC(0, &rtk.Opt)
	p += fmt.Sprintf("$CLK,%d,%.3f,%d,%d,%.3f,%.3f,%.3f,%.3f\n",
		week, tow, rtk.RtkSol.Stat, 1, x[i]*1e9/CLIGHT, x[i+1]*1e9/CLIGHT,
		STD(rtk, i)*1e9/CLIGHT, STD(rtk, i+1)*1e9/CLIGHT)

	/* tropospheric parameters */
	if rtk.Opt.TropOpt == TROPOPT_EST || rtk.Opt.TropOpt == TROPOPT_ESTG {
		i = IT(&rtk.Opt)
		p += fmt.Sprintf("$TROP,%d,%.3f,%d,%d,%.4f,%.4f\n", week, tow, rtk.RtkSol.Stat,
			1, x[i], STD(rtk, i))
	}
	if rtk.Opt.TropOpt == TROPOPT_ESTG {
		i = IT(&rtk.Opt)
		p += fmt.Sprintf("$TRPG,%d,%.3f,%d,%d,%.5f,%.5f,%.5f,%.5f\n", week, tow,
			rtk.RtkSol.Stat, 1, x[i+1], x[i+2], STD(rtk, i+1), STD(rtk, i+2))
	}
	/* ionosphere parameters */
	if rtk.Opt.IonoOpt == IONOOPT_EST {
		for i = 0; i < MAXSAT; i++ {
			ssat = &rtk.Ssat[i]
			if ssat.Vs == 0 {
				continue
			}
			j = II(i+1, &rtk.Opt)
			if rtk.X[j] == 0.0 {
				continue
			}
			var id string
			SatNo2Id(i+1, &id)
			p += fmt.Sprintf("$ION,%d,%.3f,%d,%s,%.1f,%.1f,%.4f,%.4f\n", week, tow,
				rtk.RtkSol.Stat, id, rtk.Ssat[i].Azel[0]*R2D,
				rtk.Ssat[i].Azel[1]*R2D, x[j], STD(rtk, j))
		}
	}
	// #ifdef OUTSTAT_AMB
	//     /* ambiguity parameters */
	//     for (i=0;i<MAXSAT;i++) for (j=0;j<NF(&rtk.opt);j++) {
	//         k=IB(i+1,j,&rtk.opt);
	//         if (rtk.x[k]==0.0) continue;
	//         satno2id(i+1,id);
	//         p+=sprintf(p,"$AMB,%d,%.3f,%d,%s,%d,%.4f,%.4f\n",week,tow,
	//                    rtk.sol.stat,id,j+1,x[k],STD(rtk,k));
	//     }
	// #endif
	n := len(p)
	*buff += p
	return n
}

/* exclude meas of eclipsing satellite (block IIA) ---------------------------*/
func TestEclipse(obs []ObsD, n int, nav *Nav, rs []float64) {
	var (
		rsun, esun   [3]float64
		r, ang, cosa float64
		erpv         [5]float64
		i, j         int
		dtype        string
	)

	Trace(4, "testeclipse:\n")

	/* unit vector of sun direction (ecef) */
	SunMoonPos(GpsT2Utc(obs[0].Time), erpv[:], rsun[:], nil, nil)
	NormV3(rsun[:], esun[:])

	for i = 0; i < n; i++ {
		dtype = nav.Pcvs[obs[i].Sat].Type

		if r = Norm(rs[i*6:], 3); r <= 0.0 {
			continue
		}

		/* only block IIA */
		if !strings.Contains(dtype, "BLOCK IIA") {
			continue
		}

		/* sun-earth-satellite angle */
		cosa = Dot(rs[i*6:], esun[:], 3) / r
		if cosa < -1.0 {
			cosa = -1.0
		} else if cosa > 1.0 {
			cosa = 1.0
		}
		ang = math.Acos(cosa)

		/* test eclipse */
		if ang < PI/2.0 || r*math.Sin(ang) > RE_WGS84 {
			continue
		}

		Trace(2, "eclipsing sat excluded %s sat=%2d\n", TimeStr(obs[0].Time, 0), obs[i].Sat)

		for j = 0; j < 3; j++ {
			rs[j+i*6] = 0.0
		}
	}
}

/* nominal yaw-angle ---------------------------------------------------------*/
func Yaw_Nominal(beta, mu float64) float64 {
	if math.Abs(beta) < 1e-12 && math.Abs(mu) < 1e-12 {
		return PI
	}
	return math.Atan2(-math.Tan(beta), math.Sin(mu)) + PI
}

/* yaw-angle of satellite ----------------------------------------------------*/
func Yaw_Angle(sat int, stype string, opt int, beta, mu float64, yaw *float64) int {
	*yaw = Yaw_Nominal(beta, mu)
	return 1
}

/* satellite attitude model --------------------------------------------------*/
func Sat_Yaw(time Gtime, sat int, stype string, opt int, rs, exs, eys []float64) int {
	var (
		rsun, es, esun, n, p, en, ep, ex [3]float64
		yaw, cosy, siny, E, beta, mu     float64
		ri                               [6]float64
		erpv                             [5]float64
	)

	SunMoonPos(GpsT2Utc(time), erpv[:], rsun[:], nil, nil)

	/* beta and orbit angle */
	MatCpy(ri[:], rs, 6, 1)
	ri[3] -= OMGE * ri[1]
	ri[4] += OMGE * ri[0]
	Cross3(ri[:], ri[3:], n[:])
	Cross3(rsun[:], n[:], p[:])
	if NormV3(rs, es[:]) == 0 || NormV3(rsun[:], esun[:]) == 0 || NormV3(n[:], en[:]) == 0 ||
		NormV3(p[:], ep[:]) == 0 {
		return 0
	}
	beta = PI/2.0 - math.Acos(Dot(esun[:], en[:], 3))
	E = math.Acos(Dot(es[:], ep[:], 3))
	if Dot(es[:], esun[:], 3) <= 0 {
		mu = PI/2.0 - E
	} else {
		mu = PI/2.0 + E
	}

	if mu < -PI/2.0 {
		mu += 2.0 * PI
	} else if mu >= PI/2.0 {
		mu -= 2.0 * PI
	}
	/* yaw-angle of satellite */
	if Yaw_Angle(sat, stype, opt, beta, mu, &yaw) == 0 {
		return 0
	}

	/* satellite fixed x,y-vector */
	Cross3(en[:], es[:], ex[:])
	cosy = math.Cos(yaw)
	siny = math.Sin(yaw)
	for i := 0; i < 3; i++ {
		exs[i] = -siny*en[i] + cosy*ex[i]
		eys[i] = -cosy*en[i] - siny*ex[i]
	}
	return 1
}

/* phase windup model --------------------------------------------------------*/
func Model_Phw(time Gtime, sat int, stype string, opt int, rs, rr []float64, phw *float64) int {
	var (
		exs, eys, ek, exr, eyr, eks, ekr, dr, ds, drs, r, pos [3]float64
		cosp, ph                                              float64
		E                                                     [9]float64
		i                                                     int
	)

	if opt <= 0 {
		return 1 /* no phase windup */
	}

	/* satellite yaw attitude model */
	if Sat_Yaw(time, sat, stype, opt, rs, exs[:], eys[:]) == 0 {
		return 0
	}

	/* unit vector satellite to receiver */
	for i = 0; i < 3; i++ {
		r[i] = rr[i] - rs[i]
	}
	if NormV3(r[:], ek[:]) == 0 {
		return 0
	}

	/* unit vectors of receiver antenna */
	Ecef2Pos(rr, pos[:])
	XYZ2Enu(pos[:], E[:])
	exr[0] = E[1]
	exr[1] = E[4]
	exr[2] = E[7] /* x = north */
	eyr[0] = -E[0]
	eyr[1] = -E[3]
	eyr[2] = -E[6] /* y = west  */

	/* phase windup effect */
	Cross3(ek[:], eys[:], eks[:])
	Cross3(ek[:], eyr[:], ekr[:])
	for i = 0; i < 3; i++ {
		ds[i] = exs[i] - ek[i]*Dot(ek[:], exs[:], 3) - eks[i]
		dr[i] = exr[i] - ek[i]*Dot(ek[:], exr[:], 3) + ekr[i]
	}
	cosp = Dot(ds[:], dr[:], 3) / Norm(ds[:], 3) / Norm(dr[:], 3)
	if cosp < -1.0 {
		cosp = -1.0
	} else if cosp > 1.0 {
		cosp = 1.0
	}
	ph = math.Acos(cosp) / 2.0 / PI
	Cross3(ds[:], dr[:], drs[:])
	if Dot(ek[:], drs[:], 3) < 0.0 {
		ph = -ph
	}

	*phw = ph + math.Floor(*phw-ph+0.5) /* in cycle */
	return 1
}

/* measurement error variance ------------------------------------------------*/
func PPPVarianceErr(sat, sys int, el float64, idx, itype int, opt *PrcOpt) float64 {
	fact := 1.0
	sinel := math.Sin(el)

	if itype == 1 {
		if idx == 0 {
			fact *= opt.eratio[0]
		} else {
			fact *= opt.eratio[1]
		}
	}
	if sys == SYS_GLO {
		fact *= float64(EFACT_GLO)

	} else if sys == SYS_SBS {
		fact *= float64(EFACT_SBS)
	} else {
		fact *= float64(EFACT_GPS)
	}
	if sys == SYS_GPS || sys == SYS_QZS {
		if idx == 2 {
			fact *= EFACT_GPS_L5 /* GPS/QZS L5 error factor */
		}
	}
	if opt.IonoOpt == IONOOPT_IFLC {
		fact *= 3.0
	}
	return SQR(fact*opt.Err[1]) + SQR(fact*opt.Err[2]/sinel)
}

/* initialize state and covariance -------------------------------------------*/
func initx(rtk *Rtk, xi, vari float64, i int) {

	rtk.X[i] = xi
	for j := 0; j < rtk.Nx; j++ {
		if i == j {
			rtk.P[i+j*rtk.Nx], rtk.P[j+i*rtk.Nx] = vari, vari

		} else {
			rtk.P[i+j*rtk.Nx], rtk.P[j+i*rtk.Nx] = 0.0, 0.0
		}
	}
}

/* geometry-free phase measurement -------------------------------------------*/
func GfMeas(obs *ObsD, nav *Nav) float64 {
	freq1 := Sat2Freq(obs.Sat, obs.Code[0], nav)
	freq2 := Sat2Freq(obs.Sat, obs.Code[1], nav)
	if freq1 == 0.0 || freq2 == 0.0 || obs.L[0] == 0.0 || obs.L[1] == 0.0 {
		return 0.0
	}
	return (obs.L[0]/freq1 - obs.L[1]/freq2) * CLIGHT
}

/* Melbourne-Wubbena linear combination --------------------------------------*/
func MWMeas(obs *ObsD, nav *Nav) float64 {
	freq1 := Sat2Freq(obs.Sat, obs.Code[0], nav)
	freq2 := Sat2Freq(obs.Sat, obs.Code[1], nav)

	if freq1 == 0.0 || freq2 == 0.0 || obs.L[0] == 0.0 || obs.L[1] == 0.0 ||
		obs.P[0] == 0.0 || obs.P[1] == 0.0 {
		return 0.0
	}
	return (obs.L[0]-obs.L[1])*CLIGHT/(freq1-freq2) -
		(freq1*obs.P[0]+freq2*obs.P[1])/(freq1+freq2)
}

/* antenna corrected measurements --------------------------------------------*/
func CorrMeas(obs *ObsD, nav *Nav, azel []float64,
	opt *PrcOpt, dantr, dants []float64, phw float64, L, P []float64,
	Lc, Pc *float64) {
	var (
		freq   [NFREQ]float64
		C1, C2 float64
		i, sys int = 0, SatSys(obs.Sat, nil)
	)

	for i = 0; i < NFREQ; i++ {
		L[i], P[i] = 0.0, 0.0
		freq[i] = Sat2Freq(obs.Sat, obs.Code[i], nav)
		if freq[i] == 0.0 || obs.L[i] == 0.0 || obs.P[i] == 0.0 {
			continue
		}
		if TestSnr(0, 0, azel[1], float64(obs.SNR[i])*float64(SNR_UNIT), &opt.SnrMask) > 0 {
			continue
		}

		/* antenna phase center and phase windup correction */
		L[i] = obs.L[i]*CLIGHT/freq[i] - dants[i] - dantr[i] - phw*CLIGHT/freq[i]
		P[i] = obs.P[i] - dants[i] - dantr[i]

		/* P1-C1,P2-C2 dcb correction (C1.P1,C2.P2) */
		if sys == SYS_GPS || sys == SYS_GLO {
			switch obs.Code[i] {
			case CODE_L1C:
				P[i] += nav.CBias[obs.Sat][1]

			case CODE_L2C:
				P[i] += nav.CBias[obs.Sat][2]
			}
		}
	}
	/* iono-free LC */
	*Lc, *Pc = 0.0, 0.0
	// modified by fxb
	// add L5 process
	switch {
	case freq[0] != 0.0 && freq[1] != 0.0:
		C1 = SQR(freq[0]) / (SQR(freq[0]) - SQR(freq[1]))
		C2 = -SQR(freq[1]) / (SQR(freq[0]) - SQR(freq[1]))

		if L[0] != 0.0 && L[1] != 0.0 {
			*Lc = C1*L[0] + C2*L[1]
		}
		if P[0] != 0.0 && P[1] != 0.0 {
			*Pc = C1*P[0] + C2*P[1]
		}
	case freq[0] != 0.0 && freq[2] != 0.0:
		C1 = SQR(freq[0]) / (SQR(freq[0]) - SQR(freq[2]))
		C2 = -SQR(freq[2]) / (SQR(freq[0]) - SQR(freq[2]))

		if L[0] != 0.0 && L[2] != 0.0 {
			*Lc = C1*L[0] + C2*L[2]
		}
		if P[0] != 0.0 && P[2] != 0.0 {
			*Pc = C1*P[0] + C2*P[2]
		}
	default:
		return
	}
	// if freq[0] == 0.0 || freq[1] == 0.0 {
	// 	return
	// }
	// C1 = SQR(freq[0]) / (SQR(freq[0]) - SQR(freq[1]))
	// C2 = -SQR(freq[1]) / (SQR(freq[0]) - SQR(freq[1]))

	// if L[0] != 0.0 && L[1] != 0.0 {
	// 	*Lc = C1*L[0] + C2*L[1]
	// }
	// if P[0] != 0.0 && P[1] != 0.0 {
	// 	*Pc = C1*P[0] + C2*P[1]
	// }
}

/* detect cycle slip by LLI --------------------------------------------------*/
func DetectSlp_ll(rtk *Rtk, obs []ObsD, n int) {
	Trace(4, "detslp_ll: n=%d\n", n)

	for i := 0; i < n && i < MAXOBS; i++ {
		for j := 0; j < rtk.Opt.Nf; j++ {
			if obs[i].L[j] == 0.0 || (obs[i].LLI[j]&3) == 0 {
				continue
			}

			Trace(2, "detslp_ll: slip detected sat=%2d f=%d\n", obs[i].Sat, j+1)

			rtk.Ssat[obs[i].Sat-1].Slip[j] = 1
		}
	}
}

/* detect cycle slip by geometry free phase jump -----------------------------*/
func DetectSlp_gf(rtk *Rtk, obs []ObsD, n int, nav *Nav) {
	var g0, g1 float64

	Trace(4, "detslp_gf: n=%d\n", n)

	for i := 0; i < n && i < len(obs); i++ {

		if g1 = GfMeas(&obs[i], nav); g1 == 0.0 {
			continue
		}

		g0 = rtk.Ssat[obs[i].Sat-1].Gf[0]
		rtk.Ssat[obs[i].Sat-1].Gf[0] = g1

		Trace(2, "detslip_gf: sat=%2d gf0=%8.3f gf1=%8.3f\n", obs[i].Sat, g0, g1)

		if g0 != 0.0 && math.Abs(g1-g0) > rtk.Opt.ThresSlip {
			Trace(2, "detslip_gf: slip detected sat=%2d gf=%8.3f.%8.3f\n",
				obs[i].Sat, g0, g1)

			for j := 0; j < rtk.Opt.Nf; j++ {
				rtk.Ssat[obs[i].Sat-1].Slip[j] |= 1
			}
		}
	}
}

/* detect slip by Melbourne-Wubbena linear combination jump ------------------*/
func DetectSlp_mw(rtk *Rtk, obs []ObsD, n int, nav *Nav) {
	var w0, w1 float64

	Trace(4, "detslp_mw: n=%d\n", n)

	for i := 0; i < n && i < len(obs); i++ {
		if w1 = MWMeas(&obs[i], nav); w1 == 0.0 {
			continue
		}

		w0 = rtk.Ssat[obs[i].Sat-1].Mw[0]
		rtk.Ssat[obs[i].Sat-1].Mw[0] = w1

		Trace(2, "detslip_mw: sat=%2d mw0=%8.3f mw1=%8.3f\n", obs[i].Sat, w0, w1)

		if w0 != 0.0 && math.Abs(w1-w0) > THRES_MW_JUMP {
			Trace(2, "detslip_mw: slip detected sat=%2d mw=%8.3f.%8.3f\n",
				obs[i].Sat, w0, w1)

			for j := 0; j < rtk.Opt.Nf; j++ {
				rtk.Ssat[obs[i].Sat-1].Slip[j] |= 1
			}
		}
	}
}

/* temporal update of position -----------------------------------------------*/
func (rtk *Rtk) UpdatePosPPP() {
	var (
		F, P, FP, x, xp []float64
		pos             [3]float64
		Q, Qv           [9]float64
		i, j, nx        int
		ix              []int
	)

	Trace(4, "udpos_ppp:\n")

	/* fixed mode */
	if rtk.Opt.Mode == PMODE_PPP_FIXED {
		for i = 0; i < 3; i++ {
			initx(rtk, rtk.Opt.Ru[i], 1e-8, i)
		}
		return
	}
	/* initialize position for first epoch */
	if Norm(rtk.X, 3) <= 0.0 {
		for i = 0; i < 3; i++ {
			initx(rtk, rtk.RtkSol.Rr[i], VAR_POS, i)
		}
		if rtk.Opt.Dynamics > 0 {
			for i = 3; i < 6; i++ {
				initx(rtk, rtk.RtkSol.Rr[i], VAR_VEL, i)
			}
			for i = 6; i < 9; i++ {
				initx(rtk, 1e-6, VAR_ACC, i)
			}
		}
	}
	/* static ppp mode */
	if rtk.Opt.Mode == PMODE_PPP_STATIC {
		for i = 0; i < 3; i++ {
			rtk.P[i*(1+rtk.Nx)] += SQR(rtk.Opt.Prn[5]) * math.Abs(rtk.Tt)
		}
		return
	}
	/* kinmatic mode without dynamics */
	if rtk.Opt.Dynamics == 0 {
		for i = 0; i < 3; i++ {
			initx(rtk, rtk.RtkSol.Rr[i], VAR_POS, i)
		}
		return
	}
	/* generate valid state index */
	ix = IMat(rtk.Nx, 1)
	for i, nx = 0, 0; i < rtk.Nx; i++ {
		if rtk.X[i] != 0.0 && rtk.P[i+i*rtk.Nx] > 0.0 {
			ix[nx] = i
			nx++
		}
	}
	if nx < 9 {
		return
	}
	/* state transition of position/velocity/acceleration */
	F = Eye(nx)
	P = Mat(nx, nx)
	FP = Mat(nx, nx)
	x = Mat(nx, 1)
	xp = Mat(nx, 1)

	for i = 0; i < 6; i++ {
		F[i+(i+3)*nx] = rtk.Tt
	}
	for i = 0; i < 3; i++ {
		F[i+(i+6)*nx] = SQR(rtk.Tt) / 2.0
	}
	for i = 0; i < nx; i++ {
		x[i] = rtk.X[ix[i]]
		for j = 0; j < nx; j++ {
			P[i+j*nx] = rtk.P[ix[i]+ix[j]*rtk.Nx]
		}
	}
	/* x=F*x, P=F*P*F+Q */
	MatMul("NN", nx, 1, nx, 1.0, F, x, 0.0, xp)
	MatMul("NN", nx, nx, nx, 1.0, F, P, 0.0, FP)
	MatMul("NT", nx, nx, nx, 1.0, FP, F, 0.0, P)

	for i = 0; i < nx; i++ {
		rtk.X[ix[i]] = xp[i]
		for j = 0; j < nx; j++ {
			rtk.P[ix[i]+ix[j]*rtk.Nx] = P[i+j*nx]
		}
	}
	/* process noise added to only acceleration */
	Q[0] = SQR(rtk.Opt.Prn[3]) * math.Abs(rtk.Tt)
	Q[4] = Q[0]
	Q[8] = SQR(rtk.Opt.Prn[4]) * math.Abs(rtk.Tt)
	Ecef2Pos(rtk.X, pos[:])
	Cov2Ecef(pos[:], Q[:], Qv[:])
	for i = 0; i < 3; i++ {
		for j = 0; j < 3; j++ {
			rtk.P[i+6+(j+6)*rtk.Nx] += Qv[i+j*3]
		}
	}
}

/* temporal update of clock --------------------------------------------------*/
func (rtk *Rtk) UpdateClkPPP() {
	var dtr float64

	Trace(4, "udclk_ppp:\n")

	/* initialize every epoch for clock (white noise) */
	for i := 0; i < NSYS && i < len(rtk.RtkSol.Dtr); i++ {
		if rtk.Opt.SatEph == EPHOPT_PREC {
			/* time of prec ephemeris is based gpst */
			/* negelect receiver inter-system bias  */
			dtr = rtk.RtkSol.Dtr[0]
		} else {
			if i == 0 {
				dtr = rtk.RtkSol.Dtr[0]

			} else {
				dtr = rtk.RtkSol.Dtr[0] + rtk.RtkSol.Dtr[i]

			}
		}
		initx(rtk, CLIGHT*dtr, VAR_CLK, IC(i, &rtk.Opt))
	}
}

/* temporal update of tropospheric parameters --------------------------------*/
func (rtk *Rtk) UpdateTropPPP() {
	var (
		pos, azel [3]float64 = [3]float64{0, 0, 0}, [3]float64{0.0, PI / 2.0}
		ztd, vari float64
	)
	i := IT(&rtk.Opt)

	Trace(4, "udtrop_ppp:\n")

	if rtk.X[i] == 0.0 {
		Ecef2Pos(rtk.RtkSol.Rr[:], pos[:])
		ztd = SbsTropCorr(rtk.RtkSol.Time, pos[:], azel[:], &vari)
		initx(rtk, ztd, vari, i)

		if rtk.Opt.TropOpt >= TROPOPT_ESTG {
			for j := i + 1; j < i+3; j++ {
				initx(rtk, 1e-6, VAR_GRA, j)
			}
		}
	} else {
		rtk.P[i+i*rtk.Nx] += SQR(rtk.Opt.Prn[2]) * math.Abs(rtk.Tt)

		if rtk.Opt.TropOpt >= TROPOPT_ESTG {
			for j := i + 1; j < i+3; j++ {
				rtk.P[j+j*rtk.Nx] += SQR(rtk.Opt.Prn[2]*0.1) * math.Abs(rtk.Tt)
			}
		}
	}
}

/* temporal update of ionospheric parameters ---------------------------------*/
func (rtk *Rtk) UpdateIonoPPP(obs []ObsD, n int, nav *Nav) {
	var freq1, freq2, ion, sinel float64
	var pos [3]float64
	var azel []float64
	var i, j, gap_resion int = 0, 0, GAP_RESION

	Trace(4, "udiono_ppp:\n")
	index := strings.Index(rtk.Opt.PPPOpt, "-GAP_RESION=")
	if index >= 0 {
		fmt.Scanf(rtk.Opt.PPPOpt[index:], "-GAP_RESION=%d", &gap_resion)
	}
	for i = 0; i < len(rtk.Ssat); i++ {
		j = II(i+1, &rtk.Opt)
		if rtk.X[j] != 0.0 && int(rtk.Ssat[i].Outc[0]) > gap_resion {
			rtk.X[j] = 0.0
		}
	}
	for i = 0; i < n; i++ {
		j = II(obs[i].Sat, &rtk.Opt)
		if rtk.X[j] == 0.0 {
			freq1 = Sat2Freq(obs[i].Sat, obs[i].Code[0], nav)
			freq2 = Sat2Freq(obs[i].Sat, obs[i].Code[1], nav)
			if obs[i].P[0] == 0.0 || obs[i].P[1] == 0.0 || freq1 == 0.0 || freq2 == 0.0 {
				continue
			}
			ion = (obs[i].P[0] - obs[i].P[1]) / (SQR(FREQ1/freq1) - SQR(FREQ1/freq2))
			Ecef2Pos(rtk.RtkSol.Rr[:], pos[:])
			azel = rtk.Ssat[obs[i].Sat-1].Azel[:]
			ion /= IonMapf(pos[:], azel[:])
			initx(rtk, ion, VAR_IONO, j)
		} else {
			sinel = math.Sin(math.Max(rtk.Ssat[obs[i].Sat-1].Azel[1], 5.0*D2R))
			rtk.P[j+j*rtk.Nx] += SQR(rtk.Opt.Prn[1]/sinel) * math.Abs(rtk.Tt)
		}
	}
}

/* temporal update of L5-receiver-dcb parameters -----------------------------*/
func (rtk *Rtk) UpdateDcbPPP() {
	i := ID(&rtk.Opt)

	Trace(4, "uddcb_ppp:\n")

	if rtk.X[i] == 0.0 {
		initx(rtk, 1e-6, VAR_DCB, i)
	}
}

/* temporal update of phase biases -------------------------------------------*/
func (rtk *Rtk) UpdateBiasPPP(obs []ObsD, n int, nav *Nav) {
	var (
		L, P, dantr, dants                [NFREQ]float64
		Lc, Pc, offset, freq1, freq2, ion float64
		bias                              [MAXOBS]float64
		pos                               [3]float64

		i, j, k, f, sat, clk_jump int
		slip                      [MAXOBS]int
	)

	Trace(4, "udbias  : n=%d\n", n)

	/* handle day-boundary clock jump */
	if rtk.Opt.PosOpt[5] > 0 {
		if ROUND_I(Time2GpsT(obs[0].Time, nil)*10.0)%864000.0 == 0 {
			clk_jump = 1

		} else {
			clk_jump = 0
		}
	}
	for i = 0; i < len(rtk.Ssat); i++ {
		for j = 0; j < rtk.Opt.Nf; j++ {
			rtk.Ssat[i].Slip[j] = 0
		}
	}
	/* detect cycle slip by LLI */
	DetectSlp_ll(rtk, obs, n)

	/* detect cycle slip by geometry-free phase jump */
	DetectSlp_gf(rtk, obs, n, nav)

	/* detect slip by Melbourne-Wubbena linear combination jump */
	DetectSlp_mw(rtk, obs, n, nav)

	Ecef2Pos(rtk.RtkSol.Rr[:], pos[:])

	for f = 0; f < NF(&rtk.Opt); f++ {

		/* reset phase-bias if expire obs outage counter */
		for i = 0; i < len(rtk.Ssat); i++ {
			if rtk.Ssat[i].Outc[f]++; int(rtk.Ssat[i].Outc[f]) > rtk.Opt.MaxOut ||
				rtk.Opt.ModeAr == ARMODE_INST || clk_jump == 1 {
				initx(rtk, 0.0, 0.0, IB(i+1, f, &rtk.Opt))
			}
		}
		for i, k = 0, 0; i < n && i < len(obs); i++ {
			sat = obs[i].Sat
			j = IB(sat, f, &rtk.Opt)
			CorrMeas(&obs[i], nav, rtk.Ssat[sat-1].Azel[:], &rtk.Opt, dantr[:], dants[:],
				0.0, L[:], P[:], &Lc, &Pc)

			bias[i] = 0.0

			if rtk.Opt.IonoOpt == IONOOPT_IFLC {
				bias[i] = Lc - Pc
				if rtk.Ssat[sat-1].Slip[0] > 0 || rtk.Ssat[sat-1].Slip[1] > 0 {
					slip[i] = 1
				} else {
					slip[i] = 0
				}
			} else if L[f] != 0.0 && P[f] != 0.0 {
				freq1 = Sat2Freq(sat, obs[i].Code[0], nav)
				freq2 = Sat2Freq(sat, obs[i].Code[f], nav)
				slip[i] = int(rtk.Ssat[sat-1].Slip[f])
				if obs[i].P[0] == 0.0 || obs[i].P[1] == 0.0 || freq1 == 0.0 || freq2 == 0.0 {
					continue
				}
				ion = (obs[i].P[0] - obs[i].P[f]) / (1.0 - SQR(freq1/freq2))
				bias[i] = L[f] - P[f] + 2.0*ion*SQR(freq1/freq2)
			}
			if rtk.X[j] == 0.0 || slip[i] > 0 || bias[i] == 0.0 {
				continue
			}

			offset += bias[i] - rtk.X[j]
			k++
		}
		/* correct phase-code jump to ensure phase-code coherency */
		if k >= 2 && math.Abs(offset/float64(k)) > 0.0005*CLIGHT {
			for i = 0; i < len(rtk.X); i++ {
				j = IB(i+1, f, &rtk.Opt)
				if rtk.X[j] != 0.0 {
					rtk.X[j] += offset / float64(k)
				}
			}
			Trace(2, "phase-code jump corrected: %s n=%2d dt=%12.9fs\n",
				TimeStr(rtk.RtkSol.Time, 0), k, offset/float64(k)/CLIGHT)
		}
		for i = 0; i < n && i < len(obs); i++ {
			sat = obs[i].Sat
			j = IB(sat, f, &rtk.Opt)

			rtk.P[j+j*rtk.Nx] += SQR(rtk.Opt.Prn[0]) * math.Abs(rtk.Tt)

			if bias[i] == 0.0 || (rtk.X[j] != 0.0 && slip[i] == 0) {
				continue
			}

			/* reinitialize phase-bias if detecting cycle slip */
			initx(rtk, bias[i], VAR_BIAS, IB(sat, f, &rtk.Opt))

			/* reset fix flags */
			for k = 0; k < MAXSAT; k++ {
				rtk.Ambc[sat-1].flags[k] = 0
			}

			Trace(3, "udbias_ppp: sat=%2d bias=%.3f\n", sat, bias[i])
		}
	}
}

/* temporal update of states --------------------------------------------------*/
func (rtk *Rtk) UpdateStatePPP(obs []ObsD, n int, nav *Nav) {
	Trace(4, "udstate_ppp: n=%d\n", n)

	/* temporal update of position */
	rtk.UpdatePosPPP()

	/* temporal update of clock */
	rtk.UpdateClkPPP()

	/* temporal update of tropospheric parameters */
	if rtk.Opt.TropOpt == TROPOPT_EST || rtk.Opt.TropOpt == TROPOPT_ESTG {
		rtk.UpdateTropPPP()
	}
	/* temporal update of ionospheric parameters */
	if rtk.Opt.IonoOpt == IONOOPT_EST {
		rtk.UpdateIonoPPP(obs, n, nav)
	}
	/* temporal update of L5-receiver-dcb parameters */
	if rtk.Opt.Nf >= 3 {
		rtk.UpdateDcbPPP()
	}
	/* temporal update of phase-bias */
	rtk.UpdateBiasPPP(obs, n, nav)
}

/* satellite antenna phase center variation ----------------------------------*/
func SatAntPcv(rs, rr []float64, pcv *Pcv, dant []float64) {
	var (
		ru, rz, eu, ez [3]float64
		nadir, cosa    float64
	)

	for i := 0; i < 3; i++ {
		ru[i] = rr[i] - rs[i]
		rz[i] = -rs[i]
	}
	if NormV3(ru[:], eu[:]) == 0 || NormV3(rz[:], ez[:]) == 0 {
		return
	}

	cosa = Dot(eu[:], ez[:], 3)
	if cosa < -1.0 {
		cosa = -1.0
	} else if cosa > 1.0 {
		cosa = 1.0
	}

	nadir = math.Acos(cosa)

	AntModel_s(pcv, nadir, dant)
}

/* precise tropospheric model ------------------------------------------------*/
func TropModelPrec(time Gtime, pos, azel, x, dtdx []float64, vari *float64) float64 {
	var (
		zazel                               []float64 = []float64{0.0, PI / 2.0}
		zhd, m_h, m_w, cotz, grad_n, grad_e float64
	)

	/* zenith hydrostatic delay */
	zhd = TropModel(time, pos, zazel, 0.0)

	/* mapping function */
	m_h = TropMapFunc(time, pos, azel, &m_w)

	if azel[1] > 0.0 {

		/* m_w=m_0+m_0*cot(el)*(Gn*cos(az)+Ge*sin(az)): ref [6] */
		cotz = 1.0 / math.Tan(azel[1])
		grad_n = m_w * cotz * math.Cos(azel[0])
		grad_e = m_w * cotz * math.Sin(azel[0])
		m_w += grad_n*x[1] + grad_e*x[2]
		dtdx[1] = grad_n * (x[0] - zhd)
		dtdx[2] = grad_e * (x[0] - zhd)
	}
	dtdx[0] = m_w
	*vari = SQR(0.01)
	return m_h*zhd + m_w*(x[0]-zhd)
}

/* tropospheric model ---------------------------------------------------------*/
func ModelTrop(time Gtime, pos, azel []float64, opt *PrcOpt, x, dtdx []float64, nav *Nav, dtrp, vari *float64) int {
	var trp [3]float64

	switch opt.TropOpt {

	case TROPOPT_SAAS:
		{
			*dtrp = TropModel(time, pos, azel, REL_HUMI)
			*vari = SQR(ERR_SAAS)
			return 1
		}
	case TROPOPT_SBAS:
		{
			*dtrp = SbsTropCorr(time, pos[:], azel, vari)
			return 1
		}
	case TROPOPT_EST, TROPOPT_ESTG:
		{
			if opt.TropOpt == TROPOPT_EST {
				MatCpy(trp[:], x[IT(opt):], 1, 1)
			} else {
				MatCpy(trp[:], x[IT(opt):], 3, 1)
			}
			*dtrp = TropModelPrec(time, pos, azel, trp[:], dtdx, vari)
			return 1
		}
	}

	return 0
}

/* ionospheric model ---------------------------------------------------------*/
func ModelIono(time Gtime, pos, azel []float64, opt *PrcOpt, sat int, x []float64, nav *Nav, dion, vari *float64) int {
	switch opt.IonoOpt {
	case IONOOPT_SBAS:
		{
			return SbsIonCorr(time, nav, pos, azel, dion, vari)
		}
	case IONOOPT_TEC:
		{
			return nav.IonTec(time, pos, azel, 1, dion, vari)
		}
	case IONOOPT_BRDC:
		{
			*dion = IonModel(time, nav.Ion_gps[:], pos, azel)
			*vari = SQR(*dion * ERR_BRDCI)
			return 1
		}
	case IONOOPT_EST:
		{
			*dion = x[II(sat, opt)]
			*vari = 0.0
			return 1
		}
	case IONOOPT_IFLC:
		{
			*dion, *vari = 0.0, 0.0
			return 1
		}
	}
	return 0
}

/* phase and code residuals --------------------------------------------------*/
func PPPResidual(post int, obs []ObsD, n int, rs, dts, var_rs []float64, svh []int,
	dr []float64, exc []int, nav *Nav, x []float64, rtk *Rtk, v, H, R, azel []float64) int {
	opt := &rtk.Opt
	var (
		y, r, cdtr, bias, C, Lc, Pc, vmax              float64
		rr, pos, e, dtdx                               [3]float64
		L, P, dantr, dants                             [NFREQ]float64
		vars                                           [MAXOBS * 2]float64
		dtrp, dion, vart, vari, dcb, freq              float64
		ve                                             [MAXOBS * 2 * NFREQ]float64
		str                                            string
		i, j, k, sat, sys, nv, ne, maxobs, maxfrq, rej int
		obsi, frqi                                     [MAXOBS * 2 * NFREQ]int
		nx, stat                                       int = rtk.Nx, 1
	)
	Time2Str(obs[0].Time, &str, 2)

	for i = 0; i < len(rtk.Ssat); i++ {
		for j = 0; j < opt.Nf; j++ {
			rtk.Ssat[i].Vsat[j] = 0
		}
	}

	for i = 0; i < 3; i++ {
		rr[i] = x[i] + dr[i]
	}
	Ecef2Pos(rr[:], pos[:])

	for i = 0; i < n && i < len(obs); i++ {
		sat = obs[i].Sat

		if r = GeoDist(rs[i*6:], rr[:], e[:]); r <= 0.0 || SatAzel(pos[:], e[:], azel[i*2:]) < opt.Elmin {
			exc[i] = 1
			continue
		}
		if sys = SatSys(sat, nil); sys == 0 || rtk.Ssat[sat-1].Vs == 0 || SatExclude(obs[i].Sat, var_rs[i], svh[i], opt) > 0 || exc[i] > 0 {
			exc[i] = 1
			continue
		}
		/* tropospheric and ionospheric model */
		if ModelTrop(obs[i].Time, pos[:], azel[i*2:], opt, x, dtdx[:], nav, &dtrp, &vart) == 0 ||
			ModelIono(obs[i].Time, pos[:], azel[i*2:], opt, sat, x, nav, &dion, &vari) == 0 {
			continue
		}
		/* satellite and receiver antenna model */
		if opt.PosOpt[0] > 0 {
			SatAntPcv(rs[i*6:], rr[:], &nav.Pcvs[sat], dants[:])
		}
		AntModel(&opt.Pcvr[0], opt.AntDel[0][:], azel[i*2:], opt.PosOpt[1], dantr[:])

		/* phase windup model */
		posopt := 0
		if opt.PosOpt[2] > 0 {
			posopt = 2
		}
		if Model_Phw(rtk.RtkSol.Time, sat, nav.Pcvs[sat-1].Type,
			posopt, rs[i*6:], rr[:], &rtk.Ssat[sat-1].Phw) == 0 {
			continue
		}
		/* corrected phase and code measurements */
		CorrMeas(&obs[i], nav, azel[i*2:], &rtk.Opt, dantr[:], dants[:],
			rtk.Ssat[sat-1].Phw, L[:], P[:], &Lc, &Pc)

		/* stack phase and code residuals {L1,P1,L2,P2,...} */
		for j = 0; j < 2*NF(opt); j++ {

			dcb, bias = 0.0, 0.0

			if opt.IonoOpt == IONOOPT_IFLC {
				if j%2 == 0 {
					y = Lc
				} else {
					y = Pc
				}
				if y == 0.0 {
					continue
				}
			} else {
				if j%2 == 0 {
					y = L[j/2]
				} else {
					y = P[j/2]
				}

				if y == 0.0 {
					continue
				}

				if freq = Sat2Freq(sat, obs[i].Code[j/2], nav); freq == 0.0 {
					continue
				}
				C = SQR(FREQ1/freq) * IonMapf(pos[:], azel[i*2:])
				if j%2 == 0 {
					C *= -1.0
				} else {
					C *= 1.0
				}
			}
			for k = 0; k < nx; k++ {
				H[k+nx*nv] = 0.0
				if k < 3 {
					H[k+nx*nv] = -e[k]
				}
			}
			/* receiver clock */
			switch sys {
			case SYS_GLO:
				k = 1
			case SYS_GAL:
				k = 2
			case SYS_CMP:
				k = 3
			case SYS_IRN:
				k = 4
			default:
				k = 0
			}
			cdtr = x[IC(k, opt)]
			H[IC(k, opt)+nx*nv] = 1.0

			if opt.TropOpt == TROPOPT_EST || opt.TropOpt == TROPOPT_ESTG {
				tropopt := 1
				if opt.TropOpt >= TROPOPT_ESTG {
					tropopt = 3
				}
				for k = 0; k < tropopt; k++ {
					H[IT(opt)+k+nx*nv] = dtdx[k]
				}
			}
			if opt.IonoOpt == IONOOPT_EST {
				if rtk.X[II(sat, opt)] == 0.0 {
					continue
				}
				H[II(sat, opt)+nx*nv] = C
			}
			if j/2 == 2 && j%2 == 1 { /* L5-receiver-dcb */
				dcb += rtk.X[ID(opt)]
				H[ID(opt)+nx*nv] = 1.0
			}
			if j%2 == 0 { /* phase bias */
				if bias = x[IB(sat, j/2, opt)]; bias == 0.0 {
					continue
				}
				H[IB(sat, j/2, opt)+nx*nv] = 1.0
			}
			/* residual */
			v[nv] = y - (r + cdtr - CLIGHT*dts[i*2] + dtrp + C*dion + dcb + bias)

			if j%2 == 0 {
				rtk.Ssat[sat-1].Resc[j/2] = float32(v[nv])
			} else {
				rtk.Ssat[sat-1].Resp[j/2] = float32(v[nv])
			}

			/* variance */
			vars[nv] = PPPVarianceErr(obs[i].Sat, sys, azel[1+i*2], j/2, j%2, opt) +
				vart + SQR(C)*vari + var_rs[i]
			if sys == SYS_GLO && j%2 == 1 {
				vars[nv] += VAR_GLO_IFB
			}
			if j%2 == 1 {
				Trace(2, "%s sat=%2d %s%d res=%9.4f sig=%9.4f el=%4.1f\n", str, sat,
					"P", j/2+1, v[nv], math.Sqrt(vars[nv]), azel[1+i*2]*R2D)
			} else {
				Trace(2, "%s sat=%2d %s%d res=%9.4f sig=%9.4f el=%4.1f\n", str, sat,
					"L", j/2+1, v[nv], math.Sqrt(vars[nv]), azel[1+i*2]*R2D)
			}
			/* reject satellite by pre-fit residuals */
			if post == 0 && opt.MaxInno > 0.0 && math.Abs(v[nv]) > opt.MaxInno {
				if j%2 == 1 {
					Trace(2, "outlier (%d) rejected %s sat=%2d %s%d res=%9.4f el=%4.1f\n",
						post, str, sat, "P", j/2+1, v[nv], azel[1+i*2]*R2D)
				} else {
					Trace(2, "outlier (%d) rejected %s sat=%2d %s%d res=%9.4f el=%4.1f\n",
						post, str, sat, "L", j/2+1, v[nv], azel[1+i*2]*R2D)

				}
				exc[i] = 1
				rtk.Ssat[sat-1].Rejc[j%2]++
				continue
			}
			/* record large post-fit residuals */
			if post > 0 && math.Abs(v[nv]) > math.Sqrt(vars[nv])*THRES_REJECT {
				obsi[ne] = i
				frqi[ne] = j
				ve[ne] = v[nv]
				ne++
			}
			if j%2 == 0 {
				rtk.Ssat[sat-1].Vsat[j/2] = 1
			}
			nv++
		}
	}
	/* reject satellite with large and max post-fit residual */
	if post > 0 && ne > 0 {
		vmax = ve[0]
		maxobs = obsi[0]
		maxfrq = frqi[0]
		rej = 0
		for j = 1; j < ne; j++ {
			if math.Abs(vmax) >= math.Abs(ve[j]) {
				continue
			}
			vmax = ve[j]
			maxobs = obsi[j]
			maxfrq = frqi[j]
			rej = j
		}
		sat = obs[maxobs].Sat
		if maxfrq%2 == 1 {
			Trace(2, "outlier (%d) rejected %s sat=%2d %s%d res=%9.4f el=%4.1f\n",
				post, str, sat, "P", maxfrq/2+1, vmax, azel[1+maxobs*2]*R2D)
		} else {
			Trace(2, "outlier (%d) rejected %s sat=%2d %s%d res=%9.4f el=%4.1f\n",
				post, str, sat, "L", maxfrq/2+1, vmax, azel[1+maxobs*2]*R2D)

		}

		exc[maxobs] = 1
		rtk.Ssat[sat-1].Rejc[maxfrq%2]++
		stat = 0
		ve[rej] = 0
	}
	for i = 0; i < nv; i++ {
		for j = 0; j < nv; j++ {
			R[i+j*nv] = 0.0
			if i == j {
				R[i+j*nv] = vars[i]
			}

		}
	}
	if post > 0 {
		return stat
	} else {
		return nv
	}

}

/* ambiguity resolution in ppp -----------------------------------------------*/
func PPPAmbiguity(rtk *Rtk, obs []ObsD, n int, exc []int, nav *Nav, azel, x, P []float64) int {
	//"the function(ppp_ar) is truncated in 2016"
	return 0
}

/* number of estimated states ------------------------------------------------*/
func pppnx(opt *PrcOpt) int {
	return NX(opt)
}

/* update solution status ----------------------------------------------------*/
func (rtk *Rtk) UpdateStat(obs []ObsD, n, stat int) {
	opt := &rtk.Opt
	var i, j int

	/* test # of valid satellites */
	rtk.RtkSol.Ns = 0
	for i = 0; i < n && i < len(obs); i++ {
		for j = 0; j < opt.Nf; j++ {
			if rtk.Ssat[obs[i].Sat-1].Vsat[j] == 0 {
				continue
			}
			rtk.Ssat[obs[i].Sat-1].Lock[j]++
			rtk.Ssat[obs[i].Sat-1].Outc[j] = 0
			if j == 0 {
				rtk.RtkSol.Ns++
			}
		}
	}
	rtk.RtkSol.Stat = uint8(stat)
	if rtk.RtkSol.Ns < uint8(MIN_NSAT_SOL) {
		rtk.RtkSol.Stat = SOLQ_NONE
	}
	if rtk.RtkSol.Stat == SOLQ_FIX {
		for i = 0; i < 3; i++ {
			rtk.RtkSol.Rr[i] = rtk.Xa[i]
			rtk.RtkSol.Qr[i] = float32(rtk.Pa[i+i*rtk.Na])
		}
		rtk.RtkSol.Qr[3] = float32(rtk.Pa[1])
		rtk.RtkSol.Qr[4] = float32(rtk.Pa[1+2*rtk.Na])
		rtk.RtkSol.Qr[5] = float32(rtk.Pa[2])
	} else {
		for i = 0; i < 3; i++ {
			rtk.RtkSol.Rr[i] = rtk.X[i]
			rtk.RtkSol.Qr[i] = float32(rtk.P[i+i*rtk.Nx])
		}
		rtk.RtkSol.Qr[3] = float32(rtk.P[1])
		rtk.RtkSol.Qr[4] = float32(rtk.P[2+rtk.Nx])
		rtk.RtkSol.Qr[5] = float32(rtk.P[2])
	}
	rtk.RtkSol.Dtr[0] = rtk.X[IC(0, opt)]
	rtk.RtkSol.Dtr[1] = rtk.X[IC(1, opt)] - rtk.X[IC(0, opt)]

	for i = 0; i < n && i < len(obs); i++ {
		for j = 0; j < opt.Nf; j++ {
			rtk.Ssat[obs[i].Sat-1].Snr[j] = obs[i].SNR[j]
		}
	}
	for i = 0; i < MAXSAT; i++ {
		for j = 0; j < opt.Nf; j++ {
			if rtk.Ssat[i].Slip[j]&3 > 0 {
				rtk.Ssat[i].Slipc[j]++
			}
			if rtk.Ssat[i].Fix[j] == 2 && stat != int(SOLQ_FIX) {
				rtk.Ssat[i].Fix[j] = 1
			}
		}
	}
}

/* test hold ambiguity -------------------------------------------------------*/
func (rtk *Rtk) TestHoldAmb() int {
	var i, j, stat int

	/* no fix-and-hold mode */
	if rtk.Opt.ModeAr != ARMODE_FIXHOLD {
		return 0
	}

	/* reset # of continuous fixed if new ambiguity introduced */
	for i = 0; i < len(rtk.Ssat); i++ {
		if rtk.Ssat[i].Fix[0] != 2 && rtk.Ssat[i].Fix[1] != 2 {
			continue
		}
		for j = 0; j < len(rtk.Ssat); j++ {
			if rtk.Ssat[j].Fix[0] != 2 && rtk.Ssat[j].Fix[1] != 2 {
				continue
			}
			if rtk.Ambc[j].flags[i] == 0 || rtk.Ambc[i].flags[j] == 0 {
				stat = 1
			}
			rtk.Ambc[j].flags[i], rtk.Ambc[i].flags[j] = 1, 1
		}
	}
	if stat > 0 {
		rtk.Nfix = 0
		return 0
	}
	/* test # of continuous fixed */

	if rtk.Nfix++; rtk.Nfix >= rtk.Opt.MinFix {
		return 1
	} else {
		return 0
	}
}

/* precise point positioning -------------------------------------------------*/
func (rtk *Rtk) PPPos(obs []ObsD, n int, nav *Nav) {
	var (
		rs, dts, vari, v, H, R, azel, xp, Pp []float64
		dr, std                              [3]float64
		str                                  string
		i, j, nv, info                       int
		svh, exc                             [MAXOBS]int
	)
	opt := &rtk.Opt
	stat := SOLQ_SINGLE

	Time2Str(obs[0].Time, &str, 2)
	Trace(4, "pppos   : time=%s nx=%d n=%d\n", str, rtk.Nx, n)

	rs = Mat(6, n)
	dts = Mat(2, n)
	vari = Mat(1, n)
	azel = Zeros(2, n)

	for i = 0; i < len(rtk.Ssat); i++ {
		for j = 0; j < opt.Nf; j++ {
			rtk.Ssat[i].Fix[j] = 0
		}
	}

	/* temporal update of ekf states */
	rtk.UpdateStatePPP(obs, n, nav)

	/* satellite positions and clocks */
	nav.SatPoss(obs[0].Time, obs, n, rtk.Opt.SatEph, rs, dts, vari, svh[:])

	/* exclude measurements of eclipsing satellite (block IIA) */
	if rtk.Opt.PosOpt[3] > 0 {
		TestEclipse(obs, n, nav, rs)
	}
	/* earth tides correction */
	if opt.TideCorr > 0 {
		if opt.TideCorr == 1 {
			TideDisp(GpsT2Utc(obs[0].Time), rtk.X, 1, &nav.Erp, opt.Odisp[0][:], dr[:])

		} else {
			TideDisp(GpsT2Utc(obs[0].Time), rtk.X, 7, &nav.Erp, opt.Odisp[0][:], dr[:])
		}
	}
	nv = n*rtk.Opt.Nf*2 + MAXSAT + 3
	xp = Mat(rtk.Nx, 1)
	Pp = Zeros(rtk.Nx, rtk.Nx)
	v = Mat(nv, 1)
	H = Mat(rtk.Nx, nv)
	R = Mat(nv, nv)

	for i = 0; i < MAX_ITER; i++ {

		MatCpy(xp, rtk.X, rtk.Nx, 1)
		MatCpy(Pp, rtk.P, rtk.Nx, rtk.Nx)

		/* prefit residuals */
		if nv = PPPResidual(0, obs, n, rs, dts, vari, svh[:], dr[:], exc[:], nav, xp, rtk, v, H, R, azel); nv == 0 {
			Trace(2, "%s ppp (%d) no valid obs data\n", str, i+1)
			break
		}
		/* measurement update of ekf states */
		if info = Filter(xp, Pp, H, v, R, rtk.Nx, nv); info > 0 {
			Trace(2, "%s ppp (%d) filter error info=%d\n", str, i+1, info)
			break
		}
		/* postfit residuals */
		if PPPResidual(i+1, obs, n, rs, dts, vari, svh[:], dr[:], exc[:], nav, xp, rtk, v, H, R, azel) > 0 {
			MatCpy(rtk.X, xp, rtk.Nx, 1)
			MatCpy(rtk.P, Pp, rtk.Nx, rtk.Nx)
			stat = SOLQ_PPP
			break
		}
	}
	if i >= MAX_ITER {
		Trace(2, "%s ppp (%d) iteration overflows\n", str, i)
	}
	if stat == SOLQ_PPP {

		/* ambiguity resolution in ppp */
		if PPPAmbiguity(rtk, obs, n, exc[:], nav, azel, xp, Pp) != 0 &&
			PPPResidual(9, obs, n, rs, dts, vari, svh[:], dr[:], exc[:], nav, xp, rtk, v, H, R, azel) > 0 {

			MatCpy(rtk.Xa, xp, rtk.Nx, 1)
			MatCpy(rtk.Pa, Pp, rtk.Nx, rtk.Nx)

			for i = 0; i < 3; i++ {
				std[i] = math.Sqrt(Pp[i+i*rtk.Nx])
			}
			if Norm(std[:], 3) < MAX_STD_FIX {
				stat = SOLQ_FIX
			}
		} else {
			rtk.Nfix = 0
		}
		/* update solution status */
		rtk.UpdateStat(obs, n, int(stat))

		/* hold fixed ambiguities */
		if stat == SOLQ_FIX && rtk.TestHoldAmb() > 0 {
			MatCpy(rtk.X, xp, rtk.Nx, 1)
			MatCpy(rtk.P, Pp, rtk.Nx, rtk.Nx)
			Trace(2, "%s hold ambiguity\n", str)
			rtk.Nfix = 0
		}
	}
}
