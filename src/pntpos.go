/*------------------------------------------------------------------------------
* pntpos.c : standard positioning
*
*          Copyright (C) 2007-2020 by T.TAKASU, All rights reserved.
*
* version : $Revision:$ $Date:$
* history : 2010/07/28 1.0  moved from rtkcmn.c
*                           changed api:
*                               pntpos()
*                           deleted api:
*                               pntvel()
*           2011/01/12 1.1  add option to include unhealthy satellite
*                           reject duplicated observation data
*                           changed api: ionocorr()
*           2011/11/08 1.2  enable snr mask for single-mode (rtklib_2.4.1_p3)
*           2012/12/25 1.3  add variable snr mask
*           2014/05/26 1.4  support galileo and beidou
*           2015/03/19 1.5  fix bug on ionosphere correction for GLO and BDS
*           2018/10/10 1.6  support api change of satexclude()
*           2020/11/30 1.7  support NavIC/IRNSS in pntpos()
*                           no support IONOOPT_LEX option in ioncorr()
*                           improve handling of TGD correction for each system
*                           use E1-E5b for Galileo dual-freq iono-correction
*                           use API sat2freq() to get carrier frequency
*                           add output of velocity estimation error in estvel()
*		    2022/05/31 1.0  rewrite pntpos.c with golang by fxb
*-----------------------------------------------------------------------------*/

package gnssgo

import (
	"fmt"
	"math"
)

const (
	NXParam int = 4 + 4 /* # of estimated parameters */

	MAXITR    = 10  /* max number of iteration for point pos */
	ERR_ION   = 5.0 /* ionospheric delay Std (m) */
	ERR_TROP  = 3.0 /* tropspheric delay Std (m) */
	ERR_SAAS  = 0.3 /* Saastamoinen model error Std (m) */
	ERR_BRDCI = 0.5 /* broadcast ionosphere model error factor */
	ERR_CBIAS = 0.3 /* code bias error Std (m) */
	REL_HUMI  = 0.7 /* relative humidity for Saastamoinen model */
	MIN_EL    = 5.0 * D2R /* min elevation for measurement error (rad) */)

/* pseudorange measurement error variance ------------------------------------*/
func VarianceErr(opt *PrcOpt, el float64, sys int) float64 {
	var fact, varr float64
	if sys == SYS_GLO {
		fact = float64(EFACT_GLO)
	} else {
		if sys == SYS_SBS {
			fact = float64(EFACT_SBS)
		} else {
			fact = float64(EFACT_GPS)
		}
	}

	if el < MIN_EL {
		el = MIN_EL
	}
	varr = SQR(opt.Err[0]) * (SQR(opt.Err[1]) + SQR(opt.Err[2])/math.Sin(el))
	if opt.IonoOpt == IONOOPT_IFLC {
		varr *= SQR(3.0) /* iono-free */
	}
	return SQR(fact) * varr
}

/* get group delay parameter (m) ---------------------------------------------*/
func (nav *Nav) GetTgd(sat int, dtype int) float64 {
	var i int
	sys := SatSys(sat, nil)

	if sys == SYS_GLO {
		for i = 0; i < nav.Ng(); i++ {
			if nav.Geph[i].Sat == sat {
				break
			}
		}
		if i >= nav.Ng() {
			return 0.0
		} else {
			return -nav.Geph[i].DTaun * CLIGHT
		}
	} else {
		for i = 0; i < nav.N(); i++ {
			if nav.Ephs[i].Sat == sat {
				break
			}
		}
		if i >= nav.N() {
			return 0
		} else {
			return nav.Ephs[i].Tgd[dtype] * CLIGHT
		}
	}
}

/* test SNR mask -------------------------------------------------------------*/
func snrmask(obs *ObsD, azel []float64, opt *PrcOpt) int {
	if TestSnr(0, 0, azel[1], float64(obs.SNR[0])*float64(SNR_UNIT), &opt.SnrMask) > 0 {
		return 0
	}
	if opt.IonoOpt == IONOOPT_IFLC {
		if TestSnr(0, 1, azel[1], float64(obs.SNR[1])*float64(SNR_UNIT), &opt.SnrMask) > 0 {
			return 0
		}
	}
	return 1
}

/* psendorange with code bias correction -------------------------------------*/
func Prange(obs *ObsD, nav *Nav, opt *PrcOpt, vari *float64) float64 {
	var (
		P1, P2, gamma, b1, b2 float64
		sat, sys              int
		code1, code2          uint8
	)
	sat = int(obs.Sat)
	sys = SatSys(sat, nil)
	P1 = obs.P[0]
	code1 = obs.Code[0]
	switch {
	case obs.Code[1] != 0:
		P2 = obs.P[1]
		code2 = obs.Code[1]
	case obs.Code[2] != 0:
		P2 = obs.P[2]
		code2 = obs.Code[2]
	}
	//P2 = obs.P[1]
	*vari = 0.0

	if P1 == 0.0 || (opt.IonoOpt == IONOOPT_IFLC && P2 == 0.0) {
		return 0.0
	}

	/* P1-C1,P2-C2 DCB correction */
	if sys == SYS_GPS || sys == SYS_GLO {
		if code1 == CODE_L1C {
			P1 += nav.CBias[sat-1][1] /* C1.P1 */
		}
		if code2 == CODE_L2C {
			P2 += nav.CBias[sat-1][2] /* C2.P2 */
		}
	}
	if opt.IonoOpt == IONOOPT_IFLC { /* dual-frequency */

		switch sys {
		case SYS_GPS, SYS_QZS: /* L1-L2,G1-G2 */
			gamma = SQR(FREQ1 / FREQ2)
			return (P2 - gamma*P1) / (1.0 - gamma)
		case SYS_GLO: /* G1-G2 */
			gamma = SQR(FREQ1_GLO / FREQ2_GLO)
			return (P2 - gamma*P1) / (1.0 - gamma)
		case SYS_GAL: /* E1-E5b */
			gamma = SQR(FREQ1 / FREQ7)
			if GetSelEph(SYS_GAL) > 0 { /* F/NAV */
				P2 -= nav.GetTgd(sat, 0) - nav.GetTgd(sat, 1) /* BGD_E5aE5b */
			}
			return (P2 - gamma*P1) / (1.0 - gamma)
		case SYS_CMP: /* B1-B2 */
			if code1 == CODE_L2I {
				gamma = SQR(FREQ1_CMP / FREQ2_CMP)
			} else {
				gamma = SQR(FREQ1 / FREQ2_CMP)
			}

			if code1 == CODE_L2I {
				b1 = nav.GetTgd(sat, 0) /* TGD_B1I */
			} else if code1 == CODE_L1P {
				b1 = nav.GetTgd(sat, 2) /* TGD_B1Cp */
			} else {
				b1 = nav.GetTgd(sat, 2) + nav.GetTgd(sat, 4) /* TGD_B1Cp+ISC_B1Cd */
			}
			b2 = nav.GetTgd(sat, 1) /* TGD_B2I/B2bI (m) */
			return ((P2 - gamma*P1) - (b2 - gamma*b1)) / (1.0 - gamma)
		default:
			if sys == SYS_IRN { /* L5-S */
				gamma = SQR(FREQ5 / FREQ9)
				return (P2 - gamma*P1) / (1.0 - gamma)
			}
		}
	} else { /* single-freq (L1/E1/B1) */
		*vari = SQR(ERR_CBIAS)

		switch sys {
		case SYS_GPS, SYS_QZS: /* L1 */
			b1 = nav.GetTgd(sat, 0) /* TGD (m) */
			return P1 - b1
		case SYS_GLO: /* G1 */
			gamma = SQR(FREQ1_GLO / FREQ2_GLO)
			b1 = nav.GetTgd(sat, 0) /* -dtaun (m) */
			return P1 - b1/(gamma-1.0)
		case SYS_GAL: /* E1 */
			if GetSelEph(SYS_GAL) > 0 {
				b1 = nav.GetTgd(sat, 0) /* BGD_E1E5a */
			} else {
				b1 = nav.GetTgd(sat, 1) /* BGD_E1E5b */
			}
			return P1 - b1
		case SYS_CMP: /* B1I/B1Cp/B1Cd */
			if code1 == CODE_L2I {
				b1 = nav.GetTgd(sat, 0) /* TGD_B1I */
			} else if code1 == CODE_L1P {
				b1 = nav.GetTgd(sat, 2) /* TGD_B1Cp */
			} else {
				b1 = nav.GetTgd(sat, 2) + nav.GetTgd(sat, 4) /* TGD_B1Cp+ISC_B1Cd */
			}
			return P1 - b1
		case SYS_IRN: /* L5 */
			gamma = SQR(FREQ9 / FREQ5)
			b1 = nav.GetTgd(sat, 0) /* TGD (m) */
			return P1 - gamma*b1
		}
	}
	return P1
}

/* ionospheric correction ------------------------------------------------------
* compute ionospheric correction
* args   : gtime_t time     I   time
*          nav_t  *nav      I   navigation data
*          int    sat       I   satellite number
*          double *pos      I   receiver position {lat,lon,h} (rad|m)
*          double *azel     I   azimuth/elevation angle {az,el} (rad)
*          int    ionoopt   I   ionospheric correction option (IONOOPT_???)
*          double *ion      O   ionospheric delay (L1) (m)
*          double *var      O   ionospheric delay (L1) variance (m^2)
* return : status(1:ok,0:error)
*-----------------------------------------------------------------------------*/
func (nav *Nav) IonoCorr(time Gtime, sat int, pos, azel []float64, ionoopt int, ion, vari *float64) int {
	Trace(4, "ionocorr: time=%s opt=%d sat=%2d pos=%.3f %.3f azel=%.3f %.3f\n",
		TimeStr(time, 3), ionoopt, sat, pos[0]*R2D, pos[1]*R2D, azel[0]*R2D,
		azel[1]*R2D)

	/* GPS broadcast ionosphere model */
	if ionoopt == IONOOPT_BRDC {
		*ion = IonModel(time, nav.Ion_gps[:], pos, azel)
		*vari = SQR(*ion * ERR_BRDCI)
		return 1
	}
	/* SBAS ionosphere model */
	if ionoopt == IONOOPT_SBAS {
		return SbsIonCorr(time, nav, pos, azel, ion, vari)
	}
	/* IONEX TEC model */
	if ionoopt == IONOOPT_TEC {
		return nav.IonTec(time, pos, azel, 1, ion, vari)
	}
	/* QZSS broadcast ionosphere model */
	if ionoopt == IONOOPT_QZS && Norm(nav.Ion_qzs[:], 8) > 0.0 {
		*ion = IonModel(time, nav.Ion_qzs[:], pos, azel)
		*vari = SQR(*ion * ERR_BRDCI)
		return 1
	}
	*ion = 0.0
	*vari = 0.0
	if ionoopt == IONOOPT_OFF {
		*vari = SQR(ERR_ION)
	}

	return 1
}

/* tropospheric correction -----------------------------------------------------
* compute tropospheric correction
* args   : gtime_t time     I   time
*          nav_t  *nav      I   navigation data
*          double *pos      I   receiver position {lat,lon,h} (rad|m)
*          double *azel     I   azimuth/elevation angle {az,el} (rad)
*          int    tropopt   I   tropospheric correction option (TROPOPT_???)
*          double *trp      O   tropospheric delay (m)
*          double *var      O   tropospheric delay variance (m^2)
* return : status(1:ok,0:error)
*-----------------------------------------------------------------------------*/
func (nav *Nav) TropCorr(time Gtime, pos, azel []float64, tropopt int, trp, vari *float64) int {
	Trace(4, "tropcorr: time=%s opt=%d pos=%.3f %.3f azel=%.3f %.3f\n",
		TimeStr(time, 3), tropopt, pos[0]*R2D, pos[1]*R2D, azel[0]*R2D,
		azel[1]*R2D)

	/* Saastamoinen model */
	if tropopt == TROPOPT_SAAS || tropopt == TROPOPT_EST || tropopt == TROPOPT_ESTG {
		*trp = TropModel(time, pos, azel, REL_HUMI)
		*vari = SQR(ERR_SAAS / (math.Sin(azel[1]) + 0.1))
		return 1
	}
	/* SBAS (MOPS) troposphere model */
	if tropopt == TROPOPT_SBAS {
		*trp = SbsTropCorr(time, pos, azel, vari)
		return 1
	}
	/* no correction */
	*trp = 0.0
	*vari = 0.0
	if tropopt == TROPOPT_OFF {
		*vari = SQR(ERR_TROP)
	}

	return 1
}

/* pseudorange residuals -----------------------------------------------------*/
func Residuals(iter int, obs []ObsD, n int, rs, dts, vare []float64, svh []int,
	nav *Nav, x []float64, opt *PrcOpt, v, H, vari, azel []float64, vsat []int,
	resp []float64, ns *int) int {
	var (
		time                                           Gtime
		r, freq, dion, dtrp, vmeas, vion, vtrp, dtr, P float64
		rr, pos, e                                     [3]float64
		i, j, nv, sat, sys                             int
		mask                                           [NXParam - 3]int
	)

	Trace(4, "resprng : n=%d  iter=%d\n", n, iter)

	for i = 0; i < 3; i++ {
		rr[i] = x[i]
	}
	dtr = x[3]
	dion, dtrp, vmeas, vion, vtrp = 0.0, 0.0, 0.0, 0.0, 0.0 /*initialized variables to avoid a bug in windows add by cjb*/
	Ecef2Pos(rr[:], pos[:])
	*ns = 0
	for i = *ns; i < n && i < MAXOBS; i++ {
		vsat[i] = 0
		azel[i*2], azel[1+i*2], resp[i] = 0.0, 0.0, 0.0
		time = obs[i].Time
		sat = int(obs[i].Sat)
		if sys = SatSys(sat, nil); sys == 0 {
			continue
		}

		/* reject duplicated observation data */
		if i < n-1 && i < MAXOBS-1 && sat == obs[i+1].Sat {
			Trace(2, "duplicated obs data %s sat=%d\n", TimeStr(time, 3), sat)
			i++
			continue
		}
		/* excluded satellite? */
		if SatExclude(sat, vare[i], svh[i], opt) > 0 {
			continue
		}

		/* geometric distance */
		if r = GeoDist(rs[i*6:], rr[:], e[:]); r <= 0.0 {
			continue
		}

		if iter > 0 {
			/* test elevation mask */
			if SatAzel(pos[:], e[:], azel[i*2:]) < opt.Elmin {
				continue
			}

			/* test SNR mask */
			if snrmask(&obs[i], azel[i*2:], opt) == 0 {
				continue
			}

			/* ionospheric correction */
			if nav.IonoCorr(time, sat, pos[:], azel[i*2:], opt.IonoOpt, &dion, &vion) == 0 {
				continue
			}
			if freq = Sat2Freq(sat, obs[i].Code[0], nav); freq == 0.0 {
				continue
			}
			dion *= SQR(FREQ1 / freq)
			vion *= SQR(FREQ1 / freq)

			/* tropospheric correction */
			if nav.TropCorr(time, pos[:], azel[i*2:], opt.TropOpt, &dtrp, &vtrp) == 0 {
				continue
			}
		}
		/* psendorange with code bias correction */
		if P = Prange(&obs[i], nav, opt, &vmeas); P == 0.0 {
			continue
		}

		/* pseudorange residual */
		v[nv] = P - (r + dtr - CLIGHT*dts[i*2] + dion + dtrp)

		/* design matrix */
		for j = 0; j < NXParam; j++ {
			if j < 3 {
				H[j+nv*NXParam] = -e[j]
			} else {
				H[j+nv*NXParam] = 0.0
				if j == 3 {
					H[j+nv*NXParam] = 1.0
				}
			}
		}
		/* time system offset and receiver bias correction */
		switch sys {
		case SYS_GLO:
			v[nv] -= x[4]
			H[4+nv*NXParam] = 1.0
			mask[1] = 1
		case SYS_GAL:
			v[nv] -= x[5]
			H[5+nv*NXParam] = 1.0
			mask[2] = 1
		case SYS_CMP:
			v[nv] -= x[6]
			H[6+nv*NXParam] = 1.0
			mask[3] = 1
		case SYS_IRN:
			v[nv] -= x[7]
			H[7+nv*NXParam] = 1.0
			mask[4] = 1
		default:
			mask[0] = 1
		}

		vsat[i] = 1
		resp[i] = v[nv]
		(*ns)++

		/* variance of pseudorange error */
		vari[nv] = VarianceErr(opt, azel[1+i*2], sys) + vare[i] + vmeas + vion + vtrp
		nv++
		Trace(2, "sat=%2d azel=%5.1f %4.1f res=%7.3f sig=%5.3f\n", obs[i].Sat,
			azel[i*2]*R2D, azel[1+i*2]*R2D, resp[i], math.Sqrt(vari[nv-1]))
	}
	/* constraint to avoid rank-deficient */
	for i = 0; i < NXParam-3; i++ {
		if mask[i] > 0 {
			continue
		}
		v[nv] = 0.0
		for j = 0; j < NXParam; j++ {
			H[j+nv*NXParam] = 0.0
			if j == i+3 {
				H[j+nv*NXParam] = 1.0
			}
		}
		vari[nv] = 0.01
		nv++
	}
	return nv
}

/* validate solution ---------------------------------------------------------*/
func ValSol(azel []float64, vsat []int, n int, opt *PrcOpt, v []float64, nv, nx int, msg *string) int {
	var (
		azels [MAXOBS * 2]float64
		dop   [4]float64

		i, ns int
	)

	Trace(4, "valsol  : n=%d nv=%d\n", n, nv)

	/* Chi-square validation of residuals */
	vv := Dot(v, v, nv)
	if nv > nx && vv > chisqr[nv-nx-1] {
		*msg = fmt.Sprintf("chi-square error nv=%d vv=%.1f cs=%.1f", nv, vv, chisqr[nv-nx-1])
		return 0
	}
	/* large GDOP check */

	for i, ns = 0, 0; i < n; i++ {
		if vsat[i] == 0 {
			continue
		}
		azels[ns*2] = azel[i*2]
		azels[1+ns*2] = azel[1+i*2]
		ns++
	}
	DOPs(ns, azels[:], opt.Elmin, dop[:])
	if dop[0] <= 0.0 || dop[0] > opt.MaxGdop {
		*msg = fmt.Sprintf("gdop error nv=%d gdop=%.1f", nv, dop[0])
		return 0
	}
	return 1
}

/* estimate receiver position ------------------------------------------------*/
func EstimatePos(obs []ObsD, n int, rs, dts, vare []float64, svh []int, nav *Nav,
	opt *PrcOpt, sol *Sol, azel []float64, vsat []int, resp []float64, msg *string) int {
	var (
		x, dx      [NXParam]float64
		Q          [NXParam * NXParam]float64
		v, H, vari []float64
	)

	var sig float64
	var i, j, k, info, stat, nv, ns int

	Trace(4, "estpos  : n=%d\n", n)

	v = Mat(n+4, 1)
	H = Mat(NXParam, n+4)
	vari = Mat(n+4, 1)

	for i = 0; i < 3; i++ {
		x[i] = sol.Rr[i]
	}

	for i = 0; i < MAXITR; i++ {

		/* pseudorange residuals (m) */
		nv = Residuals(i, obs, n, rs, dts, vare, svh, nav, x[:], opt, v, H, vari, azel, vsat, resp, &ns)

		if nv < NXParam {
			*msg = fmt.Sprintf("lack of valid sats ns=%d", nv)
			break
		}
		/* weighted by Std */
		for j = 0; j < nv; j++ {
			sig = math.Sqrt(vari[j])
			v[j] /= sig
			for k = 0; k < NXParam; k++ {
				H[k+j*NXParam] /= sig
			}
		}
		/* least square estimation */
		if info = LSQ(H, v, NXParam, nv, dx[:], Q[:]); info != 0 {
			*msg = fmt.Sprintf("lsq error info=%d", info)
			break
		}
		for j = 0; j < NXParam; j++ {
			x[j] += dx[j]
		}
		if Norm(dx[:], NXParam) < 1e-4 {
			sol.Type = 0
			sol.Time = TimeAdd(obs[0].Time, -x[3]/CLIGHT)
			sol.Dtr[0] = x[3] / CLIGHT /* receiver clock bias (s) */
			sol.Dtr[1] = x[4] / CLIGHT /* GLO-GPS time offset (s) */
			sol.Dtr[2] = x[5] / CLIGHT /* GAL-GPS time offset (s) */
			sol.Dtr[3] = x[6] / CLIGHT /* BDS-GPS time offset (s) */
			sol.Dtr[4] = x[7] / CLIGHT /* IRN-GPS time offset (s) */
			for j = 0; j < 6; j++ {
				sol.Rr[j] = 0.0
				if j < 3 {
					sol.Rr[j] = x[j]
				}
			}
			for j = 0; j < 3; j++ {
				sol.Qr[j] = float32(Q[j+j*NXParam])
			}
			sol.Qr[3] = float32(Q[1])         /* cov xy */
			sol.Qr[4] = float32(Q[2+NXParam]) /* cov yz */
			sol.Qr[5] = float32(Q[2])         /* cov zx */
			sol.Ns = uint8(ns)
			sol.Age, sol.Ratio = 0.0, 0.0

			/* validate solution */
			if stat = ValSol(azel, vsat, n, opt, v, nv, NXParam, msg); stat > 0 {

				sol.Stat = SOLQ_SINGLE
				if opt.SatEph == EPHOPT_SBAS {
					sol.Stat = SOLQ_SBAS

				}
			}

			return stat
		}
	}
	if i >= MAXITR {
		*msg = fmt.Sprintf("iteration divergent i=%d", i)
	}

	return 0
}

/* RAIM FDE (failure detection and exclution) -------------------------------*/
func RaimFde(obs []ObsD, n int, rs, dts, vare []float64, svh []int, nav *Nav, opt *PrcOpt, sol *Sol,
	azel []float64, vsat []int, resp []float64, msg *string) int {
	var (
		obs_e []ObsD = make([]ObsD, n)
		sol_e Sol
		msg_e string

		rs_e, dts_e, vare_e, azel_e, resp_e []float64
		rms_e, rms                          float64 = 0, 100.0
		i, j, k, nvsat, stat, sat           int
		svh_e, vsat_e                       []int
	)

	Trace(4, "raim_fde: %s n=%2d\n", TimeStr(obs[0].Time, 0), n)

	rs_e = Mat(6, n)
	dts_e = Mat(2, n)
	vare_e = Mat(1, n)
	azel_e = Zeros(2, n)
	svh_e = IMat(1, n)
	vsat_e = IMat(1, n)
	resp_e = Mat(1, n)

	for i = 0; i < n; i++ {

		/* satellite exclution */
		for j, k = 0, 0; j < n; j++ {
			if j == i {
				continue
			}
			obs_e[k] = obs[j]
			MatCpy(rs_e[6*k:], rs[6*j:], 6, 1)
			MatCpy(dts_e[2*k:], dts[2*j:], 2, 1)
			vare_e[k] = vare[j]
			svh_e[k] = svh[j]
			k++
		}
		/* estimate receiver position without a satellite */
		if EstimatePos(obs_e, n-1, rs_e, dts_e, vare_e, svh_e, nav, opt, &sol_e, azel_e,
			vsat_e, resp_e, &msg_e) == 0 {
			Trace(2, "raim_fde: exsat=%2d (%s)\n", obs[i].Sat, *msg)
			continue
		}
		for j, nvsat, rms_e = 0, 0, 0.0; j < n-1; j++ {
			if vsat_e[j] == 0 {
				continue
			}
			rms_e += SQR(resp_e[j])
			nvsat++
		}
		if nvsat < 5 {
			Trace(2, "raim_fde: exsat=%2d lack of satellites nvsat=%2d\n",
				obs[i].Sat, nvsat)
			continue
		}
		rms_e = math.Sqrt(rms_e / float64(nvsat))

		Trace(2, "raim_fde: exsat=%2d rms=%8.3f\n", obs[i].Sat, rms_e)

		if rms_e > rms {
			continue
		}

		/* save result */
		for j, k = 0, 0; j < n; j++ {
			if j == i {
				continue
			}
			MatCpy(azel[2*j:], azel_e[2*k:], 2, 1)
			vsat[j] = vsat_e[k]
			resp[j] = resp_e[k]
			k++
		}
		stat = 1
		*sol = sol_e
		sat = int(obs[i].Sat)
		rms = rms_e
		vsat[i] = 0
		*msg = msg_e
	}
	if stat > 0 {
		var name, tstr string
		Time2Str(obs[0].Time, &tstr, 2)
		SatNo2Id(sat, &name)
		Trace(2, "%s: %s excluded by raim\n", tstr[11:], name)
	}

	return stat
}

/* range rate residuals ------------------------------------------------------*/
func ResidualDop(obs []ObsD, n int, rs, dts []float64, nav *Nav, rr, x, azel []float64,
	vsat []int, err float64, v, H []float64) int {
	var (
		freq, rate, cosel, sig float64
		pos, a, e, vs          [3]float64
		E                      [9]float64
		i, j, nv               int
	)

	Trace(4, "resdop  : n=%d\n", n)

	Ecef2Pos(rr, pos[:])
	XYZ2Enu(pos[:], E[:])

	for i = 0; i < n && i < MAXOBS; i++ {

		freq = Sat2Freq(int(obs[i].Sat), obs[i].Code[0], nav)

		if obs[i].D[0] == 0.0 || freq == 0.0 || vsat[i] == 0 || Norm(rs[3+i*6:], 3) <= 0.0 {
			continue
		}
		/* LOS (line-of-sight) vector in ECEF */
		cosel = math.Cos(azel[1+i*2])
		a[0] = math.Sin(azel[i*2]) * cosel
		a[1] = math.Cos(azel[i*2]) * cosel
		a[2] = math.Sin(azel[1+i*2])
		MatMul("TN", 3, 1, 3, 1.0, E[:], a[:], 0.0, e[:])

		/* satellite velocity relative to receiver in ECEF */
		for j = 0; j < 3; j++ {
			vs[j] = rs[j+3+i*6] - x[j]
		}
		/* range rate with earth rotation correction */
		rate = Dot(vs[:], e[:], 3) + OMGE/CLIGHT*(rs[4+i*6]*rr[0]+rs[1+i*6]*x[0]-
			rs[3+i*6]*rr[1]-rs[i*6]*x[1])

		/* Std of range rate error (m/s) */
		sig = 1.0
		if err > 0.0 {
			sig = err * CLIGHT / freq
		}

		/* range rate residual (m/s) */
		v[nv] = (-obs[i].D[0]*CLIGHT/freq - (rate + x[3] - CLIGHT*dts[1+i*2])) / sig

		/* design matrix */
		for j = 0; j < 4; j++ {

			H[j+nv*4] = 1.0 / sig
			if j < 3 {
				H[j+nv*4] = -e[j] / sig
			}
		}
		nv++
	}
	return nv
}

/* estimate receiver velocity ------------------------------------------------*/
func EstVel(obs []ObsD, n int, rs, dts []float64, nav *Nav, opt *PrcOpt, sol *Sol, azel []float64, vsat []int) {
	var (
		x, dx    [4]float64
		Q        [16]float64
		v, H     []float64
		i, j, nv int
	)
	err := opt.Err[4] /* Doppler error (Hz) */

	Trace(4, "estvel  : n=%d\n", n)

	v = Mat(n, 1)
	H = Mat(4, n)

	for i = 0; i < MAXITR; i++ {

		/* range rate residuals (m/s) */
		if nv = ResidualDop(obs, n, rs, dts, nav, sol.Rr[:], x[:], azel, vsat, err, v, H); nv < 4 {
			break
		}
		/* least square estimation */
		if LSQ(H, v, 4, nv, dx[:], Q[:]) != 0 {
			break
		}

		for j = 0; j < 4; j++ {
			x[j] += dx[j]
		}

		if Norm(dx[:], 4) < 1e-6 {
			MatCpy(sol.Rr[3:], x[:], 3, 1)
			sol.Qv[0] = float32(Q[0])  /* xx */
			sol.Qv[1] = float32(Q[5])  /* yy */
			sol.Qv[2] = float32(Q[10]) /* zz */
			sol.Qv[3] = float32(Q[1])  /* xy */
			sol.Qv[4] = float32(Q[6])  /* yz */
			sol.Qv[5] = float32(Q[2])  /* zx */
			break
		}
	}

}

/* single-point positioning ----------------------------------------------------
* compute receiver position, velocity, clock bias by single-point positioning
* with pseudorange and doppler observables
* args   : obsd_t *obs      I   observation data
*          int    n         I   number of observation data
*          nav_t  *nav      I   navigation data
*          prcopt_t *opt    I   processing options
*          sol_t  *sol      IO  solution
*          double *azel     IO  azimuth/elevation angle (rad) (NULL: no output)
*          ssat_t *ssat     IO  satellite status              (NULL: no output)
*          char   *msg      O   error message for error exit
* return : status(1:ok,0:error)
*-----------------------------------------------------------------------------*/
func PntPos(obs []ObsD, n int, nav *Nav, opt *PrcOpt, sol *Sol, azel []float64, ssat []SSat, msg *string) int {
	var (
		opt_                       PrcOpt = *opt
		rs, dts, vari, azel_, resp []float64
		i, stat                    int
		vsat, svh                  [MAXOBS]int
	)

	Trace(4, "pntpos  : tobs=%s n=%d\n", TimeStr(obs[0].Time, 3), n)

	sol.Stat = SOLQ_NONE

	if n <= 0 {
		*msg = "no observation data"
		return 0
	}
	sol.Time = obs[0].Time
	*msg = ""

	rs = Mat(6, n)
	dts = Mat(2, n)
	vari = Mat(1, n)
	azel_ = Zeros(2, n)
	resp = Mat(1, n)

	if opt_.Mode != PMODE_SINGLE { /* for precise positioning */
		opt_.IonoOpt = IONOOPT_BRDC
		opt_.TropOpt = TROPOPT_SAAS
	}
	/* satellite positons, velocities and clocks */
	nav.SatPoss(sol.Time, obs, n, opt_.SatEph, rs, dts, vari, svh[:])

	/* estimate receiver position with pseudorange */
	stat = EstimatePos(obs, n, rs, dts, vari, svh[:], nav, &opt_, sol, azel_, vsat[:], resp, msg)

	/* RAIM FDE */
	if stat == 0 && n >= 6 && opt.PosOpt[4] > 0 {
		stat = RaimFde(obs, n, rs, dts, vari, svh[:], nav, &opt_, sol, azel_, vsat[:], resp, msg)
	}
	/* estimate receiver velocity with Doppler */
	if stat > 0 {
		EstVel(obs, n, rs, dts, nav, &opt_, sol, azel_, vsat[:])
	}
	if azel != nil {
		for i = 0; i < n*2; i++ {
			azel[i] = azel_[i]
		}
	}
	if ssat != nil {
		for i = 0; i < MAXSAT; i++ {
			if i > len(ssat) {
				break
			}
			ssat[i].Vs = 0
			ssat[i].Azel[0], ssat[i].Azel[1] = 0.0, 0.0
			ssat[i].Resp[0], ssat[i].Resc[0] = 0.0, 0.0
			ssat[i].Snr[0] = 0
		}
		for i = 0; i < n; i++ {
			ssat[obs[i].Sat-1].Azel[0] = float64(azel_[i*2])
			ssat[obs[i].Sat-1].Azel[1] = float64(azel_[1+i*2])
			ssat[obs[i].Sat-1].Snr[0] = obs[i].SNR[0]
			if vsat[i] == 0 {
				continue
			}
			ssat[obs[i].Sat-1].Vs = 1
			ssat[obs[i].Sat-1].Resp[0] = float32(resp[i])
		}
	}
	return stat
}
