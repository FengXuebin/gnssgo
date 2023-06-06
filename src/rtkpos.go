/*------------------------------------------------------------------------------
* rtkpos.c : precise positioning
*
*          Copyright (C) 2007-2020 by T.TAKASU, All rights reserved.
*
* version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
* history : 2007/01/12 1.0  new
*           2007/03/13 1.1  add slip detection by LLI flag
*           2007/04/18 1.2  add antenna pcv correction
*                           change rtkpos argin
*           2008/07/18 1.3  refactored
*           2009/01/02 1.4  modify rtk positioning api
*           2009/03/09 1.5  support glonass, gallileo and qzs
*           2009/08/27 1.6  fix bug on numerical exception
*           2009/09/03 1.7  add check of valid satellite number
*                           add check time sync for moving-base
*           2009/11/23 1.8  add api rtkopenstat(),rtkclosestat()
*                           add receiver h/w bias estimation
*                           add solution status output
*           2010/04/04 1.9  support ppp-kinematic and ppp-static modes
*                           support earth tide correction
*                           changed api:
*                               rtkpos()
*           2010/09/07 1.10 add elevation mask to hold ambiguity
*           2012/02/01 1.11 add extended receiver error model
*                           add glonass interchannel bias correction
*                           add slip detectior by L1-L5 gf jump
*                           output snr of rover receiver in residuals
*           2013/03/10 1.12 add otl and pole tides corrections
*           2014/05/26 1.13 support beidou and galileo
*                           add output of gal-gps and bds-gps time offset
*           2014/05/28 1.14 fix bug on memory exception with many sys and freq
*           2014/08/26 1.15 add functino to swap sol-stat file with keywords
*           2014/10/21 1.16 fix bug on beidou amb-res with pos2-bdsarmode=0
*           2014/11/08 1.17 fix bug on ar-degradation by unhealthy satellites
*           2015/03/23 1.18 residuals referenced to reference satellite
*           2015/05/20 1.19 no output solution status file with Q=0
*           2015/07/22 1.20 fix bug on base station position setting
*           2016/07/30 1.21 suppress single solution if !prcopt.outsingle
*                           fix bug on slip detection of backward filter
*           2016/08/20 1.22 fix bug on ddres() function
*           2018/10/10 1.13 support api change of satexclude()
*           2018/12/15 1.14 disable ambiguity resolution for gps-qzss
*           2019/08/19 1.15 fix bug on return value of resamb_LAMBDA()
*           2020/11/30 1.16 support of NavIC/IRNSS in API rtkpos()
*                           add detecting cycle slips by L1-Lx GF phase jump
*                           delete GLONASS IFB correction in ddres()
*                           use integer types in stdint.h
*		    2022/05/31 1.0  rewrite rtkpos.c with golang by fxb
*-----------------------------------------------------------------------------*/
package gnssgo

import (
	"fmt"
	"math"
	"os"
)

/* constants/macros ----------------------------------------------------------*/
/* number of parameters (pos,ionos,tropos,hw-bias,phase-bias,real,estimated) */
func RNF(opt *PrcOpt) int {
	if opt.IonoOpt == IONOOPT_IFLC {
		return 1
	} else {
		return opt.Nf
	}
}
func RNP(opt *PrcOpt) int {
	if opt.Dynamics != 0 {
		return 9
	} else {
		return 3
	}
}
func RNT(opt *PrcOpt) int {
	switch {
	case opt.TropOpt < TROPOPT_EST:
		return 0
	case opt.TropOpt < TROPOPT_ESTG:
		return 2
	default:
		return 6
	}
}
func RNL(opt *PrcOpt) int {
	if opt.GloModeAr != 2 {
		return 0
	}
	return NFREQGLO
}

func RNB(opt *PrcOpt) int {
	if opt.Mode <= PMODE_DGPS {
		return 0
	}
	return MAXSAT * RNF(opt)
}
func RNI(opt *PrcOpt) int {
	if opt.IonoOpt == IONOOPT_EST {
		return MAXSAT
	} else {
		return 0
	}
}

/* number of parameters (pos,ionos,tropos,hw-bias,phase-bias,real,estimated) */
func RNR(opt *PrcOpt) int { return RNP(opt) + RNI(opt) + RNT(opt) + RNL(opt) }
func RNX(opt *PrcOpt) int { return RNR(opt) + RNB(opt) }

/* state variable index */
func RII(s int, opt *PrcOpt) int    { return RNP(opt) + s - 1 }                   /* ionos (s:satellite no) */
func RIT(r int, opt *PrcOpt) int    { return RNP(opt) + RNI(opt) + RNT(opt)/2*r } /* tropos (r:0=rov,1:ref) */
func RIL(f int, opt *PrcOpt) int    { return RNP(opt) + RNI(opt) + RNT(opt) + f } /* receiver h/w bias */
func RIB(s, f int, opt *PrcOpt) int { return RNR(opt) + MAXSAT*f + s - 1 }        /* phase bias (s:satno,f:freq) */
const (
	// const VAR_POS     float64= 9000.0 /* initial variance of receiver pos (m^2) */
	// const  VAR_VEL    float64= SQR(10.0) /* initial variance of receiver vel ((m/s)^2) */
	// const  VAR_ACC    float64= SQR(10.0) /* initial variance of receiver acc ((m/ss)^2) */
	VAR_HWBIAS float64 = 1.0 /* initial variance of h/w bias ((m/MHz)^2) */
	// const  VAR_GRA    float64= SQR(0.001) /* initial variance of gradient (m^2) */
	// const  INIT_ZWD   float64= 0.15     /* initial zwd (m) */
	PRN_HWBIAS float64 = 1e-6 /* process noise of h/w bias (m/MHz/sqrt(s)) */
	//const GAP_RESION float64 = 120      /* gap to reset ionosphere parameters (epochs) */
	MAXACC      float64 = 30.0  /* max accel for doppler slip detection (m/s^2) */
	VAR_HOLDAMB float64 = 0.001 /* constraint to hold ambiguity (cycle^2) */
	TTOL_MOVEB  float64 = float64(1.0 + 2*DTTOL)
	/* time sync tolerance for moving-baseline (s) */
	INIT_ZWD float64 = 0.15 /* initial zwd (m) */)

/* global variables ----------------------------------------------------------*/
var (
	statlevel int      = 0   /* rtk status output level (0:off) */
	fp_stat   *os.File = nil /* rtk status file pointer */
	file_stat string   = ""  /* rtk status file original path */
	time_stat Gtime /* rtk status file time */)

/* open solution status file ---------------------------------------------------
* open solution status file and set output level
* args   : char     *file   I   rtk status file
*          int      level   I   rtk status level (0: off)
* return : status (1:ok,0:error)
* notes  : file can constain time keywords (%Y,%y,%m...) defined in reppath().
*          The time to replace keywords is based on UTC of CPU time.
* output : solution status file record format
*
*   $POS,week,tow,stat,posx,posy,posz,posxf,posyf,poszf
*          week/tow : gps week no/time of week (s)
*          stat     : solution status
*          posx/posy/posz    : position x/y/z ecef (m) float
*          posxf/posyf/poszf : position x/y/z ecef (m) fixed
*
*   $VELACC,week,tow,stat,vele,veln,velu,acce,accn,accu,velef,velnf,veluf,accef,accnf,accuf
*          week/tow : gps week no/time of week (s)
*          stat     : solution status
*          vele/veln/velu    : velocity e/n/u (m/s) float
*          acce/accn/accu    : acceleration e/n/u (m/s^2) float
*          velef/velnf/veluf : velocity e/n/u (m/s) fixed
*          accef/accnf/accuf : acceleration e/n/u (m/s^2) fixed
*
*   $CLK,week,tow,stat,clk1,clk2,clk3,clk4
*          week/tow : gps week no/time of week (s)
*          stat     : solution status
*          clk1     : receiver clock bias GPS (ns)
*          clk2     : receiver clock bias GLO-GPS (ns)
*          clk3     : receiver clock bias GAL-GPS (ns)
*          clk4     : receiver clock bias BDS-GPS (ns)
*
*   $ION,week,tow,stat,sat,az,el,ion,ion-fixed
*          week/tow : gps week no/time of week (s)
*          stat     : solution status
*          sat      : satellite id
*          az/el    : azimuth/elevation angle(deg)
*          ion      : vertical ionospheric delay L1 (m) float
*          ion-fixed: vertical ionospheric delay L1 (m) fixed
*
*   $TROP,week,tow,stat,rcv,ztd,ztdf
*          week/tow : gps week no/time of week (s)
*          stat     : solution status
*          rcv      : receiver (1:rover,2:base station)
*          ztd      : zenith total delay (m) float
*          ztdf     : zenith total delay (m) fixed
*
*   $HWBIAS,week,tow,stat,frq,bias,biasf
*          week/tow : gps week no/time of week (s)
*          stat     : solution status
*          frq      : frequency (1:L1,2:L2,...)
*          bias     : h/w bias coefficient (m/MHz) float
*          biasf    : h/w bias coefficient (m/MHz) fixed
*
*   $SAT,week,tow,sat,frq,az,el,resp,resc,vsat,snr,fix,slip,lock,outc,slipc,rejc
*          week/tow : gps week no/time of week (s)
*          sat/frq  : satellite id/frequency (1:L1,2:L2,...)
*          az/el    : azimuth/elevation angle (deg)
*          resp     : pseudorange residual (m)
*          resc     : carrier-phase residual (m)
*          vsat     : valid data flag (0:invalid,1:valid)
*          snr      : signal strength (dbHz)
*          fix      : ambiguity flag  (0:no data,1:float,2:fixed,3:hold,4:ppp)
*          slip     : cycle-slip flag (bit1:slip,bit2:parity unknown)
*          lock     : carrier-lock count
*          outc     : data outage count
*          slipc    : cycle-slip count
*          rejc     : data reject (outlier) count
*
*-----------------------------------------------------------------------------*/
func RtkOpenStat(file string, level int) int {
	var (
		path string
		err  error
	)
	time := Utc2GpsT(TimeGet())

	Trace(4, "rtkopenstat: file=%s level=%d\n", file, level)

	if level <= 0 {
		return 0
	}

	RepPath(file, &path, time, "", "")

	fp_stat, err = os.OpenFile(path, os.O_CREATE|os.O_WRONLY|os.O_TRUNC, os.ModeAppend|os.ModePerm)
	if fp_stat == nil || err != nil {
		Trace(2, "rtkopenstat: file open error path=%s\n", path)
		return 0
	}
	file_stat = file
	time_stat = time
	statlevel = level
	return 1
}

/* close solution status file --------------------------------------------------
* close solution status file
* args   : none
* return : none
*-----------------------------------------------------------------------------*/
func RtkCloseStat() {
	Trace(4, "rtkclosestat:\n")

	if fp_stat != nil {
		fp_stat.Close()
	}
	fp_stat = nil
	file_stat = ""
	statlevel = 0
}

/* write solution status to buffer -------------------------------------------*/
func (rtk *Rtk) RtkOutStat(buff *string) int {
	var (
		ssat                          *SSat
		tow                           float64
		pos, vel, acc, vela, acca, xa [3]float64
		i, j, week, nfreq, nf         int
		id                            string
	)
	nf = RNF(&rtk.Opt)
	bufflen := len(*buff)
	if rtk.RtkSol.Stat <= SOLQ_NONE {
		return 0
	}
	/* write ppp solution status to buffer */
	if rtk.Opt.Mode >= PMODE_PPP_KINEMA {
		return OutPPPStat(rtk, buff)
	}
	est := rtk.Opt.Mode >= PMODE_DGPS
	nfreq = 1
	if est {
		nfreq = nf
	}
	tow = Time2GpsT(rtk.RtkSol.Time, &week)

	/* receiver position */
	if est {
		for i = 0; i < 3; i++ {
			xa[i] = 0.0
			if i < rtk.Na {
				xa[i] = rtk.Xa[i]
			}
		}
		*buff += fmt.Sprintf("$POS,%d,%.3f,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n", week, tow,
			rtk.RtkSol.Stat, rtk.X[0], rtk.X[1], rtk.X[2], xa[0], xa[1],
			xa[2])
	} else {
		*buff += fmt.Sprintf("$POS,%d,%.3f,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n", week, tow,
			rtk.RtkSol.Stat, rtk.RtkSol.Rr[0], rtk.RtkSol.Rr[1], rtk.RtkSol.Rr[2],
			0.0, 0.0, 0.0)
	}
	/* receiver velocity and acceleration */
	if est && rtk.Opt.Dynamics > 0 {
		Ecef2Pos(rtk.RtkSol.Rr[:], pos[:])
		Ecef2Enu(pos[:], rtk.X[3:], vel[:])
		Ecef2Enu(pos[:], rtk.X[6:], acc[:])
		if rtk.Na >= 6 {
			Ecef2Enu(pos[:], rtk.X[3:], vel[:])
		}
		if rtk.Na >= 9 {
			Ecef2Enu(pos[:], rtk.X[6:], acc[:])
		}
		*buff += fmt.Sprintf("$VELACC,%d,%.3f,%d,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f\n",
			week, tow, rtk.RtkSol.Stat, vel[0], vel[1], vel[2], acc[0], acc[1],
			acc[2], vela[0], vela[1], vela[2], acca[0], acca[1], acca[2])
	} else {
		Ecef2Pos(rtk.RtkSol.Rr[:], pos[:])
		Ecef2Enu(pos[:], rtk.RtkSol.Rr[3:], vel[:])
		*buff += fmt.Sprintf("$VELACC,%d,%.3f,%d,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f\n",
			week, tow, rtk.RtkSol.Stat, vel[0], vel[1], vel[2],
			0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
	}
	/* receiver clocks */
	*buff += fmt.Sprintf("$CLK,%d,%.3f,%d,%d,%.3f,%.3f,%.3f,%.3f\n",
		week, tow, rtk.RtkSol.Stat, 1, rtk.RtkSol.Dtr[0]*1e9, rtk.RtkSol.Dtr[1]*1e9,
		rtk.RtkSol.Dtr[2]*1e9, rtk.RtkSol.Dtr[3]*1e9)

	/* ionospheric parameters */
	if est && rtk.Opt.IonoOpt == IONOOPT_EST {
		for i = 0; i < MAXSAT; i++ {
			ssat = &rtk.Ssat[i]
			if ssat.Vs == 0 {
				continue
			}
			SatNo2Id(i+1, &id)
			j = RII(i+1, &rtk.Opt)
			xa[0] = 0.0
			if j < rtk.Na {
				xa[0] = rtk.Xa[j]
			}
			*buff += fmt.Sprintf("$ION,%d,%.3f,%d,%s,%.1f,%.1f,%.4f,%.4f\n", week, tow,
				rtk.RtkSol.Stat, id, ssat.Azel[0]*R2D, ssat.Azel[1]*R2D,
				rtk.X[j], xa[0])
		}
	}
	/* tropospheric parameters */
	if est && (rtk.Opt.TropOpt == TROPOPT_EST || rtk.Opt.TropOpt == TROPOPT_ESTG) {
		for i = 0; i < 2; i++ {
			j = RIT(i, &rtk.Opt)
			xa[0] = 0.0
			if j < rtk.Na {
				xa[0] = rtk.Xa[j]
			}
			*buff += fmt.Sprintf("$TROP,%d,%.3f,%d,%d,%.4f,%.4f\n", week, tow,
				rtk.RtkSol.Stat, i+1, rtk.X[j], xa[0])
		}
	}
	/* receiver h/w bias */
	if est && rtk.Opt.GloModeAr == 2 {
		for i = 0; i < nfreq; i++ {
			j = RIL(i, &rtk.Opt)
			xa[0] = 0.0
			if j < rtk.Na {
				xa[0] = rtk.Xa[j]
			}
			*buff += fmt.Sprintf("$HWBIAS,%d,%.3f,%d,%d,%.4f,%.4f\n", week, tow,
				rtk.RtkSol.Stat, i+1, rtk.X[j], xa[0])
		}
	}
	return len(*buff) - bufflen
}

/* swap solution status file -------------------------------------------------*/
func swapsolstat() {
	var (
		path string
		err  error
	)
	time := Utc2GpsT(TimeGet())

	if Time2GpsT(time, nil)/float64(INT_SWAP_STAT) ==
		Time2GpsT(time_stat, nil)/float64(INT_SWAP_STAT) {
		return
	}
	time_stat = time

	if RepPath(file_stat, &path, time, "", "") == 0 {
		return
	}
	if fp_stat != nil {
		fp_stat.Close()
	}

	fp_stat, err = os.OpenFile(path, os.O_CREATE|os.O_WRONLY|os.O_TRUNC, os.ModeAppend|os.ModePerm)
	if fp_stat == nil || err != nil {
		Trace(2, "swapsolstat: file open error path=%s\n", path)
		return
	}
	Trace(5, "swapsolstat: path=%s\n", path)
}

/* output solution status ----------------------------------------------------*/
func (rtk *Rtk) OutSolStat() {
	var (
		ssat                     *SSat
		tow                      float64
		buff, id                 string
		i, j, n, week, nfreq, nf int
	)
	nf = RNF(&rtk.Opt)

	if statlevel <= 0 || fp_stat == nil || rtk.RtkSol.Stat == 0 {
		return
	}

	Trace(4, "outsolstat:\n")

	/* swap solution status file */
	swapsolstat()

	/* write solution status */
	n = rtk.RtkOutStat(&buff)
	buff = buff[:n]

	fp_stat.WriteString(buff)

	if rtk.RtkSol.Stat == SOLQ_NONE || statlevel <= 1 {
		return
	}

	tow = Time2GpsT(rtk.RtkSol.Time, &week)
	nfreq = 1
	if rtk.Opt.Mode >= PMODE_DGPS {
		nfreq = nf
	}

	/* write residuals and status */
	for i = 0; i < MAXSAT; i++ {
		ssat = &rtk.Ssat[i]
		if ssat.Vs == 0 {
			continue
		}
		SatNo2Id(i+1, &id)
		for j = 0; j < nfreq; j++ {
			fmt.Fprintf(fp_stat, "$SAT,%d,%.3f,%s,%d,%.1f,%.1f,%.4f,%.4f,%d,%.1f,%d,%d,%d,%d,%d,%d\n",
				week, tow, id, j+1, ssat.Azel[0]*R2D, ssat.Azel[1]*R2D,
				ssat.Resp[j], ssat.Resc[j], ssat.Vsat[j],
				float32(ssat.Snr[j])*SNR_UNIT, ssat.Fix[j], ssat.Slip[j]&3,
				ssat.Lock[j], ssat.Outc[j], ssat.Slipc[j], ssat.Rejc[j])
		}
	}
}

/* save error message --------------------------------------------------------*/
func (rtk *Rtk) errmsg(format string, v ...interface{}) {
	var buff, tstr string
	Time2Str(rtk.RtkSol.Time, &tstr, 2)
	buff = fmt.Sprintf("%s: ", tstr[11:])
	buff += fmt.Sprintf(format, v...)

	rtk.ErrBuf += buff
	Trace(5, "%s", buff)
}

/* single-differenced observable ---------------------------------------------*/
func SingleDifferencedObs(obs []ObsD, i, j, k int) float64 {
	var pi, pj float64
	if k < NFREQ {
		pi = obs[i].L[k]
		pj = obs[j].L[k]
	} else {
		pi = obs[i].P[k-NFREQ]
		pj = obs[j].P[k-NFREQ]
	}
	if pi == 0.0 || pj == 0.0 {
		return 0.0
	}
	return pi - pj
}

/* single-differenced geometry-free linear combination of phase --------------*/
func GeometryFreeObs(obs []ObsD, i, j, k int, nav *Nav) float64 {
	var freq1, freq2, L1, L2 float64

	freq1 = Sat2Freq(obs[i].Sat, obs[i].Code[0], nav)
	freq2 = Sat2Freq(obs[i].Sat, obs[i].Code[k], nav)
	L1 = SingleDifferencedObs(obs, i, j, 0)
	L2 = SingleDifferencedObs(obs, i, j, k)
	if freq1 == 0.0 || freq2 == 0.0 || L1 == 0.0 || L2 == 0.0 {
		return 0.0
	}
	return L1*CLIGHT/freq1 - L2*CLIGHT/freq2
}

/* single-differenced measurement error variance -----------------------------*/
func RtkVarianceErr(sat, sys int, el, bl, dt float64, f int, opt *PrcOpt) float64 {
	var a, b, c, d, fact float64
	c = opt.Err[3] * bl / 1e4
	d = CLIGHT * opt.SatClkStab * dt
	fact = 1.0
	sinel := math.Sin(el)
	nf := RNF(opt)

	if f >= nf {
		fact = opt.eratio[f-nf]
	}
	if fact <= 0.0 {
		fact = opt.eratio[0]
	}
	switch sys {
	case SYS_GLO:
		fact *= float64(EFACT_GLO)
	case SYS_SBS:
		fact *= float64(EFACT_SBS)
	default:
		fact *= float64(EFACT_GPS)
	}
	a = fact * opt.Err[1]
	b = fact * opt.Err[2]
	if opt.IonoOpt == IONOOPT_IFLC {
		return 2.0*3.0*(a*a+b*b/sinel/sinel+c*c) + d*d
	}
	return 2.0*1.0*(a*a+b*b/sinel/sinel+c*c) + d*d
}

/* CalcBaseLineLen length -----------------------------------------------------------*/
func CalcBaseLineLen(ru, rb, dr []float64) float64 {

	for i := 0; i < 3; i++ {
		dr[i] = ru[i] - rb[i]
	}
	return Norm(dr, 3)
}

/* initialize state and covariance -------------------------------------------*/
func (rtk *Rtk) Initx(xi, fvar float64, i int) {

	rtk.X[i] = xi
	for j := 0; j < rtk.Nx; j++ {
		if i == j {
			rtk.P[i+j*rtk.Nx], rtk.P[j+i*rtk.Nx] = fvar, fvar
		} else {
			rtk.P[i+j*rtk.Nx], rtk.P[j+i*rtk.Nx] = 0.0, 0.0
		}
	}
}

/* select common satellites between rover and reference station --------------*/
func SelSat(obs []ObsD, azel []float64, nu, nr int, opt *PrcOpt, sat, iu, ir []int) int {
	var i, j, k int

	Trace(4, "selsat  : nu=%d nr=%d\n", nu, nr)

	for i, j = 0, nu; i < nu && j < nu+nr; i, j = i+1, j+1 {
		switch {
		case obs[i].Sat < obs[j].Sat:
			j--
		case obs[i].Sat > obs[j].Sat:
			i--
		case azel[1+j*2] >= opt.Elmin: /* elevation at base station */
			sat[k] = obs[i].Sat
			iu[k] = i
			ir[k] = j
			k++
			Trace(2, "(%2d) sat=%3d iu=%2d ir=%2d\n", k-1, obs[i].Sat, i, j)
		}
	}
	return k
}

/* temporal update of position/velocity/acceleration -------------------------*/
func (rtk *Rtk) UpdatePos(tt float64) {
	var (
		F, P, FP, x, xp []float64
		pos             [3]float64
		Q, Qv           [9]float64
		fvar            float64 = 0.0
		i, j, nx        int
		ix              []int
	)

	Trace(4, "udpos   : tt=%.3f\n", tt)

	/* fixed mode */
	if rtk.Opt.Mode == PMODE_FIXED {
		for i = 0; i < 3; i++ {
			rtk.Initx(rtk.Opt.Ru[i], 1e-8, i)
		}
		return
	}
	/* initialize position for first epoch */
	if Norm(rtk.X, 3) <= 0.0 {
		for i = 0; i < 3; i++ {
			rtk.Initx(rtk.RtkSol.Rr[i], VAR_POS, i)
		}
		if rtk.Opt.Dynamics > 0 {
			for i = 3; i < 6; i++ {
				rtk.Initx(rtk.RtkSol.Rr[i], VAR_VEL, i)
			}
			for i = 6; i < 9; i++ {
				rtk.Initx(1e-6, VAR_ACC, i)
			}
		}
	}
	/* static mode */
	if rtk.Opt.Mode == PMODE_STATIC {
		return
	}

	/* kinmatic mode without dynamics */
	if rtk.Opt.Dynamics == 0 {
		for i = 0; i < 3; i++ {
			rtk.Initx(rtk.RtkSol.Rr[i], VAR_POS, i)
		}
		return
	}
	/* check variance of estimated postion */
	for i = 0; i < 3; i++ {
		fvar += rtk.P[i+i*rtk.Nx]
	}
	fvar /= 3.0

	if fvar > VAR_POS {
		/* reset position with large variance */
		for i = 0; i < 3; i++ {
			rtk.Initx(rtk.RtkSol.Rr[i], VAR_POS, i)
		}
		for i = 3; i < 6; i++ {
			rtk.Initx(rtk.RtkSol.Rr[i], VAR_VEL, i)
		}
		for i = 6; i < 9; i++ {
			rtk.Initx(1e-6, VAR_ACC, i)
		}
		Trace(5, "reset rtk position due to large variance: var=%.3f\n", fvar)
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
		F[i+(i+3)*nx] = tt
	}
	for i = 0; i < 3; i++ {
		F[i+(i+6)*nx] = SQR(tt) / 2.0
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
	Q[0], Q[4] = SQR(rtk.Opt.Prn[3])*math.Abs(tt), SQR(rtk.Opt.Prn[3])*math.Abs(tt)
	Q[8] = SQR(rtk.Opt.Prn[4]) * math.Abs(tt)
	Ecef2Pos(rtk.X, pos[:])
	Cov2Ecef(pos[:], Q[:], Qv[:])
	for i = 0; i < 3; i++ {
		for j = 0; j < 3; j++ {
			rtk.P[i+6+(j+6)*rtk.Nx] += Qv[i+j*3]
		}
	}
}

/* temporal update of ionospheric parameters ---------------------------------*/
func (rtk *Rtk) UpdateIon(tt, bl float64, sat []int, ns int) {
	var (
		el, fact float64
		i, j     int
	)

	Trace(4, "udion   : tt=%.3f bl=%.0f ns=%d\n", tt, bl, ns)

	for i = 1; i <= MAXSAT; i++ {
		j = RII(i, &rtk.Opt)
		if rtk.X[j] != 0.0 &&
			rtk.Ssat[i-1].Outc[0] > uint32(GAP_RESION) && rtk.Ssat[i-1].Outc[1] > uint32(GAP_RESION) {
			rtk.X[j] = 0.0
		}
	}
	for i = 0; i < ns; i++ {
		j = RII(sat[i], &rtk.Opt)

		if rtk.X[j] == 0.0 {
			rtk.Initx(1e-6, SQR(rtk.Opt.Std[1]*bl/1e4), j)
		} else {
			/* elevation dependent factor of process noise */
			el = rtk.Ssat[sat[i]-1].Azel[1]
			fact = math.Cos(el)
			rtk.P[j+j*rtk.Nx] += SQR(rtk.Opt.Prn[1]*bl/1e4*fact) * math.Abs(tt)
		}
	}
}

/* temporal update of tropospheric parameters --------------------------------*/
func (rtk *Rtk) UpdateTrop(tt, bl float64) {
	var i, j, k int

	Trace(4, "udtrop  : tt=%.3f\n", tt)

	for i = 0; i < 2; i++ {
		j = RIT(i, &rtk.Opt)

		if rtk.X[j] == 0.0 {
			rtk.Initx(INIT_ZWD, SQR(rtk.Opt.Std[2]), j) /* initial zwd */

			if rtk.Opt.TropOpt >= TROPOPT_ESTG {
				for k = 0; k < 2; k++ {
					j++
					rtk.Initx(1e-6, VAR_GRA, j)
				}
			}
		} else {
			rtk.P[j+j*rtk.Nx] += SQR(rtk.Opt.Prn[2]) * math.Abs(tt)

			if rtk.Opt.TropOpt >= TROPOPT_ESTG {
				for k = 0; k < 2; k++ {
					j++
					rtk.P[j*(1+rtk.Nx)] += SQR(rtk.Opt.Prn[2]*0.3) * math.Abs(tt)
				}
			}
		}
	}
}

/* temporal update of receiver h/w biases ------------------------------------*/
func (rtk *Rtk) UpdateRcvBias(tt float64) {
	var i, j int

	Trace(4, "udrcvbias: tt=%.3f\n", tt)

	for i = 0; i < NFREQGLO; i++ {
		j = RIL(i, &rtk.Opt)

		switch {
		case rtk.X[j] == 0.0:
			rtk.Initx(1e-6, VAR_HWBIAS, j)
		case rtk.Nfix >= rtk.Opt.MinFix && float64(rtk.RtkSol.Ratio) > rtk.Opt.ThresAr[0]:
			/* hold to fixed solution */
			rtk.Initx(rtk.Xa[j], rtk.Pa[j+j*rtk.Na], j)
		default:
			rtk.P[j+j*rtk.Nx] += SQR(PRN_HWBIAS) * math.Abs(tt)
		}
	}
}

/* detect cycle slip by LLI --------------------------------------------------*/
func (rtk *Rtk) DetectSlp_ll(obs []ObsD, i, rcv int) {
	var (
		slip, LLI uint32
		f, sat    int = 0, obs[i].Sat
	)

	Trace(4, "detslp_ll: i=%d rcv=%d\n", i, rcv)

	for f = 0; f < rtk.Opt.Nf; f++ {

		if obs[i].L[f] == 0.0 || math.Abs(TimeDiff(obs[i].Time, rtk.Ssat[sat-1].Pt[rcv-1][f])) < float64(DTTOL) {
			continue
		}
		/* restore previous LLI */
		if rcv == 1 {
			LLI = GetBitU(rtk.Ssat[sat-1].Slip[f:], 0, 2) /* rover */
		} else {
			LLI = GetBitU(rtk.Ssat[sat-1].Slip[f:], 2, 2) /* base  */
		}

		/* detect slip by cycle slip flag in LLI */
		if rtk.Tt >= 0.0 { /* forward */
			if obs[i].LLI[f]&1 == 1 {
				rtk.errmsg("slip detected forward  (sat=%2d rcv=%d F=%d LLI=%x)\n",
					sat, rcv, f+1, obs[i].LLI[f])
			}
			slip = uint32(obs[i].LLI[f])
		} else { /* backward */
			if LLI&1 == 1 {
				rtk.errmsg("slip detected backward (sat=%2d rcv=%d F=%d LLI=%x)\n",
					sat, rcv, f+1, LLI)
			}
			slip = LLI
		}
		/* detect slip by parity unknown flag transition in LLI */
		if ((LLI&2 > 0) && (obs[i].LLI[f]&2 == 0)) || ((LLI&2 == 0) && (obs[i].LLI[f]&2 > 0)) {
			rtk.errmsg("slip detected half-cyc (sat=%2d rcv=%d F=%d LLI=%x.%x)\n",
				sat, rcv, f+1, LLI, obs[i].LLI[f])
			slip |= 1
		}
		/* save current LLI */
		if rcv == 1 {
			SetBitU(rtk.Ssat[sat-1].Slip[f:], 0, 2, uint32(obs[i].LLI[f]))
		} else {
			SetBitU(rtk.Ssat[sat-1].Slip[f:], 2, 2, uint32(obs[i].LLI[f]))
		}

		/* save slip and half-cycle valid flag */
		rtk.Ssat[sat-1].Slip[f] |= uint8(slip)
		rtk.Ssat[sat-1].Half[f] = 1
		if obs[i].LLI[f]&2 > 0 {
			rtk.Ssat[sat-1].Half[f] = 0
		}
	}
}

/* detect cycle slip by geometry free phase jump -----------------------------*/
func (rtk *Rtk) DetectSlp_gf(obs []ObsD, i, j int, nav *Nav) {
	var (
		k, sat int = 0, obs[i].Sat
		g0, g1 float64
	)

	Trace(4, "detslp_gf: i=%d j=%d\n", i, j)

	for k = 1; k < rtk.Opt.Nf; k++ {
		if g1 = GeometryFreeObs(obs, i, j, k, nav); g1 == 0.0 {
			return
		}

		g0 = rtk.Ssat[sat-1].Gf[k-1]
		rtk.Ssat[sat-1].Gf[k-1] = g1

		if g0 != 0.0 && math.Abs(g1-g0) > rtk.Opt.ThresSlip {
			rtk.Ssat[sat-1].Slip[0] |= 1
			rtk.Ssat[sat-1].Slip[k] |= 1
			rtk.errmsg("slip detected GF jump (sat=%2d L1-L%d GF=%.3f %.3f)\n",
				sat, k+1, g0, g1)
		}
	}
}

/* detect cycle slip by doppler and phase difference -------------------------*/
func (rtk *Rtk) DetectSlp_dop(obs []ObsD, i, rcv int, nav *Nav) {
	// #if 0 /* detection with doppler disabled because of clock-jump issue (v.2.3.0) */
	//     int f,sat=obs[i].sat;
	//     double tt,dph,dpt,lam,thres;

	//     trace(3,"detslp_dop: i=%d rcv=%d\n",i,rcv);

	//     for (f=0;f<rtk.opt.nf;f++) {
	//         if (obs[i].L[f]==0.0||obs[i].D[f]==0.0||rtk.ph[rcv-1][sat-1][f]==0.0) {
	//             continue;
	//         }
	//         if (fabs(tt=timediff(obs[i].time,rtk.pt[rcv-1][sat-1][f]))<DTTOL) continue;
	//         if ((lam=nav.lam[sat-1][f])<=0.0) continue;

	//         /* cycle slip threshold (cycle) */
	//         thres=MAXACC*tt*tt/2.0/lam+rtk.opt.err[4]*fabs(tt)*4.0;

	//         /* phase difference and doppler x time (cycle) */
	//         dph=obs[i].L[f]-rtk.ph[rcv-1][sat-1][f];
	//         dpt=-obs[i].D[f]*tt;

	//         if (fabs(dph-dpt)<=thres) continue;

	//         rtk.slip[sat-1][f]|=1;

	//         rtk.errmsg("slip detected (sat=%2d rcv=%d L%d=%.3f %.3f thres=%.3f)\n",
	//                sat,rcv,f+1,dph,dpt,thres);
	//     }
	// #endif
}

/* temporal update of phase biases -------------------------------------------*/
func (rtk *Rtk) UpdateBias(tt float64, obs []ObsD, sat, iu, ir []int, ns int, nav *Nav) {
	var (
		cp, pr, cp1, cp2, pr1, pr2, offset, freqi, freq1, freq2, C1, C2 float64
		bias                                                            []float64
		i, j, k, slip, nf                                               int
		reset                                                           bool
	)
	nf = RNF(&rtk.Opt)

	Trace(4, "udbias  : tt=%.3f ns=%d\n", tt, ns)

	for i = 0; i < ns; i++ {

		/* detect cycle slip by LLI */
		for k = 0; k < rtk.Opt.Nf; k++ {
			rtk.Ssat[sat[i]-1].Slip[k] &= 0xFC
		}
		rtk.DetectSlp_ll(obs, iu[i], 1)
		rtk.DetectSlp_ll(obs, ir[i], 2)

		/* detect cycle slip by geometry-free phase jump */
		rtk.DetectSlp_gf(obs, iu[i], ir[i], nav)

		/* detect cycle slip by doppler and phase difference */
		rtk.DetectSlp_dop(obs, iu[i], 1, nav)
		rtk.DetectSlp_dop(obs, ir[i], 2, nav)

		/* update half-cycle valid flag */
		for k = 0; k < nf; k++ {
			if (obs[iu[i]].LLI[k]&2 > 0) || (obs[ir[i]].LLI[k]&2 > 0) {
				rtk.Ssat[sat[i]-1].Half[k] = 0
			} else {
				rtk.Ssat[sat[i]-1].Half[k] = 1
			}

		}
	}
	for k = 0; k < nf; k++ {
		/* reset phase-bias if instantaneous AR or expire obs outage counter */
		for i = 1; i <= MAXSAT; i++ {
			rtk.Ssat[i-1].Outc[k]++
			reset = rtk.Ssat[i-1].Outc[k] > uint32(rtk.Opt.MaxOut)

			switch {
			case rtk.Opt.ModeAr == ARMODE_INST && rtk.X[RIB(i, k, &rtk.Opt)] != 0.0:
				rtk.Initx(0.0, 0.0, RIB(i, k, &rtk.Opt))
			case reset && rtk.X[RIB(i, k, &rtk.Opt)] != 0.0:
				rtk.Initx(0.0, 0.0, RIB(i, k, &rtk.Opt))
				Trace(4, "udbias : obs outage counter overflow (sat=%3d L%d n=%d)\n",
					i, k+1, rtk.Ssat[i-1].Outc[k])
				rtk.Ssat[i-1].Outc[k] = 0
			}
			if rtk.Opt.ModeAr != ARMODE_INST && reset {
				rtk.Ssat[i-1].Lock[k] = -rtk.Opt.MinLock
			}
		}
		/* reset phase-bias if detecting cycle slip */
		for i = 0; i < ns; i++ {
			j = RIB(sat[i], k, &rtk.Opt)
			rtk.P[j+j*rtk.Nx] += rtk.Opt.Prn[0] * rtk.Opt.Prn[0] * math.Abs(tt)
			slip = int(rtk.Ssat[sat[i]-1].Slip[k])
			if rtk.Opt.IonoOpt == IONOOPT_IFLC {
				slip |= int(rtk.Ssat[sat[i]-1].Slip[1])
			}
			if rtk.Opt.ModeAr == ARMODE_INST || slip&1 == 0 {
				continue
			}
			rtk.X[j] = 0.0
			rtk.Ssat[sat[i]-1].Lock[k] = -rtk.Opt.MinLock
		}
		bias = Zeros(ns, 1)

		/* estimate approximate phase-bias by phase - code */
		for i, j, offset = 0, 0, 0.0; i < ns; i++ {

			if rtk.Opt.IonoOpt != IONOOPT_IFLC {
				cp = SingleDifferencedObs(obs, iu[i], ir[i], k) /* cycle */
				pr = SingleDifferencedObs(obs, iu[i], ir[i], k+NFREQ)
				freqi = Sat2Freq(sat[i], obs[iu[i]].Code[k], nav)
				if cp == 0.0 || pr == 0.0 || freqi == 0.0 {
					continue
				}

				bias[i] = cp - pr*freqi/CLIGHT
			} else {
				cp1 = SingleDifferencedObs(obs, iu[i], ir[i], 0)
				cp2 = SingleDifferencedObs(obs, iu[i], ir[i], 1)
				pr1 = SingleDifferencedObs(obs, iu[i], ir[i], NFREQ)
				pr2 = SingleDifferencedObs(obs, iu[i], ir[i], NFREQ+1)
				freq1 = Sat2Freq(sat[i], obs[iu[i]].Code[0], nav)
				freq2 = Sat2Freq(sat[i], obs[iu[i]].Code[1], nav)
				if cp1 == 0.0 || cp2 == 0.0 || pr1 == 0.0 || pr2 == 0.0 || freq1 == 0.0 || freq2 <= 0.0 {
					continue
				}

				C1 = SQR(freq1) / (SQR(freq1) - SQR(freq2))
				C2 = -SQR(freq2) / (SQR(freq1) - SQR(freq2))
				bias[i] = (C1*cp1*CLIGHT/freq1 + C2*cp2*CLIGHT/freq2) - (C1*pr1 + C2*pr2)
			}
			if rtk.X[RIB(sat[i], k, &rtk.Opt)] != 0.0 {
				offset += bias[i] - rtk.X[RIB(sat[i], k, &rtk.Opt)]
				j++
			}
		}
		/* correct phase-bias offset to enssure phase-code coherency */
		if j > 0 {
			for i = 1; i <= MAXSAT; i++ {
				if rtk.X[RIB(i, k, &rtk.Opt)] != 0.0 {
					rtk.X[RIB(i, k, &rtk.Opt)] += offset / float64(j)
				}
			}
		}
		/* set initial states of phase-bias */
		for i = 0; i < ns; i++ {
			if bias[i] == 0.0 || rtk.X[RIB(sat[i], k, &rtk.Opt)] != 0.0 {
				continue
			}
			rtk.Initx(bias[i], SQR(rtk.Opt.Std[0]), RIB(sat[i], k, &rtk.Opt))
		}

	}
}

/* temporal update of states --------------------------------------------------*/
func (rtk *Rtk) UpdateState(obs []ObsD, sat, iu, ir []int, ns int, nav *Nav) {
	var (
		bl float64
		dr [3]float64
	)
	tt := rtk.Tt
	Trace(4, "udstate : ns=%d\n", ns)

	/* temporal update of position/velocity/acceleration */
	rtk.UpdatePos(tt)

	/* temporal update of ionospheric parameters */
	if rtk.Opt.IonoOpt >= IONOOPT_EST {
		bl = CalcBaseLineLen(rtk.X, rtk.Rb[:], dr[:])
		rtk.UpdateIon(tt, bl, sat, ns)
	}
	/* temporal update of tropospheric parameters */
	if rtk.Opt.TropOpt >= TROPOPT_EST {
		rtk.UpdateTrop(tt, bl)
	}
	/* temporal update of eceiver h/w bias */
	if rtk.Opt.GloModeAr == 2 && (rtk.Opt.NavSys&SYS_GLO) > 0 {
		rtk.UpdateRcvBias(tt)
	}
	/* temporal update of phase-bias */
	if rtk.Opt.Mode > PMODE_DGPS {
		rtk.UpdateBias(tt, obs, sat, iu, ir, ns, nav)
	}
}

/* UD (undifferenced) phase/code residual for satellite ----------------------*/
func ZdResSat(base int, r float64, obs *ObsD, nav *Nav,
	azel, dant []float64, opt *PrcOpt, y, freq []float64) {
	var (
		freq1, freq2, C1, C2, dant_if float64
		i, nf                         int
	)
	nf = RNF(opt)

	if opt.IonoOpt == IONOOPT_IFLC { /* iono-free linear combination */
		freq1 = Sat2Freq(obs.Sat, obs.Code[0], nav)
		freq2 = Sat2Freq(obs.Sat, obs.Code[1], nav)
		if freq1 == 0.0 || freq2 == 0.0 {
			return
		}

		if TestSnr(base, 0, azel[1], float64(obs.SNR[0])*float64(SNR_UNIT), &opt.SnrMask) > 0 ||
			TestSnr(base, 1, azel[1], float64(obs.SNR[1])*float64(SNR_UNIT), &opt.SnrMask) > 0 {
			return
		}

		C1 = SQR(freq1) / (SQR(freq1) - SQR(freq2))
		C2 = -SQR(freq2) / (SQR(freq1) - SQR(freq2))
		dant_if = C1*dant[0] + C2*dant[1]

		if obs.L[0] != 0.0 && obs.L[1] != 0.0 {
			y[0] = C1*obs.L[0]*CLIGHT/freq1 + C2*obs.L[1]*CLIGHT/freq2 - r - dant_if
		}
		if obs.P[0] != 0.0 && obs.P[1] != 0.0 {
			y[1] = C1*obs.P[0] + C2*obs.P[1] - r - dant_if
		}
		freq[0] = 1.0
	} else {
		for i = 0; i < nf; i++ {
			if freq[i] = Sat2Freq(obs.Sat, obs.Code[i], nav); freq[i] == 0.0 {
				continue
			}

			/* check SNR mask */
			if TestSnr(base, i, azel[1], float64(obs.SNR[i])*float64(SNR_UNIT), &opt.SnrMask) != 0 {
				continue
			}
			/* residuals = observable - pseudorange */
			if obs.L[i] != 0.0 {
				y[i] = obs.L[i]*CLIGHT/freq[i] - r - dant[i]
			}
			if obs.P[i] != 0.0 {
				y[i+nf] = obs.P[i] - r - dant[i]
			}
		}
	}
}

/* UD (undifferenced) phase/code residuals -----------------------------------*/
func ZDRes(base int, obs []ObsD, n int, rs, dts, fvar []float64, svh []int,
	nav *Nav, rr []float64, opt *PrcOpt, index int, y, e, azel, freq []float64) int {
	var (
		zhd, r         float64
		rr_, pos, disp [3]float64
		dant           [NFREQ]float64
		zazel          []float64 = []float64{0.0, 90.0 * D2R}
		i, nf          int
	)
	nf = RNF(opt)

	Trace(4, "zdres   : n=%d\n", n)

	for i = 0; i < n*nf*2; i++ {
		y[i] = 0.0
	}
	if Norm(rr, 3) <= 0.0 {
		return 0 /* no receiver position */
	}

	for i = 0; i < 3; i++ {
		rr_[i] = rr[i]
	}

	/* earth tide correction */
	if opt.TideCorr > 0 {
		TideDisp(GpsT2Utc(obs[0].Time), rr_[:], opt.TideCorr, &nav.Erp,
			opt.Odisp[base][:], disp[:])
		for i = 0; i < 3; i++ {
			rr_[i] += disp[i]
		}
	}
	Ecef2Pos(rr_[:], pos[:])

	for i = 0; i < n; i++ {
		/* compute geometric-range and azimuth/elevation angle */
		if r = GeoDist(rs[i*6:], rr_[:], e[i*3:]); r <= 0.0 {
			continue
		}
		if SatAzel(pos[:], e[i*3:], azel[i*2:]) < opt.Elmin {
			continue
		}

		/* excluded satellite? */
		if SatExclude(obs[i].Sat, fvar[i], svh[i], opt) > 0 {
			continue
		}

		/* satellite clock-bias */
		r += -CLIGHT * dts[i*2]

		/* troposphere delay model (hydrostatic) */
		zhd = TropModel(obs[0].Time, pos[:], zazel, 0.0)
		r += TropMapFunc(obs[i].Time, pos[:], azel[i*2:], nil) * zhd

		/* receiver antenna phase center correction */
		AntModel(&opt.Pcvr[index], opt.AntDel[index][:], azel[i*2:], opt.PosOpt[1],
			dant[:])

		/* UD phase/code residual for satellite */
		ZdResSat(base, r, &obs[i], nav, azel[i*2:], dant[:], opt, y[i*nf*2:], freq[i*nf:])
	}
	Trace(4, "rr_=%.3f %.3f %.3f\n", rr_[0], rr_[1], rr_[2])
	Trace(4, "pos=%.9f %.9f %.3f\n", pos[0]*R2D, pos[1]*R2D, pos[2])
	for i = 0; i < n; i++ {
		Trace(4, "sat=%2d %13.3f %13.3f %13.3f %13.10f %6.1f %5.1f\n",
			obs[i].Sat, rs[i*6], rs[1+i*6], rs[2+i*6], dts[i*2], azel[i*2]*R2D,
			azel[1+i*2]*R2D)
	}
	Trace(4, "y=\n")
	tracemat(4, y, nf*2, n, 13, 3)

	return 1
}

/* test valid observation data -----------------------------------------------*/
func ValidObs(i, j, f, nf int, y []float64) int {
	/* if no phase observable, psudorange is also unusable */
	if y[f+i*nf*2] != 0.0 && y[f+j*nf*2] != 0.0 &&
		(f < nf || (y[f-nf+i*nf*2] != 0.0 && y[f-nf+j*nf*2] != 0.0)) {
		return 1
	}
	return 0
}

/* DD (double-differenced) measurement error covariance ----------------------*/
func DDCovariance(nb []float64, n int, Ri, Rj []float64, nv int, R []float64) {
	var i, j, k, b int

	Trace(4, "ddcov   : n=%d\n", n)

	for i = 0; i < nv*nv; i++ {
		R[i] = 0.0
	}
	for b = 0; b < n; k, b = k+int(nb[b]), b+1 {

		for i = 0; i < int(nb[b]); i++ {
			for j = 0; j < int(nb[b]); j++ {
				R[k+i+(k+j)*nv] = Ri[k+i]
				if i == j {
					R[k+i+(k+j)*nv] = Ri[k+i] + Rj[k+i]
				}
			}
		}
	}
	Trace(5, "R=\n")
	tracemat(5, R, nv, nv, 8, 6)
}

/* baseline length constraint ------------------------------------------------*/
func (rtk *Rtk) ConstBaselineLen(x, P, v, H, Ri, Rj []float64, index int) int {
	var (
		thres    float64 = 0.1 /* threshold for nonliearity (v.2.3.0) */
		xb, b    [3]float64
		bb, fvar float64
		i        int
	)

	Trace(4, "constbl : \n")

	/* no constraint */
	if rtk.Opt.Baseline[0] <= 0.0 {
		return 0
	}

	/* time-adjusted baseline vector and length */
	for i = 0; i < 3; i++ {
		xb[i] = rtk.Rb[i]
		b[i] = x[i] - xb[i]
	}
	bb = Norm(b[:], 3)

	/* approximate variance of solution */
	if P != nil {
		for i = 0; i < 3; i++ {
			fvar += P[i+i*rtk.Nx]
		}
		fvar /= 3.0
	}
	/* check nonlinearity */
	if fvar > SQR(thres*bb) {
		Trace(2, "constbl : equation nonlinear (bb=%.3f var=%.3f)\n", bb, fvar)
		return 0
	}
	/* constraint to baseline length */
	v[index] = rtk.Opt.Baseline[0] - bb
	if H != nil {
		for i = 0; i < 3; i++ {
			H[i+index*rtk.Nx] = b[i] / bb
		}
	}
	Ri[index] = 0.0
	Rj[index] = SQR(rtk.Opt.Baseline[1])

	Trace(5, "baseline len   v=%13.3f R=%8.6f %8.6f\n", v[index], Ri[index], Rj[index])

	return 1
}

/* precise tropspheric model -------------------------------------------------*/
func PrecTrop(time Gtime, pos []float64, r int, azel []float64, opt *PrcOpt, x, dtdx []float64) float64 {
	var m_w, cotz, grad_n, grad_e float64
	i := RIT(r, opt)

	/* wet mapping function */
	TropMapFunc(time, pos, azel, &m_w)

	if opt.TropOpt >= TROPOPT_ESTG && azel[1] > 0.0 {

		/* m_w=m_0+m_0*cot(el)*(Gn*cos(az)+Ge*sin(az)): ref [6] */
		cotz = 1.0 / math.Tan(azel[1])
		grad_n = m_w * cotz * math.Cos(azel[0])
		grad_e = m_w * cotz * math.Sin(azel[0])
		m_w += grad_n*x[i+1] + grad_e*x[i+2]
		dtdx[1] = grad_n * x[i]
		dtdx[2] = grad_e * x[i]
	} else {
		dtdx[1], dtdx[2] = 0.0, 0.0
	}
	dtdx[0] = m_w
	return m_w * x[i]
}

/* test satellite system (m=0:GPS/SBS,1:GLO,2:GAL,3:BDS,4:QZS,5:IRN) ---------*/
func test_sys(sys, m int) int {
	switch sys {
	case SYS_GPS:
		if m == 0 {
			return 1
		}
	case SYS_SBS:
		if m == 0 {
			return 1
		}
	case SYS_GLO:
		if m == 1 {
			return 1
		}
	case SYS_GAL:
		if m == 2 {
			return 1
		}
	case SYS_CMP:
		if m == 3 {
			return 1
		}
	case SYS_QZS:
		if m == 4 {
			return 1
		}
	case SYS_IRN:
		if m == 5 {
			return 1
		}
	}
	return 0
}

/* DD (double-differenced) phase/code residuals ------------------------------*/
func (rtk *Rtk) DDRes(nav *Nav, dt float64, x, P []float64, sat []int, y, e,
	azel, freq []float64, iu, ir []int,
	ns int, v, H, R []float64, vflg []int) int {
	var (
		opt                                        *PrcOpt = &rtk.Opt
		bl, didxi, didxj, freqi, freqj             float64
		dr, posu, posr                             [3]float64
		tropr, tropu, dtdxr, dtdxu, Ri, Rj, im, Hi []float64
		i, j, k, m, f, nv, b, sysi, sysj, nf       int
		nb                                         [NFREQ*4*2 + 2]float64
	)
	nf = RNF(opt)

	Trace(4, "ddres   : dt=%.1f nx=%d ns=%d\n", dt, rtk.Nx, ns)

	bl = CalcBaseLineLen(x, rtk.Rb[:], dr[:])
	Ecef2Pos(x, posu[:])
	Ecef2Pos(rtk.Rb[:], posr[:])

	Ri = Mat(ns*nf*2+2, 1)
	Rj = Mat(ns*nf*2+2, 1)
	im = Mat(ns, 1)
	tropu = Mat(ns, 1)
	tropr = Mat(ns, 1)
	dtdxu = Mat(ns, 3)
	dtdxr = Mat(ns, 3)

	for i = 0; i < MAXSAT; i++ {
		for j = 0; j < NFREQ; j++ {
			rtk.Ssat[i].Resp[j], rtk.Ssat[i].Resc[j] = 0.0, 0.0
		}
	}
	/* compute factors of ionospheric and tropospheric delay */
	for i = 0; i < ns; i++ {
		if opt.IonoOpt >= IONOOPT_EST {
			im[i] = (IonMapf(posu[:], azel[iu[i]*2:]) + IonMapf(posr[:], azel[ir[i]*2:])) / 2.0
		}
		if opt.TropOpt >= TROPOPT_EST {
			tropu[i] = PrecTrop(rtk.RtkSol.Time, posu[:], 0, azel[iu[i]*2:], opt, x, dtdxu[i*3:])
			tropr[i] = PrecTrop(rtk.RtkSol.Time, posr[:], 1, azel[ir[i]*2:], opt, x, dtdxr[i*3:])
		}
	}
	for m = 0; m < 6; m++ { /* m=0:GPS/SBS,1:GLO,2:GAL,3:BDS,4:QZS,5:IRN */

		if opt.Mode > PMODE_DGPS {
			f = 0
		} else {
			f = nf
		}
		for ; f < nf*2; f++ {

			/* search reference satellite with highest elevation */
			for i, j = -1, 0; j < ns; j++ {
				sysi = int(rtk.Ssat[sat[j]-1].Sys)
				if test_sys(sysi, m) == 0 {
					continue
				}
				if ValidObs(iu[j], ir[j], f, nf, y) == 0 {
					continue
				}
				if i < 0 || azel[1+iu[j]*2] >= azel[1+iu[i]*2] {
					i = j
				}
			}
			if i < 0 {
				continue
			}

			/* make DD (double difference) */
			for j = 0; j < ns; j++ {
				if i == j {
					continue
				}
				sysi = int(rtk.Ssat[sat[i]-1].Sys)
				sysj = int(rtk.Ssat[sat[j]-1].Sys)
				freqi = freq[f%nf+iu[i]*nf]
				freqj = freq[f%nf+iu[j]*nf]
				if test_sys(sysj, m) == 0 {
					continue
				}
				if ValidObs(iu[j], ir[j], f, nf, y) == 0 {
					continue
				}

				if H != nil {
					Hi = H[nv*rtk.Nx:]
					for k = 0; k < rtk.Nx; k++ {
						Hi[k] = 0.0
					}
				}
				/* DD residual */
				v[nv] = (y[f+iu[i]*nf*2] - y[f+ir[i]*nf*2]) -
					(y[f+iu[j]*nf*2] - y[f+ir[j]*nf*2])

				/* partial derivatives by rover position */
				if H != nil {
					for k = 0; k < 3; k++ {
						Hi[k] = -e[k+iu[i]*3] + e[k+iu[j]*3]
					}
				}
				/* DD ionospheric delay term */
				if opt.IonoOpt == IONOOPT_EST {
					didxi = im[i] * SQR(FREQ1/freqi)
					didxj = im[j] * SQR(FREQ1/freqj)
					if f < nf {
						didxi = -im[i] * SQR(FREQ1/freqi)
						didxj = -im[j] * SQR(FREQ1/freqj)
					}
					v[nv] -= didxi*x[RII(sat[i], opt)] - didxj*x[RII(sat[j], opt)]
					if H != nil {
						Hi[RII(sat[i], opt)] = didxi
						Hi[RII(sat[j], opt)] = -didxj
					}
				}
				/* DD tropospheric delay term */
				if opt.TropOpt == TROPOPT_EST || opt.TropOpt == TROPOPT_ESTG {
					v[nv] -= (tropu[i] - tropu[j]) - (tropr[i] - tropr[j])
					kn := 3
					if opt.TropOpt < TROPOPT_ESTG {
						kn = 1
					}
					for k = 0; k < kn; k++ {
						if H == nil {
							continue
						}
						Hi[RIT(0, opt)+k] = (dtdxu[k+i*3] - dtdxu[k+j*3])
						Hi[RIT(1, opt)+k] = -(dtdxr[k+i*3] - dtdxr[k+j*3])
					}
				}
				/* DD phase-bias term */
				if f < nf {
					if opt.IonoOpt != IONOOPT_IFLC {
						v[nv] -= CLIGHT/freqi*x[RIB(sat[i], f, opt)] -
							CLIGHT/freqj*x[RIB(sat[j], f, opt)]
						if H != nil {
							Hi[RIB(sat[i], f, opt)] = CLIGHT / freqi
							Hi[RIB(sat[j], f, opt)] = -CLIGHT / freqj
						}
					} else {
						v[nv] -= x[RIB(sat[i], f, opt)] - x[RIB(sat[j], f, opt)]
						if H != nil {
							Hi[RIB(sat[i], f, opt)] = 1.0
							Hi[RIB(sat[j], f, opt)] = -1.0
						}
					}
				}
				if f < nf {
					rtk.Ssat[sat[j]-1].Resc[f] = float32(v[nv])
				} else {
					rtk.Ssat[sat[j]-1].Resp[f-nf] = float32(v[nv])
				}

				/* test innovation */
				if opt.MaxInno > 0.0 && math.Abs(v[nv]) > opt.MaxInno {
					c := "P"
					if f < nf {
						rtk.Ssat[sat[i]-1].Rejc[f]++
						rtk.Ssat[sat[j]-1].Rejc[f]++
						c = "L"
					}

					rtk.errmsg("outlier rejected (sat=%3d-%3d %s%d v=%.3f)\n",
						sat[i], sat[j], c, f%nf+1, v[nv])
					continue
				}
				/* SD (single-differenced) measurement error variances */
				Ri[nv] = RtkVarianceErr(sat[i], sysi, azel[1+iu[i]*2], bl, dt, f, opt)
				Rj[nv] = RtkVarianceErr(sat[j], sysj, azel[1+iu[j]*2], bl, dt, f, opt)

				/* set valid data flags */
				if opt.Mode > PMODE_DGPS {
					if f < nf {
						rtk.Ssat[sat[i]-1].Vsat[f], rtk.Ssat[sat[j]-1].Vsat[f] = 1, 1
					}
				} else {
					rtk.Ssat[sat[i]-1].Vsat[f-nf], rtk.Ssat[sat[j]-1].Vsat[f-nf] = 1, 1
				}
				c := "P"
				if f < nf {
					c = "L"
				}
				Trace(4, "sat=%3d-%3d %s%d v=%13.3f R=%8.6f %8.6f\n", sat[i],
					sat[j], c, f%nf+1, v[nv], Ri[nv], Rj[nv])
				cf := 1
				if f < nf {
					cf = 0
				}
				vflg[nf] = (sat[i] << 16) | (sat[j] << 8) | (cf << 4) | (f % nf)
				nv++
				nb[b]++
			}
			b++
		}
	}
	/* end of system loop */

	/* baseline length constraint for moving baseline */
	if opt.Mode == PMODE_MOVEB && rtk.ConstBaselineLen(x, P, v, H, Ri, Rj, nv) > 0 {
		vflg[nv] = 3 << 4
		nv++
		nb[b]++
		b++
	}
	if H != nil {
		Trace(2, "H=\n")
		tracemat(5, H, rtk.Nx, nv, 7, 4)
	}

	/* DD measurement error covariance */
	DDCovariance(nb[:], b, Ri, Rj, nv, R)

	return nv
}

/* time-interpolation of residuals (for post-processing) ---------------------*/
func (rtk *Rtk) InterpolationRes(time Gtime, obs []ObsD, n int, nav *Nav, y []float64) float64 {
	var (
		obsb        [MAXOBS]ObsD
		yb          [MAXOBS * NFREQ * 2]float64
		rs          [MAXOBS * 6]float64
		dts         [MAXOBS * 2]float64
		fvar        [MAXOBS]float64
		e           [MAXOBS * 3]float64
		azel        [MAXOBS * 2]float64
		freq        [MAXOBS * NFREQ]float64
		nb          int = 0
		svh         [MAXOBS * 2]int
		opt         *PrcOpt = &rtk.Opt
		ttb         float64
		i, j, k, nf int
	)
	tt := TimeDiff(time, obs[0].Time)
	nf = RNF(opt)

	Trace(4, "intpres : n=%d tt=%.1f\n", n, tt)

	if nb == 0 || math.Abs(tt) < float64(DTTOL) {
		nb = n
		for i = 0; i < n; i++ {
			obsb[i] = obs[i]
		}
		return tt
	}
	ttb = TimeDiff(time, obsb[0].Time)
	if math.Abs(ttb) > opt.MaxTmDiff*2.0 || ttb == tt {
		return tt
	}

	nav.SatPoss(time, obsb[:], nb, opt.SatEph, rs[:], dts[:], fvar[:], svh[:])

	if ZDRes(1, obsb[:], nb, rs[:], dts[:], fvar[:], svh[:], nav, rtk.Rb[:], opt, 1, yb[:], e[:], azel[:], freq[:]) == 0 {
		return tt
	}
	for i = 0; i < n; i++ {
		for j = 0; j < nb; j++ {
			if obsb[j].Sat == obs[i].Sat {
				break
			}
		}
		if j >= nb {
			continue
		}
		yi := i * nf * 2
		ybj := j * nf * 2
		for k = 0; k < nf*2; k, yi, ybj = k+1, yi+1, ybj+1 {
			if y[yi] == 0.0 || yb[ybj] == 0.0 {
				y[yi] = 0.0
			} else {
				y[yi] = (ttb*y[yi] - tt*yb[ybj]) / (ttb - tt)
			}
		}
	}
	if math.Abs(ttb) > math.Abs(tt) {
		return ttb
	}
	return tt
}

/* index for SD to DD transformation matrix D --------------------------------*/
func (rtk *Rtk) DDIndex(ix []int) int {
	var i, j, k, m, f, nb, na, nf int
	nf = RNF(&rtk.Opt)
	na = rtk.Na
	Trace(4, "ddidx   :\n")

	for i = 0; i < MAXSAT; i++ {
		for j = 0; j < NFREQ; j++ {
			rtk.Ssat[i].Fix[j] = 0
		}
	}
	for m = 0; m < 6; m++ { /* m=0:GPS/SBS,1:GLO,2:GAL,3:BDS,4:QZS,5:IRN */

		nofix := (m == 1 && rtk.Opt.GloModeAr == 0) || (m == 3 && rtk.Opt.BDSModeAr == 0)

		for f, k = 0, na; f < nf; f, k = f+1, k+MAXSAT {

			for i = k; i < k+MAXSAT; i++ {
				if rtk.X[i] == 0.0 || test_sys(int(rtk.Ssat[i-k].Sys), m) == 0 ||
					rtk.Ssat[i-k].Vsat[f] == 0 || rtk.Ssat[i-k].Half[f] == 0 {
					continue
				}
				if rtk.Ssat[i-k].Lock[f] > 0 && (rtk.Ssat[i-k].Slip[f]&2) == 0 &&
					rtk.Ssat[i-k].Azel[1] >= rtk.Opt.ElMaskAr && !nofix {
					rtk.Ssat[i-k].Fix[f] = 2 /* fix */
					break
				} else {
					rtk.Ssat[i-k].Fix[f] = 1
				}
			}
			for j = k; j < k+MAXSAT; j++ {
				if i == j || rtk.X[j] == 0.0 || test_sys(int(rtk.Ssat[j-k].Sys), m) == 0 ||
					rtk.Ssat[j-k].Vsat[f] == 0 {
					continue
				}
				if rtk.Ssat[j-k].Lock[f] > 0 && (rtk.Ssat[j-k].Slip[f]&2) == 0 &&
					rtk.Ssat[i-k].Vsat[f] > 0 &&
					rtk.Ssat[j-k].Azel[1] >= rtk.Opt.ElMaskAr && !nofix {
					ix[nb*2] = i   /* state index of ref bias */
					ix[nb*2+1] = j /* state index of target bias */
					nb++
					rtk.Ssat[j-k].Fix[f] = 2 /* fix */
				} else {
					rtk.Ssat[j-k].Fix[f] = 1
				}
			}
		}
	}
	return nb
}

/* restore SD (single-differenced) ambiguity ---------------------------------*/
func (rtk *Rtk) RestoreAmb(bias []float64, nb int, xa []float64) {
	var (
		i, n, m, f, nv, nf int
		index              [MAXSAT]int
	)
	nf = RNF(&rtk.Opt)

	Trace(4, "restamb :\n")

	for i = 0; i < rtk.Nx; i++ {
		xa[i] = rtk.X[i]
	}
	for i = 0; i < rtk.Na; i++ {
		xa[i] = rtk.Xa[i]
	}

	for m = 0; m < 5; m++ {
		for f = 0; f < nf; f++ {

			for n, i = 0, 0; i < MAXSAT; i++ {
				if test_sys(int(rtk.Ssat[i].Sys), m) == 0 || rtk.Ssat[i].Fix[f] != 2 {
					continue
				}
				index[n] = RIB(i+1, f, &rtk.Opt)
				n++
			}
			if n < 2 {
				continue
			}

			xa[index[0]] = rtk.X[index[0]]

			for i = 1; i < n; i++ {
				xa[index[i]] = xa[index[0]] - bias[nv]
				nv++
			}
		}
	}
}

/* hold integer ambiguity ----------------------------------------------------*/
func (rtk *Rtk) HoldAmb(xa []float64) {
	var (
		v, H, R                      []float64
		i, n, m, f, info, nb, nv, nf int
		index                        [MAXSAT]int
	)
	nb = rtk.Nx - rtk.Na
	nf = RNF(&rtk.Opt)

	Trace(4, "holdamb :\n")

	v = Mat(nb, 1)
	H = Zeros(nb, rtk.Nx)

	for m = 0; m < 5; m++ {
		for f = 0; f < nf; f++ {

			for n, i = 0, 0; i < MAXSAT; i++ {
				if test_sys(int(rtk.Ssat[i].Sys), m) == 0 || rtk.Ssat[i].Fix[f] != 2 ||
					rtk.Ssat[i].Azel[1] < rtk.Opt.ElMaskHold {
					continue
				}
				index[n] = RIB(i+1, f, &rtk.Opt)
				n++
				rtk.Ssat[i].Fix[f] = 3 /* hold */
			}
			/* constraint to fixed ambiguity */
			for i = 1; i < n; i++ {
				v[nv] = (xa[index[0]] - xa[index[i]]) - (rtk.X[index[0]] - rtk.X[index[i]])

				H[index[0]+nv*rtk.Nx] = 1.0
				H[index[i]+nv*rtk.Nx] = -1.0
				nv++
			}
		}
	}
	if nv > 0 {
		R = Zeros(nv, nv)
		for i = 0; i < nv; i++ {
			R[i+i*nv] = VAR_HOLDAMB
		}

		/* update states with constraints */
		if info = Filter(rtk.X, rtk.P, H, v, R, rtk.Nx, nv); info > 0 {
			rtk.errmsg("filter error (info=%d)\n", info)
		}
	}
}

/* resolve integer ambiguity by LAMBDA ---------------------------------------*/
func (rtk *Rtk) ResolveAmb_LAMBDA(bias, xa []float64) int {
	var (
		opt                       *PrcOpt = &rtk.Opt
		i, j, nb, info, nx, na    int
		DP, y, b, db, Qb, Qab, QQ []float64
		s                         [2]float64
		ix                        []int
	)
	nx = rtk.Nx
	na = rtk.Na

	Trace(4, "resamb_LAMBDA : nx=%d\n", nx)

	rtk.RtkSol.Ratio = 0.0

	if rtk.Opt.Mode <= PMODE_DGPS || rtk.Opt.ModeAr == ARMODE_OFF || rtk.Opt.ThresAr[0] < 1.0 {
		return 0
	}
	/* index of SD to DD transformation matrix D */
	ix = IMat(nx, 2)
	if nb = rtk.DDIndex(ix); nb <= 0 {
		rtk.errmsg("no valid double-difference\n")
		return 0
	}
	y = Mat(nb, 1)
	DP = Mat(nb, nx-na)
	b = Mat(nb, 2)
	db = Mat(nb, 1)
	Qb = Mat(nb, nb)
	Qab = Mat(na, nb)
	QQ = Mat(na, nb)

	/* y=D*xc, Qb=D*Qc*D', Qab=Qac*D' */
	for i = 0; i < nb; i++ {
		y[i] = rtk.X[ix[i*2]] - rtk.X[ix[i*2+1]]
	}
	for j = 0; j < nx-na; j++ {
		for i = 0; i < nb; i++ {
			DP[i+j*nb] = rtk.P[ix[i*2]+(na+j)*nx] - rtk.P[ix[i*2+1]+(na+j)*nx]
		}
	}
	for j = 0; j < nb; j++ {
		for i = 0; i < nb; i++ {
			Qb[i+j*nb] = DP[i+(ix[j*2]-na)*nb] - DP[i+(ix[j*2+1]-na)*nb]
		}
	}
	for j = 0; j < nb; j++ {
		for i = 0; i < na; i++ {
			Qab[i+j*na] = rtk.P[i+ix[j*2]*nx] - rtk.P[i+ix[j*2+1]*nx]
		}
	}
	/* LAMBDA/MLAMBDA ILS (integer least-square) estimation */
	if info = Lambda(nb, 2, y, Qb, b, s[:]); info == 0 {
		Trace(2, "N(1)=")
		tracemat(4, b, 1, nb, 10, 3)
		Trace(2, "N(2)=")
		tracemat(4, b[nb:], 1, nb, 10, 3)

		rtk.RtkSol.Ratio = 0.0
		if s[0] > 0 {
			rtk.RtkSol.Ratio = float32(s[1] / s[0])
		}
		if rtk.RtkSol.Ratio > 999.9 {
			rtk.RtkSol.Ratio = 999.9
		}

		/* validation by popular ratio-test */
		if s[0] <= 0.0 || s[1]/s[0] >= opt.ThresAr[0] {

			/* transform float to fixed solution (xa=xa-Qab*Qb\(b0-b)) */
			for i = 0; i < na; i++ {
				rtk.Xa[i] = rtk.X[i]
				for j = 0; j < na; j++ {
					rtk.Pa[i+j*na] = rtk.P[i+j*nx]
				}
			}
			for i = 0; i < nb; i++ {
				bias[i] = b[i]
				y[i] -= b[i]
			}
			if MatInv(Qb, nb) == 0 {
				MatMul("NN", nb, 1, nb, 1.0, Qb, y, 0.0, db)
				MatMul("NN", na, 1, nb, -1.0, Qab, db, 1.0, rtk.Xa)

				/* covariance of fixed solution (Qa=Qa-Qab*Qb^-1*Qab') */
				MatMul("NN", na, nb, nb, 1.0, Qab, Qb, 0.0, QQ)
				MatMul("NT", na, na, nb, -1.0, QQ, Qab, 1.0, rtk.Pa)

				cs := 0.0
				if s[0] != 0.0 {
					cs = s[1] / s[0]
				}
				Trace(2, "resamb : validation ok (nb=%d ratio=%.2f s=%.2f/%.2f)\n",
					nb, cs, s[0], s[1])

				/* restore SD ambiguity */
				rtk.RestoreAmb(bias, nb, xa)
			} else {
				nb = 0
			}
		} else { /* validation failed */
			rtk.errmsg("ambiguity validation failed (nb=%d ratio=%.2f s=%.2f/%.2f)\n",
				nb, s[1]/s[0], s[0], s[1])
			nb = 0
		}
	} else {
		rtk.errmsg("lambda error (info=%d)\n", info)
		nb = 0
	}
	return nb /* number of ambiguities */
}

/* validation of solution ----------------------------------------------------*/
func (rtk *Rtk) ValidPos(v, R []float64, vflg []int, nv int, thres float64) int {
	var (
		i, stat, sat1, sat2, ctype, freq int
		stype                            string
	)
	fact := thres * thres
	stat = 1

	Trace(4, "valpos  : nv=%d thres=%.1f\n", nv, thres)

	/* post-fit residual test */
	for i = 0; i < nv; i++ {
		if v[i]*v[i] <= fact*R[i+i*nv] {
			continue
		}
		sat1 = (vflg[i] >> 16) & 0xFF
		sat2 = (vflg[i] >> 8) & 0xFF
		ctype = (vflg[i] >> 4) & 0xF
		freq = vflg[i] & 0xF
		if ctype == 0 || ctype == 1 {
			stype = "L"
		} else {
			stype = "C"
		}
		rtk.errmsg("large residual (sat=%2d-%2d %s%d v=%6.3f sig=%.3f)\n",
			sat1, sat2, stype, freq+1, v[i], SQRT(R[i+i*nv]))
	}
	return stat
}

/* relative positioning ------------------------------------------------------*/
func (rtk *Rtk) RelativePos(obs []ObsD, nu, nr int, nav *Nav) int {
	var (
		rs, dts, fvar, y, e, azel, freq, v, H, R, xp, Pp, xa, bias []float64
		dt                                                         float64
		i, j, f, n, ns, ny, nv, niter, stat, nf, info              int
		sat, iu, ir                                                [MAXSAT]int
		vflg                                                       [MAXOBS*NFREQ*2 + 1]int
		svh                                                        [MAXOBS * 2]int
	)
	opt := &rtk.Opt
	time := obs[0].Time
	n = nu + nr
	stat = SOLQ_FLOAT
	if rtk.Opt.Mode <= PMODE_DGPS {
		stat = SOLQ_DGPS
	}
	nf = opt.Nf
	if opt.IonoOpt == IONOOPT_IFLC {
		nf = 1
	}

	Trace(4, "relpos  : nx=%d nu=%d nr=%d\n", rtk.Nx, nu, nr)

	dt = TimeDiff(time, obs[nu].Time)

	rs = Mat(6, n)
	dts = Mat(2, n)
	fvar = Mat(1, n)
	y = Mat(nf*2, n)
	e = Mat(3, n)
	azel = Zeros(2, n)
	freq = Zeros(nf, n)

	for i = 0; i < MAXSAT; i++ {
		rtk.Ssat[i].Sys = uint8(SatSys(i+1, nil))
		for j = 0; j < NFREQ; j++ {
			rtk.Ssat[i].Vsat[j] = 0
		}
		for j = 1; j < NFREQ; j++ {
			rtk.Ssat[i].Snr[j] = 0
		}
	}
	/* satellite positions/clocks */
	nav.SatPoss(time, obs, n, opt.SatEph, rs, dts, fvar, svh[:])

	/* UD (undifferenced) residuals for base station */
	if ZDRes(1, obs[nu:], nr, rs[nu*6:], dts[nu*2:], fvar[nu:], svh[nu:], nav, rtk.Rb[:], opt, 1,
		y[nu*nf*2:], e[nu*3:], azel[nu*2:], freq[nu*nf:]) == 0 {
		rtk.errmsg("initial base station position error\n")
		return 0
	}
	/* time-interpolation of residuals (for post-processing) */
	if opt.IntPref > 0 {
		dt = rtk.InterpolationRes(time, obs[nu:], nr, nav, y[nu*nf*2:])
	}
	/* select common satellites between rover and base-station */
	if ns = SelSat(obs, azel, nu, nr, opt, sat[:], iu[:], ir[:]); ns <= 0 {
		rtk.errmsg("no common satellite\n")
		return 0
	}
	/* temporal update of states */
	rtk.UpdateState(obs, sat[:], iu[:], ir[:], ns, nav)

	Trace(4, "x(0)=")
	tracemat(4, rtk.X, 1, RNR(opt), 13, 4)

	xp = Mat(rtk.Nx, 1)
	Pp = Zeros(rtk.Nx, rtk.Nx)
	xa = Mat(rtk.Nx, 1)
	MatCpy(xp, rtk.X, rtk.Nx, 1)

	ny = ns*nf*2 + 2
	v = Mat(ny, 1)
	H = Zeros(rtk.Nx, ny)
	R = Mat(ny, ny)
	bias = Mat(rtk.Nx, 1)

	/* add 2 iterations for baseline-constraint moving-base */
	if opt.Mode == PMODE_MOVEB && opt.Baseline[0] > 0.0 {
		niter = opt.NoIter + 2
	} else {
		niter = opt.NoIter
	}
	for i = 0; i < niter; i++ {
		/* UD (undifferenced) residuals for rover */
		if ZDRes(0, obs, nu, rs, dts, fvar, svh[:], nav, xp, opt, 0, y, e, azel, freq) == 0 {
			rtk.errmsg("rover initial position error\n")
			stat = SOLQ_NONE
			break
		}
		/* DD (double-differenced) residuals and partial derivatives */
		if nv = rtk.DDRes(nav, dt, xp, Pp, sat[:], y, e, azel, freq, iu[:], ir[:], ns, v, H, R, vflg[:]); nv < 1 {
			rtk.errmsg("no double-differenced residual\n")
			stat = SOLQ_NONE
			break
		}
		/* Kalman filter measurement update */
		MatCpy(Pp, rtk.P, rtk.Nx, rtk.Nx)
		if info = Filter(xp, Pp, H, v, R, rtk.Nx, nv); info > 0 {
			rtk.errmsg("filter error (info=%d)\n", info)
			stat = SOLQ_NONE
			break
		}
		Trace(5, "x(%d)=", i+1)
		tracemat(4, xp, 1, RNR(opt), 13, 4)
	}
	if stat != SOLQ_NONE && ZDRes(0, obs, nu, rs, dts, fvar, svh[:], nav, xp, opt, 0, y, e, azel, freq) != 0 {

		/* post-fit residuals for float solution */
		nv = rtk.DDRes(nav, dt, xp, Pp, sat[:], y, e, azel, freq, iu[:], ir[:], ns, v, nil, R, vflg[:])

		/* validation of float solution */
		if rtk.ValidPos(v, R, vflg[:], nv, 4.0) > 0 {

			/* update state and covariance matrix */
			MatCpy(rtk.X, xp, rtk.Nx, 1)
			MatCpy(rtk.P, Pp, rtk.Nx, rtk.Nx)

			/* update ambiguity control struct */
			rtk.RtkSol.Ns = 0
			for i = 0; i < ns; i++ {
				for f = 0; f < nf; f++ {
					if rtk.Ssat[sat[i]-1].Vsat[f] == 0 {
						continue
					}
					rtk.Ssat[sat[i]-1].Lock[f]++
					rtk.Ssat[sat[i]-1].Outc[f] = 0
					if f == 0 {
						rtk.RtkSol.Ns++ /* valid satellite count by L1 */
					}
				}
			}
			/* lack of valid satellites */
			if rtk.RtkSol.Ns < 4 {
				stat = SOLQ_NONE
			}
		} else {
			stat = SOLQ_NONE
		}
	}
	/* resolve integer ambiguity by LAMBDA */
	if stat != SOLQ_NONE && rtk.ResolveAmb_LAMBDA(bias, xa) > 1 {

		if ZDRes(0, obs, nu, rs, dts, fvar, svh[:], nav, xa, opt, 0, y, e, azel, freq) > 0 {

			/* post-fit reisiduals for fixed solution */
			nv = rtk.DDRes(nav, dt, xa, nil, sat[:], y, e, azel, freq, iu[:], ir[:], ns, v, nil, R, vflg[:])

			/* validation of fixed solution */
			if rtk.ValidPos(v, R, vflg[:], nv, 4.0) > 0 {

				/* hold integer ambiguity */
				if rtk.Nfix++; rtk.Nfix >= rtk.Opt.MinFix &&
					rtk.Opt.ModeAr == ARMODE_FIXHOLD {
					rtk.HoldAmb(xa)
				}
				stat = SOLQ_FIX
			}
		}
	}
	/* save solution status */
	if stat == SOLQ_FIX {
		for i = 0; i < 3; i++ {
			rtk.RtkSol.Rr[i] = rtk.Xa[i]
			rtk.RtkSol.Qr[i] = float32(rtk.Pa[i+i*rtk.Na])
		}
		rtk.RtkSol.Qr[3] = float32(rtk.Pa[1])
		rtk.RtkSol.Qr[4] = float32(rtk.Pa[1+2*rtk.Na])
		rtk.RtkSol.Qr[5] = float32(rtk.Pa[2])

		if rtk.Opt.Dynamics > 0 { /* velocity and covariance */
			for i = 3; i < 6; i++ {
				rtk.RtkSol.Rr[i] = rtk.Xa[i]
				rtk.RtkSol.Qv[i-3] = float32(rtk.Pa[i+i*rtk.Na])
			}
			rtk.RtkSol.Qv[3] = float32(rtk.Pa[4+3*rtk.Na])
			rtk.RtkSol.Qv[4] = float32(rtk.Pa[5+4*rtk.Na])
			rtk.RtkSol.Qv[5] = float32(rtk.Pa[5+3*rtk.Na])
		}
	} else {
		for i = 0; i < 3; i++ {
			rtk.RtkSol.Rr[i] = rtk.X[i]
			rtk.RtkSol.Qr[i] = float32(rtk.P[i+i*rtk.Nx])
		}
		rtk.RtkSol.Qr[3] = float32(rtk.P[1])
		rtk.RtkSol.Qr[4] = float32(rtk.P[1+2*rtk.Nx])
		rtk.RtkSol.Qr[5] = float32(rtk.P[2])

		if rtk.Opt.Dynamics > 0 { /* velocity and covariance */
			for i = 3; i < 6; i++ {
				rtk.RtkSol.Rr[i] = rtk.X[i]
				rtk.RtkSol.Qv[i-3] = float32(rtk.P[i+i*rtk.Nx])
			}
			rtk.RtkSol.Qv[3] = float32(rtk.P[4+3*rtk.Nx])
			rtk.RtkSol.Qv[4] = float32(rtk.P[5+4*rtk.Nx])
			rtk.RtkSol.Qv[5] = float32(rtk.P[5+3*rtk.Nx])
		}
		rtk.Nfix = 0
	}
	for i = 0; i < n; i++ {
		for j = 0; j < nf; j++ {
			if obs[i].L[j] == 0.0 {
				continue
			}
			rtk.Ssat[obs[i].Sat-1].Pt[obs[i].Rcv-1][j] = obs[i].Time
			rtk.Ssat[obs[i].Sat-1].Ph[obs[i].Rcv-1][j] = obs[i].L[j]
		}
	}
	for i = 0; i < ns; i++ {
		for j = 0; j < nf; j++ {

			/* output snr of rover receiver */
			rtk.Ssat[sat[i]-1].Snr[j] = obs[iu[i]].SNR[j]
		}
	}
	for i = 0; i < MAXSAT; i++ {
		for j = 0; j < nf; j++ {
			if rtk.Ssat[i].Fix[j] == 2 && stat != SOLQ_FIX {
				rtk.Ssat[i].Fix[j] = 1
			}
			if rtk.Ssat[i].Slip[j]&1 > 0 {
				rtk.Ssat[i].Slipc[j]++
			}
		}
	}

	if stat != SOLQ_NONE {
		rtk.RtkSol.Stat = uint8(stat)
		return 1
	}

	return 0
}

/* initialize RTK control ------------------------------------------------------
* initialize RTK control struct
* args   : rtk_t    *rtk    IO  TKk control/result struct
*          prcopt_t *opt    I   positioning options (see rtklib.h)
* return : none
*-----------------------------------------------------------------------------*/
func (rtk *Rtk) InitRtk(opt *PrcOpt) {
	var (
		sol0  Sol
		ambc0 AmbC
		ssat0 SSat
		i     int
	)

	Trace(4, "rtkinit :\n")

	rtk.RtkSol = sol0
	for i = 0; i < 6; i++ {
		rtk.Rb[i] = 0.0
	}
	if opt.Mode <= PMODE_FIXED {
		rtk.Nx = RNX(opt)
		rtk.Na = RNR(opt)
	} else {
		rtk.Nx = pppnx(opt)
		rtk.Na = pppnx(opt)
	}
	rtk.Tt = 0.0
	rtk.X = Zeros(rtk.Nx, 1)
	rtk.P = Zeros(rtk.Nx, rtk.Nx)
	rtk.Xa = Zeros(rtk.Na, 1)
	rtk.Pa = Zeros(rtk.Na, rtk.Na)
	rtk.Nfix = 0
	for i = 0; i < MAXSAT; i++ {
		rtk.Ambc[i] = ambc0
		rtk.Ssat[i] = ssat0
	}
	rtk.ErrBuf = ""
	rtk.Opt = *opt
}

/* free rtk control ------------------------------------------------------------
* free memory for rtk control struct
* args   : rtk_t    *rtk    IO  rtk control/result struct
* return : none
*-----------------------------------------------------------------------------*/
func (rtk *Rtk) FreeRtk() {
	Trace(4, "rtkfree :\n")

	rtk.Nx, rtk.Na = 0, 0
	rtk.X = nil
	rtk.P = nil
	rtk.Xa = nil
	rtk.Pa = nil
}

/* precise positioning ---------------------------------------------------------
* input observation data and navigation message, compute rover position by
* precise positioning
* args   : rtk *Rtk       IO  RTK control/result struct
*            rtk.sol       IO  solution
*                .time      O   solution time
*                .rr[]      IO  rover position/velocity
*                               (I:fixed mode,O:single mode)
*                .dtr[0]    O   receiver clock bias (s)
*                .dtr[1-5]  O   receiver GLO/GAL/BDS/IRN/QZS-GPS time offset (s)
*                .Qr[]      O   rover position covarinace
*                .stat      O   solution status (SOLQ_???)
*                .ns        O   number of valid satellites
*                .age       O   age of differential (s)
*                .ratio     O   ratio factor for ambiguity validation
*            rtk.rb[]      IO  base station position/velocity
*                               (I:relative mode,O:moving-base mode)
*            rtk.nx        I   number of all states
*            rtk.na        I   number of integer states
*            rtk.ns        O   number of valid satellites in use
*            rtk.tt        O   time difference between current and previous (s)
*            rtk.x[]       IO  float states pre-filter and post-filter
*            rtk.P[]       IO  float covariance pre-filter and post-filter
*            rtk.xa[]      O   fixed states after AR
*            rtk.Pa[]      O   fixed covariance after AR
*            rtk.ssat[s]   IO  satellite {s+1} status
*                .sys       O   system (SYS_???)
*                .az   [r]  O   azimuth angle   (rad) (r=0:rover,1:base)
*                .el   [r]  O   elevation angle (rad) (r=0:rover,1:base)
*                .vs   [r]  O   data valid single     (r=0:rover,1:base)
*                .resp [f]  O   freq(f+1) pseudorange residual (m)
*                .resc [f]  O   freq(f+1) carrier-phase residual (m)
*                .vsat [f]  O   freq(f+1) data vaild (0:invalid,1:valid)
*                .fix  [f]  O   freq(f+1) ambiguity flag
*                               (0:nodata,1:float,2:fix,3:hold)
*                .slip [f]  O   freq(f+1) cycle slip flag
*                               (bit8-7:rcv1 LLI, bit6-5:rcv2 LLI,
*                                bit2:parity unknown, bit1:slip)
*                .lock [f]  IO  freq(f+1) carrier lock count
*                .outc [f]  IO  freq(f+1) carrier outage count
*                .slipc[f]  IO  freq(f+1) cycle slip count
*                .rejc [f]  IO  freq(f+1) data reject count
*                .gf        IO  geometry-free phase (L1-L2) (m)
*                .gf2       IO  geometry-free phase (L1-L5) (m)
*            rtk.nfix      IO  number of continuous fixes of ambiguity
*            rtk.neb       IO  bytes of error message buffer
*            rtk.errbuf    IO  error message buffer
*            rtk.tstr      O   time string for debug
*            rtk.opt       I   processing options
*          obsd_t *obs      I   observation data for an epoch
*                               obs[i].rcv=1:rover,2:reference
*                               sorted by receiver and satellte
*          int    n         I   number of observation data
*          nav_t  *nav      I   navigation messages
* return : status (0:no solution,1:valid solution)
* notes  : before calling function, base station position rtk.sol.rb[] should
*          be properly set for relative mode except for moving-baseline
*-----------------------------------------------------------------------------*/
func (rtk *Rtk) RtkPos(obs []ObsD, n int, nav *Nav) int {
	var (
		solb      Sol
		time      Gtime
		i, nu, nr int
		msg       string
	)
	opt := &rtk.Opt

	Trace(4, "rtkpos  : time=%s n=%d\n", TimeStr(obs[0].Time, 3), n)
	Trace(4, "obs=\n")
	traceobs(4, obs, n)

	/* set base staion position */
	if opt.RefPos <= POSOPT_RINEX && opt.Mode != PMODE_SINGLE &&
		opt.Mode != PMODE_MOVEB {
		for i = 0; i < 6; i++ {
			rtk.Rb[i] = 0.0
			if i < 3 {
				rtk.Rb[i] = opt.Rb[i]
			}
		}
	}
	/* count rover/base station observations */
	for nu = 0; nu < n && obs[nu].Rcv == 1; nu++ {
	}
	for nr = 0; nu+nr < n && obs[nu+nr].Rcv == 2; nr++ {
	}

	time = rtk.RtkSol.Time /* previous epoch */

	/* rover position by single point positioning */
	if PntPos(obs, nu, nav, &rtk.Opt, &rtk.RtkSol, nil, rtk.Ssat[:], &msg) == 0 {
		rtk.errmsg("point pos error (%s)\n", msg)

		if rtk.Opt.Dynamics == 0 {
			rtk.OutSolStat()
			return 0
		}
	}
	if time.Time != 0 {
		rtk.Tt = TimeDiff(rtk.RtkSol.Time, time)
	}

	/* single point positioning */
	if opt.Mode == PMODE_SINGLE {
		rtk.OutSolStat()
		return 1
	}
	/* suppress output of single solution */
	if opt.OutSingle == 0 {
		rtk.RtkSol.Stat = SOLQ_NONE
	}
	/* precise point positioning */
	if opt.Mode >= PMODE_PPP_KINEMA {
		rtk.PPPos(obs, nu, nav)
		rtk.OutSolStat()
		return 1
	}
	/* check number of data of base station and age of differential */
	if nr == 0 {
		rtk.errmsg("no base station observation data for rtk\n")
		rtk.OutSolStat()
		return 1
	}
	if opt.Mode == PMODE_MOVEB { /*  moving baseline */

		/* estimate position/velocity of base station */
		if PntPos(obs[+nu:], nr, nav, &rtk.Opt, &solb, nil, nil, &msg) == 0 {
			rtk.errmsg("base station position error (%s)\n", msg)
			return 0
		}
		rtk.RtkSol.Age = float32(TimeDiff(rtk.RtkSol.Time, solb.Time))

		if math.Abs(float64(rtk.RtkSol.Age)) > TTOL_MOVEB {
			rtk.errmsg("time sync error for moving-base (age=%.1f)\n", rtk.RtkSol.Age)
			return 0
		}
		for i = 0; i < 6; i++ {
			rtk.Rb[i] = solb.Rr[i]
		}

		/* time-synchronized position of base station */
		for i = 0; i < 3; i++ {
			rtk.Rb[i] += rtk.Rb[i+3] * float64(rtk.RtkSol.Age)
		}
	} else {
		rtk.RtkSol.Age = float32(TimeDiff(obs[0].Time, obs[nu].Time))

		if math.Abs(float64(rtk.RtkSol.Age)) > opt.MaxTmDiff {
			rtk.errmsg("age of differential error (age=%.1f)\n", rtk.RtkSol.Age)
			rtk.OutSolStat()
			return 1
		}
	}
	/* relative potitioning */
	rtk.RelativePos(obs, nu, nr, nav)
	rtk.OutSolStat()

	return 1
}
