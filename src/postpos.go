/*------------------------------------------------------------------------------
* postpos.c : post-processing positioning
*
*          Copyright (C) 2007-2020 by T.TAKASU, All rights reserved.
*
* version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
* history : 2007/05/08  1.0  new
*           2008/06/16  1.1  support binary inputs
*           2009/01/02  1.2  support new rtk positioing api
*           2009/09/03  1.3  fix bug on combined mode of moving-baseline
*           2009/12/04  1.4  fix bug on obs data buffer overflow
*           2010/07/26  1.5  support ppp-kinematic and ppp-static
*                            support multiple sessions
*                            support sbas positioning
*                            changed api:
*                                postpos()
*                            deleted api:
*                                postposopt()
*           2010/08/16  1.6  fix bug sbas message synchronization (2.4.0_p4)
*           2010/12/09  1.7  support qzss lex and ssr corrections
*           2011/02/07  1.8  fix bug on sbas navigation data conflict
*           2011/03/22  1.9  add function reading g_tec file
*           2011/08/20  1.10 fix bug on freez if solstatic=single and combined
*           2011/09/15  1.11 add function reading stec file
*           2012/02/01  1.12 support keyword expansion of rtcm ssr corrections
*           2013/03/11  1.13 add function reading otl and erp data
*           2014/06/29  1.14 fix problem on overflow of # of satellites
*           2015/03/23  1.15 fix bug on ant type replacement by rinex header
*                            fix bug on combined filter for moving-base mode
*           2015/04/29  1.16 fix bug on reading rtcm ssr corrections
*                            add function to read satellite fcb
*                            add function to read stec and troposphere file
*                            add keyword replacement in dcb, erp and ionos file
*           2015/11/13  1.17 add support of L5 antenna phase center paramters
*                            add *.stec and *.trp file for ppp correction
*           2015/11/26  1.18 support opt.freqopt(disable L2)
*           2016/01/12  1.19 add carrier-phase bias correction by ssr
*           2016/07/31  1.20 fix error message problem in rnx2rtkp
*           2016/08/29  1.21 suppress warnings
*           2016/10/10  1.22 fix bug on identification of file fopt.blq
*           2017/06/13  1.23 add smoother of velocity solution
*           2020/11/30  1.24 use API sat2freq() to get carrier frequency
*                            fix bug on select best solution in static mode
*                            delete function to use L2 instead of L5 PCV
*                            writing solution file in binary mode
*		    2022/05/31 1.0  rewrite postpos.c with golang by fxb
*-----------------------------------------------------------------------------*/

package gnssgo

import (
	"bufio"
	"fmt"
	"math"
	"os"
	"strings"
)

const MAXPRCDAYS int = 100 /* max days of continuous processing */
const MAXINFILE int = 1000 /* max number of input files */

/* constants/global variables ------------------------------------------------*/

var (
	pcvss     Pcvs            /* receiver antenna parameters */
	pcvsr     Pcvs            /* satellite antenna parameters */
	obss      Obs             /* observation data */
	navs      Nav             /* navigation data */
	sbss      Sbs             /* sbas messages */
	stas      [MAXRCV]Sta     /* station infomation */
	nepoch    int         = 0 /* number of observation epochs */
	iobsu     int         = 0 /* current rover observation data index */
	iobsr     int         = 0 /* current reference observation data index */
	isbs      int         = 0 /* current sbas message index */
	revs      int         = 0 /* analysis direction (0:forward,1:backward) */
	aborts    int         = 0 /* abort status */
	solf      []Sol           /* forward solutions */
	solb      []Sol           /* backward solutions */
	rbf       []float64       /* forward base positions */
	rbb       []float64       /* backward base positions */
	isolf     int         = 0 /* current forward solutions index */
	isolb     int         = 0 /* current backward solutions index */
	proc_rov  string          /* rover for current processing */
	proc_base string          /* base station for current processing */
	rtcm_file string          /* rtcm data file */
	rtcm_path string          /* rtcm data path */
	rtcm      *Rtcm           /* rtcm control struct */
	fp_rtcm   *os.File    = nil /* rtcm data file pointer */)

/* show message and check break ----------------------------------------------*/
func checkbrk(format string, v ...interface{}) int {
	buff := fmt.Sprintf(format, v...)
	if len(format) == 0 {
		ShowMsg_Ptr("")
	}
	switch {
	case len(proc_rov) > 0 && len(proc_base) > 0:
		buff += fmt.Sprintf(" (%s-%s)", proc_rov, proc_base)
	case len(proc_rov) > 0:
		buff += fmt.Sprintf(" (%s)", proc_rov)
	case len(proc_base) > 0:
		buff += fmt.Sprintf(" (%s)", proc_base)
	}
	return ShowMsg_Ptr(buff)
}

/* output reference position -------------------------------------------------*/
func OutRefPos(fp *os.File, r []float64, opt *SolOpt) {
	var pos, dms1, dms2 [3]float64
	sep := opt.Sep

	Trace(4, "outrpos :\n")

	switch opt.Posf {
	case SOLF_LLH, SOLF_ENU:
		Ecef2Pos(r, pos[:])
		if opt.DegF > 0 {
			Deg2Dms(pos[0]*R2D, dms1[:], 5)
			Deg2Dms(pos[1]*R2D, dms2[:], 5)
			fmt.Fprintf(fp, "%3.0f%s%02.0f%s%08.5f%s%4.0f%s%02.0f%s%08.5f%s%10.4f",
				dms1[0], sep, dms1[1], sep, dms1[2], sep, dms2[0], sep, dms2[1],
				sep, dms2[2], sep, pos[2])
		} else {
			fmt.Fprintf(fp, "%13.9f%s%14.9f%s%10.4f", pos[0]*R2D, sep, pos[1]*R2D,
				sep, pos[2])
		}
	case SOLF_XYZ:
		fmt.Fprintf(fp, "%14.4f%s%14.4f%s%14.4f", r[0], sep, r[1], sep, r[2])
	}
}

/* output header -------------------------------------------------------------*/
func OutHeader(fp *os.File, file []string, n int, popt *PrcOpt, sopt *SolOpt) {
	var (
		s1           []string = []string{"GPST", "UTC", "JST"}
		ts, te       Gtime
		t1, t2       float64
		i, j, w1, w2 int
		s2, s3       string
	)

	Trace(4, "outheader: n=%d\n", n)

	if sopt.Posf == SOLF_NMEA || sopt.Posf == SOLF_STAT {
		return
	}
	if sopt.OutHead > 0 {
		if len(sopt.Prog) == 0 {
			fmt.Fprintf(fp, "%s program   : RTKLIB ver.%s\n", COMMENTH, VER_GNSSGO)
		} else {
			fmt.Fprintf(fp, "%s program   : %s\n", COMMENTH, sopt.Prog)
		}
		for i = 0; i < n; i++ {
			fmt.Fprintf(fp, "%s inp file  : %s\n", COMMENTH, file[i])
		}
		for i = 0; i < obss.N(); i++ {
			if obss.Data[i].Rcv == 1 {
				break
			}
		}
		for j = obss.N() - 1; j >= 0; j-- {
			if obss.Data[j].Rcv == 1 {
				break
			}
		}
		if j < i {
			fmt.Fprintf(fp, "\n%s no rover obs data\n", COMMENTH)
			return
		}
		ts = obss.Data[i].Time
		te = obss.Data[j].Time
		t1 = Time2GpsT(ts, &w1)
		t2 = Time2GpsT(te, &w2)
		if sopt.TimeS >= 1 {
			ts = GpsT2Utc(ts)
		}
		if sopt.TimeS >= 1 {
			te = GpsT2Utc(te)
		}
		if sopt.TimeS == 2 {
			ts = TimeAdd(ts, 9*3600.0)
		}
		if sopt.TimeS == 2 {
			te = TimeAdd(te, 9*3600.0)
		}
		Time2Str(ts, &s2, 1)
		Time2Str(te, &s3, 1)
		fmt.Fprintf(fp, "%s obs start : %s %s (week%04d %8.1fs)\n", COMMENTH, s2, s1[sopt.TimeS], w1, t1)
		fmt.Fprintf(fp, "%s obs end   : %s %s (week%04d %8.1fs)\n", COMMENTH, s3, s1[sopt.TimeS], w2, t2)
	}
	if sopt.OutOpt > 0 {
		OutPrcOpt(fp, popt)
	}
	if PMODE_DGPS <= popt.Mode && popt.Mode <= PMODE_FIXED && popt.Mode != PMODE_MOVEB {
		fmt.Fprintf(fp, "%s ref pos   :", COMMENTH)
		OutRefPos(fp, popt.Rb[:], sopt)
		fp.WriteString("\n")
	}
	if sopt.OutHead > 0 || sopt.OutOpt > 0 {
		fmt.Fprintf(fp, "%s\n", COMMENTH)
	}
	OutSolHead(fp, sopt)
}

/* search next observation data index ----------------------------------------*/
func (obs *Obs) NextObsf(i *int, rcv int) int {
	var n int
	for ; *i < obs.N(); (*i)++ {
		if int(obs.Data[*i].Rcv) == rcv {
			break
		}
	}
	for n = 0; *i+n < obs.N(); n++ {
		tt := TimeDiff(obs.Data[*i+n].Time, obs.Data[*i].Time)
		if int(obs.Data[*i+n].Rcv) != rcv || tt > float64(DTTOL) {
			break
		}
	}
	return n
}
func (obs *Obs) NextObsb(i *int, rcv int) int {
	var n int
	for ; *i >= 0; (*i)-- {
		if int(obs.Data[*i].Rcv) == rcv {
			break
		}
	}
	for n = 0; *i-n >= 0; n++ {
		tt := TimeDiff(obs.Data[*i-n].Time, obs.Data[*i].Time)
		if int(obs.Data[*i-n].Rcv) != rcv || tt < -float64(DTTOL) {
			break
		}
	}
	return n
}

/* update rtcm ssr correction ------------------------------------------------*/
func UpdateRtcmSsr(time Gtime) {
	var path string

	/* open or swap rtcm file */
	RepPath(rtcm_file, &path, time, "", "")

	if strings.Compare(path, rtcm_path) != 0 {
		rtcm_path = path

		if fp_rtcm != nil {
			fp_rtcm.Close()
		}
		fp_rtcm, _ = os.OpenFile(path, os.O_RDONLY, 0666)
		if fp_rtcm != nil {
			rtcm.Time = time
			rtcm.InputRtcm3f(fp_rtcm)
			Trace(2, "rtcm file open: %s\n", path)
		}
	}
	if fp_rtcm == nil {
		return
	}

	/* read rtcm file until current time */
	for TimeDiff(rtcm.Time, time) < 1e-3 {

		if rtcm.InputRtcm3f(fp_rtcm) < -1 {
			break
		}

		/* update ssr corrections */
		for i := 0; i < MAXSAT; i++ {
			if rtcm.Ssr[i].Update == 0 ||
				rtcm.Ssr[i].Iod[0] != rtcm.Ssr[i].Iod[1] ||
				TimeDiff(time, rtcm.Ssr[i].T0[0]) < -1e-3 {
				continue
			}
			navs.Ssr[i] = rtcm.Ssr[i]
			rtcm.Ssr[i].Update = 0
		}
	}
}

/* input obs data, navigation messages and sbas correction -------------------*/
func InputObs(obs []ObsD, solq int, popt *PrcOpt) int {
	var (
		time         Gtime
		i, nu, nr, n int
	)

	Trace(4, "infunc  : revs=%d iobsu=%d iobsr=%d isbs=%d\n", revs, iobsu, iobsr, isbs)

	if 0 <= iobsu && iobsu < obss.N() {
		time = obss.Data[iobsu].Time
		// settime(time);
		if checkbrk("processing : %s Q=%d", TimeStr(time, 0), solq) > 0 {
			aborts = 1
			ShowMsg_Ptr("aborted")
			return -1
		}
	}
	if revs == 0 { /* input forward data */
		if nu = obss.NextObsf(&iobsu, 1); nu <= 0 {
			return -1
		}
		if popt.IntPref > 0 {
			for nr = obss.NextObsf(&iobsr, 2); nr > 0; {
				if TimeDiff(obss.Data[iobsr].Time, obss.Data[iobsu].Time) > -float64(DTTOL) {
					break
				}
				iobsr += nr
				nr = obss.NextObsf(&iobsr, 2)
			}
		} else {
			i = iobsr
			nr = obss.NextObsf(&i, 2)
			for nr > 0 {
				{
					if TimeDiff(obss.Data[i].Time, obss.Data[iobsu].Time) > float64(DTTOL) {
						break
					}
					iobsr = i
					i += nr
					nr = obss.NextObsf(&i, 2)
				}
			}
		}
		nr = obss.NextObsf(&iobsr, 2)
		if nr <= 0 {
			nr = obss.NextObsf(&iobsr, 2)
		}
		for i = 0; i < nu && n < MAXOBS*2; i++ {
			obs[n] = obss.Data[iobsu+i]
			n++
		}
		for i = 0; i < nr && n < MAXOBS*2; i++ {
			obs[n] = obss.Data[iobsr+i]
			n++
		}
		iobsu += nu

		/* update sbas corrections */
		for isbs < sbss.N() {
			time = GpsT2Time(sbss.Msgs[isbs].Week, float64(sbss.Msgs[isbs].Tow))

			if GetBitU(sbss.Msgs[isbs].Msg[:], 8, 6) != 9 { /* except for geo nav */
				SbsUpdateCorr(&sbss.Msgs[isbs], &navs)
			}
			if TimeDiff(time, obs[0].Time) > -1.0-float64(DTTOL) {
				break
			}
			isbs++
		}
		/* update rtcm ssr corrections */
		if len(rtcm_file) > 0 {
			UpdateRtcmSsr(obs[0].Time)
		}
	} else { /* input backward data */
		if nu = obss.NextObsb(&iobsu, 1); nu <= 0 {
			return -1
		}
		if popt.IntPref > 0 {
			for nr = obss.NextObsb(&iobsr, 2); nr > 0; {
				if TimeDiff(obss.Data[iobsr].Time, obss.Data[iobsu].Time) < float64(DTTOL) {
					break
				}
				iobsr -= nr
				nr = obss.NextObsb(&iobsr, 2)
			}
		} else {
			i = iobsr
			nr = obss.NextObsb(&i, 2)
			for nr > 0 {
				if TimeDiff(obss.Data[i].Time, obss.Data[iobsu].Time) < -float64(DTTOL) {
					break
				}
				iobsr = i
				i -= nr
				nr = obss.NextObsb(&i, 2)
			}
		}
		nr = obss.NextObsb(&iobsr, 2)
		for i = 0; i < nu && n < MAXOBS*2; i++ {
			obs[n] = obss.Data[iobsu-nu+1+i]
			n++
		}
		for i = 0; i < nr && n < MAXOBS*2; i++ {
			obs[n] = obss.Data[iobsr-nr+1+i]
			n++
		}
		iobsu -= nu

		/* update sbas corrections */
		for isbs >= 0 {
			time = GpsT2Time(sbss.Msgs[isbs].Week, float64(sbss.Msgs[isbs].Tow))

			if GetBitU(sbss.Msgs[isbs].Msg[:], 8, 6) != 9 { /* except for geo nav */
				SbsUpdateCorr(&sbss.Msgs[isbs], &navs)
			}
			if TimeDiff(time, obs[0].Time) < 1.0+float64(DTTOL) {
				break
			}
			isbs--
		}
	}
	return n
}

/* carrier-phase bias correction by ssr --------------------------------------*/
func CorrPhaseBiasSsr(obs []ObsD, n int, nav *Nav) {
	var freq float64

	for i := 0; i < n; i++ {
		for j := 0; j < NFREQ; j++ {
			code := obs[i].Code[j]

			if freq = Sat2Freq(int(obs[i].Sat), code, nav); freq == 0.0 {
				continue
			}

			/* correct phase bias (cyc) */
			obs[i].L[j] -= nav.Ssr[int(obs[i].Sat)-1].Pbias[code-1] * freq / CLIGHT
		}
	}
}

/* process positioning -------------------------------------------------------*/
func ProcPos(fp *os.File, popt *PrcOpt, sopt *SolOpt, mode int) {
	var (
		time                  Gtime
		sol                   Sol
		rtk                   Rtk
		obs                   [MAXOBS * 2]ObsD /* for rover and base */
		rb                    [3]float64
		i, nobs, n, solstatic int
		pri                   []int = []int{6, 1, 2, 3, 4, 5, 1, 6}
	)

	Trace(4, "procpos : mode=%d\n", mode)

	if popt.Mode == PMODE_STATIC || popt.Mode == PMODE_PPP_STATIC {
		solstatic = sopt.SolStatic
	} else {
		solstatic = 0
	}

	rtk.InitRtk(popt)
	rtcm_path = ""

	for {
		nobs = InputObs(obs[:], int(rtk.RtkSol.Stat), popt)
		if nobs < 0 {
			break
		}
		/* exclude satellites */
		for i, n = 0, 0; i < nobs; i++ {
			if (SatSys(int(obs[i].Sat), nil)&popt.NavSys) > 0 && popt.ExSats[int(obs[i].Sat)-1] != 1 {
				obs[n] = obs[i]
				n++
			}
		}
		if n <= 0 {
			continue
		}

		/* carrier-phase bias correction */
		if !strings.Contains(string(popt.PPPOpt[:]), "-ENA_FCB") {
			CorrPhaseBiasSsr(obs[:], n, &navs)
		}
		if rtk.RtkPos(obs[:], n, &navs) == 0 {
			continue
		}

		if mode == 0 { /* forward/backward */
			if solstatic == 0 {
				rtk.RtkSol.OutSol(fp, rtk.Rb[:], sopt)
			} else if time.Time == 0 || pri[rtk.RtkSol.Stat] <= pri[sol.Stat] {
				sol = rtk.RtkSol
				for i = 0; i < 3; i++ {
					rb[i] = rtk.Rb[i]
				}
				if time.Time == 0 || TimeDiff(rtk.RtkSol.Time, time) < 0.0 {
					time = rtk.RtkSol.Time
				}
			}
		} else if revs == 0 { /* combined-forward */
			if isolf >= nepoch {
				return
			}
			solf[isolf] = rtk.RtkSol
			for i = 0; i < 3; i++ {
				rbf[i+isolf*3] = rtk.Rb[i]
			}
			isolf++
		} else { /* combined-backward */
			if isolb >= nepoch {
				return
			}
			solb[isolb] = rtk.RtkSol
			for i = 0; i < 3; i++ {
				rbb[i+isolb*3] = rtk.Rb[i]
			}
			isolb++
		}
	}
	if mode == 0 && solstatic > 0 && time.Time != 0.0 {
		sol.Time = time
		sol.OutSol(fp, rb[:], sopt)
	}
	rtk.FreeRtk()
}

/* validation of combined solutions ------------------------------------------*/
func ValComb(solf *Sol, solb *Sol) int {
	var (
		dr, vari [3]float64
		tstr     string
	)

	Trace(4, "valcomb :\n")

	/* compare forward and backward solution */
	for i := 0; i < 3; i++ {
		dr[i] = solf.Rr[i] - solb.Rr[i]
		vari[i] = float64(solf.Qr[i] + solb.Qr[i])
	}
	for i := 0; i < 3; i++ {
		if dr[i]*dr[i] <= 16.0*vari[i] {
			continue /* ok if in 4-sigma */
		}

		Time2Str(solf.Time, &tstr, 2)
		Trace(2, "degrade fix to float: %s dr=%.3f %.3f %.3f std=%.3f %.3f %.3f\n",
			tstr, dr[0], dr[1], dr[2], SQRT(vari[0]), SQRT(vari[1]), SQRT(vari[2]))
		return 0
	}
	return 1
}

/* combine forward/backward solutions and output results ---------------------*/
func CombResult(fp *os.File, popt *PrcOpt, sopt *SolOpt) {
	var (
		time                      Gtime
		sols, sol                 Sol
		tt                        float64
		Qf, Qb, Qs                [9]float64
		rbs, rb, rr_f, rr_b, rr_s [3]float64
		i, j, k, solstatic        int
		pri                       []int = []int{0, 1, 2, 3, 4, 5, 1, 6}
	)

	Trace(4, "combres : isolf=%d isolb=%d\n", isolf, isolb)

	if popt.Mode == PMODE_STATIC || popt.Mode == PMODE_PPP_STATIC {
		solstatic = sopt.SolStatic
	} else {
		solstatic = 0

	}

	for i, j = 0, isolb-1; i < isolf && j >= 0; i, j = i+1, j-1 {
		tt = TimeDiff(solf[i].Time, solb[j].Time)
		switch {
		case tt < float64(-DTTOL):
			sols = solf[i]
			for k = 0; k < 3; k++ {
				rbs[k] = rbf[k+i*3]
			}
			j++
		case tt > float64(DTTOL):
			sols = solb[j]
			for k = 0; k < 3; k++ {
				rbs[k] = rbb[k+j*3]
			}
			i--
		case solf[i].Stat < solb[j].Stat:
			sols = solf[i]
			for k = 0; k < 3; k++ {
				rbs[k] = rbf[k+i*3]
			}
		case solf[i].Stat > solb[j].Stat:
			sols = solb[j]
			for k = 0; k < 3; k++ {
				rbs[k] = rbb[k+j*3]
			}
		default:
			sols = solf[i]
			sols.Time = TimeAdd(sols.Time, -tt/2.0)

			if (popt.Mode == PMODE_KINEMA || popt.Mode == PMODE_MOVEB) && sols.Stat == SOLQ_FIX {

				/* degrade fix to float if validation failed */
				if ValComb(&solf[i], &solb[j]) == 0 {
					sols.Stat = SOLQ_FLOAT
				}
			}
			for k = 0; k < 3; k++ {
				Qf[k+k*3] = float64(solf[i].Qr[k])
				Qb[k+k*3] = float64(solb[j].Qr[k])
			}
			Qf[1], Qf[3] = float64(solf[i].Qr[3]), float64(solf[i].Qr[3])
			Qf[5], Qf[7] = float64(solf[i].Qr[4]), float64(solf[i].Qr[4])
			Qf[2], Qf[6] = float64(solf[i].Qr[5]), float64(solf[i].Qr[5])
			Qb[1], Qb[3] = float64(solb[j].Qr[3]), float64(solb[j].Qr[3])
			Qb[5], Qb[7] = float64(solb[j].Qr[4]), float64(solb[j].Qr[4])
			Qb[2], Qb[6] = float64(solb[j].Qr[5]), float64(solb[j].Qr[5])

			if popt.Mode == int(PMODE_MOVEB) {
				for k = 0; k < 3; k++ {
					rr_f[k] = solf[i].Rr[k] - rbf[k+i*3]
				}
				for k = 0; k < 3; k++ {
					rr_b[k] = solb[j].Rr[k] - rbb[k+j*3]
				}
				if Smoother(rr_f[:], Qf[:], rr_b[:], Qb[:], 3, rr_s[:], Qs[:]) > 0 {
					continue
				}
				for k = 0; k < 3; k++ {
					sols.Rr[k] = rbs[k] + rr_s[k]
				}
			} else {
				if Smoother(solf[i].Rr[:], Qf[:], solb[j].Rr[:], Qb[:], 3, sols.Rr[:], Qs[:]) > 0 {
					continue
				}
			}
			sols.Qr[0] = float32(Qs[0])
			sols.Qr[1] = float32(Qs[4])
			sols.Qr[2] = float32(Qs[8])
			sols.Qr[3] = float32(Qs[1])
			sols.Qr[4] = float32(Qs[5])
			sols.Qr[5] = float32(Qs[2])

			/* smoother for velocity solution */
			if popt.Dynamics > 0 {
				for k = 0; k < 3; k++ {
					Qf[k+k*3] = float64(solf[i].Qv[k])
					Qb[k+k*3] = float64(solb[j].Qv[k])
				}
				Qf[1], Qf[3] = float64(solf[i].Qv[3]), float64(solf[i].Qv[3])
				Qf[5], Qf[7] = float64(solf[i].Qv[4]), float64(solf[i].Qv[4])
				Qf[2], Qf[6] = float64(solf[i].Qv[5]), float64(solf[i].Qv[5])
				Qb[1], Qb[3] = float64(solb[j].Qv[3]), float64(solb[j].Qv[3])
				Qb[5], Qb[7] = float64(solb[j].Qv[4]), float64(solb[j].Qv[4])
				Qb[2], Qb[6] = float64(solb[j].Qv[5]), float64(solb[j].Qv[5])
				if Smoother(solf[i].Rr[3:], Qf[:], solb[j].Rr[3:], Qb[:], 3, sols.Rr[3:], Qs[:]) > 0 {
					continue
				}
				sols.Qv[0] = float32(Qs[0])
				sols.Qv[1] = float32(Qs[4])
				sols.Qv[2] = float32(Qs[8])
				sols.Qv[3] = float32(Qs[1])
				sols.Qv[4] = float32(Qs[5])
				sols.Qv[5] = float32(Qs[2])
			}
		}
		if solstatic == 0 {
			sols.OutSol(fp, rbs[:], sopt)
		} else if time.Time == 0 || pri[sols.Stat] <= pri[sol.Stat] {
			sol = sols
			for k = 0; k < 3; k++ {
				rb[k] = rbs[k]
			}
			if time.Time == 0 || TimeDiff(sols.Time, time) < 0.0 {
				time = sols.Time
			}
		}
	}
	if solstatic > 0 && time.Time != 0.0 {
		sol.Time = time
		sol.OutSol(fp, rb[:], sopt)
	}
}

/* read prec ephemeris, sbas data, tec grid and open rtcm --------------------*/
func (nav *Nav) ReadPrecEph(infile []string, n int, prcopt *PrcOpt, sbs *Sbs) {
	var i int

	Trace(4, "readpreceph: n=%d\n", n)

	sbs.Msgs = nil

	/* read precise ephemeris files */
	for i = 0; i < n; i++ {
		if strings.Contains(infile[i], "%r") || strings.Contains(infile[i], "%b") {
			continue
		}
		nav.ReadSp3(infile[i], 0)
	}
	/* read precise clock files */
	for i = 0; i < n; i++ {
		if strings.Contains(infile[i], "%r") || strings.Contains(infile[i], "%b") {
			continue
		}
		nav.ReadRnxC(infile[i])
	}
	/* read sbas message files */
	for i = 0; i < n; i++ {
		if strings.Contains(infile[i], "%r") || strings.Contains(infile[i], "%b") {
			continue
		}
		sbsreadmsg(infile[i], prcopt.SbasSatSel, sbs)
	}

	/* set rtcm file and initialize rtcm struct */
	rtcm_file, rtcm_path = "", ""
	fp_rtcm = nil

	for i = 0; i < n; i++ {
		index := strings.LastIndex(infile[i], ".")
		if index > 0 && strings.EqualFold(infile[i][index:], ".rtcm3") {
			rtcm_file = infile[i]

			rtcm.InitRtcm()
			break
		}
	}
}

/* free prec ephemeris and sbas data -----------------------------------------*/
func (nav *Nav) FreePrecEph(sbs *Sbs) {

	Trace(4, "freepreceph:\n")

	nav.Peph = nil
	nav.Pclk = nil
	nav.Seph = nil
	sbs.Msgs = nil

	nav.Tec = nil

	if fp_rtcm != nil {
		fp_rtcm.Close()
	}

	if rtcm != nil {
		rtcm.FreeRtcm()
	}
}

/* read obs and nav data -----------------------------------------------------*/
func ReadObsNav(ts, te Gtime, ti float64, infile []string,
	index []int, n int, prcopt *PrcOpt, obs *Obs, nav *Nav, sta []Sta) int {
	var (
		i, j, ind, nobs int
		rcv             int = 1
	)

	Trace(4, "readobsnav: ts=%s n=%d\n", TimeStr(ts, 0), n)

	obs.Data = nil
	nav.Ephs = nil
	nav.Geph = nil
	nav.Seph = nil
	nepoch = 0

	for i = 0; i < n; i++ {
		if checkbrk("") > 0 {
			return 0
		}

		if index[i] != ind {
			if obs.N() > nobs {
				rcv++
			}
			ind = index[i]
			nobs = obs.N()
		}
		/* read rinex obs and nav file */
		rcvid := 1
		if rcv <= 1 {
			rcvid = 0
		}
		var staid *Sta = nil
		if rcv <= 2 {
			staid = &sta[rcv-1]
		}
		if ReadRnxT(infile[i], rcv, ts, te, ti, prcopt.RnxOpt[rcvid], obs, nav, staid) < 0 {
			checkbrk("error : insufficient memory")
			Trace(2, "insufficient memory\n")
			return 0
		}
	}
	if obs.N() <= 0 {
		checkbrk("error : no obs data")
		Trace(2, "\n")
		return 0
	}
	if nav.N() <= 0 && nav.Ng() <= 0 && nav.Ns() <= 0 {
		checkbrk("error : no nav data")
		Trace(2, "\n")
		return 0
	}
	/* sort observation data */
	nepoch = obs.SortObs()

	/* delete duplicated ephemeris */
	nav.UniqNav()

	/* set time span for progress display */
	if ts.Time == 0 || te.Time == 0 {
		for i = 0; i < obs.N(); i++ {
			if obs.Data[i].Rcv == 1 {
				break
			}
		}
		for j = obs.N() - 1; j >= 0; j-- {
			if obs.Data[j].Rcv == 1 {
				break
			}
		}
		if i < j {
			if ts.Time == 0 {
				ts = obs.Data[i].Time
			}
			if te.Time == 0 {
				te = obs.Data[j].Time
			}
			// settspan(ts, te)
		}
	}
	return 1
}

/* free obs and nav data -----------------------------------------------------*/
func FreeObsNav(obs *Obs, nav *Nav) {
	Trace(4, "freeobsnav:\n")

	obs.Data = nil
	nav.Ephs = nil
	nav.Geph = nil
	nav.Seph = nil
}

/* average of single position ------------------------------------------------*/
func AvePos(ra []float64, rcv int, obs *Obs, nav *Nav, opt *PrcOpt) int {
	var (
		data             [MAXOBS]ObsD
		ts               Gtime
		sol              Sol
		i, j, n, m, iobs int
		msg              string
	)

	Trace(4, "avepos: rcv=%d obs.n=%d\n", rcv, obs.N())

	for i = 0; i < 3; i++ {
		ra[i] = 0.0
	}

	for iobs, m = 0, obs.NextObsf(&iobs, rcv); m > 0; iobs, m = iobs+m, obs.NextObsf(&iobs, rcv) {

		for i, j = 0, 0; i < m && i < MAXOBS; i++ {
			data[j] = obs.Data[iobs+i]
			if (SatSys(int(data[j].Sat), nil)&opt.NavSys > 0) && opt.ExSats[int(data[j].Sat)-1] != 1 {
				j++
			}
		}
		if j <= 0 || ScreenTime(data[0].Time, ts, ts, 1.0) == 0 {
			continue /* only 1 hz */
		}

		if PntPos(data[:], j, nav, opt, &sol, nil, nil, &msg) == 0 {
			continue
		}

		for i = 0; i < 3; i++ {
			ra[i] += sol.Rr[i]
		}
		n++
	}
	if n <= 0 {
		Trace(2, "no average of base station position\n")
		return 0
	}
	for i = 0; i < 3; i++ {
		ra[i] /= float64(n)
	}
	return 1
}

/* station position from file ------------------------------------------------*/
func GetStationPos(file, name string, r []float64) int {
	var (
		fp    *os.File
		pos   [3]float64
		sname string
		err   error
	)
	Trace(4, "getstapos: file=%s name=%s\n", file, name)

	fp, _ = os.OpenFile(file, os.O_RDONLY, 0666)
	if err != nil {
		Trace(2, "station position file open error: %s\n", file)
		return 0
	}
	defer fp.Close()

	rd := bufio.NewReader(fp)
	for {
		buff, err := rd.ReadString('\n')
		if len(buff) == 0 || err != nil {
			break
		}
		index := strings.Index(string(buff), "%")
		buff = buff[:index]
		if n, _ := fmt.Sscanf(string(buff), "%f %f %f %s", &pos[0], &pos[1], &pos[2], &sname); n < 4 {
			continue
		}
		if strings.EqualFold(name, sname) {
			pos[0] *= D2R
			pos[1] *= D2R
			Pos2Ecef(pos[:], r)
			return 1

		}
	}

	Trace(5, "no station position: %s %s\n", name, file)
	return 0
}

/* antenna phase center position ---------------------------------------------*/
func AntPos(opt *PrcOpt, rcvno int, obs *Obs, nav *Nav,
	sta []Sta, posfile string) int {

	var (
		del, pos, dr [3]float64
		i, postype   int = 0, opt.RefPos
		name         string
		rr           []float64 = opt.Rb[:]
		rcvnoid                = 1
	)
	if rcvno == 1 {
		postype = opt.RovPos
	}

	if rcvno == 1 {
		rr = opt.Ru[:]
	}

	Trace(3, "antpos  : rcvno=%d\n", rcvno)
	if rcvno == 1 {
		rcvnoid = 0
	}

	switch postype {
	case POSOPT_SINGLE: /* average of single position */
		if AvePos(rr, rcvno, obs, nav, opt) == 0 {
			ShowMsg_Ptr("error : station pos computation")
			return 0
		}
	case POSOPT_FILE: /* read from position file */

		name = stas[rcvnoid].Name
		if GetStationPos(posfile, name, rr) == 0 {
			ShowMsg_Ptr("error : no position of %s in %s", name, posfile)
			return 0
		}
	case POSOPT_RINEX: /* get from rinex header */
		if Norm(stas[rcvnoid].Pos[:], 3) <= 0.0 {
			ShowMsg_Ptr("error : no position in rinex header")
			Trace(3, "no position position in rinex header\n")
			return 0
		}
		/* antenna delta */
		if stas[rcvnoid].DelType == 0 { /* enu */
			for i = 0; i < 3; i++ {
				del[i] = stas[rcvnoid].Del[i]
			}
			del[2] += stas[rcvnoid].Hgt
			Ecef2Pos(stas[rcvnoid].Pos[:], pos[:])
			Enu2Ecef(pos[:], del[:], dr[:])
		} else { /* xyz */
			for i = 0; i < 3; i++ {
				dr[i] = stas[rcvnoid].Del[i]
			}
		}
		for i = 0; i < 3; i++ {
			rr[i] = stas[rcvnoid].Pos[i] + dr[i]
		}
	}
	return 1
}

/* open procssing session ----------------------------------------------------*/
func OpenSession(popt *PrcOpt, sopt *SolOpt, fopt *FilOpt, nav *Nav, pcvs, pcvr *Pcvs) int {
	Trace(4, "openses :\n")

	/* read satellite antenna parameters */
	if len(fopt.SatAntPara) > 0 && ReadPcv(fopt.SatAntPara, pcvs) == 0 {
		ShowMsg_Ptr("error : no sat ant pcv in %s", fopt.SatAntPara)
		Trace(3, "sat antenna pcv read error: %s\n", fopt.SatAntPara)
		return 0
	}
	/* read receiver antenna parameters */
	if len(fopt.RcvAntPara) > 0 && ReadPcv(fopt.RcvAntPara, pcvr) == 0 {
		ShowMsg_Ptr("error : no rec ant pcv in %s", fopt.RcvAntPara)
		Trace(3, "rec antenna pcv read error: %s\n", fopt.RcvAntPara)
		return 0
	}
	/* open geoid data */
	if sopt.Geoid > 0 && len(fopt.Geoid) > 0 {
		if OpenGeoid(sopt.Geoid, fopt.Geoid) == 0 {
			ShowMsg_Ptr("error : no geoid data %s", fopt.Geoid)
			Trace(3, "no geoid data %s\n", fopt.Geoid)
		}
	}
	return 1
}

/* close procssing session ---------------------------------------------------*/
func CloseSession(nav *Nav, pcvs, pcvr *Pcvs) {
	Trace(4, "closeses:\n")

	/* free antenna parameters */
	pcvs.Pcv = nil
	pcvr.Pcv = nil

	/* close geoid data */
	CloseGeoid()

	/* free erp data */
	nav.Erp.Data = nil
	// nav.Erp.N, nav.Erp.Nmax = 0, 0

	/* close solution statistics and debug trace */
	RtkCloseStat()
	TraceClose()
}

/* set antenna parameters ----------------------------------------------------*/
func SetPcv(time Gtime, popt *PrcOpt, nav *Nav, pcvs, pcvr *Pcvs, sta []Sta) {
	var (
		pcv0     Pcv
		pcv      *Pcv
		pos, del [3]float64
		i, j     int
		id       string
	)
	mode := PMODE_DGPS <= popt.Mode && popt.Mode <= PMODE_FIXED

	/* set satellite antenna parameters */
	for i = 0; i < MAXSAT; i++ {
		nav.Pcvs[i] = pcv0
		if SatSys(i+1, nil)&popt.NavSys == 0 {
			continue
		}
		if pcv = SearchPcv(i+1, "", time, pcvs); pcv == nil {
			SatNo2Id(i+1, &id)
			Trace(3, "no satellite antenna pcv: %s\n", id)
			continue
		}
		nav.Pcvs[i] = *pcv
	}
	imode := 1
	if mode {
		imode = 2
	}
	for i = 0; i < imode; i++ {
		popt.Pcvr[i] = pcv0
		if strings.Compare(string(popt.AntType[i][:]), "*") == 0 { /* set by station parameters */
			popt.AntType[i] = string(sta[i].AntDes[:])
			if sta[i].DelType == 1 { /* xyz */
				if Norm(sta[i].Pos[:], 3) > 0.0 {
					Ecef2Pos(sta[i].Pos[:], pos[:])
					Ecef2Enu(pos[:], sta[i].Del[:], del[:])
					for j = 0; j < 3; j++ {
						popt.AntDel[i][j] = float64(del[j])
					}
				}
			} else { /* enu */
				for j = 0; j < 3; j++ {
					popt.AntDel[i][j] = float64(stas[i].Del[j])
				}
			}
		}
		if pcv = SearchPcv(0, popt.AntType[i], time, pcvr); pcv == nil {
			Trace(3, "no receiver antenna pcv: %s\n", popt.AntType[i])
			popt.AntType[i] = ""
			continue
		}
		popt.AntType[i] = pcv.Type
		popt.Pcvr[i] = *pcv
	}
}

/* read ocean tide loading parameters ----------------------------------------*/
func ReadOtl(popt *PrcOpt, file string, sta []Sta) {
	var imode int = 1
	if PMODE_DGPS <= popt.Mode && popt.Mode <= PMODE_FIXED {
		imode = 2
	}

	for i := 0; i < imode; i++ {
		ReadBlq(file, sta[i].Name, popt.Odisp[i][:])
	}
}

/* write header to output file -----------------------------------------------*/
func OutPostHead(outfile string, infile []string, n int, popt *PrcOpt, sopt *SolOpt) int {
	var (
		fp  *os.File = os.Stdout
		err error
	)

	Trace(4, "outhead: outfile=%s n=%d\n", outfile, n)

	if len(outfile) > 0 {
		CreateDir(outfile)
		fp, _ = os.OpenFile(outfile, os.O_CREATE|os.O_TRUNC|os.O_WRONLY, os.ModeAppend|os.ModePerm)
		if err != nil {
			ShowMsg_Ptr("error : open output file %s", outfile)
			return 0
		}
		defer fp.Close()
	}
	/* output header */
	OutHeader(fp, infile, n, popt, sopt)

	return 1
}

/* open output file for append -----------------------------------------------*/
func OpenPostFile(outfile string) *os.File {
	Trace(4, "openfile: outfile=%s\n", outfile)
	if len(outfile) == 0 {
		return os.Stdout
	} else {
		fp, _ := os.OpenFile(outfile, os.O_WRONLY|os.O_CREATE|os.O_APPEND, 0666)
		return fp
	}
}

/* execute processing session ------------------------------------------------*/
func execses(ts, te Gtime, ti float64, popt *PrcOpt,
	sopt *SolOpt, fopt *FilOpt, flag int,
	infile []string, index []int, n int, outfile string) int {
	var (
		fp                        *os.File
		tracefile, statfile, path string
		id                        int
	)
	popt_ := *popt
	Trace(4, "execses : n=%d outfile=%s\n", n, outfile)

	/* open debug trace */
	if flag > 0 && sopt.Trace > 0 {
		if len(outfile) > 0 {
			tracefile = outfile + ".trace"

		} else {
			tracefile = fopt.Trace
		}
		TraceClose()
		TraceOpen(tracefile)
		TraceLevel(sopt.Trace)
	}
	/* read ionosphere data file */
	if id = strings.LastIndex(fopt.Iono, "."); len(fopt.Iono) > 0 && id > 0 {
		if len(fopt.Iono[id:]) == 4 && (fopt.Iono[id+3] == 'i' || fopt.Iono[id+3] == 'I') {
			RepPath(fopt.Iono, &path, ts, "", "")
			navs.ReadTec(path, 1)
		}
	}

	/* read erp data */
	if len(fopt.Eop) > 0 {
		navs.Erp.Data = nil
		// navs.Erp.N, navs.Erp.Nmax = 0, 0
		RepPath(fopt.Eop, &path, ts, "", "")
		if ReadErp(path, &navs.Erp) == 0 {
			ShowMsg_Ptr("error : no erp data %s", path)
			Trace(3, "no erp data %s\n", path)
		}
	}

	/* read obs and nav data */
	if ReadObsNav(ts, te, ti, infile, index, n, &popt_, &obss, &navs, stas[:]) == 0 {
		return 0
	}

	/* read dcb parameters */
	if len(fopt.Dcb) > 0 {
		RepPath(fopt.Dcb, &path, ts, "", "")
		navs.ReadDcb(path, stas[:])
	}
	/* set antenna paramters */
	if popt_.Mode != PMODE_SINGLE {
		if obss.N() > 0 {
			SetPcv(obss.Data[0].Time, &popt_, &navs, &pcvss, &pcvsr, stas[:])
		} else {
			SetPcv(TimeGet(), &popt_, &navs, &pcvss, &pcvsr, stas[:])
		}
	}
	/* read ocean tide loading parameters */
	if popt_.Mode > PMODE_SINGLE && len(fopt.Blq) > 0 {
		ReadOtl(&popt_, fopt.Blq, stas[:])
	}
	/* rover/reference fixed position */
	if popt_.Mode == PMODE_FIXED {
		if AntPos(&popt_, 1, &obss, &navs, stas[:], fopt.StaPos) == 0 {
			FreeObsNav(&obss, &navs)
			return 0
		}
	} else if PMODE_DGPS <= popt_.Mode && popt_.Mode <= PMODE_STATIC {
		if AntPos(&popt_, 2, &obss, &navs, stas[:], fopt.StaPos) == 0 {
			FreeObsNav(&obss, &navs)
			return 0
		}
	}
	/* open solution statistics */
	if flag > 0 && sopt.SStat > 0 {
		statfile = outfile + ".stat"
		RtkCloseStat()
		RtkOpenStat(statfile, sopt.SStat)
	}
	/* write header to output file */
	if flag > 0 && OutPostHead(outfile, infile, n, &popt_, sopt) == 0 {
		FreeObsNav(&obss, &navs)
		return 0
	}
	iobsu, iobsr, isbs, revs, aborts = 0, 0, 0, 0, 0

	switch {
	case popt_.Mode == PMODE_SINGLE || popt_.SolType == 0:
		if fp = OpenPostFile(outfile); fp != nil {
			ProcPos(fp, &popt_, sopt, 0) /* forward */
			fp.Close()
		}
	case popt_.SolType == 1:
		if fp = OpenPostFile(outfile); fp != nil {
			revs = 1
			iobsu, iobsr, isbs = obss.N()-1, obss.N()-1, sbss.N()-1
			ProcPos(fp, &popt_, sopt, 0) /* backward */
			fp.Close()
		}
	default: /* combined */
		solf = make([]Sol, nepoch)
		solb = make([]Sol, nepoch)
		rbf = make([]float64, nepoch*3)
		rbb = make([]float64, nepoch*3)

		isolf, isolb = 0, 0
		ProcPos(nil, &popt_, sopt, 1) /* forward */
		revs = 1
		iobsu, iobsr, isbs = obss.N()-1, sbss.N()-1, sbss.N()-1
		ProcPos(nil, &popt_, sopt, 1) /* backward */

		/* combine forward/backward solutions */
		fp = OpenPostFile(outfile)
		if aborts == 0 && fp != nil {
			CombResult(fp, &popt_, sopt)
			fp.Close()
		}
	}
	/* free obs and nav data */
	FreeObsNav(&obss, &navs)

	return aborts
}

/* execute processing session for each rover ---------------------------------*/
func execses_r(ts, te Gtime, ti float64, popt *PrcOpt, sopt *SolOpt, fopt *FilOpt, flag int,
	infile []string, index []int, n int, outfile string, rov string) int {
	var (
		t0             Gtime
		i, stat        int
		ifile          []string
		rov_, s, ofile string
	)

	Trace(4, "execses_r: n=%d outfile=%s\n", n, outfile)

	for i = 0; i < n; i++ {
		if strings.Contains(infile[i], "%r") {
			break
		}
	}

	if i < n { /* include rover keywords */

		rov_ = rov

		ifile = make([]string, n)
		p := strings.Fields(rov_)
		for _, q := range p {
			proc_rov = q
			if ts.Time > 0 {
				Time2Str(ts, &s, 0)
			} else {
				s = ""
			}
			if checkbrk("reading    : %s", s) > 0 {
				stat = 1
				break
			}
			for i = 0; i < n; i++ {
				RepPath(infile[i], &ifile[i], t0, q, "")
			}
			RepPath(outfile, &ofile, t0, q, "")

			/* execute processing session */
			stat = execses(ts, te, ti, popt, sopt, fopt, flag, ifile, index, n, ofile)
			if stat == 1 {
				break
			}
		}

	} else {
		/* execute processing session */
		stat = execses(ts, te, ti, popt, sopt, fopt, flag, infile, index, n, outfile)
	}
	return stat
}

/* execute processing session for each rover ---------------------------------*/
func execses_b(ts, te Gtime, ti float64, popt *PrcOpt, sopt *SolOpt, fopt *FilOpt, flag int,
	infile []string, index []int, n int, outfile string, rov, base string) int {
	var (
		t0              Gtime
		i, stat         int
		ifile           []string
		base_, s, ofile string
	)

	Trace(4, "execses_b: n=%d outfile=%s\n", n, outfile)

	/* read prec ephemeris and sbas data */
	navs.ReadPrecEph(infile, n, popt, &sbss)

	for i = 0; i < n; i++ {
		if strings.Contains(infile[i], "%b") {
			break
		}
	}

	if i < n { /* include rover keywords */

		base_ = base

		ifile = make([]string, MAXINFILE)
		p := strings.Fields(base_)
		for _, q := range p {
			proc_base = q
			if ts.Time > 0 {
				Time2Str(ts, &s, 0)
			} else {
				s = ""
			}
			if checkbrk("reading    : %s", s) > 0 {
				stat = 1
				break
			}
			for i = 0; i < n; i++ {
				RepPath(infile[i], &ifile[i], t0, "", q)
			}
			RepPath(outfile, &ofile, t0, "", q)

			/* execute processing session */
			stat = execses_r(ts, te, ti, popt, sopt, fopt, flag, ifile, index, n, ofile, rov)
			if stat == 1 {
				break
			}
		}

	} else {
		/* execute processing session */
		stat = execses_r(ts, te, ti, popt, sopt, fopt, flag, infile, index, n, outfile, rov)
	}
	navs.FreePrecEph(&sbss)
	return stat
}

/* post-processing positioning -------------------------------------------------
* post-processing positioning
* args   : gtime_t ts       I   processing start time (ts.time==0: no limit)
*        : gtime_t te       I   processing end time   (te.time==0: no limit)
*          double ti        I   processing interval  (s) (0:all)
*          double tu        I   processing unit time (s) (0:all)
*          prcopt_t *popt   I   processing options
*          solopt_t *sopt   I   solution options
*          filopt_t *fopt   I   file options
*          char   **infile  I   input files (see below)
*          int    n         I   number of input files
*          char   *outfile  I   output file ("":stdout, see below)
*          char   *rov      I   rover id list        (separated by " ")
*          char   *base     I   base station id list (separated by " ")
* return : status (0:ok,0>:error,1:aborted)
* notes  : input files should contain observation data, navigation data, precise
*          ephemeris/clock (optional), sbas log file (optional), ssr message
*          log file (optional) and tec grid file (optional). only the first
*          observation data file in the input files is recognized as the rover
*          data.
*
*          the type of an input file is recognized by the file extention as ]
*          follows:
*              .sp3,.SP3,.eph*,.EPH*: precise ephemeris (sp3c)
*              .sbs,.SBS,.ems,.EMS  : sbas message log files (rtklib or ems)
*              .rtcm3,.RTCM3        : ssr message log files (rtcm3)
*              .*i,.*I              : tec grid files (ionex)
*              others               : rinex obs, nav, gnav, hnav, qnav or clock
*
*          inputs files can include wild-cards (*). if an file includes
*          wild-cards, the wild-card expanded multiple files are used.
*
*          inputs files can include keywords. if an file includes keywords,
*          the keywords are replaced by date, time, rover id and base station
*          id and multiple session analyses run. refer reppath() for the
*          keywords.
*
*          the output file can also include keywords. if the output file does
*          not include keywords. the results of all multiple session analyses
*          are output to a single output file.
*
*          ssr corrections are valid only for forward estimation.
*-----------------------------------------------------------------------------*/
func PostPos(ts, te Gtime, ti, tu float64, popt *PrcOpt, sopt *SolOpt,
	fopt *FilOpt, infile []string, n int, outfile *string, rov, base string) int {
	var (
		tts, tte, ttte          Gtime
		tunit, tss              float64
		i, j, k, nf, stat, week int
		flag                    int      = 1
		index                   []int    = make([]int, MAXINFILE)
		ifile                   []string = make([]string, MAXINFILE)
		ofile                   string
	)
	Trace(4, "postpos : ti=%.0f tu=%.0f n=%d outfile=%s\n", ti, tu, n, *outfile)

	/* open processing session */
	if OpenSession(popt, sopt, fopt, &navs, &pcvss, &pcvsr) == 0 {
		return -1
	}

	if ts.Time != 0 && te.Time != 0 && tu >= 0.0 {
		if TimeDiff(te, ts) < 0.0 {
			ShowMsg_Ptr("error : no period")
			CloseSession(&navs, &pcvss, &pcvsr)
			return 0
		}

		if tu == 0.0 || tu > 86400.0*float64(MAXPRCDAYS) {
			tu = 86400.0 * float64(MAXPRCDAYS)
		}
		//		settspan(ts, te)
		tunit = 86400.0
		if tu < 86400.0 {
			tunit = tu
		}

		tss = tunit * math.Floor(Time2GpsT(ts, &week)/tunit)

		for i = 0; ; i++ { /* for each periods */
			tts = GpsT2Time(week, tss+float64(i)*tu)
			tte = TimeAdd(tts, tu-float64(DTTOL))
			if TimeDiff(tts, te) > 0.0 {
				break
			}
			if TimeDiff(tts, ts) < 0.0 {
				tts = ts
			}
			if TimeDiff(tte, te) > 0.0 {
				tte = te
			}

			proc_rov = ""
			proc_base = ""
			if checkbrk("reading    : %s", TimeStr(tts, 0)) > 0 {
				stat = 1
				break
			}
			for j, k, nf = 0, 0, 0; j < n; j++ {

				id := strings.LastIndex(infile[j], ".")
				if strings.EqualFold(infile[j][id:], ".rtcm3") {
					ifile[nf] = infile[j]
					nf++
				} else {
					/* include next day precise ephemeris or rinex brdc nav */
					ttte = tte
					if strings.EqualFold(infile[j][id:], ".sp3") || strings.EqualFold(infile[j][id:], ".eph") {
						ttte = TimeAdd(ttte, 3600.0)
					} else if strings.Contains(infile[j], "brdc") {
						ttte = TimeAdd(ttte, 7200.0)

					}
					nf += RepPaths(infile[j], ifile[nf:], MAXINFILE-nf, tts, ttte, "", "")
				}

				for k < nf {
					index[k] = j
					k++
				}

				if nf >= MAXINFILE {
					Trace(3, "too many input files. trancated\n")
					break
				}
			}
			if RepPath(*outfile, &ofile, tts, "", "") == 0 && i > 0 {
				flag = 0
			}

			/* execute processing session */
			stat = execses_b(tts, tte, ti, popt, sopt, fopt, flag, ifile, index, nf, ofile,
				rov, base)

			if stat == 1 {
				break
			}
		}

	} else if ts.Time != 0 {
		for i = 0; i < n && i < MAXINFILE; i++ {

			RepPath(infile[i], &ifile[i], ts, "", "")
			index[i] = i
		}
		RepPath(*outfile, &ofile, ts, "", "")

		/* execute processing session */
		stat = execses_b(ts, te, ti, popt, sopt, fopt, 1, ifile, index, n, ofile, rov, base)
	} else {
		for i = 0; i < n; i++ {
			index[i] = i
		}

		/* execute processing session */
		stat = execses_b(ts, te, ti, popt, sopt, fopt, 1, infile, index, n, *outfile, rov, base)
	}
	/* close processing session */
	CloseSession(&navs, &pcvss, &pcvsr)

	return stat
}
