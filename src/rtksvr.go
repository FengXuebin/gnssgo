/*------------------------------------------------------------------------------
* rtksvr.c : rtk server functions
*
*          Copyright (C) 2007-2020 by T.TAKASU, All rights reserved.
*
* options : -DWIN32    use WIN32 API
*
* version : $Revision:$ $Date:$
* history : 2009/01/07  1.0  new
*           2009/06/02  1.1  support glonass
*           2010/07/25  1.2  support correction input/log stream
*                            supoort online change of output/log streams
*                            supoort monitor stream
*                            added api:
*                                rtksvropenstr(),rtksvrclosestr()
*                            changed api:
*                                rtksvrstart()
*           2010/08/25  1.3  fix problem of ephemeris time inversion (2.4.0_p6)
*           2010/09/08  1.4  fix problem of ephemeris and ssr squence upset
*                            (2.4.0_p8)
*           2011/01/10  1.5  change api: rtksvrstart(),rtksvrostat()
*           2011/06/21  1.6  fix ephemeris handover problem
*           2012/05/14  1.7  fix bugs
*           2013/03/28  1.8  fix problem on lack of glonass freq number in raw
*                            fix problem on ephemeris with inverted toe
*                            add api rtksvrfree()
*           2014/06/28  1.9  fix probram on ephemeris update of beidou
*           2015/04/29  1.10 fix probram on ssr orbit/clock inconsistency
*           2015/07/31  1.11 add phase bias (fcb) correction
*           2015/12/05  1.12 support opt.pppopt=-DIS_FCB
*           2016/07/01  1.13 support averaging single pos as base position
*           2016/07/31  1.14 fix bug on ion/utc parameters input
*           2016/08/20  1.15 support api change of sendnmea()
*           2016/09/18  1.16 fix server-crash with server-cycle > 1000
*           2016/09/20  1.17 change api rtksvrstart()
*           2016/10/01  1.18 change api rtksvrstart()
*           2016/10/04  1.19 fix problem to send nmea of single solution
*           2016/10/09  1.20 add reset-and-single-sol mode for nmea-request
*           2017/04/11  1.21 add rtkfree() in rtksvrfree()
*           2020/11/30  1.22 add initializing svr.nav in rtksvrinit()
*                            allocate double size ephemeris in rtksvrinit()
*                            handle multiple ephemeris sets in updatesvr()
*                            use API sat2freq() to get carrier frequency
*                            use integer types in stdint.h
*		    2022/05/31 1.0  rewrite rtksvr.c with golang by fxb
*-----------------------------------------------------------------------------*/
package gnssgo

import (
	"fmt"
	"strings"
)

const MIN_INT_RESET int = 30000 /* mininum interval of reset command (ms) */
type RBSol struct {
	Rb  [6]float64
	Sol Sol
}

var ObsChannel chan ObsD
var RbSolChannel chan RBSol

/* write solution header to output stream ------------------------------------*/
func writesolhead(stream *Stream, solopt *SolOpt) {
	var buff string

	n := OutSolHeader(&buff, solopt)
	stream.StreamWrite([]byte(buff), n)
}

/* save output buffer --------------------------------------------------------*/
func (svr *RtkSvr) SaveOutBuf(buff []uint8, n, index int) {
	svr.RtkSvrLock()

	if n < svr.BuffSize-svr.Nsb[index] {
		n = svr.BuffSize - svr.Nsb[index]
	}
	if buff == nil || len(buff) < n {
		svr.RtkSvrUnlock()
		return
	}
	copy(svr.SBuf[index][svr.Nsb[index]:], buff[:n])
	svr.Nsb[index] += n

	svr.RtkSvrUnlock()
}

/* write solution to output stream -------------------------------------------*/
func (svr *RtkSvr) WriteSol(index int) {
	var (
		solopt SolOpt = DefaultSolOpt()
		buff   string
		i, n   int
	)

	Tracet(4, "writesol: index=%d\n", index)

	for i = 0; i < 2; i++ {

		if svr.Solopt[i].Posf == int(SOLF_STAT) {

			/* output solution status */
			svr.RtkSvrLock()
			n = svr.RtkCtrl.RtkOutStat(&buff)
			svr.RtkSvrUnlock()
		} else {
			/* output solution */
			n = svr.RtkCtrl.RtkSol.OutSols(&buff, svr.RtkCtrl.Rb[:], &svr.Solopt[i])
		}
		svr.Stream[i+3].StreamWrite([]byte(buff), n)

		/* save output buffer */
		svr.SaveOutBuf([]byte(buff), n, i)

		/* output extended solution */
		n = svr.RtkCtrl.RtkSol.OutSolExs(&buff, svr.RtkCtrl.Ssat[:], &svr.Solopt[i])
		svr.Stream[i+3].StreamWrite([]byte(buff), n)

		/* save output buffer */
		svr.SaveOutBuf([]byte(buff), n, i)
	}
	/* output solution to monitor port */
	if svr.Monitor != nil {
		n = svr.RtkCtrl.RtkSol.OutSols(&buff, svr.RtkCtrl.Rb[:], &solopt)
		svr.Monitor.StreamWrite([]byte(buff), n)
	}
	/* save solution buffer */
	if svr.NoSol < MAXSOLBUF {
		svr.RtkSvrLock()
		svr.SolBuf[svr.NoSol] = svr.RtkCtrl.RtkSol
		svr.NoSol++
		svr.RtkSvrUnlock()
	}
}

/* update glonass frequency channel number in raw data struct ----------------*/
func (svr *RtkSvr) UpdateGloFcn() {
	var i, j, sat, frq int

	for i = 0; i < MAXPRNGLO; i++ {
		sat = SatNo(SYS_GLO, i+1)

		for j, frq = 0, -999; j < 3; j++ {
			if svr.RawCtrl[j].NavData.Geph[i].Sat != sat {
				continue
			}
			frq = svr.RawCtrl[j].NavData.Geph[i].Frq
		}
		if frq < -7 || frq > 6 {
			continue
		}

		for j = 0; j < 3; j++ {
			if svr.RawCtrl[j].NavData.Geph[i].Sat == sat {
				continue
			}
			svr.RawCtrl[j].NavData.Geph[i].Sat = sat
			svr.RawCtrl[j].NavData.Geph[i].Frq = frq
		}
	}
}

/* update observation data ---------------------------------------------------*/
func (svr *RtkSvr) UpdateObs(obs *Obs, index, iobs int) {
	var i, n, sat, sys int

	svr.ObsData[index][iobs].Data = nil
	if iobs < MAXOBSBUF {
		for i = 0; i < obs.N(); i++ {
			sat = obs.Data[i].Sat
			sys = SatSys(sat, nil)
			if svr.RtkCtrl.Opt.ExSats[sat-1] == 1 || (sys&svr.RtkCtrl.Opt.NavSys == 0) {
				continue
			}
			svr.ObsData[index][iobs].AddObsData(&obs.Data[i])
			//svr.ObsData[index][iobs].Data[n] = obs.Data[i]
			n = svr.ObsData[index][iobs].N()
			svr.ObsData[index][iobs].Data[n-1].Rcv = index + 1
			//			n++
		}
		//		svr.ObsData[index][iobs].N = n
		svr.ObsData[index][iobs].SortObs()
	}
	svr.InputMsg[index][0]++
}

/* update ephemeris ----------------------------------------------------------*/
func (svr *RtkSvr) UpdateEph(nav *Nav, ephsat, ephset, index int) {
	var (
		eph1, eph2, eph3    *Eph
		geph1, geph2, geph3 *GEph
		prn                 int
	)

	if SatSys(ephsat, &prn) != SYS_GLO {
		if svr.NavSel == 0 || svr.NavSel == index+1 {
			/* svr.nav.eph={current_set1,current_set2,prev_set1,prev_set2} */
			eph1 = &nav.Ephs[ephsat-1+MAXSAT*ephset]             /* received */
			eph2 = &svr.NavData.Ephs[ephsat-1+MAXSAT*ephset]     /* current */
			eph3 = &svr.NavData.Ephs[ephsat-1+MAXSAT*(2+ephset)] /* previous */
			if eph2.Ttr.Time == 0 ||
				(eph1.Iode != eph3.Iode && eph1.Iode != eph2.Iode) ||
				(TimeDiff(eph1.Toe, eph3.Toe) != 0.0 &&
					TimeDiff(eph1.Toe, eph2.Toe) != 0.0) ||
				(TimeDiff(eph1.Toc, eph3.Toc) != 0.0 &&
					TimeDiff(eph1.Toc, eph2.Toc) != 0.0) {
				*eph3 = *eph2 /* current .previous */
				*eph2 = *eph1 /* received.current */
			}
		}
		svr.InputMsg[index][1]++
	} else {
		if svr.NavSel == 0 || svr.NavSel == index+1 {
			geph1 = &nav.Geph[prn-1]
			geph2 = &svr.NavData.Geph[prn-1]
			geph3 = &svr.NavData.Geph[prn-1+MAXPRNGLO]
			if geph2.Tof.Time == 0 ||
				(geph1.Iode != geph3.Iode && geph1.Iode != geph2.Iode) {
				*geph3 = *geph2
				*geph2 = *geph1
				svr.UpdateGloFcn()
			}
		}
		svr.InputMsg[index][6]++
	}
}

/* update sbas message -------------------------------------------------------*/
func (svr *RtkSvr) UpdateSbs(sbsmsg *SbsMsg, index int) {
	var i, sbssat int = 0, svr.RtkCtrl.Opt.SbasSatSel

	if sbsmsg != nil && (sbssat == int(sbsmsg.Prn) || sbssat == 0) {
		sbsmsg.Rcv = uint8(index) + 1
		if svr.NoSbs < MAXSBSMSG {
			svr.SbsMsg[svr.NoSbs] = *sbsmsg
			svr.NoSbs++
		} else {
			for i = 0; i < MAXSBSMSG-1; i++ {
				svr.SbsMsg[i] = svr.SbsMsg[i+1]
			}
			svr.SbsMsg[i] = *sbsmsg
		}
		SbsUpdateCorr(sbsmsg, &svr.NavData)
	}
	svr.InputMsg[index][3]++
}

/* update ion/utc parameters -------------------------------------------------*/
func (svr *RtkSvr) UpdateIonUtc(nav *Nav, index int) {
	if svr.NavSel == 0 || svr.NavSel == index+1 {
		MatCpy(svr.NavData.Utc_gps[:], nav.Utc_gps[:], 8, 1)
		MatCpy(svr.NavData.Utc_glo[:], nav.Utc_glo[:], 8, 1)
		MatCpy(svr.NavData.Utc_gal[:], nav.Utc_gal[:], 8, 1)
		MatCpy(svr.NavData.Utc_qzs[:], nav.Utc_qzs[:], 8, 1)
		MatCpy(svr.NavData.Utc_cmp[:], nav.Utc_cmp[:], 8, 1)
		MatCpy(svr.NavData.Utc_irn[:], nav.Utc_irn[:], 9, 1)
		MatCpy(svr.NavData.Utc_sbs[:], nav.Utc_sbs[:], 4, 1)
		MatCpy(svr.NavData.Ion_gps[:], nav.Ion_gps[:], 8, 1)
		MatCpy(svr.NavData.Ion_gal[:], nav.Ion_gal[:], 4, 1)
		MatCpy(svr.NavData.Ion_qzs[:], nav.Ion_qzs[:], 8, 1)
		MatCpy(svr.NavData.Ion_cmp[:], nav.Ion_cmp[:], 8, 1)
		MatCpy(svr.NavData.Ion_irn[:], nav.Ion_irn[:], 8, 1)
	}
	svr.InputMsg[index][2]++
}

/* update antenna position ---------------------------------------------------*/
func (svr *RtkSvr) UpdateAntPos(index int) {
	var (
		sta          *Sta
		pos, del, dr [3]float64
		i            int
	)

	if svr.RtkCtrl.Opt.RefPos == POSOPT_RTCM && index == 1 {
		if svr.Format[1] == STRFMT_RTCM2 || svr.Format[1] == STRFMT_RTCM3 {
			sta = &svr.RtcmCtrl[1].StaPara
		} else {
			sta = &svr.RawCtrl[1].StaData
		}
		/* update base station position */
		for i = 0; i < 3; i++ {
			svr.RtkCtrl.Rb[i] = sta.Pos[i]
		}
		/* antenna delta */
		Ecef2Pos(svr.RtkCtrl.Rb[:], pos[:])
		if sta.DelType > 0 { /* xyz */
			del[2] = sta.Hgt
			Enu2Ecef(pos[:], del[:], dr[:])
			for i = 0; i < 3; i++ {
				svr.RtkCtrl.Rb[i] += sta.Del[i] + dr[i]
			}
		} else { /* enu */
			Enu2Ecef(pos[:], sta.Del[:], dr[:])
			for i = 0; i < 3; i++ {
				svr.RtkCtrl.Rb[i] += dr[i]
			}
		}
	}
	svr.InputMsg[index][4]++
}

/* update ssr corrections ----------------------------------------------------*/
func (svr *RtkSvr) UpdateSsr(index int) {
	var i, sys, prn, iode int

	for i = 0; i < MAXSAT; i++ {
		if svr.RtcmCtrl[index].Ssr[i].Update == 0 {
			continue
		}

		/* check consistency between iods of orbit and clock */
		if svr.RtcmCtrl[index].Ssr[i].Iod[0] != svr.RtcmCtrl[index].Ssr[i].Iod[1] {
			continue
		}
		svr.RtcmCtrl[index].Ssr[i].Update = 0

		iode = svr.RtcmCtrl[index].Ssr[i].Iode
		sys = SatSys(i+1, &prn)

		/* check corresponding ephemeris exists */
		switch sys {
		case SYS_GPS, SYS_GAL, SYS_QZS:
			if svr.NavData.Ephs[i].Iode != iode &&
				svr.NavData.Ephs[i+MAXSAT].Iode != iode {
				continue
			}
		case SYS_GLO:
			if svr.NavData.Geph[prn-1].Iode != iode &&
				svr.NavData.Geph[prn-1+MAXPRNGLO].Iode != iode {
				continue
			}
		}
		svr.NavData.Ssr[i] = svr.RtcmCtrl[index].Ssr[i]
	}
	svr.InputMsg[index][7]++
}

/* update rtk server struct --------------------------------------------------*/
func (svr *RtkSvr) UpdateSvr(ret int, obs *Obs, nav *Nav, ephsat,
	ephset int, sbsmsg *SbsMsg, index, iobs int) {
	Tracet(4, "updatesvr: ret=%d ephsat=%d ephset=%d index=%d\n", ret, ephsat,
		ephset, index)

	switch ret {
	case 1: /* observation data */
		svr.UpdateObs(obs, index, iobs)
	case 2: /* ephemeris */
		svr.UpdateEph(nav, ephsat, ephset, index)
	case 3: /* sbas message */
		svr.UpdateSbs(sbsmsg, index)
	case 9: /* ion/utc parameters */
		svr.UpdateIonUtc(nav, index)
	case 5: /* antenna postion */
		svr.UpdateAntPos(index)
	case 7: /* dgps correction */
		svr.InputMsg[index][5]++
	case 10: /* ssr message */
		svr.UpdateSsr(index)
	case -1: /* error */
		svr.InputMsg[index][9]++
	}
}

/* decode receiver raw/rtcm data ---------------------------------------------*/
func (svr *RtkSvr) DecodeRaw(index int) int {
	var (
		obs                          *Obs
		nav                          *Nav
		sbsmsg                       *SbsMsg = nil
		i, ret, ephsat, ephset, fobs int
	)

	Tracet(4, "decoderaw: index=%d\n", index)

	svr.RtkSvrLock()

	for i = 0; i < svr.Nb[index]; i++ {

		/* input rtcm/receiver raw data from stream */
		switch svr.Format[index] {
		case STRFMT_RTCM2:
			ret = svr.RtcmCtrl[index].InputRtcm2(svr.Buff[index][i])
			obs = &svr.RtcmCtrl[index].ObsData
			nav = &svr.RtcmCtrl[index].NavData
			ephsat = svr.RtcmCtrl[index].EphSat
			ephset = svr.RtcmCtrl[index].EphSet
		case STRFMT_RTCM3:
			ret = svr.RtcmCtrl[index].InputRtcm3(svr.Buff[index][i])
			obs = &svr.RtcmCtrl[index].ObsData
			nav = &svr.RtcmCtrl[index].NavData
			ephsat = svr.RtcmCtrl[index].EphSat
			ephset = svr.RtcmCtrl[index].EphSet
		default:
			ret = svr.RawCtrl[index].InputRaw(svr.Format[index], svr.Buff[index][i])
			obs = &svr.RawCtrl[index].ObsData
			nav = &svr.RawCtrl[index].NavData
			ephsat = svr.RawCtrl[index].EphSat
			ephset = svr.RawCtrl[index].EphSet
			sbsmsg = &svr.RawCtrl[index].Sbsmsg
		}
		// #if 0 /* record for receiving tick for debug */
		//         if (ret==1) {
		//             trace(0,"%d %10d T=%s NS=%2d\n",index,tickget(),
		//                   time_str(obs.data[0].time,0),obs.n);
		//         }
		// #endif
		/* update rtk server */
		if ret > 0 {
			svr.UpdateSvr(ret, obs, nav, ephsat, ephset, sbsmsg, index, fobs)
		}
		/* observation data received */
		if ret == 1 {
			if fobs < MAXOBSBUF {
				fobs++
			} else {
				svr.PrcOut++
			}
		}
	}
	svr.Nb[index] = 0

	svr.RtkSvrUnlock()

	return fobs
}

/* decode download file ------------------------------------------------------*/
func (svr *RtkSvr) DecodeFile(index int) {
	var (
		nav  Nav
		file string
		nb   int
	)

	Tracet(4, "decodefile: index=%d\n", index)

	svr.RtkSvrLock()

	/* check file path completed */
	nb = svr.Nb[index]
	if nb <= 2 || svr.Buff[index][nb-2] != '\r' || svr.Buff[index][nb-1] != '\n' {
		svr.RtkSvrUnlock()
		return
	}
	file = string(svr.Buff[index][:nb-2])

	svr.Nb[index] = 0

	svr.RtkSvrUnlock()

	switch svr.Format[index] {
	case STRFMT_SP3: /* precise ephemeris */

		/* read sp3 precise ephemeris */
		nav.ReadSp3(file, 0)
		if nav.Ne() <= 0 {
			Tracet(1, "sp3 file read error: %s\n", file)
			return
		}
		/* update precise ephemeris */
		svr.RtkSvrLock()

		if svr.NavData.Peph != nil {
			svr.NavData.Peph = nil
		}
		svr.NavData.Peph = nav.Peph
		svr.DownloadTime[index] = Utc2GpsT(TimeGet())
		svr.Files[index] = file

		svr.RtkSvrUnlock()
	case STRFMT_RNXCLK: /* precise clock */

		/* read rinex clock */
		if nav.ReadRnxC(file) <= 0 {
			Tracet(1, "rinex clock file read error: %s\n", file)
			return
		}
		/* update precise clock */
		svr.RtkSvrLock()

		if svr.NavData.Pclk != nil {
			svr.NavData.Pclk = nil
		}
		svr.NavData.Pclk = nav.Pclk
		svr.DownloadTime[index] = Utc2GpsT(TimeGet())
		svr.Files[index] = file

		svr.RtkSvrUnlock()
	}
}

/* carrier-phase bias (fcb) correction ---------------------------------------*/
func CorrPhaseBias(obs []ObsD, n int, nav *Nav) {
	var (
		freq float64
		code uint8
		i, j int
	)

	for i = 0; i < n; i++ {
		for j = 0; j < NFREQ; j++ {
			code = obs[i].Code[j]
			if freq = Sat2Freq(obs[i].Sat, code, nav); freq == 0.0 {
				continue
			}

			/* correct phase bias (cyc) */
			obs[i].L[j] -= nav.Ssr[obs[i].Sat-1].Pbias[code-1] * freq / CLIGHT
		}
	}
}

/* periodic command ----------------------------------------------------------*/
func periodic_cmd(cycle int, cmd string, stream *Stream) {
	var (
		msg    string
		period int
	)

	cmdlets := strings.FieldsFunc(cmd, func(r rune) bool {
		if r == '\r' || r == '\n' {
			return true
		}
		return false
	})

	for _, msg = range cmdlets {
		// for (p=cmd;;p=q+1) {
		//     for (q=p;;q++) if (*q=='\r'||*q=='\n'||*q=='\0') break;
		//     n=(int)(q-p); strncpy(msg,p,n); msg[n]='\0';

		period = 0
		if idx := strings.LastIndex(msg, "#"); idx >= 0 {
			fmt.Sscanf(msg[idx:], "# %d", &period)
			msg = msg[:idx]
		}
		msg = strings.TrimSpace(msg)
		// if ((r=strrchr(msg,'#'))) {
		//     sscanf(r,"# %d",&period);
		//     *r='\0';
		//     while (*--r==' ') *r='\0'; /* delete tail spaces */
		// }
		if period <= 0 {
			period = 1000
		}
		if len(msg) > 0 && cycle%period == 0 {
			stream.StrSendCmd(msg)
		}
	}
}

/* baseline length -----------------------------------------------------------*/
func (rtk *Rtk) BaseLineLen() float64 {
	var (
		dr [3]float64
		i  int
	)

	if Norm(rtk.RtkSol.Rr[:], 3) <= 0.0 || Norm(rtk.Rb[:], 3) <= 0.0 {
		return 0.0
	}

	for i = 0; i < 3; i++ {
		dr[i] = rtk.RtkSol.Rr[i] - rtk.Rb[i]
	}
	return Norm(dr[:], 3) * 0.001 /* (km) */
}

/* send nmea request to base/nrtk input stream -------------------------------*/
func (svr *RtkSvr) SendNmea(tickreset *uint32) {
	var (
		sol_nmea Sol
		vel, bl  float64
		i        int
	)
	tick := TickGet()

	if svr.Stream[1].State != 1 {
		return
	}

	switch svr.NmeaReq {
	case 1: /* lat-lon-hgt mode */
		sol_nmea.Stat = SOLQ_SINGLE
		sol_nmea.Time = Utc2GpsT(TimeGet())
		MatCpy(sol_nmea.Rr[:], svr.NmeaPos[:], 3, 1)
		svr.Stream[1].StreamSendNmea(&sol_nmea)
	case 2: /* single-solution mode */
		if Norm(svr.RtkCtrl.RtkSol.Rr[:], 3) <= 0.0 {
			return
		}
		sol_nmea.Stat = SOLQ_SINGLE
		sol_nmea.Time = Utc2GpsT(TimeGet())
		MatCpy(sol_nmea.Rr[:], svr.RtkCtrl.RtkSol.Rr[:], 3, 1)
		svr.Stream[1].StreamSendNmea(&sol_nmea)
	case 3: /* reset-and-single-sol mode */

		/* send reset command if baseline over threshold */
		bl = svr.RtkCtrl.BaseLineLen()
		if bl >= svr.BaseLenReset && int(tick-int64(*tickreset)) > MIN_INT_RESET {
			svr.Stream[1].StrSendCmd(svr.CmdReset)

			Tracet(2, "send reset: bl=%.3f rr=%.3f %.3f %.3f rb=%.3f %.3f %.3f\n",
				bl, svr.RtkCtrl.RtkSol.Rr[0], svr.RtkCtrl.RtkSol.Rr[1], svr.RtkCtrl.RtkSol.Rr[2],
				svr.RtkCtrl.Rb[0], svr.RtkCtrl.Rb[1], svr.RtkCtrl.Rb[2])
			*tickreset = uint32(tick)
		}
		if Norm(svr.RtkCtrl.RtkSol.Rr[:], 3) <= 0.0 {
			return
		}
		sol_nmea.Stat = SOLQ_SINGLE
		sol_nmea.Time = Utc2GpsT(TimeGet())
		MatCpy(sol_nmea.Rr[:], svr.RtkCtrl.RtkSol.Rr[:], 3, 1)

		/* set predicted position if velocity > 36km/h */
		if vel = Norm(svr.RtkCtrl.RtkSol.Rr[3:], 3); vel > 10.0 {
			for i = 0; i < 3; i++ {
				sol_nmea.Rr[i] += svr.RtkCtrl.RtkSol.Rr[i+3] / vel * svr.BaseLenReset * 0.8
			}
		}
		svr.Stream[1].StreamSendNmea(&sol_nmea)

		Tracet(3, "send nmea: rr=%.3f %.3f %.3f\n", sol_nmea.Rr[0], sol_nmea.Rr[1],
			sol_nmea.Rr[2])
	}
}

/* rtk server thread ---------------------------------------------------------*/

func rtksvrthread(svr *RtkSvr) int {
	var (
		obs                                Obs
		data                               []ObsD = make([]ObsD, MAXOBS*2)
		sol                                Sol
		tt                                 float64
		tick, ticknmea, tick1hz, tickreset uint32
		msg                                string
		i, j, n, cycle, cputime            int
		fobs                               [3]int
	)

	Tracet(3, "rtksvrthread:\n")

	svr.State = 1
	obs.Data = data
	svr.Tick = uint32(TickGet())
	ticknmea, tick1hz = svr.Tick-1000, svr.Tick-1000
	tickreset = svr.Tick - uint32(MIN_INT_RESET)

	for cycle = 0; svr.State > 0; cycle++ {
		tick = uint32(TickGet())
		for i = 0; i < 3; i++ {
			p := &svr.Nb[i]

			/* read receiver raw/rtcm data from input stream */
			if n = svr.Stream[i].StreamRead(svr.Buff[i][*p:], svr.BuffSize-*p); n <= 0 {
				continue
			}
			// if n = svr.Stream[i].StreamRead(svr.Buff[i][svr.Nb[i]:], svr.BuffSize-svr.Nb[i]); n <= 0 {
			// 	continue
			// }
			/* write receiver raw/rtcm data to log stream */
			svr.Stream[i+5].StreamWrite(svr.Buff[i][*p:], n)
			*p += n
			// svr.Stream[i+5].StreamWrite(svr.Buff[i][svr.Nb[i]:], n)
			// svr.Nb[i] += n

			/* save peek buffer */
			svr.RtkSvrLock()

			if n >= svr.BuffSize-svr.Npb[i] {
				n = svr.BuffSize - svr.Npb[i]
			}
			copy(svr.PBuf[i][svr.Npb[i]:], svr.Buff[i][*p-n:*p])
			// copy(svr.PBuf[i][svr.Npb[i]:], svr.Buff[i][svr.Nb[i]-n:svr.Nb[i]])
			svr.Npb[i] += n
			svr.RtkSvrUnlock()
		}
		for i = 0; i < 3; i++ {
			if svr.Format[i] == STRFMT_SP3 || svr.Format[i] == STRFMT_RNXCLK {
				/* decode download file */
				svr.DecodeFile(i)
			} else {
				/* decode receiver raw/rtcm data */
				fobs[i] = svr.DecodeRaw(i)
			}
		}
		/* averaging single base pos */
		if fobs[1] > 0 && svr.RtkCtrl.Opt.RefPos == POSOPT_SINGLE {
			if (svr.RtkCtrl.Opt.MaxAveEp <= 0 || svr.NAve < svr.RtkCtrl.Opt.MaxAveEp) &&
				PntPos(svr.ObsData[1][0].Data, svr.ObsData[1][0].N(), &svr.NavData,
					&svr.RtkCtrl.Opt, &sol, nil, nil, &msg) > 0 {
				svr.NAve++
				for i = 0; i < 3; i++ {
					svr.Rb_ave[i] += (sol.Rr[i] - svr.Rb_ave[i]) / float64(svr.NAve)
				}
			}
			for i = 0; i < 3; i++ {
				svr.RtkCtrl.Opt.Rb[i] = svr.Rb_ave[i]
			}
		}
		for i = 0; i < fobs[0]; i++ { /* for each rover observation data */
			obs.Data = nil
			for j = 0; j < svr.ObsData[0][i].N() && obs.N() < MAXOBS*2; j++ {
				obs.AddObsData(&svr.ObsData[0][i].Data[j])
			}
			for j = 0; j < svr.ObsData[1][0].N() && obs.N() < MAXOBS*2; j++ {
				obs.AddObsData(&svr.ObsData[1][0].Data[j])
			}
			/* carrier phase bias correction */
			if strings.Contains(svr.RtkCtrl.Opt.PPPOpt, "-DIS_FCB") {
				CorrPhaseBias(obs.Data, obs.N(), &svr.NavData)
			}

			/* send obs data to channel */
			for i = 0; i < obs.N(); i++ {
				select {
				case ObsChannel <- obs.Data[i]:
				default:
				}
			}

			/* rtk positioning */
			svr.RtkSvrLock()
			svr.RtkCtrl.RtkPos(obs.Data, obs.N(), &svr.NavData)
			svr.RtkSvrUnlock()

			if svr.RtkCtrl.RtkSol.Stat != SOLQ_NONE {

				/* adjust current time */
				tt = float64(int(TickGet()-int64(tick)))/1000.0 + float64(DTTOL)
				TimeSet(GpsT2Utc(TimeAdd(svr.RtkCtrl.RtkSol.Time, tt)))

				/* write solution */
				svr.WriteSol(i)
			}

			/* send sol to channel */
			select {
			case RbSolChannel <- RBSol{svr.RtkCtrl.Rb, svr.RtkCtrl.RtkSol}:
			default:
			}
			/* if cpu overload, inclement obs outage counter and break */
			if int(TickGet()-int64(tick)) >= svr.Cycle {
				svr.PrcOut += fobs[0] - i - 1
			}
		}
		/* send null solution if no solution (1hz) */
		if svr.RtkCtrl.RtkSol.Stat == SOLQ_NONE && tick-tick1hz >= 1000 {
			svr.WriteSol(0)
			tick1hz = tick
		}
		/* write periodic command to input stream */
		for i = 0; i < 3; i++ {
			periodic_cmd(cycle*svr.Cycle, svr.CmdsPeriodic[i], &svr.Stream[i])
		}
		/* send nmea request to base/nrtk input stream */
		if svr.NmeaCycle > 0 && (int)(tick-ticknmea) >= svr.NmeaCycle {
			svr.SendNmea(&tickreset)
			ticknmea = tick
		}
		if cputime = int(TickGet() - int64(tick)); cputime > 0 {
			svr.CpuTime = cputime
		}

		/* sleep until next cycle */
		Sleepms(svr.Cycle - cputime)
	}
	for i = 0; i < MAXSTRRTK; i++ {
		svr.Stream[i].StreamClose()
	}
	for i = 0; i < 3; i++ {
		svr.Nb[i], svr.Npb[i] = 0, 0
		svr.Buff[i] = nil
		svr.PBuf[i] = nil
	}
	for i = 0; i < 2; i++ {
		svr.Nsb[i] = 0
		svr.SBuf[i] = nil
	}
	svr.Wg.Done() // indicate the thread will exit
	return 0
}

/* initialize rtk server -------------------------------------------------------
* initialize rtk server
* args   : svr *RtkSvr    IO rtk server
* return : status (0:error,1:ok)
*-----------------------------------------------------------------------------*/
func (svr *RtkSvr) InitRtkSvr() int {
	var (
		time0 Gtime
		sol0  Sol
		eph0  Eph  = Eph{Sat: 0, Iode: -1, Iodc: -1}
		geph0 GEph = GEph{Sat: 0, Iode: -1}
		seph0 SEph = SEph{Sat: 0}
		i, j  int
	)

	Tracet(3, "rtksvrinit:\n")

	svr.State, svr.Cycle, svr.NmeaCycle, svr.NmeaReq = 0, 0, 0, 0
	for i = 0; i < 3; i++ {
		svr.NmeaPos[i] = 0.0
	}
	svr.BuffSize = 0
	for i = 0; i < 3; i++ {
		svr.Format[i] = 0
	}
	for i = 0; i < 2; i++ {
		svr.Solopt[i] = DefaultSolOpt()
	}
	svr.NavSel, svr.NoSbs, svr.NoSol = 0, 0, 0
	prcopt := DefaultProcOpt()
	svr.RtkCtrl.InitRtk(&prcopt)
	for i = 0; i < 3; i++ {
		svr.Nb[i] = 0
	}
	for i = 0; i < 2; i++ {
		svr.Nsb[i] = 0
	}
	for i = 0; i < 3; i++ {
		svr.Npb[i] = 0
	}
	for i = 0; i < 3; i++ {
		svr.Buff[i] = nil
	}
	for i = 0; i < 2; i++ {
		svr.SBuf[i] = nil
	}
	for i = 0; i < 3; i++ {
		svr.PBuf[i] = nil
	}
	for i = 0; i < MAXSOLBUF; i++ {
		svr.SolBuf[i] = sol0
	}
	for i = 0; i < 3; i++ {
		for j = 0; j < 10; j++ {
			svr.InputMsg[i][j] = 0
		}
	}
	for i = 0; i < 3; i++ {
		svr.DownloadTime[i] = time0
	}
	for i = 0; i < 3; i++ {
		svr.Files[i] = ""
	}
	svr.Monitor = nil
	svr.Tick = 0
	svr.Thread = 0
	svr.CpuTime, svr.PrcOut, svr.NAve = 0, 0, 0
	for i = 0; i < 3; i++ {
		svr.Rb_ave[i] = 0.0
	}

	//  memset(&svr.nav,0,sizeof(nav_t));
	svr.NavData.Ephs = make([]Eph, MAXSAT*4)
	svr.NavData.Geph = make([]GEph, NSATGLO*2)
	svr.NavData.Seph = make([]SEph, NSATSBS*2)
	for i = 0; i < MAXSAT*4; i++ {
		svr.NavData.Ephs[i] = eph0
	}
	for i = 0; i < NSATGLO*2; i++ {
		svr.NavData.Geph[i] = geph0
	}
	for i = 0; i < NSATSBS*2; i++ {
		svr.NavData.Seph[i] = seph0
	}
	// svr.NavData.N = MAXSAT * 2
	// svr.NavData.Ng = NSATGLO * 2
	// svr.NavData.Ns = NSATSBS * 2

	for i = 0; i < 3; i++ {
		for j = 0; j < MAXOBSBUF; j++ {
			svr.ObsData[i][j].Data = make([]ObsD, MAXOBS)
		}
	}
	// for  i=0;i<3;i++  {
	//     memset(svr.raw +i,0,sizeof(raw_t ));
	//     memset(svr.rtcm+i,0,sizeof(rtcm_t));
	// }
	for i = 0; i < MAXSTRRTK; i++ {
		svr.Stream[i].InitStream()
	}

	for i = 0; i < 3; i++ {
		svr.CmdsPeriodic[i] = ""
	}
	svr.CmdReset = ""
	svr.BaseLenReset = 10.0

	return 1
}

/* free rtk server -------------------------------------------------------------
* free rtk server
* args   : svr *RtkSvr    IO rtk server
* return : none
*-----------------------------------------------------------------------------*/
func (svr *RtkSvr) FreeRtkSvr() {
	var i, j int

	svr.NavData.Ephs = nil
	svr.NavData.Geph = nil
	svr.NavData.Seph = nil
	for i = 0; i < 3; i++ {
		for j = 0; j < MAXOBSBUF; j++ {
			svr.ObsData[i][j].Data = nil
		}
	}
	svr.RtkCtrl.FreeRtk()
}

/* lock/unlock rtk server ------------------------------------------------------
* lock/unlock rtk server
* args   : svr *RtkSvr    IO rtk server
* return : status (1:ok 0:error)
*-----------------------------------------------------------------------------*/
func (svr *RtkSvr) RtkSvrLock()   { svr.Lock.Lock() }
func (svr *RtkSvr) RtkSvrUnlock() { svr.Lock.Unlock() }

/* start rtk server ------------------------------------------------------------
* start rtk server thread
* args   : svr *RtkSvr    IO rtk server
*          int     cycle    I  server cycle (ms)
*          int     buffsize I  input buffer size (bytes)
*          int     *strs    I  stream types (STR_???)
*                              types[0]=input stream rover
*                              types[1]=input stream base station
*                              types[2]=input stream correction
*                              types[3]=output stream solution 1
*                              types[4]=output stream solution 2
*                              types[5]=log stream rover
*                              types[6]=log stream base station
*                              types[7]=log stream correction
*          char    *paths   I  input stream paths
*          int     *format  I  input stream formats (STRFMT_???)
*                              format[0]=input stream rover
*                              format[1]=input stream base station
*                              format[2]=input stream correction
*          int     navsel   I  navigation message select
*                              (0:rover,1:base,2:ephem,3:all)
*          char    **cmds   I  input stream start commands
*                              cmds[0]=input stream rover (NULL: no command)
*                              cmds[1]=input stream base (NULL: no command)
*                              cmds[2]=input stream corr (NULL: no command)
*          char    **cmds_periodic I input stream periodic commands
*                              cmds[0]=input stream rover (NULL: no command)
*                              cmds[1]=input stream base (NULL: no command)
*                              cmds[2]=input stream corr (NULL: no command)
*          char    **rcvopts I receiver options
*                              rcvopt[0]=receiver option rover
*                              rcvopt[1]=receiver option base
*                              rcvopt[2]=receiver option corr
*          int     nmeacycle I nmea request cycle (ms) (0:no request)
*          int     nmeareq  I  nmea request type
*                              (0:no,1:base pos,2:single sol,3:reset and single)
*          double *nmeapos  I  transmitted nmea position (ecef) (m)
*          prcopt_t *prcopt I  rtk processing options
*          solopt_t *solopt I  solution options
*                              solopt[0]=solution 1 options
*                              solopt[1]=solution 2 options
*          stream_t *moni   I  monitor stream (NULL: not used)
*          char   *errmsg   O  error message
* return : status (1:ok 0:error)
*-----------------------------------------------------------------------------*/
func (svr *RtkSvr) RtkSvrStart(cycle, buffsize int, strs []int,
	paths []string, formats []int, navsel int, cmds,
	cmds_periodic, rcvopts []string, nmeacycle,
	nmeareq int, nmeapos []float64, prcopt *PrcOpt,
	solopt []SolOpt, moni *Stream, errmsg *string) int {
	var (
		time, time0 Gtime
		i, j, rw    int
	)

	Tracet(3, "rtksvrstart: cycle=%d buffsize=%d navsel=%d nmeacycle=%d nmeareq=%d\n",
		cycle, buffsize, navsel, nmeacycle, nmeareq)

	if svr.State > 0 {
		*errmsg = "server already started"
		return 0
	}
	strinitcom()
	svr.Cycle = 1
	if cycle > 1 {
		svr.Cycle = cycle
	}
	svr.NmeaCycle = 1000
	if nmeacycle > 1000 {
		svr.NmeaCycle = nmeacycle
	}
	svr.NmeaReq = nmeareq
	for i = 0; i < 3; i++ {
		svr.NmeaPos[i] = nmeapos[i]
	}
	svr.BuffSize = 4096
	if buffsize > 4096 {
		svr.BuffSize = buffsize
	}
	for i = 0; i < 3; i++ {
		svr.Format[i] = formats[i]
	}
	svr.NavSel = navsel
	svr.NoSbs = 0
	svr.NoSol = 0
	svr.PrcOut = 0
	svr.RtkCtrl.FreeRtk()
	svr.RtkCtrl.InitRtk(prcopt)

	if prcopt.InitRst > 0 { /* init averaging pos by restart */
		svr.NAve = 0
		for i = 0; i < 3; i++ {
			svr.Rb_ave[i] = 0.0
		}
	}
	for i = 0; i < 3; i++ { /* input/log streams */
		svr.Nb[i], svr.Npb[i] = 0, 0
		svr.Buff[i] = make([]uint8, buffsize)
		svr.PBuf[i] = make([]uint8, buffsize)

		for j = 0; j < 10; j++ {
			svr.InputMsg[i][j] = 0
		}
		for j = 0; j < MAXOBSBUF; j++ {
			svr.ObsData[i][j].Data = nil
		}
		if len(cmds_periodic[i]) == 0 {
			svr.CmdsPeriodic[i] = ""
		} else {
			svr.CmdsPeriodic[i] = cmds_periodic[i]
		}

		/* initialize receiver raw and rtcm control */
		svr.RawCtrl[i].InitRaw(formats[i])
		svr.RtcmCtrl[i].InitRtcm()

		/* set receiver and rtcm option */
		svr.RawCtrl[i].Opt = rcvopts[i]
		svr.RtcmCtrl[i].Opt = rcvopts[i]

		/* connect dgps corrections */
		copy(svr.RtcmCtrl[i].Dgps[:], svr.NavData.Dgps[:])
	}
	for i = 0; i < 2; i++ { /* output peek buffer */
		svr.SBuf[i] = make([]uint8, buffsize)
	}
	/* set solution options */
	for i = 0; i < 2; i++ {
		svr.Solopt[i] = solopt[i]
	}
	/* set base station position */
	if prcopt.RefPos != POSOPT_SINGLE {
		for i = 0; i < 6; i++ {
			svr.RtkCtrl.Rb[i] = 0.0
			if i < 3 {
				svr.RtkCtrl.Rb[i] = prcopt.Rb[i]
			}
		}
	}
	/* update navigation data */
	for i = 0; i < MAXSAT*4; i++ {
		svr.NavData.Ephs[i].Ttr = time0
	}
	for i = 0; i < NSATGLO*2; i++ {
		svr.NavData.Geph[i].Tof = time0
	}
	for i = 0; i < NSATSBS*2; i++ {
		svr.NavData.Seph[i].Tof = time0
	}

	/* set monitor stream */
	svr.Monitor = moni

	/* open input streams */
	for i = 0; i < 8; i++ {
		rw = STR_MODE_W
		if i < 3 {
			rw = STR_MODE_R
		}
		if strs[i] != STR_FILE {
			rw |= STR_MODE_W
		}
		if svr.Stream[i].OpenStream(strs[i], rw, paths[i]) == 0 {
			*errmsg = fmt.Sprintf("str%d open error path=%s", i+1, paths[i])
			for i--; i >= 0; i-- {
				svr.Stream[i].StreamClose()
			}
			return 0
		}
		/* set initial time for rtcm and raw */
		if i < 3 {
			time = Utc2GpsT(TimeGet())
			if strs[i] == STR_FILE {
				svr.RawCtrl[i].Time = StreamGetTime(&svr.Stream[i])
				svr.RtcmCtrl[i].Time = StreamGetTime(&svr.Stream[i])
			} else {
				svr.RawCtrl[i].Time = time
				svr.RtcmCtrl[i].Time = time
			}
		}
	}
	/* sync input streams */
	strsync(&svr.Stream[0], &svr.Stream[1])
	strsync(&svr.Stream[0], &svr.Stream[2])

	/* write start commands to input streams */
	for i = 0; i < 3; i++ {
		if len(cmds[i]) > 0 {
			continue
		}
		svr.Stream[i].StreamWrite([]byte(""), 0) /* for connect */
		Sleepms(100)
		svr.Stream[i].StrSendCmd(cmds[i])
	}
	/* write solution header to solution streams */
	for i = 3; i < 5; i++ {
		writesolhead(&svr.Stream[i], &svr.Solopt[i-3])
	}
	/* initialize channel */
	ObsChannel = make(chan ObsD, MAXOBSBUF)
	RbSolChannel = make(chan RBSol, 10)

	/* create rtk server thread */
	svr.Wg.Add(1)
	go rtksvrthread(svr)
	// #ifdef WIN32
	//     if (!(svr.thread=CreateThread(NULL,0,rtksvrthread,svr,0,NULL))) {
	// #else
	//     if (pthread_create(&svr.thread,NULL,rtksvrthread,svr)) {
	// #endif
	return 1
}

/* stop rtk server -------------------------------------------------------------
* start rtk server thread
* args   : svr *RtkSvr    IO rtk server
*          char    **cmds   I  input stream stop commands
*                              cmds[0]=input stream rover (NULL: no command)
*                              cmds[1]=input stream base  (NULL: no command)
*                              cmds[2]=input stream ephem (NULL: no command)
* return : none
*-----------------------------------------------------------------------------*/
func (svr *RtkSvr) RtkSvrStop(cmds []string) {
	var i int

	Tracet(3, "rtksvrstop:\n")

	/* write stop commands to input streams */
	svr.RtkSvrLock()
	for i = 0; i < 3; i++ {
		if len(cmds[i]) > 0 {
			svr.Stream[i].StrSendCmd(cmds[i])
		}
	}
	svr.RtkSvrUnlock()

	close(ObsChannel)
	close(RbSolChannel)

	/* stop rtk server */
	svr.State = 0
	svr.Wg.Wait() // wait for thread exit
}

/* open output/log stream ------------------------------------------------------
* open output/log stream
* args   : svr *RtkSvr    IO rtk server
*          int     index    I  output/log stream index
*                              (3:solution 1,4:solution 2,5:log rover,
*                               6:log base station,7:log correction)
*          int     str      I  output/log stream types (STR_???)
*          char    *path    I  output/log stream path
*          solopt_t *solopt I  solution options
* return : status (1:ok 0:error)
*-----------------------------------------------------------------------------*/
func (svr *RtkSvr) RtkSvrOpenStream(index, str int, path string, solopt []SolOpt) int {
	Tracet(3, "rtksvropenstr: index=%d str=%d path=%s\n", index, str, path)

	if index < 3 || index > 7 || svr.State == 0 {
		return 0
	}

	svr.RtkSvrLock()
	defer svr.RtkSvrUnlock()
	if svr.Stream[index].State > 0 {

		return 0
	}
	if svr.Stream[index].OpenStream(str, STR_MODE_W, path) == 0 {
		Tracet(2, "stream open error: index=%d\n", index)
		return 0
	}
	if index <= 4 {
		svr.Solopt[index-3] = solopt[0]

		/* write solution header to solution stream */
		writesolhead(&svr.Stream[index], &svr.Solopt[index-3])
	}
	return 1
}

/* close output/log stream -----------------------------------------------------
* close output/log stream
* args   : svr *RtkSvr    IO rtk server
*          int     index    I  output/log stream index
*                              (3:solution 1,4:solution 2,5:log rover,
*                               6:log base station,7:log correction)
* return : none
*-----------------------------------------------------------------------------*/
func (svr *RtkSvr) RtkSvrCloseStream(index int) {
	Tracet(3, "rtksvrclosestr: index=%d\n", index)

	if index < 3 || index > 7 || svr.State == 0 {
		return
	}

	svr.RtkSvrLock()

	svr.Stream[index].StreamClose()

	svr.RtkSvrUnlock()
}

/* get observation data status -------------------------------------------------
* get current observation data status
* args   : svr *RtkSvr    I  rtk server
*          int     rcv      I  receiver (0:rover,1:base,2:ephem)
*          gtime_t *time    O  time of observation data
*          int     *sat     O  satellite prn numbers
*          double  *az      O  satellite azimuth angles (rad)
*          double  *el      O  satellite elevation angles (rad)
*          int     **snr    O  satellite snr for each freq (dBHz)
*                              snr[i][j] = sat i freq j snr
*          int     *vsat    O  valid satellite flag
* return : number of satellites
*-----------------------------------------------------------------------------*/
func (svr *RtkSvr) RtkSvrObsStat(rcv int, time *Gtime, sat []int,
	az, el []float64, snr [][]int, vsat []int) int {
	var i, j, ns int

	Tracet(4, "rtksvrostat: rcv=%d\n", rcv)

	if svr.State == 0 {
		return 0
	}
	svr.RtkSvrLock()
	defer svr.RtkSvrUnlock()

	ns = svr.ObsData[rcv][0].N()
	if ns > 0 {
		*time = svr.ObsData[rcv][0].Data[0].Time
	}
	for i = 0; i < ns; i++ {
		sat[i] = svr.ObsData[rcv][0].Data[i].Sat
		az[i] = svr.RtkCtrl.Ssat[sat[i]-1].Azel[0]
		el[i] = svr.RtkCtrl.Ssat[sat[i]-1].Azel[1]
		for j = 0; j < NFREQ; j++ {
			snr[i][j] = int(float32(svr.ObsData[rcv][0].Data[i].SNR[j])*SNR_UNIT + 0.5)
		}
		if svr.RtkCtrl.RtkSol.Stat == SOLQ_NONE || svr.RtkCtrl.RtkSol.Stat == SOLQ_SINGLE {
			vsat[i] = int(svr.RtkCtrl.Ssat[sat[i]-1].Vs)
		} else {
			vsat[i] = int(svr.RtkCtrl.Ssat[sat[i]-1].Vsat[0])
		}
	}

	return ns
}

/* get stream status -----------------------------------------------------------
* get current stream status
* args   : svr *RtkSvr    I  rtk server
*          int     *sstat   O  status of streams
*          char    *msg     O  status messages
* return : none
*-----------------------------------------------------------------------------*/
func (svr *RtkSvr) RtkSvrStreamStat(sstat []int, msg *string) {

	var s string

	Tracet(4, "rtksvrsstat:\n")

	svr.RtkSvrLock()
	for i := 0; i < MAXSTRRTK; i++ {
		sstat[i] = svr.Stream[i].StreamStat(&s)
		if len(s) > 0 {
			*msg += fmt.Sprintf("(%d) %s ", i+1, s)
		}
	}
	svr.RtkSvrUnlock()
}

/* mark current position -------------------------------------------------------
* open output/log stream
* args   : svr *RtkSvr    IO rtk server
*          char    *name    I  marker name
*          char    *comment I  comment string
* return : status (1:ok 0:error)
*-----------------------------------------------------------------------------*/
func (svr *RtkSvr) RtkSvrMark(name, comment string) int {
	var (
		buff, tstr      string
		tow             float64
		pos             [3]float64
		i, j, sum, week int
	)

	Tracet(4, "rtksvrmark:name=%s comment=%s\n", name, comment)

	if svr.State == 0 {
		return 0
	}

	svr.RtkSvrLock()
	defer svr.RtkSvrUnlock()

	Time2Str(svr.RtkCtrl.RtkSol.Time, &tstr, 3)
	tow = Time2GpsT(svr.RtkCtrl.RtkSol.Time, &week)
	Ecef2Pos(svr.RtkCtrl.RtkSol.Rr[:], pos[:])

	for i = 0; i < 2; i++ {
		//        p=buff;
		switch svr.Solopt[i].Posf {
		case SOLF_STAT:
			{
				buff += fmt.Sprintf("$MARK,%d,%.3f,%d,%.4f,%.4f,%.4f,%s,%s\r\n", week, tow,
					svr.RtkCtrl.RtkSol.Stat, svr.RtkCtrl.RtkSol.Rr[0], svr.RtkCtrl.RtkSol.Rr[1],
					svr.RtkCtrl.RtkSol.Rr[2], name, comment)
			}
		case SOLF_NMEA:
			{
				buff += fmt.Sprintf("$GPTXT,01,01,02,MARK:%s,%s,%.9f,%.9f,%.4f,%d,%s",
					name, tstr, pos[0]*R2D, pos[1]*R2D, pos[2], svr.RtkCtrl.RtkSol.Stat,
					comment)
				for j, sum = 1, 0; j < len(buff); j++ {
					sum ^= int(buff[j])
				}
				//            for (q=(char *)buff+1,sum=0;*q;q++) sum^=*q; /* check-sum */
				buff += fmt.Sprintf("*%02X\r\n", sum)
			}
		default:
			{
				buff += fmt.Sprintf("%s MARK: %s,%s,%.9f,%.9f,%.4f,%d,%s\r\n", COMMENTH,
					name, tstr, pos[0]*R2D, pos[1]*R2D, pos[2], svr.RtkCtrl.RtkSol.Stat,
					comment)
			}
		}
		svr.Stream[i+3].StreamWrite([]byte(buff), len(buff))
		svr.SaveOutBuf([]byte(buff), len(buff), i)
	}
	if svr.Monitor != nil {
		//      p=buff;
		buff += fmt.Sprintf("%s MARK: %s,%s,%.9f,%.9f,%.4f,%d,%s\r\n", COMMENTH,
			name, tstr, pos[0]*R2D, pos[1]*R2D, pos[2], svr.RtkCtrl.RtkSol.Stat,
			comment)
		svr.Monitor.StreamWrite([]byte(buff), len(buff))
	}
	return 1
}
