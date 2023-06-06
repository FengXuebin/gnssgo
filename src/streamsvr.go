/*------------------------------------------------------------------------------
* streamsvr.c : stream server functions
*
*          Copyright (C) 2010-2020 by T.TAKASU, All rights reserved.
*
* options : -DWIN32    use WIN32 API
*
* version : $Revision:$ $Date:$
* history : 2010/07/18 1.0  moved from stream.c
*           2011/01/18 1.1  change api strsvrstart()
*           2012/12/04 1.2  add stream conversion function
*           2012/12/25 1.3  fix bug on cyclic navigation data output
*                           suppress warnings
*           2013/05/08 1.4  fix bug on 1 s offset for javad . rtcm conversion
*           2014/10/16 1.5  support input from stdout
*           2015/12/05 1.6  support rtcm 3 mt 63 beidou ephemeris
*           2016/07/23 1.7  change api strsvrstart(),strsvrstop()
*                           support command for output streams
*           2016/08/20 1.8  support api change of sendnmea()
*           2016/09/03 1.9  support ntrip caster function
*           2016/09/06 1.10 add api strsvrsetsrctbl()
*           2016/09/17 1.11 add relay back function of output stream
*                           fix bug on rtcm cyclic output of beidou ephemeris
*           2016/10/01 1.12 change api startstrserver()
*           2017/04/11 1.13 fix bug on search of next satellite in nextsat()
*           2018/11/05 1.14 update message type of beidou ephemeirs
*                           support multiple msm messages if nsat x nsig > 64
*           2020/11/30 1.15 support RTCM MT1131-1137,1041 (NavIC/IRNSS)
*                           add log paths in API strsvrstart()
*                           add log status in API strsvrstat()
*                           support multiple ephemeris sets (e.g. I/NAV-F/NAV)
*                           delete API strsvrsetsrctbl()
*                           use integer types in stdint.h
*		    2022/05/31 1.0  rewrite streamsvr.c with golang by fxb
*-----------------------------------------------------------------------------*/
package gnssgo

import (
	"fmt"
	"math"
	"strings"
)

/* test observation data message ---------------------------------------------*/
func is_obsmsg(msg int) int {
	if (1001 <= msg && msg <= 1004) || (1009 <= msg && msg <= 1012) ||
		(1071 <= msg && msg <= 1077) || (1081 <= msg && msg <= 1087) ||
		(1091 <= msg && msg <= 1097) || (1101 <= msg && msg <= 1107) ||
		(1111 <= msg && msg <= 1117) || (1121 <= msg && msg <= 1127) ||
		(1131 <= msg && msg <= 1137) {
		return 1
	}
	return 0
}

/* test navigation data message ----------------------------------------------*/
func is_navmsg(msg int) int {
	if msg == 1019 || msg == 1020 || msg == 1044 || msg == 1045 || msg == 1046 ||
		msg == 1042 || msg == 63 || msg == 1041 {
		return 1
	}
	return 0
}

/* test station info message -------------------------------------------------*/
func is_stamsg(msg int) int {
	if msg == 1005 || msg == 1006 || msg == 1007 || msg == 1008 || msg == 1033 || msg == 1230 {
		return 1
	}
	return 0
}

/* test time interval --------------------------------------------------------*/
func is_tint(time Gtime, tint float64) int {
	if tint <= 0.0 {
		return 1
	}
	if math.Mod(Time2GpsT(time, nil)+float64(DTTOL), tint) <= float64(2.0*DTTOL) {
		return 1
	}
	return 0
}

/* new stream converter --------------------------------------------------------
* generate new stream converter
* args   : int    itype     I   input stream type  (STRFMT_???)
*          int    otype     I   output stream type (STRFMT_???)
*          char   *msgs     I   output message type and interval (, separated)
*          int    staid     I   station id
*          int    stasel    I   station info selection (0:remote,1:local)
*          char   *opt      I   rtcm or receiver raw options
* return : stream generator (NULL:error)
*-----------------------------------------------------------------------------*/
func NewStreamConv(itype, otype int, msgs string, staid, stasel int, opt string) *StrConv {
	var (
		conv *StrConv = new(StrConv)
		tint float64
		buff string
		msg  int
	)

	conv.NoMsg = 0
	buff = msgs
	strs := strings.Split(buff, ",")
	for _, v := range strs {
		tint = 0.0
		if n, _ := fmt.Sscanf(v, "%d(%f)", &msg, &tint); n < 1 {
			continue
		}
		conv.MsgType[conv.NoMsg] = msg
		conv.OutInterval[conv.NoMsg] = tint
		conv.Tick[conv.NoMsg] = uint32(TickGet())
		conv.EphSat[conv.NoMsg] = 0
		conv.NoMsg++
		if conv.NoMsg >= 32 {
			break
		}
	}
	if conv.NoMsg <= 0 {
		conv = nil
		return nil
	}
	conv.InputType = itype
	conv.OutputType = otype
	conv.StationSel = stasel
	if conv.RtcmInput.InitRtcm() == 0 || conv.RtcmOutput.InitRtcm() == 0 {
		conv = nil
		return nil
	}
	if conv.RawInput.InitRaw(itype) == 0 {
		conv.RtcmInput.FreeRtcm()
		conv.RtcmOutput.FreeRtcm()
		conv = nil
		return nil
	}
	if stasel > 0 {
		conv.RtcmOutput.StaId = staid
	}
	conv.RtcmInput.Opt = fmt.Sprintf("-EPHALL %s", opt)
	conv.RawInput.Opt = fmt.Sprintf("-EPHALL %s", opt)
	return conv
}

/* free stream converter -------------------------------------------------------
* free stream converter
* args   : strconv_t *conv  IO  stream converter
* return : none
*-----------------------------------------------------------------------------*/
func (conv *StrConv) FreeStreamConv() {
	if conv == nil {
		return
	}
	conv.RtcmInput.FreeRtcm()
	conv.RtcmOutput.FreeRtcm()
	conv.RawInput.FreeRaw()
	conv = nil
}

/* copy received data from receiver raw to rtcm ------------------------------*/
func (raw *Raw) Raw2Rtcm(out *Rtcm, ret int) {
	var i, sat, set, sys, prn int

	out.Time = raw.Time

	switch ret {
	case 1:
		for i = 0; i < raw.ObsData.N(); i++ {
			out.Time = raw.ObsData.Data[i].Time
			out.ObsData.AddObsData(&raw.ObsData.Data[i])

			sys = SatSys(raw.ObsData.Data[i].Sat, &prn)
			if sys == SYS_GLO && raw.NavData.Glo_fcn[prn-1] > 0 {
				out.NavData.Glo_fcn[prn-1] = raw.NavData.Glo_fcn[prn-1]
			}
		}
	case 2:
		sat = raw.EphSat
		set = raw.EphSet
		sys = SatSys(sat, &prn)
		switch sys {
		case SYS_GLO:
			out.NavData.Geph[prn-1] = raw.NavData.Geph[prn-1]
			out.EphSat = sat
			out.EphSet = set
		case SYS_GPS, SYS_GAL, SYS_QZS, SYS_CMP, SYS_IRN:
			out.NavData.Ephs[sat-1+MAXSAT*set] = raw.NavData.Ephs[sat-1+MAXSAT*set]
			out.EphSat = sat
			out.EphSet = set
		}
	case 5:
		out.StaPara = raw.StaData
	case 9:
		MatCpy(out.NavData.Utc_gps[:], raw.NavData.Utc_gps[:], 8, 1)
		MatCpy(out.NavData.Utc_glo[:], raw.NavData.Utc_glo[:], 8, 1)
		MatCpy(out.NavData.Utc_gal[:], raw.NavData.Utc_gal[:], 8, 1)
		MatCpy(out.NavData.Utc_qzs[:], raw.NavData.Utc_qzs[:], 8, 1)
		MatCpy(out.NavData.Utc_cmp[:], raw.NavData.Utc_cmp[:], 8, 1)
		MatCpy(out.NavData.Utc_irn[:], raw.NavData.Utc_irn[:], 9, 1)
		MatCpy(out.NavData.Utc_sbs[:], raw.NavData.Utc_sbs[:], 4, 1)
		MatCpy(out.NavData.Ion_gps[:], raw.NavData.Ion_gps[:], 8, 1)
		MatCpy(out.NavData.Ion_gal[:], raw.NavData.Ion_gal[:], 4, 1)
		MatCpy(out.NavData.Ion_qzs[:], raw.NavData.Ion_qzs[:], 8, 1)
		MatCpy(out.NavData.Ion_cmp[:], raw.NavData.Ion_cmp[:], 8, 1)
		MatCpy(out.NavData.Ion_irn[:], raw.NavData.Ion_irn[:], 8, 1)
	}
}

/* copy received data from receiver rtcm to rtcm -----------------------------*/
func (rtcm *Rtcm) Rtcm2Rtcm(out *Rtcm, ret, stasel int) {
	var i, sat, set, sys, prn int

	out.Time = rtcm.Time

	if stasel == 0 {
		out.StaId = rtcm.StaId
	}

	switch ret {
	case 1:
		for i = 0; i < rtcm.ObsData.N(); i++ {
			out.ObsData.AddObsData(&rtcm.ObsData.Data[i])

			sys = SatSys(rtcm.ObsData.Data[i].Sat, &prn)
			if sys == SYS_GLO && rtcm.NavData.Glo_fcn[prn-1] > 0 {
				out.NavData.Glo_fcn[prn-1] = rtcm.NavData.Glo_fcn[prn-1]
			}
		}
	case 2:
		sat = rtcm.EphSat
		set = rtcm.EphSet
		sys = SatSys(sat, &prn)
		switch sys {
		case SYS_GLO:
			out.NavData.Geph[prn-1] = rtcm.NavData.Geph[prn-1]
			out.EphSat = sat
			out.EphSet = set
		case SYS_GPS, SYS_GAL, SYS_QZS, SYS_CMP, SYS_IRN:
			out.NavData.Ephs[sat-1+MAXSAT*set] = rtcm.NavData.Ephs[sat-1+MAXSAT*set]
			out.EphSat = sat
			out.EphSet = set
		}
	case 5:
		if stasel == 0 {
			out.StaPara = rtcm.StaPara
		}
	}
}

/* write rtcm3 msm to stream -------------------------------------------------*/
func (str *Stream) WriteRtcm3Msm(out *Rtcm, msg, sync int) {
	var (
		data                                           []ObsD
		i, j, n, ns, sys, nobs, code, nsat, nsig, nmsg int
		mask                                           [MAXCODE]int
	)

	switch {
	case 1071 <= msg && msg <= 1077:
		sys = SYS_GPS
	case 1081 <= msg && msg <= 1087:
		sys = SYS_GLO
	case 1091 <= msg && msg <= 1097:
		sys = SYS_GAL
	case 1101 <= msg && msg <= 1107:
		sys = SYS_SBS
	case 1111 <= msg && msg <= 1117:
		sys = SYS_QZS
	case 1121 <= msg && msg <= 1127:
		sys = SYS_CMP
	case 1131 <= msg && msg <= 1137:
		sys = SYS_IRN
	default:
		return
	}

	data = out.ObsData.Data
	nobs = out.ObsData.N()

	/* count number of satellites and signals */
	for i = 0; i < nobs && i < MAXOBS; i++ {
		if SatSys(data[i].Sat, nil) != sys {
			continue
		}
		nsat++
		for j = 0; j < NFREQ+NEXOBS; j++ {
			if code = int(data[i].Code[j]); code == 0 || mask[code-1] > 0 {
				continue
			}
			mask[code-1] = 1
			nsig++
		}
	}
	if nsig > 64 {
		return
	}

	/* pack data to multiple messages if nsat x nsig > 64 */
	if nsig > 0 {
		ns = 64 / nsig         /* max number of sats in a message */
		nmsg = (nsat-1)/ns + 1 /* number of messages */
	} else {
		ns = 0
		nmsg = 1
	}
	out.ObsData.Data = nil

	for i, j = 0, 0; i < nmsg; i++ {
		for n = 0; n < ns && j < nobs && j < MAXOBS; j++ {
			if SatSys(data[j].Sat, nil) != sys {
				continue
			}
			out.ObsData.AddObsData(&data[j])
			n++
		}
		isync := sync
		if i < nmsg-1 {
			isync = 1
		}
		if out.GenRtcm3(msg, 0, isync) > 0 {
			str.StreamWrite(out.Buff[:], out.Nbyte)
		}
	}
	//	out.ObsData.Data = data
	// out.ObsData.N = nobs
}

/* write obs data messages ---------------------------------------------------*/
func (str *Stream) WriteObs(time Gtime, conv *StrConv) {
	var i, j, k int

	for i = 0; i < conv.NoMsg; i++ {
		if is_obsmsg(conv.MsgType[i]) == 0 || is_tint(time, conv.OutInterval[i]) == 0 {
			continue
		}

		j = i /* index of last message */
	}
	for i = 0; i < conv.NoMsg; i++ {
		if is_obsmsg(conv.MsgType[i]) == 0 || is_tint(time, conv.OutInterval[i]) == 0 {
			continue
		}
		if i != j {
			k = 1
		} else {
			k = 0
		}
		/* generate messages */
		switch conv.OutputType {
		case STRFMT_RTCM2:
			if conv.RtcmOutput.gen_rtcm2(conv.MsgType[i], k) == 0 {
				continue
			}

			/* write messages to stream */
			str.StreamWrite(conv.RtcmOutput.Buff[:], conv.RtcmOutput.Nbyte)
		case STRFMT_RTCM3:
			if conv.MsgType[i] <= 1012 {
				if conv.RtcmOutput.GenRtcm3(conv.MsgType[i], 0, k) == 0 {
					continue
				}
				str.StreamWrite(conv.RtcmOutput.Buff[:], conv.RtcmOutput.Nbyte)
			} else { /* write rtcm3 msm to stream */
				str.WriteRtcm3Msm(&conv.RtcmOutput, conv.MsgType[i], k)
			}
		}
	}
}

/* write nav data messages ---------------------------------------------------*/
func (str *Stream) WriteNav(time Gtime, conv *StrConv) {
	for i := 0; i < conv.NoMsg; i++ {
		if is_navmsg(conv.MsgType[i]) == 0 || conv.OutInterval[i] > 0.0 {
			continue
		}

		/* generate messages */
		switch conv.OutputType {
		case STRFMT_RTCM2:
			if conv.RtcmOutput.gen_rtcm2(conv.MsgType[i], 0) == 0 {
				continue
			}
		case STRFMT_RTCM3:
			if conv.RtcmOutput.GenRtcm3(conv.MsgType[i], 0, 0) == 0 {
				continue
			}
		default:
			continue
		}

		/* write messages to stream */
		str.StreamWrite(conv.RtcmOutput.Buff[:], conv.RtcmOutput.Nbyte)
	}
}

/* next ephemeris satellite --------------------------------------------------*/
func (nav *Nav) NextSat(sat, msg int) int {
	var sys, set, p, p0, p1, p2 int

	switch msg {
	case 1019:
		sys = SYS_GPS
		set = 0
		p1 = MINPRNGPS
		p2 = MAXPRNGPS
	case 1020:
		sys = SYS_GLO
		set = 0
		p1 = MINPRNGLO
		p2 = MAXPRNGLO
	case 1044:
		sys = SYS_QZS
		set = 0
		p1 = MINPRNQZS
		p2 = MAXPRNQZS
	case 1045:
		sys = SYS_GAL
		set = 1
		p1 = MINPRNGAL
		p2 = MAXPRNGAL
	case 1046:
		sys = SYS_GAL
		set = 0
		p1 = MINPRNGAL
		p2 = MAXPRNGAL
	case 63, 1042:
		sys = SYS_CMP
		set = 0
		p1 = MINPRNCMP
		p2 = MAXPRNCMP
	case 1041:
		sys = SYS_IRN
		set = 0
		p1 = MINPRNIRN
		p2 = MAXPRNIRN
	default:
		return 0
	}
	if SatSys(sat, &p0) != sys {
		return SatNo(sys, p1)
	}

	/* search next valid ephemeris */
	p = p0 + 1
	if p0 >= p2 {
		p = p1
	}
	for p != p0 {

		if sys == SYS_GLO {
			sat = SatNo(sys, p)
			if nav.Geph[p-1].Sat == sat {
				return sat
			}
		} else {
			sat = SatNo(sys, p)
			if nav.Ephs[sat-1+MAXSAT*set].Sat == sat {
				return sat
			}
		}
		if p >= p2 {
			p = p1
		} else {
			p++
		}
	}
	return 0
}

/* write cyclic nav data messages --------------------------------------------*/
func (str *Stream) WriteNavCycle(conv *StrConv) {
	tick := TickGet()
	var i, sat, tint int

	for i = 0; i < conv.NoMsg; i++ {
		if is_navmsg(conv.MsgType[i]) == 0 || conv.OutInterval[i] <= 0.0 {
			continue
		}

		/* output cycle */
		tint = int(conv.OutInterval[i] * 1000.0)
		if int(tick-int64(conv.Tick[i])) < tint {
			continue
		}
		conv.Tick[i] = uint32(tick)

		/* next satellite */
		if sat = conv.RtcmOutput.NavData.NextSat(conv.EphSat[i], conv.MsgType[i]); sat == 0 {
			continue
		}
		conv.RtcmOutput.EphSat, conv.EphSat[i] = sat, sat

		/* generate messages */
		switch conv.OutputType {
		case STRFMT_RTCM2:
			if (conv.RtcmOutput.gen_rtcm2(conv.MsgType[i], 0)) == 0 {
				continue
			}
		case STRFMT_RTCM3:
			if (conv.RtcmOutput.GenRtcm3(conv.MsgType[i], 0, 0)) == 0 {
				continue
			}
		default:
			continue
		}

		/* write messages to stream */
		str.StreamWrite(conv.RtcmOutput.Buff[:], conv.RtcmOutput.Nbyte)
	}
}

/* write cyclic station info messages ----------------------------------------*/
func (str *Stream) WriteStaCycle(conv *StrConv) {
	tick := TickGet()
	var i, tint int

	for i = 0; i < conv.NoMsg; i++ {
		if is_stamsg(conv.MsgType[i]) == 0 {
			continue
		}

		/* output cycle */
		if conv.OutInterval[i] == 0.0 {
			tint = 30000
		} else {
			tint = int(conv.OutInterval[i] * 1000.0)
		}
		if int(tick-int64(conv.Tick[i])) < tint {
			continue
		}
		conv.Tick[i] = uint32(tick)

		/* generate messages */
		switch conv.OutputType {
		case STRFMT_RTCM2:
			if conv.RtcmOutput.gen_rtcm2(conv.MsgType[i], 0) == 0 {
				continue
			}
		case STRFMT_RTCM3:
			if conv.RtcmOutput.GenRtcm3(conv.MsgType[i], 0, 0) == 0 {
				continue
			}
		default:
			continue
		}

		/* write messages to stream */
		str.StreamWrite(conv.RtcmOutput.Buff[:], conv.RtcmOutput.Nbyte)
	}
}

/* convert stearm ------------------------------------------------------------*/
func (str *Stream) StreamConv(conv *StrConv, buff []uint8, n int) {
	var i, ret int

	for i = 0; i < n; i++ {

		/* input rtcm 2 messages */
		switch conv.InputType {
		case STRFMT_RTCM2:
			ret = conv.RtcmInput.InputRtcm2(buff[i])
			conv.RtcmInput.Rtcm2Rtcm(&conv.RtcmOutput, ret, conv.StationSel)
		case STRFMT_RTCM3: /* input rtcm 3 messages */
			ret = conv.RtcmInput.InputRtcm3(buff[i])
			conv.RtcmInput.Rtcm2Rtcm(&conv.RtcmOutput, ret, conv.StationSel)
		default: /* input receiver raw messages */
			ret = conv.RawInput.InputRaw(conv.InputType, buff[i])
			conv.RawInput.Raw2Rtcm(&conv.RtcmOutput, ret)
		}
		/* write obs and nav data messages to stream */
		switch ret {
		case 1:
			str.WriteObs(conv.RtcmOutput.Time, conv)
		case 2:
			str.WriteNav(conv.RtcmOutput.Time, conv)
		}
	}
	/* write cyclic nav data and station info messages to stream */
	str.WriteNavCycle(conv)
	str.WriteStaCycle(conv)
}

/* stearm server thread ------------------------------------------------------*/

func strsvrthread(svr *StreamSvr) int {
	var (
		sol_nmea        Sol
		tick, tick_nmea int64
		buff            []uint8 = make([]uint8, 1024)
		i, n, cyc       int
	)

	Tracet(3, "strsvrthread:\n")

	svr.Tick = TickGet()
	tick_nmea = svr.Tick - 1000

	for cyc = 0; svr.State > 0; cyc++ {
		tick = TickGet()

		/* read data from input stream */
		for {
			n = svr.InputStream[0].StreamRead(svr.Buff, svr.BuffSize)
			if n <= 0 || svr.State == 0 {
				break
			}
			// while ((n=strread(svr.stream,svr.buff,svr.buffsize))>0&&svr.state) {

			/* write data to output streams */
			for i = 1; i < svr.NoStream; i++ {
				if svr.Converter[i-1] != nil {
					svr.InputStream[i].StreamConv(svr.Converter[i-1], svr.Buff, n)
				} else {
					svr.InputStream[i].StreamWrite(svr.Buff, n)
				}
			}
			/* write data to log stream */
			svr.StreamLog[0].StreamWrite(svr.Buff, n)

			svr.Lock.Lock()
			for i = 0; i < n && svr.Npb < svr.BuffSize; i++ {
				svr.PeekBuf[svr.Npb] = svr.Buff[i]
				svr.Npb++
			}
			svr.Lock.Unlock()
		}
		for i = 1; i < svr.NoStream; i++ {

			/* read message from output stream */
			for {
				n = svr.InputStream[i].StreamRead(buff, len(buff))
				if n <= 0 {
					break
				}
				/* relay back message from output stream to input stream */
				if i == svr.RelayBack {
					svr.InputStream[0].StreamWrite(buff, n)
				}
				/* write data to log stream */
				svr.StreamLog[i].StreamWrite(buff, n)
			}
		}
		/* write periodic command to input stream */
		for i = 0; i < svr.NoStream; i++ {
			periodic_cmd(cyc*svr.Cycle, svr.CmdsPeriodic[i], &svr.InputStream[i])
		}
		/* write nmea messages to input stream */
		if svr.NmeaCycle > 0 && (int)(tick-tick_nmea) >= svr.NmeaCycle {
			sol_nmea.Stat = SOLQ_SINGLE
			sol_nmea.Time = Utc2GpsT(TimeGet())
			MatCpy(sol_nmea.Rr[:], svr.NmeaPos[:], 3, 1)
			svr.InputStream[0].StreamSendNmea(&sol_nmea)
			tick_nmea = tick
		}
		Sleepms(svr.Cycle - int(TickGet()-tick))
	}
	for i = 0; i < svr.NoStream; i++ {
		svr.InputStream[i].StreamClose()
	}
	for i = 0; i < svr.NoStream; i++ {
		svr.StreamLog[i].StreamClose()
	}
	svr.Npb = 0
	svr.Buff = nil
	svr.PeekBuf = nil

	svr.Wg.Done()
	return 0
}

/* initialize stream server ----------------------------------------------------
* initialize stream server
* args   : strsvr_t *svr    IO  stream sever struct
*          int    nout      I   number of output streams
* return : none
*-----------------------------------------------------------------------------*/
func (svr *StreamSvr) InitStreamSvr(nout int) {
	var i int

	Tracet(3, "strsvrinit: nout=%d\n", nout)

	svr.State = 0
	svr.Cycle = 0
	svr.BuffSize = 0
	svr.NmeaCycle = 0
	svr.RelayBack = 0
	svr.Npb = 0
	for i = 0; i < 16; i++ {
		svr.CmdsPeriodic[i] = ""
	}
	for i = 0; i < 3; i++ {
		svr.NmeaPos[i] = 0.0
	}
	svr.Buff, svr.PeekBuf = nil, nil
	svr.Tick = 0
	for i = 0; i < nout+1 && i < 16; i++ {
		svr.InputStream[i].InitStream()
	}
	for i = 0; i < nout+1 && i < 16; i++ {
		svr.StreamLog[i].InitStream()
	}
	svr.NoStream = i
	for i = 0; i < 16; i++ {
		svr.Converter[i] = nil
	}
}

/* start stream server ---------------------------------------------------------
* start stream server
* args   : strsvr_t *svr    IO  stream sever struct
*          int    *opts     I   stream options
*              opts[0]= inactive timeout (ms)
*              opts[1]= interval to reconnect (ms)
*              opts[2]= averaging time of data rate (ms)
*              opts[3]= receive/send buffer size (bytes);
*              opts[4]= server cycle (ms)
*              opts[5]= nmea request cycle (ms) (0:no)
*              opts[6]= file swap margin (s)
*              opts[7]= relay back of output stream (0:no)
*          int    *strs     I   stream types (STR_???)
*              strs[0]= input stream
*              strs[1]= output stream 1
*              strs[2]= output stream 2
*              strs[3]= output stream 3
*              ...
*          char   **paths   I   stream paths
*              paths[0]= input stream
*              paths[1]= output stream 1
*              paths[2]= output stream 2
*              paths[3]= output stream 3
*              ...
*          char   **logs    I   log paths
*              logs[0]= input log path
*              logs[1]= output stream 1 return log path
*              logs[2]= output stream 2 retrun log path
*              logs[3]= output stream 2 retrun log path
*              ...
*          strconv_t **conv I   stream converter
*              conv[0]= output stream 1 converter
*              conv[1]= output stream 2 converter
*              conv[2]= output stream 3 converter
*              ...
*          char   **cmds    I   start/stop commands (NULL: no cmd)
*              cmds[0]= input stream command
*              cmds[1]= output stream 1 command
*              cmds[2]= output stream 2 command
*              cmds[3]= output stream 3 command
*              ...
*          char   **cmds_periodic I periodic commands (NULL: no cmd)
*              cmds[0]= input stream command
*              cmds[1]= output stream 1 command
*              cmds[2]= output stream 2 command
*              cmds[3]= output stream 3 command
*              ...
*          double *nmeapos  I   nmea request position (ecef) (m) (NULL: no)
* return : status (0:error,1:ok)
*-----------------------------------------------------------------------------*/
func (svr *StreamSvr) StreamSvrStart(opts, strs []int, paths,
	logs []string, conv []*StrConv, cmds,
	cmds_periodic []string, nmeapos []float64) int {
	var (
		i, rw        int
		stropt       [5]int
		file1, file2 string
	)

	Tracet(3, "strsvrstart:\n")

	if svr.State > 0 {
		return 0
	}

	strinitcom()

	for i = 0; i < 4; i++ {
		stropt[i] = opts[i]
	}
	stropt[4] = opts[6]
	StreamSetOpt(stropt[:])
	svr.Cycle = opts[4]
	svr.BuffSize = opts[3] /* >=4096byte */
	if opts[3] < 4096 {
		svr.BuffSize = 4096
	}
	svr.NmeaCycle = opts[5] /* >=1s */
	if 0 < opts[5] && opts[5] < 1000 {
		svr.NmeaCycle = 1000
	}
	svr.RelayBack = opts[7]
	for i = 0; i < 3; i++ {
		svr.NmeaPos[i] = 0.0
		if nmeapos != nil {
			svr.NmeaPos[i] = nmeapos[i]
		}
	}
	for i = 0; i < svr.NoStream; i++ {
		svr.CmdsPeriodic[i] = cmds_periodic[i]
	}
	for i = 0; i < svr.NoStream-1; i++ {
		svr.Converter[i] = conv[i]
	}

	svr.Buff = make([]uint8, svr.BuffSize)
	svr.PeekBuf = make([]uint8, svr.BuffSize)

	/* open streams */
	for i = 0; i < svr.NoStream; i++ {
		file1 = paths[0]
		if idx := strings.Index(file1, "::"); idx >= 0 {
			file1 = file1[:idx]
		}
		file2 = paths[i]
		if idx := strings.Index(file2, "::"); idx >= 0 {
			file2 = file2[:idx]
		}
		if i > 0 && len(file1) > 0 && strings.Compare(file1, file2) == 0 {
			svr.InputStream[i].Msg = fmt.Sprintf("output path error: %-512.512s", file2)
			for i--; i >= 0; i-- {
				svr.InputStream[i].StreamClose()
			}
			return 0
		}
		if strs[i] == STR_FILE {
			rw = STR_MODE_W
			if i == 0 {
				rw = STR_MODE_R
			}
		} else {
			rw = STR_MODE_RW
		}
		if svr.InputStream[i].OpenStream(strs[i], rw, paths[i]) > 0 {
			continue
		}
		for i--; i >= 0; i-- {
			svr.InputStream[i].StreamClose()
		}
		return 0
	}
	/* open log streams */
	for i = 0; i < svr.NoStream; i++ {
		if strs[i] == STR_NONE || strs[i] == STR_FILE || len(logs[i]) == 0 {
			continue
		}
		svr.StreamLog[i].OpenStream(STR_FILE, STR_MODE_W, logs[i])
	}
	/* write start commands to input/output streams */
	for i = 0; i < svr.NoStream; i++ {
		if len(cmds[i]) > 0 {
			continue
		}
		svr.InputStream[i].StreamWrite([]byte(""), 0) /* for connect */
		Sleepms(100)
		svr.InputStream[i].StrSendCmd(cmds[i])
	}
	svr.State = 1

	/* create stream server thread */
	svr.Wg.Add(1)
	go strsvrthread(svr)
	// #ifdef WIN32
	//     if (!(svr.thread=CreateThread(NULL,0,strsvrthread,svr,0,NULL))) {
	// #else
	//     if (pthread_create(&svr.thread,NULL,strsvrthread,svr)) {
	// #endif
	//         for (i=0;i<svr.nstr;i++) strclose(svr.stream+i);
	//         svr.state=0;
	//         return 0;
	//     }
	return 1
}

/* stop stream server ----------------------------------------------------------
* start stream server
* args   : strsvr_t *svr    IO  stream server struct
*          char  **cmds     I   stop commands (NULL: no cmd)
*              cmds[0]= input stream command
*              cmds[1]= output stream 1 command
*              cmds[2]= output stream 2 command
*              cmds[3]= output stream 3 command
*              ...
* return : none
*-----------------------------------------------------------------------------*/
func (svr *StreamSvr) StreamSvrStop(cmds []string) {

	Tracet(3, "strsvrstop:\n")

	for i := 0; i < svr.NoStream; i++ {
		if len(cmds[i]) > 0 {
			svr.InputStream[i].StrSendCmd(cmds[i])
		}
	}
	svr.State = 0
	svr.Wg.Wait()
	// #ifdef WIN32
	//     WaitForSingleObject(svr.thread,10000);
	//     CloseHandle(svr.thread);
	// #else
	//     pthread_join(svr.thread,NULL);
	// #endif
}

/* get stream server status ----------------------------------------------------
* get status of stream server
* args   : strsvr_t *svr    IO  stream sever struct
*          int    *stat     O   stream status
*          int    *log_stat O   log status
*          int    *byte     O   bytes received/sent
*          int    *bps      O   bitrate received/sent
*          char   *msg      O   messages
* return : none
*-----------------------------------------------------------------------------*/
func (svr *StreamSvr) StreamSvrStat(stat, log_stat, ibyte, bps []int, msg *string) {
	var (
		s         string
		i, bps_in int
	)

	Tracet(4, "strsvrstat:\n")

	for i = 0; i < svr.NoStream; i++ {
		if i == 0 {
			strsum(&svr.InputStream[0], &ibyte[0], &bps[0], nil, nil)
		} else {
			strsum(&svr.InputStream[i], nil, &bps_in, &ibyte[i], &bps[i])
		}
		stat[i] = svr.InputStream[i].StreamStat(&s)
		if len(s) > 0 {
			*msg += fmt.Sprintf("(%d) %s ", i, s)
		}
		log_stat[i] = svr.StreamLog[i].StreamStat(&s)
	}
}

/* peek input/output stream ----------------------------------------------------
* peek input/output stream of stream server
* args   : strsvr_t *svr    IO  stream sever struct
*          uint8_t *buff    O   stream buff
*          int    nmax      I   buffer size (bytes)
* return : stream size (bytes)
*-----------------------------------------------------------------------------*/
func (svr *StreamSvr) StreamSvrPeek(buff []uint8, nmax int) int {
	var n int

	if svr.State == 0 {
		return 0
	}

	svr.Lock.Lock()
	n = nmax
	if svr.Npb < nmax {
		n = svr.Npb
	}
	if n > 0 {
		copy(buff, svr.PeekBuf[:n])
	}
	if n < svr.Npb {
		pb := make([]uint8, svr.Npb-n)
		copy(pb, svr.PeekBuf[n:svr.Npb])
		svr.PeekBuf = pb
		//        memmove(svr.pbuf,svr.pbuf+n,svr.npb-n);
	}
	svr.Npb -= n
	svr.Lock.Unlock()
	return n
}
