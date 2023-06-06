/*------------------------------------------------------------------------------
* preceph.c : precise ephemeris and clock functions
*
*          Copyright (C) 2007-2020 by T.TAKASU, All rights reserved.
*
* references :
*     [1] S.Hilla, The Extended Standard Product 3 Orbit Format (SP3-c),
*         12 February, 2007
*     [2] J.Ray, W.Gurtner, RINEX Extensions to Handle Clock Information,
*         27 August, 1998
*     [3] D.D.McCarthy, IERS Technical Note 21, IERS Conventions 1996, July 1996
*     [4] D.A.Vallado, Fundamentals of Astrodynamics and Applications 2nd ed,
*         Space Technology Library, 2004
*     [5] S.Hilla, The Extended Standard Product 3 Orbit Format (SP3-d),
*         February 21, 2016
*
* version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
* history : 2009/01/18 1.0  new
*           2009/01/31 1.1  fix bug on numerical error to read sp3a ephemeris
*           2009/05/15 1.2  support glonass,galileo,qzs
*           2009/12/11 1.3  support wild-card expansion of file path
*           2010/07/21 1.4  added api:
*                               eci2ecef(),sunmoonpos(),peph2pos(),satantoff(),
*                               readdcb()
*                           changed api:
*                               readsp3()
*                           deleted api:
*                               eph2posp()
*           2010/09/09 1.5  fix problem when precise clock outage
*           2011/01/23 1.6  support qzss satellite code
*           2011/09/12 1.7  fix problem on precise clock outage
*                           move sunmmonpos() to rtkcmn.c
*           2011/12/01 1.8  modify api readsp3()
*                           precede later ephemeris if ephemeris is NULL
*                           move eci2ecef() to rtkcmn.c
*           2013/05/08 1.9  fix bug on computing std-dev of precise clocks
*           2013/11/20 1.10 modify option for api readsp3()
*           2014/04/03 1.11 accept extenstion including sp3,eph,SP3,EPH
*           2014/05/23 1.12 add function to read sp3 velocity records
*                           change api: satantoff()
*           2014/08/31 1.13 add member cov and vco in peph_t sturct
*           2014/10/13 1.14 fix bug on clock error variance in peph2pos()
*           2015/05/10 1.15 add api readfcb()
*                           modify api readdcb()
*           2017/04/11 1.16 fix bug on antenna offset correction in peph2pos()
*           2020/11/30 1.17 support SP3-d [5] to accept more than 85 satellites
*                           support NavIC/IRNSS in API peph2pos()
*                           LC defined GPS/QZS L1-L2, GLO G1-G2, GAL E1-E5b,
*                            BDS B1I-B2I and IRN L5-S for API satantoff()
*                           fix bug on reading SP3 file extension
*		    2022/05/31 1.0  rewrite preceph.c with golang by fxb
*-----------------------------------------------------------------------------*/
package gnssgo

import (
	"bufio"
	"fmt"
	"math"
	"os"
	"sort"
	"strings"
)

const (
	NMAX       = 10    /* order of polynomial interpolation */
	MAXDTE     = 900.0 /* max time difference to ephem time (s) */
	EXTERR_CLK = 1e-3  /* extrapolation error for clock (m/s) */
	EXTERR_EPH = 5e-7 /* extrapolation error for ephem (m/s^2) */)

/* satellite code to satellite system ----------------------------------------*/
func code2sys(code rune) int {
	if code == 'G' || code == ' ' {
		return SYS_GPS
	}
	if code == 'R' {
		return SYS_GLO
	}
	if code == 'E' {
		return SYS_GAL /* SP3-d */
	}
	if code == 'J' {
		return SYS_QZS /* SP3-d */
	}
	if code == 'C' {
		return SYS_CMP /* SP3-d */
	}
	if code == 'I' {
		return SYS_IRN /* SP3-d */
	}
	if code == 'L' {
		return SYS_LEO /* SP3-d */
	}
	return SYS_NONE
}

/* read SP3 header -----------------------------------------------------------*/
func ReadSp3Header(rd *bufio.Reader, time *Gtime, ctype *string, sats []int, bfact []float64, tsys *string) int {
	var (
		i, j, k, ns, sys, prn int
		buff                  string
		prefix                []byte
		err                   error
	)

	Trace(4, "readsp3h:\n")

	for i = 0; ; i++ {
		prefix, err = rd.Peek(2)
		if err != nil {
			break
		}
		if strings.Compare(string(prefix), "* ") == 0 { /* first record */
			/* roll back file pointer */
			//	fp.Seek(int64(-len(buff)), 1)

			break
		}
		buff, _ = rd.ReadString('\n')
		if len(buff) == 0 {
			break
		}
		switch {
		case i == 0:
			*ctype = string(buff[2])
			if Str2Time(buff, 3, 28, time) > 0 {
				return 0
			}
		case string(prefix) == "+ ": /* satellite id */
			if ns == 0 {
				ns = int(Str2Num(buff, 3, 3)) // support multi-satellite
			}
			for j = 0; j < 17 && k < ns; j++ {
				sys = code2sys(rune(buff[9+3*j]))
				prn = int(Str2Num(buff, 10+3*j, 2))
				if k < MAXSAT {
					sats[k] = SatNo(sys, prn)
					k++
				}
			}
		case string(prefix) == "++": /* orbit accuracy */
			continue
		case string(prefix) == "%c": /* time system */
			*tsys = buff[9:12]
		case string(prefix) == "%f" && bfact[0] == 0.0: /* fp base number */
			bfact[0] = Str2Num(buff, 3, 10)
			bfact[1] = Str2Num(buff, 14, 12)
		case string(prefix) == "%i":
			continue
		case string(prefix) == "/*": /* comment */
			continue
		}
	}
	return ns
}

/* add precise ephemeris -----------------------------------------------------*/
func (nav *Nav) AddPEph(peph *PEph) int {
	nav.Peph = append(nav.Peph, *peph)
	return 1
}

/* read SP3 body -------------------------------------------------------------*/
func (nav *Nav) ReadSp3Body(rd *bufio.Reader, ctype rune, sats []int, ns int, bfact []float64,
	tsys *string, index, opt int) {
	var (
		peph                                      PEph
		time                                      Gtime
		val, std, base                            float64
		i, j, sat, sys, prn, n, pred_o, pred_c, v int
	)
	if ctype == 'P' {
		n = ns
	} else {
		n = ns * 2
	}
	var buff string

	Trace(4, "readsp3b: type=%c ns=%d index=%d opt=%d\n", ctype, ns, index, opt)

	for {
		buff, _ = rd.ReadString('\n')
		if len(buff) == 0 {
			break
		}
		if strings.Compare(buff[0:3], "EOF") == 0 {
			break
		}

		if buff[0] != '*' || Str2Time(buff, 3, 28, &time) > 0 {
			Trace(2, "sp3 invalid epoch %31.31s\n", buff)
			continue
		}
		if strings.Compare(*tsys, "UTC") == 0 {
			time = Utc2GpsT(time) /* utc.gpst */
		}
		peph.Time = time
		peph.Index = index

		for i = 0; i < len(peph.Pos); i++ {
			for j = 0; j < 4; j++ {
				peph.Pos[i][j] = 0.0
				peph.Std[i][j] = 0.0
				peph.Vel[i][j] = 0.0
				peph.Vst[i][j] = 0.0
			}
			for j = 0; j < 3; j++ {
				peph.PosCov[i][j] = 0.0
				peph.VelCov[i][j] = 0.0
			}
		}
		for i, pred_o, pred_c, v = 0, 0, 0, 0; i < n; i++ {
			buff, _ = rd.ReadString('\n')
			if len(buff) == 0 {
				break
			}
			if len(buff) < 4 || (buff[0] != 'P' && buff[0] != 'V') {
				continue
			}
			sys = code2sys(rune(buff[1]))
			if buff[1] == ' ' {
				sys = SYS_GPS
			}
			prn = int(Str2Num(buff, 2, 2))
			if sys == SYS_SBS {
				prn += 100
			} else if sys == SYS_QZS {
				prn += 192 /* extension to sp3-c */
			}

			if sat = SatNo(sys, prn); sat == 0 {
				continue
			}

			if buff[0] == 'P' {
				pred_c = 0
				if len(buff) >= 76 && buff[75] == 'P' {
					pred_c = 1
				}
				pred_o = 0
				if len(buff) >= 80 && buff[79] == 'P' {
					pred_o = 1
				}
			}
			for j = 0; j < 4; j++ {

				/* read option for predicted value */
				if j < 3 && (opt&1) > 0 && pred_o > 0 {
					continue
				}
				if j < 3 && (opt&2) > 0 && pred_o == 0 {
					continue
				}
				if j == 3 && (opt&1) > 0 && pred_c > 0 {
					continue
				}
				if j == 3 && (opt&2) > 0 && pred_c == 0 {
					continue
				}

				val = Str2Num(buff, 4+j*14, 14)
				if j < 3 {
					std = Str2Num(buff, 61+j*3, 2)

				} else {
					std = Str2Num(buff, 61+j*3, 3)
				}

				if buff[0] == 'P' { /* position */
					if val != 0.0 && math.Abs(val-999999.999999) >= 1e-6 {
						if j < 3 {
							peph.Pos[sat-1][j] = val * 1000.0
						} else {
							peph.Pos[sat-1][j] = val * (1e-6)
							v = 1 /* valid epoch */
						}
					}
					jid := 1
					if j < 3 {
						jid = 0
					}
					base = bfact[jid]
					jvar := 1e-12
					if j < 3 {
						jvar = 1e-3
					}
					if base > 0.0 && std > 0.0 {
						peph.Std[sat-1][j] = float32(math.Pow(base, std) * (jvar))
					}
				} else if v > 0 { /* velocity */
					if val != 0.0 && math.Abs(val-999999.999999) >= 1e-6 {
						jvar := 1e-10
						if j < 3 {
							jvar = 0.1
						}
						peph.Vel[sat-1][j] = val * jvar
					}
					jid := 1
					if j < 3 {
						jid = 0
					}
					base = bfact[jid]
					if base > 0.0 && std > 0.0 {
						jvar := 1e-16
						if j < 3 {
							jvar = 1e-7
						}
						peph.Vst[sat-1][j] = float32(math.Pow(base, std) * jvar)
					}
				}
			}
		}
		if v > 0 {
			if nav.AddPEph(&peph) == 0 {
				return
			}
		}
	}
}

/* compare precise ephemeris -------------------------------------------------*/
func cmppeph(p1, p2 *PEph) int {
	tt := TimeDiff(p1.Time, p2.Time)
	if tt < -1e-9 {
		return -1
	} else if tt > 1e-9 {
		return 1
	} else {
		return p1.Index - p2.Index
	}
}

/* combine precise ephemeris -------------------------------------------------*/
func (nav *Nav) CombPEph(opt int) {
	var i, j, k, m int

	Trace(4, "combpeph: ne=%d\n", nav.Ne())

	sort.Slice(nav.Peph, func(i, j int) bool {
		return cmppeph(&nav.Peph[i], &nav.Peph[j]) < 0
	})

	if opt&4 > 0 {
		return
	}

	for i, j = 0, 1; j < nav.Ne(); j++ {

		if math.Abs(TimeDiff(nav.Peph[i].Time, nav.Peph[j].Time)) < 1e-9 {

			for k = 0; k < MAXSAT; k++ {
				if Norm(nav.Peph[j].Pos[k][:], 4) <= 0.0 {
					continue
				}
				for m = 0; m < 4; m++ {
					nav.Peph[i].Pos[k][m] = nav.Peph[j].Pos[k][m]
				}
				for m = 0; m < 4; m++ {
					nav.Peph[i].Std[k][m] = nav.Peph[j].Std[k][m]
				}
				for m = 0; m < 4; m++ {
					nav.Peph[i].Vel[k][m] = nav.Peph[j].Vel[k][m]
				}
				for m = 0; m < 4; m++ {
					nav.Peph[i].Vst[k][m] = nav.Peph[j].Vst[k][m]
				}
			}
		} else if i++; i < j {
			nav.Peph[i] = nav.Peph[j]
		}
	}
	nav.Peph = nav.Peph[:i+1]
	// nav.Ne = i + 1

	Trace(5, "combpeph: ne=%d\n", nav.Ne())
}

/* read sp3 precise ephemeris file ---------------------------------------------
* read sp3 precise ephemeris/clock files and set them to navigation data
* args   : char   *file       I   sp3-c precise ephemeris file
*                                 (wind-card * is expanded)
*          nav_t  *nav        IO  navigation data
*          int    opt         I   options (1: only observed + 2: only predicted +
*                                 4: not combined)
* return : none
* notes  : see ref [1]
*          precise ephemeris is appended and combined
*          nav.peph and nav.ne must by properly initialized before calling the
*          function
*          only files with extensions of .sp3, .SP3, .eph* and .EPH* are read
*-----------------------------------------------------------------------------*/
func (nav *Nav) ReadSp3(file string, opt int) {
	var (
		fp                 *os.File
		time               Gtime
		bfact              [2]float64
		i, j, n, ns, index int
		sats               []int    = make([]int, MAXSAT)
		efiles             []string = make([]string, MAXEXFILE)
		ctype, tsys        string
		err                error
	)

	Trace(4, "readpephs: file=%s\n", file)

	/* expand wild card in file path */
	n = ExPath(file, efiles, MAXEXFILE)

	for i, j = 0, 0; i < n; i++ {
		if index = strings.LastIndex(efiles[i], "."); index < 0 {
			continue
		}
		ext := efiles[i][index:]
		if !strings.EqualFold(ext, ".sp3") && !strings.EqualFold(ext, ".phh") {
			continue
		}

		if fp, err = os.OpenFile(efiles[i], os.O_RDONLY, 0666); err != nil {
			Trace(2, "sp3 file open error %s\n", efiles[i])
			continue
		}
		defer fp.Close()

		rd := bufio.NewReader(fp)
		/* read sp3 header */
		ns = ReadSp3Header(rd, &time, &ctype, sats, bfact[:], &tsys)

		/* read sp3 body */
		nav.ReadSp3Body(rd, rune(ctype[0]), sats, ns, bfact[:], &tsys, j, opt)
		j++

	}

	/* combine precise ephemeris */
	if nav.Ne() > 0 {
		nav.CombPEph(opt)
	}
}

/* read satellite antenna parameters -------------------------------------------
* read satellite antenna parameters
* args   : char   *file       I   antenna parameter file
*          gtime_t time       I   time
*          nav_t  *nav        IO  navigation data
* return : status (1:ok,0:error)
* notes  : only support antex format for the antenna parameter file
*-----------------------------------------------------------------------------*/
func (nav *Nav) ReadSAP(file string, time Gtime) int {
	var (
		pcvs Pcvs
		pcv0 Pcv
		pcv  *Pcv
	)

	Trace(4, "readsap : file=%s time=%s\n", file, TimeStr(time, 0))

	if ReadPcv(file, &pcvs) == 0 {
		return 0
	}

	for i := 0; i < len(nav.Pcvs); i++ {
		pcv = SearchPcv(i+1, "", time, &pcvs)
		if pcv != nil {
			nav.Pcvs[i] = *pcv
		} else {
			nav.Pcvs[i] = pcv0
		}
	}

	return 1
}

/* read DCB parameters file --------------------------------------------------*/
func (nav *Nav) ReadDcbF(file string, sta []Sta) int {
	var (
		fp                  *os.File
		cbias               float64
		buff, str1, str2    string
		i, j, sat, ctype, n int
		err                 error
	)

	Trace(4, "readdcbf: file=%s\n", file)

	if fp, err = os.OpenFile(file, os.O_RDONLY, 0666); err != nil {
		Trace(2, "dcb parameters file open error: %s\n", file)
		return 0
	}
	defer fp.Close()

	rd := bufio.NewReader(fp)
	for {
		buff, err = rd.ReadString('\n')
		if err != nil {
			break
		}
		if strings.Contains(buff, "DIFFERENTIAL (P1-P2) CODE BIASES") {
			ctype = 1
		} else if strings.Contains(buff, "DIFFERENTIAL (P1-C1) CODE BIASES") {
			ctype = 2
		} else if strings.Contains(buff, "DIFFERENTIAL (P2-C2) CODE BIASES") {
			ctype = 3
		}

		n, _ = fmt.Sscanf(buff, "%s %s", &str1, &str2)
		if ctype > 0 || n < 1 {
			continue
		}

		if cbias = Str2Num(buff, 26, 9); cbias == 0.0 {
			continue
		}

		if sta != nil && (strings.Compare(str1, "G") == 0 || strings.Compare(str1, "R") == 0) { /* receiver DCB */
			for i = 0; i < len(sta); i++ {
				if strings.Compare(sta[i].Name, str2) == 0 {
					break
				}
			}
			if i < len(sta) {
				j = 1
				if strings.Compare(str1, "G") == 0 {
					j = 0
				}
				nav.RBias[i][j][ctype-1] = cbias * (1e-9) * CLIGHT /* ns . m */
			}
		} else if sat = SatId2No(str1); sat > 0 { /* satellite dcb */
			nav.CBias[sat-1][ctype-1] = cbias * 1e-9 * CLIGHT /* ns . m */
		}
	}

	return 1
}

/* read DCB parameters ---------------------------------------------------------
* read differential code bias (DCB) parameters
* args   : char   *file       I   DCB parameters file (wild-card * expanded)
*          nav_t  *nav        IO  navigation data
*          sta_t  *sta        I   station info data to inport receiver DCB
*                                 (NULL: no use)
* return : status (1:ok,0:error)
* notes  : currently only support P1-P2, P1-C1, P2-C2, bias in DCB file
*-----------------------------------------------------------------------------*/
func (nav *Nav) ReadDcb(file string, sta []Sta) int {
	var (
		i, j, n int
		efiles  []string = make([]string, MAXEXFILE)
	)

	Trace(4, "readdcb : file=%s\n", file)

	for _, v := range nav.CBias {
		for j = 0; j < 3; j++ {
			v[j] = 0.0
		}
	}

	n = ExPath(file, efiles, MAXEXFILE)

	for i = 0; i < n; i++ {
		nav.ReadDcbF(efiles[i], sta)
	}

	return 1
}

/* polynomial interpolation by Neville's algorithm ---------------------------*/
func InterpPol(x, y []float64, n int) float64 {
	for j := 1; j < n; j++ {
		for i := 0; i < n-j; i++ {
			y[i] = (x[i+j]*y[i] - x[i]*y[i+1]) / (x[i+j] - x[i])
		}
	}
	return y[0]
}

/* satellite position by precise ephemeris -----------------------------------*/
func (nav *Nav) PEphPos(time Gtime, sat int, rs, dts []float64, vare, varc *float64) int {
	var (
		t               [NMAX + 1]float64
		p               [3][NMAX + 1]float64
		c               [2]float64
		pos             []float64
		std, sinl, cosl float64
		s               [3]float64
		i, j, k, index  int
	)

	Trace(4, "pephpos : time=%s sat=%2d\n", TimeStr(time, 3), sat)

	rs[0], rs[1], rs[2], dts[0] = 0.0, 0.0, 0.0, 0.0

	if nav.Ne() < NMAX+1 || TimeDiff(time, nav.Peph[0].Time) < -MAXDTE || TimeDiff(time, nav.Peph[nav.Ne()-1].Time) > MAXDTE {
		Trace(2, "no prec ephem %s sat=%2d\n", TimeStr(time, 0), sat)
		return 0
	}
	/* binary search */
	for i, j = 0, nav.Ne()-1; i < j; {
		k = (i + j) / 2
		if TimeDiff(nav.Peph[k].Time, time) < 0.0 {
			i = k + 1
		} else {
			j = k
		}
	}
	index = 0
	if i > 0 {
		index = i - 1
	}

	/* polynomial interpolation for orbit */
	i = index - (NMAX+1)/2
	if i < 0 {
		i = 0
	} else if i+NMAX >= nav.Ne() {
		i = nav.Ne() - NMAX - 1
	}

	for j = 0; j <= NMAX; j++ {
		t[j] = TimeDiff(nav.Peph[i+j].Time, time)
		if Norm(nav.Peph[i+j].Pos[sat-1][:], 3) <= 0.0 {
			Trace(2, "prec ephem outage %s sat=%2d\n", TimeStr(time, 0), sat)
			return 0
		}
	}
	for j = 0; j <= NMAX; j++ {
		pos = nav.Peph[i+j].Pos[sat-1][:]
		/* correciton for earh rotation ver.2.4.0 */
		sinl = math.Sin(OMGE * t[j])
		cosl = math.Cos(OMGE * t[j])
		p[0][j] = cosl*pos[0] - sinl*pos[1]
		p[1][j] = sinl*pos[0] + cosl*pos[1]
		p[2][j] = pos[2]
	}
	for i = 0; i < 3; i++ {
		rs[i] = InterpPol(t[:], p[i][:], NMAX+1)
	}
	if vare != nil {
		for i = 0; i < 3; i++ {
			s[i] = float64(nav.Peph[index].Std[sat-1][i])
		}
		std = Norm(s[:], 3)

		/* extrapolation error for orbit */
		if t[0] > 0.0 {
			std += EXTERR_EPH * SQR(t[0]) / 2.0
		} else if t[NMAX] < 0.0 {
			std += EXTERR_EPH * SQR(t[NMAX]) / 2.0
		}
		*vare = SQR(std)
	}
	/* linear interpolation for clock */
	t[0] = TimeDiff(time, nav.Peph[index].Time)
	t[1] = TimeDiff(time, nav.Peph[index+1].Time)
	c[0] = nav.Peph[index].Pos[sat-1][3]
	c[1] = nav.Peph[index+1].Pos[sat-1][3]

	switch {
	case t[0] <= 0.0:
		if dts[0] = c[0]; dts[0] != 0.0 {
			std = float64(nav.Peph[index].Std[sat-1][3])*CLIGHT - EXTERR_CLK*t[0]
		}
	case t[1] >= 0.0:
		if dts[0] = c[1]; dts[0] != 0.0 {
			std = float64(nav.Peph[index+1].Std[sat-1][3])*CLIGHT + EXTERR_CLK*t[1]
		}
	case c[0] != 0.0 && c[1] != 0.0:
		dts[0] = (c[1]*t[0] - c[0]*t[1]) / (t[0] - t[1])
		i = 1
		if t[0] < -t[1] {
			i = 0
		}

		std = float64(nav.Peph[index+i].Std[sat-1][3]) + EXTERR_CLK*math.Abs(t[i])
	default:
		dts[0] = 0.0
	}
	if varc != nil {
		*varc = SQR(std)
	}
	return 1
}

/* satellite clock by precise clock ------------------------------------------*/
func (nav *Nav) PEphClk(time Gtime, sat int, dts []float64, varc *float64) int {
	var (
		t, c           [2]float64
		std            float64
		i, j, k, index int
	)

	Trace(4, "pephclk : time=%s sat=%2d\n", TimeStr(time, 3), sat)

	if nav.Nc() < 2 ||
		TimeDiff(time, nav.Pclk[0].Time) < -MAXDTE ||
		TimeDiff(time, nav.Pclk[nav.Nc()-1].Time) > MAXDTE {
		Trace(2, "no prec clock %s sat=%2d\n", TimeStr(time, 0), sat)
		return 1
	}
	/* binary search */
	for i, j = 0, nav.Nc()-1; i < j; {
		k = (i + j) / 2
		if TimeDiff(nav.Pclk[k].Time, time) < 0.0 {
			i = k + 1
		} else {
			j = k
		}
	}
	index = i - 1
	if i <= 0 {
		index = 0
	}

	/* linear interpolation for clock */
	t[0] = TimeDiff(time, nav.Pclk[index].Time)
	t[1] = TimeDiff(time, nav.Pclk[index+1].Time)
	c[0] = nav.Pclk[index].Clk[sat-1][0]
	c[1] = nav.Pclk[index+1].Clk[sat-1][0]

	switch {
	case t[0] <= 0.0:
		if dts[0] = c[0]; dts[0] == 0.0 {
			return 0
		}
		std = float64(nav.Pclk[index].Std[sat-1][0])*CLIGHT - EXTERR_CLK*t[0]
	case t[1] >= 0.0:
		if dts[0] = c[1]; dts[0] == 0.0 {
			return 0
		}
		std = float64(nav.Pclk[index+1].Std[sat-1][0])*CLIGHT + EXTERR_CLK*t[1]
	case c[0] != 0.0 && c[1] != 0.0:
		dts[0] = (c[1]*t[0] - c[0]*t[1]) / (t[0] - t[1])
		i = 1
		if t[0] < -t[1] {
			i = 0
		}
		std = float64(nav.Pclk[index+i].Std[sat-1][0])*CLIGHT + EXTERR_CLK*math.Abs(t[i])
	default:
		Trace(2, "prec clock outage %s sat=%2d\n", TimeStr(time, 0), sat)
		return 0
	}
	if varc != nil {
		*varc = SQR(std)
	}
	return 1
}

/* satellite antenna phase center offset ---------------------------------------
* compute satellite antenna phase center offset in ecef
* args   : gtime_t time       I   time (gpst)
*          double *rs         I   satellite position and velocity (ecef)
*                                 {x,y,z,vx,vy,vz} (m|m/s)
*          int    sat         I   satellite number
*          nav_t  *nav        I   navigation data
*          double *dant       I   satellite antenna phase center offset (ecef)
*                                 {dx,dy,dz} (m) (iono-free LC value)
* return : none
* notes  : iono-free LC frequencies defined as follows:
*            GPS/QZSS : L1-L2
*            GLONASS  : G1-G2
*            Galileo  : E1-E5b
*            BDS      : B1I-B2I
*            NavIC    : L5-S
*-----------------------------------------------------------------------------*/
func (nav *Nav) SatAntOffset(time Gtime, rs []float64, sat int, dant []float64) {
	var (
		pcv = nav.Pcvs[sat]

		ex, ey, ez, es, r, rsun    [3]float64
		gmst, C1, C2, dant1, dant2 float64
		erpv                       [5]float64
		freq                       [2]float64

		i, sys int
	)

	Trace(4, "satantoff: time=%s sat=%2d\n", TimeStr(time, 3), sat)

	dant[0], dant[1], dant[2] = 0.0, 0.0, 0.0

	/* sun position in ecef */
	SunMoonPos(GpsT2Utc(time), erpv[:], rsun[:], nil, &gmst)

	/* unit vectors of satellite fixed coordinates */
	for i = 0; i < 3; i++ {
		r[i] = -rs[i]
	}
	if NormV3(r[:], ez[:]) == 0 {
		return
	}
	for i = 0; i < 3; i++ {
		r[i] = rsun[i] - rs[i]
	}
	if NormV3(r[:], es[:]) == 0 {
		return
	}
	Cross3(ez[:], es[:], r[:])
	if NormV3(r[:], ey[:]) == 0 {
		return
	}
	Cross3(ey[:], ez[:], ex[:])

	/* iono-free LC coefficients */
	sys = SatSys(sat, nil)
	switch sys {
	case SYS_GPS, SYS_QZS: /* L1-L2 */
		freq[0] = FREQ1
		freq[1] = FREQ2
	case SYS_GLO: /* G1-G2 */
		freq[0] = Sat2Freq(sat, CODE_L1C, nav)
		freq[1] = Sat2Freq(sat, CODE_L2C, nav)
	case SYS_GAL: /* E1-E5b */
		freq[0] = FREQ1
		freq[1] = FREQ7
	case SYS_CMP: /* B1I-B2I */
		freq[0] = FREQ1_CMP
		freq[1] = FREQ2_CMP
	case SYS_IRN: /* B1I-B2I */
		freq[0] = FREQ5
		freq[1] = FREQ9
	default:
		return
	}

	if freq[0] != freq[1] {
		C1 = SQR(freq[0]) / (SQR(freq[0]) - SQR(freq[1]))
		C2 = -SQR(freq[1]) / (SQR(freq[0]) - SQR(freq[1]))
	} else {
		C1 = 1
		C2 = 1
	}

	/* iono-free LC */
	for i = 0; i < 3; i++ {
		dant1 = pcv.Offset[0][0]*ex[i] + pcv.Offset[0][1]*ey[i] + pcv.Offset[0][2]*ez[i]
		dant2 = pcv.Offset[1][0]*ex[i] + pcv.Offset[1][1]*ey[i] + pcv.Offset[1][2]*ez[i]
		dant[i] = C1*dant1 + C2*dant2
	}
}

/* satellite position/clock by precise ephemeris/clock -------------------------
* compute satellite position/clock with precise ephemeris/clock
* args   : gtime_t time       I   time (gpst)
*          int    sat         I   satellite number
*          nav_t  *nav        I   navigation data
*          int    opt         I   sat postion option
*                                 (0: center of mass, 1: antenna phase center)
*          double *rs         O   sat position and velocity (ecef)
*                                 {x,y,z,vx,vy,vz} (m|m/s)
*          double *dts        O   sat clock {bias,drift} (s|s/s)
*          double *var        IO  sat position and clock error variance (m)
*                                 (NULL: no output)
* return : status (1:ok,0:error or data outage)
* notes  : clock includes relativistic correction but does not contain code bias
*          before calling the function, nav.peph, nav.ne, nav.pclk and
*          nav.nc must be set by calling readsp3(), readrnx() or readrnxt()
*          if precise clocks are not set, clocks in sp3 are used instead
*-----------------------------------------------------------------------------*/
func (nav *Nav) PEph2Pos(time Gtime, sat int, opt int,
	rs, dts []float64, vari *float64) int {
	var (
		time_tt        Gtime
		rss, rst, dant [3]float64
		dtss, dtst     [1]float64
		vare, varc, tt float64 = 0.0, 0.0, 1e-3
		i              int
	)

	Trace(3, "peph2pos: time=%s sat=%2d opt=%d\n", TimeStr(time, 3), sat, opt)

	if sat <= 0 || MAXSAT < sat {
		return 0
	}

	/* satellite position and clock bias */
	if nav.PEphPos(time, sat, rss[:], dtss[:], &vare, &varc) == 0 ||
		nav.PEphClk(time, sat, dtss[:], &varc) == 0 {
		return 0
	}

	time_tt = TimeAdd(time, tt)
	if nav.PEphPos(time_tt, sat, rst[:], dtst[:], nil, nil) == 0 ||
		nav.PEphClk(time_tt, sat, dtst[:], nil) == 0 {
		return 0
	}

	/* satellite antenna offset correction */
	if opt > 0 {
		nav.SatAntOffset(time, rss[:], sat, dant[:])
	}
	for i = 0; i < 3; i++ {
		rs[i] = rss[i] + dant[i]
		rs[i+3] = (rst[i] - rss[i]) / tt
	}
	/* relativistic effect correction */
	if dtss[0] != 0.0 {
		dts[0] = dtss[0] - 2.0*Dot(rs, rs[3:], 3)/CLIGHT/CLIGHT
		dts[1] = (dtst[0] - dtss[0]) / tt
	} else { /* no precise clock */
		dts[0], dts[1] = 0.0, 0.0
	}
	if vari != nil {
		*vari = vare + varc
	}

	return 1
}
