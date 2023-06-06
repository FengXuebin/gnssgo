/*------------------------------------------------------------------------------
* ionex.c : ionex functions
*
*          Copyright (C) 2011-2013 by T.TAKASU, All rights reserved.
*
* references:
*     [1] S.Schear, W.Gurtner and J.Feltens, IONEX: The IONosphere Map EXchange
*         Format Version 1, February 25, 1998
*     [2] S.Schaer, R.Markus, B.Gerhard and A.S.Timon, Daily Global Ionosphere
*         Maps based on GPS Carrier Phase Data Routinely producted by CODE
*         Analysis Center, Proceeding of the IGS Analysis Center Workshop, 1996
*
* version : $Revision:$ $Date:$
* history : 2011/03/29 1.0 new
*           2013/03/05 1.1 change api readtec()
*                          fix problem in case of lat>85deg or lat<-85deg
*           2014/02/22 1.2 fix problem on compiled as C++
*		    2022/05/31 1.0  rewrite ionex.c with golang by fxb
*-----------------------------------------------------------------------------*/

package gnssgo

import (
	"bufio"
	"math"
	"os"
	"strings"
)

const (
	VAR_NOTEC = 30.0 * 30.0 /* variance of no tec */
	ZERO_EL   = 0.0         /* min elevation angle (rad) */
	MIN_HGT   = -1000.0 /* min user height (m) */)

/* get index -----------------------------------------------------------------*/
func getindex(value float64, range_ []float64) int {
	if range_[2] == 0.0 {
		return 0
	}
	if range_[1] > 0.0 && (value < range_[0] || range_[1] < value) {
		return -1
	}
	if range_[1] < 0.0 && (value < range_[1] || range_[0] < value) {
		return -1
	}
	return int(math.Floor((value-range_[0])/range_[2] + 0.5))
}

/* get number of items -------------------------------------------------------*/
func nitem(range_ []float64) int {
	return getindex(range_[1], range_) + 1
}

/* data index (i:lat,j:lon,k:hgt) --------------------------------------------*/
func dataindex(i, j, k int, ndata []int) int {
	if i < 0 || ndata[0] <= i || j < 0 || ndata[1] <= j || k < 0 || ndata[2] <= k {
		return -1
	}
	return i + ndata[0]*(j+ndata[1]*k)
}

/* add tec data to navigation data -------------------------------------------*/
func (nav *Nav) AddTec(lats, lons, hgts []float64, rb float64) *Tec {
	var (
		data  Tec
		time0 Gtime
		i, n  int
		ndata [3]int
	)

	Trace(4, "addtec  :\n")

	ndata[0] = nitem(lats[:])
	ndata[1] = nitem(lons[:])
	ndata[2] = nitem(hgts[:])
	if ndata[0] <= 1 || ndata[1] <= 1 || ndata[2] <= 0 {
		return nil
	}
	data.Time = time0
	data.Radius = rb
	for i = 0; i < 3; i++ {
		data.Ndata[i] = ndata[i]
		data.Lats[i] = lats[i]
		data.Lons[i] = lons[i]
		data.Hgts[i] = hgts[i]
	}
	n = ndata[0] * ndata[1] * ndata[2]
	data.Data = make([]float64, n)
	data.Rms = make([]float32, n)

	for i = 0; i < n; i++ {
		data.Data[i] = 0.0
		data.Rms[i] = 0.0
	}
	nav.Tec = append(nav.Tec, data)
	return &nav.Tec[nav.Nt()-1]
}

/* read ionex dcb aux data ----------------------------------------------------*/
func ReadIonexDcb(rd *bufio.Reader, dcb, rms []float64) {
	var (
		i, sat int
		buff   string
		err    error
	)
	Trace(4, "readionexdcb:\n")

	for i = 0; i < MAXSAT; i++ {
		dcb[i], rms[i] = 0.0, 0.0
	}

	for {
		buff, err = rd.ReadString('\n')
		if err != nil {
			break
		}
		label := buff[60:]
		if index := strings.Index(label, "PRN / BIAS / RMS"); index == 0 {
			id := buff[3:6]
			if sat = SatId2No(id); sat == 0 {
				Trace(2, "ionex invalid satellite: %s\n", id)
				continue
			}
			dcb[sat-1] = Str2Num(buff, 6, 10)
			rms[sat-1] = Str2Num(buff, 16, 10)
		} else if index := strings.Index(label, "END OF AUX DATA"); index == 0 {
			break
		}
	}
}

/* read ionex header ---------------------------------------------------------*/
func ReadIonexHeader(rd *bufio.Reader, lats, lons, hgts []float64, rb, nexp *float64, dcb, rms []float64) float64 {
	var (
		ver  float64 = 0.0
		buff string
		err  error
	)

	Trace(4, "readionexh:\n")
	//	rd := bufio.NewReader(fp)
	for {
		buff, err = rd.ReadString('\n')
		if err != nil {
			break
		}
		label := buff[60:]
		switch {
		case strings.Index(label, "IONEX VERSION / TYPE") == 0:
			if buff[20] == 'I' {
				ver = Str2Num(buff, 0, 8)
			}
		case strings.Index(label, "BASE RADIUS") == 0:
			*rb = Str2Num(buff, 0, 8)
		case strings.Index(label, "HGT1 / HGT2 / DHGT") == 0:
			hgts[0] = Str2Num(buff, 2, 6)
			hgts[1] = Str2Num(buff, 8, 6)
			hgts[2] = Str2Num(buff, 14, 6)
		case strings.Index(label, "LAT1 / LAT2 / DLAT") == 0:
			lats[0] = Str2Num(buff, 2, 6)
			lats[1] = Str2Num(buff, 8, 6)
			lats[2] = Str2Num(buff, 14, 6)
		case strings.Index(label, "LON1 / LON2 / DLON") == 0:
			lons[0] = Str2Num(buff, 2, 6)
			lons[1] = Str2Num(buff, 8, 6)
			lons[2] = Str2Num(buff, 14, 6)
		case strings.Index(label, "EXPONENT") == 0:
			*nexp = Str2Num(buff, 0, 6)
		case strings.Index(label, "START OF AUX DATA") == 0 && strings.Index(buff, "DIFFERENTIAL CODE BIASES") > 0:
			ReadIonexDcb(rd, dcb, rms)
		case strings.Index(label, "END OF HEADER") == 0:
			return ver
		}
	}
	return 0.0
}

/* read ionex body -----------------------------------------------------------*/
func ReadIonexBody(rd *bufio.Reader, lats, lons, hgts []float64, rb, nexp float64, nav *Nav) int {
	var (
		p                           *Tec = nil
		time                        Gtime
		lat, hgt, x                 float64
		lon                         [3]float64
		i, j, k, n, m, index, dtype int
		buff                        string
		err                         error
	)

	Trace(4, "readionexb:\n")
	for {
		buff, err = rd.ReadString('\n')
		if err != nil {
			break
		}

		if len(buff) < 60 {
			continue
		}

		label := buff[60:]
		switch {
		case strings.Index(label, "START OF TEC MAP") == 0:
			if p = nav.AddTec(lats, lons, hgts, rb); p != nil {
				dtype = 1
			}
		case strings.Index(label, "END OF TEC MAP") == 0:
			dtype = 0
			p = nil
		case strings.Index(label, "START OF RMS MAP") == 0:
			dtype = 2
			p = nil
		case strings.Index(label, "END OF RMS MAP") == 0:
			dtype = 0
			p = nil
		case strings.Index(label, "EPOCH OF CURRENT MAP") == 0:
			if Str2Time(buff, 0, 36, &time) > 0 {
				Trace(2, "ionex epoch invalid: %-36.36s\n", buff)
				continue
			}
			if dtype == 2 {
				for i = nav.Nt() - 1; i >= 0; i-- {
					if math.Abs(TimeDiff(time, nav.Tec[i].Time)) >= 1.0 {
						continue
					}
					p = &nav.Tec[i]
					break
				}
			} else if p != nil {
				p.Time = time
			}
		case strings.Index(label, "LAT/LON1/LON2/DLON/H") == 0 && p != nil:
			lat = Str2Num(buff, 2, 6)
			lon[0] = Str2Num(buff, 8, 6)
			lon[1] = Str2Num(buff, 14, 6)
			lon[2] = Str2Num(buff, 20, 6)
			hgt = Str2Num(buff, 26, 6)

			i = getindex(lat, p.Lats[:])
			k = getindex(hgt, p.Hgts[:])
			n = nitem(lon[:])

			for m = 0; m < n; m++ {
				if m%16 == 0 {
					buff, _ = rd.ReadString('\n')
					if len(buff) == 0 {
						break
					}
				}

				j = getindex(lon[0]+lon[2]*float64(m), p.Lons[:])
				if index = dataindex(i, j, k, p.Ndata[:]); index < 0 {
					continue
				}

				if x = Str2Num(buff, m%16*5, 5); x == 9999.0 {
					continue
				}

				if dtype == 1 {
					p.Data[index] = x * math.Pow(10.0, nexp)
				} else {
					p.Rms[index] = float32(x * math.Pow(10.0, nexp))
				}
			}
		}
	}
	return 1
}

/* combine tec grid data -----------------------------------------------------*/
func (nav *Nav) CombineTec() {
	var (
		tmp     Tec
		i, j, n int
	)

	Trace(4, "combtec : nav.nt=%d\n", nav.Nt)

	for i = 0; i < nav.Nt()-1; i++ {
		for j = i + 1; j < nav.Nt(); j++ {
			if TimeDiff(nav.Tec[j].Time, nav.Tec[i].Time) < 0.0 {
				tmp = nav.Tec[i]
				nav.Tec[i] = nav.Tec[j]
				nav.Tec[j] = tmp
			}
		}
	}
	for i = 0; i < nav.Nt(); i++ {
		if i > 0 && TimeDiff(nav.Tec[i].Time, nav.Tec[n-1].Time) == 0.0 {
			nav.Tec[n-1].Data = nil
			nav.Tec[n-1].Rms = nil
			nav.Tec[n-1] = nav.Tec[i]
			continue
		}
		nav.Tec[n] = nav.Tec[i]
		n++
	}
	nav.Tec = nav.Tec[:n]

	Trace(5, "combtec : nav.nt=%d\n", nav.Nt())
}

/* read ionex tec grid file ----------------------------------------------------
* read ionex ionospheric tec grid file
* args   : char   *file       I   ionex tec grid file
*                                 (wind-card * is expanded)
*          nav_t  *nav        IO  navigation data
*                                 nav.nt, nav.ntmax and nav.tec are modified
*          int    opt         I   read option (1: no clear of tec data,0:clear)
* return : none
* notes  : see ref [1]
*-----------------------------------------------------------------------------*/
func (nav *Nav) ReadTec(file string, opt int) {
	var (
		fp               *os.File
		lats, lons, hgts [3]float64
		rb, nexp         float64 = 0.0, -1.0
		dcb, rms         [MAXSAT]float64
		i, n             int
		efiles           [MAXEXFILE]string
		err              error
	)

	Trace(4, "readtec : file=%s\n", file)

	/* clear of tec grid data option */
	if opt > 0 {
		nav.Tec = nil
	}
	/* expand wild card in file path */
	n = ExPath(file, efiles[:], MAXEXFILE)

	for i = 0; i < n; i++ {
		fp, err = os.OpenFile(efiles[i], os.O_RDONLY, 0666)
		if err != nil {
			Trace(2, "ionex file open error %s\n", efiles[i])
			continue
		}
		defer fp.Close()
		rd := bufio.NewReader(fp)
		/* read ionex header */
		if ReadIonexHeader(rd, lats[:], lons[:], hgts[:], &rb, &nexp, dcb[:], rms[:]) <= 0.0 {
			Trace(2, "ionex file format error %s\n", efiles[i])
			continue
		}
		/* read ionex body */
		ReadIonexBody(rd, lats[:], lons[:], hgts[:], rb, nexp, nav)

	}

	/* combine tec grid data */
	if nav.Nt() > 0 {
		nav.CombineTec()
	}

	/* P1-P2 dcb */
	for i = 0; i < len(nav.CBias); i++ {
		cbias := nav.CBias[i]
		cbias[0] = CLIGHT * dcb[i] * (1e-9) /* ns.m */
	}
}

/* interpolate tec grid data -------------------------------------------------*/
func (tec *Tec) InterpTec(k int, posp []float64, value, rms *float64) int {
	var (
		dlat, dlon, a, b float64
		d, r             [4]float64
		i, j, n, index   int
	)

	Trace(4, "interptec: k=%d posp=%.2f %.2f\n", k, posp[0]*R2D, posp[1]*R2D)
	*value, *rms = 0.0, 0.0

	if tec.Lats[2] == 0.0 || tec.Lons[2] == 0.0 {
		return 0
	}

	dlat = posp[0]*R2D - tec.Lats[0]
	dlon = posp[1]*R2D - tec.Lons[0]
	if tec.Lons[2] > 0.0 {
		dlon -= math.Floor(dlon/360) * 360.0 /*  0<=dlon<360 */
	} else {
		dlon += math.Floor(-dlon/360) * 360.0
	} /* -360<dlon<=0 */

	a = dlat / tec.Lats[2]
	b = dlon / tec.Lons[2]
	i = int(math.Floor(a))
	a -= float64(i)
	j = int(math.Floor(b))
	b -= float64(j)

	/* get gridded tec data */
	for n = 0; n < 4; n++ {
		n0 := 1
		if n < 2 {
			n0 = 0
		}
		if index = dataindex(i+(n%2), j+n0, k, tec.Ndata[:]); index < 0 {
			continue
		}
		d[n] = tec.Data[index]
		r[n] = float64(tec.Rms[index])
	}

	switch {
	case d[0] > 0.0 && d[1] > 0.0 && d[2] > 0.0 && d[3] > 0.0:
		/* bilinear interpolation (inside of grid) */
		*value = (1.0-a)*(1.0-b)*d[0] + a*(1.0-b)*d[1] + (1.0-a)*b*d[2] + a*b*d[3]
		*rms = (1.0-a)*(1.0-b)*r[0] + a*(1.0-b)*r[1] + (1.0-a)*b*r[2] + a*b*r[3]
	case a <= 0.5 && b <= 0.5 && d[0] > 0.0: /* nearest-neighbour extrapolation (outside of grid) */
		*value = d[0]
		*rms = r[0]
	case a > 0.5 && b <= 0.5 && d[1] > 0.0:
		*value = d[1]
		*rms = r[1]
	case a <= 0.5 && b > 0.5 && d[2] > 0.0:
		*value = d[2]
		*rms = r[2]
	case a > 0.5 && b > 0.5 && d[3] > 0.0:
		*value = d[3]
		*rms = r[3]
	default:
		i = 0
		for n = 0; n < 4; n++ {
			if d[n] > 0.0 {
				i++
				*value += d[n]
				*rms += r[n]
			}
		}
		if i == 0 {
			return 0
		}
		*value /= float64(i)
		*rms /= float64(i)
	}
	return 1
}

/* ionosphere delay by tec grid data -----------------------------------------*/
func (tec *Tec) IonDelay(time Gtime, pos, azel []float64, opt int, delay, vari *float64) int {
	fact := 40.30e16 / FREQ1 / FREQ1 /* tecu.L1 iono (m) */
	var (
		fs, vtec, rms, hion, rp float64
		posp                    [3]float64
	)

	Trace(4, "iondelay: time=%s pos=%.1f %.1f azel=%.1f %.1f\n", TimeStr(time, 0), pos[0]*R2D, pos[1]*R2D, azel[0]*R2D, azel[1]*R2D)

	*delay, *vari = 0.0, 0.0

	for i := 0; i < tec.Ndata[2]; i++ { /* for a layer */

		hion = tec.Hgts[0] + tec.Hgts[2]*float64(i)

		/* ionospheric pierce point position */
		fs = IonPPP(pos, azel, tec.Radius, hion, posp[:])

		if opt&2 > 0 {
			/* modified single layer mapping function (M-SLM) ref [2] */
			rp = tec.Radius / (tec.Radius + hion) * math.Sin(0.9782*(PI/2.0-azel[1]))
			fs = 1.0 / math.Sqrt(1.0-rp*rp)
		}
		if opt&1 > 0 {
			/* earth rotation correction (sun-fixed coordinate) */
			posp[1] += 2.0 * PI * TimeDiff(time, tec.Time) / 86400.0
		}
		/* interpolate tec grid data */
		if tec.InterpTec(i, posp[:], &vtec, &rms) == 0 {
			return 0
		}

		*delay += fact * fs * vtec
		*vari += fact * fact * fs * fs * rms * rms
	}
	Trace(5, "iondelay: delay=%7.2f std=%6.2f\n", *delay, math.Sqrt(*vari))

	return 1
}

/* ionosphere model by tec grid data -------------------------------------------
* compute ionospheric delay by tec grid data
* args   : gtime_t time     I   time (gpst)
*          nav_t  *nav      I   navigation data
*          double *pos      I   receiver position {lat,lon,h} (rad,m)
*          double *azel     I   azimuth/elevation angle {az,el} (rad)
*          int    opt       I   model option
*                                bit0: 0:earth-fixed,1:sun-fixed
*                                bit1: 0:single-layer,1:modified single-layer
*          double *delay    O   ionospheric delay (L1) (m)
*          double *var      O   ionospheric dealy (L1) variance (m^2)
* return : status (1:ok,0:error)
* notes  : before calling the function, read tec grid data by calling readtec()
*          return ok with delay=0 and var=VAR_NOTEC if el<ZERO_EL or h<MIN_HGT
*-----------------------------------------------------------------------------*/
func (nav *Nav) IonTec(time Gtime, pos, azel []float64, opt int, delay, vari *float64) int {
	var (
		dels, vars [2]float64
		a, tt      float64
		stat       [2]int
		i          int
	)

	Trace(4, "iontec  : time=%s pos=%.1f %.1f azel=%.1f %.1f\n", TimeStr(time, 0), pos[0]*R2D, pos[1]*R2D, azel[0]*R2D, azel[1]*R2D)

	if azel[1] < ZERO_EL || pos[2] < MIN_HGT {
		*delay = 0.0
		*vari = VAR_NOTEC
		return 1
	}
	for i = 0; i < nav.Nt(); i++ {
		if TimeDiff(nav.Tec[i].Time, time) > 0.0 {
			break
		}
	}
	if i == 0 || i >= nav.Nt() {
		Trace(2, "%s: tec grid out of period\n", TimeStr(time, 0))
		return 0
	}
	if tt = TimeDiff(nav.Tec[i].Time, nav.Tec[i-1].Time); tt == 0.0 {
		Trace(2, "tec grid time interval error\n")
		return 0
	}
	/* ionospheric delay by tec grid data */
	stat[0] = nav.Tec[i-1].IonDelay(time, pos, azel, opt, &dels[0], &vars[0])
	stat[1] = nav.Tec[i].IonDelay(time, pos, azel, opt, &dels[1], &vars[1])

	if stat[0] == 0 && stat[1] == 0 {
		Trace(2, "%s: tec grid out of area pos=%6.2f %7.2f azel=%6.1f %5.1f\n",
			TimeStr(time, 0), pos[0]*R2D, pos[1]*R2D, azel[0]*R2D, azel[1]*R2D)
		return 0
	}
	if stat[0] > 0 && stat[1] > 0 { /* linear interpolation by time */
		a = TimeDiff(time, nav.Tec[i-1].Time) / tt
		*delay = dels[0]*(1.0-a) + dels[1]*a
		*vari = vars[0]*(1.0-a) + vars[1]*a
	} else if stat[0] > 0 { /* nearest-neighbour extrapolation by time */
		*delay = dels[0]
		*vari = vars[0]
	} else {
		*delay = dels[1]
		*vari = vars[1]
	}
	Trace(5, "iontec  : delay=%5.2f std=%5.2f\n", *delay, math.Sqrt(*vari))
	return 1
}
