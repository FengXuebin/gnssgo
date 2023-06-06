/*------------------------------------------------------------------------------
* tle.c: NORAD TLE (two line element) functions
*
*          Copyright (C) 2012-2020 by T.TAKASU, All rights reserved.
*
* references:
*     [1] F.R.Hoots and R.L.Roehrich, Spacetrack report No.3, Models for
*         propagation of NORAD element sets, December 1980
*     [2] D.A.Vallado, P.Crawford, R.Hujsak and T.S.Kelso, Revisiting
*         Spacetrack Report #3, AIAA 2006-6753, 2006
*     [3] CelesTrak (http://www.celestrak.com)
*
* version : $Revision:$ $Date:$
* history : 2012/11/01 1.0  new
*           2013/01/25 1.1  fix bug on binary search
*           2014/08/26 1.2  fix bug on tle_pos() to get tle by satid or desig
*           2020/11/30 1.3  fix problem on duplicated names in a satellite
*		    2022/05/31 1.0  rewrite tlc.c with golang by fxb
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

/* SGP4 model propagator by STR#3 (ref [1] sec.6,11) -------------------------*/

const (
	DE2RA  = 0.174532925e-1
	E6A    = 1.e-6
	PIO2   = 1.57079633
	QO     = 120.0
	SO     = 78.0
	TOTHRD = 0.66666667
	TWOPI  = 6.2831853
	X3PIO2 = 4.71238898
	XJ2    = 1.082616e-3
	XJ3    = -0.253881e-5
	XJ4    = -1.65597e-6
	XKE    = 0.743669161e-1
	XKMPER = 6378.135
	XMNPDA = 1440.0
	AE     = 1.0
	CK2    = 5.413080e-4   /* = 0.5*XJ2*AE*AE */
	CK4    = 0.62098875e-6 /* = -0.375*XJ4*AE*AE*AE*AE */
	QOMS2T = 1.88027916e-9 /* = pow((QO-SO)*AE/XKMPER,4.0) */
	S      = 1.01222928 /* = AE*(1.0+SO/XKMPER) */)

func SGP4_STR3(tsince float64, data *TleD, rs []float64) {
	var (
		xnodeo, omegao, xmo, eo, xincl, xno, bstar                                        float64 //, xndt2o, xndd6o
		a1, cosio, theta2, x3thm1, eosq, betao2, betao, del1, ao, delo, xnodp, aodp, s4   float64
		qoms24, perige, pinvsq, tsi, eta, etasq, eeta, psisq, coef, coef1, c1, c2, c3, c4 float64
		c5, sinio, a3ovk2, x1mth2, theta4, xmdot, x1m5th, omgdot, xhdot1, xnodot          float64
		omgcof, xmcof, xnodcf, t2cof, xlcof, aycof, delmo, sinmo, x7thm1, c1sq, d2, d3    float64
		d4, t3cof, t4cof, t5cof, xmdf, omgadf, xnoddf, omega, xmp, tsq, xnode, delomg     float64
		delm, tcube, tfour, a, e, xl, beta, xn, axn, xll, aynl, xlt, ayn, capu, sinepw    float64
		cosepw, epw, ecose, esine, elsq, pl, r, rdot, rfdot, betal, cosu, sinu, u, sin2u  float64
		cos2u, rk, uk, xnodek, xinck, rdotk, rfdotk, sinuk, cosuk, sinik, cosik, sinnok   float64
		cosnok, xmx, xmy, ux, uy, uz, vx, vy, vz, x, y, z, xdot, ydot, zdot               float64
		temp, temp1, temp2, temp3, temp4, temp5, temp6, tempa, tempe, templ               float64
		i, isimp                                                                          int
	)

	xnodeo = data.OMG * DE2RA
	omegao = data.Omg * DE2RA
	xmo = data.M * DE2RA
	xincl = data.Inc * DE2RA
	temp = TWOPI / XMNPDA / XMNPDA
	xno = data.N * temp * XMNPDA
	// xndt2o = data.ndot * temp
	// xndd6o = data.nddot * temp / XMNPDA
	bstar = data.BStar / AE
	eo = data.Ecc
	/*
	 * recover original mean motion (xnodp) and semimajor axis (aodp)
	 * from input elements
	 */
	a1 = math.Pow(XKE/xno, TOTHRD)
	cosio = math.Cos(xincl)
	theta2 = cosio * cosio
	x3thm1 = 3.0*theta2 - 1.0
	eosq = eo * eo
	betao2 = 1.0 - eosq
	betao = math.Sqrt(betao2)
	del1 = 1.5 * CK2 * x3thm1 / (a1 * a1 * betao * betao2)
	ao = a1 * (1.0 - del1*(0.5*TOTHRD+del1*(1.0+134.0/81.0*del1)))
	delo = 1.5 * CK2 * x3thm1 / (ao * ao * betao * betao2)
	xnodp = xno / (1.0 + delo)
	aodp = ao / (1.0 - delo)
	/*
	 * initialization
	 * for perigee less than 220 kilometers, the isimp flag is set and
	 * the equations are truncated to linear variation in sqrt a and
	 * quadratic variation in mean anomaly. also, the c3 term, the
	 * delta omega term, and the delta m term are dropped.
	 */
	isimp = 0
	if (aodp * (1.0 - eo) / AE) < (220.0/XKMPER + AE) {
		isimp = 1
	}

	/* for perigee below 156 km, the values of s and qoms2t are altered */
	s4 = S
	qoms24 = QOMS2T
	perige = (aodp*(1.0-eo) - AE) * XKMPER
	if perige < 156.0 {
		s4 = perige - 78.0
		if perige <= 98.0 {
			s4 = 20.0
		}
		qoms24 = math.Pow((120.0-s4)*AE/XKMPER, 4.0)
		s4 = s4/XKMPER + AE
	}
	pinvsq = 1.0 / (aodp * aodp * betao2 * betao2)
	tsi = 1.0 / (aodp - s4)
	eta = aodp * eo * tsi
	etasq = eta * eta
	eeta = eo * eta
	psisq = math.Abs(1.0 - etasq)
	coef = qoms24 * math.Pow(tsi, 4.0)
	coef1 = coef / math.Pow(psisq, 3.5)
	c2 = coef1 * xnodp * (aodp*(1.0+1.5*etasq+eeta*(4.0+etasq)) + 0.75*
		CK2*tsi/psisq*x3thm1*(8.0+3.0*etasq*(8.0+etasq)))
	c1 = bstar * c2
	sinio = math.Sin(xincl)
	a3ovk2 = -XJ3 / CK2 * math.Pow(AE, 3.0)
	c3 = coef * tsi * a3ovk2 * xnodp * AE * sinio / eo
	x1mth2 = 1.0 - theta2
	c4 = 2.0 * xnodp * coef1 * aodp * betao2 * (eta*
		(2.0+0.5*etasq) + eo*(0.5+2.0*etasq) - 2.0*CK2*tsi/
		(aodp*psisq)*(-3.0*x3thm1*(1.0-2.0*eeta+etasq*
		(1.5-0.5*eeta))+0.75*x1mth2*(2.0*etasq-eeta*
		(1.0+etasq))*math.Cos(2.0*omegao)))
	c5 = 2.0 * coef1 * aodp * betao2 * (1.0 + 2.75*(etasq+eeta) + eeta*etasq)
	theta4 = theta2 * theta2
	temp1 = 3.0 * CK2 * pinvsq * xnodp
	temp2 = temp1 * CK2 * pinvsq
	temp3 = 1.25 * CK4 * pinvsq * pinvsq * xnodp
	xmdot = xnodp + 0.5*temp1*betao*x3thm1 + 0.0625*temp2*betao*
		(13.0-78.0*theta2+137.0*theta4)
	x1m5th = 1.0 - 5.0*theta2
	omgdot = -0.5*temp1*x1m5th + 0.0625*temp2*(7.0-114.0*theta2+
		395.0*theta4) + temp3*(3.0-36.0*theta2+49.0*theta4)
	xhdot1 = -temp1 * cosio
	xnodot = xhdot1 + (0.5*temp2*(4.0-19.0*theta2)+2.0*temp3*(3.0-
		7.0*theta2))*cosio
	omgcof = bstar * c3 * math.Cos(omegao)
	xmcof = -TOTHRD * coef * bstar * AE / eeta
	xnodcf = 3.5 * betao2 * xhdot1 * c1
	t2cof = 1.5 * c1
	xlcof = 0.125 * a3ovk2 * sinio * (3.0 + 5.0*cosio) / (1.0 + cosio)
	aycof = 0.25 * a3ovk2 * sinio
	delmo = math.Pow(1.0+eta*math.Cos(xmo), 3.0)
	sinmo = math.Sin(xmo)
	x7thm1 = 7.0*theta2 - 1.0

	if isimp != 1 {
		c1sq = c1 * c1
		d2 = 4.0 * aodp * tsi * c1sq
		temp = d2 * tsi * c1 / 3.0
		d3 = (17.0*aodp + s4) * temp
		d4 = 0.5 * temp * aodp * tsi * (221.0*aodp + 31.0*s4) * c1
		t3cof = d2 + 2.0*c1sq
		t4cof = 0.25 * (3.0*d3 + c1*(12.0*d2+10.0*c1sq))
		t5cof = 0.2 * (3.0*d4 + 12.0*c1*d3 + 6.0*d2*d2 + 15.0*c1sq*(2.0*d2+c1sq))
	} else {
		d2, d3, d4, t3cof, t4cof, t5cof = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
	}
	/* update for secular gravity and atmospheric drag */
	xmdf = xmo + xmdot*tsince
	omgadf = omegao + omgdot*tsince
	xnoddf = xnodeo + xnodot*tsince
	omega = omgadf
	xmp = xmdf
	tsq = tsince * tsince
	xnode = xnoddf + xnodcf*tsq
	tempa = 1.0 - c1*tsince
	tempe = bstar * c4 * tsince
	templ = t2cof * tsq
	if isimp == 1 {
		delomg = omgcof * tsince
		delm = xmcof * (math.Pow(1.0+eta*math.Cos(xmdf), 3.0) - delmo)
		temp = delomg + delm
		xmp = xmdf + temp
		omega = omgadf - temp
		tcube = tsq * tsince
		tfour = tsince * tcube
		tempa = tempa - d2*tsq - d3*tcube - d4*tfour
		tempe = tempe + bstar*c5*(math.Sin(xmp)-sinmo)
		templ = templ + t3cof*tcube + tfour*(t4cof+tsince*t5cof)
	}
	a = aodp * math.Pow(tempa, 2.0)
	e = eo - tempe
	xl = xmp + omega + xnode + xnodp*templ
	beta = math.Sqrt(1.0 - e*e)
	xn = XKE / math.Pow(a, 1.5)

	/* long period periodics */
	axn = e * math.Cos(omega)
	temp = 1.0 / (a * beta * beta)
	xll = temp * xlcof * axn
	aynl = temp * aycof
	xlt = xl + xll
	ayn = e*math.Sin(omega) + aynl

	/* solve keplers equation */
	capu = math.Mod(xlt-xnode, TWOPI)
	temp2 = capu
	for i = 0; i < 10; i++ {
		sinepw = math.Sin(temp2)
		cosepw = math.Cos(temp2)
		temp3 = axn * sinepw
		temp4 = ayn * cosepw
		temp5 = axn * cosepw
		temp6 = ayn * sinepw
		epw = (capu-temp4+temp3-temp2)/(1.0-temp5-temp6) + temp2
		if math.Abs(epw-temp2) <= E6A {
			break
		}
		temp2 = epw
	}
	/* short period preliminary quantities */
	ecose = temp5 + temp6
	esine = temp3 - temp4
	elsq = axn*axn + ayn*ayn
	temp = 1.0 - elsq
	pl = a * temp
	r = a * (1.0 - ecose)
	temp1 = 1.0 / r
	rdot = XKE * math.Sqrt(a) * esine * temp1
	rfdot = XKE * math.Sqrt(pl) * temp1
	temp2 = a * temp1
	betal = math.Sqrt(temp)
	temp3 = 1.0 / (1.0 + betal)
	cosu = temp2 * (cosepw - axn + ayn*esine*temp3)
	sinu = temp2 * (sinepw - ayn - axn*esine*temp3)
	u = math.Atan2(sinu, cosu)
	sin2u = 2.0 * sinu * cosu
	cos2u = 2.0*cosu*cosu - 1.0
	temp = 1.0 / pl
	temp1 = CK2 * temp
	temp2 = temp1 * temp

	/* update for short periodics */
	rk = r*(1.0-1.5*temp2*betal*x3thm1) + 0.5*temp1*x1mth2*cos2u
	uk = u - 0.25*temp2*x7thm1*sin2u
	xnodek = xnode + 1.5*temp2*cosio*sin2u
	xinck = xincl + 1.5*temp2*cosio*sinio*cos2u
	rdotk = rdot - xn*temp1*x1mth2*sin2u
	rfdotk = rfdot + xn*temp1*(x1mth2*cos2u+1.5*x3thm1)

	/* orientation vectors */
	sinuk = math.Sin(uk)
	cosuk = math.Cos(uk)
	sinik = math.Sin(xinck)
	cosik = math.Cos(xinck)
	sinnok = math.Sin(xnodek)
	cosnok = math.Cos(xnodek)
	xmx = -sinnok * cosik
	xmy = cosnok * cosik
	ux = xmx*sinuk + cosnok*cosuk
	uy = xmy*sinuk + sinnok*cosuk
	uz = sinik * sinuk
	vx = xmx*cosuk - cosnok*sinuk
	vy = xmy*cosuk - sinnok*sinuk
	vz = sinik * cosuk

	/* position and velocity */
	x = rk * ux
	y = rk * uy
	z = rk * uz
	xdot = rdotk*ux + rfdotk*vx
	ydot = rdotk*uy + rfdotk*vy
	zdot = rdotk*uz + rfdotk*vz

	rs[0] = x * XKMPER / AE * 1e3 /* (m) */
	rs[1] = y * XKMPER / AE * 1e3
	rs[2] = z * XKMPER / AE * 1e3
	rs[3] = xdot * XKMPER / AE * XMNPDA / 86400.0 * 1e3 /* (m/s) */
	rs[4] = ydot * XKMPER / AE * XMNPDA / 86400.0 * 1e3
	rs[5] = zdot * XKMPER / AE * XMNPDA / 86400.0 * 1e3
}

/* drop spaces at string tail ------------------------------------------------*/
// func chop(buff *string) {
// 	*buff = strings.TrimRightFunc(*buff, func(r rune) bool {
// 		return (r == ' ' || r == '\r' || r == '\n')
// 	})
// }

func tle_chop(buff *string) {

	*buff = strings.TrimRightFunc(*buff, func(r rune) bool {
		return (r == ' ' || r == '\r' || r == '\n')
	})
}

/* test TLE line checksum ----------------------------------------------------*/
func checksum(buff []byte) int {
	var i, cs int

	if len(buff) < 69 {
		return 0
	}

	for i = 0; i < 68; i++ {
		if '0' <= buff[i] && buff[i] <= '9' {
			cs += int(buff[i] - '0')
		} else if buff[i] == '-' {
			cs += 1
		}
	}
	if int(buff[68]-'0') == cs%10 {
		return 1
	}
	return 0
}

/* decode TLE line 1 ---------------------------------------------------------*/
func (data *TleD) Decode_line1(buff string) int {
	var (
		year, doy, nddot, exp1, bstar, exp2 float64
		ep                                  [6]float64 = [6]float64{2000, 1, 1, 0, 0, 0}
	)

	data.SatNo = buff[2:7] /* satellite number */
	tle_chop(&data.SatNo)

	data.SatClass = buff[7] /* satellite classification */
	data.Desig = buff[9:17] /* international designator */
	tle_chop(&data.Desig)

	year = Str2Num(buff, 18, 2)       /* epoch year */
	doy = Str2Num(buff, 20, 12)       /* epoch day of year */
	data.Ndot = Str2Num(buff, 33, 10) /* 1st time derivative of n */
	nddot = Str2Num(buff, 44, 6)      /* 2nd time derivative of n */
	exp1 = Str2Num(buff, 50, 2)
	bstar = Str2Num(buff, 53, 6) /* Bstar drag term */
	exp2 = Str2Num(buff, 59, 2)
	data.EType = int(Str2Num(buff, 62, 1)) /* ephemeris type */
	data.EleNo = int(Str2Num(buff, 64, 4)) /* ephemeris number */
	data.NDdot = nddot * 1e-5 * math.Pow(10.0, exp1)
	data.BStar = bstar * 1e-5 * math.Pow(10.0, exp2)

	ep[0] = year + 1900.0
	if year < 57.0 {
		ep[0] = year + 2000.0
	}
	data.Epoch = TimeAdd(Epoch2Time(ep[:]), (doy-1.0)*86400.0)

	data.Inc, data.OMG, data.Ecc, data.Omg, data.M, data.N = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
	data.RevNo = 0
	return 1
}

/* decode TLE line 2 ---------------------------------------------------------*/
func (data *TleD) Decode_line2(buff string) int {
	satno := buff[2:7] /* satellite number */
	tle_chop(&satno)

	data.Inc = Str2Num(buff, 8, 8)         /* inclination (deg) */
	data.OMG = Str2Num(buff, 17, 8)        /* RAAN (deg) */
	data.Ecc = Str2Num(buff, 26, 7) * 1e-7 /* eccentricity */
	data.Omg = Str2Num(buff, 34, 8)        /* argument of perigee (deg) */
	data.M = Str2Num(buff, 43, 8)          /* mean anomaly (deg) */
	data.N = Str2Num(buff, 52, 11)         /* mean motion (rev/day) */
	data.RevNo = int(Str2Num(buff, 63, 5)) /* revolution number */

	if strings.Compare(satno, data.SatNo) != 0 {
		Trace(2, "tle satno mismatch: %s %s\n", data.SatNo, satno)
		return 0
	}
	if data.N <= 0.0 || data.Ecc < 0.0 {
		Trace(2, "tle data error: %s\n", satno)
		return 0
	}
	return 1
}

/* add TLE data --------------------------------------------------------------*/
func (tle *Tle) AddTleData(data *TleD) int {
	if tle.N >= tle.Nmax {

		if tle.Nmax > 0 {
			tle.Nmax = tle.Nmax * 2
		} else {
			tle.Nmax = 1024
		}

		var tle_data []TleD = make([]TleD, tle.Nmax)
		copy(tle_data, tle.Data)
		tle.Data = tle_data
	}
	tle.Data[tle.N] = *data
	tle.N++
	return 1
}

/* compare TLE data by satellite name ----------------------------------------*/
func cmp_tle_data(q1, q2 *TleD) int {
	return strings.Compare(q1.Name, q2.Name)
}

/* read TLE file ---------------------------------------------------------------
* read NORAD TLE (two line element) data file (ref [2],[3])
* args   : char   *file     I   NORAD TLE data file
*          tle_t  *tle      O   TLE data
* return : status (1:ok,0:error)
* notes  : before calling the function, the TLE data should be initialized.
*          the file should be in a two line (only TLE) or three line (satellite
*          name + TLE) format.
*          the characters after # in a line are treated as comments.
*-----------------------------------------------------------------------------*/
func (tle *Tle) TleRead(file string) int {
	var (
		fp        *os.File
		data      TleD
		buff      string
		line, idx int
		err       error
	)

	if fp, err = os.OpenFile(file, os.O_RDONLY, 0666); err != nil {
		Trace(2, "tle file open error: %s\n", file)
		return 0
	}
	defer fp.Close()

	rd := bufio.NewReader(fp)
	for {
		buff, err = rd.ReadString('\n')
		if err != nil || len(buff) == 0 {
			break
		}
		/* delete comments */
		idx = strings.Index(buff, "#")
		if idx >= 0 {
			buff = buff[:idx]
		}
		//       if ((p=strchr(buff,'#'))) *p='\0';
		tle_chop(&buff)
		if len(buff) == 0 {
			continue
		}
		if buff[0] == '1' && checksum([]byte(buff)) > 0 {

			/* decode TLE line 1 */
			if data.Decode_line1(buff) > 0 {
				line = 1
			}
		} else if line == 1 && buff[0] == '2' && checksum([]byte(buff)) > 0 {

			/* decode TLE line 2 */
			if data.Decode_line2(buff) == 0 {
				continue
			}

			/* add TLE data */
			if tle.AddTleData(&data) == 0 {
				return 0
			}
			data.Name = ""
			data.Alias = ""
		} else if len(buff) > 0 {

			/* satellite name in three line format */

			/* omit words in parentheses */
			idx = strings.Index(buff, "(")
			if idx >= 0 {
				data.Name = buff[:idx]
				data.Alias = buff[idx:]
			} else {
				data.Name = buff
			}
			tle_chop(&data.Name)
			tle_chop(&data.Alias)
			line = 0
		}
	}

	/* sort tle data by satellite name */
	if tle.N > 0 {
		sort.Slice(tle.Data[:tle.N], func(i, j int) bool {
			return cmp_tle_data(&tle.Data[i], &tle.Data[j]) < 0
		})
	}
	return 1
}

/* read TLE satellite name file ------------------------------------------------
* read TLE satellite name file
* args   : char   *file     I   TLE satellite name file
*          tle_t  *tle      IO  TLE data
* return : status (1:ok,0:error)
* notes  : before calling the function, call tle_read() to read tle table
*          the TLE satellite name file contains the following record as a text
*          line. strings after # are treated as comments.
*
*          name satno [desig [# comment]]
*
*            name : satellite name
*            satno: satellite catalog number
*            desig: international designator (optional)
*-----------------------------------------------------------------------------*/
func (tle *Tle) ReadTleName(file string) int {
	var (
		fp                       *os.File
		data                     TleD
		buff, name, satno, desig string
		i, n, idx                int
		err                      error
	)

	if fp, err = os.OpenFile(file, os.O_RDONLY, 0666); err != nil {
		Trace(2, "tle satellite name file open error:%s\n", file)
		return 0
	}
	defer fp.Close()

	rd := bufio.NewReader(fp)
	for {
		buff, err = rd.ReadString('\n')
		if err != nil {
			break
		}
		/* delete comments */
		idx = strings.Index(buff, "#")
		if idx >= 0 {
			buff = buff[:idx]
		}
		//       if ((p=strchr(buff,'#'))) *p='\0';
		tle_chop(&buff)

		desig = ""

		if n, _ = fmt.Sscanf(buff, "%s %s %s", name, satno, desig); n < 2 {
			continue
		}
		satno = satno[:5]

		for i = 0; i < tle.N; i++ {
			if strings.Compare(tle.Data[i].SatNo, satno) == 0 ||
				strings.Compare(tle.Data[i].Desig, desig) == 0 {
				break
			}
		}
		if i >= tle.N {
			Trace(4, "no tle data: satno=%s desig=%s\n", satno, desig)
			continue
		}
		if len(tle.Data[i].Name) == 0 {
			tle.Data[i].Name = name
		} else {
			data = tle.Data[i]
			data.Name = name
			if tle.AddTleData(&data) == 0 {
				break
			}
		}
	}

	/* sort tle data by satellite name */
	if tle.N > 0 {
		sort.Slice(tle.Data[:tle.N], func(i, j int) bool {
			return cmp_tle_data(&tle.Data[i], &tle.Data[j]) < 0
		})
	}
	return 1
}

/* satellite position and velocity with TLE data -------------------------------
* compute satellite position and velocity in ECEF with TLE data
* args   : gtime_t time     I   time (GPST)
*          char   *name     I   satellite name           ("": not specified)
*          char   *satno    I   satellite catalog number ("": not specified)
*          char   *desig    I   international designaor  ("": not specified)
*          tle_t  *tle      I   TLE data
*          erp_t  *erp      I   EOP data (NULL: not used)
*          double *rs       O   sat position/velocity {x,y,z,vx,vy,vz} (m,m/s)
* return : status (1:ok,0:error)
* notes  : the coordinates of the position and velocity are ECEF (ITRF)
*          if erp == NULL, polar motion and ut1-utc are neglected
*-----------------------------------------------------------------------------*/
func (tle *Tle) TlePos(time Gtime, name, satno, desig string, erp *Erp, rs []float64) int {
	var (
		tutc           Gtime
		tsince, gmst   float64
		rs_tle, rs_pef [6]float64
		R1, R2, R3, W  [9]float64
		erpv           [5]float64
		i, j, k        int
		stat           int = 1
	)

	/* binary search by satellite name */
	if len(name) > 0 {
		for i, j, k = 0, 0, tle.N-1; j <= k; {
			i = (j + k) / 2
			if stat = strings.Compare(name, tle.Data[i].Name); stat == 0 {
				break
			}
			if stat < 0 {
				k = i - 1
			} else {
				j = i + 1
			}
		}
	}
	/* serial search by catalog no or international designator */
	if stat != 0 && (len(satno) > 0 || len(desig) > 0) {
		for i = 0; i < tle.N; i++ {
			if strings.Compare(tle.Data[i].SatNo, satno) == 0 ||
				strings.Compare(tle.Data[i].Desig, desig) == 0 {
				break
			}
		}
		if i < tle.N {
			stat = 0
		}
	}
	if stat != 0 {
		Trace(4, "no tle data: name=%s satno=%s desig=%s\n", name, satno, desig)
		return 0
	}
	tutc = GpsT2Utc(time)

	/* time since epoch (min) */
	tsince = TimeDiff(tutc, tle.Data[i].Epoch) / 60.0

	/* SGP4 model propagator by STR#3 */
	SGP4_STR3(tsince, &tle.Data[i], rs_tle[:])

	/* erp values */
	if erp != nil {
		GetErp(erp, time, erpv[:])
	}

	/* GMST (rad) */
	gmst = Utc2GmsT(tutc, erpv[2])

	/* TEME (true equator, mean eqinox) . ECEF (ref [2] IID, Appendix C) */
	R1[0] = 1.0
	R1[4], R1[8] = math.Cos(-erpv[1]), math.Cos(-erpv[1])
	R1[7] = math.Sin(-erpv[1])
	R1[5] = -R1[7]
	R2[4] = 1.0
	R2[0], R2[8] = math.Cos(-erpv[0]), math.Cos(-erpv[0])
	R2[2] = math.Sin(-erpv[0])
	R2[6] = -R2[2]
	R3[8] = 1.0
	R3[0], R3[4] = math.Cos(gmst), math.Cos(gmst)
	R3[3] = math.Sin(gmst)
	R3[1] = -R3[3]
	MatMul("NN", 3, 1, 3, 1.0, R3[:], rs_tle[:], 0.0, rs_pef[:])
	MatMul("NN", 3, 1, 3, 1.0, R3[:], rs_tle[3:], 0.0, rs_pef[3:])
	rs_pef[3] += OMGE * rs_pef[1]
	rs_pef[4] -= OMGE * rs_pef[0]
	MatMul("NN", 3, 3, 3, 1.0, R1[:], R2[:], 0.0, W[:])
	MatMul("NN", 3, 1, 3, 1.0, W[:], rs_pef[:], 0.0, rs)
	MatMul("NN", 3, 1, 3, 1.0, W[:], rs_pef[3:], 0.0, rs[3:])
	return 1
}
