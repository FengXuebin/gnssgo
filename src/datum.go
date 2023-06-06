/*------------------------------------------------------------------------------
* datum.c : datum transformation
*
*          Copyright (C) 2007 by T.TAKASU, All rights reserved.
*
* version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
* history : 2007/02/08 1.0 new
*		    2022/05/31 1.0  rewrite datum.c with golang by fxb
*-----------------------------------------------------------------------------*/
package gnssgo

import (
	"bufio"
	"fmt"
	"io"
	"os"
	"sort"
)

const MAXPRM = 400000 /* max number of parameter records */

type Tprm struct { /* datum trans parameter type */
	code   int     /* mesh code */
	db, dl float32 /* difference of latitude/longitude (sec) */
}

var prm []Tprm = nil /* datum trans parameter table */
var prm_n int = 0    /* datum trans parameter table size */
/* compare datum trans parameters --------------------------------------------*/
// func  cmpprm(const void *p1, const void *p2)int
// {
//     tprm_t *q1=(tprm_t *)p1,*q2=(tprm_t *)p2;
//     return q1->code-q2->code;
// }

/* search datum trans parameter ----------------------------------------------*/
func SearchPrm(lat, lon float64) int {
	var i, j, k, n1, m1, n2, m2, code int

	lon -= 6000.0
	n1 = int(lat / 40.0)
	lat -= float64(n1) * 40.0
	m1 = int(lon / 60.0)
	lon -= float64(m1) * 60.0
	n2 = int(lat / 5.0)
	lat -= float64(n2) * 5.0
	m2 = int(lon / 7.5)
	lon -= float64(m2) * 7.5
	code = n1*1000000 + m1*10000 + n2*1000 + m2*100 + (int)(lat/0.5)*10 + (int)(lon/0.75)
	j = prm_n - 1
	for i = 0; i < j; { /* binary search */
		k = (i + j) / 2
		if prm[k].code == code {
			return k
		}
		if prm[k].code < code {
			i = k + 1
		} else {
			j = k
		}
	}
	return -1
}

/* tokyo datum to jgd2000 lat/lon corrections --------------------------------*/
func DLatDLon(post, dpos []float64) int {
	var (
		db, dl     [2][2]float64
		a, b, c, d float64
		i, j, k    int
	)
	lat := post[0] * R2D * 60.0
	lon := post[1] * R2D * 60.0 /* arcmin */
	dlat := 0.5
	dlon := 0.75

	if prm_n == 0 {
		return -1
	}
	for i = 0; i < 2; i++ {
		for j = 0; j < 2; j++ {
			if k = SearchPrm(lat+float64(i)*dlat, lon+float64(j)*dlon); k < 0 {
				return -1
			}
			db[i][j] = float64(prm[k].db)
			dl[i][j] = float64(prm[k].dl)
		}
	}
	a = lat/dlat - float64(int(lat/dlat))
	c = 1.0 - a
	b = lon/dlon - float64(int(lon/dlon))
	d = 1.0 - b
	dpos[0] = (db[0][0]*c*d + db[1][0]*a*d + db[0][1]*c*b + db[1][1]*a*b) * D2R / 3600.0
	dpos[1] = (dl[0][0]*c*d + dl[1][0]*a*d + dl[0][1]*c*b + dl[1][1]*a*b) * D2R / 3600.0
	return 0
}

/* load datum transformation parameter -----------------------------------------
* load datum transformation parameter
* args   : char  *file      I   datum trans parameter file path
* return : status (0:ok,0>:error)
* notes  : parameters file shall comply with GSI TKY2JGD.par
*-----------------------------------------------------------------------------*/
func LoadDatumP(file string) int {
	var (
		fp  *os.File
		err error
		n   int
	)
	buff := make([]byte, 256)

	if prm_n > 0 {
		return 0
	} /* already loaded */

	if fp, err = os.OpenFile(file, os.O_RDWR, 0666); err != nil {
		Trace(2, "datum.go: datum prm file open error : %s\n", file)
		return -1
	}
	defer fp.Close()
	prm = make([]Tprm, MAXPRM)
	for {
		rd := bufio.NewReader(fp)
		n, err = rd.Read(buff)
		if err != nil && err != io.EOF {
			Trace(2, "fail to read file")
			break
		}
		if n == 0 {
			break
		}
		if n, _ = fmt.Sscanf(string(buff), "%d %f %f", &prm[n].code, &prm[n].db, &prm[n].dl); n >= 3 {
			prm_n++
		}
	}

	sort.Slice(prm, func(i, j int) bool { /* sort parameter table */
		return prm[i].code < prm[j].code
	})

	return 0
}

/* tokyo datum to JGD2000 datum ------------------------------------------------
* transform position in Tokyo datum to JGD2000 datum
* args   : double *pos      I   position in Tokyo datum   {lat,lon,h} (rad,m)
*                           O   position in JGD2000 datum {lat,lon,h} (rad,m)
* return : status (0:ok,0>:error,out of range)
* notes  : before calling, call loaddatump() to set parameter table
*-----------------------------------------------------------------------------*/
func Tokyo2Jgd(pos []float64) int {
	var post, dpos [2]float64

	post[0] = pos[0]
	post[1] = pos[1]
	if DLatDLon(post[:], dpos[:]) > 0 {
		return -1
	}
	pos[0] = post[0] + dpos[0]
	pos[1] = post[1] + dpos[1]
	return 0
}

/* JGD2000 datum to Tokyo datum ------------------------------------------------
* transform position in JGD2000 datum to Tokyo datum
* args   : double *pos      I   position in JGD2000 datum {lat,lon,h} (rad,m)
*                           O   position in Tokyo datum   {lat,lon,h} (rad,m)
* return : status (0:ok,0>:error,out of range)
* notes  : before calling, call loaddatump() to set parameter table
*-----------------------------------------------------------------------------*/
func Jgd2Tokyo(pos []float64) int {
	var posj, dpos [2]float64

	posj[0] = pos[0]
	posj[1] = pos[1]
	for i := 0; i < 2; i++ {
		if DLatDLon(pos[:], dpos[:]) > 0 {
			return -1
		}
		pos[0] = posj[0] - dpos[0]
		pos[1] = posj[1] - dpos[1]
	}
	return 0
}
