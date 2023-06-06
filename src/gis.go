/*------------------------------------------------------------------------------
* gis.c: GIS data functions
*
*          Copyright (C) 2016 by T.TAKASU, All rights reserved.
*
* references:
*     [1] ESRI Shapefile Technical Description, An ESRI White Paper, July, 1998
*
* version : $Revision:$ $Date:$
* history : 2016/06/10 1.0  new
*           2016/07/31 1.1  add boundary of polyline and polygon
*		    2022/05/31 1.0  rewrite gis.c with golang by fxb
*-----------------------------------------------------------------------------*/

package gnssgo

import (
	"encoding/binary"
	"math"
	"os"
	"strings"
)

const SHAPE_CODE = 9994 /* shapefile code */

/* get integer big-endian ----------------------------------------------------*/
func I4_B(buff []uint8) int {
	return int(binary.BigEndian.Uint32(buff))
}

/* get integer little-endian -------------------------------------------------*/
func I4_L(buff []uint8) int {
	return int(binary.LittleEndian.Uint32(buff))
}

/* get double little-endian --------------------------------------------------*/
func D8_L(buff []uint8) float64 {

	bits := binary.LittleEndian.Uint64(buff)

	return math.Float64frombits(bits)

}

/* get double big-endian --------------------------------------------------*/
func D8_B(buff []uint8) float64 {

	bits := binary.BigEndian.Uint64(buff)

	return math.Float64frombits(bits)

}

/* read shapefile header -----------------------------------------------------*/
func ReadShapeHead(fp *os.File) int {
	var buff [128]uint8

	n, err := fp.Read(buff[:])
	if err != nil || n < 1 {
		return -1
	}
	if I4_B(buff[:]) != SHAPE_CODE {
		return -1
	}
	return I4_L(buff[32:])
}

/* initialize boundary -------------------------------------------------------*/
func init_bound(bound []float64) {
	bound[0] = PI / 2.0
	bound[1] = -PI / 2.0
	bound[2] = PI
	bound[3] = -PI
}

/* update boundary -----------------------------------------------------------*/
func update_bound(pos, bound []float64) {
	if pos[0] < bound[0] {
		bound[0] = pos[0]
	}
	if pos[0] > bound[1] {
		bound[1] = pos[0]
	}
	if pos[1] < bound[2] {
		bound[2] = pos[1]
	}
	if pos[1] > bound[3] {
		bound[3] = pos[1]
	}
}

/* add gis data --------------------------------------------------------------*/
func GisAdd(p **GisD, dtype int, data interface{}) int {
	var new_data *GisD = new(GisD)

	new_data.next = *p
	new_data.dtype = dtype
	new_data.data = data
	*p = new_data
	return 1
}

/* read point data -----------------------------------------------------------*/
func ReadPnt(fp *os.File, bound []float64, p **GisD) int {
	var (
		pnt  *Gis_Pnt = new(Gis_Pnt)
		pos  [3]float64
		buff [16]uint8
	)

	n, err := fp.Read(buff[:])
	if err != nil || n < 1 {
		return 0
	}

	pos[0] = D8_L(buff[8:]) * D2R
	pos[1] = D8_L(buff[:]) * D2R
	update_bound(pos[:], bound[:])
	Pos2Ecef(pos[:], pnt.pos[:])

	return GisAdd(p, 1, pnt)
}

/* read multi-point data ------------------------------------------------------*/
func ReadMPnt(fp *os.File, bound []float64, p **GisD) int {
	var (
		buff  [36]uint8
		i, np int
	)

	n, err := fp.Read(buff[:])
	if err != nil || n < 1 {
		return 0
	}

	np = I4_L(buff[32:])

	for i = 0; i < np; i++ {
		if ReadPnt(fp, bound, p) == 0 {
			return 0
		}
	}
	return 1
}

/* read polyline data ---------------------------------------------------------*/
func ReadPoly(fp *os.File, bound []float64, p **GisD) int {
	var (
		poly                *Gis_Poly
		pos                 [3]float64
		buff                [40]uint8
		i, j, nt, np, nr, n int
	)

	n, err := fp.Read(buff[:])
	if err != nil || n < 1 {
		return 0
	}
	nt = I4_L(buff[32:])
	np = I4_L(buff[36:])

	var part []int = make([]int, nt)

	for i = 0; i < nt; i++ {
		fp.Read(buff[:4])

		part[i] = I4_L(buff[:4])
	}
	for i = 0; i < nt; i++ {
		if i < nt-1 {
			nr = part[i+1] - part[i]
		} else {
			nr = np - part[i]

		}
		poly = new(Gis_Poly)
		poly.pos = make([]float64, nr*3)
		init_bound(poly.bound[:])
		n = 0
		for j = 0; j < nr; j++ {
			_, err = fp.Read(buff[:16])
			if err != nil {
				return 0
			}
			pos[0] = D8_L(buff[8:]) * D2R
			pos[1] = D8_L(buff[:]) * D2R
			if pos[0] < (-1e16) || pos[1] < (-1e16) {
				continue
			}
			update_bound(pos[:], poly.bound[:])
			update_bound(pos[:], bound[:])
			Pos2Ecef(pos[:], poly.pos[n*3:])
			n++
		}
		poly.npnt = n
		if GisAdd(p, 2, poly) == 0 {
			return 0
		}
	}
	return 1
}

/* read polygon data ---------------------------------------------------------*/
func ReadPolygon(fp *os.File, bound []float64, p **GisD) int {
	var (
		polygon             *Gis_Polygon
		pos                 [3]float64
		buff                [40]uint8
		i, j, nt, np, nr, n int
	)

	n, err := fp.Read(buff[:])
	if err != nil || n < 1 {
		return 0
	}
	nt = I4_L(buff[32:])
	np = I4_L(buff[36:])

	var part []int = make([]int, nt)

	for i = 0; i < nt; i++ {
		fp.Read(buff[:4])

		part[i] = I4_L(buff[:4])
	}
	for i = 0; i < nt; i++ {
		if i < nt-1 {
			nr = part[i+1] - part[i]
		} else {
			nr = np - part[i]

		}

		polygon = new(Gis_Polygon)
		polygon.pos = make([]float64, nr*3)
		init_bound(polygon.bound[:])
		n = 0
		for j = 0; j < nr; j++ {
			_, err = fp.Read(buff[:16])
			if err != nil {
				return 0
			}
			pos[0] = D8_L(buff[8:]) * D2R
			pos[1] = D8_L(buff[:]) * D2R
			if pos[0] < (-1e16) || pos[1] < (-1e16) {
				continue
			}
			update_bound(pos[:], polygon.bound[:])
			update_bound(pos[:], bound[:])
			Pos2Ecef(pos[:], polygon.pos[n*3:])
			n++
		}
		polygon.npnt = n
		if GisAdd(p, 3, polygon) == 0 {
			return 0
		}
	}
	return 1
}

/* read shapefile records ----------------------------------------------------*/
func GisReadRecord(fp, fp_idx *os.File, dtype int, bound []float64, data **GisD) int {

	var (
		p, next                       *GisD
		buff                          [16]uint8
		i, off, num, len1, len2, typ2 int
	)

	for i = 0; ; i++ {
		n, _ := fp_idx.Read(buff[:8])
		if n != 8 {
			break
		}
		off = I4_B(buff[:]) * 2
		len1 = I4_B(buff[4:]) * 2
		ns, _ := fp.Seek(int64(off), 1)
		nb, _ := fp.Read(buff[:12])
		if ns < 0 || nb != 12 {
			return 0
		}
		num = I4_B(buff[:])
		len2 = I4_B(buff[4:8]) * 2
		typ2 = I4_L(buff[8:12])

		if num != i+1 || len1 != len2 || dtype != typ2 {
			Trace(2, "shapefile record error n=%d %d len=%d %d type=%d %d\n",
				i+1, num, len1, len2, dtype, typ2)
			continue
		}
		switch dtype {
		case 1:
			ReadPnt(fp, bound, data) /* point */
		case 8:
			ReadMPnt(fp, bound, data) /* multi-point */
		case 3:
			ReadPoly(fp, bound, data) /* polyline */
		case 5:
			ReadPolygon(fp, bound, data) /* polygon */
		}

	}
	/* reverse list order */
	*data = nil
	for p = *data; p != nil; p = next {
		next = p.next
		p.next = *data
		*data = p
	}
	return 1
}

/* read gis data from shapefile ------------------------------------------------
* read gis data from shapefile (ref [1])
* args   : char   *file     I   shapefile
*          gis_t  *gis      IO  GIS data
* return : status (0:error)
* notes  : only support point, multipoint, polyline and polygon.
*          only support lat-lon for map projection.
*-----------------------------------------------------------------------------*/
func (gis *Gis) GisRead(file string, layer int) int {
	var (
		fp, fp_idx   *os.File
		type1, type2 int = 0, 0
		err          error
	)
	Trace(4, "gis_read file=%s layer=%d\n", file, layer)

	path := file
	index := strings.LastIndex(path, ".")

	if index > 0 {
		path = path[:index-1] + ".shx"
	} else {
		path = path + ".shx"
	}

	if fp, err = os.OpenFile(file, os.O_RDONLY, 0666); err != nil { /* shapefile */
		Trace(2, "shapefile open error: %s\n", file)
		return 0
	}
	defer fp.Close()
	if fp_idx, _ = os.OpenFile(path, os.O_RDONLY, 0666); fp_idx == nil { /* index file */

		Trace(2, "shapefile index open error: %s\n", path)
		return 0
	}
	defer fp_idx.Close()

	/* read header */
	type1 = ReadShapeHead(fp)
	type2 = ReadShapeHead(fp_idx)
	if type1 < 0 || type2 < 0 || type1 != type2 {
		Trace(2, "shapefile header error: %s type=%d %d\n", file, type1, type2)
		return 0
	}
	init_bound(gis.bound[:])

	/* read records */
	if GisReadRecord(fp, fp_idx, type1, gis.bound[:], &gis.data[layer]) == 0 {
		return 0
	}
	gis.name[layer] = ""
	gis.flag[layer] = 1
	return 1
}

/* free gis-data ---------------------------------------------------------------
* free and initialize gis data
* args   : gis_t  *gis      IO  gis data
* return : none
*-----------------------------------------------------------------------------*/
func (gis *Gis) GisFree() {
	var data, next *GisD

	for i := 0; i < MAXGISLAYER; i++ {
		for data = gis.data[i]; data != nil; data = next {
			next = data.next
			if data.dtype == 2 {
				data.data.(*Gis_Poly).pos = nil
			} else if data.dtype == 3 {
				data.data.(*Gis_Polygon).pos = nil
			}
			data = nil
		}
		gis.data[i] = nil
		gis.name[i] = ""
		gis.flag[i] = 0
	}
}
