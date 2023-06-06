/*------------------------------------------------------------------------------
* convgpx.c : gpx converter
*
*          Copyright (C) 2016 by T.TAKASU, All rights reserved.
*
* references :
*     [1] GPX The GPS Exchange Format http://www.topografix.com/gpx.asp
*
* version : $Revision:$ $Date:$
* history : 2016/06/11  1.0  new
*           2016/09/18  1.1  modify <fix> labels according GPX specs
*		    2022/05/31  1.0  rewrite convgpx.c with golang by fxb
*-----------------------------------------------------------------------------*/

package gnssgo

import (
	"fmt"
	"os"
	"strings"
)

const HEADXML string = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
const HEADGPX string = "<gpx version=\"1.1\" creator=\"%s\" xmlns=\"%s\">\n"
const TAILGPX string = "</gpx>"

const XMLNS string = "http://www.topografix.com/GPX/1/1"

/* output waypoint -----------------------------------------------------------*/
func OutPoint(fp *os.File, time Gtime, pos []float64, label string, stat, outalt, outtime int) {
	/* fix, float, sbas and ppp are rtklib extentions to GPX */
	var (
		fix_label = []string{"fix", "float", "sbas", "dgps", "3d", "ppp"}
		ep        [6]float64
	)

	fp.WriteString(fmt.Sprintf("<wpt lat=\"%.9f\" lon=\"%.9f\">\n", pos[0]*R2D, pos[1]*R2D))
	if outalt > 0 {
		var tmp float64
		if outalt == 2 {
			tmp = GeoidH(pos)
		}
		fp.WriteString(fmt.Sprintf(" <ele>%.4f</ele>\n", pos[2]-tmp))
	}
	if outtime > 0 {
		if outtime == 2 {
			time = GpsT2Utc(time)
		} else if outtime == 3 {
			time = TimeAdd(GpsT2Utc(time), 9*3600.0)
		}
		Time2Epoch(time, ep[:])
		fp.WriteString(fmt.Sprintf(" <time>%04.0f-%02.0f-%02.0fT%02.0f:%02.0f:%05.2fZ</time>\n",
			ep[0], ep[1], ep[2], ep[3], ep[4], ep[5]))
	}
	if outalt == 2 {
		fp.WriteString(fmt.Sprintf(" <geoidheight>%.4f</geoidheight>\n", GeoidH(pos)))
	}
	if stat >= 1 && stat <= 6 {
		fp.WriteString(fmt.Sprintf(" <fix>%s</fix>\n", fix_label[stat-1]))
	}

	fp.WriteString(fmt.Sprintf(" <name>%s</name>\n", label))

	fp.WriteString("</wpt>\n")
}

/* output track --------------------------------------------------------------*/
func OutTrack(fp *os.File, solbuf *SolBuf, outalt, outtime int) {
	var (
		time Gtime
		pos  [3]float64
		ep   [6]float64
		i    int
	)

	fp.WriteString("<trk>\n")
	fp.WriteString(" <trkseg>\n")
	for i = 0; i < solbuf.N; i++ {
		Ecef2Pos(solbuf.Data[i].Rr[:], pos[:])
		fp.WriteString(fmt.Sprintf("  <trkpt lat=\"%.9f\" lon=\"%.9f\">\n", pos[0]*R2D,
			pos[1]*R2D))
		if outalt > 0 {
			var tmp float64 = 0.0
			if outalt == 2 {
				tmp = GeoidH(pos[:])
			}
			fp.WriteString(fmt.Sprintf("   <ele>%.4f</ele>\n", pos[2]-tmp))
		}
		if outtime > 0 {
			time = solbuf.Data[i].Time
			if outtime == 2 {
				time = GpsT2Utc(time)
			} else if outtime == 3 {
				time = TimeAdd(GpsT2Utc(time), 9*3600.0)
			}
			Time2Epoch(time, ep[:])
			fp.WriteString(fmt.Sprintf("   <time>%04.0f-%02.0f-%02.0fT%02.0f:%02.0f:%05.2fZ</time>\n",
				ep[0], ep[1], ep[2], ep[3], ep[4], ep[5]))
		}
		if outalt == 2 {
			fp.WriteString(fmt.Sprintf("   <geoidheight>%.4f</geoidheight>\n", GeoidH(pos[:])))
		}
		fp.WriteString("  </trkpt>\n")
	}
	fp.WriteString(" </trkseg>\n")
	fp.WriteString("</trk>\n")
}

/* save gpx file -------------------------------------------------------------*/
func SaveGpx(file string, solbuf *SolBuf, outtrk, outpnt, outalt, outtime int) int {
	var (
		fp  *os.File
		pos [3]float64
		i   int
		err error
	)
	fp, err = os.OpenFile(file, os.O_CREATE|os.O_WRONLY|os.O_TRUNC, os.ModeAppend|os.ModePerm)
	if err != nil {
		Trace(2, "file open error : %s\n", file)
		return 0
	}
	defer fp.Close()

	fp.WriteString(HEADXML)
	fp.WriteString(fmt.Sprintf(HEADGPX, "gnssgo ", XMLNS))

	/* output waypoint */
	if outpnt > 0 {
		for i = 0; i < solbuf.N; i++ {
			Ecef2Pos(solbuf.Data[i].Rr[:], pos[:])
			OutPoint(fp, solbuf.Data[i].Time, pos[:], "", int(solbuf.Data[i].Stat), outalt,
				outtime)
		}
	}
	/* output waypoint of ref position */
	if Norm(solbuf.Rb[:], 3) > 0.0 {
		Ecef2Pos(solbuf.Rb[:], pos[:])
		OutPoint(fp, solbuf.Data[0].Time, pos[:], "Reference Position", 0, outalt, 0)
	}
	/* output track */
	if outtrk > 0 {
		OutTrack(fp, solbuf, outalt, outtime)
	}
	fp.WriteString(fmt.Sprintf("%s\n", TAILGPX))

	return 1
}

/* convert to GPX file ---------------------------------------------------------
* convert solutions to GPX file [1]
* args   : char   *infile   I   input solutions file
*          char   *outfile  I   output google earth kml file ("":<infile>.kml)
*          gtime_t ts,te    I   start/end time (gpst)
*          int    tint      I   time interval (s) (0.0:all)
*          int    qflg      I   quality flag (0:all)
*          double *offset   I   add offset {east,north,up} (m)
*          int    outtrk    I   output track    (0:off,1:on)
*          int    outpnt    I   output waypoint (0:off,1:on)
*          int    outalt    I   output altitude (0:off,1:elipsoidal,2:geodetic)
*          int    outtime   I   output time (0:off,1:gpst,2:utc,3:jst)
* return : status (0:ok,-1:file read,-2:file format,-3:no data,-4:file write)
*-----------------------------------------------------------------------------*/
func ConvGpx(infile, outfile string, ts, te Gtime, tint float64, qflg int, offset []float64,
	outtrk, outpnt, outalt, outtime int) int {
	var (
		solbuf      SolBuf
		rr, pos, dr [3]float64
		i, j        int
		file        string
	)

	Trace(4, "convgpx : infile=%s outfile=%s\n", infile, outfile)
	if len(outfile) == 0 {
		index := strings.LastIndex(infile, ".")
		if index > 0 {
			file = infile[:index]
		}
		file += ".gpx"
	} else {
		file = outfile
	}

	var files []string = []string{infile}
	/* read solution file */
	if ReadSolt(files, 1, ts, te, tint, qflg, &solbuf) == 0 {
		return -1
	}

	/* mean position */
	for i = 0; i < 3; i++ {
		for j = 0; j < solbuf.N; j++ {
			rr[i] += solbuf.Data[j].Rr[i]
		}
		rr[i] /= float64(solbuf.N)
	}
	/* add offset */
	Ecef2Pos(rr[:], pos[:])
	Enu2Ecef(pos[:], offset, dr[:])
	for i = 0; i < solbuf.N; i++ {
		for j = 0; j < 3; j++ {
			solbuf.Data[i].Rr[j] += dr[j]
		}
	}
	if Norm(solbuf.Rb[:], 3) > 0.0 {
		for i = 0; i < 3; i++ {
			solbuf.Rb[i] += dr[i]
		}
	}
	/* save gpx file */
	if SaveGpx(file, &solbuf, outtrk, outpnt, outalt, outtime) > 0 {
		return 0
	} else {
		return -4
	}
}
