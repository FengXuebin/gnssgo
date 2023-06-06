/*------------------------------------------------------------------------------
* convkml.c : google earth kml converter
*
*          Copyright (C) 2007-2017 by T.TAKASU, All rights reserved.
*
* references :
*     [1] Open Geospatial Consortium Inc., OGC 07-147r2, OGC(R) KML, 2008-04-14
*
* version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
* history : 2007/01/20  1.0  new
*           2007/03/15  1.1  modify color sequence
*           2007/04/03  1.2  add geodetic height option
*                            support input of NMEA GGA sentence
*                            delete altitude info for track
*                            add time stamp option
*                            separate readsol.c file
*           2009/01/19  1.3  fix bug on display mark with by-q-flag option
*           2010/05/10  1.4  support api readsolt() change
*           2010/08/14  1.5  fix bug on readsolt() (2.4.0_p3)
*           2017/06/10  1.6  support wild-card in input file
*		    2022/05/31  1.0  rewrite convkml.c with golang by fxb
*-----------------------------------------------------------------------------*/

package gnssgo

import (
	"fmt"
	"math"
	"os"
	"strings"
)

/* constants -----------------------------------------------------------------*/

const (
	SIZP = 0.2  /* mark size of rover positions */
	SIZR = 0.3  /* mark size of reference position */
	TINT = 60.0 /* time label interval (sec) */

	head1 = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>"
	head2 = "<kml xmlns=\"http://earth.google.com/kml/2.1\">"
	mark  = "http://maps.google.com/mapfiles/kml/pal2/icon18.png"
)

/* output track --------------------------------------------------------------*/
func OutTrackKml(f *os.File, solbuf *SolBuf, color string, outalt, outtime int) {
	var pos [3]float64

	f.WriteString("<Placemark>\n")
	f.WriteString("<name>Rover Track</name>\n")
	f.WriteString("<Style>\n")
	f.WriteString("<LineStyle>\n")
	f.WriteString(fmt.Sprintf("<color>%s</color>\n", color))
	f.WriteString("</LineStyle>\n")
	f.WriteString("</Style>\n")
	f.WriteString("<LineString>\n")
	if outalt > 0 {
		f.WriteString("<altitudeMode>absolute</altitudeMode>\n")
	}
	f.WriteString("<coordinates>\n")
	for i := 0; i < solbuf.N; i++ {
		Ecef2Pos(solbuf.Data[i].Rr[:], pos[:])
		if outalt == 0 {
			pos[2] = 0.0
		} else if outalt == 2 {
			pos[2] -= GeoidH(pos[:])
		}
		f.WriteString(fmt.Sprintf("%13.9f,%12.9f,%5.3f\n", pos[1]*R2D, pos[0]*R2D, pos[2]))
	}
	f.WriteString("</coordinates>\n")
	f.WriteString("</LineString>\n")
	f.WriteString("</Placemark>\n")
}

/* output point --------------------------------------------------------------*/
func OutPointKml(fp *os.File, time Gtime, pos []float64, label string, style, outalt, outtime int) {
	var (
		ep  [6]float64
		alt float64 = 0.0
		str string
	)

	fp.WriteString("<Placemark>\n")
	if label != "" {
		fp.WriteString(fmt.Sprintf("<name>%s</name>\n", label))
	}
	fp.WriteString(fmt.Sprintf("<styleUrl>#P%d</styleUrl>\n", style))
	if outtime > 0 {
		if outtime == 2 {
			time = GpsT2Utc(time)
		} else if outtime == 3 {
			time = TimeAdd(GpsT2Utc(time), 9*3600.0)
		}
		Time2Epoch(time, ep[:])
		if !(len(label) == 0 && math.Mod(ep[5]+0.005, TINT) < 0.01) {
			str = fmt.Sprintf("%02.0f:%02.0f", ep[3], ep[4])
			fp.WriteString(fmt.Sprintf("<name>%s</name>\n", str))
		}
		str = fmt.Sprintf("%04.0f-%02.0f-%02.0fT%02.0f:%02.0f:%05.2fZ",
			ep[0], ep[1], ep[2], ep[3], ep[4], ep[5])
		fp.WriteString(fmt.Sprintf("<TimeStamp><when>%s</when></TimeStamp>\n", str))
	}
	fp.WriteString("<Point>\n")
	if outalt > 0 {
		fp.WriteString("<extrude>1</extrude>\n")
		fp.WriteString("<altitudeMode>absolute</altitudeMode>\n")
		alt = pos[2]
		if outalt == 2 {
			alt -= GeoidH(pos)
		}
	}
	fp.WriteString(fmt.Sprintf("<coordinates>%13.9f,%12.9f,%5.3f</coordinates>\n", pos[1]*R2D,
		pos[0]*R2D, alt))
	fp.WriteString("</Point>\n")
	fp.WriteString("</Placemark>\n")
}

/* save kml file -------------------------------------------------------------*/
func SaveKml(file string, solbuf *SolBuf, tcolor, pcolor, outalt, outtime int) int {
	var (
		pos    [3]float64
		qcolor []int    = []int{0, 1, 2, 5, 4, 3, 0}
		color  []string = []string{
			"ffffffff", "ff008800", "ff00aaff", "ff0000ff", "ff00ffff", "ffff00ff"}
	)

	fp, err := os.OpenFile(file, os.O_CREATE|os.O_WRONLY|os.O_TRUNC, os.ModeAppend|os.ModePerm)
	if err != nil {
		Trace(2, "file open error : %s\n", file)
		return 0
	}
	defer fp.Close()

	fp.WriteString(fmt.Sprintf("%s\n%s\n", head1, head2))
	fp.WriteString("<Document>\n")
	for i := 0; i < 6; i++ {
		fp.WriteString(fmt.Sprintf("<Style id=\"P%d\">\n", i))
		fp.WriteString("  <IconStyle>\n")
		fp.WriteString(fmt.Sprintf("    <color>%s</color>\n", color[i]))
		if i == 0 {
			fp.WriteString(fmt.Sprintf("    <scale>%.1f</scale>\n", SIZR))
		} else {
			fp.WriteString(fmt.Sprintf("    <scale>%.1f</scale>\n", SIZP))
		}
		fp.WriteString(fmt.Sprintf("    <Icon><href>%s</href></Icon>\n", mark))
		fp.WriteString("  </IconStyle>\n")
		fp.WriteString("</Style>\n")
	}
	if tcolor > 0 {
		OutTrackKml(fp, solbuf, color[tcolor-1], outalt, outtime)
	}
	if pcolor > 0 {
		fp.WriteString("<Folder>\n")
		fp.WriteString("  <name>Rover Position</name>\n")
		for i := 0; i < solbuf.N; i++ {
			Ecef2Pos(solbuf.Data[i].Rr[:], pos[:])
			if pcolor == 5 {
				OutPointKml(fp, solbuf.Data[i].Time, pos[:], "",
					qcolor[solbuf.Data[i].Stat], outalt, outtime)
			} else {
				OutPointKml(fp, solbuf.Data[i].Time, pos[:], "",
					pcolor-1, outalt, outtime)
			}
		}
		fp.WriteString("</Folder>\n")
	}
	if Norm(solbuf.Rb[:], 3) > 0.0 {
		Ecef2Pos(solbuf.Rb[:], pos[:])
		OutPointKml(fp, solbuf.Data[0].Time, pos[:], "Reference Position", 0, outalt, 0)
	}
	fp.WriteString("</Document>\n")
	fp.WriteString("</kml>\n")

	return 1
}

/* convert to google earth kml file --------------------------------------------
* convert solutions to google earth kml file
* args   : char   *infile   I   input solutions file (wild-card (*) is expanded)
*          char   *outfile  I   output google earth kml file ("":<infile>.kml)
*          gtime_t ts,te    I   start/end time (gpst)
*          int    tint      I   time interval (s) (0.0:all)
*          int    qflg      I   quality flag (0:all)
*          double *offset   I   add offset {east,north,up} (m)
*          int    tcolor    I   track color
*                               (0:none,1:white,2:green,3:orange,4:red,5:yellow)
*          int    pcolor    I   point color
*                               (0:none,1:white,2:green,3:orange,4:red,5:by qflag)
*          int    outalt    I   output altitude (0:off,1:elipsoidal,2:geodetic)
*          int    outtime   I   output time (0:off,1:gpst,2:utc,3:jst)
* return : status (0:ok,-1:file read,-2:file format,-3:no data,-4:file write)
* notes  : see ref [1] for google earth kml file format
*-----------------------------------------------------------------------------*/
func ConvKml(infile, outfile string, ts, te Gtime, tint float64, qflg int, offset []float64,
	tcolor, pcolor, outalt, outtime int) int {
	var (
		solbuf                   SolBuf
		rr, pos, dr              [3]float64
		i, j, nfile, stat, index int
		files                    []string = make([]string, MAXEXFILE)
		file                     string
	)

	Trace(4, "convkml : infile=%s outfile=%s\n", infile, outfile)

	/* expand wild-card of infile */

	if nfile = ExPath(infile, files, MAXEXFILE); nfile <= 0 {

		return -3
	}
	if len(outfile) == 0 {
		index = strings.LastIndex(infile, ".")
		if index >= 0 {
			file = infile[:index]
			file += ".kml"
		} else {
			file = fmt.Sprintf("%s.kml", infile)
		}

	} else {
		file = outfile
	}

	/* read solution file */
	stat = ReadSolt(files, nfile, ts, te, tint, qflg, &solbuf)

	if stat == 0 {
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
	/* save kml file */
	if SaveKml(file, &solbuf, tcolor, pcolor, outalt, outtime) > 0 {
		return 0
	}
	return -4
}
