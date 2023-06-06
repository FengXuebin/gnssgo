/*------------------------------------------------------------------------------
* pos2kml.c : convert positions to google earth KML or GPX file
*
*          Copyright (C) 2007-2016 by T.TAKASU, All rights reserved.
*
* version : $Revision: 1.1 $ $Date: 2008/07/17 21:54:53 $
* history : 2007/01/20  1.0 new
*           2007/03/15  1.1 modify color sequence
*           2007/04/03  1.2 add geodetic height option
*                           support input of NMEA GGA sentence
*                           delete altitude info for track
*                           add time stamp option
*                           separate readsol.c file
*           2008/07/18  1.3 support change of convkml() arguments
*           2016/06/11  1.4 add option -gpx for gpx conversion
*-----------------------------------------------------------------------------*/
package main

import (
	"flag"
	"fmt"
	"os"
	"strings"

	"gnssgo"
	"strconv"
)

/* help text -----------------------------------------------------------------*/
var help = []string{
	"",
	" usage: pos2kml [option]... file [...]",
	"",
	" Read solution file(s) and convert it to Google Earth KML file or GPX file.",
	" Each line in the input file shall contain fields of time, position fields ",
	" (latitude/longitude/height or x/y/z-ecef), and quality flag(option). The line",
	" started with '%', '#', ';' is treated as comment. Command options are as ",
	" follows. ([]:default)",
	"",
	" -h        print help",
	" -ts ds,ts start day/time (ds=y/m/d ts=h:m:s) [obs start time]",
	" -te de,te end day/time   (de=y/m/d te=h:m:s) [obs end time]",
	" -o file   output file [infile + .kml]",
	" -c color  track color (0:off,1:white,2:green,3:orange,4:red,5:yellow) [5]",
	" -p color  point color (0:off,1:white,2:green,3:orange,4:red,5:by qflag) [5]",
	" -a        output altitude information [off]",
	" -ag       output geodetic altitude [off]",
	" -tg       output time stamp of gpst [off]",
	" -tu       output time stamp of utc [gpst]",
	" -i tint   output time interval (s) (0:all) [0]",
	" -q qflg   output q-flags (0:all) [0]",
	" -f n,e,h  add north/east/height offset to position (m) [0 0 0]",
	" -gpx      output GPX file",
}

/* print help ----------------------------------------------------------------*/
func printhelp() {
	for i := range help {
		fmt.Fprintf(os.Stderr, "%s\n", help[i])
	}
	os.Exit(0)
}

func searchHelp(key string) string {
	for _, v := range help {
		if strings.Contains(v, key) {
			return v
		}
	}
	return "no surported augument"
}

type timeFlag struct {
	time       *gnssgo.Gtime
	configured bool
}

func (f *timeFlag) Set(s string) error {
	var es []float64 = []float64{2000, 1, 1, 0, 0, 0}
	n, _ := fmt.Sscanf(s, "%f/%f/%f,%f:%f:%f", &es[0], &es[1], &es[2], &es[3], &es[4], &es[5])
	if n < 6 {
		return fmt.Errorf("too few argument")
	}
	*(f.time) = gnssgo.Epoch2Time(es)
	f.configured = true
	return nil
}
func (f *timeFlag) String() string {
	return "2000/1/1,0:0:0"
}
func newGtime(p *gnssgo.Gtime) *timeFlag {
	tf := timeFlag{p, false}
	return &tf
}

type posFlag struct {
	pos        []float64
	configured bool
}

func (f *posFlag) Set(s string) error {
	values := strings.Split(s, ",")
	if len(values) < 3 {
		return fmt.Errorf("too few arguments")
	}
	for i, v := range values {
		posFlag(*f).pos[i], _ = strconv.ParseFloat(v, 64)
	}
	f.configured = true
	return nil
}
func (f *posFlag) String() string {
	return "0,0,0"
}

func newFloatSlice(value []float64, p *[]float64) *posFlag {
	pf := posFlag{value, false}
	*p = pf.pos
	return &pf
}

/* pos2kml main --------------------------------------------------------------*/
func main() {
	var (
		i, n, outalt, outtime, qflg int
		tcolor, pcolor              int = 5, 5
		stat                        int
		infiles                     []string
		outfile                     string
		offset                      [3]float64
		tint                        float64
		ts, te                      gnssgo.Gtime
		ba, bag, btg, btu, bgpx     bool
	)

	flag.Var(newGtime(&ts), "ts", searchHelp("-ts"))
	flag.Var(newGtime(&te), "te", searchHelp("-te"))
	flag.StringVar(&outfile, "o", outfile, searchHelp("-o"))
	flag.IntVar(&tcolor, "c", tcolor, searchHelp("-c"))
	flag.IntVar(&pcolor, "p", pcolor, searchHelp("-p"))
	rp := offset[:]
	rpFlag := newFloatSlice([]float64{}, &rp)
	flag.Var(rpFlag, "f", searchHelp("-f"))

	flag.BoolVar(&ba, "a", false, searchHelp("-a"))
	flag.BoolVar(&bag, "ag", false, searchHelp("-ag"))
	flag.BoolVar(&btg, "tg", false, searchHelp("-tg"))
	flag.BoolVar(&btu, "tu", false, searchHelp("-tu"))
	flag.Float64Var(&tint, "i", tint, searchHelp("-i"))
	flag.IntVar(&qflg, "q", qflg, searchHelp("-q"))
	flag.BoolVar(&bgpx, "gpx", false, searchHelp("-gpx"))

	flag.Parse()

	if flag.NFlag() < 1 {
		// if there is not any arguments, exit
		for _, h := range help {
			fmt.Printf("%s\n", h)
		}
		return
	}
	if ba {
		outalt = 1
	}
	if bag {
		outalt = 2
	}
	if btg {
		outtime = 1
	}
	if btu {
		outtime = 2
	}

	infiles = flag.CommandLine.Args()
	if n = len(infiles); n < 1 {
		fmt.Fprintf(os.Stderr, "pos2kml : no input file\n")
		os.Exit(-1)
	}

	if tcolor < 0 || 5 < tcolor || pcolor < 0 || 5 < pcolor {
		fmt.Fprintf(os.Stderr, "pos2kml : command option error\n")
		os.Exit(-1)
	}
	for i = 0; i < n; i++ {
		if bgpx {
			stat = gnssgo.ConvGpx(infiles[i], outfile, ts, te, tint, qflg, offset[:], tcolor, pcolor,
				outalt, outtime)
		} else {
			stat = gnssgo.ConvKml(infiles[i], outfile, ts, te, tint, qflg, offset[:], tcolor, pcolor,
				outalt, outtime)
		}
		switch stat {
		case -1:
			fmt.Fprintf(os.Stderr, "pos2kml : file read error (%d)\n", i+1)
		case -2:
			fmt.Fprintf(os.Stderr, "pos2kml : file format error (%d)\n", i+1)
		case -3:
			fmt.Fprintf(os.Stderr, "pos2kml : no input data (%d)\n", i+1)
		case -4:
			fmt.Fprintf(os.Stderr, "pos2kml : file write error (%d)\n", i+1)
		}
	}
}
