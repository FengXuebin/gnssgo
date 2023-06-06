//---------------------------------------------------------------------------
// rtkplot : visualization of solution and obs data ap
//
//          Copyright (C) 2007-2012 by T.TAKASU, All rights reserved.
//          ported to Qt by Jens Reimann
//
// options : rtkplot [-t title][-i file][-r][-p path][-x level][file ...]
//
//           -t title  window title
//           -i file   ini file path
//           -r        open file as obs and nav file
//           -p path   connect to path
//                       serial://port[:brate[:bsize[:parity[:stopb[:fctr]]]]]
//                       tcpsvr://:port
//                       tcpcli://addr[:port]
//                       ntrip://[user[:passwd]@]addr[:port][/mntpnt]
//                       file://path
//           -p1 path  connect port 1 to path
//           -p2 path  connect port 2 to path
//           -x level  debug trace level (0:off)
//           file      solution files or rinex obs/nav file
//
// version : $Revision: 1.1 $ $Date: 2008/07/17 22:15:27 $
// history : 2008/07/14  1.0 new
//           2009/11/27  1.1 rtklib 2.3.0
//           2010/07/18  1.2 rtklib 2.4.0
//           2010/06/10  1.3 rtklib 2.4.1
//           2010/06/19  1.4 rtklib 2.4.1 p1
//           2012/11/21  1.5 rtklib 2.4.2
//			 2022/06/27  1.0 gnssgo by fxb
//			 read solution and observation data, export to Promethues
//---------------------------------------------------------------------------
package main

import (
	"context"
	"flag"
	"fmt"
	"gnssgo"
	"net/http"
	"os"
	"strings"
	"time"

	//	"github.com/prometheus/client_golang/prometheus/promhttp"
	influxdb "github.com/influxdata/influxdb-client-go/v2"
	"github.com/prometheus/client_golang/prometheus"
	"github.com/prometheus/client_golang/prometheus/push"
)

const (
	MAXNFILE = 256
	PROGNAME = "plot"
)

/* help text -----------------------------------------------------------------*/
var help []string = []string{
	"",
	" usage: plot [option]... file file [...]",
	"",
	" Read RINEX OBS/NAV/GNAV/HNAV/CLK, SP3, SBAS message log files and position solutions. ",
	"",
	" -?        print help",
	" -k file   input options from configuration file [off]",
	" -ts ds,ts start day/time (ds=y/m/d ts=h:m:s) [obs start time]",
	" -te de,te end day/time   (de=y/m/d te=h:m:s) [obs end time]",
	" -ti tint  time interval (sec) [all]",
	" -x level  debug trace level (0:off) [0]"}

func searchHelp(key string) string {
	for _, v := range help {
		if strings.Contains(v, key) {
			return v
		}
	}
	return "no surported augument"
}

var ch chan string

func OutMetrics(solbuf *gnssgo.SolBuf, name string) []prometheus.Collector {
	var ret []prometheus.Collector
	for i := 0; i < solbuf.N; i++ {
		posmetrics := prometheus.NewGaugeVec(
			prometheus.GaugeOpts{
				Name: "gnssgo_solution_Geodetic_Postion",
				Help: "geodetic postion of solution",
			},
			[]string{
				"rcvName",
				"latitude",
				"longitude",
			},
		)
		OutSolMetrics(&solbuf.Data[i], name, posmetrics)
		ret = append(ret, *posmetrics)
	}
	return ret
}

/* output solution metrics ------------------------------------------------------
*  output solution data to metrics
*-------------------------------------------------------------------------------*/
func OutSolMetrics(sol *gnssgo.Sol, name string, metrics *prometheus.GaugeVec) {
	var (
		// ret []Metrics
		s   string
		pos [3]float64
	)
	if sol == nil {
		return
	}
	gnssgo.Time2Str(sol.Time, &s, 3)
	gnssgo.Ecef2Pos(sol.Rr[:3], pos[:])
	metrics.WithLabelValues(name, fmt.Sprintf("%f", pos[0]*gnssgo.R2D),
		fmt.Sprintf("%f", pos[1]*gnssgo.R2D)).Add(float64(sol.Time.Time))
	// posmetrics.WithLabelValues(name, "x", s).Set(sol.Rr[0])
	// posmetrics.WithLabelValues(name, "y", s).Set(sol.Rr[1])
	// posmetrics.WithLabelValues(name, "z", s).Set(sol.Rr[2])
	// posmetrics.WithLabelValues(name, "vx", s).Set(sol.Rr[3])
	// posmetrics.WithLabelValues(name, "vy", s).Set(sol.Rr[4])
	// posmetrics.WithLabelValues(name, "vz", s).Set(sol.Rr[5])
	// posmetrics.WithLabelValues(name, "x position variance/covariance", s).Set(float64(sol.Qr[0]))
	// posmetrics.WithLabelValues(name, "y position variance/covariance", s).Set(float64(sol.Qr[1]))
	// posmetrics.WithLabelValues(name, "z position variance/covariance", s).Set(float64(sol.Qr[2]))
	// posmetrics.WithLabelValues(name, "vx position variance/covariance", s).Set(float64(sol.Qr[3]))
	// posmetrics.WithLabelValues(name, "vy position variance/covariance", s).Set(float64(sol.Qr[4]))
	// posmetrics.WithLabelValues(name, "vz position variance/covariance", s).Set(float64(sol.Qr[5]))

	// posmetrics.WithLabelValues(name, "x velocity variance/covariance", s).Set(float64(sol.Qv[0]))
	// posmetrics.WithLabelValues(name, "y velocity variance/covariance", s).Set(float64(sol.Qv[1]))
	// posmetrics.WithLabelValues(name, "z velocity variance/covariance", s).Set(float64(sol.Qv[2]))
	// posmetrics.WithLabelValues(name, "vx velocity variance/covariance", s).Set(float64(sol.Qv[3]))
	// posmetrics.WithLabelValues(name, "vy velocity variance/covariance", s).Set(float64(sol.Qv[4]))
	// posmetrics.WithLabelValues(name, "vz velocity variance/covariance", s).Set(float64(sol.Qv[5]))

	// posmetrics.WithLabelValues(name, "dtr0 receiver clock bias to time systems", s).Set(sol.Dtr[0])
	// posmetrics.WithLabelValues(name, "dtr1 receiver clock bias to time systems", s).Set(sol.Dtr[1])
	// posmetrics.WithLabelValues(name, "dtr2 receiver clock bias to time systems", s).Set(sol.Dtr[2])
	// posmetrics.WithLabelValues(name, "dtr3 receiver clock bias to time systems", s).Set(sol.Dtr[3])
	// posmetrics.WithLabelValues(name, "dtr4 receiver clock bias to time systems", s).Set(sol.Dtr[4])
	// posmetrics.WithLabelValues(name, "dtr5 receiver clock bias to time systems", s).Set(sol.Dtr[5])

	// posmetrics.WithLabelValues(name, "type", s).Set(float64(sol.Type))
	// posmetrics.WithLabelValues(name, "solution status", s).Set(float64(sol.Stat))
	// posmetrics.WithLabelValues(name, "number of valid satellites", s).Set(float64(sol.Ns))
	// posmetrics.WithLabelValues(name, "age of differential", s).Set(float64(sol.Age))
	// posmetrics.WithLabelValues(name, "AR ratio factor for valiation", s).Set(float64(sol.Ratio))
	// posmetrics.WithLabelValues(name, "AR ratio threshold for valiation", s).Set(float64(sol.Thres))
}

// read solutions -----------------------------------------------------------
func ReadSol(ts, te gnssgo.Gtime, tint float64, files []string) {
	var (
		sol     gnssgo.SolBuf
		solstat gnssgo.SolStatBuf
		n       int
		paths   [MAXNFILE]string
	)

	if len(files) <= 0 {
		return
	}

	n = len(files)
	showmsg("start read solution data...\n")
	if gnssgo.ReadSolt(files, n, ts, te, tint, 0, &sol) == 0 {
		showmsg("no solution data : 1%s...", paths[0])
		return
	}

	solstat.ReadSolStatt(files, n, ts, te, tint)

	// metrics := OutMetrics(&sol, "recv0")
	//	OutPostion(&sol)
	OutENU(&sol)
	showmsg("finished read solution data.\n")

	// for v := range metrics {
	// 	PushGaugeMetric(metrics[v])
	// }

}

func OutPostion(sol *gnssgo.SolBuf) {
	// Create a new client using an InfluxDB server base URL and an authentication token
	//	client := influxdb.NewClient("http://localhost:8086", "infux")
	client := influxdb.NewClient("http://localhost:8086", "qdhQU9SHk2xlWZGwA9UxxQDdSjFfeGJOfVLWjACQ1isHgWCZye7bziwPF00AAqb9jd2QuszxksmedJ98CZ21Sw==")
	// client = influxdb.NewClientWithOptions("http://localhost:8086", "qdhQU9SHk2xlWZGwA9UxxQDdSjFfeGJOfVLWjACQ1isHgWCZye7bziwPF00AAqb9jd2QuszxksmedJ98CZ21Sw==",
	// 	influxdb.DefaultOptions().SetBatchSize(1000))
	// get non-blocking write client
	writeAPI := client.WriteAPI("idtsz", "gnssgo")

	// write line protocol
	//	writeAPI.WriteRecord(fmt.Sprintf("stat,unit=temperature avg=%f,max=%f", 23.5, 45.0))
	var (
		pos     [3]float64
		timeStr string
	)
	for i := 0; i < sol.N; i++ {
		gnssgo.Time2Str(sol.Time, &timeStr, 3)
		timeObj := time.Unix(int64(sol.Data[i].Time.Time), 0)
		gnssgo.Ecef2Pos(sol.Data[i].Rr[:3], pos[:])
		p := influxdb.NewPointWithMeasurement("solution").
			AddTag("status", fmt.Sprintf("%d", sol.Data[i].Stat)).
			AddField("latitude", pos[0]*gnssgo.R2D).
			AddField("langitude", pos[1]*gnssgo.R2D).
			AddField("height", pos[2]).
			// SetTime(time.Now()) // Flush writes
			SetTime(timeObj) // Flush writes

		writeAPI.WritePoint(p)
		// if i%1000 == 1 {
		// }
	}
	writeAPI.Flush()
	// Get query client
	queryAPI := client.QueryAPI("idtsz")

	query := `from(bucket:"gnssgo")|> range(start: -1h) |> filter(fn: (r) => r._measurement == "solution")`

	// get QueryTableResult
	result, err := queryAPI.Query(context.Background(), query)
	if err != nil {
		panic(err)
	}

	// Iterate over query response
	for result.Next() {
		// Notice when group key has changed
		if result.TableChanged() {
			fmt.Printf("table: %s\n", result.TableMetadata().String())
		}
		// Access data
		fmt.Printf("value: %v\n", result.Record().Value())
	}
	// check for an error
	if result.Err() != nil {
		fmt.Printf("query parsing error: %\n", result.Err().Error())
	}

	client.Close()
}

func OutENU(sol *gnssgo.SolBuf) {
	// Create a new client using an InfluxDB server base URL and an authentication token
	client := influxdb.NewClient("http://localhost:8086",
		"qdhQU9SHk2xlWZGwA9UxxQDdSjFfeGJOfVLWjACQ1isHgWCZye7bziwPF00AAqb9jd2QuszxksmedJ98CZ21Sw==")

	// get non-blocking write client
	writeAPI := client.WriteAPI("idtsz", "gnssgo")

	var (
		pos, enu [3]float64
		timeStr  string
	)
	for i := 0; i < sol.N; i++ {
		gnssgo.Time2Str(sol.Time, &timeStr, 3)
		timeObj := time.Unix(int64(sol.Data[i].Time.Time), 0)
		gnssgo.Ecef2Pos(sol.Data[i].Rr[:3], pos[:])
		gnssgo.Ecef2Enu(pos[:], sol.Rb[:], enu[:])
		p := influxdb.NewPointWithMeasurement("PositionENU").
			AddTag("status", fmt.Sprintf("%d", sol.Data[i].Stat)).
			AddField("enux", enu[0]).
			AddField("enuy", enu[1]).
			AddField("enuz", enu[2]).
			SetTime(timeObj) // Flush writes

		writeAPI.WritePoint(p)
	}
	writeAPI.Flush()
	client.Close()
}

/* show message --------------------------------------------------------------*/
func showmsg(format string, v ...interface{}) int {
	fmt.Fprintf(os.Stderr, format, v...)
	if len(format) > 0 {
		fmt.Fprintf(os.Stderr, "\r")
	} else {
		fmt.Fprintf(os.Stderr, "\n")
	}
	return 0
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

func ExporterHandler(w http.ResponseWriter, r *http.Request) {
	metrics := <-ch
	//fmt.Fprintf(w, "lexporter_request_count{user=\"admin\"}")
	fmt.Fprintf(w, metrics)
}

var pusher *push.Pusher

func PushGaugeMetric(data prometheus.Collector) {
	if err := push.New("http://127.0.0.1:9091", "gnssgo_sol").Collector(data).Push(); err != nil {
		fmt.Println("Could not push solution time to Pushgateway:", err)
	}
}

func TestInflux() {
	// Create a new client using an InfluxDB server base URL and an authentication token
	//	client := influxdb.NewClient("http://localhost:8086", "infux")
	client := influxdb.NewClient("http://localhost:8086", "qdhQU9SHk2xlWZGwA9UxxQDdSjFfeGJOfVLWjACQ1isHgWCZye7bziwPF00AAqb9jd2QuszxksmedJ98CZ21Sw==")
	// get non-blocking write client
	writeAPI := client.WriteAPI("idtsz", "gnssgo")

	// write line protocol
	//	writeAPI.WriteRecord(fmt.Sprintf("stat,unit=temperature avg=%f,max=%f", 23.5, 45.0))
	p := influxdb.NewPointWithMeasurement("stat").
		AddTag("unit", "temperature").
		AddField("avg", 34.7).
		AddField("max", 75.0).
		SetTime(time.Now()) // Flush writes
	writeAPI.WritePoint(p)
	writeAPI.Flush()
	// Get query client
	queryAPI := client.QueryAPI("idtsz")

	query := `from(bucket:"gnssgo")|> range(start: -1h) |> filter(fn: (r) => r._measurement == "stat")`

	// get QueryTableResult
	result, err := queryAPI.Query(context.Background(), query)
	if err != nil {
		panic(err)
	}

	// Iterate over query response
	for result.Next() {
		// Notice when group key has changed
		if result.TableChanged() {
			fmt.Printf("table: %s\n", result.TableMetadata().String())
		}
		// Access data
		fmt.Printf("value: %v\n", result.Record().Value())
	}
	// check for an error
	if result.Err() != nil {
		fmt.Printf("query parsing error: %\n", result.Err().Error())
	}

	// Ensures background processes finishes
	client.Close()
}

func main() {
	var (
		filopt  gnssgo.FilOpt
		tint    float64 = 0.0
		ts, te  gnssgo.Gtime
		n       int
		infiles []string
	)
	prcopt := gnssgo.DefaultProcOpt()
	solopt := gnssgo.DefaultSolOpt()

	prcopt.Mode = gnssgo.PMODE_KINEMA
	prcopt.NavSys = 0
	prcopt.RefPos = 1
	prcopt.GloModeAr = 1
	solopt.TimeF = 0
	solopt.Prog = fmt.Sprintf("%s ver.%s %s", PROGNAME, gnssgo.VER_GNSSGO, gnssgo.PATCH_LEVEL)
	filopt.Trace = fmt.Sprintf("%s.trace", PROGNAME)

	gnssgo.ShowMsg_Ptr = showmsg

	flag.Var(newGtime(&ts), "ts", searchHelp("-ts"))
	flag.Var(newGtime(&te), "te", searchHelp("-te"))
	flag.Float64Var(&tint, "ti", tint, searchHelp("-ti"))
	flag.IntVar(&solopt.Trace, "x", solopt.Trace, searchHelp("-x"))

	flag.Parse()

	if flag.NFlag() < 1 {
		// if there is not any arguments, exit
		for _, h := range help {
			fmt.Printf("%s\n", h)
		}
		return
	}

	infiles = flag.CommandLine.Args()
	if n = len(infiles); n < 1 {
		return
	}

	// ch = make(chan string)
	// //Create a new instance of the foocollector and
	// //register it with the prometheus client.
	// foo := newFooCollector()
	// prometheus.MustRegister(foo)

	// //This section will start the HTTP server and expose
	// //any metrics on the /metrics endpoint.
	// http.Handle("/metrics", promhttp.Handler())
	// stopCh := make(chan bool)

	// // http.HandleFunc("/metrics", ExporterHandler)
	// // http.Handle("/metrics", promhttp.Handler())
	// TestInflux()
	ReadSol(ts, te, tint, infiles)
	// go http.ListenAndServe(":8000", nil)
	// for {
	// 	if <-stopCh {
	// 		fmt.Fprintf(os.Stderr, "Shutting down windows_exporter")
	// 		break
	// 	}
	// }
}
