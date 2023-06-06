/*------------------------------------------------------------------------------
* gnssgo unit test driver : rinex function
*-----------------------------------------------------------------------------*/
package gnss_test

import (
	"fmt"
	"gnssgo"
	"os"
	"testing"

	"github.com/stretchr/testify/assert"
)

func dumpobs(obs *gnssgo.Obs, t *testing.T) {
	var time gnssgo.Gtime
	var i int
	var str string
	assert := assert.New(t)
	fmt.Printf("obs : n=%d\n", obs.N())
	for i = 0; i < obs.N(); i++ {
		gnssgo.Time2Str(obs.Data[i].Time, &str, 3)
		fmt.Printf("%s : %2d %2d %13.3f %13.3f %13.3f %13.3f  %d %d\n", str, obs.Data[i].Sat,
			obs.Data[i].Rcv, obs.Data[i].L[0], obs.Data[i].L[1],
			obs.Data[i].P[0], obs.Data[i].P[1], obs.Data[i].LLI[0], obs.Data[i].LLI[1])

		assert.True(1 <= obs.Data[i].Sat && obs.Data[i].Sat <= 32)
		assert.True(gnssgo.TimeDiff(obs.Data[i].Time, time) >= float64(-gnssgo.DTTOL))

		time = obs.Data[i].Time
	}
}

func dumpnav(nav *gnssgo.Nav, t *testing.T) {
	var i int
	var str, s1, s2 string
	assert := assert.New(t)
	fmt.Printf("nav : n=%d\n", nav.N())
	for i = 0; i < nav.N(); i++ {
		gnssgo.Time2Str(nav.Ephs[i].Toe, &str, 3)
		gnssgo.Time2Str(nav.Ephs[i].Toc, &s1, 0)
		gnssgo.Time2Str(nav.Ephs[i].Ttr, &s2, 0)
		fmt.Printf("%s : %2d    %s %s %3d %3d %2d\n", str, nav.Ephs[i].Sat, s1, s2,
			nav.Ephs[i].Iode, nav.Ephs[i].Iodc, nav.Ephs[i].Svh)

		assert.True(nav.Ephs[i].Iode == (nav.Ephs[i].Iodc & 0xFF))
	}
}

func dumpsta(sta *gnssgo.Sta) {
	fmt.Printf("name    = %s\n", sta.Name)
	fmt.Printf("marker  = %s\n", sta.Marker)
	fmt.Printf("antdes  = %s\n", sta.AntDes)
	fmt.Printf("antsno  = %s\n", sta.AntSno)
	fmt.Printf("rectype = %s\n", sta.Type)
	fmt.Printf("recver  = %s\n", sta.RecVer)
	fmt.Printf("recsno  = %s\n", sta.RecSN)
	fmt.Printf("antsetup= %d\n", sta.AntSetup)
	fmt.Printf("itrf    = %d\n", sta.Itrf)
	fmt.Printf("deltype = %d\n", sta.DelType)
	fmt.Printf("pos     = %.3f %.3f %.3f\n", sta.Pos[0], sta.Pos[1], sta.Pos[2])
	fmt.Printf("del     = %.3f %.3f %.3f\n", sta.Del[0], sta.Del[1], sta.Del[2])
	fmt.Printf("hgt     = %.3f\n", sta.Hgt)
}

/* gnssgo.ReadRnx(), sortobs(), uniqnav()  */
func Test_renixutest1(t *testing.T) {
	var file1 string = "abc.00o"
	var file2 string = "bcd.00n"
	var file3 string = "../data/rinex/07590920.05o"
	var file4 string = "../data/rinex/07590920.05n"
	var file5 string = "../data/rinex/30400920.05o"
	var file6 string = "../data/rinex/30400920.05n"
	var obs gnssgo.Obs
	var nav gnssgo.Nav
	var sta gnssgo.Sta
	var n, stat int
	assert := assert.New(t)

	stat = gnssgo.ReadRnx(file1, 1, "", &obs, &nav, &sta)
	assert.True(stat == 0 && obs.N() == 0 && nav.N() == 0 && nav.Ng() == 0 && nav.Ns() == 0)
	stat = gnssgo.ReadRnx(file2, 1, "", &obs, &nav, &sta)
	assert.True(stat == 0 && obs.N() == 0 && nav.N() == 0 && nav.Ng() == 0 && nav.Ns() == 0)
	stat = gnssgo.ReadRnx(file3, 1, "", &obs, &nav, &sta)
	assert.True(stat == 1)
	stat = gnssgo.ReadRnx(file4, 1, "", &obs, &nav, &sta)
	assert.True(stat == 1)
	stat = gnssgo.ReadRnx(file5, 2, "", &obs, &nav, &sta)
	assert.True(stat == 1)
	stat = gnssgo.ReadRnx(file6, 2, "", &obs, &nav, &sta)
	assert.True(stat == 1)
	n = obs.SortObs()
	assert.True(n == 120)
	nav.UniqNav()
	assert.True(nav.N() == 167)
	dumpobs(&obs, t)
	dumpnav(&nav, t)
	dumpsta(&sta)
	assert.True(obs.Data != nil && obs.N() > 0 && nav.Ephs != nil && nav.N() > 0)
}

/* readrnxt() */
func Test_renixutest2(t *testing.T) {
	var t0, ts, te gnssgo.Gtime
	var ep1 []float64 = []float64{2005, 4, 2, 1, 0, 0}
	var ep2 []float64 = []float64{2005, 4, 2, 2, 0, 0}
	var file1 string = "../data/rinex/07590920.05o"
	var file2 string = "../data/rinex/07590920.05n"
	var n int
	var obs gnssgo.Obs
	var nav gnssgo.Nav
	var sta gnssgo.Sta

	ts = gnssgo.Epoch2Time(ep1)
	te = gnssgo.Epoch2Time(ep2)
	n = gnssgo.ReadRnxT(file1, 1, ts, te, 0.0, "", &obs, &nav, &sta)
	fmt.Printf("\n\nn=%d\n", n)
	n = gnssgo.ReadRnxT(file2, 1, ts, te, 0.0, "", &obs, &nav, &sta)
	dumpobs(&obs, t)
	obs.Data = nil
	n = gnssgo.ReadRnxT(file1, 1, t0, t0, 240.0, "", &obs, &nav, &sta)
	fmt.Printf("\n\nn=%d\n", n)
	dumpobs(&obs, t)
	obs.Data = nil
}

var opt1 gnssgo.RnxOpt = gnssgo.RnxOpt{}

// var opt2 gnssgo.RnxOpt = gnssgo.RnxOpt{}

var opt2 gnssgo.RnxOpt = gnssgo.RnxOpt{

	TInt:       0.0,
	TTol:       0.0,
	TUnit:      0.0,
	RnxVer:     310,
	NavSys:     gnssgo.SYS_ALL,
	ObsType:    gnssgo.OBSTYPE_ALL,
	FreqType:   gnssgo.FREQTYPE_ALL,
	Staid:      "STAID",
	Prog:       "RROG567890123456789012345678901",
	RunBy:      "RUNBY67890123456789012345678901",
	Marker:     "MARKER789012345678901234567890123456789012345678901234567890123",
	MarkerNo:   "MARKNO7890123456789012345678901",
	MarkerType: "MARKTY7890123456789012345678901",
	Name:       [2]string{"OBSERVER90123456789012345678901", "AGENCY7890123456789012345678901"},
	Rec:        [3]string{"RCV1567890123456789012345678901", "RCV2567890123456789012345678901", "RCV3567890123456789012345678901"},
	Ant:        [3]string{"ANT1567890123456789012345678901", "ANT2567890123456789012345678901", "ANT3567890123456789012345678901"},
	AppPos:     [3]float64{12345678.123, 99999999.999, 100000000.000},
	AntDel:     [3]float64{123.0345, 890123.9012, 34567.0001},
	Comment:    [100]string{"COMMENT1 012345678901234567890123456789012345678901234567890123", "COMMENT2 012345678901234567890123456789012345678901234567890123", "COMMENT3 012345678901234567890123456789012345678901234567890123", "COMMENT4 012345678901234567890123456789012345678901234567890123", "COMMENT5 012345678901234567890123456789012345678901234567890123"},
	RcvOpt:     "",
	Outiono:    1,
	OutputTime: 1,
	Outleaps:   1,
	AutoPos:    1,
	PhShift:    1}

/* outrneobsh() */
func Test_renixutest3(t *testing.T) {
	var nav gnssgo.Nav

	opt1.OutRnxObsHeader(os.Stdout, &nav)
	opt2.OutRnxObsHeader(os.Stdout, &nav)
}

/* outrneobsb() */
func Test_renixutest4(t *testing.T) {
	var file string = "../data/rinex/07590920.05o"
	var obs gnssgo.Obs
	var i, j int

	gnssgo.ReadRnx(file, 1, "", &obs, nil, nil)
	opt2.OutRnxObsBody(os.Stdout, obs.Data, 8, 9)
	opt2.OutRnxObsBody(os.Stdout, obs.Data, 8, 0)

	for i, j = 0, 0; i < obs.N(); i = j {
		for j < obs.N() && gnssgo.TimeDiff(obs.Data[j].Time, obs.Data[i].Time) <= 0.0 {
			j++
		}
		opt2.OutRnxObsBody(os.Stdout, obs.Data[i:], j-i, 0)
	}
}

/* outrnxnavh() */
func Test_renixutest5(t *testing.T) {
	var file1 string = "../data/rinex/07590920.05n"
	var ion []float64 = []float64{1e9, 2e-4, 3e8, 4e3, -4e-3, -5e99, -6e-33, -9e-123}
	var utc []float64 = []float64{1e9, 2e4, 3e2, -9999}
	var nav gnssgo.Nav
	var i int
	for i = 0; i < 8; i++ {
		nav.Ion_gps[i] = ion[i]
	}
	for i = 0; i < 4; i++ {
		nav.Utc_gps[i] = utc[i]
	}
	// nav.Leaps=14;

	gnssgo.ReadRnx(file1, 1, "", nil, &nav, nil)

	opt1.OutRnxNavHeader(os.Stdout, &nav)
	opt2.OutRnxNavHeader(os.Stdout, &nav)
}

/* outrnxnavb() */
func Test_renixutest6(t *testing.T) {
	var file string = "../data/rinex/07590920.05n"
	var nav gnssgo.Nav
	var i int
	gnssgo.ReadRnx(file, 1, "", nil, &nav, nil)
	for i = 0; i < nav.N(); i++ {
		opt2.OutRnxNavBody(os.Stdout, &nav.Ephs[i])
	}
}

func Test_renixutest7(t *testing.T) {
	var file1 string = "../data/rinex/025/brdm0250.19p"
	var file2 string = "../data/rinex/025/brdm0250.nav"
	// var ion []float64 = []float64{1e9, 2e-4, 3e8, 4e3, -4e-3, -5e99, -6e-33, -9e-123}
	// var utc []float64 = []float64{1e9, 2e4, 3e2, -9999}
	var nav gnssgo.Nav
	var i int
	// for i = 0; i < 8; i++ {
	// 	nav.Ion_gps[i] = ion[i]
	// }
	// for i = 0; i < 4; i++ {
	// 	nav.Utc_gps[i] = utc[i]
	// }
	// nav.Leaps=14;

	gnssgo.ReadRnx(file1, 1, "", nil, &nav, nil)

	opt2.OutRnxNavHeader(os.Stdout, &nav)
	fp, _ := os.OpenFile(file2, os.O_CREATE|os.O_WRONLY|os.O_TRUNC, os.ModeAppend|os.ModePerm)
	defer fp.Close()
	for i = 0; i < nav.N(); i++ {
		opt2.OutRnxNavBody(fp, &nav.Ephs[i])
	}

	for i = 0; i < nav.Ng(); i++ {
		opt2.OutRnxGnavBody(fp, &nav.Geph[i])
	}
	for i = 0; i < nav.Ns(); i++ {
		opt2.OutRnxHnavBody(fp, &nav.Seph[i])
	}
}
