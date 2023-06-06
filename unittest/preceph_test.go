/*------------------------------------------------------------------------------
* rtklib unit test driver : precise ephemeris function
*-----------------------------------------------------------------------------*/
package gnss_test

import (
	"fmt"
	"gnssgo"
	"os"
	"testing"

	"github.com/stretchr/testify/assert"
)

func dumpeph(peph []gnssgo.PEph, n int) {
	var s string
	var i, j int
	for i = 0; i < n; i++ {
		gnssgo.Time2Str(peph[i].Time, &s, 3)
		fmt.Printf("time=%s\n", s)
		for j = 0; j < gnssgo.MAXSAT; j++ {
			fmt.Printf("%03d: %14.3f %14.3f %14.3f : %5.3f %5.3f %5.3f\n",
				j+1, peph[i].Pos[j][0], peph[i].Pos[j][1], peph[i].Pos[j][2],
				peph[i].Std[j][0], peph[i].Std[j][1], peph[i].Std[j][2])
		}
	}
}
func dumpclk(pclk []gnssgo.PClk, n int) {
	var s string
	var i, j int
	for i = 0; i < n; i++ {
		gnssgo.Time2Str(pclk[i].Time, &s, 3)
		fmt.Printf("time=%s\n", s)
		for j = 0; j < gnssgo.MAXSAT; j++ {
			fmt.Printf("%03d: %14.3f : %5.3f\n",
				j+1, pclk[i].Clk[j][0]*1e9, pclk[i].Std[j][0]*1e9)
		}
	}
}

/* readsp3() */
func Test_precephutest1(t *testing.T) {
	var file1 string = "../data/sp3/igs15904.sp4"
	var file2 string = "../data/sp3/igs15904.sp3"
	var file3 string = "../data/sp3/igs1590*.sp3"
	var nav gnssgo.Nav
	assert := assert.New(t)

	fmt.Printf("file=%s\n", file1)
	nav.ReadSp3(file1, 0)
	assert.True(nav.Ne() == 0)

	fmt.Printf("file=%s\n", file2)
	nav.ReadSp3(file2, 0)
	assert.True(nav.Ne() == 96)
	dumpeph(nav.Peph, nav.Ne())

	fmt.Printf("file=%s\n", file3)
	nav.ReadSp3(file3, 0)
	assert.True(nav.Ne() == 192)
	dumpeph(nav.Peph, nav.Ne())
}

/* readsap() */
func Test_precephutest2(t *testing.T) {
	var ep1 []float64 = []float64{2008, 3, 1, 0, 0, 0}
	var ep2 []float64 = []float64{2006, 11, 4, 23, 59, 59}
	var file1 string = "../data/sp3/igs06.atx"
	var file2 string = "../data/igs05.atx"
	var pcvs gnssgo.Pcvs
	var pcv *gnssgo.Pcv
	var time gnssgo.Gtime
	var i, stat int
	assert := assert.New(t)

	fmt.Printf("file=%s\n", file1)
	stat = gnssgo.ReadPcv(file1, &pcvs)
	assert.True(stat == 0)
	stat = gnssgo.ReadPcv(file2, &pcvs)
	assert.True(stat != 0)

	time = gnssgo.Epoch2Time(ep1)
	for i = 0; i < gnssgo.MAXSAT; i++ {
		if pcv = gnssgo.SearchPcv(i+1, "", time, &pcvs); pcv == nil {
			continue
		}
		fmt.Printf("PRN%02d : %7.4f %7.4f %7.4f\n", i+1, pcv.Offset[0][0], pcv.Offset[0][1], pcv.Offset[0][2])
	}
	time = gnssgo.Epoch2Time(ep2)
	for i = 0; i < gnssgo.MAXSAT; i++ {
		if pcv = gnssgo.SearchPcv(i+1, "", time, &pcvs); pcv == nil {
			continue
		}
		fmt.Printf("PRN%02d : %7.4f %7.4f %7.4f\n", i+1, pcv.Offset[0][0], pcv.Offset[0][1], pcv.Offset[0][2])
	}
}

/* readrnxc() */
func Test_precephutest3(t *testing.T) {
	var file1 string = "../data/sp3/igs15904.cls"
	var file2 string = "../data/sp3/igs15904.clk"
	var file3 string = "../data/sp3/igs1590*.clk"
	var nav gnssgo.Nav
	assert := assert.New(t)

	fmt.Printf("file=%s\n", file1)
	nav.ReadRnxC(file1)
	assert.True(nav.Nc() == 0)

	fmt.Printf("file=%s\n", file2)
	nav.ReadRnxC(file2)
	assert.True(nav.Nc() > 0)
	dumpclk(nav.Pclk, nav.Nc())
	nav.Pclk = nil

	fmt.Printf("file=%s\n", file3)
	nav.ReadRnxC(file3)
	assert.True(nav.Nc() > 0)
	dumpclk(nav.Pclk, nav.Nc())
	nav.Pclk = nil
}

/* peph2pos() */
func Test_precephutest4(t *testing.T) {
	var fp *os.File
	var file1 string = "../data/sp3/igs1590*.sp3" /* 2010/7/1 */
	var file2 string = "../data/sp3/igs1590*.clk" /* 2010/7/1 */
	var nav gnssgo.Nav
	var i, j, stat, sat int
	var ep []float64 = []float64{2010, 7, 1, 0, 0, 0}
	var rs [6]float64
	var dts [2]float64
	var fvar float64
	var tm, time gnssgo.Gtime
	assert := assert.New(t)

	time = gnssgo.Epoch2Time(ep)

	nav.ReadSp3(file1, 0)
	assert.True(nav.Ne() > 0)
	nav.ReadRnxC(file2)
	assert.True(nav.Nc() > 0)
	stat = nav.PEph2Pos(time, 0, 0, rs[:], dts[:], &fvar)
	assert.True(stat == 0)
	stat = nav.PEph2Pos(time, 160, 0, rs[:], dts[:], &fvar)
	assert.True(stat == 0)

	fp, _ = os.OpenFile("../out/testpeph1.out", os.O_CREATE|os.O_WRONLY, os.ModePerm|os.ModeAppend)

	sat = 4

	for i = 0; i < 86400*2; i += 30 {
		tm = gnssgo.TimeAdd(time, float64(i))
		for j = 0; j < 6; j++ {
			rs[j] = 0.0
		}
		for j = 0; j < 2; j++ {
			dts[j] = 0.0
		}
		nav.PEph2Pos(tm, sat, 0, rs[:], dts[:], &fvar)
		fp.WriteString(fmt.Sprintf("%02d %6d %14.3f %14.3f %14.3f %14.3f %10.3f %10.3f %10.3f %10.3f\n",
			sat, i, rs[0], rs[1], rs[2], dts[0]*1e9, rs[3], rs[4], rs[5], dts[1]*1e9))
	}
	fp.Close()
}

/* satpos() */
func Test_precephutest5(t *testing.T) {
	var fp *os.File
	var file1 string = "../data/sp3/igs1590*.sp3" /* 2010/7/1 */
	var file2 string = "../data/sp3/igs1590*.clk" /* 2010/7/1 */
	var file3 string = "../data/igs05.atx"
	var file4 string = "../data/rinex/brdc*.10n"
	var pcvs gnssgo.Pcvs
	var pcv *gnssgo.Pcv
	var nav gnssgo.Nav
	var i, stat, sat, svh int
	var ep []float64 = []float64{2010, 7, 1, 0, 0, 0}
	var rs1, rs2 [6]float64
	var dts1, dts2 [2]float64
	var fvar float64
	var tm, time gnssgo.Gtime
	assert := assert.New(t)

	time = gnssgo.Epoch2Time(ep)

	nav.ReadSp3(file1, 0)
	assert.True(nav.Ne() > 0)
	nav.ReadRnxC(file2)
	assert.True(nav.Nc() > 0)
	stat = gnssgo.ReadPcv(file3, &pcvs)
	assert.True(stat != 0)
	gnssgo.ReadRnx(file4, 1, "", nil, &nav, nil)
	assert.True(nav.N() > 0)
	for i = 0; i < gnssgo.MAXSAT; i++ {
		if pcv = gnssgo.SearchPcv(i+1, "", time, &pcvs); pcv == nil {
			continue
		}
		nav.Pcvs[i] = *pcv
	}
	fp, _ = os.OpenFile("../out/testpeph2.out", os.O_CREATE|os.O_WRONLY, os.ModePerm|os.ModeAppend)

	sat = 3

	for i = 0; i < 86400*2; i += 30 {
		tm = gnssgo.TimeAdd(time, float64(i))
		nav.SatPos(tm, tm, sat, gnssgo.EPHOPT_BRDC, rs1[:], dts1[:], &fvar, &svh)
		nav.SatPos(tm, tm, sat, gnssgo.EPHOPT_PREC, rs2[:], dts2[:], &fvar, &svh)
		fp.WriteString(fmt.Sprintf("%02d %6d %14.3f %14.3f %14.3f %14.3f %14.3f %14.3f %14.3f %14.3f\n",
			sat, i,
			rs1[0], rs1[1], rs1[2], dts1[0]*1e9, rs2[0], rs2[1], rs2[2], dts2[0]*1e9))
	}
	fp.Close()
}
