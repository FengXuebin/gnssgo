/*------------------------------------------------------------------------------
* rtklib unit test driver : glonass ephemeris function
*-----------------------------------------------------------------------------*/
package gnss_test

import (
	"fmt"
	"gnssgo"
	"os"
	"testing"

	"github.com/stretchr/testify/assert"
)

func dumpgeph(geph []gnssgo.GEph, n int) {
	var s1, s2 string
	var i int
	for i = 0; i < n; i++ {
		gnssgo.Time2Str(geph[i].Toe, &s1, 0)
		gnssgo.Time2Str(geph[i].Tof, &s2, 0)
		fmt.Printf("(%3d) sat=%2d frq=%2d svh=%d age=%d toe=%s %s pos=%13.3f %13.3f %13.3f vel=%9.3f %9.3f %9.3f acc=%10.3E %10.3E %10.3E taun=%12.5E gamn=%12.5E\n",
			i+1, geph[i].Sat, geph[i].Frq, geph[i].Svh, geph[i].Age, s1, s2,
			geph[i].Pos[0], geph[i].Pos[1], geph[i].Pos[2],
			geph[i].Vel[0], geph[i].Vel[1], geph[i].Vel[2],
			geph[i].Acc[0], geph[i].Acc[1], geph[i].Acc[2],
			geph[i].Taun, geph[i].Gamn)
	}
}

/* readrnx() */
func Test_gloephutest1(t *testing.T) {
	var file1 string = "../data/rinex/brdd0910.09g"
	var file2 string = "../data/rinex/brdc0910.09g"
	var nav gnssgo.Nav
	assert := assert.New(t)

	gnssgo.ReadRnx(file1, 1, "", nil, &nav, nil)
	assert.True(nav.Ng() == 0)
	gnssgo.ReadRnx(file2, 1, "", nil, &nav, nil)
	assert.True(nav.Ng() > 0)
	dumpgeph(nav.Geph, nav.Ng())
}

/* readsp3() */
func Test_gloephutest2(t *testing.T) {
	var file1 string = "../data/sp3/igl15253.sp4"
	var file2 string = "../data/sp3/igl15253.sp3"
	var nav gnssgo.Nav
	var tow float64
	var pos []float64
	var i, week, sat int
	assert := assert.New(t)

	sat = gnssgo.SatNo(gnssgo.SYS_GLO, 13)

	nav.ReadSp3(file1, 0)
	assert.True(nav.Ne() <= 0)
	nav.ReadSp3(file2, 0)
	assert.True(nav.Ne() > 0)

	for i = 0; i < nav.Ne(); i++ {
		tow = gnssgo.Time2GpsT(nav.Peph[i].Time, &week)
		pos = nav.Peph[i].Pos[sat-1][:]
		fmt.Printf("%4d %6.0f %2d %13.3f %13.3f %13.3f %10.3f\n",
			week, tow, sat, pos[0], pos[1], pos[2], pos[3]*1e9)
		assert.True(gnssgo.Norm(pos, 4) > 0.0)
	}
	fmt.Printf("\n")
}

/* broadcast ephemeris */
func Test_gloephutest3(t *testing.T) {
	var time gnssgo.Gtime
	var file string = "../data/rinex/brdc0910.09g"
	var nav gnssgo.Nav
	var ep []float64 = []float64{2009, 4, 1, 0, 0, 0}
	var tspan float64 = 86400.0
	var tint float64 = 30.0
	var tow float64
	var rs [6]float64
	var dts [2]float64
	var fvar float64
	var i, sat, week, svh int
	assert := assert.New(t)

	sat = gnssgo.SatNo(gnssgo.SYS_GLO, 7)

	gnssgo.ReadRnx(file, 1, "", nil, &nav, nil)

	for i = 0; i < int(tspan/tint); i++ {
		time = gnssgo.TimeAdd(gnssgo.Epoch2Time(ep), tint*float64(i))
		nav.SatPos(time, time, sat, gnssgo.EPHOPT_BRDC, rs[:], dts[:], &fvar, &svh)
		tow = gnssgo.Time2GpsT(time, &week)
		fmt.Printf("%4d %6.0f %2d %13.3f %13.3f %13.3f %10.3f\n",
			week, tow, sat, rs[0], rs[1], rs[2], dts[0]*1e9)
		assert.True(gnssgo.Norm(rs[:], 3) > 0.0)
		assert.True(gnssgo.Norm(rs[3:], 3) > 0.0)
		assert.True(dts[0] != 0.0)
		assert.True(dts[1] != 0.0)
	}
	fmt.Printf("\n")
}

/* precise ephemeris */
func Test_gloephutest4(t *testing.T) {
	var time gnssgo.Gtime
	var file string = "../data/sp3/igl15253.sp3"
	var nav gnssgo.Nav
	var ep []float64 = []float64{2009, 4, 1, 0, 0, 0}
	var tspan float64 = 86400.0
	var tint float64 = 30.0
	var tow float64
	var rs [6]float64
	var dts [2]float64
	var fvar float64
	var i, sat, week, svh int
	assert := assert.New(t)

	sat = gnssgo.SatNo(gnssgo.SYS_GLO, 7)

	nav.ReadSp3(file, 0)

	for i = 0; i < int(tspan/tint); i++ {
		time = gnssgo.TimeAdd(gnssgo.Epoch2Time(ep), tint*float64(i))
		nav.SatPos(time, time, sat, gnssgo.EPHOPT_PREC, rs[:], dts[:], &fvar, &svh)
		tow = gnssgo.Time2GpsT(time, &week)
		fmt.Printf("%4d %6.0f %2d %13.3f %13.3f %13.3f %10.3f\n",
			week, tow, sat, rs[0], rs[1], rs[2], dts[0]*1e9)
		assert.True(gnssgo.Norm(rs[:], 3) > 0.0)
		assert.True(gnssgo.Norm(rs[3:], 3) > 0.0)
		assert.True(dts[0] != 0.0)
	}
	fmt.Printf("\n")
}

/* readsap() */
func Test_gloephutest5(t *testing.T) {
	var file string = "../data/igs05.atx"
	var id string
	var ep []float64 = []float64{2009, 4, 1, 0, 0, 0}
	time := gnssgo.Epoch2Time(ep)
	var nav gnssgo.Nav
	var i, stat int
	assert := assert.New(t)

	stat = nav.ReadSAP(file, time)
	assert.True(stat != 0)
	for i = 0; i < gnssgo.MAXSAT; i++ {
		gnssgo.SatNo2Id(i+1, &id)
		fmt.Printf("%2d %-4s %8.3f %8.3f %8.3f\n", i+1, id,
			nav.Pcvs[i].Offset[0][0], nav.Pcvs[i].Offset[0][1], nav.Pcvs[i].Offset[0][2])
	}
	fmt.Printf("\n")
}

/* satpos() */
func Test_gloephutest6(t *testing.T) {
	var fp *os.File
	var file1 string = "../data/rinex/brdc0910.09g"
	var file2 string = "../data/sp3/igl15253.sp3"
	var file3 string = "../data/igs05.atx"
	/*
	   char *file4="../data/esa15253.sp3";
	   char *file5="../data/esa15253.clk";
	*/
	var outfile string = "../out/testgloeph.out"
	var ep []float64 = []float64{2009, 4, 1, 0, 0, 0}
	var tspan float64 = 86400.0
	var tint float64 = 30.0
	var tow, ddts float64
	var rs1, rs2 [6]float64
	var dts1, dts2 [2]float64
	var dr [3]float64
	var var1, var2 float64
	time := gnssgo.Epoch2Time(ep)
	var nav gnssgo.Nav
	var i, j, sat, week, svh1, svh2 int
	assert := assert.New(t)

	gnssgo.ReadRnx(file1, 1, "", nil, &nav, nil)
	nav.ReadSp3(file2, 0)
	/*
	   readsp3(file4,&nav,0);
	   readrnxc(file5,&nav);
	*/
	nav.ReadSAP(file3, time)

	/*
	   sat=satno(SYS_GLO,21);
	*/
	sat = gnssgo.SatNo(gnssgo.SYS_GLO, 22)

	fp, _ = os.OpenFile(outfile, os.O_CREATE|os.O_WRONLY, os.ModePerm|os.ModeAppend)

	for i = 0; i < int(tspan/tint); i++ {
		time = gnssgo.TimeAdd(gnssgo.Epoch2Time(ep), tint*float64(i))
		tow = gnssgo.Time2GpsT(time, &week)
		nav.SatPos(time, time, sat, gnssgo.EPHOPT_BRDC, rs1[:], dts1[:], &var1, &svh1)
		nav.SatPos(time, time, sat, gnssgo.EPHOPT_PREC, rs2[:], dts2[:], &var2, &svh2)

		if gnssgo.Norm(rs1[:], 3) <= 0.0 || gnssgo.Norm(rs2[:], 3) <= 0.0 {
			continue
		}

		for j = 0; j < 3; j++ {
			dr[j] = rs1[j] - rs2[j]
		}
		ddts = dts1[0] - dts2[0]
		fp.WriteString(fmt.Sprintf("%4d %6.0f %2d %8.3f %8.3f %8.3f %10.3f\n", week, tow, sat,
			dr[0], dr[1], dr[2], ddts*1e9))

		assert.True(gnssgo.Norm(dr[:], 3) < 10.0)
	}
	fp.Close()
	fmt.Printf("output to: %s\n", outfile)
}
