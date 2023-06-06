/*------------------------------------------------------------------------------
* rtklib unit test driver : norad two line element function
*-----------------------------------------------------------------------------*/
package gnss_test

import (
	"fmt"
	"gnssgo"
	"os"
	"testing"

	"github.com/stretchr/testify/assert"
)

/* dump tle ------------------------------------------------------------------*/
func dumptle(fp *os.File, tle *gnssgo.Tle) {
	var i int

	for i = 0; i < tle.N; i++ {
		fp.WriteString(fmt.Sprintf("(%2d) name = %s\n", i+1, tle.Data[i].Name))
		fp.WriteString(fmt.Sprintf("(%2d) satno= %s\n", i+1, tle.Data[i].SatNo))
		fp.WriteString(fmt.Sprintf("(%2d) class= %c\n", i+1, tle.Data[i].SatClass))
		fp.WriteString(fmt.Sprintf("(%2d) desig= %s\n", i+1, tle.Data[i].Desig))
		fp.WriteString(fmt.Sprintf("(%2d) epoch= %s\n", i+1, gnssgo.TimeStr(tle.Data[i].Epoch, 0)))
		fp.WriteString(fmt.Sprintf("(%2d) etype= %d\n", i+1, tle.Data[i].EType))
		fp.WriteString(fmt.Sprintf("(%2d) eleno= %d\n", i+1, tle.Data[i].EleNo))
		fp.WriteString(fmt.Sprintf("(%2d) ndot = %19.12e\n", i+1, tle.Data[i].Ndot))
		fp.WriteString(fmt.Sprintf("(%2d) nddot= %19.12e\n", i+1, tle.Data[i].NDdot))
		fp.WriteString(fmt.Sprintf("(%2d) bstar= %19.12e\n", i+1, tle.Data[i].BStar))
		fp.WriteString(fmt.Sprintf("(%2d) inc  = %19.12e\n", i+1, tle.Data[i].Inc))
		fp.WriteString(fmt.Sprintf("(%2d) OMG  = %19.12e\n", i+1, tle.Data[i].OMG))
		fp.WriteString(fmt.Sprintf("(%2d) ecc  = %19.12e\n", i+1, tle.Data[i].Ecc))
		fp.WriteString(fmt.Sprintf("(%2d) omg  = %19.12e\n", i+1, tle.Data[i].Omg))
		fp.WriteString(fmt.Sprintf("(%2d) M    = %19.12e\n", i+1, tle.Data[i].M))
		fp.WriteString(fmt.Sprintf("(%2d) n    = %19.12e\n", i+1, tle.Data[i].N))
		fp.WriteString(fmt.Sprintf("(%2d) rev  = %d\n", i+1, tle.Data[i].RevNo))
	}
}

/* tle_read() ----------------------------------------------------------------*/
func Test_tleutest1(t *testing.T) {
	var file1 string = "../data/tle/tle_sgp4.err"
	var file2 string = "../data/tle/tle_sgp4.txt"
	var file3 string = "../data/tle/tle_nav.txt"
	var tle gnssgo.Tle
	var stat int
	assert := assert.New(t)

	stat = tle.TleRead(file1)
	assert.True(stat == 0)

	stat = tle.TleRead(file2)
	assert.True(stat != 0)
	assert.True(tle.N == 1)

	stat = tle.TleRead(file3)
	assert.Greater(stat, 0)
	assert.True(tle.N == 114)

	dumptle(os.Stdout, &tle)
}

/* tle_pos() -----------------------------------------------------------------*/
func Test_tleutest2(t *testing.T) {
	var file2 string = "../data/tle/tle_sgp4.txt"
	var ep0 [6]float64 = [6]float64{1980, 1, 1}
	var tle gnssgo.Tle
	var epoch gnssgo.Gtime
	var min float64
	var rs [6]float64
	var i, stat int
	assert := assert.New(t)

	epoch = gnssgo.Utc2GpsT(gnssgo.TimeAdd(gnssgo.Epoch2Time(ep0[:]), 274.98708465*86400.0))

	stat = tle.TleRead(file2)
	assert.Greater(stat, 0)

	stat = tle.TlePos(epoch, "TEST_ERR", "", "", nil, rs[:])
	assert.Equal(stat, 0)

	for i = 0; i < 5; i++ {
		min = 360.0 * float64(i)

		stat = tle.TlePos(gnssgo.TimeAdd(epoch, min*60.0), "TEST_SAT", "", "", nil, rs[:])
		assert.Greater(stat, 0)

		fmt.Printf("%4.0f: %14.8f %14.8f %14.8f  %11.8f %11.8f %11.8f\n", min,
			rs[0]/1e3, rs[1]/1e3, rs[2]/1e3, rs[3]/1e3, rs[4]/1e3, rs[5]/1e3)
	}
}

/* tle_pos() accuracy --------------------------------------------------------*/
func Test_tleutest3(t *testing.T) {
	var file1 string = "../data/tle/brdc3050.12*"
	var file2 string = "../data/tle/TLE_GNSS_20121101.txt"
	var file3 string = "../data/tle/igs17127.erp"
	var ep [6]float64 = [6]float64{2012, 10, 31, 0, 0, 0}
	var nav gnssgo.Nav
	var erp gnssgo.Erp
	var tle gnssgo.Tle
	var time gnssgo.Gtime
	var sat string
	var rs1, rs2, ds [6]float64
	var dts [2]float64
	var fvar float64
	var i, j, k, stat, svh int
	assert := assert.New(t)

	gnssgo.ReadRnx(file1, 0, "", nil, &nav, nil)
	assert.True(nav.N() > 0)

	stat = gnssgo.ReadErp(file3, &erp)
	assert.True(stat != 0)

	stat = tle.TleRead(file2)
	assert.True(stat != 0)

	for i = 0; i < gnssgo.MAXSAT; i++ {
		gnssgo.SatNo2Id(i+1, &sat)

		fmt.Printf("SAT=%s\n", sat)

		for j = 0; j < 96; j++ {
			time = gnssgo.TimeAdd(gnssgo.Epoch2Time(ep[:]), 900.0*float64(j))

			if nav.SatPos(time, time, i+1, int(gnssgo.EPHOPT_BRDC), rs1[:], dts[:], &fvar, &svh) == 0 {
				continue
			}

			if gnssgo.SatSys(i+1, nil) == gnssgo.SYS_QZS {
				svh &= 0xFE
			}

			if svh != 0 {
				continue
			}

			stat = tle.TlePos(time, sat, "", "", &erp, rs2[:])
			assert.True(stat != 0)

			for k = 0; k < 3; k++ {
				ds[k] = rs2[k] - rs1[k]
			}

			fmt.Printf("%6.0f %11.3f %11.3f %11.3f %11.3f\n", 900.0*float64(j),
				ds[0]/1e3, ds[1]/1e3, ds[2]/1e3, gnssgo.Norm(ds[:], 3)/1e3)

			if gnssgo.Norm(ds[:], 3)/1e3 > 300.0 {
				fmt.Printf("warning")
			}
			assert.True(gnssgo.Norm(ds[:], 3)/1e3 < 300.0)
		}
		fmt.Printf("\n")
	}
}
