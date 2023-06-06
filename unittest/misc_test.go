/*------------------------------------------------------------------------------
* gnssgo unit test driver : misc functions
*-----------------------------------------------------------------------------*/
package gnss_test

import (
	"fmt"
	"gnssgo"
	"strings"
	"testing"

	"github.com/stretchr/testify/assert"
)

/* expath() */
func utest11(path string) {
	var paths [32]string
	var i, n int
	n = gnssgo.ExPath(path, paths[:], 32)
	fmt.Printf("\npath =%s\n", path)
	fmt.Printf("paths=\n")
	for i = 0; i < n; i++ {
		fmt.Printf("%s\n", paths[i])
	}
}
func Test_miscutest1(t *testing.T) {
	utest11("")
	utest11("*")
	utest11("*.*")
	utest11("*.go")
	utest11("*.g*")
	utest11("*_test.*")
	utest11("misc_test.go")
	utest11("m*_test.go")
	utest11("misc_t*t.go")
	utest11("m*sc*_t*t.go")
	utest11("m*sc*t*")
	utest11("*.")
	utest11(".*")
	utest11("misc_test")
	utest11("_test.go")
	utest11("c:\\*")
	utest11("c:\\windows*")
	utest11("c:\\windows\\*")
}

/* reppath() */
func Test_miscutest2(t *testing.T) {
	var t0, t1, t2 gnssgo.Gtime
	var ep1 []float64 = []float64{1990, 1, 1, 0, 0, 0.00000}
	var ep2 []float64 = []float64{2010, 12, 31, 23, 59, 59.99999}
	var path0 string = "01234567890123456789"
	var path1 string = "abcde_%Y/%m/%d_%h:%M:%S_%Y%m%d%h%M%S"
	var path2 string = "abcde_%y%n_%W%D%H_%ha%hb%hc"
	var path3 string = "rover %r %r base %b %b"
	var path4 string = "%a %b %c"
	var rpath string
	var rov string = "RRRRRRRR"
	var base string = "BBBBBBBB"
	var stat int

	assert := assert.New(t)
	t1 = gnssgo.Epoch2Time(ep1)
	t2 = gnssgo.Epoch2Time(ep2)

	stat = gnssgo.RepPath(path0, &rpath, t1, "", "")
	assert.True(stat == 0)
	stat = strings.Compare(rpath, path0)
	assert.True(stat == 0)
	stat = gnssgo.RepPath(path0, &rpath, t0, rov, base)
	assert.True(stat == 0)
	stat = strings.Compare(rpath, path0)
	assert.True(stat == 0)
	stat = gnssgo.RepPath(path0, &rpath, t1, rov, base)
	assert.True(stat == 0)
	stat = strings.Compare(rpath, path0)
	assert.True(stat == 0)
	stat = gnssgo.RepPath(path1, &rpath, t1, "", "")
	assert.True(stat == 1)
	stat = strings.Compare(rpath, "abcde_1990/01/01_00:00:00_19900101000000")
	assert.True(stat == 0)
	stat = gnssgo.RepPath(path2, &rpath, t2, rov, base)
	assert.True(stat == 1)
	stat = strings.Compare(rpath, "abcde_10365_16165x_211812")
	assert.True(stat == 0)
	stat = gnssgo.RepPath(path3, &rpath, t0, rov, base)
	assert.True(stat == 1)
	stat = strings.Compare(rpath, "rover RRRRRRRR RRRRRRRR base BBBBBBBB BBBBBBBB")
	assert.True(stat == 0)
	stat = gnssgo.RepPath(path4, &rpath, t1, rov, "")
	assert.True(stat == 0)
	stat = strings.Compare(rpath, "%a %b %c")
	assert.True(stat == 0)
}

/* gnssgo.RepPaths() */
func Test_miscutest3(t *testing.T) {
	var t0, t1, t2, t3, t4 gnssgo.Gtime
	var ep1 []float64 = []float64{2010, 7, 31, 21, 36, 50.00000}
	var ep2 []float64 = []float64{2010, 8, 1, 4, 0, 0.00000}
	var ep3 []float64 = []float64{2010, 8, 31, 0, 0, 0.00000}
	var ep4 []float64 = []float64{2012, 1, 31, 0, 0, 0.00000}
	var path0 string = "01234567890123456789"
	var path1 string = "abcde_%Y/%m/%d_%h:%M:%S_%Y%m%d%h%M%S"
	var path2 string = "%r_%b_%r_%b_%y%n_%W%D%H_%ha%hb%hc"
	var path4 string = "YEAR=%Y GPSWEEK=%W"
	var paths [100]string
	var i, n, stat int
	assert := assert.New(t)

	t1 = gnssgo.Epoch2Time(ep1)
	t2 = gnssgo.Epoch2Time(ep2)
	t3 = gnssgo.Epoch2Time(ep3)
	t4 = gnssgo.Epoch2Time(ep4)

	n = gnssgo.RepPaths(path1, paths[:], 10, t0, t1, "ROV", "BASE")
	assert.True(n == 0)
	n = gnssgo.RepPaths(path1, paths[:], 10, t1, t0, "ROV", "BASE")
	assert.True(n == 0)
	n = gnssgo.RepPaths(path1, paths[:], 0, t1, t2, "ROV", "BASE")
	assert.True(n == 0)
	n = gnssgo.RepPaths(path1, paths[:], 10, t2, t1, "ROV", "BASE")
	assert.True(n == 0)
	n = gnssgo.RepPaths(path0, paths[:], 10, t1, t2, "ROV", "BASE")
	assert.True(n == 1)
	stat = strings.Compare(paths[0], path0)
	assert.True(stat == 0)
	n = gnssgo.RepPaths(path1, paths[:], 100, t1, t2, "ROV", "BASE")
	for i = 0; i < n; i++ {
		fmt.Printf("paths[%2d]=%s\n", i, paths[i])
	}
	fmt.Printf("\n")
	assert.True(n == 27)
	stat = strings.Compare(paths[0], "abcde_2010/07/31_21:30:00_20100731213000")
	assert.True(stat == 0)
	stat = strings.Compare(paths[26], "abcde_2010/08/01_04:00:00_20100801040000")
	assert.True(stat == 0)
	n = gnssgo.RepPaths(path2, paths[:], 100, t1, t3, "ROV", "BASE")
	for i = 0; i < n; i++ {
		fmt.Printf("paths[%2d]=%s\n", i, paths[i])
	}
	fmt.Printf("\n")
	assert.True(n == 100)
	stat = strings.Compare(paths[0], "ROV_BASE_ROV_BASE_10212_15946v_211812")
	assert.True(stat == 0)
	stat = strings.Compare(paths[99], "ROV_BASE_ROV_BASE_10217_15954a_000000")
	assert.True(stat == 0)
	n = gnssgo.RepPaths(path4, paths[:], 100, t1, t4, "ROV", "BASE")
	for i = 0; i < n; i++ {
		fmt.Printf("paths[%2d]=%s\n", i, paths[i])
	}
	fmt.Printf("\n")
	assert.True(n == 81)
	stat = strings.Compare(paths[0], "YEAR=2010 GPSWEEK=1594")
	assert.True(stat == 0)
	stat = strings.Compare(paths[80], "YEAR=2012 GPSWEEK=1673")
	assert.True(stat == 0)
}

/* gnssgo.GetBitU(),gnssgo.GetBits(),gnssgo.SetBitU(),gnssgo.SetBits() */
func Test_miscutest4(t *testing.T) {
	var buff [1024]uint8
	var vu uint32
	var vs uint32
	var vui int32
	var vsi int32
	assert := assert.New(t)

	gnssgo.SetBitU(buff[:], 0, 8, 1)
	vu = gnssgo.GetBitU(buff[:], 0, 8)
	assert.True(vu == 1)
	gnssgo.SetBitU(buff[:], 4, 8, 255)
	vu = gnssgo.GetBitU(buff[:], 4, 8)
	assert.True(vu == 255)
	gnssgo.SetBitU(buff[:], 13, 8, 1)
	vu = gnssgo.GetBitU(buff[:], 13, 8)
	assert.True(vu == 1)
	gnssgo.SetBitU(buff[:], 29, 8, 255)
	vu = gnssgo.GetBitU(buff[:], 29, 8)
	assert.True(vu == 255)
	gnssgo.SetBitU(buff[:], 99, 10, 1023)
	vu = gnssgo.GetBitU(buff[:], 99, 10)
	assert.True(vu == 1023)
	gnssgo.SetBitU(buff[:], 666, 31, 123456)
	vu = gnssgo.GetBitU(buff[:], 666, 31)
	assert.True(vu == 123456)
	gnssgo.SetBitU(buff[:], 777, 32, 789012)
	vu = gnssgo.GetBitU(buff[:], 777, 32)
	assert.True(vu == 789012)

	gnssgo.SetBits(buff[:], 100, 8, 1)
	vs = gnssgo.GetBitU(buff[:], 100, 8)
	assert.True(vs == 1)
	gnssgo.SetBits(buff[:], 104, 8, 127)
	vs = gnssgo.GetBitU(buff[:], 104, 8)
	assert.True(vs == 127)
	gnssgo.SetBits(buff[:], 113, 8, 1)
	vs = gnssgo.GetBitU(buff[:], 113, 8)
	assert.True(vs == 1)
	gnssgo.SetBits(buff[:], 129, 8, 127)
	vs = gnssgo.GetBitU(buff[:], 129, 8)
	assert.True(vs == 127)
	gnssgo.SetBits(buff[:], 199, 10, 511)
	vs = gnssgo.GetBitU(buff[:], 199, 10)
	assert.True(vs == 511)
	gnssgo.SetBits(buff[:], 766, 31, 123456)
	vs = gnssgo.GetBitU(buff[:], 766, 31)
	assert.True(vs == 123456)
	gnssgo.SetBits(buff[:], 877, 32, 789012)
	vs = gnssgo.GetBitU(buff[:], 877, 32)
	assert.True(vs == 789012)

	gnssgo.SetBits(buff[:], 200, 8, -1)
	vsi = gnssgo.GetBits(buff[:], 200, 8)
	assert.True(vsi == -1)
	gnssgo.SetBits(buff[:], 204, 8, -127)
	vsi = gnssgo.GetBits(buff[:], 204, 8)
	assert.True(vsi == -127)
	gnssgo.SetBits(buff[:], 213, 8, -3)
	vui = gnssgo.GetBits(buff[:], 213, 8)
	assert.True(vui == -3)
	gnssgo.SetBits(buff[:], 229, 8, -126)
	vui = gnssgo.GetBits(buff[:], 229, 8)
	assert.True(vui == -126)
	gnssgo.SetBits(buff[:], 299, 24, -99999)
	vui = gnssgo.GetBits(buff[:], 299, 24)
	assert.True(vui == -99999)
	gnssgo.SetBits(buff[:], 866, 31, -12345)
	vsi = gnssgo.GetBits(buff[:], 866, 31)
	assert.True(vsi == -12345)
	gnssgo.SetBits(buff[:], 977, 32, -67890)
	vsi = gnssgo.GetBits(buff[:], 977, 32)
	assert.True(vsi == -67890)

}
