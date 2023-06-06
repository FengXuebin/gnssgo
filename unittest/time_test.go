/*------------------------------------------------------------------------------
* gnssgo unit test driver : time and string functions
*-----------------------------------------------------------------------------*/
package gnss_test

import (
	"gnssgo"
	"math"
	"strings"
	"testing"
	"time"

	"github.com/stretchr/testify/assert"
)

/* str2num() */
func Test_str2num(t *testing.T) {
	var a float64
	var s1 string = "123456789012345678901234567890"
	var s2 string = "....3D45......................"
	var s3 string = "...  3456.789 ................"
	assert := assert.New(t)
	assert.Less(gnssgo.Str2Num(s1, 0, 0)-0.0, 1e-15)
	a = gnssgo.Str2Num(s1, 30, 10)
	assert.Less(math.Abs(a-0.0), 1e-15)
	a = gnssgo.Str2Num(s1, 10, 0)
	assert.Less(math.Abs(a-0.0), 1e-15)
	a = gnssgo.Str2Num(s1, -1, 10)
	assert.Less(math.Abs(a-0.0), 1e-15)
	a = gnssgo.Str2Num(s1, 0, 3)
	assert.Less(math.Abs(a-123.0), 1e-13)
	a = gnssgo.Str2Num(s1, 10, 6)
	assert.Less(math.Abs(a-123456.0), 1e-10)
	a = gnssgo.Str2Num(s1, 28, 10)
	assert.Less(math.Abs(a-90.0), 1e-14)
	a = gnssgo.Str2Num(s2, 4, 4)
	assert.Less(math.Abs(a-3e45), 1e+30)
	a = gnssgo.Str2Num(s3, 4, 8)
	assert.Less(math.Abs(a-3456.78), 1e-12)
}

/* str2time() */
func Test_utest2(t *testing.T) {
	var s1 string = "....2004 1 1 0 1 2.345........"
	var s2 string = "....  00 2 3 23 59 59.999....."
	var s3 string = "....  80 10 30 6 58 9........."
	var s4 string = "....  37 12 31 1 2 3 ........."
	var s int
	var time gnssgo.Gtime
	var ep [6]float64
	assert := assert.New(t)
	s = gnssgo.Str2Time(s1, 0, 0, &time)
	assert.Less(s, 0)
	s = gnssgo.Str2Time(s1, 30, 10, &time)
	assert.Less(s, 0)
	s = gnssgo.Str2Time(s1, 10, 0, &time)
	assert.Less(s, 0)
	s = gnssgo.Str2Time(s1, -1, 10, &time)
	assert.Less(s, 0)
	s = gnssgo.Str2Time(s1, 4, 17, &time)
	gnssgo.Time2Epoch(time, ep[:])
	assert.True(s == 0 && ep[0] == 2004 && ep[1] == 1 && ep[2] == 1 && ep[3] == 0 && ep[4] == 1 && math.Abs(ep[5]-2.34) < 1e-15)
	s = gnssgo.Str2Time(s2, 4, 21, &time)
	gnssgo.Time2Epoch(time, ep[:])
	assert.True(s == 0 && ep[0] == 2000 && ep[1] == 2 && ep[2] == 3 && ep[3] == 23 && ep[4] == 59 && math.Abs(ep[5]-59.999) < 1e-14)
	s = gnssgo.Str2Time(s3, 4, 20, &time)
	gnssgo.Time2Epoch(time, ep[:])
	assert.True(s == 0 && ep[0] == 1980 && ep[1] == 10 && ep[2] == 30 && ep[3] == 6 && ep[4] == 58 && ep[5] == 9)
	s = gnssgo.Str2Time(s4, 4, 20, &time)
	gnssgo.Time2Epoch(time, ep[:])
	assert.True(s == 0 && ep[0] == 2037 && ep[1] == 12 && ep[2] == 31 && ep[3] == 1 && ep[4] == 2 && ep[5] == 3)
}

/* epoch2time(),time2epoch() */
func Test_utest3(t *testing.T) {
	var ep0 []float64 = []float64{1980, 1, 6, 0, 0, 0.000000}
	var ep1 []float64 = []float64{2004, 2, 28, 2, 0, 59.999999}
	var ep2 []float64 = []float64{2004, 2, 29, 2, 0, 30.000000}
	var ep3 []float64 = []float64{2004, 12, 31, 23, 59, 59.999999}
	var ep4 []float64 = []float64{2037, 10, 1, 0, 0, 0.000000}
	//#ifdef TIME_64BIT
	var ep5 []float64 = []float64{2049, 2, 3, 4, 5, 6.000000}      /* 64bit time_t */
	var ep6 []float64 = []float64{2099, 12, 31, 23, 59, 59.999999} /* 64bit time_t */
	//#endif
	var year, month, day int
	var mday []int = []int{31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}
	var time gnssgo.Gtime
	var ep [6]float64
	assert := assert.New(t)
	time = gnssgo.Epoch2Time(ep0)
	gnssgo.Time2Epoch(time, ep[:])
	assert.True(ep[0] == 1980 && ep[1] == 1 && ep[2] == 6 && ep[3] == 0 && ep[4] == 0 && ep[5] == 0.0)
	time = gnssgo.Epoch2Time(ep1)
	gnssgo.Time2Epoch(time, ep[:])
	assert.True(ep[0] == 2004 && ep[1] == 2 && ep[2] == 28 && ep[3] == 2 && ep[4] == 0 && math.Abs(ep[5]-59.999999) < 1e-14)
	time = gnssgo.Epoch2Time(ep2)
	gnssgo.Time2Epoch(time, ep[:])
	assert.True(ep[0] == 2004 && ep[1] == 2 && ep[2] == 29 && ep[3] == 2 && ep[4] == 0 && ep[5] == 30.0)
	time = gnssgo.Epoch2Time(ep3)
	gnssgo.Time2Epoch(time, ep[:])
	assert.True(ep[0] == 2004 && ep[1] == 12 && ep[2] == 31 && ep[3] == 23 && ep[4] == 59 && math.Abs(ep[5]-59.999999) < 1e-14)
	time = gnssgo.Epoch2Time(ep4)
	gnssgo.Time2Epoch(time, ep[:])
	assert.True(ep[0] == 2037 && ep[1] == 10 && ep[2] == 1 && ep[3] == 0 && ep[4] == 0 && ep[5] == 0.0)
	//#ifdef TIME_64BIT
	time = gnssgo.Epoch2Time(ep5)
	gnssgo.Time2Epoch(time, ep[:])
	assert.True(ep[0] == 2049 && ep[1] == 2 && ep[2] == 3 && ep[3] == 4 && ep[4] == 5 && ep[5] == 6.0)
	time = gnssgo.Epoch2Time(ep6)
	gnssgo.Time2Epoch(time, ep[:])
	assert.True(ep[0] == 2099 && ep[1] == 12 && ep[2] == 31 && ep[3] == 23 && ep[4] == 59 && math.Abs(ep[5]-59.999999) < 1e-14)
	// #endif

	// #ifdef TIME_64BIT
	for year = 1970; year <= 2099; year++ {
		// #else
		//     for (year=1970;year<=2037;year++) {
		// #endif
		if year%4 == 0 {
			mday[1] = 29
		} else {
			mday[1] = 28
		}
		for month = 1; month <= 12; month++ {
			for day = 1; day <= mday[month-1]; day++ {
				if year == 1970 && month == 1 && day == 1 {
					continue
				}
				ep0[0] = float64(year)
				ep0[1] = float64(month)
				ep0[2] = float64(day)
				time = gnssgo.Epoch2Time(ep0)
				gnssgo.Time2Epoch(time, ep[:])
				/* fprintf(stderr,"ep=%.0f %2.0f %2.0f : %.0f %2.0f %2.0f\n",
				   ep0[0],ep0[1],ep0[2],ep[0],ep[1],ep[2]); */
				assert.True(ep[0] == ep0[0] && ep[1] == ep0[1] && ep[2] == ep0[2])
				assert.True(ep[3] == 0.0 && ep[4] == 0.0 && ep[5] == 0.0)
			}
		}
	}
}

/* gnssgo.GpsT2Time(), time2gpst() */
func Test_utest4(t *testing.T) {
	var tm gnssgo.Gtime
	var ep [6]float64
	var w, week int
	var time, tt float64
	assert := assert.New(t)
	tm = gnssgo.GpsT2Time(0, 0.0)
	gnssgo.Time2Epoch(tm, ep[:])
	assert.True(ep[0] == 1980 && ep[1] == 1 && ep[2] == 6 && ep[3] == 0 && ep[4] == 0 && ep[5] == 0.0)
	tm = gnssgo.GpsT2Time(1400, 86400.0)
	gnssgo.Time2Epoch(tm, ep[:])
	assert.True(ep[0] == 2006 && ep[1] == 11 && ep[2] == 6 && ep[3] == 0 && ep[4] == 0 && ep[5] == 0.0)
	tm = gnssgo.GpsT2Time(1400, 86400.0*7-1.0)
	gnssgo.Time2Epoch(tm, ep[:])
	assert.True(ep[0] == 2006 && ep[1] == 11 && ep[2] == 11 && ep[3] == 23 && ep[4] == 59 && ep[5] == 59.0)
	tm = gnssgo.GpsT2Time(1400, 86400.0*7)
	gnssgo.Time2Epoch(tm, ep[:])
	assert.True(ep[0] == 2006 && ep[1] == 11 && ep[2] == 12 && ep[3] == 0 && ep[4] == 0 && ep[5] == 0.0)
	tm = gnssgo.GpsT2Time(1401, 0.0)
	gnssgo.Time2Epoch(tm, ep[:])
	assert.True(ep[0] == 2006 && ep[1] == 11 && ep[2] == 12 && ep[3] == 0 && ep[4] == 0 && ep[5] == 0.0)
	tm = gnssgo.GpsT2Time(4000, 0.0)
	gnssgo.Time2Epoch(tm, ep[:])
	assert.True(ep[0] == 2056 && ep[1] == 9 && ep[2] == 3 && ep[3] == 0 && ep[4] == 0 && ep[5] == 0.0)
	tm = gnssgo.GpsT2Time(6260, 345600.0)
	gnssgo.Time2Epoch(tm, ep[:])
	assert.True(ep[0] == 2099 && ep[1] == 12 && ep[2] == 31 && ep[3] == 0 && ep[4] == 0 && ep[5] == 0.0)

	for w = 1000; w <= 6260; w++ {

		for time = 0.0; time < 86400.0*7; time += 3600.0 {
			tm = gnssgo.GpsT2Time(w, time)
			tt = gnssgo.Time2GpsT(tm, &week)
			assert.True(tt == time && week == w)
		}
	}
}

/* gnssgo.TimeAdd() */
func Test_utest5(t *testing.T) {
	var ep0 []float64 = []float64{2003, 12, 31, 23, 59, 59.000000}
	var ep1 []float64 = []float64{2004, 1, 1, 0, 0, 1.000000}
	var ep2 []float64 = []float64{2004, 2, 28, 0, 0, 0.000000}
	var ep3 []float64 = []float64{2004, 2, 29, 0, 0, 0.000000}
	var tm gnssgo.Gtime
	var ep [6]float64
	assert := assert.New(t)
	tm = gnssgo.TimeAdd(gnssgo.Epoch2Time(ep0), 3.0)
	gnssgo.Time2Epoch(tm, ep[:])
	assert.True(ep[0] == 2004 && ep[1] == 1 && ep[2] == 1 && ep[3] == 0 && ep[4] == 0 && ep[5] == 2.0)
	tm = gnssgo.TimeAdd(gnssgo.Epoch2Time(ep1), -3.0)
	gnssgo.Time2Epoch(tm, ep[:])
	assert.True(ep[0] == 2003 && ep[1] == 12 && ep[2] == 31 && ep[3] == 23 && ep[4] == 59 && ep[5] == 58.0)
	tm = gnssgo.TimeAdd(gnssgo.Epoch2Time(ep2), 86400.0)
	gnssgo.Time2Epoch(tm, ep[:])
	assert.True(ep[0] == 2004 && ep[1] == 2 && ep[2] == 29 && ep[3] == 0 && ep[4] == 0 && ep[5] == 0.0)
	tm = gnssgo.TimeAdd(gnssgo.Epoch2Time(ep2), 86400.0*2)
	gnssgo.Time2Epoch(tm, ep[:])
	assert.True(ep[0] == 2004 && ep[1] == 3 && ep[2] == 1 && ep[3] == 0 && ep[4] == 0 && ep[5] == 0.0)
	tm = gnssgo.TimeAdd(gnssgo.Epoch2Time(ep3), 86400.0*2)
	gnssgo.Time2Epoch(tm, ep[:])
	assert.True(ep[0] == 2004 && ep[1] == 3 && ep[2] == 2 && ep[3] == 0 && ep[4] == 0 && ep[5] == 0.0)
}

/* timediff() */
func Test_utest6(t *testing.T) {
	var ep0 []float64 = []float64{2003, 12, 31, 23, 59, 59.000000}
	var ep1 []float64 = []float64{2004, 1, 1, 0, 0, 1.000000}
	var ep2 []float64 = []float64{2004, 2, 28, 0, 0, 0.000000}
	var ep3 []float64 = []float64{2004, 2, 29, 0, 0, 0.000000}
	var ep4 []float64 = []float64{2004, 3, 1, 0, 0, 0.000000}
	var sec float64
	assert := assert.New(t)
	sec = gnssgo.TimeDiff(gnssgo.Epoch2Time(ep1), gnssgo.Epoch2Time(ep0))
	assert.True(sec == 2.0)
	sec = gnssgo.TimeDiff(gnssgo.Epoch2Time(ep0), gnssgo.Epoch2Time(ep1))
	assert.True(sec == -2.0)
	sec = gnssgo.TimeDiff(gnssgo.Epoch2Time(ep3), gnssgo.Epoch2Time(ep2))
	assert.True(sec == 86400.0)
	sec = gnssgo.TimeDiff(gnssgo.Epoch2Time(ep4), gnssgo.Epoch2Time(ep2))
	assert.True(sec == 86400.0*2)
	sec = gnssgo.TimeDiff(gnssgo.Epoch2Time(ep3), gnssgo.Epoch2Time(ep4))
	assert.True(sec == -86400.0)
}

/* gnssgo.GpsT2Utc() */
func Test_utest7(t *testing.T) {
	var ep0 []float64 = []float64{1980, 1, 6, 0, 0, 0.000000}
	var ep1 []float64 = []float64{1992, 7, 1, 0, 0, 6.999999}
	var ep2 []float64 = []float64{1992, 7, 1, 0, 0, 7.000000}
	var ep3 []float64 = []float64{1992, 7, 1, 0, 0, 8.000000}
	var ep4 []float64 = []float64{2004, 12, 31, 23, 59, 59.999999}
	var ep5 []float64 = []float64{2006, 1, 1, 0, 0, 0.000000}
	var ep6 []float64 = []float64{2038, 1, 1, 0, 0, 0.000000}
	var tm gnssgo.Gtime
	var ep [6]float64
	assert := assert.New(t)
	tm = gnssgo.GpsT2Utc(gnssgo.Epoch2Time(ep0))
	gnssgo.Time2Epoch(tm, ep[:])
	assert.True(ep[0] == 1980 && ep[1] == 1 && ep[2] == 6 && ep[3] == 0 && ep[4] == 0 && ep[5] == 0.0)
	tm = gnssgo.GpsT2Utc(gnssgo.Epoch2Time(ep1))
	gnssgo.Time2Epoch(tm, ep[:])
	assert.True(ep[0] == 1992 && ep[1] == 6 && ep[2] == 30 && ep[3] == 23 && ep[4] == 59 && math.Abs(ep[5]-59.999999) < 1e-14)
	tm = gnssgo.GpsT2Utc(gnssgo.Epoch2Time(ep2))
	gnssgo.Time2Epoch(tm, ep[:])
	assert.True(ep[0] == 1992 && ep[1] == 7 && ep[2] == 1 && ep[3] == 0 && ep[4] == 0 && ep[5] == 0.0)
	tm = gnssgo.GpsT2Utc(gnssgo.Epoch2Time(ep3))
	gnssgo.Time2Epoch(tm, ep[:])
	assert.True(ep[0] == 1992 && ep[1] == 7 && ep[2] == 1 && ep[3] == 0 && ep[4] == 0 && ep[5] == 0.0)
	tm = gnssgo.GpsT2Utc(gnssgo.Epoch2Time(ep4))
	gnssgo.Time2Epoch(tm, ep[:])
	assert.True(ep[0] == 2004 && ep[1] == 12 && ep[2] == 31 && ep[3] == 23 && ep[4] == 59 && math.Abs(ep[5]-46.999999) < 1e-14)
	tm = gnssgo.GpsT2Utc(gnssgo.Epoch2Time(ep5))
	gnssgo.Time2Epoch(tm, ep[:])
	assert.True(ep[0] == 2005 && ep[1] == 12 && ep[2] == 31 && ep[3] == 23 && ep[4] == 59 && ep[5] == 47.0)
	tm = gnssgo.GpsT2Utc(gnssgo.Epoch2Time(ep6))
	gnssgo.Time2Epoch(tm, ep[:])
	assert.True(ep[0] == 2037 && ep[1] == 12 && ep[2] == 31 && ep[3] == 23 && ep[4] == 59 && ep[5] == 42.0)
}

/* utc2gpst(), gpst2utc() */
func Test_utest8(t *testing.T) {
	var ep0 []float64 = []float64{1980, 1, 6, 0, 0, 0.000000}
	var ep1 []float64 = []float64{2010, 12, 31, 23, 59, 59.999999}
	var t0, t1, t2, t3 gnssgo.Gtime
	assert := assert.New(t)
	t0 = gnssgo.Epoch2Time(ep0)
	t1 = gnssgo.Epoch2Time(ep1)
	for t0.Time < t1.Time {
		t2 = gnssgo.Utc2GpsT(t0)
		t3 = gnssgo.GpsT2Utc(t2)
		assert.True(t0.Time == t3.Time && t0.Sec == t3.Sec)
		t0 = gnssgo.TimeAdd(t0, 86400.0)
	}
}

/* gnssgo.Time2Str() */
func Test_utest9(t *testing.T) {
	var ep0 []float64 = []float64{1970, 12, 31, 23, 59, 59.1234567890123456}
	var ep1 []float64 = []float64{2004, 1, 1, 0, 0, 0.0000000000000000}
	var ep2 []float64 = []float64{2006, 2, 28, 23, 59, 59.9999995000000000}
	var s string
	var ret int
	assert := assert.New(t)
	gnssgo.Time2Str(gnssgo.Epoch2Time(ep0), &s, 0)
	ret = strings.Compare(s, "1970/12/31 23:59:59")
	assert.True(ret == 0)
	gnssgo.Time2Str(gnssgo.Epoch2Time(ep0), &s, -1)
	ret = strings.Compare(s, "1970/12/31 23:59:59")
	assert.True(ret == 0)
	gnssgo.Time2Str(gnssgo.Epoch2Time(ep0), &s, 10)
	ret = strings.Compare(s, "1970/12/31 23:59:59.1234567890")
	assert.True(ret == 0)
	gnssgo.Time2Str(gnssgo.Epoch2Time(ep1), &s, 0)
	ret = strings.Compare(s, "2004/01/01 00:00:00")
	assert.True(ret == 0)
	gnssgo.Time2Str(gnssgo.Epoch2Time(ep1), &s, 16)
	ret = strings.Compare(s, "2004/01/01 00:00:00.000000000000")
	assert.True(ret == 0)
	gnssgo.Time2Str(gnssgo.Epoch2Time(ep2), &s, 0)
	ret = strings.Compare(s, "2006/03/01 00:00:00")
	assert.True(ret == 0)
	gnssgo.Time2Str(gnssgo.Epoch2Time(ep2), &s, 6)
	ret = strings.Compare(s, "2006/03/01 00:00:00.000000")
	assert.True(ret == 0)
	gnssgo.Time2Str(gnssgo.Epoch2Time(ep2), &s, 7)
	ret = strings.Compare(s, "2006/02/28 23:59:59.9999995")
	assert.True(ret == 0)
}

/* timeget() */
func Test_utest10(t *testing.T) {
	var s1, s2 string
	var time1, time2 gnssgo.Gtime
	assert := assert.New(t)
	time1 = gnssgo.TimeGet()
	time.Sleep(time.Duration(2) * time.Second)
	time2 = gnssgo.TimeGet()
	gnssgo.Time2Str(time1, &s1, 6)
	gnssgo.Time2Str(time2, &s2, 6)
	assert.True(gnssgo.TimeDiff(time1, time2) <= 0.0)
}

/* time2doy() */
func Test_utest11(t *testing.T) {
	var ep1 []float64 = []float64{2004, 1, 1, 0, 0, 0}
	var ep2 []float64 = []float64{2004, 12, 31, 0, 0, 0}
	var ep3 []float64 = []float64{2005, 12, 31, 12, 0, 0}
	var doy1, doy2, doy3 float64
	assert := assert.New(t)
	doy1 = gnssgo.Time2DayOfYeay(gnssgo.Epoch2Time(ep1))
	doy2 = gnssgo.Time2DayOfYeay(gnssgo.Epoch2Time(ep2))
	doy3 = gnssgo.Time2DayOfYeay(gnssgo.Epoch2Time(ep3))
	assert.True(math.Abs(doy1-1.0) < 1e-6)
	assert.True(math.Abs(doy2-366.0) < 1e-6)
	assert.True(math.Abs(doy3-365.5) < 1e-6)
}
