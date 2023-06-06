/*------------------------------------------------------------------------------
* options.c : options functions
*
*          Copyright (C) 2010-2020 by T.TAKASU, All rights reserved.
*
* version : $Revision:$ $Date:$
* history : 2010/07/20  1.1  moved from postpos.c
*                            added api:
*                                searchopt(),str2opt(),opt2str(),opt2buf(),
*                                loadopts(),saveopts(),resetsysopts(),
*                                getsysopts(),setsysopts()
*           2010/09/11  1.2  add options
*                                pos2-elmaskhold,pos1.snrmaskena
*                                pos1-snrmask1,2,3
*           2013/03/11  1.3  add pos1-posopt1,2,3,4,5,pos2-syncsol
*                                misc-rnxopt1,2,pos1-snrmask_r,_b,_L1,_L2,_L5
*           2014/10/21  1.4  add pos2-bdsarmode
*           2015/02/20  1.4  add ppp-fixed as pos1-posmode option
*           2015/05/10  1.5  add pos2-arthres1,2,3,4
*           2015/05/31  1.6  add pos2-armaxiter, pos1-posopt6
*                            add selection precise for pos1-pospot3
*           2015/11/26  1.7  modify pos1-frequency 4:l1+l2+l5+l6 . l1+l5
*           2015/12/05  1.8  add misc-pppopt
*           2016/06/10  1.9  add ant2-maxaveep,ant2-initrst
*           2016/07/31  1.10 add out-outsingle,out-maxsolstd
*           2017/06/14  1.11 add out-outvel
*           2020/11/30  1.12 change options pos1-frequency, pos1-ionoopt,
*                             pos1-tropopt, pos1-sateph, pos1-navsys,
*                             pos2-gloarmode,
*		    2022/05/31 1.0  rewrite options.c with golang by fxb
*-----------------------------------------------------------------------------*/
package gnssgo

import (
	"bufio"
	"fmt"
	"math"
	"os"
	"strconv"
	"strings"
)

/* system options buffer -----------------------------------------------------*/
var (
	prcopt_                         PrcOpt
	solopt_                         SolOpt
	filopt_                         FilOpt
	antpostype_                     [2]int
	elmask_, elmaskar_, elmaskhold_ float64
	antpos_                         [2][3]float64
	exsats_                         string
	snrmask_                        [NFREQ]string

	/* system options table ------------------------------------------------------*/
	SWTOPT  string = "0:off,1:on"
	MODOPT  string = "0:single,1:dgps,2:kinematic,3:static,4:movingbase,5:fixed,6:ppp-kine,7:ppp-static,8:ppp-fixed"
	FRQOPT  string = "1:l1,2:l1+l2,3:l1+l2+l5,4:l1+l5"
	TYPOPT  string = "0:forward,1:backward,2:combined"
	IONOPT  string = "0:off,1:brdc,2:sbas,3:dual-freq,4:est-stec,5:ionex-tec,6:qzs-brdc"
	TRPOPT  string = "0:off,1:saas,2:sbas,3:est-ztd,4:est-ztdgrad"
	EPHOPT  string = "0:brdc,1:precise,2:brdc+sbas,3:brdc+ssrapc,4:brdc+ssrcom"
	NAVOPT  string = "1:gps+2:sbas+4:glo+8:gal+16:qzs+32:bds+64:navic"
	GAROPT  string = "0:off,1:on"
	SOLOPT  string = "0:llh,1:xyz,2:enu,3:nmea"
	TSYOPT  string = "0:gpst,1:utc,2:jst"
	TFTOPT  string = "0:tow,1:hms"
	DFTOPT  string = "0:deg,1:dms"
	HGTOPT  string = "0:ellipsoidal,1:geodetic"
	GEOOPT  string = "0:internal,1:egm96,2:egm08_2.5,3:egm08_1,4:gsi2000"
	STAOPT  string = "0:all,1:single"
	STSOPT  string = "0:off,1:state,2:residual"
	ARMOPT  string = "0:off,1:continuous,2:instantaneous,3:fix-and-hold"
	POSOPT  string = "0:llh,1:xyz,2:single,3:posfile,4:rinexhead,5:rtcm,6:raw"
	TIDEOPT string = "0:off,1:on,2:otl"
	PHWOPT  string = "0:off,1:on,2:precise"
)

var SysOpts map[string]*Opt = map[string]*Opt{
	"pos1-posmode":     {"pos1-posmode", 3, &prcopt_.Mode, nil, nil, MODOPT},
	"pos1-frequency":   {"pos1-frequency", 3, &prcopt_.Nf, nil, nil, FRQOPT},
	"pos1-soltype":     {"pos1-soltype", 3, &prcopt_.SolType, nil, nil, TYPOPT},
	"pos1-elmask":      {"pos1-elmask", 1, nil, &elmask_, nil, "deg"},
	"pos1-snrmask_r":   {"pos1-snrmask_r", 3, &prcopt_.SnrMask.ena[0], nil, nil, SWTOPT},
	"pos1-snrmask_b":   {"pos1-snrmask_b", 3, &prcopt_.SnrMask.ena[1], nil, nil, SWTOPT},
	"pos1-snrmask_L1":  {"pos1-snrmask_L1", 2, nil, nil, &snrmask_[0], ""},
	"pos1-snrmask_L2":  {"pos1-snrmask_L2", 2, nil, nil, &snrmask_[1], ""},
	"pos1-snrmask_L5":  {"pos1-snrmask_L5", 2, nil, nil, &snrmask_[2], ""},
	"pos1-dynamics":    {"pos1-dynamics", 3, &prcopt_.Dynamics, nil, nil, SWTOPT},
	"pos1-tidecorr":    {"pos1-tidecorr", 3, &prcopt_.TideCorr, nil, nil, TIDEOPT},
	"pos1-ionoopt":     {"pos1-ionoopt", 3, &prcopt_.IonoOpt, nil, nil, IONOPT},
	"pos1-tropopt":     {"pos1-tropopt", 3, &prcopt_.TropOpt, nil, nil, TRPOPT},
	"pos1-sateph":      {"pos1-sateph", 3, &prcopt_.SatEph, nil, nil, EPHOPT},
	"pos1-posopt1":     {"pos1-posopt1", 3, &prcopt_.PosOpt[0], nil, nil, SWTOPT},
	"pos1-posopt2":     {"pos1-posopt2", 3, &prcopt_.PosOpt[1], nil, nil, SWTOPT},
	"pos1-posopt3":     {"pos1-posopt3", 3, &prcopt_.PosOpt[2], nil, nil, PHWOPT},
	"pos1-posopt4":     {"pos1-posopt4", 3, &prcopt_.PosOpt[3], nil, nil, SWTOPT},
	"pos1-posopt5":     {"pos1-posopt5", 3, &prcopt_.PosOpt[4], nil, nil, SWTOPT},
	"pos1-posopt6":     {"pos1-posopt6", 3, &prcopt_.PosOpt[5], nil, nil, SWTOPT},
	"pos1-exclsats":    {"pos1-exclsats", 2, nil, nil, &exsats_, "prn ..."},
	"pos1-navsys":      {"pos1-navsys", 0, &prcopt_.NavSys, nil, nil, NAVOPT},
	"pos2-armode":      {"pos2-armode", 3, &prcopt_.ModeAr, nil, nil, ARMOPT},
	"pos2-gloarmode":   {"pos2-gloarmode", 3, &prcopt_.GloModeAr, nil, nil, GAROPT},
	"pos2-bdsarmode":   {"pos2-bdsarmode", 3, &prcopt_.BDSModeAr, nil, nil, SWTOPT},
	"pos2-arthres":     {"pos2-arthres", 1, nil, &prcopt_.ThresAr[0], nil, ""},
	"pos2-arthres1":    {"pos2-arthres1", 1, nil, &prcopt_.ThresAr[1], nil, ""},
	"pos2-arthres2":    {"pos2-arthres2", 1, nil, &prcopt_.ThresAr[2], nil, ""},
	"pos2-arthres3":    {"pos2-arthres3", 1, nil, &prcopt_.ThresAr[3], nil, ""},
	"pos2-arthres4":    {"pos2-arthres4", 1, nil, &prcopt_.ThresAr[4], nil, ""},
	"pos2-arlockcnt":   {"pos2-arlockcnt", 0, &prcopt_.MinLock, nil, nil, ""},
	"pos2-arelmask":    {"pos2-arelmask", 1, nil, &elmaskar_, nil, "deg"},
	"pos2-arminfix":    {"pos2-arminfix", 0, &prcopt_.MinFix, nil, nil, ""},
	"pos2-armaxiter":   {"pos2-armaxiter", 0, &prcopt_.ArMaxIter, nil, nil, ""},
	"pos2-elmaskhold":  {"pos2-elmaskhold", 1, nil, &elmaskhold_, nil, "deg"},
	"pos2-aroutcnt":    {"pos2-aroutcnt", 0, &prcopt_.MaxOut, nil, nil, ""},
	"pos2-maxage":      {"pos2-maxage", 1, nil, &prcopt_.MaxTmDiff, nil, "s"},
	"pos2-syncsol":     {"pos2-syncsol", 3, &prcopt_.SyncSol, nil, nil, SWTOPT},
	"pos2-slipthres":   {"pos2-slipthres", 1, nil, &prcopt_.ThresSlip, nil, "m"},
	"pos2-rejionno":    {"pos2-rejionno", 1, nil, &prcopt_.MaxInno, nil, "m"},
	"pos2-rejgdop":     {"pos2-rejgdop", 1, nil, &prcopt_.MaxGdop, nil, ""},
	"pos2-niter":       {"pos2-niter", 0, &prcopt_.NoIter, nil, nil, ""},
	"pos2-baselen":     {"pos2-baselen", 1, nil, &prcopt_.Baseline[0], nil, "m"},
	"pos2-basesig":     {"pos2-basesig", 1, nil, &prcopt_.Baseline[1], nil, "m"},
	"out-solformat":    {"out-solformat", 3, &solopt_.Posf, nil, nil, SOLOPT},
	"out-outhead":      {"out-outhead", 3, &solopt_.OutHead, nil, nil, SWTOPT},
	"out-outopt":       {"out-outopt", 3, &solopt_.OutOpt, nil, nil, SWTOPT},
	"out-outvel":       {"out-outvel", 3, &solopt_.OutVel, nil, nil, SWTOPT},
	"out-timesys":      {"out-timesys", 3, &solopt_.TimeS, nil, nil, TSYOPT},
	"out-timeform":     {"out-timeform", 3, &solopt_.TimeF, nil, nil, TFTOPT},
	"out-timendec":     {"out-timendec", 0, &solopt_.TimeU, nil, nil, ""},
	"out-degform":      {"out-degform", 3, &solopt_.DegF, nil, nil, DFTOPT},
	"out-fieldsep":     {"out-fieldsep", 2, nil, nil, &solopt_.Sep, ""},
	"out-outsingle":    {"out-outsingle", 3, &prcopt_.OutSingle, nil, nil, SWTOPT},
	"out-maxsolstd":    {"out-maxsolstd", 1, nil, &solopt_.MaxSolStd, nil, "m"},
	"out-height":       {"out-height", 3, &solopt_.Height, nil, nil, HGTOPT},
	"out-geoid":        {"out-geoid", 3, &solopt_.Geoid, nil, nil, GEOOPT},
	"out-solstatic":    {"out-solstatic", 3, &solopt_.SolStatic, nil, nil, STAOPT},
	"out-nmeaintv1":    {"out-nmeaintv1", 1, nil, &solopt_.NmeaIntv[0], nil, "s"},
	"out-nmeaintv2":    {"out-nmeaintv2", 1, nil, &solopt_.NmeaIntv[1], nil, "s"},
	"out-outstat":      {"out-outstat", 3, &solopt_.SStat, nil, nil, STSOPT},
	"stats-eratio1":    {"stats-eratio1", 1, nil, &prcopt_.eratio[0], nil, ""},
	"stats-eratio2":    {"stats-eratio2", 1, nil, &prcopt_.eratio[1], nil, ""},
	"stats-errphase":   {"stats-errphase", 1, nil, &prcopt_.Err[1], nil, "m"},
	"stats-errphaseel": {"stats-errphaseel", 1, nil, &prcopt_.Err[2], nil, "m"},
	"stats-errphasebl": {"stats-errphasebl", 1, nil, &prcopt_.Err[3], nil, "m/10km"},
	"stats-errdoppler": {"stats-errdoppler", 1, nil, &prcopt_.Err[4], nil, "Hz"},
	"stats-stdbias":    {"stats-stdbias", 1, nil, &prcopt_.Std[0], nil, "m"},
	"stats-stdiono":    {"stats-stdiono", 1, nil, &prcopt_.Std[1], nil, "m"},
	"stats-stdtrop":    {"stats-stdtrop", 1, nil, &prcopt_.Std[2], nil, "m"},
	"stats-prnaccelh":  {"stats-prnaccelh", 1, nil, &prcopt_.Prn[3], nil, "m/s^2"},
	"stats-prnaccelv":  {"stats-prnaccelv", 1, nil, &prcopt_.Prn[4], nil, "m/s^2"},
	"stats-prnbias":    {"stats-prnbias", 1, nil, &prcopt_.Prn[0], nil, "m"},
	"stats-prniono":    {"stats-prniono", 1, nil, &prcopt_.Prn[1], nil, "m"},
	"stats-prntrop":    {"stats-prntrop", 1, nil, &prcopt_.Prn[2], nil, "m"},
	"stats-prnpos":     {"stats-prnpos", 1, nil, &prcopt_.Prn[5], nil, "m"},
	"stats-clkstab":    {"stats-clkstab", 1, nil, &prcopt_.SatClkStab, nil, "s/s"},
	"ant1-postype":     {"ant1-postype", 3, &antpostype_[0], nil, nil, POSOPT},
	"ant1-pos1":        {"ant1-pos1", 1, nil, &antpos_[0][0], nil, "deg|m"},
	"ant1-pos2":        {"ant1-pos2", 1, nil, &antpos_[0][1], nil, "deg|m"},
	"ant1-pos3":        {"ant1-pos3", 1, nil, &antpos_[0][2], nil, "m|m"},
	"ant1-anttype":     {"ant1-anttype", 2, nil, nil, &prcopt_.AntType[0], ""},
	"ant1-antdele":     {"ant1-antdele", 1, nil, &prcopt_.AntDel[0][0], nil, "m"},
	"ant1-antdeln":     {"ant1-antdeln", 1, nil, &prcopt_.AntDel[0][1], nil, "m"},
	"ant1-antdelu":     {"ant1-antdelu", 1, nil, &prcopt_.AntDel[0][2], nil, "m"},
	"ant2-postype":     {"ant2-postype", 3, &antpostype_[1], nil, nil, POSOPT},
	"ant2-pos1":        {"ant2-pos1", 1, nil, &antpos_[1][0], nil, "deg|m"},
	"ant2-pos2":        {"ant2-pos2", 1, nil, &antpos_[1][1], nil, "deg|m"},
	"ant2-pos3":        {"ant2-pos3", 1, nil, &antpos_[1][2], nil, "m|m"},
	"ant2-anttype":     {"ant2-anttype", 2, nil, nil, &prcopt_.AntType[1], ""},
	"ant2-antdele":     {"ant2-antdele", 1, nil, &prcopt_.AntDel[1][0], nil, "m"},
	"ant2-antdeln":     {"ant2-antdeln", 1, nil, &prcopt_.AntDel[1][1], nil, "m"},
	"ant2-antdelu":     {"ant2-antdelu", 1, nil, &prcopt_.AntDel[1][2], nil, "m"},
	"ant2-maxaveep":    {"ant2-maxaveep", 0, &prcopt_.MaxAveEp, nil, nil, ""},
	"ant2-initrst":     {"ant2-initrst", 3, &prcopt_.InitRst, nil, nil, SWTOPT},
	"misc-timeinterp":  {"misc-timeinterp", 3, &prcopt_.IntPref, nil, nil, SWTOPT},
	"misc-sbasatsel":   {"misc-sbasatsel", 0, &prcopt_.SbasSatSel, nil, nil, "0:all"},
	"misc-rnxopt1":     {"misc-rnxopt1", 2, nil, nil, &prcopt_.RnxOpt[0], ""},
	"misc-rnxopt2":     {"misc-rnxopt2", 2, nil, nil, &prcopt_.RnxOpt[1], ""},
	"misc-pppopt":      {"misc-pppopt", 2, nil, nil, &prcopt_.PPPOpt, ""},
	"file-satantfile":  {"file-satantfile", 2, nil, nil, &filopt_.SatAntPara, ""},
	"file-rcvantfile":  {"file-rcvantfile", 2, nil, nil, &filopt_.RcvAntPara, ""},
	"file-staposfile":  {"file-staposfile", 2, nil, nil, &filopt_.StaPos, ""},
	"file-geoidfile":   {"file-geoidfile", 2, nil, nil, &filopt_.Geoid, ""},
	"file-ionofile":    {"file-ionofile", 2, nil, nil, &filopt_.Iono, ""},
	"file-dcbfile":     {"file-dcbfile", 2, nil, nil, &filopt_.Dcb, ""},
	"file-eopfile":     {"file-eopfile", 2, nil, nil, &filopt_.Eop, ""},
	"file-blqfile":     {"file-blqfile", 2, nil, nil, &filopt_.Blq, ""},
	"file-tempdir":     {"file-tempdir", 2, nil, nil, &filopt_.TempDir, ""},
	"file-geexefile":   {"file-geexefile", 2, nil, nil, &filopt_.GeExe, ""},
	"file-solstatfile": {"file-solstatfile", 2, nil, nil, &filopt_.SolStat, ""},
	"file-tracefile":   {"file-tracefile", 2, nil, nil, &filopt_.Trace, ""}}

/* discard space characters at tail ------------------------------------------*/
func options_chop(buff *string) {
	idx := strings.Index(*buff, "#")
	if idx >= 0 {
		*buff = (*buff)[:idx]
	}
	*buff = strings.TrimFunc(*buff, func(r rune) bool {
		return !strconv.IsGraphic(r)
	})
}

/* enum to string ------------------------------------------------------------*/
func Enum2Str(s *string, comment string, val int) int {
	str := fmt.Sprintf("%d:", val)
	n := len(str)
	var index int
	if index = strings.Index(comment, str); index < 0 {
		*s = str
		return n
	}

	q1 := strings.Index(comment[index+n:], ",")
	q2 := strings.Index(comment[index+n:], ")")
	if q1 < 0 && q2 < 0 {
		*s = comment[index+n:]
		return len(*s)
	}
	*s = comment[index+n : int(math.Min(float64(q1), float64(q2)))-index-n]
	return len(*s)
}

/* string to enum ------------------------------------------------------------*/
func Str2Enum(str, comment string, val *int) int {
	index := strings.Index(comment, str)
	if index >= 0 {
		q := strings.LastIndex(comment[:index], ":")
		if q > 0 {
			q--
		}
		n, _ := fmt.Sscanf(comment[q:], "%d", val)
		if n == 1 {
			return 1
		}
		return 0
	}

	s := fmt.Sprintf("%.30s:", str)
	if index = strings.Index(comment, s); index > 0 {
		n, _ := fmt.Sscanf(comment[index:], "%d", val)
		if n == 1 {
			return 1
		}
		return 0

	}
	return 0
}

/* search option ---------------------------------------------------------------
* search option record
* args   : char   *name     I  option name
*          opt_t  *opts     I  options table
*                              (terminated with table[i].name="")
* return : option record (NULL: not found)
*-----------------------------------------------------------------------------*/
func SearchOpt(name string, opts map[string]*Opt) *Opt {
	Trace(4, "searchopt: name=%s\n", name)

	return opts[name]
}

/* string to option value ------------------------------------------------------
* convert string to option value
* args   : opt_t  *opt      O  option
*          char   *str      I  option value string
* return : status (1:ok,0:error)
*-----------------------------------------------------------------------------*/
func (opt *Opt) Str2Opt(str string) int {
	switch opt.Format {
	case 0:
		*opt.VarInt, _ = strconv.Atoi(str)
	case 1:
		*opt.VarFloat, _ = strconv.ParseFloat(str, 64)
	case 2:
		*opt.VarString = str
	case 3:
		var enum int
		ret := Str2Enum(str, opt.Comment, &enum)
		*opt.VarInt = enum
		return ret
	default:
		return 0
	}
	return 1
}

/* option value to string ------------------------------------------------------
* convert option value to string
* args   : opt_t  *opt      I  option
*          char   *str      O  option value string
* return : length of output string
*-----------------------------------------------------------------------------*/
func (opt *Opt) Opt2Str(str *string) int {

	n := len(*str)
	switch opt.Format {
	case 0:
		*str += fmt.Sprintf("%d", *opt.VarInt)
	case 1:
		*str += fmt.Sprintf("%.15f", *opt.VarFloat)
	case 2:
		*str += *opt.VarString
	case 3:
		return Enum2Str(str, opt.Comment, *opt.VarInt)
	}
	return len(*str) - n
}

/* option to string -------------------------------------------------------------
* convert option to string (keyword=value # comment)
* args   : opt_t  *opt      I  option
*          char   *buff     O  option string
* return : length of output string
*-----------------------------------------------------------------------------*/
func (opt *Opt) Opt2Buf(buff *string) int {
	p := *buff
	p += fmt.Sprintf("%-18s =", opt.Name)
	opt.Opt2Str(&p)
	if opt.Comment != "" {
		if len(*buff)+30 > len(p) {
			p += fmt.Sprintf("%*s", len(*buff)+30-len(p), "")
		}
		p += fmt.Sprintf(" # (%s)", opt.Comment)
	}
	n := len(p) - len(*buff)
	*buff = p
	return n
}

/* load options ----------------------------------------------------------------
* load options from file
* args   : char   *file     I  options file
*          opt_t  *opts     IO options table
*                              (terminated with table[i].name="")
* return : status (1:ok,0:error)
*-----------------------------------------------------------------------------*/
func LoadOpts(file string, opts *map[string]*Opt) int {
	var (
		fp    *os.File
		opt   *Opt
		err   error
		n     int = 0
		index int
	)
	Trace(4, "loadopts: file=%s\n", file)
	fp, err = os.OpenFile(file, os.O_RDONLY, 0666)
	if err != nil {
		Trace(2, "loadopts: options file open error (%s)\n", file)
		return 0
	}
	defer fp.Close()
	rd := bufio.NewReader(fp)
	for {
		n++
		buff, err := rd.ReadString('\n')
		if err != nil {
			break
		}

		options_chop(&buff)
		if index = strings.Index(buff, "="); index < 0 {
			Trace(2, "invalid option %s (%s:%d)\n", buff, file, n)
			continue
		}
		name := strings.TrimSpace(buff[:index])
		value := strings.TrimSpace(buff[index+1:])
		options_chop(&name)
		options_chop(&value)
		if opt = SearchOpt(name, *opts); opt == nil {
			continue
		}

		if opt.Str2Opt(value) == 0 {
			Trace(2, "invalid option value %s (%s:%d)\n", buff, file, n)
			continue
		}
	}

	return 1
}

/* save options to file --------------------------------------------------------
* save options to file
* args   : char   *file     I  options file
*          char   *mode     I  write mode ("w":overwrite,"a":append);
*          char   *comment  I  header comment (NULL: no comment)
*          opt_t  *opts     I  options table
*                              (terminated with table[i].name="")
* return : status (1:ok,0:error)
*-----------------------------------------------------------------------------*/
func SaveOpts(file, mode, comment string, opts *map[string]*Opt) int {
	var (
		fp  *os.File
		err error
	)

	Trace(4, "saveopts: file=%s mode=%s\n", file, mode)
	fp, err = os.OpenFile(file, os.O_CREATE|os.O_WRONLY|os.O_TRUNC, os.ModeAppend|os.ModePerm)
	if err != nil {
		Trace(2, "saveopts: options file open error (%s)\n", file)
		return 0
	}
	defer fp.Close()
	fp.WriteString(fmt.Sprintf("# %s\n\n", comment))

	for i := range *opts {
		var buff string

		(*opts)[i].Opt2Buf(&buff)
		fp.WriteString(buff)
	}

	return 1
}

/* system options buffer to options ------------------------------------------*/
func Buff2SysOpts() {
	var (
		pos       [3]float64
		rr        []float64
		buff, id  string
		i, j, sat int
		ps        *int
	)

	prcopt_.Elmin = elmask_ * D2R
	prcopt_.ElMaskAr = elmaskar_ * D2R
	prcopt_.ElMaskHold = elmaskhold_ * D2R

	for i = 0; i < 2; i++ {
		if i == 0 {
			ps = &prcopt_.RovPos
			rr = prcopt_.Ru[:]
		} else {
			ps = &prcopt_.RefPos
			rr = prcopt_.Rb[:]
		}

		if antpostype_[i] == 0 { /* lat/lon/hgt */
			*ps = 0
			pos[0] = antpos_[i][0] * D2R
			pos[1] = antpos_[i][1] * D2R
			pos[2] = antpos_[i][2]
			Pos2Ecef(pos[:], rr)
		} else if antpostype_[i] == 1 { /* xyz-ecef */
			*ps = 0
			rr[0] = antpos_[i][0]
			rr[1] = antpos_[i][1]
			rr[2] = antpos_[i][2]
		} else {
			*ps = antpostype_[i] - 1
		}
	}
	/* excluded satellites */
	for i = 0; i < MAXSAT; i++ {
		prcopt_.ExSats[i] = 0
	}
	if len(exsats_) > 0 {
		buff = exsats_

		satids := strings.Fields(buff)
		for i = range satids {
			if satids[i][0] == '+' {
				id = satids[i][1:]
			} else {
				id = satids[i]
			}
			if sat = SatId2No(id); sat == 0 {
				continue
			}
			if satids[i][0] == '+' {
				prcopt_.ExSats[sat] = 2
			} else {
				prcopt_.ExSats[sat] = 1

			}
		}
	}
	/* snrmask */
	for i = 0; i < NFREQ; i++ {
		for j = 0; j < 9; j++ {
			prcopt_.SnrMask.mask[i][j] = 0.0
		}
		buff = snrmask_[i]
		snrs := strings.Split(buff, ",")
		j = 0
		for k := range snrs {
			prcopt_.SnrMask.mask[i][j], _ = strconv.ParseFloat(snrs[k], 64)
			j++
		}
	}
	/* number of frequency (4:L1+L5) */
	if prcopt_.Nf == 4 {
		prcopt_.Nf = 3
		prcopt_.FreqOpt = 1
	}
}

/* options to system options buffer ------------------------------------------*/
func SysOpts2Buff() {
	var (
		pos       [3]float64
		rr        []float64
		id        string
		i, j, sat int
		ps        *int
	)

	elmask_ = prcopt_.Elmin * R2D
	elmaskar_ = prcopt_.ElMaskAr * R2D
	elmaskhold_ = prcopt_.ElMaskHold * R2D

	for i = 0; i < 2; i++ {
		if i == 0 {
			ps = &prcopt_.RovPos
			rr = prcopt_.Ru[:]
		} else {
			ps = &prcopt_.RefPos
			rr = prcopt_.Rb[:]
		}

		if *ps == 0 {
			antpostype_[i] = 0
			Ecef2Pos(rr, pos[:])
			antpos_[i][0] = pos[0] * R2D
			antpos_[i][1] = pos[1] * R2D
			antpos_[i][2] = pos[2]
		} else {
			antpostype_[i] = *ps + 1
		}
	}
	/* excluded satellites */
	exsats_ = ""
	j = 0
	var s1, s2 string
	for sat = range prcopt_.ExSats {
		SatNo2Id(sat, &id)
		if j == 0 {
			s1 = ""
			j += 1
		}
		if prcopt_.ExSats[sat] == 2 {
			s2 = "+"
		} else {
			s2 = ""
		}
		exsats_ += fmt.Sprintf("%s%s%s", s1, s2, id)
	}

	/* snrmask */
	for i = 0; i < NFREQ; i++ {
		snrmask_[i] = ""
		s1 = ""
		for j = 0; j < 9; j++ {
			if j > 0 {
				s1 = ","
			}
			snrmask_[i] += fmt.Sprintf("%s%.0f", s1, prcopt_.SnrMask.mask[i][j])

		}
	}
	/* number of frequency (4:L1+L5) */
	if prcopt_.Nf == 3 && prcopt_.FreqOpt == 1 {
		prcopt_.Nf = 4
		prcopt_.FreqOpt = 0
	}
}

/* reset system options to default ---------------------------------------------
* reset system options to default
* args   : none
* return : none
*-----------------------------------------------------------------------------*/
func ResetSysOpts() {

	Trace(4, "resetsysopts:\n")

	prcopt_ = DefaultProcOpt()
	solopt_ = DefaultSolOpt()
	filopt_.SatAntPara = ""
	filopt_.RcvAntPara = ""
	filopt_.StaPos = ""
	filopt_.Geoid = ""
	filopt_.Dcb = ""
	filopt_.Blq = ""
	filopt_.SolStat = ""
	filopt_.Trace = ""
	for i := 0; i < 2; i++ {
		antpostype_[i] = 0
	}
	elmask_ = 15.0
	elmaskar_ = 0.0
	elmaskhold_ = 0.0
	for i := 0; i < 2; i++ {
		for j := 0; j < 3; j++ {
			antpos_[i][j] = 0.0
		}
	}
	exsats_ = ""
}

/* get system options ----------------------------------------------------------
* get system options
* args   : prcopt_t *popt   IO processing options (NULL: no output)
*          solopt_t *sopt   IO solution options   (NULL: no output)
*          folopt_t *fopt   IO file options       (NULL: no output)
* return : none
* notes  : to load system options, use loadopts() before calling the function
*-----------------------------------------------------------------------------*/
func GetSysOpts(popt *PrcOpt, sopt *SolOpt, fopt *FilOpt) {
	Trace(4, "getsysopts:\n")

	Buff2SysOpts()
	if popt != nil {
		*popt = prcopt_
	}
	if sopt != nil {
		*sopt = solopt_
	}
	if fopt != nil {
		*fopt = filopt_
	}
}

/* set system options ----------------------------------------------------------
* set system options
* args   : prcopt_t *prcopt I  processing options (NULL: default)
*          solopt_t *solopt I  solution options   (NULL: default)
*          filopt_t *filopt I  file options       (NULL: default)
* return : none
* notes  : to save system options, use saveopts() after calling the function
*-----------------------------------------------------------------------------*/
func SetSysOpts(popt *PrcOpt, sopt *SolOpt, fopt *FilOpt) {
	Trace(4, "setsysopts:\n")

	ResetSysOpts()
	if popt != nil {
		prcopt_ = *popt
	}
	if sopt != nil {
		solopt_ = *sopt
	}
	if fopt != nil {
		filopt_ = *fopt
	}
	SysOpts2Buff()
}
