/*------------------------------------------------------------------------------
* download.c : gnss data downloader
*
*          Copyright (C) 2012-2020 by T.TAKASU, All rights reserved.
*
* version : $Revision:$ $Date:$
* history : 2012/12/28  1.0  new
*           2013/06/02  1.1  replace S_IREAD by S_IRUSR
*           2020/11/30  1.2  support protocol https:// and ftps://
*                            support compressed RINEX (CRX) files
*                            support wild-card (*) in URL for FTP and FTPS
*                            use "=" to separate file name from URL
*                            fix bug on double-free of download paths
*                            limit max number of download paths
*                            use integer types in stdint.h
*		    2022/05/31  1.0  rewrite download.c with golang by fxb
*-----------------------------------------------------------------------------*/
package gnssgo

import (
	"bufio"
	"fmt"
	"io"
	"math"
	"os"
	"runtime"
	"strings"
)

const FTP_CMD string = "wget"         /* FTP/HTTP command */
const FTP_TIMEOUT int = 60            /* FTP/HTTP timeout (s) */
const FTP_LISTING string = ".listing" /* FTP listing file */
const FTP_NOFILE int = 2048           /* FTP error no file */
const HTTP_NOFILE int = 1             /* HTTP error no file */
const FTP_RETRY int = 3               /* FTP number of retry */
const MAX_PATHS int = 131072          /* max number of download paths */

/* type definitions ----------------------------------------------------------*/

type Path struct { /* download path type */
	remot string /* remote path */
	local string /* local path */
}

//

type Paths struct { /* download paths type */
	path    []Path /* download paths */
	n, nmax int    /* number and max number of paths */
}

/* execute command with test timeout -----------------------------------------*/
func execcmd_to(cmd string) int {
	return ExecCmd(cmd)
}

/* generate path by replacing keywords ---------------------------------------*/
func GenPath(file, name string, time Gtime, seqno int, path *string) {
	var (
		buff           string
		env            string
		l_name, u_name string
		i              int
	)
	l_name = strings.ToLower(name)
	u_name = strings.ToUpper(name)
	ss := strings.Split(file, "%")
	for _, s := range ss {
		for i = 0; i < len(s); i++ {

			switch s[i] {
			case 's', 'r':
				buff += l_name
			case 'S', 'R':
				buff += u_name
			case 'N':
				buff += fmt.Sprintf("%d", seqno)
			case '{':
				if idx := strings.Index(s[1:], "}"); idx > 0 {
					v := s[1:idx]
					if env = os.Getenv(v); len(env) > 0 {
						buff += env
					}
					i = idx
				}
			}
		}
	}

	RepPath(buff, path, time, "", "")
}

/* compare str1 and str2 with wildcards (*) ----------------------------------*/
func cmp_str(str1, str2 string) int {
	var idx int = 0
	ss := strings.Split(str2, "*")
	for _, s := range ss {
		idx = strings.Index(str1[idx:], s)
		if idx < 0 {
			break
		}
	}
	return idx
}

/* remote to local file path -------------------------------------------------*/
func Remot2Local(remot, dir string, local *string) {
	var p string
	var idx int
	if idx = strings.LastIndex(remot, "="); idx >= 0 {
		p = remot[idx:]
	} else if idx = strings.LastIndex(remot, "/"); idx >= 0 {
		p = remot[idx:]
	} else {
		p = remot
	}

	*local = fmt.Sprintf("%s%s%s", dir, FILEPATHSEP, p)
}

/* test file existence -------------------------------------------------------*/
func exist_file(local string) int {
	_, err := os.Lstat(local)
	if os.IsExist(err) {
		return 1
	}
	return 0
}

/* test local file existence -------------------------------------------------*/
func test_file(local string) int {
	var (
		buff      string
		comp, idx int
	)

	if strings.Contains(local, "*") { /* test wild-card (*) in path */
		return 0
	}

	buff = local
	idx = strings.LastIndex(buff, ".")
	if idx >= 0 {
		if strings.EqualFold(buff[idx:], ".z") || strings.EqualFold(buff[idx:], ".gz") ||
			strings.EqualFold(buff[idx:], ".zip") {
			buff = buff[:idx]
			if exist_file(buff) > 0 {
				return 1
			}
			comp = 1
		}
	}

	idx = strings.LastIndex(buff, ".")
	if idx >= 0 && len(buff[idx:]) == 4 && (buff[idx+3] == 'd' || buff[idx+3] == 'D') {
		var p []byte = []byte(buff)
		if p[idx+3] == 'd' {
			p[idx+3] = 'o'
		} else {
			p[idx+3] = 'O'
		}
		buff = string(p)
		if exist_file(buff) > 0 {
			return 1
		}
		comp = 1
	}

	idx = strings.LastIndex(buff, ".")
	if idx >= 0 {
		if strings.EqualFold(buff[idx:], ".crx") {
			buff = buff[:idx] + ".cro"
		} else {
			buff = buff[:idx] + ".CRO"
		}
		if exist_file(buff) > 0 {
			return 1
		}
		comp = 1
	}
	if exist_file(buff) == 0 {
		return 0
	}
	if comp > 0 {
		return 2
	}
	return 1
}

/* free download paths -------------------------------------------------------*/
func free_path(paths *Paths) {

	if paths == nil {
		return
	}
	paths.n = 0
	paths.nmax = 0
	paths.path = nil
}

/* add download paths --------------------------------------------------------*/
func add_path(paths *Paths, remot, dir string) int {
	var (
		paths_path []Path
		local      string
	)

	if paths.n >= paths.nmax {
		if paths.nmax <= 0 {
			paths.nmax = 1024
		} else {
			paths.nmax = paths.nmax * 2
		}

		if paths.nmax > MAX_PATHS {
			return 0
		}
		paths_path = make([]Path, paths.nmax)
		copy(paths_path, paths.path)
		paths.path = paths_path
	}
	Remot2Local(remot, dir, &local)
	paths.path[paths.n].remot = remot
	paths.path[paths.n].local = local
	paths.n++
	return 1
}

/* generate download path ----------------------------------------------------*/
func gen_path(time, time_p Gtime, seqnos, seqnoe int, url *Url, sta, dir string, paths *Paths) int {
	var (
		remot, remot_p, dir_t string
		i                     int
	)

	if len(dir) == 0 {
		dir = url.dir
	}
	if len(dir) == 0 {
		dir = "."
	}
	if strings.Contains(url.path, "%N") {
		for i = seqnos; i <= seqnoe; i++ {
			GenPath(url.path, sta, time, i, &remot)
			GenPath(dir, sta, time, i, &dir_t)
			if time_p.Time > 0 {
				GenPath(url.path, sta, time_p, i, &remot_p)
				if strings.Compare(remot_p, remot) == 0 {
					continue
				}
			}
			if add_path(paths, remot, dir_t) == 0 {
				return 0
			}
		}
	} else {
		GenPath(url.path, sta, time, 0, &remot)
		GenPath(dir, sta, time, 0, &dir_t)
		if time_p.Time > 0 {
			GenPath(url.path, sta, time_p, 0, &remot_p)
			if strings.Compare(remot_p, remot) == 0 {
				return 1
			}
		}
		if add_path(paths, remot, dir_t) == 0 {
			return 0
		}
	}
	return 1
}

/* generate download paths ---------------------------------------------------*/
func gen_paths(time, time_p Gtime, seqnos, seqnoe int, url *Url, stas []string, nsta int, dir string, paths *Paths) int {

	if strings.Contains(url.path, "%s") || strings.Contains(url.path, "%S") {
		for i := 0; i < nsta; i++ {
			if gen_path(time, time_p, seqnos, seqnoe, url, stas[i], dir, paths) == 0 {
				return 0
			}
		}
	} else {
		if gen_path(time, time_p, seqnos, seqnoe, url, "", dir, paths) == 0 {
			return 0
		}
	}
	return 1
}

/* compact download paths ----------------------------------------------------*/
func compact_paths(paths *Paths) {
	for i := 0; i < paths.n; i++ {
		for j := i + 1; j < paths.n; j++ {
			if strings.Compare(paths.path[i].remot, paths.path[j].remot) != 0 {
				continue
			}
			for k := j; k < paths.n-1; k++ {
				paths.path[k] = paths.path[k+1]
			}
			paths.n--
			j--
		}
	}
}

/* get remote file list for FTP or FTPS --------------------------------------*/
func get_list(path *Path, usr, pwd, proxy string) int {
	var (
		fp                         *os.File
		cmd, env, remot, opt, opt2 string //,*opt="",*opt2="",*p;
		idx                        int
	)

	// #ifndef WIN32
	//     opt2=" -o /dev/null";
	// #endif
	if runtime.GOOS == "windows" {
		opt2 = " -o /dev/null"
	}
	os.Remove(FTP_LISTING)

	remot = path.remot
	if idx = strings.LastIndex(remot, "/"); idx > 0 {
		remot = remot[:1] + string("__REQUEST_LIST__")
	} else {
		return 0
	}

	if len(proxy) > 0 {
		env = fmt.Sprintf("set ftp_proxy=http://%s & ", proxy)
		opt = "--proxy=on "
	}
	cmd = fmt.Sprintf("%s%s %s --ftp-user=%s --ftp-password=%s --glob=off --passive-ftp --no-remove-listing -N %s-t 1 -T %d%s\n",
		env, FTP_CMD, remot, usr, pwd, opt, FTP_TIMEOUT, opt2)
	execcmd_to(cmd)
	if fp, _ = os.OpenFile(FTP_LISTING, os.O_RDONLY, 0666); fp == nil {
		return 0
	}
	fp.Close()
	return 1
}

/* replace wild-card (*) in the paths ----------------------------------------*/
func rep_paths(path *Path, file string) int {
	var (
		buff1, buff2 string //,*p,*q,*remot,*local;
		idx          int
	)
	buff1 = path.remot
	buff2 = path.local

	if idx = strings.LastIndex(buff1, "/"); idx > 0 {
		buff1 = buff1[idx:]
	}
	if idx = strings.LastIndex(buff2, FILEPATHSEP); idx > 0 {
		buff2 = buff2[idx:]
	}
	buff1 += file
	buff2 += file
	path.remot = buff1
	path.local = buff2
	return 1
}

/* test file in remote file list ---------------------------------------------*/
func test_list(path *Path) int {
	var (
		fp               *os.File
		buff, file, list string
		idx              int
		err              error
	)

	if fp, err = os.OpenFile(FTP_LISTING, os.O_RDONLY, 0666); err != nil {
		return 1
	}
	defer fp.Close()

	if idx = strings.LastIndex(path.remot, "/"); idx > 0 {
		file = path.remot[idx:]
	} else {
		return 1
	}

	/* search file in remote file list */
	rd := bufio.NewReader(fp)
	for {
		buff, err = rd.ReadString('\n')
		if err != nil {
			break
		}
		/* remove symbolic link */
		if idx = strings.Index(buff, "->"); idx > 0 {
			buff = buff[:idx]
		}
		buff = strings.TrimRightFunc(buff, func(r rune) bool {
			return (r == ' ' || r == '\r' || r == '\n')
		})

		/* file as last field */
		if idx = strings.LastIndex(buff, " "); idx > 0 {
			list = buff[idx:]
		} else {
			list = buff
		}

		if strings.Compare(file, list) == 0 {
			return 1
		}
		/* compare with wild-card (*) */
		if cmp_str(list, file) > 0 {

			/* replace wild-card (*) in the paths */
			if rep_paths(path, list) == 0 {
				return 0
			}
			return 1
		}
	}
	return 0
}

/* execute download ----------------------------------------------------------*/
func exec_down(path *Path, remot_p *string, usr, pwd, proxy string, opts int, n []int, fp *os.File) int {
	var (
		dir, errfile, tmpfile, cmd, env, opt, opt2 string
		ret, proto, idx                            int
	)

	if runtime.GOOS == "windows" {
		opt2 = " 2> /dev/null"
	}

	dir = path.local
	if idx = strings.LastIndex(dir, FILEPATHSEP); idx > 0 {
		dir = dir[:idx]
	}

	switch {
	case path.remot[:6] == "ftp://":
		proto = 0
	case path.remot[:7] == "ftps://":
		proto = 2
	case path.remot[:7] == "http://":
		proto = 1
	case path.remot[:8] == "https://":
		proto = 1
	default:
		Trace(2, "exec_down: invalid path %s\n", path.remot)
		ShowMsg_Ptr("STAT=X")
		if fp != nil {
			fp.WriteString(fmt.Sprintf("%s ERROR (INVALID PATH)\n", path.remot))
		}
		n[1]++
		return 0
	}
	/* test local file existence */
	if opts&DLOPT_FORCE == 0 && test_file(path.local) > 0 {
		ShowMsg_Ptr("STAT=.")
		if fp != nil {
			fp.WriteString(fmt.Sprintf("%s in %s\n", path.remot, dir))
		}
		n[2]++
		return 0
	}
	ShowMsg_Ptr("STAT=_")

	/* get remote file list for FTP or FTPS */
	if idx = strings.LastIndex(path.remot, "/"); (proto == 0 || proto == 2) && idx >= 0 &&
		strings.Compare(path.remot[:idx], *remot_p) != 0 {

		if get_list(path, usr, pwd, proxy) > 0 {
			*remot_p = path.remot
		}
	}
	/* test file in listing for FTP or FTPS or extend wild-card in file path */
	if (proto == 0 || proto == 2) && test_list(path) == 0 {
		ShowMsg_Ptr("STAT=x")
		if fp != nil {
			fp.WriteString(fmt.Sprintf("%s NO_FILE\n", path.remot))
		}
		n[1]++
		return 0
	}
	/* generate local directory recursively */
	if mkdir_r(dir) == 0 {
		ShowMsg_Ptr("STAT=X")
		if fp != nil {
			fp.WriteString(fmt.Sprintf("%s . %s ERROR (LOCAL DIR)\n", path.remot, dir))
		}
		n[3]++
		return 0
	}
	/* re-test local file existence for file with wild-card */
	if (opts&DLOPT_FORCE) == 0 && test_file(path.local) > 0 {
		ShowMsg_Ptr("STAT=.")
		if fp != nil {
			fp.WriteString(fmt.Sprintf("%s in %s\n", path.remot, dir))
		}
		n[2]++
		return 0
	}
	/* proxy option */
	if len(proxy) > 0 {
		sproto := "http"
		if proto == 0 || proto == 2 {
			sproto = "ftp"
		}
		env = fmt.Sprintf("set %s_proxy=http://%s & ", sproto, proxy)
		opt = " --proxy=on "
	}
	/* download command */
	errfile = fmt.Sprintf("%s.err", path.local)
	if proto == 0 || proto == 2 {
		cmd = fmt.Sprintf("%s%s %s --ftp-user=%s --ftp-password=%s --glob=off --passive-ftp %s-t %d -T %d -O \"%s\" -o \"%s\"%s\n",
			env, FTP_CMD, path.remot, usr, pwd, opt, FTP_RETRY, FTP_TIMEOUT,
			path.local, errfile, opt2)
	} else {
		if len(pwd) > 0 {
			opt += fmt.Sprintf(" --http-user=%s --http-password=%s ", usr, pwd)
		}
		cmd = fmt.Sprintf("%s%s %s %s-t %d -T %d -O \"%s\" -o \"%s\"%s\n", env, FTP_CMD,
			path.remot, opt, FTP_RETRY, FTP_TIMEOUT, path.local, errfile, opt2)
	}
	if fp != nil {
		fp.WriteString(fmt.Sprintf("%s . %s", path.remot, dir))
	}

	/* execute download command */
	if ret = execcmd_to(cmd); ret > 0 {
		if (proto == 0 && ret == FTP_NOFILE) ||
			(proto == 1 && ret == HTTP_NOFILE) {
			ShowMsg_Ptr("STAT=x")
			if fp != nil {
				fp.WriteString(" NO_FILE\n")
			}
			n[1]++
		} else {
			Trace(2, "exec_down: error proto=%d %d\n", proto, ret)
			ShowMsg_Ptr("STAT=X")
			if fp != nil {
				fp.WriteString(fmt.Sprintf(" ERROR (%d)\n", ret))
			}
			n[3]++
		}
		os.Remove(path.local)
		if (opts & DLOPT_HOLDERR) == 0 {
			os.Remove(errfile)
		}
		if ret == 2 {
			return 1
		}
		return 0
	}
	os.Remove(errfile)

	/* uncompress download file */
	if idx = strings.LastIndex(path.local, "."); (opts&DLOPT_KEEPCMP) == 0 && idx > 0 &&
		strings.EqualFold(path.local[idx:], ".z") || strings.EqualFold(path.local[idx:], ".gz") ||
		strings.EqualFold(path.local[idx:], ".zip") {

		if Rtk_Uncompress(path.local, &tmpfile) > 0 {
			os.Remove(path.local)
		} else {
			Trace(2, "exec_down: uncompress error\n")
			ShowMsg_Ptr("STAT=C")
			if fp != nil {
				fp.WriteString(" ERROR (UNCOMP)\n")
			}
			n[3]++
			return 0
		}
	}
	ShowMsg_Ptr("STAT=o")
	if fp != nil {
		fp.WriteString(" OK\n")
	}
	n[0]++
	return 0
}

/* test local file -----------------------------------------------------------*/
func test_local(ts, te Gtime, ti float64, path, sta, dir string, nc, nt *int, fp *os.File) int {
	var (
		time                     Gtime
		remot, dir_t, local, str string
		stat, abort              int
	)

	for time = ts; TimeDiff(time, te) <= 1e-3; time = TimeAdd(time, ti) {

		str = fmt.Sprintf("%s.%s", path, local)

		Trace(2, str)
		if ShowMsg_Ptr(str) > 0 {
			abort = 1
			break
		}
		GenPath(path, sta, time, 0, &remot)
		GenPath(dir, sta, time, 0, &dir_t)
		Remot2Local(remot, dir_t, &local)

		stat = test_file(local)

		sep := "-"
		if stat != 0 {
			if stat == 1 {
				sep = "o"
			} else {
				sep = "z"
			}
		}
		fp.WriteString(fmt.Sprintf(" %s", sep))

		ShowMsg_Ptr("STAT=%s", sep)

		(*nt)++
		if stat > 0 {
			(*nc)++
		}
	}
	fp.WriteString("\n")
	return abort
}

/* test local files ----------------------------------------------------------*/
func test_locals(ts, te Gtime, ti float64, url *Url, stas []string, nsta int, dir string, nc, nt []int, fp *os.File) int {
	if strings.Contains(url.path, "%s") || strings.Contains(url.path, "%S") {
		fp.WriteString(fmt.Sprintf("%s\n", url.dtype))
		for i := 0; i < nsta; i++ {
			fp.WriteString(fmt.Sprintf("%-12s:", stas[i]))
			if len(dir) == 0 {
				dir = url.dir
			}
			if test_local(ts, te, ti, url.path, stas[i], dir, &nc[i], &nt[i], fp) > 0 {
				return 1
			}
		}
	} else {
		fp.WriteString(fmt.Sprintf("%-12s:", url.dtype))
		if len(dir) == 0 {
			dir = url.dir
		}
		if test_local(ts, te, ti, url.path, "", dir, &nc[0], &nt[0], fp) > 0 {
			return 1
		}
	}
	return 0
}

/* print total count of local files ------------------------------------------*/
func print_total(url *Url, stas []string, nsta int, nc, nt []int, fp *os.File) int {
	if strings.Contains(url.path, "%s") || strings.Contains(url.path, "%S") {
		fp.WriteString(fmt.Sprintf("%s\n", url.dtype))
		for i := 0; i < nsta; i++ {
			fp.WriteString(fmt.Sprintf("%-12s: %5d/%5d\n", stas[i], nc[i], nt[i]))
		}
		return nsta
	}
	fp.WriteString(fmt.Sprintf("%-12s: %5d/%5d\n", url.dtype, nc[0], nt[0]))
	return 1
}

/* read URL list file ----------------------------------------------------------
* read URL list file for GNSS data
* args   : char   *file     I   URL list file
*          char   **types   I   selected types ("*":wildcard)
*          int    ntype     I   number of selected types
*          urls_t *urls     O   URL list
*          int    nmax      I   max number of URL list
* return : number of URL addresses (0:error)
* notes  :
*    (1) URL list file contains records containing the following fields
*        separated by spaces. if a field contains spaces, enclose it within "".
*
*        data_type  url_address       default_local_directory
*
*    (2) strings after # in a line are treated as comments
*    (3) url_address should be:
*
*        ftp://host_address/file_path or
*        ftps://host_address/file_path or
*        http://host_address/file_path or
*        https://host_address/file_path
*
*    (4) the field url_address or default_local_directory can include the
*        follwing keywords replaced by date, time, station names and environment
*        variables.
*
*        %Y . yyyy    : year (4 digits) (2000-2099)
*        %y . yy      : year (2 digits) (00-99)
*        %m . mm      : month           (01-12)
*        %d . dd      : day of month    (01-31)
*        %h . hh      : hours           (00-23)
*        %H . a       : hour code       (a-x)
*        %M . mm      : minutes         (00-59)
*        %n . ddd     : day of year     (001-366)
*        %W . wwww    : gps week        (0001-9999)
*        %D . d       : day of gps week (0-6)
*        %N . nnn     : general number
*        %s . ssss    : station name    (lower-case)
*        %S . SSSS    : station name    (upper-case)
*        %r . rrrr    : station name
*        %{env} . env : environment variable
*-----------------------------------------------------------------------------*/
func DL_ReadUrls(file string, types []string, ntype int, urls []Url, nmax int) int {
	var (
		fp                     *os.File
		buff, stype, path, dir string
		i, n, idx              int
		err                    error
	)

	if fp, err = os.OpenFile(file, os.O_RDONLY, 0666); err != nil {
		Trace(1, "options file read error %s\n", file)
		return 0
	}
	defer fp.Close()

	for i = 0; i < ntype; i++ {
		fp.Seek(0, io.SeekStart)
		rd := bufio.NewReader(fp)
		for {
			buff, err = rd.ReadString('\n')
			if err != nil {
				break
			}
			if idx = strings.Index(buff, "#"); idx >= 0 {
				buff = buff[:idx]
			}
			ss := strings.Fields(buff)
			if len(ss) > 2 {
				dir = ss[2]
			}
			if len(ss) > 1 {
				path = ss[1]
			}
			if len(ss) == 1 {
				stype = ss[0]
			}
			if len(ss) < 1 {
				continue
			}
			if cmp_str(stype, types[i]) == 0 {
				continue
			}
			urls[n].dtype = stype
			urls[n].path = path
			urls[n].dir = dir
			n++
		}
	}

	if n <= 0 {
		Trace(1, "no url in options file %s\n", file)
		return 0
	}
	return n
}

/* read station list file ------------------------------------------------------
* read station list file
* args   : char   *file     I   station list file
*          char   **stas    O   stations
*          int    nmax      I   max number of stations
* return : number of stations (0:error)
* notes  :
*    (1) station list file contains station names separated by spaces.
*    (2) strings after # in a line are treated as comments
*-----------------------------------------------------------------------------*/
func DL_ReadStas(file string, stas []string, nmax int) int {
	var (
		fp     *os.File
		buff   string
		n, idx int
		err    error
	)

	if fp, err = os.OpenFile(file, os.O_RDONLY, 0666); err != nil {
		Trace(1, "station list file read error %s\n", file)
		return 0
	}
	defer fp.Close()

	rd := bufio.NewReader(fp)
	for {
		buff, err = rd.ReadString('\n')
		if err != nil {
			break
		}
		if n >= nmax {
			break
		}

		if idx = strings.Index(buff, "#"); idx >= 0 {
			buff = buff[:idx]
		}
		ss := strings.FieldsFunc(buff, func(r rune) bool {
			return strings.Index(" \r\n", string(r)) == 0
		})
		for _, v := range ss {
			if n > nmax {
				break
			}
			stas[n] = v
			n++
		}
	}

	if n <= 0 {
		Trace(2, "no station in station file %s\n", file)
		return 0
	}
	return n
}

/* execute download ------------------------------------------------------------
* execute download
* args   : gtime_t ts,te    I   time start and end
*          double tint      I   time interval (s)
*          int    seqnos    I   sequence number start
*          int    seqnoe    I   sequence number end
*          url_t  *urls     I   URL list
*          int    nurl      I   number of URL list
*          char   **stas    I   station list
*          int    nsta      I   number of station list
*          char   *dir      I   local directory
*          char   *remote_p I   previous remote file path
*          char   *usr      I   login user for FTP or FTPS
*          char   *pwd      I   login password for FTP or FTPS
*          char   *proxy    I   proxy server address
*          int    opts      I   download options (or of the followings)
*                                 DLOPT_FORCE = force download existing file
*                                 DLOPT_KEEPCMP=keep compressed file
*                                 DLOPT_HOLDERR=hold on error file
*                                 DLOPT_HOLDLST=hold on listing file
*          char   *msg      O   output messages
*          FILE   *fp       IO  log file pointer (NULL: no output log)
* return : status (1:ok,0:error,-1:aborted)
* notes  : The URL list should be read by using dl_readurl()
*          In the FTP or FTPS cases, the file name in a URL can contain wild-
*          cards (*). The directory in a URL can not contain any wild-cards.
*          If the file name contains wild-cards, DL_Exec() gets a file-list in
*          the remote directory and downloads the firstly matched file in the
*          remote file-list. The secondary matched or the following files are
*          not downloaded.
*-----------------------------------------------------------------------------*/
func DL_Exec(ts, te Gtime, ti float64, seqnos, seqnoe int,
	urls []Url, nurl int, stas []string, nsta int,
	dir, usr, pwd, proxy string, opts int, msg *string, fp *os.File) int {
	var (
		paths        Paths
		ts_p         Gtime
		str, remot_p string
		i            int
		n            [4]int
		tick         uint32 = uint32(TickGet())
	)

	ShowMsg_Ptr("STAT=_")

	/* generate download paths  */
	for TimeDiff(ts, te) < 1e-3 {

		for i = 0; i < nurl; i++ {
			if gen_paths(ts, ts_p, seqnos, seqnoe, &urls[i], stas, nsta, dir, &paths) == 0 {
				free_path(&paths)
				*msg = "too many download files"
				return 0
			}
		}
		ts_p = ts
		ts = TimeAdd(ts, ti)
	}
	/* compact download paths */
	compact_paths(&paths)

	if paths.n <= 0 {
		*msg = "no download data"
		return 0
	}
	for i = 0; i < paths.n; i++ {

		str = fmt.Sprintf("%s.%s (%d/%d)", paths.path[i].remot, paths.path[i].local, i+1,
			paths.n)
		if ShowMsg_Ptr(str) > 0 {
			break
		}

		/* execute download */
		if exec_down(&paths.path[i], &remot_p, usr, pwd, proxy, opts, n[:], fp) > 0 {
			break
		}
	}
	if (opts & DLOPT_HOLDLST) == 0 {
		os.Remove(FTP_LISTING)
	}
	*msg = fmt.Sprintf("OK=%d No_File=%d Skip=%d Error=%d (Time=%.1f s)", n[0], n[1], n[2],
		n[3], float64(uint32(TickGet())-tick)*0.001)

	free_path(&paths)

	return 1
}

/* execute local file test -----------------------------------------------------
* execute local file test
* args   : gtime_t ts,te    I   time start and end
*          double tint      I   time interval (s)
*          url_t  *urls     I   remote URL addresses
*          int    nurl      I   number of remote URL addresses
*          char   **stas    I   stations
*          int    nsta      I   number of stations
*          char   *dir      I   local directory
*          int    ncol      I   number of column
*          int    datefmt   I   date format (0:year-dow,1:year-dd/mm,2:week)
*          FILE   *fp       IO  log test result file pointer
* return : status (1:ok,0:error,-1:aborted)
*-----------------------------------------------------------------------------*/
func DL_Test(ts, te Gtime, ti float64, urls []Url,
	nurl int, stas []string, nsta int, dir string,
	ncol, datefmt int, fp *os.File) {
	var (
		time                          Gtime
		year, date, date_p            string
		i, j, n, m, week, flag, abort int
		nc, nt                        []int
	)

	if ncol < 1 {
		ncol = 1
	} else if ncol > 200 {
		ncol = 200
	}

	if len(dir) == 0 {
		dir = "*"
	}
	fp.WriteString(fmt.Sprintf("** LOCAL DATA AVAILABILITY (%s, %s) **\n\n",
		TimeStr(TimeGet(), 0), dir))

	for i, n = 0, 0; i < nurl; i++ {
		if strings.EqualFold(urls[i].path, "%s") {
			n += nsta
		} else {
			n = 1
		}
	}
	nc = IMat(n, 1)
	nt = IMat(n, 1)
	for i = 0; i < n; i++ {
		nc[i], nt[i] = 0, 0
	}

	for ; TimeDiff(ts, te) < 1e-3 && abort == 0; ts = TimeAdd(ts, ti*float64(ncol)) {
		if datefmt == 0 {
			GenPath("   %Y-", "", ts, 0, &year)
		} else {
			GenPath("%Y/%m/", "", ts, 0, &year)
		}
		if datefmt <= 1 {
			s := "DATE"
			if datefmt == 0 {
				s = "DOY "
			}
			fp.WriteString(fmt.Sprintf("%s %s", s, year))
		} else {
			fp.WriteString("WEEK          ")
		}
		date_p = ""
		flag = 0

		m = 2
		if datefmt == 2 {
			m = 1
		}
		for i = 0; i < (ncol+m-1)/m; i++ {
			time = TimeAdd(ts, ti*float64(i*m))
			if TimeDiff(time, te) >= 1e-3 {
				break
			}

			if datefmt <= 1 {
				if datefmt == 0 {
					GenPath("%n", "", time, 0, &date)
				} else {
					GenPath("%d", "", time, 0, &date)
				}
				if strings.Compare(date, date_p) != 0 {
					fp.WriteString(fmt.Sprintf("%-4s", date))
				} else {
					fp.WriteString(fmt.Sprintf("%-4s", ""))
				}
			} else {
				if math.Abs(Time2GpsT(time, &week)) < 1.0 {
					fp.WriteString(fmt.Sprintf("%04d", week))
					flag = 1
				} else {
					if flag > 0 {
						fp.WriteString("")
					} else {
						fp.WriteString("  ")
					}
					flag = 0
				}
			}
			date_p = date
		}
		fp.WriteString("\n")

		for i, j = 0, 0; i < nurl && abort == 0; i++ {
			time = TimeAdd(ts, ti*float64(ncol)-1.0)
			if TimeDiff(time, te) >= 0.0 {
				time = te
			}

			/* test local files */
			abort = test_locals(ts, time, ti, &urls[i], stas, nsta, dir, nc[j:], nt[j:], fp)
			if strings.Contains(urls[i].path, "%s") || strings.Contains(urls[i].path, "%S") {
				j += nsta
			} else {
				j += 1
			}
		}
		fp.WriteString("\n")
	}
	fp.WriteString("# COUNT     : FILES/TOTAL\n")

	for i, j = 0, 0; i < nurl; i++ {
		j += print_total(&urls[i], stas, nsta, nc[j:], nt[j:], fp)
	}

}
