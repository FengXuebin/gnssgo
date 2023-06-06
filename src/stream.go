/*------------------------------------------------------------------------------
* stream.c : stream input/output functions
*
*          Copyright (C) 2008-2020 by T.TAKASU, All rights reserved.
*
* options : -DWIN32    use WIN32 API
*           -DSVR_REUSEADDR reuse tcp server address
*
* references :
*     [1] RTCM Recommendaed Standards for Networked Transport for RTCM via
*         Internet Protocol (Ntrip), Version 1.0, Semptember 30, 2004
*     [2] H.Niksic and others, GNU Wget 1.12, The non-iteractive download
*         utility, 4 September 2009
*     [3] RTCM Recommendaed Standards for Networked Transport for RTCM via
*         Internet Protocol (Ntrip), Version 2.0, June 28, 2011
*
* version : $Revision:$ $Date:$
* history : 2009/01/16 1.0  new
*           2009/04/02 1.1  support nmea request in ntrip request
*                           support time-tag of file as stream
*           2009/09/04 1.2  ported to linux environment
*                           add fflush() to save file stream
*           2009/10/10 1.3  support multiple connection for tcp server
*                           add keyword replacement in file path
*                           add function strsendnmea(), strsendcmd()
*           2010/07/18 1.4  support ftp/http stream types
*                           add keywords replacement of %ha,%hb,%hc in path
*                           add api: strsetdir(),strsettimeout()
*           2010/08/31 1.5  reconnect after error of ntrip client
*                           fix bug on no file swap at week start (2.4.0_p6)
*           2011/05/29 1.6  add fast stream replay mode
*                           add time margin to swap file
*                           change api strsetopt()
*                           introduce non_block send for send socket
*                           add api: strsetproxy()
*           2011/12/21 1.7  fix bug decode tcppath (rtklib_2.4.1_p5)
*           2012/06/09 1.8  fix problem if user or password contains /
*                           (rtklib_2.4.1_p7)
*           2012/12/25 1.9  compile option SVR_REUSEADDR added
*           2013/03/10 1.10 fix problem with ntrip mountpoint containing "/"
*           2013/04/15 1.11 fix bug on swapping files if swapmargin=0
*           2013/05/28 1.12 fix bug on playback of file with 64 bit size_t
*           2014/05/23 1.13 retry to connect after gethostbyname() error
*                           fix bug on malloc size in openftp()
*           2014/06/21 1.14 add general hex message rcv command by !HEX ...
*           2014/10/16 1.15 support stdin/stdout for input/output from/to file
*           2014/11/08 1.16 fix getconfig error (87) with bluetooth device
*           2015/01/12 1.15 add rcv command to change bitrate by !BRATE
*           2016/01/16 1.16 add constant CRTSCTS for non-CRTSCTS-defined env.
*                           fix serial status for non-windows systems
*           2016/06/09 1.17 fix bug on !BRATE rcv command always failed
*                           fix program on struct alignment in time tag header
*           2016/06/21 1.18 reverse time-tag handler of file to previous
*           2016/07/23 1.19 add output of received stream to tcp port for serial
*           2016/08/20 1.20 modify api strsendnmea()
*           2016/08/29 1.21 fix bug on starting serial thread for windows
*           2016/09/03 1.22 add ntrip caster functions
*                           add api strstatx(),strsetsrctbl()
*                           add api strsetsel(),strgetsel()
*           2016/09/06 1.23 fix bug on ntrip caster socket and request handling
*           2016/09/27 1.24 support udp server and client
*           2016/10/10 1.25 support ::P={4|8} option in path for STR_FILE
*           2018/11/05 1.26 fix bug on default playback speed (= 0)
*                           fix bug on file playback as slave mode
*                           fix bug on timeset() in gpst instead of utc
*                           update trace levels and buffer sizes
*           2019/05/10 1.27 fix bug on dropping message on tcp stream (#144)
*           2019/08/19 1.28 support 460800 and 921600 bps for serial
*           2020/11/30 1.29 delete API strsetsrctbl(), strsetsel(), strgetsel()
*                           fix bug on numerical error in computing output rate
*                           no support stream type STR_NTRIPC_S in API stropen()
*                           no support rcv. command LEXR in API strsendcmd()
*                           change stream type STR_NTRIPC_C to STR_NTRIPCAS
*                           accept HTTP/1.1 as protocol for NTRIP caster
*                           suppress warning for buffer overflow by sprintf()
*                           use integer types in stdint.h
*		    2022/05/31 1.0  rewrite stream.c with golang by fxb
*-----------------------------------------------------------------------------*/
package gnssgo

import (
	"encoding/binary"
	"fmt"

	//	"gnssreceiver"
	"io"
	"math"
	"net"
	"os"
	"sort"
	"strconv"
	"strings"
	"sync"
	"time"
	"unicode"

	//"goserialgo"
	serial "github.com/tarm/goserial"
	//	serial "github.com/tarm/goserial"go
)

/* constants -----------------------------------------------------------------*/
const (
	CRTSCTS             = "020000000000"
	TINTACT             = 200  /* period for stream active (ms) */
	SERIBUFFSIZE        = 4096 /* serial buffer size (bytes) */
	TIMETAGH_LEN        = 64   /* time tag file header length */
	MAXCLI              = 32   /* max client connection for tcp svr */
	MAXSTATMSG          = 32   /* max length of status message */
	DEFAULT_MEMBUF_SIZE = 4096 /* default memory buffer size (bytes) */
	NTRIP_AGENT         = "RTKLIB/" + VER_GNSSGO
	NTRIP_CLI_PORT      = 2101                     /* default ntrip-client connection port */
	NTRIP_SVR_PORT      = 80                       /* default ntrip-server connection port */
	NTRIP_MAXRSP        = 32768                    /* max size of ntrip response */
	NTRIP_MAXSTR        = 256                      /* max length of mountpoint string */
	NTRIP_RSP_OK_CLI    = "ICY 200 OK\r\n"         /* ntrip response: client */
	NTRIP_RSP_OK_SVR    = "OK\r\n"                 /* ntrip response: server */
	NTRIP_RSP_SRCTBL    = "SOURCETABLE 200 OK\r\n" /* ntrip response: source table */
	NTRIP_RSP_TBLEND    = "ENDSOURCETABLE"
	NTRIP_RSP_HTTP      = "HTTP/" /* ntrip response: http */
	NTRIP_RSP_ERROR     = "ERROR" /* ntrip response: error */
	NTRIP_RSP_UNAUTH    = "HTTP/1.0 401 Unauthorized\r\n"
	NTRIP_RSP_ERR_PWD   = "ERROR - Bad Pasword\r\n"
	NTRIP_RSP_ERR_MNTP  = "ERROR - Bad Mountpoint\r\n"
)

var errno error = nil

type Dev int

/* type definition -----------------------------------------------------------*/

type FileType struct { /* file control type */
	fp         *os.File   /* file pointer */
	fp_tag     *os.File   /* file pointer of tag file */
	fp_tmp     *os.File   /* temporary file pointer for swap */
	fp_tag_tmp *os.File   /* temporary file pointer of tag file for swap */
	path       string     /* file path */
	openpath   string     /* open file path */
	mode       int        /* file mode */
	timetag    int        /* time tag flag (0:off,1:on) */
	repmode    int        /* replay mode (0:master,1:slave) */
	offset     int        /* time offset (ms) for slave */
	size_fpos  int        /* file position size (bytes) */
	time       Gtime      /* start time */
	wtime      Gtime      /* write time */
	tick       uint32     /* start tick */
	tick_f     uint32     /* start tick in file */
	fpos_n     int64      /* next file position */
	tick_n     uint32     /* next tick */
	start      float64    /* start offset (s) */
	speed      float64    /* replay speed (time factor) */
	swapintv   float64    /* swap interval (hr) (0: no swap) */
	lock       sync.Mutex /* lock flag */
}

type TcpConn struct { /* tcp control type */
	state int      /* state (0:close,1:wait,2:connect) */
	saddr string   /* address string */
	port  int      /* port */
	addr  net.Addr /* address resolved */
	sock  net.Conn /* socket descriptor */
	tcon  int      /* reconnect time (ms) (-1:never,0:now) */
	tact  int64    /* data active tick */
	tdis  int64    /* disconnect tick */
}

func (conn *TcpConn) ResolveAddr() string {
	return fmt.Sprintf("%s:%d", conn.saddr, conn.port)
}

type TcpSvr struct { /* tcp server type */
	svr TcpConn         /* tcp server control */
	cli [MAXCLI]TcpConn /* tcp client controls */
}

type TcpClient struct { /* tcp cilent type */
	svr     TcpConn /* tcp server control */
	toinact int     /* inactive timeout (ms) (0:no timeout) */
	tirecon int     /* reconnect interval (ms) (0:no reconnect) */
}

type SerialComm struct { /* serial control type */
	dev      Dev /* serial device */
	serialio io.ReadWriteCloser
	err      int /* error state */
	//	lock     sync.Mutex /* lock flag */
	tcpsvr *TcpSvr /* tcp server for received stream */
}

type NTrip struct { /* ntrip control type */
	state  int        /* state (0:close,1:wait,2:connect) */
	ctype  int        /* type (0:server,1:client) */
	nb     int        /* response buffer size */
	url    string     /* url for proxy */
	mntpnt string     /* mountpoint */
	user   string     /* user */
	passwd string     /* password */
	str    string     /* mountpoint string for server */
	buff   string     /* response buffer */
	tcp    *TcpClient /* tcp client */
}

type NTripc_con struct { /* ntrip client/server connection type */
	state  int    /* state (0:close,1:connect) */
	mntpnt string /* mountpoint */
	//	str    string /* mountpoint string for server */
	nb   int    /* request buffer size */
	buff string /* request buffer */
}

type NTripc struct { /* ntrip caster control type */
	state  int          /* state (0:close,1:wait,2:connect) */
	ctype  int          /* type (0:server,1:client) */
	mntpnt string       /* mountpoint */
	user   string       /* user */
	passwd string       /* password */
	srctbl string       /* source table */
	tcp    *TcpSvr      /* tcp server */
	con    []NTripc_con /* ntrip client/server connections */
}

type UdpConn struct { /* udp type */
	state int    /* state (0:close,1:open) */
	ctype int    /* type (0:server,1:client) */
	port  int    /* port */
	saddr string /* address (server:filter,client:server) */
	//	addr  *net.UDPAddr /* address resolved */
	sock net.Conn /* socket descriptor */
}

type FtpConn struct { /* ftp download control type */
	state int /* state (0:close,1:download,2:complete,3:error) */
	proto int /* protocol (0:ftp,1:http) */
	error int /* error code (0:no error,1-10:wget error, */
	/*            11:no temp dir,12:uncompact error) */
	addr   string /* download address */
	file   string /* download file path */
	user   string /* user for ftp */
	passwd string /* password for ftp */
	local  string /* local file path */
	topts  [4]int /* time options {poff,tint,toff,tretry} (s) */
	tnext  Gtime  /* next retry time (gpst) */
	thread int    /* download thread */
}

type MemBuf struct { /* memory buffer type */
	state, wp, rp int        /* state,write/read pointer */
	bufsize       int        /* buffer size (bytes) */
	lock          sync.Mutex /* lock flag */
	buf           []byte     /* write buffer */
}

/* proto types for static functions ------------------------------------------*/

/* global options ------------------------------------------------------------*/

var (
	toinact   int = 10000 /* inactive timeout (ms) */
	ticonnect int = 1000  /* interval to re-connect (ms) */
	tirate    int = 1000  /* averaging time for data rate (ms) */
	//var buffsize int = 32768   /* receive/send buffer size (bytes) */
	localdir    string = "" /* local directory for ftp/http */
	proxyaddr   string = "" /* http/ntrip/ftp proxy address */
	tick_master uint32 = 0  /* time tick master for replay */
	fswapmargin int    = 30 /* file swap margin (s) */)

/* read/write serial buffer --------------------------------------------------*/

/* open serial ---------------------------------------------------------------*/
func OpenSerial(path string, mode int, msg *string) *SerialComm {
	var (
		br []int = []int{
			300, 600, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800,
			921600}
		seri                             *SerialComm = new(SerialComm)
		i, brate, bsize, stopb, tcp_port int         = 0, 9600, 8, 1, 0
		parity                           rune        = 'N'
		port, fctr, path_tcp, msg_tcp    string
	)
	index := strings.Index(path, ":")
	if index > 0 {
		port = path[:index]
		fmt.Sscanf(path[index:], ":%d:%d:%c:%d:%s", &brate, &bsize, &parity, &stopb, fctr)
	} else {
		port = path
	}

	index = strings.Index(path, "#")
	if index >= 0 {
		fmt.Sscanf(path[index:], "#%d", &tcp_port)
	}
	i = sort.SearchInts(br, brate)

	if i >= 14 {
		*msg = fmt.Sprintf("bitrate error (%d)", brate)
		Tracet(1, "openserial: %s path=%s\n", *msg, path)
		return nil
	}
	parity = unicode.ToUpper(parity)

	c := &serial.Config{Name: port, Baud: brate}
	s, err := serial.OpenPort(c)
	seri.serialio = s
	if err != nil {
		seri.err = 1
	} else {
		seri.err = 0
	}
	seri.tcpsvr = nil

	/* open tcp sever to output received stream */
	if tcp_port > 0 {
		path_tcp = fmt.Sprintf(":%d", tcp_port)
		seri.tcpsvr = OpenTcpSvr(path_tcp, &msg_tcp)
	}
	Tracet(3, "openserial: dev=%d\n", seri.dev)
	return seri
}

/* close serial --------------------------------------------------------------*/
func (seri *SerialComm) CloseSerial() {
	if seri == nil {
		return
	}
	seri.serialio.Close()
	if seri.tcpsvr != nil {
		seri.tcpsvr.CloseTcpSvr()
	}
}

/* read serial ---------------------------------------------------------------*/
func (seri *SerialComm) ReadSerial(buff []byte, n int, msg *string) int {
	var msg_tcp string
	Tracet(4, "readserial: dev= n=%d\n", n)
	if seri == nil || seri.serialio == nil {
		return 0
	}
	nr, err := seri.serialio.Read(buff)
	if err != nil {
		seri.err = 1
	} else {
		seri.err = 0
	}
	Tracet(5, "readserial: exit dev=%d nr=%d\n", seri.dev, nr)

	/* write received stream to tcp server port */
	if seri.tcpsvr != nil && nr > 0 {
		seri.tcpsvr.WriteTcpSvr(buff, nr, &msg_tcp)
	}
	return nr
}

/* write serial --------------------------------------------------------------*/
func (seri *SerialComm) WriteSerial(buff []uint8, n int, msg *string) int {

	Tracet(3, "writeserial: dev=,n=%d\n", n)

	if seri == nil {
		return 0
	}
	if n <= 0 {
		return 0
	}
	ns, err := seri.serialio.Write(buff)
	if err != nil {
		seri.err = 1
	} else {
		seri.err = 0
	}
	Tracet(5, "writeserial: exit dev=%d ns=%d\n", seri.dev, ns)
	return ns
}

/* get state serial ----------------------------------------------------------*/
func (seri *SerialComm) StateSerial() int {
	if seri == nil {
		return 0
	} else if seri.err != 0 {
		return -1
	}
	return 2
}

/* get extended state serial -------------------------------------------------*/
func (seri *SerialComm) StatExSerial(msg *string) int {
	//    char *p=msg;
	state := seri.StateSerial()

	*msg += "serial:\n"
	*msg += fmt.Sprintf("  state   = %d\n", state)
	if state == 0 {
		return 0
	}
	*msg += fmt.Sprintf("  dev     = %d\n", seri.dev)
	*msg += fmt.Sprintf("  error   = %d\n", seri.err)
	return state
}

/* open file -----------------------------------------------------------------*/
func openfile_(file *FileType, time Gtime, msg *string) int {
	var (
		fp        *os.File
		time_sec  float64
		time_time uint32
		tagpath   string
		tagh      []byte = make([]byte, TIMETAGH_LEN)
		rw        int
		err       error
	)

	Tracet(3, "openfile_: path=%s time=%s\n", file.path, TimeStr(time, 0))

	file.time = Utc2GpsT(TimeGet())
	file.tick = uint32(TickGet())
	file.tick_f = file.tick
	file.fpos_n = 0
	file.tick_n = 0

	/* use stdin or stdout if file path is null */
	if len(file.path) == 0 {
		if file.mode&STR_MODE_R > 0 {
			file.fp = os.Stdin
		} else {
			file.fp = os.Stdout
		}

		return 1
	}
	/* replace keywords */
	RepPath(file.path, &file.openpath, time, "", "")

	/* create directory */
	if (file.mode&STR_MODE_W > 0) && (file.mode&STR_MODE_R == 0) {
		CreateDir(file.openpath)
	}
	if file.mode&STR_MODE_R > 0 {
		rw = os.O_RDONLY
	} else {
		rw = os.O_CREATE | os.O_WRONLY | os.O_TRUNC
	}
	file.fp, err = os.OpenFile(file.openpath, rw, os.ModeAppend|os.ModePerm)
	if err != nil {
		*msg = fmt.Sprintf("file open error: %s", file.openpath)
		Tracet(1, "openfile: %s\n", *msg)
		return 0
	}
	Tracet(4, "openfile_: open file %s (%d)\n", file.openpath, rw)

	tagpath = fmt.Sprintf("%s.tag", file.openpath)

	if file.timetag > 0 { /* output/sync time-tag */
		file.fp_tag, err = os.OpenFile(tagpath, rw, 0666)
		if err != nil {
			*msg = fmt.Sprintf("tag open error: %s", tagpath)
			Tracet(1, "openfile: %s\n", *msg)
			file.fp.Close()
			return 0
		}
		Tracet(4, "openfile_: open tag file %s (%d)\n", tagpath, rw)

		if file.mode&STR_MODE_R > 0 {
			if binary.Read(file.fp_tag, binary.BigEndian, tagh) == nil &&
				binary.Read(file.fp_tag, binary.BigEndian, &time_time) == nil &&
				binary.Read(file.fp_tag, binary.BigEndian, &time_sec) == nil {
				file.time.Time = uint64(time_time)
				file.time.Sec = time_sec
				file.wtime = file.time
				file.tick_f = binary.BigEndian.Uint32(tagh[TIMETAGH_LEN-4:])
			} else {
				file.tick_f = 0
			}
			/* adust time to read playback file */
			TimeSet(GpsT2Utc(file.time))
		} else {
			tagh = []byte(fmt.Sprintf("TIMETAG RTKLIB %s", VER_GNSSGO))
			binary.BigEndian.PutUint32(tagh[TIMETAGH_LEN-4:], file.tick_f)
			//memcpy(tagh+TIMETAGH_LEN-4,&file.tick_f,sizeof(file.tick_f));
			time_time = uint32(file.time.Time)
			time_sec = file.time.Sec
			binary.Write(file.fp_tag, binary.BigEndian, tagh)
			binary.Write(file.fp_tag, binary.BigEndian, time_time)
			binary.Write(file.fp_tag, binary.BigEndian, time_sec)
			/* time tag file structure   */
			/*   HEADER(60)+TICK(4)+TIME(4+8)+ */
			/*   TICK0(4)+FPOS0(4/8)+    */
			/*   TICK1(4)+FPOS1(4/8)+... */
		}
	} else if file.mode&STR_MODE_W > 0 { /* remove time-tag */
		if fp, err = os.Open(tagpath); err == nil {
			fp.Close()
			os.Remove(tagpath)
		}
	}
	return 1
}

/* close file ----------------------------------------------------------------*/
func closefile_(file *FileType) {
	Tracet(3, "closefile_: path=%s\n", file.path)

	if file.fp != nil {
		file.fp.Close()
	}
	if file.fp_tag != nil {
		file.fp_tag.Close()
	}
	if file.fp_tmp != nil {
		file.fp_tmp.Close()
	}
	if file.fp_tag_tmp != nil {
		file.fp_tag_tmp.Close()
	}

	file.fp, file.fp_tag, file.fp_tmp, file.fp_tag_tmp = nil, nil, nil, nil

	/* reset time offset */
	TimeReset()
}

/* open file (path=filepath[::T[::+<off>][::x<speed>]][::S=swapintv][::P={4|8}] */
func OpenStreamFile(path string, mode int, msg *string) *FileType {
	var (
		file                   *FileType = new(FileType)
		time, time0            Gtime
		speed, start, swapintv float64 = 1.0, 0.0, 0.0
		timetag, size_fpos     int     = 0, 4 /* default 4B */)

	Tracet(3, "openfile: path=%s mode=%d\n", path, mode)

	if mode&(STR_MODE_R|STR_MODE_W) == 0 {
		return nil
	}

	/* file options */
	p := path
	for idx := strings.Index(path, "::"); idx >= 0; idx = strings.Index(p, "::") { /* file options */
		p = p[idx+2:]
		switch p[0] {
		case 'T':
			timetag = 1
		case '+':
			fmt.Sscanf(p, "+%f", &start)
		case 'x':
			fmt.Sscanf(p, "x%f", &speed)
		case 'S':
			fmt.Sscanf(p, "S=%f", &swapintv)
		case 'P':
			fmt.Sscanf(p, "P=%d", &size_fpos)
		}
	}

	if start <= 0.0 {
		start = 0.0
	}
	if swapintv <= 0.0 {
		swapintv = 0.0
	}

	file.fp, file.fp_tag, file.fp_tmp, file.fp_tag_tmp = nil, nil, nil, nil
	file.path = path
	if idx := strings.Index(path, "::"); idx > 0 {
		file.path = path[:idx]
	}

	file.openpath = ""
	file.mode = mode
	file.timetag = timetag
	file.repmode = 0
	file.offset = 0
	file.size_fpos = size_fpos
	file.time, file.wtime = time0, time0
	file.tick, file.tick_f, file.tick_n, file.fpos_n = 0, 0, 0, 0
	file.start = start
	file.speed = speed
	file.swapintv = swapintv

	time = Utc2GpsT(TimeGet())

	/* open new file */
	if openfile_(file, time, msg) == 0 {
		file = nil
		return nil
	}
	return file
}

/* close file ----------------------------------------------------------------*/
func (file *FileType) CloseStreamFile() {
	if file == nil {
		return
	}
	closefile_(file)
	file = nil
}

/* open new swap file --------------------------------------------------------*/
func (file *FileType) SwapStreamFile(time Gtime, msg *string) {
	var openpath string

	Tracet(3, "swapfile: fp=%d time=%s\n", file.fp.Fd(), TimeStr(time, 0))

	/* return if old swap file open */
	if file.fp_tmp != nil || file.fp_tag_tmp != nil {
		return
	}

	/* check path of new swap file */
	RepPath(file.path, &openpath, time, "", "")

	if strings.Compare(openpath, file.openpath) == 0 {
		Tracet(2, "swapfile: no need to swap %s\n", openpath)
		return
	}
	/* save file pointer to temporary pointer */
	file.fp_tmp = file.fp
	file.fp_tag_tmp = file.fp_tag

	/* open new swap file */
	openfile_(file, time, msg)
}

/* close old swap file -------------------------------------------------------*/
func (file *FileType) CloseSwapFile() {
	Tracet(3, "swapclose: fp_tmp=%d\n", file.fp_tmp.Fd())

	if file.fp_tmp != nil {
		file.fp_tmp.Close()
	}
	if file.fp_tag_tmp != nil {
		file.fp_tag_tmp.Close()
	}
	file.fp_tmp, file.fp_tag_tmp = nil, nil
}

/* get state file ------------------------------------------------------------*/
func (file *FileType) StateFile() int {
	if file != nil {
		return 2
	}
	return 0
}

/* get extended state file ---------------------------------------------------*/
func (file *FileType) StatExFile(msg *string) int {
	var tstr1, tstr2 string
	state := file.StateFile()

	*msg += "file:\n"
	*msg += fmt.Sprintf("  state   = %d\n", state)
	if state == 0 {
		return 0
	}
	Time2Str(file.time, &tstr1, 3)
	Time2Str(file.wtime, &tstr2, 3)
	*msg += fmt.Sprintf("  path    = %s\n", file.path)
	*msg += fmt.Sprintf("  openpath= %s\n", file.openpath)
	*msg += fmt.Sprintf("  mode    = %d\n", file.mode)
	*msg += fmt.Sprintf("  timetag = %d\n", file.timetag)
	*msg += fmt.Sprintf("  repmode = %d\n", file.repmode)
	*msg += fmt.Sprintf("  offsete = %d\n", file.offset)
	*msg += fmt.Sprintf("  time    = %s\n", tstr1)
	*msg += fmt.Sprintf("  wtime   = %s\n", tstr2)
	*msg += fmt.Sprintf("  tick    = %d\n", file.tick)
	*msg += fmt.Sprintf("  tick_f  = %d\n", file.tick_f)
	*msg += fmt.Sprintf("  start   = %.3f\n", file.start)
	*msg += fmt.Sprintf("  speed   = %.3f\n", file.speed)
	*msg += fmt.Sprintf("  swapintv= %.3f\n", file.swapintv)
	return state
}

/* read file -----------------------------------------------------------------*/
func (file *FileType) ReadFile(buff []uint8, nmax int64, msg *string) int {
	var (
		fpos_8B    uint64
		t, fpos_4B uint32
		pos, n     int64
		nr         int = 0
		err        error
	)

	Tracet(4, "readfile: fp= nmax=%d\n", nmax)

	if file == nil {
		return 0
	}

	if file.fp == os.Stdin {
		return 0
	}
	if file.fp_tag != nil {

		/* target tick */
		if file.repmode > 0 { /* slave */
			t = tick_master + uint32(file.offset)
		} else { /* master */
			t = uint32(float64(TickGet()-int64(file.tick))*file.speed + file.start*1000.0)
			tick_master = t
		}
		/* seek time-tag file to get next tick and file position */
		for int(file.tick_n-t) <= 0 {

			if binary.Read(file.fp_tag, binary.BigEndian, &file.tick_n) != nil {
				if file.size_fpos == 4 {
					err = binary.Read(file.fp_tag, binary.BigEndian, &fpos_4B)
				} else {
					err = binary.Read(file.fp_tag, binary.BigEndian, &fpos_8B)
				}

				if err != nil {
					file.tick_n = 0xffffffff
					pos, _ = file.fp.Seek(0, 1)
					file.fp.Seek(0, 2)
					file.fpos_n, _ = file.fp.Seek(0, 1)
					file.fp.Seek(pos, 0)
					break
				}
			}
			if file.size_fpos == 4 {
				file.fpos_n = int64(fpos_4B)
			} else {
				file.fpos_n = int64(fpos_8B)
			}
		}
		if file.tick_n == 0xffffffff {
			*msg = "end"
		} else {
			*msg = fmt.Sprintf("T%+.1fs", float32(t)*0.001)
			file.wtime = TimeAdd(file.time, float64(t)*0.001)
			TimeSet(TimeAdd(GpsT2Utc(file.time), float64(file.tick_n)*0.001))
		}
		pos, _ = file.fp.Seek(0, 1)
		n = file.fpos_n - pos
		if n < nmax {
			nmax = n
		}
	}
	if nmax > 0 {
		nr, err = file.fp.Read(buff)
	}
	if err == io.EOF {
		*msg = "end"
	}
	Tracet(5, "readfile: fp=%d nr=%d\n", file.fp.Fd(), nr)
	return nr
}

/* write file ----------------------------------------------------------------*/
func (file *FileType) WriteFile(buff []uint8, n int, msg *string) int {
	var (
		wtime            Gtime
		fpos_8B          uint64
		tick             int64 = TickGet()
		fpos_4B          uint32
		week1, week2, ns int
		tow1, tow2, intv float64
		fpos, fpos_tmp   int64
	)

	Tracet(4, "writefile: fp=n=%d\n", n)

	if file == nil {
		return 0
	}

	wtime = Utc2GpsT(TimeGet()) /* write time in gpst */

	/* swap writing file */
	if file.swapintv > 0.0 && file.wtime.Time != 0 {
		intv = file.swapintv * 3600.0
		tow1 = Time2GpsT(file.wtime, &week1)
		tow2 = Time2GpsT(wtime, &week2)
		tow2 += 604800.0 * float64(week2-week1)

		/* open new swap file */
		if math.Floor((tow1+float64(fswapmargin))/intv) < math.Floor((tow2+float64(fswapmargin))/intv) {
			file.SwapStreamFile(TimeAdd(wtime, float64(fswapmargin)), msg)
		}
		/* close old swap file */
		if math.Floor((tow1-float64(fswapmargin))/intv) < math.Floor((tow2-float64(fswapmargin))/intv) {
			file.CloseSwapFile()
		}
	}
	if file.fp == nil {
		return 0
	}

	ns, _ = file.fp.Write(buff[:n])

	fpos, _ = file.fp.Seek(0, 1)

	file.wtime = wtime

	if file.fp_tmp != nil {
		_, _ = file.fp_tmp.Write(buff[:n])

		fpos_tmp, _ = file.fp_tmp.Seek(0, 1)
	}
	if file.fp_tag != nil {
		tick -= int64(file.tick)
		binary.Write(file.fp_tag, binary.BigEndian, &tick)

		if file.size_fpos == 4 {
			fpos_4B = uint32(fpos)
			binary.Write(file.fp_tag, binary.BigEndian, &fpos_4B)
			// fwrite(&fpos_4B,1,sizeof(fpos_4B),file.fp_tag);
		} else {
			fpos_8B = uint64(fpos)
			binary.Write(file.fp_tag, binary.BigEndian, &fpos_8B)
		}

		if file.fp_tag_tmp != nil {
			binary.Write(file.fp_tag_tmp, binary.BigEndian, &tick)
			// fwrite(&tick,1,sizeof(tick),file.fp_tag_tmp);
			if file.size_fpos == 4 {
				fpos_4B = uint32(fpos_tmp)
				binary.Write(file.fp_tag_tmp, binary.BigEndian, &fpos_4B)
				//  fwrite(&fpos_4B,1,sizeof(fpos_4B),file.fp_tag_tmp);
			} else {
				fpos_8B = uint64(fpos_tmp)
				binary.Write(file.fp_tag_tmp, binary.BigEndian, &fpos_8B)
				//     fwrite(&fpos_8B,1,sizeof(fpos_8B),file.fp_tag_tmp);
			}
		}
	}
	Tracet(5, "writefile: fp=%d ns=%d tick=%5d fpos=%d\n", file.fp.Fd(), ns, tick, fpos)

	return ns
}

/* sync files by time-tag ----------------------------------------------------*/
func syncfile(file1, file2 *FileType) {
	if file1.fp_tag == nil || file2.fp_tag == nil {
		return
	}
	file1.repmode = 0
	file2.repmode = 1
	file2.offset = int(file1.tick_f - file2.tick_f)
}

/* decode tcp/ntrip path (path=[user[:passwd]@]addr[:port][/mntpnt[:str]]) ---*/
func DecodeTcpPath(path string, addr, port, user, passwd, mntpnt, str *string) {
	var (
		buff  string
		index int
	)
	Tracet(4, "decodetcpepath: path=%s\n", path)

	if port != nil {
		*port = ""
	}
	if user != nil {
		*user = ""
	}
	if passwd != nil {
		*passwd = ""
	}
	if mntpnt != nil {
		*mntpnt = ""
	}
	if str != nil {
		*str = ""
	}

	buff = path
	if index = strings.Index(buff, "@"); index >= 0 {
		// include username:password
		idx := strings.Index(buff[:index], ":")
		if idx >= 0 {
			q := buff[idx+1 : index]
			if passwd != nil {
				*passwd = fmt.Sprintf("%.255s", q)
			}
		} else {
			idx = index
		}
		if user != nil {
			*user = fmt.Sprintf("%.255s", buff[:idx])
		}
		buff = buff[index+1:]
	}

	if index = strings.Index(buff, "/"); index >= 0 {
		p := buff[index+1:]
		if idx := strings.Index(p, ":"); idx >= 0 {
			q := p[idx:]
			if str != nil {
				*str = fmt.Sprintf("%.*s", NTRIP_MAXSTR-1, q)
			}
			p = p[:idx]
		}
		if mntpnt != nil {
			*mntpnt = fmt.Sprintf("%.255s", p)
		}
		buff = buff[:index]
	}

	if index = strings.Index(buff, ":"); index >= 0 {
		if port != nil {
			*port = fmt.Sprintf("%.255s", buff[index+1:])
		}
		buff = buff[:index]
	}

	if addr != nil {
		*addr = fmt.Sprintf("%.255s", buff)
	}
}

func errsock() error { return errno }
func seterrsock(err error) {
	errno = err
}

/* set socket option ---------------------------------------------------------*/
func setsock(conn net.Conn, msg *string) int {

	return 1
}

/* non-block accept ----------------------------------------------------------*/
func Accept_nb(addr net.Addr) net.Conn {
	listerner, err := net.Listen(addr.Network(), addr.String())
	if err == nil {
		sock, err1 := listerner.Accept()
		seterrsock(err1)
		return sock
	}
	seterrsock(err)
	return nil
}

/* non-block connect ---------------------------------------------------------*/
func Connect_nb(sock net.Conn, addr net.Addr) int {
	return 1
}

/* non-block receive ---------------------------------------------------------*/
func Recv_nb(sock net.Conn, buff []byte, n int) int {
	var p [NTRIP_MAXSTR]byte
	nr, err := sock.Read(p[:])
	seterrsock(err)
	if nr <= 0 {
		return -1
	}
	copy(buff, p[:nr])
	return nr
}

/* non-block send ------------------------------------------------------------*/
func Send_nb(sock net.Conn, buff []byte, n int) int {
	ns, err := sock.Write(buff)
	seterrsock(err)

	if ns < n {
		return -1
	}
	return ns
}

/* generate tcp socket -------------------------------------------------------*/
func (tcp *TcpConn) GenTcp(ctype int, msg *string) int {

	if ctype == 0 { /* server socket */
		tcp.sock = Accept_nb(tcp.addr)
		if tcp.sock == nil {
			*msg = "bind error"
			tcp.state = -1
			return 0
		}
		tcp.state = 1
	} else {
		var err error

		tcp.sock, err = net.Dial("tcp", tcp.ResolveAddr())
		seterrsock(err)
		if tcp.sock == nil {
			*msg = fmt.Sprintf("connect error: %s", err)
			tcp.state = -1
			return 0
		}
	}
	tcp.state = 1
	tcp.tact = TickGet()
	Tracet(5, "gentcp: exit sock=%d\n", tcp.sock)

	return 1
}

/* disconnect tcp ------------------------------------------------------------*/
func (tcp *TcpConn) DisconnectTcp(tcon int) {
	Tracet(3, "discontcp: sock=%d tcon=%d\n", tcp.sock, tcon)

	tcp.sock.Close()
	tcp.state = 0
	tcp.tcon = tcon
	tcp.tdis = TickGet()
}

/* open tcp server -----------------------------------------------------------*/
func OpenTcpSvr(path string, msg *string) *TcpSvr {
	var (
		tcpsvr *TcpSvr = new(TcpSvr)
		port   string
	)
	Tracet(3, "opentcpsvr: path=%s\n", path)

	DecodeTcpPath(path, &tcpsvr.svr.saddr, &port, nil, nil, nil, nil)
	if n, _ := fmt.Sscanf(port, "%d", &tcpsvr.svr.port); n < 1 {
		*msg = fmt.Sprintf("port error: %s", port)
		Tracet(1, "opentcpsvr: port error port=%s\n", port)
		tcpsvr = nil
		return nil
	}
	if tcpsvr.svr.GenTcp(0, msg) == 0 {
		tcpsvr = nil
		return nil
	}
	tcpsvr.svr.tcon = 0
	return tcpsvr
}

/* close tcp server ----------------------------------------------------------*/
func (tcpsvr *TcpSvr) CloseTcpSvr() {
	Tracet(3, "closetcpsvr:\n")

	for i := 0; i < MAXCLI; i++ {
		if tcpsvr.cli[i].state > 0 {
			tcpsvr.cli[i].sock.Close()
			tcpsvr.cli[i].state = 0
		}
	}
	tcpsvr.svr.sock.Close()
	tcpsvr.svr.state = 0
	tcpsvr = nil
}

/* update tcp server ---------------------------------------------------------*/
func (tcpsvr *TcpSvr) UpdateTcpSvr(msg *string) {
	var (
		saddr string
		i, n  int
	)

	Tracet(4, "updatetcpsvr: state=%d\n", tcpsvr.svr.state)

	if tcpsvr.svr.state == 0 {
		return
	}

	for i = 0; i < MAXCLI; i++ {
		if tcpsvr.cli[i].state == 0 {
			continue
		}
		saddr = tcpsvr.cli[i].saddr
		n++
	}
	if n == 0 {
		tcpsvr.svr.state = 1
		*msg = "waiting..."
		return
	}
	tcpsvr.svr.state = 2
	if n == 1 {
		*msg = saddr
	} else {
		*msg = fmt.Sprintf("%d clients", n)
	}
}

/* accept client connection --------------------------------------------------*/
func (tcpsvr *TcpSvr) AccSock(msg *string) int {
	var (
		sock net.Conn
		i    int
	)

	Tracet(4, "accsock: sock=%d\n", tcpsvr.svr.sock)

	for i = 0; i < MAXCLI; i++ {
		if tcpsvr.cli[i].state == 0 {
			break
		}
	}
	if i >= MAXCLI {
		Tracet(2, "accsock: too many clients sock=%d\n", tcpsvr.svr.sock)
		return 0
	}
	if sock = Accept_nb(tcpsvr.svr.addr); sock == nil {
		if err := errsock(); err != nil {
			*msg = fmt.Sprintf("accept error (%s)", err.Error())
			Tracet(1, "accsock: accept error sock=%d err=%s\n", tcpsvr.svr.sock, err.Error())
		}
		tcpsvr.svr.sock.Close()
		tcpsvr.svr.state = 0
		return 0
	}
	if sock == nil {
		return 0
	}
	if setsock(sock, msg) == 0 {
		return 0
	}

	tcpsvr.cli[i].sock = sock
	tcpsvr.cli[i].addr = sock.RemoteAddr()
	tcpsvr.cli[i].saddr = tcpsvr.svr.addr.String()
	*msg = tcpsvr.cli[i].saddr
	Tracet(3, "accsock: connected sock=%d addr=%s i=%d\n",
		tcpsvr.cli[i].sock, tcpsvr.cli[i].saddr, i)
	tcpsvr.cli[i].state = 2
	tcpsvr.cli[i].tact = TickGet()
	return 1
}

/* wait socket accept --------------------------------------------------------*/
func (tcpsvr *TcpSvr) WaitTcpSvr(msg *string) int {
	Tracet(4, "waittcpsvr: sock=%d state=%d\n", tcpsvr.svr.sock, tcpsvr.svr.state)

	if tcpsvr.svr.state <= 0 {
		return 0
	}

	for tcpsvr.AccSock(msg) > 0 {
	}

	tcpsvr.UpdateTcpSvr(msg)
	if tcpsvr.svr.state == 2 {
		return 1
	}
	return 0
}

/* read tcp server -----------------------------------------------------------*/
func (tcpsvr *TcpSvr) ReadTcpSvr(buff []uint8, n int, msg *string) int {
	var i, nr int

	Tracet(4, "readtcpsvr: state=%d\n", tcpsvr.svr.state)

	if tcpsvr.WaitTcpSvr(msg) == 0 {
		return 0
	}

	for i = 0; i < MAXCLI; i++ {
		if tcpsvr.cli[i].state != 2 {
			continue
		}

		if nr = Recv_nb(tcpsvr.cli[i].sock, buff, n); nr == -1 {
			if err := errsock(); err != nil {
				Tracet(2, "readtcpsvr: recv error sock=%d err=%s\n",
					tcpsvr.cli[i].sock, err.Error())
			}
			tcpsvr.cli[i].DisconnectTcp(ticonnect)
			tcpsvr.UpdateTcpSvr(msg)
		}
		if nr > 0 {
			tcpsvr.cli[i].tact = TickGet()
			return nr
		}
	}
	return 0
}

/* write tcp server ----------------------------------------------------------*/
func (tcpsvr *TcpSvr) WriteTcpSvr(buff []uint8, n int, msg *string) int {
	var i, ns, nmax int

	Tracet(4, "writetcpsvr: state=%d n=%d\n", tcpsvr.svr.state, n)

	if tcpsvr.WaitTcpSvr(msg) == 0 {
		return 0
	}

	for i = 0; i < MAXCLI; i++ {
		if tcpsvr.cli[i].state != 2 {
			continue
		}

		if ns = Send_nb(tcpsvr.cli[i].sock, buff, n); ns == -1 {
			if err := errsock(); err != nil {
				Tracet(2, "writetcpsvr: send error i=%d sock=%d err=%s\n", i,
					tcpsvr.cli[i].sock, err.Error())
			}
			tcpsvr.cli[i].DisconnectTcp(ticonnect)
			tcpsvr.UpdateTcpSvr(msg)
		} else {
			if ns > nmax {
				nmax = ns
			}
			if ns > 0 {
				tcpsvr.cli[i].tact = TickGet()
			}
		}
	}
	return nmax
}

/* get state tcp server ------------------------------------------------------*/
func (tcpsvr *TcpSvr) StateTcpSvr() int {
	if tcpsvr != nil {
		return tcpsvr.svr.state
	}
	return 0
}

/* print extended state tcp --------------------------------------------------*/
func (tcp *TcpConn) StatExTcp(msg *string) int {
	n := len(*msg)
	*msg += fmt.Sprintf("    state = %d\n", tcp.state)
	*msg += fmt.Sprintf("    saddr = %s\n", tcp.saddr)
	*msg += fmt.Sprintf("    port  = %d\n", tcp.port)
	*msg += fmt.Sprintf("    sock  = %d\n", tcp.sock)

	return len(*msg) - n
}

/* get extended state tcp server ---------------------------------------------*/
func (tcpsvr *TcpSvr) StatExTcpSvr(msg *string) int {
	var state int
	if tcpsvr != nil {
		state = tcpsvr.svr.state
	}

	*msg += "tcpsvr:\n"
	*msg += fmt.Sprintf("  state   = %d\n", state)
	if state == 0 {
		return 0
	}
	*msg += "  svr:\n"
	tcpsvr.svr.StatExTcp(msg)
	for i := 0; i < MAXCLI; i++ {
		if tcpsvr.cli[i].state == 0 {
			continue
		}
		*msg += fmt.Sprintf("  cli#%d:\n", i)
		tcpsvr.cli[i].StatExTcp(msg)
	}
	return state
}

/* connect server ------------------------------------------------------------*/
func (tcpcli *TcpClient) ConnectSock(msg *string) int {
	var stat int

	Tracet(4, "consock: sock=%d\n", tcpcli.svr.sock)

	/* wait re-connect */
	if tcpcli.svr.tcon < 0 || (tcpcli.svr.tcon > 0 &&
		int(TickGet()-int64(tcpcli.svr.tdis)) < tcpcli.svr.tcon) {
		return 0
	}
	/* non-block connect */
	if stat = Connect_nb(tcpcli.svr.sock, tcpcli.svr.addr); stat == -1 {
		if err := errsock(); err != nil {
			*msg = fmt.Sprintf("connect error (%s)", err.Error())
			Tracet(2, "consock: connect error sock=%d err=%s\n", tcpcli.svr.sock, err.Error())
		}
		tcpcli.svr.sock.Close()
		tcpcli.svr.state = 0
		return 0
	}
	if stat == 0 { /* not connect */
		*msg = "connecting..."
		return 0
	}
	*msg = tcpcli.svr.saddr
	Tracet(3, "consock: connected sock=%d addr=%s\n", tcpcli.svr.sock, tcpcli.svr.saddr)
	tcpcli.svr.state = 2
	tcpcli.svr.tact = TickGet()
	return 1
}

/* open tcp client -----------------------------------------------------------*/
func OpenTcpClient(path string, msg *string) *TcpClient {
	var (
		tcpcli *TcpClient = new(TcpClient)
		port   string
		err    error
	)

	Tracet(3, "opentcpcli: path=%s\n", path)

	DecodeTcpPath(path, &tcpcli.svr.saddr, &port, nil, nil, nil, nil)
	tcpcli.svr.port, err = strconv.Atoi(port)
	if err != nil {
		*msg = fmt.Sprintf("port error: %s", port)
		Tracet(2, "opentcp: port error port=%s\n", port)
		return nil
	}

	tcpcli.svr.tcon = 0
	tcpcli.toinact = toinact
	tcpcli.tirecon = ticonnect
	return tcpcli
}

/* close tcp client ----------------------------------------------------------*/
func (tcpcli *TcpClient) CloseTcpClient() {
	Tracet(3, "closetcpcli: sock=%d\n", tcpcli.svr.sock)

	tcpcli.svr.sock.Close()
	tcpcli.svr.state = 0
	//tcpcli.svr.sock = nil
	tcpcli = nil
}

/* wait socket connect -------------------------------------------------------*/
func (tcpcli *TcpClient) WaitTcpClient(msg *string) int {
	Tracet(4, "waittcpcli: sock=%d state=%d\n", tcpcli.svr.sock, tcpcli.svr.state)

	if tcpcli.svr.state < 0 {
		return 0
	}

	if tcpcli.svr.state == 0 { /* close */
		if tcpcli.svr.GenTcp(1, msg) == 0 {
			return 0
		}
	}
	if tcpcli.svr.state == 1 { /* wait */
		if tcpcli.ConnectSock(msg) == 0 {
			//			tcpcli.svr.DisconnectTcp(tcpcli.tirecon)
			return 0
		}
	}
	if tcpcli.svr.state == 2 { /* connect */
		if tcpcli.toinact > 0 &&
			int(TickGet()-int64(tcpcli.svr.tact)) > tcpcli.toinact {
			*msg = "timeout"
			Tracet(2, "waittcpcli: inactive timeout sock=%d\n", tcpcli.svr.sock)
			tcpcli.svr.DisconnectTcp(tcpcli.tirecon)
			return 0
		}
	}
	return 1
}

/* read tcp client -----------------------------------------------------------*/
func (tcpcli *TcpClient) ReadTcpClient(buff []uint8, n int, msg *string) int {
	var nr int

	Tracet(4, "readtcpcli: sock=%d\n", tcpcli.svr.sock)

	if tcpcli.WaitTcpClient(msg) == 0 {
		return 0
	}

	if nr = Recv_nb(tcpcli.svr.sock, buff, n); nr == -1 {
		if err := errsock(); err != nil {
			Tracet(2, "readtcpcli: recv error sock=%d err=%s\n", tcpcli.svr.sock, err.Error())
			*msg = fmt.Sprintf("recv error (%s)", err.Error())
		} else {
			*msg = "disconnected"
		}
		tcpcli.svr.DisconnectTcp(tcpcli.tirecon)
		return 0
	}
	if nr > 0 {
		tcpcli.svr.tact = TickGet()
	}
	Tracet(5, "readtcpcli: exit sock=%d nr=%d\n", tcpcli.svr.sock, nr)
	return nr
}

/* write tcp client ----------------------------------------------------------*/
func (tcpcli *TcpClient) WriteTcpClient(buff []uint8, n int, msg *string) int {
	var ns int

	Tracet(3, "writetcpcli: sock=%d state=%d n=%d\n", tcpcli.svr.sock, tcpcli.svr.state, n)

	if tcpcli.WaitTcpClient(msg) == 0 {
		return 0
	}

	if ns = Send_nb(tcpcli.svr.sock, buff, n); ns == -1 {
		if err := errsock(); err != nil {
			Tracet(2, "writetcp: send error sock=%d err=%s\n", tcpcli.svr.sock, err.Error())
			*msg = fmt.Sprintf("send error (%s)", err.Error())
		}
		tcpcli.svr.DisconnectTcp(tcpcli.tirecon)
		return 0
	}
	if ns > 0 {
		tcpcli.svr.tact = TickGet()
	}
	Tracet(5, "writetcpcli: exit sock=%d ns=%d\n", tcpcli.svr.sock, ns)
	return ns
}

/* get state tcp client ------------------------------------------------------*/
func (tcpcli *TcpClient) StateTcpCli() int {
	if tcpcli != nil {
		return tcpcli.svr.state
	}
	return 0
}

/* get extended state tcp client ---------------------------------------------*/
func (tcpcli *TcpClient) StatExTcpClient(msg *string) int {
	if tcpcli != nil {
		return tcpcli.svr.state
	}
	return 0
}

/* base64 encoder ------------------------------------------------------------*/
func encbase64(str []rune, bytes []uint8, n int) string {
	var (
		table      string = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/"
		i, j, k, b int
	)

	Tracet(4, "encbase64: n=%d\n", n)

	for i, j = 0, 0; i/8 < n; j++ {
		for k, b = 0, 0; k < 6; k, i = k+1, i+1 {
			b <<= 1
			if i/8 < n {
				b |= int(bytes[i/8]>>(7-i%8)) & 0x1
			}
		}
		str[j] = rune(table[b])
	}
	for ; j&0x3 > 0; j++ {
		str[j] = '='
	}
	Tracet(5, "encbase64: str=%s\n", string(str[:j]))
	return string(str[:j])
}

/* send ntrip server request -------------------------------------------------*/
func (ntrip *NTrip) RequestNtrip_s(msg *string) int {
	//var buff[]rune = make(rune, 1024+NTRIP_MAXSTR)
	var p string

	Tracet(3, "reqntrip_s: state=%d\n", ntrip.state)

	p += fmt.Sprintf("SOURCE %s %s\r\n", ntrip.passwd, ntrip.mntpnt)
	p += fmt.Sprintf("Source-Agent: NTRIP %s\r\n", NTRIP_AGENT)
	p += fmt.Sprintf("STR: %s\r\n", ntrip.str)
	p += "\r\n"
	if ntrip.tcp.WriteTcpClient([]uint8(p), len(p), msg) != len(p) {
		return 0
	}

	Tracet(3, "reqntrip_s: send request state=%d ns=%d\n", ntrip.state, len(p))
	Tracet(5, "reqntrip_s: n=%d buff=\n%s\n", len(p), p)
	ntrip.state = 1
	return 1
}

/* send ntrip client request -------------------------------------------------*/
func (ntrip *NTrip) RequestNtrip_c(msg *string) int {
	//char buff[MAXSTRPATH+1024],user[514],*p=buff;
	var user, p string
	Tracet(3, "reqntrip_c: state=%d\n", ntrip.state)

	p += fmt.Sprintf("GET %s/%s HTTP/1.0\r\n", ntrip.url, ntrip.mntpnt)
	p += fmt.Sprintf("User-Agent: NTRIP %s\r\n", NTRIP_AGENT)

	if len(ntrip.user) == 0 {
		p += "Accept: */*\r\n"
		p += "Connection: close\r\n"
	} else {
		user = fmt.Sprintf("%s:%s", ntrip.user, ntrip.passwd)
		p += "Authorization: Basic "
		q := []rune(p)
		p += encbase64(q, []uint8(user), len(user))
		p += "\r\n"
	}
	p += "\r\n"

	if ntrip.tcp.WriteTcpClient([]uint8(p), len(p), msg) != len(p) {
		return 0
	}

	Tracet(3, "reqntrip_c: send request state=%d ns=%d\n", ntrip.state, len(p))
	Tracet(5, "reqntrip_c: n=%d buff=\n%s\n", len(p), p)
	ntrip.state = 1
	return 1
}

/* test ntrip server response ------------------------------------------------*/
func (ntrip *NTrip) ResponseNtrip_s(msg *string) int {
	var i, nb, idx int
	//char *p,*q;

	Tracet(3, "rspntrip_s: state=%d nb=%d\n", ntrip.state, ntrip.nb)
	ntrip.buff = ntrip.buff[:ntrip.nb]
	Tracet(5, "rspntrip_s: n=%d buff=\n%s\n", ntrip.nb, ntrip.buff)

	if idx = strings.Index(string(ntrip.buff[:]), NTRIP_RSP_OK_SVR); idx >= 0 { /* ok */
		// q=(char *)ntrip.buff;
		// p+=strlen(NTRIP_RSP_OK_SVR);
		idx += len(NTRIP_RSP_OK_SVR)
		//ntrip.nb-=p-q;
		ntrip.nb -= idx
		var p = []rune(ntrip.buff)
		for i = 0; i < ntrip.nb; i++ {
			//ntrip.buff[i] = ntrip.buff[i+idx]
			p[i] = p[i+idx]
		}
		ntrip.buff = string(p)
		ntrip.state = 2
		*msg = fmt.Sprintf("%s/%s", ntrip.tcp.svr.saddr, ntrip.mntpnt)
		Tracet(3, "rspntrip_s: response ok nb=%d\n", ntrip.nb)
		return 1
	} else if idx = strings.Index(string(ntrip.buff[:]), NTRIP_RSP_ERROR); idx >= 0 { /* error */
		nb = MAXSTATMSG
		if ntrip.nb < MAXSTATMSG {
			nb = ntrip.nb
		}
		*msg = fmt.Sprintf("%.*s", nb, string(ntrip.buff[:]))
		//if ((p=strchr(msg,'\r'))) *p='\0';
		p := *msg
		*msg = p[:strings.Index(*msg, "\r")]
		Tracet(3, "rspntrip_s: %s nb=%d\n", *msg, ntrip.nb)
		ntrip.nb = 0
		ntrip.buff = ""
		ntrip.state = 0
		ntrip.tcp.svr.DisconnectTcp(ntrip.tcp.tirecon)
	} else if ntrip.nb >= NTRIP_MAXRSP { /* buffer overflow */
		*msg = "response overflow"
		Tracet(3, "rspntrip_s: response overflow nb=%d\n", ntrip.nb)
		ntrip.nb = 0
		ntrip.buff = ""
		ntrip.state = 0
		ntrip.tcp.svr.DisconnectTcp(ntrip.tcp.tirecon)
	}
	Tracet(5, "rspntrip_s: exit state=%d nb=%d\n", ntrip.state, ntrip.nb)
	return 0
}

/* test ntrip client response ------------------------------------------------*/
func (ntrip *NTrip) ResponseNtrip_c(msg *string) int {
	var idx int
	//char *p,*q;

	Tracet(3, "rspntrip_c: state=%d nb=%d\n", ntrip.state, ntrip.nb)
	ntrip.buff = ntrip.buff[:ntrip.nb]
	Tracet(5, "rspntrip_c: n=%d buff=\n%s\n", ntrip.nb, ntrip.buff)

	if idx = strings.Index(string(ntrip.buff[:]), NTRIP_RSP_OK_CLI); idx >= 0 { /* ok */
		idx += len(NTRIP_RSP_OK_CLI)
		ntrip.nb -= idx
		ntrip.buff = ntrip.buff[idx:]
		ntrip.state = 2
		*msg = fmt.Sprintf("%s/%s", ntrip.tcp.svr.saddr, ntrip.mntpnt)
		Tracet(3, "rspntrip_c: response ok nb=%d\n", ntrip.nb)
		return 1
	}
	if idx = strings.Index(string(ntrip.buff[:]), NTRIP_RSP_SRCTBL); idx >= 0 { /* source table */
		if len(ntrip.mntpnt) == 0 { /* source table request */
			ntrip.state = 2
			*msg = "source table received"
			Tracet(3, "rspntrip_c: receive source table nb=%d\n", ntrip.nb)
			return 1
		}
		*msg = "no mountp. reconnect..."
		Tracet(2, "rspntrip_c: no mount point nb=%d\n", ntrip.nb)
		ntrip.nb = 0
		ntrip.buff = ""
		ntrip.state = 0
		ntrip.tcp.svr.DisconnectTcp(ntrip.tcp.tirecon)
	} else if idx = strings.Index(string(ntrip.buff[:]), NTRIP_RSP_HTTP); idx >= 0 { /* http response */
		if q := strings.Index(ntrip.buff[idx:], "\r"); q >= 0 {
			ntrip.buff = ntrip.buff[:q]
		} else if len(ntrip.buff) > 128 {
			ntrip.buff = ntrip.buff[:128]
		}
		*msg = ntrip.buff[idx:]
		Tracet(3, "rspntrip_s: %s nb=%d\n", *msg, ntrip.nb)
		ntrip.nb = 0
		ntrip.buff = ""
		ntrip.state = 0
		ntrip.tcp.svr.DisconnectTcp(ntrip.tcp.tirecon)
	} else if ntrip.nb >= NTRIP_MAXRSP { /* buffer overflow */
		*msg = "response overflow"
		Tracet(2, "rspntrip_s: response overflow nb=%d\n", ntrip.nb)
		ntrip.nb = 0
		ntrip.buff = ""
		ntrip.state = 0
		ntrip.tcp.svr.DisconnectTcp(ntrip.tcp.tirecon)
	}
	Tracet(5, "rspntrip_c: exit state=%d nb=%d\n", ntrip.state, ntrip.nb)
	return 0
}

/* wait ntrip request/response -----------------------------------------------*/
func (ntrip *NTrip) WaitNtrip(msg *string) int {
	var n, ret int
	//char *p;

	Tracet(4, "waitntrip: state=%d nb=%d\n", ntrip.state, ntrip.nb)

	if ntrip.state < 0 {
		return 0 /* error */
	}

	if ntrip.tcp.svr.state < 2 {
		ntrip.state = 0 /* tcp disconnected */
	}

	if ntrip.state == 0 { /* send request */
		if ntrip.ctype == 0 {
			ret = ntrip.RequestNtrip_s(msg)
		} else {
			ret = ntrip.RequestNtrip_c(msg)
		}
		if ret == 0 {
			return 0
		}
		Tracet(3, "waitntrip: state=%d nb=%d\n", ntrip.state, ntrip.nb)
	}
	if ntrip.state == 1 { /* read response */
		//p := []uint8(ntrip.buff[ntrip.nb:])
		p := make([]uint8, NTRIP_MAXRSP)
		if n = ntrip.tcp.ReadTcpClient(p, NTRIP_MAXRSP-ntrip.nb-1, msg); n == 0 {
			Tracet(5, "waitntrip: readtcp n=%d\n", n)
			return 0
		}
		ntrip.buff += string(p)
		ntrip.nb += n
		//		ntrip.buff = ntrip.buff[:ntrip.nb]

		/* wait response */
		if ntrip.ctype == 0 {
			return ntrip.ResponseNtrip_s(msg)
		}
		return ntrip.ResponseNtrip_c(msg)
	}
	return 1
}

/* open ntrip ----------------------------------------------------------------*/
func OpenNtrip(path string, ctype int, msg *string) *NTrip {
	var (
		ntrip             = new(NTrip)
		addr, port, tpath string
	)

	Tracet(3, "openntrip: path=%s type=%d\n", path, ctype)

	ntrip.state = 0
	ntrip.ctype = ctype /* 0:server,1:client */
	ntrip.nb = 0
	/* decode tcp/ntrip path */
	DecodeTcpPath(path, &addr, &port, &ntrip.user, &ntrip.passwd, &ntrip.mntpnt, &ntrip.str)

	/* use default port if no port specified */
	if len(port) == 0 {
		if ctype > 0 {
			port = strconv.Itoa(NTRIP_CLI_PORT)
		} else {
			port = strconv.Itoa(NTRIP_SVR_PORT)
		}
	}
	tpath = fmt.Sprintf("%s:%s", addr, port)

	/* ntrip access via proxy server */
	if len(proxyaddr) > 0 {
		ntrip.url = fmt.Sprintf("http://%.*s", MAXSTRPATH-8, tpath)
		tpath = fmt.Sprintf("%.*s", MAXSTRPATH-1, proxyaddr)
	}
	/* open tcp client stream */
	if ntrip.tcp = OpenTcpClient(tpath, msg); ntrip.tcp == nil {
		Tracet(2, "openntrip: opentcp error\n")
		ntrip = nil
		return nil
	}
	return ntrip
}

/* close ntrip ---------------------------------------------------------------*/
func (ntrip *NTrip) CloseNtrip() {
	Tracet(3, "closentrip: state=%d\n", ntrip.state)

	ntrip.tcp.CloseTcpClient()
	ntrip = nil
}

/* read ntrip ----------------------------------------------------------------*/
func (ntrip *NTrip) ReadNtrip(buff []uint8, n int, msg *string) int {
	var nb int

	Tracet(4, "readntrip:\n")

	if ntrip.WaitNtrip(msg) == 0 {
		return 0
	}

	if ntrip.nb > 0 { /* read response buffer first */
		nb = n
		if ntrip.nb <= n {
			nb = ntrip.nb
		}
		copy(buff, []uint8(ntrip.buff)[ntrip.nb-nb:ntrip.nb])
		ntrip.nb = 0
		ntrip.buff = ""
		return nb
	}
	return ntrip.tcp.ReadTcpClient(buff, n, msg)
}

/* write ntrip ---------------------------------------------------------------*/
func (ntrip *NTrip) WriteNtrip(buff []uint8, n int, msg *string) int {
	Tracet(3, "writentrip: n=%d\n", n)

	if ntrip.WaitNtrip(msg) == 0 {
		return 0
	}

	return ntrip.tcp.WriteTcpClient(buff, n, msg)
}

/* get state ntrip -----------------------------------------------------------*/
func (ntrip *NTrip) StateNtrip() int {
	if ntrip == nil {
		return 0
	}
	if ntrip.state == 0 {
		return ntrip.tcp.svr.state
	}
	return ntrip.state
}

/* get extended state ntrip --------------------------------------------------*/
func (ntrip *NTrip) StatExNtrip(msg *string) int {
	//char *p=msg;
	state := ntrip.StateNtrip()

	*msg += "ntrip:\n"
	*msg += fmt.Sprintf("  state   = %d\n", state)
	if state == 0 {
		return 0
	}
	*msg += fmt.Sprintf("  state   = %d\n", state)
	*msg += fmt.Sprintf("  type    = %d\n", ntrip.ctype)
	*msg += fmt.Sprintf("  nb      = %d\n", ntrip.nb)
	*msg += fmt.Sprintf("  url     = %s\n", ntrip.url)
	*msg += fmt.Sprintf("  mntpnt  = %s\n", ntrip.mntpnt)
	*msg += fmt.Sprintf("  user    = %s\n", ntrip.user)
	*msg += fmt.Sprintf("  passwd  = %s\n", ntrip.passwd)
	*msg += fmt.Sprintf("  str     = %s\n", ntrip.str)
	*msg += "  svr:\n"
	ntrip.tcp.svr.StatExTcp(msg)
	return state
}

/* open ntrip-caster ---------------------------------------------------------*/
func OpenNtripc(path string, msg *string) *NTripc {
	var (
		ntripc      *NTripc = new(NTripc)
		port, tpath string
	)
	ntripc.con = make([]NTripc_con, MAXCLI)

	Tracet(3, "openntripc: path=%s\n", path)

	/* decode tcp/ntrip path */
	DecodeTcpPath(path, nil, &port, &ntripc.user, &ntripc.passwd, &ntripc.mntpnt,
		&ntripc.srctbl)

	if len(ntripc.mntpnt) == 0 {
		Tracet(2, "openntripc: no mountpoint path=%s\n", path)
		ntripc = nil
		return nil
	}
	/* use default port if no port specified */
	if len(port) == 0 {
		port = strconv.Itoa(NTRIP_CLI_PORT)
	}
	tpath = fmt.Sprintf(":%s", port)

	/* open tcp server stream */
	if ntripc.tcp = OpenTcpSvr(tpath, msg); ntripc.tcp == nil {
		Tracet(2, "openntripc: opentcpsvr error port=%s\n", port)
		ntripc = nil
		return nil
	}
	return ntripc
}

/* close ntrip-caster --------------------------------------------------------*/
func (ntripc *NTripc) CloseNtripc() {
	Tracet(3, "closentripc: state=%d\n", ntripc.state)

	ntripc.tcp.CloseTcpSvr()
	ntripc = nil
}

/* disconnect ntrip-caster connection ----------------------------------------*/
func (ntripc *NTripc) DisconnectNtripc(i int) {
	Tracet(3, "discon_ntripc: i=%d\n", i)

	ntripc.tcp.cli[i].DisconnectTcp(ticonnect)
	ntripc.con[i].nb = 0
	ntripc.con[i].buff = ""
	ntripc.con[i].state = 0
}

/* send ntrip source table ---------------------------------------------------*/
func (ntripc *NTripc) Send_SrcTbl(sock net.Conn) {
	var srctbl, buff string

	srctbl = fmt.Sprintf("STR;%s;%s\r\n%s\r\n", ntripc.mntpnt, ntripc.srctbl,
		NTRIP_RSP_TBLEND)
	buff += NTRIP_RSP_SRCTBL
	buff += fmt.Sprintf("Server: %s %s %s\r\n", "RTKLIB", VER_GNSSGO, PATCH_LEVEL)
	buff += fmt.Sprintf("Date: %s UTC\r\n", TimeStr(TimeGet(), 0))
	buff += "Connection: close\r\n"
	buff += "Content-Type: text/plain\r\n"
	buff += fmt.Sprintf("Content-Length: %d\r\n\r\n", len(srctbl))
	Send_nb(sock, []byte(buff), len(buff))
	Send_nb(sock, []byte(srctbl), len(srctbl))
}

/* test ntrip client request -------------------------------------------------*/
func (ntripc *NTripc) RequestNtripc(i int) {
	rsp1 := NTRIP_RSP_UNAUTH
	rsp2 := NTRIP_RSP_OK_CLI
	con := &ntripc.con[i]
	var url, mntpnt, proto, user, user_pwd string

	Tracet(3, "rspntripc_c i=%d\n", i)
	con.buff = con.buff[:con.nb]
	Tracet(5, "rspntripc_c: n=%d,buff=\n%s\n", con.nb, con.buff)

	if con.nb >= NTRIP_MAXRSP-1 { /* buffer overflow */
		Tracet(2, "rsp_ntripc_c: request buffer overflow\n")
		ntripc.DisconnectNtripc(i)
		return
	}
	headers := ParseHttp(con.buff)
	_, ok1 := headers.GetHead("GET")
	_, ok2 := headers.GetHead("User-Agent")
	/* test GET and User-Agent */
	// idx1 = strings.Index(con.buff, "GET")
	// if idx1 >= 0 {
	// 	idx2 = strings.Index(con.buff[idx1:], "\r\n")
	// }
	if !ok1 || !ok2 {
		Tracet(2, "rsp_ntripc_c: NTRIP request error\n")
		ntripc.DisconnectNtripc(i)
		return
	}
	// if idx1,idx2 = strings.Index(con.buff, "GET"), strings.Index(con.buff, "User-Agent:");idx1 == -1 || idx2 == -1 {
	// // if (!(p=strstr((char *)con.buff,"GET"))||!(q=strstr(p,"\r\n"))||
	// //     !(q=strstr(q,"User-Agent:"))||!strstr(q,"\r\n")) {
	//     tracet(2,"rsp_ntripc_c: NTRIP request error\n");
	//     discon_ntripc(ntripc,i);
	//     return;
	// }
	/* test protocol */
	if headers.ParseBody("GET", &url, &proto) != nil || strings.Compare(proto, "HTTP/1.0") != 0 ||
		strings.Compare(proto, "HTTP/1.1") != 0 {
		// if (sscanf(p,"GET %255s %255s",url,proto)<2||
		//     (strcmp(proto,"HTTP/1.0")&&strcmp(proto,"HTTP/1.1"))) {
		//     tracet(2,"rsp_ntripc_c: NTRIP request error proto=%s\n",proto);
		//     discon_ntripc(ntripc,i);
		return
	}
	mntpnt = strings.TrimPrefix(url, "/")
	//   if ((p=strchr(url,'/'))) strcpy(mntpnt,p+1);

	/* test mountpoint */
	if len(mntpnt) == 0 || strings.Compare(mntpnt, ntripc.mntpnt) != 0 {
		//   if (!*mntpnt||strcmp(mntpnt,ntripc.mntpnt)) {
		Tracet(2, "rsp_ntripc_c: no mountpoint %s\n", mntpnt)

		/* send source table */
		ntripc.Send_SrcTbl(ntripc.tcp.cli[i].sock)
		ntripc.DisconnectNtripc(i)
		return
	}
	/* test authentication */
	if len(ntripc.passwd) > 0 {
		user = fmt.Sprintf("%s:%s", ntripc.user, ntripc.passwd)
		q := user_pwd
		q += "Authorization: Basic "
		q += encbase64([]rune(q), []byte(user), len(user))
		var pwd string
		if headers.ParseBody("Authorization", &pwd) != nil || strings.Compare(pwd, q) != 0 {
			// if (!(p=strstr((char *)con.buff,"Authorization:"))||
			//     strncmp(p,user_pwd,strlen(user_pwd))) {
			Tracet(2, "rsp_ntripc_c: authroziation error\n")
			Send_nb(ntripc.tcp.cli[i].sock, []byte(rsp1), len(rsp1))
			ntripc.DisconnectNtripc(i)
			return
		}
	}
	/* send OK response */
	Send_nb(ntripc.tcp.cli[i].sock, []byte(rsp2), len(rsp2))

	con.state = 1
	con.mntpnt = mntpnt
}

/* handle ntrip client connect request ---------------------------------------*/
func (ntripc *NTripc) WaitNtripc(msg *string) {
	var (
		buff       []byte
		i, n, nmax int
	)

	Tracet(4, "wait_ntripc\n")

	ntripc.state = ntripc.tcp.svr.state

	if ntripc.tcp.WaitTcpSvr(msg) == 0 {
		return
	}

	for i = 0; i < len(ntripc.tcp.cli); i++ {
		if ntripc.tcp.cli[i].state != 2 || ntripc.con[i].state > 0 {
			continue
		}

		/* receive ntrip client request */
		buff = []byte(ntripc.con[i].buff[ntripc.con[i].nb:])
		nmax = NTRIP_MAXRSP - ntripc.con[i].nb - 1

		if n = Recv_nb(ntripc.tcp.cli[i].sock, buff, nmax); n == -1 {
			if err := errsock(); err != nil {
				Tracet(2, "wait_ntripc: recv error sock=%d err=%s\n",
					ntripc.tcp.cli[i].sock, err.Error())
			}
			ntripc.DisconnectNtripc(i)
			continue
		}
		if n <= 0 {
			continue
		}

		/* test ntrip client request */
		ntripc.con[i].nb += n
		ntripc.RequestNtripc(i)
	}
}

/* read ntrip-caster ---------------------------------------------------------*/
func (ntripc *NTripc) ReadNtripc(buff []byte, n int, msg *string) int {
	var i, nr int

	Tracet(4, "readntripc:\n")

	ntripc.WaitNtripc(msg)

	for i = 0; i < len(ntripc.con); i++ {
		if ntripc.con[i].state == 0 {
			continue
		}

		nr = Recv_nb(ntripc.tcp.cli[i].sock, buff, n)

		if nr < 0 {
			if err := errsock(); err != nil {
				Tracet(2, "readntripc: recv error i=%d sock=%d err=%s\n", i,
					ntripc.tcp.cli[i].sock, err.Error())
			}
			ntripc.DisconnectNtripc(i)
		} else if nr > 0 {
			ntripc.tcp.cli[i].tact = TickGet()
			return nr
		}
	}
	return 0
}

/* write ntrip-caster --------------------------------------------------------*/
func (ntripc *NTripc) WriteNtripc(buff []byte, n int, msg *string) int {
	var i, ns int

	Tracet(4, "writentripc: n=%d\n", n)

	ntripc.WaitNtripc(msg)

	for i = 0; i < len(ntripc.con); i++ {
		if ntripc.con[i].state == 0 {
			continue
		}

		ns = Send_nb(ntripc.tcp.cli[i].sock, buff, n)

		if ns < n {
			if err := errsock(); err != nil {
				Tracet(2, "writentripc: send error i=%d sock=%d err=%s\n", i,
					ntripc.tcp.cli[i].sock, err.Error())
			}
			ntripc.DisconnectNtripc(i)
		} else {
			ntripc.tcp.cli[i].tact = TickGet()
		}
	}
	return ns
}

/* get state ntrip-caster ----------------------------------------------------*/
func (ntripc *NTripc) StateNtripc() int {
	if ntripc == nil {
		return 0
	}
	return ntripc.state
}

/* get extended state ntrip-caster -------------------------------------------*/
func (ntripc *NTripc) StatExNtripc(msg *string) int {
	var i int
	state := ntripc.StateNtripc()

	*msg += "ntripc:\n"
	*msg += fmt.Sprintf("  state   = %d\n", ntripc.state)
	if state == 0 {
		return 0
	}

	*msg += fmt.Sprintf("  type    = %d\n", ntripc.ctype)
	*msg += fmt.Sprintf("  mntpnt  = %s\n", ntripc.mntpnt)
	*msg += fmt.Sprintf("  user    = %s\n", ntripc.user)
	*msg += fmt.Sprintf("  passwd  = %s\n", ntripc.passwd)
	*msg += fmt.Sprintf("  srctbl  = %s\n", ntripc.srctbl)
	*msg += "  svr:\n"
	ntripc.tcp.svr.StatExTcp(msg)
	for i = 0; i < len(ntripc.tcp.cli); i++ {
		if ntripc.tcp.cli[i].state == 0 {
			continue
		}
		*msg += fmt.Sprintf("  cli#%d:\n", i)
		ntripc.tcp.cli[i].StatExTcp(msg)
		*msg += fmt.Sprintf("    mntpnt= %s\n", ntripc.con[i].mntpnt)
		*msg += fmt.Sprintf("    nb    = %d\n", ntripc.con[i].nb)
	}
	return state
}

/* generate udp socket -------------------------------------------------------*/
func GenUdp(ctype, port int, saddr string, msg *string) *UdpConn {
	var (
		udp *UdpConn = new(UdpConn)
		err error
	)
	//  struct hostent *hp;
	// bs:=buffsize
	// opt:=1;

	Tracet(3, "genudp: type=%d\n", ctype)

	udp.state = 2
	udp.ctype = ctype
	udp.port = port
	udp.saddr = saddr

	// if ((udp.sock=socket(AF_INET,SOCK_DGRAM,0))==(socket_t)-1) {
	//     sprintf(msg,"socket error (%d)",errsock());
	//     return NULL;
	// }
	// if (setsockopt(udp.sock,SOL_SOCKET,SO_RCVBUF,(const char *)&bs,sizeof(bs))==-1||
	//     setsockopt(udp.sock,SOL_SOCKET,SO_SNDBUF,(const char *)&bs,sizeof(bs))==-1) {
	//     tracet(2,"genudp: setsockopt error sock=%d err=%d bs=%d\n",udp.sock,errsock(),bs);
	//     sprintf(msg,"sockopt error: bufsiz");
	// }
	// memset(&udp.addr,0,sizeof(udp.addr));
	// udp.addr.sin_family=AF_INET;
	// udp.addr.sin_port=htons(port);

	addr, _ := net.ResolveUDPAddr("udp", fmt.Sprintf("%s:%d", saddr, port))
	udp.sock, _ = net.ListenUDP("udp", addr)
	if udp.ctype == 0 { /* udp server */

		udp.sock, err = net.ListenUDP("UDP", addr)
		// udp.addr.sin_addr.s_addr=htonl(INADDR_ANY);
		if err != nil {
			Tracet(2, "genudp: bind error sock= %s port=%d err=%s\n", saddr, port, errsock())
			*msg = fmt.Sprintf("bind error (%s): %d", errsock(), port)
			udp.sock.Close()
			udp.state = 0
			//		udp.sock = nil
			return nil
		}

		// #ifdef SVR_REUSEADDR
		//         setsockopt(udp.sock,SOL_SOCKET,SO_REUSEADDR,(const char *)&opt, sizeof(opt));
		// #endif
		//         if (bind(udp.sock,(struct sockaddr *)&udp.addr,sizeof(udp.addr))==-1) {
		//             tracet(2,"genudp: bind error sock=%d port=%d err=%d\n",udp.sock,port,errsock());
		//             sprintf(msg,"bind error (%d): %d",errsock(),port);
		//             closesocket(udp.sock);
		//             free(udp);
		//             return NULL;
		//         }
	} else { /* udp client */
		udp.sock, err = net.Dial("udp", addr.IP.String())
		if err != nil {
			*msg = fmt.Sprintf("address error (%s)", saddr)
			udp.sock.Close()
			udp.state = 0
			//		udp.sock = nil
			return nil
		}
		// if (!strcmp(saddr,"255.255.255.255")&&
		//     setsockopt(udp.sock,SOL_SOCKET,SO_BROADCAST,(const char *)&opt,
		//                sizeof(opt))==-1) {
		//     tracet(2,"genudp: setsockopt error sock=%d err=%d\n",udp.sock,errsock());
		//     sprintf(msg,"sockopt error: broadcast");
		// }
		// if (!(hp=gethostbyname(saddr))) {
		//     sprintf(msg,"address error (%s)",saddr);
		//     closesocket(udp.sock);
		//     free(udp);
		//     return NULL;
		// }
		// memcpy(&udp.addr.sin_addr,hp.h_addr,hp.h_length);
	}
	return udp
}

/* open udp server -----------------------------------------------------------*/
func OpenUdpSvr(path string, msg *string) *UdpConn {
	var (
		sport string
		port  int
	)

	Tracet(3, "openudpsvr: path=%s\n", path)

	DecodeTcpPath(path, nil, &sport, nil, nil, nil, nil)

	port, _ = strconv.Atoi(sport)
	// if (sscanf(sport,"%d",&port)<1) {
	//     sprintf(msg,"port error: %s",sport);
	//     tracet(2,"openudpsvr: port error port=%s\n",port);
	//     return NULL;
	// }
	return GenUdp(0, port, "localhost", msg)
}

/* close udp server ----------------------------------------------------------*/
func (udpsvr *UdpConn) CloseUdpSvr() {
	Tracet(3, "closeudpsvr: sock=%s\n", udpsvr.sock.LocalAddr().String())

	udpsvr.sock.Close()
	udpsvr.state = 0
	// udpsvr.sock = nil
	udpsvr = nil
}

/* read udp server -----------------------------------------------------------*/
func (udpsvr *UdpConn) ReadUdpSvr(buff []byte, n int, msg *string) int {

	// struct timeval tv={0};
	// fd_set rs;
	// int ret,nr;

	// tracet(4,"readudpsvr: sock=%d n=%d\n",udpsvr.sock,n);

	// FD_ZERO(&rs); FD_SET(udpsvr.sock,&rs);
	// ret=select(udpsvr.sock+1,&rs,NULL,NULL,&tv);
	// if (ret<=0) return ret;
	udpsvr.sock.SetDeadline(time.Now())
	nr, _ := udpsvr.sock.Read([]byte(buff))
	defer udpsvr.sock.SetDeadline(time.Time{}) // remove deadline
	if nr <= 0 {
		return -1
	}
	return nr
}

/* get state udp server ------------------------------------------------------*/
func (udpsvr *UdpConn) StateUdpSvr() int {
	if udpsvr != nil {
		return udpsvr.state
	}
	return 0
}

/* get extended state udp server ---------------------------------------------*/
func (udpsvr *UdpConn) StatExUdpSvr(msg *string) int {
	// char *p=msg;
	state := udpsvr.StateUdpSvr()

	*msg += "udpsvr:\n"
	*msg += fmt.Sprintf("  state   = %d\n", state)
	if state == 0 {
		return 0
	}
	*msg += fmt.Sprintf("  type    = %d\n", udpsvr.ctype)
	*msg += fmt.Sprintf("  sock    = %s\n", udpsvr.sock.LocalAddr())
	*msg += fmt.Sprintf("  port    = %d\n", udpsvr.port)
	return state
}

/* open udp client -----------------------------------------------------------*/
func OpenUdpClient(path string, msg *string) *UdpConn {
	var (
		sport, saddr string
		port         int
		err          error
	)

	Tracet(3, "openudpsvr: path=%s\n", path)

	DecodeTcpPath(path, &saddr, &sport, nil, nil, nil, nil)

	if port, err = strconv.Atoi(sport); err != nil {
		//if (sscanf(sport,"%d",&port)<1) {
		*msg += fmt.Sprintf("port error: %s", sport)
		Tracet(2, "openudpcli: port error port=%s\n", sport)
		return nil
	}
	return GenUdp(1, port, saddr, msg)
}

/* close udp client ----------------------------------------------------------*/
func (udpcli *UdpConn) CloseUdpClient() {
	Tracet(3, "closeudpcli: sock=%s\n", udpcli.sock.LocalAddr())

	udpcli.sock.Close()
	udpcli.state = 0
	// udpcli.sock = nil
	udpcli = nil
}

/* write udp client -----------------------------------------------------------*/
func (udpcli *UdpConn) WriteUdpClient(buff []byte, n int, msg *string) int {
	Tracet(4, "writeudpcli: sock=%d n=%d\n", udpcli.sock, n)

	m, _ := udpcli.sock.Write(buff)
	return m
}

/* get state udp client ------------------------------------------------------*/
func (udpcli *UdpConn) StateUdpClient() int {
	if udpcli == nil {
		return 0
	}
	return udpcli.state
}

/* get extended state udp client ---------------------------------------------*/
func (udpcli *UdpConn) StateXUdpClient(msg *string) int {
	//   char *p=msg;
	state := udpcli.StateUdpClient()

	*msg += "udpsvr:\n"
	*msg += fmt.Sprintf("  state   = %d\n", state)
	if state == 0 {
		return 0
	}
	*msg += fmt.Sprintf("  type    = %d\n", udpcli.ctype)
	*msg += fmt.Sprintf("  sock    = %s\n", udpcli.sock.RemoteAddr())
	*msg += fmt.Sprintf("  addr    = %s\n", udpcli.saddr)
	*msg += fmt.Sprintf("  port    = %d\n", udpcli.port)
	return state
}

/* decode ftp path -----------------------------------------------------------*/
func DecodeFtpPath(path string, addr, file, user, passwd *string, topts []int) {
	var (
		buff string
		idx  int //, idx1, idx2, idx3, idx4
	)

	Tracet(4, "decodeftpath: path=%s\n", path)

	if user != nil {
		*user = ""
	}
	if passwd != nil {
		*passwd = ""
	}
	if len(topts) > 3 {
		topts[0] = 0    /* time offset in path (s) */
		topts[1] = 3600 /* download interval (s) */
		topts[2] = 0    /* download time offset (s) */
		topts[3] = 0    /* retry interval (s) (0: no retry) */
	}
	buff = path

	if idx = strings.Index(buff, "::"); idx > 0 {
		if topts != nil {
			fmt.Sscanf(buff[idx+2:], "T=%d,%d,%d,%d", &topts[0], &topts[1], &topts[2], &topts[3])
		}
		buff = buff[:idx]
	}
	if idx = strings.Index(buff, "/"); idx >= 0 {
		if file != nil {
			*file = buff[idx+1:]
		}
		buff = buff[:idx]
	}
	if idx = strings.Index(buff, "@"); idx >= 0 {
		if addr != nil {
			*addr = buff[idx+1:]
		}
		buff = buff[:idx]
		if idx = strings.Index(buff, ":"); idx >= 0 {
			if passwd != nil {
				*passwd = buff[idx+1:]
			}
			if user != nil {
				*user = buff[:idx]
			}

		}
	} else {
		if addr != nil {
			*addr = buff
		}
	}
	// idx1 = strings.Index(buff, "/")
	// if idx1 >= 0 {
	// 	idx2 = strings.Index(buff[idx1:], "::")
	// 	if idx2 >= 0 {
	// 		if topts != nil {
	// 			fmt.Sscanf(buff[idx1+idx2+2:], "T=%d,%d,%d,%d", &topts[0], &topts[1], &topts[2], &topts[3])
	// 		}
	// 	}
	// 		*file = buff[idx1+1 : idx1+idx2]
	// } else {
	// 	*file = ""
	// }
	// // if ((p=strchr(buff,'/'))) {
	// //     if ((q=strstr(p+1,"::"))) {
	// //         *q='\0';
	// //         if (topts) sscanf(q+2,"T=%d,%d,%d,%d",topts,topts+1,topts+2,topts+3);
	// //     }
	// //     strcpy(file,p+1);
	// //     *p='\0';
	// // }
	// // else file[0]='\0';

	// idx3 = strings.LastIndex(buff, "@")
	// idx4 = strings.Index(buff, ":")
	// if idx1 >= 0 {
	// 	if passwd != nil {
	// 		*passwd = buff[idx4+1 : idx3]
	// 	}
	// 	if user != nil {
	// 		*user = buff[:idx4]
	// 	}
	// } else {
	// 	idx3 = 0
	// }
	// // if ((p=strrchr(buff,'@'))) {
	// //     *p++='\0';
	// //     if ((q=strchr(buff,':'))) {
	// //          *q='\0'; if (passwd) strcpy(passwd,q+1);
	// //     }
	// //     *q='\0'; if (user) strcpy(user,buff);
	// // }
	// // else p=buff;
	// if addr != nil {
	// 	if idx1 > idx3 {
	// 		*addr = buff[idx3+1 : idx1]
	// 	} else {
	// 		*addr = buff[idx3+1:]
	// 	}
	// }
	// strcpy(addr,p);
}

/* next download time --------------------------------------------------------*/
func nextdltime(topts []int, stat int) Gtime {
	var (
		time       Gtime
		tow        float64
		week, tint int
	)

	Tracet(3, "nextdltime: topts=%d %d %d %d stat=%d\n", topts[0], topts[1],
		topts[2], topts[3], stat)

	/* current time (gpst) */
	time = Utc2GpsT(TimeGet())
	tow = Time2GpsT(time, &week)

	/* next retry time */
	if stat == 0 && topts[3] > 0 {
		tow = (math.Floor((tow-float64(topts[2]))/float64(topts[3]))+1.0)*float64(topts[3]) + float64(topts[2])
		return GpsT2Time(week, tow)
	}
	/* next interval time */
	tint = 3600
	if topts[1] > 0 {
		tint = topts[1]
	}
	tow = (math.Floor((tow-float64(topts[2]))/float64(tint))+1.0)*float64(tint) + float64(topts[2])
	time = GpsT2Time(week, tow)

	return time
}

/* ftp thread ----------------------------------------------------------------*/
// #ifdef WIN32
// static DWORD WINAPI ftpthread(void *arg)
// #else
func ftpthread(ftp *FtpConn) {
	// #endif

	//   ftp_t *ftp=(ftp_t *)arg;
	var (
		fp                              *os.File
		time                            Gtime
		remote, local, tmpfile, errfile string
		cmd, env, opt                   string
		proxyopt, proto                 string
		ret, idx                        int
		p                               string
	)

	Tracet(3, "ftpthread:\n")

	if len(localdir) == 0 {
		Tracet(2, "no local directory\n")
		ftp.error = 11
		ftp.state = 3
		return
	}
	/* replace keyword in file path and local path */
	time = TimeAdd(Utc2GpsT(TimeGet()), float64(ftp.topts[0]))
	RepPath(ftp.file, &remote, time, "", "")

	idx = strings.LastIndex(remote, "/")
	if idx >= 0 {
		p = remote[idx+1:]
	} else {
		p = remote
	}
	//    if ((p=strrchr(remote,'/'))) p++; else p=remote;
	local = fmt.Sprintf("%.768s%s%.254s", localdir, FILEPATHSEP, p)
	errfile = fmt.Sprintf("%.1019s.err", local)

	/* if local file exist, skip download */
	tmpfile = local
	idx = strings.LastIndex(tmpfile, ".")
	if idx >= 0 {
		if strings.HasSuffix(tmpfile, ".z") || strings.HasSuffix(tmpfile, ".gz") || strings.HasSuffix(tmpfile, ".zip") ||
			strings.HasSuffix(tmpfile, ".Z") || strings.HasSuffix(tmpfile, ".GZ") || strings.HasSuffix(tmpfile, ".ZIP") {
			tmpfile = tmpfile[:idx]
		}
	}
	fp, _ = os.OpenFile(tmpfile, os.O_RDONLY, 0666)
	if fp != nil {
		fp.Close()
		ftp.local = fmt.Sprintf("%.1023s", tmpfile)
		Tracet(3, "ftpthread: file exists %s\n", ftp.local)
		ftp.state = 2
		return
	}

	if len(proxyaddr) > 0 {
		if ftp.proto > 0 {
			proto = "http"
		} else {
			proto = "ftp"
		}

		env = fmt.Sprintf("set %.4s_proxy=http://%.998s & ", proto, proxyaddr)
		proxyopt = "--proxy=on "
	}
	/* download command (ref [2]) */
	if ftp.proto == 0 { /* ftp */
		opt = fmt.Sprintf("--ftp-user=%.32s --ftp-password=%.32s --glob=off --passive-ftp %.32s -t 1 -T %d -O \"%.768s\"",
			ftp.user, ftp.passwd, proxyopt, FTP_TIMEOUT, local)
		cmd = fmt.Sprintf("%s%s %s \"ftp://%s/%s\" 2> \"%.768s\"\n", env, FTP_CMD, opt,
			ftp.addr, remote, errfile)
	} else { /* http */
		opt = fmt.Sprintf("%.32s -t 1 -T %d -O \"%.768s\"", proxyopt, FTP_TIMEOUT,
			local)
		cmd = fmt.Sprintf("%s%s %s \"http://%s/%s\" 2> \"%.768s\"\n", env, FTP_CMD,
			opt, ftp.addr, remote, errfile)
	}
	/* execute download command */
	if ret = ExecCmd(cmd); ret > 0 {
		os.Remove(local)
		Tracet(2, "execcmd error: cmd=%s ret=%d\n", cmd, ret)
		ftp.error = ret
		ftp.state = 3
		return
	}
	os.Remove(errfile)

	/* uncompress downloaded file */
	idx = strings.LastIndex(local, ".")
	if idx >= 0 {
		if strings.HasSuffix(local, ".z") || strings.HasSuffix(local, ".gz") || strings.HasSuffix(local, ".zip") ||
			strings.HasSuffix(local, ".Z") || strings.HasSuffix(local, ".GZ") || strings.HasSuffix(local, ".ZIP") {

			if Rtk_Uncompress(local, &tmpfile) > 0 {
				os.Remove(local)
				local = tmpfile
			} else {
				Tracet(2, "file uncompact error: %s\n", local)
				ftp.error = 12
				ftp.state = 3
				return
			}
		}
	}
	ftp.local = local
	ftp.state = 2 /* ftp completed */

	Tracet(3, "ftpthread: complete cmd=%s\n", cmd)
}

/* open ftp ------------------------------------------------------------------*/
func OpenFtp(path string, ctype int, msg *string) *FtpConn {
	var ftp *FtpConn = new(FtpConn)

	Tracet(3, "openftp: path=%s type=%d\n", path, ctype)

	*msg = ""

	ftp.state = 0
	ftp.proto = ctype
	ftp.error = 0
	ftp.thread = 0
	ftp.local = ""

	/* decode ftp path */
	DecodeFtpPath(path, &ftp.addr, &ftp.file, &ftp.user, &ftp.passwd, ftp.topts[:])

	/* set first download time */
	ftp.tnext = TimeAdd(TimeGet(), 10.0)

	return ftp
}

/* close ftp -----------------------------------------------------------------*/
func (ftp *FtpConn) CloseFtp() {
	Tracet(3, "closeftp: state=%d\n", ftp.state)

	if ftp.state != 1 {
		ftp = nil
	}
}

/* read ftp ------------------------------------------------------------------*/
func (ftp *FtpConn) ReadFtp(buff []byte, n int, msg *string) int {
	var time Gtime

	Tracet(4, "readftp: n=%d\n", n)

	time = Utc2GpsT(TimeGet())

	if TimeDiff(time, ftp.tnext) < 0.0 { /* until download time? */
		return 0
	}
	if ftp.state <= 0 { /* ftp/http not executed? */
		ftp.state = 1
		if ftp.proto > 0 {
			*msg = fmt.Sprintf("%s://%s", "http", ftp.addr)
		} else {
			*msg = fmt.Sprintf("%s://%s", "ftp", ftp.addr)
		}
		go ftpthread(ftp)
	}
	// #ifdef WIN32
	//         if (!(ftp.thread=CreateThread(NULL,0,ftpthread,ftp,0,NULL))) {
	// #else
	//         if (pthread_create(&ftp.thread,NULL,ftpthread,ftp)) {
	// #endif
	//         tracet(2,"readftp: ftp thread create error\n");
	//         ftp.state=3;
	//         *msg = "ftp thread error"
	//         return 0;
	//     }
	// }
	if ftp.state <= 1 {
		return 0 /* ftp/http on going? */
	}

	if ftp.state == 3 { /* ftp error */
		if ftp.proto > 0 {
			*msg = fmt.Sprintf("%s error (%d)", "http", ftp.error)
		} else {
			*msg = fmt.Sprintf("%s error (%d)", "ftp", ftp.error)
		}
		/* set next retry time */
		ftp.tnext = nextdltime(ftp.topts[:], 0)
		ftp.state = 0
		return 0
	}
	/* return local file path if ftp completed */
	n1 := len(buff)
	buff = []byte(ftp.local[:n] + "\r\n")

	/* set next download time */
	ftp.tnext = nextdltime(ftp.topts[:], 1)
	ftp.state = 0

	*msg = ""

	return len(buff) - n1
}

/* get state ftp -------------------------------------------------------------*/
func (ftp *FtpConn) StateFtp() int {
	if ftp == nil {
		return 0
	}
	if ftp.state == 0 {
		return 2
	} else if ftp.state <= 2 {
		return 3
	}
	return -1
}

/* get extended state ftp ----------------------------------------------------*/
func (ftp *FtpConn) StateXFtp(msg *string) int {
	return ftp.StateFtp()
}

/* open memory buffer --------------------------------------------------------*/
func OpenMemBuf(path string, msg *string) *MemBuf {
	var (
		membuf  *MemBuf = new(MemBuf)
		bufsize int     = DEFAULT_MEMBUF_SIZE
	)

	Tracet(3, "openmembuf: path=%s\n", path)

	*msg = ""

	fmt.Sscanf(path, "%d", &bufsize)

	membuf.state = 1
	membuf.rp = 0
	membuf.wp = 0
	membuf.bufsize = bufsize
	membuf.buf = make([]byte, bufsize)
	*msg = fmt.Sprintf("membuf sizebuf=%d", bufsize)

	return membuf
}

/* close memory buffer -------------------------------------------------------*/
func (membuf *MemBuf) CloseMemBuf() {
	Tracet(3, "closemembufp\n")
	membuf.buf = nil
	membuf = nil
}

/* read memory buffer --------------------------------------------------------*/
func (membuf *MemBuf) ReadMemBuf(buff []byte, n int, msg *string) int {
	var i, nr int

	Tracet(4, "readmembuf: n=%d\n", n)

	if membuf == nil {
		return 0
	}

	membuf.lock.Lock()

	for i = membuf.rp; i != membuf.wp && nr < n; i++ {
		if i >= membuf.bufsize {
			i = 0
		}
		buff[nr] = membuf.buf[i]
		nr++
	}
	membuf.rp = i
	membuf.lock.Unlock()
	return nr
}

/* write memory buffer -------------------------------------------------------*/
func (membuf *MemBuf) WriteMemBuf(buff []byte, n int, msg *string) int {
	var i int

	Tracet(3, "writemembuf: n=%d\n", n)

	if membuf == nil {
		return 0
	}

	membuf.lock.Lock()

	for i = 0; i < n; i++ {
		membuf.buf[membuf.wp] = buff[i]
		membuf.wp++
		if membuf.wp >= membuf.bufsize {
			membuf.wp = 0
		}
		if membuf.wp == membuf.rp {
			*msg = "mem-buffer overflow"
			membuf.state = -1
			membuf.lock.Unlock()
			return i + 1
		}
	}
	membuf.lock.Unlock()
	return i
}

/* get state memory buffer ---------------------------------------------------*/
func (membuf *MemBuf) StateMemBuf() int {
	if membuf == nil {
		return 0
	}
	return membuf.state
}

/* get extended state memory buffer ------------------------------------------*/
func (membuf *MemBuf) StateXMemBuf(msg *string) int {
	state := membuf.StateMemBuf()

	*msg += "membuf:\n"
	*msg += fmt.Sprintf("  state   = %d\n", state)
	if state == 0 {
		return 0
	}
	*msg += fmt.Sprintf("  buffsize= %d\n", membuf.bufsize)
	*msg += fmt.Sprintf("  wp      = %d\n", membuf.wp)
	*msg += fmt.Sprintf("  rp      = %d\n", membuf.rp)
	return state
}

/* initialize stream environment -----------------------------------------------
* initialize stream environment
* args   : none
* return : none
*-----------------------------------------------------------------------------*/
func strinitcom() {
	// #ifdef WIN32
	//     WSADATA data;
	// #endif
	//     tracet(3,"strinitcom:\n");

	// #ifdef WIN32
	//     WSAStartup(MAKEWORD(2,0),&data);
	// #endif
}

/* initialize stream -----------------------------------------------------------
* initialize stream struct
* args   : stream_t *stream IO  stream
* return : none
*-----------------------------------------------------------------------------*/
func (stream *Stream) InitStream() {
	Tracet(3, "strinit:\n")

	stream.Type = 0
	stream.Mode = 0
	stream.State = 0
	stream.InBytes, stream.InRate, stream.OutBytes, stream.OutRate = 0, 0, 0, 0
	stream.TickInput, stream.TickOutput, stream.TickActive, stream.InByeTick, stream.OutByteTick = 0, 0, 0, 0, 0

	stream.Port = nil
	stream.Path = ""
	stream.Msg = ""
}

/* open stream -----------------------------------------------------------------
*
* open stream to read or write data from or to virtual devices.
*
* args   : stream_t *stream IO  stream
*          int type         I   stream type
*                                 STR_SERIAL   = serial device
*                                 STR_FILE     = file (record and playback)
*                                 STR_TCPSVR   = TCP server
*                                 STR_TCPCLI   = TCP client
*                                 STR_NTRIPSVR = NTRIP server
*                                 STR_NTRIPCLI = NTRIP client
*                                 STR_NTRIPC_S = NTRIP caster server
*                                 STR_NTRIPC_C = NTRIP caster client
*                                 STR_UDPSVR   = UDP server (read only)
*                                 STR_UDPCLI   = UDP client (write only)
*                                 STR_MEMBUF   = memory buffer (FIFO)
*                                 STR_FTP      = download by FTP (raed only)
*                                 STR_HTTP     = download by HTTP (raed only)
*          int mode         I   stream mode (STR_MODE_???)
*                                 STR_MODE_R   = read only
*                                 STR_MODE_W   = write only
*                                 STR_MODE_RW  = read and write
*          char *path       I   stream path (see below)
*
* return : status (0:error,1:ok)
*
* notes  : see reference [1] for NTRIP
*          STR_FTP/HTTP needs "wget" in command search paths
*
* stream path ([] options):
*
*   STR_SERIAL   port[:brate[:bsize[:parity[:stopb[:fctr[#port]]]]]]
*                    port  = COM??  (windows)
*                            tty??? (linuex, omit /dev/)
*                    brate = bit rate     (bps)
*                    bsize = bit size     (7|8)
*                    parity= parity       (n|o|e)
*                    stopb = stop bits    (1|2)
*                    fctr  = flow control (off|rts)
*                    port  = tcp server port to output received stream
*
*   STR_FILE     path[::T][::+start][::xseppd][::S=swap][::P={4|8}]
*                    path  = file path
*                            (can include keywords defined by )
*                    ::T   = enable time tag
*                    start = replay start offset (s)
*                    speed = replay speed factor
*                    swap  = output swap interval (hr) (0: no swap)
*                    ::P={4|8} = file pointer size (4:32bit,8:64bit)
*
*   STR_TCPSVR   :port
*                    port  = TCP server port to accept
*
*   STR_TCPCLI   addr:port
*                    addr  = TCP server address to connect
*                    port  = TCP server port to connect
*
*   STR_NTRIPSVR [:passwd@]addr[:port]/mponit[:string]
*                    addr  = NTRIP caster address to connect
*                    port  = NTRIP caster server port to connect
*                    passwd= NTRIP caster server password to connect
*                    mpoint= NTRIP mountpoint
*                    string= NTRIP server string
*
*   STR_NTRIPCLI [user[:passwd]@]addr[:port]/mpoint
*                    addr  = NTRIP caster address to connect
*                    port  = NTRIP caster client port to connect
*                    user  = NTRIP caster client user to connect
*                    passwd= NTRIP caster client password to connect
*                    mpoint= NTRIP mountpoint
*
*   STR_NTRIPCAS [user[:passwd]@][:port]/mpoint[:srctbl]
*                    port  = NTRIP caster client port to accept connection
*                    user  = NTRIP caster client user to accept connection
*                    passwd= NTRIP caster client password to accept connection
*                    mpoint= NTRIP mountpoint
*                    srctbl= NTRIP source table entry (STR) (ref [3] 6.3)
*                      (ID;format;format-details;carrier;nav-system;network;
*                       country;latitude;longitude;nmea;solution;generator;
*                       compr-encrp;autentication;fee;bitrate;...;misc)
*
*   STR_UDPSVR   :port
*                    port  = UDP server port to receive
*
*   STR_UDPCLI   addr:port
*                    addr  = UDP server or broadcast address to send
*                    port  = UDP server or broadcast port to send
*
*   STR_MEMBUF   [size]
*                    size  = FIFO size (bytes) ("":4096)
*
*   STR_FTP      [user[:passwd]@]addr/path[::T=poff[,tint[,toff,tret]]]]
*                    user  = FTP server user
*                    passwd= FTP server password
*                    addr  = FTP server address
*                    path  = FTP server file path
*                    poff  = time offset for path extension (s)
*                    tint  = download interval (s)
*                    toff  = download time offset (s)
*                    tret  = download retry interval (s) (0:no retry)
*
*   STR_HTTP     addr/path[::T=poff[,tint[,toff,tret]]]]
*                    addr  = HTTP server address
*                    path  = HTTP server file path
*                    poff  = time offset for path extension (s)
*                    tint  = download interval (s)
*                    toff  = download time offset (s)
*                    tret  = download retry interval (s) (0:no retry)
*
*-----------------------------------------------------------------------------*/
func (stream *Stream) OpenStream(ctype, mode int, path string) int {
	Tracet(3, "stropen: type=%d mode=%d path=%s\n", ctype, mode, path)

	stream.Type = ctype
	stream.Mode = mode
	stream.Path = path
	stream.InBytes, stream.InRate, stream.OutBytes, stream.OutRate = 0, 0, 0, 0
	stream.TickInput = TickGet()
	stream.TickOutput = stream.TickInput
	stream.InByeTick, stream.OutByteTick = 0, 0
	stream.Msg = ""
	stream.Port = nil
	switch byte(ctype) {
	case STR_SERIAL:
		stream.Port = OpenSerial(path, mode, &stream.Msg)

	case STR_FILE:
		stream.Port = OpenStreamFile(path, mode, &stream.Msg)

	case STR_TCPSVR:
		stream.Port = OpenTcpSvr(path, &stream.Msg)

	case STR_TCPCLI:
		stream.Port = OpenTcpClient(path, &stream.Msg)

	case STR_NTRIPSVR:
		stream.Port = OpenNtrip(path, 0, &stream.Msg)

	case STR_NTRIPCLI:
		stream.Port = OpenNtrip(path, 1, &stream.Msg)

	case STR_NTRIPCAS:
		stream.Port = OpenNtripc(path, &stream.Msg)

	case STR_UDPSVR:
		stream.Port = OpenUdpSvr(path, &stream.Msg)

	case STR_UDPCLI:
		stream.Port = OpenUdpClient(path, &stream.Msg)

	case STR_MEMBUF:
		stream.Port = OpenMemBuf(path, &stream.Msg)

	case STR_FTP:
		stream.Port = OpenFtp(path, 0, &stream.Msg)

	case STR_HTTP:
		stream.Port = OpenFtp(path, 1, &stream.Msg)

	default:
		stream.State = 0
		return 1
	}
	if stream.Port == nil {
		stream.State = -1
		return 0
	}
	stream.State = 1
	return 1
}

/* close stream ----------------------------------------------------------------
* close stream
* args   : stream_t *stream IO  stream
* return : none
*-----------------------------------------------------------------------------*/
func (stream *Stream) StreamClose() {
	Tracet(3, "strclose: type=%d mode=%d\n", stream.Type, stream.Mode)

	streamlock(stream)

	if stream.Port != nil {
		switch byte(stream.Type) {
		case STR_SERIAL:
			stream.Port.(*SerialComm).CloseSerial()

		case STR_FILE:
			stream.Port.(*FileType).CloseStreamFile()

		case STR_TCPSVR:
			stream.Port.(*TcpSvr).CloseTcpSvr()

		case STR_TCPCLI:
			stream.Port.(*TcpClient).CloseTcpClient()

		case STR_NTRIPSVR:
			stream.Port.(*NTrip).CloseNtrip()

		case STR_NTRIPCLI:
			stream.Port.(*NTrip).CloseNtrip()

		case STR_NTRIPCAS:
			stream.Port.(*NTripc).CloseNtripc()

		case STR_UDPSVR:
			stream.Port.(*UdpConn).CloseUdpSvr()

		case STR_UDPCLI:
			stream.Port.(*UdpConn).CloseUdpClient()

		case STR_MEMBUF:
			stream.Port.(*MemBuf).CloseMemBuf()

		case STR_FTP:
			stream.Port.(*FtpConn).CloseFtp()

		case STR_HTTP:
			stream.Port.(*FtpConn).CloseFtp()

		}
	} else {
		Trace(2, "no port to close stream: type=%d\n", stream.Type)
	}
	stream.Type = 0
	stream.Mode = 0
	stream.State = 0
	stream.InRate, stream.OutRate = 0, 0
	stream.Path = ""
	stream.Msg = ""
	stream.Port = nil

	streamunlock(stream)
}

/* sync streams ----------------------------------------------------------------
* sync time for streams
* args   : stream_t *stream1 IO stream 1
*          stream_t *stream2 IO stream 2
* return : none
* notes  : for replay files with time tags
*-----------------------------------------------------------------------------*/
func strsync(stream1, stream2 *Stream) {
	if stream1.Type != STR_FILE || stream2.Type != STR_FILE {
		return
	}
	file1 := stream1.Port.(*FileType)
	file2 := stream2.Port.(*FileType)
	if file1 != nil && file2 != nil {
		syncfile(file1, file2)
	}
}

/* lock/unlock stream ----------------------------------------------------------
* lock/unlock stream
* args   : stream_t *stream I  stream
* return : none
*-----------------------------------------------------------------------------*/
func streamlock(stream *Stream)   { stream.Lock.Lock() }
func streamunlock(stream *Stream) { stream.Lock.Unlock() }

/* read stream -----------------------------------------------------------------
* read data from stream (unblocked)
* args   : stream_t *stream I  stream
*          unsinged char *buff O data buffer
*          int    n         I  maximum data length
* return : read data length
* notes  : if no data, return immediately with no data
*-----------------------------------------------------------------------------*/
func (stream *Stream) StreamRead(buff []byte, n int) int {
	tick := TickGet()
	var msg *string = &stream.Msg
	var nr, tt int

	Tracet(4, "strread: n=%d\n", n)

	if (stream.Mode&STR_MODE_R == 0) || stream.Port == nil {
		return 0
	}

	streamlock(stream)

	switch byte(stream.Type) {
	case STR_SERIAL:
		nr = stream.Port.(*SerialComm).ReadSerial(buff, n, msg)

	case STR_FILE:
		nr = stream.Port.(*FileType).ReadFile(buff, int64(n), msg)

	case STR_TCPSVR:
		nr = stream.Port.(*TcpSvr).ReadTcpSvr(buff, n, msg)

	case STR_TCPCLI:
		nr = stream.Port.(*TcpClient).ReadTcpClient(buff, n, msg)

	case STR_NTRIPSVR, STR_NTRIPCLI:
		nr = stream.Port.(*NTrip).ReadNtrip(buff, n, msg)

	case STR_NTRIPCAS:
		nr = stream.Port.(*NTripc).ReadNtripc(buff, n, msg)

	case STR_UDPSVR:
		nr = stream.Port.(*UdpConn).ReadUdpSvr(buff, n, msg)

	case STR_MEMBUF:
		nr = stream.Port.(*MemBuf).ReadMemBuf(buff, n, msg)

	case STR_FTP:
		nr = stream.Port.(*FtpConn).ReadFtp(buff, n, msg)

	case STR_HTTP:
		nr = stream.Port.(*FtpConn).ReadFtp(buff, n, msg)

	default:
		streamunlock(stream)
		return 0
	}
	//	stream.Msg = msg
	if nr > 0 {
		stream.InBytes += uint32(nr)
		stream.TickActive = tick
	}
	tt = int(tick - stream.TickInput)
	if tt >= tirate {
		stream.InRate =
			uint32(float64((stream.InBytes-stream.InByeTick)*8.0) / (float64(tt) * 0.001))
		stream.TickInput = tick
		stream.InByeTick = stream.InBytes
	}
	streamunlock(stream)
	return nr
}

/* write stream ----------------------------------------------------------------
* write data to stream (unblocked)
* args   : stream_t *stream I   stream
*          unsinged char *buff I data buffer
*          int    n         I   data length
* return : status (0:error,1:ok)
* notes  : write data to buffer and return immediately
*-----------------------------------------------------------------------------*/
func (stream *Stream) StreamWrite(buff []byte, n int) int {
	tick := TickGet()
	var msg *string = &stream.Msg
	var ns, tt int

	Tracet(4, "strwrite: n=%d\n", n)

	if (stream.Mode&STR_MODE_W) == 0 || stream.Port == nil {
		return 0
	}

	streamlock(stream)

	switch byte(stream.Type) {
	case STR_SERIAL:
		ns = stream.Port.(*SerialComm).WriteSerial(buff, n, msg)

	case STR_FILE:
		ns = stream.Port.(*FileType).WriteFile(buff, n, msg)

	case STR_TCPSVR:
		ns = stream.Port.(*TcpSvr).WriteTcpSvr(buff, n, msg)

	case STR_TCPCLI:
		ns = stream.Port.(*TcpClient).WriteTcpClient(buff, n, msg)

	case STR_NTRIPSVR, STR_NTRIPCLI:
		ns = stream.Port.(*NTrip).WriteNtrip(buff, n, msg)

	case STR_NTRIPCAS:
		ns = stream.Port.(*NTripc).WriteNtripc(buff, n, msg)

	case STR_UDPCLI:
		ns = stream.Port.(*UdpConn).WriteUdpClient(buff, n, msg)

	case STR_MEMBUF:
		ns = stream.Port.(*MemBuf).WriteMemBuf(buff, n, msg)

	case STR_FTP, STR_HTTP:
		streamunlock(stream)
		return 0
	default:
		streamunlock(stream)
		return 0
	}
	if ns > 0 {
		stream.OutBytes += uint32(ns)
		stream.TickActive = tick
	}
	tt = int(tick - stream.TickOutput)
	if tt > tirate {
		stream.OutRate =
			uint32(float64((stream.OutBytes-stream.OutByteTick)*8) / (float64(tt) * 0.001))
		stream.TickOutput = tick
		stream.OutByteTick = stream.OutBytes
	}
	streamunlock(stream)
	return ns
}

/* get stream status -----------------------------------------------------------
* get stream status
* args   : stream_t *stream I   stream
*          char   *msg      IO  status message (NULL: no output)
* return : status (-1:error,0:close,1:wait,2:connect,3:active)
*-----------------------------------------------------------------------------*/
func (stream *Stream) StreamStat(msg *string) int {
	var state int = 0

	Tracet(4, "strstat:\n")

	streamlock(stream)
	if msg != nil {
		*msg = stream.Msg
	}
	if stream.Port == nil {
		streamunlock(stream)
		return stream.State
	}
	switch byte(stream.Type) {
	case STR_SERIAL:
		state = stream.Port.(*SerialComm).StateSerial()

	case STR_FILE:
		state = stream.Port.(*FileType).StateFile()

	case STR_TCPSVR:
		state = stream.Port.(*TcpSvr).StateTcpSvr()

	case STR_TCPCLI:
		state = stream.Port.(*TcpClient).StateTcpCli()

	case STR_NTRIPSVR, STR_NTRIPCLI:
		state = stream.Port.(*NTrip).StateNtrip()

	case STR_NTRIPCAS:
		state = stream.Port.(*NTripc).StateNtripc()

	case STR_UDPSVR:
		state = stream.Port.(*UdpConn).StateUdpSvr()

	case STR_UDPCLI:
		state = stream.Port.(*UdpConn).StateUdpClient()

	case STR_MEMBUF:
		state = stream.Port.(*MemBuf).StateMemBuf()

	case STR_FTP:
		state = stream.Port.(*FtpConn).StateFtp()

	case STR_HTTP:
		state = stream.Port.(*FtpConn).StateFtp()

	default:
		streamunlock(stream)
		return 0
	}
	if state == 2 && int(TickGet()-stream.TickActive) <= TINTACT {
		state = 3
	}
	streamunlock(stream)
	return state
}

/* get extended stream status ---------------------------------------------------
* get extended stream status
* args   : stream_t *stream I   stream
*          char   *msg      IO  extended status message
* return : status (-1:error,0:close,1:wait,2:connect,3:active)
*-----------------------------------------------------------------------------*/
func (stream *Stream) StreamStateX(msg *string) int {
	var state int = 0

	Tracet(4, "strstatx:\n")

	streamlock(stream)

	if stream.Port == nil {
		streamunlock(stream)
		return stream.State
	}
	switch byte(stream.Type) {
	case STR_SERIAL:
		state = stream.Port.(*SerialComm).StatExSerial(msg)

	case STR_FILE:
		state = stream.Port.(*FileType).StatExFile(msg)

	case STR_TCPSVR:
		state = stream.Port.(*TcpSvr).StatExTcpSvr(msg)

	case STR_TCPCLI:
		state = stream.Port.(*TcpClient).StatExTcpClient(msg)

	case STR_NTRIPSVR, STR_NTRIPCLI:
		state = stream.Port.(*NTrip).StatExNtrip(msg)

	case STR_NTRIPCAS:
		state = stream.Port.(*NTripc).StatExNtripc(msg)

	case STR_UDPSVR:
		state = stream.Port.(*UdpConn).StatExUdpSvr(msg)

	case STR_UDPCLI:
		state = stream.Port.(*UdpConn).StateXUdpClient(msg)

	case STR_MEMBUF:
		state = stream.Port.(*MemBuf).StateXMemBuf(msg)

	case STR_FTP:
		state = stream.Port.(*FtpConn).StateXFtp(msg)

	case STR_HTTP:
		state = stream.Port.(*FtpConn).StateXFtp(msg)

	default:
		*msg = ""
		streamunlock(stream)
		return 0
	}
	if state == 2 && int(TickGet()-stream.TickActive) <= TINTACT {
		state = 3
	}
	streamunlock(stream)
	return state
}

/* get stream statistics summary -----------------------------------------------
* get stream statistics summary
* args   : stream_t *stream I   stream
*          int    *inb      IO   bytes of input  (NULL: no output)
*          int    *inr      IO   bps of input    (NULL: no output)
*          int    *outb     IO   bytes of output (NULL: no output)
*          int    *outr     IO   bps of output   (NULL: no output)
* return : none
*-----------------------------------------------------------------------------*/
func strsum(stream *Stream, inb, inr, outb, outr *int) {
	Tracet(4, "strsum:\n")

	streamlock(stream)
	if inb != nil {
		*inb = int(stream.InBytes)
	}
	if inr != nil {
		*inr = int(stream.InRate)
	}
	if outb != nil {
		*outb = int(stream.OutBytes)
	}
	if outr != nil {
		*outr = int(stream.OutRate)
	}
	streamunlock(stream)
}

/* set global stream options ---------------------------------------------------
* set global stream options
* args   : int    *opt      I   options
*              opt[0]= inactive timeout (ms) (0: no timeout)
*              opt[1]= interval to reconnect (ms)
*              opt[2]= averaging time of data rate (ms)
*              opt[3]= receive/send buffer size (bytes);
*              opt[4]= file swap margin (s)
*              opt[5]= reserved
*              opt[6]= reserved
*              opt[7]= reserved
* return : none
*-----------------------------------------------------------------------------*/
func StreamSetOpt(opt []int) {
	Tracet(3, "strsetopt: opt=")
	for _, v := range opt {
		Tracet(3, "%d ", v)
	}
	Tracet(3, "\n")

	toinact = opt[0] /* >=1s */
	if opt[0] < 1000 {
		toinact = 1000 /* >=1s */
	}
	ticonnect = opt[1] /* >=1s */
	if opt[1] < 1000 {
		ticonnect = 1000
	}
	tirate = opt[2] /* >=0.1s */
	if opt[2] < 100 {
		tirate = 100
	}
	// buffsize = opt[3] /* >=4096byte */
	// if opt[3] < 4096 {
	// 	buffsize = 4096
	// }
	fswapmargin = opt[4]
	if opt[4] < 0 {
		fswapmargin = 0
	}
}

/* set timeout time ------------------------------------------------------------
* set timeout time
* args   : stream_t *stream I   stream (STR_TCPCLI,STR_NTRIPCLI,STR_NTRIPSVR)
*          int     toinact  I   inactive timeout (ms) (0: no timeout)
*          int     tirecon  I   reconnect interval (ms) (0: no reconnect)
* return : none
*-----------------------------------------------------------------------------*/
func (stream *Stream) StreamSetTimeout(toinact, tirecon int) {
	var tcpcli *TcpClient

	Tracet(3, "strsettimeout: toinact=%d tirecon=%d\n", toinact, tirecon)

	if stream.Type == STR_TCPCLI {
		tcpcli = stream.Port.(*TcpClient)
	} else if stream.Type == STR_NTRIPCLI || stream.Type == STR_NTRIPSVR {
		tcpcli = stream.Port.(*NTrip).tcp
	} else {
		return
	}

	tcpcli.toinact = toinact
	tcpcli.tirecon = tirecon
}

/* set local directory ---------------------------------------------------------
* set local directory path for ftp/http download
* args   : char   *dir      I   directory for download files
* return : none
*-----------------------------------------------------------------------------*/
func StreamSetDir(dir string) {
	Tracet(3, "strsetdir: dir=%s\n", dir)

	localdir = dir
}

/* set http/ntrip proxy address ------------------------------------------------
* set http/ntrip proxy address
* args   : char   *addr     I   http/ntrip proxy address <address>:<port>
* return : none
*-----------------------------------------------------------------------------*/
func StreamSetProxy(addr string) {
	Tracet(3, "strsetproxy: addr=%s\n", addr)

	proxyaddr = addr
}

/* get stream time -------------------------------------------------------------
* get stream time
* args   : stream_t *stream I   stream
* return : current time or replay time for playback file
*-----------------------------------------------------------------------------*/
func StreamGetTime(stream *Stream) Gtime {
	var file *FileType
	if stream.Type == STR_FILE && (stream.Mode&STR_MODE_R) > 0 {
		if file = stream.Port.(*FileType); file != nil {
			return TimeAdd(file.time, file.start) /* replay start time */
		}
	}
	return Utc2GpsT(TimeGet())
}

/* send nmea request -----------------------------------------------------------
* send nmea gpgga message to stream
* args   : stream_t *stream I   stream
*          sol_t *sol       I   solution
* return : none
*-----------------------------------------------------------------------------*/
func (stream *Stream) StreamSendNmea(sol *Sol) {
	var (
		buff string
		n    int
	)

	Tracet(3, "strsendnmea: rr=%.3f %.3f %.3f\n", sol.Rr[0], sol.Rr[1], sol.Rr[2])

	n = sol.OutSolNmeaGga(&buff)
	stream.StreamWrite([]byte(buff), n)
}

/* generate general hex message ----------------------------------------------*/
func gen_hex(msg string, buff []byte) int {
	var mbuff string
	var b uint32
	var j int

	Trace(4, "gen_hex: msg=%s\n", msg)

	if len(msg) > 1023 {
		mbuff = msg[:1023]
	} else {
		mbuff = msg
	}

	args := strings.Fields(mbuff)
	for i := range args {
		if _, err := fmt.Sscanf(args[i], "%x", &b); err == nil {
			buff[j] = byte(b)
			j++
		}
	}
	return j
}

/* set bitrate ---------------------------------------------------------------*/
func set_brate(str *Stream, brate int) int {
	var (
		path, buff  string
		ctype, mode int = str.Type, str.Mode
	)

	if ctype != STR_SERIAL {
		return 0
	}

	path = str.Path

	idx := strings.Index(path, ":")
	if idx < 0 {
		path += fmt.Sprintf(":%d", brate)
	} else {
		idx2 := strings.Index(path[idx+1:], ":")
		if idx2 >= 0 {
			buff = path[idx2:]
		}
		path = fmt.Sprintf("%s:%d%s", path[:idx], brate, buff)
	}
	str.StreamClose()
	return str.OpenStream(ctype, mode, path)
}

var Gen_ubx_ptr func(msg string, buff []byte) int = gen_ubx
var Gen_stq_ptr func(msg string, buff []byte) int = gen_stq
var Gen_nvs_ptr func(msg string, buff []byte) int = gen_nvs

func default_gen_rcv(msg string, buff []byte) int {
	panic("not implemented")
}

/* send receiver command -------------------------------------------------------
* send receiver commands to stream
* args   : stream_t *stream I   stream
*          char   *cmd      I   receiver command strings
* return : none
*-----------------------------------------------------------------------------*/
func (str *Stream) StrSendCmd(cmd string) {
	var (
		buff            []byte = make([]byte, 1024)
		msg             string
		cmdend          string = "\r\n"
		n, m, ms, brate int
	)

	Tracet(3, "strsendcmd: cmd=%s\n", cmd)

	cmdlets := strings.FieldsFunc(cmd, func(r rune) bool {
		if r == '\r' || r == '\n' {
			return true
		}
		return false
	})

	for _, msg = range cmdlets {
		switch {
		case len(msg) == 0 || strings.HasPrefix(msg, "#"): /* null or comment */

		case strings.HasPrefix(msg, "!"): /* binary escape */
			switch {
			case strings.EqualFold(msg[1:5], "WAIT"):
				if n, _ = fmt.Sscanf(msg[5:], "%d", &ms); n < 1 {
					ms = 100
				}
				if ms > 3000 {
					ms = 3000 /* max 3 s */
				}
				Sleepms(ms)
			case strings.EqualFold(msg[1:6], "BRATE"):
				if n, _ = fmt.Sscanf(msg[6:], "%d", &brate); n < 1 {
					brate = 9600
				}
				set_brate(str, brate)
				Sleepms(500)
			case strings.EqualFold(msg[1:4], "UBX"):
				if m = Gen_ubx_ptr(msg[4:], buff); m > 0 {
					str.StreamWrite(buff, m)
				}
			case strings.EqualFold(msg[1:4], "STQ"):
				if m = Gen_stq_ptr(msg[4:], buff); m > 0 {
					str.StreamWrite(buff, m)
				}
			case strings.EqualFold(msg[1:4], "NVS"):
				if m = Gen_nvs_ptr(msg[4:], buff); m > 0 {
					str.StreamWrite(buff, m)
				}
			case strings.EqualFold(msg[1:4], "HEX"):
				if m = gen_hex(msg[4:], buff); m > 0 {
					str.StreamWrite(buff, m)
				}
			}
		default:
			msg += cmdend
			str.StreamWrite([]byte(msg), n+2)
		}
	}

}
