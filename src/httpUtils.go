package gnssgo

import (
	"errors"
	"strings"
)

type httpHeaders map[string]string

func ParseHttp(buff string) httpHeaders {
	var headers httpHeaders = make(httpHeaders)
	lines := strings.Split(buff, "\r\n")
	for _, v := range lines {
		s1 := strings.Index(v, ":")
		if s1 == -1 {
			s1 = strings.Index(v, " ")
		}
		if s1 == -1 {
			continue
		}
		headers[v[:s1]] = v[s1+1:]
	}
	return headers
}

func (headers httpHeaders) GetHead(key string) (string, bool) {
	var (
		v  string
		ok bool = true
	)
	if headers == nil {
		return "", false
	}
	v = headers[key]
	if len(v) == 0 {
		ok = false
	}
	return v, ok
}

func (headers httpHeaders) ParseBody(k string, v ...interface{}) error {
	value, ok := headers.GetHead(k)
	if ok {
		bodys := strings.Fields(value)
		if len(v) != len(bodys) {
			return errors.New("num of argument is not right")
		}
		for i := 0; i < len(v); i++ {
			switch a := v[i].(type) {
			case *string:
				*a = bodys[i]

			}
		}
	}
	return nil
}
