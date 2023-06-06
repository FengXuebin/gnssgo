package gnss_test

import (
	"fmt"
	"os"
	"testing"
	"time"

	"github.com/stretchr/testify/assert"
)

func Test_String2Byte(t *testing.T) {
	var s string = "hello world"
	var b [64]byte
	assert := assert.New(t)

	copy(b[:], []byte(s))

	b[25] = 'a'
	s = string(b[:])

	assert.True(s == "hello world")
}

func Test_Fprintf(t *testing.T) {

	for {
		for _, r := range "-\\|/" {
			fmt.Printf("\r%c", r)
			time.Sleep(1)
		}
	}
	for i := 0; i < 100; i++ {
		fmt.Fprintf(os.Stdout, "\rthis is a output %2d", i)

		//fmt.Fprintf(os.Stderr, "\r")
	}
}
